// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/carrier_panel.hpp>

#include <rviz_common/display_context.hpp>

#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>
#include <sstream>
#include <vector>

namespace moveit_cable_carrier
{
namespace
{
constexpr const char* kVisualizer = "carrier_visualizer";

/** Split the visualizer's "key=value;key=value" status line. Deliberately a plain string rather
 *  than a generated message type: it keeps the panel free of an interface package and stays
 *  readable with `ros2 topic echo` while debugging. */
std::map<std::string, std::string> parseStatus(const std::string& text)
{
  std::map<std::string, std::string> out;
  std::stringstream ss(text);
  std::string field;
  while (std::getline(ss, field, ';'))
  {
    const size_t eq = field.find('=');
    if (eq != std::string::npos)
    {
      out[field.substr(0, eq)] = field.substr(eq + 1);
    }
  }
  return out;
}

double numberOr(const std::map<std::string, std::string>& fields, const char* key, double fallback)
{
  const auto it = fields.find(key);
  if (it == fields.end())
  {
    return fallback;
  }
  try
  {
    return std::stod(it->second);
  }
  catch (const std::exception&)
  {
    return fallback;
  }
}
}  // namespace

CarrierPanel::CarrierPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  auto* root = new QVBoxLayout(this);

  auto* geometry = new QGroupBox("Carrier", this);
  auto* grid = new QGridLayout(geometry);
  int row = 0;

  grid->addWidget(new QLabel("carrier index"), row, 0);
  carrier_index_ = new QSpinBox(this);
  carrier_index_->setRange(0, 15);
  carrier_index_->setToolTip("Which run in the YAML the sliders below edit. The others stay as configured.");
  grid->addWidget(carrier_index_, row++, 1);

  length_ = addRow(grid, row++, "length", 0.05, 5.0, 0.01, 3, " m",
                   "Arc length between the two brackets. Must exceed the largest bracket-to-bracket "
                   "distance the arm produces, with slack on top.");
  bend_radius_ = addRow(grid, row++, "bend radius", 0.005, 1.0, 0.005, 3, " m",
                        "The carrier's own minimum bend radius (R_min). Hardware stop, not a preference.");
  safety_margin_ = addRow(grid, row++, "safety margin", 0.0, 0.2, 0.001, 3, " m",
                          "Added to every capsule radius. Covers model error, hysteresis and the "
                          "capsule approximation of a rectangular section.");
  twist_limit_ = addRow(grid, row++, "twist limit", 0.0, 90.0, 1.0, 1, " deg/link",
                        "Torsion stop. A 3D dresspack allows roughly 10 deg per link; a planar chain "
                        "essentially none.");

  grid->addWidget(new QLabel("segments"), row, 0);
  num_segments_ = new QSpinBox(this);
  num_segments_->setRange(4, 80);
  num_segments_->setToolTip("Discretisation. More segments resolve the bend limit better and cost "
                            "proportionally more collision geometry.");
  grid->addWidget(num_segments_, row++, 1);
  root->addWidget(geometry);

  auto* cable = new QGroupBox("Cable inside", this);
  auto* cgrid = new QGridLayout(cable);
  row = 0;
  cable_od_ = addRow(cgrid, row++, "outer diameter", 0.0, 0.1, 0.001, 3, " m",
                     "Outer diameter of the worst cable routed inside. 0 disables the cable check.");
  cable_factor_ = addRow(cgrid, row++, "min bend factor", 0.0, 30.0, 0.5, 1, " x OD",
                         "Minimum bend radius as a multiple of the cable's outer diameter. "
                         "10x is the usual figure for continuous-flex robot cable.");
  root->addWidget(cable);

  auto* solver = new QGroupBox("Solver", this);
  auto* sgrid = new QGridLayout(solver);
  row = 0;
  gravity_sag_ = addRow(sgrid, row++, "gravity sag", 0.0, 1.0, 0.05, 2, "",
                        "Heuristic sag weight. Not a static equilibrium solve, so treat it as a "
                        "tuning knob rather than physics.");
  energy_relaxation_ = addRow(sgrid, row++, "energy relaxation", 0.0, 1.0, 0.05, 2, "",
                              "Strength of the straighten-out objective in the endpoint task's null "
                              "space. 0 lets the solve sit against the bend limit.");
  show_all_ = new QCheckBox("show every carrier", this);
  show_all_->setChecked(true);
  sgrid->addWidget(show_all_, row++, 0, 1, 2);
  root->addWidget(solver);

  auto* buttons = new QHBoxLayout();
  apply_ = new QPushButton("Apply", this);
  reload_ = new QPushButton("Read from node", this);
  buttons->addWidget(apply_);
  buttons->addWidget(reload_);
  root->addLayout(buttons);

  verdict_ = new QLabel("waiting for carrier_visualizer…", this);
  QFont bold = verdict_->font();
  bold.setBold(true);
  verdict_->setFont(bold);
  verdict_->setWordWrap(true);
  root->addWidget(verdict_);

  detail_ = new QLabel(this);
  detail_->setWordWrap(true);
  detail_->setTextFormat(Qt::PlainText);
  root->addWidget(detail_);
  root->addStretch();

  connect(apply_, &QPushButton::clicked, this, &CarrierPanel::apply);
  connect(reload_, &QPushButton::clicked, this, &CarrierPanel::refreshFromNode);
}

QDoubleSpinBox* CarrierPanel::addRow(QGridLayout* grid, int row, const QString& label, double min, double max,
                                     double step, int decimals, const QString& suffix, const QString& tip)
{
  grid->addWidget(new QLabel(label), row, 0);
  auto* box = new QDoubleSpinBox(this);
  box->setRange(min, max);
  box->setSingleStep(step);
  box->setDecimals(decimals);
  box->setSuffix(suffix);
  box->setToolTip(tip);
  grid->addWidget(box, row, 1);
  return box;
}

void CarrierPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  params_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, kVisualizer);

  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "cable_carrier_status", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // Only cache here. This runs on RViz's executor thread, and Qt widgets may only be touched
        // from the GUI thread, so the repaint happens in updateReadouts() under a QTimer.
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = msg->data;
        status_seen_ = true;
      });

  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &CarrierPanel::updateReadouts);
  timer->start(200);

  // Give the visualizer a moment to come up before asking it for values.
  QTimer::singleShot(1500, this, &CarrierPanel::refreshFromNode);
}

void CarrierPanel::apply()
{
  if (!params_ || !params_->service_is_ready())
  {
    verdict_->setText("carrier_visualizer is not up — nothing to apply to");
    return;
  }
  params_->set_parameters({
      rclcpp::Parameter("carrier_index", carrier_index_->value()),
      rclcpp::Parameter("length", length_->value()),
      rclcpp::Parameter("bend_radius", bend_radius_->value()),
      rclcpp::Parameter("safety_margin", safety_margin_->value()),
      rclcpp::Parameter("twist_limit_deg", twist_limit_->value()),
      rclcpp::Parameter("gravity_sag", gravity_sag_->value()),
      rclcpp::Parameter("energy_relaxation", energy_relaxation_->value()),
      rclcpp::Parameter("num_segments", num_segments_->value()),
      rclcpp::Parameter("cable_outer_diameter", cable_od_->value()),
      rclcpp::Parameter("cable_min_bend_factor", cable_factor_->value()),
      rclcpp::Parameter("show_all", show_all_->isChecked()),
  });
}

void CarrierPanel::refreshFromNode()
{
  if (!params_ || !params_->service_is_ready())
  {
    verdict_->setText("carrier_visualizer is not up yet");
    return;
  }
  const std::vector<std::string> names = { "carrier_index",    "length",           "bend_radius",
                                           "safety_margin",    "twist_limit_deg",  "gravity_sag",
                                           "energy_relaxation", "num_segments",    "cable_outer_diameter",
                                           "cable_min_bend_factor", "show_all" };
  // Fire and forget: the reply is applied by the lambda on the executor thread, so it only stores
  // plain values that updateReadouts() picks up. Widgets themselves are set here only when the
  // future is already ready, which it is for a local node in practice.
  auto future = params_->get_parameters(names);
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready)
  {
    verdict_->setText("carrier_visualizer did not answer the parameter query");
    return;
  }
  const auto values = future.get();
  if (values.size() != names.size())
  {
    return;
  }
  carrier_index_->setValue(static_cast<int>(values[0].as_int()));
  length_->setValue(values[1].as_double());
  bend_radius_->setValue(values[2].as_double());
  safety_margin_->setValue(values[3].as_double());
  twist_limit_->setValue(values[4].as_double());
  gravity_sag_->setValue(values[5].as_double());
  energy_relaxation_->setValue(values[6].as_double());
  num_segments_->setValue(static_cast<int>(values[7].as_int()));
  cable_od_->setValue(values[8].as_double());
  cable_factor_->setValue(values[9].as_double());
  show_all_->setChecked(values[10].as_bool());
}

void CarrierPanel::updateReadouts()
{
  std::string text;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_seen_)
    {
      return;
    }
    text = status_;
  }
  if (text.empty())
  {
    return;
  }

  const auto fields = parseStatus(text);
  const bool feasible = numberOr(fields, "feasible", 0.0) > 0.5;
  const bool cable_ok = numberOr(fields, "cable_ok", 1.0) > 0.5;
  const double radius_mm = numberOr(fields, "radius_mm", 0.0);
  const double bend_util = numberOr(fields, "bend_util", 0.0);
  const double twist_util = numberOr(fields, "twist_util", 0.0);
  const double ratio = numberOr(fields, "cable_ratio", 0.0);
  const double required = numberOr(fields, "cable_required", 0.0);
  const double strain = numberOr(fields, "strain", 0.0);

  QString verdict;
  QString colour;
  if (!feasible)
  {
    verdict = "UNREACHABLE — the carrier cannot span the brackets in this pose";
    colour = "#c0392b";
  }
  else if (!cable_ok)
  {
    verdict = "CABLE OVER-BENT — the carrier is fine, what is inside it is not";
    colour = "#b9770e";
  }
  else
  {
    verdict = "OK — within the carrier's bend limit and the cable's";
    colour = "#1e8449";
  }
  verdict_->setText(verdict);
  verdict_->setStyleSheet(QString("color: %1;").arg(colour));

  QString detail = QString("achieved radius %1 mm   carrier %2 of its limit   twist %3 of the stop")
                       .arg(radius_mm, 0, 'f', 1)
                       .arg(bend_util, 0, 'f', 2)
                       .arg(twist_util, 0, 'f', 2);
  if (required > 0.0)
  {
    detail += QString("\ncable '%1': %2x OD, needs %3x   (bending strain %4 %)")
                  .arg(QString::fromStdString(fields.count("cable") ? fields.at("cable") : "-"))
                  .arg(ratio, 0, 'f', 1)
                  .arg(required, 0, 'f', 0)
                  .arg(100.0 * strain, 0, 'f', 2);
  }
  else
  {
    detail += "\nno cable declared, so no cable bend-radius check is being made";
  }
  detail_->setText(detail);
}

void CarrierPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("carrier_index", carrier_index_->value());
  config.mapSetValue("length", length_->value());
  config.mapSetValue("bend_radius", bend_radius_->value());
  config.mapSetValue("safety_margin", safety_margin_->value());
}

void CarrierPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  float value = 0.0f;
  int count = 0;
  if (config.mapGetInt("carrier_index", &count))
  {
    carrier_index_->setValue(count);
  }
  if (config.mapGetFloat("length", &value))
  {
    length_->setValue(value);
  }
  if (config.mapGetFloat("bend_radius", &value))
  {
    bend_radius_->setValue(value);
  }
  if (config.mapGetFloat("safety_margin", &value))
  {
    safety_margin_->setValue(value);
  }
}

}  // namespace moveit_cable_carrier

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_cable_carrier::CarrierPanel, rviz_common::Panel)
