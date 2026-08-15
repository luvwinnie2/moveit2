// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

#include <map>
#include <memory>
#include <mutex>
#include <string>

// Forward declared at global scope on purpose: an elaborated `class QGridLayout*` written inside
// the namespace below would declare moveit_cable_carrier::QGridLayout, not Qt's.
class QGridLayout;

namespace moveit_cable_carrier
{

/** RViz panel for trying cable-carrier settings against the live robot pose.
 *
 *  Every field maps to a ROS 2 parameter on the carrier_visualizer node, so changing one re-solves
 *  the carrier immediately and the markers in the 3D view update. The point is to answer "would a
 *  longer run, or a bigger bend radius, or a thinner cable fix this?" in seconds, instead of
 *  editing YAML and restarting move_group for every trial.
 *
 *  The readouts below the controls are the ones that decide whether a design works: the achieved
 *  bend radius against the carrier's own limit, the torsion stop, and -- usually the binding
 *  one -- how the radius compares with what the cable inside needs. */
class CarrierPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CarrierPanel(QWidget* parent = nullptr);
  ~CarrierPanel() override = default;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

private Q_SLOTS:
  /** Push the current widget values to the visualizer node. */
  void apply();
  /** Pull the node's current parameter values back into the widgets. */
  void refreshFromNode();
  /** Repaint the readouts. Runs on the GUI thread; the subscription only caches a string, because
   *  touching Qt widgets from the executor thread is not safe. */
  void updateReadouts();

private:
  QDoubleSpinBox* addRow(QGridLayout* grid, int row, const QString& label, double min, double max,
                         double step, int decimals, const QString& suffix, const QString& tip);

  rclcpp::Node::SharedPtr node_;
  rclcpp::AsyncParametersClient::SharedPtr params_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  QSpinBox* carrier_index_{ nullptr };
  QSpinBox* num_segments_{ nullptr };
  QDoubleSpinBox* length_{ nullptr };
  QDoubleSpinBox* bend_radius_{ nullptr };
  QDoubleSpinBox* safety_margin_{ nullptr };
  QDoubleSpinBox* twist_limit_{ nullptr };
  QDoubleSpinBox* gravity_sag_{ nullptr };
  QDoubleSpinBox* energy_relaxation_{ nullptr };
  QDoubleSpinBox* cable_od_{ nullptr };
  QDoubleSpinBox* cable_factor_{ nullptr };
  QCheckBox* show_all_{ nullptr };
  QPushButton* apply_{ nullptr };
  QPushButton* reload_{ nullptr };
  QLabel* verdict_{ nullptr };
  QLabel* detail_{ nullptr };

  mutable std::mutex mutex_;
  std::string status_;
  bool status_seen_{ false };
};

}  // namespace moveit_cable_carrier
