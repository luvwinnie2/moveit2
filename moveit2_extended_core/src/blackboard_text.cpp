// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/blackboard_text.hpp>

#include <typeinfo>

namespace moveit2_extended
{

std::string anyToText(const BT::Any& value)
{
  if (value.empty())
  {
    return "<empty>";
  }

  // BT stores every integral as int64_t, so a bool that came off the blackboard would print as
  // 1 or 0. type() keeps the type it was *given*, which is what lets us tell them apart.
  if (value.type() == typeid(bool))
  {
    try
    {
      return value.cast<int64_t>() != 0 ? "true" : "false";
    }
    catch (const std::exception&)
    {
      return "<bool>";
    }
  }

  if (value.isString() || value.isNumber())
  {
    try
    {
      return value.cast<std::string>();
    }
    catch (const std::exception&)
    {
      // fall through to the type name
    }
  }

  return std::string("<") + value.type().name() + ">";
}

std::string expandBlackboardReferences(const BT::Blackboard::Ptr& blackboard, const std::string& text)
{
  std::string out;
  out.reserve(text.size());

  for (std::size_t i = 0; i < text.size();)
  {
    const char c = text[i];

    if (c == '{' && i + 1 < text.size() && text[i + 1] == '{')
    {
      out += '{';
      i += 2;
      continue;
    }
    if (c == '}' && i + 1 < text.size() && text[i + 1] == '}')
    {
      out += '}';
      i += 2;
      continue;
    }
    if (c != '{')
    {
      out += c;
      ++i;
      continue;
    }

    const std::size_t close = text.find('}', i + 1);
    if (close == std::string::npos)
    {
      // Unterminated: the rest is literal. Better a stray brace in a log line than a message
      // truncated at the point a user forgot to close it.
      out.append(text, i, std::string::npos);
      break;
    }

    const std::string key = text.substr(i + 1, close - i - 1);
    const bool plausible_key =
        !key.empty() && key.find_first_of("{} \t\n") == std::string::npos;

    const BT::Any* entry = (plausible_key && blackboard) ? blackboard->getAny(key) : nullptr;
    if (entry != nullptr)
    {
      out += anyToText(*entry);
    }
    else
    {
      // Left verbatim on purpose -- see the header. A typo'd key stays visible.
      out.append(text, i, close - i + 1);
    }
    i = close + 1;
  }

  return out;
}

}  // namespace moveit2_extended
