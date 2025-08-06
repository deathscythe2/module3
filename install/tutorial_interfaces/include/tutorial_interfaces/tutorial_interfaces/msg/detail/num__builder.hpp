// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__NUM__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Num_data
{
public:
  explicit Init_Num_data(::tutorial_interfaces::msg::Num & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Num data(::tutorial_interfaces::msg::Num::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Num msg_;
};

class Init_Num_radius
{
public:
  explicit Init_Num_radius(::tutorial_interfaces::msg::Num & msg)
  : msg_(msg)
  {}
  Init_Num_data radius(::tutorial_interfaces::msg::Num::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_Num_data(msg_);
  }

private:
  ::tutorial_interfaces::msg::Num msg_;
};

class Init_Num_diameter
{
public:
  Init_Num_diameter()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Num_radius diameter(::tutorial_interfaces::msg::Num::_diameter_type arg)
  {
    msg_.diameter = std::move(arg);
    return Init_Num_radius(msg_);
  }

private:
  ::tutorial_interfaces::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Num>()
{
  return tutorial_interfaces::msg::builder::Init_Num_diameter();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__NUM__BUILDER_HPP_
