// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_motor4
{
public:
  explicit Init_MotorCommand_motor4(::drone_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::drone_msgs::msg::MotorCommand motor4(::drone_msgs::msg::MotorCommand::_motor4_type arg)
  {
    msg_.motor4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_motor3
{
public:
  explicit Init_MotorCommand_motor3(::drone_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_motor4 motor3(::drone_msgs::msg::MotorCommand::_motor3_type arg)
  {
    msg_.motor3 = std::move(arg);
    return Init_MotorCommand_motor4(msg_);
  }

private:
  ::drone_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_motor2
{
public:
  explicit Init_MotorCommand_motor2(::drone_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_motor3 motor2(::drone_msgs::msg::MotorCommand::_motor2_type arg)
  {
    msg_.motor2 = std::move(arg);
    return Init_MotorCommand_motor3(msg_);
  }

private:
  ::drone_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_motor1
{
public:
  Init_MotorCommand_motor1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_motor2 motor1(::drone_msgs::msg::MotorCommand::_motor1_type arg)
  {
    msg_.motor1 = std::move(arg);
    return Init_MotorCommand_motor2(msg_);
  }

private:
  ::drone_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_msgs::msg::MotorCommand>()
{
  return drone_msgs::msg::builder::Init_MotorCommand_motor1();
}

}  // namespace drone_msgs

#endif  // DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
