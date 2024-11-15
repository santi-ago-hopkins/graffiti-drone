// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drone_msgs__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__drone_msgs__msg__MotorCommand __declspec(deprecated)
#endif

namespace drone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor1 = 0ll;
      this->motor2 = 0ll;
      this->motor3 = 0ll;
      this->motor4 = 0ll;
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor1 = 0ll;
      this->motor2 = 0ll;
      this->motor3 = 0ll;
      this->motor4 = 0ll;
    }
  }

  // field types and members
  using _motor1_type =
    int64_t;
  _motor1_type motor1;
  using _motor2_type =
    int64_t;
  _motor2_type motor2;
  using _motor3_type =
    int64_t;
  _motor3_type motor3;
  using _motor4_type =
    int64_t;
  _motor4_type motor4;

  // setters for named parameter idiom
  Type & set__motor1(
    const int64_t & _arg)
  {
    this->motor1 = _arg;
    return *this;
  }
  Type & set__motor2(
    const int64_t & _arg)
  {
    this->motor2 = _arg;
    return *this;
  }
  Type & set__motor3(
    const int64_t & _arg)
  {
    this->motor3 = _arg;
    return *this;
  }
  Type & set__motor4(
    const int64_t & _arg)
  {
    this->motor4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_msgs__msg__MotorCommand
    std::shared_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_msgs__msg__MotorCommand
    std::shared_ptr<drone_msgs::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->motor1 != other.motor1) {
      return false;
    }
    if (this->motor2 != other.motor2) {
      return false;
    }
    if (this->motor3 != other.motor3) {
      return false;
    }
    if (this->motor4 != other.motor4) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  drone_msgs::msg::MotorCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace drone_msgs

#endif  // DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
