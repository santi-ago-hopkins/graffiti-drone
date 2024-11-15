// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorCommand in the package drone_msgs.
typedef struct drone_msgs__msg__MotorCommand
{
  int64_t motor1;
  int64_t motor2;
  int64_t motor3;
  int64_t motor4;
} drone_msgs__msg__MotorCommand;

// Struct for a sequence of drone_msgs__msg__MotorCommand.
typedef struct drone_msgs__msg__MotorCommand__Sequence
{
  drone_msgs__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_msgs__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
