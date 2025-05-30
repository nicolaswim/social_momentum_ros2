// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "my_social_nav_interfaces/msg/detail/human_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_social_nav_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HumanArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_social_nav_interfaces::msg::HumanArray(_init);
}

void HumanArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_social_nav_interfaces::msg::HumanArray *>(message_memory);
  typed_message->~HumanArray();
}

size_t size_function__HumanArray__humans(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<my_social_nav_interfaces::msg::HumanPoseVel> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HumanArray__humans(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<my_social_nav_interfaces::msg::HumanPoseVel> *>(untyped_member);
  return &member[index];
}

void * get_function__HumanArray__humans(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<my_social_nav_interfaces::msg::HumanPoseVel> *>(untyped_member);
  return &member[index];
}

void fetch_function__HumanArray__humans(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const my_social_nav_interfaces::msg::HumanPoseVel *>(
    get_const_function__HumanArray__humans(untyped_member, index));
  auto & value = *reinterpret_cast<my_social_nav_interfaces::msg::HumanPoseVel *>(untyped_value);
  value = item;
}

void assign_function__HumanArray__humans(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<my_social_nav_interfaces::msg::HumanPoseVel *>(
    get_function__HumanArray__humans(untyped_member, index));
  const auto & value = *reinterpret_cast<const my_social_nav_interfaces::msg::HumanPoseVel *>(untyped_value);
  item = value;
}

void resize_function__HumanArray__humans(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<my_social_nav_interfaces::msg::HumanPoseVel> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HumanArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces::msg::HumanArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "humans",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_social_nav_interfaces::msg::HumanPoseVel>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces::msg::HumanArray, humans),  // bytes offset in struct
    nullptr,  // default value
    size_function__HumanArray__humans,  // size() function pointer
    get_const_function__HumanArray__humans,  // get_const(index) function pointer
    get_function__HumanArray__humans,  // get(index) function pointer
    fetch_function__HumanArray__humans,  // fetch(index, &value) function pointer
    assign_function__HumanArray__humans,  // assign(index, value) function pointer
    resize_function__HumanArray__humans  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HumanArray_message_members = {
  "my_social_nav_interfaces::msg",  // message namespace
  "HumanArray",  // message name
  2,  // number of fields
  sizeof(my_social_nav_interfaces::msg::HumanArray),
  HumanArray_message_member_array,  // message members
  HumanArray_init_function,  // function to initialize message memory (memory has to be allocated)
  HumanArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HumanArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HumanArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace my_social_nav_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_social_nav_interfaces::msg::HumanArray>()
{
  return &::my_social_nav_interfaces::msg::rosidl_typesupport_introspection_cpp::HumanArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_social_nav_interfaces, msg, HumanArray)() {
  return &::my_social_nav_interfaces::msg::rosidl_typesupport_introspection_cpp::HumanArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
