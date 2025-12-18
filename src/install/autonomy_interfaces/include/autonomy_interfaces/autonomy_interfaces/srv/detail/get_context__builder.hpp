// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:srv/GetContext.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__BUILDER_HPP_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/srv/detail/get_context__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetContext_Request>()
{
  return ::autonomy_interfaces::srv::GetContext_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autonomy_interfaces


namespace autonomy_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetContext_Response_context
{
public:
  Init_GetContext_Response_context()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autonomy_interfaces::srv::GetContext_Response context(::autonomy_interfaces::srv::GetContext_Response::_context_type arg)
  {
    msg_.context = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::srv::GetContext_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::srv::GetContext_Response>()
{
  return autonomy_interfaces::srv::builder::Init_GetContext_Response_context();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_CONTEXT__BUILDER_HPP_
