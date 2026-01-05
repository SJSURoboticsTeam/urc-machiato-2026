// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autonomy_interfaces:msg/QoSTopicProfile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_topic_profile.hpp"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__BUILDER_HPP_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autonomy_interfaces/msg/detail/qo_s_topic_profile__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autonomy_interfaces
{

namespace msg
{

namespace builder
{

class Init_QoSTopicProfile_samples_count
{
public:
  explicit Init_QoSTopicProfile_samples_count(::autonomy_interfaces::msg::QoSTopicProfile & msg)
  : msg_(msg)
  {}
  ::autonomy_interfaces::msg::QoSTopicProfile samples_count(::autonomy_interfaces::msg::QoSTopicProfile::_samples_count_type arg)
  {
    msg_.samples_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSTopicProfile msg_;
};

class Init_QoSTopicProfile_packet_loss_rate
{
public:
  explicit Init_QoSTopicProfile_packet_loss_rate(::autonomy_interfaces::msg::QoSTopicProfile & msg)
  : msg_(msg)
  {}
  Init_QoSTopicProfile_samples_count packet_loss_rate(::autonomy_interfaces::msg::QoSTopicProfile::_packet_loss_rate_type arg)
  {
    msg_.packet_loss_rate = std::move(arg);
    return Init_QoSTopicProfile_samples_count(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSTopicProfile msg_;
};

class Init_QoSTopicProfile_jitter_ms
{
public:
  explicit Init_QoSTopicProfile_jitter_ms(::autonomy_interfaces::msg::QoSTopicProfile & msg)
  : msg_(msg)
  {}
  Init_QoSTopicProfile_packet_loss_rate jitter_ms(::autonomy_interfaces::msg::QoSTopicProfile::_jitter_ms_type arg)
  {
    msg_.jitter_ms = std::move(arg);
    return Init_QoSTopicProfile_packet_loss_rate(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSTopicProfile msg_;
};

class Init_QoSTopicProfile_avg_latency_ms
{
public:
  explicit Init_QoSTopicProfile_avg_latency_ms(::autonomy_interfaces::msg::QoSTopicProfile & msg)
  : msg_(msg)
  {}
  Init_QoSTopicProfile_jitter_ms avg_latency_ms(::autonomy_interfaces::msg::QoSTopicProfile::_avg_latency_ms_type arg)
  {
    msg_.avg_latency_ms = std::move(arg);
    return Init_QoSTopicProfile_jitter_ms(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSTopicProfile msg_;
};

class Init_QoSTopicProfile_topic_name
{
public:
  Init_QoSTopicProfile_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QoSTopicProfile_avg_latency_ms topic_name(::autonomy_interfaces::msg::QoSTopicProfile::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_QoSTopicProfile_avg_latency_ms(msg_);
  }

private:
  ::autonomy_interfaces::msg::QoSTopicProfile msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autonomy_interfaces::msg::QoSTopicProfile>()
{
  return autonomy_interfaces::msg::builder::Init_QoSTopicProfile_topic_name();
}

}  // namespace autonomy_interfaces

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_TOPIC_PROFILE__BUILDER_HPP_
