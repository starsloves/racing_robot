// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice

#ifndef VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__BUILDER_HPP_
#define VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "voice_driver/srv/detail/play_voice__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace voice_driver
{

namespace srv
{

namespace builder
{

class Init_PlayVoice_Request_voice_id
{
public:
  Init_PlayVoice_Request_voice_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::voice_driver::srv::PlayVoice_Request voice_id(::voice_driver::srv::PlayVoice_Request::_voice_id_type arg)
  {
    msg_.voice_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::voice_driver::srv::PlayVoice_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::voice_driver::srv::PlayVoice_Request>()
{
  return voice_driver::srv::builder::Init_PlayVoice_Request_voice_id();
}

}  // namespace voice_driver


namespace voice_driver
{

namespace srv
{

namespace builder
{

class Init_PlayVoice_Response_message
{
public:
  explicit Init_PlayVoice_Response_message(::voice_driver::srv::PlayVoice_Response & msg)
  : msg_(msg)
  {}
  ::voice_driver::srv::PlayVoice_Response message(::voice_driver::srv::PlayVoice_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::voice_driver::srv::PlayVoice_Response msg_;
};

class Init_PlayVoice_Response_success
{
public:
  Init_PlayVoice_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayVoice_Response_message success(::voice_driver::srv::PlayVoice_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlayVoice_Response_message(msg_);
  }

private:
  ::voice_driver::srv::PlayVoice_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::voice_driver::srv::PlayVoice_Response>()
{
  return voice_driver::srv::builder::Init_PlayVoice_Response_success();
}

}  // namespace voice_driver

#endif  // VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__BUILDER_HPP_
