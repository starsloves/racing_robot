// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice

#ifndef VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__TRAITS_HPP_
#define VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "voice_driver/srv/detail/play_voice__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace voice_driver
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayVoice_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: voice_id
  {
    out << "voice_id: ";
    rosidl_generator_traits::value_to_yaml(msg.voice_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayVoice_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voice_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voice_id: ";
    rosidl_generator_traits::value_to_yaml(msg.voice_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayVoice_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace voice_driver

namespace rosidl_generator_traits
{

[[deprecated("use voice_driver::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const voice_driver::srv::PlayVoice_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  voice_driver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use voice_driver::srv::to_yaml() instead")]]
inline std::string to_yaml(const voice_driver::srv::PlayVoice_Request & msg)
{
  return voice_driver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<voice_driver::srv::PlayVoice_Request>()
{
  return "voice_driver::srv::PlayVoice_Request";
}

template<>
inline const char * name<voice_driver::srv::PlayVoice_Request>()
{
  return "voice_driver/srv/PlayVoice_Request";
}

template<>
struct has_fixed_size<voice_driver::srv::PlayVoice_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<voice_driver::srv::PlayVoice_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<voice_driver::srv::PlayVoice_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace voice_driver
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayVoice_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayVoice_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayVoice_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace voice_driver

namespace rosidl_generator_traits
{

[[deprecated("use voice_driver::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const voice_driver::srv::PlayVoice_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  voice_driver::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use voice_driver::srv::to_yaml() instead")]]
inline std::string to_yaml(const voice_driver::srv::PlayVoice_Response & msg)
{
  return voice_driver::srv::to_yaml(msg);
}

template<>
inline const char * data_type<voice_driver::srv::PlayVoice_Response>()
{
  return "voice_driver::srv::PlayVoice_Response";
}

template<>
inline const char * name<voice_driver::srv::PlayVoice_Response>()
{
  return "voice_driver/srv/PlayVoice_Response";
}

template<>
struct has_fixed_size<voice_driver::srv::PlayVoice_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<voice_driver::srv::PlayVoice_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<voice_driver::srv::PlayVoice_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<voice_driver::srv::PlayVoice>()
{
  return "voice_driver::srv::PlayVoice";
}

template<>
inline const char * name<voice_driver::srv::PlayVoice>()
{
  return "voice_driver/srv/PlayVoice";
}

template<>
struct has_fixed_size<voice_driver::srv::PlayVoice>
  : std::integral_constant<
    bool,
    has_fixed_size<voice_driver::srv::PlayVoice_Request>::value &&
    has_fixed_size<voice_driver::srv::PlayVoice_Response>::value
  >
{
};

template<>
struct has_bounded_size<voice_driver::srv::PlayVoice>
  : std::integral_constant<
    bool,
    has_bounded_size<voice_driver::srv::PlayVoice_Request>::value &&
    has_bounded_size<voice_driver::srv::PlayVoice_Response>::value
  >
{
};

template<>
struct is_service<voice_driver::srv::PlayVoice>
  : std::true_type
{
};

template<>
struct is_service_request<voice_driver::srv::PlayVoice_Request>
  : std::true_type
{
};

template<>
struct is_service_response<voice_driver::srv::PlayVoice_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__TRAITS_HPP_
