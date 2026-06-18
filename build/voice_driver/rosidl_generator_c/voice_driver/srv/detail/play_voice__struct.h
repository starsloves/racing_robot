// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice

#ifndef VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__STRUCT_H_
#define VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PlayVoice in the package voice_driver.
typedef struct voice_driver__srv__PlayVoice_Request
{
  int32_t voice_id;
} voice_driver__srv__PlayVoice_Request;

// Struct for a sequence of voice_driver__srv__PlayVoice_Request.
typedef struct voice_driver__srv__PlayVoice_Request__Sequence
{
  voice_driver__srv__PlayVoice_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} voice_driver__srv__PlayVoice_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlayVoice in the package voice_driver.
typedef struct voice_driver__srv__PlayVoice_Response
{
  bool success;
  rosidl_runtime_c__String message;
} voice_driver__srv__PlayVoice_Response;

// Struct for a sequence of voice_driver__srv__PlayVoice_Response.
typedef struct voice_driver__srv__PlayVoice_Response__Sequence
{
  voice_driver__srv__PlayVoice_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} voice_driver__srv__PlayVoice_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__STRUCT_H_
