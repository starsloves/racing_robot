// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "voice_driver/srv/detail/play_voice__rosidl_typesupport_introspection_c.h"
#include "voice_driver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "voice_driver/srv/detail/play_voice__functions.h"
#include "voice_driver/srv/detail/play_voice__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  voice_driver__srv__PlayVoice_Request__init(message_memory);
}

void voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_fini_function(void * message_memory)
{
  voice_driver__srv__PlayVoice_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_member_array[1] = {
  {
    "voice_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(voice_driver__srv__PlayVoice_Request, voice_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_members = {
  "voice_driver__srv",  // message namespace
  "PlayVoice_Request",  // message name
  1,  // number of fields
  sizeof(voice_driver__srv__PlayVoice_Request),
  voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_member_array,  // message members
  voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_type_support_handle = {
  0,
  &voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_voice_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Request)() {
  if (!voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_type_support_handle.typesupport_identifier) {
    voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &voice_driver__srv__PlayVoice_Request__rosidl_typesupport_introspection_c__PlayVoice_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "voice_driver/srv/detail/play_voice__rosidl_typesupport_introspection_c.h"
// already included above
// #include "voice_driver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "voice_driver/srv/detail/play_voice__functions.h"
// already included above
// #include "voice_driver/srv/detail/play_voice__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  voice_driver__srv__PlayVoice_Response__init(message_memory);
}

void voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_fini_function(void * message_memory)
{
  voice_driver__srv__PlayVoice_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(voice_driver__srv__PlayVoice_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(voice_driver__srv__PlayVoice_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_members = {
  "voice_driver__srv",  // message namespace
  "PlayVoice_Response",  // message name
  2,  // number of fields
  sizeof(voice_driver__srv__PlayVoice_Response),
  voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_member_array,  // message members
  voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_type_support_handle = {
  0,
  &voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_voice_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Response)() {
  if (!voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_type_support_handle.typesupport_identifier) {
    voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &voice_driver__srv__PlayVoice_Response__rosidl_typesupport_introspection_c__PlayVoice_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "voice_driver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "voice_driver/srv/detail/play_voice__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_members = {
  "voice_driver__srv",  // service namespace
  "PlayVoice",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_Request_message_type_support_handle,
  NULL  // response message
  // voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_Response_message_type_support_handle
};

static rosidl_service_type_support_t voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_type_support_handle = {
  0,
  &voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_voice_driver
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice)() {
  if (!voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_type_support_handle.typesupport_identifier) {
    voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, voice_driver, srv, PlayVoice_Response)()->data;
  }

  return &voice_driver__srv__detail__play_voice__rosidl_typesupport_introspection_c__PlayVoice_service_type_support_handle;
}
