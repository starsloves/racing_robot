// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice

#ifndef VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__FUNCTIONS_H_
#define VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "voice_driver/msg/rosidl_generator_c__visibility_control.h"

#include "voice_driver/srv/detail/play_voice__struct.h"

/// Initialize srv/PlayVoice message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * voice_driver__srv__PlayVoice_Request
 * )) before or use
 * voice_driver__srv__PlayVoice_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__init(voice_driver__srv__PlayVoice_Request * msg);

/// Finalize srv/PlayVoice message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Request__fini(voice_driver__srv__PlayVoice_Request * msg);

/// Create srv/PlayVoice message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * voice_driver__srv__PlayVoice_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
voice_driver__srv__PlayVoice_Request *
voice_driver__srv__PlayVoice_Request__create();

/// Destroy srv/PlayVoice message.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Request__destroy(voice_driver__srv__PlayVoice_Request * msg);

/// Check for srv/PlayVoice message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__are_equal(const voice_driver__srv__PlayVoice_Request * lhs, const voice_driver__srv__PlayVoice_Request * rhs);

/// Copy a srv/PlayVoice message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__copy(
  const voice_driver__srv__PlayVoice_Request * input,
  voice_driver__srv__PlayVoice_Request * output);

/// Initialize array of srv/PlayVoice messages.
/**
 * It allocates the memory for the number of elements and calls
 * voice_driver__srv__PlayVoice_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__Sequence__init(voice_driver__srv__PlayVoice_Request__Sequence * array, size_t size);

/// Finalize array of srv/PlayVoice messages.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Request__Sequence__fini(voice_driver__srv__PlayVoice_Request__Sequence * array);

/// Create array of srv/PlayVoice messages.
/**
 * It allocates the memory for the array and calls
 * voice_driver__srv__PlayVoice_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
voice_driver__srv__PlayVoice_Request__Sequence *
voice_driver__srv__PlayVoice_Request__Sequence__create(size_t size);

/// Destroy array of srv/PlayVoice messages.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Request__Sequence__destroy(voice_driver__srv__PlayVoice_Request__Sequence * array);

/// Check for srv/PlayVoice message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__Sequence__are_equal(const voice_driver__srv__PlayVoice_Request__Sequence * lhs, const voice_driver__srv__PlayVoice_Request__Sequence * rhs);

/// Copy an array of srv/PlayVoice messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Request__Sequence__copy(
  const voice_driver__srv__PlayVoice_Request__Sequence * input,
  voice_driver__srv__PlayVoice_Request__Sequence * output);

/// Initialize srv/PlayVoice message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * voice_driver__srv__PlayVoice_Response
 * )) before or use
 * voice_driver__srv__PlayVoice_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__init(voice_driver__srv__PlayVoice_Response * msg);

/// Finalize srv/PlayVoice message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Response__fini(voice_driver__srv__PlayVoice_Response * msg);

/// Create srv/PlayVoice message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * voice_driver__srv__PlayVoice_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
voice_driver__srv__PlayVoice_Response *
voice_driver__srv__PlayVoice_Response__create();

/// Destroy srv/PlayVoice message.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Response__destroy(voice_driver__srv__PlayVoice_Response * msg);

/// Check for srv/PlayVoice message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__are_equal(const voice_driver__srv__PlayVoice_Response * lhs, const voice_driver__srv__PlayVoice_Response * rhs);

/// Copy a srv/PlayVoice message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__copy(
  const voice_driver__srv__PlayVoice_Response * input,
  voice_driver__srv__PlayVoice_Response * output);

/// Initialize array of srv/PlayVoice messages.
/**
 * It allocates the memory for the number of elements and calls
 * voice_driver__srv__PlayVoice_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__Sequence__init(voice_driver__srv__PlayVoice_Response__Sequence * array, size_t size);

/// Finalize array of srv/PlayVoice messages.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Response__Sequence__fini(voice_driver__srv__PlayVoice_Response__Sequence * array);

/// Create array of srv/PlayVoice messages.
/**
 * It allocates the memory for the array and calls
 * voice_driver__srv__PlayVoice_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
voice_driver__srv__PlayVoice_Response__Sequence *
voice_driver__srv__PlayVoice_Response__Sequence__create(size_t size);

/// Destroy array of srv/PlayVoice messages.
/**
 * It calls
 * voice_driver__srv__PlayVoice_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
void
voice_driver__srv__PlayVoice_Response__Sequence__destroy(voice_driver__srv__PlayVoice_Response__Sequence * array);

/// Check for srv/PlayVoice message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__Sequence__are_equal(const voice_driver__srv__PlayVoice_Response__Sequence * lhs, const voice_driver__srv__PlayVoice_Response__Sequence * rhs);

/// Copy an array of srv/PlayVoice messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_voice_driver
bool
voice_driver__srv__PlayVoice_Response__Sequence__copy(
  const voice_driver__srv__PlayVoice_Response__Sequence * input,
  voice_driver__srv__PlayVoice_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VOICE_DRIVER__SRV__DETAIL__PLAY_VOICE__FUNCTIONS_H_
