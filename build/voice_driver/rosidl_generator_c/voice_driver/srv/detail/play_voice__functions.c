// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from voice_driver:srv/PlayVoice.idl
// generated code does not contain a copyright notice
#include "voice_driver/srv/detail/play_voice__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
voice_driver__srv__PlayVoice_Request__init(voice_driver__srv__PlayVoice_Request * msg)
{
  if (!msg) {
    return false;
  }
  // voice_id
  return true;
}

void
voice_driver__srv__PlayVoice_Request__fini(voice_driver__srv__PlayVoice_Request * msg)
{
  if (!msg) {
    return;
  }
  // voice_id
}

bool
voice_driver__srv__PlayVoice_Request__are_equal(const voice_driver__srv__PlayVoice_Request * lhs, const voice_driver__srv__PlayVoice_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // voice_id
  if (lhs->voice_id != rhs->voice_id) {
    return false;
  }
  return true;
}

bool
voice_driver__srv__PlayVoice_Request__copy(
  const voice_driver__srv__PlayVoice_Request * input,
  voice_driver__srv__PlayVoice_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // voice_id
  output->voice_id = input->voice_id;
  return true;
}

voice_driver__srv__PlayVoice_Request *
voice_driver__srv__PlayVoice_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Request * msg = (voice_driver__srv__PlayVoice_Request *)allocator.allocate(sizeof(voice_driver__srv__PlayVoice_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(voice_driver__srv__PlayVoice_Request));
  bool success = voice_driver__srv__PlayVoice_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
voice_driver__srv__PlayVoice_Request__destroy(voice_driver__srv__PlayVoice_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    voice_driver__srv__PlayVoice_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
voice_driver__srv__PlayVoice_Request__Sequence__init(voice_driver__srv__PlayVoice_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Request * data = NULL;

  if (size) {
    data = (voice_driver__srv__PlayVoice_Request *)allocator.zero_allocate(size, sizeof(voice_driver__srv__PlayVoice_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = voice_driver__srv__PlayVoice_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        voice_driver__srv__PlayVoice_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
voice_driver__srv__PlayVoice_Request__Sequence__fini(voice_driver__srv__PlayVoice_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      voice_driver__srv__PlayVoice_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

voice_driver__srv__PlayVoice_Request__Sequence *
voice_driver__srv__PlayVoice_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Request__Sequence * array = (voice_driver__srv__PlayVoice_Request__Sequence *)allocator.allocate(sizeof(voice_driver__srv__PlayVoice_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = voice_driver__srv__PlayVoice_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
voice_driver__srv__PlayVoice_Request__Sequence__destroy(voice_driver__srv__PlayVoice_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    voice_driver__srv__PlayVoice_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
voice_driver__srv__PlayVoice_Request__Sequence__are_equal(const voice_driver__srv__PlayVoice_Request__Sequence * lhs, const voice_driver__srv__PlayVoice_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!voice_driver__srv__PlayVoice_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
voice_driver__srv__PlayVoice_Request__Sequence__copy(
  const voice_driver__srv__PlayVoice_Request__Sequence * input,
  voice_driver__srv__PlayVoice_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(voice_driver__srv__PlayVoice_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    voice_driver__srv__PlayVoice_Request * data =
      (voice_driver__srv__PlayVoice_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!voice_driver__srv__PlayVoice_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          voice_driver__srv__PlayVoice_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!voice_driver__srv__PlayVoice_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
voice_driver__srv__PlayVoice_Response__init(voice_driver__srv__PlayVoice_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    voice_driver__srv__PlayVoice_Response__fini(msg);
    return false;
  }
  return true;
}

void
voice_driver__srv__PlayVoice_Response__fini(voice_driver__srv__PlayVoice_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
voice_driver__srv__PlayVoice_Response__are_equal(const voice_driver__srv__PlayVoice_Response * lhs, const voice_driver__srv__PlayVoice_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
voice_driver__srv__PlayVoice_Response__copy(
  const voice_driver__srv__PlayVoice_Response * input,
  voice_driver__srv__PlayVoice_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

voice_driver__srv__PlayVoice_Response *
voice_driver__srv__PlayVoice_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Response * msg = (voice_driver__srv__PlayVoice_Response *)allocator.allocate(sizeof(voice_driver__srv__PlayVoice_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(voice_driver__srv__PlayVoice_Response));
  bool success = voice_driver__srv__PlayVoice_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
voice_driver__srv__PlayVoice_Response__destroy(voice_driver__srv__PlayVoice_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    voice_driver__srv__PlayVoice_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
voice_driver__srv__PlayVoice_Response__Sequence__init(voice_driver__srv__PlayVoice_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Response * data = NULL;

  if (size) {
    data = (voice_driver__srv__PlayVoice_Response *)allocator.zero_allocate(size, sizeof(voice_driver__srv__PlayVoice_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = voice_driver__srv__PlayVoice_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        voice_driver__srv__PlayVoice_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
voice_driver__srv__PlayVoice_Response__Sequence__fini(voice_driver__srv__PlayVoice_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      voice_driver__srv__PlayVoice_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

voice_driver__srv__PlayVoice_Response__Sequence *
voice_driver__srv__PlayVoice_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  voice_driver__srv__PlayVoice_Response__Sequence * array = (voice_driver__srv__PlayVoice_Response__Sequence *)allocator.allocate(sizeof(voice_driver__srv__PlayVoice_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = voice_driver__srv__PlayVoice_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
voice_driver__srv__PlayVoice_Response__Sequence__destroy(voice_driver__srv__PlayVoice_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    voice_driver__srv__PlayVoice_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
voice_driver__srv__PlayVoice_Response__Sequence__are_equal(const voice_driver__srv__PlayVoice_Response__Sequence * lhs, const voice_driver__srv__PlayVoice_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!voice_driver__srv__PlayVoice_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
voice_driver__srv__PlayVoice_Response__Sequence__copy(
  const voice_driver__srv__PlayVoice_Response__Sequence * input,
  voice_driver__srv__PlayVoice_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(voice_driver__srv__PlayVoice_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    voice_driver__srv__PlayVoice_Response * data =
      (voice_driver__srv__PlayVoice_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!voice_driver__srv__PlayVoice_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          voice_driver__srv__PlayVoice_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!voice_driver__srv__PlayVoice_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
