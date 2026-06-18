#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "voice_driver__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__voice_driver__srv__PlayVoice_Request() -> *const std::ffi::c_void;
}

#[link(name = "voice_driver__rosidl_generator_c")]
extern "C" {
    fn voice_driver__srv__PlayVoice_Request__init(msg: *mut PlayVoice_Request) -> bool;
    fn voice_driver__srv__PlayVoice_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Request>, size: usize) -> bool;
    fn voice_driver__srv__PlayVoice_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Request>);
    fn voice_driver__srv__PlayVoice_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlayVoice_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Request>) -> bool;
}

// Corresponds to voice_driver__srv__PlayVoice_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlayVoice_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub voice_id: i32,

}



impl Default for PlayVoice_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !voice_driver__srv__PlayVoice_Request__init(&mut msg as *mut _) {
        panic!("Call to voice_driver__srv__PlayVoice_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlayVoice_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlayVoice_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlayVoice_Request where Self: Sized {
  const TYPE_NAME: &'static str = "voice_driver/srv/PlayVoice_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__voice_driver__srv__PlayVoice_Request() }
  }
}


#[link(name = "voice_driver__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__voice_driver__srv__PlayVoice_Response() -> *const std::ffi::c_void;
}

#[link(name = "voice_driver__rosidl_generator_c")]
extern "C" {
    fn voice_driver__srv__PlayVoice_Response__init(msg: *mut PlayVoice_Response) -> bool;
    fn voice_driver__srv__PlayVoice_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Response>, size: usize) -> bool;
    fn voice_driver__srv__PlayVoice_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Response>);
    fn voice_driver__srv__PlayVoice_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlayVoice_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlayVoice_Response>) -> bool;
}

// Corresponds to voice_driver__srv__PlayVoice_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlayVoice_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for PlayVoice_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !voice_driver__srv__PlayVoice_Response__init(&mut msg as *mut _) {
        panic!("Call to voice_driver__srv__PlayVoice_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlayVoice_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { voice_driver__srv__PlayVoice_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlayVoice_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlayVoice_Response where Self: Sized {
  const TYPE_NAME: &'static str = "voice_driver/srv/PlayVoice_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__voice_driver__srv__PlayVoice_Response() }
  }
}






#[link(name = "voice_driver__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__voice_driver__srv__PlayVoice() -> *const std::ffi::c_void;
}

// Corresponds to voice_driver__srv__PlayVoice
#[allow(missing_docs, non_camel_case_types)]
pub struct PlayVoice;

impl rosidl_runtime_rs::Service for PlayVoice {
    type Request = PlayVoice_Request;
    type Response = PlayVoice_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__voice_driver__srv__PlayVoice() }
    }
}


