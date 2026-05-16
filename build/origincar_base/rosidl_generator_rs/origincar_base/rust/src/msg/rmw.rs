#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "origincar_base__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__origincar_base__msg__Position() -> *const std::ffi::c_void;
}

#[link(name = "origincar_base__rosidl_generator_c")]
extern "C" {
    fn origincar_base__msg__Position__init(msg: *mut Position) -> bool;
    fn origincar_base__msg__Position__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Position>, size: usize) -> bool;
    fn origincar_base__msg__Position__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Position>);
    fn origincar_base__msg__Position__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Position>, out_seq: *mut rosidl_runtime_rs::Sequence<Position>) -> bool;
}

// Corresponds to origincar_base__msg__Position
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Position {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angle_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angle_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f32,

}



impl Default for Position {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !origincar_base__msg__Position__init(&mut msg as *mut _) {
        panic!("Call to origincar_base__msg__Position__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Position {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { origincar_base__msg__Position__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { origincar_base__msg__Position__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { origincar_base__msg__Position__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Position {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Position where Self: Sized {
  const TYPE_NAME: &'static str = "origincar_base/msg/Position";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__origincar_base__msg__Position() }
  }
}


