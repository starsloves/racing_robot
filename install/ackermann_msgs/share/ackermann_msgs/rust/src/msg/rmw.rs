#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ackermann_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ackermann_msgs__msg__AckermannDrive() -> *const std::ffi::c_void;
}

#[link(name = "ackermann_msgs__rosidl_generator_c")]
extern "C" {
    fn ackermann_msgs__msg__AckermannDrive__init(msg: *mut AckermannDrive) -> bool;
    fn ackermann_msgs__msg__AckermannDrive__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AckermannDrive>, size: usize) -> bool;
    fn ackermann_msgs__msg__AckermannDrive__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AckermannDrive>);
    fn ackermann_msgs__msg__AckermannDrive__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AckermannDrive>, out_seq: *mut rosidl_runtime_rs::Sequence<AckermannDrive>) -> bool;
}

// Corresponds to ackermann_msgs__msg__AckermannDrive
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Driving command for a car-like vehicle using Ackermann steering.
///  $Id$

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AckermannDrive {
    /// Assumes Ackermann front-wheel steering. The left and right front
    /// wheels are generally at different angles. To simplify, the commanded
    /// angle corresponds to the yaw of a virtual wheel located at the
    /// center of the front axle, like on a tricycle.  Positive yaw is to
    /// the left. (This is *not* the angle of the steering wheel inside the
    /// passenger compartment.)
    ///
    /// Zero steering angle velocity means change the steering angle as
    /// quickly as possible. Positive velocity indicates a desired absolute
    /// rate of change either left or right. The controller tries not to
    /// exceed this limit in either direction, but sometimes it might.
    ///
    /// Drive at requested speed, acceleration and jerk (the 1st, 2nd and
    /// 3rd derivatives of position). All are measured at the vehicle's
    /// center of rotation, typically the center of the rear axle. The
    /// controller tries not to exceed these limits in either direction, but
    /// sometimes it might.
    ///
    /// Speed is the desired scalar magnitude of the velocity vector.
    /// Direction is forward unless the sign is negative, indicating reverse.
    ///
    /// Zero acceleration means change speed as quickly as
    /// possible. Positive acceleration indicates a desired absolute
    /// magnitude; that includes deceleration.
    ///
    /// Zero jerk means change acceleration as quickly as possible. Positive
    /// jerk indicates a desired absolute rate of acceleration change in
    /// either direction (increasing or decreasing).
    ///
    /// desired virtual angle (radians)
    pub steering_angle: f32,

    /// desired rate of change (radians/s)
    pub steering_angle_velocity: f32,

    /// desired forward speed (m/s)
    pub speed: f32,

    /// desired acceleration (m/s^2)
    pub acceleration: f32,

    /// desired jerk (m/s^3)
    pub jerk: f32,

}



impl Default for AckermannDrive {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ackermann_msgs__msg__AckermannDrive__init(&mut msg as *mut _) {
        panic!("Call to ackermann_msgs__msg__AckermannDrive__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AckermannDrive {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDrive__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDrive__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDrive__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AckermannDrive {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AckermannDrive where Self: Sized {
  const TYPE_NAME: &'static str = "ackermann_msgs/msg/AckermannDrive";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ackermann_msgs__msg__AckermannDrive() }
  }
}


#[link(name = "ackermann_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ackermann_msgs__msg__AckermannDriveStamped() -> *const std::ffi::c_void;
}

#[link(name = "ackermann_msgs__rosidl_generator_c")]
extern "C" {
    fn ackermann_msgs__msg__AckermannDriveStamped__init(msg: *mut AckermannDriveStamped) -> bool;
    fn ackermann_msgs__msg__AckermannDriveStamped__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AckermannDriveStamped>, size: usize) -> bool;
    fn ackermann_msgs__msg__AckermannDriveStamped__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AckermannDriveStamped>);
    fn ackermann_msgs__msg__AckermannDriveStamped__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AckermannDriveStamped>, out_seq: *mut rosidl_runtime_rs::Sequence<AckermannDriveStamped>) -> bool;
}

// Corresponds to ackermann_msgs__msg__AckermannDriveStamped
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Time stamped drive command for robots with Ackermann steering.
///  $Id$

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AckermannDriveStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub drive: super::super::msg::rmw::AckermannDrive,

}



impl Default for AckermannDriveStamped {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ackermann_msgs__msg__AckermannDriveStamped__init(&mut msg as *mut _) {
        panic!("Call to ackermann_msgs__msg__AckermannDriveStamped__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AckermannDriveStamped {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDriveStamped__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDriveStamped__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ackermann_msgs__msg__AckermannDriveStamped__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AckermannDriveStamped {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AckermannDriveStamped where Self: Sized {
  const TYPE_NAME: &'static str = "ackermann_msgs/msg/AckermannDriveStamped";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ackermann_msgs__msg__AckermannDriveStamped() }
  }
}


