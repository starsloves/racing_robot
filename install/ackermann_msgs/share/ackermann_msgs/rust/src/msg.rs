#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ackermann_msgs__msg__AckermannDrive
/// Driving command for a car-like vehicle using Ackermann steering.
///  $Id$

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AckermannDrive::default())
  }
}

impl rosidl_runtime_rs::Message for AckermannDrive {
  type RmwMsg = super::msg::rmw::AckermannDrive;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        steering_angle: msg.steering_angle,
        steering_angle_velocity: msg.steering_angle_velocity,
        speed: msg.speed,
        acceleration: msg.acceleration,
        jerk: msg.jerk,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      steering_angle: msg.steering_angle,
      steering_angle_velocity: msg.steering_angle_velocity,
      speed: msg.speed,
      acceleration: msg.acceleration,
      jerk: msg.jerk,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      steering_angle: msg.steering_angle,
      steering_angle_velocity: msg.steering_angle_velocity,
      speed: msg.speed,
      acceleration: msg.acceleration,
      jerk: msg.jerk,
    }
  }
}


// Corresponds to ackermann_msgs__msg__AckermannDriveStamped
/// Time stamped drive command for robots with Ackermann steering.
///  $Id$

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AckermannDriveStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub drive: super::msg::AckermannDrive,

}



impl Default for AckermannDriveStamped {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AckermannDriveStamped::default())
  }
}

impl rosidl_runtime_rs::Message for AckermannDriveStamped {
  type RmwMsg = super::msg::rmw::AckermannDriveStamped;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        drive: super::msg::AckermannDrive::into_rmw_message(std::borrow::Cow::Owned(msg.drive)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        drive: super::msg::AckermannDrive::into_rmw_message(std::borrow::Cow::Borrowed(&msg.drive)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      drive: super::msg::AckermannDrive::from_rmw_message(msg.drive),
    }
  }
}


