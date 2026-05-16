#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to origincar_base__msg__Position

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Position::default())
  }
}

impl rosidl_runtime_rs::Message for Position {
  type RmwMsg = super::msg::rmw::Position;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angle_x: msg.angle_x,
        angle_y: msg.angle_y,
        distance: msg.distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      angle_x: msg.angle_x,
      angle_y: msg.angle_y,
      distance: msg.distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angle_x: msg.angle_x,
      angle_y: msg.angle_y,
      distance: msg.distance,
    }
  }
}


