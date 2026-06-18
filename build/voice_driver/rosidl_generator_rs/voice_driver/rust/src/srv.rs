#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to voice_driver__srv__PlayVoice_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlayVoice_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub voice_id: i32,

}



impl Default for PlayVoice_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PlayVoice_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PlayVoice_Request {
  type RmwMsg = super::srv::rmw::PlayVoice_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        voice_id: msg.voice_id,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      voice_id: msg.voice_id,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      voice_id: msg.voice_id,
    }
  }
}


// Corresponds to voice_driver__srv__PlayVoice_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlayVoice_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for PlayVoice_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PlayVoice_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PlayVoice_Response {
  type RmwMsg = super::srv::rmw::PlayVoice_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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


