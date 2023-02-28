// Auto-generated. Do not edit!

// (in-package omni_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OmniButtonEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grey_button = null;
      this.white_button = null;
      this.grey_button_clicked = null;
      this.white_button_clicked = null;
    }
    else {
      if (initObj.hasOwnProperty('grey_button')) {
        this.grey_button = initObj.grey_button
      }
      else {
        this.grey_button = false;
      }
      if (initObj.hasOwnProperty('white_button')) {
        this.white_button = initObj.white_button
      }
      else {
        this.white_button = false;
      }
      if (initObj.hasOwnProperty('grey_button_clicked')) {
        this.grey_button_clicked = initObj.grey_button_clicked
      }
      else {
        this.grey_button_clicked = false;
      }
      if (initObj.hasOwnProperty('white_button_clicked')) {
        this.white_button_clicked = initObj.white_button_clicked
      }
      else {
        this.white_button_clicked = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OmniButtonEvent
    // Serialize message field [grey_button]
    bufferOffset = _serializer.bool(obj.grey_button, buffer, bufferOffset);
    // Serialize message field [white_button]
    bufferOffset = _serializer.bool(obj.white_button, buffer, bufferOffset);
    // Serialize message field [grey_button_clicked]
    bufferOffset = _serializer.bool(obj.grey_button_clicked, buffer, bufferOffset);
    // Serialize message field [white_button_clicked]
    bufferOffset = _serializer.bool(obj.white_button_clicked, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OmniButtonEvent
    let len;
    let data = new OmniButtonEvent(null);
    // Deserialize message field [grey_button]
    data.grey_button = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [white_button]
    data.white_button = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [grey_button_clicked]
    data.grey_button_clicked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [white_button_clicked]
    data.white_button_clicked = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'omni_driver/OmniButtonEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67ad876972cf46727cadcf266dde0b5c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool grey_button
    bool white_button
    bool grey_button_clicked
    bool white_button_clicked
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OmniButtonEvent(null);
    if (msg.grey_button !== undefined) {
      resolved.grey_button = msg.grey_button;
    }
    else {
      resolved.grey_button = false
    }

    if (msg.white_button !== undefined) {
      resolved.white_button = msg.white_button;
    }
    else {
      resolved.white_button = false
    }

    if (msg.grey_button_clicked !== undefined) {
      resolved.grey_button_clicked = msg.grey_button_clicked;
    }
    else {
      resolved.grey_button_clicked = false
    }

    if (msg.white_button_clicked !== undefined) {
      resolved.white_button_clicked = msg.white_button_clicked;
    }
    else {
      resolved.white_button_clicked = false
    }

    return resolved;
    }
};

module.exports = OmniButtonEvent;
