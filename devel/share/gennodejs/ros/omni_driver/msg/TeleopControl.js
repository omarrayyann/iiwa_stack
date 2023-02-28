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

class TeleopControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_joint = null;
      this.vel_effector = null;
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('vel_joint')) {
        this.vel_joint = initObj.vel_joint
      }
      else {
        this.vel_joint = [];
      }
      if (initObj.hasOwnProperty('vel_effector')) {
        this.vel_effector = initObj.vel_effector
      }
      else {
        this.vel_effector = [];
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TeleopControl
    // Serialize message field [vel_joint]
    bufferOffset = _arraySerializer.float64(obj.vel_joint, buffer, bufferOffset, null);
    // Serialize message field [vel_effector]
    bufferOffset = _arraySerializer.float64(obj.vel_effector, buffer, bufferOffset, null);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TeleopControl
    let len;
    let data = new TeleopControl(null);
    // Deserialize message field [vel_joint]
    data.vel_joint = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [vel_effector]
    data.vel_effector = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.vel_joint.length;
    length += 8 * object.vel_effector.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'omni_driver/TeleopControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'acc00fe3a847581a668e7968e27d235a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] vel_joint
    float64[] vel_effector
    uint8 mode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TeleopControl(null);
    if (msg.vel_joint !== undefined) {
      resolved.vel_joint = msg.vel_joint;
    }
    else {
      resolved.vel_joint = []
    }

    if (msg.vel_effector !== undefined) {
      resolved.vel_effector = msg.vel_effector;
    }
    else {
      resolved.vel_effector = []
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

module.exports = TeleopControl;
