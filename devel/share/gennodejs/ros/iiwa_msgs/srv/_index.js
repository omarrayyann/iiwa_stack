
"use strict";

let SetSmartServoJointSpeedLimits = require('./SetSmartServoJointSpeedLimits.js')
let SetPTPCartesianSpeedLimits = require('./SetPTPCartesianSpeedLimits.js')
let SetSmartServoLinSpeedLimits = require('./SetSmartServoLinSpeedLimits.js')
let ConfigureControlMode = require('./ConfigureControlMode.js')
let SetEndpointFrame = require('./SetEndpointFrame.js')
let SetWorkpiece = require('./SetWorkpiece.js')
let TimeToDestination = require('./TimeToDestination.js')
let SetSpeedOverride = require('./SetSpeedOverride.js')
let SetPTPJointSpeedLimits = require('./SetPTPJointSpeedLimits.js')

module.exports = {
  SetSmartServoJointSpeedLimits: SetSmartServoJointSpeedLimits,
  SetPTPCartesianSpeedLimits: SetPTPCartesianSpeedLimits,
  SetSmartServoLinSpeedLimits: SetSmartServoLinSpeedLimits,
  ConfigureControlMode: ConfigureControlMode,
  SetEndpointFrame: SetEndpointFrame,
  SetWorkpiece: SetWorkpiece,
  TimeToDestination: TimeToDestination,
  SetSpeedOverride: SetSpeedOverride,
  SetPTPJointSpeedLimits: SetPTPJointSpeedLimits,
};
