
"use strict";

let AddCrazyflie = require('./AddCrazyflie.js')
let Takeoff = require('./Takeoff.js')
let Stop = require('./Stop.js')
let sendPacket = require('./sendPacket.js')
let GoTo = require('./GoTo.js')
let NotifySetpointsStop = require('./NotifySetpointsStop.js')
let UploadTrajectory = require('./UploadTrajectory.js')
let Land = require('./Land.js')
let UpdateParams = require('./UpdateParams.js')
let StartTrajectory = require('./StartTrajectory.js')
let RemoveCrazyflie = require('./RemoveCrazyflie.js')
let SetGroupMask = require('./SetGroupMask.js')

module.exports = {
  AddCrazyflie: AddCrazyflie,
  Takeoff: Takeoff,
  Stop: Stop,
  sendPacket: sendPacket,
  GoTo: GoTo,
  NotifySetpointsStop: NotifySetpointsStop,
  UploadTrajectory: UploadTrajectory,
  Land: Land,
  UpdateParams: UpdateParams,
  StartTrajectory: StartTrajectory,
  RemoveCrazyflie: RemoveCrazyflie,
  SetGroupMask: SetGroupMask,
};
