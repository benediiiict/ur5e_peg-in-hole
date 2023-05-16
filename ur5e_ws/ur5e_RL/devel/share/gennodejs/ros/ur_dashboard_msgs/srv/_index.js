
"use strict";

let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let AddToLog = require('./AddToLog.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetProgramState = require('./GetProgramState.js')
let RawRequest = require('./RawRequest.js')
let GetRobotMode = require('./GetRobotMode.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let Popup = require('./Popup.js')
let Load = require('./Load.js')

module.exports = {
  GetLoadedProgram: GetLoadedProgram,
  GetSafetyMode: GetSafetyMode,
  AddToLog: AddToLog,
  IsInRemoteControl: IsInRemoteControl,
  IsProgramSaved: IsProgramSaved,
  GetProgramState: GetProgramState,
  RawRequest: RawRequest,
  GetRobotMode: GetRobotMode,
  IsProgramRunning: IsProgramRunning,
  Popup: Popup,
  Load: Load,
};
