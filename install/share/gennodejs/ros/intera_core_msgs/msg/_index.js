
"use strict";

let JointCommand = require('./JointCommand.js');
let HeadState = require('./HeadState.js');
let SEAJointState = require('./SEAJointState.js');
let JointLimits = require('./JointLimits.js');
let IONodeStatus = require('./IONodeStatus.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let HomingState = require('./HomingState.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let IONodeConfiguration = require('./IONodeConfiguration.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let InteractionControlState = require('./InteractionControlState.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let IODeviceStatus = require('./IODeviceStatus.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let IOStatus = require('./IOStatus.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let CameraControl = require('./CameraControl.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let EndpointState = require('./EndpointState.js');
let CameraSettings = require('./CameraSettings.js');
let HomingCommand = require('./HomingCommand.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let NavigatorStates = require('./NavigatorStates.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let NavigatorState = require('./NavigatorState.js');
let DigitalIOState = require('./DigitalIOState.js');
let IODataStatus = require('./IODataStatus.js');
let AnalogIOState = require('./AnalogIOState.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let EndpointStates = require('./EndpointStates.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');

module.exports = {
  JointCommand: JointCommand,
  HeadState: HeadState,
  SEAJointState: SEAJointState,
  JointLimits: JointLimits,
  IONodeStatus: IONodeStatus,
  IOComponentCommand: IOComponentCommand,
  HomingState: HomingState,
  AnalogIOStates: AnalogIOStates,
  DigitalOutputCommand: DigitalOutputCommand,
  IONodeConfiguration: IONodeConfiguration,
  EndpointNamesArray: EndpointNamesArray,
  InteractionControlState: InteractionControlState,
  RobotAssemblyState: RobotAssemblyState,
  IODeviceStatus: IODeviceStatus,
  CollisionDetectionState: CollisionDetectionState,
  IOStatus: IOStatus,
  CollisionAvoidanceState: CollisionAvoidanceState,
  CameraControl: CameraControl,
  DigitalIOStates: DigitalIOStates,
  HeadPanCommand: HeadPanCommand,
  IOComponentConfiguration: IOComponentConfiguration,
  EndpointState: EndpointState,
  CameraSettings: CameraSettings,
  HomingCommand: HomingCommand,
  URDFConfiguration: URDFConfiguration,
  NavigatorStates: NavigatorStates,
  AnalogOutputCommand: AnalogOutputCommand,
  InteractionControlCommand: InteractionControlCommand,
  IOComponentStatus: IOComponentStatus,
  NavigatorState: NavigatorState,
  DigitalIOState: DigitalIOState,
  IODataStatus: IODataStatus,
  AnalogIOState: AnalogIOState,
  IODeviceConfiguration: IODeviceConfiguration,
  EndpointStates: EndpointStates,
  CalibrationCommandResult: CalibrationCommandResult,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandAction: CalibrationCommandAction,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
};
