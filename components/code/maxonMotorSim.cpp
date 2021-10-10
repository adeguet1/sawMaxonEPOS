#include "maxonMotor.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <climits>

#include "Definitions.h"
#include <json/json.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstParameterTypes/prmRobotState.h>

#ifndef WIN32
typedef int BOOL;
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(maxonMotor, mtsTaskPeriodic)

maxonMotor::maxonMotor(const unsigned short id, HANDLE c, const std::string &taskName) :
  mtsTaskPeriodic(taskName, 100.0*cmn_ms),
  controller(c),
  errorCode(0),
  nodeId(id),
  accel(10000),
  decel(10000),
  zeroPos(0),  
  targetVelocity(0),
  pitch(1.0),
  gearRatio(1),
  quadrature(4),
  countsPerTurn(512),
  negLimit(-100),
  posLimit(100),
  stroke(20.0),
  enabled(false),
  fault(false),
  targetReached(false),
  hasAI(false),
  aiCal(5.0/4096.0), // based on maxon spec sheet
  forceGain(1.0),
  operationalMode(OMD_PROFILE_POSITION_MODE),
  position_raw(0),
  velocity_raw(0),
  velocityAverage_raw(0),
  position(0),
  velocity(0),
  velocityAverage(0),
  current(0),
  currentAverage(0),
  analogInput_raw(0.0),
  force(0.0)
{
  createInterface();

  clearFault();
  disable();
}

maxonMotor::maxonMotor(const unsigned short id, const std::string &filename, HANDLE c, const std::string &taskName) :
  mtsTaskPeriodic(taskName, 10.0*cmn_ms),
  controller(c),
  errorCode(0),
  nodeId(id),
  accel(10000),
  decel(10000),
  zeroPos(0),  
  targetVelocity(0),
  pitch(1.0),
  gearRatio(1),
  quadrature(4),
  countsPerTurn(512),
  negLimit(-100),
  posLimit(100),
  stroke(20.0),
  enabled(false),
  fault(false),
  targetReached(false),
  hasAI(false),
  aiCal(5.0/4096.0), // based on maxon spec sheet
  forceGain(1.0),
  operationalMode(OMD_PROFILE_POSITION_MODE),
  position_raw(0),
  velocity_raw(0),
  velocityAverage_raw(0),
  position(0),
  velocity(0),
  velocityAverage(0),
  current(0),
  currentAverage(0),
  analogInput_raw(0.0),
  force(0.0)
{
  createInterface();

  clearFault();
  disable();

  loadMotorConfiguration(filename);
}

maxonMotor::~maxonMotor()
{

}

void maxonMotor::Startup()
{

}

void maxonMotor::Cleanup()
{
  disable();
}

void maxonMotor::Configure(const std::string &filename)
{
  loadMotorConfiguration(filename);
}

void maxonMotor::Run()
{
  ProcessQueuedCommands();

  CMN_LOG_CLASS_RUN_DEBUG << this->Name << " is running!" << std::endl;
  // this is a hack to read the system time
  std::string t;
  osaGetTimeString(t, osaGetTime(), ':');
  time = t;

  // if velocity is greater than zero, integrate position
  if (abs(velocity_raw) > 0)
  {
    position_raw = position_raw + this->Period*velocity_raw/60;
  }

  // update the statu
  if(enabled)
    updateStatus();
}

void maxonMotor::createInterface()
{
  // Construct the state table
  StateTable.AddData(operationMode, "operationMode");
  StateTable.AddData(enabled, "enabled");
  StateTable.AddData(position_raw, "position_raw");
  StateTable.AddData(velocity_raw, "velocity_raw");
  StateTable.AddData(velocityAverage_raw, "velocityAverage_raw");
  StateTable.AddData(position, "position");
  StateTable.AddData(velocity, "velocity");
  StateTable.AddData(velocityAverage, "velocityAverage");
  StateTable.AddData(current, "current");
  StateTable.AddData(currentAverage, "currentAverage");
  StateTable.AddData(fault, "fault");
  StateTable.AddData(time, "time");
  StateTable.AddData(targetReached, "targetReached");
  StateTable.AddData(force, "force");

  mtsInterfaceProvided *provided = AddInterfaceProvided("state");
  if(provided) {
    provided->AddCommandReadState(StateTable, operationMode, "operationMode");
    provided->AddCommandReadState(StateTable, enabled, "enabled");
    provided->AddCommandReadState(StateTable, position_raw, "position_raw");
    provided->AddCommandReadState(StateTable, velocity_raw, "velocity_raw");
    provided->AddCommandReadState(StateTable, velocityAverage_raw, "velocityAverage_raw");
    provided->AddCommandReadState(StateTable, position, "position");
    provided->AddCommandReadState(StateTable, velocity, "velocity");
    provided->AddCommandReadState(StateTable, velocityAverage, "velocityAverage");
    provided->AddCommandReadState(StateTable, current, "current");
    provided->AddCommandReadState(StateTable, currentAverage, "currentAverage");
    provided->AddCommandReadState(StateTable, fault, "fault");
    provided->AddCommandReadState(StateTable, time, "time");
    provided->AddCommandReadState(StateTable, force, "force");
    //provided->AddCommandReadState(StateTable, targetReached, "targetReached");
  } else
    std::cerr << "[maxonMotor] Could not add state interface" << std::endl;

  provided = AddInterfaceProvided("control");
  if(provided) {
    provided->AddCommandVoidReturn(&maxonMotor::enable, this, "enable", mtsBool());
    provided->AddCommandVoid(&maxonMotor::disable, this, "disable");
    provided->AddCommandVoid(&maxonMotor::clearFault, this, "clearFault");
    provided->AddCommandVoid(&maxonMotor::stop, this, "stop");
    provided->AddCommandWrite(&maxonMotor::setVelocity, this, "setVelocity", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setVelocity_raw, this, "setVelocity_raw", mtsLong());
    provided->AddCommandVoidReturn(&maxonMotor::jogPlus, this, "jogPlus", mtsBool());
    provided->AddCommandVoidReturn(&maxonMotor::jogMinus, this, "jogMinus", mtsBool());

    provided->AddCommandRead(&maxonMotor::getPositiveLimit, this, "positiveLimit", mtsDouble());
    provided->AddCommandRead(&maxonMotor::getNegativeLimit, this, "negativeLimit", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setPositiveLimit, this, "setPositiveLimit", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::setNegativeLimit, this, "setNegativeLimit", mtsDouble());
    provided->AddCommandVoid(&maxonMotor::setNegativeLimitCurrentPosition, this, "setNegativeLimitCurrentPosition");
    provided->AddCommandVoid(&maxonMotor::setPositiveLimitCurrentPosition, this, "setPositiveLimitCurrentPosition");

    provided->AddCommandVoidReturn(&maxonMotor::getTargetVelocity, this, "getTargetVelocity", mtsDouble());
    provided->AddCommandVoidReturn(&maxonMotor::getTargetVelocity_raw, this, "getTargetVelocity_raw", mtsLong());

    provided->AddCommandWrite(&maxonMotor::moveToAbsolutePosition, this, "moveToAbsolutePosition", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::moveToRelativePosition, this, "moveToRelativePosition", mtsDouble());
    provided->AddCommandWrite(&maxonMotor::moveToAbsolutePosition_raw, this, "moveToAbsolutePosition_raw", mtsLong());
    provided->AddCommandWrite(&maxonMotor::moveToRelativePosition_raw, this, "moveToRelativePosition_raw", mtsLong());

    provided->AddCommandVoid(&maxonMotor::setPositionMode, this, "setPositionMode");
    provided->AddCommandVoid(&maxonMotor::setVelocityMode, this, "setVelocityMode");
    provided->AddCommandVoidReturn(&maxonMotor::getErrorString, this, "getErrorString", mtsStdString());

    provided->AddCommandRead(&maxonMotor::getNodeId, this, "getNodeId", mtsUShort());

    provided->AddCommandVoid(&maxonMotor::zero, this, "zero");
    provided->AddCommandVoid(&maxonMotor::moveToZero, this, "moveToZero");
    provided->AddCommandWrite(&maxonMotor::setDigitalOutput, this, "setDigitalOutput");
    provided->AddCommandVoid(&maxonMotor::rebias, this, "rebias");
    provided->AddCommandWrite(&maxonMotor::setHomeForce, this, "setHomeForce", mtsDouble());
    provided->AddCommandVoid(&maxonMotor::startForceHoming, this, "startForceHoming");
  } else
    std::cerr << "[maxonMotor] Could not add control interface" << std::endl;
}

void maxonMotor::getNodeId(mtsUShort &id) const 
{
  id = nodeId;
}

void maxonMotor::clearFault()
{
}

void maxonMotor::zero()
{
  // should this grab the information from the state table? 
  double currPos = position;
  zeroPos += position_raw;
  posLimit += currPos;
  negLimit += currPos;
}

void maxonMotor::moveToZero()
{
  moveToPosition_raw(0, true);
}

bool maxonMotor::loadMotorConfiguration(const std::string &filename)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream config(filename.data());
  bool ok = reader.parse(config, root, false);
  if (!ok) {
    return false;
  }

  // stroke
  stroke = root["motor"]["motorParams"].get("stroke", 20).asDouble();

  // motor parameters
  WORD nominalCurrent = static_cast<WORD>(root["motor"]["motorParams"].get("nominalCurrent", 569).asInt());
  WORD maxCurrent = static_cast<WORD>(root["motor"]["motorParams"].get("maxCurrent", 1138).asInt());
  WORD thermalTimeConstant = static_cast<WORD>(root["motor"]["motorParams"].get("thermalTimeConstant", 106).asInt());
  pitch = root["motor"]["motorParams"].get("pitch", 1).asDouble();
  gearRatio = root["motor"]["motorParams"].get("gearRatio", 16).asDouble();

  // sensor parameters
  DWORD encoderResolution = static_cast<DWORD>(root["motor"]["sensorParams"].get("encoderResolution", 512).asInt());
  BOOL invertedPolarity = static_cast<BOOL>(root["motor"]["sensorParams"].get("invertedPolarity", 0).asInt());
  quadrature = root["motor"]["sensorParams"].get("quadrature", 4).asInt();
  WORD sensorType = static_cast<WORD>(root["motor"]["sensorParams"].get("type", 2).asInt());
  countsPerTurn = (int)encoderResolution;
  
  // safety parameters
  DWORD maxFollowError = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxFollowError", 2000).asLargestUInt());
  DWORD maxProfileVelocity = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxProfileVelocity", 14000).asLargestUInt());
  DWORD maxAccel = static_cast<DWORD>(root["motor"]["safetyParams"].get("maxAccel", 0xffffffff).asLargestUInt());

  // position regulator
  WORD p = static_cast<WORD>(root["motor"]["positionRegulator"].get("p", 65).asInt());
  WORD i = static_cast<WORD>(root["motor"]["positionRegulator"].get("i", 345).asInt());
  WORD d = static_cast<WORD>(root["motor"]["positionRegulator"].get("d", 65).asInt());
  WORD vFF= static_cast<WORD>(root["motor"]["positionRegulator"].get("vFF", 0).asInt());
  WORD aFF = static_cast<WORD>(root["motor"]["positionRegulator"].get("aFF", 14).asInt());

  // velocity regulator
  p = static_cast<WORD>(root["motor"]["velocityRegulator"].get("p", 173).asInt());
  i = static_cast<WORD>(root["motor"]["velocityRegulator"].get("i", 43).asInt());
  vFF= static_cast<WORD>(root["motor"]["velocityRegulator"].get("vFF", 0).asInt());
  aFF = static_cast<WORD>(root["motor"]["velocityRegulator"].get("aFF", 14).asInt());

  // current regulator
  p = static_cast<WORD>(root["motor"]["currentRegulator"].get("p", 732).asInt());
  i = static_cast<WORD>(root["motor"]["currentRegulator"].get("i", 451).asInt());

  return true;
}


void maxonMotor::updateStatus()
{
  char opMode;
  WORD ai1;

  int fault = 0;
  this->fault.Data = (fault != 0);

  if(operationalMode) {
    switch(opMode) {
      case OMD_PROFILE_POSITION_MODE:
        operationMode.Data = "Profile position";
        break;
      case OMD_PROFILE_VELOCITY_MODE:
        operationMode.Data = "Profile velocity";
        break;
      case OMD_HOMING_MODE:
        operationMode.Data = "Homing";
        break;
      case OMD_INTERPOLATED_POSITION_MODE:
        operationMode.Data = "Interpolated position";
        break;
      case OMD_POSITION_MODE:
        operationMode.Data = "Position";
        break;
      case OMD_VELOCITY_MODE:
        operationMode.Data = "Velocity";
        break;
      case OMD_CURRENT_MODE:
        operationMode.Data = "Current";
        break;
      case OMD_MASTER_ENCODER_MODE:
        operationMode.Data = "Master encoder";
        break;
      case OMD_STEP_DIRECTION_MODE:
        operationMode.Data = "Step direction";
        break;
      default:
        operationMode.Data = "Unknown";
    }
  }

  int enabled_;
  enabled_ = 1;
  enabled.Data = (enabled_ != 0) && !this->fault.Data;

  // convert measures to appropriate units
  position = encoderCountsToUnits(position_raw);
  velocity = rpmToUnits(velocity_raw);
  velocityAverage = rpmToUnits(velocityAverage_raw);
}

bool maxonMotor::getStatus(struct status &stat)
{
  stat.position = position;
  stat.position_raw = position_raw;
  stat.velocity = velocity;
  stat.velocity_raw = velocity_raw;
  stat.velocityAverage = velocityAverage;
  stat.velocityAverage_raw = velocityAverage_raw;
  stat.operationMode = operationMode;
  stat.current = current;
  stat.currentAverage = current;
  stat.force = force;
  stat.ai_raw = analogInput_raw;

  return enabled;
}

void maxonMotor::rebias()
{
  CMN_LOG_CLASS_RUN_WARNING << "function rebias not implemented for motor simulator class" << std::endl;
}

void maxonMotor::setHomeForce(const mtsDouble &homeForce)
{
  CMN_LOG_CLASS_RUN_WARNING << "function setHomeForce not implemented for motor simulator class" << std::endl;
}

void maxonMotor::startForceHoming()
{
  CMN_LOG_CLASS_RUN_WARNING << "function startForceHoming not implemented for motor simulator class" << std::endl;
}

void maxonMotor::checkForceHoming()
{
  CMN_LOG_CLASS_RUN_WARNING << "function checkForceHoming not implemented for motor simulator class" << std::endl;
}

void maxonMotor::enable(mtsBool &result)
{
  result = false;
  int fault = 0;

  // set the accel/decel profile limits
  setVelocityProfile();
  setPositionProfile();
  setPositionMode();

  enabled = true;
  result = true;
}

void maxonMotor::setDigitalOutput(const mtsInt &val)
{
}

void maxonMotor::disable()
{
  if(!enabled)
    return;

  stop();
  enabled = false;
}

void maxonMotor::stop()
{
  velocity_raw = 0;
}

void maxonMotor::getNegativeLimit(mtsDouble &limit) const
{
  limit = negLimit;
}

void maxonMotor::getPositiveLimit(mtsDouble &limit) const
{
  limit = posLimit;
}

void maxonMotor::setNegativeLimit(const mtsDouble &limit)
{
  negLimit = limit;
  posLimit = negLimit + stroke;
}

void maxonMotor::setNegativeLimitCurrentPosition()
{
  this->setNegativeLimit(position);
}

void maxonMotor::setPositiveLimit(const mtsDouble &limit)
{
  posLimit = limit;
  negLimit = posLimit - stroke;
}

void maxonMotor::setPositiveLimitCurrentPosition()
{
  this->setPositiveLimit(position);
}



void maxonMotor::setVelocity(const mtsDouble &vel)
{
  targetVelocity = std::abs(unitsToRPM(vel.Data));
  setPositionProfile();
}

void maxonMotor::setVelocity_raw(const mtsLong &vel)
{
  targetVelocity = std::abs(vel.Data);
  setPositionProfile();
}


void maxonMotor::getTargetVelocity_raw(mtsLong &vel)
{
  vel = targetVelocity;
}

void maxonMotor::getTargetVelocity(mtsDouble &vel)
{
  vel = rpmToUnits(targetVelocity);
}

void maxonMotor::setMaxVelocity(const double vel)
{
  unsigned int maxVel = static_cast<unsigned int>(unitsToRPM(vel));
//  VCS_SetMaxProfileVelocity(controller, nodeId, maxVel, &errorCode);
}

void maxonMotor::setMaxVelocity_raw(const unsigned int maxVel)
{
//  VCS_SetMaxProfileVelocity(controller, nodeId, maxVel, &errorCode);
}

double maxonMotor::getMaxVelocity()
{
  DWORD maxVel;
//  VCS_GetMaxProfileVelocity(controller, nodeId, &maxVel, &errorCode);
  return rpmToUnits(maxVel);
}

void maxonMotor::setMaxAcceleration(const double acc)
{
  // convert to rpm/s from mm/s2
  DWORD maxAcc = static_cast<DWORD>(unitsToRPM(acc));
//  VCS_SetMaxAcceleration(controller, nodeId, maxAcc, &errorCode);
}

void maxonMotor::setMaxAcceleration_raw(const unsigned int acc)
{
//  VCS_SetMaxAcceleration(controller, nodeId, acc, &errorCode);
}

double maxonMotor::getMaxAcceleration()
{
  DWORD maxAcc;
//  VCS_GetMaxAcceleration(controller, nodeId, &maxAcc, &errorCode);
  // convert to mm/min/sec and then to mm/sec2
  return rpmToUnits(maxAcc);
}

void maxonMotor::setPositionMode()
{
  stop();
  operationalMode = OMD_PROFILE_POSITION_MODE;
}

void maxonMotor::setVelocityMode()
{
  stop();
  operationalMode = OMD_PROFILE_VELOCITY_MODE;
}

bool maxonMotor::getVelocityProfile()
{
//  return VCS_GetVelocityProfile(controller, nodeId, &accel, &decel, &errorCode) != 0;
}

bool maxonMotor::setVelocityProfile()
{
//  return VCS_SetVelocityProfile(controller, nodeId, accel, decel, &errorCode) != 0;
}

bool maxonMotor::getPositionProfile()
{
  DWORD vel;
//  return VCS_GetPositionProfile(controller, nodeId, &vel, &accel, &decel, &errorCode) != 0;
}

bool maxonMotor::setPositionProfile()
{
  DWORD vel = std::abs(targetVelocity);
  // here set velocity and velocity raw
  return true;
}

void maxonMotor::moveToAbsolutePosition(const mtsDouble & pos)
{
  moveToPosition(pos.GetData(), true);
}

void maxonMotor::moveToAbsolutePosition_raw(const mtsLong & pos)
{
  moveToPosition_raw(pos.GetData(), true);
}

void maxonMotor::moveToRelativePosition(const mtsDouble & pos)
{
  moveToPosition(pos.GetData(), false);
}

void maxonMotor::moveToRelativePosition_raw(const mtsLong & pos)
{
  moveToPosition_raw(pos.GetData(), false);
}

bool maxonMotor::moveToPosition(const double pos, const bool abs, const bool immediate)
{
  long posl = unitsToEncoderCounts(pos);
  return moveToPosition_raw(posl, abs, immediate);
}

bool maxonMotor::moveToPosition_raw(const long pos, const bool abs, const bool immediate)
{
  long posl = pos;
  if(abs)
    posl += zeroPos;
  // here set position, position_raw
}

void maxonMotor::jogPlus(mtsBool &result)
{
  velocity_raw = targetVelocity;
}

void maxonMotor::jogMinus(mtsBool &result)
{
  velocity_raw = targetVelocity;
}

void maxonMotor::getErrorString(mtsStdString &errorStr)
{
  char error[100];
  errorStr.Data = "No error";
}

inline double maxonMotor::encoderCountsToUnits(const long &encCounts) const
{
  return (double)encCounts * pitch / (gearRatio * countsPerTurn * quadrature);
}

inline long maxonMotor::unitsToEncoderCounts(const double &units) const
{
  return static_cast<long>(units * (countsPerTurn * quadrature * gearRatio) / (pitch));
}

inline long maxonMotor::unitsToRPM(const double &units) const
{
  return static_cast<long>(units * gearRatio / (pitch) * 60.0);
}

inline double maxonMotor::rpmToUnits(const long &rpm) const
{
  return (double)rpm * (pitch) / (60.0 * gearRatio);
}

inline double maxonMotor::aiToVolts(const WORD &ai) const
{
  return (double)ai * aiCal;
}

inline double maxonMotor::voltsToForce (const double &volts) const
{
  return volts * forceGain;
}

