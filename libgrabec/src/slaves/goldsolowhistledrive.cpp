/**
 * @file goldsolowhistledrive.cpp
 * @author Edoardo Idà, Simone Comari
 * @date Jan 2022
 * @brief File containing class implementation declared in goldsolowhistledrive.h.
 */

#include "slaves/goldsolowhistledrive.h"

namespace grabec {

//------------------------------------------------------------------------------------//
// GoldSoloWhistleDriveData
//------------------------------------------------------------------------------------//

GoldSoloWhistleDriveData::GoldSoloWhistleDriveData(const int8_t _op_mode,
                                                   const int32_t _value /*= 0*/)
  : op_mode(_op_mode), value(_value)
{}

GoldSoloWhistleDriveData::GoldSoloWhistleDriveData(const int8_t _op_mode,
                                                   const GSWDriveInPdos& input_pdos,
                                                   const bool verbose /* = false */)
  : op_mode(_op_mode)
{
  // Set target value to current one
  switch (op_mode)
  {
    case GoldSoloWhistleOperationModes::CYCLIC_POSITION:
      value = input_pdos.pos_actual_value;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_POSITION @ %d\n", value);
      break;
    case GoldSoloWhistleOperationModes::CYCLIC_VELOCITY:
      value = input_pdos.vel_actual_value;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_VELOCITY @ %d\n", value);
      break;
    case GoldSoloWhistleOperationModes::CYCLIC_TORQUE:
      value = input_pdos.torque_actual_value;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_TORQUE @ %d\n", value);
      break;
    default:
      if (verbose)
        printf("\tTarget operational mode: NO_MODE\n");
      break;
  }
}

//------------------------------------------------------------------------------------//
// GoldSoloWhistleDrive
//------------------------------------------------------------------------------------//

// Must provide redundant definition of the static member as well as the declaration.
constexpr ec_pdo_entry_info_t GoldSoloWhistleDrive::kPdoEntries_[];
constexpr ec_pdo_info_t GoldSoloWhistleDrive::kPDOs_[];
constexpr ec_sync_info_t GoldSoloWhistleDrive::kSyncs_[];
constexpr char* GoldSoloWhistleDrive::kStatesStr_[];

GoldSoloWhistleDrive::GoldSoloWhistleDrive(const id_t id, const uint8_t slave_position
#if USE_QT
                                           ,
                                           QObject* parent /*= NULL*/
#endif
                                           )
  :
#if USE_QT
    QObject(parent),
#endif
    StateMachine(ST_MAX_STATES)
{
  alias_              = kAlias;
  position_           = slave_position;
  vendor_id_          = kVendorID;
  product_code_       = kProductCode;
  num_domain_entries_ = kDomainEntries;
  id_                 = id;

  domain_registers_[0]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kControlWordIdx,
                          kControlWordSubIdx,
                          &offset_out_.control_word,
                          nullptr};
  domain_registers_[1]  = {alias_,     position_,     vendor_id_,           product_code_,
                          kOpModeIdx, kOpModeSubIdx, &offset_out_.op_mode, nullptr};
  domain_registers_[2]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kTargetTorqueIdx,
                          kTargetTorqueSubIdx,
                          &offset_out_.target_torque,
                          nullptr};
  domain_registers_[3]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kTargetPosIdx,
                          kTargetPosSubIdx,
                          &offset_out_.target_position,
                          nullptr};
  domain_registers_[4]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kTargetVelIdx,
                          kTargetVelSubIdx,
                          &offset_out_.target_velocity,
                          nullptr};
  domain_registers_[5]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kStatusWordIdx,
                          kStatusWordSubIdx,
                          &offset_in_.status_word,
                          nullptr};
  domain_registers_[6]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kDisplayOpModeIdx,
                          kDisplayOpModeSubIdx,
                          &offset_in_.display_op_mode,
                          nullptr};
  domain_registers_[7]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPosActualValueIdx,
                          kPosActualValueSubIdx,
                          &offset_in_.position_actual_value,
                          nullptr};
  domain_registers_[8]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kVelActualValueIdx,
                          kVelActualValueSubIdx,
                          &offset_in_.velocity_actual_value,
                          nullptr};
  domain_registers_[9]  = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kTorqueActualValueIdx,
                          kTorqueActualValueSubIdx,
                          &offset_in_.torque_actual_value,
                          nullptr};
  domain_registers_[10] = {alias_,
                           position_,
                           vendor_id_,
                           product_code_,
                           kAnalogIdx,
                           kAnalogSubIdx,
                           &offset_in_.analog_input,
                           nullptr};
  domain_registers_[11] = {alias_,
                           position_,
                           vendor_id_,
                           product_code_,
                           kDigInIndex,
                           kDigInSubIndex,
                           &offset_in_.digital_inputs,
                           nullptr};
  domain_registers_[12] = {alias_,
                           position_,
                           vendor_id_,
                           product_code_,
                           kAuxPosActualValueIdx,
                           kAuxPosActualValueSubIdx,
                           &offset_in_.aux_pos_actual_value,
                           nullptr};

  domain_registers_ptr_  = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_        = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_        = const_cast<ec_sync_info_t*>(kSyncs_);

  drive_state_ = ST_START;
  prev_state_  = static_cast<States>(GetCurrentState());
}

GoldSoloWhistleDrive::States
GoldSoloWhistleDrive::getDriveState(const std::bitset<16>& status_word)
{
  if (status_word[StatusBit::SWITCH_ON_DISABLED] == SET)
  { // drive idle: OFF=true
    return ST_SWITCH_ON_DISABLED;
  }
  if (status_word[StatusBit::QUICK_STOP] == SET)
  {
    if (status_word[StatusBit::SWITCHED_ON] == UNSET)
    { // drive in operational progress: OFF=false, ON=true, SWITCH_ON=false
      return ST_READY_TO_SWITCH_ON;
    }
    if (status_word[StatusBit::OPERATION_ENABLED] == UNSET)
    { // drive in operational progress: OFF=false, ON=true, SWITCH_ON=true, ENABLED=false
      return ST_SWITCHED_ON;
    }
    // drive operational: OFF=false, ON=true, SWITCH_ON=true, ENABLED=true
    return ST_OPERATION_ENABLED;
  }
  if (status_word[StatusBit::FAULT] == UNSET)
  { // drive in quick stop: OFF=false, ON=false, FAULT=false
    return ST_QUICK_STOP_ACTIVE;
  }
  if (status_word[StatusBit::OPERATION_ENABLED] == SET)
  { // drive in fault reaction: OFF=false, ON=false, FAULT=true, ENABLED=true
    return ST_FAULT_REACTION_ACTIVE;
  }
  // drive in fault: OFF=false, ON=false, FAULT=true, ENABLED=false
  return ST_FAULT;
}

std::string GoldSoloWhistleDrive::getDriveStateStr(const std::bitset<16>& status_word)
{
  return kStatesStr_[getDriveState(status_word)];
}

//----- Overwritten virtual functions from base class --------------------------------//

RetVal GoldSoloWhistleDrive::sdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = nullptr;

  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(
          config_ptr, kOpModeIdx, kOpModeSubIdx,
          GoldSoloWhistleOperationModes::CYCLIC_POSITION)))
  {
    std::string msg;
#if USE_QT
    msg = QString("Drive %1 failed to create OpMode SDO request").arg(id_).toStdString();
#else
    std::ostringstream msg_stream;
    msg_stream << "Drive " << id_ << " failed to create OpMode SDO request";
    msg = msg_stream.str();
#endif
    ecPrintCb(msg, 'r');
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  if (ecrt_slave_config_sdo8(config_ptr, kOpModeIdx, kOpModeSubIdx,
                             GoldSoloWhistleOperationModes::CYCLIC_POSITION) != 0)
  {
    std::string msg;
#if USE_QT
    msg = QString("Drive %1 failed to add a config value to OpMode SDO")
            .arg(id_)
            .toStdString();
#else
    std::ostringstream msg_stream;
    msg_stream << "Drive " << id_ << " failed to add a config value to OpMode SDO";
    msg = msg_stream.str();
#endif
    ecPrintCb(msg, 'r');
    return ECONFIG;
  }

  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(
          config_ptr, kHomingMethodIdx, kHomingMethodSubIdx, kHomingOnPosMethod)))
  {
    std::string msg;
#if USE_QT
    msg = QString("Drive %1 failed to create HomingMethod SDO request")
            .arg(id_)
            .toStdString();
#else
    std::ostringstream msg_stream;
    msg_stream << "Drive " << id_ << " failed to create HomingMethod SDO request";
    msg = msg_stream.str();
#endif
    ecPrintCb(msg, 'r');
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  if (ecrt_slave_config_sdo8(config_ptr, kHomingMethodIdx, kHomingMethodSubIdx,
                             kHomingOnPosMethod) != 0)
  {
    std::string msg;
#if USE_QT
    msg = QString("Drive %1 failed to add a config value to HomingMethod SDO")
            .arg(id_)
            .toStdString();
#else
    std::ostringstream msg_stream;
    msg_stream << "Drive " << id_ << " failed to add a config value to HomingMethod SDO";
    msg = msg_stream.str();
#endif
    ecPrintCb(msg, 'r');
    return ECONFIG;
  }

  return OK;
}

void GoldSoloWhistleDrive::readInputs()
{
  input_pdos_.status_word     = EC_READ_U16(domain_data_ptr_ + offset_in_.status_word);
  input_pdos_.display_op_mode = EC_READ_S8(domain_data_ptr_ + offset_in_.display_op_mode);
  input_pdos_.pos_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.position_actual_value);
  input_pdos_.vel_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.velocity_actual_value);
  input_pdos_.torque_actual_value =
    EC_READ_S16(domain_data_ptr_ + offset_in_.torque_actual_value);
  input_pdos_.analog_input   = EC_READ_S16(domain_data_ptr_ + offset_in_.analog_input);
  input_pdos_.digital_inputs = EC_READ_U32(domain_data_ptr_ + offset_in_.digital_inputs);
  input_pdos_.aux_pos_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.aux_pos_actual_value);

  drive_state_ = getDriveState(input_pdos_.status_word);
  if (drive_state_ != GetCurrentState())
  {
    if (drive_state_ == ST_OPERATION_ENABLED)
    {
      // Get target default
      GoldSoloWhistleDriveData* data =
        new GoldSoloWhistleDriveData(input_pdos_.display_op_mode, input_pdos_);
      ExternalEvent(ST_OPERATION_ENABLED, data);
    }
    else
      ExternalEvent(drive_state_);
  }
}

void GoldSoloWhistleDrive::writeOutputs()
{
  EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word,
               output_pdos_.control_word.to_ulong());
  EC_WRITE_S8(domain_data_ptr_ + offset_out_.op_mode, output_pdos_.op_mode);
  if (drive_state_ == ST_OPERATION_ENABLED || drive_state_ == ST_SWITCHED_ON)
  {
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_position,
                 output_pdos_.target_position);
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_velocity,
                 output_pdos_.target_velocity);
    EC_WRITE_S16(domain_data_ptr_ + offset_out_.target_torque,
                 output_pdos_.target_torque);
  }
}

void GoldSoloWhistleDrive::safeExit()
{
  switch (prev_state_)
  {
    case ST_START:
      break;
    case ST_NOT_READY_TO_SWITCH_ON:
      break;
    case ST_SWITCH_ON_DISABLED:
      break;
    case ST_FAULT:
      faultReset(); // clear fault and disable drive completely
      break;
    default:
      disableVoltage(); // disable drive completely
      break;
  }
}

bool GoldSoloWhistleDrive::isReadyToShutDown() const
{
  return prev_state_ <= ST_SWITCH_ON_DISABLED;
}

void GoldSoloWhistleDrive::ecPrintCb(const std::string& msg,
                                     const char color /* = 'w' */) const
{
  EthercatSlave::ecPrintCb(msg, color);
#if USE_QT
  if (color == 'r')
    emit printMessage(msg.c_str());
  else
    emit logMessage(msg.c_str());
#endif
}

//----- External events taken by this state machine ----------------------------------//

void GoldSoloWhistleDrive::shutdown()
{
  printCommand("Shutdown");
  output_pdos_.control_word.reset(ControlBit::SWITCH_ON);
  output_pdos_.control_word.set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.reset(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::switchOn()
{
  printCommand("SwitchOn");
  // Trigger device control command
  output_pdos_.control_word.set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.reset(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.reset(ControlBit::FAULT);
  // Setup default operational mode before enabling the drive
  output_pdos_.op_mode         = GoldSoloWhistleOperationModes::CYCLIC_POSITION;
  output_pdos_.target_position = input_pdos_.pos_actual_value;
}

void GoldSoloWhistleDrive::enableOperation()
{
  printCommand("EnableOperation");
  // Trigger device control command
  output_pdos_.control_word.set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.set(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.reset(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::disableOperation()
{
  printCommand("DisableOperation");
  // Trigger device control command
  output_pdos_.control_word.set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.reset(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.reset(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::disableVoltage()
{
  printCommand("DisableVoltage");
  // Trigger device control command
  output_pdos_.control_word.reset(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.reset(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::quickStop()
{
  printCommand("QuickStop");
  // Trigger device control command
  output_pdos_.control_word.set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.reset(ControlBit::QUICK_STOP);
  output_pdos_.control_word.reset(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::faultReset()
{
  printCommand("FaultReset");
  // Trigger device control command
  output_pdos_.control_word.set(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::changePosition(const int32_t target_position,
                                          const bool verbose /*=false*/)
{
  if (input_pdos_.display_op_mode == GoldSoloWhistleOperationModes::CYCLIC_POSITION &&
      prev_pos_target_ == target_position)
    return;
  prev_pos_target_ = target_position;

  if (verbose)
  {
    printCommand("ChangePosition");
    printTarget(target_position);
  }

  GoldSoloWhistleDriveData data(GoldSoloWhistleOperationModes::CYCLIC_POSITION,
                                target_position);
  setChange(data);
}

void GoldSoloWhistleDrive::changeDeltaPosition(const int32_t delta_position)
{
  changePosition(input_pdos_.pos_actual_value + delta_position);
}

void GoldSoloWhistleDrive::changeVelocity(const int32_t target_velocity,
                                          const bool verbose /*=false*/)
{
  if (input_pdos_.display_op_mode == GoldSoloWhistleOperationModes::CYCLIC_VELOCITY &&
      prev_vel_target_ == target_velocity)
    return;
  prev_vel_target_ = target_velocity;

  if (verbose)
  {
    printCommand("ChangeVelocity");
    printTarget(target_velocity);
  }

  GoldSoloWhistleDriveData data(GoldSoloWhistleOperationModes::CYCLIC_VELOCITY,
                                target_velocity);
  setChange(data);
}

void GoldSoloWhistleDrive::changeDeltaVelocity(const int32_t delta_velocity)
{
  changeVelocity(input_pdos_.vel_actual_value + delta_velocity);
}

void GoldSoloWhistleDrive::changeTorque(const int16_t target_torque,
                                        const bool verbose /*=false*/)
{
  if (input_pdos_.display_op_mode == GoldSoloWhistleOperationModes::CYCLIC_TORQUE &&
      prev_torque_target_ == target_torque)
    return;
  prev_torque_target_ = target_torque;

  if (verbose)
  {
    printCommand("ChangeTorque");
    printTarget(target_torque);
  }

  GoldSoloWhistleDriveData data(GoldSoloWhistleOperationModes::CYCLIC_TORQUE,
                                target_torque);
  setChange(data);
}

void GoldSoloWhistleDrive::changeDeltaTorque(const int16_t delta_torque)
{
  changeTorque(input_pdos_.torque_actual_value + delta_torque);
}

void GoldSoloWhistleDrive::changeOpMode(const int8_t target_op_mode,
                                        const bool verbose /*=false*/)
{
  printCommand("ChangeOpMode");
  // Set target value to current one
  GoldSoloWhistleDriveData data(target_op_mode, input_pdos_, true);
  if (verbose)
    printTarget(data);
  setChange(data);
}

void GoldSoloWhistleDrive::setTargetDefaults(const bool verbose /*=false*/)
{
  printCommand("SetTargetDefaults");
  // Set target operational mode and value to current ones
  GoldSoloWhistleDriveData data(input_pdos_.display_op_mode, input_pdos_, true);
  if (verbose)
    printTarget(data);
  // Apply
  setChange(data);
}

void GoldSoloWhistleDrive::setChange(const GoldSoloWhistleDriveData& data)
{
  // clang-format off
  BEGIN_TRANSITION_MAP                               // - Current State -
    TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)              // ST_START
    TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)              // ST_NOT_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_SWITCH_ON_DISABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_SWITCHED_ON
    TRANSITION_MAP_ENTRY(ST_OPERATION_ENABLED)       // ST_OPERATION_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_QUICK_STOP_ACTIVE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_FAULT_REACTION_ACTIVE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_FAULT
  END_TRANSITION_MAP(&data)
  // clang-format on
}

//----- Protected functions ----------------------------------------------------------//

void GoldSoloWhistleDrive::initFun() {
  // clang-format off
  BEGIN_TRANSITION_MAP			          // - Current State -
    TRANSITION_MAP_ENTRY (ST_START)               // ST_START
    TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_NOT_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_SWITCH_ON_DISABLED
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_SWITCHED_ON
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_OPERATION_ENABLED
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_QUICK_STOP_ACTIVE
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_FAULT_REACTION_ACTIVE
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

//----- States actions ---------------------------------------------------------------//

STATE_DEFINE(GoldSoloWhistleDrive, Start, NoEventData)
{
  prev_state_ = ST_START;
  std::string msg;
#if USE_QT
  msg = QString("Drive %1 initial state: %2")
          .arg(id_)
          .arg(kStatesStr_[ST_START])
          .toStdString();
#else
  std::ostringstream msg_stream;
  msg_stream << "Drive " << id_ << " initial state: " << kStatesStr_[ST_START];
  msg = msg_stream.str();
#endif
  ecPrintCb(msg);
  // This happens automatically on drive's start up. We simply imitate the behavior here.
  InternalEvent(ST_NOT_READY_TO_SWITCH_ON);
}

STATE_DEFINE(GoldSoloWhistleDrive, NotReadyToSwitchOn, NoEventData)
{
  printStateTransition(prev_state_, ST_NOT_READY_TO_SWITCH_ON);
  prev_state_ = ST_NOT_READY_TO_SWITCH_ON;
  // This happens automatically on drive's start up. We simply imitate the behavior here.
  InternalEvent(ST_SWITCH_ON_DISABLED);
}

STATE_DEFINE(GoldSoloWhistleDrive, SwitchOnDisabled, NoEventData)
{
  printStateTransition(prev_state_, ST_SWITCH_ON_DISABLED);
  prev_state_ = ST_SWITCH_ON_DISABLED;
}

STATE_DEFINE(GoldSoloWhistleDrive, ReadyToSwitchOn, NoEventData)
{
  printStateTransition(prev_state_, ST_READY_TO_SWITCH_ON);
  prev_state_ = ST_READY_TO_SWITCH_ON;
}

STATE_DEFINE(GoldSoloWhistleDrive, SwitchedOn, NoEventData)
{
  printStateTransition(prev_state_, ST_SWITCHED_ON);
  prev_state_ = ST_SWITCHED_ON;
}

STATE_DEFINE(GoldSoloWhistleDrive, OperationEnabled, GoldSoloWhistleDriveData)
{
  printStateTransition(prev_state_, ST_OPERATION_ENABLED);

  // Setup operational mode and target value
  output_pdos_.op_mode = data->op_mode;
  switch (data->op_mode)
  {
    case GoldSoloWhistleOperationModes::CYCLIC_POSITION:
      output_pdos_.target_position = data->value;
      break;
    case GoldSoloWhistleOperationModes::CYCLIC_VELOCITY:
      output_pdos_.target_velocity = data->value;
      break;
    case GoldSoloWhistleOperationModes::CYCLIC_TORQUE:
      output_pdos_.target_torque = static_cast<int16_t>(data->value);
      break;
    default:
      break;
  }

  prev_state_ = ST_OPERATION_ENABLED;
}

STATE_DEFINE(GoldSoloWhistleDrive, QuickStopActive, NoEventData)
{
  printStateTransition(prev_state_, ST_QUICK_STOP_ACTIVE);
  prev_state_ = ST_QUICK_STOP_ACTIVE;
}

STATE_DEFINE(GoldSoloWhistleDrive, FaultReactionActive, NoEventData)
{
  printStateTransition(prev_state_, ST_FAULT_REACTION_ACTIVE);
  prev_state_ = ST_FAULT_REACTION_ACTIVE;
}

STATE_DEFINE(GoldSoloWhistleDrive, Fault, NoEventData)
{
  printStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;
#if USE_QT
  emit driveFaulted();
#endif
}

//----- Miscellaneous ----------------------------------------------------------------//

inline void GoldSoloWhistleDrive::printCommand(const char* cmd) const
{
  std::string msg;
#if USE_QT
  msg = QString("Drive %1 received command: %2").arg(id_).arg(cmd).toStdString();
#else
  std::ostringstream msg_stream;
  msg_stream << "Drive " << id_ << " received command: " << cmd;
  msg = msg_stream.str();
#endif
  ecPrintCb(msg);
}

inline void GoldSoloWhistleDrive::printTarget(const int target) const
{
  std::string msg;
#if USE_QT
  msg = QString("Drive %1 new target: %2").arg(id_).arg(target).toStdString();
#else
  std::ostringstream msg_stream;
  msg_stream << "Drive " << id_ << " new target: " << target;
  msg = msg_stream.str();
#endif
  ecPrintCb(msg);
}

inline void GoldSoloWhistleDrive::printTarget(const GoldSoloWhistleDriveData& data) const
{
  std::string msg;
#if USE_QT
  msg = QString("Drive %1 op mode %2 with target: %3")
          .arg(id_)
          .arg(data.op_mode)
          .arg(data.value)
          .toStdString();
#else
  std::ostringstream msg_stream;
  msg_stream << "Drive " << id_ << " op mode " << data.op_mode
             << " with target: " << target;
  msg = msg_stream.str();
#endif
  ecPrintCb(msg);
}

void GoldSoloWhistleDrive::printStateTransition(const States current_state,
                                                const States new_state) const
{
  if (current_state == new_state)
    return;
  std::string msg;
#if USE_QT
  msg = QString("Drive %1 state transition: %2 --> %3")
          .arg(id_)
          .arg(kStatesStr_[current_state], kStatesStr_[new_state])
          .toStdString();
#else
  std::ostringstream msg_stream;
  msg_stream << "Drive " << id_ << " state transition: " << kStatesStr_[current_state]
             << " --> " << kStatesStr_[new_state];
  msg = msg_stream.str();
#endif
  ecPrintCb(msg);
}

} // end namespace grabec
