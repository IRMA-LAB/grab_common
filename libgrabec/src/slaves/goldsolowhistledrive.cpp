/**
 * @file goldsolowhistledrive.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 22 Gen 2019
 * @brief File containing class implementation declared in goldsolowhistledrive.h.
 */

#include "slaves/goldsolowhistledrive.h"

namespace grabec
{

////////////////////////////////////////////////////////////////////////////
//// GoldSoloWhistleDriveData
////////////////////////////////////////////////////////////////////////////

GoldSoloWhistleDriveData::GoldSoloWhistleDriveData(const int8_t _op_mode,
                                                   const int32_t _value /*= 0*/)
  : op_mode(_op_mode), value(_value)
{
}

GoldSoloWhistleDriveData::GoldSoloWhistleDriveData(const int8_t _op_mode,
                                                   const GSWDriveInPdos& input_pdos,
                                                   const bool verbose /* = false */)
  : op_mode(_op_mode)
{
  // Set target value to current one
  switch (op_mode)
  {
  case CYCLIC_POSITION:
    value = input_pdos.pos_actual_value;
    if (verbose)
      printf("\tTarget operational mode: CYCLIC_POSITION @ %d\n", value);
    break;
  case CYCLIC_VELOCITY:
    value = input_pdos.vel_actual_value;
    if (verbose)
      printf("\tTarget operational mode: CYCLIC_VELOCITY @ %d\n", value);
    break;
  case CYCLIC_TORQUE:
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

////////////////////////////////////////////////////////////////////////////
//// GoldSoloWhistleDrive
////////////////////////////////////////////////////////////////////////////

// Must provide redundant definition of the static member as well as the declaration.
constexpr ec_pdo_entry_info_t GoldSoloWhistleDrive::kPdoEntries_[];
constexpr ec_pdo_info_t GoldSoloWhistleDrive::kPDOs_[];
constexpr ec_sync_info_t GoldSoloWhistleDrive::kSyncs_[];
constexpr char* GoldSoloWhistleDrive::kStatesStr_[];

GoldSoloWhistleDrive::GoldSoloWhistleDrive(const id_t id, const uint8_t slave_position,
                                           QObject* parent /*= NULL*/)
  : QObject(parent), StateMachine(ST_MAX_STATES), id_(id)
{
  alias_ = kAlias;
  position_ = slave_position;
  vendor_id_ = kVendorID;
  product_code_ = kProductCode;
  num_domain_entries_ = kDomainEntries;

  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_, kControlWordIdx,
                          kControlWordSubIdx, &offset_out_.control_word, NULL};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_, kOpModeIdx,
                          kOpModeSubIdx, &offset_out_.op_mode, NULL};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_, kTargetTorqueIdx,
                          kTargetTorqueSubIdx, &offset_out_.target_torque, NULL};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_, kTargetPosIdx,
                          kTargetPosSubIdx, &offset_out_.target_position, NULL};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_, kTargetVelIdx,
                          kTargetVelSubIdx, &offset_out_.target_velocity, NULL};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_, kStatusWordIdx,
                          kStatusWordSubIdx, &offset_in_.status_word, NULL};
  domain_registers_[6] = {alias_, position_, vendor_id_, product_code_, kDisplayOpModeIdx,
                          kDisplayOpModeSubIdx, &offset_in_.display_op_mode, NULL};
  domain_registers_[7] = {alias_, position_, vendor_id_, product_code_,
                          kPosActualValueIdx, kPosActualValueSubIdx,
                          &offset_in_.position_actual_value, NULL};
  domain_registers_[8] = {alias_, position_, vendor_id_, product_code_,
                          kVelActualValueIdx, kVelActualValueSubIdx,
                          &offset_in_.velocity_actual_value, NULL};
  domain_registers_[9] = {alias_, position_, vendor_id_, product_code_,
                          kTorqueActualValueIdx, kTorqueActualValueSubIdx,
                          &offset_in_.torque_actual_value, NULL};
  domain_registers_[10] = {alias_, position_, vendor_id_, product_code_, kDigInIndex,
                           kDigInSubIndex, &offset_in_.digital_inputs, NULL};
  domain_registers_[11] = {alias_, position_, vendor_id_, product_code_,
                           kAuxPosActualValueIdx, kAuxPosActualValueSubIdx,
                           &offset_in_.aux_pos_actual_value, NULL};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);

  drive_state_ = ST_START;
  prev_state_ = static_cast<GoldSoloWhistleDriveStates>(GetCurrentState());
}

GoldSoloWhistleDriveStates
GoldSoloWhistleDrive::GetDriveState(const Bitfield16& status_word)
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

std::string GoldSoloWhistleDrive::GetDriveStateStr(const Bitfield16& status_word)
{
  return kStatesStr_[GetDriveState(status_word)];
}

////////////////////////////////////////////////////////////////////////////
//// Overwritten virtual functions from base class
////////////////////////////////////////////////////////////////////////////

RetVal GoldSoloWhistleDrive::SdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = NULL;

  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(config_ptr, kOpModeIdx,
                                                       kOpModeSubIdx, CYCLIC_POSITION)))
  {
    EcPrintCb(
      QString("Drive %1 failed to create OpMode SDO request").arg(id_).toStdString(),
      'r');
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  if (ecrt_slave_config_sdo8(config_ptr, kOpModeIdx, kOpModeSubIdx, CYCLIC_POSITION) != 0)
  {
    EcPrintCb(QString("Drive %1 failed to add a config value to OpMode SDO")
                .arg(id_)
                .toStdString(),
              'r');
    return ECONFIG;
  }

  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(
          config_ptr, kHomingMethodIdx, kHomingMethodSubIdx, kHomingOnPosMethod)))
  {
    EcPrintCb(QString("Drive %1 failed to create HomingMethod SDO request")
                .arg(id_)
                .toStdString(),
              'r');
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  if (ecrt_slave_config_sdo8(config_ptr, kHomingMethodIdx, kHomingMethodSubIdx,
                             kHomingOnPosMethod) != 0)
  {
    EcPrintCb(QString("Drive %1 failed to add a config value to HomingMethod SDO")
                .arg(id_)
                .toStdString(),
              'r');
    return ECONFIG;
  }

  return OK;
}

void GoldSoloWhistleDrive::ReadInputs()
{
  input_pdos_.status_word.SetBitset(
    EC_READ_U16(domain_data_ptr_ + offset_in_.status_word));
  input_pdos_.display_op_mode = EC_READ_S8(domain_data_ptr_ + offset_in_.display_op_mode);
  input_pdos_.pos_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.position_actual_value);
  input_pdos_.vel_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.velocity_actual_value);
  input_pdos_.torque_actual_value =
    EC_READ_S16(domain_data_ptr_ + offset_in_.torque_actual_value);
  input_pdos_.digital_inputs = EC_READ_U32(domain_data_ptr_ + offset_in_.digital_inputs);
  input_pdos_.aux_pos_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.aux_pos_actual_value);

  drive_state_ = GetDriveState(input_pdos_.status_word);
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

void GoldSoloWhistleDrive::WriteOutputs()
{
  EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word,
               output_pdos_.control_word.GetBitset().to_ulong());
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

void GoldSoloWhistleDrive::EcPrintCb(const std::string& msg,
                                     const char color /* = 'w' */) const
{
  EthercatSlave::EcPrintCb(msg, color);
  if (color == 'r')
    emit printMessage(msg.c_str());
  else
    emit logMessage(msg.c_str());
}

////////////////////////////////////////////////////////////////////////////
//// External events taken by this state machine
////////////////////////////////////////////////////////////////////////////

void GoldSoloWhistleDrive::Shutdown()
{
  PrintCommand("Shutdown");
  output_pdos_.control_word.Clear(ControlBit::SWITCH_ON);
  output_pdos_.control_word.Set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::SwitchOn()
{
  PrintCommand("SwitchOn");
  // Trigger device control command
  output_pdos_.control_word.Set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.Set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.Clear(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
  // Setup default operational mode before enabling the drive
  output_pdos_.op_mode = CYCLIC_POSITION;
  output_pdos_.target_position = input_pdos_.pos_actual_value;
}

void GoldSoloWhistleDrive::EnableOperation()
{
  PrintCommand("EnableOperation");
  // Trigger device control command
  output_pdos_.control_word.Set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.Set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.Set(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::DisableOperation()
{
  PrintCommand("DisableOperation");
  // Trigger device control command
  output_pdos_.control_word.Set(ControlBit::SWITCH_ON);
  output_pdos_.control_word.Set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Set(ControlBit::QUICK_STOP);
  output_pdos_.control_word.Clear(ControlBit::ENABLE_OPERATION);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::DisableVoltage()
{
  PrintCommand("DisableVoltage");
  // Trigger device control command
  output_pdos_.control_word.Clear(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::QuickStop()
{
  PrintCommand("QuickStop");
  // Trigger device control command
  output_pdos_.control_word.Set(ControlBit::ENABLE_VOLTAGE);
  output_pdos_.control_word.Clear(ControlBit::QUICK_STOP);
  output_pdos_.control_word.Clear(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::FaultReset()
{
  PrintCommand("FaultReset");
  // Trigger device control command
  output_pdos_.control_word.Set(ControlBit::FAULT);
}

void GoldSoloWhistleDrive::ChangePosition(const int32_t target_position)
{
  static int32_t prev_target = 0;

  if (input_pdos_.display_op_mode == CYCLIC_POSITION && prev_target == target_position)
    return;
  prev_target = target_position;

  //  PrintCommand("ChangePosition");
  //  printf("\tTarget position: %d\n", target_position);

  GoldSoloWhistleDriveData* data =
    new GoldSoloWhistleDriveData(CYCLIC_POSITION, target_position);
  SetChange(data);
}

void GoldSoloWhistleDrive::ChangeDeltaPosition(const int32_t delta_position)
{
  ChangePosition(input_pdos_.pos_actual_value + delta_position);
}

void GoldSoloWhistleDrive::ChangeVelocity(const int32_t target_velocity)
{
  static int32_t prev_target = 0;

  if (input_pdos_.display_op_mode == CYCLIC_VELOCITY && prev_target == target_velocity)
    return;
  prev_target = target_velocity;

  //  PrintCommand("ChangeVelocity");
  //  printf("\tTarget velocity: %d\n", target_velocity);

  GoldSoloWhistleDriveData* data =
    new GoldSoloWhistleDriveData(CYCLIC_VELOCITY, target_velocity);
  SetChange(data);
}

void GoldSoloWhistleDrive::ChangeDeltaVelocity(const int32_t delta_velocity)
{
  ChangeVelocity(input_pdos_.vel_actual_value + delta_velocity);
}

void GoldSoloWhistleDrive::ChangeTorque(const int16_t target_torque)
{
  static int16_t prev_target = 0;

  if (input_pdos_.display_op_mode == CYCLIC_TORQUE && prev_target == target_torque)
    return;
  prev_target = target_torque;

  //  PrintCommand("ChangeTorque");
  //  printf("\tTarget torque: %d\n", target_torque);

  GoldSoloWhistleDriveData* data =
    new GoldSoloWhistleDriveData(CYCLIC_TORQUE, target_torque);
  SetChange(data);
}

void GoldSoloWhistleDrive::ChangeDeltaTorque(const int16_t delta_torque)
{
  ChangeTorque(input_pdos_.torque_actual_value + delta_torque);
}

void GoldSoloWhistleDrive::ChangeOpMode(const int8_t target_op_mode)
{
  PrintCommand("ChangeOpMode");
  // Set target value to current one
  GoldSoloWhistleDriveData* data =
    new GoldSoloWhistleDriveData(target_op_mode, input_pdos_, true);
  SetChange(data);
}

void GoldSoloWhistleDrive::SetTargetDefaults()
{
  PrintCommand("SetTargetDefaults");
  // Set target operational mode and value to current ones
  GoldSoloWhistleDriveData* data =
    new GoldSoloWhistleDriveData(input_pdos_.display_op_mode, input_pdos_, true);
  // Apply
  SetChange(data);
}

void GoldSoloWhistleDrive::SetChange(const GoldSoloWhistleDriveData* data)
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                               // - Current State -
    TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)              // ST_START
    TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)              // ST_NOT_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)               // ST_SWITCH_ON_DISABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)               // ST_READY_TO_SWITCH_ON
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)               // ST_SWITCHED_ON
    TRANSITION_MAP_ENTRY(ST_OPERATION_ENABLED) // ST_OPERATION_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_QUICK_STOP_ACTIVE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_FAULT_REACTION_ACTIVE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)              // ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

////////////////////////////////////////////////////////////////////////////
//// States actions
////////////////////////////////////////////////////////////////////////////

STATE_DEFINE(GoldSoloWhistleDrive, Start, NoEventData)
{
  prev_state_ = ST_START;
  EcPrintCb(QString("Drive %1 initial state: %2")
              .arg(id_)
              .arg(kStatesStr_[ST_START])
              .toStdString());
  // This happens automatically on drive's start up. We simply imitate the behavior here.
  InternalEvent(ST_NOT_READY_TO_SWITCH_ON);
}

STATE_DEFINE(GoldSoloWhistleDrive, NotReadyToSwitchOn, NoEventData)
{
  PrintStateTransition(prev_state_, ST_NOT_READY_TO_SWITCH_ON);
  prev_state_ = ST_NOT_READY_TO_SWITCH_ON;
  // This happens automatically on drive's start up. We simply imitate the behavior here.
  InternalEvent(ST_SWITCH_ON_DISABLED);
}

STATE_DEFINE(GoldSoloWhistleDrive, SwitchOnDisabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_SWITCH_ON_DISABLED);
  prev_state_ = ST_SWITCH_ON_DISABLED;
}

STATE_DEFINE(GoldSoloWhistleDrive, ReadyToSwitchOn, NoEventData)
{
  PrintStateTransition(prev_state_, ST_READY_TO_SWITCH_ON);
  prev_state_ = ST_READY_TO_SWITCH_ON;
}

STATE_DEFINE(GoldSoloWhistleDrive, SwitchedOn, NoEventData)
{
  PrintStateTransition(prev_state_, ST_SWITCHED_ON);
  prev_state_ = ST_SWITCHED_ON;
}

STATE_DEFINE(GoldSoloWhistleDrive, OperationEnabled, GoldSoloWhistleDriveData)
{
  PrintStateTransition(prev_state_, ST_OPERATION_ENABLED);

  // Setup operational mode and target value
  output_pdos_.op_mode = data->op_mode;
  switch (data->op_mode)
  {
  case CYCLIC_POSITION:
    output_pdos_.target_position = data->value;
    break;
  case CYCLIC_VELOCITY:
    output_pdos_.target_velocity = data->value;
    break;
  case CYCLIC_TORQUE:
    output_pdos_.target_torque = static_cast<int16_t>(data->value);
    break;
  default:
    break;
  }

  prev_state_ = ST_OPERATION_ENABLED;
}

STATE_DEFINE(GoldSoloWhistleDrive, QuickStopActive, NoEventData)
{
  PrintStateTransition(prev_state_, ST_QUICK_STOP_ACTIVE);
  prev_state_ = ST_QUICK_STOP_ACTIVE;
}

STATE_DEFINE(GoldSoloWhistleDrive, FaultReactionActive, NoEventData)
{
  PrintStateTransition(prev_state_, ST_FAULT_REACTION_ACTIVE);
  prev_state_ = ST_FAULT_REACTION_ACTIVE;
}

STATE_DEFINE(GoldSoloWhistleDrive, Fault, NoEventData)
{
  PrintStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;

  emit driveFaulted();
}

////////////////////////////////////////////////////////////////////////////
//// Miscellaneous
////////////////////////////////////////////////////////////////////////////

inline void GoldSoloWhistleDrive::PrintCommand(const char* cmd) const
{
  EcPrintCb(QString("Drive %1 received command: %2").arg(id_).arg(cmd).toStdString());
}

void GoldSoloWhistleDrive::PrintStateTransition(
  const GoldSoloWhistleDriveStates current_state,
  const GoldSoloWhistleDriveStates new_state) const
{
  if (current_state == new_state)
    return;
  EcPrintCb(QString("Drive %1 state transition: %2 --> %3")
              .arg(id_)
              .arg(kStatesStr_[current_state], kStatesStr_[new_state])
              .toStdString());
}

} // end namespace grabec
