#include "slaves/goldsolowhistledrive.h"

namespace grabec
{

GoldSoloWhistleDrive::GoldSoloWhistleDrive(uint8_t slave_position): StateMachine(ST_MAX_STATE)
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
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs);

  requested_state_ = ST_MAX_STATE;
  drive_state_ = ST_IDLE;
  requeste_operation_state_ = NULL_OPERATION;
  operation_state_ = CYCLIC_POSITION;
}

void GoldSoloWhistleDrive::SetTargetDefaults()
{
  switch (input_pdos_.display_op_mode)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.target_position = input_pdos_.pos_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.target_velocity = input_pdos_.vel_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

RetVal GoldSoloWhistleDrive::SdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = NULL;

  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(config_ptr, kOpModeIdx,
                                                       kOpModeSubIdx, CYCLIC_POSITION)))
  {
    std::cout << "Failed to create SDO request." << std::endl;
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  ecrt_slave_config_sdo8(config_ptr, kOpModeIdx, kOpModeSubIdx, CYCLIC_POSITION);
  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(
          config_ptr, kHomingMethodIdx, kHomingMethodSubIdx, kHomingOnPosMethod)))
  {
    std::cout << "Failed to create SDO request." << std::endl;
    return ECONFIG;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  ecrt_slave_config_sdo8(config_ptr, kHomingMethodIdx, kHomingMethodSubIdx,
                         kHomingOnPosMethod);
  return OK;
}

void GoldSoloWhistleDrive::DoWork() { (this->*state_machine_[drive_state_])(); }

void GoldSoloWhistleDrive::ReadInputs()
{
  input_pdos_.status_word = EC_READ_U16(domain_data_ptr_ + offset_in_.status_word);
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
  drive_state_ = GetDriveState();
  if (drive_state_ != GetCurrentState())
    InternalEvent(drive_state_);
}

void GoldSoloWhistleDrive::WriteOutputs()
{
  EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word,
               static_cast<unsigned short>(output_pdos_.control_word.to_ulong()));
  EC_WRITE_S8(domain_data_ptr_ + offset_out_.op_mode, output_pdos_.op_mode);
  if (drive_state_ == ST_OPERATION_ENABLED || drive_state_ == ST_SWITCH_ON)
  {
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_position,
                 output_pdos_.target_position);
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_velocity,
                 output_pdos_.target_velocity);
    EC_WRITE_S16(domain_data_ptr_ + offset_out_.target_torque,
                 output_pdos_.target_torque);
  }
}

GoldSoloWhistleDrive::States GoldSoloWhistleDrive::GetDriveState()
{
  if (input_pdos_.status_word[StatusBit::OFF] == SET)
  { // drive idle: OFF=true
    return ST_IDLE;
  }
  if (input_pdos_.status_word[StatusBit::ON] == SET)
  {
    if (input_pdos_.status_word[StatusBit::SWITCH_ON] == UNSET)
    { // drive in operational progress: OFF=false, ON=true, SWITCH_ON=false
      return ST_READY2SWITCH_ON;
    }
    if (input_pdos_.status_word[StatusBit::ENABLED] == UNSET)
    { // drive in operational progress: OFF=false, ON=true, SWITCH_ON=true, ENABLED=false
      return ST_SWITCH_ON;
    }
    // drive operational: OFF=false, ON=true, SWITCH_ON=true, ENABLED=true
    DetermineOperationState();
    return ST_OPERATION_ENABLED;
  }
  if (input_pdos_.status_word[StatusBit::FAULT] == UNSET)
  { // drive in quick stop: OFF=false, ON=false, FAULT=false
    return ST_QUICK_STOP_ACTIVE;
  }
  if (input_pdos_.status_word[StatusBit::ENABLED] == SET)
  { // drive in fault reaction: OFF=false, ON=false, FAULT=true, ENABLED=true
    return ST_FAULT_REACTION_ACTIVE;
  }
  // drive in fault: OFF=false, ON=false, FAULT=true, ENABLED=false
  return ST_FAULT;
}

void GoldSoloWhistleDrive::DetermineOperationState()
{
  switch (input_pdos_.display_op_mode)
  {
  case CYCLIC_POSITION:
  {
    operation_state_ = CYCLIC_POSITION;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    operation_state_ = CYCLIC_VELOCITY;
    break;
  }
  case CYCLIC_TORQUE:
  {
    operation_state_ = CYCLIC_TORQUE;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::SwitchOnDisabledFun() {}

void GoldSoloWhistleDrive::ReadyToSwitchOnFun() {}

void GoldSoloWhistleDrive::SwitchOnFun() {}

void GoldSoloWhistleDrive::OperationEnabledFun() {}

void GoldSoloWhistleDrive::QuickStopActiveFun() {}

void GoldSoloWhistleDrive::FaultReactionActiveFun() {}

void GoldSoloWhistleDrive::FaultFun() {}

void GoldSoloWhistleDrive::SwitchOnDisabledTransitions()
{
  switch (requested_state_)
  {
  case ST_IDLE:
  { // We previously asked for a state change: it occurred
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Idle." << std::endl;
    requested_state_ = ST_MAX_STATE;
    break;
  }
  case ST_READY2SWITCH_ON:
  { // We are starting the enabling sequence, transition 2
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " requesting Ready To Switch On." << std::endl;
    output_pdos_.control_word[ControlBit::SWITCH_ON] = UNSET;
    output_pdos_.control_word[ControlBit::ENABLE_VOLTAGE] = SET;
    output_pdos_.control_word[ControlBit::QUICK_STOP] = SET;
    output_pdos_.control_word[ControlBit::ENABLE] = UNSET;
    output_pdos_.control_word[ControlBit::FAULT] = UNSET;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::ReadyToSwitchOnTransitions()
{
  switch (requested_state_)
  {
  case ST_READY2SWITCH_ON:
  { // We previously asked for a feasible state change, now we ask for another
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Ready To Switch On." << std::endl;
    output_pdos_.control_word[ControlBit::SWITCH_ON] = SET;
    requested_state_ = ST_SWITCH_ON;
    output_pdos_.op_mode = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.pos_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::SwitchOnTransitions()
{
  switch (requested_state_)
  {
  case ST_SWITCH_ON:
  { // We previously asked for a feasible state change, now we ask for another
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Switch On." << std::endl;
    // outputPdos.TargetPosition = inputPdos.positionActualValue;
    // outputPdos.TargetTorque = inputPdos.torqueActualValue;
    // outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    output_pdos_.control_word[ControlBit::ENABLE] = SET;
    requested_state_ = ST_OPERATION_ENABLED;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::OperationEnabledTransitions()
{
  switch (requested_state_)
  {
  case ST_OPERATION_ENABLED:
  { // We previously asked for a state change: it occurred
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << "  Enabled." << std::endl;
    requested_state_ = ST_MAX_STATE;
    (this->*operation_state_manager_[operation_state_ - kOperationOffset])();
    break;
  }
  case ST_IDLE:
  { // We want to disable the drive
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " going Idle" << std::endl;
    output_pdos_.control_word.reset();
    break;
  }
  default:
  {
    (this->*operation_state_manager_[operation_state_ - kOperationOffset])();
    break;
  }
  }
}

void GoldSoloWhistleDrive::QuickStopActiveTransitions()
{
  requested_state_ = ST_MAX_STATE; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultReactionActiveTransitions()
{
  requested_state_ = ST_MAX_STATE; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultTransitions()
{
  switch (requested_state_)
  {
  case ST_IDLE:
  { // we are requesting a fault reset
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " requesting Fault Reset: going Idle"
              << std::endl;
    output_pdos_.control_word.reset();
    output_pdos_.control_word[ControlBit::FAULT] = SET;
    break;
  }
  case ST_MAX_STATE:
    break;
  default:
  {
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_
              << " encountered an error asking for state change code: "
              << requested_state_ << std::endl;
    requested_state_ = ST_MAX_STATE;
    break;
  }
  }
}

void GoldSoloWhistleDrive::CyclicPositionFun() {}

void GoldSoloWhistleDrive::CyclicVelocityFun() {}

void GoldSoloWhistleDrive::CyclicTorqueFun() {}

void GoldSoloWhistleDrive::CyclicPositionTransition()
{
  switch (requeste_operation_state_)
  {
  case CYCLIC_POSITION:
  {
    requeste_operation_state_ = NULL_OPERATION;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.op_mode = CYCLIC_VELOCITY;
    output_pdos_.target_velocity = input_pdos_.vel_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.op_mode = CYCLIC_TORQUE;
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicVelocityTransition()
{
  switch (requeste_operation_state_)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.op_mode = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.pos_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    requeste_operation_state_ = NULL_OPERATION;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.op_mode = CYCLIC_TORQUE;
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicTorqueTransition()
{
  switch (requeste_operation_state_)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.op_mode = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.pos_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.op_mode = CYCLIC_VELOCITY;
    output_pdos_.target_velocity = input_pdos_.vel_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    requeste_operation_state_ = NULL_OPERATION;
    break;
  }
  default:
    break;
  }
}

} // end namespace grabec
