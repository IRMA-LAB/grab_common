/**
 * @file Cable_robot_winch_slave.h
 * @author Simone Comari
 * @date 08 Feb 2021
 * @brief File containing class implementation declared in cable_robot_winch_slave.h.
 */

#include "slaves/easycat/cable_robot_winch_slave.h"

namespace grabec {

//------------------------------------------------------------------------------------//
// CableRobotWinchData
//------------------------------------------------------------------------------------//

CableRobotWinchData::CableRobotWinchData(const int8_t _op_mode,
                                         const int32_t _value /*= 0*/,
                                         const bool verbose /* = false */)
  : op_mode(_op_mode), value(_value)
{
  if (verbose)
    switch (op_mode)
    {
      case CableRobotWinchOperationModes::CYCLIC_POSITION:
        printf("\tTarget operational mode: CYCLIC_POSITION @ %d\n", value);
        break;
      case CableRobotWinchOperationModes::CYCLIC_VELOCITY:
        printf("\tTarget operational mode: CYCLIC_VELOCITY @ %d\n", value);
        break;
      case CableRobotWinchOperationModes::CYCLIC_TORQUE:
        printf("\tTarget operational mode: CYCLIC_TORQUE @ %d\n", value);
        break;
      case CableRobotWinchOperationModes::NONE:
        printf("\tTarget operational mode: NONE\n");
        break;
    }
}

CableRobotWinchData::CableRobotWinchData(const int8_t _op_mode,
                                         const CRWSlaveInPdos& input_pdos,
                                         const bool verbose /* = false */)
  : op_mode(_op_mode)
{
  // Set target value to current one
  switch (op_mode)
  {
    case CableRobotWinchOperationModes::CYCLIC_POSITION:
      value = input_pdos.Cust.actual_position;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_POSITION @ %d\n", value);
      break;
    case CableRobotWinchOperationModes::CYCLIC_VELOCITY:
      value = input_pdos.Cust.actual_speed;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_VELOCITY @ %d\n", value);
      break;
    case CableRobotWinchOperationModes::CYCLIC_TORQUE:
      value = std::max(input_pdos.Cust.actual_torque, kMinInitTorque_);
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_TORQUE @ %d\n", value);
      break;
    default:
      if (verbose)
        printf("\tTarget operational mode: NONE\n");
      break;
  }
}

CableRobotWinchData::CableRobotWinchData(const CRWSlaveInPdos& input_pdos,
                                         const bool verbose /* = false */)
{
  // Get operational mode from status word
  op_mode = CableRobotWinchSlave::GetDriveOpMode(input_pdos.Cust.status_word);
  // Set target value to current one
  switch (op_mode)
  {
    case CableRobotWinchOperationModes::CYCLIC_POSITION:
      value = input_pdos.Cust.actual_position;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_POSITION @ %d\n", value);
      break;
    case CableRobotWinchOperationModes::CYCLIC_VELOCITY:
      value = input_pdos.Cust.actual_speed;
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_VELOCITY @ %d\n", value);
      break;
    case CableRobotWinchOperationModes::CYCLIC_TORQUE:
      value = std::max(input_pdos.Cust.actual_torque, kMinInitTorque_);
      if (verbose)
        printf("\tTarget operational mode: CYCLIC_TORQUE @ %d\n", value);
      break;
    case CableRobotWinchOperationModes::NONE:
      if (verbose)
        printf("\tTarget operational mode: NONE\n");
      break;
  }
}

//------------------------------------------------------------------------------------//
// CableRobotWinchSlave
//------------------------------------------------------------------------------------//

// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t CableRobotWinchSlave::kPdoEntries_[];
constexpr ec_pdo_info_t CableRobotWinchSlave::kPDOs_[];
constexpr ec_sync_info_t CableRobotWinchSlave::kSyncs_[];
constexpr char* CableRobotWinchSlave::kStatesStr_[];

CableRobotWinchSlave::CableRobotWinchSlave(const id_t id, const uint8_t slave_position,
                                           QObject* parent /*=nullptr*/)
  : QObject(parent), StateMachine(ST_MAX_STATES)
{
  alias_              = kAlias_;
  vendor_id_          = kVendorID_;
  product_code_       = kProductCode_;
  num_domain_entries_ = kDomainEntries_;
  position_           = slave_position;
  id_                 = id;

  domain_registers_[0] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[0].index,
                          kPdoEntries_[0].subindex,
                          &offset_out_.target_position,
                          nullptr};
  domain_registers_[1] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[1].index,
                          kPdoEntries_[1].subindex,
                          &offset_out_.target_speed,
                          nullptr};
  domain_registers_[2] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[2].index,
                          kPdoEntries_[2].subindex,
                          &offset_out_.control_word,
                          nullptr};
  domain_registers_[3] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[3].index,
                          kPdoEntries_[3].subindex,
                          &offset_out_.target_torque,
                          nullptr};
  domain_registers_[4] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[4].index,
                          kPdoEntries_[4].subindex,
                          &offset_in_.actual_position,
                          nullptr};
  domain_registers_[5] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[5].index,
                          kPdoEntries_[5].subindex,
                          &offset_in_.actual_speed,
                          nullptr};
  domain_registers_[6] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[6].index,
                          kPdoEntries_[6].subindex,
                          &offset_in_.status_word,
                          nullptr};
  domain_registers_[7] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[7].index,
                          kPdoEntries_[7].subindex,
                          &offset_in_.loadcell_value,
                          nullptr};
  domain_registers_[8] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[8].index,
                          kPdoEntries_[8].subindex,
                          &offset_in_.actual_torque,
                          nullptr};
  domain_registers_[9] = {alias_,
                          position_,
                          vendor_id_,
                          product_code_,
                          kPdoEntries_[9].index,
                          kPdoEntries_[9].subindex,
                          &offset_in_.actual_position_aux,
                          nullptr};

  domain_registers_ptr_  = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_        = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_        = const_cast<ec_sync_info_t*>(kSyncs_);

  drive_state_ = ST_MAX_STATES;
  prev_state_  = static_cast<States>(GetCurrentState());
}

CableRobotWinchSlave::States
CableRobotWinchSlave::GetDriveState(const uint16_t _status_word)
{
  std::bitset<8> status_word(_status_word);
  if (status_word[StatusBit::IDLE_STATE] == SET)
    return ST_IDLE;
  if (status_word[StatusBit::OPERATIONAL_STATE] == SET)
    return ST_OPERATIONAL;
  return ST_ERROR;
}

std::string CableRobotWinchSlave::GetDriveStateStr(const uint16_t status_word)
{
  return kStatesStr_[GetDriveState(status_word)];
}

CableRobotWinchOperationModes
CableRobotWinchSlave::GetDriveOpMode(const uint16_t _status_word)
{
  std::bitset<8> status_word(_status_word);
  if (status_word[StatusBit::OPERATIONAL_STATE] == UNSET)
    return CableRobotWinchOperationModes::NONE;
  if (status_word[StatusBit::POSITION_CONTROL] == SET)
    return CableRobotWinchOperationModes::CYCLIC_POSITION;
  if (status_word[StatusBit::SPEED_CONTROL] == SET)
    return CableRobotWinchOperationModes::CYCLIC_VELOCITY;
  if (status_word[StatusBit::TORQUE_CONTROL] == SET)
    return CableRobotWinchOperationModes::CYCLIC_TORQUE;
  return CableRobotWinchOperationModes::NONE;
}

CableRobotWinchOperationModes CableRobotWinchSlave::GetOpMode() const
{
  return GetDriveOpMode(BufferIn.Cust.status_word);
}

//----- Overwritten virtual functions from base class --------------------------------//

void CableRobotWinchSlave::ReadInputs()
{
  // This is the way we can read the PDOs, according to ecrt.h
  BufferIn.Cust.actual_position =
    EC_READ_S32(domain_data_ptr_ + offset_in_.actual_position);
  BufferIn.Cust.actual_speed = EC_READ_S32(domain_data_ptr_ + offset_in_.actual_speed);
  BufferIn.Cust.status_word  = EC_READ_U16(domain_data_ptr_ + offset_in_.status_word);
  BufferIn.Cust.loadcell_value =
    EC_READ_U16(domain_data_ptr_ + offset_in_.loadcell_value);
  BufferIn.Cust.actual_torque = EC_READ_S16(domain_data_ptr_ + offset_in_.actual_torque);
  BufferIn.Cust.actual_position_aux =
    EC_READ_S16(domain_data_ptr_ + offset_in_.actual_position_aux);

  // The actual state transition is triggered here
  drive_state_ = GetDriveState(BufferIn.Cust.status_word);
  if (drive_state_ != GetCurrentState())
    ExternalEvent(drive_state_);
}

void CableRobotWinchSlave::WriteOutputs()
{
  EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word, BufferOut.Cust.control_word);
  if (drive_state_ != ST_OPERATIONAL)
    return;
  EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_position,
               BufferOut.Cust.target_position);
  EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_speed, BufferOut.Cust.target_speed);
  EC_WRITE_S16(domain_data_ptr_ + offset_out_.target_torque,
               BufferOut.Cust.target_torque);
}

void CableRobotWinchSlave::SafeExit()
{
  /*
   * Your code here..
   */
}

bool CableRobotWinchSlave::IsReadyToShutDown() const
{
  /*
   * Your code here..
   * Return bool accordingly..
   */
  return true;
}

void CableRobotWinchSlave::InitFun()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                      // - Current State -
    TRANSITION_MAP_ENTRY (ST_IDLE)          // ST_IDLE
    TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)    // ST_OPERATIONAL
    TRANSITION_MAP_ENTRY (EVENT_IGNORED)    // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobotWinchSlave::EcPrintCb(const std::string& msg,
                                     const char color /* = 'w' */) const
{
  EthercatSlave::EcPrintCb(msg, color);
  if (color == 'r')
    emit printMessage(msg.c_str());
  else
    emit logMessage(msg.c_str());
}

//----- External events taken by this state machine ----------------------------------//

void CableRobotWinchSlave::EnableOperation()
{
  PrintCommand("EnableOperation");
  // Trigger device control command
  ChangeOpMode(CableRobotWinchOperationModes::CYCLIC_POSITION);
}

void CableRobotWinchSlave::DisableOperation()
{
  PrintCommand("DisableOperation");
  // Trigger device control command
  std::bitset<16> control_word;
  control_word.set(StatusBit::IDLE_STATE);
  BufferOut.Cust.control_word = static_cast<uint16_t>(control_word.to_ulong());
}

void CableRobotWinchSlave::FaultReset()
{
  PrintCommand("FaultReset");
  // Trigger device control command
  std::bitset<16> control_word;
  control_word.set(StatusBit::ERROR_RESET);
  control_word.set(StatusBit::IDLE_STATE);
  BufferOut.Cust.control_word = static_cast<uint16_t>(control_word.to_ulong());
}

void CableRobotWinchSlave::ChangePosition(const int32_t target_position)
{
  if (GetOpMode() == CableRobotWinchOperationModes::CYCLIC_POSITION &&
      prev_pos_target_ == target_position)
    return;
  prev_pos_target_ = target_position;

  //  PrintCommand("ChangePosition");
  //  printf("\tTarget position: %d\n", target_position);

  CableRobotWinchData data =
    CableRobotWinchData(CableRobotWinchOperationModes::CYCLIC_POSITION, target_position);
  SetChange(data);
}

void CableRobotWinchSlave::ChangeVelocity(const int32_t target_velocity)
{
  if (GetOpMode() == CableRobotWinchOperationModes::CYCLIC_VELOCITY &&
      prev_vel_target_ == target_velocity)
    return;
  prev_vel_target_ = target_velocity;

  //  PrintCommand("ChangeVelocity");
  //  printf("\tTarget velocity: %d\n", target_velocity);

  CableRobotWinchData data =
    CableRobotWinchData(CableRobotWinchOperationModes::CYCLIC_VELOCITY, target_velocity);
  SetChange(data);
}

void CableRobotWinchSlave::ChangeTorque(const int16_t target_torque)
{
  if (GetOpMode() == CableRobotWinchOperationModes::CYCLIC_TORQUE &&
      prev_torque_target_ == target_torque)
    return;
  prev_torque_target_ = target_torque;

  //  PrintCommand("ChangeTorque");
  //  printf("\tTarget torque: %d\n", target_torque);

  CableRobotWinchData data =
    CableRobotWinchData(CableRobotWinchOperationModes::CYCLIC_TORQUE, target_torque);
  SetChange(data);
}

void CableRobotWinchSlave::ChangeOpMode(const int8_t target_op_mode)
{
  PrintCommand("ChangeOpMode");
  // Set target value to current one
  CableRobotWinchData data = CableRobotWinchData(target_op_mode, BufferIn, true);
  SetChange(data);
}

void CableRobotWinchSlave::SetTargetDefaults()
{
  PrintCommand("SetTargetDefaults");
  // Set target operational mode and value to current ones
  CableRobotWinchData data = CableRobotWinchData(BufferIn, true);
  // Apply
  SetChange(data);
}

void CableRobotWinchSlave::SetChange(const CableRobotWinchData& data)
{
  // Setup operational mode and target value
  std::bitset<8> control_word;
  control_word.set(StatusBit::OPERATIONAL_STATE);
  switch (data.op_mode)
  {
    case CableRobotWinchOperationModes::CYCLIC_POSITION:
      BufferOut.Cust.target_position = data.value;
      control_word.set(StatusBit::POSITION_CONTROL);
      break;
    case CableRobotWinchOperationModes::CYCLIC_VELOCITY:
      BufferOut.Cust.target_speed = data.value;
      control_word.set(StatusBit::SPEED_CONTROL);
      break;
    case CableRobotWinchOperationModes::CYCLIC_TORQUE:
      BufferOut.Cust.target_torque = static_cast<int16_t>(data.value);
      control_word.set(StatusBit::TORQUE_CONTROL);
      break;
    default:
      break;
  }
  BufferOut.Cust.control_word = static_cast<uint16_t>(control_word.to_ulong());
}

//----- States actions ---------------------------------------------------------------//

STATE_DEFINE(CableRobotWinchSlave, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  std::bitset<16> control_word;
  control_word.set(StatusBit::IDLE_STATE);
  BufferOut.Cust.control_word = static_cast<uint16_t>(control_word.to_ulong());
  prev_state_                 = ST_IDLE;
}

STATE_DEFINE(CableRobotWinchSlave, Operational, NoEventData)
{
  PrintStateTransition(prev_state_, ST_OPERATIONAL);
  SetTargetDefaults();
  prev_state_ = ST_OPERATIONAL;
}

STATE_DEFINE(CableRobotWinchSlave, Error, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ERROR);
  prev_state_ = ST_ERROR;
  emit driveFaulted();
}

//----- Miscellaneous ----------------------------------------------------------------//

inline void CableRobotWinchSlave::PrintCommand(const char* cmd) const
{
  std::string msg;
  msg = QString("Drive %1 received command: %2").arg(id_).arg(cmd).toStdString();
  EcPrintCb(msg);
}

void CableRobotWinchSlave::PrintStateTransition(const States current_state,
                                                const States new_state) const
{
  if (current_state == new_state)
    return;
  std::string msg;
  msg = QString("Drive %1 state transition: %2 --> %3")
          .arg(id_)
          .arg(kStatesStr_[current_state], kStatesStr_[new_state])
          .toStdString();
  EcPrintCb(msg);
}

} // end namespace grabec
