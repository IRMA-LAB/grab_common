#ifndef GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H
#define GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H

#include <bitset>
#include <iostream>
#include "ethercatslave.h"
#include "types.h"
#include "grabcommon.h"
#include "state_machine/inc/StateMachine.h"

namespace grabec
{

/**
*@brief The GoldSoloWhistleDrive class
*/
class GoldSoloWhistleDrive : public EthercatSlave, public StateMachine
{
public:
  enum States: uint8_t
  {
    ST_IDLE,
    ST_READY2SWITCH_ON,
    ST_SWITCH_ON,
    ST_OPERATION_ENABLED,
    ST_QUICK_STOP_ACTIVE,     // To be implemented, fast stop during operation
    ST_FAULT_REACTION_ACTIVE, // To be implemented, reaction to a specific fault
    ST_FAULT,
    ST_MAX_STATE
  };

  /**
   * @brief GoldSoloWhistleDrive
   * @param slave_position
   */
  GoldSoloWhistleDrive(uint8_t slave_position);

  /**
   * @brief SetTargetDefaults
   */
  void SetTargetDefaults();
  /**
   * @brief GetDriveState
   */
  States GetDriveState();
  /**
   * @brief DetermineOperationState
   */
  void DetermineOperationState();

  virtual RetVal SdoRequests(ec_slave_config_t* config_ptr) final;
  virtual void DoWork() final;
  virtual void ReadInputs() final;
  virtual void WriteOutputs() final;

private:
  static constexpr uint8_t kDomainInputs = 7;
  static constexpr uint8_t kDomainOutputs = 5;
  static constexpr uint8_t kDomainEntries = kDomainInputs + kDomainOutputs;
  static constexpr uint8_t kAlias = 0;
  static constexpr uint32_t kVendorID = 0x0000009a;
  static constexpr uint32_t kProductCode = 0x00030924;
  static constexpr uint16_t kControlWordIdx = 0x6040;
  static constexpr uint8_t kControlWordSubIdx = 0x00;
  static constexpr uint16_t kHomingMethodIdx = 0x6098;
  static constexpr uint8_t kHomingMethodSubIdx = 0x00;
  static constexpr uint16_t kOpModeIdx = 0x6060;
  static constexpr uint8_t kOpModeSubIdx = 0x00;
  static constexpr uint16_t kTargetTorqueIdx = 0x6071;
  static constexpr uint8_t kTargetTorqueSubIdx = 0x00;
  static constexpr uint16_t kTargetPosIdx = 0x607a;
  static constexpr uint8_t kTargetPosSubIdx = 0x00;
  static constexpr uint16_t kTargetVelIdx = 0x60FF;
  static constexpr uint8_t kTargetVelSubIdx = 0x00;
  static constexpr uint16_t kStatusWordIdx = 0x6041;
  static constexpr uint8_t kStatusWordSubIdx = 0x00;
  static constexpr uint16_t kDisplayOpModeIdx = 0x6061;
  static constexpr uint8_t kDisplayOpModeSubIdx = 0x00;
  static constexpr uint16_t kPosActualValueIdx = 0x6064;
  static constexpr uint8_t kPosActualValueSubIdx = 0x00;
  static constexpr uint16_t kVelActualValueIdx = 0x606C;
  static constexpr uint8_t kVelActualValueSubIdx = 0x00;
  static constexpr uint16_t kTorqueActualValueIdx = 0x6077;
  static constexpr uint8_t kTorqueActualValueSubIdx = 0x00;
  static constexpr uint16_t kDigInIndex = 0x60FD;
  static constexpr uint8_t kDigInSubIndex = 0x00;
  static constexpr uint16_t kAuxPosActualValueIdx = 0x20A0;
  static constexpr uint8_t kAuxPosActualValueSubIdx = 0x00;
  static constexpr uint8_t kHomingOnPosMethod = 35;
  static constexpr uint8_t kNumSupportedOperations = 3;
  static constexpr uint8_t kOperationOffset = 8;

  // ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_pdo_entry_info_t kPdoEntries[kDomainEntries] = {
    {kControlWordIdx, kControlWordSubIdx, 16}, // Start of RxPdo mapping (Outputs)
    {kOpModeIdx, kOpModeSubIdx, 8},
    {kTargetTorqueIdx, kTargetTorqueSubIdx, 16},
    {kTargetPosIdx, kTargetPosSubIdx, 32},
    {kTargetVelIdx, kTargetVelSubIdx, 32},
    {kStatusWordIdx, kStatusWordSubIdx, 16}, // Start of TxPdo mapping (Inputs)
    {kDisplayOpModeIdx, kDisplayOpModeSubIdx, 8},
    {kPosActualValueIdx, kPosActualValueSubIdx, 32},
    {kVelActualValueIdx, kVelActualValueSubIdx, 32},
    {kTorqueActualValueIdx, kTorqueActualValueSubIdx, 16},
    {kDigInIndex, kDigInSubIndex, 32},
    {kAuxPosActualValueIdx, kAuxPosActualValueSubIdx, 32}};

  // ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_pdo_info_t kPDOs[2] = {
    {0x1607, 5, const_cast<ec_pdo_entry_info_t*>(kPdoEntries) + 0}, /* Outputs */
    {0x1a07, 7, const_cast<ec_pdo_entry_info_t*>(kPdoEntries) + 5}, /* Inputs */
  };

  static constexpr ec_sync_info_t kSyncs[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs) + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs) + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, 0x00, EC_WD_DEFAULT}};

  // A simple way to store the pdos input values
  struct InputPdos
  {
    std::bitset<16> status_word;
    signed char display_op_mode;
    int pos_actual_value;
    int vel_actual_value;
    short torque_actual_value;
    unsigned int digital_inputs;
    int aux_pos_actual_value;
  } input_pdos_;

  // A simple way to store the pdos output values
  struct OutputPdos
  {
    std::bitset<16> control_word;
    signed char op_mode;
    short target_torque;
    int target_position;
    int target_velocity;
  } output_pdos_;

  // Useful ethercat struct
  struct OffsetOut
  {
    unsigned int control_word;
    unsigned int op_mode;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int target_velocity;
  } offset_out_;

  // Useful ethercat struct
  struct OffsetIn
  {
    unsigned int status_word;
    unsigned int display_op_mode;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_inputs;
    unsigned int aux_pos_actual_value;
  } offset_in_;

  ENUM_CLASS(StatusBit,
             READY2SWITCH_ON = 0,
             SWITCH_ON = 1,
             ENABLED = 2,
             FAULT = 3,
             ON = 5,
             OFF = 6
             )

  ENUM_CLASS(ControlBit,
             SWITCH_ON = 0,
             ENABLE_VOLTAGE = 1,
             QUICK_STOP = 2,
             ENABLE = 3,
             FAULT = 7
             )

  enum Command : uint8_t
  {
    UNSET,
    SET
  };

  States drive_state_, requested_state_;

  enum GoldSoloWhistleOperationState
  {
    NULL_OPERATION = 0,
    CYCLIC_POSITION = 8,
    CYCLIC_VELOCITY = 9,
    CYCLIC_TORQUE = 10,
  } operation_state_,
    requeste_operation_state_;

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries]; // ethercat utilities

  void SwitchOnDisabledFun();
  void ReadyToSwitchOnFun();
  void SwitchOnFun();
  void OperationEnabledFun();
  void QuickStopActiveFun();
  void FaultReactionActiveFun();
  void FaultFun();

  void SwitchOnDisabledTransitions();
  void ReadyToSwitchOnTransitions();
  void SwitchOnTransitions();
  void OperationEnabledTransitions();
  void QuickStopActiveTransitions();
  void FaultReactionActiveTransitions();
  void FaultTransitions();

  void CyclicPositionFun();
  void CyclicVelocityFun();
  void CyclicTorqueFun();

  void CyclicPositionTransition();
  void CyclicVelocityTransition();
  void CyclicTorqueTransition();

  // Define the state machine state functions with event data type
  STATE_DECLARE(GoldSoloWhistleDrive, Idle, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, Ready2SwitchOn, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, SwitchOn, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, OperationEnabled, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, QuickStopActive, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, FaultReactionActive, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, Fault, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP
  STATE_MAP_ENTRY(&Idle)
  STATE_MAP_ENTRY(&Ready2SwitchOn)
  STATE_MAP_ENTRY(&SwitchOn)
  STATE_MAP_ENTRY(&OperationEnabled)
  STATE_MAP_ENTRY(&QuickStopActive)
  STATE_MAP_ENTRY(&FaultReactionActive)
  STATE_MAP_ENTRY(&Fault)
  END_STATE_MAP

  typedef void (
    GoldSoloWhistleDrive::*StateFunction)(); // Easyway to implement state machine

  // State machine function array
  StateFunction state_machine_[ST_MAX_STATE] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledFun, &GoldSoloWhistleDrive::ReadyToSwitchOnFun,
    &GoldSoloWhistleDrive::SwitchOnFun, &GoldSoloWhistleDrive::OperationEnabledFun,
    &GoldSoloWhistleDrive::QuickStopActiveFun,
    &GoldSoloWhistleDrive::FaultReactionActiveFun, &GoldSoloWhistleDrive::FaultFun};
  // State machine transition function array
  StateFunction state_manager_[ST_MAX_STATE] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledTransitions,
    &GoldSoloWhistleDrive::ReadyToSwitchOnTransitions,
    &GoldSoloWhistleDrive::SwitchOnTransitions,
    &GoldSoloWhistleDrive::OperationEnabledTransitions,
    &GoldSoloWhistleDrive::QuickStopActiveTransitions,
    &GoldSoloWhistleDrive::FaultReactionActiveTransitions,
    &GoldSoloWhistleDrive::FaultTransitions};

  StateFunction operation_state_machine_[kNumSupportedOperations] = {
    &GoldSoloWhistleDrive::CyclicPositionFun, &GoldSoloWhistleDrive::CyclicVelocityFun,
    &GoldSoloWhistleDrive::CyclicTorqueFun};
  // State machine transition function array
  StateFunction operation_state_manager_[kNumSupportedOperations] = {
    &GoldSoloWhistleDrive::CyclicPositionTransition,
    &GoldSoloWhistleDrive::CyclicVelocityTransition,
    &GoldSoloWhistleDrive::CyclicTorqueTransition};
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H
