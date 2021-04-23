#ifndef GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ELMO_DRIVE_H
#define GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ELMO_DRIVE_H

//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project Elmo_Drive.prj
//     Easy Configurator XML Elmo_Drive.xml
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include "ethercatslave.h"
#include "grabec_types.h"
#include <bitset>

namespace grabec
{
struct GSWDriveInPdos
{
  std::bitset<16> status_word; /**< status_word */
  int8_t display_op_mode;      /**< display_op_mode */
  int32_t pos_actual_value;    /**< pos_actual_value */
  int32_t vel_actual_value;    /**< vel_actual_value */
  int16_t torque_actual_value; /**< torque_actual_value */
  int16_t analog_input; /**< torque_actual_value */
  uint digital_inputs;         /**< digital_inputs */
  int aux_pos_actual_value;    /**< aux_pos_actual_value */
};
/**
 * @brief The Elmo_DriveSlave class
 */
class Elmo_DriveSlave : public virtual EthercatSlave
{
public:
  /**
   * @brief Elmo_DriveSlave
   * @param slave_position
   */
  Elmo_DriveSlave(const uint8_t slave_position);
  ~Elmo_DriveSlave() override;

  /**
   * @brief Slave's main function to be cycled.
   */
  void DoWork() override;

  /**
   * @brief Function to specify what to read.
   */
  void ReadInputs() override final;

  /**
   * @brief Function to specify what to write.
   */
  void WriteOutputs() override final;
  /**
   * @brief Function called before shutting down the slave safely.
   */
  void SafeExit() override;
  /**
   * @brief Check if slave is ready to be shut down safely.
   * @return _True_ if slave is ready, _false_ otherwise.
   */
  bool IsReadyToShutDown() const override;

  /**
   * @brief Output buffer union, i.e. data received from master (read).
   */
//  union CustBufferOut
//  {
//    uint8_t Byte[8]; /**< Raw output buffer content. */
//    struct
//    {
//      uint16_t Control_word;
//      int8_t Mode_of_operation;
//      int8_t Homing_method;
//      int16_t Target_Torque;
//      int32_t Target_Position;
//      int32_t Target_Velocity;
//      uint32_t Digital_Outputs;
//      uint32_t Extended_Outputs;
//    } Cust; /**< Custom structure resembling output entries as defined in the config. */
//  } BufferOut; /**< Output buffer, i.e. data received from master (read). */

//  /**
//   * @brief Input buffer union, i.e. data sent to master (write).
//   */
//  union CustBufferIn
//  {
//    uint8_t Byte[12]; /**< Raw input buffer content. */
//    struct
//    {
//      uint16_t Status_word;
//      int8_t Mode_of_operation_display;
//      int32_t Position_actual_value;
//      int32_t Velocity_actual_value;
//      int16_t Torque_actual_value;
//      int32_t Digital_Inputs;
//      int16_t Analog_Input_1;
//      int32_t Auxiliary_position_actual_value;
//      int16_t Current_actual_value;
//      uint32_t Extended_Inputs_Value;
//    } Cust; /**< Custom structure resembling input entries as defined in the config. */
//  } BufferIn; /**< Input buffer, i.e. data sent to master (write). */
  GSWDriveInPdos input_pdos_; /**< input_pdos_ */

  /**
   * A simple way to store the pdos output values
   */
  struct OutputPdos
  {
    std::bitset<16> control_word; /**< control_word */
    int8_t op_mode;               /**< op_mode */
    int16_t target_torque;        /**< target_torque */
    int32_t target_position;      /**< target_position */
    int32_t target_velocity;      /**< target_velocity */
  } output_pdos_;                 /**< output_pdos_ */

protected:
  void InitFun() override;

private:
  // EasyCAT slave device specific info
//  static constexpr uint8_t kDomainEntries_ = 17;
//  static constexpr uint8_t kAlias_         = 0;
//  static constexpr uint32_t kVendorID_     = 0x0000009a;
//  static constexpr uint32_t kProductCode_  = 0x00030924;

  static constexpr uint8_t kDomainInputs            = 8;
  static constexpr uint8_t kDomainOutputs           = 5;
  static constexpr uint8_t kDomainEntries           = kDomainInputs + kDomainOutputs;
  static constexpr uint8_t kAlias                   = 0;
  static constexpr uint32_t kVendorID               = 0x0000009a;
  static constexpr uint32_t kProductCode            = 0x00030924;
  static constexpr uint16_t kControlWordIdx         = 0x6040;
  static constexpr uint8_t kControlWordSubIdx       = 0x00;
  static constexpr uint16_t kHomingMethodIdx        = 0x6098;
  static constexpr uint8_t kHomingMethodSubIdx      = 0x00;
  static constexpr uint16_t kOpModeIdx              = 0x6060;
  static constexpr uint8_t kOpModeSubIdx            = 0x00;
  static constexpr uint16_t kTargetTorqueIdx        = 0x6071;
  static constexpr uint8_t kTargetTorqueSubIdx      = 0x00;
  static constexpr uint16_t kTargetPosIdx           = 0x607a;
  static constexpr uint8_t kTargetPosSubIdx         = 0x00;
  static constexpr uint16_t kTargetVelIdx           = 0x60FF;
  static constexpr uint8_t kTargetVelSubIdx         = 0x00;
  static constexpr uint16_t kStatusWordIdx          = 0x6041;
  static constexpr uint8_t kStatusWordSubIdx        = 0x00;
  static constexpr uint16_t kDisplayOpModeIdx       = 0x6061;
  static constexpr uint8_t kDisplayOpModeSubIdx     = 0x00;
  static constexpr uint16_t kPosActualValueIdx      = 0x6064;
  static constexpr uint8_t kPosActualValueSubIdx    = 0x00;
  static constexpr uint16_t kVelActualValueIdx      = 0x606C;
  static constexpr uint8_t kVelActualValueSubIdx    = 0x00;
  static constexpr uint16_t kTorqueActualValueIdx   = 0x6077;
  static constexpr uint8_t kTorqueActualValueSubIdx = 0x00;
  static constexpr uint16_t kDigInIndex             = 0x60FD;
  static constexpr uint8_t kDigInSubIndex           = 0x00;
  static constexpr uint16_t kAuxPosActualValueIdx   = 0x20A0;
  static constexpr uint8_t kAuxPosActualValueSubIdx = 0x00;
  static constexpr uint8_t kHomingOnPosMethod       = 35;
  static constexpr uint8_t kNumSupportedOperations  = 3;
  static constexpr uint8_t kOperationOffset         = 8;
  static constexpr uint16_t kAnalogIdx   = 0x2205;
  static constexpr uint8_t kAnalogSubIdx = 0x1;


  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
//  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
//    {0x6040, 0x0, 16}, /**< output PDO: Control_word */
//    {0x6060, 0x0, 8}, /**< output PDO: Mode_of_operation */
//    {0x6098, 0x0, 8}, /**< output PDO: Homing_method */
//    {0x6071, 0x0, 16}, /**< output PDO: Target_Torque */
//    {0x607a, 0x0, 32}, /**< output PDO: Target_Position */
//    {0x60ff, 0x0, 32}, /**< output PDO: Target_Velocity */
//    {0x60fe, 0x1, 32}, /**< output PDO: Digital_Outputs */
//    {0x6041, 0x0, 16}, /**< input PDO: Status_word */
//    {0x6061, 0x0, 8}, /**< input PDO: Mode_of_operation_display */
//    {0x6064, 0x0, 32}, /**< input PDO: Position_actual_value */
//    {0x606c, 0x0, 32}, /**< input PDO: Velocity_actual_value */
//    {0x6077, 0x0, 16}, /**< input PDO: Torque_actual_value */
//    {0x60fd, 0x0, 32}, /**< input PDO: Digital_Inputs */
//    {0x2205, 0x1, 16}, /**< input PDO: Analog_Input_1 */
//    {0x20a0, 0x0, 32}, /**< input PDO: Auxiliary_position_actual_value */
//    {0x6078, 0x0, 16}, /**< input PDO: Current_actual_value */
//    {0x2202, 0x1, 32}, /**< input PDO: Extended_Inputs_Value */
//  };

//  // Ethercat utilities, describing memory position of input and output PDOs
//  // stack.
//  static constexpr ec_pdo_info_t kPDOs_[2] = {
//    {0x1607, 7, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_)}, /**< Output PDOs */
//    {0x1a07, 10, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 7}, /**< Inputs PDOs */
//  };

//  // Ethercat utilities, synchronization information
//  static constexpr ec_sync_info_t kSyncs_[5] = {
//      {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
//      {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
//      {2, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 0, EC_WD_ENABLE},
//      {3, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 1, EC_WD_DISABLE},
//      {0xff}};

//  // Useful ethercat struct to store input PDOs memory offset
//  struct OffsetIn
//  {
//    unsigned int Status_word;
//    unsigned int Mode_of_operation_display;
//    unsigned int Position_actual_value;
//    unsigned int Velocity_actual_value;
//    unsigned int Torque_actual_value;
//    unsigned int Digital_Inputs;
//    unsigned int Analog_Input_1;
//    unsigned int Auxiliary_position_actual_value;
//    unsigned int Current_actual_value;
//    unsigned int Extended_Inputs_Value;
//  } offset_in_;

//  // Useful ethercat struct to store output PDOs memory offset
//  struct OffsetOut
//  {
//    unsigned int Control_word;
//    unsigned int Mode_of_operation;
//    unsigned int Homing_method;
//    unsigned int Target_Torque;
//    unsigned int Target_Position;
//    unsigned int Target_Velocity;
//    unsigned int Digital_Outputs;
//    unsigned int Extended_Outputs;
//  } offset_out_;

  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries] = {
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
    {kAnalogIdx, kAnalogSubIdx, 16},
    {kDigInIndex, kDigInSubIndex, 32},
    {kAuxPosActualValueIdx, kAuxPosActualValueSubIdx, 32}
    };

  // ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_pdo_info_t kPDOs_[2] = {
    {0x1607, 5, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 0}, /* Outputs */
    {0x1a07, 8, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 5}, /* Inputs */
  };

  static constexpr ec_sync_info_t kSyncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};

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
    unsigned int analog_input;
    unsigned int digital_inputs;
    unsigned int aux_pos_actual_value;
  } offset_in_;

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries]; // ethercat utility
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ELMO_DRIVE_H
