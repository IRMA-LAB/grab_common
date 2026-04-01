#ifndef GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ETHERCAT_XSENS_CONFIG_MDO_H
#define GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ETHERCAT_XSENS_CONFIG_MDO_H

//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project ethercat_xsens_config_mdo.prj
//     Easy Configurator XML ethercat_xsens_config_mdo.xml
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include "ethercatslave.h"
#include "grabec_types.h"

namespace grabec
{

/**
 * @brief The Ethercat_xsens_config_mdoSlave class
 */
class Ethercat_xsens_config_mdoSlave : public virtual EthercatSlave
{
public:
  /**
   * @brief Ethercat_xsens_config_mdoSlave
   * @param slave_position
   */
  Ethercat_xsens_config_mdoSlave(const uint8_t slave_position);
  ~Ethercat_xsens_config_mdoSlave() override;

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
  union CustBufferOut
  {
    uint8_t Byte[12]; /**< Raw output buffer content. */
    struct
    {
      uint8_t CMD_ID;
      uint8_t byte1;
      uint8_t byte2;
      uint8_t byte3;
      uint8_t byte4;
      uint8_t CMD_ID_check;
      uint8_t CMD_servo;
      uint8_t trajectory_type_servo;
      uint8_t speed_type_servo;
    } Cust; /**< Custom structure resembling output entries as defined in the config. */
  } BufferOut; /**< Output buffer, i.e. data received from master (read). */

  /**
   * @brief Input buffer union, i.e. data sent to master (write).
   */
  union CustBufferIn
  {
    uint8_t Byte[20]; /**< Raw input buffer content. */
    struct
    {
      float ang_eul_pitch;
      float ang_eul_roll;
      float ang_eul_yaw;
      float acc_x;
      float acc_y;
      float acc_z;
      float rate_of_turn_x;
      float rate_of_turn_y;
      float rate_of_turn_z;
      float quaternion_q1;
      float quaternion_q2;
      float quaternion_q3;
      float quaternion_q4;
      uint32_t status_word;
      uint16_t selftest_result;
      uint8_t state_servo;
      uint8_t CMD_servo_check;
      uint8_t status_byte;
      uint8_t resp_CMD_ID;
    } Cust; /**< Custom structure resembling input entries as defined in the config. */
  } BufferIn; /**< Input buffer, i.e. data sent to master (write). */

protected:
  void InitFun() override;

private:
  // EasyCAT slave device specific info
  static constexpr uint8_t kDomainEntries_ = 28;
  static constexpr uint8_t kAlias_         = 0;
  static constexpr uint32_t kVendorID_     = 0x0000079a;
  static constexpr uint32_t kProductCode_  = 0xdeafbeef;

  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
    {0x5, 0x1, 8}, /**< output PDO: CMD_ID */
    {0x5, 0x2, 8}, /**< output PDO: byte1 */
    {0x5, 0x3, 8}, /**< output PDO: byte2 */
    {0x5, 0x4, 8}, /**< output PDO: byte3 */
    {0x5, 0x5, 8}, /**< output PDO: byte4 */
    {0x5, 0x6, 8}, /**< output PDO: CMD_ID_check */
    {0x5, 0x7, 8}, /**< output PDO: CMD_servo */
    {0x5, 0x8, 8}, /**< output PDO: trajectory_type_servo */
    {0x5, 0x9, 8}, /**< output PDO: speed_type_servo */
    {0x6, 0x1, 32}, /**< input PDO: ang_eul_pitch */
    {0x6, 0x2, 32}, /**< input PDO: ang_eul_roll */
    {0x6, 0x3, 32}, /**< input PDO: ang_eul_yaw */
    {0x6, 0x4, 32}, /**< input PDO: acc_x */
    {0x6, 0x5, 32}, /**< input PDO: acc_y */
    {0x6, 0x6, 32}, /**< input PDO: acc_z */
    {0x6, 0x7, 32}, /**< input PDO: rate_of_turn_x */
    {0x6, 0x8, 32}, /**< input PDO: rate_of_turn_y */
    {0x6, 0x9, 32}, /**< input PDO: rate_of_turn_z */
    {0x6, 0x10, 32}, /**< input PDO: quaternion_q1 */
    {0x6, 0x11, 32}, /**< input PDO: quaternion_q2 */
    {0x6, 0x12, 32}, /**< input PDO: quaternion_q3 */
    {0x6, 0x13, 32}, /**< input PDO: quaternion_q4 */
    {0x6, 0x14, 32}, /**< input PDO: status_word */
    {0x6, 0x15, 16}, /**< input PDO: selftest_result */
    {0x6, 0x16, 8}, /**< input PDO: state_servo */
    {0x6, 0x17, 8}, /**< input PDO: CMD_servo_check */
    {0x6, 0x18, 8}, /**< input PDO: status_byte */
    {0x6, 0x19, 8}, /**< input PDO: resp_CMD_ID */
  };

  // Ethercat utilities, describing memory position of input and output PDOs
  // stack.
  static constexpr ec_pdo_info_t kPDOs_[2] = {
    {0x1600, 9, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_)}, /**< Output PDOs */
    {0x1a00, 19, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 9}, /**< Inputs PDOs */
  };

  // Ethercat utilities, synchronization information
  static constexpr ec_sync_info_t kSyncs_[3] = {
    {0, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 0,
       EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 1,
       EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, nullptr,
       static_cast<ec_watchdog_mode_t>(0)}};

  // Useful ethercat struct to store input PDOs memory offset
  struct OffsetIn
  {
    unsigned int ang_eul_pitch;
    unsigned int ang_eul_roll;
    unsigned int ang_eul_yaw;
    unsigned int acc_x;
    unsigned int acc_y;
    unsigned int acc_z;
    unsigned int rate_of_turn_x;
    unsigned int rate_of_turn_y;
    unsigned int rate_of_turn_z;
    unsigned int quaternion_q1;
    unsigned int quaternion_q2;
    unsigned int quaternion_q3;
    unsigned int quaternion_q4;
    unsigned int status_word;
    unsigned int selftest_result;
    unsigned int state_servo;
    unsigned int CMD_servo_check;
    unsigned int status_byte;
    unsigned int resp_CMD_ID;
  } offset_in_;

  // Useful ethercat struct to store output PDOs memory offset
  struct OffsetOut
  {
    unsigned int CMD_ID;
    unsigned int byte1;
    unsigned int byte2;
    unsigned int byte3;
    unsigned int byte4;
    unsigned int CMD_ID_check;
    unsigned int CMD_servo;
    unsigned int trajectory_type_servo;
    unsigned int speed_type_servo;
  } offset_out_;

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries_]; // ethercat utility
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_ETHERCAT_XSENS_CONFIG_MDO_H
