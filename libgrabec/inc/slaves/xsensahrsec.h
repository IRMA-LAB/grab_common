#ifndef XSENSAHRSEC_H
#define XSENSAHRSEC_H

//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project ethercat_xsens_config_mdo.prj				 //
//     Easy Configurator XML ethercat_xsens_config_mdo.xml					 //
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
 * @brief The xsensahrsec class
 */
class xsensahrsec : public virtual EthercatSlave
{
public:
  /**
   * @brief xsensahrsec
   * @param slave_position
   */
  xsensahrsec(const uint8_t slave_position);
  ~xsensahrsec() override;

  /**
    * @brief Function to retrieve Roll measure
   */
  float getRoll() const {return BufferIn.Cust.ang_eul_roll; };
  /**
   * @brief Function to retrieve Pitch measure
   */
  float getPitch() const {return BufferIn.Cust.ang_eul_pitch; };
  /**
   * @brief Function to retrieve Yaw measure
   */
  float getYaw() const {return BufferIn.Cust.ang_eul_yaw; };
  /**
   * @brief Function to retrieve AccX measure
   */
  float getAccX() const {return BufferIn.Cust.acc_x; };
  /**
   * @brief Function to retrieve AccY measure
   */
  float getAccY() const {return BufferIn.Cust.acc_y; };
  /**
   * @brief Function to retrieve AccZ measure
   */
  float getAccZ() const {return BufferIn.Cust.acc_z; };
  /**
   * @brief Function to retrieve AngVelX measure
   */
  float getAngVelX() const {return BufferIn.Cust.rate_of_turn_x; };
  /**
   * @brief Function to retrieve AngVelY measure
   */
  float getAngVelY() const {return BufferIn.Cust.rate_of_turn_y; };
  /**
   * @brief Function to retrieve AngVelZ measure
   */
    float getAngVelZ() const {return BufferIn.Cust.rate_of_turn_z; };
  /**
  * @brief Function to enable data transfer from the AHRS
  */
  bool SendData();

  /**
  * @brief Function to config the AHRS
  */
  bool GoToConfig();

  /**
  * @brief External transition function to Measurement_State
  */
  bool GoToMeasurement();

  /**
  * @brief External reset of the AHRS, WARN: the output data is set to default
  */
  bool Reset();

  /**
  * @brief Function to set the filter to be used, WARN: use after GoToConfig() function
  */
  bool FilterSelection(uint8_t filter, uint8_t bias);

  /**
  * @brief Function to execute a selftest
  */
  bool RunSelfTest();

  /**
  * @brief Gives the RotLocal rotation matrix as output in quaternions.
  * RotLocal is the rotation matrix between the pre-defined inertial reference frame
  * (ENU) and the desired one
  */
  bool AlignmentRotLocal();

  /**
  * @brief Gives the RotSensor rotation matrix as output in quaternions.
  * RotSensor is the rotation matrix between the AHRS frame and the desired one
  * (usually EE frame)
  */
  bool AlignmentRotSensor();

  /**
  * @brief Function to modify the inertial and mobile reference frames through resets
  * of inclination, heading and aligment
  */
  bool ResetOrientation(uint8_t reset_mode);

  /**
  * @brief Function to modify the inertial and mobile reference frames through resets
  * of inclination, heading and aligment. Additionally to the previous command, it
  * keeps the frame modifications after the reset.
  */
  bool ResetStoreOrientation(uint8_t reset_mode);

  /**
  * @brief Function to perform a manual estimate of the gyroscope bias. No rotation is updated.
  */
  bool NoRotation(uint16_t bias_compute_time);

  /**
  * @brief Function to give a null input to the inclinometer, nothing change.
  */
  bool Null();

  /**
  * @brief Function to check if the AHRS is in error state.
  */
  bool CheckErrorAHRS();

  /**
   * @brief Function to specify what to read.
   */
  void readInputs() override final;

  /**
   * @brief Function to specify what to write.
   */
  void writeOutputs() override final;


  /**
   * @brief Function to set motor command
   */
  void setCmdServo(uint8_t cmd_value)
  {BufferOut.Cust.CMD_servo=cmd_value;}
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
    uint8_t Byte[62]; /**< Raw input buffer content. */
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

 enum ahrs_filters {
  responsive = 0x01,
  robust = 0x02,
  general = 0x03
  };

  enum ahrs_bias {
  north_reference = 0x01,
  fixed_mag_ref = 0x02,
  vru = 0x03,
  vruahrs = 0x04
  };

  enum ahrs_resetmode {
  heading = 0x01,
  object_inclination = 0x03,
  aligment = 0x04,
  default_heading = 0x05,
  default_inclination = 0x06,
  default_aligment = 0x07
  };

private:
  // EasyCAT slave device specific info
  static constexpr uint16_t kDomainEntries_ = 28;
  static constexpr uint8_t kAlias_         = 0;
  static constexpr uint32_t kVendorID_     = 0x0000079a;
  static constexpr uint32_t kProductCode_  = 0xdeafbeef;

  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
    {0x0005, 0x01, 8}, /**< output PDO: CMD_ID */
    {0x0005, 0x02, 8}, /**< output PDO: byte1 */
    {0x0005, 0x03, 8}, /**< output PDO: byte2 */
    {0x0005, 0x04, 8}, /**< output PDO: byte3 */
    {0x0005, 0x05, 8}, /**< output PDO: byte4 */
    {0x0005, 0x06, 8}, /**< output PDO: CMD_ID_check */
    {0x0005, 0x07, 8}, /**< output PDO: CMD_servo */
    {0x0005, 0x08, 8}, /**< output PDO: trajectory_type_servo */
    {0x0005, 0x09, 8}, /**< output PDO: speed_type_servo */
    {0x0006, 0x01, 32}, /**< input PDO: ang_eul_pitch */
    {0x0006, 0x02, 32}, /**< input PDO: ang_eul_roll */
    {0x0006, 0x03, 32}, /**< input PDO: ang_eul_yaw */
    {0x0006, 0x04, 32}, /**< input PDO: acc_x */
    {0x0006, 0x05, 32}, /**< input PDO: acc_y */
    {0x0006, 0x06, 32}, /**< input PDO: acc_z */
    {0x0006, 0x07, 32}, /**< input PDO: rate_of_turn_x */
    {0x0006, 0x08, 32}, /**< input PDO: rate_of_turn_y */
    {0x0006, 0x09, 32}, /**< input PDO: rate_of_turn_z */
    {0x0006, 0x0a, 32}, /**< input PDO: quaternion_q1 */
    {0x0006, 0x0b, 32}, /**< input PDO: quaternion_q2 */
    {0x0006, 0x0c, 32}, /**< input PDO: quaternion_q3 */
    {0x0006, 0x0d, 32}, /**< input PDO: quaternion_q4 */
    {0x0006, 0x0e, 32}, /**< input PDO: status_word */
    {0x0006, 0x0f, 16}, /**< input PDO: selftest_result */
    {0x0006, 0x10, 8}, /**< input PDO: state_servo */
    {0x0006, 0x11, 8}, /**< input PDO: CMD_servo_check */
    {0x0006, 0x12, 8}, /**< input PDO: status_byte */
    {0x0006, 0x13, 8}, /**< input PDO: resp_CMD_ID */
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

  sync_dc_t xsens_sync_dc_ = {true, 0x300, 2000000,2000200000,0,0};

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries_ + 1]; // ethercat utility
};

} // end namespace grabec

#endif // XSENSAHRSEC_H
