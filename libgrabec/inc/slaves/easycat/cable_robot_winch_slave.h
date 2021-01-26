/**
 * @file Cable_robot_winch_slave.h
 * @author Simone Comari
 * @date 26 Jan 2021
 * @brief File containing _Cable Robot Winch_ slave interface to be included in the GRAB
 * ethercat library.
 */

#ifndef GRABCOMMON_LIBGRABEC_EASYCATSLAVE_CABLE_ROBOT_WINCH_H
#define GRABCOMMON_LIBGRABEC_EASYCATSLAVE_CABLE_ROBOT_WINCH_H

#include <bitset>

#include <QObject>

#include "StateMachine.h"

#include "ethercatslave.h"
#include "grabec_types.h"

namespace grabec {
/**
 * @brief Cable Robot Winch _operation modes_.
 */
// clang-format off
ENUM_CLASS(CableRobotWinchOperationModes,
           NONE            = -1,
           CYCLIC_POSITION = 0,
           CYCLIC_VELOCITY = 1,
           CYCLIC_TORQUE   = 2)
// clang-format on

/**
 * @brief Input buffer union, i.e. data sent to master (write).
 */
union CRWSlaveInPdos
{
  uint8_t Byte[8]; /**< Raw input buffer content. */
  struct
  {
    int32_t actual_position;
    int32_t actual_speed;
    uint16_t status_word;
    uint16_t loadcell_value;
    int16_t actual_torque;
    int16_t actual_position_aux;
  } Cust; /**< Custom structure resembling input entries as defined in the config. */
};

/**
 * @brief The CableRobotWinchData class.
 *
 * This is the data type to be passed when transitioning to OPERATION ENABLED state
 * according to our implementation of drive interface's state machine.
 * For further details about state machine, click <a
 * href="https://www.codeproject.com/Articles/1087619/State-Machine-Design-in-Cplusplus">here</a>.
 */
class CableRobotWinchData: public EventData
{
 public:
  /**
   * @brief Full constructor.
   * @param[in] _op_mode The desired operation mode of the drive. See
   * GoldSoloWhistleOperationModes for valid accounted entries.
   * @param[in] _value The target set point for the desired operation mode (position,
   * velocity or torque).
   * @param[in] verbose If _true_ display the data content.
   */
  CableRobotWinchData(const int8_t _op_mode, const int32_t _value = 0,
                      const bool verbose = false);
  /**
   * @brief Full constructor.
   * @param[in] _op_mode The desired operation mode of the drive. See
   * GoldSoloWhistleOperationModes for valid accounted entries.
   * @param[in] input_pdos The current set of PDOs, from which to extract the value to be
   * set according to the desired operation mode. It basically maintain the previous
   * value.
   * @param[in] verbose If _true_ display the data content.
   */
  CableRobotWinchData(const int8_t _op_mode, const CRWSlaveInPdos& input_pdos,
                      const bool verbose = false);
  /**
   * @brief Constructor from drive current status (for safe switch).
   * @param[in] input_pdos The current set of PDOs, from which to extract the value to be
   * set according to the desired operation mode. It basically maintain the previous
   * value.
   * @param[in] verbose If _true_ display the data content.
   */
  CableRobotWinchData(const CRWSlaveInPdos& input_pdos, const bool verbose = false);

  int8_t op_mode =
    CableRobotWinchOperationModes::NONE; /**< The desired operation mode of the drive. */
  int32_t value = 0; /**< The target set point for the desired operation mode. */
};

/**
 * @brief The Cable_robot_winchSlave class.
 */
class CableRobotWinchSlave: public QObject,
                            public virtual EthercatSlave,
                            public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief Cable robot winch _states_.
   */
  enum States : BYTE
  {
    ST_IDLE,
    ST_OPERATIONAL,
    ST_ERROR,
    ST_MAX_STATES
  };

 public:
  /**
   * @brief Constructor.
   * @param[in] id Drive ID.
   * @param[in] slave_position Slave position in ethercat chain.
   */
  CableRobotWinchSlave(const id_t id, const uint8_t slave_position,
                       QObject* parent = nullptr);

  /**
   * @brief Get latest known physical drive state.
   * @param[in] status_word Drive status bit word as read from the corresponding PDO.
   * @return Latest known physical drive state.
   */
  static States GetDriveState(const uint16_t _status_word);
  /**
   * @brief Get latest known physical drive state.
   * @param[in] status_word Drive status bit word as read from the corresponding PDO.
   * @return Latest known physical drive state.
   */
  static std::string GetDriveStateStr(const uint16_t status_word);
  /**
   * @brief Get latest known drive operational mode (when operational).
   * @param[in] status_word Drive status bit word as read from the corresponding PDO.
   * @return Latest known drive operational mode.
   */
  static CableRobotWinchOperationModes GetDriveOpMode(const uint16_t status_word);
  /**
   * @brief Get actual drive position (aka counts).
   * @return Actual drive position (aka counts).
   */
  int32_t GetPosition() const { return BufferIn.Cust.actual_position; }
  /**
   * @brief Get actual drive velocity.
   * @return Actual drive velocity in user-defined units.
   */
  int32_t GetVelocity() const { return BufferIn.Cust.actual_speed; }
  /**
   * @brief Get actual drive torque.
   * @return Actual drive torque in user-defined units.
   */
  int16_t GetTorque() const { return BufferIn.Cust.actual_torque; }
  /**
   * @brief Get actual drive auxiliary position (aka counts).
   *
   * This field can be used by external sensors connected to the drive, for instance an
   * additional encoder.
   * @return Actual drive auxiliary position (aka counts).
   */
  int GetAuxPosition() const { return BufferIn.Cust.actual_position_aux; }
  /**
   * @brief Get actual drive operational mode {_position, velocity, torque, none_}.
   * @return Actual drive operational mode.
   * @see CableRobotWinchOperationModes
   */
  CableRobotWinchOperationModes GetOpMode() const;
  /**
   * @brief Get actual drive status, i.e. the latest values of its input PDOs.
   * @return Actual drive status, i.e. the latest values of its input PDOs.
   */
  CRWSlaveInPdos GetDriveStatus() const { return BufferIn; }

  //------- External events resembling the ones internal to physical drive -----------//

  /**
   * @brief _Enable Operation_ external event.
   *
   * Triggers following transition:
   * - IDLE --> OPERATIONAL
   */
  void EnableOperation();
  /**
   * @brief Disable Operation external event.
   *
   * Triggers following transition:
   * - OPERATIONAL --> IDLE
   */
  void DisableOperation();
  /**
   * @brief Fault Reset external event.
   *
   * Triggers following transition:
   * - ERROR --> IDLE ?
   */
  void FaultReset();

  //-- Additional external events taken by this state machine when it's operational --//

  /**
   * @brief Change Position external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_POSITION and sets target position to @c target_position.
   * @param[in] target_position New position setpoint in user-defined units.
   */
  void ChangePosition(const int32_t target_position);
  /**
   * @brief Change Velocity external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_VELOCITY and sets target velocity to @c target_velocity.
   * @param[in] target_velocity New velocity setpoint in user-defined units.
   */
  void ChangeVelocity(const int32_t target_velocity);
  /**
   * @brief Change Torque external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_TORQUE and sets target torque to @c target_torque.
   * @param[in] target_torque New torque setpoint in user-defined units.
   */
  void ChangeTorque(const int16_t target_torque);
  /**
   * @brief Change Operation Mode external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * @c target_op_mode and sets corresponding target to its current value.
   * @param[in] target_op_mode New _operation mode_.
   */
  void ChangeOpMode(const int8_t target_op_mode);
  /**
   * @brief SetTargetDefaults external event.
   *
   * If drive is in in OPERATION ENABLED state, according to current _operation mode_, it
   * sets corresponding target value to its current value.
   */
  void SetTargetDefaults();

  //------- Methods called within the real-time cycle --------------------------------//

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

 signals:
  /**
   * @brief Drive faulted notice.
   */
  void driveFaulted() const;
  /**
   * @brief Emit a message to be logged.
   */
  void logMessage(const QString&) const;
  /**
   * @brief Emit a message to be printed.
   */
  void printMessage(const QString&) const;

 private:
  //--------- EtherCAT ----------------------------------------------------------//

  /**
   * @brief Output buffer union, i.e. data received from master (read).
   */
  union CustBufferOut
  {
    uint8_t Byte[4]; /**< Raw output buffer content. */
    struct
    {
      int32_t target_position;
      int32_t target_speed;
      uint16_t control_word;
      int16_t target_torque;
    } Cust; /**< Custom structure resembling output entries as defined in the config. */
  } BufferOut; /**< Output buffer, i.e. data received from master (read). */

  CRWSlaveInPdos BufferIn; /**< Input buffer, i.e. data sent to master (write). */

  // EasyCAT slave device specific info
  static constexpr uint8_t kDomainEntries_ = 10;
  static constexpr uint8_t kAlias_         = 0;
  static constexpr uint32_t kVendorID_     = 0x0000079a;
  static constexpr uint32_t kProductCode_  = 0xdeadbeef;

  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
    {0x5, 0x1, 32}, /**< output PDO: target_position */
    {0x5, 0x2, 32}, /**< output PDO: target_speed */
    {0x5, 0x3, 16}, /**< output PDO: control_word */
    {0x5, 0x4, 16}, /**< output PDO: target_torque */
    {0x6, 0x1, 32}, /**< input PDO: actual_position */
    {0x6, 0x2, 32}, /**< input PDO: actual_speed */
    {0x6, 0x3, 16}, /**< input PDO: status_word */
    {0x6, 0x4, 16}, /**< input PDO: loadcell_value */
    {0x6, 0x5, 16}, /**< input PDO: actual_torque */
    {0x6, 0x6, 16}, /**< input PDO: actual_position_aux */
  };

  // Ethercat utilities, describing memory position of input and output PDOs
  // stack.
  static constexpr ec_pdo_info_t kPDOs_[2] = {
    {0x1600, 4, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_)},     /**< Output PDOs */
    {0x1a00, 6, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 4}, /**< Inputs PDOs */
  };

  // Ethercat utilities, synchronization information
  static constexpr ec_sync_info_t kSyncs_[3] = {
    {0, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 1, EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, nullptr,
     static_cast<ec_watchdog_mode_t>(0)}};

  // Useful ethercat struct to store input PDOs memory offset
  struct OffsetIn
  {
    unsigned int actual_position;
    unsigned int actual_speed;
    unsigned int status_word;
    unsigned int loadcell_value;
    unsigned int actual_torque;
    unsigned int actual_position_aux;
  } offset_in_;

  // Useful ethercat struct to store output PDOs memory offset
  struct OffsetOut
  {
    unsigned int target_position;
    unsigned int target_speed;
    unsigned int control_word;
    unsigned int target_torque;
  } offset_out_;

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries_]; // ethercat utility
  States drive_state_;                                   // drive physical actual state

  void InitFun() override;
  void EcPrintCb(const std::string& msg, const char color = 'w') const override;

 private:
  //--------- State machine ----------------------------------------------------------//

  // clang-format off
  static constexpr char* kStatesStr_[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("OPERATIONAL"),
    const_cast<char*>("ERROR"),
    const_cast<char*>("MAX_STATE")};
  // clang-format on

  States prev_state_;

  // Define the state machine state functions with event data type
  STATE_DECLARE(CableRobotWinchSlave, Idle, NoEventData)
  STATE_DECLARE(CableRobotWinchSlave, Operational, CableRobotWinchData)
  STATE_DECLARE(CableRobotWinchSlave, Error, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP
  // clang-format off
    STATE_MAP_ENTRY({&Idle})
    STATE_MAP_ENTRY({&Operational})
    STATE_MAP_ENTRY({&Error})
  // clang-format on
  END_STATE_MAP

  void PrintStateTransition(const States current_state, const States new_state) const;

 private:
  /**
   * @brief Shared bit-map between status and control word.
   */
  enum StatusBit : uint8_t
  {
    IDLE_STATE,
    OPERATIONAL_STATE,
    ERROR_STATE,
    POSITION_CONTROL,
    SPEED_CONTROL,
    TORQUE_CONTROL,
    ERROR_RESET,
    TARGET_REACHED
  };

  int32_t prev_pos_target_    = 0; /**< previous motor position target in counts. */
  int32_t prev_vel_target_    = 0; /**< previous motor velocity target in counts/s. */
  int16_t prev_torque_target_ = 0; /**< previous motor torque target in nominal points. */

  void SetChange(const CableRobotWinchData* data);

  inline void PrintCommand(const char* cmd) const;
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_CABLE_ROBOT_WINCH_H
