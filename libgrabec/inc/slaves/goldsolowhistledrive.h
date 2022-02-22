/**
 * @file goldsolowhistledrive.h
 * @author Edoardo Idà, Simone Comari
 * @date Jan 2022
 * @brief File containing _Gold Solo Whistle Drive_ slave interface to be included in the
 * GRAB ethercat library.
 */

#ifndef GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H
#define GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H

#include <bitset>
#include <iostream>
#include <sstream>

#if USE_QT
#include <QObject>
#endif

#include "StateMachine.h"

#include "ethercatslave.h"
#include "grabec_types.h"

/**
 * @brief Namespace for GRAB EtherCAT library.
 */
namespace grabec {
/**
 * @brief Gold Solo Whistle _operation modes_.
 *
 * Only few values are considered here for our purposes. For further details, click <a
 * href="https://www.elmomc.com/members/NetHelp1/Elmo.htm#!object0x6060modesofo.htm">here</a>.
 */
// clang-format off
ENUM_CLASS(GoldSoloWhistleOperationModes,
           NONE                  = -1,
           PROFILE_POSITION      = 1,
           VELOCITY_MODE         = 2,
           PROFILE_VELOCITY      = 3,
           TORQUE_PROFILE        = 4,
           HOMING                = 6,
           INTERPOLATED_POSITION = 7,
           CYCLIC_POSITION       = 8,
           CYCLIC_VELOCITY       = 9,
           CYCLIC_TORQUE         = 10)
// clang-format on

/**
 * A simple way to store the pdos input values
 */
struct GSWDriveInPdos
{
  std::bitset<16> status_word; /**< status_word */
  int8_t display_op_mode;      /**< display_op_mode */
  int32_t pos_actual_value;    /**< pos_actual_value */
  int32_t vel_actual_value;    /**< vel_actual_value */
  int16_t torque_actual_value; /**< torque_actual_value */
  int16_t analog_input;        /**< analog_input */
  uint digital_inputs;         /**< digital_inputs */
  int aux_pos_actual_value;    /**< aux_pos_actual_value */
};

/**
 * @brief The GoldSoloWhistleDriveData class.
 *
 * This is the data type to be passed when transitioning to OPERATION ENABLED state
 * according to our implementation of drive interface's state machine.
 * For further details about state machine, click <a
 * href="https://www.codeproject.com/Articles/1087619/State-Machine-Design-in-Cplusplus">here</a>.
 */
class GoldSoloWhistleDriveData: public EventData
{
 public:
  /**
   * @brief Full constructor.
   * @param[in] _op_mode The desired operation mode of the drive. See
   * GoldSoloWhistleOperationModes for valid accounted entries.
   * @param[in] _value The target set point for the desired operation mode (position,
   * velocity or torque).
   * @attention Torque setpoint is automaticallly casted to INT16, so values needs to be
   * in range [–32,768, 32,767].
   */
  GoldSoloWhistleDriveData(const int8_t _op_mode, const int32_t _value = 0);
  /**
   * @brief Constructor from drive current status (for safe switch).
   * @param[in] _op_mode The desired operation mode of the drive. See
   * GoldSoloWhistleOperationModes for valid accounted entries.
   * @param[in] input_pdos The current set of PDOs, from which to extract the value to be
   * set according to the desired operation mode.
   * @param[in] verbose If _true_ display the data content.
   */
  GoldSoloWhistleDriveData(const int8_t _op_mode, const GSWDriveInPdos& input_pdos,
                           const bool verbose = false);

  int8_t op_mode =
    GoldSoloWhistleOperationModes::NONE; /**< The desired operation mode of the drive. */
  int32_t value = 0; /**< The target set point for the desired operation mode. */
};

/**
 * @brief The Gold Solo Whistle Drive class.
 *
 * This class provides an interface to the physical drive threated here as an ethercat
 * slave. As its physical component, this class includes a state machine, which represents
 * the status of the physical drive itself. The user can trigger a state transition by
 * means of @ref ExternalEvents. The basic ones resemble the external events taken by the
 * physical drive and can be found <a
 * href="https://www.elmomc.com/members/NetHelp1/Elmo.htm#!object0x6040controlw.htm">here</a>.
 * On top of those, setpoint changing events can be used to modify the target position,
 * velocity or torque in OPERATION_ENABLED mode.
 * @note The events only request a state transition carried on by WriteOutputs(), but the
 * actual state depends of the current state of the physical drive, obtained by
 * ReadInputs(). Hence, it is recommended to always check the transition effectively
 * occured before requesting a new one. Use GetCurrentState() inherited function for this
 * purpose.
 * @todo implement:
 * - fast stop during operation
 * - reaction to a specific fault
 */
class GoldSoloWhistleDrive:
#if USE_QT
  public QObject,
#endif
  public virtual EthercatSlave,
  public StateMachine
{
#if USE_QT
  Q_OBJECT
#endif

 public:
  /**
   * @brief Gold Solo Whistle Drive _states_ as in physical drive documentation.
   *
   * For further details, click
   * <a href="https://www.elmomc.com/members/NetHelp1/Elmo.htm#!objects.htm">here</a>.
   */
  enum States : BYTE
  {
    ST_START,
    ST_NOT_READY_TO_SWITCH_ON,
    ST_SWITCH_ON_DISABLED,
    ST_READY_TO_SWITCH_ON,
    ST_SWITCHED_ON,
    ST_OPERATION_ENABLED,
    ST_QUICK_STOP_ACTIVE,
    ST_FAULT_REACTION_ACTIVE,
    ST_FAULT,
    ST_MAX_STATES
  };

 public:
  /**
   * @brief Constructor.
   * @param[in] id Drive ID.
   * @param[in] slave_position Slave position in ethercat chain.
   * @param[in] parent The Qt parent, in this case the actuator it belongs to.
   */
  GoldSoloWhistleDrive(const id_t id, const uint8_t slave_position
#if USE_QT
                       ,
                       QObject* parent = nullptr
#endif
  );

  /**
   * @brief Get latest known physical drive state.
   * @param[in] status_word Drive status bit word as read from the corresponding PDO.
   * @return Latest known physical drive state.
   */
  static States getDriveState(const std::bitset<16>& status_word);
  /**
   * @brief Get latest known physical drive state.
   * @param[in] status_word Drive status bit word as read from the corresponding PDO.
   * @return Latest known physical drive state.
   */
  static std::string getDriveStateStr(const std::bitset<16>& status_word);
  /**
   * @brief GetID
   * @return
   */
  id_t getID() const { return id_; }
  /**
   * @brief Get actual drive position (aka counts).
   * @return Actual drive position (aka counts).
   */
  int32_t getPosition() const { return input_pdos_.pos_actual_value; }
  /**
   * @brief Get actual drive velocity.
   * @return Actual drive velocity in user-defined units.
   */
  int32_t getVelocity() const { return input_pdos_.vel_actual_value; }
  /**
   * @brief Get actual drive torque.
   * @return Actual drive torque in user-defined units.
   */
  int16_t getTorque() const { return input_pdos_.torque_actual_value; }
  /**
   * @brief Get actual drive auxiliary position (aka counts).
   *
   * This field can be used by external sensors connected to the drive, for instance an
   * additional encoder.
   * @return Actual drive auxiliary position (aka counts).
   */
  int getAuxPosition() const { return input_pdos_.aux_pos_actual_value; }
  /**
   * @brief GetAnalogInput
   * @return
   */
  int16_t getAnalogInput() const { return input_pdos_.analog_input; }
  /**
   * @brief Get actual drive operational mode {_position, velocity, torque, none_}.
   * @return Actual drive operational mode.
   * @see GoldSoloWhistleOperationModes
   */
  int8_t getOpMode() const { return input_pdos_.display_op_mode; }
  /**
   * @brief Get actual drive status, i.e. the latest values of its input PDOs.
   * @return Actual drive status, i.e. the latest values of its input PDOs.
   */
  GSWDriveInPdos getDriveStatus() const { return input_pdos_; }

  /**
   * @defgroup ExternalEvents GoldSoloWhistle Drive External Events
   * The external events which user can call to trigger a state transition. For furter
   * details
   * click <a
   * href="https://www.elmomc.com/members/NetHelp1/Elmo.htm#!object0x6040controlw.htm">here</a>.
   * @{
   */
  //------- External events resembling the ones internal to physical drive -----------//

  /**
   * @brief Disable Voltage external event.
   *
   * Triggers following transitions:
   * - READY TO SWITCH ON --> SWITCH ON DISABLED
   * - OPERATION ENABLED --> SWITCH ON DISABLED
   * - SWITCHED ON --> SWITCH ON DISABLED
   * - QUICK STOP ACTIVE --> SWITCH ON DISABLED
   */
  void disableVoltage();
  /**
   * @brief _Shutdown_ external event.
   *
   * Triggers following transitions:
   * - SWITCH ON DISABLED --> READY TO SWITCH ON
   * - SWITCHED ON --> READY TO SWITCH ON
   * - OPERATION ENABLED --> READY TO SWITCH ON
   */
  void shutdown();
  /**
   * @brief _Switch ON_ external event.
   *
   * Triggers following transition:
   * - READY TO SWITCH ON --> SWITCHED ON
   */
  void switchOn();
  /**
   * @brief _Enable Operation_ external event.
   *
   * Triggers following transitions:
   * - SWITCHED ON --> OPERATION ENABLED
   * - QUICK STOP ACTIVE --> OPERATION ENABLED
   */
  void enableOperation();
  /**
   * @brief Disable Operation external event.
   *
   * Triggers following transition:
   * - OPERATION ENABLED --> SWITCHED ON
   */
  void disableOperation();
  /**
   * @brief Quick Stop external event.
   *
   * Triggers following transitions:
   * - READY TO SWITCH ON --> SWITCH ON DISABLED
   * - SWITCHED ON --> SWITCH ON DISABLED
   * - OPERATION ENABLED --> QUICK STOP ACTIVE
   */
  void quickStop();
  /**
   * @brief Fault Reset external event.
   *
   * Triggers following transition:
   * - FAULT --> SWITCH ON DISABLED
   */
  void faultReset();

  //-- Additional external events taken by this state machine when it's operational --//

  /**
   * @brief Change Position external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_POSITION and sets target position to @c target_position.
   * @param[in] target_position New position setpoint in user-defined units.
   */
  void changePosition(const int32_t target_position, const bool verbose = false);
  /**
   * @brief ChangeDeltaPosition
   * @param[in] delta_position
   */
  void changeDeltaPosition(const int32_t delta_position);
  /**
   * @brief Change Velocity external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_VELOCITY and sets target velocity to @c target_velocity.
   * @param[in] target_velocity New velocity setpoint in user-defined units.
   */
  void changeVelocity(const int32_t target_velocity, const bool verbose = false);
  /**
   * @brief ChangeDeltaVelocity
   * @param[in] delta_velocity
   */
  void changeDeltaVelocity(const int32_t delta_velocity);
  /**
   * @brief Change Torque external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * CYCLIC_TORQUE and sets target torque to @c target_torque.
   * @param[in] target_torque New torque setpoint in user-defined units.
   */
  void changeTorque(const int16_t target_torque, const bool verbose = false);
  /**
   * @brief ChangeDeltaTorque
   * @param[in] delta_torque
   */
  void changeDeltaTorque(const int16_t delta_torque);
  /**
   * @brief Change Operation Mode external event.
   *
   * If drive is in in OPERATION ENABLED state, it changes _operation mode_ to
   * @c target_op_mode and sets corresponding target to its current value.
   * @param[in] target_op_mode New _operation mode_.
   */
  void changeOpMode(const int8_t target_op_mode, const bool verbose = false);
  /**
   * @brief SetTargetDefaults external event.
   *
   * If drive is in in OPERATION ENABLED state, according to current _operation mode_, it
   * sets corresponding target value to its current value.
   */
  void setTargetDefaults(const bool verbose = false);
  /** @} */

  //------- Methods called within the real-time cycle --------------------------------//
  /**
   * @brief Read input PDOs.
   */
  void readInputs() override final;
  /**
   * @brief Write output PDOs.
   */
  void writeOutputs() override final;
  /**
   * @brief Function called before shutting down the slave.
   */
  void safeExit() override final;
  /**
   * @brief Check if slave is ready to be shut down safely.
   * @return _True_ if slave is ready, _false_ otherwise.
   */
  bool isReadyToShutDown() const override final;

#if USE_QT
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
#endif

 protected:
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

  States drive_state_; /**< physical drive state */

  int32_t prev_pos_target_    = 0; /**< previous motor position target in counts. */
  int32_t prev_vel_target_    = 0; /**< previous motor velocity target in counts/s. */
  int16_t prev_torque_target_ = 0; /**< previous motor torque target in nominal points. */

  /**
   * @brief InitFun
   */
  void initFun() override;

 private:
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

  static constexpr uint16_t kAnalogIdx   = 0x2205;
  static constexpr uint8_t kAnalogSubIdx = 0x1;

  static constexpr uint16_t kAuxPosActualValueIdx   = 0x20A0;
  static constexpr uint8_t kAuxPosActualValueSubIdx = 0x00;
  static constexpr uint8_t kHomingOnPosMethod       = 35;
  static constexpr uint8_t kNumSupportedOperations  = 3;
  static constexpr uint8_t kOperationOffset         = 8;

  // ethercat utilities, can be retrieved in the xml config file provided by the vendor
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
    {kAuxPosActualValueIdx, kAuxPosActualValueSubIdx, 32}};

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

  // clang-format off
  ENUM_CLASS(StatusBit,
             READY_TO_SWITCH_ON = 0,
             SWITCHED_ON        = 1,
             OPERATION_ENABLED  = 2,
             FAULT              = 3,
             QUICK_STOP         = 5,
             SWITCH_ON_DISABLED = 6)

  ENUM_CLASS(ControlBit,
             SWITCH_ON        = 0,
             ENABLE_VOLTAGE   = 1,
             QUICK_STOP       = 2,
             ENABLE_OPERATION = 3,
             FAULT            = 7)
  // clang-format on

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries]; // ethercat utilities

  void setChange(const GoldSoloWhistleDriveData* data);

  RetVal sdoRequests(ec_slave_config_t* config_ptr) override final;

  inline void printCommand(const char* cmd) const;
  inline void printTarget(const int target) const;
  inline void printTarget(const GoldSoloWhistleDriveData& data) const;

  void ecPrintCb(const std::string& msg, const char color = 'w') const override;

 private:
  //--------- State machine ----------------------------------------------------------//

  // clang-format off
  static constexpr char* kStatesStr_[] = {
    const_cast<char*>("START"),
    const_cast<char*>("NOT_READY_TO_SWITCH_ON"),
    const_cast<char*>("SWITCH_ON_DISABLED"),
    const_cast<char*>("READY_TO_SWITCH_ON"),
    const_cast<char*>("SWITCHED_ON"),
    const_cast<char*>("OPERATION_ENABLED"),
    const_cast<char*>("QUICK_STOP_ACTIVE"),
    const_cast<char*>("FAULT_REACTION_ACTIVE"),
    const_cast<char*>("FAULT"),
    const_cast<char*>("MAX_STATE")};
  // clang-format on

  States prev_state_;

  // Define the state machine state functions with event data type
  STATE_DECLARE(GoldSoloWhistleDrive, Start, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, NotReadyToSwitchOn, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, SwitchOnDisabled, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, ReadyToSwitchOn, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, SwitchedOn, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, OperationEnabled, GoldSoloWhistleDriveData)
  STATE_DECLARE(GoldSoloWhistleDrive, QuickStopActive, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, FaultReactionActive, NoEventData)
  STATE_DECLARE(GoldSoloWhistleDrive, Fault, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP
  // clang-format off
    STATE_MAP_ENTRY({&Start})
    STATE_MAP_ENTRY({&NotReadyToSwitchOn})
    STATE_MAP_ENTRY({&SwitchOnDisabled})
    STATE_MAP_ENTRY({&ReadyToSwitchOn})
    STATE_MAP_ENTRY({&SwitchedOn})
    STATE_MAP_ENTRY({&OperationEnabled})
    STATE_MAP_ENTRY({&QuickStopActive})
    STATE_MAP_ENTRY({&FaultReactionActive})
    STATE_MAP_ENTRY({&Fault})
  // clang-format on
  END_STATE_MAP

  void printStateTransition(const States current_state, const States new_state) const;
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_GOLDSOLOWHISTLEDRIVE_H
