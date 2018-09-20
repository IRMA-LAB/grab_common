#ifndef GRABCOMMON_LIBGRABEC_EASYCATSLAVE_H
#define GRABCOMMON_LIBGRABEC_EASYCATSLAVE_H

#include "ethercatslave.h"
#include "types.h"
#include "state_machine/inc/StateMachine.h"

#define METHOD 0

namespace grabec
{

/**
 * @brief The EasyCatSlave class
 * @todo understand transition condition UPDATE --> IDLE
 */
class EasyCatSlave : public EthercatSlave, public StateMachine
{
public:
  /**
   * @brief EasyCatSlave
   * @param slave_position
   */
  EasyCatSlave(const uint8_t slave_position);
  ~EasyCatSlave();

#if METHOD
  /**
   * @brief SetUpdateFlag
   * @param value
   */
  void SetUpdateFlag(bool value) { can_update_ = value; }
#else
  /**
   * @brief Start
   */
  void Start();
#endif

  /**
   * @brief Slave's main function to be cycled.
   */
  virtual void DoWork() final;

  /**
   * @brief Function to specify what to read.
   */
  virtual void ReadInputs() final;

  /**
   * @brief Function to specify what to write.
   */
  virtual void WriteOutputs() final;

private:
  // Easycat Slave specific info
  static constexpr uint8_t kDomainEntries = 6;
  static constexpr uint8_t kAlias = 0;
  static constexpr uint32_t kVendorID = 0x0000079a;
  static constexpr uint32_t kProductCode = 0x00defede;

  // Ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_pdo_entry_info_t kPdoEntries[6] = {
    {0x0005, 0x01, 8}, // Byte0
    {0x0005, 0x02, 8}, // Byte1
    {0x0005, 0x03, 8}, // Byte2
    {0x0006, 0x01, 8}, // Byte0
    {0x0006, 0x02, 8}, // Byte1
    {0x0006, 0x03, 8}, // Byte2
  };

  // Ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_pdo_info_t kPDOs[2] = {
    {0x1600, 3, const_cast<ec_pdo_entry_info_t*>(kPdoEntries) + 0}, /* Outputs */
    {0x1a00, 3, const_cast<ec_pdo_entry_info_t*>(kPdoEntries) + 3}, /* Inputs */
  };

  // Ethercat utilities, can be retrieved in the xml config file provided by the vendor
  static constexpr ec_sync_info_t kSyncs[3] = {
    {0, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs) + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs) + 1, EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, NULL, static_cast<ec_watchdog_mode_t>(0)}};

  // A simple way to store the pdos input values
  struct InputPDOs
  {
    uint8_t slave_state;
    uint8_t num_calls;
    uint8_t cycle_counter;
  } input_pdos_;

  // A simple way to store the pdos output values
  struct OutputPDOs
  {
    uint8_t slave_status;
    uint8_t control_word;
    uint8_t led_frequency;
  } output_pdos_;

  // Useful ethercat struct
  struct OffsetIn
  {
    unsigned int slave_state;
    unsigned int num_calls;
    unsigned int cycle_counter;
  } offset_in_;

  // Useful ethercat struct
  struct OffsetOut
  {
    unsigned int slave_status;
    unsigned int control_word;
    unsigned int led_frequency;
  } offset_out_;

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries]; // ethercat utility
  uint8_t temp_;
  bool can_update_ = false;

  // State enumeration order must match the order of state method entries in the state map
  enum States : uint8_t
  {
    ST_IDLE,
    ST_UPDATE,
    ST_MAX_STATES
  };

  // Define the state machine state functions with event data type
  STATE_DECLARE(EasyCatSlave, Idle, NoEventData)
#if METHOD
  GUARD_DECLARE(EasyCatSlave, GuardIdle, NoEventData)
#endif
  STATE_DECLARE(EasyCatSlave, Update, NoEventData)
  GUARD_DECLARE(EasyCatSlave, GuardUpdate, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP_EX
#if METHOD
  STATE_MAP_ENTRY_ALL_EX(&Idle, &GuardIdle, 0, 0)
#else
  STATE_MAP_ENTRY_EX(&Idle)
#endif
  STATE_MAP_ENTRY_ALL_EX(&Update, &GuardUpdate, 0, 0)
  END_STATE_MAP_EX
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_H
