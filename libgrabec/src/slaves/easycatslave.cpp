#include "slaves/easycatslave.h"

namespace grabec
{
// Must provide redundant definition of the static member as well as the declaration.
constexpr ec_pdo_entry_info_t EasyCatSlave::kPdoEntries[];
constexpr ec_pdo_info_t EasyCatSlave::kPDOs[];
constexpr ec_sync_info_t EasyCatSlave::kSyncs[];

EasyCatSlave::EasyCatSlave(const uint8_t slave_position) : StateMachine(ST_MAX_STATES)
{
  alias_ = kAlias;
  position_ = slave_position;
  vendor_id_ = kVendorID;
  product_code_ = kProductCode;
  num_domain_entries_ = kDomainEntries;

  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[0].index, kPdoEntries[0].subindex,
                          &offset_out_.slave_status, NULL};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[1].index, kPdoEntries[1].subindex,
                          &offset_out_.control_word, NULL};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[2].index, kPdoEntries[2].subindex,
                          &offset_out_.led_frequency, NULL};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[3].index, kPdoEntries[3].subindex,
                          &offset_in_.slave_state, NULL};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[4].index, kPdoEntries[4].subindex,
                          &offset_in_.num_calls, NULL};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries[5].index, kPdoEntries[5].subindex,
                          &offset_in_.cycle_counter, NULL};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs);

  output_pdos_.slave_status = OPERATIONAL;
}

EasyCatSlave::~EasyCatSlave()
{
  output_pdos_.slave_status = NOT_OPERATIONAL;
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.slave_status, output_pdos_.slave_status);
}

#if !METHOD
void EasyCatSlave::Start()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                   // - Current State -
    TRANSITION_MAP_ENTRY(ST_UPDATE)            // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)   // ST_UPDATE
  END_TRANSITION_MAP(NULL)
  // clang-format on
}
#endif

void EasyCatSlave::DoWork()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                  // - Current State -
#if METHOD
    TRANSITION_MAP_ENTRY(ST_UPDATE)          // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_IDLE)                // ST_UPDATE
#else
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)  // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)  // ST_UPDATE
#endif
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void EasyCatSlave::ReadInputs()
{
  // This is the way we can read the PDOs, according to ecrt.h
  input_pdos_.slave_state = EC_READ_U8(domain_data_ptr_ + offset_in_.slave_state);
  input_pdos_.num_calls = EC_READ_U8(domain_data_ptr_ + offset_in_.num_calls);
  input_pdos_.cycle_counter = EC_READ_U8(domain_data_ptr_ + offset_in_.cycle_counter);
}

void EasyCatSlave::WriteOutputs()
{
  // This is the way we can write the PDOs, according to ecrt.h
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.slave_status, output_pdos_.slave_status);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.control_word, output_pdos_.control_word);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.led_frequency, output_pdos_.led_frequency);
}

// State machine sits here when slave is not running
STATE_DEFINE(EasyCatSlave, Idle, NoEventData) { output_pdos_.control_word = ST_IDLE; }

#if METHOD
// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(EasyCatSlave, GuardIdle, NoEventData)
{
  if (input_pdos_.num_calls > temp_)
    return TRUE; // ???
  else
    return FALSE; // otherwise
}
#endif

// Update slave
STATE_DEFINE(EasyCatSlave, Update, NoEventData)
{
#if !METHOD
  if (input_pdos_.num_calls > temp_)
    InternalEvent(ST_IDLE);
#endif
  output_pdos_.control_word = ST_UPDATE;
  temp_ = input_pdos_.num_calls;
}

// Guard condition to detemine whether Update state is executed.
GUARD_DEFINE(EasyCatSlave, GuardUpdate, NoEventData)
{
  if (can_update_ == ST_UPDATE && input_pdos_.slave_state == ST_IDLE)
    return TRUE; // physical slave is idle and user wants to make it operational
  else
    return FALSE; // otherwise
}

} // end namespace grabec
