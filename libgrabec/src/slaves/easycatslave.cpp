#include "slaves/easycatslave.h"

namespace grabec
{

EasyCatSlave::EasyCatSlave(const uint8_t slave_position) : StateMachine(ST_MAX_STATES)
{
  /* It is FUNDAMENTAL that the constructor has this form. There are several
   * other ways we can program
   * the slave interface so that we do not have to deal with the assignment of
   * this variables in the constructor,
   * but they will require the user to initialize some of the variables in the
   * main file and then pass the as
   * input variable in the constructor. I find that this way it is easy for "not
   * experienced user" to write all their code
   * in just two files, th .h and .cpp of their actual implementation of the
   * ethercat slave
  */
  // From here, it must be edited by the user
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
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(slave_syncs_);
  // and stop here, the rest is additional

  output_pdos_.slave_status = OPERATIONAL;
}

EasyCatSlave::~EasyCatSlave()
{
  output_pdos_.slave_status = NOT_OPERATIONAL;
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.slave_status, output_pdos_.slave_status);
}

void EasyCatSlave::DoWork()
{
  BEGIN_TRANSITION_MAP                            // - Current State -
    TRANSITION_MAP_ENTRY(ST_UPDATE)     // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_IDLE)          // ST_UPDATE
    END_TRANSITION_MAP(NULL)
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

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(EasyCatSlave, GuardIdle, NoEventData)
{
  if (input_pdos_.num_calls > temp_)
    return TRUE; // ???
  else
    return FALSE; // otherwise
}

// Update slave
STATE_DEFINE(EasyCatSlave, Update, NoEventData)
{
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
