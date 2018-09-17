#include "ethercatmaster.h"

namespace grabec
{

static EthercatMaster* this_instance;

void* EthercatMaster::TheRtThread(void* args)
{
  this_instance->StartUpFunction();
  while (cycle_flag)
  {
    pthread_mutex_lock(
      &this_instance->rt_mutex_); // Lock the resources while we are using it
    ecrt_master_receive(this_instance->master_ptr_); // Receive data
    ecrt_domain_process(this_instance->domain_ptr_);
    this_instance->CheckConfigState(); // Check ethercat State Machine
    this_instance->CheckMasterState();
    this_instance->CheckDomainState();
    if (!this_instance->flags_.not_sync && this_instance->flags_.config_state &&
        this_instance->flags_.master_state && this_instance->flags_.domain_state)
    {
      this_instance->LoopFunction(); // If everything is in order, execute
                                     // loopfunction of master
    }
    else
    {
      cycle_flag = this_instance->FlagManagement();
    }
    ecrt_domain_queue(this_instance->domain_ptr_); // Write data
    ecrt_master_send(this_instance->master_ptr_);
    pthread_mutex_unlock(&this_instance->rt_mutex_); // Unlock resources
    this_instance->flags_.not_sync = WaitUntilPeriodElapsed(&period_info); // sleep
  }
  return args;
}

void EthercatMaster::CheckDomainState() // Check ethercat domain state machine
{
  ec_domain_state_t domain_state_local;

  ecrt_domain_state(domain_ptr_, &domain_state_local);

  if (domain_state_local.working_counter != domain_state_.working_counter)
  {
    std::cout << "Domain: WC " << domain_state_local.working_counter << std::endl;
  }
  if (domain_state_local.wc_state != domain_state_.wc_state)
  {
    std::cout << "Domain: State " << domain_state_local.wc_state << std::endl;
    if (domain_state_local.wc_state == kDomainOperational)
      flags_.domain_state = kOperationalState;
    else
      flags_.domain_state = kNotOperationalState;
  }

  domain_state_ = domain_state_local;
}

void EthercatMaster::CheckMasterState() // Check ethercat master state machine
{
  ec_master_state_t master_state_local;

  ecrt_master_state(master_ptr_, &master_state_local);

  if (master_state_local.slaves_responding != master_state_.slaves_responding)
  {
    std::cout << master_state_local.slaves_responding << " slave(s) on the bus"
              << std::endl;
  }
  if (master_state_local.al_states != master_state_.al_states)
  {
    std::cout << "Master states: " << master_state_local.al_states << std::endl;
    if (master_state_local.al_states == kMasterOperational)
      flags_.master_state = kOperationalState;
    else
      flags_.master_state = kNotOperationalState;
  }
  if (master_state_local.link_up != master_state_.link_up)
  {
    std::cout << "Master Link is " << (master_state_local.link_up ? "up" : "down")
              << std::endl;
  }

  master_state_ = master_state_local;
}

void EthercatMaster::CheckConfigState() // Check ethercat slave configuration
                                        // state machine
{
  ec_slave_config_state_t config_state_local;

  ecrt_slave_config_state(config_ptr_, &config_state_local);

  if (config_state_local.al_state != config_state_.al_state)
  {
    std::cout << "Slaves State " << config_state_local.al_state << std::endl;
    if (config_state_local.al_state == kSlaveOperational)
      flags_.config_state = kOperationalState;
    else
      flags_.config_state = kNotOperationalState;
  }
  if (config_state_local.online != config_state_.online)
  {
    std::cout << "Slaves: " << (config_state_local.online ? "online" : "offline")
              << std::endl;
  }
  if (config_state_local.operational != config_state_.operational)
  {
    std::cout << "Slaves: " << (config_state_local.operational ? "" : "Not ")
              << "operational" << std::endl;
  }

  config_state_ = config_state_local;
}

void EthercatMaster::GetDomainElements(ec_pdo_entry_reg_t* regs) // Wrapper..
{
  uint16_t index = 0;
  for (int i = 0; i < num_slaves_; i++)
  {
    for (uint8_t j = 0; j < slave_[i]->num_domain_entries_; j++)
    {
      regs[index] = slave_[i]->domain_registers_ptr_[j];
      index++;
    }
  }
}

uint8_t EthercatMaster::FlagManagement()
{
  // Nothing is currently implemented for ethercat state transition
  // Even though we do nothings, if no damage is present in the hardware
  // and our installation, the transition to operational gets 10 ms
  if (flags_.not_sync)
  { // We check for
    return kNotValidate_;
  }
  return kValidate_;
}

uint8_t EthercatMaster::InitProtocol()
{
  ec_pdo_entry_reg_t domain_registers_local[num_domain_elements_];

  if (!(master_ptr_ = ecrt_request_master(0)))
  { // Requesting to initialize master 0
    std::cout << "Error requesting master" << std::endl;
    return kNotValidate_;
  }
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  { // Creating Domain Process associated with master 0
    std::cout << "Error Creating Domain" << std::endl;
    return kNotValidate_;
  }

  for (int i = 0; i < num_slaves_; i++)
  { // Configuring Slaves
    if (!(config_ptr_ =
            ecrt_master_slave_config(master_ptr_, slave_[i]->alias_, slave_[i]->position_,
                                     slave_[i]->vendor_id_, slave_[i]->product_code_)))
    {
      std::cout << "Error Configuring Slave Devices" << std::endl;
      return kNotValidate_;
    }
    std::cout << "Configuring PDOs" << std::endl;
    if (ecrt_slave_config_pdos(config_ptr_, EC_END, slave_[i]->slave_sync_ptr_))
    {
      std::cout << "Error Configuring PDOs" << std::endl;
      return kNotValidate_;
    }
    if (slave_[i]->SdoRequests(sdo_ptr_, config_ptr_))
    {
      return kNotValidate_;
    }
  }

  GetDomainElements(domain_registers_local); // Configuring Domain
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers_local))
  {
    std::cout << "Error Registering PDOs' entries" << std::endl;
    return kNotValidate_;
  }

  std::cout << "Activating master" << std::endl;
  if (ecrt_master_activate(master_ptr_))
  { // Activating Master
    std::cout << "Error activating Master" << std::endl;
    return kNotValidate_;
  }
  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  { // Activating Domain
    std::cout << "Error Initializing Domain Data" << std::endl;
    return kNotValidate_;
  }

  for (int i = 0; i < num_slaves_; i++)
    slave_[i]->Init(domain_data_ptr_); // Performing Slave initialization
                                       // function, default is empty function

  return kValidate_;
}

EthercatMaster::EthercatMaster()
{
  this_instance = this; // we need the address of the actual master, we'll need
                        // it in the static functions
}

EthercatMaster::~EthercatMaster() {}

void EthercatMaster::Start()
{
  // Setup the rt-thread
  grabrt::Thread thread_rt;
  thread_rt.SetCPUs(master_data_.rt_cpu_id);
  thread_rt.SetSchedAttr(SCHED_RR, master_data_.rt_priority);
  thread_rt.SetInitFunc(StartUpFunction, this);
  thread_rt.SetLoopFunc(&TheRtThread, this);
  // Setup ethercat communication
  uint8_t ret = this_instance->InitProtocol();
  DispError(ret, "Initializing EtherCAT devices... ");
  if (ret)
    return;

  // Adjust this thread
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(master_data_.gui_cpu_id));
  grabrt::SetThreadSchedAttr(SCHED_RR, master_data_.gui_priority);
  // start the new rt-thread
  if (thread_rt.GetReady(master_data_.cycle_cime_nsec))
  {
    THREAD_RUN(thread_rt);
  }
}

} // end namespace grabec
