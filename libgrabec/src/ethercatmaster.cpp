/**
 * @file ethercatmaster.cpp
 * @author Simone Comari
 * @date 18 Sep 2018
 * @brief File containing definitions of functions and class declared in ethercatmaster.h.
 */

#include "ethercatmaster.h"

namespace grabec
{

/////////////////////////////////////////////////
/// Public methods
/////////////////////////////////////////////////

EthercatMaster::~EthercatMaster() {}  // necessary for pure abstract destructor

EthercatMaster::EthercatMaster() { check_state_flags_.ClearAll(); }

void EthercatMaster::SetThreadsParams(const RtThreadsParams& params)
{
  threads_params_ = params;
}

void EthercatMaster::Start()
{
  // Setup the rt-thread
  grabrt::Thread thread_rt;
  thread_rt.SetCPUs(threads_params_.rt_cpu_id);
  thread_rt.SetSchedAttr(SCHED_RR, threads_params_.rt_priority);
  thread_rt.SetInitFunc(&StartUpFunWrapper, this);
  thread_rt.SetLoopFunc(&ThreadFunWrapper, this);
  // Setup ethercat communication
  uint8_t ret = InitProtocol();
  DispRetVal(ret, "Initializing EtherCAT devices... ");
  if (ret)
    return;
  // Adjust this thread
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(threads_params_.gui_cpu_id));
  grabrt::SetThreadSchedAttr(SCHED_RR, threads_params_.gui_priority);
  // Start the new rt-thread
  if (thread_rt.GetReady(threads_params_.cycle_cime_nsec))
  {
    THREAD_RUN(thread_rt);
  }
}

/////////////////////////////////////////////////
/// Private methods
/////////////////////////////////////////////////

void EthercatMaster::ThreadFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->LoopFunction();
}

void EthercatMaster::ThreadFunction()
{
  // Receive data
  ecrt_master_receive(master_ptr_);
  ecrt_domain_process(domain_ptr_);
  // Check ethercat State Machine
  CheckConfigState();
  CheckMasterState();
  CheckDomainState();
  // If everything is in order, execute main function of master
  if (check_state_flags_.AnyOff())
    LoopFunction();
  // Write data
  ecrt_domain_queue(domain_ptr_);
  ecrt_master_send(master_ptr_);
}

void EthercatMaster::StartUpFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->StartUpFunction();
}

void EthercatMaster::CheckDomainState()
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
    check_state_flags_.Set(EC_DOMAIN,
                           domain_state_local.wc_state == EC_WC_COMPLETE); // operational?
  }
  domain_state_ = domain_state_local;
}

void EthercatMaster::CheckMasterState()
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
    check_state_flags_.Set(MASTER, master_state_local.al_states ==
                                     EC_AL_STATE_OP); // operational?
  }
  if (master_state_local.link_up != master_state_.link_up)
  {
    std::cout << "Master Link is " << (master_state_local.link_up ? "up" : "down")
              << std::endl;
  }
  master_state_ = master_state_local;
}

void EthercatMaster::CheckConfigState()
{
  ec_slave_config_state_t config_state_local;
  ecrt_slave_config_state(config_ptr_, &config_state_local);
  if (config_state_local.al_state != config_state_.al_state)
  {
    std::cout << "Slaves State " << config_state_local.al_state << std::endl;
    check_state_flags_.Set(CONFIG,
                           config_state_local.al_state == EC_AL_STATE_OP); // operational?
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

void EthercatMaster::GetDomainElements(ec_pdo_entry_reg_t* regs)
{
  uint16_t index = 0;
  for (int i = 0; i < num_slaves_; i++)
  {
    for (uint8_t j = 0; j < slave_[i]->GetDomainEntriesNum(); j++)
    {
      regs[index] = slave_[i]->GetDomainRegister(j);
      index++;
    }
  }
}

uint8_t EthercatMaster::InitProtocol()
{
  ec_pdo_entry_reg_t domain_registers_local[num_domain_elements_];

  // Requesting to initialize master 0
  if (!(master_ptr_ = ecrt_request_master(0)))
  {
    DispRetVal(EINIT, "Requesting master... ");
    return EINIT;
  }

  // Creating Domain Process associated with master 0
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  {
    DispRetVal(EINIT, "Creating domain... ");
    return EINIT;
  }

  // Configuring Slaves
  RetVal ret;
  for (uint8_t i = 0; i < num_slaves_; i++)
  {
    ret = slave_[i]->Configure(master_ptr_, config_ptr_);
    DispRetVal(ret, "Configuring slave %u... ", i);
    if (ret != OK)
      return ECONFIG;
  }

  // Configuring domain
  GetDomainElements(domain_registers_local);
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers_local))
  {
    DispRetVal(EREG, "Registering PDOs' entries... ");
    return EREG;
  }
  DispRetVal(OK, "Registering PDOs' entries... ");

  if (ecrt_master_activate(master_ptr_))
  {
    DispRetVal(EACTIVE, "Activating master... ");
    return EACTIVE;
  }
  DispRetVal(OK, "Activating master... ");

  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  {
    DispRetVal(EACTIVE, "Initializing domain data... ");
    return EACTIVE;
  }
  DispRetVal(OK, "Initializing domain data... ");

  // Initialize slaves
  for (int i = 0; i < num_slaves_; i++)
    slave_[i]->Init(domain_data_ptr_);

  return OK;
}

} // end namespace grabec
