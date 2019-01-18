/**
 * @file ethercatmaster.cpp
 * @author Simone Comari
 * @date 17 Gen 2019
 * @brief File containing definitions of functions and class declared in ethercatmaster.h.
 */

#include "ethercatmaster.h"

namespace grabec
{

/////////////////////////////////////////////////
/// Public methods
/////////////////////////////////////////////////

EthercatMaster::EthercatMaster() { check_state_flags_.ClearAll(); }

EthercatMaster::~EthercatMaster()
{
  if (thread_rt_.IsActive())
    thread_rt_.Stop();
}

void EthercatMaster::SetThreadsParams(const RtThreadsParams& params)
{
  threads_params_ = params;
}

void EthercatMaster::Start()
{
  // Setup the rt-thread
  thread_rt_.SetCPUs(threads_params_.rt_cpu_id);
  thread_rt_.SetSchedAttr(SCHED_RR, threads_params_.rt_priority);
  thread_rt_.SetInitFunc(&StartUpFunWrapper, this);
  thread_rt_.SetLoopFunc(&ThreadFunWrapper, this);
  thread_rt_.SetEndFunc(&ExitFunWrapper, this);
  // Setup ethercat communication
  printf("[EthercatMaster] Initializing EtherCAT network...\n");
//  PrintToQConsoleCb("[EthercatMaster] Initializing EtherCAT network...\n");
  uint8_t ret = InitProtocol();
  printf("[EthercatMaster] Initialization EtherCAT network %s\n",
         ret ? "FAILED" : "COMPLETE");
  if (ret)
  {
    EcStateChangedCb(check_state_flags_);
    if (master_ptr_ != NULL)
      ReleaseMaster();
    return;
  }
  mutex_ = thread_rt_.Mutex();
  // Adjust this thread
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(threads_params_.gui_cpu_id));
  grabrt::SetThreadSchedAttr(SCHED_RR, threads_params_.gui_priority);
  // Start the new rt-thread
  if (~thread_rt_.GetReady(threads_params_.cycle_cime_nsec))
  {
    THREAD_RUN(thread_rt_);
  }
}

/////////////////////////////////////////////////
/// Private methods
/////////////////////////////////////////////////

uint8_t EthercatMaster::InitProtocol()
{
  std::vector<ec_pdo_entry_reg_t> domain_registers;
  domain_registers.resize(num_domain_elements_);

  // Requesting to initialize master 0
  if (!(master_ptr_ = ecrt_request_master(0)))
  {
    DispRetVal(EINIT, "[EthercatMaster]\tRequesting master...");
    return EINIT;
  }
  DispRetVal(OK, "[EthercatMaster]\tRequesting master...");

  // Creating Domain Process associated with master 0
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  {
    DispRetVal(EINIT, "[EthercatMaster]\tCreating domain...");
    return EINIT;
  }
  DispRetVal(OK, "[EthercatMaster]\tCreating domain...");

  // Configuring Slaves
  RetVal ret;
  for (uint8_t i = 0; i < slaves_ptrs_.size(); i++)
  {
    ret = slaves_ptrs_[i]->Configure(master_ptr_, &slave_config_ptr_);
    DispRetVal(ret, "[EthercatMaster]\tConfiguration slave %u...", i);
    if (ret != OK)
      return ECONFIG;
  }

  // Configuring domain
  GetDomainElements(domain_registers);
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers.data()))
  {
    DispRetVal(EREG, "[EthercatMaster]\tRegistering PDOs' entries...");
    return EREG;
  }
  DispRetVal(OK, "[EthercatMaster]\tRegistering PDOs' entries...");

  if (ecrt_master_activate(master_ptr_))
  {
    DispRetVal(EACTIVE, "[EthercatMaster]\tActivating master... ");
    return EACTIVE;
  }
  DispRetVal(OK, "[EthercatMaster]\tActivating master... ");

  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  {
    DispRetVal(EACTIVE, "[EthercatMaster]\tInitializing domain data... ");
    return EACTIVE;
  }
  DispRetVal(OK, "[EthercatMaster]\tInitializing domain data... ");

  // Initialize slaves
  for (size_t i = 0; i < slaves_ptrs_.size(); i++)
    slaves_ptrs_[i]->Init(domain_data_ptr_);

  return OK;
}

void EthercatMaster::StartUpFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->StartUpFunction();
}

void EthercatMaster::ThreadFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->ThreadFunction();
}

void EthercatMaster::ExitFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->ExitFunction();
}

void EthercatMaster::ThreadFunction()
{
  // Receive data
  ecrt_master_receive(master_ptr_);
  ecrt_domain_process(domain_ptr_);
  // Check EtherCAT network state
  CheckConfigState();
  CheckMasterState();
  CheckDomainState();
  // If everything is ok, execute main function of master
  if (check_state_flags_.Count() == 3) // EthercatStateFlagsBit all set
    LoopFunction();
  // Write data
  ecrt_domain_queue(domain_ptr_);
  ecrt_master_send(master_ptr_);
}

void EthercatMaster::ExitFunction()
{
  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  printf("[EthercatMaster] Sending out SHUTDOWN signals...\n");
  while (!AllSlavesReadyToShutDown())
  {
    // Receive data
    ecrt_master_receive(master_ptr_);
    ecrt_domain_process(domain_ptr_);
    // Check EtherCAT network state
    CheckConfigState();
    CheckMasterState();
    CheckDomainState();
    // Send out signal to safely shut down
    for (EthercatSlave* slave_ptr : slaves_ptrs_)
      slave_ptr->SafeExit();
    // Write data
    ecrt_domain_queue(domain_ptr_);
    ecrt_master_send(master_ptr_);

    pthread_mutex_unlock(&mutex_);
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_);
  }
  printf("[EthercatMaster] All slaves ready to be disconnected\n");

  ReleaseMaster();
}

void EthercatMaster::CheckConfigState()
{
  ec_slave_config_state_t slave_config_state;
  ecrt_slave_config_state(slave_config_ptr_, &slave_config_state);
  if (slave_config_state.al_state != slave_config_state_.al_state)
  {
    printf("[EthercatMaster] Slaves application-layer state: ");
    PrintAlState(slave_config_state.al_state);
    check_state_flags_.Set(CONFIG, slave_config_state.al_state == EC_AL_STATE_OP);
    EcStateChangedCb(check_state_flags_);
  }
  if (slave_config_state.online != slave_config_state_.online)
  {
    printf("[EthercatMaster] Slaves status: %s\n",
           slave_config_state.online ? "online" : "offline");
  }
  if (slave_config_state.operational != slave_config_state_.operational)
  {
    printf("[EthercatMaster] Slaves state:%s operational\n",
           slave_config_state.operational ? "" : " Not");
  }
  slave_config_state_ = slave_config_state; // update
}

void EthercatMaster::CheckMasterState()
{
  ec_master_state_t master_state;
  ecrt_master_state(master_ptr_, &master_state);
  if (master_state.slaves_responding != master_state_.slaves_responding)
  {
    printf("[EthercatMaster] %u slave(s) on the bus\n", master_state.slaves_responding);
  }
  if (master_state.al_states != master_state_.al_states)
  {
    printf("[EthercatMaster] Master state: ");
    PrintAlState(master_state.al_states);
    check_state_flags_.Set(MASTER, master_state.al_states == EC_AL_STATE_OP);
    EcStateChangedCb(check_state_flags_);
  }
  if (master_state.link_up != master_state_.link_up)
  {
    printf("[EthercatMaster] Master link is %s\n", master_state.link_up ? "UP" : "DOWN");
  }
  master_state_ = master_state;
}

void EthercatMaster::CheckDomainState()
{
  ec_domain_state_t domain_state;
  ecrt_domain_state(domain_ptr_, &domain_state);
  /*
   * When a frame is sent by the master, the Working Counter is zero for all EtherCAT
   * datagrams. Frames then sequentially encounter all the slaves. If a slave is addressed
   * by a specific datagram, and if the corresponding command was processed correctly
   * (that means if slave memory could be read/written successfully), the associated
   * Working Counter is incremented in a well-defined way.
   */
  if (domain_state.working_counter != domain_state_.working_counter)
  {
    printf("[EthercatMaster] Domain WC: %d\n", domain_state.working_counter);
  }
  if (domain_state.wc_state != domain_state_.wc_state)
  {
    printf("[EthercatMaster] Domain State: %s\n", WcStateStr[domain_state.wc_state]);
    check_state_flags_.Set(EC_DOMAIN, domain_state.wc_state == EC_WC_COMPLETE);
    EcStateChangedCb(check_state_flags_);
  }
  domain_state_ = domain_state;
}

bool EthercatMaster::AllSlavesReadyToShutDown() const
{
  for (EthercatSlave* slave_ptr : slaves_ptrs_)
    if (!slave_ptr->IsReadyToShutDown())
      return false;
  return true;
}

void EthercatMaster::ReleaseMaster()
{
  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  ecrt_master_deactivate(master_ptr_);
  while (1)
  {
    CheckMasterState();
    // Check if master is still up
    if (!master_state_.link_up)
      break;
    // Check if master has been deactivated
    if (master_state_.al_states == EC_AL_STATE_PREOP)
      break;

    pthread_mutex_unlock(&mutex_);
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_);
  }
  printf("[EthercatMaster] Master DEACTIVATED\n");
  ecrt_release_master(master_ptr_);
  printf("[EthercatMaster] Master RELEASED\n");
}

void EthercatMaster::GetDomainElements(std::vector<ec_pdo_entry_reg_t>& regs) const
{
  size_t index = 0;
  for (EthercatSlave* slave_ptr : slaves_ptrs_)
  {
    for (uint8_t j = 0; j < slave_ptr->GetDomainEntriesNum(); j++)
    {
      regs[index] = slave_ptr->GetDomainRegister(j);
      index++;
    }
  }
}

void EthercatMaster::PrintAlState(const uint al_state) const
{
  switch (al_state)
  {
  case EC_AL_STATE_INIT:
    printf("INIT\n");
    break;
  case EC_AL_STATE_PREOP:
    printf("PREOP\n");
    break;
  case EC_AL_STATE_SAFEOP:
    printf("SAFEOP\n");
    break;
  case EC_AL_STATE_OP:
    printf("OP\n");
    break;
  default:
    printf("UNKNOWN (%u)\n", al_state);
    break;
  }
}

} // end namespace grabec
