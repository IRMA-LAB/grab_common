/**
 * @file ethercatmaster.cpp
 * @author Simone Comari
 * @date 14 Feb 2019
 * @brief File containing definitions of functions and class declared in ethercatmaster.h.
 */

#include "ethercatmaster.h"

namespace grabec {

EthercatMaster::EthercatMaster() { check_state_flags_.ClearAll(); }

EthercatMaster::~EthercatMaster()
{
  if (thread_rt_.IsActive())
    thread_rt_.Stop();
}

//--------- Public functions ---------------------------------------------------------//

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
  thread_rt_.SetLoopFunc(&LoopFunWrapper, this);
  thread_rt_.SetEndFunc(&EndFunWrapper, this);
  thread_rt_.SetEmergencyExitFunc(&EmergencyExitFunWrapper, this);
  mutex_ = thread_rt_.Mutex();
  // Adjust this thread
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(threads_params_.gui_cpu_id));
  grabrt::SetThreadSchedAttr(SCHED_OTHER, threads_params_.gui_priority);
  // Setup EtherCAT communication
  if (!SetupEcNtw())
    return;
  // Start the new rt-thread
  if (~thread_rt_.GetReady(threads_params_.cycle_time_nsec))
  {
    THREAD_RUN(thread_rt_);
  }
  EcRtThreadStatusChanged(thread_rt_.IsActive());
}

void EthercatMaster::Reset()
{
  if (thread_rt_.IsActive())
    thread_rt_.Stop();
  // We must create a fresh new thread once the previous is stopped.
  thread_rt_ = grabrt::Thread();
  Start();
}

//--------- Protected functions --------------------------------------------------------//

void EthercatMaster::EcPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  if (color == 'w')
    printf("[EthercatMaster] %s\n", msg.c_str());
  else
    PrintColor(color, "[EthercatMaster] %s", msg.c_str());
}

//--------- Private thread related functions -----------------------------------------//

void EthercatMaster::StartUpFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->EcStartUpFun();
}

void EthercatMaster::LoopFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->LoopFunction();
}

void EthercatMaster::EndFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->EndFunction();
}

void EthercatMaster::EmergencyExitFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->EmergencyExitFunction();
}

void EthercatMaster::LoopFunction()
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
    EcWorkFun();
  // Write data
  ecrt_domain_queue(domain_ptr_);
  ecrt_master_send(master_ptr_);
}

void EthercatMaster::EndFunction()
{
  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  EcPrintCb("Sending out SHUTDOWN signals to all slaves...");
  while (true)
  {
    if (AllSlavesReadyToShutDown())
    {
      EcPrintCb("All slaves ready to be disconnected");
      break;
    }
    if (clock.ElapsedFromStart() > max_shutdown_wait_time_sec_)
    {
      EcPrintCb("WARNING: Taking too long to safely shutdown slaves. Aborting operation",
                'y');
      break;
    }
    // Receive data
    ecrt_master_receive(master_ptr_);
    ecrt_domain_process(domain_ptr_);
    // Check EtherCAT network state
    CheckConfigState();
    CheckMasterState();
    CheckDomainState();
    // Send out signals to safely shut down slaves
    if (check_state_flags_.Count() == 3) // EthercatStateFlagsBit all set
      for (EthercatSlave* slave_ptr : slaves_ptrs_)
      {
        slave_ptr->ReadInputs();
        slave_ptr->SafeExit();
        slave_ptr->WriteOutputs();
      }
    // Write data
    ecrt_domain_queue(domain_ptr_);
    ecrt_master_send(master_ptr_);

    pthread_mutex_unlock(&mutex_); // mutex was locked by thread obj on this function call
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_); // restore locked mutex
  }
  ReleaseMaster();
}

void EthercatMaster::EmergencyExitFunction()
{
  EcPrintCb("ERROR: Real-Time deadline missed", 'r');
  EcEmergencyFun();
  EndFunction();
  EcRtThreadStatusChanged(false);
}

//--------- Private functions --------------------------------------------------------//

uint8_t EthercatMaster::InitProtocol()
{
  std::vector<ec_pdo_entry_reg_t> domain_registers;
  domain_registers.resize(num_domain_elements_);

  // Requesting to initialize master 0
  if (!(master_ptr_ = ecrt_request_master(0)))
  {
    EcPrintCb("...Requesting master: " + GetRetValStr(EINIT), 'r');
    //    DispRetVal(EINIT, "[EthercatMaster] ...Requesting master: ");
    return EINIT;
  }
  EcPrintCb("...Requesting master: " + GetRetValStr(OK));

  // Creating domain process associated with master 0
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  {
    EcPrintCb("...Creating domain: " + GetRetValStr(EINIT), 'r');
    return EINIT;
  }
  EcPrintCb("...Creating domain: " + GetRetValStr(OK));

  // Configuring slaves
  RetVal ret;
  for (uint8_t i = 0; i < slaves_ptrs_.size(); i++)
  {
    ret = slaves_ptrs_[i]->Configure(master_ptr_, &slave_config_ptr_);
    EcPrintCb("...Configuration slave " + std::to_string(i) + ": " + GetRetValStr(ret),
              ret ? 'r' : 'w');
    if (ret != OK)
      return ECONFIG;
  }

  // Configuring domain
  GetDomainElements(domain_registers);
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers.data()) != 0)
  {
    EcPrintCb("...Registering PDOs' entries: " + GetRetValStr(EREG), 'r');
    return EREG;
  }
  EcPrintCb("...Registering PDOs' entries: " + GetRetValStr(OK));

  if (ecrt_master_activate(master_ptr_) < 0)
  {
    EcPrintCb("...Activating master: " + GetRetValStr(EACTIVE), 'r');
    return EACTIVE;
  }
  EcPrintCb("...Activating master: " + GetRetValStr(OK));

  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  {
    EcPrintCb("...Initializing domain data: " + GetRetValStr(EACTIVE), 'r');
    return EACTIVE;
  }
  EcPrintCb("...Initializing domain data: " + GetRetValStr(OK));

  // Initialize slaves
  for (size_t i = 0; i < slaves_ptrs_.size(); i++)
    slaves_ptrs_[i]->Init(domain_data_ptr_);

  return OK;
}

bool EthercatMaster::SetupEcNtw()
{
  EcPrintCb("Initializing EtherCAT network...");
  uint8_t ret = InitProtocol();
  if (ret)
  {
    EcPrintCb("Initialization EtherCAT network " + GetRetValStr(EFAIL), 'r');
    EcStateChangedCb(check_state_flags_);
    if (master_ptr_ != NULL)
      ReleaseMaster();
    return false;
  }
  EcPrintCb("Initialization EtherCAT network COMPLETE");
  return true;
}

void EthercatMaster::CheckConfigState()
{
  ec_slave_config_state_t slave_config_state;
  ecrt_slave_config_state(slave_config_ptr_, &slave_config_state);
  if (slave_config_state.al_state != slave_config_state_.al_state)
  {
    check_state_flags_.Set(CONFIG, slave_config_state.al_state == EC_AL_STATE_OP);
    EcStateChangedCb(check_state_flags_);
    EcPrintCb("Slaves application-layer state: " +
                GetAlStateStr(slave_config_state.al_state),
              check_state_flags_.CheckBit(CONFIG) ? 'w' : 'y');
  }
  if (slave_config_state.online != slave_config_state_.online)
  {
    if (slave_config_state.online)
      EcPrintCb(std::string("Slaves status: ONLINE"));
    else
      EcPrintCb(std::string("Slaves status: OFFLINE"), 'y');
  }
  if (slave_config_state.operational != slave_config_state_.operational)
  {
    if (slave_config_state.operational)
      EcPrintCb(std::string("Slaves state: OPERATIONAL"));
    else
      EcPrintCb(std::string("Slaves state: NOT OPERATIONAL"), 'y');
  }
  slave_config_state_ = slave_config_state; // update
}

void EthercatMaster::CheckMasterState()
{
  ec_master_state_t master_state;
  ecrt_master_state(master_ptr_, &master_state);
  if (master_state.slaves_responding != master_state_.slaves_responding)
  {
    EcPrintCb(std::to_string(master_state.slaves_responding) + "/" +
                std::to_string(slaves_ptrs_.size()) + " slave(s) on the bus",
              master_state.slaves_responding < slaves_ptrs_.size() ? 'y' : 'w');
  }
  if (master_state.al_states != master_state_.al_states)
  {
    check_state_flags_.Set(MASTER, master_state.al_states == EC_AL_STATE_OP);
    EcStateChangedCb(check_state_flags_);
    EcPrintCb("Master state: " + GetAlStateStr(master_state.al_states),
              check_state_flags_.CheckBit(MASTER) ? 'w' : 'y');
  }
  if (master_state.link_up != master_state_.link_up)
  {
    if (master_state.link_up)
      EcPrintCb(std::string("Master link is UP"));
    else
      EcPrintCb(std::string("Master link is DOWN"), 'y');
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
    EcPrintCb("Domain WC: " + std::to_string(domain_state.working_counter));
  }
  if (domain_state.wc_state != domain_state_.wc_state)
  {
    check_state_flags_.Set(EC_DOMAIN, domain_state.wc_state == EC_WC_COMPLETE);
    EcStateChangedCb(check_state_flags_);
    EcPrintCb("Domain State: " + std::string(WcStateStr[domain_state.wc_state]),
              check_state_flags_.CheckBit(EC_DOMAIN) ? 'w' : 'y');
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
  static const double kMaxWaitTimeSec = 3.0;

  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  ecrt_master_deactivate(master_ptr_);
  while (1)
  {
    CheckMasterState();
    // Check if master is still up
    if (!master_state_.link_up)
    {
      EcPrintCb("Master is DOWN. Skipping deactivation step", 'y');
      break;
    }
    // Check if master has been deactivated
    if (master_state_.al_states == EC_AL_STATE_PREOP)
    {
      EcPrintCb("Master DEACTIVATED");
      break;
    }
    // Break if it takes too long (something is wrong)
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec)
    {
      EcPrintCb("Master is taking too long to deactivate. Skipping this step", 'y');
      break;
    }

    pthread_mutex_unlock(&mutex_);
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_);
  }
  ecrt_release_master(master_ptr_);
  EcPrintCb("Master RELEASED");
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

std::string EthercatMaster::GetAlStateStr(const uint al_state) const
{
  switch (al_state)
  {
  case EC_AL_STATE_INIT:
    return std::string("INIT");
  case EC_AL_STATE_PREOP:
    return std::string("PREOP");
  case EC_AL_STATE_SAFEOP:
    return std::string("SAFEOP");
  case EC_AL_STATE_OP:
    return std::string("OP");
  default:
    return "UNKNOWN (" + std::to_string(al_state) + ")";
  }
}

} // end namespace grabec
