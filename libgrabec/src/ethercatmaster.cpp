/**
 * @file ethercatmaster.cpp
 * @author Simone Comari
 * @date Jan 2022
 * @brief File containing definitions of functions and class declared in ethercatmaster.h.
 */

#include "ethercatmaster.h"

namespace grabec {

struct timespec EthercatMaster::wakeupTime; // static attribute re-definition

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}


EthercatMaster::EthercatMaster() { check_state_flags_.reset(); }

EthercatMaster::~EthercatMaster()
{
  if (thread_rt_.IsActive())
    thread_rt_.Stop();
}

//--------- Public functions ---------------------------------------------------------//

void EthercatMaster::setThreadsParams(const RtThreadsParams& params)
{
  threads_params_ = params;
}

void EthercatMaster::start()
{
  // Setup the rt-thread
  thread_rt_.SetCPUs(threads_params_.rt_cpu_id);
  thread_rt_.SetSchedAttr(SCHED_RR, threads_params_.rt_priority);
  thread_rt_.SetInitFunc(&startUpFunWrapper, this);
  thread_rt_.SetLoopFunc(&loopFunWrapper, this);
  thread_rt_.SetEndFunc(&endFunWrapper, this);
  thread_rt_.SetEmergencyExitFunc(&emergencyExitFunWrapper, this);
  mutex_ = thread_rt_.Mutex();
  // Adjust this thread
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(threads_params_.main_cpu_id));
  grabrt::SetThreadSchedAttr(SCHED_OTHER);
  // Setup EtherCAT communication
  if (!setupEcNtw())
    return;
  // Start the new rt-thread
  if (~thread_rt_.GetReady(threads_params_.cycle_time_nsec))
  {
    THREAD_RUN(thread_rt_);
  }
  ecRtThreadStatusChanged(thread_rt_.IsActive());
}

void EthercatMaster::reset()
{
  if (thread_rt_.IsActive())
    thread_rt_.Stop();
  // We must create a fresh new thread once the previous is stopped.
  thread_rt_ = grabrt::Thread();
  start();
}

//--------- Protected functions --------------------------------------------------------//

void EthercatMaster::ecPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  if (color == 'w')
    printf("[EthercatMaster] %s\n", msg.c_str());
  else
    printColor(color, "[EthercatMaster] %s", msg.c_str());
}

//--------- Private thread related functions -----------------------------------------//

void EthercatMaster::startUpFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->ecStartUpFun();
  // get current time - from Etherlab dc_user example for DC synchronization
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

}

void EthercatMaster::loopFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->loopFunction();
}

void EthercatMaster::endFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->endFunction();
}

void EthercatMaster::emergencyExitFunWrapper(void* obj)
{
  static_cast<EthercatMaster*>(obj)->emergencyExitFunction();
}

void EthercatMaster::loopFunction()
{
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

       // Write application time to master
       //
       // It is a good idea to use the target time (not the measured time) as
       // application time, because it is more stable.
       //
    ecrt_master_application_time(master_ptr_, TIMESPEC2NS(wakeupTime));

  // Receive data
  ecrt_master_receive(master_ptr_);
  ecrt_domain_process(domain_ptr_);

  // get current time - from Etherlab dc_user example for DC synchronization

  // Check EtherCAT network state
  checkConfigState();
  checkMasterState();
  checkDomainState();
  // If everything is ok, execute main function of master
  if (check_state_flags_.all()) // EthercatStateFlagsBit all set
    ecWorkFun();

  //DC sync stuff
  if (sync_ref_counter) {
    sync_ref_counter--;
  } else {
    sync_ref_counter = 1; // sync every cycle

    clock_gettime(CLOCK_TO_USE, &sync_time);

    ecrt_master_sync_reference_clock_to(master_ptr_, TIMESPEC2NS(sync_time));
  }
  ecrt_master_sync_slave_clocks(master_ptr_);


  // Write data
  ecrt_domain_queue(domain_ptr_);
  ecrt_master_send(master_ptr_);
}

void EthercatMaster::endFunction()
{
  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  ecPrintCb("Sending out SHUTDOWN signals to all slaves...");
  while (true)
  {
    if (allSlavesReadyToShutDown())
    {
      ecPrintCb("All slaves ready to be disconnected");
      break;
    }
    if (clock.ElapsedFromStart() > max_shutdown_wait_time_sec_)
    {
      ecPrintCb("WARNING: Taking too long to safely shutdown slaves. Aborting operation",
                'y');
      break;
    }
    // Receive data
    ecrt_master_receive(master_ptr_);
    ecrt_domain_process(domain_ptr_);
    // Check EtherCAT network state
    checkConfigState();
    checkMasterState();
    checkDomainState();
    // Send out signals to safely shut down slaves
    if (check_state_flags_.all()) // EthercatStateFlagsBit all set
      for (EthercatSlave* slave_ptr : slaves_ptrs_)
      {
        slave_ptr->readInputs();
        slave_ptr->safeExit();
        slave_ptr->writeOutputs();
      }

    if (sync_ref_counter) {
      sync_ref_counter--;
    } else {
      sync_ref_counter = 1; // sync every cycle

      clock_gettime(CLOCK_TO_USE, &sync_time);

      ecrt_master_sync_reference_clock_to(master_ptr_, TIMESPEC2NS(sync_time));
    }
    ecrt_master_sync_slave_clocks(master_ptr_);

    // Write data
    ecrt_domain_queue(domain_ptr_);
    ecrt_master_send(master_ptr_);

    pthread_mutex_unlock(&mutex_); // mutex was locked by thread obj on this function call
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_); // restore locked mutex
  }
  releaseMaster();
}

void EthercatMaster::emergencyExitFunction()
{
  ecPrintCb("ERROR: Real-Time deadline missed", 'r');
  ecEmergencyFun();
  endFunction();
  ecRtThreadStatusChanged(false);
}

//--------- Private functions --------------------------------------------------------//

uint8_t EthercatMaster::initProtocol()
{
  std::vector<ec_pdo_entry_reg_t> domain_registers;
  domain_registers.resize(num_domain_elements_);//correzione
  //domain_registers.resize(128);
  // Requesting to initialize master 0
  if (!(master_ptr_ = ecrt_request_master(0)))
  {
    ecPrintCb("...Requesting master: " + getRetValStr(EINIT), 'r');
    //    DispRetVal(EINIT, "[EthercatMaster] ...Requesting master: ");
    return EINIT;
  }
  ecPrintCb("...Requesting master: " + getRetValStr(OK));

  // Creating domain process associated with master 0
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  {
    ecPrintCb("...Creating domain: " + getRetValStr(EINIT), 'r');
    return EINIT;
  }
  ecPrintCb("...Creating domain: " + getRetValStr(OK));

  // Configuring slaves
  RetVal ret;
  for (uint8_t i = 0; i < slaves_ptrs_.size(); i++)
  {
    ecPrintCb("...Configuring slave " + std::to_string(i) + "...");
    ret = slaves_ptrs_[i]->configure(master_ptr_, &slave_config_ptr_);
    ecPrintCb("...Configuration slave " + std::to_string(i) + ": " + getRetValStr(ret),
              ret ? 'r' : 'w');
    if (ret != OK)
      return ECONFIG;
  }

  // Configuring domain
  getDomainElements(domain_registers);
  std::cout<<"domain_reg_size"<<domain_registers.size();
  for (size_t i = 0; i < domain_registers.size(); ++i) {
    printf("Reg %zu: idx=0x%X sub=0x%X => offset=%ud\n",
           i,
           domain_registers[i].index,
           domain_registers[i].subindex,
           *domain_registers[i].offset);
  }
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers.data()) != 0)
  {
    ecPrintCb("...Registering PDOs' entries: " + getRetValStr(EREG), 'r');
    return EREG;
  }
  ecPrintCb("...Registering PDOs' entries: " + getRetValStr(OK));


  if (ecrt_master_activate(master_ptr_) < 0)
  {
    ecPrintCb("...Activating master: " + getRetValStr(EACTIVE), 'r');
    return EACTIVE;
  }
  ecPrintCb("...Activating master: " + getRetValStr(OK));

  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  {
    ecPrintCb("...Initializing domain data: " + getRetValStr(EACTIVE), 'r');
    return EACTIVE;
  }
  ecPrintCb("...Initializing domain data: " + getRetValStr(OK));

  // Initialize slaves
  for (size_t i = 0; i < slaves_ptrs_.size(); i++)
    slaves_ptrs_[i]->init(domain_data_ptr_);
  return OK;
}

bool EthercatMaster::setupEcNtw()
{
  ecPrintCb("Initializing EtherCAT network...");
  uint8_t ret = initProtocol();
  if (ret)
  {
    ecPrintCb("Initialization EtherCAT network ret " + getRetValStr(EFAIL), 'r');
    ecStateChangedCb(check_state_flags_);
    if (master_ptr_ != nullptr)
      releaseMaster();
    return false;
  }
  ecPrintCb("Initialization EtherCAT network COMPLETE");
  return true;
}

void EthercatMaster::checkConfigState()
{
  ec_slave_config_state_t slave_config_state;
  ecrt_slave_config_state(slave_config_ptr_, &slave_config_state);
  if (slave_config_state.al_state != slave_config_state_.al_state)
  {
    check_state_flags_.set(CONFIG, slave_config_state.al_state == EC_AL_STATE_OP);
    ecStateChangedCb(check_state_flags_);
    ecPrintCb("Slaves application-layer state: " +
                getAlStateStr(slave_config_state.al_state),
              check_state_flags_.test(CONFIG) ? 'w' : 'y');
  }
  if (slave_config_state.online != slave_config_state_.online)
  {
    if (slave_config_state.online)
      ecPrintCb(std::string("Slaves status: ONLINE"));
    else
      ecPrintCb(std::string("Slaves status: OFFLINE"), 'y');
  }
  if (slave_config_state.operational != slave_config_state_.operational)
  {
    if (slave_config_state.operational)
      ecPrintCb(std::string("Slaves state: OPERATIONAL"));
    else
      ecPrintCb(std::string("Slaves state: NOT OPERATIONAL"), 'y');
  }
  slave_config_state_ = slave_config_state; // update
}

void EthercatMaster::checkMasterState()
{
  ec_master_state_t master_state;
  ecrt_master_state(master_ptr_, &master_state);
  if (master_state.slaves_responding != master_state_.slaves_responding)
  {
    ecPrintCb(std::to_string(master_state.slaves_responding) + "/" +
                std::to_string(slaves_ptrs_.size()) + " slave(s) on the bus",
              master_state.slaves_responding < slaves_ptrs_.size() ? 'y' : 'w');
  }
  if (master_state.al_states != master_state_.al_states)
  {
    check_state_flags_.set(MASTER, master_state.al_states == EC_AL_STATE_OP);
    ecStateChangedCb(check_state_flags_);
    ecPrintCb("Master state: " + getAlStateStr(master_state.al_states),
              check_state_flags_.test(MASTER) ? 'w' : 'y');
  }
  if (master_state.link_up != master_state_.link_up)
  {
    if (master_state.link_up)
      ecPrintCb(std::string("Master link is UP"));
    else
      ecPrintCb(std::string("Master link is DOWN"), 'y');
  }
  master_state_ = master_state;
}

void EthercatMaster::checkDomainState()
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
    ecPrintCb("Domain WC: " + std::to_string(domain_state.working_counter));
  }
  if (domain_state.wc_state != domain_state_.wc_state)
  {
    check_state_flags_.set(EC_DOMAIN, domain_state.wc_state == EC_WC_COMPLETE);
    ecStateChangedCb(check_state_flags_);
    ecPrintCb("Domain State: " + std::string(WcStateStr[domain_state.wc_state]),
              check_state_flags_.test(EC_DOMAIN) ? 'w' : 'y');
  }
  domain_state_ = domain_state;
}

bool EthercatMaster::allSlavesReadyToShutDown() const
{
  for (EthercatSlave* slave_ptr : slaves_ptrs_)
    if (!slave_ptr->isReadyToShutDown())
      return false;
  return true;
}

void EthercatMaster::releaseMaster()
{
  static const double kMaxWaitTimeSec = 3.0;

  grabrt::ThreadClock clock(thread_rt_.GetCycleTimeNsec());
  ecrt_master_deactivate(master_ptr_);
  while (1)
  {
    checkMasterState();
    // Check if master is still up
    if (!master_state_.link_up)
    {
      ecPrintCb("Master is DOWN. Skipping deactivation step", 'y');
      break;
    }
    // Check if master has been deactivated
    if (master_state_.al_states == EC_AL_STATE_PREOP)
    {
      ecPrintCb("Master DEACTIVATED");
      break;
    }
    // Break if it takes too long (something is wrong)
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec)
    {
      ecPrintCb("Master is taking too long to deactivate. Skipping this step", 'y');
      break;
    }

    pthread_mutex_unlock(&mutex_);
    clock.WaitUntilNext();
    pthread_mutex_lock(&mutex_);
  }
  ecrt_release_master(master_ptr_);
  ecPrintCb("Master RELEASED");
}

void EthercatMaster::getDomainElements(std::vector<ec_pdo_entry_reg_t>& regs) const
{
  size_t index = 0;
  for (EthercatSlave* slave_ptr : slaves_ptrs_)
  {
    for (uint16_t j = 0; j < slave_ptr->getDomainEntriesNum(); j++)
    {
      regs[index] = slave_ptr->getDomainRegister(j);
      index++;
    }
  }
  std::cout<<"final index"<<index<<std::endl;
}

std::string EthercatMaster::getAlStateStr(const uint al_state) const
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
