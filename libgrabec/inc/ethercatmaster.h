#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H

/* Ethercat Master interface. This class is used as base class
   in our master. So we won't need every time to deal with:
   - real time stuff
   - memory locking
   - ethercat initialization process
   - how to cyclically loop
   This way all the effort can be put in the design of our specific
   master, with in mind that the ethercat master interface requires
   to overload some function, which are actually called every loop
   This functions are the ones marked as virtual
*/

#include <signal.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <pthread.h>
#include <limits.h>
#include <iostream>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>

#include "ecrt.h" // ethercat library

#include "threads.h"
#include "ethercatslave.h" // ethercat slave degisn interface

namespace grabec
{

class EthercatMaster
{
private:
  static void TheRtThread(void* args); // The actual real time thread that is executed

  void CheckDomainState();                          // ethercat utility
  void CheckMasterState();                          // ethercat utility
  void CheckConfigState();                          // ethercat utility
  void GetDomainElements(ec_pdo_entry_reg_t* regs); // ethercat utility
  uint8_t FlagManagement();                             // real time process management
  uint8_t InitProtocol();                               // ethercat utility

public:
  EthercatMaster();
  virtual ~EthercatMaster() = 0;

  // constexpr static members of a class can be viewed as #define replacement
  constexpr static uint8_t kMasterOperational = 8;
  constexpr static uint8_t kSlaveOperational = 8;
  constexpr static uint8_t kDomainOperational = 2;
  constexpr static uint8_t kOperationalState = 1;
  constexpr static uint8_t kNotOperationalState = 0;

  // Variables related to our project, must be gived as input
  struct MasterData
  {
  public:
    int gui_cpu_id = 0;
    int rt_cpu_id = -1;
    uint8_t gui_priority = 60;
    uint8_t rt_priority = 98;
    uint32_t cycle_cime_nsec = 1000000;
  } master_data_;                             // Default Values

  ec_master_t* master_ptr_ = NULL;            // ethercat utility
  ec_master_state_t master_state_ = {};       // ethercat utility
  ec_domain_t* domain_ptr_ = NULL;            // ethercat utility
  ec_domain_state_t domain_state_ = {};       // ethercat utility
  ec_slave_config_t* config_ptr_ = NULL;      // ethercat utility
  ec_slave_config_state_t config_state_ = {}; // ethercat utility
  ec_sdo_request_t* sdo_ptr_ = NULL;          // ethercat utility
  uint8_t* domain_data_ptr_ = NULL;           // ethercat utility

  struct EthercatFlags
  {
    uint8_t domain_state;
    uint8_t master_state;
    uint8_t config_state;
    uint8_t not_sync;
  } flags_; // ethercat utility

  pthread_mutex_t rt_mutex_ = PTHREAD_MUTEX_INITIALIZER; // real time process utility
  EthercatSlave** slave_;                                // master utility
  uint8_t num_domain_elements_ = 0;                      // master utility
  int num_slaves_;                                       // master utility

  void Start();                       // the only thing you need to callnin the main
  virtual void StartUpFunction() = 0; // called before the cycle begins
  virtual void LoopFunction() = 0;    // called every cycle
};

} // end namespace grabec

#endif // ETHERCATMASTER_H
