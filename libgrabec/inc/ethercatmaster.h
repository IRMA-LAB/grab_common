/**
 * @file ethercatmaster.h
 * @author Edoardo Idà, Simone Comari
 * @date 18 Sep 2018
 * @brief This file includes an abstract class to setup an ethercat master-slave
 * communication.
 */

#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H

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
#include "types.h"
#include "ethercatslave.h"

namespace grabec
{

/**
 * @brief Ethercat Master interface.
 *
 * This class is used as base class in our master. So we won't need every time to deal
 * with:
 * - real time stuff
 * - memory locking
 * - ethercat initialization process
 * - how to cyclically loop
 * This way all the effort can be put in the design of our specific master, reminding
 * that the ethercat master interface requires to overload some functions, which are
 * actually called every loop. This functions are the ones marked as @c virtual.
 */
class EthercatMaster
{
public:
  /**
   * @brief EthercatMaster
   */
  EthercatMaster();
  virtual ~EthercatMaster() = 0;

  /**
   * @brief SetThreadsParams
   * @param params
   */
  void SetThreadsParams(const RtThreadsParams& params);

  /**
   * @brief Start
   */
  void Start();

private:
  /**
  *@brief Ethercat state check flags bit position enum.
  *
  * The check_state_flags_ is a bitfield object where every bit represents the state of an
  * element. 1 means element is _operational_, 0 that it is not. This enum helps grabbing the
  * right element without remembering its position index.
  *
  * For instance, to check if master is operational:
  * @code{.cpp}
  * Bitfield8 check_state_flags;
  * bool master_state = check_state_flags.CheckBit(MASTER);
  * @endcode
  * @todo understand why this can't go in types.h
  */
  enum EthercatStateFlagsBit
  {
    DOMAIN,
    MASTER,
    CONFIG
  };

  RtThreadsParams threads_params_; // Default Values

  ec_master_t* master_ptr_ = NULL;            // ethercat utility
  ec_master_state_t master_state_ = {};       // ethercat utility
  ec_domain_t* domain_ptr_ = NULL;            // ethercat utility
  ec_domain_state_t domain_state_ = {};       // ethercat utility
  ec_slave_config_t* config_ptr_ = NULL;      // ethercat utility
  ec_slave_config_state_t config_state_ = {}; // ethercat utility
  ec_sdo_request_t* sdo_ptr_ = NULL;          // ethercat utility
  uint8_t* domain_data_ptr_ = NULL;           // ethercat utility

  Bitfield8 check_state_flags_; // ethercat utility

  EthercatSlave** slave_ = NULL;    // master utility
  uint8_t num_domain_elements_ = 0; // master utility
  int num_slaves_ = 0;              // master utility

  /**
   * @brief ThreadFunWrapper
   * @param obj
   */
  static void ThreadFunWrapper(void* obj);
  /**
   * @brief ThreadFunction
   */
  void ThreadFunction();
  /**
   * @brief LoopFunction
   */
  virtual void LoopFunction() = 0;
  /**
   * @brief StartUpFunWrapper
   * @param obj
   */
  static void StartUpFunWrapper(void* obj);
  /**
   * @brief StartUpFunction
   */
  virtual void StartUpFunction() = 0; // called before the cycle begins

  /**
   * @brief CheckDomainState
   */
  void CheckDomainState();
  /**
   * @brief CheckMasterState
   */
  void CheckMasterState();
  /**
   * @brief CheckConfigState
   */
  void CheckConfigState();
  /**
   * @brief GetDomainElements
   * @param regs
   */
  void GetDomainElements(ec_pdo_entry_reg_t* regs);
  /**
   * @brief InitProtocol
   * @return
   */
  uint8_t InitProtocol();
};

} // end namespace grabec

#endif // ETHERCATMASTER_H
