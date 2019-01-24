/**
 * @file ethercatmaster.h
 * @author Edoardo Idà, Simone Comari
 * @date 24 Gen 2019
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
   * @brief Default constructor.
   */
  EthercatMaster();
  virtual ~EthercatMaster() = 0;

  /**
   * @brief Set threads parameters.
   * @param params Threads parameters structure.
   */
  void SetThreadsParams(const RtThreadsParams& params);

  /**
   * @brief Start the master on a new real-time thread.
   */
  void Start();

  /**
   * @brief Mutex
   * @return
   */
  pthread_mutex_t& Mutex() { return mutex_; }
  /**
   * @brief Mutex
   * @return
   */
  const pthread_mutex_t& Mutex() const { return mutex_; }

protected:
  //------- Workaround to generate pseudo-qt-signals ------------------//

  virtual void EcStateChangedCb(const Bitfield8& /*new_state*/) {}
  virtual void EcPrintCb(const std::string& msg, const char color = 'w') const;
  virtual void EcRtThreadStatusChanged(const bool /*active*/) const {}

protected:
  /**
   *@brief Ethercat state check flags bit position enum.
   *
   * The check_state_flags_ is a bitfield object where every bit represents the state of
   *an
   * element. 1 means element is _operational_, 0 that it is not. This enum helps grabbing
   * the right element without remembering its position index.
   *
   * For instance, to check if master is operational:
   * @code{.cpp}
   * Bitfield8 check_state_flags;
   * bool master_state = check_state_flags.CheckBit(MASTER);
   * @endcode
   * @todo understand why this can't go in types.h
   */
  enum EthercatStateFlagsBit : uint8_t
  {
    EC_DOMAIN,
    MASTER,
    CONFIG
  };

  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER; /**< RT thread mutex. */

  RtThreadsParams threads_params_; /**< Threads scheduler parameters. */

  /** @defgroup EthercatUtilities EtherCAT Utilities
   * This group collects all EtherCAT-specific elements in a generic master-slave
   * implemenation.
   * @{
   */
  ec_master_t* master_ptr_ = NULL;             /**< Pointer to ethercat master. */
  ec_master_state_t master_state_ = {};        /**< State of ethercat master. */
  ec_domain_t* domain_ptr_ = NULL;             /**< Pointer to ethercat domain. */
  ec_domain_state_t domain_state_ = {};        /**< State of ethercat domain. */
  ec_slave_config_t* slave_config_ptr_ = NULL; /**< Pointer to ethercat configuration. */
  ec_slave_config_state_t slave_config_state_ =
    {};                             /**< State of ethercat configuration. */
  uint8_t* domain_data_ptr_ = NULL; /**< Pointer to ethercat domain data. */

  Bitfield8
    check_state_flags_; /**< Bitfield object where every bit represents the state of an
                                      * element. 1 means element is _operational_, 0 that
                                      * it is not. */
  /** @} */             // end of EthercatUtilities group

  std::vector<EthercatSlave*> slaves_ptrs_; /**< Vector of pointers to slaves. */
  uint8_t num_domain_elements_ = 0;         /**< Number of elements in ethercat domain. */

  /**
   * @brief EcStartUpFun
   */
  virtual void EcStartUpFun() {} // called before the cycle begins

  /**
   * @brief EcWorkFun
   */
  virtual void EcWorkFun() = 0; // called at every cycle

  /**
   * @brief EcEmergencyFun
   */
  virtual void EcEmergencyFun() {} // called at emergency exit (on rt deadline missed)

private:
  //------- Thread related --------------------------------//

  grabrt::Thread thread_rt_;

  static void StartUpFunWrapper(void* obj);
  static void LoopFunWrapper(void* obj);
  static void EndFunWrapper(void* obj);
  static void EmergencyExitFunWrapper(void* obj);

  void LoopFunction();
  void EndFunction();
  void EmergencyExitFunction();

private:
  uint8_t InitProtocol();

  void CheckDomainState();
  void CheckMasterState();
  void CheckConfigState();

  bool AllSlavesReadyToShutDown() const;
  void ReleaseMaster();

  void GetDomainElements(std::vector<ec_pdo_entry_reg_t>& regs) const;

  std::string GetAlStateStr(const uint al_state) const;
};

} // end namespace grabec

#endif // ETHERCATMASTER_H
