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

  RtThreadsParams threads_params_; /**< Threads scheduler parameters. */

  /** @defgroup EthercatUtilities EtherCAT Utilities
   * This group collects all EtherCAT-specific elements in a generic master-slave
   * implemenation.
   * @{
   */
  ec_master_t* master_ptr_ = NULL;            /**< Pointer to ethercat master. */
  ec_master_state_t master_state_ = {};       /**< State of ethercat master. */
  ec_domain_t* domain_ptr_ = NULL;            /**< Pointer to ethercat domain. */
  ec_domain_state_t domain_state_ = {};       /**< State of ethercat domain. */
  ec_slave_config_t* config_ptr_ = NULL;      /**< Pointer to ethercat configuration. */
  ec_slave_config_state_t config_state_ = {}; /**< State of ethercat configuration. */
  uint8_t* domain_data_ptr_ = NULL;           /**< Pointer to ethercat domain data. */

  Bitfield8
    check_state_flags_; /**< Bitfield object where every bit represents the state of an
                                      * element. 1 means element is _operational_, 0 that
                                      * it is not. */
  /** @} */             // end of EthercatUtilities group

  EthercatSlave** slave_ = NULL;   /**< Vector of pointers to slaves. */
  uint8_t num_domain_elements_ = 0;  /**< Number of elements in ethercat domain. */
  int num_slaves_ = 0;              /**< Number of slaves. */

  /**
   * @brief LoopFunction
   */
  virtual void LoopFunction() = 0;
  /**
   * @brief StartUpFunction
   */
  virtual void StartUpFunction() = 0; // called before the cycle begins

private:
  /**
   * @brief ThreadFunction
   */
  void ThreadFunction();
  /**
   * @brief ThreadFunWrapper
   * @param obj
   */
  static void ThreadFunWrapper(void* obj);
  /**
   * @brief StartUpFunWrapper
   * @param obj
   */
  static void StartUpFunWrapper(void* obj);

  /**
   * @brief CheckDomainState
   * @ingroup EthercatUtilities
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
