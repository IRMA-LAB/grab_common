/**
 * @file ethercatmaster.h
 * @author Edoardo Idà, Simone Comari
 * @date Jan 2022
 * @brief This file includes an abstract class to setup an ethercat master-slave
 * communication.
 */

#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H

#include <bitset>
#include <cstring>
#include <iostream>
#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ecrt.h" // ethercat library

#include "ethercatslave.h"
#include "grabec_types.h"
#include "threads.h"

/**
 * @brief Namespace for GRAB EtherCAT library.
 */
namespace grabec {

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
  void setThreadsParams(const RtThreadsParams& params);

  /**
   * @brief Start the master on a new real-time thread.
   */
  void start();

  /**
   * @brief Reset the master on a fresh new real-time thread.
   */
  void reset();

  /**
   * @brief Get a reference to real-time thread mutex.
   * @return A reference to real-time thread mutex.
   */
  pthread_mutex_t& mutex() { return mutex_; }
  /**
   * @brief Get a constant reference to real-time thread mutex.
   * @return A constant reference to real-time thread mutex.
   */
  const pthread_mutex_t& mutex() const { return mutex_; }
  /**
   * @brief Get real-time thread cycle time in nanoseconds.
   * @return Real-time thread cycle time in nanoseconds.
   */
  uint32_t getRtCycleTimeNsec() const { return threads_params_.cycle_time_nsec; }

 protected:
  //------- Workaround to generate pseudo-qt-signals ------------------//

  /**
   * @brief EtherCAT network state changed pseudo-signal.
   *
   * This function emulates a signal emit, and it can be overridden to perform the actual
   * signal emition in a Qt context, i.e. in a QObject which inherits from this class.
   * @warning This function lives in the real-time thread, so keep it short if overridden.
   */
  virtual void ecStateChangedCb(const std::bitset<3>& /*new_state*/) {}
  /**
   * @brief EtherCAT information print pseudo-signal.
   *
   * This function emulates a signal emit, and it can be overridden to perform the actual
   * signal emition in a Qt context, i.e. in a QObject which inherits from this class.
   * @param msg Message to be printed.
   * @param color Color of the message. It can be 'w'(white) for standard messages
   * (default), 'y' (yellow) for warnings, 'r' (red) for errors.
   * @warning This function lives in the real-time thread, so keep it short if overridden.
   */
  virtual void ecPrintCb(const std::string& msg, const char color = 'w') const;
  /**
   * @brief EtherCAT real-time thread state changed pseudo-signal.
   *
   * This function is called when the real-time deadline is missed.
   * This function emulates a signal emit, and it can be overridden to perform the actual
   * signal emition in a Qt context, i.e. in a QObject which inherits from this class.
   * @warning This function lives in the real-time thread, so keep it short if overridden.
   */
  virtual void ecRtThreadStatusChanged(const bool /*active*/) {}

 protected:
  /**
   * @brief Ethercat state check flags bit position enum.
   *
   * The check_state_flags_ is a bitfield object where every bit represents the state of
   * an element. 1 means element is _operational_, 0 that it is not. This enum helps
   * grabbing the right element without remembering its position index.
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
  grabrt::Thread thread_rt_;       /**< Real-time thread. */

  /** @defgroup EthercatUtilities EtherCAT Utilities
   * This group collects all EtherCAT-specific elements in a generic master-slave
   * implemenation.
   * @{
   */
  ec_master_t* master_ptr_        = nullptr; /**< Pointer to ethercat master. */
  ec_master_state_t master_state_ = {};      /**< State of ethercat master. */
  ec_domain_t* domain_ptr_        = nullptr; /**< Pointer to ethercat domain. */
  ec_domain_state_t domain_state_ = {};      /**< State of ethercat domain. */
  ec_slave_config_t* slave_config_ptr_ =
    nullptr; /**< Pointer to ethercat configuration. */
  ec_slave_config_state_t slave_config_state_ =
    {};                                /**< State of ethercat configuration. */
  uint8_t* domain_data_ptr_ = nullptr; /**< Pointer to ethercat domain data. */

  std::bitset<3> check_state_flags_; /**< Container where every bit represents the state
                                      * of an element. 1 means element is _operational_, 0
                                      * that it is not. */
  /** @} */                          // end of EthercatUtilities group

  std::vector<EthercatSlave*> slaves_ptrs_; /**< Vector of pointers to slaves. */
  size_t num_slaves_;                     /**< Number of slaves on the ethercat network */
  uint8_t num_domain_elements_       = 0; /**< Number of elements in ethercat domain. */
  double max_shutdown_wait_time_sec_ = 1; /**< Maximum waiting time to shutdown slaves */

  /**
   * @brief EtherCAT start up function, called once before the cycle begins.
   */
  virtual void ecStartUpFun() {}

  /**
   * @brief EtherCAT main working/loop function, called at every cycle.
   */
  virtual void ecWorkFun() = 0;

  /**
   * @brief EtherCAT emergency exit function, called once on real-time deadline missed.
   */
  virtual void ecEmergencyFun() {}

 private:
  //------- Thread related --------------------------------//

  static void startUpFunWrapper(void* obj);
  static void loopFunWrapper(void* obj);
  static void endFunWrapper(void* obj);
  static void emergencyExitFunWrapper(void* obj);

  void loopFunction();
  void endFunction();
  void emergencyExitFunction();

 private:
  uint8_t initProtocol();
  bool setupEcNtw();

  void checkDomainState();
  void checkMasterState();
  void checkConfigState();

  bool allSlavesReadyToShutDown() const;
  void releaseMaster();

  void getDomainElements(std::vector<ec_pdo_entry_reg_t>& regs) const;

  std::string getAlStateStr(const uint al_state) const;
};

} // end namespace grabec

#endif // ETHERCATMASTER_H
