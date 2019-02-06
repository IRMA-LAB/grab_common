/**
 * @file ethercatslave.h
 * @author Edoardo Idà, Simone Comari
 * @date 05 Feb 2019
 * @brief File containing EtherCAT slaves to be included in the GRAB EtherCAT library.
 */

#ifndef GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
#define GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H

#include "ecrt.h"
#include "types.h"

namespace grabec {

/**
 * @brief EtherCAT slave pure abstract class interface.
 *
 * This class is used as base class for any EtherCAT slave. All the effort can be put in
 * the design of our specific slave, reminding that the ethercat slave interface requires
 * to overload some functions. These functions are the ones marked as virtual.
 */
class EthercatSlave
{
 public:
  /**
   * @brief EthercatSlave
   */
  EthercatSlave() {}
  virtual ~EthercatSlave() = 0; // pure virtual

  /**
   * @brief Initial function called before the cyclical task begins.
   * @param _domain_data_ptr
   */
  void Init(uint8_t* domain_data_ptr);

  /**
   * @brief Configure
   * @param[in] master_ptr
   * @param[out] config_ptr
   */
  RetVal Configure(ec_master_t* master_ptr, ec_slave_config_t** config_ptr);

  /**
   * @brief DoWork
   */
  virtual void DoWork() {}
  /**
   * @brief ReadInputs
   */
  virtual void ReadInputs() = 0; // pure virtual
  /**
   * @brief WriteOutputs
   */
  virtual void WriteOutputs() = 0; // pure virtual
  /**
   * @brief SafeExit
   */
  virtual void SafeExit() {}
  /**
   * @brief IsReadyToShutDown
   * @return
   */
  virtual bool IsReadyToShutDown() const { return true; }

  /**
   * @brief GetDomainRegister
   * @param index
   * @return
   */
  ec_pdo_entry_reg_t GetDomainRegister(uint8_t index) const;

  /**
   * @brief GetDomainEntriesNum
   * @return
   */
  uint8_t GetDomainEntriesNum() const { return num_domain_entries_; }

 protected:
  /**
   * @addtogroup EthercatUtilities
   * @{
   */
  uint8_t num_domain_entries_; /**< Number of ethercat domain entries. */
  uint16_t alias_;             /**< Position of master(?) */
  uint16_t position_;          /**< Position of slave wrt master's. */
  uint32_t vendor_id_;         /**< Vendor unique identifier. */
  uint32_t product_code_;      /**< Product code (unique). */
  id_t id_;                    /**< Slave unique identifier. */

  uint8_t* domain_data_ptr_;                 /**< Pointer to ethercat domain data. */
  ec_pdo_entry_reg_t* domain_registers_ptr_; /**< Pointer to ethercat domain registers. */
  ec_pdo_entry_info_t* slave_pdo_entries_ptr_; /**< Pointer to ethercat PDOs entries. */
  ec_pdo_info_t* slave_pdos_ptr_;              /**< Pointer to ethercat PDOs. */
  ec_sync_info_t* slave_sync_ptr_;             /**< Pointer to ethercat sunc(?) */
  /** @} */                                    // end of EthercatUtilities group

  /**
   * @brief SdoRequests
   * @param config_ptr
   * @return
   * @note This is protected despite being virtual because we may want to use base
   * definition in derived class.
   */
  virtual RetVal SdoRequests(ec_slave_config_t* config_ptr);
  /**
   * @brief InitFun
   */
  virtual void InitFun() {}
  /**
   * @brief EcPrintCb
   * @param msg
   * @param color
   */
  virtual void EcPrintCb(const std::string& msg, const char color = 'w') const;

 private:
  void SetDomainDataPtr(uint8_t* domain_data_ptr);
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
