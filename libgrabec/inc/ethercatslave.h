/**
 * @file ethercatslave.h
 * @author Edoardo Idà, Simone Comari
 * @date 18 Sep 2018
 * @brief File containing EtherCAT slaves to be included in the GRAB EtherCAT library.
 */

#ifndef GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
#define GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H

#include "ecrt.h"
#include "types.h"

namespace grabec
{

/**
 * @brief EtherCAT slave abstract interface.
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
  virtual ~EthercatSlave() = 0;

  /**
   * @brief Initial function called before the cyclical task begins.
   * @param _domain_data_ptr
   */
  void Init(uint8_t* _domain_data_ptr);

  /**
   * @brief Configure
   * @param[in] master_ptr
   * @param[out] config_ptr
   */
  RetVal Configure(ec_master_t* master_ptr, ec_slave_config_t* config_ptr);

  /**
   * @brief Slave's main function to be cycled.
   */
  virtual void DoWork() = 0;

  /**
   * @brief Function to specify what to read.
   */
  virtual void ReadInputs() = 0;

  /**
   * @brief Function to specify what to write.
   */
  virtual void WriteOutputs() = 0;

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

private:
  /////////////////////////////////////////////////
  /// EtherCAT-specific utilities
  /////////////////////////////////////////////////
  uint8_t num_domain_entries_;
  uint16_t alias_;
  uint16_t position_;
  uint32_t vendor_id_;
  uint32_t product_code_;
  uint8_t id_;

  uint8_t* domain_data_ptr_;
  ec_pdo_entry_reg_t* domain_registers_ptr_;
  ec_pdo_entry_info_t* slave_pdo_entries_ptr_;
  ec_pdo_info_t* slave_pdos_ptr_;
  ec_sync_info_t* slave_sync_ptr_;
  /////////////////////////////////////////////////
  /// End EtherCAT-specific utilities
  /////////////////////////////////////////////////

  /**
   * @brief SetDomainDataPtr
   * @param _domain_data_ptr
   */
  void SetDomainDataPtr(uint8_t* _domain_data_ptr);

protected:
  /**
   * @brief InitFun
   */
  virtual void InitFun() = 0;

  /**
   * @brief SdoRequests
   * @param config_ptr
   * @return
   */
  virtual RetVal SdoRequests(ec_slave_config_t* config_ptr);
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
