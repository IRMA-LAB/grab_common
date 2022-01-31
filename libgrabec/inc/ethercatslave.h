/**
 * @file ethercatslave.h
 * @author Edoardo Idà, Simone Comari
 * @date Jan 2022
 * @brief File containing EtherCAT slaves to be included in the GRAB EtherCAT library.
 */

#ifndef GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
#define GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H

#include "ecrt.h"
#include "grabec_types.h"

/**
 * @brief Namespace for GRAB EtherCAT library.
 */
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
   * @brief EthercatSlave constructor.
   */
  EthercatSlave() {}
  virtual ~EthercatSlave() = 0; // pure virtual

  /**
   * @brief Initial function called before the cyclical task begins.
   * @param[in] domain_data_ptr EtherCAT domain data pointer.
   */
  void init(uint8_t* domain_data_ptr);

  /**
   * @brief Configure EtherCAT slave.
   * @param[in] master_ptr Pointer to EtherCAT master.
   * @param[out] config_ptr Pointer to EtherCAT configuration.
   * @return 0 if configuration was successful, a strictly positive number otherwise.
   */
  RetVal configure(ec_master_t* master_ptr, ec_slave_config_t** config_ptr);

  /**
   * @brief The main working function called at every cycle, after reading input and
   * before writing outputs to the network.
   */
  virtual void doWork() {}
  /**
   * @brief Read input PDOs.
   */
  virtual void readInputs() = 0; // pure virtual
  /**
   * @brief Write output PDOs.
   */
  virtual void writeOutputs() = 0; // pure virtual
  /**
   * @brief Function called before shutting down the slave.
   */
  virtual void safeExit() {}
  /**
   * @brief Check if slave is ready to be shut down safely.
   * @return _True_ if slave is ready, _false_ otherwise.
   */
  virtual bool isReadyToShutDown() const { return true; }

  /**
   * @brief Get EtherCAT domain register.
   * @param[in] index Index of inquired domain register.
   * @return EtherCAT domain register.
   */
  ec_pdo_entry_reg_t getDomainRegister(const uint8_t index) const;

  /**
   * @brief Get total number of domain entries.
   * @return Total number of domain entries.
   */
  uint8_t getDomainEntriesNum() const { return num_domain_entries_; }

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
  ec_sync_info_t* slave_sync_ptr_;             /**< Pointer to ethercat sync(?) */
  /** @} */                                    // end of EthercatUtilities group

  /**
   * @brief SdoRequests
   * @param[in] config_ptr
   * @return
   * @note This is protected despite being virtual because we may want to use base
   * definition in derived class.
   */
  virtual RetVal sdoRequests(ec_slave_config_t* config_ptr);
  /**
   * @brief Initialization function, called on object instantiation.
   */
  virtual void initFun() {}
  /**
   * @brief EtherCAT information print pseudo-signal.
   *
   * This function emulates a signal emit, and it can be overridden to perform the actual
   * signal emition in a Qt context, i.e. in a QObject which inherits from this class.
   * @param[in] msg Message to be printed.
   * @param[in] color Color of the message. It can be 'w'(white) for standard messages
   * (default), 'y' (yellow) for warnings, 'r' (red) for errors.
   * @warning This function lives in the real-time thread, so keep it short if overridden.
   */
  virtual void ecPrintCb(const std::string& msg, const char color = 'w') const;

 private:
  void setDomainDataPtr(uint8_t* domain_data_ptr);
};

} // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_ETHERCATSLAVE_H
