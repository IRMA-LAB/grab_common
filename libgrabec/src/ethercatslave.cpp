/**
 * @file ethercatslave.cpp
 * @author Simone Comari, Edoardo Idà
 * @date Jan 2022
 * @brief File containing definitions of functions and class declared in ethercatslave.h.
 */

#include "ethercatslave.h"

namespace grabec {

EthercatSlave::~EthercatSlave() {} // necessary for pure abstract destructor

//--------- Public functions ---------------------------------------------------------//

void EthercatSlave::init(uint8_t* domain_data_ptr)
{
  setDomainDataPtr(domain_data_ptr);
  initFun();
}

RetVal EthercatSlave::configure(ec_master_t* master_ptr, ec_slave_config_t** config_ptr)
{
  if (!(*config_ptr = ecrt_master_slave_config(master_ptr, alias_, position_, vendor_id_,
                                               product_code_)))
  {
    ecPrintCb("Configuring device: " + getRetValStr(ECONFIG), 'r');
    return ECONFIG;
  }
  ecPrintCb("Configuring device: " + getRetValStr(OK));

  if (ecrt_slave_config_pdos(*config_ptr, EC_END, slave_sync_ptr_))
  {
    ecPrintCb("Configuring PDOs: " + getRetValStr(ECONFIG), 'r');
    return ECONFIG;
  }
  ecPrintCb("Configuring PDOs: " + getRetValStr(OK));

  RetVal ret = sdoRequests(*config_ptr);
  ecPrintCb("Creating SDO request: " + getRetValStr(ret), ret ? 'r' : 'w');
  return ret;
}

ec_pdo_entry_reg_t EthercatSlave::getDomainRegister(const uint8_t index) const
{
  return domain_registers_ptr_[index];
}

//--------- Protected virtual functions ----------------------------------------------//

RetVal EthercatSlave::sdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = nullptr;

  if (sdo_ptr != nullptr)
    return EINV;
  if (config_ptr == nullptr)
    return ECONFIG;
  return OK;
}

void EthercatSlave::ecPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  if (color == 'w')
    printf("[EthercatSlave] %s\n", msg.c_str());
  else
    printColor(color, "[EthercatSlave] %s", msg.c_str());
}

//--------- Private functions --------------------------------------------------------//

void EthercatSlave::setDomainDataPtr(uint8_t* domain_data_ptr)
{
  domain_data_ptr_ = domain_data_ptr;
}

} // end namespace grabec
