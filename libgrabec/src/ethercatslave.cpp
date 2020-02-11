/**
 * @file ethercatslave.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 30 May 2019
 * @brief File containing definitions of functions and class declared in ethercatslave.h.
 */

#include "ethercatslave.h"

namespace grabec {

EthercatSlave::~EthercatSlave() {} // necessary for pure abstract destructor

//--------- Public functions ---------------------------------------------------------//

void EthercatSlave::Init(uint8_t* domain_data_ptr)
{
  SetDomainDataPtr(domain_data_ptr);
  InitFun();
}

RetVal EthercatSlave::Configure(ec_master_t* master_ptr, ec_slave_config_t** config_ptr)
{
  if (!(*config_ptr = ecrt_master_slave_config(master_ptr, alias_, position_, vendor_id_,
                                               product_code_)))
  {
    EcPrintCb("Configuring device: " + GetRetValStr(ECONFIG), 'r');
    return ECONFIG;
  }
  EcPrintCb("Configuring device: " + GetRetValStr(OK));

  if (ecrt_slave_config_pdos(*config_ptr, EC_END, slave_sync_ptr_))
  {
    EcPrintCb("Configuring PDOs: " + GetRetValStr(ECONFIG), 'r');
    return ECONFIG;
  }
  EcPrintCb("Configuring PDOs: " + GetRetValStr(OK));

  RetVal ret = SdoRequests(*config_ptr);
  EcPrintCb("Creating SDO request: " + GetRetValStr(ret), ret ? 'r' : 'w');
  return ret;
}

ec_pdo_entry_reg_t EthercatSlave::GetDomainRegister(const uint8_t index) const
{
  return domain_registers_ptr_[index];
}

//--------- Protected virtual functions ----------------------------------------------//

RetVal EthercatSlave::SdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = nullptr;

  if (sdo_ptr != nullptr)
    return EINV;
  if (config_ptr == nullptr)
    return ECONFIG;
  return OK;
}

void EthercatSlave::EcPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  if (color == 'w')
    printf("[EthercatSlave] %s\n", msg.c_str());
  else
    PrintColor(color, "[EthercatSlave] %s", msg.c_str());
}

//--------- Private functions --------------------------------------------------------//

void EthercatSlave::SetDomainDataPtr(uint8_t* domain_data_ptr)
{
  domain_data_ptr_ = domain_data_ptr;
}

} // end namespace grabec
