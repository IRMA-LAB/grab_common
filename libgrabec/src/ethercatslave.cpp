/**
 * @file ethercatslave.cpp
 * @author Simone Comari
 * @date 18 Sep 2018
 * @brief File containing definitions of functions and class declared in ethercatslave.h.
 */

#include "ethercatslave.h"

namespace grabec
{
/////////////////////////////////////////////////
/// Public methods
/////////////////////////////////////////////////

EthercatSlave::~EthercatSlave() {}  // necessary for pure abstract destructor

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
    DispRetVal(ECONFIG, "[EthercatSlave]\tConfiguring device... ");
    return ECONFIG;
  }
  DispRetVal(OK, "[EthercatSlave]\tConfiguring slave device... ");

  if (ecrt_slave_config_pdos(*config_ptr, EC_END, slave_sync_ptr_))
  {
    DispRetVal(ECONFIG, "[EthercatSlave]\tConfiguring PDOs... ");
    return ECONFIG;
  }
  DispRetVal(OK, "[EthercatSlave]\tConfiguring PDOs... ");

  RetVal ret = SdoRequests(*config_ptr);
  DispRetVal(ret, "[EthercatSlave]\tCreating SDO request... ");
  return ret;
}

ec_pdo_entry_reg_t EthercatSlave::GetDomainRegister(uint8_t index) const
{
  return domain_registers_ptr_[index];
}

/////////////////////////////////////////////////
/// Protected methods
/////////////////////////////////////////////////

RetVal EthercatSlave::SdoRequests(ec_slave_config_t* config_ptr)  // virtual
{
  static ec_sdo_request_t* sdo_ptr = NULL;

  if (sdo_ptr != NULL)
    return EINV;
  if (config_ptr == NULL)
    return ECONFIG;
  return OK;
}

/////////////////////////////////////////////////
/// Private methods
/////////////////////////////////////////////////

void EthercatSlave::SetDomainDataPtr(uint8_t* domain_data_ptr)
{
  domain_data_ptr_ = domain_data_ptr;
}

} // end namespace grabec
