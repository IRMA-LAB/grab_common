#include "ethercatslave.h"

namespace grabec
{

EthercatSlave::~EthercatSlave() {}

/////////////////////////////////////////////////
/// Public methods
/////////////////////////////////////////////////

void EthercatSlave::Init(uint8_t* _domain_data_ptr)
{
  SetDomainDataPtr(_domain_data_ptr);
  InitFun();
}

int EthercatSlave::Configure(ec_master_t* master_ptr, ec_slave_config_t* config_ptr)
{
  if (!(config_ptr = ecrt_master_slave_config(master_ptr, alias_, position_, vendor_id_,
                                              product_code_)))
  {
    DispError(ECONFIG, "Configuring slave device... ");
    return ECONFIG;
  }
  DispError(OK, "Configuring slave device... ");

  if (ecrt_slave_config_pdos(config_ptr, EC_END, slave_sync_ptr_))
  {
    DispError(ECONFIG, "Configuring PDOs... ");
    return ECONFIG;
  }
  DispError(OK, "Configuring PDOs... ");

  int ret = SdoRequests(config_ptr);
  DispError(ret, "Creating SDO request... ");
  return ret;
}

ec_pdo_entry_reg_t EthercatSlave::GetDomainRegister(uint8_t index) const
{
  return domain_registers_ptr_[index];
}

/////////////////////////////////////////////////
/// Private methods
/////////////////////////////////////////////////

void EthercatSlave::SetDomainDataPtr(uint8_t* _domain_data_ptr)
{
  domain_data_ptr_ = _domain_data_ptr;
}

/////////////////////////////////////////////////
/// Protected methods
/////////////////////////////////////////////////

int EthercatSlave::SdoRequests(ec_slave_config_t* config_ptr)
{
  static ec_sdo_request_t* sdo_ptr = NULL;

  if (sdo_ptr != NULL)
    return EINV;
  if (config_ptr == NULL)
    return ECONFIG;
  return OK;
}

} // end namespace grabec
