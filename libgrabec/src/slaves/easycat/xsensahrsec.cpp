//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project ethercat_xsens_config_mdo.prj
//     Easy Configurator XML ethercat_xsens_config_mdo.xml
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include <cstring>

#include "slaves/easycat/xsensahrsec.h"

namespace grabec
{
// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t Ethercat_xsens_config_mdoSlave::kPdoEntries_[];
constexpr ec_pdo_info_t Ethercat_xsens_config_mdoSlave::kPDOs_[];
constexpr ec_sync_info_t Ethercat_xsens_config_mdoSlave::kSyncs_[];

Ethercat_xsens_config_mdoSlave::Ethercat_xsens_config_mdoSlave(const uint8_t slave_position)
{
  alias_ = kAlias_;
  vendor_id_ = kVendorID_;
  product_code_ = kProductCode_;
  num_domain_entries_ = kDomainEntries_;
  position_ = slave_position;
  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[0].index, kPdoEntries_[0].subindex,
                          &offset_out_.CMD_ID, nullptr};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[1].index, kPdoEntries_[1].subindex,
                          &offset_out_.byte1, nullptr};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[2].index, kPdoEntries_[2].subindex,
                          &offset_out_.byte2, nullptr};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[3].index, kPdoEntries_[3].subindex,
                          &offset_out_.byte3, nullptr};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[4].index, kPdoEntries_[4].subindex,
                          &offset_out_.byte4, nullptr};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[5].index, kPdoEntries_[5].subindex,
                          &offset_out_.CMD_ID_check, nullptr};
  domain_registers_[6] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[6].index, kPdoEntries_[6].subindex,
                          &offset_out_.CMD_servo, nullptr};
  domain_registers_[7] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[7].index, kPdoEntries_[7].subindex,
                          &offset_out_.trajectory_type_servo, nullptr};
  domain_registers_[8] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[8].index, kPdoEntries_[8].subindex,
                          &offset_out_.speed_type_servo, nullptr};
  domain_registers_[9] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[9].index, kPdoEntries_[9].subindex,
                          &offset_in_.ang_eul_pitch, nullptr};
  domain_registers_[10] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[10].index, kPdoEntries_[10].subindex,
                          &offset_in_.ang_eul_roll, nullptr};
  domain_registers_[11] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[11].index, kPdoEntries_[11].subindex,
                          &offset_in_.ang_eul_yaw, nullptr};
  domain_registers_[12] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[12].index, kPdoEntries_[12].subindex,
                          &offset_in_.acc_x, nullptr};
  domain_registers_[13] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[13].index, kPdoEntries_[13].subindex,
                          &offset_in_.acc_y, nullptr};
  domain_registers_[14] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[14].index, kPdoEntries_[14].subindex,
                          &offset_in_.acc_z, nullptr};
  domain_registers_[15] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[15].index, kPdoEntries_[15].subindex,
                          &offset_in_.rate_of_turn_x, nullptr};
  domain_registers_[16] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[16].index, kPdoEntries_[16].subindex,
                          &offset_in_.rate_of_turn_y, nullptr};
  domain_registers_[17] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[17].index, kPdoEntries_[17].subindex,
                          &offset_in_.rate_of_turn_z, nullptr};
  domain_registers_[18] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[18].index, kPdoEntries_[18].subindex,
                          &offset_in_.quaternion_q1, nullptr};
  domain_registers_[19] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[19].index, kPdoEntries_[19].subindex,
                          &offset_in_.quaternion_q2, nullptr};
  domain_registers_[20] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[20].index, kPdoEntries_[20].subindex,
                          &offset_in_.quaternion_q3, nullptr};
  domain_registers_[21] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[21].index, kPdoEntries_[21].subindex,
                          &offset_in_.quaternion_q4, nullptr};
  domain_registers_[22] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[22].index, kPdoEntries_[22].subindex,
                          &offset_in_.status_word, nullptr};
  domain_registers_[23] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[23].index, kPdoEntries_[23].subindex,
                          &offset_in_.selftest_result, nullptr};
  domain_registers_[24] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[24].index, kPdoEntries_[24].subindex,
                          &offset_in_.state_servo, nullptr};
  domain_registers_[25] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[25].index, kPdoEntries_[25].subindex,
                          &offset_in_.CMD_servo_check, nullptr};
  domain_registers_[26] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[26].index, kPdoEntries_[26].subindex,
                          &offset_in_.status_byte, nullptr};
  domain_registers_[27] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[27].index, kPdoEntries_[27].subindex,
                          &offset_in_.resp_CMD_ID, nullptr};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);
}

Ethercat_xsens_config_mdoSlave::~Ethercat_xsens_config_mdoSlave()
{
  /*
   * Your code here..
   */
}

void Ethercat_xsens_config_mdoSlave::DoWork()
{
  /*
   * Your code here..
   */
}

void Ethercat_xsens_config_mdoSlave::ReadInputs()
{
  // This is the way we can read the PDOs, according to ecrt.h
  int32_t ang_eul_pitch = EC_READ_S32(domain_data_ptr_ + offset_in_.ang_eul_pitch);
  memcpy(&BufferIn.Cust.ang_eul_pitch, &ang_eul_pitch, sizeof(float));
  int32_t ang_eul_roll = EC_READ_S32(domain_data_ptr_ + offset_in_.ang_eul_roll);
  memcpy(&BufferIn.Cust.ang_eul_roll, &ang_eul_roll, sizeof(float));
  int32_t ang_eul_yaw = EC_READ_S32(domain_data_ptr_ + offset_in_.ang_eul_yaw);
  memcpy(&BufferIn.Cust.ang_eul_yaw, &ang_eul_yaw, sizeof(float));
  int32_t acc_x = EC_READ_S32(domain_data_ptr_ + offset_in_.acc_x);
  memcpy(&BufferIn.Cust.acc_x, &acc_x, sizeof(float));
  int32_t acc_y = EC_READ_S32(domain_data_ptr_ + offset_in_.acc_y);
  memcpy(&BufferIn.Cust.acc_y, &acc_y, sizeof(float));
  int32_t acc_z = EC_READ_S32(domain_data_ptr_ + offset_in_.acc_z);
  memcpy(&BufferIn.Cust.acc_z, &acc_z, sizeof(float));
  int32_t rate_of_turn_x = EC_READ_S32(domain_data_ptr_ + offset_in_.rate_of_turn_x);
  memcpy(&BufferIn.Cust.rate_of_turn_x, &rate_of_turn_x, sizeof(float));
  int32_t rate_of_turn_y = EC_READ_S32(domain_data_ptr_ + offset_in_.rate_of_turn_y);
  memcpy(&BufferIn.Cust.rate_of_turn_y, &rate_of_turn_y, sizeof(float));
  int32_t rate_of_turn_z = EC_READ_S32(domain_data_ptr_ + offset_in_.rate_of_turn_z);
  memcpy(&BufferIn.Cust.rate_of_turn_z, &rate_of_turn_z, sizeof(float));
  int32_t quaternion_q1 = EC_READ_S32(domain_data_ptr_ + offset_in_.quaternion_q1);
  memcpy(&BufferIn.Cust.quaternion_q1, &quaternion_q1, sizeof(float));
  int32_t quaternion_q2 = EC_READ_S32(domain_data_ptr_ + offset_in_.quaternion_q2);
  memcpy(&BufferIn.Cust.quaternion_q2, &quaternion_q2, sizeof(float));
  int32_t quaternion_q3 = EC_READ_S32(domain_data_ptr_ + offset_in_.quaternion_q3);
  memcpy(&BufferIn.Cust.quaternion_q3, &quaternion_q3, sizeof(float));
  int32_t quaternion_q4 = EC_READ_S32(domain_data_ptr_ + offset_in_.quaternion_q4);
  memcpy(&BufferIn.Cust.quaternion_q4, &quaternion_q4, sizeof(float));
  BufferIn.Cust.status_word = EC_READ_U32(domain_data_ptr_ + offset_in_.status_word);
  BufferIn.Cust.selftest_result = EC_READ_U16(domain_data_ptr_ + offset_in_.selftest_result);
  BufferIn.Cust.state_servo = EC_READ_U8(domain_data_ptr_ + offset_in_.state_servo);
  BufferIn.Cust.CMD_servo_check = EC_READ_U8(domain_data_ptr_ + offset_in_.CMD_servo_check);
  BufferIn.Cust.status_byte = EC_READ_U8(domain_data_ptr_ + offset_in_.status_byte);
  BufferIn.Cust.resp_CMD_ID = EC_READ_U8(domain_data_ptr_ + offset_in_.resp_CMD_ID);
}

void Ethercat_xsens_config_mdoSlave::WriteOutputs()
{
  // This is the way we can write the PDOs, according to ecrt.h
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CMD_ID, BufferOut.Cust.CMD_ID);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.byte1, BufferOut.Cust.byte1);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.byte2, BufferOut.Cust.byte2);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.byte3, BufferOut.Cust.byte3);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.byte4, BufferOut.Cust.byte4);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CMD_ID_check, BufferOut.Cust.CMD_ID_check);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CMD_servo, BufferOut.Cust.CMD_servo);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.trajectory_type_servo, BufferOut.Cust.trajectory_type_servo);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.speed_type_servo, BufferOut.Cust.speed_type_servo);
}

void Ethercat_xsens_config_mdoSlave::SafeExit()
{
  /*
   * Your code here..
   */
}

bool Ethercat_xsens_config_mdoSlave::IsReadyToShutDown() const
{
  /*
   * Your code here..
   * Return bool accordingly..
   */
  return true;
}

void Ethercat_xsens_config_mdoSlave::InitFun()
{
  /*
   * Your code here..
   */
}

} // end namespace grabec
