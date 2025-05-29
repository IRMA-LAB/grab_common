//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project XEL-BSSCT_EtherCAT_Slave(MDP)_Rev2.prj
//     Easy Configurator XML XEL-BSSCT_EtherCAT_Slave(MDP)_Rev2.xml
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include <cstring>

#include "slaves/xelbssctirma8.h"

namespace grabec
{
// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t XelBssctIrma8::kPdoEntries_[];
constexpr ec_pdo_info_t XelBssctIrma8::kPDOs_[];
constexpr ec_sync_info_t XelBssctIrma8::kSyncs_[];

XelBssctIrma8::XelBssctIrma8(const uint8_t slave_position)
{
  alias_ = kAlias_;
  vendor_id_ = kVendorID_;
  product_code_ = kProductCode_;
  num_domain_entries_ = kDomainEntries_;
  position_ = slave_position;
  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_,
                                0x6021, 1,
                                &offset_in_.CH0_Digital_Output_Data_4, nullptr};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_,
                                0x6021, 2,
                                &offset_in_.CH1_Digital_Output_Data_4, nullptr};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_,
                                0x6021, 3,
                                &offset_in_.CH2_Digital_Output_Data_4, nullptr};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_,
                                0x6021, 4,
                                &offset_in_.CH3_Digital_Output_Data_4, nullptr};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_,
                                0x6031, 1,
                                &offset_in_.CH0_Digital_Output_Data_5, nullptr};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_,
                                0x6031, 2,
                                &offset_in_.CH1_Digital_Output_Data_5, nullptr};
  domain_registers_[6] = {alias_, position_, vendor_id_, product_code_,
                                0x6031, 3,
                                &offset_in_.CH2_Digital_Output_Data_5, nullptr};
  domain_registers_[7] = {alias_, position_, vendor_id_, product_code_,
                                0x6031, 4,
                                &offset_in_.CH3_Digital_Output_Data_5, nullptr};
  domain_registers_[8] = {alias_, position_, vendor_id_, product_code_,
                                0x7040, 15,
                                &offset_out_.CH1_Enable_Counter_6, nullptr};
  domain_registers_[9] = {alias_, position_, vendor_id_, product_code_,
                                0x7050, 15,
                                &offset_out_.CH1_Enable_Counter_7, nullptr};
  domain_registers_[10] = {alias_, position_, vendor_id_, product_code_,
                                0x7060, 15,
                                &offset_out_.CH1_Enable_Counter_8, nullptr};
  domain_registers_[11] = {alias_, position_, vendor_id_, product_code_,
                                0x7070, 15,
                                &offset_out_.CH1_Enable_Counter_9, nullptr};
  domain_registers_[12] = {alias_, position_, vendor_id_, product_code_,
                                0x7040, 1,
                                &offset_out_.CH0_Enable_Counter_6, nullptr};
  domain_registers_[13] = {alias_, position_, vendor_id_, product_code_,
                                0x7050, 1,
                                &offset_out_.CH0_Enable_Counter_7, nullptr};
  domain_registers_[14] = {alias_, position_, vendor_id_, product_code_,
                                0x7060, 1,
                                &offset_out_.CH0_Enable_Counter_8, nullptr};
  domain_registers_[15] = {alias_, position_, vendor_id_, product_code_,
                                0x7070, 1,
                                &offset_out_.CH0_Enable_Counter_9, nullptr};
  domain_registers_[16] = {alias_, position_, vendor_id_, product_code_,
                                0x6041, 1,
                                &offset_in_.CH0_Count_Data_6, nullptr};
  domain_registers_[17] = {alias_, position_, vendor_id_, product_code_,
                                0x6041, 6,
                                &offset_in_.CH1_Count_Data_6, nullptr};
  domain_registers_[18] = {alias_, position_, vendor_id_, product_code_,
                                0x6051, 1,
                                &offset_in_.CH0_Count_Data_7, nullptr};
  domain_registers_[19] = {alias_, position_, vendor_id_, product_code_,
                                0x6051, 6,
                                &offset_in_.CH1_Count_Data_7, nullptr};
  domain_registers_[20] = {alias_, position_, vendor_id_, product_code_,
                                0x6061, 1,
                                &offset_in_.CH0_Count_Data_8, nullptr};
  domain_registers_[21] = {alias_, position_, vendor_id_, product_code_,
                                0x6061, 6,
                                &offset_in_.CH1_Count_Data_8, nullptr};
  domain_registers_[22] = {alias_, position_, vendor_id_, product_code_,
                                0x6071, 1,
                                &offset_in_.CH0_Count_Data_9, nullptr};
  domain_registers_[23] = {alias_, position_, vendor_id_, product_code_,
                                0x6071, 6,
                                &offset_in_.CH1_Count_Data_9, nullptr};
  domain_registers_[24] = {0, 0, 0, 0, 0, 0, 0, nullptr};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);
}

XelBssctIrma8::~XelBssctIrma8()
{
  /*
   * Your code here..
   */
}

void XelBssctIrma8::doWork()
{
  /*
   * Your code here..
   */
}

void XelBssctIrma8::readInputs()
{
  // This is the way we can read the PDOs, according to ecrt.h

  BufferIn.CH0_Digital_Output_Data_4 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH0_Digital_Output_Data_4);
  BufferIn.CH1_Digital_Output_Data_4 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH1_Digital_Output_Data_4);
  BufferIn.CH2_Digital_Output_Data_4 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH2_Digital_Output_Data_4);
  BufferIn.CH3_Digital_Output_Data_4 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH3_Digital_Output_Data_4);
  BufferIn.CH0_Digital_Output_Data_5 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH0_Digital_Output_Data_5);
  BufferIn.CH1_Digital_Output_Data_5 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH1_Digital_Output_Data_5);
  BufferIn.CH2_Digital_Output_Data_5 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH2_Digital_Output_Data_5);
  BufferIn.CH3_Digital_Output_Data_5 = EC_READ_S16(domain_data_ptr_ + offset_in_.CH3_Digital_Output_Data_5);

  BufferIn.CH0_Count_Data_6 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH0_Count_Data_6);

  BufferIn.CH1_Count_Data_6 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH1_Count_Data_6);

  BufferIn.CH0_Count_Data_7 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH0_Count_Data_7);

  BufferIn.CH1_Count_Data_7 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH1_Count_Data_7);

  BufferIn.CH0_Count_Data_8 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH0_Count_Data_8);

  BufferIn.CH1_Count_Data_8 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH1_Count_Data_8);

  BufferIn.CH0_Count_Data_9 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH0_Count_Data_9);

  BufferIn.CH1_Count_Data_9 = EC_READ_S32(domain_data_ptr_ + offset_in_.CH1_Count_Data_9);

}

void XelBssctIrma8::writeOutputs()
{
  // This is the way we can write the PDOs, according to ecrt.h

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH0_Enable_Counter_6, BufferOut.CH0_Enable_Counter_6);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH1_Enable_Counter_6, BufferOut.CH1_Enable_Counter_6);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH0_Enable_Counter_7, BufferOut.CH0_Enable_Counter_7);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH1_Enable_Counter_7, BufferOut.CH1_Enable_Counter_7);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH0_Enable_Counter_8, BufferOut.CH0_Enable_Counter_8);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH1_Enable_Counter_8, BufferOut.CH1_Enable_Counter_8);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH0_Enable_Counter_9, BufferOut.CH0_Enable_Counter_9);

  EC_WRITE_U8(domain_data_ptr_ + offset_out_.CH1_Enable_Counter_9, BufferOut.CH1_Enable_Counter_9);

}

void XelBssctIrma8::safeExit()
{
  /*
   * Your code here..
   */
}

bool XelBssctIrma8::isReadyToShutDown() const
{
  /*
   * Your code here..
   * Return bool accordingly..
   */
  return true;
}

void XelBssctIrma8::initFun()
{
  /*
   * Your code here..
   */
}

} // end namespace grabec
