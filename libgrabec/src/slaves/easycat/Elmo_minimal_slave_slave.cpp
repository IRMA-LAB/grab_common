//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
//     Easy Configurator project Elmo_Drive.prj
//     Easy Configurator XML Elmo_Drive.xml
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include <cstring>

#include "slaves/easycat/Elmo_minimal_slave_slave.h"

namespace grabec
{
// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t Elmo_DriveSlave::kPdoEntries_[];
constexpr ec_pdo_info_t Elmo_DriveSlave::kPDOs_[];
constexpr ec_sync_info_t Elmo_DriveSlave::kSyncs_[];

Elmo_DriveSlave::Elmo_DriveSlave(const uint8_t slave_position)
{
    alias_              = kAlias;
    position_           = slave_position;
    vendor_id_          = kVendorID;
    product_code_       = kProductCode;
    num_domain_entries_ = kDomainEntries;
    id_                 = 0;

    domain_registers_[0]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kControlWordIdx,
                            kControlWordSubIdx,
                            &offset_out_.control_word,
                            nullptr};
    domain_registers_[1]  = {alias_,     position_,     vendor_id_,           product_code_,
                            kOpModeIdx, kOpModeSubIdx, &offset_out_.op_mode, nullptr};
    domain_registers_[2]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kTargetTorqueIdx,
                            kTargetTorqueSubIdx,
                            &offset_out_.target_torque,
                            nullptr};
    domain_registers_[3]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kTargetPosIdx,
                            kTargetPosSubIdx,
                            &offset_out_.target_position,
                            nullptr};
    domain_registers_[4]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kTargetVelIdx,
                            kTargetVelSubIdx,
                            &offset_out_.target_velocity,
                            nullptr};
    domain_registers_[5]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kStatusWordIdx,
                            kStatusWordSubIdx,
                            &offset_in_.status_word,
                            nullptr};
    domain_registers_[6]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kDisplayOpModeIdx,
                            kDisplayOpModeSubIdx,
                            &offset_in_.display_op_mode,
                            nullptr};
    domain_registers_[7]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kPosActualValueIdx,
                            kPosActualValueSubIdx,
                            &offset_in_.position_actual_value,
                            nullptr};
    domain_registers_[8]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kVelActualValueIdx,
                            kVelActualValueSubIdx,
                            &offset_in_.velocity_actual_value,
                            nullptr};
    domain_registers_[9]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kTorqueActualValueIdx,
                            kTorqueActualValueSubIdx,
                            &offset_in_.torque_actual_value,
                            nullptr};
    domain_registers_[10]  = {alias_,
                            position_,
                            vendor_id_,
                            product_code_,
                            kAnalogIdx,
                            kAnalogSubIdx,
                            &offset_in_.analog_input,
                            nullptr};
    domain_registers_[11] = {alias_,
                             position_,
                             vendor_id_,
                             product_code_,
                             kDigInIndex,
                             kDigInSubIndex,
                             &offset_in_.digital_inputs,
                             nullptr};
    domain_registers_[12] = {alias_,
                             position_,
                             vendor_id_,
                             product_code_,
                             kAuxPosActualValueIdx,
                             kAuxPosActualValueSubIdx,
                             &offset_in_.aux_pos_actual_value,
                             nullptr};

    domain_registers_ptr_  = domain_registers_;
    slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
    slave_pdos_ptr_        = const_cast<ec_pdo_info_t*>(kPDOs_);
    slave_sync_ptr_        = const_cast<ec_sync_info_t*>(kSyncs_);
//  alias_ = kAlias_;
//  vendor_id_ = kVendorID_;
//  product_code_ = kProductCode_;
//  num_domain_entries_ = kDomainEntries_;
//  position_ = slave_position;
//  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[0].index, kPdoEntries_[0].subindex,
//                          &offset_out_.Control_word, nullptr};
//  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[1].index, kPdoEntries_[1].subindex,
//                          &offset_out_.Mode_of_operation, nullptr};
//  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[2].index, kPdoEntries_[2].subindex,
//                          &offset_out_.Homing_method, nullptr};
//  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[3].index, kPdoEntries_[3].subindex,
//                          &offset_out_.Target_Torque, nullptr};
//  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[4].index, kPdoEntries_[4].subindex,
//                          &offset_out_.Target_Position, nullptr};
//  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[5].index, kPdoEntries_[5].subindex,
//                          &offset_out_.Target_Velocity, nullptr};
//  domain_registers_[6] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[6].index, kPdoEntries_[6].subindex,
//                          &offset_out_.Digital_Outputs, nullptr};
//  domain_registers_[7] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[7].index, kPdoEntries_[7].subindex,
//                          &offset_in_.Status_word, nullptr};
//  domain_registers_[8] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[8].index, kPdoEntries_[8].subindex,
//                          &offset_in_.Mode_of_operation_display, nullptr};
//  domain_registers_[9] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[9].index, kPdoEntries_[9].subindex,
//                          &offset_in_.Position_actual_value, nullptr};
//  domain_registers_[10] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[10].index, kPdoEntries_[10].subindex,
//                          &offset_in_.Velocity_actual_value, nullptr};
//  domain_registers_[11] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[11].index, kPdoEntries_[11].subindex,
//                          &offset_in_.Torque_actual_value, nullptr};
//  domain_registers_[12] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[12].index, kPdoEntries_[12].subindex,
//                          &offset_in_.Digital_Inputs, nullptr};
//  domain_registers_[13] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[13].index, kPdoEntries_[13].subindex,
//                          &offset_in_.Analog_Input_1, nullptr};
//  domain_registers_[14] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[14].index, kPdoEntries_[14].subindex,
//                          &offset_in_.Auxiliary_position_actual_value, nullptr};
//  domain_registers_[15] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[15].index, kPdoEntries_[15].subindex,
//                          &offset_in_.Current_actual_value, nullptr};
//  domain_registers_[16] = {alias_, position_, vendor_id_, product_code_,
//                          kPdoEntries_[16].index, kPdoEntries_[16].subindex,
//                          &offset_in_.Extended_Inputs_Value, nullptr};

//  domain_registers_ptr_ = domain_registers_;
//  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
//  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
//  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);
}

Elmo_DriveSlave::~Elmo_DriveSlave()
{
  /*
   * Your code here..
   */
}

void Elmo_DriveSlave::DoWork()
{
  /*
   * Your code here..
   */
}

void Elmo_DriveSlave::ReadInputs()
{
  // This is the way we can read the PDOs, according to ecrt.h
//  BufferIn.Cust.Status_word = EC_READ_U16(domain_data_ptr_ + offset_in_.Status_word);
//  BufferIn.Cust.Mode_of_operation_display = EC_READ_S8(domain_data_ptr_ + offset_in_.Mode_of_operation_display);
//  BufferIn.Cust.Position_actual_value = EC_READ_S32(domain_data_ptr_ + offset_in_.Position_actual_value);
//  BufferIn.Cust.Velocity_actual_value = EC_READ_S32(domain_data_ptr_ + offset_in_.Velocity_actual_value);
//  BufferIn.Cust.Torque_actual_value = EC_READ_S16(domain_data_ptr_ + offset_in_.Torque_actual_value);
//  BufferIn.Cust.Digital_Inputs = EC_READ_S32(domain_data_ptr_ + offset_in_.Digital_Inputs);
//  BufferIn.Cust.Analog_Input_1 = EC_READ_S16(domain_data_ptr_ + offset_in_.Analog_Input_1);
//  BufferIn.Cust.Auxiliary_position_actual_value = EC_READ_S32(domain_data_ptr_ + offset_in_.Auxiliary_position_actual_value);
//  BufferIn.Cust.Current_actual_value = EC_READ_S16(domain_data_ptr_ + offset_in_.Current_actual_value);
//  BufferIn.Cust.Extended_Inputs_Value = EC_READ_U32(domain_data_ptr_ + offset_in_.Extended_Inputs_Value);
    input_pdos_.status_word     = EC_READ_U16(domain_data_ptr_ + offset_in_.status_word);
    input_pdos_.display_op_mode = EC_READ_S8(domain_data_ptr_ + offset_in_.display_op_mode);
    input_pdos_.pos_actual_value =
      EC_READ_S32(domain_data_ptr_ + offset_in_.position_actual_value);
    input_pdos_.vel_actual_value =
      EC_READ_S32(domain_data_ptr_ + offset_in_.velocity_actual_value);
    input_pdos_.torque_actual_value =
      EC_READ_S16(domain_data_ptr_ + offset_in_.torque_actual_value);
    input_pdos_.digital_inputs = EC_READ_U32(domain_data_ptr_ + offset_in_.digital_inputs);
    input_pdos_.aux_pos_actual_value =
      EC_READ_S32(domain_data_ptr_ + offset_in_.aux_pos_actual_value);
    input_pdos_.analog_input =
      EC_READ_S16(domain_data_ptr_ + offset_in_.analog_input);
}

void Elmo_DriveSlave::WriteOutputs()
{
  // This is the way we can write the PDOs, according to ecrt.h
//  EC_WRITE_U16(domain_data_ptr_ + offset_out_.Control_word, BufferOut.Cust.Control_word);
//  EC_WRITE_S8(domain_data_ptr_ + offset_out_.Mode_of_operation, BufferOut.Cust.Mode_of_operation);
//  EC_WRITE_S8(domain_data_ptr_ + offset_out_.Homing_method, BufferOut.Cust.Homing_method);
//  EC_WRITE_S16(domain_data_ptr_ + offset_out_.Target_Torque, BufferOut.Cust.Target_Torque);
//  EC_WRITE_S32(domain_data_ptr_ + offset_out_.Target_Position, BufferOut.Cust.Target_Position);
//  EC_WRITE_S32(domain_data_ptr_ + offset_out_.Target_Velocity, BufferOut.Cust.Target_Velocity);
//  EC_WRITE_U32(domain_data_ptr_ + offset_out_.Digital_Outputs, BufferOut.Cust.Digital_Outputs);
    EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word,
                 output_pdos_.control_word.to_ulong());
    EC_WRITE_S8(domain_data_ptr_ + offset_out_.op_mode, output_pdos_.op_mode);
  //  if (drive_state_ == ST_OPERATION_ENABLED || drive_state_ == ST_SWITCHED_ON)
  //  {
  //    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_position,
  //                 output_pdos_.target_position);
  //    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_velocity,
  //                 output_pdos_.target_velocity);
  //    EC_WRITE_S16(domain_data_ptr_ + offset_out_.target_torque,
  //                 output_pdos_.target_torque);
  //  }
}

void Elmo_DriveSlave::SafeExit()
{
  /*
   * Your code here..
   */
}

bool Elmo_DriveSlave::IsReadyToShutDown() const
{
  /*
   * Your code here..
   * Return bool accordingly..
   */
  return true;
}

void Elmo_DriveSlave::InitFun()
{
  /*
   * Your code here..
   */
}

} // end namespace grabec
