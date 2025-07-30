#ifndef XELBSSCTIRMA8_H
#define XELBSSCTIRMA8_H

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

#include "ethercatslave.h"
#include "grabec_types.h"

namespace grabec
{

struct DataForSingleActuator
{
  const int16_t& loadcell_;
  const int32_t& encoder_;
  uint8_t& encoder_enable_;

  DataForSingleActuator(const int16_t& loadcell, const int32_t& encoder, uint8_t& encoder_enable)
    : loadcell_(loadcell), encoder_(encoder), encoder_enable_(encoder_enable) {}
};


/**
 * @brief The XelBssctIrma8 class
 */
class XelBssctIrma8 : public virtual EthercatSlave
{
public:
  /**
   * @brief XelBssctIrma8
   * @param slave_position
   */
  XelBssctIrma8(const uint8_t slave_position);
  ~XelBssctIrma8() override;

  /**
   * @brief Slave's main function to be cycled.
   */
  void doWork() override;

  /**
   * @brief Function to specify what to read.
   */
  void readInputs() override final;

  /**
   * @brief Function to specify what to write.
   */
  void writeOutputs() override final;
  /**
   * @brief Function called before shutting down the slave safely.
   */
  void safeExit() override;
  /**
   * @brief Check if slave is ready to be shut down safely.
   * @return _True_ if slave is ready, _false_ otherwise.
   */
  bool isReadyToShutDown() const override;

  /**
   * @brief Output buffer union, i.e. data received from master (read).
   */

  typedef struct
  {
    uint8_t CH0_Enable_Counter_6;
    uint8_t CH1_Enable_Counter_6;
    uint8_t CH0_Enable_Counter_7;
    uint8_t CH1_Enable_Counter_7;
    uint8_t CH0_Enable_Counter_8;
    uint8_t CH1_Enable_Counter_8;
    uint8_t CH0_Enable_Counter_9;
    uint8_t CH1_Enable_Counter_9;

  } CustBufferOut; /**< Output buffer, i.e. data received from master (read). */

  CustBufferOut BufferOut;
  /**
   * @brief Input buffer union, i.e. data sent to master (write).
   */
  typedef struct
  {
    int16_t CH0_Digital_Output_Data_4;
    int16_t CH1_Digital_Output_Data_4;
    int16_t CH2_Digital_Output_Data_4;
    int16_t CH3_Digital_Output_Data_4;
    int16_t CH0_Digital_Output_Data_5;
    int16_t CH1_Digital_Output_Data_5;
    int16_t CH2_Digital_Output_Data_5;
    int16_t CH3_Digital_Output_Data_5;
    int32_t CH0_Count_Data_6;
    int32_t CH1_Count_Data_6;
    int32_t CH0_Count_Data_7;
    int32_t CH1_Count_Data_7;
    int32_t CH0_Count_Data_8;
    int32_t CH1_Count_Data_8;
    int32_t CH0_Count_Data_9;
    int32_t CH1_Count_Data_9;

  } CustBufferIn; /**< Input buffer, i.e. data sent to master (write). */
  CustBufferIn BufferIn;

  // Vector filled by parse_mapping(). Will conatin referencese to the PDOs
  std::vector<DataForSingleActuator> sensor_data_for_actuators_;

protected:
  void initFun() override;

private:
  // This function prepare some strutures to send to the actuators in order to have
  // the encoder, loadcell and encoder enable associated to the corresponding actuator
  // this is done to keep the ifrastructure of functions build in GoldSoloWhistleDrive
  std::vector<DataForSingleActuator> parse_mapping();

  // EasyCAT slave device specific info
  static constexpr uint16_t kDomainEntries_ = 24;
  static constexpr uint8_t kAlias_         = 0;
  static constexpr uint32_t kVendorID_     = 0x000005e1;
  static constexpr uint32_t kProductCode_  = 0x00005fc1;

  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[] = {
    {0x2040, 3, 1}, /**< output PDO: _DC_SYNC_COUNT_CLR_1 */
    {0x0000, 0, 15}, /**< output PDstd::vector<DataForSingleActuator> parse_mapping();O: GAP_2_1 */
    {0x7000, 1, 1}, /**< output PDO: DO_0_2 */
    {0x7000, 2, 1}, /**< output PDO: DO_1_2 */
    {0x7000, 3, 1}, /**< output PDO: DO_2_2 */
    {0x7000, 4, 1}, /**< output PDO: DO_3_2 */
    {0x7000, 5, 1}, /**< output PDO: DO_4_2 */
    {0x7000, 6, 1}, /**< output PDO: DO_5_2 */
    {0x7000, 7, 1}, /**< output PDO: DO_6_2 */
    {0x7000, 8, 1}, /**< output PDO: DO_7_2 */
    {0x7000, 9, 8}, /**< output PDO: GAP_9_2 */
    {0x7010, 1, 1}, /**< output PDO: DO_0_3 */
    {0x7010, 2, 1}, /**< output PDO: DO_1_3 */
    {0x7010, 3, 1}, /**< output PDO: DO_2_3 */
    {0x7010, 4, 1}, /**< output PDO: DO_3_3 */
    {0x7010, 5, 1}, /**< output PDO: DO_4_3 */
    {0x7010, 6, 1}, /**< output PDO: DO_5_3 */
    {0x7010, 7, 1}, /**< output PDO: DO_6_3 */
    {0x7010, 8, 1}, /**< output PDO: DO_7_3 */
    {0x7010, 9, 8}, /**< output PDO: GAP_9_3 */
    {0x7020, 1, 1}, /**< output PDO: Error_Clear_Request_4 */
    {0x0000, 0, 15}, /**< output PDO: GAP_2_4 */
    {0x7030, 1, 1}, /**< output PDO: Error_Clear_Request_5 */
    {0x0000, 0, 15}, /**< output PDO: GAP_2_5 */
    {0x7040, 1, 1}, /**< output PDO: CH0_Enable_Counter_6 */
    {0x7040, 2, 1}, /**< output PDO: CH0_Preset_Enable_6 */
    {0x7040, 3, 1}, /**< output PDO: CH0_Count_Direction_Select_6 */
    {0x7040, 4, 1}, /**< output PDO: CH0_Auxiliary_Function_Request_6 */
    {0x7040, 5, 1}, /**< output PDO: CH0_Enable_Compare_Function_6 */
    {0x7040, 6, 1}, /**< output PDO: CH0_Enable_Compare_Output_Signal_6 */
    {0x7040, 7, 1}, /**< output PDO: CH0_Compare_0_EQUAL_Reset_6 */
    {0x7040, 8, 1}, /**< output PDO: CH0_Compare_1_EQUAL_Reset_6 */
    {0x0000, 0, 2}, /**< output PDO: GAP_9_6 */
    {0x7040, 10, 1}, /**< output PDO: CH0_CarryBorrow_Reset_Request_6 */
    {0x7040, 11, 1}, /**< output PDO: CH0_Preset_Ext_Input_Enable_6 */
    {0x7040, 12, 1}, /**< output PDO: CH0_Enable_AuxFunc_Ext_Input_6 */
    {0x7040, 13, 1}, /**< output PDO: CH0_Preset_Ext_Input_Reset_Request_6 */
    {0x0000, 0, 2}, /**< output PDO: GAP_14_6 */
    {0x7040, 15, 1}, /**< output PDO: CH1_Enable_Counter_6 */
    {0x7040, 16, 1}, /**< output PDO: CH1_Preset_Enable_6 */
    {0x7040, 17, 1}, /**< output PDO: CH1_Count_Direction_Select_6 */
    {0x7040, 18, 1}, /**< output PDO: CH1_Auxiliary_Function_Request_6 */
    {0x7040, 19, 1}, /**< output PDO: CH1_Enable_Compare_Function_6 */
    {0x7040, 20, 1}, /**< output PDO: CH1_Enable_Compare_Output_Signal_6 */
    {0x7040, 21, 1}, /**< output PDO: CH1_Compare_0_EQUAL_Reset_6 */
    {0x7040, 22, 1}, /**< output PDO: CH1_Compare_1_EQUAL_Reset_6 */
    {0x0000, 0, 2}, /**< output PDO: GAP_23_6 */
    {0x7040, 24, 1}, /**< output PDO: CH1_CarryBorrow_Reset_Request_6 */
    {0x7040, 25, 1}, /**< output PDO: CH1_Preset_Ext_Input_Enable_6 */
    {0x7040, 26, 1}, /**< output PDO: CH1_Enable_AuxFunc_Ext_Input_6 */
    {0x7040, 27, 1}, /**< output PDO: CH1_Preset_Ext_Input_Reset_Request_6 */
    {0x0000, 0, 2}, /**< output PDO: GAP_28_6 */
    {0x7050, 1, 1}, /**< output PDO: CH0_Enable_Counter_7 */
    {0x7050, 2, 1}, /**< output PDO: CH0_Preset_Enable_7 */
    {0x7050, 3, 1}, /**< output PDO: CH0_Count_Direction_Select_7 */
    {0x7050, 4, 1}, /**< output PDO: CH0_Auxiliary_Function_Request_7 */
    {0x7050, 5, 1}, /**< output PDO: CH0_Enable_Compare_Function_7 */
    {0x7050, 6, 1}, /**< output PDO: CH0_Enable_Compare_Output_Signal_7 */
    {0x7050, 7, 1}, /**< output PDO: CH0_Compare_0_EQUAL_Reset_7 */
    {0x7050, 8, 1}, /**< output PDO: CH0_Compare_1_EQUAL_Reset_7 */
    {0x0000, 0, 2}, /**< output PDO: GAP_9_7 */
    {0x7050, 10, 1}, /**< output PDO: CH0_CarryBorrow_Reset_Request_7 */
    {0x7050, 11, 1}, /**< output PDO: CH0_Preset_Ext_Input_Enable_7 */
    {0x7050, 12, 1}, /**< output PDO: CH0_Enable_AuxFunc_Ext_Input_7 */
    {0x7050, 13, 1}, /**< output PDO: CH0_Preset_Ext_Input_Reset_Request_7 */
    {0x0000, 0, 2}, /**< output PDO: GAP_14_7 */
    {0x7050, 15, 1}, /**< output PDO: CH1_Enable_Counter_7 */
    {0x7050, 16, 1}, /**< output PDO: CH1_Preset_Enable_7 */
    {0x7050, 17, 1}, /**< output PDO: CH1_Count_Direction_Select_7 */
    {0x7050, 18, 1}, /**< output PDO: CH1_Auxiliary_Function_Request_7 */
    {0x7050, 19, 1}, /**< output PDO: CH1_Enable_Compare_Function_7 */
    {0x7050, 20, 1}, /**< output PDO: CH1_Enable_Compare_Output_Signal_7 */
    {0x7050, 21, 1}, /**< output PDO: CH1_Compare_0_EQUAL_Reset_7 */
    {0x7050, 22, 1}, /**< output PDO: CH1_Compare_1_EQUAL_Reset_7 */
    {0x0000, 0, 2}, /**< output PDO: GAP_23_7 */
    {0x7050, 24, 1}, /**< output PDO: CH1_CarryBorrow_Reset_Request_7 */
    {0x7050, 25, 1}, /**< output PDO: CH1_Preset_Ext_Input_Enable_7 */
    {0x7050, 26, 1}, /**< output PDO: CH1_Enable_AuxFunc_Ext_Input_7 */
    {0x7050, 27, 1}, /**< output PDO: CH1_Preset_Ext_Input_Reset_Request_7 */
    {0x0000, 0, 2}, /**< output PDO: GAP_28_7 */
    {0x7060, 1, 1}, /**< output PDO: CH0_Enable_Counter_8 */
    {0x7060, 2, 1}, /**< output PDO: CH0_Preset_Enable_8 */
    {0x7060, 3, 1}, /**< output PDO: CH0_Count_Direction_Select_8 */
    {0x7060, 4, 1}, /**< output PDO: CH0_Auxiliary_Function_Request_8 */
    {0x7060, 5, 1}, /**< output PDO: CH0_Enable_Compare_Function_8 */
    {0x7060, 6, 1}, /**< output PDO: CH0_Enable_Compare_Output_Signal_8 */
    {0x7060, 7, 1}, /**< output PDO: CH0_Compare_0_EQUAL_Reset_8 */
    {0x7060, 8, 1}, /**< output PDO: CH0_Compare_1_EQUAL_Reset_8 */
    {0x0000, 0, 2}, /**< output PDO: GAP_9_8 */
    {0x7060, 10, 1}, /**< output PDO: CH0_CarryBorrow_Reset_Request_8 */
    {0x7060, 11, 1}, /**< output PDO: CH0_Preset_Ext_Input_Enable_8 */
    {0x7060, 12, 1}, /**< output PDO: CH0_Enable_AuxFunc_Ext_Input_8 */
    {0x7060, 13, 1}, /**< output PDO: CH0_Preset_Ext_Input_Reset_Request_8 */
    {0x0000, 0, 2}, /**< output PDO: GAP_14_8 */
    {0x7060, 15, 1}, /**< output PDO: CH1_Enable_Counter_8 */
    {0x7060, 16, 1}, /**< output PDO: CH1_Preset_Enable_8 */
    {0x7060, 17, 1}, /**< output PDO: CH1_Count_Direction_Select_8 */
    {0x7060, 18, 1}, /**< output PDO: CH1_Auxiliary_Function_Request_8 */
    {0x7060, 19, 1}, /**< output PDO: CH1_Enable_Compare_Function_8 */
    {0x7060, 20, 1}, /**< output PDO: CH1_Enable_Compare_Output_Signal_8 */
    {0x7060, 21, 1}, /**< output PDO: CH1_Compare_0_EQUAL_Reset_8 */
    {0x7060, 22, 1}, /**< output PDO: CH1_Compare_1_EQUAL_Reset_8 */
    {0x0000, 0, 2}, /**< output PDO: GAP_23_8 */
    {0x7060, 24, 1}, /**< output PDO: CH1_CarryBorrow_Reset_Request_8 */
    {0x7060, 25, 1}, /**< output PDO: CH1_Preset_Ext_Input_Enable_8 */
    {0x7060, 26, 1}, /**< output PDO: CH1_Enable_AuxFunc_Ext_Input_8 */
    {0x7060, 27, 1}, /**< output PDO: CH1_Preset_Ext_Input_Reset_Request_8 */
    {0x0000, 0, 2}, /**< output PDO: GAP_28_8 */
    {0x7070, 1, 1}, /**< output PDO: CH0_Enable_Counter_9 */
    {0x7070, 2, 1}, /**< output PDO: CH0_Preset_Enable_9 */
    {0x7070, 3, 1}, /**< output PDO: CH0_Count_Direction_Select_9 */
    {0x7070, 4, 1}, /**< output PDO: CH0_Auxiliary_Function_Request_9 */
    {0x7070, 5, 1}, /**< output PDO: CH0_Enable_Compare_Function_9 */
    {0x7070, 6, 1}, /**< output PDO: CH0_Enable_Compare_Output_Signal_9 */
    {0x7070, 7, 1}, /**< output PDO: CH0_Compare_0_EQUAL_Reset_9 */
    {0x7070, 8, 1}, /**< output PDO: CH0_Compare_1_EQUAL_Reset_9 */
    {0x0000, 0, 2}, /**< output PDO: GAP_9_9 */
    {0x7070, 10, 1}, /**< output PDO: CH0_CarryBorrow_Reset_Request_9 */
    {0x7070, 11, 1}, /**< output PDO: CH0_Preset_Ext_Input_Enable_9 */
    {0x7070, 12, 1}, /**< output PDO: CH0_Enable_AuxFunc_Ext_Input_9 */
    {0x7070, 13, 1}, /**< output PDO: CH0_Preset_Ext_Input_Reset_Request_9 */
    {0x0000, 0, 2}, /**< output PDO: GAP_14_9 */
    {0x7070, 15, 1}, /**< output PDO: CH1_Enable_Counter_9 */
    {0x7070, 16, 1}, /**< output PDO: CH1_Preset_Enable_9 */
    {0x7070, 17, 1}, /**< output PDO: CH1_Count_Direction_Select_9 */
    {0x7070, 18, 1}, /**< output PDO: CH1_Auxiliary_Function_Reques_9 */
    {0x7070, 19, 1}, /**< output PDO: CH1_Enable_Compare_Function_9 */
    {0x7070, 20, 1}, /**< output PDO: CH1_Enable_Compare_Output_Signal_9 */
    {0x7070, 21, 1}, /**< output PDO: CH1_Compare_0_EQUAL_Reset_9 */
    {0x7070, 22, 1}, /**< output PDO: CH1_Compare_1_EQUAL_Reset_9 */
    {0x0000, 0, 2}, /**< output PDO: GAP_23_9 */
    {0x7070, 24, 1}, /**< output PDO: CH1_CarryBorrow_Reset_Request_9 */
    {0x7070, 25, 1}, /**< output PDO: CH1_Preset_Ext_Input_Enable_9 */
    {0x7070, 26, 1}, /**< output PDO: CH1_Enable_AuxFunc_Ext_Input_9 */
    {0x7070, 27, 1}, /**< output PDO: CH1_Preset_Ext_Input_Reset_Request_9 */
    {0x0000, 0, 2}, /**< output PDO: GAP_28_9 */
    {0x2000, 1, 1}, /**< input PDO: _RUN_1 */
    {0x2000, 3, 1}, /**< input PDO: _ERROR_1 */
    {0x2010, 2, 1}, /**< input PDO: _IO_TYER_1 */
    {0x2010, 3, 1}, /**< input PDO: _IO_DEER_1 */
    {0x2010, 5, 1}, /**< input PDO: _IO_RWER_1 */
    {0x2010, 6, 1}, /**< input PDO: _IP_IFER_1 */
    {0x2010, 8, 1}, /**< input PDO: _BPRM_ER_1 */
    {0x2010, 9, 1}, /**< input PDO: _IOPRM_ER_1 */
    {0x2010, 10, 1}, /**< input PDO: _SPPRM_ER_1 */
    {0x2010, 11, 1}, /**< input PDO: _CPPRM_ER_1 */
    {0x2010, 13, 1}, /**< input PDO: _SWDT_ER_1 */
    {0x2010, 15, 1}, /**< input PDO: _IOSIZE_ER_1 */
    {0x2020, 2, 1}, /**< input PDO: _REFRESH_OT_WAR_1 */
    {0x0000, 0, 3}, /**< input PDO: GAP_14_1 */
    {0x2040, 26, 1}, /**< input PDO: _EXT_ERR_FLAG_0SLT_1 */
    {0x2040, 27, 1}, /**< input PDO: _EXT_ERR_FLAG_1SLT_1 */
    {0x2040, 28, 1}, /**< input PDO: _EXT_ERR_FLAG_2SLT_1 */
    {0x2040, 29, 1}, /**< input PDO: _EXT_ERR_FLAG_3SLT_1 */
    {0x2040, 30, 1}, /**< input PDO: _EXT_ERR_FLAG_4SLT_1 */
    {0x2040, 31, 1}, /**< input PDO: _EXT_ERR_FLAG_5SLT_1 */
    {0x2040, 32, 1}, /**< input PDO: _EXT_ERR_FLAG_6SLT_1 */
    {0x2040, 33, 1}, /**< input PDO: _EXT_ERR_FLAG_7SLT_1 */
    {0x0000, 0, 8}, /**< input PDO: GAP_23_1 */
    {0x2030, 5, 16}, /**< input PDO: _REFRESH_MAX_1 */
    {0x2030, 6, 16}, /**< input PDO: _REFRESH_MIN_1 */
    {0x2030, 7, 16}, /**< input PDO: _REFRESH_CUR_1 */
    {0x6000, 1, 1}, /**< input PDO: DI_0_2 */
    {0x6000, 2, 1}, /**< input PDO: DI_1_2 */
    {0x6000, 3, 1}, /**< input PDO: DI_2_2 */
    {0x6000, 4, 1}, /**< input PDO: DI_3_2 */
    {0x6000, 5, 1}, /**< input PDO: DI_4_2 */
    {0x6000, 6, 1}, /**< input PDO: DI_5_2 */
    {0x6000, 7, 1}, /**< input PDO: DI_6_2 */
    {0x6000, 8, 1}, /**< input PDO: DI_7_2 */
    {0x6000, 9, 8}, /**< input PDO: GAP_9_2 */
    {0x6010, 1, 1}, /**< input PDO: DI_0_3 */
    {0x6010, 2, 1}, /**< input PDO: DI_1_3 */
    {0x6010, 3, 1}, /**< input PDO: DI_2_3 */
    {0x6010, 4, 1}, /**< input PDO: DI_3_3 */
    {0x6010, 5, 1}, /**< input PDO: DI_4_3 */
    {0x6010, 6, 1}, /**< input PDO: DI_5_3 */
    {0x6010, 7, 1}, /**< input PDO: DI_6_3 */
    {0x6010, 8, 1}, /**< input PDO: DI_7_3 */
    {0x6010, 9, 8}, /**< input PDO: GAP_9_3 */
    {0x6020, 1, 1}, /**< input PDO: Error_Flag_4 */
    {0x0000, 0, 14}, /**< input PDO: GAP_2_4 */
    {0x6020, 3, 1}, /**< input PDO: Ready_Flag_4 */
    {0x6020, 4, 1}, /**< input PDO: CH0_Activation_Status_4 */
    {0x6020, 5, 1}, /**< input PDO: CH1_Activation_Status_4 */
    {0x6020, 6, 1}, /**< input PDO: CH2_Activation_Status_4 */
    {0x6020, 7, 1}, /**< input PDO: CH3_Activation_Status_4 */
    {0x0000, 0, 4}, /**< input PDO: GAP_8_4 */
    {0x6020, 9, 1}, /**< input PDO: CH0_Error_4 */
    {0x6020, 10, 1}, /**< input PDO: CH1_Error_4 */
    {0x6020, 11, 1}, /**< input PDO: CH2_Error_4 */
    {0x6020, 12, 1}, /**< input PDO: CH3_Error_4 */
    {0x0000, 0, 4}, /**< input PDO: GAP_13_4 */
    {0x6022, 1, 1}, /**< input PDO: CH0_Disconnection_Flag_4 */
    {0x6022, 2, 1}, /**< input PDO: CH1_Disconnection_Flag_4 */
    {0x6022, 3, 1}, /**< input PDO: CH2_Disconnection_Flag_4 */
    {0x6022, 4, 1}, /**< input PDO: CH3_Disconnection_Flag_4 */
    {0x0000, 0, 12}, /**< input PDO: GAP_18_4 */
    {0x6022, 6, 1}, /**< input PDO: CH0_Upper_Alarm_4 */
    {0x6022, 7, 1}, /**< input PDO: CH1_Upper_Alarm_4 */
    {0x6022, 8, 1}, /**< input PDO: CH2_Upper_Alarm_4 */
    {0x6022, 9, 1}, /**< input PDO: CH3_Upper_Alarm_4 */
    {0x0000, 0, 12}, /**< input PDO: GAP_23_4 */
    {0x6022, 11, 1}, /**< input PDO: CH0_Lower_Alarm_4 */
    {0x6022, 12, 1}, /**< input PDO: CH1_Lower_Alarm_4 */
    {0x6022, 13, 1}, /**< input PDO: CH2_Lower_Alarm_4 */
    {0x6022, 14, 1}, /**< input PDO: CH3_Lower_Alarm_4 */
    {0x0000, 0, 12}, /**< input PDO: GAP_28_4 */
    {0x6021, 1, 16}, /**< input PDO: CH0_Digital_Output_Data_4 */
    {0x6021, 2, 16}, /**< input PDO: CH1_Digital_Output_Data_4 */
    {0x6021, 3, 16}, /**< input PDO: CH2_Digital_Output_Data_4 */
    {0x6021, 4, 16}, /**< input PDO: CH3_Digital_Output_Data_4 */
    {0x6030, 1, 1}, /**< input PDO: Error_Flag_5 */
    {0x0000, 0, 14}, /**< input PDO: GAP_2_5 */
    {0x6030, 3, 1}, /**< input PDO: Ready_Flag_5 */
    {0x6030, 4, 1}, /**< input PDO: CH0_Activation_Status_5 */
    {0x6030, 5, 1}, /**< input PDO: CH1_Activation_Status_5 */
    {0x6030, 6, 1}, /**< input PDO: CH2_Activation_Status_5 */
    {0x6030, 7, 1}, /**< input PDO: CH3_Activation_Status_5 */
    {0x0000, 0, 4}, /**< input PDO: GAP_8_5 */
    {0x6030, 9, 1}, /**< input PDO: CH0_Error_5 */
    {0x6030, 10, 1}, /**< input PDO: CH1_Error_5 */
    {0x6030, 11, 1}, /**< input PDO: CH2_Error_5 */
    {0x6030, 12, 1}, /**< input PDO: CH3_Error_5 */
    {0x0000, 0, 4}, /**< input PDO: GAP_13_5 */
    {0x6032, 1, 1}, /**< input PDO: CH0_Disconnection_Flag_5 */
    {0x6032, 2, 1}, /**< input PDO: CH1_Disconnection_Flag_5 */
    {0x6032, 3, 1}, /**< input PDO: CH2_Disconnection_Flag_5 */
    {0x6032, 4, 1}, /**< input PDO: CH3_Disconnection_Flag_5 */
    {0x0000, 0, 12}, /**< input PDO: GAP_18_5 */
    {0x6032, 6, 1}, /**< input PDO: CH0_Upper_Alarm_5 */
    {0x6032, 7, 1}, /**< input PDO: CH1_Upper_Alarm_5 */
    {0x6032, 8, 1}, /**< input PDO: CH2_Upper_Alarm_5 */
    {0x6032, 9, 1}, /**< input PDO: CH3_Upper_Alarm_5 */
    {0x0000, 0, 12}, /**< input PDO: GAP_23_5 */
    {0x6032, 11, 1}, /**< input PDO: CH0_Lower_Alarm_5 */
    {0x6032, 12, 1}, /**< input PDO: CH1_Lower_Alarm_5 */
    {0x6032, 13, 1}, /**< input PDO: CH2_Lower_Alarm_5 */
    {0x6032, 14, 1}, /**< input PDO: CH3_Lower_Alarm_5 */
    {0x0000, 0, 12}, /**< input PDO: GAP_28_5 */
    {0x6031, 1, 16}, /**< input PDO: CH0_Digital_Output_Data_5 */
    {0x6031, 2, 16}, /**< input PDO: CH1_Digital_Output_Data_5 */
    {0x6031, 3, 16}, /**< input PDO: CH2_Digital_Output_Data_5 */
    {0x6031, 4, 16}, /**< input PDO: CH3_Digital_Output_Data_5 */
    {0x6040, 1, 1}, /**< input PDO: CH0_Count_Direction_Status_6 */
    {0x6040, 2, 1}, /**< input PDO: CH0_Preset_Ext_Input_Flag_6 */
    {0x0000, 0, 1}, /**< input PDO: GAP_3_6 */
    {0x6040, 4, 1}, /**< input PDO: CH0_Carry_Flag_6 */
    {0x6040, 5, 1}, /**< input PDO: CH0_Borrow_Flag_6 */
    {0x6040, 6, 1}, /**< input PDO: CH0_Auxiliary_Function_Status_6 */
    {0x6040, 7, 1}, /**< input PDO: CH0_Compare_0_Output_Status_6 */
    {0x6040, 8, 1}, /**< input PDO: CH0_Compare_1_Output_Status_6 */
    {0x0000, 0, 6}, /**< input PDO: GAP_9_6 */
    {0x6040, 10, 1}, /**< input PDO: CH0_Error_Flag_6 */
    {0x6040, 11, 1}, /**< input PDO: Ready_Flag_6 */
    {0x6040, 12, 1}, /**< input PDO: CH1_Count_Direction_Status_6 */
    {0x6040, 13, 1}, /**< input PDO: CH1_Preset_Ext_Input_Flag_6 */
    {0x0000, 0, 1}, /**< input PDO: GAP_14_6 */
    {0x6040, 15, 1}, /**< input PDO: CH1_Carry_Flag_6 */
    {0x6040, 16, 1}, /**< input PDO: CH1_Borrow_Flag_6 */
    {0x6040, 17, 1}, /**< input PDO: CH1_Auxiliary_Function_Status_6 */
    {0x6040, 18, 1}, /**< input PDO: CH1_Compare_0_Output_Status_6 */
    {0x6040, 19, 1}, /**< input PDO: CH1_Compare_1_Output_Status_6 */
    {0x0000, 0, 6}, /**< input PDO: GAP_20_6 */
    {0x6040, 21, 1}, /**< input PDO: CH1_Error_Flag_6 */
    {0x0000, 0, 1}, /**< input PDO: GAP_22_6 */
    {0x6041, 1, 32}, /**< input PDO: CH0_Count_Data_6 */
    {0x6041, 2, 32}, /**< input PDO: CH0_Latch_Count_Data_6 */
    {0x6041, 3, 32}, /**< input PDO: CH0_Sampling_Count_Data_6 */
    {0x6041, 4, 32}, /**< input PDO: CH0_Input_Frequency_Data_6 */
    {0x6041, 5, 32}, /**< input PDO: CH0_RevUnit_Time_Data_6 */
    {0x6041, 6, 32}, /**< input PDO: CH1_Count_Data_6 */
    {0x6041, 7, 32}, /**< input PDO: CH1_Latch_Count_Data_6 */
    {0x6041, 8, 32}, /**< input PDO: CH1_Sampling_Count_Data_6 */
    {0x6041, 9, 32}, /**< input PDO: CH1_Input_Frequency_Data_6 */
    {0x6041, 10, 32}, /**< input PDO: CH1_RevUnit_Time_Data_6 */
    {0x6050, 1, 1}, /**< input PDO: CH0_Count_Direction_Status_7 */
    {0x6050, 2, 1}, /**< input PDO: CH0_Preset_Ext_Input_Flag_7 */
    {0x0000, 0, 1}, /**< input PDO: GAP_3_7 */
    {0x6050, 4, 1}, /**< input PDO: CH0_Carry_Flag_7 */
    {0x6050, 5, 1}, /**< input PDO: CH0_Borrow_Flag_7 */
    {0x6050, 6, 1}, /**< input PDO: CH0_Auxiliary_Function_Status_7 */
    {0x6050, 7, 1}, /**< input PDO: CH0_Compare_0_Output_Status_7 */
    {0x6050, 8, 1}, /**< input PDO: CH0_Compare_1_Output_Status_7 */
    {0x0000, 0, 6}, /**< input PDO: GAP_9_7 */
    {0x6050, 10, 1}, /**< input PDO: CH0_Error_Flag_7 */
    {0x6050, 11, 1}, /**< input PDO: Ready_Flag_7 */
    {0x6050, 12, 1}, /**< input PDO: CH1_Count_Direction_Status_7 */
    {0x6050, 13, 1}, /**< input PDO: CH1_Preset_Ext_Input_Flag_7 */
    {0x0000, 0, 1}, /**< input PDO: GAP_14_7 */
    {0x6050, 15, 1}, /**< input PDO: CH1_Carry_Flag_7 */
    {0x6050, 16, 1}, /**< input PDO: CH1_Borrow_Flag_7 */
    {0x6050, 17, 1}, /**< input PDO: CH1_Auxiliary_Function_Status_7 */
    {0x6050, 18, 1}, /**< input PDO: CH1_Compare_0_Output_Status_7 */
    {0x6050, 19, 1}, /**< input PDO: CH1_Compare_1_Output_Status_7 */
    {0x0000, 0, 6}, /**< input PDO: GAP_20_7 */
    {0x6050, 21, 1}, /**< input PDO: CH1_Error_Flag_7 */
    {0x0000, 0, 1}, /**< input PDO: GAP_22_7 */
    {0x6051, 1, 32}, /**< input PDO: CH0_Count_Data_7 */
    {0x6051, 2, 32}, /**< input PDO: CH0_Latch_Count_Data_7 */
    {0x6051, 3, 32}, /**< input PDO: CH0_Sampling_Count_Data_7 */
    {0x6051, 4, 32}, /**< input PDO: CH0_Input_Frequency_Data_7 */
    {0x6051, 5, 32}, /**< input PDO: CH0_RevUnit_Time_Data_7 */
    {0x6051, 6, 32}, /**< input PDO: CH1_Count_Data_7 */
    {0x6051, 7, 32}, /**< input PDO: CH1_Latch_Count_Data_7 */
    {0x6051, 8, 32}, /**< input PDO: CH1_Sampling_Count_Data_7 */
    {0x6051, 9, 32}, /**< input PDO: CH1_Input_Frequency_Data_7 */
    {0x6051, 10, 32}, /**< input PDO: CH1_RevUnit_Time_Data_7 */
    {0x6060, 1, 1}, /**< input PDO: CH0_Count_Direction_Status_8 */
    {0x6060, 2, 1}, /**< input PDO: CH0_Preset_Ext_Input_Flag_8 */
    {0x0000, 0, 1}, /**< input PDO: GAP_3_8 */
    {0x6060, 4, 1}, /**< input PDO: CH0_Carry_Flag_8 */
    {0x6060, 5, 1}, /**< input PDO: CH0_Borrow_Flag_8 */
    {0x6060, 6, 1}, /**< input PDO: CH0_Auxiliary_Function_Status_8 */
    {0x6060, 7, 1}, /**< input PDO: CH0_Compare_0_Output_Status_8 */
    {0x6060, 8, 1}, /**< input PDO: CH0_Compare_1_Output_Status_8 */
    {0x0000, 0, 6}, /**< input PDO: GAP_9_8 */
    {0x6060, 10, 1}, /**< input PDO: CH0_Error_Flag_8 */
    {0x6060, 11, 1}, /**< input PDO: Ready_Flag_8 */
    {0x6060, 12, 1}, /**< input PDO: CH1_Count_Direction_Status_8 */
    {0x6060, 13, 1}, /**< input PDO: CH1_Preset_Ext_Input_Flag_8 */
    {0x0000, 0, 1}, /**< input PDO: GAP_14_8 */
    {0x6060, 15, 1}, /**< input PDO: CH1_Carry_Flag_8 */
    {0x6060, 16, 1}, /**< input PDO: CH1_Borrow_Flag_8 */
    {0x6060, 17, 1}, /**< input PDO: CH1_Auxiliary_Function_Status_8 */
    {0x6060, 18, 1}, /**< input PDO: CH1_Compare_0_Output_Status_8 */
    {0x6060, 19, 1}, /**< input PDO: CH1_Compare_1_Output_Status_8 */
    {0x0000, 0, 6}, /**< input PDO: GAP_20_8 */
    {0x6060, 21, 1}, /**< input PDO: CH1_Error_Flag_8 */
    {0x0000, 0, 1}, /**< input PDO: GAP_22_8 */
    {0x6061, 1, 32}, /**< input PDO: CH0_Count_Data_8 */
    {0x6061, 2, 32}, /**< input PDO: CH0_Latch_Count_Data_8 */
    {0x6061, 3, 32}, /**< input PDO: CH0_Sampling_Count_Data_8 */
    {0x6061, 4, 32}, /**< input PDO: CH0_Input_Frequency_Data_8 */
    {0x6061, 5, 32}, /**< input PDO: CH0_RevUnit_Time_Data_8 */
    {0x6061, 6, 32}, /**< input PDO: CH1_Count_Data_8 */
    {0x6061, 7, 32}, /**< input PDO: CH1_Latch_Count_Data_8 */
    {0x6061, 8, 32}, /**< input PDO: CH1_Sampling_Count_Data_8 */
    {0x6061, 9, 32}, /**< input PDO: CH1_Input_Frequency_Data_8 */
    {0x6061, 10, 32}, /**< input PDO: CH1_RevUnit_Time_Data_8 */
    {0x6070, 1, 1}, /**< input PDO: CH0_Count_Direction_Status_9 */
    {0x6070, 2, 1}, /**< input PDO: CH0_Preset_Ext_Input_Flag_9 */
    {0x0000, 0, 1}, /**< input PDO: GAP_3_9 */
    {0x6070, 4, 1}, /**< input PDO: CH0_Carry_Flag_9 */
    {0x6070, 5, 1}, /**< input PDO: CH0_Borrow_Flag_9 */
    {0x6070, 6, 1}, /**< input PDO: CH0_Auxiliary_Function_Status_9 */
    {0x6070, 7, 1}, /**< input PDO: CH0_Compare_0_Output_Status_9 */
    {0x6070, 8, 1}, /**< input PDO: CH0_Compare_1_Output_Status_9 */
    {0x0000, 0, 6}, /**< input PDO: GAP_9_9 */
    {0x6070, 10, 1}, /**< input PDO: CH0_Error_Flag_9 */
    {0x6070, 11, 1}, /**< input PDO: Ready_Flag_9 */
    {0x6070, 12, 1}, /**< input PDO: CH1_Count_Direction_Status_9 */
    {0x6070, 13, 1}, /**< input PDO: CH1_Preset_Ext_Input_Flag_9 */
    {0x0000, 0, 1}, /**< input PDO: GAP_14_9 */
    {0x6070, 15, 1}, /**< input PDO: CH1_Carry_Flag_9 */
    {0x6070, 16, 1}, /**< input PDO: CH1_Borrow_Flag_9 */
    {0x6070, 17, 1}, /**< input PDO: CH1_Auxiliary_Function_Status_9 */
    {0x6070, 18, 1}, /**< input PDO: CH1_Compare_0_Output_Status_9 */
    {0x6070, 19, 1}, /**< input PDO: CH1_Compare_1_Output_Status_9 */
    {0x0000, 0, 6}, /**< input PDO: GAP_20_9 */
    {0x6070, 21, 1}, /**< input PDO: CH1_Error_Flag_9 */
    {0x0000, 0, 1}, /**< input PDO: GAP_22_9 */
    {0x6071, 1, 32}, /**< input PDO: CH0_Count_Data_9 */
    {0x6071, 2, 32}, /**< input PDO: CH0_Latch_Count_Data_9 */
    {0x6071, 3, 32}, /**< input PDO: CH0_Sampling_Count_Data_9 */
    {0x6071, 4, 32}, /**< input PDO: CH0_Input_Frequency_Data_9 */
    {0x6071, 5, 32}, /**< input PDO: CH0_RevUnit_Time_Data_9 */
    {0x6071, 6, 32}, /**< input PDO: CH1_Count_Data_9 */
    {0x6071, 7, 32}, /**< input PDO: CH1_Latch_Count_Data_9 */
    {0x6071, 8, 32}, /**< input PDO: CH1_Sampling_Count_Data_9 */
    {0x6071, 9, 32}, /**< input PDO: CH1_Input_Frequency_Data_9 */
    {0x6071, 10, 32}, /**< input PDO: CH1_RevUnit_Time_Data_9 */
  };

  // Ethercat utilities, describing memory position of input and output PDOs
  // stack.
  static constexpr ec_pdo_info_t kPDOs_[] = {
    {0x1680, 2, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 0},
    {0x1600, 9, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 2},
    {0x1601, 9, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 11},
    {0x1602, 2, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 20},
    {0x1603, 2, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 22},
    {0x1604, 28, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 24},
    {0x1605, 28, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 52},
    {0x1606, 28, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 80},
    {0x1607, 28, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 108},
    {0x1a80, 26, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 136},
    {0x1a00, 9, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 162},
    {0x1a01, 9, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 171},
    {0x1a02, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 180},
    {0x1a03, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 212},
    {0x1a04, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 244},
    {0x1a05, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 276},
    {0x1a06, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 308},
    {0x1a07, 32, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + 340},
  };


  // Ethercat utilities, synchronization information
  static constexpr ec_sync_info_t kSyncs_[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 9, const_cast<ec_pdo_info_t*>(kPDOs_) + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 9, const_cast<ec_pdo_info_t*>(kPDOs_) + 9, EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, nullptr, static_cast<ec_watchdog_mode_t>(0)}};

  // Useful ethercat struct to store input PDOs memory offset
  struct OffsetIn
  {
    unsigned int CH0_Digital_Output_Data_4;
    unsigned int CH1_Digital_Output_Data_4;
    unsigned int CH2_Digital_Output_Data_4;
    unsigned int CH3_Digital_Output_Data_4;
    unsigned int CH0_Digital_Output_Data_5;
    unsigned int CH1_Digital_Output_Data_5;
    unsigned int CH2_Digital_Output_Data_5;
    unsigned int CH3_Digital_Output_Data_5;
    unsigned int CH0_Count_Data_6;
    unsigned int CH1_Count_Data_6;
    unsigned int CH0_Count_Data_7;
    unsigned int CH1_Count_Data_7;
    unsigned int CH0_Count_Data_8;
    unsigned int CH1_Count_Data_8;
    unsigned int CH0_Count_Data_9;
    unsigned int CH1_Count_Data_9;
  } offset_in_;

  // Useful ethercat struct to store output PDOs memory offset
  struct OffsetOut
  {
    unsigned int CH0_Enable_Counter_6;
    unsigned int CH0_Enable_Counter_7;
    unsigned int CH0_Enable_Counter_8;
    unsigned int CH0_Enable_Counter_9;
    unsigned int CH1_Enable_Counter_6;
    unsigned int CH1_Enable_Counter_7;
    unsigned int CH1_Enable_Counter_8;
    unsigned int CH1_Enable_Counter_9;
  } offset_out_;

  sync_dc_t xelb_sync_dc_ = {true, 0x300, 2000000,0,0,0};

  ec_pdo_entry_reg_t domain_registers_[kDomainEntries_ +1 ]; // ethercat utility
};

} // end namespace grabec

#endif // XELBSSCTIRMA8_H
