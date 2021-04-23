#ifndef EASYCATMASTER_H
#define EASYCATMASTER_H

#include "ethercatmaster.h"
//#include "slaves/easycat/Elmo_minimal_slave_slave.h"
#include "slaves/goldsolowhistledrive.h"

class EasycatMaster : public virtual grabec::EthercatMaster
{
public:
  EasycatMaster(const uint32_t cycle_time_nsec=1000000U);  // default cycle time = 1ms
  ~EasycatMaster() override;

  /**
   * @brief An example of how an asynchronous external call from main should be handled.
   *
   * Any external asynchronous call implementation should be included in mutex when
   * accessing a variable shared with the RT thread.
   */
  void extAsyncCallExampleFun();
  // Your functions are to be written from here
  void extAsyncCallDisplayAll();
  void extAsyncCallInitiateEnableProcedure();
  void preliminaryOperations();

  uint8_t flag_enable=0;

  // to here

private:
  //grabec::Elmo_DriveSlave* easycat_ptr_;
  grabec::GoldSoloWhistleDrive* easycat_ptr_;

  //-------- Pseudo-signals from EthercatMaster base class (live in RT thread) ------//
  void EcStateChangedCb(const std::bitset<3>& new_state) override final;
  void EcRtThreadStatusChanged(const bool active) override final;

  // Ethercat related
  void EcWorkFun() override final;      // lives in the RT thread
  void EcEmergencyFun() override final {} // lives in the RT thread
};

#endif // EASYCATMASTER_H

