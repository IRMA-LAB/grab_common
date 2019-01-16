#include <QString>
#include <QtTest>

#include "types.h"
#include "slaves/easycat/TestEasyCAT1_slave.h"
#include "slaves/easycat/TestEasyCAT2_slave.h"
#include "ethercatmaster.h"

class MinimalMaster : public virtual grabec::EthercatMaster
{
public:
  MinimalMaster();
  ~MinimalMaster() {}

private:
  // Ethercat related
  grabec::TestEasyCAT1Slave* easycat1_;
  grabec::TestEasyCAT2Slave* easycat2_;

  void StartUpFunction() override final {}
  void LoopFunction() override final;
};

MinimalMaster::MinimalMaster()
{
  // Setup RT thread
  grabec::RtThreadsParams params; // default values
  SetThreadsParams(params);

  // Configure network
  quint8 slave_pos = 0;
  easycat1_ = new grabec::TestEasyCAT1Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat1_);
  easycat2_ = new grabec::TestEasyCAT2Slave(slave_pos);
  slaves_ptrs_.push_back(easycat2_);
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();
}

void MinimalMaster::LoopFunction()
{
  static uint16_t analog = 0;

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->ReadInputs(); // read pdos

  // Read/write easyCATs' PDOs
  easycat1_->BufferOut.Cust.LedRed = easycat2_->BufferIn.Cust.LedRed;
  easycat1_->BufferOut.Cust.LedOrange = easycat2_->BufferIn.Cust.LedOrange;
  easycat1_->BufferOut.Cust.LedBlue = easycat2_->BufferIn.Cust.LedYellow;
  easycat2_->BufferOut.Cust.Analog = easycat1_->BufferIn.Cust.Analog;
  easycat1_->BufferOut.Cust.FloatVarOut = easycat1_->BufferIn.Cust.Analog + 0.1234f;
  easycat1_->BufferOut.Cust.DoubleVarOut = easycat1_->BufferIn.Cust.Analog + 0.6789;
  if (analog != easycat1_->BufferIn.Cust.Analog)
  {
    analog = easycat1_->BufferIn.Cust.Analog;
    std::cout << analog << " " << easycat1_->BufferIn.Cust.FloatVarIn << " "
              << easycat1_->BufferIn.Cust.DoubleVarIn << std::endl;
  }

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->WriteOutputs(); // write all the necessary pdos
}

class LibgrabecTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  void testDispRetVal();

  void testEasyCAT();
};

void LibgrabecTest::testDispRetVal()
{
  grabec::RetVal ret = grabec::OK;
  grabec::DispRetVal(ret, "Displaying value %u. This should be black: ", ret);
  ret = grabec::ECONFIG;
  grabec::DispRetVal(ret, "Displaying value %u. This should be red: ", ret);
}

void LibgrabecTest::testEasyCAT()
{
  MinimalMaster master;
  master.Start();

  sleep(10);
}

QTEST_APPLESS_MAIN(LibgrabecTest)

#include "libgrabec_test.moc"
