#include <QString>
#include <QtTest>

#include "types.h"
#include "slaves/easycat/TestEasyCAT1_slave.h"
#include "slaves/easycat/TestEasyCAT2_slave.h"
#include "slaves/goldsolowhistledrive.h"
#include "ethercatmaster.h"

class MinimalMaster : public virtual grabec::EthercatMaster
{
public:
  MinimalMaster();
  ~MinimalMaster();

private:
  // Ethercat related
  grabec::TestEasyCAT1Slave* easycat1_ptr_;
  grabec::TestEasyCAT2Slave* easycat2_ptr_;

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
  easycat1_ptr_ = new grabec::TestEasyCAT1Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat1_ptr_);
  easycat2_ptr_ = new grabec::TestEasyCAT2Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat2_ptr_);
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();
}

MinimalMaster::~MinimalMaster()
{
  free(easycat1_ptr_);
  free(easycat2_ptr_);
}

void MinimalMaster::LoopFunction()
{
  static uint16_t analog = 0;

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->ReadInputs(); // read pdos

  // Read/write easyCATs' PDOs
  easycat1_ptr_->BufferOut.Cust.LedRed = easycat2_ptr_->BufferIn.Cust.LedRed;
  easycat1_ptr_->BufferOut.Cust.LedOrange = easycat2_ptr_->BufferIn.Cust.LedOrange;
  easycat1_ptr_->BufferOut.Cust.LedBlue = easycat2_ptr_->BufferIn.Cust.LedYellow;
  easycat2_ptr_->BufferOut.Cust.Analog = easycat1_ptr_->BufferIn.Cust.Analog;
  easycat1_ptr_->BufferOut.Cust.FloatVarOut =
    easycat1_ptr_->BufferIn.Cust.Analog + 0.1234f;
  easycat1_ptr_->BufferOut.Cust.DoubleVarOut =
    easycat1_ptr_->BufferIn.Cust.Analog + 0.6789;
  if (analog != easycat1_ptr_->BufferIn.Cust.Analog)
  {
    analog = easycat1_ptr_->BufferIn.Cust.Analog;
    std::cout << analog << " " << easycat1_ptr_->BufferIn.Cust.FloatVarIn << " "
              << easycat1_ptr_->BufferIn.Cust.DoubleVarIn << std::endl;
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

  sleep(1);
}

QTEST_APPLESS_MAIN(LibgrabecTest)

#include "libgrabec_test.moc"
