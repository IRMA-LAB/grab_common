#include "easycatmaster.h"

EasycatMaster::EasycatMaster(const uint32_t cycle_time_nsec/*=1000000U*/)
{
  // This is the pointer to the custom EasyCat slave you generated. Use it to access its
  // PDOs. E.g.
  // easycat_ptr_->BufferIn.Cust.your_read_field (SLAVE -> MASTER)
  // easycat_ptr_->BufferOut.Cust.your_write_field (MASTER -> SLAVE)
  //easycat_ptr_ = new grabec::Elmo_DriveSlave(0);
  easycat_ptr_ = new grabec::GoldSoloWhistleDrive(0,0);

  // Add it to slave list and configure the network
  slaves_ptrs_.push_back(easycat_ptr_);
  num_slaves_ = slaves_ptrs_.size();
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();
  // Set real-time cycle time in nanoseconds
  threads_params_.cycle_time_nsec = cycle_time_nsec;
}

EasycatMaster::~EasycatMaster()
{
  // Stop RT thread before removing slaves
  thread_rt_.Stop();

  // Delete robot components (i.e. ethercat slaves)
  delete easycat_ptr_;
}

void EasycatMaster::extAsyncCallExampleFun()
{
  pthread_mutex_lock(&mutex_);

  // Put you code to be executed only once here!!!!

  pthread_mutex_unlock(&mutex_);
}

// Put your function implementation from here...

void EasycatMaster::extAsyncCallDisplayAll()
{
    pthread_mutex_lock(&mutex_);
    grabec::GSWDriveInPdos input_loc = easycat_ptr_->GetDriveStatus();
    std::cout << "status word: " << input_loc.status_word  << std::endl;
    std::cout << "display op mode: " << input_loc.display_op_mode<< std::endl;
    std::cout << "position actual value: " << input_loc.pos_actual_value << std::endl;
    std::cout << "velocity actual value: " << input_loc.vel_actual_value << std::endl;
    std::cout << "torque actual value: " << input_loc.torque_actual_value << std::endl;
    std::cout << "digital inputs: " << input_loc.digital_inputs << std::endl;
    std::cout << "auxiliary position actual value: " << input_loc.aux_pos_actual_value << std::endl;
    std::cout << "analog input: " << input_loc.analog_input << std::endl;
    std::cout << "status word literal: " <<  easycat_ptr_->GetDriveStateStr(input_loc.status_word) << std::endl;

    pthread_mutex_unlock(&mutex_);
}

void EasycatMaster::extAsyncCallInitiateEnableProcedure()
{
    pthread_mutex_lock(&mutex_);

    flag_enable = 1;

    pthread_mutex_unlock(&mutex_);
}

void EasycatMaster::preliminaryOperations()
{
    std::cout << "Preliminary operations start" << std::endl;

    // Do everything you want BEFORE master start

    std::cout << "Preliminary operations end" << std::endl;

}

// to here....

//--------- Ethercat related private functions --------------------------------------//

void EasycatMaster::EcStateChangedCb(const std::bitset<3>& new_state)
{
  if (new_state.all())
    PrintColor('g', "[EasycatMaster] EtherCAT network state: valid");
  else if (new_state.any())
    PrintColor('y', "[EasycatMaster] EtherCAT network state: invalid (%s)",
               new_state.to_string().c_str());
  else
    PrintColor('r', "[EasycatMaster] EtherCAT network state: invalid");
}

void EasycatMaster::EcRtThreadStatusChanged(const bool active)
{
  if (active)
    PrintColor('g', "[EasycatMaster] EtherCAT network: active");
  else
    PrintColor('r', "[EasycatMaster] EtherCAT network: inactive");
}

void EasycatMaster::EcWorkFun()
{
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->ReadInputs(); // read pdos

  // Your cyclic code starts here

  switch (flag_enable)
    {
    case 1:
      std::cout << "start enabling procedure" << std::endl;
      flag_enable = 2;
      break;
    case 2:
      std::cout << "continuing enabling procedure" << std::endl;
      flag_enable = 3;
      break;
    case 3:
      std::cout << "ending enabling procedure" << std::endl;
      flag_enable = 0;
      break;
    default:
      break;
    }

  // and ends here..

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->WriteOutputs(); // write all the necessary pdos
}
