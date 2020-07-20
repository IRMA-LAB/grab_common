import os
import argparse
import sys


def gen_qt_pro_file(ofilepath, grab_common_dir, easycat_class_filename):    
    # Write Qt project file.
    f = open(ofilepath, 'w')
    f.write("""TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

GRAB_COMMON_DIR = %s

HEADERS += \\
        easycatmaster.h \\
        $$GRAB_COMMON_DIR/libgrabec/inc/slaves/easycat/%s.h

SOURCES += \\
        main.cpp \\
        easycatmaster.cpp \\
        $$GRAB_COMMON_DIR/libgrabec/src/slaves/easycat/%s.cpp

INCLUDEPATH += $$GRAB_COMMON_DIR \

TARGET = MinimalEasyCAT

DEFINES += SRCDIR=\\\"$$PWD/\\\"

# GRAB Ethercat lib
unix:!macx: LIBS += -L$$GRAB_COMMON_DIR/libgrabec/lib/ -lgrabec
INCLUDEPATH += $$GRAB_COMMON_DIR/libgrabec \
    $$GRAB_COMMON_DIR/libgrabec/inc
DEPENDPATH += $$GRAB_COMMON_DIR/libgrabec
unix:!macx: PRE_TARGETDEPS += $$GRAB_COMMON_DIR/libgrabec/lib/libgrabec.a

# GRAB Real-time lib
unix:!macx: LIBS += -L$$GRAB_COMMON_DIR/libgrabrt/lib/ -lgrabrt
INCLUDEPATH += $$GRAB_COMMON_DIR/libgrabrt \
    $$GRAB_COMMON_DIR/libgrabrt/inc
DEPENDPATH += $$GRAB_COMMON_DIR/libgrabrt
unix:!macx: PRE_TARGETDEPS += $$GRAB_COMMON_DIR/libgrabrt/lib/libgrabrt.a

# EtherCAT lib
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

LIBS += -pthread
""" % ((grab_common_dir, ) + (easycat_class_filename,) * 2))
    f.close()


def gen_main(ofilepath, cycle_time_ms):
    # Write minimal main with simple keyboard control
    f = open(ofilepath, 'w')
    f.write("""#include <iostream>
#include <string>

#include "easycatmaster.h"

using namespace std;

int main()
{
  EasycatMaster master(%d);
  master.Start();
  sleep(2);

  cout << "Q to quit (case insensitive).. ";
  char key;
  cin >> key;
  while (key != 'q' && key != 'Q')
  {
    switch (key)
      {
      case 'e':
      case 'E':
        master.extAsyncCallExampleFun();
        break;
      default:
        cout << "Pressed key: " << key << ". Press Q to quit (case insensitive).. ";
        break;
      }
    cin >> key;
  }
  cout << "exiting.." << endl;
  return 0;
}""" % (cycle_time_ms * 1000000))
    f.close()


def gen_easycat_master_header(ofilepath, easycat_class_filename, easycat_class_name):
    f = open(ofilepath, 'w')
    f.write("""#ifndef EASYCATMASTER_H
#define EASYCATMASTER_H

#include "ethercatmaster.h"
#include "slaves/easycat/%s.h"

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

private:
  grabec::%s* easycat_ptr_;

  //-------- Pseudo-signals from EthercatMaster base class (live in RT thread) ------//
  void EcStateChangedCb(const std::bitset<3>& new_state) override final;
  void EcRtThreadStatusChanged(const bool active) override final;

  // Ethercat related
  void EcWorkFun() override final;      // lives in the RT thread
  void EcEmergencyFun() override final {} // lives in the RT thread
};

#endif // EASYCATMASTER_H

""" % (easycat_class_filename, easycat_class_name))
    f.close()


def gen_easycat_master_source(ofilepath, easycat_class_name):
    f = open(ofilepath, 'w')
    f.write("""#include "easycatmaster.h"

EasycatMaster::EasycatMaster(const uint32_t cycle_time_nsec/*=1000000U*/)
{
  // This is the pointer to the custom EasyCat slave you generated. Use it to access its
  // PDOs. E.g.
  // easycat_ptr_->BufferIn.Cust.your_read_field (SLAVE -> MASTER)
  // easycat_ptr_->BufferOut.Cust.your_write_field (MASTER -> SLAVE)
  easycat_ptr_ = new grabec::%s(0);

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
  // Here you can manipulate a shared resource...
  // Only "atomic" operations or you'll miss the RT thread!
  pthread_mutex_unlock(&mutex_);
}

//--------- Ethercat related private functions --------------------------------------//

void EasycatMaster::EcStateChangedCb(const std::bitset<3>& new_state)
{
  if (new_state.all())
    PrintColor('g', "[EasycatMaster] EtherCAT network state: valid");
  else if (new_state.any())
    PrintColor('y', "[EasycatMaster] EtherCAT network state: invalid (%%s)",
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

  // This function is cycled at the specified rate
  // Insert your code here..
  // This code is executed at every RT cycle

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->WriteOutputs(); // write all the necessary pdos
}
""" % easycat_class_name)
    f.close()

''' MAIN '''

def main():
    """Method to execute parser/decoder directly from command line."""
    # Command line argument parser.
    parser = argparse.ArgumentParser(description='Parse a .XML file produced '
                                     'by EasyCAT_Config_GUI and generate '
                                     'corresponding header and source file to '
                                     'include custom device in an EtherCAT '
                                     'network.')
    parser.add_argument('-i', '--ifile', dest='easycat_header_filepath', 
                        required=True, help='absolute path of auto-generated '
                        'header file of easycat slave')
    parser.add_argument('-t', '--cycle-time', dest='cycle_time_msec', default=1,
                        help='real-time cycle time in millisecond. Default is 1ms.')
    parser.add_argument('-o', '--output-dir', dest='odir', 
                        default=os.path.dirname(__file__), help='output '
                        'directory path. If not given, current directory will'
                        ' be used')
    parsed_args = parser.parse_args()

    # Extract useful strings
    grab_common_dir = os.path.join(*('/',) + tuple(parsed_args.easycat_header_filepath.split('/')[:-5]))
    easycat_class_filename = parsed_args.easycat_header_filepath.split('/')[-1][:-2]
    with open(parsed_args.easycat_header_filepath, "r") as f:
        for line in f:
            if line.startswith("class"):
                easycat_class_name = line.split()[1]
                break
    
    # Check easycat class files exist
    easycat_source_filepath = os.path.join(grab_common_dir, 'libgrabec', 'src', 'slaves',
                            'easycat', easycat_class_filename + '.cpp')
    if not (os.path.exists(parsed_args.easycat_header_filepath) and 
            os.path.exists(easycat_source_filepath)):
        print("WARNING: missing easycat slave class files! Don't forget to run"
              " 'python gen_slave_class_from_EasyCAT_Config.py'!")

    # Make minimal easycat master app folder
    app_name = 'minimal_easycat_master'
    odir = os.path.join(parsed_args.odir, app_name)
    if not os.path.exists(odir):
        os.mkdir(odir)
        
    # Generate app files.
    print('Generating Qt project file...')
    gen_qt_pro_file(os.path.join(odir, app_name + '.pro'),
                    grab_common_dir, easycat_class_filename)
    print('Generating main file...')
    gen_main(os.path.join(odir, 'main.cpp'), parsed_args.cycle_time_msec)
    print('Generating master header file...')
    gen_easycat_master_header(os.path.join(odir, 'easycatmaster.h'),
                              easycat_class_filename, easycat_class_name)
    print('Generating master source file...')
    gen_easycat_master_source(os.path.join(odir, 'easycatmaster.cpp'),
                              easycat_class_name)
    print('Generation complete.\nYou can find the files of your new minimal ' 
          'application in "%s"' % odir)


if __name__ == "__main__":
    # Uncomment these line to run this script from IDE.
    sys.argv.append('-i')
    sys.argv.append('/home/simo/repos/cable_robot/libs/grab_common/libgrabec/inc/slaves/easycat/easyCatST_config02_slave.h')
    # sys.argv.append('/path/to/input/file.h')

    # sys.argv.append('-o')
    # sys.argv.append('/path/to/output/dir')
    # sys.argv.append('-t')
    # sys.argv.append('2')

    main()
