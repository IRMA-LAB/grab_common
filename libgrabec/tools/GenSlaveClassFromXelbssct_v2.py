from EcMasterXmlParserXelbssct import EasyCatXmlParser
import os
import numpy as np
import argparse
import sys
name_of_class = "XelBssctIrma8"
pdos_to_use = [
    #INPUT PDOS
    "CH0_Digital_Output_Data_4",
    "CH1_Digital_Output_Data_4",
    "CH2_Digital_Output_Data_4",
    "CH3_Digital_Output_Data_4",
    "CH0_Digital_Output_Data_5",
    "CH1_Digital_Output_Data_5",
    "CH2_Digital_Output_Data_5",
    "CH3_Digital_Output_Data_5",
    "CH0_Count_Data_6",
    "CH1_Count_Data_6",
    "CH0_Count_Data_7",
    "CH1_Count_Data_7",
    "CH0_Count_Data_8",
    "CH1_Count_Data_8",
    "CH0_Count_Data_9",
    "CH1_Count_Data_9",
    #OUTPUT PDOS
    "CH0_Enable_Counter_6",
    "CH0_Enable_Counter_7",
    "CH0_Enable_Counter_8",
    "CH0_Enable_Counter_9",
    "CH1_Enable_Counter_6",
    "CH1_Enable_Counter_7",
    "CH1_Enable_Counter_8",
    "CH1_Enable_Counter_9",

]

def gen_header(filepath, config_params):
    # Flags
    outputs_present = 'RxPdos' in config_params
    inputs_present = 'TxPdos' in config_params

    # Write header file.
    f = open(filepath, 'w')
    f.write('#ifndef GRABCOMMON_LIBGRABEC_EASYCATSLAVE_%s_H\n' %
            config_params['DeviceName'].upper())
    f.write('#define GRABCOMMON_LIBGRABEC_EASYCATSLAVE_%s_H\n' %
            config_params['DeviceName'].upper())
    f.write("""
//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
""")
    f.write('//     Easy Configurator project %s.prj\n' %
            config_params['DeviceName'])
    f.write('//     Easy Configurator XML %s.xml' %
            config_params['DeviceName'])
    f.write("""
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

""")
    f.write("""#include "ethercatslave.h"
#include "grabec_types.h"

namespace grabec
{

/**
 * @brief The %s class
 */
class %s : public virtual EthercatSlave
{
public:
  /**
   * @brief %s
   * @param slave_position
   */
  %s(const uint8_t slave_position);
  ~%s() override;

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
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 5))
    if outputs_present:
        f.write("""
  /**
   * @brief Output buffer union, i.e. data received from master (read).
   */
  struct CustBufferOut
  {
""")
    for pdo in config_params['RxPdos']:
        for entry in pdo['Entries']:
            if (entry.Index!='0x0000') and entry.Name in pdos_to_use:
                f.write('      %s %s;\n' % (entry.DataType, entry.Name))
    f.write("""
  } BufferOut; /**< Output buffer, i.e. data received from master (read). */
""")
    if inputs_present:
        f.write("""
  /**
   * @brief Input buffer union, i.e. data sent to master (write).
   */
  struct CustBufferIn
  {
""")
    for pdo in config_params['TxPdos']:
        for entry in pdo['Entries']:
            if (entry.Index!='0x0000') and entry.Name in pdos_to_use:
                f.write('      %s %s;\n' % (entry.DataType, entry.Name))
    f.write(""" 
  } BufferIn; /**< Input buffer, i.e. data sent to master (write). */
""")
    f.write("""
protected:
  void initFun() override;

private:
  // EasyCAT slave device specific info
""")
    f.write('  static constexpr uint16_t kDomainEntries_ = %s;\n' %
            config_params['DomainEntriesNum'])
    f.write('  static constexpr uint8_t kAlias_         = %s;\n' %
            config_params['Alias'])
    f.write('  static constexpr uint32_t kVendorID_     = %s;\n' %
            config_params['VendorId'])
    f.write('  static constexpr uint32_t kProductCode_  = %s;\n' %
            config_params['ProductCode'])
    f.write("""
  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
""")
    if outputs_present:
        for pdo in config_params['RxPdos']:
            for entry in pdo['Entries']:
                f.write('    {%s, %s, %s}, /**< output PDO: %s */\n' %
                        (entry.Index, entry.SubIndex, entry.BitLen, entry.Name))
    if inputs_present:
        for pdo in config_params['TxPdos']:
            for entry in pdo['Entries']:
                f.write('    {%s, %s, %s}, /**< input PDO: %s */\n' %
                        (entry.Index, entry.SubIndex, entry.BitLen, entry.Name))
    f.write("""  };

  // Ethercat utilities, describing memory position of input and output PDOs
  // stack.
""")
    all_pdos = []
    if 'RxPdos' in config_params:
        all_pdos.extend(config_params['RxPdos'])
    if 'TxPdos' in config_params:
        all_pdos.extend(config_params['TxPdos'])

    f.write('  static constexpr ec_pdo_info_t kPDOs_[%d] = {\n' % len(all_pdos))

    entry_offset = 0
    for pdo in all_pdos:
        num_entries = len(pdo['Entries'])
        f.write('    {%s, %d, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) + %d},\n' %
                (pdo['Index'], num_entries, entry_offset))
        entry_offset += num_entries

    f.write('  };\n')
    f.write("""

  // Ethercat utilities, synchronization information
  static constexpr ec_sync_info_t kSyncs_[] = {
""")
    syncs = config_params.get('SyncManagers', [])
    pdo_offset = 0
    for sm in syncs:
        direction = 'EC_DIR_INPUT' if sm['Direction'] == 'INPUT' else 'EC_DIR_OUTPUT'
        count = sm['Count']
        if (count != 0):
            f.write(f"""    {{{sm['Index']}, {direction}, {count}, const_cast<ec_pdo_info_t*>(kPDOs_) + {pdo_offset}, {sm['Watchdog']}}},\n""")
        else:
            f.write(f"""    {{{sm['Index']}, {direction}, {count}, nullptr, {sm['Watchdog']}}},\n""")
        pdo_offset += count

    f.write("""    {0xff, static_cast<ec_direction_t>(0), 0, nullptr,
       static_cast<ec_watchdog_mode_t>(0)}};
""")
    
    if inputs_present:
        f.write("""
  // Useful ethercat struct to store input PDOs memory offset
  struct OffsetIn
  {
""")
        for pdo in config_params['TxPdos']:
            for entry in pdo['Entries']:
                if (entry.Index!='0x0000')  and entry.Name in pdos_to_use:
                    f.write('    unsigned int %s;\n' % entry.Name)
        f.write('  } offset_in_;\n')
    if outputs_present:
        f.write("""
  // Useful ethercat struct to store output PDOs memory offset
  struct OffsetOut
  {
""")
        for pdo in config_params['RxPdos']:
            for entry in pdo['Entries']:
                if (entry.Index!='0x0000') and entry.Name in pdos_to_use:
                    f.write('    unsigned int %s;\n' % entry.Name)
        f.write('  } offset_out_;\n\n')
        
    f.write('ec_pdo_entry_reg_t domain_registers_[%d]; // ethercat utility\n' % (len(pdos_to_use)+1))
    f.write(""" };

} // end namespace grabec

""")

    f.write('#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_%s_H\n' %
            config_params['DeviceName'].upper())
    f.close()


def ec_readln(f, entry):
    # Excluding placeholders
    if entry.Index == '0x0000':
        return

    # Floating
    if entry.DataType == 'float':
        f.write('  int32_t %s = EC_READ_S32(domain_data_ptr_ + '
                'offset_in_.%s);\n' % (entry.Name, entry.Name))
        f.write('  memcpy(&BufferIn.Cust.%s, &%s, '
                'sizeof(float));\n' % (entry.Name, entry.Name))
        return
    if entry.DataType == 'double':
        f.write('  int64_t %s = EC_READ_S64(domain_data_ptr_ + '
                'offset_in_.%s);\n' % (entry.Name, entry.Name))
        f.write('  memcpy(&BufferIn.Cust.%s, &%s, '
                'sizeof(double));\n' % (entry.Name, entry.Name))
        return

    # Integers
    if entry.DataType[0] == 'u':
        sign = 'U'
        i = 1
    else:
        sign = 'S'
        i = 0

    if entry.DataType[i:] == 'int8_t':
        data_type = sign + '8'
    if entry.DataType[i:] == 'int16_t':
        data_type = sign + '16'
    if entry.DataType[i:] == 'int32_t':
        data_type = sign + '32'
    if entry.DataType[i:] == 'int64_t':
        data_type = sign + '64'
    f.write('  BufferIn.%s = EC_READ_%s(domain_data_ptr_ + '
            'offset_in_.%s);\n' % (entry.Name, data_type, entry.Name))


def ec_writeln(f, entry):
    # Excluding placeholders
    if entry.Index == '0x0000':
        return
        
    # Floating
    if entry.DataType == 'float':
        f.write('  int32_t %s;\n' % entry.Name)
        f.write('  memcpy(&%s, &BufferOut.%s, sizeof(float));\n' %
                (entry.Name, entry.Name))
        f.write('  EC_WRITE_S32(domain_data_ptr_ + offset_out_.%s, %s);\n' %
                (entry.Name, entry.Name))
        return
    if entry.DataType == 'double':
        f.write('  int64_t %s;\n' % entry.Name)
        f.write('  memcpy(&%s, &BufferOut.%s, sizeof(double));\n' %
                (entry.Name, entry.Name))
        f.write('  EC_WRITE_S64(domain_data_ptr_ + offset_out_.%s, %s);\n' %
                (entry.Name, entry.Name))
        return

    # Integers
    if entry.DataType[0] == 'u':
        sign = 'U'
        i = 1
    else:
        sign = 'S'
        i = 0

    if entry.DataType[i:] == 'int8_t':
        data_type = sign + '8'
    if entry.DataType[i:] == 'int16_t':
        data_type = sign + '16'
    if entry.DataType[i:] == 'int32_t':
        data_type = sign + '32'
    if entry.DataType[i:] == 'int64_t':
        data_type = sign + '64'
    f.write('  EC_WRITE_%s(domain_data_ptr_ + offset_out_.%s, '
            'BufferOut.%s);\n' % (data_type, entry.Name, entry.Name))


def gen_source(filepath, config_params):
    # Flags
    outputs_present = 'RxPdos' in config_params
    inputs_present = 'TxPdos' in config_params

    # Write source file.
    f = open(filepath, 'w')
    f.write("""//---------------------------------------------------------------------------//
//                                                                           //
//   This file has been created by GRAB EasyCAT C++ class generation tool    //
//                                                                           //
""")
    f.write('//     Easy Configurator project %s.prj\n' %
            config_params['DeviceName'])
    f.write('//     Easy Configurator XML %s.xml' %
            config_params['DeviceName'])
    f.write("""
//                                                                           //
//   You can either insert your code in the designated areas and extend it   //
//   or inherit from this class.                                             //
//                                                                           //
//---------------------------------------------------------------------------//

#include <cstring>

""")
    f.write('#include "%s.h"\n' %
            os.path.join('slaves', 'easycat',
                         os.path.splitext(os.path.split(filepath)[1])[0]))
    f.write("""
namespace grabec
{
// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t %s::kPdoEntries_[];
constexpr ec_pdo_info_t %s::kPDOs_[];
constexpr ec_sync_info_t %s::kSyncs_[];

%s::%s(const uint8_t slave_position)
{
  alias_ = kAlias_;
  vendor_id_ = kVendorID_;
  product_code_ = kProductCode_;
  num_domain_entries_ = kDomainEntries_;
  position_ = slave_position;
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 5))
    i = 0
    j = 0
    if outputs_present:
        for pdo in config_params['RxPdos']:
            for entry in pdo['Entries']:
                if (entry.Index!='0x0000') and entry.Name in pdos_to_use:
                    f.write("""  domain_registers_[%d] = {alias_, position_, vendor_id_, product_code_,
                                %s, %s,
                                &offset_out_.%s, nullptr};\n""" % (i, entry.Index, entry.SubIndex, entry.Name))
                    i+=1
                    
    if inputs_present:
        for pdo in config_params['TxPdos']:
            for entry in pdo['Entries']:
                if (entry.Index!='0x0000') and entry.Name in pdos_to_use:
                    f.write("""  domain_registers_[%d] = {alias_, position_, vendor_id_, product_code_,
                                %s, %s,
                                &offset_in_.%s, nullptr};\n""" %
                            (i + j, entry.Index, entry.SubIndex, entry.Name))
                    j += 1
    f.write("""  domain_registers_[%d] = {0, 0, 0, 0, 0, 0, 0, nullptr};\n"""% (i+j))
    f.write("""
  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);
}

%s::~%s()
{
  /*
   * Your code here..
   */
}

void %s::doWork()
{
  /*
   * Your code here..
   */
}

void %s::readInputs()
{
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 4))
    if inputs_present:
        f.write('  // This is the way we can read the PDOs, according to '
                'ecrt.h\n')
        for pdo in config_params['TxPdos']:
            for entry in pdo['Entries']:
                if entry.Name in pdos_to_use:
                    ec_readln(f, entry)
    f.write("""}

void %s::writeOutputs()
{
""" % (config_params['DeviceName'][0].upper() +
       config_params['DeviceName'][1:]))
    if outputs_present:
        f.write('  // This is the way we can write the PDOs, according to '
                'ecrt.h\n')
        for pdo in config_params['RxPdos']:
            for entry in pdo['Entries']:
                if entry.Name in pdos_to_use:
                    ec_writeln(f, entry)
    f.write("""}

void %s::safeExit()
{
  /*
   * Your code here..
   */
}

bool %s::isReadyToShutDown() const
{
  /*
   * Your code here..
   * Return bool accordingly..
   */
  return true;
}

void %s::initFun()
{
  /*
   * Your code here..
   */
}

} // end namespace grabec
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 3))
    f.close()


def main():
    """Method to execute parser/decoder directly from command line."""
    # Command line argument parser.
    parser = argparse.ArgumentParser(description='Parse a .XML file produced '
                                     'by EasyCAT_Config_GUI and generate '
                                     'corresponding header and source file to '
                                     'include custom device in an EtherCAT '
                                     'network.')
    parser.add_argument('-i', '--ifile', dest='ifile', required=True,
                        help='absolute or relative path of input configuration'
                        ' file')
    parser.add_argument('-o', '--ofile', dest='ofile',
                        help='output class file name suffix (not path). If not'
                        ' given, input file name will be used')
    parsed_args = parser.parse_args()

    # Configuration file parsing and decoding.
    decoder = EasyCatXmlParser(filepath=parsed_args.ifile)
    if not decoder.decode():
        sys.exit(1)
    config_params = decoder.get_decoded_info()

    # Output file name.
    if parsed_args.ofile is None:
        ofilename = (os.path.splitext(os.path.split(parsed_args.ifile)[1])[0] +
                     '_slave')
    else:
        ofilename = parsed_args.ofile

    # Generate class files.
    print('Generating header file...')
    ofiledir = os.path.join(os.path.dirname(__file__), '..', 'inc', 'slaves',
                            'easycat')
    if not os.path.exists(ofiledir):
        os.mkdir(ofiledir)
    config_params['DeviceName'] = name_of_class
    gen_header(os.path.join(ofiledir, ofilename + '.h'), config_params)
    print('Generating source file...')
    ofiledir = os.path.join(os.path.dirname(__file__), '..', 'src', 'slaves',
                            'easycat')
    if not os.path.exists(ofiledir):
        os.mkdir(ofiledir)
    gen_source(os.path.join(ofiledir, ofilename + '.cpp'), config_params)
    print('Generation complete.\nYou can find the new files in the "slaves" '
          'folders of your local "libgrabec" directory.')
    print("Don't forget to copy '%s.h' in '%s'!" %
          (os.path.splitext(os.path.split(parsed_args.ifile)[1])[0],
           os.path.abspath(os.path.join(os.path.dirname(__file__),
                                        '..', 'inc', 'slaves', 'easycat'))))


if __name__ == "__main__":
    # Uncomment these line to run this script from IDE.
    # sys.argv.append('-i')
    # sys.argv.append('/path/to/input/file.xml')
    # sys.argv.append('-o')
    # sys.argv.append('output_filename')

    main()
