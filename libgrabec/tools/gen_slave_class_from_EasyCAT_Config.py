from EasyCAT_Config_xml_parser import EasyCatXmlParser
import os
import numpy as np
import argparse
import sys


def gen_header(filepath, config_params):
    # Flags
    outputs_present = 'Outputs' in config_params
    inputs_present = 'Inputs' in config_params

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
//   You can either insert your code in the designated areas or inherit      //
//   from this class.                                                        //
//                                                                           //
//---------------------------------------------------------------------------//

""")
    f.write("""#include "ethercatslave.h"
#include "types.h"

namespace grabec
{

/**
 * @brief The %sSlave class
 */
class %sSlave : public virtual EthercatSlave
{
public:
  /**
   * @brief %sSlave
   * @param slave_position
   */
  %sSlave(const uint8_t slave_position);
  ~%sSlave();

  /**
   * @brief Slave's main function to be cycled.
   */
  void DoWork() override; // need to override it even if not used

  /**
   * @brief Function to specify what to read.
   */
  void ReadInputs() override final;

  /**
   * @brief Function to specify what to write.
   */
  void WriteOutputs() override final;

""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 5))
    if outputs_present:
        f.write("""
  // Output buffer, i.e. data received from master (read)
  union CustBufferOut
  {
    uint8_t Byte[%d];
    struct
    {
""" % (4 * np.ceil(len(config_params['Outputs']['Entries']) / 4.)))
        for entry in config_params['Outputs']['Entries']:
            f.write('      %s %s;\n' % (entry.DataType, entry.Name))
        f.write("""    } Cust;
  } BufferOut;
""")
    if inputs_present:
        f.write("""
  // Input buffer, i.e. data sent to master (write)
  union CustBufferIn
  {
    uint8_t Byte[%d];
    struct
    {
""" % (4 * np.ceil(len(config_params['Inputs']['Entries']) / 4.)))
        for entry in config_params['Inputs']['Entries']:
            f.write('      %s %s;\n' % (entry.DataType, entry.Name))
        f.write("""    } Cust;
  } BufferIn;
""")
    f.write("""
protected:
  void InitFun() override; // need to override it even if not used

private:
  // EasyCAT slave device specific info
""")
    f.write('  static constexpr uint8_t kDomainEntries_ = %s;\n' %
            config_params['DomainEntriesNum'])
    f.write('  static constexpr uint8_t kAlias_ = %s;\n' %
            config_params['Alias'])
    f.write('  static constexpr uint32_t kVendorID_ = %s;\n' %
            config_params['VendorId'])
    f.write('  static constexpr uint32_t kProductCode_ = %s;\n' %
            config_params['ProductCode'])
    f.write("""
  // Ethercat utilities, describing index, subindex and bit length of each
  // configured PDO entry.
  static constexpr ec_pdo_entry_info_t kPdoEntries_[kDomainEntries_] = {
""")
    if outputs_present:
        for entry in config_params['Outputs']['Entries']:
            f.write('    {%s, %s, %s}, /**< output PDO: %s */\n' %
                    (entry.Index, entry.SubIndex, entry.BitLen, entry.Name))
    if inputs_present:
        for entry in config_params['Inputs']['Entries']:
            f.write('    {%s, %s, %s}, /**< input PDO: %s */\n' %
                    (entry.Index, entry.SubIndex, entry.BitLen, entry.Name))
    f.write("""  };

  // Ethercat utilities, describing memory position of input and output PDOs
  // stack.
""")
    f.write('  static constexpr ec_pdo_info_t kPDOs_[%d] = {\n' %
            (outputs_present + inputs_present))
    if outputs_present:
        num_output_pdos = len(config_params['Outputs']['Entries'])
        f.write('    {%s, %d, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_)},'
                ' /**< Output PDOs */\n'
                % (config_params['Outputs']['Index'], num_output_pdos))
    else:
        num_output_pdos = 0
    if inputs_present:
        f.write('    {%s, %d, const_cast<ec_pdo_entry_info_t*>(kPdoEntries_) +'
                ' %d}, /**< Inputs PDOs */\n'
                % (config_params['Inputs']['Index'],
                   len(config_params['Inputs']['Entries']), num_output_pdos))
    f.write("""  };

  // Ethercat utilities, synchronization information
  static constexpr ec_sync_info_t kSyncs_[3] = {
    {0, EC_DIR_OUTPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 0,
       EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, const_cast<ec_pdo_info_t*>(kPDOs_) + 1,
       EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, NULL,
       static_cast<ec_watchdog_mode_t>(0)}};
""")
    if inputs_present:
        f.write("""
  // Useful ethercat struct to store input PDOs memory offset
  struct OffsetIn
  {
""")
        for entry in config_params['Inputs']['Entries']:
            f.write('    unsigned int %s;\n' % entry.Name)
        f.write('  } offset_in_;\n')
    if outputs_present:
        f.write("""
  // Useful ethercat struct to store output PDOs memory offset
  struct OffsetOut
  {
""")
        for entry in config_params['Outputs']['Entries']:
            f.write('    unsigned int %s;\n' % entry.Name)
        f.write('  } offset_out_;\n')
    f.write("""
  ec_pdo_entry_reg_t domain_registers_[kDomainEntries_]; // ethercat utility
};

} // end namespace grabec

""")
    f.write('#endif // GRABCOMMON_LIBGRABEC_EASYCATSLAVE_%s_H\n' %
            config_params['DeviceName'].upper())
    f.close()


def data_type2macro_suffix(text):
    # Integers
    if text[0] == 'u':
        sign = 'U'
    else:
        sign = 'S'

    if text[1:] == 'int8_t':
        return sign + '8'
    if text[1:] == 'int16_t':
        return sign + '16'
    if text[1:] == 'int32_t' or text == 'float':
        return sign + '32'
    if text[1:] == 'int64_t' or text == 'double':
        return sign + '64'


def gen_source(filepath, config_params):
    # Flags
    outputs_present = 'Outputs' in config_params
    inputs_present = 'Inputs' in config_params

    # Write source file.
    f = open(filepath, 'w')
    f.write('#include "slaves/%s.h"\n' %
            os.path.splitext(filepath.split('\\')[-1])[0])
    f.write("""
namespace grabec
{
// Must provide redundant definition of static members as well
constexpr ec_pdo_entry_info_t %sSlave::kPdoEntries_[];
constexpr ec_pdo_info_t %sSlave::kPDOs_[];
constexpr ec_sync_info_t %sSlave::kSyncs_[];

%sSlave::%sSlave(const uint8_t slave_position)
{
  alias_ = kAlias_;
  vendor_id_ = kVendorID_;
  product_code_ = kProductCode_;
  num_domain_entries_ = kDomainEntries_;
  position_ = slave_position;
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 5))
    if outputs_present:
        for i, entry in enumerate(config_params['Outputs']['Entries']):
            f.write("""  domain_registers_[%d] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[%d].index, kPdoEntries_[%d].subindex,
                          &offset_out_.%s, NULL};\n""" % (i, i, i, entry.Name))
        i += 1
    else:
        i = 0
    if outputs_present:
        for j, entry in enumerate(config_params['Inputs']['Entries']):
            f.write("""  domain_registers_[%d] = {alias_, position_, vendor_id_, product_code_,
                          kPdoEntries_[%d].index, kPdoEntries_[%d].subindex,
                          &offset_in_.%s, NULL};\n""" %
                    (i + j, i + j, i + j, entry.Name))
    f.write("""
  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = const_cast<ec_pdo_entry_info_t*>(kPdoEntries_);
  slave_pdos_ptr_ = const_cast<ec_pdo_info_t*>(kPDOs_);
  slave_sync_ptr_ = const_cast<ec_sync_info_t*>(kSyncs_);
}

%sSlave::~%sSlave()
{
  /*
   * Your code here..
   */
}

void %sSlave::DoWork()
{
  /*
   * Your code here..
   */
}

void %sSlave::ReadInputs()
{
""" % tuple([config_params['DeviceName'][0].upper() +
             config_params['DeviceName'][1:]] * 4))
    if inputs_present:
        f.write('  // This is the way we can read the PDOs, according to '
                'ecrt.h\n')
        for entry in config_params['Inputs']['Entries']:
            f.write('  BufferIn.Cust.%s = EC_READ_%s(domain_data_ptr_ +\n\t'
                    'offset_in_.%s);\n' %
                    (entry.Name, data_type2macro_suffix(entry.DataType),
                     entry.Name))
    f.write("""}

void %sSlave::WriteOutputs()
{
""" % (config_params['DeviceName'][0].upper() +
       config_params['DeviceName'][1:]))
    if outputs_present:
        f.write('  // This is the way we can write the PDOs, according to '
                'ecrt.h\n')
        for entry in config_params['Outputs']['Entries']:
            f.write('  EC_WRITE_%s(domain_data_ptr_ + offset_out_.%s,\n\t'
                    'BufferOut.Cust.%s);\n' %
                    (data_type2macro_suffix(entry.DataType), entry.Name,
                     entry.Name))
    f.write("""}

void %sSlave::InitFun()
{
  /*
   * Your code here..
   */
}

} // end namespace grabec
""" % (config_params['DeviceName'][0].upper() +
       config_params['DeviceName'][1:]))
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
    gen_header(os.path.join(os.path.dirname(__file__), '..\\inc\\slaves',
                            ofilename + '.h'), config_params)
    print('Generating source file...')
    gen_source(os.path.join(os.path.dirname(__file__), '..\\src\\slaves',
                            ofilename + '.cpp'), config_params)
    print('Generation complete.\nYou can find the new files in the "slaves" '
          'folders of your local "libgrabec" directory.')
    print("Don't forget to copy '%s.h' in '%s'!" %
          (os.path.splitext(parsed_args.ifile.split('\\')[-1])[0],
           os.path.abspath(os.path.join(os.path.dirname(__file__),
                                        '..\\inc\\slaves'))))


if __name__ == "__main__":
    # Uncomment these line to run this script from IDE.
    # sys.argv.append('-i')
    # sys.argv.append('/path/to/input/file.xml')
    # sys.argv.append('-o')
    # sys.argv.append('output_filename')

    main()
