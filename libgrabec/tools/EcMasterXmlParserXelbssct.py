import os
import xml.etree.ElementTree as et
import pickle
import argparse
import warnings
import copy
import re

def erase_invalid_chars(name):
    """Convert name to valid C++ identifier."""
    # Replace invalid chars with space
    name = re.sub(r'[^a-zA-Z0-9_]', '', name)
    # Collapse repeated spaces
    name = re.sub(r' +', '', name)
    # Strip leading/trailing spaces
    name = name.strip()
    # Prevent leading digits
    if re.match(r'^[0-9]', name):
        name = 'x ' + name
    return name

class DomainEntry():
    """Domain entry element."""
    def __init__(self, element, suffix='', gap_idx=''):
        """Constructor."""
        self.Index = '0' + element.find('Index').text[1:].lower()
        if (self.Index=='0x0000'):
            self.SubIndex = '0'
            self.BitLen = element.find('BitLen').text
            base_name = 'GAP'
            self.Name = base_name + '_' + gap_idx + suffix
            self.DataType = 'uint8_t'
        else:
            self.SubIndex = element.find('SubIndex').text.lower()
            self.BitLen = element.find('BitLen').text
            base_name = erase_invalid_chars(element.find('Name').text.replace(' ', '_'))
            self.Name = base_name + suffix
            self.DataType = self._convert2Ctype(element.find('DataType').text)
            

    def print_fields(self):
        """Print fields in a pretty format."""
        print("Index =", self.Index)
        print("SubIndex =", self.SubIndex)
        print("BitLen =", self.BitLen)
        print("Name =", self.Name)
        print("DataType =", self.DataType)

    def _convert2Ctype(self, text):
        """Convert from IEC 61131-3 to C-types."""
        # Floating point
        if text == 'REAL':
            return 'float'
        if text == 'LREAL':
            return 'double'

        # Integers
        if text[0] == 'U':
            c_int_sign = 'u'
            text = text[1:]
        else:
            c_int_sign = ''
        if text == 'SINT':
            return c_int_sign + 'int8_t'
        if text == 'INT':
            return c_int_sign + 'int16_t'
        if text == 'DINT':
            return c_int_sign + 'int32_t'
        if text == 'LINT':
            return c_int_sign + 'int64_t'  
        
        # Booleans
        if text == 'BOOL':
            return 'uint8_t'


class EasyCatXmlParser(object):
    """EasyCAT XML configuration file parser."""
    def __init__(self, filepath=None):
        """Constructor."""
        # Init
        self._root = None
        self._output = {}
        self._file_loaded = False
        self._file_decoded = False
        self._filepath = ''

        self.load(filepath)

    ''' PUBLIC METHODs '''

    def load(self, filepath):
        """Load an EasyCAT configuration file.

        Parameters
        ----------
        filepath : str
            The absolute or relative (to the script) path to the .XML log file
            to be decoded.

        Return
        ------
        boolean
            The outcome of the loading operation.
        """
        self._file_loaded = False

        # Check right format.
        if not filepath.endswith('xml'):
            raise IOError('Wrong input file format. Please load an .xml file.')
        self._filepath = filepath

        # Parse the file.
        tree = et.parse(filepath,parser = et.XMLParser(encoding = 'iso-8859-5'))
        self._root = tree.getroot()

        self._file_loaded = True

    def decode(self, out=False):
        """Read `EasyCAT_Config` xml file and arrange it in dictionary format.

        Parameters
        ----------
        out: boolean, optional
            If set to `True`, the resulting dictionary is output, too.

        Returns
        -------
        boolean
            `True` if decoding process was successful, `False` otherwise.
        dict, optional
            The decoded information in dictionary format. Given only if `out`
            is set to `True`.
        """
        if not self._file_loaded:
            warnings.warn(
                'Configuration file not found. Please load a new file first.')
            self._file_decoded = False
            if out:
                return False, None
            else:
                return False

        print('Decoding "%s"...' % os.path.split(self._filepath)[1])
        
        # Collect all I/O entries
        rx_pdos = []
        for i, rxpdo_el in enumerate(self._root.findall('.//RxPdo')):
            pdo_index = '0' + rxpdo_el.find('Index').text[1:].lower()
            suffix = f"_{i+1}" if len(self._root.findall('.//RxPdo')) > 1 else ""
            entries = []
            gap_idx = 1
            for entry_el in rxpdo_el.findall('Entry'):
                # if entry_el.find('Index') is None or entry_el.find('Index').text == '#x0000':
                #     continue
                entries.append(DomainEntry(entry_el,suffix,str(gap_idx)))
                gap_idx+=1
            if entries:
                rx_pdos.append({'Index': pdo_index, 'Entries': entries})

        tx_pdos = []
        for i, txpdo_el in enumerate(self._root.findall('.//TxPdo')):
            pdo_index = '0' + txpdo_el.find('Index').text[1:].lower()
            suffix = f"_{i+1}" if len(self._root.findall('.//TxPdo')) > 1 else ""
            entries = []
            gap_idx = 1
            for entry_el in txpdo_el.findall('Entry'):
                # if entry_el.find('Index') is None or entry_el.find('Index').text == '#x0000':
                #     continue
                entries.append(DomainEntry(entry_el, suffix, str(gap_idx)))
                gap_idx+=1
            if entries:
                tx_pdos.append({'Index': pdo_index, 'Entries': entries})

        # Reset
        self._output = {}
        
        device_name_raw = self._root.find('.//Device/Type').text.replace(' ', '_')
        self._output = {
            'DeviceName': erase_invalid_chars(device_name_raw),
            'DomainEntriesNum': sum(len(pdo["Entries"]) for pdo in rx_pdos) +
                     sum(len(pdo["Entries"]) for pdo in tx_pdos),
            'Alias': 0,
            'VendorId': '0' +
            self._root.find('Vendor').find('Id').text[1:].lower(),
            'ProductCode': '0' +
            self._root.find('.//Device/Type').get('ProductCode')[1:].lower(),
            'RxPdos': rx_pdos,
            'TxPdos': tx_pdos
            }
            
        self._output['SyncManagers'] = [
            {
                'Index': i,
                'Direction': 'INPUT' if (int(sm.get('ControlByte').replace('#x', ''), 16) == 0x22) or 
                                            (int(sm.get('ControlByte').replace('#x', ''), 16) == 0x20) else 'OUTPUT',
                'Count' : len(rx_pdos) if (int(sm.get('ControlByte').replace('#x', ''), 16) == 0x64)
                            else len(tx_pdos) if (int(sm.get('ControlByte').replace('#x', ''), 16) == 0x20) else 0,
                'Watchdog': 'EC_WD_ENABLE' if int(sm.get('ControlByte').replace('#x', ''), 16) == 0x64 else 'EC_WD_DISABLE',
            }
            for i, sm in enumerate(self._root.findall('.//Sm'))
            if sm.get('Enable') == '1'
        ]

        self._file_decoded = True
        print('Decoding complete.')

        if out:
            return True, self.get_decoded_info()
        else:
            return True

    def get_decoded_info(self):
        """Return decoded information in dictionary format.

        Returns
        -------
        dict
            A dictionary containing decoded information with following format:

            {

            }
        """
        if not self._file_decoded:
            warnings.warn(
                    'Decoded information missing. Please run "decode" first.')
            return None

        output = copy.deepcopy(self._output)
        return output

    def save(self, ofilename=None, outdir=None):
        """Save the outcome of the reading process in a text file with pickle.

        Pickle is used to dump the dictionary containing the decoded
        information in a text file that can be loaded and used in future.

        Parameters
        ----------
        ofilename : str, optional
            The name of the output text file. This is NOT the path to that
            file. If not given the name of the original file will be used and
            extended with `_decoded`.
        outdir : str, optional
            The absolute or relative (to this script) path of the directory
            where the output file should be saved. If not given the same
            directory of the input file will be used.

        Returns
        -------
        boolean
            The outcome of the saving operation.
        """
        if not self._file_decoded:
            warnings.warn(
                    'Decoded information missing. Please run "decode" first.')
            return False

        # Output file name.
        if ofilename is None:
            ofilename = os.path.splitext(os.path.split(self._ifile.name)[1])[0]
            +'_decoded.txt'
        elif not ofilename.endswith('.txt'):
            ofilename += '.txt'

        # Parent directory.
        if outdir is not None:
            if not os.path.exists(outdir):
                warnings.warn('Output directory "%s" not found. Please '
                              'provide an existing path.' %
                              os.path.abspath(outdir))
                return False
        else:
            outdir = os.path.split(os.path.abspath(self._ifile.name))[0]

        # Save decoded information in pickle file.
        ofilepath = os.path.join(outdir, ofilename)
        with open(ofilepath, "w") as outf:
            pickle.dump(self.get_decoded_info(), outf)

        print('Decoded log file "%s" saved in "%s".' %
              (os.path.split(self._ifile.name)[1], os.path.abspath(ofilepath)))
        return True

''' MAIN '''


def main():
    """Method to execute parser/decoder directly from command line."""
    # Command line argument parser.
    parser = argparse.ArgumentParser(description='Parse a .XML file produced'
                                     ' by EasyCAT_Config_GUI.')
    parser.add_argument('-i', '--ifile', dest='ifile', required=True,
                        help='absolute or relative path of input configuration'
                        'file')
    parser.add_argument('-s', '--save', dest='save', action='store_true',
                        default=False, help='save decoded data in pickle'
                        ' format (i.e. text file)')
    parser.add_argument('-o', '--ofile', dest='ofile',
                        help='output text file name (not path)')
    parser.add_argument('-d', '--outdir', dest='odir',
                        help='output directory')
    parsed_args = parser.parse_args()

    # Log file parsing and decoding.
    decoder = EasyCatXmlParser(filepath=parsed_args.ifile)
    if not decoder.decode():
        sys.exit(1)

    if parsed_args.save:
        decoder.save(ofilename=parsed_args.ofile, outdir=parsed_args.odir)


if __name__ == "__main__":
    import sys

    # Uncomment these line to run this script from IDE.
    # sys.argv.append('-i')
    # sys.argv.append('/path/to/input/file.xml')
    # sys.argv.append('-s')
    # sys.argv.append('-o')
    # sys.argv.append('output_filename')
    # sys.argv.append('-d')
    # sys.argv.append('path/to/output/directory')

    main()
    sys.exit(0)
