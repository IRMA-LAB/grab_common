## Usage

### On Windows 10
-------------
1. Right click on the Windows icon at the bottom left corner of your screen and select "Windows PowerShell"
2. Type and execute the following command to generate the desired files (assuming you already have Python installed on your PC and you already configured your EasyCAT device through EasyCAT_Config_GUI):
```bash
$PATH_TO_PYTHON_EXE $PATH_TO_THIS_FOLDER\gen_slave_class_from_EasyCAT_Config.py -i $PATH_TO_YOUR_XML_CONFIGURATION_FILE
```
where:
- `$PATH_TO_PYTHON_EXE` is the absolute path of your Python executables, typically something like _C:\Python27\python.exe_
- `$PATH_TO_THIS_FOLDER` can be copied by clicking on the address bar when opening this folder on explorer
