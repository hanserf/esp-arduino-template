# Aeolus Firmware Development

### Short info

Embedded development on ESP32 MCU can be done in C or Arduino.  
https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md   
Outlines a way to write arduino/C++ code and compile it using the official ESP-IDF tools.   
Using Menuconfig. the MCU has been configured to run Arduino on processor core 1.   
User tasks in C should be set to run on core 0.   

### Clone all submodules 

```
cd $PROJECTROOT
git submodule update --init --recursive --progress
```


### Install ESP-IDF Toolchain from espressif

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/  

Windows users can get everything needed here:  

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html

The main source of the development tools is found here:  
https://github.com/espressif/esp-idf



### Update ESP toolchain to MASTER
When first installing ESP-IDF you were prompted for two install paths.  
One path was esp-idf repository location, second was location for binary executables, python environment and tools.  
locate this folder named .espressif, it default to  "C:\Users\youruser\"   
find uninstaller and uninstall esp-idf tools.  

Go to Esp-idf install catalog  

In git bash, cmd or powershell:  

``` 
git checkout master
git submodule update --init --recursive
wait for submodules to update, then install ESP-idf tools with:
install.bat in cmd.exe
or
install.ps in powershell

```


### Patch Necessery known bugs

For UNIX:  
```
bash patches.sh
```
Windows:  
Open pathces.sh in notepad.  
sed is a non interactive text editor replacing a match regexp i a file using pattern :FIND:REPLACE:g  
Do text replacement in files listed in patches.sh  


## Compile Flash and Monitor:

Windows:  
Go to root folder of ESP-IDF  
open cmd.exe    
```
export.bat
```

OR  

open powershell   
```
export.ps1
```

Linux:  
```
source export.sh
```

If you see a response like this, all is good:

```
Setting IDF_PATH: C:\esp32

Adding ESP-IDF tools to PATH...
    C:\Users\hanse\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin
    C:\Users\hanse\.espressif\tools\xtensa-esp32s2-elf\esp-2020r3-8.4.0\xtensa-esp32s2-elf\bin
    C:\Users\hanse\.espressif\tools\xtensa-esp32s3-elf\esp-2020r3-8.4.0\xtensa-esp32s3-elf\bin
    C:\Users\hanse\.espressif\tools\riscv32-esp-elf\1.24.0.123_64eb9ff-8.4.0\riscv32-esp-elf\bin
    C:\Users\hanse\.espressif\tools\esp32ulp-elf\2.28.51-esp-20191205\esp32ulp-elf-binutils\bin
    C:\Users\hanse\.espressif\tools\esp32s2ulp-elf\2.28.51-esp-20191205\esp32s2ulp-elf-binutils\bin
    C:\Users\hanse\.espressif\tools\cmake\3.16.4\bin
    C:\Users\hanse\.espressif\tools\openocd-esp32\v0.10.0-esp32-20200709\openocd-esp32\bin
    C:\Users\hanse\.espressif\tools\ninja\1.10.2\
    C:\Users\hanse\.espressif\tools\idf-exe\1.0.1\
    C:\Users\hanse\.espressif\tools\ccache\3.7\
    C:\Users\hanse\.espressif\tools\dfu-util\0.9\dfu-util-0.9-win64
    C:\Users\hanse\.espressif\python_env\idf4.4_py3.8_env\Scripts
    C:\esp32\tools

Checking if Python packages are up to date...
Python requirements from C:\esp32\requirements.txt are satisfied.

Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build
```
