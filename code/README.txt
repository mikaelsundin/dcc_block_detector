Tool used:
 * Emblocks 2.3 

Emblocks Segger Jlink GDB settings:

J-Link GDB Server Arguments:
 "-select USB -device STM32F030F4 -if SWD -speed 1000 -noir"
Reset Command(s):
 "monitor reset"

After Connection:
  "monitor flash download = 1
   monitor flash breakpoints = 1

   load
   monitor reset
   break main
   continue"
 
 

 
Note:
Remember to change linker script to match CPU flash/ram size if changed.

