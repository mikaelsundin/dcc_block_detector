# dcc_block_detector
A dcc block detector based on STM32F030 and measure current over diode-drop

#This detector is usefull both for N-Scale and H0-scale. 
A SMC diode of 5A can handle alot of current.
For N-sclale its possible to use regular pin header at IO1, IO2 (max 3A).
For HO scale i recommend screw terminal to handle more current.

#Note
A second board with a MCU is needed to translate from uart to other protocols.
