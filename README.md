# Low cost thermostat using Padauk PFS123

This is the repository of a simple super low cost thermostat using extremely cheap microcontroller Padauk PFS123.

It uses SDCC 4.2.10 compiler for Windows to compile the code and EASYPDKPROG_WIN_20200713_1.3 firmware downloading tool software from the free-pdk https://github.com/free-pdk.

The hardware is based on Padauk PFS123 microcontroller, but it is the same as PFS173 supported by the SDCC toolchain.

Use batch files to compile, program the microcontroller and run.

Also Altium design files and Python scripts to control the device are included. The Python code commands reference temperature to the microcontroller and reads back the temperature.

This is not a complete device, but just an illustration of Padauk capabilities, namely:

- software-defined UART (both TX and RX, sorry the RX is not very robust thats why the Python has retry mechanism)

- ADC

- PWM

- PID controller implementation