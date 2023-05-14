import serial
import re
import time
import sys
import glob
import numpy as np

TIME_OUT_S = 0.02
RETRY_COUNT = 10

def MCU_open(port_name: str):
    """! Open a serial port to the hardware.
    @param  str Serial port name in the operating system (like COM5).
    @return  An instance of the serial class or None if unsuccessful.
    """

    ser = serial.Serial(port_name)
    # this is to flush the buffer
    ser.read_all()
    ser.flush()
    return ser

def send_on_off_cmd(ser, cmd, on):
    for _ in range(RETRY_COUNT): # retry count
        cmd_str = cmd + '%d' % on
        ser.write(cmd_str.encode())

        start_time = time.time()
        response = 0
        while time.time() - start_time < TIME_OUT_S:
            if ser.in_waiting:
                try:
                    response = ser.read().decode()
                except:
                    pass # this will exit from the reading loop and we will retry

                break

        if response == '%d' % on: # we should receive the same parameter value as acknowledgement
            return True

    return False


def green_LED_set(ser, freq: int):
    """! controls the green LED on the board.
    @param  ser Instance of the serial class.
    @param  freq LED blinking frequency (in 0.1 Hz); max is 100 (beyond which LED stays on), min is 0 (off)
    @return  True if successful, False is unsuccessful
    """

    # doesn't support blinking for now, only on or off
    on = freq > 0

    return send_on_off_cmd(ser, 'G', on)

    
def red_LED_set(ser, freq: int):
    """! controls the green LED on the board.
    @param  ser Instance of the serial class.
    @param  freq LED blinking frequency (in 0.1 Hz); max is 100 (beyond which LED stays on), min is 0 (off)
    @return  True if successful, False is unsuccessful
    """

    # doesn't support blinking for now, only on or off
    on = freq > 0

    return send_on_off_cmd(ser, 'R', on)
    
    
def yellow_LED_set(ser, freq: int): # TODO: it is actually blue!
    """! controls the yellow LED on the board.
    @param  ser Instance of the serial class.
    @param  freq LED blinking frequency (in 0.1 Hz); max is 100 (beyond which LED stays on), min is 0 (off)
    @return  True if successful, False is unsuccessful
    """

    on = freq > 0

    return send_on_off_cmd(ser, 'B', on)
    

def heater1_switch(ser, power: float):
    """! controls heater 1 (the high temperature one).
    @param  ser Instance of the serial class.
    @param  power normilized power level from 0.0 (off) to 1.0 (on) -- currently supported only 0 or 1, and nothing in between
    @return  True if successful, False is unsuccessful
    """

    on = power > 0.5

    return send_on_off_cmd(ser, 'H', on)
    


# thermistor data (TODO: refine it later)
ADC = [970,988,1007,1025,1045,1064,1084,1104,1124,1145,1167,1188,1210,1232,1255,1278,1301,1325,1349,1373,1398,1423,1449,1475,1501,1527,1554,1581,1609,1637,1665,1694,1722,1752,1781,1811,1841,1871,1901,1932,1963,1994,2026,2058,2089,2121,2154,2186,2218,2251,2284,2316,2349,2382,2415,2448,2481,2513,2546,2579,2612,2644,2677,2709,2741,2773,2805,2836,2868,2899,2930,2960,2990,3020,3050,3079,3108,3136,3164,3192,3219,3246,3273,3299,3324,3349,3374,3398,3422,3445,3467,3490,3511,3532,3553,3573,3593,3612,3631,3649,3666]
T = [100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0]

heater1_temperature_data = [0,0,0] # 3 last measurements for median filter
heater2_temperature_data = [0,0,0] # 3 last measurements for median filter

def heater1_get_temp(ser):
    """! get the temperature of heater 1 (the high temperature one).
    @param  ser Instance of the serial class.
    @return  Floating point value of temperature degrees in C, or None if unsuccessful
    """

    cmd = 'A'
    param = 5 # arbitrary number to check the loopback

    for _ in range(RETRY_COUNT): # retry count
        cmd_str = cmd + '%d' % param
        ser.write(cmd_str.encode())

        start_time = time.time()
        response = 0
        while time.time() - start_time < TIME_OUT_S:
            if ser.in_waiting > 2:
                try:
                    response = ser.read().decode()

                    byte_h = ser.read()
                    int_h = ord(byte_h)
                    byte_l = ser.read()
                    int_l = ord(byte_l)
                    adc = (int_h << 8) | int_l
                    adc = adc >> 4

                    if response == '%d' % param:  # we should receive the same parameter value as acknowledgement
                        #return adc
                        heater1_temperature_data.pop(0)
                        heater1_temperature_data.append(adc)
                        adc_filtered = np.median(heater1_temperature_data)

                        return np.interp(adc_filtered, ADC, T)
                    else:
                        ser.read_all() # flush the buffer

                except:
                    pass # this will exit from the reading loop and we will retry

                break

    return None
    
    
def heater2_get_temp(ser):
    """! get the temperature of heater 2 (the low temperature one).
    @param  ser Instance of the serial class.
    @return  Floating point value of temperature degrees in C, or None if unsuccessful
    """

    cmd = 'a'
    param = 5 # arbitrary number to check the loopback

    for _ in range(RETRY_COUNT): # retry count
        cmd_str = cmd + '%d' % param
        ser.write(cmd_str.encode())

        start_time = time.time()
        response = 0
        while time.time() - start_time < TIME_OUT_S:
            if ser.in_waiting > 2:
                try:
                    response = ser.read().decode()

                    byte_h = ser.read()
                    int_h = ord(byte_h)
                    byte_l = ser.read()
                    int_l = ord(byte_l)
                    adc = (int_h << 8) | int_l
                    adc = adc >> 4

                    if response == '%d' % param:  # we should receive the same parameter value as acknowledgement
                        #return adc
                        heater2_temperature_data.pop(0)
                        heater2_temperature_data.append(adc)
                        adc_filtered = np.median(heater2_temperature_data)

                        return np.interp(adc_filtered, ADC, T)
                    else:
                        ser.read_all() # flush the buffer

                except:
                    pass # this will exit from the reading loop and we will retry

                break

    return None


def heater_get_temperatures(ser):
    # this function only works with the Rev 2 firmware that splits the temperature data by itself

    # the message starts with 0x55,0xAA -- waiting for it
    while True:
        b = ser.read()
        if ord(b) != 0x55:
            continue
        b = ser.read()
        if ord(b) == 0xAA:
            break

    byte_h = ser.read()
    int_h = ord(byte_h)
    byte_l = ser.read()
    int_l = ord(byte_l)
    adc1 = (int_h << 8) | int_l

    byte_h = ser.read()
    int_h = ord(byte_h)
    byte_l = ser.read()
    int_l = ord(byte_l)
    adc2 = (int_h << 8) | int_l

    ser.read_all() # flush all the accumulated data in case we are reading slower than the data is appearing

    heater1_temperature_data.pop(0)
    heater1_temperature_data.append(adc1)
    adc1_filtered = np.median(heater1_temperature_data)
    temp1 = np.interp(adc1_filtered, ADC, T)

    heater2_temperature_data.pop(0)
    heater2_temperature_data.append(adc2)
    adc2_filtered = np.median(heater2_temperature_data)
    temp2 = np.interp(adc2_filtered, ADC, T)

    return temp1, temp2

def send_ref_cmd(ser, cmd, ref):
    for _ in range(RETRY_COUNT): # retry count
        ser.read_all() # to get rid of all residual data

        cmd_str = cmd
        ser.write(cmd_str.encode())
        packet = bytearray()
        packet.append(ref >> 8)
        packet.append(ref & 0xFF)
        ser.write(packet)

        start_time = time.time()
        response = 0
        while time.time() - start_time < TIME_OUT_S:
            if ser.in_waiting >= 2:
                try:
                    response = ser.read(2)
                except:
                    pass # this will exit from the reading loop and we will retry

                break

        if packet == response: # we should receive the same parameter value as acknowledgement
            return True

    return False

def heater1_regulate(ser, reference_T):
    # first we determine the ADC code corresponding to the reference temperature
    T_ = T[::-1]
    ADC_ = ADC[::-1]
    reference_adc = int(np.interp(reference_T, T_, ADC_))
    return send_ref_cmd(ser, 'T', reference_adc)

def set_Kp(ser, Kp):
    return send_ref_cmd(ser, 'P', int(Kp))

def set_Ki(ser, Ki):
    return send_ref_cmd(ser, 'I', int(Ki))

def set_Kd(ser, Kd):
    return send_ref_cmd(ser, 'D', int(Kd))

def list_serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
