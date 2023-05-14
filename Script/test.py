import time

from controller import *


ser = MCU_open('COM3')

Kp = 200
Ki = 10
Kd = 0

Kp = 500
Ki = 50
Kd = 0

Kp = 100
Ki = 5
Kd = 0

resp = set_Kp(ser, Kp)
print(resp)
resp = set_Ki(ser, Ki)
print(resp)
resp = set_Kd(ser, Kd)
print(resp)



resp = heater1_regulate(ser, 93)
print(resp)

start_time = time.time()
while True:
    resp = heater_get_temperatures(ser)
    print(resp)
    T = resp[0]

    time.sleep(1)
    f = open("Log.csv", 'a')
    stamp = time.time() - start_time
    f.write(str(stamp) + ',' + str(T) + '\n')
    f.close()
