import pyvisa as visa
import time

rm = visa.ResourceManager('@py')
smu = rm.open_resource("ASRL/dev/ttyACM0::INSTR")

print(smu.query("*IDN?"))

voltage = float(smu.query("MEAS:VOLT:DC:IN?"))
print(voltage)

voltage = float(smu.query("MEAS:VOLT:DC:OUT?"))
print(voltage)

voltage = float(smu.query("MEAS:VOLT:DC:CONT? 1"))
print(voltage)

voltage = float(smu.query("MEAS:VOLT:DC:CONT? 2"))
print(voltage)

#for i in range(0, 5000):
#    smu.write("CONF:DEAD %f,%f" % ((i / 100.0),(i / 100.0)))
smu.write("CONF:DEAD %d,%d" % (0,0))
for i in range(100, 10000):
    smu.write("CONF:FREQ %i" % (i * 1000))

#for i in range(100, 10000):
#    smu.write("CONF:FREQ %i" % ((10100 - i) * 1000))
#smu.write("CONF:FREQ %i" % (1000000))

#for i in range(0, 511):
#    smu.write("CONF:DEAD %d,%d" % (i,i))
#    time.sleep(0.01)

#4608
