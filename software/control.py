import pyvisa as visa
import time

rm = visa.ResourceManager('@py')
smu = rm.open_resource("ASRL/dev/ttyACM0::INSTR")

print(smu.query("*IDN?"))

voltage = float(smu.query("MEAS:VOLT:DC:IN?"))
print("Input voltage: %fV" % voltage)

while True:
    knob1 = float(smu.query("MEAS:VOLT:DC:CONTrol? 1"))
    knob2 = float(smu.query("MEAS:VOLT:DC:CONTrol? 2"))
    setvoltage = knob1 * 10.0
    setfreq = knob2 * 2000000 + 1000000
    smu.write("CONF:FREQ %i" % (setfreq))
    smu.write("CONF:VOLT:DC:OUT %i" % (setvoltage))
    time.sleep(0.1)

    isvoltage = float(smu.query("MEAS:VOLT:DC:OUT?"))
    iscurrent = float(smu.query("MEAS:CURR:DC:IN?"))
    print("Amplifier Voltage: %fV, Power: %fW, Frequency: %fMHz" % (isvoltage, iscurrent * 5.0, setfreq / 1000000.0), end='\r')
