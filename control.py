import pyvisa as visa

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

smu.write("CONF:VOLT:DC:OUT 42.23")
