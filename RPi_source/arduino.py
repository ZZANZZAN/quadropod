import serial
import numpy as np

class Arduino:
    def __init__(self, COMport):
        self.ser = serial.Serial(COMport, 115000, timeout = 1)
        self.ser.flush()
        
    def comPortListener(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').rstrip()
            if data[0] == 'D':
                ndata = data[1,data.len()-1]
                roll, pitch, yaw = np.fromstring(ndata, sep = '$')
                return roll, pitch, yaw

    def comPortTransmitter(self, angle):
        data = "S" + angle.joint('$') + "#"
        self.ser.write(data .encode ('utf-8'))
