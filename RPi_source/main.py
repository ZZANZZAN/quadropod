from arduino import Arduino

arduino = Arduino('')

flag = True

while(flag):
    roll, pitch, yaw = arduino.comPortListener()
    print(roll, pitch, yaw)

