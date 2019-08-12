from roboclaw import Roboclaw

#Windows comport name
rc = Roboclaw("ttyUSB0",115200)
#Linux comport name
#rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
