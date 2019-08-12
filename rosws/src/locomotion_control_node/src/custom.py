
#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000
import time
from roboclaw import Roboclaw
rc = Roboclaw("/dev/ttyACM0",115200)
def displayspeed():
    enc1 = rc.ReadEncM1(address)
    enc2 = rc.ReadEncM2(address)
    speed1 = rc.ReadSpeedM1(address)
    speed2 = rc.ReadSpeedM2(address)
    print "Motor 1 PWM:"
    print rc.ReadPWMs(address)
    print("Encoder1:"),
    if(enc1[0]==1):
        print enc1[1],
        print format(enc1[2],'02x'),
    else:
        print "failed",
    print "Encoder2:",
    if(enc2[0]==1):
        print enc2[1],
        print format(enc2[2],'02x'),
    else:
        print "failed " ,
    print "Speed1:",
    if(speed1[0]):
        print speed1[1],
    else:
        print "failed",
    print("Speed2:"),
    if(speed2[0]):
        print speed2[1]
    else:
        print "failed "

rc.Open()
address = 0x80

version = rc.ReadVersion(address)
if version[0]==False:
    print "GETVERSION Failed"
else:
    print repr(version[1])

while(1):

    #rc.S20000
    vel=10000
    acc=5000
    pwm = 128
    duty = 64
    
    #rc.SpeedAccelM1(address, acc, vel)
    #rc.ForwardBackwardM2(address, 64+duty)
    #rc.SpeedAccelM2(address, acc, -vel)
    for i in range(0,5):
        #displayspeed()
        rc.ForwardBackwardM2(address, 32)
        time.sleep(0.05)
        #rc.ForwardBackwardM2(address, 48)
        #print rc. ReadCurrents(address)
        
    for i in range(0,60):
        #rc.ForwardBackwardM2(address, 64-i)
        time.sleep(0.05)
    # rc.SpeedM1(address,-3000)
    # #rc.SpeedM2(address,12000)
    # for i in range(0,200):
    # 	displayspeed()
    # 	time.sleep(0.01)
    #rc.SpeedAccelM1(address, acc, 0)
    
    time.sleep(0.5)
    rc.ForwardBackwardM2(address, 64)
    #rc.DutyAccelM1(address, acc_pwm, 0)
    #rc.SpeedAccelM2(address, acc, 0)
    for i in range(0,400):
        #displayspeed()
        print rc. ReadCurrents(address)
        time.sleep(0.001)
    break