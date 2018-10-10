import numpy as np
import sys
import time
from time import sleep
import csv
import rospy
from std_msgs.msg import Int32
from random import randint

try:
    from ST_VL6180X_py2 import VL6180X
except ImportError:
    print("Error importing ST_VL6180X.VL6180X!")
    exit()

"""-- Setup --"""
debug = True
if len(sys.argv) > 1:
    if sys.argv[1] == "debug":  # sys.argv[0] is the filename
        debug = True
tof_address_new = 0x25
#for different ICS address config  BCM GPIO 16,20,21 with i2c address 21,2$
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)

print ("Output on")
GPIO.output(16,GPIO.LOW)
GPIO.output(20,GPIO.LOW)
GPIO.output(21,GPIO.LOW)
GPIO.output(16,GPIO.HIGH)
sleep(0.1)
tof_address_new=0x21
tof_address = 0x29
tof_sensor = VL6180X(address=tof_address, debug=debug)
add = tof_sensor.change_address(tof_address,tof_address_new)
print ("Changed First device address :",add)
tof_sensor_s1 = VL6180X(address=tof_address_new, debug=debug)
sleep(0.1)
#GPIO.output(16,GPIO.LOW)
GPIO.output(20,GPIO.HIGH)
sleep(0.1)
tof_address_new=0x22
tof_address = 0x29
tof_sensor = VL6180X(address=tof_address, debug=debug)
add = tof_sensor.change_address(tof_address,tof_address_new)
print ("Changed Second device address :  ",add)
tof_sensor_s2 = VL6180X(address=tof_address_new, debug=debug)
# setup ToF ranging/ALS sensor
GPIO.output(21,GPIO.HIGH)
sleep(0.1)
tof_address = 0x29
tof_address_new=0x23
tof_sensor = VL6180X(address=tof_address, debug=debug)
add = tof_sensor.change_address(tof_address,tof_address_new)
tof_sensor_s3 = VL6180X(address=tof_address_new, debug=debug)

# apply pre calibrated offset
tof_sensor.set_range_offset(23)
print("Range offset set to: {:d}".format(tof_sensor.get_range_offset()))
# setup ToF ranging/ALS sensor
tof_sensor.get_identification()
if tof_sensor.idModel != 0xB4:
    print("Not a valid sensor id: {:X}".format(tof_sensor.idModel))
else:
    print("Sensor model: {:X}".format(tof_sensor.idModel))
    print("Sensor model rev.: {:d}.{:d}"
          .format(tof_sensor.idModelRevMajor, tof_sensor.idModelRevMinor))
    print("Sensor module rev.: {:d}.{:d}"
          .format(tof_sensor.idModuleRevMajor, tof_sensor.idModuleRevMinor))
    print("Sensor date/time: {:X}/{:X}".format(tof_sensor.idDate, tof_sensor.idTime))
tof_sensor.default_settings()
tof_sensor_s1.default_settings()
tof_sensor_s2.default_settings()
temp,c_s1, c_s2,c_s3 = 0,0,0,0
sens1 = []
sens2 = [] #[0] * 101
sens3 = []

#global count_s2
print(c_s2)

def s1_callback(channel):
#    print("muntahi ami ekhane   --- callback")
    temp = tof_sensor_s1.range_result_acquire_cont()
    print("s1: ", temp)
    global c_s1,sens1
    c_s1 += 1
    sens1.append(temp)
def s2_callback(channel):
#    print("S2: Measured distance is : ",tof_sensor_s2.range_result_acquire_cont())
    temp = (tof_sensor_s2.range_result_acquire_cont())
    print("s2: ", temp)
    global c_s2,sens2
    c_s2 += 1
    sens2.append(temp)
#    print("count S2 :",c_s2)
def s3_callback(channel):
    temp =tof_sensor_s3.range_result_acquire_cont()
    print("S3: ",temp)
#    tof_sensor_s3.range_result_acquire_cont()
    global c_s3,sens3
    c_s3 += 1
    sens3.append(temp)

GPIO.setup(6, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
GPIO.setup(19, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.IN) # , pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(6, GPIO.FALLING, callback =s1_callback) #, bouncetime=1 )
GPIO.add_event_detect(19, GPIO.FALLING, callback =s2_callback) #, bouncetime=1 )
GPIO.add_event_detect(26, GPIO.FALLING, callback =s3_callback) #, bouncetime=1 )

print("gpio ----", GPIO.input(6))
sleep(0.1)
"""-- MAIN LOOP --"""
print("val:",tof_sensor_s1.range_result_acquire_cont())
print("val:",tof_sensor_s2.range_result_acquire_cont())
print("val:",tof_sensor_s3.range_result_acquire_cont())
try:
    while True:
      t1= time.time()
      while (c_s3<100):
          time.sleep(0.001)
      t2=time.time()
      print ('t2-t1 =', t2-t1)    
      print ('c_s2  =', c_s2, "c_s3",c_s3," // Len_s1 : ", len(sens1), " // Length arraay s2 : ", len(sens2), "|| Length arraay s3 : ", len(sens3))
#        x = [i for i in range(len(sens1))]
      break
    np.asarray(sens1)
    np.asarray(sens2)
    np.asarray(sens3)

    np.savetxt("output_s1.csv",sens1,delimiter =',' )
    np.savetxt("output_s2.csv",sens2,delimiter =',' )
    np.savetxt("output_s3.csv",sens3,delimiter =',' )
    np.savetxt('myfile1.txt', np.c_[sens3,sens2,sens1])

    np.asarray(sens1)
except KeyboardInterrupt:
    print("\n quit")

GPIO.remove_event_detect(6)
GPIO.cleanup()

def random_number_publisher():
    rospy.init_node('random_number')
    pub=rospy.Publisher('rand_no', Int32, queue_size=10)
    rate= rospy.Rate(2)
    while not rospy.is_shutdown():
        random_msg=randint(0,5000)
        rospy.loginfo(random_msg)
        pub.publish(random_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        random_number_publisher()
    except rospy.ROSInterruptException:
        pass
