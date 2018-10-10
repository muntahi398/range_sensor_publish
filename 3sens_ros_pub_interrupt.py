#!/usr/bin/env python
import rospy
import sys
import time
from time import sleep
import RPi.GPIO as GPIO
from std_msgs.msg import Int32
from random import randint
try:
    from ST_VL6180X_py2 import VL6180X
except ImportError:
    print("Error importing ST_VL6180X.VL6180X!")
    exit()

debug = True

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
GPIO.output(21,GPIO.HIGH)
sleep(0.1)
tof_address = 0x29
tof_address_new=0x23
tof_sensor = VL6180X(address=tof_address, debug=debug)
add = tof_sensor.change_address(tof_address,tof_address_new)
tof_sensor_s3 = VL6180X(address=tof_address_new, debug=debug)

tof_sensor.default_settings()
tof_sensor_s1.default_settings()
tof_sensor_s2.default_settings()
temp,c_s1, c_s2,c_s3 = 0,0,0,0
sens1 = []
sens2 = [] #[0] * 101
sens3 = []
print(c_s2)

rospy.init_node('range_sensor_val')
pub_s1=rospy.Publisher('range_val_s1', Int32, queue_size=10)
pub_s2=rospy.Publisher('range_val_s2', Int32, queue_size=10)
pub_s3=rospy.Publisher('range_val_s3', Int32, queue_size=10)

def s1_callback(channel):
    temp = tof_sensor_s1.range_result_acquire_cont()
#    print("s1: ", temp)
    global c_s1,sens1,pub_s1
#    c_s1 += 1
#    sens1.append(temp)
    pub_s1.publish(temp)
def s2_callback(channel):
    temp = (tof_sensor_s2.range_result_acquire_cont())
    global c_s2,sens2,pub_s2
    pub_s2.publish(temp)

def s3_callback(channel):
    temp =tof_sensor_s3.range_result_acquire_cont()
    global c_s3,sens3,pub_s3
    pub_s3.publish(temp)

GPIO.setup(6, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
GPIO.setup(19, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.IN) # , pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(6, GPIO.FALLING, callback =s1_callback) #, bounceti$
GPIO.add_event_detect(19, GPIO.FALLING, callback =s2_callback) #, bouncet$
GPIO.add_event_detect(26, GPIO.FALLING, callback =s3_callback) #, bouncet$

print("gpio ----", GPIO.input(6))
sleep(0.1)
#j=tof_sensor_s1.time_sensr()

print "time",tof_sensor_s1.time_sensr()
def distance_publisher():
#    rospy.init_node('range_sensor_val')
    print ("Output on")
    print("val:",tof_sensor_s1.range_result_acquire_cont())
    print("val:",tof_sensor_s2.range_result_acquire_cont())
    print("val:",tof_sensor_s3.range_result_acquire_cont())

    while not rospy.is_shutdown():
         time.sleep(0.001)
if __name__=='__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass
