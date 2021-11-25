#!/usr/bin/env python2

"""
@description: Weighing scale node. Interface with the dymo M10 weighing scale unit
@file: dymoM10.py
@date: Nov 2021
@author: Lee Chan Wai <chanwai.lee@dyson.com>
@notes : Copyright (c) 2021 Dyson technology Ltd, All Rights
"""

import usb.core
import usb.util
import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
import sys
import Jetson.GPIO as GPIO

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

class WeighingScale:

    namespace = 0
    idVendor = 0
    idProduct = 0
    data = 0
    isReady = False

    def __init__(self, namespace, data = None, isReady = False):
        self.namespace = namespace
        self.idVendor = VENDOR_ID
        self.idProduct = PRODUCT_ID
        self.data = data
        self.device = None
        self.isReady = isReady
        self.mass = None

        # self.namespacePubber = None
        self.massPubber = None
        self.isReadyPubber = None

        self.startTime = rospy.get_rostime()
        self.timeInterval = rospy.Duration(secs=10) #units in seconds
        self.powerPin = 11
        self.unitsSwitchPin = 12
        self.tarePin = 13
        
        self.initPubbers()

        self.initJetsonGPIO()
        # self.toggleUnits()
    
    # def initNamespacePubber(self):
    #     self.namespacePubber = rospy.Publisher("namespace", string, queue_size=1)

    def initPubbers(self):
        topic = self.namespace + "/mass"
        self.massPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.namespace + "/is_ready"
        self.isReadyPubber = rospy.Publisher(topic, Bool, queue_size=10)
        print("pubbers ready.")

    def initJetsonGPIO(self):
        GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
        # set pin as an output pin with optional initial state of HIGH
        GPIO.setup(self.powerPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.unitsSwitchPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.tarePin, GPIO.OUT, initial=GPIO.LOW)

    def initInterruptSubber(self):
        topic = "/" + self.ns + "/interrupt"
        self.interruptSubber - rospy.Subscriber(topic, Bool, self.gpioInterruptCallback)

    def gpioInterruptCallback(self, msg):
        self.isReady = False
        rospy.logwarn("device is {}".format(self.isReady))
        if msg == (self.powerPin or self.unitsSwitchPin or self.tarePin):
            # pin trigger code
            pass
        self.isReady = True
        rospy.logwarn("device is {}".format(self.isReady))

    # function to toggle the weighing scale between kg/g and lb/oz to prevent inactivity
    def toggleUnits(self):
        try:
            self.startTime = rospy.get_rostime()
            prevIsReady = self.isReady

            rospy.logwarn("Toggling units now")

            self.isReady = False

            GPIO.output(self.unitsSwitchPin, GPIO.HIGH)
            rospy.sleep(1)
            GPIO.output(self.unitsSwitchPin, GPIO.LOW)
            rospy.sleep(1)
            GPIO.output(self.unitsSwitchPin, GPIO.HIGH)
            rospy.sleep(1)
            GPIO.output(self.unitsSwitchPin, GPIO.LOW)
            
            if prevIsReady == True:
                self.isReady = True

        except usb.core.USBError as e:
            print(e)
            rospy.core.logwarn("USB error: Device not found")
            self.data = None
            self.isReady = False

        except KeyboardInterrupt:
                print("bye at toggle")

    def publishIsReady(self):
        self.isReadyPubber.publish(self.isReady)

    def initializeDevice(self):
        rospy.core.loginfo("Identifying...")
        self.device = usb.core.find(idVendor=self.idVendor, idProduct=self.idProduct)
        if self.device is not None:
            rospy.core.loginfo("Device {}:{} found!" .format(hex(self.idVendor), hex(self.idProduct)))
            rospy.core.loginfo("Device {}:{} is initializing!" .format(hex(self.idVendor), hex(self.idProduct)))
            usb.util.dispose_resources(self.device)
            try:
                c = 1
                for config in self.device:
                    print 'config', c
                    print 'Interfaces', config.bNumInterfaces
                    for i in range(config.bNumInterfaces):
                        print("checking if kernel driver {} is active while c = {}".format(i, c))
                        if self.device.is_kernel_driver_active(i):
                            rospy.loginfo("detaching kernel driver {}".format(i))
                            self.device.detach_kernel_driver(i)
                        # usb.util.claim_interface(self.device, i)
                        # rospy.loginfo("claiming interface {}".format(i))
                    c += 1

                self.device.set_configuration()
                rospy.core.loginfo("Device config set!")
                self.isReady = True
            
            except usb.core.USBError as e:
                print(e)
                rospy.core.logwarn("USB error: Device not found")
                self.data = None
                self.isReady = False
                
            except KeyboardInterrupt:
                print("bye at initialize")

    def publishMass(self):
        data = None
        mass = None

        if self.isReady == True:
            try:
                # first endpoint
                endpoint = self.device[0][(0, 0)][0]

                # read a data packet
                self.data = self.device.read(endpoint.bEndpointAddress,
                                         endpoint.wMaxPacketSize)

                self.mass = self.data[4] + (256 * self.data[5])
                if self.data[2] == 11:
                    self.mass *= math.pow(10, 254 - self.data[3])

                rospy.loginfo(self.mass)
                self.massPubber.publish(self.mass)

            except usb.core.USBError as e:
                rospy.core.logwarn("USB error: Device not found")
                self.data = None
                self.isReady = False
                if e.args == ('Operation timed out',):
                    rospy.core.loginfo("operation timed out")
            
            except KeyboardInterrupt:
                print("bye at publish mass")

if __name__ == "__main__":
    rospy.init_node("dymoM10")
    
    try:
        if str(sys.argv[1]):
            ns = str(sys.argv[1])
        else:
            print(("No namespace input. Using /cell2 as default"))
            ns = "/cell2"  
    except IndexError:
        print("Please specify the namespace to connect to, e.g. /cell4")
        sys.exit()  

    ws = WeighingScale(namespace = ns)
    rospy.sleep(1)
    quit = False

    print("Script ready.")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        ws.publishIsReady()
        try:
            if not ws.isReady:
                ws.initializeDevice()
            else:
                ws.publishMass()

            now =  rospy.get_rostime()
            if (rospy.get_rostime() - ws.startTime >= ws.timeInterval):
                ws.toggleUnits()

            rate.sleep()

        except KeyboardInterrupt:
            GPIO.cleanup()
            print("bye")
            quit = True