#!/usr/bin/env python2

import usb.core
import usb.util
import math
import rospy
from std_msgs.msg import Float64

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

class WeighingScale:

    namespace = 0
    idVendor = 0
    idProduct = 0
    deviceFound = False
    data = 0
    deviceInitialized = False

    # def __init__(self, idVendor, idProduct):
    #     self.idVendor = idVendor
    #     self.idProduct = idProduct
    #     deviceFound = False

    def __init__(self, namespace, deviceFound = False, data = None, deviceInitialized = False):
        self.namespace = namespace
        self.idVendor = VENDOR_ID
        self.idProduct = PRODUCT_ID
        self.deviceFound = deviceFound
        self.data = data
        self.device = None
        self.deviceInitialized = deviceInitialized
        self.mass = None

    # bool return on whether device has been found via getDevice()
    def getDeviceFound(self):
        return self.deviceFound

    # setter for bool deviceFound flaf    
    def setDeviceFound(self, newStatus):
        newStatus = newStatus
        self.deviceFound = newStatus
        print("deviceFound = {status}".format(status = self.deviceFound))

    def getDeviceInitializedStatus(self):
        return self.deviceInitialized

    def setDeviceInitializedStatus(self, newStatus):
        self.deviceInitialized = newStatus

    def getDevice(self):
        device = usb.core.find(idVendor=self.idVendor, idProduct=self.idProduct)

        if device is not None:
            self.device = device
            self.setDeviceFound(True)
            return device

        # raise ValueError('Device not found')
        print("device not found")
        self.setDeviceFound(False)
        self.setData(None)
        return None

    def identifyDevice(self):
        print("Identifying...")
        # find the USB device
        device = self.getDevice()
        if device is not None:
            self.device = device
            print("Device %s:%s found!" %
                    (hex(self.idVendor), hex(self.idProduct)))

    def initializeDevice(self):
        print("Device %s:%s is initializing!" %
              (hex(self.idVendor), hex(self.idProduct)))
        
        device = self.device

        if device is not None:
            self.setDeviceFound(True)
            c = 1
            for config in device:
                print 'config', c
                print 'Interfaces', config.bNumInterfaces
                for i in range(config.bNumInterfaces):
                    if device.is_kernel_driver_active(i):
                        device.detach_kernel_driver(i)
                    print(i)
                c += 1

            device.set_configuration()
            print("Device config set!")

            self.setDeviceInitializedStatus(True)
        else:
            self.setDeviceFound(False)

    def getData(self):
        data = None

        if data is None and self.getDeviceFound() == True:
            try:
                # first endpoint
                endpoint = self.device[0][(0, 0)][0]

                # read a data packet
                self.setData(self.device.read(endpoint.bEndpointAddress,
                                         endpoint.wMaxPacketSize))

                print(self.getMass())

            except usb.core.USBError as e:
                print("USB error")
                self.data = None
                self.setDeviceFound(False)

                print("device not found 1")
                if e.args == ('Operation timed out',):
                    print("operation timed out")

    def setData(self, newData):
        self.data = newData

    def getMassReading(self):
        # data[2] == 2 is kg mode, 11 is lb/oz mode
        mass = self.data[4] + (256 * self.data[5])
        if self.data[2] == 11:
            mass *= math.pow(10, 254 - self.data[3])

        self.mass = mass

        return mass

    

    def publishMass(self):
        dataPublisher = rospy.Publisher("mass", Float64, queue_size=10)
        msg = self.getMass()
        if self.device is None:
            self.setDeviceFound(False)
            # raise ValueError('Device not found')
        else:
            dataPublisher.publish(msg)

if name == "main":
    rospy.init_node("dymoM10")
    ws = WeighingScale(ns = "resetSlide")
    
    try:
        if str(sys.argv[1]):
            ns = str(sys.argv[1])
        else:
            rospy.loginfo(("No namespace input. Using /cell2 as default"))
            ns = "/cell2"  
    except IndexError:
        print("Please specify the namespace to connect to, e.g. /cell4")
        sys.exit()  


    rospy.loginfo("Script ready.")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if (ws.getDeviceFound() == False):
            ws.setData(None)
            ws.identifyDevice()
        else:
            ws.initializeDevice()
            if (ws.getDeviceInitializedStatus()):
                ws.getData()
                ws.publishMass()
        # else:
        #     print("device is none")

        rate.sleep()
