#!/usr/bin/env python2

import usb.core
import usb.util
import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
import sys

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

class WeighingScale:

    namespace = 0
    idVendor = 0
    idProduct = 0
    data = 0
    deviceInitialized = False

    # def __init__(self, idVendor, idProduct):
    #     self.idVendor = idVendor
    #     self.idProduct = idProduct
    #     deviceFound = False

    def __init__(self, namespace, data = None, deviceInitialized = False):
        self.namespace = namespace
        self.idVendor = VENDOR_ID
        self.idProduct = PRODUCT_ID
        self.data = data
        self.device = None
        self.deviceInitialized = deviceInitialized
        self.mass = None

        # self.namespacePubber = None
        self.massPubber = None
        self.isReadyPubber = None

    # def initNamespacePubber(self):
    #     self.namespacePubber = rospy.Publisher("namespace", string, queue_size=1)

    def initPubbers(self):
        topic = self.namespace + "/mass"
        self.massPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.namespace + "/is_ready"
        self.isReadyPubber = rospy.Publisher(topic, Bool, queue_size=10)
        print("pubbers ready.")

    def publishIsReady(self):
        self.isReadyPubber.publish(self.deviceInitialized)

    def initializeDevice(self):
        rospy.core.loginfo("Identifying...")
        self.device = usb.core.find(idVendor=self.idVendor, idProduct=self.idProduct)
        if self.device is not None:
            rospy.core.loginfo("Device {}:{} found!" .format(hex(self.idVendor), hex(self.idProduct)))
            rospy.core.loginfo("Device {}:{} is initializing!" .format(hex(self.idVendor), hex(self.idProduct)))
            try:
                c = 1
                for config in self.device:
                    print 'config', c
                    print 'Interfaces', config.bNumInterfaces
                    for i in range(config.bNumInterfaces):
                        print("checking if kernel driver {} is active while c = {}".format(i, c))
                        if self.device.is_kernel_driver_active(i):
                            print("detaching kernel driver")
                            self.device.detach_kernel_driver(i)
                        print(i)
                    c += 1

                self.device.set_configuration()
                rospy.core.loginfo("Device config set!")
                self.deviceInitialized = True
            
            except usb.core.USBError as e:
                print(e)
                rospy.core.logwarn("USB error: Device not found")
                self.data = None
                self.deviceInitialized = False

    def publishMass(self):
        data = None
        mass = None

        if self.deviceInitialized == True:
            try:
                # first endpoint
                endpoint = self.device[0][(0, 0)][0]

                # read a data packet
                self.data = self.device.read(endpoint.bEndpointAddress,
                                         endpoint.wMaxPacketSize)

                self.mass = self.data[4] + (256 * self.data[5])
                if self.data[2] == 11:
                    self.mass *= math.pow(10, 254 - self.data[3])

                print(self.mass)
                self.massPubber.publish(self.mass)

            except usb.core.USBError as e:
                rospy.core.logwarn("USB error: Device not found")
                self.data = None
                self.deviceInitialized = False
                if e.args == ('Operation timed out',):
                    rospy.core.loginfo("operation timed out")


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

    ws.initPubbers()
    ws.publishIsReady()

    print("Script ready.")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        ws.publishIsReady()
        try:
            if not ws.deviceInitialized:
                ws.initializeDevice()
            else:
                ws.publishMass()
                
            rate.sleep()

        except KeyboardInterrupt:
                quit = True

    # def getDevice(self):
    #     device = usb.core.find(idVendor=self.idVendor, idProduct=self.idProduct)

    #     if device is not None:
    #         self.device = device
    #         self.setDeviceFound(True)
    #         return device

    #     # raise ValueError('Device not found')
    #     print("device not found")
    #     self.setDeviceFound(False)
    #     self.setData(None)
    #     return None

    # def identifyDevice(self):
    #     print("Identifying...")
    #     # find the USB device
    #     device = self.getDevice()
    #     if device is not None:
    #         self.device = device
    #         print("Device %s:%s found!" %
    #                 (hex(self.idVendor), hex(self.idProduct)))
    #     self.deviceInitialized = False

    # def initializeDevice(self):
    #     print("Device %s:%s is initializing!" %
    #           (hex(self.idVendor), hex(self.idProduct)))
        
    #     device = self.device

    #     if device is not None:
    #         self.setDeviceFound(True)
    #         # device.set_configuration()
    #         c = 1
    #         for config in device:
    #             print 'config', c
    #             print 'Interfaces', config.bNumInterfaces
    #             for i in range(config.bNumInterfaces):
    #                 print("checking if kernel driver {} is active while c = {}".format(i, c))
    #                 if device.is_kernel_driver_active(i):
    #                     print("detaching kernel driver")
    #                     device.detach_kernel_driver(i)
    #                 print(i)
    #             c += 1

    #         device.set_configuration()
    #         print("Device config set!")

    #         self.setDeviceInitializedStatus(True)
    #     else:
    #         self.setDeviceFound(False)
    #         self.setDeviceInitializedStatus(False)

    #     print("self.deviceInitialized = {}".format(self.getDeviceInitializedStatus()))

    # def getMassReading(self):
    # if self.deviceInitialized:
    # # data[2] == 2 is kg mode, 11 is lb/oz mode
    #     mass = self.data[4] + (256 * self.data[5])
    #     if self.data[2] == 11:
    #         mass *= math.pow(10, 254 - self.data[3])

    #     self.mass = mass

    #     return mass

    # rospy.logwarn("Device not found")

    # def publishMass(self):
    # mass = self.getMassReading()
    # if self.device is None:
    #     self.deviceFound = False
    #     rospy.logwarn("Device not found")
    # else:
    #     self.massPubber.publish(mass)

    #     dataPublisher = rospy.Publisher(self.namespace + "/mass", Float64, queue_size=10)
    #     msg = self.getMassReading()
    #     if self.device is None:
    #         self.setDeviceFound(False)
    #         print("Device not found")
    #         # raise ValueError('Device not found')
    #     else:
    #         dataPublisher.publish(msg)