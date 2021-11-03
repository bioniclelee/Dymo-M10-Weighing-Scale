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
        rospy.loginfo("pubbers ready.")

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
        self.setDeviceInitializedStatus(False)

    def initializeDevice(self):
        print("Device %s:%s is initializing!" %
              (hex(self.idVendor), hex(self.idProduct)))
        
        device = self.device

        if device is not None:
            self.setDeviceFound(True)
            # device.set_configuration()
            c = 1
            for config in device:
                print 'config', c
                print 'Interfaces', config.bNumInterfaces
                for i in range(config.bNumInterfaces):
                    rospy.loginfo("checking if kernel driver {} is active while c = {}".format(i, c))
                    if device.is_kernel_driver_active(i):
                        rospy.loginfo("detaching kernel driver")
                        device.detach_kernel_driver(i)
                    print(i)
                c += 1

            device.set_configuration()
            print("Device config set!")

            self.setDeviceInitializedStatus(True)
        else:
            self.setDeviceFound(False)
            self.setDeviceInitializedStatus(False)

        rospy.loginfo("self.deviceInitialized = {}".format(self.deviceInitialized))
        self.isReadyPubber.publish(self.getDeviceInitializedStatus())
            

    def getData(self):
        data = None

        if data is None and self.getDeviceFound() == True:
            try:
                # first endpoint
                endpoint = self.device[0][(0, 0)][0]

                # read a data packet
                self.setData(self.device.read(endpoint.bEndpointAddress,
                                         endpoint.wMaxPacketSize))

                print(self.getMassReading())

            except usb.core.USBError as e:
                print("USB error")
                self.data = None
                self.setDeviceFound(False)

                print("device not found")
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
        mass = self.getMassReading()
        if self.device is None:
            self.setDeviceFound(False)
            rospy.loginfo("Device not found")
        else:
            self.massPubber.publish(mass)

    #     dataPublisher = rospy.Publisher(self.namespace + "/mass", Float64, queue_size=10)
    #     msg = self.getMassReading()
    #     if self.device is None:
    #         self.setDeviceFound(False)
    #         rospy.loginfo("Device not found")
    #         # raise ValueError('Device not found')
    #     else:
    #         dataPublisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node("dymoM10")
    
    try:
        if str(sys.argv[1]):
            ns = str(sys.argv[1])
        else:
            rospy.loginfo(("No namespace input. Using /cell2 as default"))
            ns = "/cell2"  
    except IndexError:
        print("Please specify the namespace to connect to, e.g. /cell4")
        sys.exit()  

    ws = WeighingScale(namespace = ns)
    rospy.sleep(1)
    quit = False

    ws.initPubbers()

    rospy.loginfo("Script ready.")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            if not ws.getDeviceFound():
                ws.setData(None)
                ws.identifyDevice()
            elif ws.getDeviceInitializedStatus():
                    ws.getData()
                    ws.publishMass()
            else:
                ws.initializeDevice()

            rate.sleep()
        except KeyboardInterrupt:
                quit = True