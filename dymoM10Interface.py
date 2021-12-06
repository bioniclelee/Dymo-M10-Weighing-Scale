#!/usr/bin/env python2

"""
@description: Weighing scale interface. Connects to the topics published by the 
weighing scale and provides additional functions.
@file: dymoM10Interface.py
@date: Nov 2021
@author: Lee Chan Wai <chanwai.lee@dyson.com>
@notes : Copyright (c) 2021 Dyson technology Ltd, All Rights
"""

import math
import rospy
from std_msgs.msg import Float64, Bool, String, Int32
import sys

class WeighingScaleInterface:

    def __init__(self, ns, staticAverageMassArrSize = 10, movingAverageMassArrSize = 10):
        self.ns = ns
        self.isReady = False
        self.prevIsReady = False
        self.isReadySubber = None
        
        self.tempMassArr = []

        self.movingAverageMassArrSize = movingAverageMassArrSize

        self.prevAverageMass = 0
        self.prevMassArr = []
        self.currAverageMass = 0
        self.currMassArr = []
        self.weightChanged = False
        self.threshold = 0.3 #units in grams

        self.movingAverageMassPubber = None
        self.staticAverageMassPubber = None
        self.weightChangePubber = None

        self.gpioInterruptPubber = None
        self.gpioPin = None

        self.initPubbers()
        rospy.loginfo("pubbers initialized")
        self.initIsReadySubber()
        rospy.loginfo("is_ready subber initialized")
        self.initMovingAverageMassSubber()
        rospy.loginfo("moving average mass subbers initialized")
        # self.initNamespaceSubber()

    def initPubbers(self):
        topic = self.ns + "/movingAverageMass"
        self.movingAverageMassPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.ns + "/staticAverageMass"
        self.staticAverageMassPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.ns + "/weight_changed"
        self.weightChangePubber = rospy.Publisher(topic, Bool, queue_size=1)
        topic = self.ns + "/gpio_interrupt"
        self.gpioInterruptPubber = rospy.Publisher(topic, Int32, queue_size=1)

    def initIsReadySubber(self, topic = "/is_ready"):
        topic = "/" + self.ns + topic
        # print(topic)
        self.isReadySubber = rospy.Subscriber(topic, Bool, self.update_is_ready)

    def update_is_ready(self, msg):
        if msg.data:
            status = "ready"
        else:
            status = "not ready"
            rospy.logwarn("weighing scale is not ready")
        self.isReady = msg.data
    
    def is_ready(self):
        return (self.isReady and ((len(self.tempMassArr)) 
                                    >= 2 * self.movingAverageMassArrSize))

    def initMovingAverageMassSubber(self, topic = "/mass"):
        topic = "/" + self.ns + topic
        self.movingAverageMassSubber = rospy.Subscriber(topic, Float64, self.updateMassArr)
                                                        # callback_args = self)

    def updateMassArr(self, msg):
        if (self.prevIsReady == False and self.isReady == True):
            rospy.sleep(1)
        if (len(self.tempMassArr)) == (2 * self.movingAverageMassArrSize):
            self.tempMassArr = self.pushPop(self.tempMassArr, msg.data)
            self.prevAverageMass = self.tempMassArr[: self.movingAverageMassArrSize]
            self.currMassArr = self.tempMassArr[self.movingAverageMassArrSize:]
            # self.prevMovingAverage = self.currMovingAverage
            # self.currMovingAverage = sum(self.movingAverageMassArr)/self.movingAverageMassArrSize
        else:
            self.tempMassArr.append(msg.data)
        
        self.prevIsReady = self.isReady
    
    def updateMovingAverageMassArrSize(self, newSize):
        self.movingAverageMassArrSize = newSize

    # def returnMovingAverageMassArr(self):
    #     return self.movingAverageMassArr

    def getCurrMassArr(self):
        while not (self.prevIsReady == True and self.isReady == True):
            rospy.logwarn("Waiting for device to be ready...")
            rospy.sleep(1)
        return self.currMassArr

    def getPrevMassArr(self):
        while not (self.prevIsReady == True and self.isReady == True):
            rospy.logwarn("Waiting for device to be ready...")
            rospy.sleep(1)
        return self.prevMassArr

    def pushPop(self, list, newElem):
        newList = []
        for i in range(1, len(list)-1):
            # rospy.loginfo("newList index:{} | list index:{}".format(i-1, i))
            newList.append(list[i])
            # rospy.loginfo("assigned list[{}] to newList[{}]".format(i, i-1))
        newList.append(newElem)
        return newList         

    # def initStaticAverageMassSubber(self, topic = "/staticAverageMass"):
    #     topic = "/" + self.ns + topic
    #     self.staticAverageMassSubber = rospy.Subscriber(topic, Float64,
    #                                             self.updateStaticAverageMassArr,
    #                                             callback_args = self)

    # def updateStaticAverageMassArr(msg, self):
    #     if self.staticAverageMassArrSize != (len(self.staticAverageMassArr)):
    #         # self.prevStaticAverage = self.currStaticAverage
    #         # self.currStaticAverage = sum(self.staticAverageMassArr)/self.massArrSize
    #         rospy.loginfo("msg is", type(msg))
    #         self.staticMassArr.append(Float64(msg))

    def getWeightChanged(self):
        while not (self.prevIsReady == True and self.isReady == True):
            rospy.logwarn("Waiting for device to be ready...")
            rospy.sleep(1)
        return self.weightChanged
    
    def setWeightChanged(self, newStatus):
        self.weightChanged = newStatus

    def update(self):
        rospy.loginfo("self.currAverageMass = {} | self.prevAverageMass = {}"
        .format(self.currAverageMass, self.prevAverageMass))

        if self.is_ready:
            self.currAverageMass = sum(self.currMassArr)/len(self.currMassArr)
            # self.currAverageMass = sum(self.movingAverageMassArr)/float(self.movingAverageMassArrSize)
            self.movingAverageMassPubber.publish(self.currAverageMass)

            self.weightChanged = self.checkWeightChange(self.prevMassArr,self.currMassArr)
            rospy.logwarn("self.weightChanged: {}".format(self.weightChanged))
    
    # take in 2 lists, return bool
    def checkWeightChange(self, oldArr, newArr, threshold = None):
        weightChanged = False
                        
        if threshold == None:
            threshold = self.threshold

        oldArrAverage = 0 if len(oldArr) == 0 else sum(oldArr) / len(oldArr)
        newArrAverage = 0 if len(newArr) == 0 else sum(newArr) / len(newArr)

        if abs(oldArrAverage - newArrAverage) > threshold:
            weightChanged = True
        
        return weightChanged


    def resetWeightChangeCheck(self):
        # Resets the weight changed variable, and sets the 
        # current average force value as the comparison value for the next
        # weight change check.

        if self.weightChanged:
            self.weightChanged = False
            rospy.logwarn("self.weightChanged: {}".format(self.weightChanged))
        
        self.prevAverageMass = self.currAverageMass

if __name__ == "__main__":
    rospy.init_node("fss_test")
    
    try:
        if str(sys.argv[1]):
            ns = str(sys.argv[1])
        else:
            rospy.loginfo(("No namespace input. Using /cell2 as default"))
            ns = "/cell2"  
    except IndexError:
        print("Please specify the namespace to connect to, e.g. /cell4")
        sys.exit()  
        
    wsi = WeighingScaleInterface(ns=ns)
    quit = False

    while not quit and not rospy.is_shutdown():
            
        try:
            if wsi.is_ready():
                
                rospy.loginfo("Dymo M10 weighing scale connected at {}".format(wsi.ns))
                rospy.loginfo("This script tests the Force Sensing Surface functionality.")
                rospy.loginfo("Weight changed reset will be called every 5 seconds.")
                rospy.sleep(1)
                
                rospy.loginfo("Current Moving Average Mass: {}".format(wsi.currAverageMass))
                rospy.loginfo("updating weighing scale weight...")
                rospy.sleep(3)

                wsi.update()
                
                rospy.loginfo("Weight changed: {}".format(wsi.weightChanged))
                if wsi.weightChanged:
                    if rospy.get_time()%5 > 0.2:
                        rospy.logwarn("resetting weight change...")
                        wsi.resetWeightChangeCheck()
            
        except KeyboardInterrupt:
            quit = True