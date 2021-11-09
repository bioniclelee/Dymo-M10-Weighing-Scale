#!/usr/bin/env python2

"""
@description: Weighing scale interface. Connects to the topics published by the 
weighing scale and provides additional functions.
@file: dymoM10Interface.py
@date: Nov 2021
@author: Lee Chan Wai <chanwai.lee@dyson.com>
@notes : Copyright (c) 2020 Dyson technology Ltd, All Rights
"""

import math
import rospy
from std_msgs.msg import Float64, Bool, String
import sys

class WeighingScaleInterface:

    def __init__(self, ns, staticAverageMassArrSize = 10, movingAverageMassArrSize = 10):
        self.ns = ns
        self.isReady = False
        self.isReadySubber = None
        
        self.staticAverageMassSubber = None
        # self.staticAverageWeightChanged = False
        self.staticAverageMassArr = []
        self.staticAverageMassArrSize = staticAverageMassArrSize
        # self.prevStaticAverage = 0
        # self.currStaticAverage = 0
        self.staticAverageMassThreshold = 0.3

        self.movingAverageMassSubber = None
        # self.movingAverageWeightChanged = False
        self.movingAverageMassArr = []
        self.movingAverageMassArrSize = movingAverageMassArrSize
        # self.prevMovingAverage = 0
        # self.currMovingAverage = 0
        self.movingAverageMassThreshold = 0.3

        self.prevAverageMass = 0
        self.currAverageMass = 0
        self.weightChanged = False
        self.threshold = 0.3

        self.movingAverageMassPubber = None
        self.staticAverageMassPubber = None
        self.weightChangePubber = None

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

    # def initNamespaceSubber(self, topic = "namespace"):
    #     self.namespaceSubber = rospy.Subscriber(topic, String,
    #                                             self.updateNameSpace,
    #                                             callback_args = self)

    # def updateNamespace(msg, self):
    #     self.ns = msg.data
    #     rospy.loginfo("Namespace updated as {}".format(self.ns))

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
        # rospy.loginfo("Weighing scale at {} is {}".format(self.ns, status))
        self.isReady = msg.data
    
    def is_ready(self):
        return self.isReady

    def initMovingAverageMassSubber(self, topic = "/mass"):
        topic = "/" + self.ns + topic
        self.movingAverageMassSubber = rospy.Subscriber(topic, Float64, self.updateMovingAverageMassArr)
                                                        # callback_args = self)

    def updateMovingAverageMassArr(self, msg):
        # rospy.loginfo("current mass reading: {}".format(msg.data))
        # rospy.loginfo("len(movingAverageMass):{}".format(len(self.movingAverageMassArr)))
        if self.movingAverageMassArrSize == (len(self.movingAverageMassArr)):
            self.movingAverageMassArr = self.pushPop(self.movingAverageMassArr, msg.data)
            # self.prevMovingAverage = self.currMovingAverage
            # self.currMovingAverage = sum(self.movingAverageMassArr)/self.movingAverageMassArrSize
        else:
            self.movingAverageMassArr.append(msg.data)

    def pushPop(self, list, newElem):
        newList = []
        for i in range(1, len(list)-1):
            # rospy.loginfo("newList index:{} | list index:{}".format(i-1, i))
            newList.append(list[i])
            # rospy.loginfo("assigned list[{}] to newList[{}]".format(i, i-1))
        newList.append(newElem)
        return newList         

    def initStaticAverageMassSubber(self, topic = "/staticAverageMass"):
        topic = "/" + self.ns + topic
        self.staticAverageMassSubber = rospy.Subscriber(topic, Float64,
                                                self.updateStaticAverageMassArr,
                                                callback_args = self)

    def updateStaticAverageMassArr(msg, self):
        if self.staticAverageMassArrSize != (len(self.staticAverageMassArr)):
            # self.prevStaticAverage = self.currStaticAverage
            # self.currStaticAverage = sum(self.staticAverageMassArr)/self.massArrSize
            rospy.loginfo("msg is", type(msg))
            self.staticMassArr.append(Float64(msg))

    # def getWeightChanged(self):
    #     return self.weightChanged
    
    # def setWeightChanged(self, newStatus):
    #     self.weightChanged = newStatus

    def resetWeightChangeCheck(self):
        # Resets the weight changed variable, and sets the 
        # current average force value as the comparison value for the next
        # weight change check.
        if self.weightChanged:
            self.weightChanged = False
            rospy.logwarn("self.weightChanged: {}".format(self.weightChanged))
        self.prevAverageMass = self.currAverageMass

    def update(self, mode):
        self.prevAverage = self.currAverageMass
        if mode == 1:
            # print("datatype of self.movingAverageMassArrSize:", type(self.movingAverageMassArrSize))
            # print("datatype of sum(self.movingAverageMassArr:", type(sum(self.movingAverageMassArr)))
            self.currAverageMass = sum(self.movingAverageMassArr)/len(self.movingAverageMassArr)
            # self.currAverageMass = sum(self.movingAverageMassArr)/float(self.movingAverageMassArrSize)
            self.movingAverageMassPubber.publish(self.currAverageMass)
        else:
            self.currAverageMass = sum(self.staticAverageMassArr)/self.staticMassArrSize
            self.staticAverageMassPubber.publish(self.currAverageMass)

        rospy.loginfo("self.currAverageMass = {} | self.prevAverageMass = {}"
        .format(self.currAverageMass, self.prevAverageMass))

        if abs(self.currAverageMass - self.prevAverageMass) > self.threshold:
            self.weightChanged = True
            rospy.logwarn("self.weightChanged: {}".format(self.weightChanged))

    # def getCurrAverage(self):
    #     return self.currAverageMass

    # def getPrevAverage(self):
    #     return self.prevAverageMass

    # def recalibrate(self): 
    #     # Setup rospy service proxy for calling the recalibration service
    #     recalib = rospy.ServiceProxy(self._ns + '/force_surface_sensor/recalibrate', Empty)
    #     time.sleep(1)
    #     try:
    #         # Checks if service is available with a 3s timeout
    #         rospy.wait_for_service(self._ns + '/force_surface_sensor/recalibrate', timeout=3)
    #         # Calls service
    #         response = recalib()
    #     except rospy.ServiceException as exc:
    #         # Catch the 'failed' service call exception raised due to the rosserial v0.8 bug
    #         # print str(exc)
    #         pass

    #     # If service is not available
    #     except rospy.ROSException:
    #         rospy.loginfo('Service unavailable. \
    #                       Double check that the cell number is valid.')

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

    # rospy.loginfo("ns = {}".format(self.ns))
        
    wsi = WeighingScaleInterface(ns=ns)
    quit = False

    while not quit and not rospy.is_shutdown():
        # wsi.is_ready()

        # rospy.loginfo("wsi is {}".format(wsi.isReady))
            
        try:
            if wsi.is_ready():
                rospy.loginfo("Dymo M10 weighing scale Connected at {}".format(wsi.ns))
                rospy.loginfo("This script tests the Force Sensing Surface functionality.")
                # rospy.loginfo("Recalibrating...")
                # fss.recalibrate()
                rospy.loginfo("Weight changed reset will be called every 5 seconds.")
                rospy.sleep(1)

                # rospy.loginfo("__main__ is running")
                # print("Current mass: {}".format(wsi.))
                rospy.loginfo("Current Moving Average Mass: {}".format(wsi.currAverageMass))
                # print("Current Force Location: {}".format(fss.force_location))
                # print(fss.get_all_data())
                # print("")
                rospy.loginfo("updating weighing scale weight...")
                rospy.sleep(3)
                wsi.update(1)
                rospy.loginfo("Weight changed: {}".format(wsi.weightChanged))
                if wsi.weightChanged:
                    rospy.logwarn("resetting weight change...")
                    wsi.resetWeightChangeCheck()
            
        except KeyboardInterrupt:
            quit = True

# Chan Wai Lee, [28/10/21 6:07 PM]
# #!/usr/bin/env python

# """
# @description: Force Sensing surface interface. Connects to the topics published by the FSS and provides additional functions.
# @file: force_surface_interface.py
# @date: Jan 2021
# @author: Michal Rachowiecki <michal.rachowiecki@dyson.com>
# @notes : Copyright (c) 2020 Dyson technology Ltd, All Rights
# """

# import numpy as np
# from surface_msgs.msg import ForcePositionStamped
# import rospy
# from std_msgs.msg import Float32, Bool
# from collections import deque
# from std_srvs.srv import Empty
# import time
# import sys

# class ForceSurface(object):

#     def init(self, ns, weight_check_threshold = 0.3): #0.15
#         self._ns = ns
#         self._ready = False
#         self._surface_sub = None
#         self.moving_avg_force = []
#         self.force_location = []
#         self.force_value = []
        
#         self.buffer = []
#         self.maxlen = 300
#         self.hidden_buf = deque([],self.maxlen)
#         self.data = Float32
        
#         self.weight_changed = False
#         self.curr_avg = 0.0
#         self.threshold = weight_check_threshold
#         self.prev_avg = 0.0
        
#         self.init_sub()
#         self.init_pubs()

#     def is_ready(self):
#         return self._ready

#     def init_sub(self, topic_name="/force_surface_sensor/surface_data"):
#         topic_name = "%s%s" % (self._ns, topic_name)
#         self._surface_sub = rospy.Subscriber(
#                                     topic_name,
#                                     ForcePositionStamped,
#                                     self.update,
#                                     queue_size=10)
#         self._ready = True

#     def init_pubs(self):
#         topic_name = self._ns + "/surface_data/moving_average_filter"
#         self.ma_pub = rospy.Publisher(topic_name, Float32, queue_size=1)
#         topic_name = self._ns + "/surface_data/weight_changed"
#         self.wc_pub = rospy.Publisher(topic_name, Bool, queue_size=1)

#     def update(self,msg):
#         self.force_value = msg.fp.force.z
#         self.force_location = msg.fp.location
#         self.hidden_buf.append(self.force_value)
        
#         self.buffer = [i for _, i in enumerate(self.hidden_buf)]
#         self.curr_avg = np.average(self.buffer)

#         if abs(self.curr_avg - self.prev_avg) > self.threshold:
#             self.weight_changed = True
#         self.ma_pub.publish(self.curr_avg)
#         self.wc_pub.publish(self.weight_changed)

#     def get_all_data(self):
#         # Returns all surface data
#         return [self.force_value, self.force_location]
        
            
#     def reset_weight_change_check(self):
#         # Resets the weight changed variable, and sets the 
#         # current average force value as the comparison value for the next
#         # weight change check.
#         if self.weight_changed:
#             self.weight_changed = False
#         self.prev_avg = self.curr_avg
            

#     def recalibrate(self): 
#         # Setup rospy service proxy for calling the recalibration service
#         recalib = rospy.ServiceProxy(self._ns + '/force_surface_sensor/recalibrate', Empty)
#         time.sleep(1)
#         try:
#             # Checks if service is available with a 3s timeout
#             rospy.wait_for_service(self._ns + '/force_surface_sensor/recalibrate', timeout=3)
#             # Calls service
#             response = recalib()
#         except rospy.ServiceException as exc:
#             # Catch the 'failed' service call exception raised due to the rosserial v0.8 bug
#             # print str(exc)
#             pass

#         # If service is not available
#         except rospy.ROSException:
#             rospy.loginfo('Service unavailable. \
#                           Double check that the cell number is valid.')

#         rospy.loginfo('Recalibration service called')

# Chan Wai Lee, [28/10/21 6:07 PM]
# if name == "main":
#     rospy.init_node("fss_test")
    
#     try:
#         if str(sys.argv[1]):
#             ns = str(sys.argv[1])
#         else:
#             rospy.loginfo(("No namespace input. Using /cell2 as default"))
#             ns = "/cell2"  
#     except IndexError:
#         print("Please specify the namespace to connect to, e.g. /cell4")
#         sys.exit()  
        
#     fss = ForceSurface(ns=ns)
#     rospy.sleep(1)
#     quit = False
    
#     if fss.is_ready():
#         print('FSS Connected at {}'.format("/cell2"))
#         rospy.loginfo("This script tests the Force Sensing Surface functionality.")
#         rospy.loginfo("Recalibrating...")
#         fss.recalibrate()
#         rospy.loginfo("Weight changed reset will be called every 5 seconds.")
#         rospy.sleep(3)
#         while not quit and not rospy.is_shutdown():            
#             try:
                
#                 print("Current Force Value: {}".format(fss.force_value))
#                 print("Current Moving Average Force Value: {}".format(fss.curr_avg))
#                 print("Current Force Location: {}".format(fss.force_location))
#                 print(fss.get_all_data())
#                 print("")
#                 print("Weight changed: {}".format(fss.weight_changed))
#                 rospy.sleep(0.05)
#                 if fss.weight_changed:
#                     if rospy.get_time()%5.0 < 0.2:
#                         fss.reset_weight_change_check()
                
                
#             except KeyboardInterrupt:
#                 quit = True