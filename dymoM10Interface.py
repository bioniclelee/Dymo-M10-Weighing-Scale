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

class WeighingScaleInterface:

    def __init__(self, ns, staticAverageMassArrSize = 10, movingAverageMassArrSize = 10):
        self.ns = ns
        self.isReady = False
        
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

        self.movingAverageMassPubber = None
        self.staticAverageMasspubber = None
        self.weightChangePubber = None

        self.initMovingAverageMassSubber()
        self.initNamespaceSubber()
        self.initPubbers()

    def initPubbers(self):
        topic = self.ns + "/movingAverageMass"
        self.movingAverageMassPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.ns + "/staticAverageMass"
        self.staticAverageMassPubber = rospy.Publisher(topic, Float64, queue_size=10)
        topic = self.ns + "/weight_changed"
        self.weightChangePubber = rospy.Publisher(topic, Bool, queue_size=1)

    def initNamespaceSubber(self, topic = "namespace"):
        self.namespaceSubber = rospy.Subscriber(topic, String,
                                                self.updateNameSpace,
                                                callback_args = self)

    def updateNamespace(msg, self):
        self.ns = msg.data
        rospy.loginfo("Namespace updated as {}".format(self.ns))

    def initMovingAverageMassSubber(self, topic = "/movingAverageMass"):
        topic += self.ns
        self.movingAverageMassSubber = rospy.Subscriber(topic, Float64,
                                                        self.updateMovingAverageMassArr,
                                                        callback_args = self)

    def updateMovingAverageMassArr(msg, self):
        if self.movingAverageMassArrSize == (len(self.movingAverageMassArr)):
            self.movingAverageMassArr = self.pushPop(self.movingAverageMassArr, msg)
            # self.prevMovingAverage = self.currMovingAverage
            # self.currMovingAverage = sum(self.movingAverageMassArray)/self.movingAverageMassArrSize
        self.staticMassArray.append(msg)

    def pushPop(list, newElem):
        newList = []
        for i in range(1, len(list)-1):
            newList[i-1] = list[i]
        newList.append(newElem)
        return newList         

    def initStaticAverageMassSubber(self, topic = "/staticAverageMass"):
        topic += self.ns
        self.staticAverageMassSubber = rospy.Subscriber(topic, Float64,
                                                self.updateStaticAverageMassArr,
                                                callback_args = self)

    def updateStaticAverageMassArr(msg, self):
        if self.staticAverageMassArrSize != (len(self.staticAverageMassArray)):
            # self.prevStaticAverage = self.currStaticAverage
            # self.currStaticAverage = sum(self.staticAverageMassArray)/self.massArrSize
            self.staticMassArray.append(msg)

    def update(self, mode):
        self.prevAverage = self.currAverage
        if mode == "moving":
            self.currAverage = sum(self.movingAverageMassArray)/self.movingAverageMassArrSize
            self.movingAverageMassPubber(self.currAverage)
        else:
            self.currAverage = sum(self.staticAverageMassArray)/self.massArrSize
            self.staticAverageMasspubber(self.currAverage)

    def resetWeightChangeCheck(self):
        # Resets the weight changed variable, and sets the 
        # current average force value as the comparison value for the next
        # weight change check.
        if self.weightChanged:
            self.weightChanged = False
        self.prevAverage = self.currAverage

    def recalibrate(self): 
        # Setup rospy service proxy for calling the recalibration service
        recalib = rospy.ServiceProxy(self._ns + '/force_surface_sensor/recalibrate', Empty)
        time.sleep(1)
        try:
            # Checks if service is available with a 3s timeout
            rospy.wait_for_service(self._ns + '/force_surface_sensor/recalibrate', timeout=3)
            # Calls service
            response = recalib()
        except rospy.ServiceException as exc:
            # Catch the 'failed' service call exception raised due to the rosserial v0.8 bug
            # print str(exc)
            pass

        # If service is not available
        except rospy.ROSException:
            rospy.loginfo('Service unavailable. \
                          Double check that the cell number is valid.')

if name == "main":
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
    rospy.sleep(1)
    quit = False
    
    if wsi.is_ready():
        print('FSS Connected at {}'.format(self.ns))
        rospy.loginfo("This script tests the Force Sensing Surface functionality.")
        rospy.loginfo("Recalibrating...")
        fss.recalibrate()
        rospy.loginfo("Weight changed reset will be called every 5 seconds.")
        rospy.sleep(3)
        while not quit and not rospy.is_shutdown():            
            try:
                
                print("Current Force Value: {}".format(fss.force_value))
                print("Current Moving Average Force Value: {}".format(fss.curr_avg))
                print("Current Force Location: {}".format(fss.force_location))
                print(fss.get_all_data())
                print("")
                print("Weight changed: {}".format(fss.weight_changed))
                rospy.sleep(0.05)
                if fss.weight_changed:
                    if rospy.get_time()%5.0 < 0.2:
                        fss.reset_weight_change_check()
                
                
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