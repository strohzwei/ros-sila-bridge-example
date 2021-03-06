#! /usr/bin/env python
import rospy
import actionlib
from multiprocessing import Lock
from ros_battery_simulator.msg import ChargeAction, ChargeFeedback, ChargeResult, Status

rospy.init_node('battery_simulator')

# params
PARAM_CAPACITY_A_H = float(rospy.get_param("~capacity_a_h",8))
PARAM_MAX_CHARGE_RATE_A_H = float(rospy.get_param("~param_max_charge_rate_a_h",20))
PARAM_CONTINUOUS_DISCHARGE_RATE_A_H = float(rospy.get_param("~continuous_discharge_rate_a_h",8))
    
class ChargeActionServer(object):
    
    def __init__(self, name, battery):
        self._feedback = ChargeFeedback()
        self._result = ChargeResult()
        self._action_name = name
        self.battery = battery
        self._as = actionlib.SimpleActionServer(self._action_name, ChargeAction, execute_cb=self.execute_callback, auto_start = False)
        self._as.start()
        
    def execute_callback(self, goal):
        self._result.success = False

        if goal.charge_rate_a_h > PARAM_MAX_CHARGE_RATE_A_H:
            rospy.logerr("Maximum discharge/charge rate exceeded (%s > %s)." % (goal.charge_rate_a_h, PARAM_MAX_CHARGE_RATE_A_H))
            return self._as.set_aborted(self._result)
            
        try:
            self.battery.set_charging(goal.charge_rate_a_h)
            _r = rospy.Rate(1) # hz
            percentage = 0.
            self._feedback.percentage = 0.
            while percentage < 100.:
                _r.sleep()
                percentage, _, _ = self.battery.status()
                if self._feedback.percentage > percentage:
                    raise Exception("Still discharging abort.")
                    
                self._feedback.percentage = percentage
                self._as.publish_feedback(self._feedback)
                if rospy.is_shutdown():
                    raise Exception("Ros shutdown.")
                    
            self._feedback.percentage = 100.
            self._as.publish_feedback(self._feedback)   
             
            self.battery.set_charging(0.)
            self._result.success = True
            
            return self._as.set_succeeded(self._result)
            
        except rospy.ServiceException, e:
            rospy.logerr("failed: %s" %  e)
            return self._as.set_aborted(self._result)
       
        

def synchronized(fn):
    def mutex_fn(self, *arg, **kws):
        with self.mutex:
            return fn(self, *arg, **kws)
    return mutex_fn
        
class Battery:        
    def __init__(self):
        self.mutex = Lock()
        self.charging = 0.
        self.discharging = abs(PARAM_CONTINUOUS_DISCHARGE_RATE_A_H) * -1.
        self.capacity = PARAM_CAPACITY_A_H
        self.percentage = 100.
        self.time_stamp_s = rospy.get_time()

    @synchronized
    def update(self):
        delta_t_s = rospy.get_time() - self.time_stamp_s
        self.time_stamp_s = rospy.get_time()

        charge = self.discharging + abs(self.charging)
        
        p = 100./self.capacity
        p_change_per_s = p * charge / 60.**2.
        p_change = p_change_per_s * delta_t_s
        self.percentage += p_change
        
        
        if self.percentage < 0.:
            self.percentage = 0.
        if self.percentage > 100.:
            self.percentage = 100.

    @synchronized
    def status(self):  
        return self.percentage, self.charging, self.discharging

    @synchronized
    def set_charging(self, rate_a_h):
        self.charging = rate_a_h

bat = Battery()
server = ChargeActionServer(rospy.get_name(), bat)

bat_status_pub = rospy.Publisher("status", Status, queue_size=1)

r = rospy.Rate(10) # hz
while not rospy.is_shutdown():
    r.sleep()
    bat.update()

    percentage, charging, discharging = bat.status()

    status_msg = Status()
    status_msg.percentage = percentage
    status_msg.charging = charging
    status_msg.discharging = discharging
    bat_status_pub.publish(status_msg)
