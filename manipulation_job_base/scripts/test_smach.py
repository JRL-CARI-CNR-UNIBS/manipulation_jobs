#!/usr/bin/env python

import rospy
import smach
import smach_ros
import manipulation_msgs.srv
import configuration_msgs.srv
import simple_touch_controller_msgs.msg

# define state Foo
class Init(smach.State):
    def __init__(self,job):
        smach.State.__init__(self, outcomes=['ready_to_start','error','exit'])
        self.job=job
    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            if (self.job.new_prepare_cb==True):
                self.job.new_prepare_cb=False
                return 'ready_to_start'
        return 'exit'
        

# define state Bar
class Prepared(smach.State):
    def __init__(self,job):
        smach.State.__init__(self, outcomes=['done','error'])
        self.job=job
    def execute(self, userdata):
        rospy.loginfo('Executing state Prepared')
        
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            if (self.job.new_exec_cb==True):
                self.job.new_exec_cb=False
                return 'done'
        if (self.job.property_id=="default"):
            return 'done'
        
        # do something
        return 'done'
        

# define state Bar
class SwitchControl(smach.State):
    def __init__(self,job):
        smach.State.__init__(self, outcomes=['done','error'])
        self.job=job
    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchControl')
        
        req=configuration_msgs.srv.StartConfigurationRequest()
        req.strictness=req.BEST_EFFORT
        req.start_configuration="simple_touch"
        try:
            res=self.job.config_client(req)
            if (res.ok):
                return 'done'
        except:
            rospy.loginfo('no server')
        
        # qui andrebbe l'errore
        return 'done'
        

# define state Bar
class Error(smach.State):
    def __init__(self,job):
        smach.State.__init__(self, outcomes=['done'])
        self.job=job
    def execute(self, userdata):
        rospy.sleep(1.0)
        rospy.loginfo('Executing state Error')
        return 'done'


class JobExecution():
    def __init__(self):
        self.property_id=''
        rospy.loginfo('Init server')
        self.job_server = rospy.Service('push', manipulation_msgs.srv.JobExecution, self.serverCb)
        self.config_client = rospy.ServiceProxy('configuration_manager/start_configuration', configuration_msgs.srv.StartConfiguration)
        
        rospy.loginfo('Init server DONE')
        self.new_prepare_cb=False
        self.new_exec_cb=False
    def serverCb(self,req):
        
        res=manipulation_msgs.srv.JobExecutionResponse()
        if len(req.property_id)==0:
            res.results=res.PropertyIdNotFound
            return res
        res.results=res.Success
        self.property_id=req.property_id
        
        if (self.property_id == 'default'): # qui va il check se è una property_id di preparazione
            self.new_prepare_cb=True
        else:
            self.new_exec_cb=True
        return res

def main():
    rospy.init_node('smach_example_state_machine')

    job=JobExecution()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminated'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/manipulation_job')
    sis.start()
    # Open the container
    
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(job), 
                               transitions={'ready_to_start':'Prepared', 'error': 'Error', 'exit': 'terminated'})
        smach.StateMachine.add('Prepared', Prepared(job), 
                               transitions={'done':'SwitchControl', 'error': 'Error'})
        smach.StateMachine.add('SwitchControl', SwitchControl(job), 
                               transitions={'done':'Init', 'error': 'Error'})
        smach.StateMachine.add('Error', Error(job), 
                               transitions={'done':'Init'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()