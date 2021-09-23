#!/usr/bin/env python

import rospy
import smach
import smach_ros
import manipulation_msgs.srv

# define state Foo
class Init(smach.State):
    def __init__(self,job):
        smach.State.__init__(self, outcomes=['ready_to_start','error'])
        self.job=job
    def execute(self, userdata):
        print(self.job.property_id)
        rospy.loginfo('Executing state Init')
        return 'ready_to_start'
        

# define state Bar
class Prepared(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','error'])

    def execute(self, userdata):
        rospy.sleep(1.0)
        rospy.loginfo('Executing state Prepared')
        return 'done'
        

# define state Bar
class SwitchControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','error'])

    def execute(self, userdata):
        rospy.sleep(1.0)
        rospy.loginfo('Executing state SwitchControl')
        return 'done'
        

# define state Bar
class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.sleep(1.0)
        rospy.loginfo('Executing state Error')
        return 'done'


class JobExecution():
    def __init__(self):
        self.property_id=''
        job_server = rospy.Service('push', manipulation_msgs.srv.JobExecution, self.serverCb)
        
    def serverCb(self,req):
        
        res=manipulation_msgs.srv.JobExecutionResponse()
        if len(req.property_id)==0:
            res.results=res.PropertyIdNotFound
            return res
        res.results=res.Success
        self.property_id=req.property_id
     
        return res

def main():
    rospy.init_node('smach_example_state_machine')

    job=JobExecution()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/manipulation_job')
    sis.start()
    # Open the container
    a="aaa"
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(job), 
                               transitions={'ready_to_start':'Prepared', 'error': 'Error'})
        smach.StateMachine.add('Prepared', Prepared(), 
                               transitions={'done':'SwitchControl', 'error': 'Error'})
        smach.StateMachine.add('SwitchControl', SwitchControl(), 
                               transitions={'done':'Init', 'error': 'Error'})
        smach.StateMachine.add('Error', Error(), 
                               transitions={'done':'Init'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()