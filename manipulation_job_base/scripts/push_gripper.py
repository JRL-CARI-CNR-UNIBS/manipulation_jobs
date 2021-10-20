#!/usr/bin/env python

import rospy
import smach
import smach_ros
import manipulation_msgs.srv
import configuration_msgs.srv
import simple_touch_controller_msgs.msg
import actionlib

def main():
    rospy.init_node('test_push')


    
    config_client = rospy.ServiceProxy('/configuration_manager/start_configuration', configuration_msgs.srv.StartConfiguration)
    config_client.wait_for_service()
    touch_client = actionlib.SimpleActionClient('/simple_touch', simple_touch_controller_msgs.msg.simpleTouchAction)

    subjobs=rospy.get_param('subjobs_list')
    current_state_name=rospy.get_param('initial_state')
    
    while (not rospy.is_shutdown()):
        rospy.loginfo("Current state = "+current_state_name)
        if (current_state_name=="FAIL"):
            return 0
        if (current_state_name=="SUCCESS"):
            return 0
        
        current_state=subjobs[current_state_name]
        
        if (current_state["type"]=='Touch'):
            
            rospy.loginfo(current_state_name+ " TOUCH")
            req=configuration_msgs.srv.StartConfigurationRequest()
            req.strictness=req.BEST_EFFORT
            req.start_configuration="simple_touch"
            try:
                res=config_client(req)
                
            except:
                rospy.loginfo('no server')
                current_state_name = "FAIL"
                continue

            touch_client.wait_for_server()

            goal=simple_touch_controller_msgs.msg.simpleTouchGoal
            goal.goal_twist=current_state["goal_twist"]
            goal.goal_twist_frame=current_state["goal_twist_frame"]
            goal.target_force=current_state["target_force"]
            goal.relative_target=current_state["relative_target"]
            if current_state["release_condition"]=="Force":
                goal.release=current_state["release_force"]
                goal.release_condition=simple_touch_controller_msgs.msg.simpleTouchGoal.FORCE
            rospy.loginfo(current_state_name+ " TOUCH SEND GOAL")
            touch_client.send_goal(goal)
            touch_client.wait_for_result()
            touch_result=touch_client.get_result()
            if (touch_result.error_code>=0):
                current_state_name=current_state["next_state_if_success"]
            else:
                current_state_name=current_state["next_state_if_fail"]

    print(subjobs.keys())


if __name__ == '__main__':
    main()