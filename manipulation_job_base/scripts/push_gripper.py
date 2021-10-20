#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import manipulation_msgs.srv
import configuration_msgs.srv
import simple_touch_controller_msgs.msg
import cnr_cartesian_position_controller.msg
import actionlib

def main():
    rospy.init_node('test_push')



    config_client = rospy.ServiceProxy('/configuration_manager/start_configuration', configuration_msgs.srv.StartConfiguration)
    config_client.wait_for_service()
    touch_client = actionlib.SimpleActionClient('/simple_touch', simple_touch_controller_msgs.msg.SimpleTouchAction)
    move_client = actionlib.SimpleActionClient('/relative_move', cnr_cartesian_position_controller.msg.RelativeMoveAction)
    gripper_client = rospy.ServiceProxy('/robotiq_gripper', manipulation_msgs.srv.JobExecution)

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

            goal=simple_touch_controller_msgs.msg.SimpleTouchGoal
            goal.goal_twist=current_state["goal_twist"]
            goal.goal_twist_frame=current_state["goal_twist_frame"]
            goal.target_force=current_state["target_force"]
            goal.relative_target=current_state["relative_target"]
            if current_state["release_condition"]=="Force":
                goal.release=current_state["release_force"]
                goal.release_condition=simple_touch_controller_msgs.msg.simpleTouchGoal.FORCE
            if current_state["release_condition"]=="None":
                goal.release_condition=simple_touch_controller_msgs.msg.simpleTouchGoal.NONE
            if current_state["release_condition"]=="Position":
                goal.release=current_state["release_position"]
                goal.release_condition=simple_touch_controller_msgs.msg.simpleTouchGoal.POSITION

            rospy.loginfo(current_state_name+ " TOUCH SEND GOAL")
            touch_client.send_goal(goal)
            touch_client.wait_for_result()
            touch_result=touch_client.get_result()
            if (touch_result.error_code>=0):
                current_state_name=current_state["next_state_if_success"]
            else:
                current_state_name=current_state["next_state_if_fail"]


        if (current_state["type"]=='Gripper'):

            rospy.loginfo(current_state_name+ " Gripper")
            req=manipulation_msgs.srv.JobExecutionRequest()
            gripper_client.wait_for_service()
            property_id="pos_"+str(current_state["position"])+"_force_"+str(current_state["force"])+"_vel_"+str(current_state["velocity"])
            print(property_id)

            req.property_id=property_id
            rospy.loginfo(current_state_name+ " Gripper SEND GOAL")
            res=gripper_client(req)
            if (res.results>=0):
                current_state_name=current_state["next_state_if_success"]
            else:
                current_state_name=current_state["next_state_if_fail"]

        if (current_state["type"]=='RelativeMove'):

            rospy.loginfo(current_state_name+ " MOVE")
            req=configuration_msgs.srv.StartConfigurationRequest()
            req.strictness=req.BEST_EFFORT
            req.start_configuration="cartesian_position"
            try:
                res=config_client(req)

            except:
                rospy.loginfo('no server')
                current_state_name = "FAIL"
                continue

            move_client.wait_for_server()

            goal=cnr_cartesian_position_controller.msg.RelativeMoveGoal()
            goal.relative_pose.header.frame_id=current_state["frame"]
            goal.relative_pose.pose.position.x=current_state["position"][0]
            goal.relative_pose.pose.position.y=current_state["position"][1]
            goal.relative_pose.pose.position.z=current_state["position"][2]
            goal.relative_pose.pose.orientation.x=current_state["orientation"][0]
            goal.relative_pose.pose.orientation.y=current_state["orientation"][1]
            goal.relative_pose.pose.orientation.z=current_state["orientation"][2]
            goal.relative_pose.pose.orientation.w=current_state["orientation"][3]
            goal.target_velocity=current_state["velocity"]


            rospy.loginfo(current_state_name+ " MOVE SEND GOAL")
            move_client.send_goal(goal)
            move_client.wait_for_result()
            move_result=move_client.get_result()
            if (move_result.error_code>=0):
                current_state_name=current_state["next_state_if_success"]
            else:
                current_state_name=current_state["next_state_if_fail"]

    print(subjobs.keys())


if __name__ == '__main__':
    main()
