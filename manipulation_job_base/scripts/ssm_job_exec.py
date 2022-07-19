#!/usr/bin/env python3

import rospy
import actionlib
import tf
import manipulation_msgs.srv
import configuration_msgs.srv
import simple_touch_controller_msgs.msg
import relative_cartesian_controller_msgs.msg
import math
import object_loader_msgs.srv

import ur_dashboard_msgs.srv
from ur_msgs.msg

class JobExecution:

    def __init__(self):

        self.config_client = rospy.ServiceProxy('/configuration_manager/start_configuration', configuration_msgs.srv.StartConfiguration)
        rospy.logdebug("[%s] waiting for service", rospy.get_name())
        self.config_client.wait_for_service()

        self.gripper_client = rospy.ServiceProxy('/robotiq_gripper', manipulation_msgs.srv.JobExecution)
        self.gripper_client.wait_for_service()

        self.script_server = rospy.ServiceProxy('/linear_guide/go', ur_dashboard_msgs.srv.Load)
        self.script_server.wait_for_service()

        self.set_io_server = rospy.ServiceProxy('/ut10e_hw/set_io'', ur_msgs.srv.SetIO)
        self.set_io_server.wait_for_service()

        self.attach_client = rospy.ServiceProxy('/attach_object_to_link', object_loader_msgs.srv.AttachObject)
        self.attach_client.wait_for_service()

        self.detach_client = rospy.ServiceProxy('/detach_object_to_link', object_loader_msgs.srv.DetachObject)
        self.detach_client.wait_for_service()

        self.remote_obj_client = rospy.ServiceProxy('/remove_object_from_scene', object_loader_msgs.srv.RemoveObjects)
        self.remote_obj_client.wait_for_service()
        rospy.logdebug("[%s] connected to services", rospy.get_name())

        self.job_ex_srv = rospy.Service(rospy.get_name()+'/pre_exec', manipulation_msgs.srv.JobExecution, self.preExecutionCb)
        self.pre_ex_srv = rospy.Service(rospy.get_name()+'/exec', manipulation_msgs.srv.JobExecution, self.executionCb)
        self.post_ex_srv = rospy.Service(rospy.get_name()+'/post_exec', manipulation_msgs.srv.JobExecution, self.postExecutionCb)

        rospy.loginfo("[%s] job execution is ready", rospy.get_name())
    def preExecutionCb(self,req):

        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/pre_execution_subjobs_list'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res

        subjobs=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/pre_execution_subjobs_list')

        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/pre_execution_initial_state'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res
        current_state_name=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/pre_execution_initial_state')
        return self.stateMachine(req,subjobs,current_state_name)

    def executionCb(self,req):
        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/subjobs_list'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res
        subjobs=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/subjobs_list')

        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/initial_state'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res
        current_state_name=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/initial_state')
        return self.stateMachine(req,subjobs,current_state_name)

    def postExecutionCb(self,req):
        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/post_execution_subjobs_list'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res
        subjobs=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/post_execution_subjobs_list')

        if not rospy.has_param(rospy.get_name()+"/"+req.property_id+'/post_execution_initial_state'):
            res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.PropertyIdNotFound)
            return res
        current_state_name=rospy.get_param(rospy.get_name()+"/"+req.property_id+'/post_execution_initial_state')
        return self.stateMachine(req,subjobs,current_state_name)

    def stateMachine(self,job_req,subjobs,current_state_name):
        print(job_req)

        self.touch_client = actionlib.SimpleActionClient('/simple_touch', simple_touch_controller_msgs.msg.SimpleTouchAction)
        self.move_client = actionlib.SimpleActionClient('/relative_move', relative_cartesian_controller_msgs.msg.RelativeMoveAction)

        while (not rospy.is_shutdown()):
            rospy.loginfo("Current state = "+current_state_name)
            if (current_state_name=="FAIL"):
                res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.HwError)
                return res
            if (current_state_name=="SUCCESS"):
                res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.Success)
                return res
            try:
                current_state=subjobs[current_state_name]
            except:
                rospy.logerror('[%s] state %s is undefined',rospy.get_name(),current_state_name)
                res=manipulation_msgs.srv.JobExecutionResponse(manipulation_msgs.srv.JobExecutionResponse.HwError)
                return res

            if (current_state["type"]=='Script'):
                rospy.loginfo(current_state_name+ " SCRIPT")
                req=configuration_msgs.srv.StartConfigurationRequest()
                req.strictness=req.BEST_EFFORT
                req.start_configuration="watch"
                try:
                    res=self.config_client(req)
                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

                self.script_server.wait_for_service()

                script_req=ur_dashboard_msgs.srv.Load()
                script_req.filename=current_state["name"]
                rospy.loginfo(current_state_name+ " Script SEND GOAL: "+script_req.filename)
                script_res=self.script_server(script_req.filename)

                print(script_res)
                if (not script_res.success):
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = current_state["next_state_if_fail"]
                else:
                    current_state_name=current_state["next_state_if_success"]

                req.start_configuration="trajectory_tracking"
                try:
                    res=self.config_client(req)
                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

            if (current_state["type"]=='SetIO'):
                print("DOSOMETHING")
                self.set_io_server.wait_for_service()
                set_io_req=ur_msgs.srv.SetIO()
                if (current_state["output"]=="Digital"):
                    set_io_req.fun=1
                else if (current_state["output"]=="Flag"):
                    set_io_req.fun=2
                else if (current_state["output"]=="Analog"):
                    set_io_req.fun=3
                else if (current_state["output"]=="ToolVoltage"):
                    set_io_req.fun=4
                else:
                    rospy.logerror('[%s] undefined io type. Valid options are Digital, Flag, Analaog, ToolVoltage',rospy.get_name())

                set_io_req.pin=current_state["pin"]
                set_io_req.state=current_state["state"]

                rospy.loginfo(current_state_name+ " Set IO: fun: "+set_io_req.fun+", pin: "+set_io_req.pin+", state: "+set_io_req.state)
                set_io_res=self.set_io_server(set_io_req)

                print(set_io_res)
                if (not set_io_res.success):
                    rospy.logerror('[%s] set io UR failed',rospy.get_name())
                    current_state_name = current_state["next_state_if_fail"]
                else:
                    current_state_name=current_state["next_state_if_success"]

            if (current_state["type"]=='Touch'):

                rospy.loginfo(current_state_name+ " TOUCH")
                req=configuration_msgs.srv.StartConfigurationRequest()
                req.strictness=req.BEST_EFFORT
                req.start_configuration="simple_touch"
                try:
                    res=self.config_client(req)
                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

                self.touch_client.wait_for_server()

                goal=simple_touch_controller_msgs.msg.SimpleTouchGoal()
                print("goal ",goal)
                print("current_state",current_state)
                goal.goal_twist=current_state["goal_twist"]
                goal.goal_twist_frame=current_state["goal_twist_frame"]
                goal.target_force=current_state["target_force"]
                goal.relative_target=current_state["relative_target"]
                if current_state["release_condition"]=="Force":
                    goal.release=current_state["release_force"]
                    goal.release_condition=goal.FORCE
                if current_state["release_condition"]=="None":
                    goal.release_condition=goal.NONE
                    goal.release_condition=0.0
                if current_state["release_condition"]=="Position":
                    goal.release=current_state["release_position"]
                    goal.release_condition=goal.POSITION

                rospy.loginfo(current_state_name+ " TOUCH SEND GOAL")
                self.touch_client.send_goal(goal)
                self.touch_client.wait_for_result()
                touch_result=self.touch_client.get_result()
                if (touch_result.error_code>=0):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    current_state_name=current_state["next_state_if_fail"]

                req.start_configuration="trajectory_tracking"
                try:
                    res=self.config_client(req)
                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

            elif (current_state["type"]=='Gripper'):

                rospy.loginfo(current_state_name+ " Gripper")
                req=manipulation_msgs.srv.JobExecutionRequest()
                self.gripper_client.wait_for_service()
                property_id="pos_"+str(current_state["position"])+"_force_"+str(current_state["force"])+"_vel_"+str(current_state["velocity"])
                print(property_id)

                req.property_id=property_id
                rospy.loginfo(current_state_name+ " Gripper SEND GOAL")
                res=self.gripper_client(req)
                if (res.results>=0):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    rospy.logwarn('[%s] unable to grasp',rospy.get_name())
                    current_state_name=current_state["next_state_if_fail"]

            elif (current_state["type"]=='RelativeMove'):

                rospy.loginfo(current_state_name+ " MOVE")
                req=configuration_msgs.srv.StartConfigurationRequest()
                req.strictness=req.BEST_EFFORT
                req.start_configuration="cartesian_position"
                try:
                    res=self.config_client(req)

                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

                self.move_client.wait_for_server()

                goal=relative_cartesian_controller_msgs.msg.RelativeMoveGoal()
                goal.relative_pose.header.frame_id=current_state["frame"]
                if "rotZdeg" in current_state:
                    angle=current_state["rotZdeg"]*math.pi/180.0
                    orientation = tf.transformations.quaternion_from_euler(0, 0, angle)
                    position=[0.0,0.0,0.0]
                elif "rotYdeg" in current_state:
                    angle=current_state["rotYdeg"]*math.pi/180.0
                    orientation = tf.transformations.quaternion_from_euler(0, angle,0)
                    position=[0.0,0.0,0.0]
                elif "rotXdeg" in current_state:
                    angle=current_state["rotXdeg"]*math.pi/180.0
                    orientation = tf.transformations.quaternion_from_euler(angle, 0, 0)
                    position=[0.0,0.0,0.0]
                elif "traXmm" in current_state:
                    tra=current_state["traXmm"]/1000.0
                    orientation = [0,0,0,1]
                    position=[tra,0.0,0.0]
                elif "traYmm" in current_state:
                    tra=current_state["traYmm"]/1000.0
                    orientation = [0,0,0,1]
                    position=[0.0,tra,0.0]
                elif "traZmm" in current_state:
                    tra=current_state["traZmm"]/1000.0
                    orientation = [0,0,0,1]
                    position=[0.0,0.0,tra]
                else:
                    position=current_state["position"]
                    orientation=current_state["orientation"]

                goal.relative_pose.pose.position.x=position[0]
                goal.relative_pose.pose.position.y=position[1]
                goal.relative_pose.pose.position.z=position[2]
                goal.relative_pose.pose.orientation.x=orientation[0]
                goal.relative_pose.pose.orientation.y=orientation[1]
                goal.relative_pose.pose.orientation.z=orientation[2]
                goal.relative_pose.pose.orientation.w=orientation[3]

                if "linear_velocity_m_s" in current_state:
                    goal.target_linear_velocity=current_state["linear_velocity_m_s"]
                elif "linear_velocity_mm_s" in current_state:
                    goal.target_linear_velocity=current_state["linear_velocity_mm_s"]/1000.0
                else:
                    goal.target_linear_velocity=0.250

                if "angular_velocity_deg_s" in current_state:
                    goal.target_angular_velocity=current_state["angular_velocity_deg_s"]*math.pi/180.0
                elif "angular_velocity_rad_s" in current_state:
                    goal.target_angular_velocity=current_state["angular_velocity_rad_s"]
                else:
                    goal.target_angular_velocity=30.0*math.pi/180.0


                rospy.loginfo(current_state_name+ " MOVE SEND GOAL")
                self.move_client.send_goal(goal)
                self.move_client.wait_for_result()
                move_result=self.move_client.get_result()
                if (move_result.error_code>=0):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    if "next_state_if_fail" in current_state:
                        current_state_name=current_state["next_state_if_fail"]
                    else:
                        current_state_name="FAIL"
                rospy.loginfo('[%s] trajectory_tracking',rospy.get_name())
                req.start_configuration="trajectory_tracking"
                try:
                    res=self.config_client(req)
                except:
                    rospy.logerror('[%s] no configuration_manager server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

            elif (current_state["type"]=='Attach'):

                rospy.loginfo(current_state_name+ " Attach object %s to %s",job_req.object_id,job_req.tool_id)

                req=object_loader_msgs.srv.AttachObjectRequest()
                req.obj_id=job_req.object_id
                req.link_name=job_req.tool_id
                try:
                    res=self.attach_client(req)
                except:
                    rospy.logerror('[%s] no attach_object_to_link server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

                if (res.success):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    if "next_state_if_fail" in current_state:
                        current_state_name=current_state["next_state_if_fail"]
                    else:
                        current_state_name="FAIL"

            elif (current_state["type"]=='Detach'):

                rospy.loginfo(current_state_name+ " Detach object %s",job_req.object_id)

                req=object_loader_msgs.srv.DetachObjectRequest()
                req.obj_id=job_req.object_id
                try:
                    res=self.detach_client(req)
                except:
                    rospy.logerror('[%s] no detach_object_to_link server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue

                if (res.success):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    if "next_state_if_fail" in current_state:
                        current_state_name=current_state["next_state_if_fail"]
                    else:
                        current_state_name="FAIL"

            elif (current_state["type"]=='RemoveObject'):

                rospy.loginfo(current_state_name+ " Remove object %s",job_req.object_id)

                req=object_loader_msgs.srv.RemoveObjectsRequest()
                req.obj_ids.append(job_req.object_id)
                try:
                    res=self.remote_obj_client(req)
                except:
                    rospy.logerror('[%s] no /remove_object_from_scene server',rospy.get_name())
                    current_state_name = "FAIL"
                    continue
                print("sucess")
                if (res.success):
                    current_state_name=current_state["next_state_if_success"]
                else:
                    if "next_state_if_fail" in current_state:
                        current_state_name=current_state["next_state_if_fail"]
                    else:
                        current_state_name="FAIL"

        print(subjobs.keys())

def main():
    rospy.init_node('ssm_job_execution')
    server =JobExecution()

    rospy.spin()




if __name__ == '__main__':
    main()
