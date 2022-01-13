#!/usr/bin/env python3
from sismic.io import import_from_yaml
from sismic.interpreter import Interpreter


import rospy
import actionlib
import tf
import manipulation_msgs.srv
import configuration_msgs.srv
import simple_touch_controller_msgs.msg
import relative_cartesian_controller_msgs.msg
import math
import time


def main():
    rospy.init_node('sismic_job_execution')

    # Load statechart from yaml file
    statechart_text=rospy.get_param('/test')


    elevator = import_from_yaml(filepath=statechart_text)

    # Create an interpreter for this statechart
    interpreter = Interpreter(elevator)
    interpreter.clock.start()

    print('Before:', interpreter.configuration)

    step = interpreter.execute_once()

    print('After:', interpreter.configuration)

    while True:
        step = interpreter.execute_once()
        if (len(interpreter.configuration)==0):
            break
        time.sleep(0.5)
    print(interpreter.context['x'])



if __name__ == '__main__':
    main()
