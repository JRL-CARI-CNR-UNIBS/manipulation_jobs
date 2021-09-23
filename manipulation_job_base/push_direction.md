# Push in specific direction

## Parameters


- `[PROPERTY_ID_NAME]`:
  - `return_flag`: if true return in initial position
  - `goal_twist`: [linear_velocity, angular velocity] in `goal_twist_frame` to reach contact.

  - `target_wrench`: [force,torque] in `target_wrench_frame`
  - `target_wrench_frame`: XXX
  - `goal_twist_frame`: XXX

  - `wrench_toll`: XXX, if not specified use `default/wrench_toll`
  - `wrench_deadband`: XXX, if not specified use `default/wrench_deadband`

- `default`:
  - `wrench_toll`: XXX

  - `wrench_deadband`: XXX

## State descriptions
- `Init`: sleep
- `Prepared`: open/close gripper
- `Switch control`: change configuration to push
- `Simple Touch`: touch the surface using parameters read from ROS Param `[PROPERTY_ID_NAME]`
- `Return`: return in the initial position, if return_flag==True

## Transition descriptions

From | To | Condition |
:------------ | :------------- | :------------ |
Init | Prepared | pre_execution server callback |
Prepared | Switch control | execution server callback |
Switch control | Simple Touch | --- |
Simple Touch | Return | if return_flag==True, wait for simple touch result |
Return | Init |  wait for movement result |
Simple Touch | Init | if return_flag==False |
