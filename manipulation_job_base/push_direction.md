# Push in specific direction

## Parameters


- `[PROPERTY_ID_NAME]`:
  - `return_flag`: if true return in initial position
  - `goal_twist`: [linear_velocity, angular velocity] in `goal_twist_frame` to reach contact.

  - `target_wrench`: [force,torque] in `target_wrench_frame`

  - `wrench_toll`: ....
  - `wrench_deadband`: ....
  - `target_wrench_frame`:
  - `goal_twist_frame`:


## State descriptions
- `Init`: sleep
- `Prepared`: open/close gripper
- `Switch control`: change configuration to push
- `Simple Touch`: touch the surface
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
