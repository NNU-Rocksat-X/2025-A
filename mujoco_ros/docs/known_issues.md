## Instability Error
Occasionally, you may recieve the following error or a similar error:

`WARNING: Nan, Inf or huge value in QACC at DOF 6. The simulation is unstable. Time = 0.0878.`

The solution is to edit the robot's xml file and add the `armature` attribute to the joint which is creating the error.
Ex:

```<joint name="joint_3" pos="0 0 0" axis="-0.991445 0 0.130526" range="-4.10152 0.698132" armature="1"/>```

Adjust the armature value as necessary. It may be difficult to determine which joint is creating the error, the simplest way is to use RVIZ to manually move each joint individually to it's limits and the error should occur.
Whichever joint was being moved when the error occured is the joint that should have the armature value added.

## Joint Ranges Error
Occasionally, the arm will attempt to move beyond the range of a joint and it will essentially stop moving at a wierd spot. Still looking for a fix.
