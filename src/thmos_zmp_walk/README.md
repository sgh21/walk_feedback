> There is no simulation file in this branch.

### description

* controller : zmp mpc preview (citbrains) 

### environment

```shell
pip install numpy
pip install control
```

* bulid `libthmos_leg_ik.so`

```shell
cd scripts/THMOS_IK_MAKER/build
cmake ..
make
cp libthmos_leg_ik.so ../../../lib/
```

* build ros packet `thmos_zmp_walk`

```shell
catkin build thmos_zmp_walk
```

### example
* ros (use thmos_code)

```shell
rosrun thmos_zmp_walk mos_zmp_run.launch
```

### else

* kinematics -> _cpp is better!_
* planner -> _threading is better!_
