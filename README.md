# tiago_ltl_flexbe

Integration of TIAGo simulation environment with LTL motion planner and FlexBE real-time execution framework

## Description

This package is based on the following three sources:

* TIAGo simulation repo: http://wiki.ros.org/Robots/TIAGo/Tutorials

* LTL motion planner: https://github.com/MengGuo/P_MAS_TG

* FlexBE framework: http://wiki.ros.org/flexbe

## Usage 

- The robot model as finite transition system (FTS) is specified in [\[robot_fts.py\]](https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/robot_fts.py):

```python
robot_model = [robot_motion, init_pose, robot_action]

```

   where the robot motion and action abstractions are given as [FTS models](https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/ltl_tools/ts.py). 


- While calling the [\[ltl_planner.py\]](https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/ltl_planner.py), a complex navigation task specified as LTL formulas can be fed directly:

```python
# python ltl_planner.py '<> (r2 && <>r3)'
# python ltl_planner.py '([]<> r2) && ([]<> r3) && ([]<> r1)'
python ltl_planner.py '<> r2 && ([]<> r3) && ([]<> r1)'
```

  <p align="center">  
  <img src="https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/figures/tiago_ltl_flexbe.png" width="800"/>
  </p>

- Simulation [video](https://vimeo.com/215800825) for different LTL tasks. 

