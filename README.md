# Ewellix Lift Kit

ros2_control hardware drivers for the [Ewellix TLT Lift Kit](https://www.ewellix.com/en/products/lifting-columns/tlt) and [Elmo Gold motor controllers.](https://www.elmomc.com/product/gold-solo-twitter/) 


The drivers have been built and tested against ROS 2 Jazzy.

<div align="center">
  <figure>
    <img width="400" alt="Ewellix TLT Lift" src="./docs/ewellix_tlt_lift.png">
  </figure>
  <figure>
    <img width="450" alt="Elmo Gold Solo Motor Controller" src="https://github.com/user-attachments/assets/bb71d110-6ed5-438b-bcb6-df6a0746bcdd">
  </figure>
</div>

# Overview 
This project provides a complete ros2_control system for the Ewellix actuator with dual Elmo Gold brushless servo motor controllers. The Elmo Gold controllers have many advantages that make development of a robotic system a lot easier and more precise. This is meant to be an expandable framework that can be adapted to different robotic systems.

This implementation provides:
- **Elmo Gold C++ API** - Includes Elmo commands for actuator control
- **Dual-Motor Control** - For use with a two-motor actuator
- **Actuator Calibration** - Finds the endpoints of the actuator and encoder ranges
- **ros2_control Integration** - Full `ros2_control` framework support including RViz
- **Actuator and Controller Telemetry** - Monitoring of motor and controller status
- **Configurable Motion Profiles** - Tunable acceleration, deceleration, speed, and more
- **Serial Communication** - RS-232/USB via Elmo TLT protocol 
  
## Build and Configure

To compile, add this repo to a colcon workspace, then install relevant ROS dependencies with `rosdep`.

The drivers communicate using a serial (RS232) connection, the port is configurable though the [com_port]() parameter.
Be sure that the [serial](https://github.com/tylerjw/serial.git) project is available either on the machine or in the same workspace.

Once all dependencies are installed, the drivers can be compiled with `colcon build`.

## Run

The deploy packages include launch files for both hardware and a kinematic simulation.
To launch the drivers:

```bash
# Run the kinematic simulation
ros2 launch ewellix_liftkit_deploy liftkit.launch.py use_fake_hardware:=true

# Run the hardware drivers
ros2 launch ewellix_liftkit_deploy liftkit.launch.py
```

We also include a basic MoveIt configuration for testing planning and execution.

```bash
ros2 launch ewellix_liftkit_moveit_config liftkit_moveit.launch.py
```

## A Note on Control

The liftkit motors are controlled solely through velocity commands, but the hardware interface ingests position commands.

The crux of the issue is that there is a minimum speed that can be sent to the motors such that the lift can be moved.
When you command too low of a speed, the robot will stop and you have to call `stop()` followed by `moveDown()` or `moveUp()` to start movement again.
This can cause some issues with movement, particularly when executing trajectories with slower velocities.
In many cases, the commanded final position of a trajectory will not be reached because the commanded motor speed towards the end is insufficient for movement.
It can also cause "stuttering" in the movement of the lift.

To improve both the tracking and steady state accuracy of the drivers, there are a few control principles that have been added to compensensate for the problems above,

1) A feedforward term to the controller so that we do not follow too far behind the commanded velocity and stop every time we get close.
2) An integral component so that we pick up the slack if trajectory starts are delayed due to the ramp up velocity commands.
3) We have separated the actuation of motor 1 and motor 2, which effectively cuts down the minimum speed by half.

## Calibration Procedure

The liftkits are not all made exactly the same (apparently).
There are small discrepancies that can result in a couple of mm of error, which we would like to avoid.
This calibration procedure allows you to take a couple of observations, and then let the driver do all of the math for you.

First, run the calibration script to make the liftkit go all the way up (replace `com_port` with the com port for your liftkit)!

```sh
ros2 launch ewellix_liftkit_deploy liftkit_calibration.launch.py com_port:=/dev/ewellix_left calibration_direction:=up
```
This will print out something when it gets there that looks like this:
```sh
[ros2_control_node-2] [INFO] [1748892022.647655552] [LiftkitHardwareInterface]: Calibration complete! Direction: up
[ros2_control_node-2] [INFO] [1748892022.647735345] [LiftkitHardwareInterface]: mot1_min_ticks: 862
[ros2_control_node-2] [INFO] [1748892022.647748887] [LiftkitHardwareInterface]: mot2_min_ticks: 860
```

Note this, and measure the height of the liftkit. Note that this height should be w.r.t. the nominal height of the liftkit in the URDF when the motors are at their "0" position. For example, if at the 0 position, the urdf has the top of the second stack 50 mm away from the bottom, and you measure that position to be 582.3 mm at the top, you would call this number 0.5323 (meters).

Now do the same thing going down

```sh
ros2 launch ewellix_liftkit_deploy liftkit_calibration.launch.py com_port:=/dev/ewellix_left calibration_direction:=down
```

you will get something that looks like this
```sh
[ros2_control_node-2] [INFO] [1748892022.647655552] [LiftkitHardwareInterface]: Calibration complete! Direction: down
[ros2_control_node-2] [INFO] [1748892022.647735345] [LiftkitHardwareInterface]: mot1_min_ticks: 9
[ros2_control_node-2] [INFO] [1748892022.647748887] [LiftkitHardwareInterface]: mot2_min_ticks: 10
```
Take the same measurement as mentioned before. Lets say that number was measured to be 1.6mm off of nominal down position.

Now you can make your calibration file as shown below, and pass that to [your ros2 control tag](ewellix_liftkit_description/urdf/ewellix_lift.urdf.xacro)
```yaml
min_ticks_mot_1: 9
max_ticks_mot_1: 862
min_ticks_mot_2: 10
max_ticks_mot_2: 860
min_height_m: 0.0016
max_height_m: 0.5323
```
## Citation

This project falls under the purview of the iMETRO project.
If you use this in your own work, please cite the following paper:

```bibtex
@INPROCEEDINGS{imetro-facility-2025,
  author={Dunkelberger, Nathan and Sheetz, Emily and Rainen, Connor and Graf, Jodi and Hart, Nikki and Zemler, Emma and Azimi, Shaun},
  booktitle={2025 22nd International Conference on Ubiquitous Robots (UR)},
  title={Design of the iMETRO Facility: A Platform for Intravehicular Space Robotics Research},
  year={2025},
  volume={},
  number={},
  pages={390-397},
  keywords={NASA;Moon;Seals;Maintenance engineering;Maintenance;Robots;Standards;Open source software;Testing;Logistics},
  doi={10.1109/UR65550.2025.11077983}}
```
