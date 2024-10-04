# Laboratory 2
Antes de empezar **importante**:
```bash
source devel/setup.bash
```
Es buena práctica ejecutar al inicio:
```bash
roscore
```

## Inicio
Comprobar que todo funciona
```bash
catkin_make
roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
```
## Exercise 1

**Identify the mass of the quadrotor, where the sonar sensor is described and the topic where ros publishes the measurements of this sensor. Indicate also the type of data published by this topic and the values published by the sonar sensor when the quadrotor is standing on the ground. In order to see the measurement, it will be necessary to display the content of the topic by executing the command "rostopic echo".**

* Sonar quadrotor : `0.001`
* Sonar sensor : `hector_quadrotor_description/urdf/quadrotor.gazebo.xacro`
* Topic: `sonar height`
`hector_quadrotor_noetic/hector_models/hector_sensors_description/urdf/sonar_sensor.urdf.xacro`

```bash
$ rostopic echo sonar_height
```
```
header: 
  seq: 9820
  stamp: 
    secs: 982
    nsecs: 300000000
  frame_id: "sonar_link"
radiation_type: 0
field_of_view: 0.6981319785118103
min_range: 0.029999999329447746
max_range: 3.0
range: 0.16743995249271393

```

## Exercise 2

**Another way to publish the speed is by using the "rostopic pub" command. Write the command that publishes a linear velocity at z of 0.15 in the "cmd_vel" topic. Check it out visually on the simulator**
```bash
$ rostopic pub -r 100 /cmd_vel geometry_msgs/Twist '[0, 0, 0.15]' '[0, 0, 0]'
```

## Exercise 3

**Write the default values of the following PID controllers: “twist controller linear.x”, “twist controller angular.z” and “pose controller yaw”.**
```
controller:
  pose:
	…
    yaw:
      k_p: 2.0
      k_i: 0.0
      k_d: 0.0
      limit_output: 1.0

  …
    linear/xy:
      k_p: 5.0
      k_i: 1.0
      k_d: 0.0
      limit_output: 10.0
      time_constant: 0.05

  …
    angular/z:
      k_p: 5.0
      k_i: 2.5
      k_d: 0.0
      limit_output: 3.0
      time_constant: 0.1

```

## Exercise 4

**In the twist_controller.cpp file, what is the name of the variable that stores the desired and current linear.y velocities?. Copy the lines of code that implement the linear.z controller.**

`struct hector_quadrotor_controller::TwistController::<unnamed>`
`hector_quadrotor_controller::TwistController::pid_`
`pid_.linear.y.init(ros::NodeHandle(controller_nh, "linear/xy"));`
```c++
// Auto engage/shutdown
   if (auto_engage_) {
     if (!motors_running_ && command.linear.z > 0.1 && load_factor > 0.0) {
       motors_running_ = true;
       ROS_INFO_NAMED("twist_controller", "Engaging motors!");
     } else if (motors_running_ && command.linear.z < -0.1 /* && (twist.linear.z > -0.1 && twist.linear.z < 0.1) */) {
       double shutdown_limit = 0.25 * std::min(command.linear.z, -0.5);
       if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
       if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 5.0, period) < shutdown_limit) {
         motors_running_ = false;
         ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
       } else {
         ROS_DEBUG_STREAM_NAMED("twist_controller", "z control error = " << linear_z_control_error_ << " >= " << shutdown_limit);
       }
     } else {
       linear_z_control_error_ = 0.0;
     }

```

## Exercise 5

**In the motor_controller.cpp file, copy the lines of code which rotate the quadrotor in z**
```c++
// Update output
   if (wrench_.wrench.force.z > 0.0) {


     double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;
     motor_.force[0] =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 2.0 / parameters_.lever;
     motor_.force[1] =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 2.0 / parameters_.lever;
     motor_.force[2] =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 2.0 / parameters_.lever;
     motor_.force[3] =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 2.0 / parameters_.lever;


     double nominal_torque_per_motor = wrench_.wrench.torque.z / 4.0;
     motor_.voltage[0] = motor_.force[0] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
     motor_.voltage[1] = motor_.force[1] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;
     motor_.voltage[2] = motor_.force[2] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
     motor_.voltage[3] = motor_.force[3] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;


     motor_.torque[0] = motor_.voltage[0] * parameters_.torque_per_voltage;
     motor_.torque[1] = motor_.voltage[1] * parameters_.torque_per_voltage;
     motor_.torque[2] = motor_.voltage[2] * parameters_.torque_per_voltage;
     motor_.torque[3] = motor_.voltage[3] * parameters_.torque_per_voltage;


     if (motor_.voltage[0] < 0.0) motor_.voltage[0] = 0.0;
     if (motor_.voltage[1] < 0.0) motor_.voltage[1] = 0.0;
     if (motor_.voltage[2] < 0.0) motor_.voltage[2] = 0.0;
     if (motor_.voltage[3] < 0.0) motor_.voltage[3] = 0.0;


   } else {
     reset();
   }

```

## Exercise 6

**Modify the controller.launch in order to perform a pose controller. Use "rostopic pub" on the “command/pose” topic to make the quadrotor go up 2 meters.**

NOTE: making changes in a file that is not “ours” is not a good practice. Ideally, we would replicate a launch file in a new package to modify the controller there. Since we would have to replicate the launch files structure, we do this for the sake of simplicity.**

Modificar el fichero `~/catkin_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_controller/launch/controller.launch`

```xml
<launch>
  <rosparam file="$(find hector_quadrotor_controller)/params/controller.yaml" />

  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="
    controller/twist controller/pose
     --shutdown-timeout 3"/>
</launch>
```

```bash
rostopic pub /command/pose geometry_msgs/PoseStamped '{ header: {stamp: now, frame_id: "world"}, pose: { position: { x: 0.0, y: 0.0, z: 2.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }}}'
```
## Exercise 7

**Complete the package, pXX_arob_lab2_drones (where pXX corresponds to the assigned pair number for this course (e.g., p00, p01 ...)), with a node called followTargets3D that reads from a text file the list of 3D targets (X,Y,Z) and publish them one by one to the topic /command/pose. The new goal is published when the robot is “sufficiently close” to the previous target. We provide a preliminary structure of the package. Your tasks are to complete the package information in the “CMakeLists.txt” and “package.xml” with the dependencies and executables, and implement the missing parts of the code in “follow_targets_3d.hpp” and “follow_targets_3d.cpp”.**
```bash
rosrun lab2_drones followTargets3DNode _parameter_name:="targets.txt"
```

## Exercise 8
**Use the configuration cfg/arob_lab2.rviz to visualize the trajectory followed by the quadrotor for the different lists of targets provided in the lab. Include a screenshot of each one of them. Explain what is the cause for the different behaviors.**