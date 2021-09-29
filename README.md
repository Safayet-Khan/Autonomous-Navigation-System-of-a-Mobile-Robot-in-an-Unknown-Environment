# Autonomous-Navigation-System-of-a-Mobile-Robot-in-an-Unknown-Environment
This repository contains my source code and implementation video of autonomous navigation system of a mobile robot. The proposed algorithms follow short and smooth trajectory. The algorithm we have used is a goal-oriented algorithm which always sets its path according to the location of the goal point. But in the wall following algorithm, while the robot is trying to follow a wall, certainly it does not know the outside of its disk abstraction. The robot takes the decision depending on the data it is receiving using sonar range sensors. But the sensor does not have a wide range. The robot takes the decision about its movement based on the clock-wise and counter-clock-wise direction vector multiplication. Therefore, sometimes it may travel more path than expected.

### Short Description of Each File
1. **pid_error_calculation.ino**-
2. **encoder_sonar_setup_with_previous.ino**-
3. **go_to_goal_algorithm_with_previous.ino**-
4. **obstacle_avoidance_algorithm_with_previous.ino**-
5. **follow_wall_algorithm_with_previous.ino**-
