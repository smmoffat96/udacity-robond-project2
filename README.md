# udacity-robond-project2
Project 2 submission for Udacity Robotics Software Engineer Nanodegree Program

Download packages ball_chaser and my_robot into catkin_ws folder
1. Build the packages and launch the world
    catkin_make
    source devel/setup.bash
    roslaunch my_robot world.launch
2. Launch ball_chaser launch in new terminal window
    source devel/setup.bash
    roslaunch ball_chaser ball_chaser.launch
3. Move the white ball around in Gazebo to watch the robot chase the ball!
    Note: The robot should stop when there is no white ball in view, or when it gets close to the ball.
