# A_Star_Implementation_Using_Turtlebot
ENPM661


## Authors:

    Name: Ishan Kharat (ishanmk@umd.edu)   UID: 120427145

    Name: Abhey Sharma (abheys16@umd.edu)  UID: 120110306




## Libraries Used:

    time
    cv2 - OpenCV
    heap
    NumPy
    matplotlib
    copy
    math
    rclpy
    geometry
    imutils
    



## How to Run the Code:

    git clone https://github.com/IshanMahesh/A_Star_Implementation_Using_Turtlebot.git

### Part 1

#### Run the command below:

    cd A_Star_Implementation_Using_Turtlebot/Part1_Ph2
    python3 a_star_abhey_ishan.py

#### Enter the following inputs:
    
    Enter the robot clearance (in mm): 5
    
    Enter the obstacle clearance (in mm): 5
    
    Enter the coordinates of starting node (x,y,theta): 500,500,0
    
    Enter the coordinates of Goal node (x,y):5750,1000
    

#### Output Video:

Link: https://drive.google.com/file/d/1NYKFlfsSCPhLPeSJHk0FXOuWjbfaKdDo/view?usp=sharing


### Part 2

#### Output Video:

Link: https://drive.google.com/file/d/1d280lUseZ6i3Nwq-4se6-K_NOdSmtCfc/view?usp=sharing

#### Open the terminal and run the following:

    #create a workspace
    cd /project3 ws

    source install/setup.zsh

    colcon build

    ros2 launch turtlebot3_project3 competition_world.py



#### In another terminal simultaneously run:

    cd project3_ws

    source install/setup.zsh

    ros2 run turtlebot3_project3 astar.py




#### Enter the following inputs:
    
    Enter the robot clearance (in mm): 5
    
    Enter the obstacle clearance (in mm): 5
    
    Enter the coordinates of starting node (x,y,theta): 500,500,0
    
    Enter the coordinates of Goal node (x,y):5750,1000
    
    Enter rpm1,rpm2: 5,10

Note: the following inputs above are being hardcoded [i.e already implemented in the code]



    
