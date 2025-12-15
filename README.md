# File structure
src  
├── rf2o_laser_odometry  
├── rplidar_ros  
├── Fields2Cover  
├── opennav_coverage  
├── servera_description    
├── servera_control   
└── servera_nav  

## Workspace Description:
**Ignore all the submodules, these are only dependencies.**

### Servera Description:
Defines only the definition of the robot. Nothing fancy!  
  
**src/servera_description/**  
├── **config** *:rviz config, ros2_gz_bridge configs are stored here*  
├── **src** *:Nothing is here, can be ignored !*  
├── **launch**  
├── **maps** *:All the generated slam maps are stored here!*  
├── **worlds** *:All the gz worlds are stored here!*   
├── **urdf** *:URDF file configured to work with gazebo, included ros_gz_bridge and gazebo_diff_drive_control*   (1)  
└── **hw_urdf** *:URDF file configured to work with physical robot, included ros2_control::diff_drive_controller.mock_controller*  (3)  

#### Launch Files  
**src/servera_description/launch/**  
├── **servera_basic.launch.py** *:launches the most basic urdf robot in rviz only. With the help of `joint_state_publisher_gui`; NO Sim, NO Hardware*  
└── **servera_sim.launch.py** *:launches the urdf robot in Gazebo, with gz_ros2_control*  

### Servera Control
Defines All the ros2_control settings and another urdf file configured with `ros2_control::diff_drive_controller` instead of `gazebo_diff_drive`.  

**src/servera_control/**  
├── **config** *:rviz config, ros2_gz_bridge config and ros2_controller configs are stored here*  
├── **launch**  
├── **src** *: [IGNORE] Nothing is here, can be ignored !*  
└── **urdf** *:URDF file configured to work with physical robot, included ros2_control::diff_drive_controller by gz_ros2_control*  (2)    

#### Launch Files   
**src/servera_control/launch/**  
└── **servera_control.launch.py** *:launches the urdf robot in Gazebo, with ros2_contro::diff_drive_controller by gz_ros2_control*    

### Servera Nav  
Puts together Everything from Nav2 stack to ros2_control and gazebo to Hardware. Full fancy !!!  

**src/servera_nav/**  
├── **config** *:rviz config, nav2 config and ros2_controller configs are stored here*  
├── **launch**  
└── **src** *: [IGNORE] Nothing is here, can be ignored !*  

#### Launch Files  
**src/servera_nav/launch/**    
├── **bringup_launch.py** *:`nav2` launch file, fully configured*  
├── **localization_launch.py** *:[IGNORE] `nav2` related launch file, generally do not need to be changed.*  
├── **navigation_launch.py** *:[IGNORE] `nav2` related launch file, generally do not need to be changed.*  
├── **rviz_launch.py** *:[IGNORE] Just for Debugging.*  
├── **servera_hw_nav.launch.py** *:launch file to launch everything with real hardware, utilises ros2_control and all required nodes.*  
├── **servera_sim_nav.launch.py**  *:launch file to launch everything in simulator, utilises gz_ros2_control and all required bridges.*  
└── **slam_launch.py** *:[IGNORE] `slam_toolbox` launch file, generally do not need to be changed.*  

## Note
**(1,2,3)** : This denotes the thought progression of this project.    
First we have created the `_description` which only is the URDF file, and uses (1)th URDF.  
then We graduated from the `gz_diff_drive` to `ros2_control::diff_drive_controller` which utilises (2)nd URDF.  
Finally we reached full autonomous in hardware, so started using (3)rd URDF file.  

I am aware the sequences are not syncronised properly, that is why included this note here.  