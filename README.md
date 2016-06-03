*** A follow human program by Kosei Demura *** 

This program finds human by using the Hokuyo UTM-30LX lidar.

1. Environment  
   Ubuntu14.04 and ROS Iindigo

2. Build  
   $ cd ~/catkin_ws  
   $ catkin_make

3. Run  
   (1) Launch with other nodes  
   $ rosrun follow_human follow_human  

   (2) Stand-alone  
   $ roscd follow_human  
   $ cd script  
   $ ./follow_human.sh  

4. Topic   
  (1) Publish   
      name: find_human   
      type: std_msg/String   
      value: "true", "false"  
       
  (2) Subscribe   
      name: follow_human  
      type: std_msg/String  
      value: "start", "stop", "none"