# Description
I set out to have a Sawyer arm play ping pong. I used MoveIt to do the motion planning, an Intel Real Sense camera to capture point clouds, and ROS to integrate all of these parts. 

# Results
Below is an image from RVIZ. The blue points describe the volume over the ping pong table. The gray points indicate the measured point's position from the point cloud camera. The red points are the parabola I fit (in real time) to the measured position of the ball.   

   
![Real Tiem Object Tracking]( https://github.com/PeterJochem/Sawyer_Ping_Pong/blob/master/images/trajectory_1.png "Logo Title Text 1")
Direct people to look at the image folder - insert link


## ROS Packages in this Repo
This repo contains three ROS packages. It contains the SawyerMoveIt package with a handful of changes needed to get our particular Sawyer to run. I needed to modify the URDF and make a few more small, minor changes. 

It also has a package called point_cloud_processing which handles setting up and processing the point cloud data from the structured light camera. I decided to make this a seperate package from the rest of the project's code because I think I will end up re-using the point cloud processing code. It will be easier/more modular to keep all the code related to point cloud processing in it's own package. The repo also contains a package called sawyer_ping_pong. This contains all the code related to integrating the SawyerMoveIt and the point cloud processing code.    




