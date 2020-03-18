# Description

# Results
Insert images
![Real Tiem Object Tracking]( https://github.com/PeterJochem/Sawyer_Ping_Pong/blob/master/images/trajectory_1.png "Logo Title Text 1")
Direct people to look at the image folder - insert link


## ROS Packages in this Repo
This repo contains three ROS packages. It contains the SawyerMoveIt package with a handful of changes needed to get our particular Sawyer to run. I needed to modify the URDF and make a few more small, minor changes. 

It also has a package called point_cloud_processing which handles setting up and processing the point cloud data from the structured light camera. I decided to make this a seperate package from the rest of the project's code because I think I will end up re-using the point cloud processing code. It will be easier/more modular to keep all the code related to point cloud processing in it's own package. The repo also contains a package called sawyer_ping_pong. This contains all the code related to integrating the SawyerMoveIt and the point cloud processing code.    




