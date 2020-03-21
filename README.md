# Description
I set out to have a Sawyer arm play ping pong. I used MoveIt to do the motion planning, an Intel Real Sense camera to capture point clouds, and ROS to integrate all of these parts. 

# Results
Below is an image from RVIZ. The blue points describe the volume over the ping pong table. The gray points indicate the measured point's position from the point cloud camera. The green points are a plane I fit (in real time) to the observed ball's position. The red points are the parabola I fit (in real time) to the measured position of the ball.   

   
![Real Time Object Tracking]( https://github.com/PeterJochem/Sawyer_Ping_Pong/blob/master/images/trajectory_1.png "Logo Title Text 1")

![Real Time Object Tracking]( https://github.com/PeterJochem/Sawyer_Ping_Pong/blob/master/images/trajectory.gif "Logo Title Text 1")

More videos and images from the system running in RVIZ are available [here](https://github.com/PeterJochem/Sawyer_Ping_Pong/tree/master/images)

# Implementation
So, for Sawyer to play ping pong, I needed time to capture point cloud data, process it, do motion planning, and actually have the arm move. In order to speed up the point cloud processing, I first downsample the initial point cloud with the point cloud library (PCL). I then assume that the only object in the volume above the table is the ball. This allows me to compute the ball's location by computing the centroid of the given point cloud. I gather 3-5 point clouds and compute the ball's position in each. I then use a least squares approximation to fit a plane to the given locations. Ignoring aerodynamic effects, the ball should move in a plane since only the z-component of its acceleration is ever non-zero. Once we have the parameters describing the plane, we can fit a parabola to the points in the plane frame. This allows me to predict where the ball will be at future time steps. I then do a binary search on points in the parabola to figure out where the parabola intersects the plane of contact. I choose to do a binary search since the parabola is defined in the plane frame and the plane of contact is defined in the robot's frame. A solution is found extremly fast so I opted to not do the linear algebra and derive a closed form solution. Once I know the location of predicted contact in the robot's frame, I can use MoveIt to move the arm there.         


# ROS Packages in this Repo
This repo contains three ROS packages. It contains the SawyerMoveIt package with a handful of changes needed to get our particular Sawyer to run it. I needed to modify the URDF and make a few more small, minor changes. 

It also has a package called point_cloud_processing which handles setting up and processing the point cloud data from the structured light camera. I decided to make this a seperate package from the rest of the project's code because I think I will end up re-using the point cloud processing code. It will be easier/more modular to keep all the code related to point cloud processing in it's own package. The repo also contains a package called sawyer_ping_pong. This contains all the code related to integrating the SawyerMoveIt and the point cloud processing code.     

# How to run My Code
Download ROS-melodic. Download the .rosinstall file. 
Run ```wstool init```
Run ```wstool set .rosinstall```
Run ```wstool update```

