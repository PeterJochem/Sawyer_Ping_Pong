# Schedule

## Week 1
This week I considered what exactly I wanted to do for my winter quarter project. I am considering doing a machine learning project or try to make Sawyer play ping pong. I downloaded and installed Tensorflow and began reading up on how to use it.

## Week 2
I continued research on how to use Tensorflow. I took the triple penduluum simulation that we built for dynamics and built a small machine learning project with it. I wanted to see if I could have a neural network learn the function which we used Lagrangian dynamics to derive. I did a few demos on building SQL databases and implemented one that uses the penduluum simulation to generate data. I then used Tensorflow to predict the trajectory of the penduluum. It worked fairly well but would require to run with a lot of initial conditions to learn a good function approximation.  

## Week 3
I decided to do the Sawyer robot ping pong project. I have a few things I need to read up on. I need to learn the basics of using C++ ROS. I also need to get MoveIt for Sawyer setup. I starting exploring the point cloud cameras that the lab has. Intel has a pretty extensive sdk for it and even a ROS wrapper for it. 

## Week 4
Got MoveIt for Sawyer setup. Experiemnted with it. Needed to make a few changes  
Struggled to set up point cloud camera. Very frustrated.


## Week 5
More struggle with the pointcloud camera. Really frustrated. My computer also crashed. Matt helped me reinstall Ubuntu


## Week 6
Matt helped me solve the point cloud issues. I found a github issue showing that the ROS wrapper had issues related to timestamping of the data which had caused issues and error messages identical to mine. The post was very recent, within 2 weeks. Intel is aware of the problem and is working to fix it. Matt helped me roll back my kernel module and we have it working. 


## Week 7
I learned the basics of PCL. I browsed Jarvis's point cloud demo and put together a pipeline. My code takes in a point cloud from the camera and computes the centroid of the cloud. This information is then published to other nodes. I set up a node to record the position and the time and then use multiple data points to compute the velocity. I then computed where the ball would cross the plane of contact.


## Week 8
Matt advised me to change my approach to computing where the ball would be in the future. He suggested waiting for a few points to come in and then fit a plane to the data points. Once we have the plane, we can convert the points into the plane frame and fit a parabola to the data. He recommended using Armadillo to do the linear algebra. I implemented the first part. I setup the code to get a few data points and then compute the plane which it is moving in.   


## Week 9
Found where MoveIt is limiting the velocity and acceleration of the arm. I changed the file and now the arm moves faster. I also implemented fitting of the parabola to the points in the plane frame. I also added a table to the RVIZ enviroment. 


## Week 10
I struggled with the linear algebra of how to compute where the plane of contact would intersect the ball's trajectory. I implemented a binary search instead that works quite well. 


## Week 11
I did a lot of cleaning up of my code and documenting
