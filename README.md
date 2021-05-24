# SLAM: ROS environement packages
## Description:
During my intern, the goal was to prepare an environement for the back-end of a SLAM (Simultanious Localization And Mapping) algorithm. Based on a fusion of sensors data (rover speed-steering and camera-based estimation of landmarks position), the back-end estimates the localization of the robot and the position of the scene landmarks (mapping). The front end goal is to provide synchronised and adapted sensors data to the back-end. 

For validation and testing purposes, a complete suit is devolopped to visualize the results of the SLAM algorithm and compare them to an odometry based localization and mapping.

The used back-end algorithm is EKF-SLAM. GraphSLAM was also tested.

## Results:
The following image represents a map of the test building. The map is acquired using an out-of-the-box SLAM algorithm (gmapping) and a LIDAR. The green squares represents the real position of the landmarks.
![1a0da84d5b00c8599af9b8c20b9f3b87.png](./_resources/419c72a5c85647c293528d9c09d877d7.png)

The next image illustrate the result of odometry-based path estimation. Odometry consists of the rover's wheels speed, steering and the rover's (x,y) coordinates in the frame associated to initial position:
![37026b4cf6d40527c93d7c6df42132c4.png](./_resources/bda52593e41b4cc58a9aeccc8fa07812.png)

As one can remark, the rover's path goes through the building walls. Odomotry accumulates error and path estimation finishes by deviating from the real path. The position of landmarks (April tag detectable by camera) placed in the buildings is also estimated:
![46f436053f9c8eeb64e52da34c052319.png](./_resources/bd28d114c646486db3cedc6dabb29268.png)

Instead of using the real landmarks position to correct the path estimation (simpler filtering problem), this information is considered unknown and estimated using mono-camera and the current odometry pose estimation. The odometry path and landmarks positions estimations serves for a sensor fusion to correct both the path and the landmarks position:

![e7ac5be6b8a43bb0881777b6adb1831b.png](./_resources/45f7dc43dce14ed4849b298e41372412.png)

For further details, please refer to the intern presentation's slides:
https://docs.google.com/presentation/d/1Lh5SFXUTre3Tx0ith_qOZM8cdH3SMtDPEAFWdO35tQM/edit?usp=sharing
