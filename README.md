# Sensor_Fusion_of_GPS_and_IMU_Data_for_Automotive_Dead_Reckoning
This project was done as a lab assignment for the EECE 5554 (Robotics Sensing and Navigation) course at Northeastern University.


### Table of Contents
* [Aim](#aim)
* [Data Collection](#data-collection)
  * [USB-based GNSS GPS puck](#usb-based-gnss-gps-puck)
  * [VN-100 VectorNav IMU](#vn-100-vectornav-imu)
  * [NUANCE Autonomous Car](#nuance-autonomous-car)
  * [data going in circles](#data-going-in-circles)
  * [data driving](#data-driving)
  * [Videos of the data collection](#videos-of-the-data-collection)
* [Custom ROS 2 Messages and Drivers](#custom-ros-2-messages-and-drivers)
* [Launching the ROS 2 Drivers](#launching-the-ros-2-drivers)
* [The list of plots included in the analysis of the data](#the-list-of-plots-included-in-the-analysis-of-the-data)
  * [Using the data going in circles dataset](#using-the-data-going-in-circles-dataset)
  * [Using the data driving dataset](#using-the-data-driving-dataset)
* [Results](#results)
* [Acknowledgements](#acknowledgements)

  
### Aim 
The main goal of the project was to collect GPS and IMU data using the NUANCE autonomous car provided by Northeastern University. For collecting the data, we had to create custom ROS 2 messages and drivers for the GPS and IMU sensors and combine those drivers to get a single custom message with a single timestamp. After collecting the data, we had to analyzed the IMU's noise characteristics through Allan Variance and calibrate magnetometer by correcting hard and soft iron distortions along with error compensation in IMU and GPS data. After that, we had to compensate for accelerometer bias to estimate vehicle’s forward velocity, and fuse the yaw angle computed from gyroscope and magnetometer data using a complementary filter to estimate heading for Dead Reckoning with IMU.


### Data Collection
We used a USB-based GNSS GPS puck and the VN-100 VectorNav IMU for data collection. We attached these sensors to the NUANCE autonomous car. 

#### USB-based GNSS GPS puck
![gps-ezgif com-webp-to-png-converter](https://github.com/user-attachments/assets/dfa6b4fe-38c9-49cc-ae0f-dd0d0631bc75)

#### VN-100 VectorNav IMU
![vn-100-rugged](https://github.com/user-attachments/assets/7c60ed0e-e17e-4c02-ba53-d41ea1d8813b)

#### NUANCE Autonomous Car
![NUACNE](https://github.com/user-attachments/assets/515a50da-5f9c-4a47-9ce9-91551e6d95a3)

We collected two different datasets namely: ```data_going_in_circles``` and ```data_driving```. Each contains both, GPS and IMU data. Both the datasets are described below:

#### data going in circles
- We travelled 3 circular laps, with increasing speeds in each lap, around the Ruggles Circle near Northeastern University.
- The purpose of this dataset is solely to remove the hard and soft iron distortions from the data and calibrate the magnetometer.
  
#### data driving
- For this dataset, we drove around the streets of Boston for several minutes and ensured that our path had a lot of turns.
- We returned to the point from where we started.
- Using the calibration parameters obtained by working on the ```data_going_in_circles```, the magnetometer data is calibrated for this dataset too, and the entire sensor fusion and dead reckoning part of this project is performed on this dataset.

#### Videos of the data collection
- [For the ```data_going_in_circles``` dataset](https://drive.google.com/file/d/1-yoloWRnFQICCxjGrL79DWdCdvbKACdr/view?usp=sharing)
- [For the ```data_driving``` dataset](https://drive.google.com/file/d/1S9o-afP9GGBGx-hGzHpTd-3-I1fUXB0i/view?usp=sharing)


### Custom ROS 2 Messages and Drivers
- The ```gps_msgs``` package contains a custom ROS 2 message called ```GPSmsg.msg``` that has ```header```, ```latitude```, ```longitude```, ```altitude```, ```utm_easting```, ```utm_northing```, ```zone```, and ```letter``` as fields.
- The ```imu_custom_message``` package contains a custom ROS 2 message called ```IMUmsg.msg``` that has ```header```, ```imu```, ```mag_field```, and ```raw_data (A string with any name containing the raw IMU string)``` as fields. 
- The ```gps_driver``` reads in serial data from the puck, parses it for the latitude, longitude and altitude. It converts the latitude and longitude to UTM. It then publishes this custom ROS 2 message over a topic called ```/gps```. 
- The ```imu_driver``` parses the $VNYMR string, to get accel, gyro, orientation (roll, pitch, yaw) and magnetometer data. It converts the Yaw, Pitch, Roll data into quaternions and publishes it as orientation in the same ```imu_custom_msg```.
- Since the GNSS puck and the VectorNav IMU run at different frequencies, with the IMU having a much higher frequency, the timestamps obtained from the IMU are utilized while performing the analysis. 

### Launching the ROS 2 Drivers
To run the launch file that will launch both the GPS and IMU drivers together, please use the following command: <br>
```ros2 launch imu_driver master_launch.py gps_port:=<Insert the GPS port's path> imu_port:=<Insert the IMU port's path>```

For example,<br>
```ros2 launch imu_driver master_launch.py gps_port:=/dev/pts/3 imu_port:=/dev/pts/4```


### The list of plots included in the analysis of the data

#### Using the ```data going in circles``` dataset
- The magnetometer X-Y plot before and after hard and soft iron calibration.

#### Using the ```data driving``` dataset
- The time series magnetometer data before and after the correction.
- Magnetometer Yaw and Yaw Integrated from Gyro together.
- Low Pass Filter, High Pass Filter, and Complementary Filter plots together.
- Yaw from the Complementary filter and Yaw angle computed by the IMU together.
- Velocity estimate from the GPS with Velocity estimate from accelerometer before adjustment.
- Velocity estimate from the GPS with Velocity estimate from accelerometer after adjustment.
- 𝜔𝑋̇ and 𝑦̈𝑜𝑏𝑠 plotted together.
- A single plot showing the path followed shown by GPS & path followed estimated by IMU.


### Results
The analysis, all the plots, and the final results are included in the ```Project_Report.pdf``` file in detail. 


### Acknowledgements
I would like to specially thank our course instructor Prof. Hanumant Singh as well as our course teaching assistants Vishnu Rohit Annadanam and Jasen Levoy who helped me immensely not only while doing this project but throught the course. They were great at teaching, managing the course, and were always available and enthusiastic to solve everyone's doubts.
I would also like to thank Northeastern University for providing the students with the NUANCE autonomous car so that we could collect our data.
