# Sensor_Fusion_of_GPS_and_IMU_Data_for_Automotive_Dead_Reckoning

This project was done as a lab assignment for the EECE 5554 (Robotics Sensing and Navigation) course at Northeastern University.

### Aim 

The main goal of the project was to collect GPS and IMU data using the NUANCE autonomous car provided by Northeastern University. For collecting the data, we had to create custom ROS 2 messages and drivers for the GPS and IMU sensors and combine those drivers to get a single custom message with a single timestamp. After collecting the data, we had to analyzed the IMU's noise characteristics through Allan Variance and calibrate magnetometer by correcting hard and soft iron distortions along with error compensation in IMU and GPS data. After that, we had to compensate for accelerometer bias to estimate vehicle’s forward velocity, and fuse the yaw angle computed from gyroscope and magnetometer data using a complementary filter to estimate heading for Dead Reckoning with IMU.

### Data Collection

We used a USB-based GNSS GPS puck and the VN-100 VectoNav IMU for data collection.

##### USB-based GNSS GPS puck
![gps-ezgif com-webp-to-png-converter](https://github.com/user-attachments/assets/dfa6b4fe-38c9-49cc-ae0f-dd0d0631bc75)

##### VN-100 VectoNav IMU
![vn-100-rugged](https://github.com/user-attachments/assets/7c60ed0e-e17e-4c02-ba53-d41ea1d8813b)

We collected two different datasets namely: ```data_going_in_circles``` and ```data_driving```. Each contains both, GPS and IMU data. 

##### data_going_in_circles

- We travelled 3 circular laps, with increasing speeds in each lap, around the Ruggles Circle near Northeastern University.
- The purpose of this dataset is solely to remove the hard and soft iron distortions from the data and calibrate the magnetometer.
  
##### data_driving

- For this dataset, we drove around the streets of Boston for several minutes and ensured that our path had a lot of turns.
- We returned to the point from where we started.
- Using the calibration parameters obtained by working on the ```data_going_in_circles```, the magnetometer data is calibrated for this dataset too, and the entire sensor fusion and dead reckoning part of this project is performed on this dataset.

##### Videos of the data collection










### Acknowledgements

I would like to specially thank our course instructor Prof. Hanumat Singh as well as our course teaching assistants Vishnu Rohit Annadanam and Jasen Levoy who helped me immensely not only while doing this project but throught the course. They were great at teaching, managing the course, and were always available and enthusiastic to solve everyone's doubts.
I would also like to thank Northeastern University for providing the students with the NUANCE autonomous car so that we could collect our data.
