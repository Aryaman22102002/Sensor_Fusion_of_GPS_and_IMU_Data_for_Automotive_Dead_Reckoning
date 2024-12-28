import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from scipy.integrate import cumulative_trapezoid
from scipy import stats
from scipy import signal
from scipy.interpolate import interp1d
import matplotlib.ticker as ticker

def calculate_gps_heading(gps_easting,gps_northing):
    gps_data = np.arctan2(gps_northing.diff(), gps_easting.diff())
    gps_data = np.rad2deg(gps_data)
    gps_data = np.nan_to_num(gps_data, nan=0.0)
    gps_data = np.unwrap(gps_data)
    
    return gps_data

def calculate_gps_forward_vel(gps_easting, gps_northing, gps_time):
    # Calculate the differences in easting, northing, and time
    delta_easting = np.diff(gps_easting)
    delta_northing = np.diff(gps_northing)
    delta_time = np.diff(gps_time)

    # Calculate velocities in East and North directions
    velocity_east = delta_easting / delta_time
    velocity_north = delta_northing / delta_time

    # Calculate forward velocity
    velocity_forward_gps = np.sqrt(velocity_east**2 + velocity_north**2)
    velocity_forward_gps = np.pad(velocity_forward_gps, (1, 0), 'edge')
    
    return velocity_forward_gps

def quaternion_to_euler(w, x, y, z):
    t1 = 2.0 * (w * z + x * y)
    t2 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t1, t2)

    return yaw

def find_stationary_periods(accel_x, threshold=0.05):
    forward_accel = accel_x.copy()  
    offset_value = 0
    count = 0
    start_index = 0
    flag = False

    # Loop through each accelerometer data point
    for i in range(len(forward_accel)):
        # If absolute acceleration is below threshold, consider the car stationary
        if abs(forward_accel[i]) < threshold:
            count += 1
            if count == 1:
                # Start of a stationary cluster
                start_index = i
                flag = True
            # Continue until the cluster of stationary points is over
            continue
        else:
            # If a stationary cluster is over
            if flag:
                # Take the last value in the cluster as the offset (the last stationary point)
                offset_value = forward_accel[i - 1]
                flag = False

                # Adjust the values in the stationary period (from start_index to i-1) o 0
                for j in range(start_index, i):
                    if offset_value > 0:
                        forward_accel[j] = 0 
                    elif offset_value < 0:
                        forward_accel[j] = 0  

            # Reset count for the next cluster
            count = 0

        # Continue adjusting the rest of the data as the car moves
        if forward_accel[i] > 0:
            forward_accel[i] -= offset_value
        elif forward_accel[i] < 0:
            forward_accel[i] += abs(offset_value)

    return forward_accel

def correct_accelerometer_estimate(accel_x):
    index = 0
    for i in range(30,len(accel_x)):
        initial_neg_bias = np.mean(accel_x[:i])
        if initial_neg_bias > 0:
            index = i
            break
    accel_x[:index] = accel_x[:index] - initial_neg_bias
    accel_x[index:] = accel_x[index:] - np.mean(accel_x[index:])
    corrected_accel_x = find_stationary_periods(accel_x, 0.035)
   
    return corrected_accel_x

def calibration_value_mag(data):
    # Center the data by subtracting the mean
    magxmean, magymean = data[0].mean(), data[1].mean()
    data[0] -= magxmean
    data[1] -= magymean

    # Singular Value Decomposition (SVD) for finding the best-fit ellipse
    U, S, V = np.linalg.svd(np.stack((data[0], data[1])))
    N = len(data[0])
    
    # Create a unit circle for fitting
    tt = np.linspace(0, 2 * np.pi, N)
    circle = np.stack((np.cos(tt), np.sin(tt)))  # unit circle
    transform = np.sqrt(2 / N) * U.dot(np.diag(S))  # transformation matrix
    fit = transform.dot(circle)  # apply transformation to the unit circle

    # Get calibration values from the best fit ellipse
    a_index = np.argmax(fit[0, :])  # Major axis (X axis)
    b_index = np.argmax(fit[1, :])  # Minor axis (Y axis)

    # Calculate the rotation angle theta of the ellipse
    theta = math.atan2(fit[1, a_index], fit[0, a_index])

    # Convert theta to a positive angle (if needed)
    if theta < 0:
        theta += 2 * np.pi

    # Calculate the lengths of the semi-major (a) and semi-minor (b) axes
    a = math.sqrt(fit[1, a_index]**2 + fit[0, a_index]**2)
    b = math.sqrt(fit[1, b_index]**2 + fit[0, b_index]**2)

    # Restore original mean offsets
    data[0] += magxmean
    data[1] += magymean
    
    return magxmean, magymean, theta, a, b

def calibrate_mag(data, cx, cy, theta, a, b):
    # Translate (remove offset)
    translated_x = data[0] - cx
    translated_y = data[1] - cy

    # Rotate (undo misalignment)
    rotated_x = np.cos(theta) * translated_x + np.sin(theta) * translated_y
    rotated_y = -np.sin(theta) * translated_x + np.cos(theta) * translated_y

    # Scale (correct axis distortion)
    scaled_x = rotated_x * (b / a)  # Apply scaling correction
    scaled_y = rotated_y  # No scaling needed for Y-axis
    
    return [scaled_x, scaled_y]

def butter_filter(data, fs, cutoff, type):
    nyq = 0.5*fs
    order = 2 
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    sos = signal.butter(order, normal_cutoff, btype=type, output='sos', analog=False)
    y = signal.sosfilt(sos, data)
    
    return y

def rotate_points_about_z(x, y, angle):
    c, s = np.cos(angle), np.sin(angle)
    x_rot = x * c - y * s
    y_rot = x * s + y * c
    
    return x_rot, y_rot

def process_gps_imu_graphs(xe, xn, gps_easting, gps_northing):
    # Calculate distances between consecutive GPS points
    gps_segment_distances = np.sqrt(np.diff(gps_easting)**2 + np.diff(gps_northing)**2)
    imu_segment_distances = np.sqrt(np.diff(xe)**2 + np.diff(xn)**2)
    non_zero_mask = gps_segment_distances != 0
    scale_factor = np.mean(imu_segment_distances[non_zero_mask] / gps_segment_distances[non_zero_mask])
    xe *= 1 / scale_factor
    xn *= 1 / scale_factor
    diff_easting = np.diff(gps_easting)
    diff_northing = np.diff(gps_northing)
    angles = np.arctan2(diff_northing, diff_easting)
    angles = np.rad2deg(angles)
    angle_changes = np.diff(angles)

    # Define threshold and find first direction change
    threshold = 10
    direction_changes = None  # Initialize to None in case no turn is found

    for i, change in enumerate(angle_changes):
        if abs(change) > threshold:
            direction_changes = i  # Store the index of the first detected change
            break  # Stop the loop after finding the first change

   # print("First significant direction change index:", direction_changes)
    
    # Only proceed if a direction change is found
    if direction_changes is not None:
        heading_gps = np.arctan2(gps_northing[direction_changes] - gps_northing[0], 
                                 gps_easting[direction_changes] - gps_easting[0])
      #  print("Initial GPS heading at first turn:", np.degrees(heading_gps))
    else:
        print("No significant direction change found.")
        
  #  heading_gps_next = heading_gps
  #  index = 1

    # Adjust for heading difference between IMU and GPS
  #  while abs(heading_gps_next - heading_gps) < 4:
  #      index = index + 1
   #     heading_gps_next = np.arctan2(gps_northing[index] - gps_northing[index-1], gps_easting[index] - gps_easting[index-1])

    heading_imu = np.arctan2(xn[direction_changes] - xn[0], xe[direction_changes] - xe[0])
    rotation_angle = heading_gps - heading_imu

    # Rotate IMU coordinates to match GPS
    xe_rotated, xn_rotated = rotate_points_about_z(xe, xn, rotation_angle)
    #print(heading_gps)

    # Apply translation to align the first point
    shift_easting = gps_easting[0] - xe_rotated[0]
    shift_northing = gps_northing[0] - xn_rotated[0]
    xe_shifted = xe_rotated + shift_easting
    xn_shifted = xn_rotated + shift_northing

    return xe_shifted, xn_shifted



# Importing the data files.
file_1_path = 'data_going_in_circles.csv'  
file_2_path = 'data_driving.csv'
df_1 =  pd.read_csv(file_1_path)
df_2 = pd.read_csv(file_2_path)

df_1 = df_1.dropna(subset=['magnetic_field_x', 'magnetic_field_y'])
df_imu = df_2.dropna(subset=['timestamp_sec_imu','timestamp_nanosec_imu'])
df_gps = df_2.dropna(subset=['timestamp_sec_gps','timestamp_nanosec_gps'])

# Convert magnetic field from Tesla to microtesla (µT) for circles dataset
df_1['magnetic_field_x_µT'] = df_1['magnetic_field_x'] * 1e6
df_1['magnetic_field_y_µT'] = df_1['magnetic_field_y'] * 1e6
# Convert magnetic field from Tesla to microtesla (µT) for driving dataset
df_imu['magnetic_field_x_µT'] = df_imu['magnetic_field_x'] * 1e6
df_imu['magnetic_field_y_µT'] = df_imu['magnetic_field_y'] * 1e6
# Convert time data in proper format
df_imu['timestamp_imu'] = df_imu['timestamp_sec_imu'] + (df_imu['timestamp_nanosec_imu'] * 1e-9)
df_imu['timestamp_imu'] = df_imu['timestamp_imu'] - df_imu['timestamp_imu'].iloc[0]
imu_time = df_imu['timestamp_imu']
df_gps['timestamp_gps'] = df_gps['timestamp_sec_gps'] + (df_gps['timestamp_nanosec_gps'] * 1e-9)
df_gps['timestamp_gps'] = df_gps['timestamp_gps'] - df_gps['timestamp_gps'].iloc[0]
gyro_z = df_imu['angular_velocity_z']
# Convert angular velocity from rad/s to deg/s
gyro_z_degrees = np.rad2deg(gyro_z)
# Get the GPS data
gps_easting = df_gps['utm_easting']
gps_northing = df_gps['utm_northing']
gps_time = df_gps['timestamp_gps']

# Get the accelerometer data
accel_x = df_imu['linear_acceleration_x']
accel_y = df_imu['linear_acceleration_y']

# Caclulate yaw obtained from quaternions
quaternion_w = df_imu['orientation_w'].values
quaternion_x = df_imu['orientation_x'].values
quaternion_y = df_imu['orientation_y'].values
quaternion_z = df_imu['orientation_z'].values

yaw_imu = []

# Loop over each quaternion and convert to Euler
for w, x, y, z in zip(quaternion_w, quaternion_x, quaternion_y, quaternion_z):
    yaw = quaternion_to_euler(w, x, y, z)
    yaw_imu.append(yaw)
    
yaw_imu = np.array(yaw_imu)
yaw_imu = np.unwrap(yaw_imu)
yaw_imu = np.rad2deg(yaw_imu)


# Prepare magnetometer data for circlesdataset
mag_x_1 = df_1['magnetic_field_x_µT'].values
mag_y_1 = df_1['magnetic_field_y_µT'].values
# Combine magnetometer data into a 2xN array
mag_data_1 = np.vstack((mag_x_1, mag_y_1))
# Get calibration values from the remaining points
cal_center_x_1, cal_center_y_1, rot_angle_1, major_ax_1, minor_ax_1 = calibration_value_mag(mag_data_1)
# Calibrate the magnetometer data using the filtered data only
calb_mag_data_1 = calibrate_mag(mag_data_1, cal_center_x_1, cal_center_y_1, rot_angle_1, major_ax_1, minor_ax_1)


# Prepare magnetometer data for driving dataset
mag_x_2 = df_imu['magnetic_field_x_µT'].values
mag_y_2 = df_imu['magnetic_field_y_µT'].values
# Combine magnetometer data into a 2xN array
mag_data_2 = np.vstack((mag_x_2, mag_y_2))
# Calculating yaw from uncorrected magnetometer
yaw_raw_mag = np.arctan2(mag_data_2[1], mag_data_2[0])
yaw_unwrapped_raw_mag = np.unwrap(yaw_raw_mag)
yaw_degrees_raw_mag = np.degrees(yaw_unwrapped_raw_mag)
# Get calibration values from the remaining points
cal_center_x_2, cal_center_y_2, rot_angle_2, major_ax_2, minor_ax_2 = calibration_value_mag(mag_data_2)
# Calibrate the magnetometer data using the filtered data only
calb_mag_data_2 = calibrate_mag(mag_data_2, cal_center_x_2, cal_center_y_2, rot_angle_2, major_ax_2, minor_ax_2)
# Calculating yaw from corrected magnetometer
yaw_corrected_mag = np.arctan2(calb_mag_data_2[1], calb_mag_data_2[0])
yaw_corrected_mag = np.array(yaw_corrected_mag)
yaw_corrected_mag = np.unwrap(yaw_corrected_mag)
yaw_degrees_corrected_mag = np.degrees(yaw_corrected_mag)



# Integrate the yaw rate from gyroscope over time
integrated_yaw_from_gyro = cumulative_trapezoid(gyro_z_degrees, imu_time, initial=0)
# Applying low pass filter to Mag data
mag_yaw_filtered = butter_filter(yaw_degrees_corrected_mag, 40, 0.09, 'low')
#Applying high pass filter to Gyro data
gyro_yaw_filtered = butter_filter(integrated_yaw_from_gyro, 40, 0.00001, 'high')
# Combine the low-pass filtered magnetometer yaw and high-pass filtered gyro yaw
alpha = 0.97
complementary_filter_yaw = (1 - alpha) * mag_yaw_filtered + alpha * gyro_yaw_filtered  
complementary_filter_yaw = np.array(complementary_filter_yaw)
complementary_filter_yaw = np.deg2rad(complementary_filter_yaw)
complementary_filter_yaw = np.unwrap(complementary_filter_yaw)
complementary_filter_yaw = np.rad2deg(complementary_filter_yaw)
# Correcting offset for the yaw from VN-100 IMU
raw_imu_yaw_offset = abs(complementary_filter_yaw[0] - yaw_imu[0])
for i in range(len(yaw_imu)):
    yaw_imu[i] += raw_imu_yaw_offset
#gps_data = calculate_gps_heading(gps_easting,gps_northing)




# Integrate forward acceleration to obtain forward velocity
integrated_accel_x = cumulative_trapezoid(accel_x, imu_time, initial=0) 
# Calculate the forward velocity from the GPS
velocity_forward_gps = calculate_gps_forward_vel(gps_easting,gps_northing,gps_time)
# Correct the bias due to stationary points in the forward acceleration data
accel_x = np.array(accel_x.copy())
corrected_accel_x = correct_accelerometer_estimate(accel_x)
integrated_corrected_accel_x = cumulative_trapezoid(corrected_accel_x, imu_time, initial=0) 
for x in range(len(integrated_corrected_accel_x)):
    if integrated_corrected_accel_x[x] < 0:
       integrated_corrected_accel_x = np.maximum(integrated_corrected_accel_x, 0)
       



# Compute ωẊ
omega_x_dot = gyro_z * integrated_corrected_accel_x

# Integrate Forward velocity from IMU to get trajectory
v_imu = integrated_corrected_accel_x
complementary_filter_yaw = np.deg2rad(complementary_filter_yaw)
ve = v_imu * np.cos(complementary_filter_yaw)
vn = v_imu * np.sin(complementary_filter_yaw)
xe = cumulative_trapezoid(ve,imu_time,initial=0)
xn = cumulative_trapezoid(vn,imu_time,initial=0)
# Create interpolation functions for GPS easting and northing based on GPS timestamps
gps_easting_resampled = interp1d(gps_time, gps_easting, kind='linear', fill_value="extrapolate")(imu_time)
gps_northing_resampled = interp1d(gps_time, gps_northing, kind='linear', fill_value="extrapolate")(imu_time)
# Shift, Rotate, and Scale IMU trajectory to match with GPS
xe_processed, xn_processed = process_gps_imu_graphs(xe, xn, gps_easting_resampled, gps_northing_resampled)
#gps_data = calculate_gps_heading(gps_easting,gps_northing)

    
    

# Plotting the magnetometer data for going around in cricles dataset
plt.figure(figsize=(8, 8))
plt.scatter(mag_data_1[0], mag_data_1[1], c='blue', label='Raw Magnetometer Data', alpha=0.5, s = 1.5)
plt.scatter(calb_mag_data_1[0], calb_mag_data_1[1], c='green', label='Magnetometer data after hard iron and soft iron corrections', alpha=0.5, s = 1.5)
plt.title('Magnetometer X-Y plot before and after hard and soft iron calibrations')
plt.xlabel('Magnetic Field X (µT)')
plt.ylabel('Magnetic Field Y (µT)')
plt.axis('equal')  
plt.xlim([-35, 25])  
plt.ylim([-35, 25])  
plt.grid(True)
plt.legend(markerscale=5.0)
plt.tight_layout()

# Plot Raw Yaw (Magnetometer) vs Corrected Yaw (Magnetometer)
plt.figure(figsize=(10, 6))
plt.plot(df_imu['timestamp_imu'], yaw_degrees_raw_mag, label='Raw Yaw (Magnetometer)', color='blue', alpha=0.7)
plt.plot(df_imu['timestamp_imu'], yaw_degrees_corrected_mag, label='Corrected Yaw (Magnetometer)', color='green', alpha=0.7)
plt.title('Raw Yaw (Magnetometer) vs Corrected Yaw (Magnetometer)')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.ylim([-650,650])
plt.grid()
plt.legend()
plt.tight_layout()

# Plot Corrected Yaw (Magnetometer) vs Integrated Yaw (Gyroscope)
plt.figure(figsize=(10, 6))
plt.plot(imu_time, yaw_degrees_corrected_mag, label='Corrected Yaw (Magnetometer)', color='green', alpha=0.7)
plt.plot(imu_time, integrated_yaw_from_gyro, label='Integrated Yaw (Gyroscope)', color='red', alpha=0.7)
plt.title('Corrected Yaw (Magnetometer) vs Integrated Yaw (Gyroscope)')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.ylim([-650,650])
plt.grid()
plt.legend()
plt.tight_layout()

# Plot Yaw Angle Estimations: Low Pass, High Pass, and Complementary Filters
plt.figure(figsize=(12, 6))
plt.plot(imu_time, mag_yaw_filtered, label='Low Pass Filtered Yaw', color='blue', alpha=0.7)
plt.plot(imu_time, gyro_yaw_filtered, label='High Pass Filtered Yaw', color='red', alpha=0.7)
plt.plot(imu_time, complementary_filter_yaw, label='Complementary Filtered Yaw', color='purple', alpha=0.7)
plt.title('Yaw Angle Estimations: Low Pass, High Pass, and Complementary Filters')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.ylim([-650,650])
plt.grid(True)
plt.legend(loc='upper right')  
plt.tight_layout()

# Plot Complementary Filtered Yaw vs Yaw from IMU
plt.figure(figsize=(12, 6))
plt.plot(imu_time, complementary_filter_yaw, label='Complementary Filtered Yaw', color='purple', alpha=0.7)
plt.plot(imu_time, yaw_imu, label='Yaw from IMU', color='red', alpha=0.7)
plt.title('Complementary Filtered Yaw vs Yaw from IMU')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (deg)')
plt.ylim([-650,650])
plt.grid(True)
plt.legend(loc='upper right') 
plt.tight_layout()

# Plot Forward Velocity estimate from the GPS with Velocity estimate from accelerometer before adjustment
plt.figure(figsize=(10, 6))
plt.plot(imu_time, integrated_accel_x, label='Forward Velocity using IMU', color='green', alpha=0.7)
plt.plot(gps_time, velocity_forward_gps, label='Forward Velocity using GPS', color='blue', alpha=0.7)
plt.title('Forward Velocity estimate from the GPS with Velocity estimate from accelerometer before adjustment')
plt.xlabel('Time (s)')
plt.ylabel('Linear Velocity (m/s)')
plt.grid()
plt.legend()
plt.tight_layout()

# Plot Forward Velocity estimate from the GPS with Velocity estimate from accelerometer after adjustment
plt.figure(figsize=(10, 6))
plt.plot(imu_time, integrated_corrected_accel_x, label='Forward Velocity using IMU', color='green', alpha=0.7)
plt.plot(gps_time, velocity_forward_gps, label='Forward Velocity using GPS', color='blue', alpha=0.7)
plt.title('Forward Velocity estimate from the GPS with Velocity estimate from accelerometer after adjustment')
plt.xlabel('Time (s)')
plt.ylabel('Linear Velocity (m/s)')
plt.ylim([-25,25])
plt.grid()
plt.legend()
plt.tight_layout()

# Plot Comparison of raw accel and corrected accel
#plt.figure(figsize=(10, 6))
#plt.plot(imu_time, corrected_accel_x, label='corrected_accel', color='green', alpha=0.7)
#plt.plot(imu_time, df_imu['linear_acceleration_x'], label='raw_accel', color='blue', alpha=0.7)
#plt.title('Comparison of raw accel and corrected accel')
#plt.xlabel('Time (s)')
#plt.ylabel('Linear Accelration (m/s^2)')
#plt.grid()
#plt.legend()
#plt.tight_layout()

# Plot Comparison of ωẊ and ÿ_obs
plt.figure(figsize=(10, 6))
plt.plot(imu_time, omega_x_dot, label='ωẊ', color='green', alpha=0.7)
plt.plot(imu_time, accel_y, label='ÿ_obs', color='blue', alpha=0.7)
plt.title('Comparison of ωẊ and ÿ_obs')
plt.xlabel('Time (s)')
plt.ylabel('Linear Acceleration (m/s^2)')
plt.grid()
plt.legend()
plt.tight_layout()

# Plot Comparison of IMU vs GPS trajectories
plt.figure(figsize=(10, 6))
plt.plot(xe_processed, xn_processed, label='IMU Trajectory', color='green', alpha=0.7)
plt.plot(gps_easting_resampled, gps_northing_resampled, label='GPS Trajectory', color='blue', alpha=0.7)
plt.title('Comparison of IMU vs GPS trajectories')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.grid()
plt.legend()
plt.ticklabel_format(style='scientific', axis='both', scilimits=(0, 0))
plt.tight_layout()

plt.show()
