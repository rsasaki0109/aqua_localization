std_msgs/Header header

string estimator_name
string backend
bool initialized
uint64 update_count
float64 last_prediction_dt
float64 position_covariance_trace
float64 orientation_covariance_trace
string status

# Current accelerometer bias estimate in body frame [m/s^2].
float64[3] accel_bias
# Current gyroscope bias estimate in body frame [rad/s].
float64[3] gyro_bias

# True when the imu.use_ahrs_gyro_bias_z hook has been enabled.
bool ahrs_gyro_bias_z_enabled
# True when at least one bz observation has been applied since startup or reset.
bool ahrs_gyro_bias_z_active
# Last bz observation value supplied to the UKF [rad/s]. NaN before first observation.
float64 ahrs_gyro_bias_z_last_observed
