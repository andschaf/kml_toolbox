swe_app_standard = {"v":"1",
"colorlogtostderr":"false",
"alsologtostderr":"false",
"swe_start_console":"false",
"swe_visualization":"false",

"swe_imu_rate":"200",
"swe_cam_rate":"20",
"swe_estimator_rate":"20",
"swe_write_csv":"true",

"swe_add_date_suffix_to_csv_filename":"false",
"swe_write_csv_covariance_diagonals":"false",
"swe_write_csv_covariance_full":"true",

"swe_datasource":"1",
"swe_sigma_position_initial":"1.0e2",
"swe_sigma_velocity_initial":"20.0",

"swe_vinode_csv_path_filename":"./swe_positions.csv",
"swe_vinode_covariance_csv_path_filename":"./swe_covariances.csv",
}

swe_app_optional = {
"swe_use_magnetometer":"false",
"vis_check_landmark_constrained":"false",
"swe_use_gps_llh":"true",
"swe_simulate_gps_outage":"false",
"swe_gps_outage_length_s":"100.0",
"swe_min_observer":"10",
"swe_min_closest":"0.1",
"swe_max_closest":"150.0",
"swe_use_inverse_depth_landmarks":"false",
"swe_use_ground_plane_projection":"false",
}

swe_app_Skate = {
"swe_calibration_yaml":"/home/andschaf/1_skate_rosbags/calibration6/ncamera.yaml",
"swe_imu_topic":"/imu0",
"swe_gps_llh_topic":"/gps/fix",
"swe_rosbag_start_s":"1439369140.882964",
"swe_rosbag_end_s":"1439369459.882964",
}

swe_app_Techpod = {
"swe_calibration_yaml":"/home/andschaf/2_techpod_rosbags/calibration/camera_fixed_wing.yaml",
"swe_rosbag_start_s":"1262304339.73536",
"swe_rosbag_end_s":"1262304650.1429",
"swe_imu_topic":"/imu0",
"swe_gps_llh_topic":"/mavros/gps/fix",
}

swe_dataset_evaluation = {
"filename_in_estimated_trajectory":"./swe_positions.csv",
"filename_covariances":"./swe_covariances.csv",
"filename_out_prefix":"./swe_",
"filename_in_groundtruth_trajectory":"./gpsfix_positions.csv",
"filename_gpsfix_covariances":"./gpsfix_covariances.csv",
}


