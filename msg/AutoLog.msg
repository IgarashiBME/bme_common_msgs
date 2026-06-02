#
builtin_interfaces/Time stamp
float64 itow
uint8 rtk_status
uint8 movingbase_status

uint32 waypoint_seq
float64 waypoint_start_x
float64 waypoint_start_y
float64 waypoint_end_x
float64 waypoint_end_y
float64 pose_x
float64 pose_y
float64 pose_yaw

float64 tf_waypoint_x
float64 tf_waypoint_y
float64 tf_pose_x
float64 tf_pose_y
float64 cross_track_error

float64 kp
float64 ki_cte
float64 kd
float64 kcte
float64 look_ahead_dist
float64 cte_control_dist
float64 cte_limit
float64 cte_i_limit
float64 cte_i_active
float64 cte_i_decay

float64 p_output
float64 i_output
float64 d_output
float64 cte_output
float64 relative_bearing #[degree]
float64 linear_x  # [meter/sec]
float64 angular_z # [rad/sec]
uint16 ch1_pwm    # [us]
uint16 ch2_pwm    # [us]
uint16 ch3_pwm    # [us] rudder
