local M = {}

M.gravity = 9.81
M.fwd_friction_coeff = 0.85
M.rev_friction_coeff = 0.8
M.max_steer_radius = 4.3
M.min_steer_radius = 3.1

M.fwd_aeb_params = {
    min_speed = 2.78,
    brake_till_stop_speed = 5,
    apply_parking_brake_speed = 16.7,
    braking_time_leeway = 1.5,
    braking_distance_leeway = 8,

    vehicle_relative_speed_threshold = 1.0,
    obstacle_brake_acc_factor = 0.7,

    vehicle_search_radius = 200,
    
    min_distance_from_car = 1,
    lateral_acc_to_avoid_collision = 0.15,
    
    parking_sensor_rel_height = -0.5,
    num_of_sensors = 9,
    sensors_polled_per_iteration = 1,
    sensor_offset_forward = 0.0,
    sensor_max_distance = 60
}

M.rev_aeb_params = {
    min_speed = 0.75,

    parking_sensor_rel_height = -0.5,
    num_of_sensors = 9,
    sensors_polled_per_iteration = 2,
    sensor_offset_forward = 0.2,
    sensor_max_distance = 15
}

M.beeper_params = {
    fwd_warning_tone_hertz = 7,

    parking_warning_tone_hertz = 20,
    parking_warning_tone_dist_per_hertz = 2.75,
}

M.rev_cam_params = {
    cam_fov = 120,
    cam_down_angle = 35,
    rel_cam_height = 0.25,
    cam_to_wheel_len = -1,

    parking_lines_params = {
        num_of_lines = 10,
        line_width = 0.05,
        line_length = 2.95,
        perp_line_length = 0.2,
      
        parking_line_offset_long = 0.2,
        parking_line_red_len = 0.2,
        parking_line_yellow_len = 0.6,
        parking_line_green_len = 1.0,
        parking_line_total_len = 0.2 + 0.6 + 1.0
    }
}

M.lane_centering_params = {
    steer_kp = 0.35,
    steer_ki = 0.08,
    steer_kd = 0.3,
    heading_kp = 0.6,
    steer_smoothing = 0.06,
    warning_ratio = 0.8,
    steer_limit = 0.15,
    override_threshold = 0.2,
    override_cooldown = 10,
    offset_deadzone = 0.002,
    lookahead_base = 22,
    lookahead_min = 120,
    lookahead_speed_gain = 1.6,
    lookahead_max = 220,
    path_segments = 24,
    path_segments_extend = 24,
    path_segments_cap = 160,
    segment_length_hint = 5,
    curve_subdivisions = 6,
    future_dir_distance = 20,
    curvature_samples = 9,
    segment_alignment_weight = 0.7,
    segment_heading_weight = 0.3,
    segment_probe_distance = 6,
    curvature_feedforward = 0.45,
    assist_weight_gain = 4.0,
    min_active_speed = 1.0,
    lane_offset_smooth = 0.2,
}

return M

