-- luacheck: globals ui_message Engine electrics_values_angelo234

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local _ = require("controlSystems")

local abs = math.abs
local max = math.max
local min = math.min
local atan2 = math.atan2
local clamp = function(value, low, high)
  if value < low then return low end
  if value > high then return high end
  return value
end

local latest_data = {
  status = {
    installed = false,
    enabled = false,
    active = false,
    available = false,
    reason = "init"
  }
}

local warning_played = false
local steering_pid = nil
local steering_smoother = nil
local controller_signature = nil
local override_timer = 0
local activation_handler = nil

local lane_state = {
  prev_wps = nil,
  desired_offset = nil,
  lane_width = nil
}

local function resetControllers()
  if steering_pid then
    steering_pid:reset()
  end
  steering_pid = nil
  steering_smoother = nil
  controller_signature = nil
end

local function ensureControllers(params, steer_limit)
  local kp = params.steer_kp or 0.35
  local ki = params.steer_ki or 0.08
  local kd = params.steer_kd or 0.3
  local smoothing = (params.steer_smoothing or 0.05) * 1000
  local signature = string.format("%.3f|%.3f|%.3f|%.3f|%.3f", kp, ki, kd, steer_limit or 0.15, smoothing)
  if signature ~= controller_signature or not steering_pid or not steering_smoother then
    steering_pid = newPIDStandard(kp, ki, kd, -(steer_limit or 0.15), steer_limit or 0.15)
    steering_smoother = newTemporalSmoothing(smoothing, smoothing)
    controller_signature = signature
  end
end

local function buildLaneOffsets(lane_width, lanes)
  lanes = lanes or 1
  lane_width = lane_width or 0
  if lane_width <= 0 or lanes < 1 then return {0} end

  local offsets = {}

  if lanes == 1 then
    offsets[1] = 0
  elseif lanes % 2 == 1 then
    local half = (lanes - 1) * 0.5
    for i = -half, half do
      offsets[#offsets + 1] = i * lane_width
    end
  else
    local half = lanes * 0.5
    for i = 1, half do
      local offset = (i - 0.5) * lane_width
      offsets[#offsets + 1] = -offset
      offsets[#offsets + 1] = offset
    end
    table.sort(offsets)
  end

  return offsets
end

local function selectOffset(offsets, reference)
  if #offsets == 0 then return 0 end
  if reference == nil then return offsets[1] end

  local best = offsets[1]
  local best_dist = math.huge
  for _, value in ipairs(offsets) do
    local dist = abs(value - reference)
    if dist < best_dist then
      best_dist = dist
      best = value
    end
  end
  return best
end

local function gatherPath(veh_props, initial_wps, desired_offset, params)
  if not initial_wps then return nil end

  local max_segments = params.path_segments or 12
  if max_segments < 1 then max_segments = 1 end
  local lookahead_base = params.lookahead_base or 22
  local lookahead_speed_gain = params.lookahead_speed_gain or 1.6
  local lookahead_max = params.lookahead_max or 80
  local target_length = lookahead_base + veh_props.speed * lookahead_speed_gain
  target_length = clamp(target_length, lookahead_base * 0.5, lookahead_max)

  local centers = {}
  local previous_point = nil
  local previous_dir = nil
  local future_dir = nil
  local curvature_sum = 0
  local curvature_samples = 0
  local total_length = 0
  local segments = 0
  local lane_width = initial_wps.lane_width or 0
  local current_wps = initial_wps
  local preferred_offset = desired_offset

  while current_wps and segments < max_segments and total_length < target_length do
    local seg_dir = extra_utils.toNormXYVec(current_wps.end_wp_pos - current_wps.start_wp_pos)
    if seg_dir:length() < 1e-6 then break end
    local seg_right = vec3(seg_dir.y, -seg_dir.x, 0)

    local start_center = vec3(current_wps.start_wp_pos.x, current_wps.start_wp_pos.y, veh_props.center_pos.z) + seg_right * preferred_offset
    local end_center = vec3(current_wps.end_wp_pos.x, current_wps.end_wp_pos.y, veh_props.center_pos.z) + seg_right * preferred_offset

    if not previous_point then
      previous_point = start_center
      centers[#centers + 1] = start_center
    end

    centers[#centers + 1] = end_center

    local seg_length = (end_center - previous_point):length()
    if seg_length < 1e-3 then
      seg_length = (vec3(current_wps.end_wp_pos) - vec3(current_wps.start_wp_pos)):length()
    end
    total_length = total_length + seg_length
    previous_point = end_center

    if previous_dir then
      local angle = atan2(previous_dir:cross(seg_dir).z, previous_dir:dot(seg_dir))
      if seg_length > 1e-3 then
        curvature_sum = curvature_sum + angle / seg_length
        curvature_samples = curvature_samples + 1
      end
    end

    previous_dir = seg_dir
    future_dir = seg_dir
    lane_width = current_wps.lane_width or lane_width

    segments = segments + 1
    if total_length >= target_length then break end

    local next_query = vec3(current_wps.end_wp_pos.x, current_wps.end_wp_pos.y, veh_props.center_pos.z) + seg_dir * 0.5
    local next_wps = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, next_query, current_wps)
    if not next_wps or (next_wps.start_wp == current_wps.start_wp and next_wps.end_wp == current_wps.end_wp) then
      break
    end

    local offsets = buildLaneOffsets(next_wps.lane_width or lane_width, next_wps.num_of_lanes or 1)
    preferred_offset = selectOffset(offsets, preferred_offset)
    current_wps = next_wps
  end

  if #centers < 2 then return nil end

  local half_width = lane_width * 0.5
  local center_local = {}
  local left_local = {}
  local right_local = {}
  for i = 1, #centers do
    local rel = centers[i] - veh_props.center_pos
    local cx = rel:dot(veh_props.dir_right)
    local cy = rel:dot(veh_props.dir)
    center_local[#center_local + 1] = {x = cx, y = cy}
    left_local[#left_local + 1] = {x = cx - half_width, y = cy}
    right_local[#right_local + 1] = {x = cx + half_width, y = cy}
  end

  return {
    center = center_local,
    left = left_local,
    right = right_local,
    dir = extra_utils.toNormXYVec(initial_wps.end_wp_pos - initial_wps.start_wp_pos),
    future_dir = future_dir,
    curvature = curvature_samples > 0 and curvature_sum / curvature_samples or 0,
    length = total_length,
    offset = preferred_offset,
    last_wps = current_wps,
    lane_width = lane_width
  }
end

local function computeLaneModel(veh_props, params)
  if not veh_props then return nil end

  local wps = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.front_pos, lane_state.prev_wps)
  lane_state.prev_wps = wps
  if not wps then
    lane_state.desired_offset = nil
    return nil
  end

  local lane_width = wps.lane_width or 0
  if lane_width <= 0 then
    lane_state.desired_offset = nil
    return nil
  end

  local offsets = buildLaneOffsets(lane_width, wps.num_of_lanes or 1)
  if #offsets == 0 then
    lane_state.desired_offset = nil
    return nil
  end

  local start_xy = vec3(wps.start_wp_pos.x, wps.start_wp_pos.y, 0)
  local end_xy = vec3(wps.end_wp_pos.x, wps.end_wp_pos.y, 0)
  local veh_xy = vec3(veh_props.center_pos.x, veh_props.center_pos.y, 0)
  local seg_vec = end_xy - start_xy
  local seg_len_sq = seg_vec:squaredLength()
  if seg_len_sq < 1e-6 then
    lane_state.desired_offset = nil
    return nil
  end

  local xnorm = clamp((veh_xy - start_xy):dot(seg_vec) / seg_len_sq, 0, 1)
  local lane_point_xy = start_xy + seg_vec * xnorm
  local lane_dir = extra_utils.toNormXYVec(seg_vec)
  if lane_dir:length() < 1e-6 then
    lane_state.desired_offset = nil
    return nil
  end

  local lane_right = vec3(lane_dir.y, -lane_dir.x, 0)
  local current_offset = (veh_xy - lane_point_xy):dot(lane_right)

  local desired_offset = selectOffset(offsets, lane_state.desired_offset or current_offset)

  local offset_smooth = clamp(params.lane_offset_smooth or 0.0, 0, 1)
  if lane_state.desired_offset then
    desired_offset = lane_state.desired_offset + (desired_offset - lane_state.desired_offset) * (1 - offset_smooth)
  end

  local path = gatherPath(veh_props, wps, desired_offset, params)
  if not path then
    lane_state.desired_offset = desired_offset
    return nil
  end

  lane_state.desired_offset = path.offset or desired_offset
  lane_state.prev_wps = path.last_wps or wps
  lane_state.lane_width = path.lane_width or lane_width

  local lane_dir_norm = path.dir
  local vehicle_dir = extra_utils.toNormXYVec(veh_props.dir)
  local heading_error = vehicle_dir:cross(lane_dir_norm).z
  local half_width = (path.lane_width or lane_width) * 0.5
  local target_offset = path.offset or desired_offset
  local lateral_error = current_offset - target_offset
  local normalized_error = 0
  if half_width > 1e-3 then
    normalized_error = clamp(lateral_error / half_width, -5, 5)
  end

  local future_dir = path.future_dir
  local curvature = path.curvature or 0

  local lane_model = {
    width = path.lane_width or lane_width,
    offset = {
      current = current_offset,
      target = target_offset,
      error = lateral_error,
      normalized = normalized_error
    },
    heading = {
      vehicle = {x = vehicle_dir.x, y = vehicle_dir.y},
      lane = {x = lane_dir_norm.x, y = lane_dir_norm.y},
      error = heading_error
    },
    curvature = curvature,
    path = {
      center = path.center,
      left = path.left,
      right = path.right,
      length = path.length or 0
    },
    speed = veh_props.speed,
    xnorm = xnorm
  }

  if future_dir then
    lane_model.heading.future = {x = future_dir.x, y = future_dir.y}
  end

  return lane_model
end

local function buildAssistInfo(params, lane_model)
  local steer_limit = params.steer_limit or 0.15
  return {
    active = false,
    available = lane_model ~= nil,
    steering = {
      limit = steer_limit,
      target = 0,
      pid = 0,
      feedforward = 0,
      weight = 0,
      final = 0,
      driver = 0,
      error = lane_model and lane_model.offset.normalized or 0,
      headingError = lane_model and lane_model.heading.error or 0,
      curvature = lane_model and lane_model.curvature or 0
    },
    warning = false,
    driverOverride = false
  }
end

local function update(dt, veh, system_params, enabled)
  override_timer = max(override_timer - (dt or 0), 0)

  local params = (system_params and system_params.lane_centering_params) or {}
  local steer_limit = params.steer_limit or 0.15
  local disable_threshold = params.override_threshold or 0.2
  local assist_weight_gain = params.assist_weight_gain or 4.0
  local min_active_speed = params.min_active_speed or 1.0

  local installed = extra_utils.getPart("lane_centering_assist_system_angelo234")
    or extra_utils.getPart("lane_centering_assist_angelo234")

  local status = {
    installed = installed and true or false,
    enabled = installed and enabled or false,
    active = false,
    available = false,
    reason = nil,
    driverOverride = false
  }

  if not installed then
    lane_state.prev_wps = nil
    lane_state.desired_offset = nil
    lane_state.lane_width = nil
    warning_played = false
    latest_data = {status = status}
    resetControllers()
    return
  end

  if not veh then
    latest_data = {status = status}
    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then
    latest_data = {status = status}
    return
  end

  local lane_model = computeLaneModel(veh_props, params)
  if not lane_model then
    status.reason = "no_lane_data"
    warning_played = false
    resetControllers()
  else
    status.available = true
  end

  local assist_info = buildAssistInfo(params, lane_model)
  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  local user_enabled = status.enabled

  local raw_input = 0
  if electrics_values_angelo234 then
    raw_input = electrics_values_angelo234["steering_input"] or 0
  end
  assist_info.steering.driver = raw_input

  if not user_enabled then
    status.reason = "user_disabled"
  elseif override_timer > 0 then
    status.reason = "cooldown"
  elseif lane_model and forward_speed <= min_active_speed then
    status.reason = "low_speed"
  elseif not lane_model then
    status.reason = status.reason or "no_lane_data"
  end

  local assist_ready = user_enabled and lane_model ~= nil and forward_speed > min_active_speed and override_timer <= 0

  if assist_ready and abs(raw_input) > disable_threshold then
    status.driverOverride = true
    status.reason = "driver_override"
    assist_ready = false
    override_timer = params.override_cooldown or 5
    warning_played = false
    resetControllers()
    if activation_handler then
      activation_handler(false, "driver_override")
    end
  end

  if assist_ready then
    ensureControllers(params, steer_limit)
    local half_width = (lane_model.width or 0) * 0.5
    local norm_error = lane_model.offset.normalized or 0
    local heading_error = lane_model.heading.error or 0
    local heading_kp = params.heading_kp or 0.6
    local curvature_ff = params.curvature_feedforward or 0.45

    local combined_error = norm_error + heading_kp * heading_error
    local pid_out = steering_pid:get(combined_error, 0, dt)
    local feedforward = curvature_ff * (lane_model.curvature or 0)
    local raw_target = pid_out + feedforward
    local smoothed = steering_smoother:getUncapped(raw_target, dt)
    local target = clamp(smoothed, -steer_limit, steer_limit)

    local assist_weight = clamp(1 - abs(raw_input) * assist_weight_gain, 0, 1)
    local final = clamp(raw_input + target * assist_weight, -1, 1)

    veh:queueLuaCommand(string.format("input.event('steering', %f, 0)", final))

    assist_info.active = true
    assist_info.steering.target = target
    assist_info.steering.pid = pid_out
    assist_info.steering.feedforward = feedforward
    assist_info.steering.weight = assist_weight
    assist_info.steering.final = final
    assist_info.steering.error = norm_error
    assist_info.steering.headingError = heading_error
    assist_info.steering.curvature = lane_model.curvature or 0

    status.active = true
    status.reason = nil
  else
    resetControllers()
  end

  if lane_model then
    local half_width = (lane_model.width or 0) * 0.5
    if half_width > 0 then
      local warn_zone = (params.warning_ratio or 0.8) * half_width
      local abs_offset = abs(lane_model.offset.error or 0)
      if abs_offset > warn_zone and abs_offset <= half_width then
        assist_info.warning = true
        if not warning_played then
          Engine.Audio.playOnce('AudioGui', 'art/sound/proximity_tone_50ms_moderate.wav')
          warning_played = true
        end
      else
        assist_info.warning = false
        warning_played = false
      end
    else
      assist_info.warning = false
      warning_played = false
    end
  else
    assist_info.warning = false
    warning_played = false
  end

  assist_info.driverOverride = status.driverOverride

  latest_data = {
    status = status,
    lane = lane_model,
    assist = assist_info
  }
end

local function getLaneData()
  return latest_data
end

local function setActivationCallback(cb)
  activation_handler = cb
end

M.update = update
M.getLaneData = getLaneData
M.setActivationCallback = setActivationCallback

return M
