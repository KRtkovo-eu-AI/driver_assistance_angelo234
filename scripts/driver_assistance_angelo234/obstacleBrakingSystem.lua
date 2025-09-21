-- luacheck: globals vec3 ui_message gearbox_mode_angelo234 input_throttle_angelo234 input_brake_angelo234
-- luacheck: globals Engine

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')
local logger = require('scripts/driver_assistance_angelo234/logger')

local OBSTACLE_LIDAR_MAX_RAYS = 150

-- system states: "ready", "braking", "holding"
local system_state = "ready"
local aeb_clear_timer = 0

local beeper_timer = 0

local latest_point_cloud = {}

local function enableHazardLights(veh)
  -- Force hazard lights on and leave them for the driver to switch off manually.
  veh:queueLuaCommand("electrics.set_warn_signal(true)")
end

local function enableABS(veh)
  -- Ensure ABS is active if the vehicle supports it.
  veh:queueLuaCommand("if electrics.setABSMode then electrics.setABSMode(1) end")
end

-- speed is in m/s and is used to relax slope filtering at lower speeds
local function frontObstacleDistance(veh, veh_props, aeb_params, speed, front_sensors, use_lidar)
  local maxDistance = aeb_params.sensor_max_distance
  local pos = veh:getPosition()
  local dir = veh:getDirectionVector()
  dir.z = 0
  dir = dir:normalized()
  -- use world up so pitch/roll of the vehicle doesn't tilt the scan
  local up = vec3(0, 0, 1)
  local forwardOffset = 1.5
  local origin = vec3(pos.x + dir.x * forwardOffset, pos.y + dir.y * forwardOffset, pos.z + 0.5)
  -- ignore points below groundThreshold or above the vehicle roof to avoid
  -- triggering on walkways or bridges that are safe to pass under
  local groundThreshold = -0.3
  local top_z = veh_props.bb:getCenter().z + veh_props.bb:getHalfExtents().z
  local roofClearance = top_z - origin.z + 0.25
  local right = dir:cross(up)
  local half_width = veh_props.bb:getHalfExtents().x + 0.25

  -- allow some vertical rise before considering slope to handle gentle inclines
  local base_allowance = aeb_params.slope_height_allowance or 0.25
  local extra_allowance = 0
  local speed_kmh = speed * 3.6
  if speed_kmh < 20 then
    local low_allowance = aeb_params.low_speed_slope_allowance or 0.25
    extra_allowance = low_allowance * (1 - speed_kmh / 20)
  end
  local height_allowance = base_allowance + extra_allowance

  local overhead_dist = aeb_params.overhead_ignore_distance or 10
  local overhead_margin = aeb_params.overhead_height_margin or 0.1

  local scan = {}
  if use_lidar then
    scan = virtual_lidar.scan(
      origin,
      dir,
      up,
      maxDistance,
      math.rad(30),
      math.rad(20),
      30,
      10,
      0,
      veh:getID(),
      {maxRays = OBSTACLE_LIDAR_MAX_RAYS}
    )
    -- add points for nearby vehicles similar to sensor approach
    for i = 0, be:getObjectCount() - 1 do
      local other = be:getObject(i)
      if other:getID() ~= veh:getID() and other:getJBeamFilename() ~= "unicycle" then
        local props = extra_utils.getVehicleProperties(other)
        if not extra_utils.isVehicleGhost(other, props) then
          local rel = props.center_pos - origin
          local dist = rel:length()
          if dist < maxDistance and rel:dot(dir) > 0 then
            local rayDir = rel / dist
            local hit = castRay(origin, origin + rayDir * dist, true, true)
            if hit and hit.obj and hit.obj.getID and hit.obj:getID() == props.id then
              scan[#scan + 1] = hit.pt
            end
          end
        end
      end
    end
  end

  latest_point_cloud = {}
  local best
  local side_best
  local lidar_best

  if use_lidar and #scan > 0 then
    local bin_size = aeb_params.lidar_forward_bin_size or 1.0
    local min_clearance = aeb_params.lidar_min_clearance or 0.35
    local ground_variation_limit = aeb_params.lidar_ground_variation_limit or 0.25
    local ground_fit_margin = aeb_params.lidar_ground_fit_margin or 0.15
    local min_points_per_bin = aeb_params.lidar_min_points_per_bin or 1
    local max_fit_slope = math.tan(math.rad(aeb_params.lidar_max_fit_slope_deg or 18))
    local occupancy_ratio_threshold = aeb_params.lidar_block_ratio_threshold or 0.45
    local tall_ratio_threshold = aeb_params.lidar_tall_ratio_threshold or 0.2
    local sustained_bins_required = math.max(1, aeb_params.lidar_sustained_bins or 2)
    local block_min_points = aeb_params.lidar_block_min_points or 2

    local bins = {}
    local bin_order = {}

    for idx, p in ipairs(scan) do
      local rel = p - origin
      local forward = rel:dot(dir)
      local lateral = math.abs(rel:dot(right))
      local height = rel:dot(up)

      if forward > 0 then
        if height >= groundThreshold and height <= roofClearance then
          if not (forward > overhead_dist and height >= roofClearance - overhead_margin) then
            if lateral <= half_width then
              local bin_idx = math.floor(forward / bin_size)
              local bin = bins[bin_idx]
              if not bin then
                bin = {
                  minHeight = height,
                  maxHeight = height,
                  minForward = forward,
                  forwardSum = forward,
                  count = 1,
                  indices = {idx}
                }
                bins[bin_idx] = bin
                bin_order[#bin_order + 1] = bin_idx
              else
                if height < bin.minHeight then bin.minHeight = height end
                if height > bin.maxHeight then bin.maxHeight = height end
                if forward < bin.minForward then bin.minForward = forward end
                bin.forwardSum = bin.forwardSum + forward
                bin.count = bin.count + 1
                bin.indices[#bin.indices + 1] = idx
              end
            end
          end
        end
      end

      if forward > 0 and forward <= 2 and height >= groundThreshold and height <= roofClearance then
        local clearance = lateral - half_width
        if clearance >= 0 then
          side_best = side_best and math.min(side_best, clearance) or clearance
        end
      end
    end

    local ground_samples = {}
    for _, bin_idx in ipairs(bin_order) do
      local bin = bins[bin_idx]
      if bin and bin.count >= min_points_per_bin then
        local span = bin.maxHeight - bin.minHeight
        if span <= ground_variation_limit then
          local center_forward = bin.forwardSum / bin.count
          ground_samples[#ground_samples + 1] = {forward = center_forward, height = bin.minHeight}
        end
      end
    end

    local slope
    local intercept
    if #ground_samples >= 2 then
      local sum_f, sum_h, sum_fh, sum_ff = 0, 0, 0, 0
      for _, sample in ipairs(ground_samples) do
        sum_f = sum_f + sample.forward
        sum_h = sum_h + sample.height
        sum_fh = sum_fh + sample.forward * sample.height
        sum_ff = sum_ff + sample.forward * sample.forward
      end
      local n = #ground_samples
      local denom = n * sum_ff - sum_f * sum_f
      if math.abs(denom) > 1e-6 then
        slope = (n * sum_fh - sum_f * sum_h) / denom
        intercept = (sum_h - slope * sum_f) / n
      end
    end

    if not slope then
      local veh_dir = veh_props.dir
      local horiz = math.sqrt(veh_dir.x * veh_dir.x + veh_dir.y * veh_dir.y)
      if horiz < 1e-3 then horiz = 1e-3 end
      slope = veh_dir.z / horiz
      intercept = groundThreshold
    end

    if not intercept then
      if #ground_samples >= 1 then
        local first = ground_samples[1]
        intercept = first.height - slope * first.forward
      else
        intercept = groundThreshold
      end
    end

    if slope > max_fit_slope then slope = max_fit_slope end
    if slope < -max_fit_slope then slope = -max_fit_slope end

    local added_points = {}
    local log_clearance
    local log_ratio
    local log_tall_ratio
    local log_reason
    local consecutive_block_bins = 0
    for _, bin_idx in ipairs(bin_order) do
      local bin = bins[bin_idx]
      if bin and bin.count >= min_points_per_bin then
        local forward_mean = bin.forwardSum / bin.count
        local ground_est = intercept + slope * forward_mean + ground_fit_margin
        if ground_est > bin.minHeight then
          ground_est = bin.minHeight
        end
        local clearance = bin.maxHeight - ground_est
        local above_allow_count = 0
        local tall_count = 0
        for _, point_idx in ipairs(bin.indices) do
          local point = scan[point_idx]
          if point then
            local rel = point - origin
            local height = rel:dot(up)
            if height >= ground_est + height_allowance then
              above_allow_count = above_allow_count + 1
              if height >= ground_est + min_clearance then
                tall_count = tall_count + 1
              end
            end
          end
        end

        local ratio = above_allow_count / bin.count
        local tall_ratio = tall_count / bin.count
        local qualifies_height = clearance >= min_clearance or tall_ratio >= tall_ratio_threshold
        local qualifies_block = ratio >= occupancy_ratio_threshold and bin.count >= block_min_points
        local qualifies_bin = false

        if qualifies_height then
          consecutive_block_bins = sustained_bins_required
          qualifies_bin = true
        elseif qualifies_block then
          consecutive_block_bins = consecutive_block_bins + 1
          if consecutive_block_bins >= sustained_bins_required then
            qualifies_bin = true
          end
        else
          consecutive_block_bins = 0
        end

        if qualifies_bin then
          local forward_hit = bin.minForward
          if not lidar_best or forward_hit < lidar_best then
            log_clearance = clearance
            log_ratio = ratio
            log_tall_ratio = tall_ratio
            log_reason = qualifies_height and "height" or "profile"
          end
          best = best and math.min(best, forward_hit) or forward_hit
          lidar_best = lidar_best and math.min(lidar_best, forward_hit) or forward_hit
          local point_height_threshold = ground_est + height_allowance
          if qualifies_height and point_height_threshold > ground_est + min_clearance * 0.5 then
            point_height_threshold = ground_est + min_clearance * 0.5
          end
          for _, point_idx in ipairs(bin.indices) do
            if not added_points[point_idx] then
              local point = scan[point_idx]
              if point then
                local rel = point - origin
                local height = rel:dot(up)
                if height >= point_height_threshold then
                  latest_point_cloud[#latest_point_cloud + 1] = point
                  added_points[point_idx] = true
                end
              end
            end
          end
        end
      end
    end
  else
    for _, p in ipairs(scan) do
      local rel = p - origin
      local forward = rel:dot(dir)
      local lateral = math.abs(rel:dot(right))
      local height = rel:dot(up)

      if forward > 0 and forward <= 2 and height >= groundThreshold and height <= roofClearance then
        local clearance = lateral - half_width
        if clearance >= 0 then
          side_best = side_best and math.min(side_best, clearance) or clearance
        end
      end
    end
  end

  -- incorporate sensor measurements without modifying the scan
  if front_sensors then
    local sensor_best
    local front_static = front_sensors[1]
    if front_static and front_static > 0 and front_static < 9999 then
      sensor_best = front_static
    end
    local vehs = front_sensors[2]
    if vehs then
      for _, data in ipairs(vehs) do
        if data.distance and data.distance > 0 and data.other_veh_props then
          local rel_speed = (data.other_veh_props.velocity - veh_props.velocity):length()
          if rel_speed < (aeb_params.vehicle_relative_speed_threshold or 1) then
            sensor_best = sensor_best and math.min(sensor_best, data.distance) or data.distance
          end
        end
      end
    end
    if sensor_best then
      best = best and math.min(best, sensor_best) or sensor_best
    end
  end

  if use_lidar and lidar_best then
    if log_clearance and log_ratio and log_tall_ratio and log_reason then
      logger.log(
        'I',
        'lidar',
        string.format(
          'LiDAR detected obstacle at %.1f m (%s clr=%.2f occ=%.2f tall=%.2f)',
          lidar_best,
          log_reason,
          log_clearance,
          log_ratio,
          log_tall_ratio
        )
      )
    else
      logger.log('I', 'lidar', string.format('LiDAR detected obstacle at %.1f', lidar_best))
    end
  end

  return best, side_best
end

local function getPointCloud()
  return latest_point_cloud
end

local function calculateTimeBeforeBraking(distance, speed, system_params, aeb_params)
  local base_acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff
  local acc = base_acc * (aeb_params.obstacle_brake_acc_factor or 0.7)
  local speed_kmh = speed * 3.6
  local extra_distance_leeway = speed_kmh > 95 and 10 or 0
  local speed_leeway = (aeb_params.braking_speed_leeway_factor or 0.2) * speed
  local dist = math.max(0, distance - (aeb_params.braking_distance_leeway or 0) - speed_leeway - extra_distance_leeway)
  local ttc = dist / speed
  local time_to_brake = speed / acc
  -- add extra safety margin at higher speeds to begin braking earlier
  local extra_leeway = 0
  if speed_kmh > 60 then
    local clamped = math.min(speed_kmh, 150)
    local exponent = (clamped - 60) / 20
    extra_leeway = (aeb_params.high_speed_braking_time_leeway or 0.5) * (math.exp(exponent) - 1)
    if speed_kmh > 85 then
      extra_leeway = (extra_leeway
        + (speed_kmh - 85) / 10) * (aeb_params.very_high_speed_braking_time_leeway or 1.0)
    end
    extra_leeway = math.min(extra_leeway, aeb_params.max_high_speed_braking_leeway or 20)
  end
  return ttc - time_to_brake - aeb_params.braking_time_leeway - extra_leeway
end

local function soundBeepers(dt, beeper_params)
  if system_state == "braking" then
    beeper_timer = beeper_timer + dt
    if beeper_timer >= 1.0 / beeper_params.fwd_warning_tone_hertz then
      Engine.Audio.playOnce('AudioGui', 'art/sound/proximity_tone_50ms_loud.wav')
      beeper_timer = 0
    end
  else
    beeper_timer = 0
  end
end

local function holdBrakes(veh, veh_props, aeb_params)
  if veh_props.speed <= aeb_params.min_speed then
    if system_state == "braking" then
      if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
        veh:queueLuaCommand("electrics.values.brakeOverride = 1")
        veh:queueLuaCommand("input.event('brake', 1, 1)")
      else
        veh:queueLuaCommand("electrics.values.brakeOverride = nil")
        veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")
      end
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      system_state = "holding"
    end
  end

  if system_state == "holding" then
    if input_throttle_angelo234 > 0.5 or input_brake_angelo234 > 0.3 then
      veh:queueLuaCommand("electrics.values.brakeOverride = nil")
      veh:queueLuaCommand("electrics.values.throttleOverride = nil")
      veh:queueLuaCommand("input.event('brake', 0, 1)")
      veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
      system_state = "ready"
    end
  end

  return system_state == "holding"
end

local function performEmergencyBraking(dt, veh, aeb_params, time_before_braking, speed)
  if system_state == "braking" and speed < aeb_params.brake_till_stop_speed then
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")
    veh:queueLuaCommand("input.event('throttle', 0, 1)")
    veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    veh:queueLuaCommand("input.event('brake', 1, 1)")
    veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
    return
  end

  if time_before_braking <= 0 then
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")
    veh:queueLuaCommand("input.event('throttle', 0, 1)")
    veh:queueLuaCommand("electrics.values.brakeOverride = 1")
    veh:queueLuaCommand("input.event('brake', 1, 1)")
    if speed > aeb_params.apply_parking_brake_speed then
      veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")
    else
      veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
    end
    if system_state ~= "braking" then
      ui_message("Obstacle Collision AEB Activated", 3)
      enableHazardLights(veh)
      enableABS(veh)
    end
    system_state = "braking"
    aeb_clear_timer = 0
  else
    if system_state == "braking" then
      aeb_clear_timer = aeb_clear_timer + dt
      if aeb_clear_timer > (aeb_params.no_obstacle_brake_release_time or 2) then
        veh:queueLuaCommand("electrics.values.brakeOverride = nil")
        veh:queueLuaCommand("electrics.values.throttleOverride = nil")
        veh:queueLuaCommand("input.event('brake', 0, 1)")
        veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
        system_state = "ready"
        aeb_clear_timer = 0
      end
    end
  end
end

local function update(dt, veh, system_params, aeb_params, beeper_params, front_sensors, rear_sensors)
  local veh_props = extra_utils.getVehicleProperties(veh)
  if holdBrakes(veh, veh_props, aeb_params) then return end

  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  if forward_speed <= aeb_params.min_speed then return end

  local use_lidar = extra_utils.getPart("lidar_angelo234")

  local distance, side_clearance = frontObstacleDistance(veh, veh_props, aeb_params, forward_speed, front_sensors, use_lidar)

  local side_threshold = aeb_params.side_clearance_threshold or 0.3
  if side_clearance and side_clearance < side_threshold then
    distance = distance and math.min(distance, side_clearance) or side_clearance
  end

  if not distance then
    if system_state == "braking" then
      performEmergencyBraking(dt, veh, aeb_params, math.huge, forward_speed)
      soundBeepers(dt, beeper_params)
    end
    return
  end

  local time_before_braking = calculateTimeBeforeBraking(distance, forward_speed, system_params, aeb_params)
  performEmergencyBraking(dt, veh, aeb_params, time_before_braking, forward_speed)
  soundBeepers(dt, beeper_params)
end

M.update = update
M.getPointCloud = getPointCloud

return M
