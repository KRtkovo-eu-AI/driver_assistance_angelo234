-- luacheck: globals vec3 ui_message gearbox_mode_angelo234 input_throttle_angelo234 input_brake_angelo234
-- luacheck: globals Engine

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

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
local function frontObstacleDistance(veh, veh_props, aeb_params, speed, front_sensors, rear_sensors)
  local maxDistance = aeb_params.sensor_max_distance
  local pos = veh:getPosition()
  local dir = veh:getDirectionVector()
  dir.z = 0
  dir = dir:normalized()
  -- use world up so pitch/roll of the vehicle doesn't tilt the scan
  local up = vec3(0, 0, 1)
  local forwardOffset = 1.5
  local origin = vec3(pos.x + dir.x * forwardOffset, pos.y + dir.y * forwardOffset, pos.z + 0.5)

  local scan =
    virtual_lidar.scan(origin, dir, up, maxDistance, math.rad(30), math.rad(20), 30, 10, 0, veh:getID())

  -- augment scan with sensor data
  if front_sensors then
    local front_static = front_sensors[1]
    if front_static and front_static < 9999 then
      scan[#scan + 1] = origin + dir * front_static
    end
    local vehs = front_sensors[2]
    if vehs then
      for _, data in ipairs(vehs) do
        if data.other_veh_props and data.other_veh_props.center_pos then
          scan[#scan + 1] = data.other_veh_props.center_pos
        end
      end
    end
  end

  if rear_sensors then
    local rear_vehicle = rear_sensors[1]
    local rear_dist = rear_sensors[2]
    if rear_dist and rear_dist < 9999 then
      scan[#scan + 1] = origin - dir * rear_dist
    end
    if rear_vehicle then
      local props = extra_utils.getVehicleProperties(rear_vehicle)
      if props and props.center_pos then
        scan[#scan + 1] = props.center_pos
      end
    end
  end

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

  latest_point_cloud = {}
  local best
  local side_best
  for _, p in ipairs(scan) do
    local rel = p - origin
    local forward = rel:dot(dir)
    local lateral = math.abs(rel:dot(right))
    local height = rel:dot(up)

    -- check forward obstacles
      if forward > 0 then
        if height >= groundThreshold and height <= roofClearance then
          if not (forward > overhead_dist and height >= roofClearance - overhead_margin) then
            local slope_height = height - height_allowance
            if slope_height > 0 and lateral <= half_width then
              latest_point_cloud[#latest_point_cloud + 1] = p
              best = best and math.min(best, forward) or forward
            end
          end
        end
      end

    -- check potential side collisions near the vehicle
    if forward > 0 and forward <= 2 and height >= groundThreshold and height <= roofClearance then
      local clearance = lateral - half_width
      if clearance >= 0 then
        side_best = side_best and math.min(side_best, clearance) or clearance
      end
    end
  end

  return best, side_best
end

local function getPointCloud()
  return latest_point_cloud
end

local function calculateTimeBeforeBraking(distance, speed, system_params, aeb_params)
  local acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff
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
    veh:queueLuaCommand("electrics.values.brakeOverride = nil")
    veh:queueLuaCommand("input.event('brake', 1, 1)")
    veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
    return
  end

  if time_before_braking <= 0 then
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")
    veh:queueLuaCommand("input.event('throttle', 0, 1)")
    veh:queueLuaCommand("electrics.values.brakeOverride = nil")
    veh:queueLuaCommand("input.event('brake', 1, 1)")
    if speed > aeb_params.apply_parking_brake_speed then
      veh:queueLuaCommand("input.event('parkingbrake', 1, 2)")
    else
      veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
    end
    if system_state ~= "braking" then
      ui_message("Obstacle Collision Mitigation Activated", 3)
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

  local distance, side_clearance = frontObstacleDistance(veh, veh_props, aeb_params, forward_speed, front_sensors, rear_sensors)

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
