-- luacheck: globals vec3 ui_message gearbox_mode_angelo234 input_throttle_angelo234 input_brake_angelo234

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

-- system states: "ready", "braking", "holding"
local system_state = "ready"
local release_brake_confidence_level = 0

local beeper_timer = 0

local latest_point_cloud = {}

-- Returns distance to closest obstacle in front of the vehicle while also
-- storing the last gathered point cloud from the virtual lidar
local function frontObstacleDistance(veh, veh_props, maxDistance)
  local pos = veh:getPosition()
  local dir = veh:getDirectionVector()
  local up = veh:getDirectionVectorUp()
  dir.z = 0
  dir = dir:normalized()
  up = up:normalized()
  local forwardOffset = 1.5
  local origin = vec3(pos.x + dir.x * forwardOffset, pos.y + dir.y * forwardOffset, pos.z + 0.5)

  local scan = virtual_lidar.scan(origin, dir, up, maxDistance, math.rad(30), math.rad(20), 30, 10, 0, veh:getID())
  local groundThreshold = -0.3
  latest_point_cloud = {}
  for _, p in ipairs(scan) do
    local rel = p - origin
    if rel:dot(up) >= groundThreshold then
      latest_point_cloud[#latest_point_cloud + 1] = p
    end
  end

  local right = dir:cross(up)
  local half_width = veh_props.bb:getHalfExtents().x + 0.25

  local best
  for _, p in ipairs(latest_point_cloud) do
    local rel = p - origin
    local forward = rel:dot(dir)
    if forward > 0 and math.abs(rel:dot(right)) <= half_width then
      best = best and math.min(best, forward) or forward
    end
  end

  return best
end

local function getPointCloud()
  return latest_point_cloud
end

local function calculateTimeBeforeBraking(distance, speed, system_params, aeb_params)
  local acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff
  local dist = math.max(0, distance - (aeb_params.braking_distance_leeway or 0))
  local ttc = dist / speed
  local time_to_brake = speed / acc
  return ttc - time_to_brake - aeb_params.braking_time_leeway
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
      ui_message("Obstacle Collision Mitigation Activated", 3)
    end
    system_state = "braking"
  else
    if system_state == "braking" then
      release_brake_confidence_level = release_brake_confidence_level + dt
      if release_brake_confidence_level > 0.25 then
        veh:queueLuaCommand("electrics.values.brakeOverride = nil")
        veh:queueLuaCommand("electrics.values.throttleOverride = nil")
        veh:queueLuaCommand("input.event('brake', 0, 1)")
        veh:queueLuaCommand("input.event('parkingbrake', 0, 2)")
        system_state = "ready"
        release_brake_confidence_level = 0
      end
    end
  end
end

local function update(dt, veh, system_params, aeb_params, beeper_params)
  local veh_props = extra_utils.getVehicleProperties(veh)
  if holdBrakes(veh, veh_props, aeb_params) then return end

  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  if forward_speed <= aeb_params.min_speed then return end

  local distance = frontObstacleDistance(veh, veh_props, aeb_params.sensor_max_distance)
  if not distance then return end

  local time_before_braking = calculateTimeBeforeBraking(distance, forward_speed, system_params, aeb_params)
  performEmergencyBraking(dt, veh, aeb_params, time_before_braking, forward_speed)
  soundBeepers(dt, beeper_params)
end

M.update = update
M.getPointCloud = getPointCloud

return M
