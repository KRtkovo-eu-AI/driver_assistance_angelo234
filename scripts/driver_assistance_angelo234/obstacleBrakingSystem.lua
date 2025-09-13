local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

-- system states: "ready", "braking", "holding"
local system_state = "ready"
local beeper_timer = 0
local release_brake_confidence_level = 0

-- Casts multiple rays ahead and returns the nearest hit distance
local function castRays(baseOrigin, dir, maxDistance, widthDir)
  local heights = {0.3, 0.8, 1.3}
  local laterals = widthDir and {0, 0.4, -0.4} or {0}
  local best

  for _, lateral in ipairs(laterals) do
    for _, h in ipairs(heights) do
      local origin = vec3(baseOrigin.x, baseOrigin.y, baseOrigin.z + h)
      if widthDir then
        origin.x = origin.x + widthDir.x * lateral
        origin.y = origin.y + widthDir.y * lateral
        origin.z = origin.z + widthDir.z * lateral
      end

      local dist = castRayStatic(origin, dir, maxDistance)
      if dist and dist < maxDistance then
        best = best and math.min(best, dist) or dist
      end
    end
  end

  return best
end

local function frontObstacleDistance(veh, maxDistance)
  local pos = veh:getPosition()
  local dir = veh:getDirectionVector()
  dir.z = 0
  dir = dir:normalized()
  local sideDir = vec3(-dir.y, dir.x, 0)
  local forwardOffset = 1.5
  local baseOrigin = vec3(pos.x + dir.x * forwardOffset, pos.y + dir.y * forwardOffset, pos.z)

  local dist = castRays(baseOrigin, dir, maxDistance, sideDir)
  return dist
end

local function calculateTimeBeforeBraking(distance, speed, system_params, aeb_params)
  local acc = math.min(10, system_params.gravity) * system_params.fwd_friction_coeff
  local ttc = distance / speed
  local time_to_brake = speed / (2 * acc)
  return ttc - time_to_brake - aeb_params.braking_time_leeway
end

local function holdBrakes(veh, veh_props, aeb_params)
  if veh_props.speed <= aeb_params.min_speed then
    if system_state == "braking" then
      if gearbox_mode_angelo234.previousGearboxBehavior == "realistic" then
        veh:queueLuaCommand("electrics.values.brakeOverride = 1")
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
    return
  end

  if time_before_braking <= 0 then
    veh:queueLuaCommand("electrics.values.throttleOverride = 0")
    veh:queueLuaCommand("input.event('throttle', 0, 1)")
    veh:queueLuaCommand("electrics.values.brakeOverride = 1")
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
        system_state = "ready"
        release_brake_confidence_level = 0
      end
    end
  end
end

local function update(dt, veh, system_params, aeb_params)
  local veh_props = extra_utils.getVehicleProperties(veh)
  if holdBrakes(veh, veh_props, aeb_params) then return end

  local distance = frontObstacleDistance(veh, aeb_params.sensor_max_distance)
  if not distance or veh_props.speed <= aeb_params.min_speed then return end

  local time_before_braking = calculateTimeBeforeBraking(distance, veh_props.speed, system_params, aeb_params)
  performEmergencyBraking(dt, veh, aeb_params, time_before_braking, veh_props.speed)
end

M.update = update

return M
