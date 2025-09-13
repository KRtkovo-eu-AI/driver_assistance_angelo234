-- luacheck: globals ui_message Engine electrics_values_angelo234

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local lane_sensor = require('scripts/driver_assistance_angelo234/laneSensor')

local latest_data = nil
local current_steer = 0
local warning_played = false

-- Apply proportional control with smoothing to keep vehicle centered and aligned with lane
local function update(dt, veh, system_params)
  if not veh or not system_params then return end
  if not extra_utils.getPart("lane_assist_system_angelo234") then return end
  local sensor = lane_sensor.sense(veh)
  latest_data = sensor
  if not sensor then return end
  local params = system_params.lane_assist_params or {}
  local kp = params.steer_kp or 0.5
  local heading_kp = params.heading_kp or 1.0
  local smoothing = params.steer_smoothing or 0.2
  local warn_ratio = params.warning_ratio or 0.8
  local steer_limit = params.steer_limit or 0.5

  local lane_width = sensor.lane_width or 0
  if lane_width <= 0 then return end
  local offset = sensor.lateral_offset or 0

  local half_width = lane_width * 0.5
  local abs_off = math.abs(offset)
  local warn_zone = warn_ratio * half_width

  -- Compute heading error between vehicle and road direction
  local veh_props = extra_utils.getVehicleProperties(veh)
  local road_dir = sensor.road_dir or veh_props.dir
  local heading_error = veh_props.dir:cross(road_dir).z

  local target = 0
  if abs_off > half_width then
    local outside = abs_off - half_width
    target = -(kp * outside / half_width * (offset > 0 and 1 or -1) + heading_kp * heading_error)
    if target > steer_limit then target = steer_limit elseif target < -steer_limit then target = -steer_limit end
  else
    target = -heading_kp * heading_error
  end

  if abs_off > warn_zone and abs_off <= half_width then
    if not warning_played then
      Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_moderate.wav')
      warning_played = true
    end
  else
    warning_played = false
  end

  local alpha = math.min(1, smoothing * dt * 60)
  current_steer = current_steer + (target - current_steer) * alpha

  local driver_input = 0
  if electrics_values_angelo234 then
    driver_input = electrics_values_angelo234["steering_input"] or 0
  end
  local final = driver_input + current_steer
  if final > 1 then final = 1 elseif final < -1 then final = -1 end
  veh:queueLuaCommand(string.format("input.event('steering', %f, 1)", final))
end

local function getSensorData()
  return latest_data
end

M.update = update
M.getSensorData = getSensorData

return M
