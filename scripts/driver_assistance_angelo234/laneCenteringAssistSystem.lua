-- luacheck: globals ui_message Engine electrics_values_angelo234

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local lane_sensor = require('scripts/driver_assistance_angelo234/laneSensor')
local _ = require("controlSystems")

local latest_data = nil
local warning_played = false

local steering_pid = newPIDStandard(0.1, 2, 1.25, -0.3, 0.3)
local steering_smooth = newTemporalSmoothing(200, 200)

-- Apply PID control with smoothing to keep vehicle centered and aligned with lane
local function update(dt, veh, system_params)
  if not veh or not system_params then return end
  if not extra_utils.getPart("lane_centering_assist_system_angelo234") then return end
  local sensor = lane_sensor.sense(veh)
  latest_data = sensor
  if not sensor then return end
  local params = system_params.lane_centering_params or {}
  local heading_kp = params.heading_kp or 0.5
  local warn_ratio = params.warning_ratio or 0.8
  local steer_limit = params.steer_limit or 0.3

  local lane_width = sensor.lane_width or 0
  if lane_width <= 0 then return end
  local offset = sensor.lateral_offset or 0

  local half_width = lane_width * 0.5
  local norm_offset = offset / half_width
  local abs_off = math.abs(offset)
  local warn_zone = warn_ratio * half_width

  -- Compute heading error using look-ahead direction when available
  local veh_props = extra_utils.getVehicleProperties(veh)
  local desired_dir = sensor.future_dir or sensor.road_dir or veh_props.dir
  local heading_error = veh_props.dir:cross(desired_dir).z

  local error = norm_offset + heading_kp * heading_error
  local pid_out = steering_pid:get(error, 0, dt)
  local target = steering_smooth:getUncapped(pid_out, dt)
  if target > steer_limit then target = steer_limit elseif target < -steer_limit then target = -steer_limit end

  if abs_off > warn_zone and abs_off <= half_width then
    if not warning_played then
      Engine.Audio.playOnce('AudioGui','art/sound/proximity_tone_50ms_moderate.wav')
      warning_played = true
    end
  else
    warning_played = false
  end

  local driver_input = 0
  if electrics_values_angelo234 then
    driver_input = electrics_values_angelo234["steering_input"] or 0
  end
  local assist_weight = math.max(0, 1 - math.abs(driver_input))
  local final = driver_input + target * assist_weight
  if final > 1 then final = 1 elseif final < -1 then final = -1 end
  veh:queueLuaCommand(string.format("input.event('steering', %f, 0)", final))
end

local function getLaneData()
  return latest_data
end

M.update = update
M.getLaneData = getLaneData

return M
