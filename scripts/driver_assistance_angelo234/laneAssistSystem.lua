-- luacheck: globals ui_message

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local lane_sensor = require('scripts/driver_assistance_angelo234/laneSensor')

local latest_data = nil
local current_steer = 0

-- Apply proportional control with smoothing to keep vehicle centered in lane
local function update(dt, veh, system_params)
  if not veh or not system_params then return end
  if not extra_utils.getPart("lane_assist_system_angelo234") then return end
  local sensor = lane_sensor.sense(veh)
  latest_data = sensor
  if not sensor then return end
  local params = system_params.lane_assist_params or {}
  local kp = params.steer_kp or 1.0
  local smoothing = params.steer_smoothing or 0.2
  local lane_width = sensor.lane_width or 0
  if lane_width <= 0 then return end
  local offset = sensor.lateral_offset or 0
  local target = -kp * offset / (lane_width * 0.5)
  if target > 1 then target = 1 elseif target < -1 then target = -1 end
  local alpha = math.min(1, smoothing * dt * 60)
  current_steer = current_steer + (target - current_steer) * alpha
  veh:queueLuaCommand(string.format("input.event('steering', %f, 1)", current_steer))
end

local function getSensorData()
  return latest_data
end

M.update = update
M.getSensorData = getSensorData

return M
