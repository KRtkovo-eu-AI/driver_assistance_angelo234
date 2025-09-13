-- luacheck: globals ui_message

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local lane_sensor = require('scripts/driver_assistance_angelo234/laneSensor')

local latest_data = nil

-- Apply simple proportional control to keep vehicle centered in lane
local function update(dt, veh, system_params)
  if not veh or not system_params then return end
  if not extra_utils.getPart("lane_assist_system_angelo234") then return end
  local sensor = lane_sensor.sense(veh)
  latest_data = sensor
  if not sensor then return end
  local params = system_params.lane_assist_params or {}
  local kp = params.steer_kp or 1.0
  local lane_width = sensor.lane_width or 0
  if lane_width <= 0 then return end
  local offset = sensor.lateral_offset or 0
  local steer = -kp * offset / (lane_width * 0.5)
  if steer > 1 then steer = 1 elseif steer < -1 then steer = -1 end
  veh:queueLuaCommand(string.format("input.event('steering', %f, 1)", steer))
end

local function getSensorData()
  return latest_data
end

M.update = update
M.getSensorData = getSensorData

return M
