local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local min_speed = 16.67 -- ~60 km/h
local prev_wps = nil
local override_active = false

local function reset(veh)
  if override_active then
    veh:queueLuaCommand("electrics.values.steeringOverride = nil")
    override_active = false
  end
  prev_wps = nil
end

local function update(dt, veh)
  local veh_props = extra_utils.getVehicleProperties(veh)

  local signal_left = electrics_values_angelo234 and (electrics_values_angelo234["signal_left_input"] or electrics_values_angelo234["signal_left"] or electrics_values_angelo234["signal_L"]) or 0
  local signal_right = electrics_values_angelo234 and (electrics_values_angelo234["signal_right_input"] or electrics_values_angelo234["signal_right"] or electrics_values_angelo234["signal_R"]) or 0

  if veh_props.speed < min_speed or (signal_left and signal_left > 0) or (signal_right and signal_right > 0) then
    reset(veh)
    return
  end

  local wps_props = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.center_pos, prev_wps)
  prev_wps = wps_props

  if not wps_props or not wps_props.lane_width or wps_props.lane_width <= 0 then
    reset(veh)
    return
  end

  local lane_width = wps_props.lane_width
  local lat = wps_props.lat_dist_from_wp or 0

  local lane_index = 0
  local lane_center = 0
  if lat >= 0 then
    lane_index = math.floor(lat / lane_width) + 1
    lane_center = lane_width * (lane_index - 0.5)
  else
    lane_index = math.floor(-lat / lane_width) + 1
    lane_center = -lane_width * (lane_index - 0.5)
  end

  local deviation = lat - lane_center
  local margin = lane_width * 0.05

  if math.abs(deviation) > margin then
    local steer = math.min(math.max(-deviation / (lane_width * 0.5), -0.5), 0.5)
    veh:queueLuaCommand("electrics.values.steeringOverride = " .. steer)
    override_active = true
  else
    reset(veh)
  end
end

M.update = update
M.reset = reset

return M
