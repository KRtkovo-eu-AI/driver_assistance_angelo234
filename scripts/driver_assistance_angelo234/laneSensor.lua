local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local past_wps_props_table = {}

-- Detect lane information using map waypoints around the vehicle
-- Returns a table with lane width, lateral offset from center and road direction
local function sense(veh)
  if not veh then return nil end
  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then return nil end
  local prev = past_wps_props_table[veh_props.id]
  local wps_props = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.front_pos, prev)
  past_wps_props_table[veh_props.id] = wps_props
  if not wps_props then return nil end
  local road_dir = (wps_props.end_wp_pos - wps_props.start_wp_pos):normalized()
  return {
    lane_width = wps_props.lane_width,
    lateral_offset = wps_props.lat_dist_from_wp,
    road_dir = road_dir
  }
end

M.sense = sense

return M
