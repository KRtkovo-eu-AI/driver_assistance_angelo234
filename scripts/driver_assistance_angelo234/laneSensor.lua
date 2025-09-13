local M = {}

-- luacheck: globals vec3

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

-- Scan the road ahead using ray casts to estimate lane boundaries.
-- Returns a table with lane width, lateral offset from center and road direction
local function sense(veh)
  if not veh then return nil end
  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then return nil end

  -- Cast a lidar-like fan in front of the vehicle slightly above the ground
  local origin = veh_props.front_pos + veh_props.dir_up * 0.5
  local pts = virtual_lidar.scan(
    origin,
    veh_props.dir,
    veh_props.dir_up,
    40,
    math.rad(60),
    math.rad(10),
    20,
    3,
    0.5,
    veh_props.id
  )

  if #pts == 0 then return nil end

  local right = veh_props.dir_right
  local up = veh_props.dir_up
  local forward = veh_props.dir

  local lats = {}
  local sum_vec = vec3()
  local count = 0

  for _, pt in ipairs(pts) do
    local rel = pt - origin
    local fwd = rel:dot(forward)
    if fwd > 0 then
      local lat = rel:dot(right)
      local vert = rel:dot(up)
      if math.abs(vert) < 0.3 and math.abs(lat) < 5 then
        lats[#lats + 1] = lat
        sum_vec = sum_vec + rel
        count = count + 1
      end
    end
  end

  if #lats < 2 then return nil end
  table.sort(lats)
  local low_idx = math.max(1, math.floor(#lats * 0.1))
  local high_idx = math.min(#lats, math.ceil(#lats * 0.9))
  local min_lat = lats[low_idx]
  local max_lat = lats[high_idx]
  local lane_width = max_lat - min_lat
  if lane_width <= 0 then return nil end

  local lane_center = 0.5 * (min_lat + max_lat)
  local lateral_offset = -lane_center

  -- Smooth results to reduce jitter
  M.prev_width = M.prev_width and (M.prev_width + (lane_width - M.prev_width) * 0.2) or lane_width
  M.prev_offset = M.prev_offset and (M.prev_offset + (lateral_offset - M.prev_offset) * 0.2) or lateral_offset
  M.prev_dir = M.prev_dir or veh_props.dir

  local road_dir
  if count > 0 then
    road_dir = vec3(sum_vec.x, sum_vec.y, 0):normalized()
  else
    road_dir = veh_props.dir
  end

  -- Smooth road direction for stability
  M.prev_dir = (M.prev_dir + (road_dir - M.prev_dir) * 0.2):normalized()

  return {
    lane_width = M.prev_width,
    lateral_offset = M.prev_offset,
    road_dir = M.prev_dir
  }
end

M.sense = sense

return M
