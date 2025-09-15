local M = {}

-- luacheck: globals vec3

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

local function fitLine(points)
  local n = #points
  if n < 2 then return 0, 0 end
  local sum_f, sum_l, sum_ff, sum_fl = 0, 0, 0, 0
  for i = 1, n do
    local p = points[i]
    local f, l = p[1], p[2]
    sum_f = sum_f + f
    sum_l = sum_l + l
    sum_ff = sum_ff + f * f
    sum_fl = sum_fl + f * l
  end
  local denom = n * sum_ff - sum_f * sum_f
  if math.abs(denom) < 1e-6 then
    return 0, sum_l / n
  end
  local a = (n * sum_fl - sum_f * sum_l) / denom
  local b = (sum_l - a * sum_f) / n
  return a, b
end

-- Scan the road ahead using ray casts to estimate lane boundaries and heading.
-- Returns lane width, lateral offset from center and road direction.
local function sense(veh)
  if not veh then return nil end
  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then return nil end

  local wps = extra_utils.getWaypointStartEnd(veh_props, veh_props, veh_props.center_pos)

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

  local lidar_valid = #pts > 0

  local right = veh_props.dir_right
  local up = veh_props.dir_up
  local forward = veh_props.dir

  local left_pts = {}
  local right_pts = {}

  for _, pt in ipairs(pts) do
    local rel = pt - origin
    local fwd = rel:dot(forward)
    if fwd > 0 then
      local lat = rel:dot(right)
      local vert = rel:dot(up)
      if math.abs(vert) < 0.3 and math.abs(lat) < 5 then
        if lat < 0 then
          left_pts[#left_pts + 1] = {fwd, lat}
        else
          right_pts[#right_pts + 1] = {fwd, lat}
        end
      end
    end
  end

  local lane_width, lateral_offset, road_dir

  if wps then
    lane_width = wps.lane_width
    lateral_offset = wps.lat_dist_from_wp
    road_dir = extra_utils.toNormXYVec(wps.end_wp_pos - wps.start_wp_pos)
  end

  if lidar_valid and #left_pts >= 2 and #right_pts >= 2 then
    local aL, bL = fitLine(left_pts)
    local aR, bR = fitLine(right_pts)

    local f_ref = 10
    local left_lat = aL * f_ref + bL
    local right_lat = aR * f_ref + bR
    local lidar_width = right_lat - left_lat
    if lidar_width > 0 then
      local lane_center = 0.5 * (left_lat + right_lat)
      local lidar_offset = -lane_center
      local slope = (aL + aR) * 0.5
      local lidar_dir = (forward + right * slope):normalized()

      lane_width = lane_width and (lane_width * 0.7 + lidar_width * 0.3) or lidar_width
      lateral_offset = lateral_offset and (lateral_offset * 0.7 + lidar_offset * 0.3) or lidar_offset
      road_dir = road_dir and (road_dir + lidar_dir):normalized() or lidar_dir
    end
  end

  if not lane_width or not lateral_offset or not road_dir then return nil end

  M.prev_width = M.prev_width and (M.prev_width + (lane_width - M.prev_width) * 0.1) or lane_width
  M.prev_offset = M.prev_offset and (M.prev_offset + (lateral_offset - M.prev_offset) * 0.1) or lateral_offset
  M.prev_dir = M.prev_dir and (M.prev_dir + (road_dir - M.prev_dir) * 0.1):normalized() or road_dir

  return {
    lane_width = M.prev_width,
    lateral_offset = M.prev_offset,
    road_dir = M.prev_dir
  }
end

M.sense = sense

return M
