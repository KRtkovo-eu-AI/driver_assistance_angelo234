local M = {}

-- luacheck: globals vec3

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

local function clampDelta(prev, new, max_delta)
  if not prev then return new end
  local delta = new - prev
  if delta > max_delta then
    return prev + max_delta
  elseif delta < -max_delta then
    return prev - max_delta
  end
  return new
end

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

local function fitLineRange(points, f_min, f_max)
  local subset = {}
  for i = 1, #points do
    local p = points[i]
    local f = p[1]
    if f >= f_min and f <= f_max then
      subset[#subset + 1] = p
    end
  end
  local a, b = fitLine(subset)
  return a, b, #subset
end

-- Scan the road ahead using ray casts to estimate lane boundaries and heading.
-- Returns lane width, lateral offset from center and road direction.
local function sense(veh)
  if not veh then return nil end
  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then return nil end

  local wps = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.center_pos, M.prev_wps)
  M.prev_wps = wps

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

local lane_width, lateral_offset, road_dir, future_dir, curvature

  if wps then
    lane_width = wps.lane_width
    if wps.one_way then
      lateral_offset = wps.lat_dist_from_wp
    else
      lateral_offset = wps.lat_dist_from_wp - (lane_width * 0.5)
    end
    road_dir = extra_utils.toNormXYVec(wps.end_wp_pos - wps.start_wp_pos)

    local look_ahead = 20
    local future_pos = veh_props.front_pos + road_dir * look_ahead
    local future_wps = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, future_pos, wps)
    if future_wps then
      future_dir = extra_utils.toNormXYVec(future_wps.end_wp_pos - future_wps.start_wp_pos)
      curvature = math.atan2(road_dir:cross(future_dir).z, road_dir:dot(future_dir))
    end
  end

  if lidar_valid and #left_pts >= 2 and #right_pts >= 2 then
    local aL, bL = fitLine(left_pts)
    local aR, bR = fitLine(right_pts)

    local aL_near, bL_near, nLn = fitLineRange(left_pts, 0, 10)
    local aR_near, bR_near, nRn = fitLineRange(right_pts, 0, 10)
    local aL_far, bL_far, nLf = fitLineRange(left_pts, 20, 30)
    local aR_far, bR_far, nRf = fitLineRange(right_pts, 20, 30)

    if nLn < 2 then aL_near, bL_near = aL, bL end
    if nRn < 2 then aR_near, bR_near = aR, bR end
    if nLf < 2 then aL_far, bL_far = aL, bL end
    if nRf < 2 then aR_far, bR_far = aR, bR end

    local slope_near = (aL_near + aR_near) * 0.5
    local slope_far = (aL_far + aR_far) * 0.5

    local lidar_dir = (forward + right * slope_near):normalized()
    local lidar_future_dir = (forward + right * slope_far):normalized()
    local lidar_curvature = (slope_far - slope_near) / 20

    local f_ref = 5
    local left_lat = aL_near * f_ref + bL_near
    local right_lat = aR_near * f_ref + bR_near
    local lidar_width = right_lat - left_lat
    if lidar_width > 0 then
      local lane_center = 0.5 * (left_lat + right_lat)
      local lidar_offset = -lane_center

      lane_width = lane_width and (lane_width * 0.95 + lidar_width * 0.05) or lidar_width
      lateral_offset = lateral_offset and (lateral_offset * 0.95 + lidar_offset * 0.05) or lidar_offset
      road_dir = road_dir and (road_dir * 0.95 + lidar_dir * 0.05):normalized() or lidar_dir
      future_dir = future_dir and (future_dir * 0.95 + lidar_future_dir * 0.05):normalized() or lidar_future_dir
      curvature = curvature and (curvature * 0.95 + lidar_curvature * 0.05) or lidar_curvature
    end
  end

  if lane_width and lateral_offset then
    local half = lane_width * 0.5
    if lateral_offset > half then
      lateral_offset = half
    elseif lateral_offset < -half then
      lateral_offset = -half
    end
    lateral_offset = clampDelta(M.prev_offset, lateral_offset, lane_width * 0.2)
  end

  if M.prev_dir and road_dir and M.prev_dir:dot(road_dir) < 0 then
    road_dir = M.prev_dir
  end
  if M.prev_future_dir and future_dir and M.prev_future_dir:dot(future_dir) < 0 then
    future_dir = M.prev_future_dir
  end

  if not lane_width or not lateral_offset or not road_dir then return nil end

  local smooth = 0.02
  M.prev_width = M.prev_width and (M.prev_width + (lane_width - M.prev_width) * smooth) or lane_width
  M.prev_offset = M.prev_offset and (M.prev_offset + (lateral_offset - M.prev_offset) * smooth) or lateral_offset
  M.prev_dir = M.prev_dir and (M.prev_dir + (road_dir - M.prev_dir) * smooth):normalized() or road_dir
  if future_dir then
    M.prev_future_dir = M.prev_future_dir and (M.prev_future_dir + (future_dir - M.prev_future_dir) * smooth):normalized() or future_dir
  end
  if curvature then
    M.prev_curvature = M.prev_curvature and (M.prev_curvature + (curvature - M.prev_curvature) * smooth) or curvature
  end

  return {
    lane_width = M.prev_width,
    lateral_offset = M.prev_offset,
    road_dir = M.prev_dir,
    future_dir = M.prev_future_dir,
    curvature = M.prev_curvature
  }
end

M.sense = sense

return M
