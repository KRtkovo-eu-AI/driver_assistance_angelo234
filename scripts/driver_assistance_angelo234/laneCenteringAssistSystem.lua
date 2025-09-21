-- luacheck: globals ui_message Engine electrics_values_angelo234

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local _ = require("controlSystems")

local abs = math.abs
local max = math.max
local min = math.min
local sqrt = math.sqrt
local ceil = math.ceil
local huge = math.huge
local clamp = function(value, low, high)
  if value < low then return low end
  if value > high then return high end
  return value
end

local latest_data = {
  status = {
    installed = false,
    enabled = false,
    active = false,
    available = false,
    reason = "init"
  }
}

local warning_played = false
local steering_pid = nil
local steering_smoother = nil
local controller_signature = nil
local activation_grace_timer = 0
local activation_handler = nil
local last_assist_delta = 0

local DOUBLE_CHIME_SOUND = 'art/sound/proximity_tone_50ms_moderate.wav'
local DOUBLE_CHIME_GAP = 0.15
local double_chime_pending = 0
local double_chime_timer = 0

local function queueDoubleChime()
  if double_chime_pending <= 0 then
    double_chime_timer = 0
  end
  double_chime_pending = double_chime_pending + 2
end

local function updateDoubleChime(dt)
  if double_chime_pending <= 0 then return end
  double_chime_timer = double_chime_timer - (dt or 0)
  if double_chime_timer > 0 then return end

  Engine.Audio.playOnce('AudioGui', DOUBLE_CHIME_SOUND)
  double_chime_pending = double_chime_pending - 1

  if double_chime_pending > 0 then
    double_chime_timer = DOUBLE_CHIME_GAP
  else
    double_chime_timer = 0
  end
end

local lane_state = {
  prev_wps = nil,
  desired_offset = nil,
  lane_width = nil,
  prev_path_nodes = nil
}

local function resetControllers()
  if steering_pid then
    steering_pid:reset()
  end
  steering_pid = nil
  steering_smoother = nil
  controller_signature = nil
  last_assist_delta = 0
end

local function ensureControllers(params, steer_limit)
  local kp = params.steer_kp or 0.35
  local ki = params.steer_ki or 0.08
  local kd = params.steer_kd or 0.3
  local smoothing = (params.steer_smoothing or 0.05) * 1000
  local signature = string.format("%.3f|%.3f|%.3f|%.3f|%.3f", kp, ki, kd, steer_limit or 0.15, smoothing)
  if signature ~= controller_signature or not steering_pid or not steering_smoother then
    steering_pid = newPIDStandard(kp, ki, kd, -(steer_limit or 0.15), steer_limit or 0.15)
    steering_smoother = newTemporalSmoothing(smoothing, smoothing)
    controller_signature = signature
  end
end

local function buildLaneOffsets(lane_width, lanes)
  lanes = lanes or 1
  lane_width = lane_width or 0
  if lane_width <= 0 or lanes < 1 then return {0} end

  local offsets = {}

  if lanes == 1 then
    offsets[1] = 0
  elseif lanes % 2 == 1 then
    local half = (lanes - 1) * 0.5
    for i = -half, half do
      offsets[#offsets + 1] = i * lane_width
    end
  else
    local half = lanes * 0.5
    for i = 1, half do
      local offset = (i - 0.5) * lane_width
      offsets[#offsets + 1] = -offset
      offsets[#offsets + 1] = offset
    end
    table.sort(offsets)
  end

  return offsets
end

local function selectOffset(offsets, reference)
  if #offsets == 0 then return 0 end
  if reference == nil then return offsets[1] end

  local best = offsets[1]
  local best_dist = math.huge
  for _, value in ipairs(offsets) do
    local dist = abs(value - reference)
    if dist < best_dist then
      best_dist = dist
      best = value
    end
  end
  return best
end

local function makeEdgeKey(a, b)
  return string.format("%s|%s", tostring(a or "?"), tostring(b or "?"))
end

local function cloneVec3(v)
  return vec3(v.x, v.y, v.z)
end

local function catmullRomPoint(p0, p1, p2, p3, t)
  local t2 = t * t
  local t3 = t2 * t
  local function interpolate(c0, c1, c2, c3)
    return 0.5 * ((2 * c1) + (-c0 + c2) * t + (2 * c0 - 5 * c1 + 4 * c2 - c3) * t2 + (-c0 + 3 * c1 - 3 * c2 + c3) * t3)
  end

  return vec3(
    interpolate(p0.x, p1.x, p2.x, p3.x),
    interpolate(p0.y, p1.y, p2.y, p3.y),
    interpolate(p0.z, p1.z, p2.z, p3.z)
  )
end

local function smoothCenterline(points, subdivisions)
  if not points or #points < 2 then return points end
  subdivisions = max(1, subdivisions or 1)

  local smoothed = {}
  smoothed[#smoothed + 1] = cloneVec3(points[1])

  for i = 1, #points - 1 do
    local p0 = points[max(1, i - 1)]
    local p1 = points[i]
    local p2 = points[i + 1]
    local p3 = points[min(#points, i + 2)]

    for step = 1, subdivisions do
      local t = step / subdivisions
      smoothed[#smoothed + 1] = catmullRomPoint(p0, p1, p2, p3, t)
    end
  end

  return smoothed
end

local function computeNormals(points)
  local normals = {}
  if not points or #points == 0 then return normals end

  for i = 1, #points do
    local prev = points[max(1, i - 1)]
    local next = points[min(#points, i + 1)]
    local tangent = vec3(next.x - prev.x, next.y - prev.y, 0)
    if tangent:squaredLength() > 1e-9 then
      tangent:normalize()
    else
      tangent = vec3(0, 1, 0)
    end

    normals[i] = vec3(tangent.y, -tangent.x, 0)
  end

  return normals
end

local function convertSamplesToLocal(samples, normals, veh_props, half_width)
  local center_local = {}
  local left_local = {}
  local right_local = {}

  if not samples then return center_local, left_local, right_local end

  local veh_pos = veh_props.center_pos
  local dir = veh_props.dir
  local dir_right = veh_props.dir_right

  for i = 1, #samples do
    local sample = samples[i]
    local normal = normals[i] or dir_right

    local center_rel = sample - veh_pos
    local left_point = vec3(sample.x - normal.x * half_width, sample.y - normal.y * half_width, sample.z)
    local right_point = vec3(sample.x + normal.x * half_width, sample.y + normal.y * half_width, sample.z)

    local left_rel = left_point - veh_pos
    local right_rel = right_point - veh_pos

    center_local[#center_local + 1] = {x = center_rel:dot(dir_right), y = center_rel:dot(dir)}
    left_local[#left_local + 1] = {x = left_rel:dot(dir_right), y = left_rel:dot(dir)}
    right_local[#right_local + 1] = {x = right_rel:dot(dir_right), y = right_rel:dot(dir)}
  end

  return center_local, left_local, right_local
end

local function computeAverageCurvature(center_local, sample_limit)
  if not center_local or #center_local < 3 then return 0 end

  local max_samples = min(#center_local - 2, sample_limit or 10)
  if max_samples < 1 then return 0 end

  local sum = 0
  local count = 0

  for i = 2, max_samples + 1 do
    local p0 = center_local[i - 1]
    local p1 = center_local[i]
    local p2 = center_local[i + 1]
    if not p2 then break end

    local abx = p1.x - p0.x
    local aby = p1.y - p0.y
    local bcx = p2.x - p1.x
    local bcy = p2.y - p1.y
    local acx = p2.x - p0.x
    local acy = p2.y - p0.y

    local cross = abx * bcy - aby * bcx
    local ab = sqrt(abx * abx + aby * aby)
    local bc = sqrt(bcx * bcx + bcy * bcy)
    local ac = sqrt(acx * acx + acy * acy)
    local denom = ab * bc * ac

    if denom > 1e-6 then
      local curvature = 2 * cross / denom
      sum = sum + curvature
      count = count + 1
    end
  end

  if count == 0 then return 0 end
  return sum / count
end

local function computePathLength(points)
  if not points or #points < 2 then return 0 end
  local length = 0
  for i = 2, #points do
    length = length + (points[i] - points[i - 1]):length()
  end
  return length
end

local function selectNextSegment(veh_props, current_wps, params, _preferred_offset, visited, map_data)
  if not current_wps or not current_wps.end_wp then return nil end

  visited = visited or {}
  map_data = map_data or extra_utils.getGraphData()

  local current_node = current_wps.end_wp
  local current_pos = vec3(current_wps.end_wp_pos.x, current_wps.end_wp_pos.y, veh_props.center_pos.z)
  local prev_pos = vec3(current_wps.start_wp_pos.x, current_wps.start_wp_pos.y, veh_props.center_pos.z)
  local current_dir = extra_utils.toNormXYVec(current_pos - prev_pos)
  if current_dir:length() < 1e-6 then
    current_dir = extra_utils.toNormXYVec(veh_props.dir)
  end
  local vehicle_dir = extra_utils.toNormXYVec(veh_props.dir)
  local align_weight = clamp(params.segment_alignment_weight or 0.7, 0, 1)
  local heading_weight = clamp(params.segment_heading_weight or (1 - align_weight), 0, 1)
  local drivability_weight = params.segment_drivability_weight or 0.15

  local best_wp = nil
  local best_score = -huge

  local function evaluateCandidate(next_wp, link)
    if next_wp == current_wps.start_wp then return end
    local key = makeEdgeKey(current_node, next_wp)
    if visited[key] then return end

    if link and link.oneWay and link.inNode and link.inNode ~= current_node then
      return
    end

    local next_pos = extra_utils.getWaypointPosition(next_wp)
    if not next_pos then return end
    next_pos = vec3(next_pos.x, next_pos.y, veh_props.center_pos.z)

    local seg_vec = next_pos - current_pos
    if seg_vec:squaredLength() <= 1e-6 then return end

    local seg_dir = extra_utils.toNormXYVec(seg_vec)
    local forward_score = seg_dir:dot(current_dir)
    local heading_score = seg_dir:dot(vehicle_dir)
    local drivability = (link and link.drivability) or 1

    local penalty = 0
    if drivability <= 0 then
      penalty = penalty + 10
    end
    if link and link.gated and link.gated ~= 0 then
      penalty = penalty + 1
    end

    local score =
      forward_score * align_weight + heading_score * heading_weight + drivability * drivability_weight - penalty

    if score > best_score then
      best_score = score
      best_wp = next_wp
    end
  end

  if map_data and map_data.graph then
    local graph_links = map_data.graph[current_node]
    if graph_links then
      for next_wp, link in pairs(graph_links) do
        evaluateCandidate(next_wp, link)
      end
    end
  end

  if not best_wp then
    local node = extra_utils.getMapNode(current_node)
    if node and node.links then
      for next_wp, link in pairs(node.links) do
        evaluateCandidate(next_wp, link)
      end
    end
  end

  if not best_wp then return nil end
  return extra_utils.getWaypointSegmentFromNodes(veh_props, veh_props, current_node, best_wp)
end

local function buildSegmentsFromNodeList(nodes, veh_props)
  if not nodes or #nodes < 2 then return {} end
  local segments = {}
  for i = 1, #nodes - 1 do
    local seg = extra_utils.getWaypointSegmentFromNodes(veh_props, veh_props, nodes[i], nodes[i + 1])
    if seg then
      segments[#segments + 1] = seg
    end
  end
  return segments
end

local function computeBranchOrientation(prev_node, node, next_node, veh_dir)
  if not node or not next_node then return 0, 0 end
  local node_pos = extra_utils.getWaypointPosition(node)
  local next_pos = extra_utils.getWaypointPosition(next_node)
  if not node_pos or not next_pos then return 0, 0 end

  local base_dir = nil
  if prev_node then
    local prev_pos = extra_utils.getWaypointPosition(prev_node)
    if prev_pos then
      base_dir = extra_utils.toNormXYVec(node_pos - prev_pos)
    end
  end
  if not base_dir or base_dir:length() < 1e-6 then
    if veh_dir then
      base_dir = extra_utils.toNormXYVec(veh_dir)
    else
      base_dir = extra_utils.toNormXYVec(next_pos - node_pos)
    end
  end

  local branch_dir = extra_utils.toNormXYVec(next_pos - node_pos)
  if branch_dir:length() < 1e-6 then
    return 0, 0
  end

  local align = base_dir:dot(branch_dir)
  local turn = base_dir.x * branch_dir.y - base_dir.y * branch_dir.x
  return align, turn
end

local function gatherBranchSpecs(map_data, nodes, params, veh_props, branch_length)
  if not map_data or not map_data.getPathTWithState or not nodes then return {} end

  local specs = {}
  local per_node = params.branch_max_per_node or 3
  local total_cap = params.branch_total_cap or 12
  local seen = {}
  local added = 0
  branch_length = branch_length or params.branch_lookahead or 60

  for idx = 2, #nodes - 1 do
    local current = nodes[idx]
    local prev = nodes[idx - 1]
    local next_node = nodes[idx + 1]
    local neighbors = extra_utils.getGraphLinks(current)
    if neighbors then
      local count = 0
      for neighbor, link in pairs(neighbors) do
        if neighbor ~= prev then
          if not link or link.drivability == nil or link.drivability > 0 then
            local key = string.format("%s|%s", tostring(current), tostring(neighbor))
            if not seen[key] then
              seen[key] = true
              local branch_nodes = {current}
              local state_path = {current, neighbor}
              local waypoint_pos = extra_utils.getWaypointPosition(neighbor)
              local branch_path = map_data:getPathTWithState(neighbor, waypoint_pos or veh_props.center_pos, branch_length, state_path)
              local last_node = current
              if branch_path and #branch_path > 0 then
                for j = 1, #branch_path do
                  local bn = branch_path[j]
                  if bn ~= last_node then
                    branch_nodes[#branch_nodes + 1] = bn
                    last_node = bn
                  end
                end
              else
                branch_nodes[#branch_nodes + 1] = neighbor
              end

              if #branch_nodes >= 2 then
                local align, turn = computeBranchOrientation(prev, current, branch_nodes[2], veh_props.dir)
                specs[#specs + 1] = {
                  originIndex = idx,
                  nodes = branch_nodes,
                  align = align,
                  turn = turn,
                  source = current,
                  target = branch_nodes[2],
                  isMain = neighbor == next_node
                }
                count = count + 1
                added = added + 1
                if total_cap and total_cap > 0 and added >= total_cap then
                  return specs
                end
                if per_node and per_node > 0 and count >= per_node then
                  break
                end
              end
            end
          end
        end
      end
    end
  end

  return specs
end

local function buildSegmentPlan(veh_props, initial_wps, params, target_length)
  local map_data = extra_utils.getGraphData()
  if not map_data or not map_data.getPathTWithState then return nil end

  local state_node = initial_wps.end_wp
  if not state_node then return nil end

  local prev_path = lane_state.prev_path_nodes
  local use_prev = prev_path and prev_path[1] == state_node

  local heading_keep = params.prev_path_heading_keep or 0
  if use_prev and heading_keep > 0 and #prev_path >= 2 then
    local first_pos = extra_utils.getWaypointPosition(prev_path[1])
    local second_pos = extra_utils.getWaypointPosition(prev_path[2])
    if first_pos and second_pos then
      local prev_dir = extra_utils.toNormXYVec(second_pos - first_pos)
      if prev_dir:length() > 0 then
        local heading = prev_dir:dot(extra_utils.toNormXYVec(veh_props.dir))
        if heading < heading_keep then
          use_prev = false
        end
      end
    end
  end

  local state
  if use_prev then
    state = prev_path
  else
    state = extra_utils.toNormXYVec(veh_props.dir)
  end

  local branch_extra = params.branch_lookahead or 60
  if branch_extra < 20 then branch_extra = 20 end
  local lookahead_goal = target_length + branch_extra
  local path_nodes = map_data:getPathTWithState(state_node, veh_props.center_pos, lookahead_goal, state)
  if not path_nodes or #path_nodes == 0 then
    lane_state.prev_path_nodes = nil
    return nil
  end

  lane_state.prev_path_nodes = path_nodes

  local nodes = {initial_wps.start_wp}
  local last_node = initial_wps.start_wp
  for i = 1, #path_nodes do
    local node = path_nodes[i]
    if node ~= last_node then
      nodes[#nodes + 1] = node
      last_node = node
    end
  end

  local segments = buildSegmentsFromNodeList(nodes, veh_props)
  if #segments == 0 then
    return nil
  end

  local segment_length_hint = params.segment_length_hint or 5
  if segment_length_hint < 0.5 then segment_length_hint = 0.5 end
  local segments_cap = params.path_segments_cap or 0
  local segment_goal = ceil(target_length / segment_length_hint)

  local limit_cap = segments_cap > 0 and segments_cap or #segments
  local plan = {
    nodes = nodes,
    segments = segments,
    segment_goal = segment_goal,
    segment_limit = math.min(#segments, limit_cap),
    segment_cap = segments_cap > 0 and segments_cap or nil,
    segment_step = params.path_segments_extend or segment_goal,
    branch_length = branch_extra
  }

  plan.branches = gatherBranchSpecs(map_data, nodes, params, veh_props, branch_extra)

  return plan
end

local function buildPathGeometryFromSegments(veh_props, segments_list, desired_offset, params, target_length)
  if not segments_list or #segments_list == 0 then return nil end

  local subdivisions = max(1, params.curve_subdivisions or 6)
  local curvature_samples = params.curvature_samples or 8
  local future_dir_distance = params.future_dir_distance or 18

  local centers_world = {}
  local lane_width_sum = 0
  local lane_width_count = 0
  local lane_width_est = segments_list[1].lane_width or 0
  local preferred_offset = desired_offset or 0
  local total_length = 0
  local segments_used = 0
  local last_dir = nil
  local last_wps = nil

  for i = 1, #segments_list do
    local wps = segments_list[i]
    local start_pos = wps.start_wp_pos and vec3(wps.start_wp_pos.x, wps.start_wp_pos.y, veh_props.center_pos.z)
    local end_pos = wps.end_wp_pos and vec3(wps.end_wp_pos.x, wps.end_wp_pos.y, veh_props.center_pos.z)
    if start_pos and end_pos then
      local seg_vec = end_pos - start_pos
      local seg_len = seg_vec:length()
      if seg_len > 1e-6 then
        local seg_dir = vec3(seg_vec.x / seg_len, seg_vec.y / seg_len, 0)
        local seg_right = vec3(seg_dir.y, -seg_dir.x, 0)
        lane_width_est = wps.lane_width or lane_width_est
        if wps.lane_width and wps.lane_width > 0 then
          lane_width_sum = lane_width_sum + wps.lane_width
          lane_width_count = lane_width_count + 1
        end
        local offsets = buildLaneOffsets(wps.lane_width or lane_width_est, wps.num_of_lanes or 1)
        preferred_offset = selectOffset(offsets, preferred_offset)

        local start_center = vec3(start_pos.x + seg_right.x * preferred_offset, start_pos.y + seg_right.y * preferred_offset, start_pos.z)
        local end_center = vec3(end_pos.x + seg_right.x * preferred_offset, end_pos.y + seg_right.y * preferred_offset, end_pos.z)

        if #centers_world == 0 then
          centers_world[1] = start_center
        end

        local prev_point = centers_world[#centers_world]
        if (end_center - prev_point):squaredLength() > 1e-6 then
          centers_world[#centers_world + 1] = end_center
          total_length = total_length + (end_center - prev_point):length()
        end

        segments_used = segments_used + 1
        last_dir = seg_dir
        last_wps = wps

        if target_length and target_length > 0 and total_length >= target_length then
          break
        end
      end
    end
  end

  if #centers_world < 2 then return nil end

  local lane_width = lane_width_est
  if lane_width_count > 0 then
    lane_width = lane_width_sum / lane_width_count
  end
  if not lane_width or lane_width <= 0 then
    lane_width = params.default_lane_width or 3.5
  end
  local half_width = lane_width * 0.5

  local smoothed = smoothCenterline(centers_world, subdivisions)
  local path_length = computePathLength(smoothed)

  local accumulated = 0
  local future_dir = nil
  local future_distance = future_dir_distance
  if future_distance > path_length then
    future_distance = path_length * 0.8
  end
  for i = 2, #smoothed do
    local seg_len = (smoothed[i] - smoothed[i - 1]):length()
    if seg_len > 1e-6 then
      accumulated = accumulated + seg_len
      if not future_dir and accumulated >= future_distance then
        future_dir = extra_utils.toNormXYVec(smoothed[i] - smoothed[i - 1])
      end
    end
  end
  if not future_dir and #smoothed >= 2 then
    future_dir = extra_utils.toNormXYVec(smoothed[#smoothed] - smoothed[#smoothed - 1])
  end

  local normals = computeNormals(smoothed)
  local center_local, left_local, right_local = convertSamplesToLocal(smoothed, normals, veh_props, half_width)

  local lane_dir_norm = nil
  if #smoothed >= 2 then
    lane_dir_norm = extra_utils.toNormXYVec(smoothed[2] - smoothed[1])
  end
  if (not lane_dir_norm or lane_dir_norm:length() < 1e-6) and last_dir then
    lane_dir_norm = last_dir
  end
  if not lane_dir_norm then
    lane_dir_norm = extra_utils.toNormXYVec(veh_props.dir)
  end

  local curvature = computeAverageCurvature(center_local, curvature_samples)
  local sample_spacing = (#center_local > 1) and (path_length / (#center_local - 1)) or 0
  local truncated = target_length and (path_length + 1e-6 < target_length) or false

  return {
    center = center_local,
    left = left_local,
    right = right_local,
    dir = lane_dir_norm,
    future_dir = future_dir,
    curvature = curvature,
    length = path_length,
    offset = preferred_offset or 0,
    lane_width = lane_width,
    sample_spacing = sample_spacing,
    segments_used = segments_used,
    covered_length = total_length,
    last_wps = last_wps,
    truncated = truncated,
    lane_width_sum = lane_width_sum,
    lane_width_count = lane_width_count
  }
end

local function gatherPathLegacy(veh_props, initial_wps, desired_offset, params)
  if not initial_wps then return nil end

  local map_data = extra_utils.getGraphData()

  local lookahead_base = params.lookahead_base or 22
  local lookahead_min = params.lookahead_min or lookahead_base
  if lookahead_min < 0 then lookahead_min = 0 end
  if lookahead_base < lookahead_min then
    lookahead_base = lookahead_min
  end

  local lookahead_speed_gain = params.lookahead_speed_gain or 1.6
  local lookahead_max = params.lookahead_max or 120
  if lookahead_max < lookahead_min then
    lookahead_max = lookahead_min
  end

  local target_length = lookahead_base + veh_props.speed * lookahead_speed_gain
  if target_length < lookahead_min then
    target_length = lookahead_min
  end
  target_length = clamp(target_length, lookahead_min, lookahead_max)

  local segment_length_hint = params.segment_length_hint or 5
  if segment_length_hint < 0.5 then
    segment_length_hint = 0.5
  end

  local segments_cap = params.path_segments_cap or 0

  local base_segments = params.path_segments or 20
  if base_segments < 1 then base_segments = 1 end
  if segments_cap > 0 and base_segments > segments_cap then
    base_segments = segments_cap
  end

  local dynamic_segments = ceil(target_length / segment_length_hint)
  local initial_segments = max(base_segments, dynamic_segments)

  local hard_cap = segments_cap > 0 and segments_cap or huge

  local max_segments = min(initial_segments, hard_cap)

  local segments_extension = params.path_segments_extend or base_segments
  if segments_extension < 1 then segments_extension = 1 end

  local subdivisions = max(1, params.curve_subdivisions or 6)
  local curvature_samples = params.curvature_samples or 8
  local future_dir_distance = params.future_dir_distance or 18

  local centers_world = {}
  local visited = {}
  local segments = 0
  local total_length = 0
  local lane_width_est = initial_wps.lane_width or 0
  local lane_width_sum = 0
  local lane_width_count = 0
  local current_wps = initial_wps
  local preferred_offset = desired_offset or 0
  local last_dir = nil

  while current_wps and total_length < target_length do
    if segments >= max_segments then
      if max_segments >= hard_cap then
        break
      end
      local new_limit = max_segments + segments_extension
      if new_limit > hard_cap then
        new_limit = hard_cap
      end
      if new_limit <= max_segments then
        break
      end
      max_segments = new_limit
    end

    local seg_vec = current_wps.end_wp_pos - current_wps.start_wp_pos
    local seg_dir = extra_utils.toNormXYVec(seg_vec)
    if seg_dir:length() < 1e-6 then break end
    local seg_right = vec3(seg_dir.y, -seg_dir.x, 0)

    local start_center = vec3(
      current_wps.start_wp_pos.x + seg_right.x * (preferred_offset or 0),
      current_wps.start_wp_pos.y + seg_right.y * (preferred_offset or 0),
      veh_props.center_pos.z
    )
    local end_center = vec3(
      current_wps.end_wp_pos.x + seg_right.x * (preferred_offset or 0),
      current_wps.end_wp_pos.y + seg_right.y * (preferred_offset or 0),
      veh_props.center_pos.z
    )

    if #centers_world == 0 then
      centers_world[1] = start_center
    end

    local prev_point = centers_world[#centers_world]
    if (end_center - prev_point):squaredLength() > 1e-6 then
      centers_world[#centers_world + 1] = end_center
      total_length = total_length + (end_center - prev_point):length()
    end

    lane_width_est = current_wps.lane_width or lane_width_est
    if lane_width_est and lane_width_est > 0 then
      lane_width_sum = lane_width_sum + lane_width_est
      lane_width_count = lane_width_count + 1
    end

    last_dir = seg_dir
    segments = segments + 1
    visited[makeEdgeKey(current_wps.start_wp, current_wps.end_wp)] = true

    if total_length >= target_length then break end

    local next_wps = selectNextSegment(veh_props, current_wps, params, preferred_offset, visited, map_data)
    if not next_wps then
      local probe_dist = params.segment_probe_distance or 5
      local fallback_query = vec3(
        current_wps.end_wp_pos.x + seg_dir.x * probe_dist,
        current_wps.end_wp_pos.y + seg_dir.y * probe_dist,
        veh_props.center_pos.z
      )
      local fallback = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, fallback_query, current_wps)
      if fallback then
        local fallback_key = makeEdgeKey(fallback.start_wp, fallback.end_wp)
        if not visited[fallback_key] then
          next_wps = fallback
        end
      end
    end
    if not next_wps then break end

    local offsets = buildLaneOffsets(next_wps.lane_width or lane_width_est, next_wps.num_of_lanes or 1)
    preferred_offset = selectOffset(offsets, preferred_offset)

    current_wps = next_wps
  end

  if #centers_world < 2 then return nil end

  local smoothed = smoothCenterline(centers_world, subdivisions)
  local path_length = computePathLength(smoothed)

  local accumulated = 0
  local future_dir = nil
  if future_dir_distance > path_length then
    future_dir_distance = path_length * 0.8
  end
  for i = 2, #smoothed do
    local seg_len = (smoothed[i] - smoothed[i - 1]):length()
    if seg_len > 1e-6 then
      accumulated = accumulated + seg_len
      if not future_dir and accumulated >= future_dir_distance then
        future_dir = extra_utils.toNormXYVec(smoothed[i] - smoothed[i - 1])
      end
    end
  end
  if not future_dir and #smoothed >= 2 then
    future_dir = extra_utils.toNormXYVec(smoothed[#smoothed] - smoothed[#smoothed - 1])
  end

  local normals = computeNormals(smoothed)
  local half_width = 0
  if lane_width_count > 0 then
    half_width = (lane_width_sum / lane_width_count) * 0.5
  elseif lane_width_est and lane_width_est > 0 then
    half_width = lane_width_est * 0.5
  end

  local center_local, left_local, right_local = convertSamplesToLocal(smoothed, normals, veh_props, half_width)
  local lane_dir_norm = nil
  if #smoothed >= 2 then
    lane_dir_norm = extra_utils.toNormXYVec(smoothed[2] - smoothed[1])
  end
  if (not lane_dir_norm or lane_dir_norm:length() < 1e-6) and last_dir then
    lane_dir_norm = last_dir
  end
  if not lane_dir_norm then
    lane_dir_norm = extra_utils.toNormXYVec(veh_props.dir)
  end

  local curvature = computeAverageCurvature(center_local, curvature_samples)

  return {
    center = center_local,
    left = left_local,
    right = right_local,
    dir = lane_dir_norm,
    future_dir = future_dir,
    curvature = curvature,
    length = path_length,
    offset = preferred_offset or 0,
    last_wps = current_wps,
    lane_width = half_width * 2,
    sample_spacing = (#center_local > 1) and (path_length / (#center_local - 1)) or 0,
    target_length = target_length,
    segment_count = segments,
    segment_limit = max_segments,
    segment_goal = initial_segments,
    segment_cap = segments_cap > 0 and segments_cap or nil,
    segment_step = segments_extension,
    truncated = total_length < target_length,
    covered_length = total_length,
    branches = {},
    nodes = nil
  }
end

local function gatherPath(veh_props, initial_wps, desired_offset, params)
  if not initial_wps then return nil end

  local lookahead_base = params.lookahead_base or 22
  local lookahead_min = params.lookahead_min or lookahead_base
  if lookahead_min < 0 then lookahead_min = 0 end
  if lookahead_base < lookahead_min then
    lookahead_base = lookahead_min
  end

  local lookahead_speed_gain = params.lookahead_speed_gain or 1.6
  local lookahead_max = params.lookahead_max or 120
  if lookahead_max < lookahead_min then
    lookahead_max = lookahead_min
  end

  local target_length = lookahead_base + veh_props.speed * lookahead_speed_gain
  if target_length < lookahead_min then
    target_length = lookahead_min
  end
  target_length = clamp(target_length, lookahead_min, lookahead_max)

  local segment_length_hint = params.segment_length_hint or 5
  if segment_length_hint < 0.5 then
    segment_length_hint = 0.5
  end

  local segments_cap = params.path_segments_cap or 0

  local base_segments = params.path_segments or 20
  if base_segments < 1 then base_segments = 1 end
  if segments_cap > 0 and base_segments > segments_cap then
    base_segments = segments_cap
  end

  local dynamic_segments = ceil(target_length / segment_length_hint)
  local initial_segments = max(base_segments, dynamic_segments)

  local segments_extension = params.path_segments_extend or base_segments
  if segments_extension < 1 then segments_extension = 1 end

  local plan = buildSegmentPlan(veh_props, initial_wps, params, target_length)
  if plan and plan.segments and #plan.segments > 0 then
    local geometry = buildPathGeometryFromSegments(veh_props, plan.segments, desired_offset, params, target_length)
    if geometry then
      local path = geometry
      path.segment_count = geometry.segments_used or #plan.segments
      path.segment_limit = plan.segment_limit or #plan.segments
      path.segment_goal = plan.segment_goal or initial_segments
      path.segment_cap = plan.segment_cap or (segments_cap > 0 and segments_cap or nil)
      path.segment_step = plan.segment_step or segments_extension
      path.target_length = target_length
      path.truncated = geometry.truncated or (geometry.length + 1e-6 < target_length)
      path.nodes = plan.nodes

      local branches = {}
      if plan.branches then
        for _, spec in ipairs(plan.branches) do
          if not spec.isMain then
            local branch_segments = buildSegmentsFromNodeList(spec.nodes, veh_props)
            if branch_segments and #branch_segments > 1 then
              local branch_geom = buildPathGeometryFromSegments(veh_props, branch_segments, geometry.offset, params, plan.branch_length or target_length)
              if branch_geom then
                branch_geom.originIndex = spec.originIndex
                branch_geom.align = spec.align
                branch_geom.turn = spec.turn
                branch_geom.nodes = spec.nodes
                branches[#branches + 1] = branch_geom
              end
            end
          end
        end
      end

      path.branches = branches
      return path
    end
  end

  lane_state.prev_path_nodes = nil
  return gatherPathLegacy(veh_props, initial_wps, desired_offset, params)
end

local function computeLaneModel(veh_props, params)
  if not veh_props then return nil end

  local wps = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.front_pos, lane_state.prev_wps)
  lane_state.prev_wps = wps
  if not wps then
    lane_state.desired_offset = nil
    return nil
  end

  local lane_width = wps.lane_width or 0
  if lane_width <= 0 then
    lane_state.desired_offset = nil
    return nil
  end

  local offsets = buildLaneOffsets(lane_width, wps.num_of_lanes or 1)
  if #offsets == 0 then
    lane_state.desired_offset = nil
    return nil
  end

  local start_xy = vec3(wps.start_wp_pos.x, wps.start_wp_pos.y, 0)
  local end_xy = vec3(wps.end_wp_pos.x, wps.end_wp_pos.y, 0)
  local veh_xy = vec3(veh_props.center_pos.x, veh_props.center_pos.y, 0)
  local seg_vec = end_xy - start_xy
  local seg_len_sq = seg_vec:squaredLength()
  if seg_len_sq < 1e-6 then
    lane_state.desired_offset = nil
    return nil
  end

  local xnorm = clamp((veh_xy - start_xy):dot(seg_vec) / seg_len_sq, 0, 1)
  local lane_point_xy = start_xy + seg_vec * xnorm
  local lane_dir = extra_utils.toNormXYVec(seg_vec)
  if lane_dir:length() < 1e-6 then
    lane_state.desired_offset = nil
    return nil
  end

  local lane_right = vec3(lane_dir.y, -lane_dir.x, 0)
  local current_offset = (veh_xy - lane_point_xy):dot(lane_right)

  local desired_offset = selectOffset(offsets, lane_state.desired_offset or current_offset)

  local offset_smooth = clamp(params.lane_offset_smooth or 0.0, 0, 1)
  if lane_state.desired_offset then
    desired_offset = lane_state.desired_offset + (desired_offset - lane_state.desired_offset) * (1 - offset_smooth)
  end

  local path = gatherPath(veh_props, wps, desired_offset, params)
  if not path then
    lane_state.desired_offset = desired_offset
    return nil
  end

  lane_state.desired_offset = path.offset or desired_offset
  lane_state.prev_wps = path.last_wps or wps
  lane_state.lane_width = path.lane_width or lane_width

  local lane_dir_norm = path.dir
  if not lane_dir_norm or lane_dir_norm:length() < 1e-6 then
    lane_dir_norm = extra_utils.toNormXYVec(wps.end_wp_pos - wps.start_wp_pos)
  end
  local vehicle_dir = extra_utils.toNormXYVec(veh_props.dir)
  local heading_error = vehicle_dir:cross(lane_dir_norm).z
  local half_width = (path.lane_width or lane_width) * 0.5
  local target_offset = path.offset or desired_offset
  local lateral_error = current_offset - target_offset
  local normalized_error = 0
  if half_width > 1e-3 then
    normalized_error = clamp(lateral_error / half_width, -5, 5)
  end

  local future_dir = path.future_dir
  local curvature = path.curvature or 0

  local lane_model = {
    width = path.lane_width or lane_width,
    offset = {
      current = current_offset,
      target = target_offset,
      error = lateral_error,
      normalized = normalized_error
    },
    heading = {
      vehicle = {x = vehicle_dir.x, y = vehicle_dir.y},
      lane = {x = lane_dir_norm.x, y = lane_dir_norm.y},
      error = heading_error
    },
    curvature = curvature,
    path = {
      center = path.center,
      left = path.left,
      right = path.right,
      length = path.length or 0,
      targetLength = path.target_length or 0,
      coveredLength = path.covered_length or path.length or 0,
      segments = path.segment_count or 0,
      sampleSpacing = path.sample_spacing or 0,
      truncated = path.truncated or false,
      segmentLimit = path.segment_limit or 0,
      segmentGoal = path.segment_goal or 0,
      segmentCap = path.segment_cap,
      segmentStep = path.segment_step or 0,
      branches = path.branches or {},
      nodes = path.nodes
    },
    speed = veh_props.speed,
    xnorm = xnorm
  }

  if future_dir then
    lane_model.heading.future = {x = future_dir.x, y = future_dir.y}
  end

  return lane_model
end

local function buildAssistInfo(params, lane_model)
  local steer_limit = params.steer_limit or 0.15
  return {
    active = false,
    available = lane_model ~= nil,
    steering = {
      limit = steer_limit,
      target = 0,
      pid = 0,
      feedforward = 0,
      weight = 0,
      final = 0,
      driver = 0,
      applied = 0,
      error = lane_model and lane_model.offset.normalized or 0,
      headingError = lane_model and lane_model.heading.error or 0,
      curvature = lane_model and lane_model.curvature or 0
    },
    warning = false,
    driverOverride = false
  }
end

local function update(dt, veh, system_params, enabled)
  activation_grace_timer = max(activation_grace_timer - (dt or 0), 0)
  updateDoubleChime(dt)

  local params = (system_params and system_params.lane_centering_params) or {}
  local steer_limit = params.steer_limit or 0.15
  local disable_threshold = params.override_threshold or 0.2
  local assist_weight_gain = params.assist_weight_gain or 4.0
  local min_active_speed = params.min_active_speed or 1.0
  local activation_grace_period = max(params.activation_override_grace or 1.0, 0)

  local prev_status = latest_data and latest_data.status or nil
  local previously_active = prev_status and prev_status.active
  local previously_enabled = prev_status and prev_status.enabled

  local installed = extra_utils.getPart("lane_centering_assist_system_angelo234")
    or extra_utils.getPart("lane_centering_assist_angelo234")

  local status = {
    installed = installed and true or false,
    enabled = installed and enabled or false,
    active = false,
    available = false,
    reason = nil,
    driverOverride = false,
    aiTrafficControlsSpeed = false
  }

  if not installed then
    lane_state.prev_wps = nil
    lane_state.desired_offset = nil
    lane_state.lane_width = nil
    lane_state.prev_path_nodes = nil
    warning_played = false
    latest_data = {status = status}
    resetControllers()
    return
  end

  if not veh then
    latest_data = {status = status}
    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then
    latest_data = {status = status}
    return
  end

  local lane_model = computeLaneModel(veh_props, params)
  if not lane_model then
    status.reason = "no_lane_data"
    warning_played = false
    resetControllers()
  else
    status.available = true
  end

  local assist_info = buildAssistInfo(params, lane_model)
  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  local user_enabled = status.enabled
  local assist_low_speed_shutdown = false

  if previously_active and previously_enabled and enabled and forward_speed <= min_active_speed then
    assist_low_speed_shutdown = true
    status.reason = "low_speed"
    warning_played = false
    queueDoubleChime()
    if activation_handler then
      activation_handler(false, "low_speed")
    end
    status.enabled = false
    user_enabled = false
    resetControllers()
  end

  local driver_axis = rawget(_G, "input_steering_driver_angelo234")
  local driver_input = driver_axis or rawget(_G, "input_steering_angelo234")
  if driver_input == nil then
    local estimated = 0
    if electrics_values_angelo234 then
      estimated = (electrics_values_angelo234["steering_input"] or 0) - last_assist_delta
    end
    driver_input = estimated
  end
  driver_input = clamp(driver_input or 0, -1, 1)
  assist_info.steering.driver = driver_input

  if abs(driver_input) > (disable_threshold * 0.6) then
    lane_state.prev_path_nodes = nil
  end

  if not user_enabled then
    if not assist_low_speed_shutdown then
      status.reason = "user_disabled"
    end
  elseif lane_model and forward_speed <= min_active_speed then
    status.reason = "low_speed"
  elseif not lane_model then
    status.reason = status.reason or "no_lane_data"
  end

  local ai_mode_active = rawget(_G, "lane_centering_ai_mode_active_angelo234") and true or false
  local ai_speed_control_active = rawget(_G, "lane_centering_ai_speed_control_active_angelo234") and true or false
  local assist_ready = user_enabled and lane_model ~= nil and forward_speed > min_active_speed
  local newly_active = assist_ready and not previously_active

  if newly_active then
    activation_grace_timer = activation_grace_period
  end

  local ignoring_driver_override = activation_grace_timer > 0

  local override_value = driver_axis ~= nil and clamp(driver_axis, -1, 1) or driver_input
  if assist_ready and not ignoring_driver_override and abs(override_value) > disable_threshold then
    status.driverOverride = true
    status.reason = "driver_override"
    assist_ready = false
    warning_played = false
    lane_state.prev_path_nodes = nil
    resetControllers()
    queueDoubleChime()
    if activation_handler then
      activation_handler(false, "driver_override")
    end
  end

  if assist_ready then
    ensureControllers(params, steer_limit)
    local half_width = (lane_model.width or 0) * 0.5
    local norm_error = lane_model.offset.normalized or 0
    local heading_error = lane_model.heading.error or 0
    local heading_kp = params.heading_kp or 0.6
    local curvature_ff = params.curvature_feedforward or 0.45

    local combined_error = norm_error + heading_kp * heading_error
    local pid_out = steering_pid:get(combined_error, 0, dt)
    local feedforward = curvature_ff * (lane_model.curvature or 0)
    local raw_target = pid_out + feedforward
    local smoothed = steering_smoother:getUncapped(raw_target, dt)
    local target = clamp(smoothed, -steer_limit, steer_limit)

    local assist_weight = clamp(1 - abs(driver_input) * assist_weight_gain, 0, 1)
    local blended = target * assist_weight
    local final = clamp(driver_input + blended, -1, 1)

    local applied_delta = final - driver_input

    last_assist_delta = 0

    assist_info.active = true
    assist_info.steering.target = target
    assist_info.steering.pid = pid_out
    assist_info.steering.feedforward = feedforward
    assist_info.steering.weight = assist_weight
    assist_info.steering.final = final
    assist_info.steering.error = norm_error
    assist_info.steering.headingError = heading_error
    assist_info.steering.curvature = lane_model.curvature or 0
    assist_info.steering.applied = applied_delta

    status.active = true
    status.reason = nil
  else
    resetControllers()
    activation_grace_timer = 0
  end

  if newly_active then
    queueDoubleChime()
    if activation_handler then
      activation_handler(true, "speed_ready")
    end
  end

  if lane_model then
    local half_width = (lane_model.width or 0) * 0.5
    if half_width > 0 then
      local warn_zone = (params.warning_ratio or 0.8) * half_width
      local abs_offset = abs(lane_model.offset.error or 0)
      if abs_offset > warn_zone and abs_offset <= half_width then
        assist_info.warning = true
        if not warning_played then
          Engine.Audio.playOnce('AudioGui', 'art/sound/proximity_tone_50ms_moderate.wav')
          warning_played = true
        end
      else
        assist_info.warning = false
        warning_played = false
      end
    else
      assist_info.warning = false
      warning_played = false
    end
  else
    assist_info.warning = false
    warning_played = false
  end

  assist_info.driverOverride = status.driverOverride
  assist_info.aiTrafficActive = ai_mode_active
  assist_info.aiTrafficControlsSpeed = ai_speed_control_active
  status.aiTrafficActive = ai_mode_active
  status.aiTrafficControlsSpeed = ai_speed_control_active

  latest_data = {
    status = status,
    lane = lane_model,
    assist = assist_info
  }
end

local function getLaneData()
  return latest_data
end

local function setActivationCallback(cb)
  activation_handler = cb
end

local function playActivationChime()
  queueDoubleChime()
end

local function playDeactivationChime()
  queueDoubleChime()
end

M.update = update
M.getLaneData = getLaneData
M.setActivationCallback = setActivationCallback
M.playActivationChime = playActivationChime
M.playDeactivationChime = playDeactivationChime

return M
