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
  prev_path_nodes = nil,
  traffic_side = nil,
  travel_sign = nil,
  offroad = false,
  last_road_distance = nil,
  offroad_confirm_timer = 0,
  offroad_release_timer = 0,
  last_lane_model = nil,
  lane_loss_timer = 0
}

local AI_ROUTE_REQUEST_INTERVAL = 0.8
local AI_ROUTE_REQUEST_COMMAND = [[
local routeData = nil
if ai and ai.dumpCurrentRoute and type(debug) == 'table' and type(debug.getupvalue) == 'function' then
  local idx = 1
  while true do
    local name, value = debug.getupvalue(ai.dumpCurrentRoute, idx)
    if not name then break end
    if name == 'currentRoute' then
      routeData = value
      break
    end
    idx = idx + 1
  end
end

if routeData and routeData.path and #routeData.path > 1 then
  local simplified = {}
  for i = 1, #routeData.path do
    local node = routeData.path[i]
    if type(node) == 'number' then
      simplified[#simplified + 1] = node
    end
  end

  local totalCount = #simplified
  local limit = 240
  if totalCount > limit then
    local trimmed = {}
    for i = 1, limit do
      trimmed[#trimmed + 1] = simplified[i]
    end
    simplified = trimmed
  end

  local ok, encoded = pcall(jsonEncode, {path = simplified, total = totalCount})
  if ok and encoded then
    obj:queueGameEngineLua(string.format('extensions.driver_assistance_angelo234.receiveLaneCenteringAiRoute(%q)', encoded))
  else
    obj:queueGameEngineLua('extensions.driver_assistance_angelo234.receiveLaneCenteringAiRoute(nil)')
  end
else
  obj:queueGameEngineLua('extensions.driver_assistance_angelo234.receiveLaneCenteringAiRoute(nil)')
end
]]

local ai_route_state = {
  nodes = nil,
  total_nodes = 0,
  truncated = false,
  seq = 0,
  updated_at = 0,
  timer = 0,
  pending = false,
  pending_timer = 0
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

local function chooseLegalOffset(offsets, lane_width, traffic_side, orientation_sign)
  if not offsets or #offsets == 0 or not traffic_side then return nil end
  local target_sign = traffic_side == 'left' and -1 or 1
  if orientation_sign and orientation_sign < 0 then
    target_sign = -target_sign
  end
  local reference = nil
  if lane_width and lane_width > 0 then
    reference = target_sign * lane_width * 0.5
  end

  local best = nil
  local best_dist = huge
  for _, offset in ipairs(offsets) do
    if (target_sign < 0 and offset < 0) or (target_sign > 0 and offset > 0) then
      local dist = reference and abs(offset - reference) or abs(offset)
      if dist < best_dist then
        best = offset
        best_dist = dist
      end
    end
  end

  if best then
    return best
  end

  if reference then
    return selectOffset(offsets, reference)
  end

  return selectOffset(offsets, 0)
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

  local dir_vec = veh_props.dir
  local dirx, diry, dirz = 0, 1, 0
  if dir_vec then
    local len = sqrt(dir_vec.x * dir_vec.x + dir_vec.y * dir_vec.y + dir_vec.z * dir_vec.z)
    if len > 1e-6 then
      dirx, diry, dirz = dir_vec.x / len, dir_vec.y / len, dir_vec.z / len
    end
  end

  local right_vec = veh_props.dir_right
  local rightx, righty, rightz = diry, -dirx, 0
  if right_vec then
    local len = sqrt(right_vec.x * right_vec.x + right_vec.y * right_vec.y + right_vec.z * right_vec.z)
    if len > 1e-6 then
      rightx, righty, rightz = right_vec.x / len, right_vec.y / len, right_vec.z / len
    end
  end

  local fallback_norm = {x = rightx, y = righty, z = rightz}

  for i = 1, #samples do
    local sample = samples[i]
    local normal = normals[i] or fallback_norm
    local nx, ny, nz = normal.x, normal.y, normal.z
    local nlen = sqrt(nx * nx + ny * ny + nz * nz)
    if nlen > 1e-6 then
      nx, ny, nz = nx / nlen, ny / nlen, nz / nlen
    else
      nx, ny, nz = rightx, righty, rightz
    end

    local center_rel = sample - veh_pos
    local left_point = vec3(sample.x - nx * half_width, sample.y - ny * half_width, sample.z)
    local right_point = vec3(sample.x + nx * half_width, sample.y + ny * half_width, sample.z)

    local left_rel = left_point - veh_pos
    local right_rel = right_point - veh_pos

    center_local[#center_local + 1] = {
      x = center_rel.x * rightx + center_rel.y * righty + center_rel.z * rightz,
      y = center_rel.x * dirx + center_rel.y * diry + center_rel.z * dirz
    }
    left_local[#left_local + 1] = {
      x = left_rel.x * rightx + left_rel.y * righty + left_rel.z * rightz,
      y = left_rel.x * dirx + left_rel.y * diry + left_rel.z * dirz
    }
    right_local[#right_local + 1] = {
      x = right_rel.x * rightx + right_rel.y * righty + right_rel.z * rightz,
      y = right_rel.x * dirx + right_rel.y * diry + right_rel.z * dirz
    }
  end

  return center_local, left_local, right_local
end

local function requestAiRouteData(veh)
  if not veh or ai_route_state.pending then return false end
  veh:queueLuaCommand(AI_ROUTE_REQUEST_COMMAND)
  ai_route_state.pending = true
  ai_route_state.pending_timer = 0
  return true
end

local function updateAiRouteData(payload)
  ai_route_state.pending = false
  ai_route_state.pending_timer = 0
  ai_route_state.seq = ai_route_state.seq + 1
  ai_route_state.updated_at = os.time and os.time() or 0

  if type(payload) ~= 'string' or payload == '' then
    ai_route_state.nodes = nil
    ai_route_state.total_nodes = 0
    ai_route_state.truncated = false
    return
  end

  local ok, data = pcall(jsonDecode, payload)
  if not ok or type(data) ~= 'table' then return end

  local path = data.path
  if type(path) ~= 'table' then
    ai_route_state.nodes = nil
    ai_route_state.total_nodes = 0
    ai_route_state.truncated = false
    return
  end

  local cleaned = {}
  for i = 1, #path do
    local node = tonumber(path[i])
    if node then
      cleaned[#cleaned + 1] = node
    end
  end

  if #cleaned >= 2 then
    ai_route_state.nodes = cleaned
    ai_route_state.total_nodes = tonumber(data.total) or #cleaned
    ai_route_state.truncated = ai_route_state.total_nodes > #cleaned
  else
    ai_route_state.nodes = nil
    ai_route_state.total_nodes = 0
    ai_route_state.truncated = false
  end
end

local function buildAiRoutePreview(veh_props, initial_wps, desired_offset, params)
  if not ai_route_state.nodes or not initial_wps then return nil end

  local nodes = ai_route_state.nodes
  if #nodes < 2 then return nil end

  local start_node = initial_wps.start_wp
  local end_node = initial_wps.end_wp
  if not start_node or not end_node then return nil end

  local start_idx = nil
  for i = 1, #nodes - 1 do
    if nodes[i] == start_node and nodes[i + 1] == end_node then
      start_idx = i
      break
    end
  end

  if not start_idx then
    for i = 1, #nodes do
      if nodes[i] == start_node then
        start_idx = i
        break
      end
    end
  end

  if not start_idx then return nil end

  local node_limit = params.route_preview_node_limit or 120
  if node_limit < 3 then node_limit = 3 end
  local end_idx = min(#nodes, start_idx + node_limit - 1)
  if end_idx <= start_idx then
    end_idx = min(#nodes, start_idx + 1)
  end

  local subset = {}
  for i = start_idx, end_idx do
    subset[#subset + 1] = nodes[i]
  end
  if #subset < 2 then return nil end

  local segments = buildSegmentsFromNodeList(subset, veh_props)
  if not segments or #segments == 0 then return nil end

  local preview_length = params.route_preview_length
    or ((params.lookahead_max or 120) + (params.branch_lookahead or 60))
  preview_length = clamp(preview_length, 40, params.route_preview_max or 600)

  local geometry = buildPathGeometryFromSegments(veh_props, segments, desired_offset, params, preview_length)
  if not geometry then return nil end

  geometry.nodes = subset
  geometry.start_index = start_idx
  geometry.preview_nodes = #subset
  geometry.total_nodes = ai_route_state.total_nodes or #nodes
  geometry.seq = ai_route_state.seq
  geometry.truncated = geometry.truncated or (geometry.preview_nodes < geometry.total_nodes)
  return geometry
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

local function selectStraightThroughNeighbor(initial_wps, veh_props, params)
  if not initial_wps or not veh_props then return nil end

  local start_node = initial_wps.start_wp
  local end_node = initial_wps.end_wp
  if not end_node then return nil end

  local neighbors = extra_utils.getGraphLinks(end_node)
  if not neighbors then return nil end

  local params_table = params or {}
  local min_align = params_table.prefer_straight_min_align or 0
  local gate_penalty = params_table.prefer_straight_gate_penalty or 0
  local fallback_align = params_table.prefer_straight_fallback_align
  if fallback_align == nil then
    fallback_align = min_align
    if fallback_align > 0 then
      fallback_align = fallback_align * 0.5
    end
  end
  fallback_align = clamp(fallback_align, -1, 1)
  local score_margin = params_table.prefer_straight_score_margin or 0.15
  if score_margin < 0 then score_margin = 0 end

  if min_align > 1 then min_align = 1 end
  if min_align < -1 then min_align = -1 end

  local end_pos = extra_utils.getWaypointPosition(end_node)
  if not end_pos then return nil end
  end_pos = vec3(end_pos.x, end_pos.y, veh_props.center_pos.z)

  local base_dir = nil
  if start_node then
    local start_pos = extra_utils.getWaypointPosition(start_node)
    if start_pos then
      start_pos = vec3(start_pos.x, start_pos.y, veh_props.center_pos.z)
      base_dir = extra_utils.toNormXYVec(end_pos - start_pos)
      if base_dir:length() < 1e-6 then
        base_dir = nil
      end
    end
  end
  if not base_dir then
    base_dir = extra_utils.toNormXYVec(veh_props.dir)
    if base_dir:length() < 1e-6 then
      base_dir = nil
    end
  end
  if not base_dir then return nil end

  local best_neighbor = nil
  local best_align = nil
  local best_score = -huge
  local second_score = nil

  for neighbor, link in pairs(neighbors) do
    if neighbor ~= start_node then
      local link_data = link or extra_utils.getGraphLinkData(end_node, neighbor)
      if link_data then
        if link_data.oneWay and link_data.inNode and link_data.inNode ~= end_node then
          goto continue
        end
        if link_data.drivability and link_data.drivability <= 0 then
          goto continue
        end

        local neighbor_pos = extra_utils.getWaypointPosition(neighbor)
        if neighbor_pos then
          neighbor_pos = vec3(neighbor_pos.x, neighbor_pos.y, veh_props.center_pos.z)
          local delta = neighbor_pos - end_pos
          if delta:squaredLength() > 1e-6 then
            local direction = extra_utils.toNormXYVec(delta)
            if direction:length() >= 1e-6 then
              local align = direction:dot(base_dir)
              local score = align
              if link_data.gated and link_data.gated ~= 0 then
                score = score - gate_penalty
              end

              if score > best_score then
                if best_neighbor ~= nil then
                  second_score = best_score
                end
                best_score = score
                best_neighbor = neighbor
                best_align = align
              elseif second_score == nil or score > second_score then
                second_score = score
              end
            end
          end
        end
      end
    end
    ::continue::
  end

  if best_neighbor and best_align then
    if best_align >= min_align then
      return best_neighbor, best_align
    end

    if best_align > 0 and best_align >= fallback_align then
      if not second_score or best_score >= (second_score + score_margin) then
        return best_neighbor, best_align
      end
    end
  end

  return nil
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
    local straight_neighbor = selectStraightThroughNeighbor(initial_wps, veh_props, params)
    if straight_neighbor then
      state = {state_node, straight_neighbor}
    else
      state = extra_utils.toNormXYVec(veh_props.dir)
    end
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

  local road_rules = extra_utils.getRoadRules and extra_utils.getRoadRules() or nil
  local right_hand_drive = nil
  local traffic_side = nil
  if road_rules and road_rules.rightHandDrive ~= nil then
    right_hand_drive = road_rules.rightHandDrive and true or false
    traffic_side = right_hand_drive and 'left' or 'right'
  end
  if not traffic_side and extra_utils.getTrafficSide then
    traffic_side = extra_utils.getTrafficSide()
  end
  if traffic_side and lane_state.traffic_side ~= traffic_side then
    lane_state.traffic_side = traffic_side
    lane_state.desired_offset = nil
  elseif traffic_side == nil and lane_state.traffic_side ~= traffic_side then
    lane_state.traffic_side = traffic_side
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

  local vehicle_dir = extra_utils.toNormXYVec(veh_props.dir)
  local orientation_sign = nil
  if vehicle_dir:length() >= 1e-6 then
    local alignment = lane_dir:dot(vehicle_dir)
    if alignment > 0.15 then
      orientation_sign = 1
    elseif alignment < -0.15 then
      orientation_sign = -1
    end
  end
  if orientation_sign and lane_state.travel_sign ~= orientation_sign then
    lane_state.travel_sign = orientation_sign
    lane_state.desired_offset = nil
  elseif orientation_sign then
    lane_state.travel_sign = orientation_sign
  end

  local lane_right = vec3(lane_dir.y, -lane_dir.x, 0)
  local current_offset = (veh_xy - lane_point_xy):dot(lane_right)

  local desired_offset = selectOffset(offsets, lane_state.desired_offset or current_offset)
  local legal_offset = chooseLegalOffset(offsets, lane_width, traffic_side, orientation_sign)
  if not lane_state.desired_offset and legal_offset then
    desired_offset = legal_offset
  end

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

  local route_preview = buildAiRoutePreview(veh_props, wps, lane_state.desired_offset, params)

  local lane_dir_norm = path.dir
  if not lane_dir_norm or lane_dir_norm:length() < 1e-6 then
    lane_dir_norm = extra_utils.toNormXYVec(wps.end_wp_pos - wps.start_wp_pos)
  end
  local heading_dir = vehicle_dir
  if heading_dir:length() < 1e-6 then
    heading_dir = extra_utils.toNormXYVec(veh_props.dir)
    vehicle_dir = heading_dir
  end
  local heading_error = heading_dir:cross(lane_dir_norm).z
  local half_width = (path.lane_width or lane_width) * 0.5
  local target_offset = path.offset or desired_offset
  local lateral_error = current_offset - target_offset
  local normalized_error = 0
  if half_width > 1e-3 then
    normalized_error = clamp(lateral_error / half_width, -5, 5)
  end

  local legal_error = nil
  local legal_normalized = nil
  if legal_offset then
    legal_error = current_offset - legal_offset
    if half_width > 1e-3 then
      legal_normalized = clamp(legal_error / half_width, -5, 5)
    end
  end

  local future_dir = path.future_dir
  local curvature = path.curvature or 0

  local route_info = nil
  if route_preview then
    route_info = {
      center = route_preview.center,
      left = route_preview.left,
      right = route_preview.right,
      length = route_preview.length or 0,
      coveredLength = route_preview.covered_length or route_preview.length or 0,
      targetLength = route_preview.target_length or 0,
      nodes = route_preview.nodes,
      startIndex = route_preview.start_index,
      previewNodes = route_preview.preview_nodes or (route_preview.nodes and #route_preview.nodes) or 0,
      totalNodes = route_preview.total_nodes or (ai_route_state.nodes and #ai_route_state.nodes) or 0,
      truncated = route_preview.truncated or false,
      seq = route_preview.seq or ai_route_state.seq,
      updatedAt = ai_route_state.updated_at
    }
  end

  local lane_model = {
    width = path.lane_width or lane_width,
    offset = {
      current = current_offset,
      target = target_offset,
      error = lateral_error,
      normalized = normalized_error,
      legal = legal_offset,
      legalError = legal_error,
      legalNormalized = legal_normalized
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
      offset = path.offset or desired_offset,
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
      nodes = path.nodes,
      routePreview = route_info
    },
    speed = veh_props.speed,
    xnorm = xnorm,
    route = route_info
  }

  if right_hand_drive ~= nil or traffic_side then
    lane_model.roadRules = {
      rightHandDrive = right_hand_drive,
      trafficSide = traffic_side
    }
  end

  if future_dir then
    lane_model.heading.future = {x = future_dir.x, y = future_dir.y}
  end

  if offsets and #offsets > 0 then
    local lane_offsets = {}
    for i = 1, #offsets do
      lane_offsets[i] = offsets[i]
    end

    local selected_offset = path.offset or desired_offset or current_offset or 0
    local selected_index = nil
    local legal_index = nil
    local best_sel = huge
    local best_legal = huge
    for i = 1, #lane_offsets do
      local offset_value = lane_offsets[i]
      if offset_value then
        local sel_dist = abs((selected_offset or 0) - offset_value)
        if sel_dist < best_sel then
          best_sel = sel_dist
          selected_index = i
        end
        if legal_offset ~= nil then
          local legal_dist = abs(legal_offset - offset_value)
          if legal_dist < best_legal then
            best_legal = legal_dist
            legal_index = i
          end
        end
      end
    end

    lane_model.lanes = {
      width = path.lane_width or lane_width,
      count = #lane_offsets,
      offsets = lane_offsets,
      selectedOffset = selected_offset,
      legalOffset = legal_offset,
      currentOffset = current_offset,
      selectedIndex = selected_index,
      legalIndex = legal_index,
      trafficSide = traffic_side,
      orientation = orientation_sign
    }
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
  local dt_value = dt or 0
  activation_grace_timer = max(activation_grace_timer - dt_value, 0)
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
    aiTrafficControlsSpeed = false,
    offRoad = false,
    roadDistance = nil,
    offRoadThreshold = nil,
    onRoadThreshold = nil,
    laneDataHold = false,
    laneDataFresh = false,
    laneDataGrace = 0
  }

  if not installed then
    lane_state.prev_wps = nil
    lane_state.desired_offset = nil
    lane_state.lane_width = nil
    lane_state.prev_path_nodes = nil
    lane_state.offroad = false
    lane_state.last_road_distance = nil
    lane_state.offroad_confirm_timer = 0
    lane_state.offroad_release_timer = 0
    lane_state.last_lane_model = nil
    lane_state.lane_loss_timer = 0
    ai_route_state.nodes = nil
    ai_route_state.total_nodes = 0
    ai_route_state.truncated = false
    ai_route_state.pending = false
    ai_route_state.timer = 0
    ai_route_state.pending_timer = 0
    warning_played = false
    latest_data = {status = status}
    resetControllers()
    return
  end

  if not veh then
    lane_state.last_lane_model = nil
    lane_state.lane_loss_timer = 0
    latest_data = {status = status}
    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then
    lane_state.last_lane_model = nil
    lane_state.lane_loss_timer = 0
    latest_data = {status = status}
    return
  end

  local lane_model_raw = computeLaneModel(veh_props, params)
  local lane_data_grace = params.lane_data_grace or 0.9
  if lane_data_grace < 0 then lane_data_grace = 0 end
  local lane_loss_timer = lane_state.lane_loss_timer or 0
  local lane_model = lane_model_raw
  if lane_model_raw then
    lane_state.last_lane_model = lane_model_raw
    lane_loss_timer = 0
  else
    lane_loss_timer = lane_loss_timer + dt_value
    if lane_state.last_lane_model and lane_loss_timer <= lane_data_grace then
      lane_model = lane_state.last_lane_model
    else
      lane_model = nil
      if lane_loss_timer > lane_data_grace then
        lane_state.last_lane_model = nil
      end
    end
  end
  lane_state.lane_loss_timer = lane_loss_timer
  local lane_model_valid = lane_model_raw ~= nil
  local lane_hold_active = (not lane_model_valid) and lane_model ~= nil
  if lane_model then
    lane_model.holdActive = lane_hold_active or nil
    lane_model.fresh = lane_model_valid or nil
  end
  status.laneDataGrace = lane_data_grace

  local lane_width_hint = params.default_lane_width or 3.75
  if lane_state.lane_width and lane_state.lane_width > 0 then
    lane_width_hint = lane_state.lane_width
  end
  if lane_model and lane_model.width and lane_model.width > 0 then
    lane_width_hint = lane_model.width
  end

  local offroad_ratio = clamp(params.offroad_distance_ratio or 0.65, 0, 2)
  local offroad_release_ratio = clamp(params.offroad_distance_release_ratio or 0.5, 0, offroad_ratio)
  local offroad_min_distance = params.offroad_distance_min or 1.5
  local offroad_release_min = params.offroad_distance_release_min or (offroad_min_distance * 0.5)
  if offroad_release_min > offroad_min_distance then
    offroad_release_min = offroad_min_distance * 0.8
  end

  local road_distance_sample = extra_utils.getClosestRoadDistance(veh_props.center_pos)
  local road_distance = nil
  local distance_valid = road_distance_sample ~= nil
  if distance_valid then
    road_distance = abs(road_distance_sample)
    lane_state.last_road_distance = road_distance
  else
    road_distance = lane_state.last_road_distance
  end

  local offroad_disable_threshold = max(offroad_min_distance, lane_width_hint * offroad_ratio)
  local offroad_enable_threshold = max(offroad_release_min, lane_width_hint * offroad_release_ratio)
  if offroad_enable_threshold > offroad_disable_threshold then
    offroad_enable_threshold = offroad_disable_threshold * 0.8
  end
  if offroad_enable_threshold < 0 then offroad_enable_threshold = 0 end

  status.roadDistance = road_distance
  status.offRoadThreshold = offroad_disable_threshold
  status.onRoadThreshold = offroad_enable_threshold

  local prev_offroad = lane_state.offroad and true or false
  local offroad = prev_offroad
  local confirm_timer = lane_state.offroad_confirm_timer or 0
  local release_timer = lane_state.offroad_release_timer or 0
  local confirm_time = max(params.offroad_confirm_time or 0.35, 0)
  local release_time = max(params.offroad_release_time or 0.5, 0)
  local offroad_distance_margin = params.offroad_distance_margin
  if offroad_distance_margin == nil then
    offroad_distance_margin = lane_width_hint * 0.15
  end
  if offroad_distance_margin < 0 then
    offroad_distance_margin = 0
  end

  local maintain_onroad = (lane_model and (not distance_valid or (road_distance and road_distance < offroad_disable_threshold)))
    or (lane_hold_active and (not distance_valid or (road_distance and road_distance < offroad_disable_threshold)))

  if maintain_onroad then
    offroad = false
    confirm_timer = 0
    if prev_offroad then
      release_timer = 0
    end
  elseif road_distance and road_distance >= offroad_disable_threshold then
    if not prev_offroad then
      local allow_confirm = false
      if not lane_model_valid then
        if lane_loss_timer > lane_data_grace or road_distance >= (offroad_disable_threshold + offroad_distance_margin * 2) then
          allow_confirm = true
        end
      else
        if road_distance >= (offroad_disable_threshold + offroad_distance_margin) then
          allow_confirm = true
        end
      end

      if allow_confirm then
        confirm_timer = confirm_timer + dt_value
        if confirm_timer >= confirm_time then
          offroad = true
          release_timer = 0
        end
      else
        confirm_timer = max(confirm_timer - dt_value, 0)
      end
    else
      release_timer = max(release_timer - dt_value, 0)
    end
  else
    if prev_offroad then
      if road_distance and road_distance <= offroad_enable_threshold then
        release_timer = release_timer + dt_value
        if release_timer >= release_time then
          offroad = false
          confirm_timer = 0
        end
      else
        release_timer = max(release_timer - dt_value, 0)
      end
    else
      confirm_timer = max(confirm_timer - dt_value, 0)
    end
  end

  lane_state.offroad = offroad
  lane_state.offroad_confirm_timer = confirm_timer
  lane_state.offroad_release_timer = release_timer
  status.offRoad = offroad
  status.laneDataHold = lane_hold_active
  status.laneDataFresh = lane_model_valid

  if offroad and not prev_offroad then
    lane_state.prev_wps = nil
    lane_state.desired_offset = nil
    lane_state.prev_path_nodes = nil
    lane_state.lane_width = nil
    lane_state.last_lane_model = nil
    ai_route_state.nodes = nil
    ai_route_state.total_nodes = 0
    ai_route_state.truncated = false
    ai_route_state.pending = false
    ai_route_state.pending_timer = 0
    ai_route_state.timer = 0
  elseif not offroad and prev_offroad then
    lane_state.offroad_confirm_timer = 0
    lane_state.offroad_release_timer = 0
  end

  if offroad then
    lane_model = nil
  elseif lane_model then
    status.available = true
  end

  local lane_ready = lane_model ~= nil
  if not lane_ready then
    if offroad then
      status.reason = "off_road"
    else
      status.reason = "no_lane_data"
    end
    warning_played = false
  end

  local assist_info = buildAssistInfo(params, lane_model)
  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  local user_enabled = status.enabled
  local assist_low_speed_shutdown = false
  local offroad_shutdown = false

  if previously_active and previously_enabled and enabled and forward_speed <= min_active_speed then
    assist_low_speed_shutdown = true
    if not offroad then
      status.reason = "low_speed"
    end
    warning_played = false
    queueDoubleChime()
    if activation_handler then
      activation_handler(false, "low_speed")
    end
    status.enabled = false
    user_enabled = false
    resetControllers()
  end

  if offroad then
    if user_enabled then
      offroad_shutdown = true
      status.reason = "off_road"
      queueDoubleChime()
      if activation_handler then
        activation_handler(false, "off_road")
      end
    end
    status.enabled = false
    user_enabled = false
    resetControllers()
  elseif not lane_ready then
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
    if status.reason == "off_road" then
      -- keep off-road reason
    elseif not assist_low_speed_shutdown and not offroad_shutdown then
      status.reason = "user_disabled"
    end
  elseif offroad then
    status.reason = "off_road"
    if not assist_low_speed_shutdown then
      warning_played = false
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

  if ai_mode_active and veh and not offroad then
    local interval = params.ai_route_refresh_interval or AI_ROUTE_REQUEST_INTERVAL
    if ai_route_state.pending then
      ai_route_state.pending_timer = ai_route_state.pending_timer + (dt or 0)
      local pending_timeout = params.ai_route_timeout or max(interval * 2.5, 2.5)
      if ai_route_state.pending_timer >= pending_timeout then
        ai_route_state.pending = false
        ai_route_state.pending_timer = 0
      end
    end

    ai_route_state.timer = ai_route_state.timer + (dt or 0)
    local need_request = newly_active or ai_route_state.timer >= interval
    if not ai_route_state.nodes or (lane_model and not lane_model.route) then
      need_request = true
    end
    if need_request then
      if requestAiRouteData(veh) then
        ai_route_state.timer = 0
      end
    end
  else
    ai_route_state.timer = 0
    ai_route_state.pending = false
    ai_route_state.pending_timer = 0
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

local function isVehicleOnRoad()
  return lane_state.offroad ~= true
end

local function getRoadDistance()
  return lane_state.last_road_distance
end

M.update = update
M.getLaneData = getLaneData
M.setActivationCallback = setActivationCallback
M.playActivationChime = playActivationChime
M.playDeactivationChime = playDeactivationChime
M.updateAiRouteData = updateAiRouteData
M.isVehicleOnRoad = isVehicleOnRoad
M.getRoadDistance = getRoadDistance

return M
