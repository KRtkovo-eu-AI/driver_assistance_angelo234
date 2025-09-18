-- luacheck: globals ui_message Engine electrics_values_angelo234 map

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local latest_data = {status = 'OFF'}
local steering_pid
local steering_smooth
local warning_played = false

local function resetController()
  steering_pid = nil
  steering_smooth = nil
  warning_played = false
end

local function setLatestStatus(status)
  latest_data = {status = status or latest_data.status}
end

local function canTravel(fromNode, toNode, linkData)
  if not linkData then return true end
  if linkData.oneWay then
    return linkData.inNode == fromNode and linkData.outNode == toNode
  end
  return true
end

local function orientNodes(nodes, n1, n2, vehicleDir)
  local start = n1
  local nextNode = n2
  local p1 = nodes[start] and nodes[start].pos
  local p2 = nodes[nextNode] and nodes[nextNode].pos
  if not (p1 and p2) then return nil end

  local dir = (p2 - p1):normalized()
  if dir:dot(vehicleDir) < 0 then
    start, nextNode = nextNode, start
    p1, p2 = p2, p1
    dir = -dir
  end

  local link = nodes[start] and nodes[start].links and nodes[start].links[nextNode]
  if link and not canTravel(start, nextNode, link) then
    if canTravel(nextNode, start, link) then
      start, nextNode = nextNode, start
      p1 = nodes[start].pos
      p2 = nodes[nextNode].pos
      dir = (p2 - p1):normalized()
      link = nodes[start].links and nodes[start].links[nextNode]
      if link and not canTravel(start, nextNode, link) then return nil end
    else
      return nil
    end
  end

  return start, nextNode, dir, link
end

local function chooseNextNode(nodes, prevNode, currentNode, baseDir)
  local current = nodes[currentNode]
  if not current or not current.links then return nil end

  local bestNode
  local bestScore = -math.huge
  local currentPos = current.pos

  for neighbor, data in pairs(current.links) do
    if neighbor ~= prevNode and nodes[neighbor] then
      if canTravel(currentNode, neighbor, data) then
        local dir = (nodes[neighbor].pos - currentPos):normalized()
        local score = dir:dot(baseDir)
        if data and data.oneWay and data.inNode == currentNode and data.outNode == neighbor then
          score = score + 0.25
        end
        if score > bestScore then
          bestScore = score
          bestNode = neighbor
        end
      end
    end
  end

  return bestNode
end

local function buildPath(nodes, startNode, nextNode, startPos, lookaheadDist)
  local points = {startPos}
  local total = 0
  local prev = startNode
  local current = nextNode
  local previousPos = startPos
  local maxSegments = 24

  while current and maxSegments > 0 do
    maxSegments = maxSegments - 1
    local currentPos = nodes[current] and nodes[current].pos
    if not currentPos then break end

    points[#points + 1] = vec3(currentPos)
    local segLen = (currentPos - previousPos):length()
    total = total + segLen
    previousPos = currentPos

    if total >= lookaheadDist then break end

    local baseDir = (currentPos - (nodes[prev] and nodes[prev].pos or currentPos)):normalized()
    local nextHop = chooseNextNode(nodes, prev, current, baseDir)
    if not nextHop then break end

    prev, current = current, nextHop
  end

  return points, total
end

local function toLocal(point, origin, right, forward)
  local rel = point - origin
  return {
    x = rel:dot(right),
    y = rel:dot(forward)
  }
end

local function computeCurvature(path)
  if #path < 3 then return 0 end
  local p0 = path[1]
  local p1 = path[2]
  local p2 = path[3]
  local v1x, v1y = p1.x - p0.x, p1.y - p0.y
  local v2x, v2y = p2.x - p1.x, p2.y - p1.y
  local len1 = math.sqrt(v1x * v1x + v1y * v1y)
  local len2 = math.sqrt(v2x * v2x + v2y * v2y)
  if len1 < 1e-6 or len2 < 1e-6 then return 0 end
  local cross = v1x * v2y - v1y * v2x
  local dot = v1x * v2x + v1y * v2y
  local angle = math.atan2(cross, dot)
  local avgLen = (len1 + len2) * 0.5
  if avgLen < 1e-6 then return 0 end
  return angle / avgLen
end

local function computeTargetPoint(points, lookahead)
  if #points < 2 then return points[#points], (points[#points] - points[#points]) end
  local remaining = lookahead
  local prev = points[1]
  for i = 2, #points do
    local current = points[i]
    local seg = current - prev
    local segLen = seg:length()
    if segLen >= remaining then
      local t = remaining / math.max(segLen, 1e-9)
      return prev + seg * t, seg:normalized()
    end
    remaining = remaining - segLen
    prev = current
  end
  local lastDir = (points[#points] - points[#points - 1])
  return points[#points], lastDir:length() > 1e-6 and lastDir:normalized() or lastDir
end

local function computeLaneWidth(nodes, startNode, nextNode, link)
  local radiusStart = nodes[startNode] and nodes[startNode].radius or 3.75
  local radiusEnd = nodes[nextNode] and nodes[nextNode].radius or radiusStart
  local avgRadius = (radiusStart + radiusEnd) * 0.5
  local laneWidth = select(1, extra_utils.getLaneWidth({one_way = link and link.oneWay or false, wp_radius = avgRadius}))
  if not laneWidth or laneWidth <= 0 then
    laneWidth = (link and link.oneWay) and avgRadius * 0.5 or avgRadius
  end
  laneWidth = math.max(laneWidth or 0, 2.8)
  return laneWidth
end

local function createBoundary(pathWorld, laneHalfWidth, veh_props)
  local up = vec3(0, 0, 1)
  local leftPoints, rightPoints = {}, {}
  for i = 1, #pathWorld do
    local current = pathWorld[i]
    local ahead = pathWorld[math.min(i + 1, #pathWorld)]
    local behind = pathWorld[math.max(i - 1, 1)]
    local dir = (ahead - behind)
    if dir:length() > 1e-6 then
      dir = dir:normalized()
    else
      dir = veh_props.dir
    end
    local leftDir = up:cross(dir):normalized()
    local leftWorld = current + leftDir * laneHalfWidth
    local rightWorld = current - leftDir * laneHalfWidth
    leftPoints[#leftPoints + 1] = toLocal(leftWorld, veh_props.center_pos, veh_props.dir_right, veh_props.dir)
    rightPoints[#rightPoints + 1] = toLocal(rightWorld, veh_props.center_pos, veh_props.dir_right, veh_props.dir)
  end
  return leftPoints, rightPoints
end

local function update(dt, veh, system_params)
  if not veh or not system_params then return end
  if not extra_utils.getPart('lane_centering_assist_system_angelo234') then
    resetController()
    setLatestStatus('OFF')
    return
  end

  local params = system_params.lane_centering_params or {}
  local mapData = map and map.getMap and map.getMap()
  local nodes = mapData and mapData.nodes
  if not nodes then
    resetController()
    setLatestStatus('SEARCHING')
    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)
  local forward_speed = veh_props.velocity:dot(veh_props.dir)
  local min_speed = params.min_speed or 1.0
  if forward_speed < min_speed then
    if steering_pid then steering_pid:reset() end
    setLatestStatus('SEARCHING')
    return
  end

  local raw_input = 0
  if electrics_values_angelo234 then
    raw_input = electrics_values_angelo234['steering_input'] or 0
  end

  local disable_thresh = params.override_threshold or 0.2
  if math.abs(raw_input) > disable_thresh then
    resetController()
    setLatestStatus('OVERRIDDEN')
    return {disengaged = true, reason = 'manual_override'}
  end

  local n1, n2 = map.findBestRoad(veh_props.center_pos, veh_props.dir)
  if not (n1 and n2) then
    if steering_pid then steering_pid:reset() end
    setLatestStatus('SEARCHING')
    return
  end

  local startNode, nextNode, initialDir, link = orientNodes(nodes, n1, n2, veh_props.dir)
  if not (startNode and nextNode and initialDir) then
    if steering_pid then steering_pid:reset() end
    setLatestStatus('SEARCHING')
    return
  end

  local startPos = vec3(nodes[startNode].pos)
  local nextPos = vec3(nodes[nextNode].pos)
  local segVec = nextPos - startPos
  local segLenSq = segVec:squaredLength()
  if segLenSq < 1e-6 then
    if steering_pid then steering_pid:reset() end
    setLatestStatus('SEARCHING')
    return
  end

  local rel = veh_props.center_pos - startPos
  local t = rel:dot(segVec) / segLenSq
  t = math.max(0, math.min(1, t))
  local projection = startPos + segVec * t

  local lookahead = (params.lookahead_base or 6) + forward_speed * (params.lookahead_speed or 0.45)
  lookahead = math.max(params.min_lookahead or 5, math.min(params.max_lookahead or 45, lookahead))

  local pathWorld, pathLen = buildPath(nodes, startNode, nextNode, projection, lookahead * 1.2)
  if #pathWorld < 2 then
    if steering_pid then steering_pid:reset() end
    setLatestStatus('SEARCHING')
    return
  end

  local targetWorld, pathDir = computeTargetPoint(pathWorld, lookahead)
  pathDir = pathDir:length() > 1e-6 and pathDir:normalized() or initialDir

  local lane_width = computeLaneWidth(nodes, startNode, nextNode, link)
  local half_width = lane_width * 0.5

  local offset_vec = veh_props.center_pos - projection
  local lateral_offset = offset_vec:dot(veh_props.dir_right)
  local norm_offset = half_width > 1e-6 and lateral_offset / half_width or 0

  local heading_error = veh_props.dir:cross(pathDir).z

  local error = (params.offset_kp or 1.0) * norm_offset + (params.heading_kp or 0.7) * heading_error + (params.curvature_kp or 0.2) * computeCurvature({
    toLocal(pathWorld[1], veh_props.center_pos, veh_props.dir_right, veh_props.dir),
    toLocal(pathWorld[2], veh_props.center_pos, veh_props.dir_right, veh_props.dir),
    toLocal(pathWorld[math.min(3, #pathWorld)], veh_props.center_pos, veh_props.dir_right, veh_props.dir)
  })

  if not steering_pid then
    steering_pid = newPIDStandard(
      params.steer_kp or 0.6,
      params.steer_ki or 0.1,
      params.steer_kd or 0.25,
      -(params.steer_limit or 0.45),
      params.steer_limit or 0.45
    )
    local smooth = (params.steer_smoothing or 0.06) * 1000
    steering_smooth = newTemporalSmoothing(smooth, smooth)
  end

  local pid_out = steering_pid:get(error, 0, dt)
  local target = steering_smooth:getUncapped(pid_out, dt)
  local steer_limit = params.steer_limit or 0.45
  if target > steer_limit then target = steer_limit elseif target < -steer_limit then target = -steer_limit end

  local assist_weight = math.max(0, 1 - math.abs(raw_input) * (params.input_blend_gain or 5))
  local final = raw_input + target * assist_weight
  final = math.max(-1, math.min(1, final))

  if veh and veh.queueLuaCommand then
    veh:queueLuaCommand(string.format("input.event('steering', %f, 0)", final))
  end

  local abs_off = math.abs(lateral_offset)
  local warn_ratio = params.warning_ratio or 0.8
  if abs_off > warn_ratio * half_width and abs_off <= half_width then
    if not warning_played then
      Engine.Audio.playOnce('AudioGui', 'art/sound/proximity_tone_50ms_moderate.wav')
      warning_played = true
    end
  else
    warning_played = false
  end

  local path_local = {}
  for i = 1, #pathWorld do
    path_local[i] = toLocal(pathWorld[i], veh_props.center_pos, veh_props.dir_right, veh_props.dir)
  end
  local left_path, right_path = createBoundary(pathWorld, half_width, veh_props)

  local target_local = toLocal(targetWorld, veh_props.center_pos, veh_props.dir_right, veh_props.dir)
  local proj_local = toLocal(projection, veh_props.center_pos, veh_props.dir_right, veh_props.dir)

  latest_data = {
    status = 'ACTIVE',
    lane_width = lane_width,
    lateral_offset = lateral_offset,
    heading_error = heading_error,
    curvature = computeCurvature(path_local),
    lookahead = lookahead,
    path = path_local,
    path_left = left_path,
    path_right = right_path,
    target_point = target_local,
    projection = proj_local,
    speed = forward_speed,
    assist_weight = assist_weight,
    view = {
      forward = math.max(lookahead * 1.1, 20),
      lateral = math.max(lane_width * 0.75, 6)
    }
  }

  return nil
end

local function onActivated()
  resetController()
  latest_data = {status = 'ARMING'}
end

local function onDeactivated()
  resetController()
  setLatestStatus('OFF')
end

local function getLaneData()
  return latest_data
end

M.update = update
M.onActivated = onActivated
M.onDeactivated = onDeactivated
M.getLaneData = getLaneData

return M
