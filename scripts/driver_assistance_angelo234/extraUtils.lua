local M = {}

require("lua/common/luaProfiler")
local mapmgr = require("lua/vehicle/mapmgr")
local string_buffer = require("string.buffer")

--For efficiency
local max = math.max
local min = math.min
local sin = math.sin
local asin = math.asin
local acos = math.acos
local atan2 = math.atan2
local pi = math.pi
local abs = math.abs
local sqrt = math.sqrt
local floor = math.floor
local ceil = math.ceil
local function clamp(value, low, high)
  if value < low then return low end
  if value > high then return high end
  return value
end

local graph_data
local graph_positions
local graph_radius
local graph_links

local function refreshGraphData()
  graph_data = mapmgr and mapmgr.mapData or graph_data
  if graph_data and graph_data.graph then
    graph_positions = graph_data.positions
    graph_radius = graph_data.radius
    graph_links = graph_data.graph
  else
    graph_positions = nil
    graph_radius = nil
    graph_links = nil
  end
end

local map_nodes = map.getMap().nodes
local findClosestRoad = map.findClosestRoad

refreshGraphData()

local lane_string_buffer = string_buffer.new()

local function flipLaneString(lanes)
  if not lanes or lanes == "" then return lanes end
  lane_string_buffer:reset()
  for i = #lanes, 1, -1 do
    local ch = lanes:byte(i)
    if ch == 43 then -- '+'
      lane_string_buffer:put(45)
    elseif ch == 45 then -- '-'
      lane_string_buffer:put(43)
    else
      lane_string_buffer:put(ch)
    end
  end
  return lane_string_buffer:get()
end

local function countLanesInDirection(lanes, dir)
  if not lanes or lanes == "" then return 0 end
  local target = dir == '-' and 45 or 43
  local count = 0
  for i = 1, #lanes do
    if lanes:byte(i) == target then
      count = count + 1
    end
  end
  return count
end

local function laneRangeIdx(laneConfig)
  if not laneConfig or laneConfig == "" then return nil end
  local total = #laneConfig
  local left_idx, right_idx = nil, nil
  for i = 1, total do
    if laneConfig:byte(i) == 43 then
      left_idx = i
      break
    end
  end
  if not left_idx then return nil end
  for i = total, 1, -1 do
    if laneConfig:byte(i) == 43 then
      right_idx = i
      break
    end
  end
  if not right_idx then return nil end
  return left_idx, right_idx, total
end

local function laneRange(laneConfig)
  local left_idx, right_idx, total = laneRangeIdx(laneConfig)
  if not left_idx or not right_idx then return nil end
  return (left_idx - 1) / total, right_idx / total, right_idx - left_idx + 1, total
end

local function getLaneOffsetsFromConfig(laneConfig, halfWidth)
  if not laneConfig or laneConfig == "" then return nil end
  if not halfWidth or halfWidth <= 0 then return nil end
  local total = #laneConfig
  if total <= 0 then return nil end
  local offsets = {}
  for i = 1, total do
    if laneConfig:byte(i) == 43 then
      local normalized_center = (i - 0.5) / total
      offsets[#offsets + 1] = (normalized_center - 0.5) * 2 * halfWidth
    end
  end
  if #offsets == 0 then return nil end
  return offsets
end

local function computeAverageLaneSpacing(offsets)
  if not offsets or #offsets < 2 then return nil end
  local sorted = {}
  for i = 1, #offsets do
    sorted[i] = offsets[i]
  end
  table.sort(sorted)
  local sum = 0
  local count = 0
  for i = 2, #sorted do
    sum = sum + (sorted[i] - sorted[i - 1])
    count = count + 1
  end
  if count == 0 then return nil end
  return sum / count
end

local function getLaneBoundsFromConfig(laneConfig, halfWidth)
  if not laneConfig or not halfWidth or halfWidth <= 0 then return nil end
  local left_norm, right_norm, lane_slots, total_slots = laneRange(laneConfig)
  if not left_norm or not right_norm then return nil end
  local scale = 2 * halfWidth
  local left = (left_norm - 0.5) * scale
  local right = (right_norm - 0.5) * scale
  return left, right, lane_slots, total_slots
end

local function numOfLanesFromRadius(rad1, rad2)
  local r1 = tonumber(rad1) or 0
  local r2 = tonumber(rad2) or r1
  local min_radius = min(r1, r2)
  if min_radius <= 0 then
    min_radius = max(r1, r2)
  end
  if min_radius <= 0 then
    min_radius = 3.6
  end
  return max(1, math.floor(min_radius * 2 / 3.61 + 0.5))
end

local function getEdgeLaneConfig(fromNode, toNode)
  if not fromNode or not toNode then return nil end
  if not graph_data or not graph_data.graph then
    refreshGraphData()
  end
  if not graph_data or not graph_data.graph then return nil end

  local graph = graph_data.graph
  if not graph[fromNode] then return nil end
  local edge = graph[fromNode][toNode]
  if not edge then return nil end

  local lanes = edge.lanes
  if not lanes or lanes == "" then
    local rad_from, rad_to
    if graph_data.getEdgeRadii then
      rad_from, rad_to = graph_data:getEdgeRadii(fromNode, toNode)
    end
    rad_from = rad_from or (graph_radius and graph_radius[fromNode])
    rad_to = rad_to or (graph_radius and graph_radius[toNode])
    local count = numOfLanesFromRadius(rad_from, rad_to)
    if edge.oneWay then
      lanes = ('+'):rep(max(1, count))
    else
      local per_dir = max(1, math.floor(count * 0.5 + 0.5))
      local rules = getRoadRules()
      local rightHand = rules and rules.rightHandDrive
      if rightHand then
        lanes = ('+'):rep(per_dir)..('-'):rep(per_dir)
      else
        lanes = ('-'):rep(per_dir)..('+'):rep(per_dir)
      end
    end
  end

  if edge.inNode and edge.inNode ~= fromNode then
    lanes = flipLaneString(lanes)
  end

  return lanes
end

local function getRoadRules()
  if mapmgr and mapmgr.rules then
    return mapmgr.rules
  end
  if map and type(map.getRoadRules) == "function" then
    local ok, rules = pcall(map.getRoadRules)
    if ok and type(rules) == "table" then
      return rules
    end
  end
  return nil
end

local function getTrafficSide()
  local rules = getRoadRules()
  if rules and rules.rightHandDrive ~= nil then
    return rules.rightHandDrive and "left" or "right"
  end
  return nil
end

local function getPart(partName)
  local vehData = core_vehicle_manager.getPlayerVehicleData()
  if not vehData then return nil end
  return vehData.vdata.activePartsData[partName]
end

local vehs_props_reusing = {}

local function safeCallMethod(obj, name)
  if not obj then return nil end
  local fn = obj[name]
  if type(fn) ~= "function" then return nil end
  local ok, res = pcall(fn, obj)
  if ok then return res end
  return nil
end

local function safeCallFunction(fn, ...)
  if type(fn) ~= "function" then return nil end
  local ok, res = pcall(fn, ...)
  if ok then return res end
  return nil
end

local function hasGhostTrait(candidate)
  if not candidate then return false end

  local truthyChecks = {
    "isGhost",
    "getIsGhost",
    "isTrafficGhost",
    "isGhostModeEnabled"
  }

  for _, name in ipairs(truthyChecks) do
    local res = safeCallMethod(candidate, name)
    if res then return true end
  end

  local falseyChecks = {
    "getActive",
    "isActive",
    "getActiveStatus"
  }

  for _, name in ipairs(falseyChecks) do
    local res = safeCallMethod(candidate, name)
    if res ~= nil then
      if res == false then return true end
      if type(res) == "string" then
        local lowered = res:lower()
        if lowered == "ghost" or lowered == "ghosted" or lowered == "ghosting"
          or lowered == "inactive" then
          return true
        end
      end
    end
  end

  local shown = safeCallMethod(candidate, "isObjectShown")
  if shown ~= nil and shown == false then return true end

  local hidden = safeCallMethod(candidate, "isObjectHidden")
  if hidden ~= nil and hidden == true then return true end

  local visible = safeCallMethod(candidate, "getMeshVisibility")
  if visible ~= nil and visible == false then return true end

  return false
end

local function getVehicleDataSafe(id)
  if not id or not core_vehicle_manager then return nil end
  local data = safeCallFunction(core_vehicle_manager.getVehicleData, id)
  if data ~= nil then return data end
  if type(core_vehicle_manager.getVehicleData) == "function" then
    local ok, res = pcall(core_vehicle_manager.getVehicleData, core_vehicle_manager, id)
    if ok then return res end
  end
  return nil
end

local function vehicleDataMarksGhost(data)
  if not data then return false end

  if data.ghost or data.isGhost or data.ghostMode or data.ghosted then
    return true
  end

  local booleanFields = {
    "isSpawned",
    "spawned",
    "active",
    "isActive",
    "inGame",
    "isInGame",
    "visible",
    "isVisible",
    "render",
    "rendered",
    "renderVisible",
    "renderedVisible"
  }

  for _, field in ipairs(booleanFields) do
    local value = data[field]
    if value ~= nil and value == false then
      return true
    end
  end

  local stringFields = {
    "state",
    "status",
    "spawnState",
    "spawn_state",
    "spawnStatus",
    "spawn_status",
    "renderState",
    "render_state",
    "visibility",
    "presence",
    "mode"
  }

  for _, field in ipairs(stringFields) do
    local value = data[field]
    if type(value) == "string" then
      local lowered = value:lower()
      if lowered == "ghost" or lowered == "ghosted" or lowered == "ghosting"
        or lowered == "despawned" or lowered == "hidden" or lowered == "inactive" then
        return true
      end
    end
  end

  return false
end

local function isVehicleGhost(veh, props)
  if not veh then return true end

  if hasGhostTrait(veh) or hasGhostTrait(veh.obj) then
    return true
  end

  local id = veh.getID and veh:getID()
  if id then
    local data = getVehicleDataSafe(id)
    if vehicleDataMarksGhost(data) then
      return true
    end
  end

  if props and props.bb then
    local half_extents = props.bb:getHalfExtents()
    if half_extents and half_extents.x == 0 and half_extents.y == 0 and half_extents.z == 0 then
      return true
    end
  end

  return false
end

local function getVehicleProperties(veh)
  if not vehs_props_reusing[veh:getID()] then
    vehs_props_reusing[veh:getID()] = {
      name = nil,
      id = nil,
      dir = vec3(),
      dir_up = vec3(),
      dir_right = nil,
      bb = nil,
      center_pos = vec3(),
      front_pos = nil,
      rear_pos = vec3(),
      velocity = vec3(),
      speed = nil,
      acceleration = vec3()
    }
  end

  local props = vehs_props_reusing[veh:getID()]

  props.name = veh:getJBeamFilename()
  props.id = veh:getID()

  --Direction vectors of vehicle
  props.dir:set(veh.obj:getDirectionVector())
  props.dir_up:set(veh.obj:getDirectionVectorUp())
  props.dir_right = props.dir:cross(props.dir_up)

  --Bounding box of vehicle
  props.bb = veh:getSpawnWorldOOBB()
  props.center_pos:set(props.bb:getCenter())
  props.front_pos = props.center_pos + props.dir * veh:getSpawnAABBRadius()
  props.rear_pos:set(veh:getSpawnWorldOOBBRearPoint())

  props.velocity:set(veh:getVelocity())
  props.speed = props.velocity:length()

  local acceleration = veh_accs_angelo234[veh:getID()]

  if acceleration == nil then
    props.acceleration:set(0,0,0)
  else
    props.acceleration:set(acceleration[1], acceleration[2], 9.81 - acceleration[3])
  end

  return props
end


local function getPathLen(path, startIdx, stopIdx)
  if not path then return end
  startIdx = startIdx or 1
  stopIdx = stopIdx or #path
  local pathLen = 0
  for i = startIdx+1, stopIdx do
    pathLen = pathLen + (map_nodes[path[i]].pos - map_nodes[path[i-1]].pos):length()
  end

  return pathLen
end

--Check if waypoint is on the same road as me (not lane)
local function checkIfWaypointsWithinMyCar(veh_props, wps_props)
  local xnorm = veh_props.center_pos:xnormOnLine(wps_props.start_wp_pos, wps_props.end_wp_pos)
  local wp_dir = wps_props.end_wp_pos - wps_props.start_wp_pos
  local veh_pos_on_wp_line = xnorm * wp_dir + wps_props.start_wp_pos

  local lat_dist = (veh_props.center_pos - veh_pos_on_wp_line):length()

  --debugDrawer:drawTextAdvanced((wps_props.end_wp_pos), String("Lateral Distance: " .. tostring(lat_dist)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  return lat_dist < wps_props.wp_radius
end

local function toNormXYVec(dir)
  return dir:z0():normalized()
end

local function getFuturePosition(veh_props, time, rel_car_pos)
  local acc_vec = vec3(-veh_props.acceleration)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(veh_props.dir, vec3(0,0,1)) * acc_vec

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = veh_props.front_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = veh_props.center_pos + (veh_props.velocity + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = veh_props.rear_pos + (veh_props.velocity + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getFuturePositionXY(veh_props, time, rel_car_pos)
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local acc_vec = vec3(-veh_props.acceleration):z0()

  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * acc_vec

  local front_pos_xy = veh_props.front_pos:z0()
  local center_pos_xy = veh_props.center_pos:z0()
  local rear_pos_xy = veh_props.rear_pos:z0()

  local velocity_xy = veh_props.velocity:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getFuturePositionXYWithAcc(veh_props, time, acc_vec, rel_car_pos)
  --Set position, velocity, and acceleration vectors z component to zero
  --for less error in calculating future position
  local dir_xy = toNormXYVec(veh_props.dir)

  --Convert from local space into world space acceleration vector
  acc_vec = quatFromDir(dir_xy, vec3(0,0,1)) * -acc_vec

  local front_pos_xy = veh_props.front_pos:z0()
  local center_pos_xy = veh_props.center_pos:z0()
  local rear_pos_xy = veh_props.rear_pos:z0()

  local velocity_xy = veh_props.velocity:z0()

  local veh_pos_future = vec3(0,0,0)

  if rel_car_pos == "front" then
    veh_pos_future = front_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "center" then
    veh_pos_future = center_pos_xy + (velocity_xy + acc_vec * time) * time
  elseif rel_car_pos == "rear" then
    veh_pos_future = rear_pos_xy + (velocity_xy + acc_vec * time) * time
  end

  return veh_pos_future
end

local function getWaypointPosition(wp)
  if not wp then return nil end
  if not graph_positions then
    refreshGraphData()
  end
  if graph_positions and graph_positions[wp] then
    return vec3(graph_positions[wp])
  end
  if map_nodes and map_nodes[wp] and map_nodes[wp].pos then
    return vec3(map_nodes[wp].pos)
  end
  return nil
end

local function getLaneWidth(wps_info)
  local max_lane_width = 4.5
  local min_lane_width = 2.7
  local lane_width = 0
  local lanes = 0

  local half_width = wps_info.wp_radius or 0
  local lane_config = nil
  if wps_info.start_wp and wps_info.end_wp then
    lane_config = getEdgeLaneConfig(wps_info.start_wp, wps_info.end_wp)
  end

  if lane_config and lane_config ~= "" and half_width > 0 then
    local offsets = getLaneOffsetsFromConfig(lane_config, half_width)
    local spacing = computeAverageLaneSpacing(offsets)
    local left_bound, right_bound, slot_count, slot_total = getLaneBoundsFromConfig(lane_config, half_width)
    if spacing and spacing > 0 then
      lane_width = spacing
    elseif slot_total and slot_total > 0 then
      lane_width = (half_width * 2) / slot_total
    end
    if offsets then
      lanes = #offsets
    end
    if not lanes or lanes <= 0 then
      lanes = countLanesInDirection(lane_config, '+')
    end
    if not lanes or lanes <= 0 then
      lanes = 1
    end

    if lane_width <= 0 then
      lane_width = (half_width * 2) / max(lanes, 1)
    end

    if lane_width > 0 then
      lane_width = clamp(lane_width, min_lane_width, max_lane_width)
    end

    wps_info.lane_config = lane_config
    if offsets and #offsets > 0 then
      wps_info.forward_lane_offsets = offsets
    else
      wps_info.forward_lane_offsets = nil
    end
    if left_bound and right_bound then
      wps_info.forward_lane_left = left_bound
      wps_info.forward_lane_right = right_bound
      wps_info.forward_lane_slot_count = slot_count
      wps_info.forward_lane_total_slots = slot_total
      wps_info.forward_lane_slot_width = slot_total and slot_total > 0 and (half_width * 2) / slot_total or nil
    else
      wps_info.forward_lane_left = nil
      wps_info.forward_lane_right = nil
      wps_info.forward_lane_slot_count = nil
      wps_info.forward_lane_total_slots = nil
      wps_info.forward_lane_slot_width = nil
    end
  else
    wps_info.lane_config = lane_config
    wps_info.forward_lane_offsets = nil
    wps_info.forward_lane_left = nil
    wps_info.forward_lane_right = nil
    wps_info.forward_lane_slot_count = nil
    wps_info.forward_lane_total_slots = nil
    wps_info.forward_lane_slot_width = nil
  end

  if lane_width <= 0 then
    if wps_info.one_way then
      if half_width * 2 < max_lane_width then
        lane_width = half_width * 2.0
        lanes = 1
      elseif half_width * 2 < max_lane_width * 2 then
        lane_width = half_width
        lanes = 2
      elseif half_width * 2 < max_lane_width * 3 then
        lane_width = half_width * 2.0 / 3.0
        lanes = 3
      elseif half_width * 2 < max_lane_width * 4 then
        lane_width = half_width * 2.0 / 4.0
        lanes = 4
      end
    else
      lane_width = half_width
      lanes = 2
    end
  end

  if lanes <= 0 then
    lanes = wps_info.one_way and 1 or 2
  end

  if lane_width <= 0 then
    lane_width = clamp(half_width, min_lane_width, max_lane_width)
  end

  return lane_width, lanes
end

--Gets whether we are to the left or right of waypoints
local function getWhichSideOfWaypointsCarIsOn(veh_props, start_pos, end_pos)
  local road_line_dir = toNormXYVec(end_pos - start_pos)
  local road_line_dir_perp_right = vec3(road_line_dir.y, -road_line_dir.x)
  local car_dir_xy = toNormXYVec(veh_props.dir)

  local wp_mid_pos = (end_pos - start_pos) * 0.5 + start_pos

  local line_to_center_dir = (veh_props.center_pos - wp_mid_pos):normalized()

  local center_angle = acos(line_to_center_dir:dot(road_line_dir_perp_right))

  --Determine side of road car is on
   --Right side
  if center_angle < pi / 2.0 then
    return "right"
    --Left side
  else
    return "left"
  end
end

local function getWaypointsProperties(veh_props, start_wp, end_wp, start_wp_pos, end_wp_pos, lat_dist_from_wp)
  local wps_props = {}

  --Get lane width
  if not graph_positions and (not map_nodes or not map_nodes[start_wp]) then
    refreshGraphData()
  end

  local wp_radius = 0
  if graph_radius and graph_radius[start_wp] then
    wp_radius = graph_radius[start_wp]
  elseif map_nodes and map_nodes[start_wp] and map_nodes[start_wp].radius then
    wp_radius = map_nodes[start_wp].radius
  end

  local one_way = false
  if graph_links and graph_links[start_wp] then
    local link = graph_links[start_wp][end_wp]
    if link and link.oneWay ~= nil then
      one_way = link.oneWay ~= 0
    else
      for _, data in pairs(graph_links[start_wp]) do
        if data.oneWay ~= nil then
          one_way = data.oneWay ~= 0
          break
        end
      end
    end
  elseif map_nodes and map_nodes[start_wp] and map_nodes[start_wp].links then
    for _, data in pairs(map_nodes[start_wp].links) do
      if data.oneWay ~= nil then
        one_way = data.oneWay
        break
      end
    end
  end

  wps_props.start_wp = start_wp
  wps_props.end_wp = end_wp
  wps_props.start_wp_pos = start_wp_pos
  wps_props.end_wp_pos = end_wp_pos
  wps_props.one_way = one_way
  wps_props.wp_radius = wp_radius
  wps_props.lat_dist_from_wp = lat_dist_from_wp

  local lane_width, num_of_lanes = getLaneWidth(wps_props)

  wps_props.lane_width = lane_width
  wps_props.num_of_lanes = num_of_lanes

  local my_veh_side_of_wp = getWhichSideOfWaypointsCarIsOn(veh_props, start_wp_pos, end_wp_pos)

  if my_veh_side_of_wp == "left" then
    wps_props.lat_dist_from_wp = -wps_props.lat_dist_from_wp
  end

  wps_props.veh_side_of_wp = my_veh_side_of_wp

  return wps_props
end

local function getRoadNaturalContinuation(prev_node, current_node)
  if not prev_node or not current_node then return nil end
  if not graph_data or not graph_data.graph then
    refreshGraphData()
  end
  if not graph_data or not graph_data.graph then return nil end

  local graph = graph_data.graph
  local positions = graph_data.positions or graph_positions
  if not graph[current_node] or not graph[prev_node] then return nil end

  local in_edge = graph[prev_node][current_node]
  if not in_edge then return nil end

  local lane_config = getEdgeLaneConfig(prev_node, current_node)
  if not lane_config or lane_config == "" then return nil end

  local in_lane_count = countLanesInDirection(lane_config, '+')
  if in_lane_count <= 0 then return nil end

  local in_radius_current, in_radius_prev
  if graph_data.getEdgeRadii then
    in_radius_current, in_radius_prev = graph_data:getEdgeRadii(current_node, prev_node)
  end
  in_radius_prev = in_radius_prev or (graph_radius and graph_radius[prev_node])
  in_radius_current = in_radius_current or (graph_radius and graph_radius[current_node])
  if not in_radius_current or not in_radius_prev then
    return nil
  end

  local edge_pos_prev_1, edge_pos_prev_2
  if graph_data.getEdgePositions then
    edge_pos_prev_1, edge_pos_prev_2 = graph_data:getEdgePositions(prev_node, current_node)
  end
  if not edge_pos_prev_1 or not edge_pos_prev_2 then
    if positions then
      edge_pos_prev_1 = positions[prev_node]
      edge_pos_prev_2 = positions[current_node]
    end
  end
  if not edge_pos_prev_1 or not edge_pos_prev_2 then return nil end

  local in_dir = (vec3(edge_pos_prev_2) - vec3(edge_pos_prev_1))
  local len = in_dir:length()
  if len <= 1e-6 then return nil end
  in_dir:scale(1 / len)

  local lane_flow_base = (in_edge.drivability or 1) * 4 * min(in_radius_current, in_radius_prev) / max(#lane_config, 1)
  local in_back_lanes = #lane_config - in_lane_count
  local in_forward_flow = in_lane_count * lane_flow_base
  local in_backward_flow = in_back_lanes * lane_flow_base

  local best_node = nil
  local best_score = -1
  local out_dir = vec3()

  for neighbor, edge in pairs(graph[current_node]) do
    if neighbor ~= prev_node then
      local out_lane_config = getEdgeLaneConfig(current_node, neighbor)
      if out_lane_config and out_lane_config ~= "" then
        local out_lane_count = countLanesInDirection(out_lane_config, '+')
        if out_lane_count > 0 then
          local edge_pos_curr_1, edge_pos_curr_2
          if graph_data.getEdgePositions then
            edge_pos_curr_1, edge_pos_curr_2 = graph_data:getEdgePositions(current_node, neighbor)
          end
          if not edge_pos_curr_1 or not edge_pos_curr_2 then
            if positions then
              edge_pos_curr_1 = positions[current_node]
              edge_pos_curr_2 = positions[neighbor]
            end
          end
          if edge_pos_curr_1 and edge_pos_curr_2 then
            out_dir:set(edge_pos_curr_2)
            out_dir:setSub(edge_pos_curr_1)
            local out_len = out_dir:length()
            if out_len > 1e-6 then
              out_dir:scale(1 / out_len)
              local dir_coeff = 0.5 * max(0, 1 + out_dir:dot(in_dir))
              if dir_coeff > 0 then
                local out_radius_current, out_radius_next
                if graph_data.getEdgeRadii then
                  out_radius_current, out_radius_next = graph_data:getEdgeRadii(current_node, neighbor)
                end
                out_radius_current = out_radius_current or (graph_radius and graph_radius[current_node]) or in_radius_current
                out_radius_next = out_radius_next or (graph_radius and graph_radius[neighbor]) or out_radius_current
                local lane_flow_out
                if out_lane_count == in_lane_count and (#out_lane_config - out_lane_count) == in_back_lanes then
                  lane_flow_out = (edge.drivability or 1) * 4 * 0.5 * ((in_radius_current or 0) + (out_radius_current or 0)) / max(#out_lane_config, 1)
                else
                  lane_flow_out = (edge.drivability or 1) * 4 * min(out_radius_current or 0, out_radius_next or 0) / max(#out_lane_config, 1)
                end
                local out_forward_flow = min(in_forward_flow, out_lane_count * lane_flow_out)
                local out_backward_lanes = #out_lane_config - out_lane_count
                local out_backward_flow = min(in_backward_flow, out_backward_lanes * lane_flow_out)
                local out_flow = out_forward_flow * (1 + out_backward_flow) * dir_coeff
                if out_flow > best_score then
                  best_score = out_flow
                  best_node = neighbor
                end
              end
            end
          end
        end
      end
    end
  end

  return best_node, best_score
end

--Get start and end waypoints relative to my vehicle nearest to a position
local function getWaypointStartEnd(my_veh_props, veh_props, position)
  local start_wp = nil
  local end_wp = nil

  local start_wp_pos = nil
  local end_wp_pos = nil

  local wp1, wp2, lat_dist_from_wp = findClosestRoad(position)

  if wp1 == nil or wp2 == nil then
    return nil
  end

  local wp1_pos = getWaypointPosition(wp1)
  local wp2_pos = getWaypointPosition(wp2)

  --Relative to my car
  local origin = my_veh_props.dir * -9999

  --Figure out which waypoints are the start and end waypoint
  if abs((origin - wp1_pos):length()) < abs((origin - wp2_pos):length()) then
    start_wp = wp1
    end_wp = wp2

    start_wp_pos = wp1_pos
    end_wp_pos = wp2_pos
  else
    start_wp = wp2
    end_wp = wp1

    start_wp_pos = wp2_pos
    end_wp_pos = wp1_pos
  end

  local wps_props = getWaypointsProperties(veh_props, start_wp, end_wp, start_wp_pos, end_wp_pos, lat_dist_from_wp)

  return wps_props
end

local wps_reusing = {}

--Uses past wps to find most suitable waypoints to use
local function getWaypointStartEndAdvanced(my_veh_props, veh_props, position, past_wps_props)
  for k in pairs(wps_reusing) do
    wps_reusing[k] = nil
  end

  local wps_props = getWaypointStartEnd(my_veh_props, veh_props, position)

  table.insert(wps_reusing, wps_props)

  if past_wps_props then
    table.insert(wps_reusing, past_wps_props)
  end

  local angle_between_vehs = acos(my_veh_props.dir:dot(veh_props.dir))

  if angle_between_vehs > pi / 2.0 then
    angle_between_vehs = pi
  else
    angle_between_vehs = 0
  end

  local min_wp_props_angle = {pi, nil}

  for _, curr_wps_props in pairs(wps_reusing) do
    --Get direction between our waypoints and one of its linked waypoints
  local wp_dir = (curr_wps_props.end_wp_pos - curr_wps_props.start_wp_pos):normalized()

    --Angle between waypoint dir and car dir
    local angle = acos(wp_dir:dot(veh_props.dir))

    --print(angle * 180.0 / pi)

    angle = abs(angle_between_vehs - angle)

    if angle > pi / 2.0 then
      angle = pi - angle
    end

    if angle < min_wp_props_angle[1] then
      min_wp_props_angle[1] = angle
      min_wp_props_angle[2] = curr_wps_props
    end

    --debugDrawer:drawSphere((curr_wps_props.start_wp_pos + vec3(0,0,2)), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp1_pos + vec3(0,0,3)), String("ID: " .. tostring(i)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

    --debugDrawer:drawSphere((curr_wps_props.end_wp_pos + vec3(0,0,2)), 0.5, ColorF(1,1,0,1))
    --debugDrawer:drawTextAdvanced((wp2_pos + vec3(0,0,3)), String("ID: " .. tostring(i + 1)),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  end

  return min_wp_props_angle[2]
end

local function getWaypointSegmentFromNodes(my_veh_props, veh_props, start_wp, end_wp)
  if not start_wp or not end_wp then return nil end

  local start_wp_pos = getWaypointPosition(start_wp)
  local end_wp_pos = getWaypointPosition(end_wp)

  if not start_wp_pos or not end_wp_pos then
    return nil
  end

  local lat_dist_from_wp = 0

  if veh_props and veh_props.center_pos then
    local start_xy = start_wp_pos:z0()
    local end_xy = end_wp_pos:z0()
    local veh_xy = veh_props.center_pos:z0()
    local seg_vec = end_xy - start_xy
    local seg_len_sq = seg_vec:squaredLength()

    if seg_len_sq > 1e-9 then
      local xnorm = clamp((veh_xy - start_xy):dot(seg_vec) / seg_len_sq, 0, 1)
      local lane_point = start_xy + seg_vec * xnorm
      local seg_dir = toNormXYVec(seg_vec)
      if seg_dir:length() > 1e-6 then
        local right = vec3(seg_dir.y, -seg_dir.x, 0)
        lat_dist_from_wp = (veh_xy - lane_point):dot(right)
      end
    end
  end

  return getWaypointsProperties(veh_props or my_veh_props, start_wp, end_wp, start_wp_pos, end_wp_pos, lat_dist_from_wp)
end

--Check if other car is on the same road as me (not lane)
local function checkIfOtherCarOnSameRoad(my_veh_props, other_veh_props, wps_props)
  --Calculate distance to get from my waypoint to other vehicle's waypoint using graphpath
  --and if that distance is equal or less than shortest distance * 1.05 between two then
  --vehicle is in path

  local other_wps_props_in_other_dir = getWaypointStartEnd(other_veh_props, other_veh_props, other_veh_props.center_pos)

  local path = map.getPath(wps_props.start_wp, other_wps_props_in_other_dir.start_wp, 0, 100)
  local path_len = getPathLen(path)

  --debugDrawer:drawTextAdvanced((other_veh_props.front_pos), String(path_len),  ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))

  return path_len <= (wps_props.start_wp_pos - other_wps_props_in_other_dir.start_wp_pos):length() * 1.05
end

local function getCircularDistance(my_veh_props, other_veh_props)
  local other_bb = other_veh_props.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local shoot_ray_dir = (other_veh_props.center_pos - my_veh_props.front_pos):normalized()

  local min_distance, max_distance = intersectsRay_OBB(my_veh_props.front_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

  if min_distance < 0 then
    min_distance = 0
  end

  --Convert to circular distance
  local turning_radius = abs(my_veh_props.speed / angular_speed_angelo234)

  local angle = acos(
    (min_distance * min_distance) / (-2 * turning_radius * turning_radius) + 1)

  local cir_dist = angle * turning_radius

  return cir_dist
end

local function getStraightDistance(my_veh_props, other_veh_props, front, in_my_vehs_straight_path)
  local my_bb = my_veh_props.bb

  local other_bb = other_veh_props.bb

  local other_x = other_bb:getHalfExtents().x * vec3(other_bb:getAxis(0))
  local other_y = other_bb:getHalfExtents().y * vec3(other_bb:getAxis(1))
  local other_z = other_bb:getHalfExtents().z * vec3(other_bb:getAxis(2))

  local ray_pos = nil

  if front then
    ray_pos = my_veh_props.front_pos
  else
    ray_pos = my_veh_props.rear_pos
  end

  local min_distance = 9999
  local shoot_ray_dir = nil

  if in_my_vehs_straight_path then
    if front then
      shoot_ray_dir = my_veh_props.dir
    else
      shoot_ray_dir = -my_veh_props.dir
    end

    local min_distance1, max_distance1 = intersectsRay_OBB(ray_pos + my_veh_props.dir_right * my_bb:getHalfExtents().x * 0.75, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)
    local min_distance2, max_distance2 = intersectsRay_OBB(ray_pos - my_veh_props.dir_right * my_bb:getHalfExtents().x * 0.75, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)
    local min_distance3, max_distance3 = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

    min_distance = math.min(min_distance1, min_distance2, min_distance3)
  else
    shoot_ray_dir = (other_veh_props.center_pos - ray_pos):normalized()

    local min_distance1, max_distance1 = intersectsRay_OBB(ray_pos, shoot_ray_dir, other_veh_props.center_pos, other_x, other_y, other_z)

    min_distance = min_distance1
  end

  if min_distance < -0.1 then
    min_distance = 9999
  end

  return min_distance
end

local function onClientPostStartMission(levelpath)
  map_nodes = map.getMap().nodes
  findClosestRoad = map.findClosestRoad
  refreshGraphData()
end

M.getPart = getPart
M.getVehicleProperties = getVehicleProperties
M.toNormXYVec = toNormXYVec
M.getFuturePosition = getFuturePosition
M.getFuturePositionXY = getFuturePositionXY
M.getFuturePositionXYWithAcc = getFuturePositionXYWithAcc
M.getWaypointPosition = getWaypointPosition
M.getLaneWidth = getLaneWidth
M.getWhichSideOfWaypointsCarIsOn = getWhichSideOfWaypointsCarIsOn
M.getWaypointStartEnd = getWaypointStartEnd
M.getWaypointsProperties = getWaypointsProperties
M.getWaypointStartEndAdvanced = getWaypointStartEndAdvanced
M.getWaypointSegmentFromNodes = getWaypointSegmentFromNodes
local function getClosestRoadInfo(position)
  if not position or not findClosestRoad then return nil end
  local wp1, wp2, distance = findClosestRoad(position)
  if not wp1 or not wp2 then return nil end
  return wp1, wp2, distance
end

M.getGraphData = function()
  if not graph_data or not graph_links then
    refreshGraphData()
  end
  return graph_data
end
M.getGraphLinks = function(node)
  if not graph_links then
    refreshGraphData()
  end
  if graph_links and graph_links[node] then
    return graph_links[node]
  end
  if map_nodes and map_nodes[node] then
    return map_nodes[node].links
  end
  return nil
end
M.getGraphLinkData = function(node, neighbor)
  if not node or not neighbor then return nil end
  if graph_links and graph_links[node] then
    return graph_links[node][neighbor]
  end
  if map_nodes and map_nodes[node] and map_nodes[node].links then
    return map_nodes[node].links[neighbor]
  end
  return nil
end
M.getMapNode = function(id)
  if not map_nodes then return nil end
  return map_nodes[id]
end
M.getClosestRoadInfo = getClosestRoadInfo
M.getClosestRoadDistance = function(position)
  local _, _, distance = getClosestRoadInfo(position)
  if distance == nil then return nil end
  return math.abs(distance)
end
M.checkIfOtherCarOnSameRoad = checkIfOtherCarOnSameRoad
M.getCircularDistance = getCircularDistance
M.getStraightDistance = getStraightDistance
M.onClientPostStartMission = onClientPostStartMission
M.isVehicleGhost = isVehicleGhost
M.getRoadRules = getRoadRules
M.getTrafficSide = getTrafficSide
M.getEdgeLaneConfig = getEdgeLaneConfig
M.getRoadNaturalContinuation = getRoadNaturalContinuation
M.countLanesInDirection = countLanesInDirection

return M
