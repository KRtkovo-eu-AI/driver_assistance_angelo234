-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local graphpath = require('graphpath')
local quadtree = require('quadtree')
local kdTreeBox2D = require('kdtreebox2d')
local buffer = require("string.buffer")

-- cache frequently used functions from other modules in upvalues
local min, max, abs, sqrt, huge = math.min, math.max, math.abs, math.sqrt, math.huge
local tableInsert, tableClear = table.insert, table.clear
local stringMatch, stringFind, stringSub = string.match, string.find, string.sub
local pointBBox = quadtree.pointBBox

local M = {}

M.objectNames = {}
M.objects = {}

local objectsCache = {}
local mapFilename = ''
local map = {nodes = {}}
local loadedMap = false
local objectsCount = 0
local maxRadius = nil
local rules = nil
local isEditorEnabled
local visualLog = {}
local gp = nil
local edgeKdTree = nil
local nodeKdTree = nil
local manualWaypoints
local buildSerial = -1
local emptyTable = setmetatable({}, {__newindex = function(t, key, val) log('E', 'map', 'Tried to insert new elements into map.objects') end})
local vecUp = vec3(0,0,1)
local tmpBuf = buffer.new()
local function highToLow(a, b) return a < b end
local function lowToHigh(a, b) return a > b end
local function randomOrder(a, b) return math.random() > math.random() end

local _pairs = pairs
-- for debugging issues in the deterministic build of the navgraph
local function toggleShuffledPairs(mode)
  if mode then
    if mode == 'shuffledPairs' then
      _pairs = shuffledPairs
    elseif mode == 'pairs' then
      _pairs = pairs
    end
  else
    _pairs = _pairs == pairs and shuffledPairs or pairs
  end
  if _pairs == pairs then
    log('W', 'map.lua', 'Using pairs')
  elseif _pairs == shuffledPairs then
    log('W', 'map.lua', 'Using shuffledPairs')
  end
end

local function safeVecSum(l)
  local len = #l
  local tmp = table.new(len, 0)

  for i = 1, len do tmp[i] = l[i].x end
  table.sort(tmp)
  local sumX = 0
  for i = 1, len do sumX = sumX + tmp[i] end

  for i = 1, len do tmp[i] = l[i].y end
  table.sort(tmp)
  local sumY = 0
  for i = 1, len do sumY = sumY + tmp[i] end

  for i = 1, len do tmp[i] = l[i].z end
  table.sort(tmp)
  local sumZ = 0
  for i = 1, len do sumZ = sumZ + tmp[i] end

  return vec3(sumX, sumY, sumZ)
end

local function safeNumberSum(l)
  local len = #l
  local _l = table.new(len, 0)

  for i = 1, len do _l[i] = l[i] end -- copy to preserve order of input list
  table.sort(_l)
  local sum = 0
  for i = 1, len do sum = sum + _l[i] end

  return sum
end

-- enforces rerendering the loading screen if required
local function _updateProgress()
  LoadingManager:triggerUpdate()
end

local singleEventTimer = {}
singleEventTimer.__index = singleEventTimer

local function newSingleEventTimer()
  local data = {waitDt = -1, update = nop, eventFun = nop}
  setmetatable(data, singleEventTimer)
  return data
end

function singleEventTimer:update(dt)
  local waitDt = max(0, self.waitDt - dt)
  self.waitDt = waitDt
  if waitDt == 0 then
    self.update = nop
    self.eventFun(unpack(self.params))
  end
end

function singleEventTimer:callAfter(dt, eventFun, ...)
  self.waitDt = dt
  self.eventFun = eventFun
  self.params = {...}
  self.update = singleEventTimer.update
end

local delayedLoad = newSingleEventTimer()

local function visLog(type, pos, msg)
  tableInsert(visualLog, {type = type, pos = pos, msg = msg})
end

local function nameNode(prefix, idx)
  local nodeName = prefix..idx
  if map.nodes[nodeName] then
    nodeName = nodeName.."_"
    local postfix = 1
    while map.nodes[nodeName..postfix] do
      postfix = postfix + 1
    end
    nodeName = nodeName..postfix
  end
  return nodeName
end

local function setRoadRules()
  rules = {rightHandDrive = false, turnOnRed = false} -- default road rules
  local fileName = path.getPathLevelInfo(getCurrentLevelIdentifier() or '')
  local info = jsonReadFile(fileName)
  if info and info.roadRules then
    rules = tableMerge(rules, info.roadRules)
  end
end

local function getRoadRules()
  return rules or {}
end

local function isOneWay(lanes)
  return not lanes:find('-') or not lanes:find('+')
end

local laneStringBuffer = buffer.new()
local function flipLanes(lanes)
  -- ex. '--+++' becomes '---++'
  for i = #lanes, 1, -1 do
    laneStringBuffer:put(lanes:byte(i) == 43 and '-' or lanes:byte(i) == 45 and '+' or '0')
  end
  return laneStringBuffer:get()
end

-- returns the edge node positions of edge n1-n2 in the given order
local function getEdgeNodePositions(n1id, n2id)
  if not (map.nodes[n1id] and map.nodes[n2id] and (map.nodes[n1id].links[n2id] or map.nodes[n2id].links[n1id])) then return end
  local edgeData = map.nodes[n1id].links[n2id] or map.nodes[n2id].links[n1id]
  return edgeData[edgeData.inNode == n1id and 'inPos' or 'outPos'] or map.nodes[n1id].pos,
         edgeData[edgeData.inNode == n2id and 'inPos' or 'outPos'] or map.nodes[n2id].pos
end

-- returns the node position of node n1 of edge
local function getEdgeNodePosition(nid, edge)
  -- nid: a node id (string)
  -- edge: an edge data table.(ex. given node n1id and n2id the edge data table is map.nodes[n1id].links[n2id])
  return edge[edge.inNode == nid and 'inPos' or 'outPos'] or map.nodes[nid].pos
end

-- returns the edge node positions of edge n1-n2 in the given order
local function getEdgeNodeRadii(n1id, n2id)
  if not (map.nodes[n1id] and map.nodes[n2id] and (map.nodes[n1id].links[n2id] or map.nodes[n2id].links[n1id])) then return end
  local edgeData = map.nodes[n1id].links[n2id] or map.nodes[n2id].links[n1id]
  return edgeData[edgeData.inNode == n1id and 'inRadius' or 'outRadius'] or map.nodes[n1id].radius,
         edgeData[edgeData.inNode == n2id and 'inRadius' or 'outRadius'] or map.nodes[n2id].radius
end

-- returns the node position of node n1 of edge
local function getEdgeNodeRadius(nid, edge)
  -- nid: a node id (string)
  -- edge: an edge data table.(ex. given node nid and n2id the edge data table is map.nodes[nid].links[n2id])
  return edge[edge.inNode == nid and 'inRadius' or 'outRadius'] or map.nodes[nid].radius
end

local function createSpeedLimits(metric)
  local list = metric and {30, 50, 60, 80, 100, 120, huge} or {20, 35, 40, 50, 60, 70, huge} -- common speed limits
  local unit = metric and 3.6 or 2.24
  local baseValue = 19.444 -- 70 km/h

  for nid, n in pairs(map.nodes) do
    for lid, edge in pairs(n.links) do
      if not edge.speedLimit then
        local speedLimit = huge
        local nidRad, lidRad = getEdgeNodeRadii(nid, lid)
        local radius = (nidRad + lidRad) * 0.5
        local highway = edge.oneWay and 2 or 1
        local autoSpeed = baseValue * clamp(((radius * highway + 5) / 8) * edge.drivability, 0.4, 2)

        for i, speed in ipairs(list) do
          speed = speed / unit
          if speed > autoSpeed then
            autoSpeed = i == 1 and speed or list[i - 1] / unit -- round down to previous speed in list
            speedLimit = min(speedLimit, autoSpeed)
            break
          end
        end

        edge.speedLimit = speedLimit
      end
    end
  end
end

-- returns minimum point in x, y, z order comparison or nil if coordinates are equal
local function min3D(a, b)
  -- TODO: returns nil if all equal
  if a.x ~= b.x then
    if a.x < b.x then
      return a
    else
      return b
    end
  elseif a.y ~= b.y then
    if a.y < b.y then
      return a
    else
      return b
    end
  elseif a.z ~= b.z then
    if a.z < b.z then
      return a
    else
      return b
    end
  else
    return a
  end
end

-- less than function for nodes
-- returns true if n1 < n2
local function nodeCompare(n1, n2)
  if n1.pos.x ~= n2.pos.x then
    return n1.pos.x < n2.pos.x
  elseif n1.pos.y ~= n2.pos.y then
    return n1.pos.y < n2.pos.y
  elseif n1.pos.z ~= n2.pos.z then
    return n1.pos.z < n2.pos.z
  elseif n1.radius ~= n2.radius then
    return n1.radius < n2.radius
  elseif n1.manual ~= n2.manual then
    return (n1.manual or 0) < (n2.manual or 0)
  elseif n1.noMerge ~= n2.noMerge then
    return (n1.noMerge and 1 or 0) < (n2.noMerge and 1 or 0)
  elseif n1.endNode ~= n2.endNode then
    return (n1.endNode and 1 or 0) < (n2.endNode and 1 or 0)
  elseif tableSize(n1.links) ~= tableSize(n2.links) then
    return tableSize(n1.links) < tableSize(n2.links)
  else
    return false
  end
end

local function hashNodeData()
  local nodeList = {}
  for _, node in _pairs(map.nodes) do
    table.insert(nodeList, node)
  end
  table.sort(nodeList, nodeCompare)
  for i = 1, #nodeList do
    local node = nodeList[i]
    tmpBuf:put(node.pos.x, node.pos.y, node.pos.z, node.radius, node.manual or 0, node.noMerge and 1 or 0, node.endNode and 1 or 0, tableSize(node.links))
  end
  return hashStringSHA256(tmpBuf:get())
end

-- local function sortEdgeNodes(edge)
--   local n1id, n1Pos = next(edge.pos)
--   local n2id, n2Pos = next(edge.pos, n1id)
--   if n1Pos.x ~= n2Pos.x then
--     if n1Pos.x < n2Pos.x then
--       return n1id, n2id
--     else
--       return n2id, n1id
--     end
--   elseif n1Pos.y ~= n2Pos.y then
--     if n1Pos.y < n2Pos.y then
--       return n1id, n2id
--     else
--       return n2id, n1id
--     end
--   elseif n1Pos.z ~= n2Pos.z then
--     if n1Pos.z < n2Pos.z then
--       return n1id, n2id
--     else
--       return n2id, n1id
--     end
--   elseif edge.radius[n1id] ~= edge.radius[n2id] then
--     if edge.radius[n1id] < edge.radius[n2id] then
--       return n1id, n2id
--     else
--       return n2id, n1id
--     end
--   else
--     return n1id, n2id
--   end
-- end

local function sortEdgeNodes(edge)
  local inPos, inRadius = edge.inPos, edge.inRadius
  local outPos, outRadius = edge.outPos, edge.outRadius
  if inPos.x ~= outPos.x then
    if inPos.x < outPos.x then
      return inPos, inRadius, outPos, outRadius
    else
      return outPos, outRadius, inPos, inRadius
    end
  elseif inPos.y ~= outPos.y then
    if inPos.y < outPos.y then
      return inPos, inRadius, outPos, outRadius
    else
      return outPos, outRadius, inPos, inRadius
    end
  elseif inPos.z ~= outPos.z then
    if inPos.z < outPos.z then
      return inPos, inRadius, outPos, outRadius
    else
      return outPos, outRadius, inPos, inRadius
    end
  elseif inRadius ~= outRadius then
    if inRadius < outRadius then
      return inPos, inRadius, outPos, outRadius
    else
      return outPos, outRadius, inPos, inRadius
    end
  else
    return inPos, inRadius, outPos, outRadius
  end
end

-- -- returns the node id of the minimum of the two nodes of edge
-- -- There is an expectation here that the two nodes will not be equal
-- local function minEdgeNode(edge)
--   local inPos, inRadius = edge.inPos, edge.inRadius
--   local outPos, outRadius = edge.outPos, edge.outRadius
--   if inPos.x ~= outPos.x then
--     if inPos.x < outPos.x then
--       return edge.inNode
--     else
--       return edge.outNode
--     end
--   elseif inPos.y ~= outPos.y then
--     if inPos.y < outPos.y then
--       return edge.inNode
--     else
--       return edge.outNode
--     end
--   elseif inPos.z ~= outPos.z then
--     if inPos.z < outPos.z then
--       return edge.inNode
--     else
--       return edge.outNode
--     end
--   elseif inRadius ~= outRadius then
--     if inRadius < outRadius then
--       return edge.inNode
--     else
--       return edge.outNode
--     end
--   else
--     return edge.inNode
--   end
-- end

-- -- ordering function for edges
-- local function edgeCompare(a, b)
--   local aMin, aMax = sortEdgeNodes(a)
--   local bMin, bMax = sortEdgeNodes(b)

--   if a.pos[aMin].x ~= b.pos[bMin].x then
--     return a.pos[aMin].x < b.pos[bMin].x
--   elseif a.pos[aMin].y ~= b.pos[bMin].y then
--     return a.pos[aMin].y < b.pos[bMin].y
--   elseif a.pos[aMin].z ~= b.pos[bMin].z then
--     return a.pos[aMin].z < b.pos[bMin].z
--   elseif a.radius[aMin] ~= b.radius[bMin] then
--     return a.radius[aMin] < b.radius[bMin]
--   elseif a.pos[aMax].x ~= b.pos[bMax].x then
--     return a.pos[aMax].x < b.pos[bMax].x
--   elseif a.pos[aMax].y ~= b.pos[bMax].y then
--     return a.pos[aMax].y < b.pos[bMax].y
--   elseif a.pos[aMax].z ~= b.pos[bMax].z then
--     return a.pos[aMax].z < b.pos[bMax].z
--   elseif a.radius[aMax] ~= b.radius[bMax] then
--     return a.radius[aMax] < b.radius[bMax]
--   elseif a.drivability ~= b.drivability then
--     return a.drivability < b.drivability
--   elseif a.oneWay ~= b.oneWay then
--     return (a.oneWay and 1 or 0) < (b.oneWay and 1 or 0)
--   elseif a.speedLimit ~= b.speedLimit then
--     return (a.speedLimit or 0) < (b.speedLimit or 0)
--   elseif a.type ~= b.type then
--     return (a.type or '') < (b.type or '')
--   elseif a.noMerge ~= b.noMerge then
--     return (a.noMerge and 1 or 0) < (b.noMerge and 1 or 0)
--   else -- inNode?
--     return false
--   end
-- end

-- less than function for edges
-- returns true if a < b
local function edgeCompare(a, b)
  local aPosMin, aRadMin, aPosMax, aRadMax = sortEdgeNodes(a) -- do i need to sort, why not just always compare in a vs in b and out a vs out b?
  local bPosMin, bRadMin, bPosMax, bRadMax = sortEdgeNodes(b)

  if aPosMin.x ~= bPosMin.x then
    return aPosMin.x < bPosMin.x
  elseif aPosMin.y ~= bPosMin.y then
    return aPosMin.y < bPosMin.y
  elseif aPosMin.z ~= bPosMin.z then
    return aPosMin.z < bPosMin.z
  elseif aRadMin ~= bRadMin then
    return aRadMin < bRadMin
  elseif aPosMax.x ~= bPosMax.x then
    return aPosMax.x < bPosMax.x
  elseif aPosMax.y ~= bPosMax.y then
    return aPosMax.y < bPosMax.y
  elseif aPosMax.z ~= bPosMax.z then
    return aPosMax.z < bPosMax.z
  elseif aRadMax ~= bRadMax then
    return aRadMax < bRadMax
  elseif a.drivability ~= b.drivability then
    return a.drivability < b.drivability
  elseif a.oneWay ~= b.oneWay then
    return (a.oneWay and 1 or 0) < (b.oneWay and 1 or 0)
  elseif a.speedLimit ~= b.speedLimit then
    return (a.speedLimit or 0) < (b.speedLimit or 0)
  elseif a.lanes ~= b.lanes then
    return a.lanes < b.lanes
  elseif a.type ~= b.type then
    return (a.type or '') < (b.type or '')
  elseif a.noMerge ~= b.noMerge then
    return (a.noMerge and 1 or 0) < (b.noMerge and 1 or 0)
  else -- inNode?
    return false
  end
end

local function hashEdgeData()
  local edgeList = {}
  for n1id, node1 in _pairs(map.nodes) do
    for n2id, edge in _pairs(node1.links) do
      if n1id < n2id or not map.nodes[n2id].links[n1id] then -- second condition checks if edge is single sided
        table.insert(edgeList, edge)
      end
    end
  end
  table.sort(edgeList, edgeCompare)
  for i = 1, #edgeList do
    local edge = edgeList[i]
    local nodeMinPos, nodeMinRad, nodeMaxPos, nodeMaxRad = sortEdgeNodes(edge)
    local inPos, outPos = getEdgeNodePositions(edge.inNode, edge.outNode)
    local inRadius, outRadius = getEdgeNodeRadii(edge.inNode, edge.outNode)
    tmpBuf:put(
      nodeMinPos.x, nodeMinPos.y, nodeMinPos.z, nodeMinRad,
      nodeMaxPos.x, nodeMaxPos.y, nodeMaxPos.z, nodeMaxRad,
      inPos.x, inPos.y, inPos.z,
      outPos.x, outPos.y, outPos.z,
      inRadius, outRadius,
      edge.drivability,
      edge.oneWay and 1 or 0,
      edge.speedLimit or 0,
      edge.lanes,
      edge.type or 0,
      edge.noMerge and 1 or 0
    )
  end
  return hashStringSHA256(tmpBuf:get())
end

local function logGraphHashes()
  log('I', 'node hash', hashNodeData())
  log('I', 'edge hash', hashEdgeData())
end

local function surfaceNormal(p, r)
  --   p3
  --     \
  --      \ r
  --       \     r
  --        p - - - - p1     | - > y
  --       /                 v
  --      / r                x
  --     /
  --   p2

  r = r or 2
  local hr = 1.2 * r -- calculation is guaranteed to be accurate up to ~ 50 deg (argtan(1.2)) inclination

  local p1 = vec3(p.x, p.y + r, p.z + hr)
  local p2 = vec3(p.x + 0.8660254037844386 * r, p.y - 0.5 * r, p1.z) -- sin(60) => 0.8660254037844386, cos(60) => 0.5
  local p3 = p2:copy()
  p3.x = p.x - 0.8660254037844386 * r

  p1.z = be:getSurfaceHeightBelow(p1)
  p2.z = be:getSurfaceHeightBelow(p2)
  p3.z = be:getSurfaceHeightBelow(p3)

  -- store the result in p3
  if min(p1.z, p2.z, p3.z) + hr < p.z then
    p3:set(0, 0, 1)
  else
    p2:setSub(p3)
    p1:setSub(p3)
    p3:set(p2.y * p1.z - p2.z * p1.y, p2.z * p1.x - p2.x * p1.z, p2.x * p1.y - p2.y * p1.x) -- p2 x p1
    p3:normalize()
  end

  return p3
end

local function numOfLanesFromRadius(rad1, rad2)
  return max(1, math.floor(min(rad1, rad2 or math.huge) * 2 / 3.61 + 0.5)) -- math.floor(min(rad1, rad2) / 2.7) + 1
end

-- Returns the lane configuration of an edge as traversed in the inNode -> outNode direction (predessesor and succesor if along a path)
-- if an edge does not have lane data they are deduced from the node radii
local function getEdgeLaneConfig(node1, node2)
  local lanes
  local edge = map.nodes[node1].links[node2]
  local rad1, rad2 = getEdgeNodeRadii(edge.inNode, edge.outNode)
  if edge.oneWay then
    local numOfLanes = numOfLanesFromRadius(rad1, rad2)
    lanes = string.rep("+", numOfLanes)
  else
    local numOfLanes = max(1, math.floor(numOfLanesFromRadius(rad1, rad2) * 0.5))
    if rules.rightHandDrive then
      lanes = string.rep("+", numOfLanes)..string.rep("-", numOfLanes)
    else
      lanes = string.rep("-", numOfLanes)..string.rep("+", numOfLanes)
    end
  end

  return lanes
end

local function loadJsonDecalMap()
  local mapNodes = map.nodes

  -- load BeamNG Waypoint data
  manualWaypoints = {}
  for _, nodeName in ipairs(scenetree.findClassObjects('BeamNGWaypoint')) do
    local o = scenetree.findObject(nodeName)
    if o and (not o.excludeFromMap) and mapNodes[nodeName] == nil then
      local radius = getSceneWaypointRadius(o)
      local pos = o:getPosition()
      mapNodes[nodeName] = {pos = pos, radius = radius, links = {}, manual = 1}
      manualWaypoints[nodeName] = {pos = vec3(pos), radius = radius}
    end
  end

  _updateProgress()

  do -- load DecalRoad data
    local nodePos, nodeSqRad, stack, stackIdx = {}, {}, {}, 0
    for _, decalRoadName in ipairs(scenetree.findClassObjects('DecalRoad')) do
      local road = scenetree.findObject(decalRoadName)
      if road and road.drivability > 0 then
        local edgeCount = road:getEdgeCount()
        local nodeCount = road:getNodeCount()
        if max(edgeCount, nodeCount) > 1 then
          local prefix = (tonumber(decalRoadName) and 'DR'..decalRoadName..'_') or decalRoadName
          local drivability = road.drivability
          local hiddenInNavi = road.hiddenInNavi
          local roadType = road.gatedRoad and 'private' or road.type -- TODO: deprecate gatedRoad if we get more road types?
          local oneWay = road.oneWay or false
          local speedLimit = road.speedLimit
          local flipDirection = road.flipDirection or false
          local noMerge
          if not road.autoJunction then
            noMerge = true
          end

          if speedLimit then
            speedLimit = tonumber(speedLimit)
            if speedLimit <= 0 then
              speedLimit = nil -- auto calculate the speed limit
            end
          end

          local lanes -- string encoding for lanes (currently: dir)
          -- if not road.autoLanes then
          --   if oneWay then
          --     lanes = ('+'):rep(max(1, road.lanesRight))
          --   else
          --     lanes = ('-'):rep(road.lanesLeft or 0)..('+'):rep(road.lanesRight or 0) -- max(1, road.lanesLeft), max(1, road.lanesRight)
          --   end
          -- end

          if not road.autoLanes then
            local lanesLeft = max(0, road.lanesLeft or 0)
            local lanesRight = max(0, road.lanesRight or 0)
            if rules.rightHandDrive then
              lanes = ('+'):rep(lanesLeft)..('-'):rep(lanesRight)
            else
              lanes = ('-'):rep(lanesLeft)..('+'):rep(lanesRight)
            end
            oneWay = isOneWay(lanes)
          end

          if edgeCount > 2 and edgeCount >= nodeCount and road.useSubdivisions then -- use decalRoad edge (subdivision) data to generate AI path
            -- Polyline simplification: Radial distance
            local count, segCount, warningCount = 0, edgeCount - 1, 0
            for i = 0, segCount do
              local pos = road:getMiddleEdgePosition(i)
              local radius = min(pos:squaredDistance(road:getLeftEdgePosition(i)), pos:squaredDistance(road:getRightEdgePosition(i)))
              if i == 0 or i == segCount or pos:squaredDistance(nodePos[count]) >= 4 * min(nodeSqRad[count], radius) then
                count = count + 1
                nodePos[count] = pos
                nodeSqRad[count] = radius
              end
              if radius > 400 then warningCount = warningCount + 1 end
            end

            if warningCount > 0 then
              log('W', "map", "Road "..prefix.." centerline to edge distance exceeding 20m on "..warningCount.." counts.")
            end

            -- Polyline simplification: Ramer-Douglas-Peucker algorithm
            local i, k = 1, count
            count = 1
            local nodeName = nameNode(prefix, count)
            mapNodes[nodeName] = {pos = nodePos[1], radius = sqrt(nodeSqRad[1]), links = {}, noMerge = noMerge, endNode = true}
            local prevName = nodeName

            repeat
              local d2max, jmax = 0, nil
              local pi, pk = nodePos[i], nodePos[k]
              for j = i+1, k-1 do
                local sqDist = nodePos[j]:squaredDistanceToLineSegment(pi, pk)
                if sqDist > d2max then
                  d2max = sqDist
                  jmax = j
                end
              end

              if jmax and d2max > max(0.005 * nodeSqRad[jmax], 0.04) then
                stackIdx = stackIdx + 1
                stack[stackIdx] = k
                k = jmax
              else
                count = count + 1
                nodeName = nameNode(prefix, count)
                local inNode = flipDirection and nodeName or prevName
                local outNode = inNode == nodeName and prevName or nodeName
                mapNodes[nodeName] = {pos = pk, radius = sqrt(nodeSqRad[k]), links = {}, noMerge = noMerge}
                local data = {
                  drivability = drivability,
                  hiddenInNavi = hiddenInNavi,
                  oneWay = oneWay,
                  lanes = lanes,
                  speedLimit = speedLimit,
                  inNode = inNode,
                  outNode = outNode,
                  type = roadType,
                  inPos = mapNodes[inNode].pos,
                  outPos = mapNodes[outNode].pos,
                  inRadius = mapNodes[inNode].radius,
                  outRadius = mapNodes[outNode].radius,
                  noMerge = noMerge
                }
                mapNodes[nodeName].links[prevName] = data
                mapNodes[prevName].links[nodeName] = data
                if not data.lanes then
                  data.lanes = getEdgeLaneConfig(prevName, nodeName)
                end
                prevName = nodeName

                i = k
                k = stack[stackIdx]
                stackIdx = stackIdx - 1
              end
            until not k
            mapNodes[nodeName].endNode = true -- set the last node of this road as an end node

            tableClear(nodePos)
            tableClear(nodeSqRad)
            tableClear(stack)
            stackIdx = 0
          else -- use decalRoad control point data to generate AI path
            local prevName = nameNode(prefix, 1)
            local lNode
            if road.looped and nodeCount > 2 then
              lNode = prevName
            end
            mapNodes[prevName] = {pos = road:getNodePosition(0), radius = road:getNodeWidth(0) * 0.5, links = {}, noMerge = noMerge, endNode = true}
            for i = 1, nodeCount - 1 do
              local nodeName = nameNode(prefix, i+1)
              local pos = road:getNodePosition(i)
              local radius = road:getNodeWidth(i) * 0.5
              local inNode = flipDirection and nodeName or prevName
              local outNode = inNode == nodeName and prevName or nodeName
              mapNodes[nodeName] = {pos = pos, radius = radius, links = {}, noMerge = noMerge}
              local data = {
                drivability = drivability,
                hiddenInNavi = hiddenInNavi,
                oneWay = oneWay,
                lanes = lanes,
                speedLimit = speedLimit,
                inNode = inNode,
                outNode = outNode,
                inPos = mapNodes[inNode].pos,
                outPos = mapNodes[outNode].pos,
                inRadius = mapNodes[inNode].radius,
                outRadius = mapNodes[outNode].radius,
                type = roadType,
                noMerge = noMerge
              }
              mapNodes[nodeName].links[prevName] = data
              mapNodes[prevName].links[nodeName] = data
              if not data.lanes then
                data.lanes = getEdgeLaneConfig(prevName, nodeName)
              end
              prevName = nodeName
            end
            mapNodes[prevName].endNode = true
            -- road is looped: add edge between first and last nodes
            if lNode then
              local inNode = flipDirection and lNode or prevName
              local data = {
                drivability = drivability,
                hiddenInNavi = hiddenInNavi,
                oneWay = oneWay,
                lanes = lanes,
                speedLimit = speedLimit,
                inNode = inNode,
                type = roadType,
                inPos = mapNodes[inNode].pos,
                outPos = mapNodes[inNode == lNode and prevName or lNode].pos,
                inRadius = mapNodes[inNode].radius,
                outRadius = mapNodes[inNode == lNode and prevName or lNode].radius,
                noMerge = noMerge
              }
              mapNodes[lNode].links[prevName] = data
              mapNodes[prevName].links[lNode] = data
            end
          end
        end
      end
    end
  end

  _updateProgress()

  -- load manual road segments
  local levelDir = path.split(getMissionFilename())
  if not levelDir then return end
  mapFilename = levelDir .. 'map.json'
  --log('D', 'map', 'loading map.json: '.. mapFilename)
  local content = readFile(mapFilename)
  if content == nil then
    --log('D', 'map', 'map system disabled due to missing/unreadable file: '.. mapFilename)
    return
  end

  _updateProgress()

  local state, jsonMap = pcall(json.decode, content)
  if state == false then
    log('W', 'map', 'unable to parse file: '.. mapFilename)
    return
  end

  if not jsonMap or not jsonMap.segments then
    log('W', 'map', 'map file is empty or invalid: '.. dumps(mapFilename))
    return
  end

  for _, v in pairs(jsonMap.segments) do
    if type(v.nodes) == 'string' then
      local nodeList = {}
      local nargs = split(v.nodes, ',')
      for _, nv in ipairs(nargs) do
        local nargs2 = split(nv, '-')
        if #nargs2 == 1 then
          tableInsert(nodeList, trim(nargs2[1]))
        elseif #nargs2 == 2 then
          local prefix1 = stringMatch(nargs2[1], "[^%d]+")
          local num1 = stringMatch(nargs2[1], "[%d]+")
          local prefix2 = stringMatch(nargs2[2], "[^%d]+")
          local num2 = stringMatch(nargs2[2], "[%d]+")
          if prefix1 ~= prefix2 then
            log('E', 'map', "segment format issue: not the same prefix: ".. tostring(nargs2[1]) .. " and " .. tostring(nargs2[2]) .. " > discarding nodes. Please fix")
          end
          for k = num1, num2 do
            tableInsert(nodeList, prefix1 .. tostring(k))
          end
        end
        v.nodes = nodeList
      end
    end

    local drivability = max(0, v.drivability or 1)
    local hiddenInNavi = v.hiddenInNavi
    local roadType = v.gatedRoad and 'private' or v.type
    local speedLimit = v.speedLimit
    local flipDirection = v.flipDirection or false
    local oneWay = v.oneWay or false

    local lanes
    -- if v.autoLanes == false then
    --   local lanesLeft = v.lanesLeft or 0
    --   local lanesRight = v.lanesRight or 0
    --   if oneWay then
    --     lanes = ('+'):rep(max(1, lanesRight))
    --   else
    --     lanes = ('-'):rep(max(1, lanesLeft))..('+'):rep(max(1, lanesRight))
    --   end
    -- end

    if v.autoLanes == false then
      local lanesLeft = max(0, v.lanesLeft or 0)
      local lanesRight = max(0, v.lanesRight or 0)
      if rules.rightHandDrive then
        lanes = ('+'):rep(lanesLeft)..('-'):rep(lanesRight)
      else
        lanes = ('-'):rep(lanesLeft)..('+'):rep(lanesRight)
      end
      oneWay = isOneWay(lanes)
    end

    local noMerge
    if v.autoJunction == false then
      noMerge = true
    end
    local wp1 = v.nodes[1]
    if mapNodes[wp1] then
      mapNodes[wp1].noMerge = noMerge
      mapNodes[wp1].endNode = true
      local nodeCount = #v.nodes
      for i = 2, nodeCount do
        local wp2 = v.nodes[i]
        if wp2 ~= wp1 then -- guards against a node name appearing consequtively in the nodelist.
          if mapNodes[wp2] == nil then
            log('E', 'map', "the manual waypoint (beamNGWaypoint) "..tostring(wp2).." referenced in "..mapFilename.." does not exist")
            break
          end
          mapNodes[wp2].noMerge = noMerge
          local inNode = flipDirection and wp2 or wp1
          local outNode = inNode == wp2 and wp1 or wp2
          local data = {
            drivability = drivability,
            hiddenInNavi = hiddenInNavi,
            oneWay = oneWay,
            lanes = lanes,
            speedLimit = speedLimit,
            inNode = inNode,
            outNode = outNode,
            type = roadType,
            inPos = mapNodes[inNode].pos,
            outPos = mapNodes[outNode].pos,
            inRadius = mapNodes[inNode].radius,
            outRadius = mapNodes[outNode].radius,
            noMerge = noMerge
          }
          mapNodes[wp1].links[wp2] = data
          mapNodes[wp2].links[wp1] = data
          if not data.lanes then
            data.lanes = getEdgeLaneConfig(wp1, wp2)
          end
          if i == nodeCount then mapNodes[wp2].endNode = true end
          wp1 = wp2
        end
      end
    else
      log('E', 'map', "manual waypoint not found: "..tostring(wp1));
    end
  end

  _updateProgress()
end

local function checkNodeMatches(nodeList1, nodeList2) -- TODO: add node attributes to match checks
  local list1Count, list2Count = #nodeList1, #nodeList2
  if list1Count ~= list2Count then
    print('!!!!!!!!!!!!!!!! Node List Sizes Do Not Match ('..list1Count..' - '..list2Count..') !!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    return
  end

  local nodeTree = kdTreeBox2D.new(list2Count)
  for i = 1, list2Count do
    nodeTree:preLoad(i, quadtree.pointBBox(nodeList2[i].pos.x, nodeList2[i].pos.y, nodeList2[i].radius))
  end
  nodeTree:build()

  local matchedL2 = {}
  for i = 1, list1Count do
    for j in nodeTree:queryNotNested(quadtree.pointBBox(nodeList1[i].pos.x, nodeList1[i].pos.y, nodeList1[i].radius)) do
      if not matchedL2[j] then
        if nodeList1[i].degree == nodeList2[j].degree
        and nodeList1[i].radius == nodeList2[j].radius
        and nodeList1[i].pos == nodeList2[j].pos
        and nodeList1[i].noMerge == nodeList2[j].noMerge
        and nodeList1[i].endNode == nodeList2[j].endNode
        and nodeList1[i].manual == nodeList2[j].manual
        then
          matchedL2[j] = i
          break
        end
      end
    end
  end

  local nodesNotMatchedCount = list2Count - tableSize(matchedL2)
  if nodesNotMatchedCount > 0 then
    print('!!!!!!!!!!!!!!!!! Did not find match for '..nodesNotMatchedCount..' nodes !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    local matchedL1 = {}
    for i = 1, nodeList2 do
      if not matchedL2[i] then
        dump(nodeList2[i])
      else
        matchedL1[matchedL2[i]] = i
      end
    end
    print('-------------')
    for i = 1, nodeList1 do
      if not matchedL1[i] then
        dump(nodeList1[i])
      end
    end
  else
    print('Matches found for all ('..list1Count..') nodes.')
  end
end

local function checkEdgeMatches(edgeList1, edgeList2)
  local edgeList1Count, edgeList2Count = #edgeList1, #edgeList2
  if edgeList1Count ~= edgeList2Count then
    print('!!!!!!!!!!!!!!!! Edge List Sizes Do Not Match ('..edgeList1..' - '..edgeList2..') !!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    return
  end

  local edgeTree = kdTreeBox2D.new(edgeList2Count)
  for i = 1, edgeList2Count do
    local n1id, n1Pos = next(edgeList2[i].pos)
    local n2id, n2Pos = next(edgeList2[i].pos, n1id)
    local n1Rad = edgeList2[i].radius[n1id]
    local n2Rad = edgeList2[i].radius[n2id]
    edgeTree:preLoad(i, quadtree.lineBBox(n1Pos.x, n1Pos.y, n2Pos.x, n2Pos.y, max(n1Rad, n2Rad)))
  end
  edgeTree:build()

  local matchedL2 = {}
  for i = 1, edgeList1Count do
    local e1 = edgeList1[i]
    local e1n1Id, e1n1Pos = next(edgeList1[i].pos)
    local e1n2Id, e1n2Pos = next(edgeList1[i].pos, e1n1Id)
    local e1n1Rad = e1.radius[e1n1Id]
    local e1n2Rad = e1.radius[e1n2Id]
    local e1NodeMin = min3D(e1n1Pos, e1n2Pos) == e1n1Pos and e1n1Id or e1n2Id
    local e1NodeMax = e1NodeMin == e1n1Id and e1n2Id or e1n1Id
    local e1InNodePos = e1.pos[e1.inNode]
    for j in edgeTree:queryNotNested(quadtree.lineBBox(e1n1Pos.x, e1n1Pos.y, e1n2Pos.x, e1n2Pos.y, max(e1n1Rad, e1n2Rad))) do
      if not matchedL2[j] then
        local e2 = edgeList2[j]
        local e2n1Id, e2n1Pos = next(edgeList2[j].pos)
        local e2n2Id, e2n2Pos = next(edgeList2[j].pos, e2n1Id)
        local e2NodeMin = min3D(e2n1Pos, e2n2Pos) == e2n1Pos and e2n1Id or e2n2Id
        local e2NodeMax = e2NodeMin == e2n1Id and e2n2Id or e2n1Id
        local inNodesMatch = (not e2.oneWay and not e2.oneWay) or (e2.oneWay and e1.oneWay and e2.pos[e2.inNode] == e1InNodePos)
        if e1.pos[e1NodeMin] == e2.pos[e2NodeMin] and e1.pos[e1NodeMax] == e2.pos[e2NodeMax] -- positions
        and e1.radius[e1NodeMin] == e2.radius[e2NodeMin] and e1.radius[e1NodeMax] == e2.radius[e2NodeMax] -- radii
        and e2.drivability == e1.drivability
        and e2.oneWay == e1.oneWay
        and inNodesMatch
        and e2.speedLimit == e1.speedLimit
        and e2.type == e1.type
        and e2.noMerge == e1.noMerge
        then
          matchedL2[j] = i
          break
        end
      end
    end
  end

  local edgesNotMatchedCount = edgeList2Count - tableSize(matchedL2)
  if edgesNotMatchedCount > 0 then
    print('!!!!!!!!!!!!!!!!!!!!!!!!!!! DID NOT FIND MATCH FOR '..edgesNotMatchedCount..' edges !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    local matchedL1 = {}
    for i = 1, edgeList2Count do
      if not matchedL2[i] then
        dump(edgeList2[i])
      else
        matchedL1[edgeList2[i]] = i
      end
    end
    print('--------------------')
    for i = 1, edgeList1Count do
      if not matchedL1[i] then
        dump(edgeList1[i])
      end
    end
  else
    print('Matches found for all ('..edgeList1Count..') edges.')
  end
end

local function checkLinks()
  local linksOk = true
  local numOfLinks = 0
  local numOfNodes = 0
  for n1id, node in _pairs(map.nodes) do
    numOfNodes = numOfNodes + 1
    for n2id, edgeData in _pairs(node.links) do
      if not map.nodes[n2id] then
        print('!!!! Link is not a map node !!!!!')
        linksOk = false
      end
      if n1id == n2id then
        print('!!!! Link is connected to itself !!!!')
        linksOk = false
      end
      -- if not edgeData.pos[n1id] or not edgeData.pos[n2id] or not edgeData.radius[n1id] or not edgeData.radius[n2id] then
      --   print('!!!! edge Node ids do not match graph ids !!!!')
      --   linksOk = false
      -- end
      if linksOk then numOfLinks = numOfLinks + 1 end
    end
  end

  if linksOk and (numOfLinks % 2 == 0) then
    dump('---> Links Are Ok. -> ', numOfLinks * 0.5, numOfNodes)
  else
    print('!!!!!! Links Not Ok !!!!!!!')
  end
  return linksOk
end

local function is2SegMergeValid(middleNode, d1, d2)
  if d1.lanes and d2.lanes then
    if d1.inNode == d2.inNode or (d1.inNode ~= middleNode and d2.inNode ~= middleNode) then -- reverse the string of one of the edges and compare to the other
      return d1.lanes == flipLanes(d2.lanes)
    else
      return d1.lanes == d2.lanes
    end
  elseif not d1.lanes and not d2.lanes then
    if d1.oneWay == d2.oneWay then
      return d1.oneWay == false or not (d1.inNode == d2.inNode or (d1.inNode ~= middleNode and d2.inNode ~= middleNode))
    else
      return false
    end
  else
    return false
  end
end

local function is3SegMergeValid(middleNode, d1, d2, dchord)
  if is2SegMergeValid(middleNode, d1, d2) then
    if d1.lanes and dchord.lanes then
      if d1.inNode == dchord.inNode then
        return d1.lanes == dchord.lanes
      elseif d2.inNode == dchord.inNode then
        return d2.lanes == dchord.lanes
      else
        return false
      end
    elseif not d1.lanes and not dchord.lanes then
      if dchord.oneWay == d1.oneWay then
        return (dchord.oneWay == false) or (d1.inNode == dchord.inNode or d2.inNode == dchord.inNode)
      else
        return false
      end
    else
      return false
    end
  else
    return false
  end
end

local function mergeEdgeData(e1, e2)
  local drivability = max(e1.drivability, e2.drivability)
  local type = e1.type or e2.type -- TODO: This will be a problem when/if we have more than two types
  local speedLimit = max(e1.speedLimit, e2.speedLimit)
  local noMerge = e1.noMerge or e2.noMerge
  local e1n1Id = next(e1.pos)
  local e1n2Id = next(e1.pos, e1n1Id)
  local e2n1Id = next(e2.pos)
  local e2n2Id = next(e2.pos, e2n1Id)
  local commonNodeId = (e1n1Id == e2n1Id or e1n1Id == e2n2Id) and e1n1Id or e1n2Id
  local oneWay = e1.oneWay or e2.oneWay
  local inNode
  if e1.oneWay == e2.oneWay then
    if e1.oneWay then
      if e1.inNode == e2.inNode then
        inNode = e1.inNode
      elseif e1.inNode ~= commonNodeId and e2.inNode ~= commonNodeId then
        inNode = commonNodeId == e1n1Id and e1n2Id or e1n1Id
      else -- TODO: both segments are oneWay but in opposite directions. Use compare edges to decide which to keep?
        inNode = commonNodeId == e1n1Id and e1n2Id or e1n1Id
      end
    else -- if both edges are two way inNode selection is immaterial.
      inNode = e1.inNode
    end
  else
    if e1.oneWay then
      inNode = e1.inNode
    else
      inNode = (e2.inNode == commonNodeId and commonNodeId) or (commonNodeId == e1n1Id and e1n2Id or e1n1Id)
    end
  end

  return drivability, oneWay, inNode, speedLimit, type, noMerge
end

local function mergeLinks(n1id, n2id)
  local mapNodes = map.nodes
  if mapNodes[n2id].manual then --> TODO: what if both are manual?
    n1id, n2id = n2id, n1id
  end

  local n1 = mapNodes[n1id]
  local n2 = mapNodes[n2id]

  -- if n1 is already linked with n2
  n1.links[n2id] = nil
  n2.links[n1id] = nil

  -- remap neighbors of n2 to n1
  for lnid, edgeData in _pairs(n2.links) do
    local ln = mapNodes[lnid]
    if ln then
      if n1.links[lnid] and edgeCompare(n1.links[lnid], edgeData) then
        ln.links[n2id] = nil -- delete link to n2id (n2id will cease to exist after the merge)
        n2.links[lnid] = nil
      else
        edgeData.inNode = edgeData.inNode == n2id and n1id or lnid
        edgeData.outNode = edgeData.inNode == n1id and lnid or n1id
        ln.links[n2id] = nil -- delete link to n2id (n2id will cease to exist after the merge)
        ln.links[n1id] = edgeData -- reference the edge data to the other side of the edge
        n1.links[lnid] = edgeData
      end
    end
  end

  mapNodes[n2id] = nil

  return n1id
end

-- merge groups of overlapping nodes
local function mergeOverlappingNodes(endNodesOnly)
  local mapNodes = map.nodes

  local nodeCount = 0
  local q = kdTreeBox2D.new()
  for k, v in _pairs(mapNodes) do
    if not (v.noMerge or endNodesOnly) or v.endNode then
      q:preLoad(k, pointBBox(v.pos.x, v.pos.y, v.radius))
      nodeCount = nodeCount + 1
    end
  end
  q:build()

  -- create node overlap graph
  -- i.e. a graph whereby an edge is added between any two nodes that satisfy the overlap condition
  local nodeOverlapGraph = table.new(0, nodeCount)
  local groupIds = table.new(0, nodeCount)
  for n1id, n1 in _pairs(mapNodes) do
    if not (n1.noMerge or endNodesOnly) or n1.endNode then
      for n2id in q:queryNotNested(pointBBox(n1.pos.x, n1.pos.y, n1.radius)) do
        if n1id < n2id then
          local nodeDist = n1.pos:squaredDistance(mapNodes[n2id].pos)
          if (endNodesOnly and nodeDist < 0.01 and abs(n1.radius - mapNodes[n2id].radius) < 0.1)
            or (not endNodesOnly and nodeDist < square(max(n1.radius, mapNodes[n2id].radius))) then
            -- create edge between overlapping nodes (graph is one sided)
            if not nodeOverlapGraph[n1id] then
              nodeOverlapGraph[n1id] = {}
              groupIds[n1id] = n1id
            end
            nodeOverlapGraph[n1id][n2id] = true
            groupIds[n2id] = groupIds[n1id]
          end
        end
      end
    end
  end

  -- calculate node overlap graph connected components
  repeat
    local change = false
    for n1id, n1Links in _pairs(nodeOverlapGraph) do
      for n2id in _pairs(n1Links) do
        if groupIds[n1id] < groupIds[n2id] then
          groupIds[n2id] = groupIds[n1id]
          change = true
        elseif groupIds[n1id] > groupIds[n2id] then
          groupIds[n1id] = groupIds[n2id]
          change = true
        end
      end
    end
  until not change

  -- arrange nodes by the overlap group they belong to
  local nodeOverlapGroups = {} -- keys are group ids and values are tables containing all the nodes belonging to the group
  for nodeId, groupId in _pairs(groupIds) do
    if nodeOverlapGroups[groupId] then
      table.insert(nodeOverlapGroups[groupId], nodeId)
    else
      nodeOverlapGroups[groupId] = {nodeId}
    end
  end

  -- merge nodes that belong to the same overlap group
  local nodePosTab, nodeRadiusTab = {}, {}
  for groupId, nodeGroup in _pairs(nodeOverlapGroups) do
    local nid = nodeGroup[1]

    nodePosTab[1] = mapNodes[nid].pos
    nodeRadiusTab[1] = mapNodes[nid].radius

    local count = #nodeGroup
    for i = 2, count do
      nodePosTab[i] = mapNodes[nodeGroup[i]].pos
      nodeRadiusTab[i] = mapNodes[nodeGroup[i]].radius
      nid = mergeLinks(nid, nodeGroup[i])
    end

    mapNodes[nid].pos = safeVecSum(nodePosTab) / count
    mapNodes[nid].radius = safeNumberSum(nodeRadiusTab) / count
    mapNodes[nid].noMerge = nil
    mapNodes[nid].endNode = nil -- TODO: does it make sense to retain the endNode attribute if true for any of the merged nodes?
    mapNodes[nid].junction = true

    table.clear(nodePosTab)
    table.clear(nodeRadiusTab)
  end

  _updateProgress()
end

-- Creates a list of all the edges in the graph -> {node1, node2, edgeData}
local function getEdgeList()
  local edges = {}
  for n1id, node in _pairs(map.nodes) do
    for n2id, edgeData in _pairs(node.links) do
      if n1id ~= n2id and map.nodes[n2id] then -- TODO: why is this check needed? (first check is needed because of possible errors in manual edges map.json)
        if n1id < n2id then -- every edge gets in the array once
          -- local a, b
          -- -- TODO: depends on edge end poins not being identical, a fair assumption for an edge, although theoretically not guaranteed
          -- if min3D(edgeData.pos[n1id], edgeData.pos[n2id]) == edgeData.pos[n1id] then
          --   a, b = n1id, n2id
          -- else
          --   a, b = n2id, n1id
          -- end
          -- tableInsert(edges, {a, b, edgeData}) -- TODO: I don't need to save edge node names anymore. i can access them from either the pos or radius tables in edgeData
          tableInsert(edges, {edgeData.inNode, edgeData.outNode, edgeData}) -- TODO: I don't need to save edge node names anymore. i can access them from either the pos or radius tables in edgeData
        end
      end
    end
  end

  _updateProgress()

  return edges
end

local function resolveTJunction(edges, q_edges, i, l1n1id) -- could use 'inPos' and 'outPos' instead of node id (l1n1id)?
  local l1Data = edges[i][3]
  local l1n1pos, l1n1rad = l1Data[l1Data.inNode == l1n1id and 'inPos' or 'outPos'], l1Data[l1Data.inNode == l1n1id and 'inRadius' or 'outRadius']

  local l1n2id = l1n1id == l1Data.inNode and l1Data.outNode or l1Data.inNode
  local l1n2pos = l1Data[l1Data.inNode == l1n2id and 'inPos' or 'outPos']

  local minXnorm = -huge
  local edgeId, l2Xnorm
  for l_id in q_edges:queryNotNested(pointBBox(l1n1pos.x, l1n1pos.y, l1n1rad)) do
    local l2n1id, l2n2id = edges[l_id][1], edges[l_id][2]
    if l1n1id ~= l2n1id and l1n1id ~= l2n2id and l1n2id ~= l2n1id and l1n2id ~= l2n2id then
      --local pos1 = l1n1pos + (l1n2pos - l1n1pos):normalized() * l1n1rad -- why do this?
      local l1xn, l2xn = closestLinePoints(l1n1pos, l1n2pos, edges[l_id][3].inPos, edges[l_id][3].outPos)
      if l2xn >= 0 and l2xn <= 1 and l1xn <= 0 and l1xn > minXnorm then -- find largest negative xnorm
        edgeId, minXnorm, l2Xnorm = l_id, l1xn, l2xn -- edge here is the horizontal part of the T junction
      end
    end
  end

  if edgeId then
    local l2n1pos = edges[edgeId][3].inPos
    local l2n2pos = edges[edgeId][3].outPos
    local l2n1rad = edges[edgeId][3].inRadius
    local l2n2rad = edges[edgeId][3].outRadius
    local l2Prad = lerp(l2n1rad, l2n2rad, l2Xnorm)
    local tempVec = (linePointFromXnorm(l2n1pos, l2n2pos, l1n1pos:xnormOnLine(l2n1pos, l2n2pos)) - l1n1pos):normalized() * (l2Prad + l1n1rad)
    if l1n1pos:squaredDistanceToLine(l2n1pos, l2n2pos) < tempVec:z0():squaredLength() then -- square(l2Prad + l1n1rad) -- Why z0?
      return edgeId, l2Xnorm
    end
  end
end

local function resolveTJunctions()
  local edges = getEdgeList()
  local edgeCount = #edges

  -- Create a kd-tree with map edges
  local q_edges = kdTreeBox2D.new(edgeCount)
  for i = 1, edgeCount do
    if not edges[i][3].noMerge then
      local inPos = edges[i][3].inPos
      local outPos = edges[i][3].outPos
      q_edges:preLoad(i, quadtree.lineBBox(inPos.x, inPos.y, outPos.x, outPos.y, max(edges[i][3].inRadius, edges[i][3].outRadius)))
    end
  end
  q_edges:build()

  local edgeSplits = {}
  for i = 1, edgeCount do
    local l1n1id = edges[i][1] -- the vertical edge in the T junction
    if tableSize(map.nodes[l1n1id].links) == 1 then
      local edgeId, xnorm = resolveTJunction(edges, q_edges, i, l1n1id)
      if edgeId then
        if edgeSplits[edgeId] then
          table.insert(edgeSplits[edgeId], {xnorm, l1n1id})
        else
          edgeSplits[edgeId] = {{xnorm, l1n1id}}
        end
      end
    end

    local l1n2id = edges[i][2]
    if tableSize(map.nodes[l1n2id].links) == 1 then
      local edgeId, xnorm = resolveTJunction(edges, q_edges, i, l1n2id)
      if edgeId then
        if edgeSplits[edgeId] then
          table.insert(edgeSplits[edgeId], {xnorm, l1n2id})
        else
          edgeSplits[edgeId] = {{xnorm, l1n2id}}
        end
      end
    end
  end

  local positions = {}
  local radii = {}
  for edgeId, splitData in _pairs(edgeSplits) do
    local n1 = edges[edgeId][1] -- edge inNode
    local n2 = edges[edgeId][2] -- edge outNode
    table.sort(splitData, function(a, b) return a[1] < b[1] end) -- TODO: what happens if equal?

    for i = 1, #splitData do
      positions[i] = linePointFromXnorm(map.nodes[n1].links[n2].inPos, map.nodes[n1].links[n2].outPos, splitData[i][1])
      radii[i] = lerp(map.nodes[n1].links[n2].inRadius, map.nodes[n1].links[n2].outRadius, splitData[i][1])
    end

    for i = 1, #splitData do
      local nodeName = splitData[i][2] -- node id of T-junction vertical edge

      map.nodes[nodeName].pos = positions[i]
      map.nodes[nodeName].radius = radii[i]
      map.nodes[nodeName].junction = true

      -- clear link between n1 and n2
      local data = map.nodes[n1].links[n2]
      map.nodes[n1].links[n2] = nil
      map.nodes[n2].links[n1] = nil

      -- connect n1 to nodeName
      --if map.nodes[n1].links[nodeName] then dump('1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', n1, nodeName, n2) end -- TODO: remove debug
      map.nodes[n1].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = n1,
        outNode = nodeName,
        inPos = data.inPos,
        outPos = positions[i],
        inRadius = data.inRadius,
        outRadius = radii[i],
        noMerge = data.noMerge
      }

      -- connect nodeName to n1
      map.nodes[nodeName].links[n1] = map.nodes[n1].links[nodeName]

      -- connect n2 to nodeName
      map.nodes[n2].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = nodeName,
        outNode = n2,
        inPos = positions[i],
        outPos = data.outPos,
        inRadius = radii[i],
        outRadius = data.outRadius,
        noMerge = data.noMerge
      }

      -- connect nodeName to n2
      map.nodes[nodeName].links[n2] = map.nodes[n2].links[nodeName]

      n1 = nodeName
    end
    table.clear(positions)
    table.clear(radii)
  end

  _updateProgress()
end

-- Resolve X junctions
local function resolveXJunctions()
  local edges = getEdgeList()
  local edgeCount = #edges

  -- Create a kd-tree with map edges
  local q_edges = kdTreeBox2D.new(edgeCount)
  for i = 1, edgeCount do
    if not edges[i][3].noMerge then
      local inPos = edges[i][3].inPos
      local outPos = edges[i][3].outPos
      q_edges:preLoad(i, quadtree.lineBBox(inPos.x, inPos.y, outPos.x, outPos.y, max(edges[i][3].inRadius, edges[i][3].outRadius)))
    end
  end
  q_edges:build()

  local edgeSplits = {}
  local junctionid = 0
  for i = 1, edgeCount do
    local l1n1id = edges[i][1] -- edge inNode id
    local l1n2id = edges[i][2] -- edge outNode id
    if not edges[i][3].noMerge then
      local l1n1pos = edges[i][3].inPos
      local l1n2pos = edges[i][3].outPos
      local l1n1rad = edges[i][3].inRadius
      local l1n2rad = edges[i][3].outRadius
      for j in q_edges:queryNotNested(quadtree.lineBBox(l1n1pos.x, l1n1pos.y, l1n2pos.x, l1n2pos.y)) do
        if j > i then
          local l2n1id = edges[j][1] -- edge inNode id
          local l2n2id = edges[j][2] -- edge outNode id
          if l1n1id ~= l2n1id and l1n1id ~= l2n2id and l1n2id ~= l2n1id and l1n2id ~= l2n2id then
            local l2n1pos = edges[j][3].inPos
            local l2n2pos = edges[j][3].outPos
            local l2n1rad = edges[j][3].inRadius
            local l2n2rad = edges[j][3].outRadius
            local l1xn, l2xn = closestLinePoints(l1n1pos, l1n2pos, l2n1pos, l2n2pos)
            if l1xn > 0 and l1xn < 1 and l2xn > 0 and l2xn < 1 then
              local t1 = linePointFromXnorm(l1n1pos, l1n2pos, l1xn)
              local t2 = linePointFromXnorm(l2n1pos, l2n2pos, l2xn)
              local l1Prad = lerp(l1n1rad, l1n2rad, l1xn)
              local l2Prad = lerp(l2n1rad, l2n2rad, l2xn)
              if t1:squaredDistance(t2) < square(min(l1Prad, l2Prad) * 0.5) then
                junctionid = junctionid + 1
                local nodeName = 'autojunction_'..junctionid

                if edgeSplits[i] then
                  table.insert(edgeSplits[i], {l1xn, nodeName})
                else
                  edgeSplits[i] = {{l1xn, nodeName}}
                end

                if edgeSplits[j] then
                  table.insert(edgeSplits[j], {l2xn, nodeName})
                else
                  edgeSplits[j] = {{l2xn, nodeName}}
                end
              end
            end
          end
        end
      end
    end
  end

  local positions, radii = {}, {}
  for edgeId, splitData in _pairs(edgeSplits) do
    local n1 = edges[edgeId][1]
    local n2 = edges[edgeId][2]
    table.sort(splitData, function(a, b) return a[1] < b[1] end)

    for j = 1, #splitData do
      positions[j] = linePointFromXnorm(map.nodes[n1].links[n2].inPos, map.nodes[n1].links[n2].outPos, splitData[j][1])
      radii[j] = lerp(map.nodes[n1].links[n2].inRadius, map.nodes[n1].links[n2].outRadius, splitData[j][1])
    end

    for j = 1, #splitData do
      local nodeName = splitData[j][2]
      local pos = positions[j]
      local radius = radii[j]

      if not map.nodes[nodeName] then
        map.nodes[nodeName] = {pos = pos, radius = radius, links = {}}
        map.nodes[nodeName].junction = true
      else
        map.nodes[nodeName].pos = (map.nodes[nodeName].pos + pos) * 0.5
        map.nodes[nodeName].radius = (map.nodes[nodeName].radius + radius) * 0.5
      end

      -- clear link between n1 and n2
      local data = map.nodes[n1].links[n2]
      map.nodes[n1].links[n2] = nil
      map.nodes[n2].links[n1] = nil

      -- connect n1 to nodeName
      map.nodes[n1].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = n1,
        outNode = nodeName,
        inPos = data.inPos,
        outPos = positions[j],
        inRadius = data.inRadius,
        outRadius = radii[j],
        noMerge = data.noMerge
      }

      -- connect nodeName to n1
      map.nodes[nodeName].links[n1] = map.nodes[n1].links[nodeName]

      -- connect n2 to nodeName
      map.nodes[n2].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = nodeName,
        outNode = n2,
        inPos = positions[j],
        outPos = data.outPos,
        inRadius = radii[j],
        outRadius = data.outRadius,
        noMerge = data.noMerge
      }

      -- connect nodeName to n2
      map.nodes[nodeName].links[n2] = map.nodes[n2].links[nodeName]

      n1 = nodeName
    end
    table.clear(positions)
    table.clear(radii)
  end

  _updateProgress()
end

-- Merge nodes to lines if they are closeby
local function mergeNodesToLines()
  local edges = getEdgeList()
  local edgeCount = #edges

  -- Create a quadtree with map edges
  local q_edges = kdTreeBox2D.new(edgeCount)
  for i = 1, edgeCount do
    if not edges[i][3].noMerge then
      local inPos = edges[i][3].inPos
      local outPos = edges[i][3].outPos
      q_edges:preLoad(i, quadtree.lineBBox(inPos.x, inPos.y, outPos.x, outPos.y, max(edges[i][3].inRadius, edges[i][3].outRadius)))
    end
  end
  q_edges:build()

  _updateProgress()

  local mapNodes = map.nodes

  local nodesToMerge = {}
  -- for every node find the edge that is closest and close enough to it. Each node can only merge with one edge.
  for nid, n in _pairs(mapNodes) do
    if not n.noMerge or n.endNode then
      local d2Min = math.huge
      local nodeMin, eIdMin, xMin
      for edgeId in q_edges:queryNotNested(quadtree.pointBBox(n.pos.x, n.pos.y, n.radius)) do
        local l1n1id = edges[edgeId][1] -- edge inNode id
        local l1n2id = edges[edgeId][2] -- edge outNode id
        if nid ~= l1n1id and nid ~= l1n2id then -- TODO: consider adding the following condition: and not (mapNodes[nid].links[l1n1id] or mapNodes[nid].links[l1n2id])
          local l1n1pos = edges[edgeId][3].inPos
          local l1n2pos = edges[edgeId][3].outPos
          local xnorm = n.pos:xnormOnLine(l1n1pos, l1n2pos)
          if xnorm > 0 and xnorm < 1 then
            local linePoint = linePointFromXnorm(l1n1pos, l1n2pos, xnorm)
            local l1n1rad = edges[edgeId][3].inRadius
            local l1n2rad = edges[edgeId][3].outRadius
            local linePrad = lerp(l1n1rad, l1n2rad, xnorm)
            local d2 = n.pos:squaredDistance(linePoint)
            if d2 < min(square(min(linePrad, n.radius)), d2Min) then -- TODO: do i need to keep the minimum? Why not keep all that safisfy the radius condition?
              d2Min, nodeMin, eIdMin, xMin = d2, nid, edgeId, xnorm
            end
          end
        end
      end
      if nodeMin then
        nodesToMerge[nodeMin] = {eIdMin, xMin}
      end
    end
  end

  local splits = {}
  for nid, data in pairs(nodesToMerge) do
    local edgeId = data[1]
    if splits[edgeId] then
      table.insert(splits[edgeId], {data[2], nid}) -- TODO: i'm only keeping nid here for debugging
    else
      splits[edgeId] = {{data[2], nid}}
    end
  end

  local positions, radii = {}, {} -- TODO: I don't need two tables here
  --local newNodesCreated = {} -- TODO: remove debug
  --local numOfNewNodesCreated = 0 -- TODO: remove debug
  for edgeId, splitData in _pairs(splits) do
    local n1 = edges[edgeId][1] -- inNode
    local n2 = edges[edgeId][2] -- outNode
    table.sort(splitData, function(a, b) return a[1] < b[1] end)

    for j = 1, #splitData do
      positions[j] = linePointFromXnorm(map.nodes[n1].links[n2].inPos, map.nodes[n1].links[n2].outPos, splitData[j][1])
      radii[j] = lerp(map.nodes[n1].links[n2].inRadius, map.nodes[n1].links[n2].outRadius, splitData[j][1])
    end

    -- This loop splits the n1-n2 edge at the disignated xnorms it does nothing else, i.e. it does not process nid at all.
    -- The merging of nid with the newly produced node along n1-n2 is done by the following mergeOverlappingNodes.
    -- This creates a dependence of mergeNodesToLines with mergeOverlappingNodes. Not Good!!
    for j = 1, #splitData do
      local nodeName = nameNode(n1, j)
      --newNodesCreated[nodeName] = splitData[j][2] -- TODO: remove debug
      --numOfNewNodesCreated = numOfNewNodesCreated + 1 -- TODO: remove debug
      local pos = positions[j]
      local radius = radii[j]

      map.nodes[nodeName] = {pos = pos, radius = radius, links = {}}
      map.nodes[nodeName].junction = true

      -- clear link between n1 and n2
      local data = map.nodes[n1].links[n2]
      map.nodes[n1].links[n2] = nil
      map.nodes[n2].links[n1] = nil

      -- connect n1 to nodeName
      map.nodes[n1].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = n1,
        outNode = nodeName,
        inPos = data.inPos,
        outPos = positions[j],
        inRadius = data.inRadius,
        outRadius = radii[j],
        noMerge = data.noMerge
      }

      -- connect nodeName to n1
      map.nodes[nodeName].links[n1] = map.nodes[n1].links[nodeName]

      -- connect n2 to nodeName
      map.nodes[n2].links[nodeName] = {
        drivability = data.drivability,
        hiddenInNavi = data.hiddenInNavi,
        oneWay = data.oneWay,
        lanes = data.lanes,
        speedLimit = data.speedLimit,
        type = data.type,
        inNode = nodeName,
        outNode = n2,
        inPos = positions[j],
        outPos = data.outPos,
        inRadius = radii[j],
        outRadius = data.outRadius,
        noMerge = data.noMerge
      }

      -- connect nodeName to n2
      map.nodes[nodeName].links[n2] = map.nodes[n2].links[nodeName]

      n1 = nodeName
    end

    table.clear(positions)
    table.clear(radii)
  end

  -- for n1id, n2id in pairs(newNodesCreated) do -- TODO: remove debug
  --   local n1 = map.nodes[n1id]
  --   local n2 = map.nodes[n2id]
  --   if not (n1.pos:squaredDistance(n2.pos) < square(min(n1.radius, n2.radius))) then
  --     print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! mergeNodesToLines Error 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
  --   end
  -- end

  -- if tableSize(newNodesCreated) ~= numOfNewNodesCreated then -- TODO: remove debug
  --   print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! mergeNodesToLines Error 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
  -- end

  _updateProgress()
end

local function processPrivateRoads()
  -- check for private roads joining public roads
  for nid, n in pairs(map.nodes) do
    local privateCount, count = 0, 0
    for lnid, d in pairs(n.links) do -- first, get any private links that exist from the current node
      if d.type == 'private' then
        privateCount = privateCount + 1
      end
      count = count + 1
    end

    if count >= 2 then
      local otherCount = count - privateCount
      --if privateCount == 1 and otherCount == 1 then
        --log('W', 'map', "Node "..tostring(nid).." has single private link connected to single public link; it is recommended to make the private road join to an intersection.")
      --elseif otherCount == 1 and privateCount == count - 1 then
        --log('W', 'map', "Node "..tostring(nid).." has single public link connected to only private links; it is recommended to make this public road private as well.")
      --end

      -- this section sets single segments, to be processed later by graphpath
      if otherCount == 1 and privateCount == count - 1 then
        for lnid, d in pairs(n.links) do
          if d.type ~= 'private' then -- processes non-private segment due to it being the only one compared to the other links
            n.links[lnid].gatedRoad = true
          end
        end
      elseif privateCount >= 1 and otherCount >= 1 then
        for lnid, d in pairs(n.links) do
          if d.type == 'private' then
            n.links[lnid].gatedRoad = true
          end
        end
      end
    end
  end

  _updateProgress()
end

local function isJunction(nId)
  local nodeDegree = tableSize(map.nodes[nId].links)
  if nodeDegree ~= 2 then
    return true
  else
    local n1Id, d1 = next(map.nodes[nId].links)
    local _, d2 = next(map.nodes[nId].links, n1Id)
    return not (is2SegMergeValid(nId, d1, d2) and (d1.type == d2.type) and d1.drivability == d2.drivability and d1.speedLimit == d2.speedLimit)
  end
end

local function optimizeNodes()
  -- identify "junction" nodes
  local isJunctionNode = {}
  for nid in _pairs(map.nodes) do
    isJunctionNode[nid] = isJunction(nid)
  end

  -- Simplify paths between any two junction nodes
  local visited, path, nodesToKeep, stack, pathLen = {}, {}, {}, {}, {}
  for nid in _pairs(map.nodes) do
    if not (visited[nid] or isJunctionNode[nid]) then

      ------------------------------------------------------------------------------
      -- Unfold the path (set of nodes between two "junction" nodes) containing nid
      ------------------------------------------------------------------------------

      local pathCount = 1
      path[1] = nid
      visited[nid] = true

      -- explore the path nid belongs to
      while true do
        -- get a neighbour of nid that is not already in the path
        local nextNid = next(map.nodes[path[pathCount]].links, path[pathCount-1]) or next(map.nodes[path[pathCount]].links)

        if nextNid == nid then -- search has come full circle without encountering a junction node. Path is an isolated loop.
          local iMin = 1
          for i = 2, pathCount do -- deterministically decide which node will be considered the first/last node of the path
            iMin = min3D(map.nodes[path[iMin]].pos, map.nodes[path[i]].pos) == map.nodes[path[iMin]].pos and iMin or i
          end

          local n = iMin
          for i = 1, n * 0.5 do -- reverse array from 1 to iMin. this will make iMin the first node in the path
            path[i], path[n] = path[n], path[i]
            n = n - 1
          end

          n = pathCount
          for i = iMin + 1, (n + iMin) * 0.5 do -- reverse array from iMin+1 to n
            path[i], path[n] = path[n], path[i]
            n = n - 1
          end

          pathCount = pathCount + 1
          path[pathCount] = path[1] -- make first node in the path also the last node in the path (a path has to have two end nodes)

          break
        end

        pathCount = pathCount + 1
        path[pathCount] = nextNid
        visited[nextNid] = true

        -- check if path has reached an end/junction
        if isJunctionNode[nextNid] then
          -- check if the first node in the path is also an end/junction
          if isJunctionNode[path[1]] then
            -- both end nodes of the path have been discovered. Terminate the search
            break
          else
            -- reverse the path up to this point to place junction node at the path start
            local n = pathCount
            for i = 1, n * 0.5 do
              path[i], path[n] = path[n], path[i]
              n = n - 1
            end
          end
        end
      end

      -- We want to always traverse the path reduction algorithm in the same order
      -- so make sure that the minimum position node (in the min3D sense) is at the start of the path
      -- and reverse path oreder if necessary
      local first, last
      if path[1] ~= path[pathCount] then -- path is not a loop
        first = path[1]
        last = path[pathCount]
      else -- path is a loop (first and last nodes are the same) so pick the second and one before last nodes to decide on the path order
        first = path[2]
        last = path[pathCount-1]
      end
      if min3D(map.nodes[last].pos, map.nodes[first].pos) == map.nodes[last].pos then
        local n = pathCount
        for i = 1, n * 0.5 do
          path[i], path[n] = path[n], path[i]
          n = n - 1
        end
      end

      -------------------------
      -- Simplify path polyline
      -------------------------

      -- Consider positions
      -- Decides which nodes need to be part of the path based on their positions
      local i, k = 1, pathCount
      local pi = map.nodes[path[i]].pos
      repeat
        local maxd2, maxj = 0, nil
        local pk = map.nodes[path[k]].pos
        for j = i+1, k-1 do
          local d2 = map.nodes[path[j]].pos:squaredDistanceToLineSegment(pi, pk)
          if d2 > maxd2 then -- TODO: does the overall result change if i have two nodes at the same distance?
            maxd2 = d2
            maxj = j
          end
        end

        if maxj and maxd2 > square(map.nodes[path[maxj]].radius * 0.05) then
          nodesToKeep[maxj] = true
          table.insert(stack, k)
          k = maxj
        else
          i = k
          pi = pk
          k = table.remove(stack)
        end
      until not k

      -- Consider radii
      -- Decides which nodes need to be part of the path based on their radius
      pathLen[1] = 0
      for i = 2, pathCount do
        pathLen[i] = pathLen[i-1] + map.nodes[path[i]].pos:distance(map.nodes[path[i-1]].pos)
      end

      local i, k = 1, pathCount
      repeat
        local maxDiff, maxj = 0, nil
        for j = i+1, k-1 do
          local xnorm = (pathLen[j] - pathLen[i]) / (pathLen[k] - pathLen[i])
          local rDiff = abs(map.nodes[path[j]].radius - lerp(map.nodes[path[i]].radius, map.nodes[path[k]].radius, xnorm))
          if rDiff > maxDiff then
            maxDiff = rDiff
            maxj = j
          end
        end

        if maxj and maxDiff > 0.1 * map.nodes[path[maxj]].radius then
          nodesToKeep[maxj] = true
          table.insert(stack, k)
          k = maxj
        else
          i = k
          k = table.remove(stack)
        end
      until not k

      -- Remove nodes that don't need to be part of the path.
      local prevNode
      for i = 2, pathCount-1 do
        if not nodesToKeep[i] then
          local n1id = prevNode or path[i-1]
          local nid = path[i]
          local n2id = path[i+1]
          local n1 = map.nodes[n1id]
          local n2 = map.nodes[n2id]
          local d1 = n1.links[nid]
          local d2 = n2.links[nid]

          -- n1.links[n1id] = nil -- TODO: this makes no sense
          -- n2.links[n2id] = nil

          n1.links[nid] = nil
          n2.links[nid] = nil
          map.nodes[nid] = nil

          if n1.links[n2id] then
            -- local dchord = n1.links[n2id]
            -- if is3SegMergeValid(nid, d1, d2, dchord) and (d1.type == d2.type and d2.type == dchord.type) then
            --   n1.links[nid] = nil
            --   n2.links[nid] = nil
            --   map.nodes[nid] = nil
            --   prevNode = n1id
            -- else
            --   prevNode = nil
            -- end

            local e1 = n1.links[n2id]
            local inNode = d1.inNode == nid and n2id or n1id
            local outNode = d1.outNode == nid and n2id or n1id
            local e2 = {
              drivability = min(d1.drivability, d2.drivability),
              speedLimit = min(d1.speedLimit, d2.speedLimit),
              oneWay = d1.oneWay,
              inNode = inNode,
              outNode = outNode,
              inPos = inNode == n1id and d1[d1.inNode == n1id and 'inPos' or 'outPos'] or d2[d2.inNode == n2id and 'inPos' or 'outPos'],
              outPos = outNode == n1id and d1[d1.outNode == n1id and 'outPos' or 'inPos'] or d2[d2.outNode == n2id and 'outPos' or 'inPos'],
              inRadius = inNode == n1id and d1[d1.inNode == n1id and 'inRadius' or 'outRadius'] or d2[d2.outNode == n2id and 'inRadius' or 'outRadius'],
              outRadius = outNode == n1id and d1[d1.outNode == n1id and 'outRadius' or 'inRadius'] or d2[d2.outNode == n2id and 'outRadius' or 'inRadius'],
              type = d1.type,
            }

            prevNode = n1id

            if edgeCompare(e2, e1) then
              n1.links[n2id] = e2
              n2.links[n1id] = e2
            end
          else
            d1.drivability = min(d1.drivability, d2.drivability)
            d1.speedLimit = min(d1.speedLimit, d2.speedLimit)
            d1.inNode = d1.inNode == nid and n2id or n1id
            d1.outNode = d1.outNode == nid and n2id or n1id

            if not (d1.hiddenInNavi and d2.hiddenInNavi) then
              d1.hiddenInNavi = nil
            end

            -- n1.links[nid] = nil
            -- n2.links[nid] = nil
            -- n1.links[n1id] = nil -- i possibly ment map.nodes[nid].links[n1id]
            -- n2.links[n2id] = nil
            -- map.nodes[nid] = nil

            d1.inPos = d1.inNode == n1id and d1[d1.inNode == n1id and 'inPos' or 'outPos'] or d2[d2.inNode == n2id and 'inPos' or 'outPos']
            d1.outPos = d1.outNode == n1id and d1[d1.outNode == n1id and 'outPos' or 'inPos'] or d2[d2.outNode == n2id and 'outPos' or 'inPos']
            d1.inRadius = d1.inNode == n1id and d1[d1.inNode == n1id and 'inRadius' or 'outRadius'] or d2[d2.inNode == n2id and 'inRadius' or 'outRadius']
            d1.outRadius = d1.outNode == n1id and d1[d1.outNode == n1id and 'outRadius' or 'inRadius'] or d2[d2.outNode == n2id and 'outRadius' or 'inRadius']
            d1.noMerge = nil
            n1.links[n2id] = d1
            n2.links[n1id] = d1

            prevNode = n1id
          end
        else
          prevNode = nil
        end
      end

      tableClear(path)
      tableClear(pathLen)
      tableClear(nodesToKeep)
      tableClear(stack)
    end
  end
end

local function validateMapData(singleSided)
  local noOfNodes = 0
  local noOfValidEdges = 0
  local noOfInvalidEdges = 0
  local nonManualIsolatedNodes = 0
  local numOfEdges = 0
  for n1id, n1 in pairs(map.nodes) do
    noOfNodes = noOfNodes + 1
    local degree = 0
    for n2id, data in pairs(n1.links) do
      if n1id ~= n2id then
        degree = degree + 1
        if singleSided then
          if map.nodes[n2id] then
            if map.nodes[n2id].links[n1id] == nil then
              noOfValidEdges = noOfValidEdges + 1
            else -- if singleSided is true and the edge is double sided consider it invalid
              noOfInvalidEdges = noOfInvalidEdges + 1
            end
          else -- linked node does not exist
            noOfInvalidEdges = noOfInvalidEdges + 1
          end
        else -- graph is double sided
          -- check if node n2id exists and if both sides refer to the same data
          if map.nodes[n2id] and map.nodes[n2id].links[n1id] == data then
            noOfValidEdges = noOfValidEdges + 1 -- edges will be counted twice
          else
            noOfInvalidEdges = noOfInvalidEdges + 1
          end
        end
      else
        noOfInvalidEdges = noOfInvalidEdges + 1
      end
    end
    if not singleSided then
      if degree == 0 and not n1.manual then
        nonManualIsolatedNodes = nonManualIsolatedNodes + 1
      end
    end
    numOfEdges = numOfEdges + degree
  end

  local case
  if singleSided then
    case = 'Single Sided: '
  else
    case = 'Double Sided: '
  end
  if noOfValidEdges > 0 then
    log('W', 'map', case.."There are "..tonumber(noOfValidEdges).." valid edges")
  end
  if noOfInvalidEdges > 0 then
    log('W', 'map', case.."There are "..tonumber(noOfInvalidEdges).." invalid edges")
  end
  if nonManualIsolatedNodes > 0 then
    log('W', 'map', case.."There are "..tonumber(nonManualIsolatedNodes).." non manual isolated nodes")
  end
end

local function generateVisLog()
  visualLog = {}
  for nid, n in pairs(map.nodes) do
    local linksize = tableSize(n.links)
    if linksize == 1 then
      visLog("warn", n.pos, "dead end:"..tostring(nid))
    elseif linksize == 0 then
      visLog("error", n.pos, "isolated node:"..tostring(nid))
    end
  end

  _updateProgress()
end

local function convertToSingleSided()
  local edgeCount, nodeCount = 0, 0
  for nid, n in _pairs(map.nodes) do
    nodeCount = nodeCount + 1
    local newLinks = {}
    for lid, data in _pairs(n.links) do
      if data.inNode == lid then -- lid > nid
        edgeCount = edgeCount + 1
        newLinks[lid] = data
      end
    end
    n.links = newLinks
    n.manual = nil
  end

  _updateProgress()

  return edgeCount, nodeCount
end

local function getNodeLinkCount(nId)
  if not gp or not gp.graph[nId] then return -1 end
  return tableSize(gp.graph[nId])
end

local function linkMap()
  local graphDataHistory = {}

  loadJsonDecalMap()
  --[[ Load Map Data Hashes
  print('--------------- Load ---------------')
  checkLinks()
  graphDataHistory['load'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['load'][1])
  dump(graphDataHistory['load'][2])
  print('')
  --]]
  createSpeedLimits(true)

  mergeOverlappingNodes(true)
  --[[ Merge Overlapping Nodes 0 Hashes
  print('--------------- Merge Overlapping Nodes 0 ---------------')
  checkLinks()
  graphDataHistory['Merge_Nodes_0'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['Merge_Nodes_0'][1])
  dump(graphDataHistory['Merge_Nodes_0'][2])
  print('')
  --]]

  optimizeNodes()
  --[[ optimizeNodes_0
  print('--------------- Optimize Nodes 0 ---------------')
  checkLinks()
  graphDataHistory['optimizeNodes_0'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['optimizeNodes_0'][1])
  dump(graphDataHistory['optimizeNodes_0'][2])
  print('')
  --]]

  mergeOverlappingNodes() -- this opperation could delete edges (if there is an edge between two nodes in the same overlap group or two nodes in the same group link to a third node in another group)
  --[[ Merge Overlapping Nodes 1 Hashes
  print('--------------- Merge Overlapping Nodes 1 ---------------')
  checkLinks()
  graphDataHistory['Merge_Nodes_1'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['Merge_Nodes_1'][1])
  dump(graphDataHistory['Merge_Nodes_1'][2])
  print('')
  --]]


  resolveTJunctions()
  --[[ T - Junctions Hashes
  print('--------------- Resolve T Junctions ---------------')
  checkLinks()
  graphDataHistory['T_Junctions'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['T_Junctions'][1])
  dump(graphDataHistory['T_Junctions'][2])
  print('')
  --]]


  resolveXJunctions()
  --[[ X - Junctions Hashes
  print('--------------- Resolve X Junctions ---------------')
  checkLinks()
  graphDataHistory['X_Junctions'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['X_Junctions'][1])
  dump(graphDataHistory['X_Junctions'][2])
  print('')
  --]]


  mergeNodesToLines()
  --[[ Merge Nodes to Lines Hashes
  print('--------------- After merge nodes to lines ---------------')
  checkLinks()
  graphDataHistory['mergeNodesToLines'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['mergeNodesToLines'][1])
  dump(graphDataHistory['mergeNodesToLines'][2])
  print('')
  --]]


  mergeOverlappingNodes() -- this opperation could delete edges (if there is an edge between two nodes in the same overlap group or two nodes in the same group link to a third node in another group)
  --[[ Merge Overlapping Nodes 2 Hashes
  print('--------------- Merge Overlapping Nodes 2 ---------------')
  checkLinks()
  graphDataHistory['Merge_Nodes_2'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['Merge_Nodes_2'][1])
  dump(graphDataHistory['Merge_Nodes_2'][2])
  print('')
  --]]

  processPrivateRoads()

  optimizeNodes() -- optimize nodes does not preserve the noMerge property of edges
  --[[ Optimize Nodes Hashes
  print('------------------ Optimize Nodes ----------------------')
  checkLinks()
  graphDataHistory['optimizeNodes'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['optimizeNodes'][1])
  dump(graphDataHistory['optimizeNodes'][2])
  print('')
  --]]

  convertToSingleSided()
  --[[ Convert to single sided Hashes
  print('------------------ Convert To Single Sided ----------------------')
  -- checkLinks() -- this does not work for single sided links
  graphDataHistory['convertToSingleSided'] = {hashNodeData(), hashEdgeData()}
  dump(graphDataHistory['convertToSingleSided'][1])
  dump(graphDataHistory['convertToSingleSided'][2])
  print('')
  --]]

  return graphDataHistory
end

-- local function colorNodes(mapNodes)
--   local maxColorCode = 0
--   for nodeId, node in pairs(mapNodes) do
--     if node.junction then
--       local colorCode = -1
--       repeat
--         colorCode = colorCode + 1
--         local validColorFound = true
--         for k in nodeKdTree:queryNotNested(pointBBox(node.pos.x, node.pos.y, node.radius * 4)) do -- check all nodes with this colorCode
--           if mapNodes[k].debugColorCode == colorCode then -- check distance between nodes with the same colorCode -- and nodes[k].pos:squaredDistance(nodes[nodeId].pos) < 400
--             validColorFound = false
--             break
--           end
--         end
--       until validColorFound
--       node.debugColorCode = colorCode
--       maxColorCode = max(maxColorCode, colorCode)
--     end
--   end

--   --[[ Debug color distribution: number of times each color is used
--   local dist = {}
--   for k, v in pairs(mapNodes) do
--     if v.debugColorCode then
--       dist[v.debugColorCode+1] = (dist[v.debugColorCode+1] or 0) + 1
--     end
--   end
--   dump(dist)
--   --]]

--   for nodeId, node in pairs(mapNodes) do
--     if node.debugColorCode then node.debugColorCode = node.debugColorCode / maxColorCode end
--   end
--   -- dump('nodeColors', nodeColors)
--   -- dump('colorCodes', colorCodes)
--   dump('maxColorCode', maxColorCode)
-- end

-- colors junction nodes so that nodes that are close by get different colors: for visual debugging
local function colorNodes(mapNodes)
  local numOfColorCodes = 8 -- number of colors to be used. Tentative, will grow as needed at the expense of skewing the color distribution.
  local colorCode = -1 -- initialize color code (-- TODO: can we get something from random initialization?)
  for nodeId, node in pairs(mapNodes) do
    if node.junction then
      local numOfColorsRejected = 0
      repeat
        if numOfColorsRejected == numOfColorCodes then -- all available colors have been rejected
          colorCode = numOfColorCodes -- set color code to the newly available color code
          numOfColorCodes = numOfColorCodes + 1
          break -- we can be sure this color is unused, so break from the loop
        else
          colorCode = (colorCode+1) % numOfColorCodes -- colorCodes will run from 0 to numOfColorCodes-1
        end
        local validColorFound = true
        for k in nodeKdTree:queryNotNested(pointBBox(node.pos.x, node.pos.y, node.radius * 4)) do -- check nodes around nodeId
          if mapNodes[k].debugColorCode == colorCode then
            -- TODO when i reject a color code i should try to get a color that is not adjacent to the one rejected
            -- it might be possible to do this with an integer variant of getBlueNoise1d
            numOfColorsRejected = numOfColorsRejected + 1
            validColorFound = false
            break
          end
        end
      until validColorFound
      node.debugColorCode = colorCode
    end
  end

  --[[ Debug color distribution: number of times each color is used (a uniform distribution is desirable)
  local dist = {}
  for k, v in pairs(mapNodes) do
    if v.debugColorCode then
      dist[v.debugColorCode+1] = (dist[v.debugColorCode+1] or 0) + 1
    end
  end
  dump(dist)
  --]]
  -- dump('numOfColorCodes', numOfColorCodes)

  numOfColorCodes = 1 / (numOfColorCodes-1)
  for nodeId, node in pairs(mapNodes) do
    if node.debugColorCode then node.debugColorCode = node.debugColorCode * numOfColorCodes end
  end
end

local function loadMap(customMapNodes)
  if not be then return end
  --log('A', "map.loadMap-calledby", debug.traceback())
  --local timer = hptimer()
  profilerPushEvent('aiMap')
  M.objects = {}
  M.objectNames = {}

  -- preserve map references
  local nodes = map.nodes
  tableClear(nodes)
  tableClear(map)
  map.nodes = nodes

  setRoadRules()

  local edgeCount, nodeCount
  if customMapNodes then
    map.nodes = customMapNodes
    edgeCount, nodeCount = convertToSingleSided()
  else
    linkMap()

    --[[ For debugging: comment out linkMap() call above.
    print('============== Load Map ===============')
    toggleShuffledPairs('shuffledPairs')

    local graphHistory = {}

    graphHistory[1] = linkMap()

    print('')
    print('=========================================================================')
    print('')

    local nodes = map.nodes
    tableClear(nodes)
    tableClear(map)
    map.nodes = nodes

    graphHistory[2] = linkMap()
    print('')

    local checkTabNames = {'load', 'Merge_Nodes_1', 'T_Junctions', 'X_Junctions', 'mergeNodesToLines', 'Merge_Nodes_2', 'optimizeNodes', 'convertToSingleSided'}
    for i, v in ipairs(checkTabNames) do
      if graphHistory[1][v] and graphHistory[2][v] then
        print('========='.. ' '..v..' '..'=========')

        if graphHistory[1][v][1] == graphHistory[2][v][1] then
          print('Node Check PASSED')
        else
          print('Node check FAILED !!!!!!!!!!!!!!!!!!!!!!!')
        end

        if graphHistory[1][v][2] == graphHistory[2][v][2] then
          print('Edge Check PASSED')
        else
          print('Edge check FAILED !!!!!!!!!!!!!!!!!!!!!!!')
        end

        print('')
      end
    end
    --]]
  end

  --generateVisLog()

  local mapNodes = map.nodes

  local nodeDrivabilities = {}
  for k, n in pairs(mapNodes) do
    nodeDrivabilities[k] = be:getTerrainDrivability(n.pos, n.radius)
    if nodeDrivabilities[k] <= 0 then nodeDrivabilities[k] = 1 end -- guard against zero values in getTerrainDrivability (case in point: void ground model used in tunnel entrances)
  end

  -- build the graph and the tree
  maxRadius = 4 -- case there are no nodes in the map i.e. next(map.nodes) == nil avoids infinite loop in findClosestRoad()
  gp = graphpath.newGraphpath()
  edgeKdTree = kdTreeBox2D.new(edgeCount)
  nodeKdTree = kdTreeBox2D.new(nodeCount)
  local edgeTab = {'','\0',''}

  for nid, n in pairs(mapNodes) do -- edges are now single sided
    local nPos = n.pos
    local radius = n.radius
    gp:setPointPositionRadius(nid, nPos, radius)
    nodeKdTree:preLoad(nid, pointBBox(nPos.x, nPos.y, radius))
    maxRadius = max(maxRadius, radius)
    local nidDrivability = nodeDrivabilities[nid]
    edgeTab[1] = nid
    for lid, data in pairs(n.links) do
      local edgeDrivability = min(1, max(1e-30, (nodeDrivabilities[lid] + nidDrivability) * 0.5 * data.drivability))
      local distanceConst = data.gatedRoad and 10000 or 0
      local inNodeId = data.inNode
      local outNodeId = data.outNode
      gp:bidiEdge(
        inNodeId,
        outNodeId,
        nPos:distance(mapNodes[lid].pos) / edgeDrivability + distanceConst,
        data.drivability,
        data.speedLimit,
        data.lanes,
        data.oneWay,
        distanceConst,
        mapNodes[inNodeId].pos ~= data.inPos and data.inPos or nil,
        mapNodes[inNodeId].radius ~= data.inRadius and data.inRadius or nil,
        mapNodes[outNodeId].pos ~= data.outPos and data.outPos or nil,
        mapNodes[outNodeId].radius ~= data.ouRadius and data.outRadius or nil
      )
      edgeTab[3] = lid
      edgeKdTree:preLoad(table.concat(edgeTab), quadtree.lineBBox(data.inPos.x, data.inPos.y, data.outPos.x, data.outPos.y, min(data.inRadius, data.outRadius)))
    end

    n.normal = surfaceNormal(nPos, radius * 0.5)
  end

  maxRadius = min(15, maxRadius)

  edgeKdTree:build()

  _updateProgress()

  nodeKdTree:build()

  _updateProgress()

  -- Find closest mapNode to a manualWaypoint not in the map and create Alias
  local nodeAliases = {}
  if not customMapNodes then
    for nodeName, v in pairs(manualWaypoints) do
      if mapNodes[nodeName] == nil or gp.graph[nodeName] == nil then
        local closestNode
        local minDist = huge
        local vPos = v.pos
        for item_id in nodeKdTree:queryNotNested(pointBBox(vPos.x, vPos.y, v.radius)) do
          if item_id ~= nodeName then -- what if the closest node is also an orphan?
            local dist = mapNodes[item_id].pos:squaredDistance(vPos)
            if dist < minDist then
              closestNode = item_id
              minDist = dist
            end
          end
        end
        nodeAliases[nodeName] = closestNode
      end
      manualWaypoints[nodeName] = 1
    end
  end
  map.nodeAliases = nodeAliases

  _updateProgress()

  buildSerial = (buildSerial or -1) + 1
  map.buildSerial = buildSerial

  --log('D', 'map', "generating roads took " .. string.format("%2.3f ms", timer:stopAndReset()))
  be:sendToMailbox("mapData", lpack.encodeBinWorkBuffer(
    {
      nodeAliases = map.nodeAliases,
      maxRadius = maxRadius,
      graphData = gp:export(edgeCount),
      edgeKdTree = edgeKdTree:export(),
      rules = rules
    }
  ))

  be:sendToMailbox("updateDrivabilities", lpack.encodeBinWorkBuffer(nil)) -- clear updateDrivabilities mailbox

  guihooks.trigger("NavigationMapChanged", map)
  profilerPopEvent('aiMap')

  _updateProgress()
  extensions.hook("onNavgraphReloaded")
end

-- this is also in vehicle/mapmgr.lua
-- find road (edge) closest to "position" and return the nodes ids (closestRoadNode1, closestRoadNode2) of that edge and distance to it.
local function findClosestRoad(pos, searchRadiusLim)
  -- searchRadiusLim: Optional

  if edgeKdTree == nil then return end
  searchRadiusLim = searchRadiusLim or 200
  local searchRadius = min(maxRadius, searchRadiusLim)

  local mapNodes = map.nodes
  local closestRoadNode1, closestRoadNode2, closestDist
  repeat
    closestDist = searchRadius * searchRadius
    for item_id in edgeKdTree:queryNotNested(pointBBox(pos.x, pos.y, searchRadius)) do
      local i = stringFind(item_id, '\0')
      local n1id = stringSub(item_id, 1, i-1)
      local n2id = stringSub(item_id, i+1, #item_id)
      local curDist = pos:squaredDistanceToLineSegment(mapNodes[n1id].pos, mapNodes[n2id].pos)
      if curDist < closestDist then
        closestDist = curDist
        closestRoadNode1 = n1id
        closestRoadNode2 = n2id
      end
    end
    searchRadius = searchRadius * 2
  until closestRoadNode1 or searchRadius > searchRadiusLim

  return closestRoadNode1, closestRoadNode2, sqrt(closestDist)
end

local function findBestRoad(pos, dir)
  -- searches for best road with respect to position and direction, with a fallback to the generic findClosestRoad function
  local mapNodes = map.nodes
  local bestRoad1, bestRoad2, bestDist
  local currRoads = {}

  for item_id in edgeKdTree:queryNotNested(pointBBox(pos.x, pos.y, 20)) do -- assuming that no roads would have a radius greater than 20 m
    local i = stringFind(item_id, '\0')
    local n1id = stringSub(item_id, 1, i-1)
    local n2id = stringSub(item_id, i+1, #item_id)
    local curDist = pos:squaredDistanceToLineSegment(mapNodes[n1id].pos, mapNodes[n2id].pos)

    if curDist <= square(math.max(mapNodes[n1id].radius, mapNodes[n2id].radius)) then
      local xnorm = pos:xnormOnLine(mapNodes[n1id].pos, mapNodes[n2id].pos)
      if xnorm >= 0 and xnorm <= 1 then -- insert result if it is within road boundaries
        table.insert(currRoads, {n1id, n2id, curDist})
      end
    end
  end

  if not currRoads[1] then
    --log('W', 'map', 'no results for findBestRoad, now using findClosestRoad')
    return findClosestRoad(pos, wZ) -- fallback
  elseif not currRoads[2] then -- only one entry in the table
    return currRoads[1][1], currRoads[1][2], math.sqrt(currRoads[1][3])
    -- need to return inNode to outNode
  end

  local bestDot = 0
  for _, v in ipairs(currRoads) do
    local dirDot = math.abs(dir:dot((mapNodes[v[1]].pos - mapNodes[v[2]].pos):normalized()))
    if dirDot >= bestDot then -- best direction
      bestDot = dirDot
      bestRoad1, bestRoad2, bestDist = v[1], v[2], v[3]
    end
  end

  return bestRoad1, bestRoad2, math.sqrt(bestDist)
end

local function getPath(start, target, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff)
  -- arguments:
  -- start: starting node
  -- target: target node
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: amount of penalty to impose to path if it does not respect road
  --          legal directions (should be larger than 1). If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  if gp == nil then return {} end
  return gp:getFilteredPath(start, target, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff)
end

local function getPointNodePath(start, target, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
  -- Shortest path between a point and a node or vice versa.
  -- start/target: either start or target should be a node name, the other a vec3 point
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: amount of penalty for traversing edges in the 'illegal direction' (reasonable penalty values: 1e3-1e4). 1 = no penalty
  --          If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  -- wZ: number. When higher than 1 distance minimization is biased to minimizing z diamension more so than x, y.

  if gp == nil then return {} end
  return gp:getPointNodePath(start, target, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
end

local function getNodesFromPathDist(path, dist)
  -- finds and returns nodes and an xnorm based on distance along path
  local mapNodes = map.nodes
  if not mapNodes or not path or not path[2] then return end
  local pathCount = #path
  dist = dist or huge

  for i = 1, pathCount - 1 do
    local n1, n2 = path[i], path[i + 1]
    if mapNodes[n1] and mapNodes[n2] then
      local length = mapNodes[n1].pos:distance(mapNodes[n2].pos)

      if dist > length then
        dist = dist - length
      else
        return n1, n2, clamp(dist / (length + 1e-30), 0, 1)
      end
    end
  end

  return path[pathCount - 1], path[pathCount], 1
end

local function getPathLen(path)
  -- returns the path length
  local mapNodes = map.nodes
  if not (mapNodes and path) then return 0 end
  local pathLen = 0
  for i = 2, #path do pathLen = pathLen + mapNodes[path[i]].pos:distance(mapNodes[path[i-1]].pos) end
  return pathLen
end

local function getPathPositions(path)
  local pathCount = #path
  local tab = table.new(pathCount, 0)
  tab[1] = gp:getEdgePositions(path[1], path[2])
  tab[1] = tab[1]:copy()
  for i = 2, pathCount-1 do
    local _, wp2Pos = gp:getEdgePositions(path[i-1], path[i])
    local wp3Pos, _ = gp:getEdgePositions(path[i], path[i+1])
    tab[i] = (wp2Pos + wp3Pos) * 0.5
  end
  tab[pathCount] = gp:getEdgePositions(path[pathCount], path[pathCount-1])
  tab[pathCount] = tab[pathCount]:copy()
  return tab
end

local function startPosLinks(position, wZ)
  --log('A','map', 'findClosestRoad called with '..position.x..','..position.y..','..position.z)
  wZ = wZ or 1 -- zbias
  local nodePositions = gp.positions
  local nodeRadius = gp.radius
  local costs = table.new(0, 32)
  local xnorms = table.new(0, 32)
  local seenEdges = table.new(0, 32)
  local j, names = 0, table.new(32, 0)
  local searchRadius = maxRadius * 5
  local tmpVec = vec3()
  local edgeVec = vec3()

  local sortComparator = function(n1, n2) return costs[n1] > costs[n2] end

  return function ()
    repeat
      if j > 0 then
        local name = names[j]
        names[j] = nil
        j = j - 1
        return name, costs[name], xnorms[name]
      else
        for item_id in edgeKdTree:queryNotNested(pointBBox(position.x, position.y, searchRadius)) do
          if not seenEdges[item_id] then
            seenEdges[item_id] = true
            local i = stringFind(item_id, '\0')
            local n1id = stringSub(item_id, 1, i-1)
            local n2id = stringSub(item_id, i+1, #item_id)
            local n1Pos = nodePositions[n1id]
            edgeVec:setSub2(nodePositions[n2id], n1Pos)
            tmpVec:setSub2(position, n1Pos) -- node1ToPosVec
            local xnorm = min(1, max(0, edgeVec:dot(tmpVec) / (edgeVec:squaredLength() + 1e-30)))
            local key
            if xnorm == 0 then
              key = n1id
            elseif xnorm == 1 then
              key = n2id
            else
              key = {n1id, n2id}
              xnorms[key] = xnorm -- we only need to store the xnorm if 0 < xnorm < 1
            end
            if not costs[key] then
              edgeVec:setScaled(xnorm)
              tmpVec:setSub(edgeVec) -- distVec
              tmpVec:setScaled(max(0, 1 - max(nodeRadius[n1id], nodeRadius[n2id]) / (tmpVec:length() + 1e-30)))
              costs[key] = square(square(tmpVec.x) + square(tmpVec.y) + square(wZ * tmpVec.z))
              j = j + 1
              names[j] = key
            end
          end
        end

        table.sort(names, sortComparator)

        searchRadius = searchRadius * 2
      end
    until searchRadius > 2000

    return nil, nil, nil
  end
end

local function getPointToPointPath(startPos, targetPos, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wD, wZ)
  -- startPos: path source position
  -- targetPos: target position (vec3)
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: amount of penalty to impose to path if it does not respect road legal directions (should be larger than 1 typically >= 10e4).
  --          If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  -- wZ: number (typically >= 1). When higher than 1 destination node of optimum path will be biased towards minimizing height difference to targetPos.
  -- wD has been depricated (left here for backwards compatibility)

  if gp == nil then return {} end
  wZ = wZ or 4
  local iter = startPosLinks(startPos, wZ)
  return gp:getPointToPointPath(startPos, iter, targetPos, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
end

local function saveSVG(filename, includeLinks, includeNodes)
  local svg = require('libs/EzSVG/EzSVG')

  includeLinks = includeLinks ~= false -- Default to true if not provided
  includeNodes = includeNodes or false -- Default to false if not provided

  local terrain = scenetree.findObject(scenetree.findClassObjects('TerrainBlock')[1])
  local terrainPosition = vec3(terrain:getPosition())

  local m = map
  if not m or not next(m.nodes) then return end

  -- Calculate bounding box for all nodes
  local minX, minY, maxX, maxY = math.huge, math.huge, -math.huge, -math.huge
  for nid, n in pairs(m.nodes) do
    local pos = n.pos - terrainPosition
    minX = math.min(minX, pos.x)
    minY = math.min(minY, pos.y)
    maxX = math.max(maxX, pos.x)
    maxY = math.max(maxY, pos.y)
  end

  -- Add a margin for link widths
  local margin = 20
  minX = minX - margin
  minY = minY - margin
  maxX = maxX + margin
  maxY = maxY + margin

  -- Calculate SVG size
  local width = maxX - minX
  local height = maxY - minY

  local svgDoc = svg.Document(width, height, svg.gray(255))
  local lines = svg.Group()

  -- Draw links
  if includeLinks then
    for nid, n in pairs(m.nodes) do
      for lid, dif in pairs(n.links) do
        local p1 = n.pos - terrainPosition
        local p2 = m.nodes[lid].pos - terrainPosition

        -- TODO: add proper fading between some colors
        local typeColor = 'black'
        local d = dif
        if type(dif) == "table" then
          d = dif.drivability or 0
        end
        if d < 0.9 and d >= 0 then
          typeColor = svg.rgb(170, 68, 0) -- dirt road = brown
        end

        lines:add(svg.Polyline({p1.x - minX, maxY - p1.y, p2.x - minX, maxY - p2.y}, {
          fill = 'none',
          stroke = typeColor,
          stroke_width = n.radius * 2,
          stroke_opacity = 0.4,
        }))
      end
    end
    svgDoc:add(lines)
  end

  -- Draw nodes
  if includeNodes then
    local nodes = svg.Group()
    for nid, n in pairs(m.nodes) do
      local p = n.pos - terrainPosition
      nodes:add(svg.Circle(p.x - minX, maxY - p.y, n.radius, {
        fill = 'black',
        fill_opacity = 0.4,
        stroke = 'none',
      }))
    end
    svgDoc:add(nodes)
  end

  svgDoc:writeTo(filename or 'map.svg')
end

-- curvature formulation based on interpolating circle
local function inCurvature(vec1, vec2)
  --[[
    Given three points A, B, C (with AB being the vector from A to B), the curvature (= 1 / radius)
    of the circle going through them is:

    curvature = 2 * (AB x BC) / ( |AB| * |BC| * |CA| ) =>
              = 2 * |AB| * |BC| * Sin(th) / ( |AB| * |BC| * |CA| ) =>
              = 2 * (+/-) * sqrt ( 1 - Cos^2(th) ) / |CA| =>
              = 2 * (+/-) sqrt [ ( 1 - Cos^2(th) ) / |CA|^2 ) ] -- This is an sqrt optimization step

    In the calculation below the (+/-) which indicates the turning direction (direction of AB x BC) has been dropped
  --]]

  local vec1Sqlen, vec2Sqlen = vec1:squaredLength(), vec2:squaredLength()
  local dot12 = vec1:dot(vec2)
  local cos8sq = min(1, dot12 * dot12 / max(0, 1e-100 + vec1Sqlen * vec2Sqlen))

  if dot12 < 0 then -- angle between the two segments is acute
    local minDsq = min(vec1Sqlen, vec2Sqlen)
    local maxDsq = minDsq / max(1e-30, cos8sq)
    if max(vec1Sqlen, vec2Sqlen) > (minDsq + maxDsq) * 0.5 then
      if vec1Sqlen > vec2Sqlen then
        vec1, vec2 = vec2, vec1
        vec1Sqlen, vec2Sqlen = vec2Sqlen, vec1Sqlen
      end
      vec2:setScaled(sqrt(0.5 * (minDsq + maxDsq) / max(0, 1e-100 + vec2Sqlen)))
    end
  end

  vec2:setScaled(-1)
  return 2 * sqrt((1 - cos8sq) / max(0, 1e-100 + vec1:squaredDistance(vec2)))
end

-- an alternative discrete curvature formulation
local function dCurvature(v1, v2)
  local l1, l2 = v1:length(), v2:length()
  return 2 * math.acos(clamp(v1:dot(v2) / (l1 * l2), -1, 1)) / (l1 + l2)
end

local function createTrajectory(path, s)
  local pathCount = #path
  -- Read path data
  local trajectory = table.new(pathCount, 0)
  trajectory.converged = false

  for i = 1, pathCount do
    local p = gp.positions[path[i]]
    local r = gp.radius[path[i]]

    trajectory[i] = {
      posOrig = vec3(p),
      pos = vec3(p),
      normalOrig = vec3(),
      normal = vec3(),
      biNormal = surfaceNormal(p, r * 0.5),
      edgeVec = vec3(),
      radius = r,
      halfWidth = r,
      radiusMult = 1,
      curvature = 0,
      pathNode = true,
      fixed = s.fixPathNodes or ((i == 1 or i == pathCount) and s.fixEndpoints),
      evolveNormal = ((i == 1 or i == pathCount) and (s.evolveEndNormals and s.evolvePathNormals)) or ((i ~= 1 and i ~= pathCount) and s.evolvePathNormals),
      force = vec3(),
      length = nil,
      prevFDir = 0,
      lateralXnorm = 0,
    }
  end

  local trajectoryCount = pathCount
  if s.segDistSplitLim then
    -- Augment path data: adds nodes to the path to increase fidelity, such that the maximum distance between two nodes is less or equal to segDistSplitLim
    -- smaller distances between nodes is known to increase convergence time (number of iterations to converge)
    local dd = s.segDistSplitLim * s.segDistSplitLim
    repeat
      local noSplit = true -- will keep traversing the trajectory until no segment is split
      for i = 1, trajectoryCount-1 do
        if trajectory[i].pos:squaredDistance(trajectory[i+1].pos) > dd then
          local posOrig = (trajectory[i].posOrig + trajectory[i+1].posOrig) * 0.5
          local radius = (trajectory[i].radius + trajectory[i+1].radius) * 0.5
          local node = {
            posOrig = posOrig,
            pos = vec3(posOrig),
            normalOrig = vec3(),
            normal = vec3(),
            biNormal = surfaceNormal(posOrig, radius * 0.5),
            edgeVec = vec3(),
            radius = radius,
            halfWidth = radius,
            radiusMult = 1,
            curvature = 0,
            pathNode = false,
            fixed = false,
            evolveNormal = s.evolveMidNormals,
            force = vec3(),
            length = nil,
            prevFDir = 0,
            lateralXnorm = 0,
          }
          table.insert(trajectory, i+1, node)
          trajectoryCount = trajectoryCount + 1
          noSplit = false
          break
        end
      end
    until noSplit
  end

  -- Calculate Edge Vectors and lengths
  for i = 1, trajectoryCount-1 do
    trajectory[i].edgeVec:setSub2(trajectory[i+1].pos, trajectory[i].pos)
    trajectory[i].length = trajectory[i].edgeVec:length()
  end

  -- Calculate Initial path normal vectors and adjust node halfwidth
  trajectory[1].normalOrig:setCross(trajectory[1].biNormal, trajectory[1].edgeVec); trajectory[1].normalOrig:normalize()
  trajectory[1].normal:set(trajectory[1].normalOrig)
  local nVec1, nVec2 = vec3(), vec3()
  for i = 2, trajectoryCount-1 do
    nVec1:setCross(trajectory[i].biNormal, trajectory[i-1].edgeVec); nVec1:normalize()
    nVec2:setCross(trajectory[i].biNormal, trajectory[i].edgeVec); nVec2:normalize()

    trajectory[i].normalOrig:setAdd2(nVec2, nVec1)
    local normVecLen = trajectory[i].normalOrig:length()
    trajectory[i].normalOrig:setScaled(1 / max(normVecLen, 1e-30))

    trajectory[i].normal:set(trajectory[i].normalOrig)

    trajectory[i].radiusMult = (1 - nVec1:dot(nVec2) * 0.5) * normVecLen
    trajectory[i].halfWidth = trajectory[i].radius * trajectory[i].radiusMult
  end
  trajectory[trajectoryCount].normalOrig:setCross(trajectory[trajectoryCount].biNormal, trajectory[trajectoryCount-1].edgeVec); trajectory[trajectoryCount].normalOrig:normalize()
  trajectory[trajectoryCount].normal:set(trajectory[trajectoryCount].normalOrig)

  local energy = 0
  trajectory[1].curvature = 0
  for i = 2, trajectoryCount-1 do
    trajectory[i].curvature = inCurvature(-trajectory[i-1].edgeVec, -trajectory[i].edgeVec)
    --trajectory[i].curvature = dCurvature(trajectory[i-1].edgeVec, trajectory[i].edgeVec)
    energy = energy + trajectory[i].curvature * trajectory[i].curvature
  end
  trajectory[trajectoryCount].curvature = 0
  trajectory.energy = energy

  return trajectory
end

local function curvatureFlow(trajectory, s)
  local trajectoryCount = #trajectory

  -- init forces
  for i = 1, trajectoryCount do trajectory[i].force:set(0, 0, 0) end

  -- Calculate gradient ("spring") forces
  local nforce, v1, v2 = vec3(), vec3(), vec3()
  for i = 2, trajectoryCount-1 do
    v1:setScaled2(trajectory[i-1].edgeVec, 1 / trajectory[i-1].length)
    v2:setScaled2(trajectory[i].edgeVec, 1 / trajectory[i].length)

    nforce:setSub2(v2, v1); nforce:normalize() -- force in the direction of the angle bisector

    -- force scales with either the angle between the edges incident on node i or the path curvature at node i
    if s.forceMag == 'angle' then
      nforce:setScaled(max(0, 1 - v1:dot(v2)) * s.forceMultiplier)
    elseif s.forceMag == 'curvature' then
      nforce:setScaled(trajectory[i].curvature * s.forceMultiplier)
    end

    if s.forceScale == 'normalize' then
      nforce:setScaled(1 / (s.fRange * 2))
    end

    -- when applying a force to a node, we subtract equal and opposite (in sum) forces from its adjacent nodes
    -- when fRange is 1 we subtract those forces only from the immediate neighboors (1 left and 1 right) of node i.
    -- the sum of these forces i.e. force applied to node i and nodes i-nw, i+fRange, should equal zero.
    for j = max(1, i-s.fRange), min(i+s.fRange, trajectoryCount) do
      if j ~= i then
        trajectory[j].force:setSub(nforce) -- subtract nforce from node i's adjecent nodes (these are the reaction forces)
        trajectory[i].force:setAdd(nforce) -- add nforce to node i
      end
    end
  end

  -- Integrate Forces
  local tmpVec = vec3()
  local nodeConv = true
  for i = 1, trajectoryCount do
    if not trajectory[i].fixed then
      local k = trajectory[i].normal:dot(trajectory[i].force)
      local forceDir = sign(k) * max(0, sign2(k * trajectory[i].prevFDir)) -- stop moving before switching direction (improves stability)
      local displacement = forceDir * min(abs(k), s.hta)
      trajectory[i].prevFDir = forceDir

      local dispFromCenter = s.dispLimFromCenterAbs or (s.dispLimFromCenterRel and s.dispLimFromCenterRel * trajectory[i].halfWidth) or math.huge

      -- Calculate the new bounded lateral co-ordinate (x-norm) of the node
      local leftLimit = max(-trajectory[i].halfWidth + s.distFromEdge, -dispFromCenter)  -- left limit
      local rightLimit = min(trajectory[i].halfWidth - s.distFromEdge, dispFromCenter) -- right limit
      local newLateralXnorm = clamp(trajectory[i].lateralXnorm + displacement, leftLimit, rightLimit)

      -- Calculate the new node position
      tmpVec:setScaled2(trajectory[i].normal, newLateralXnorm - trajectory[i].lateralXnorm)
      trajectory[i].pos:setAdd(tmpVec)
      trajectory[i].lateralXnorm = newLateralXnorm

      -- check if I'm not limited by frame limit or if I'm limited by frame limit but I'm hitting lower/upper bounds
      if abs(k) < s.hta then
        nodeConv = nodeConv and true
      elseif newLateralXnorm == leftLimit or newLateralXnorm == rightLimit then
        nodeConv = nodeConv and true
      else
        nodeConv = nodeConv and false
      end

      -- tmpVec:setScaled2(trajectory[i].normal, displacement)
      -- trajectory[i].pos:setAdd(tmpVec)

      -- -- Check boundaries
      -- -- TODO: This does not work nicely with evolving normals
      -- if not trajectory[i].pathNode then -- TODO: if a node would go beyond the boundary do not displace it (to avoid node drifting)
      --   local xnorm = trajectory[i].pos:xnormOnLine(trajectory[i-1].posOrig, trajectory[i+1].posOrig)
      --   local dist = trajectory[i].pos:distanceToLine(trajectory[i-1].posOrig, trajectory[i+1].posOrig)
      --   local halfWidth = trajectory[i-1].halfWidth + (trajectory[i+1].halfWidth - trajectory[i-1].halfWidth) * xnorm
      --   local p = linePointFromXnorm(trajectory[i-1].posOrig, trajectory[i+1].posOrig, xnorm)
      --   local dispFromCenter = s.dispLimFromCenterAbs or (s.dispLimFromCenterRel and s.dispLimFromCenterRel * halfWidth) or math.huge
      --   -- this line might not work nicely if p and trajectory[i].pos are very close together
      --   -- but (p - trajectory[i].pos) should be parallel to the original normal
      --   -- trajectory[i].pos:setSub((p - trajectory[i].pos):normalized() * max(0, dist - min(dispFromCenter, min(0, halfWidth - s.distFromEdge)))) -- TODO: add distFromEdge limit here
      --   local dirSign = sign2((p - trajectory[i].pos):dot(trajectory[i].normalOrig))
      --   trajectory[i].pos:setSub(dirSign * trajectory[i].normalOrig * max(0, dist - min(dispFromCenter, min(0, halfWidth - s.distFromEdge))))
      -- else
      --   -- if node position is outside the boundaries move it back
      --   local dispFromCenter = s.dispLimFromCenterAbs or (s.dispLimFromCenterRel and s.dispLimFromCenterRel * trajectory[i].halfWidth) or math.huge
      --   local xnorm = trajectory[i].pos:xnormOnLine(trajectory[i].posOrig, trajectory[i].posOrig + trajectory[i].normalOrig)
      --   trajectory[i].pos:setAdd(sign(xnorm) * min(0, max(0, min(dispFromCenter, trajectory[i].halfWidth - s.distFromEdge)) - abs(xnorm)) * trajectory[i].normalOrig)
      -- end
    end
  end

  -- Update Edge Vectors and lengths
  for i = 1, trajectoryCount-1 do
    trajectory[i].edgeVec:setSub2(trajectory[i+1].pos, trajectory[i].pos)
    trajectory[i].length = trajectory[i].edgeVec:length()
  end

  -- Update normals wherever applicable
  if trajectory[1].evolveNormal then
    trajectory[1].normal:setCross(trajectory[1].biNormal, trajectory[1].edgeVec)
    trajectory[1].normal:normalize()
  end
  for i = 2, trajectoryCount-1 do
    if trajectory[i].evolveNormal then
      v1:setCross(trajectory[i].biNormal, trajectory[i-1].edgeVec); v1:normalize()
      v2:setCross(trajectory[i].biNormal, trajectory[i].edgeVec); v2:normalize()
      trajectory[i].normal:setAdd2(v2, v1); trajectory[i].normal:normalize()
    end
  end
  if trajectory[trajectoryCount].evolveNormal then
    trajectory[trajectoryCount].normal:setCross(trajectory[trajectoryCount].biNormal, trajectory[trajectoryCount-1].edgeVec)
    trajectory[trajectoryCount].normal:normalize()
  end

  -- Calculate curvatures
  local energy_prev = trajectory.energy
  local energy = 0
  trajectory[1].curvature = 0
  for i = 2, trajectoryCount-1 do
    trajectory[i].curvature = inCurvature(trajectory[i-1].pos - trajectory[i].pos, trajectory[i].pos - trajectory[i+1].pos)
    --trajectory[i].curvature = dCurvature(trajectory[i-1].edgeVec, trajectory[i].edgeVec)
    energy = energy + trajectory[i].curvature * trajectory[i].curvature
  end
  trajectory[trajectoryCount].curvature = 0
  trajectory.energy = energy

  -- Trajectory has converged if:
  -- 1. The change in total curvature energy is less than 0.1% of previous energy
  -- 2. All points have converged (nodeConv is true)
  -- Sets to nil if not converged, true if converged
  trajectory.converged = abs(energy - energy_prev) < s.tolFactor * energy_prev and nodeConv

  return trajectory
end

local function optimizePath(path, s)
  --[[ Inputs:
    path: a sequence of navigraph waypoint names ('strings')

    Optional:
    s = {
      segDistSplitLim = nil, -- Optional: will generate new nodes between path nodes that guarantee the maximum distance between any two nodes does not exceed segDistSplitLim meters
      fixPathNodes = nil, -- Do not dispace trajectory nodes that are path nodes (will only displace the generated intermediate nodes)
      fixEndpoints = nil, -- Value: boolean/nil. End points of path will not be displaced
      evolvePathNormals = nil, -- Value: boolean/nil. normal vectors of path nodes will be recalculated on each iteration (might affect stability, when road width limits are reached)
      evolveMidNormals = nil, -- Value: boolean/nil. normal vectors of generated nodes (nodes not belonging to the original path) will be recalculated on each iteration (might affect stability, when road width limits are reached)
      evolveEndNormals = false, -- Value: boolean/nil. normal vectors of path end nodes will be recalculated on each iteration (might affect stability, when road width limits are reached)
      hta = 0.5, -- maximum displacement on each optimization cycle (meters). Higher values means faster convergence but posibly lower stability
      fRange = 1, -- number of nodes before and after node i onto which reaction forces from node i will be applied (a value of 1 will only apply reaction forces to the immediate neighboors of a node)
      forceMag = 'curvature', -- Values: 'curvature' or 'angle' -- whether the force on each node will scale with the angle between the incident edges on that node or with the curvature calculated for that node
      forceScale = nil, -- Values: 'normalize' or nil
      forceMultiplier = 1, -- default: 1. A multiplier to forceMag.
      iterations = 120, -- number of optimizations cycles. -- TODO: At this point this is just a guess.
      distFromEdge = 1 -- Distance limit from road edge (m)
      dispLimFromCenterAbs = nil -- Displacement limit from road centerline (m)
      dispLimFromCenterRel = nil -- Displacement limit from road centerline in units relative to the road half width (radius) at that point
      logData = nil, -- If not nil, bending energy data will be logged to a csv file. If a string, it will be used as the file name.
      logTrajectory = nil, -- If not nil, trajectory data will be logged to a csv file. If a string, it will be used as the file name.
      s.tolFactor = nil -- (Typ. Val 0.001) Optimization will terminate if the relative difference between the previous and current iteration bending energy is less than tolFactor
                        -- The bending energy is calculated as the squared sum of node curvatures
    }
  --]]

  if not path[2] then return end

  s.hta = s.hta or 0.1
  s.fRange = s.fRange or 1
  s.forceMag = s.forceMag or 'angle'
  s.forceMultiplier = s.forceMultiplier or 2
  s.iterations = s.iterations or 120
  s.distFromEdge = s.distFromEdge or 1
  s.tolFactor = s.tolFactor or 0

  local dataToCSV
  local trajectoryToCSV2
  if s.logData then
    dataToCSV = require('csvlib').newCSV("Iteration (i)", "energy (1/m^2)")
  end
  if s.logTrajectory then
    trajectoryToCSV2 = require('csvlib').newCSV("posX", "posY", "posZ", "radius")
  end

  --timeprobe()
  local trajectory = createTrajectory(path, s)

  if dataToCSV then
    dataToCSV:add(0, trajectory.energy)
  end

  for i = 1, s.iterations do
    curvatureFlow(trajectory, s)

    if dataToCSV then
      dataToCSV:add(i, trajectory.energy)
    end

    if trajectory.converged then
     break
    end
  end
  --timeprobe()

  if trajectoryToCSV2 then
    for i = 1, #trajectory do
      trajectoryToCSV2:add(trajectory[i].pos.x, trajectory[i].pos.y, trajectory[i].pos.z, trajectory[i].halfWidth)
    end
  end

  if dataToCSV then
    dataToCSV:write(type(s.logData) == 'string' and s.logData or nil)
  end

  if trajectoryToCSV2 then
    trajectoryToCSV2:write(type(s.logTrajectory) == 'string' and s.logTrajectory or nil)
  end

  return trajectory
end

local function trajectoryVisualDebug(trajectory)
  local red = ColorF(1, 0, 0, 1)
  local blue = ColorF(0, 0, 1, 1)
  local gray = ColorF(0.5, 0.5, 0.5, 1)
  local grayOpaque = ColorF(0, 0, 0, 0.1)
  local black = ColorF(0, 0, 0, 1)
  local green = ColorF(0, 1, 0, 1)

  -- Draw Nodes
  for i = 1, #trajectory do
    -- Position
    debugDrawer:drawSphere(trajectory[i].pos, 0.2, red)

    -- Original Position
    debugDrawer:drawSphere(trajectory[i].posOrig, 0.1, black)

    -- Node width
    debugDrawer:drawSphere(trajectory[i].posOrig, trajectory[i].halfWidth, grayOpaque)

    -- Normal Vector
    debugDrawer:drawLine(trajectory[i].pos, trajectory[i].pos + trajectory[i].normal, red)

    -- Force Vector
    debugDrawer:drawLine(trajectory[i].pos, trajectory[i].pos + trajectory[i].force * 10, green)
  end

  -- Draw Edges
  for i = 1, #trajectory-1 do
    debugDrawer:drawCylinder(trajectory[i].pos, trajectory[i+1].pos, 0.02, red)
  end
end

local drawTrajectory = nop
local trajectoryToDraw = nil
local function debugDrawTrajectory(trajectory)
  if trajectory then
    drawTrajectory = trajectoryVisualDebug
    trajectoryToDraw = trajectory
  else
    drawTrajectory = nop
    trajectoryToDraw = nil
  end
end

-- returns the displacement value of the lane (negative = left, positive = right)
local function getLaneOffset(nid1, nid2, width, lane, laneCount)
  local link = map.nodes[nid1].links[nid2] or map.nodes[nid2].links[nid1]
  if link.inNode == nid2 then
    nid1, nid2 = nid2, nid1
  end

  return (lane - laneCount / 2 - 0.5) * (width / laneCount)
end

local function stringToNumber(str)
  str = hashStringSHA256(str)
  local res = 0
  for i = 1, #str do
    res = res + str:byte(i)
  end
  return res % 13
end

local function jetColorF(x, a)
  return ColorF(min(1, max(0, 4 * x - 2)), min(1, max(0, 2 - abs(4 * x - 2))), min(1, max(0, 2 - 4 * x)), a)
end

local function debugDraw1()
  if gp then
    local camPos = core_camera.getPosition()
    local black = ColorF(0, 0, 0, 1)
    local gatedRoadColor = ColorF(1, 0.5, 0.5, 0.5)
    local arrowSize1 = Point2F(1, 1)
    local arrowSize2 = Point2F(1, 0)
    local laneColor1 = ColorF(1, 0.2, 0.2, 1)
    local laneColor2 = ColorF(0.2, 0.4, 1, 1)
    local drawDist = 100

    for node1id in nodeKdTree:queryNotNested(quadtree.pointBBox(camPos.x, camPos.y, drawDist)) do
      local node = map.nodes[node1id]
      if node.junction then
        debugDrawer:drawText(map.nodes[node1id].pos + vec3(0, 0, 2), node1id, black)
      end
      for node2id, edgeData in pairs(node.links) do
        if node.junction then
          local n1Pos = getEdgeNodePosition(node1id, edgeData)
          local n1Rad = getEdgeNodeRadius(node1id, edgeData)
          debugDrawer:drawSphere(n1Pos, n1Rad, jetColorF(stringToNumber(node1id) / 12, 0.3))
        end

        if map.nodes[node2id].junction then
          local n2Pos = getEdgeNodePosition(node2id, edgeData)
          local n2Rad = getEdgeNodeRadius(node2id, edgeData)
          debugDrawer:drawSphere(n2Pos, n2Rad, jetColorF(stringToNumber(node2id) / 12, 0.3))
        end

        local edgeColor
        if edgeData.gatedRoad then
          edgeColor = gatedRoadColor
        else
          local rainbow = rainbowColor(50, clamp(edgeData.drivability, 0, 1) * 15, 1)
          edgeColor = ColorF(rainbow[1], rainbow[2], rainbow[3], 0.5)
        end
        debugDrawer:drawSquarePrism(edgeData.inPos, edgeData.outPos, Point2F(0.6, edgeData.inRadius * 2), Point2F(0.6, edgeData.outRadius * 2), edgeColor)

        if edgeData.lanes then
          local strLen = 1 -- number of characters representing a single lane in the lane string
          local laneCount = #edgeData.lanes / strLen

          local edgeDirVec = edgeData.outPos - edgeData.inPos
          local edgeLength = edgeDirVec:length()
          edgeDirVec:setScaled(1 / (edgeLength + 1e-30))
          local right1 = edgeDirVec:cross(map.nodes[edgeData.inNode].normal)

          -- calculate arrow spacing to draw lane direction indicator arrows
          local arrowLength = 2
          local usableEdgeLength = edgeLength - arrowLength
          local k = max(1, math.floor(usableEdgeLength / 30) - 1) -- number of arrows per lane (30m between arrows). skip first and last.
          local dispVec = (usableEdgeLength / (k + 1)) * edgeDirVec
          local arrowLengthVec = arrowLength * edgeDirVec

          for i = 1, laneCount do -- draw lanes
            -- Draw arrows indicating lane direction
            local offset1 = getLaneOffset(edgeData.inNode, edgeData.outNode, min(edgeData.inRadius, edgeData.outRadius) * 2, i, laneCount)
            local laneDir = edgeData.lanes:byte(i) == 43 --> ascii code for '+'
            local color = laneDir and laneColor2 or laneColor1
            local tailPos = edgeData.inPos + right1 * offset1
            local tipPos  = vec3()
            for j = 1, k do
              tailPos:setAdd(dispVec)
              tipPos:setAdd2(tailPos, arrowLengthVec)
              debugDrawer:drawSquarePrism(tailPos, tipPos, laneDir and arrowSize1 or arrowSize2, laneDir and arrowSize2 or arrowSize1, color)
            end
          end
        end
      end
    end
  end
end

local nodeIdsDrawn
local function edgeDebugDraw()
  if gp then
    local camPos = core_camera.getPosition()
    local black = ColorF(0, 0, 0, 1)
    local gatedRoadColor = ColorF(1, 0.5, 0.5, 0.5)
    local arrowSize1 = Point2F(1, 1)
    local arrowSize2 = Point2F(1, 0)
    local laneColor1 = ColorF(1, 0.2, 0.2, 1)
    local laneColor2 = ColorF(0.2, 0.4, 1, 1)
    local drawDist = clamp(camPos.z - be:getSurfaceHeightBelow(camPos), 20, 300)

    local tmpPos, edgeDirVec, right1 = vec3(), vec3(), vec3()
    for edgeID in edgeKdTree:queryNotNested(quadtree.pointBBox(camPos.x, camPos.y, drawDist)) do
      local i = stringFind(edgeID, '\0')
      local node1id = stringSub(edgeID, 1, i-1)
      local node2id = stringSub(edgeID, i+1, #edgeID)

      local n1Pos, n2Pos = gp:getEdgePositions(node1id, node2id)
      local n1Rad, n2Rad = gp:getEdgeRadii(node1id, node2id)

      -- Draw node 1 sphere if node is a junction or of degree 1
      if map.nodes[node1id].junction or not next(gp.graph[node1id], next(gp.graph[node1id])) then
        debugDrawer:drawSphere(n1Pos, n1Rad, jetColorF(stringToNumber(node1id) / 12, 0.3))
        --debugDrawer:drawSphere(n1Pos, n1Rad, jetColorF(map.nodes[node1id].debugColorCode, 0.3))
        if not nodeIdsDrawn[node1id] then
          nodeIdsDrawn[node1id] = true
          tmpPos:set(0, 0, 2)
          tmpPos:setAdd(map.nodes[node1id].pos)
          debugDrawer:drawText(tmpPos, node1id, black)
        end
      end

      -- Draw node 2 sphere if node is a junction or of degree 1
      if map.nodes[node2id].junction or not next(gp.graph[node2id], next(gp.graph[node2id])) then
        debugDrawer:drawSphere(n2Pos, n2Rad, jetColorF(stringToNumber(node2id) / 12, 0.3))
        --debugDrawer:drawSphere(n2Pos, n2Rad, jetColorF(map.nodes[node2id].debugColorCode, 0.3))
        if not nodeIdsDrawn[node2id] then
          nodeIdsDrawn[node2id] = true
          tmpPos:set(0, 0, 2)
          tmpPos:setAdd(map.nodes[node2id].pos)
          debugDrawer:drawText(tmpPos, node2id, black)
        end
      end

      local edgeData = gp.graph[node1id][node2id]

      -- Draw Edge
      local edgeColor
      if edgeData.gated > 0 then
        edgeColor = gatedRoadColor
      else
        local rainbow = rainbowColor(50, clamp(edgeData.drivability, 0, 1) * 15, 1)
        edgeColor = ColorF(rainbow[1], rainbow[2], rainbow[3], 0.5)
      end
      debugDrawer:drawSquarePrism(n1Pos, n2Pos, Point2F(0.6, n1Rad * 2), Point2F(0.6, n2Rad * 2), edgeColor)

      if edgeData.lanes then
        local strLen = 1 -- number of characters representing a single lane in the lane string
        local laneCount = #edgeData.lanes / strLen

        local inPos = edgeData.inPos or gp.positions[edgeData.inNode]
        local outPos = edgeData.outPos or gp.positions[edgeData.outNode]

        edgeDirVec:setSub2(outPos, inPos)
        local edgeLength = edgeDirVec:length()
        edgeDirVec:setScaled(1 / (edgeLength + 1e-30))
        right1:setCross(edgeDirVec, map.nodes[edgeData.inNode].normal)

        -- calculate arrow spacing to draw lane direction indicator arrows
        local arrowLength = 2
        local usableEdgeLength = edgeLength - arrowLength
        local k = max(1, math.floor(usableEdgeLength / 30) - 1) -- number of arrows per lane (30m between arrows). skip first and last.
        local dispVec = (usableEdgeLength / (k + 1)) * edgeDirVec
        local arrowLengthVec = arrowLength * edgeDirVec
        local minRadius = min(n1Rad, n2Rad)

        local tailPos, tipPos = vec3(), vec3()
        for i = 1, laneCount do -- draw lanes
          -- Draw arrows indicating lane direction
          local offset1 = getLaneOffset(edgeData.inNode, edgeData.outNode, minRadius * 2, i, laneCount)
          local laneDir = edgeData.lanes:byte(i) == 43 --> ascii code for '+'
          local color = laneDir and laneColor2 or laneColor1
          tailPos:setAdd2(inPos, right1 * offset1)
          for j = 1, k do
            tailPos:setAdd(dispVec)
            tipPos:setAdd2(tailPos, arrowLengthVec)
            debugDrawer:drawSquarePrism(tailPos, tipPos, laneDir and arrowSize1 or arrowSize2, laneDir and arrowSize2 or arrowSize1, color)
          end
        end
      end
    end
    table.clear(nodeIdsDrawn)
  end
end

M.drawNavGraphState = 'off'
local drawNavGraph = nop
local function toggleDrawNavGraph()
  if drawNavGraph == nop then
    --colorNodes(map.nodes)
    drawNavGraph = edgeDebugDraw
    M.drawNavGraphState = 'on'
    nodeIdsDrawn = table.new(0, 50)
  else
    drawNavGraph = nop
    M.drawNavGraphState = 'off'
    nodeIdsDrawn = nil
  end
end

local function updateGFX(dtReal, dtSim)
  if dtSim > 0 then
    be:sendToMailbox("objUpdate", lpack.encodeBinWorkBuffer(M.objects))
    if objectsCount == 0 then
      table.clear(M.objects)
    end
    objectsCount = 0
  end
  delayedLoad:update(dtReal)
  drawTrajectory(trajectoryToDraw)
  drawNavGraph()
end

local function Mload()
  if loadedMap then return end
  loadedMap = true
  loadMap()
end

local function assureLoad()
  if not loadedMap then
    loadMap()
  end
  loadedMap = false
end

local function onMissionLoaded()
  loadedMap = false
end

local function onWaypoint(args)
  --print('onWaypoint')
  --dump(args)

  -- local aiData = {subjectName = args.subjectName, triggerName = args.triggerName, event = args.event, mode = args.mode}
  -- args.subject:queueLuaCommand('ai.onWaypoint(' .. serialize(aiData) .. ')')

  --[[
  --if args.triggerName
  local triggerName = string.match(args.triggerName, "(%a*)(%d+)")
  local triggerNum = string.match(args.triggerName, "(%d+)")

  local v = scenetree.findObject(args.subjectName)
  local nextTrigger = scenetree.findObject(triggerName .. (triggerNum + 1))
  if args.subject and nextTrigger then
    --local ppos = player:getPosition()
    local tpos = nextTrigger:getPosition()
    --print("player pos: " .. tostring(ppos))
    --print("trigger pos: " .. tostring(tpos))
    local l = 'ai.setTarget('..tostring(tpos)..')'
    --print(l)
    args.subject:queueLuaCommand(l)

  end
  ]]

  --guihooks.trigger('Message', {msg = 'Trigger "' .. args.triggerName .. '" : ' .. args.event, time = 1})
end

-- TODO: please fix these functions, so users can interactively add/remove/modify the waypoints in the editor and directly see changes.
local function onAddWaypoint(wp)
  --print("waypoint added: " .. tostring(wp))
  if isEditorEnabled then
    delayedLoad:callAfter(0.5, loadMap)
  end
end

local function onRemoveWaypoint(wp)
  --print("waypoint removed: " .. tostring(wp))
  if isEditorEnabled then
    delayedLoad:callAfter(0.5, loadMap)
  end
end

local function onModifiedWaypoint(wp)
  --print("waypoint modified: " .. tostring(wp))
  if isEditorEnabled then
    delayedLoad:callAfter(0.5, loadMap)
  end
end

local function onFilesChanged(files)
  for _,v in pairs(files) do
    if v.filename == mapFilename then
      log('D', 'map', "map.json changed, reloading map")
      loadMap()
      return
    end
  end
end

local function request(objId, objbuildSerial)
  if objbuildSerial ~= buildSerial then
    be:queueObjectLua(objId, string.format("mapmgr.setMap(%d)", buildSerial))

    if core_trafficSignals then
      local signalsDict = core_trafficSignals.getMapNodeSignals() -- table of current traffic signal states, with map node names as keys
      if signalsDict and next(signalsDict) then
        be:queueObjectLua(objId, string.format("mapmgr.setSignals(%s)", serialize(signalsDict)))
      end
    end
  end
end

local function updateDrivabilities(changeSet)
  -- Dynamically Change edge Drivability for the Navgraph
  -- changeSet format {nodeA1, nodeB1, drivability1, nodeA2, nodeB2, drivability2, ...}
  if #changeSet % 3 ~= 0 then return end

  local hasChanged = false
  local graph = gp.graph
  for i = 1, #changeSet, 3 do
    if graph[changeSet[i]] then
      local edge = graph[changeSet[i]][changeSet[i+1]]
      local newDrivability = max(1e-30, changeSet[i+2])
      if edge and edge.drivability ~= newDrivability then
        edge.len = max(0, (edge.len - edge.gated) * edge.drivability / newDrivability + edge.gated)
        edge.drivability = newDrivability
        hasChanged = true
      end
    end
  end

  if hasChanged then -- send data if there is at least one change
    be:sendToMailbox("updateDrivabilities", lpack.encodeBinWorkBuffer(changeSet)) -- mailbox is cleared when map is loaded
    for objId, _ in pairs(M.objects) do
      be:queueObjectLua(objId, "mapmgr.updateDrivabilities()")
    end
  end
end

local function onSerialize()
  return {isEditorEnabled, buildSerial}
end

local function onDeserialize(s)
  isEditorEnabled, buildSerial = unpack(s)
  buildSerial = buildSerial or -1
end

local function setEditorState(enabled)
  isEditorEnabled = enabled
end

local function setState(newState)
  tableMerge(M, newState)
end

local function getState()
  for k, v in pairs(M.objectNames) do
    if type(k) == 'string' then
      if M.objects[v] then
        M.objects[v].name = k
      end
    end
  end
  for k, v in pairs(M.objects) do
    v.name = v.name or ''
    local vehicle = getObjectByID(k)
    v.licensePlate = vehicle and vehicle:getDynDataFieldbyName("licenseText", 0) or dumps(k)
  end
  return M
end

local function getMap()
  return map
end

local function getGraphpath()
  return gp
end

local function getManualWaypoints()
  return manualWaypoints
end

local function getTrackedObjects()
  return M.objects
end

-- recieves vehicle data from vehicles
local function objectData(objId, isactive, damage, states, objectCollisions)
  if objectsCount == 0 then
    tableClear(M.objects)
  end
  objectsCount = objectsCount + 1

  local object = getObjectByID(objId)
  if object and M.objects[objId] == nil then
    local obj = objectsCache[objId] or {view = true, pos = vec3(), vel = vec3(), dirVec = vec3(), dirVecUp = vec3()}

    obj.id = objId
    obj.active = isactive
    obj.damage = damage
    obj.states = states or emptyTable
    obj.uiState = object.uiState and tonumber(object.uiState)
    obj.objectCollisions = objectCollisions or emptyTable
    obj.pos:set(object:getPositionXYZ())
    obj.vel:set(object:getVelocityXYZ())
    obj.dirVec:set(object:getDirectionVectorXYZ())
    obj.dirVecUp:set(object:getDirectionVectorUpXYZ())
    obj.isParked = object.isParked and true

    objectsCache[objId] = obj
    M.objects[objId] = obj
  end
end

-- used to add explicit vehicle data
local function tempObjectData(objId, isactive, pos, vel, dirVec, dirVecUp, damage, objectCollisions)
  if objectsCount == 0 then
    tableClear(M.objects)
  end
  objectsCount = objectsCount + 1

  local obj = objectsCache[objId] or {
    id = objId,
    view = true,
    active = isactive,
    pos = pos,
    vel = vel,
    dirVec = dirVec,
    dirVecUp = dirVecUp,
    damage = damage,
    objectCollisions = objectCollisions}

  obj.id = objId
  obj.active = isactive
  obj.damage = damage
  obj.objectCollisions = objectCollisions or {}
  obj.pos:set(pos)
  obj.vel:set(vel)
  obj.dirVec:set(dirVec)
  obj.dirVecUp:set(dirVecUp)

  objectsCache[objId] = obj
  M.objects[objId] = obj
end

local function setNameForId(name, id)
  M.objectNames[name] = id
end

local function isCrashAvoidable(objectID, pos, radius)
  -- check if position (pos) with dimension radius is safe to spawn given object (objectID) in motion

  local obj = getObjectByID(objectID)
  if not obj then return true end

  radius = radius or 7.5

  local relativePos = pos - vec3(obj:getSpawnWorldOOBB():getCenter())
  local relativePosLen = relativePos:length()

  -- Over 150m, we assume it's safe to spawn
  if relativePosLen > 150 then return true end

  local objVel = vec3(obj:getVelocity())
  local relativeSpeed = max(objVel:dot(relativePos / (relativePosLen + 1e-30)), 0)
  local ff = 0.5 * vecUp:dot(vec3(obj:getDirectionVectorUp())) -- frictionCoeff * Normal Force.
  local objDirVec = vec3(obj:getDirectionVector())
  local fw = vecUp:dot(sign(objVel:dot(objDirVec)) * objDirVec) -- road grade force

  -- Prevents division by zero gravity
  local gravity = core_environment.getGravity()
  gravity = max(0.1, abs(gravity)) * sign2(gravity)

  local a = max(1e-30, -gravity * (ff + fw))
  return relativePosLen > relativeSpeed * relativeSpeed / (2 * a) + obj:getInitialLength() * 0.5 + radius
end

local function safeTeleport(vehId, posX, posY, posZ, rotX, rotY, rotZ, rotW, checkOnlyStatics_, visibilityPoint_, removeTraffic_, centeredPosition, resetVehicle)
  -- Wrapper function for spawn.safeTeleport()
  local veh = scenetree.findObject(vehId)
  --local veh = getObjectByID(id)
  local pos = vec3(posX, posY, posZ)
  local rot = quat(rotX, rotY, rotZ, rotW)
  spawn.safeTeleport(veh, pos, rot, checkOnlyStatics_, visibilityPoint_, removeTraffic_, centeredPosition, resetVehicle)
end

-- public interface
M.updateGFX = updateGFX
M.objectData = objectData
M.tempObjectData = tempObjectData
M.setNameForId = setNameForId
M.onWaypoint = onWaypoint
M.reset = loadMap
M.load = Mload
M.assureLoad = assureLoad
M.onMissionLoaded = onMissionLoaded
M.request = request
M.onAddWaypoint = onAddWaypoint
M.onRemoveWaypoint = onRemoveWaypoint
M.onModifiedWaypoint = onModifiedWaypoint
M.onFilesChanged = onFilesChanged
M.setState = setState
M.getState = getState
M.setEditorState = setEditorState
M.getMap = getMap
M.getGraphpath = getGraphpath
M.getRoadRules = getRoadRules
M.getManualWaypoints = getManualWaypoints
M.getTrackedObjects = getTrackedObjects
M.findClosestRoad = findClosestRoad
M.findBestRoad = findBestRoad
M.getPath = getPath
M.getNodesFromPathDist = getNodesFromPathDist
M.getPathLen = getPathLen
M.getPointNodePath = getPointNodePath
M.getPointToPointPath = getPointToPointPath
M.saveSVG = saveSVG
M.onSerialize = onSerialize
M.onDeserialize = onDeserialize
M.surfaceNormal = surfaceNormal
M.isCrashAvoidable = isCrashAvoidable
M.nameNode = nameNode
M.getNodeLinkCount = getNodeLinkCount
M.updateDrivabilities = updateDrivabilities
M.safeTeleport = safeTeleport
M.optimizePath = optimizePath
M.debugDrawTrajectory = debugDrawTrajectory
M.toggleDrawNavGraph = toggleDrawNavGraph
M.logGraphHashes = logGraphHashes
M.toggleShuffledPairs = toggleShuffledPairs

-- backward compatibility fixes below
setmetatable(M, {
  __index = function(tbl, key)
    if key == 'map' then
      if not M.warnedMapBackwardCompatibility then
        log('E', 'map', 'map.map API is deprecated. Please use map.getMap()')
        M.warnedMapBackwardCompatibility = true
      end
      return M.getMap()
    end
    return rawget(tbl, key)
  end
})

return M