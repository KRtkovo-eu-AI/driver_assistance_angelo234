-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local graphpath = require('graphpath')
local pointBBox = require('quadtree').pointBBox
local kdTreeBox2D = require('kdtreebox2d')
local buffer = require("string.buffer")

local stringFind, stringSub, stringFormat, max, min = string.find, string.sub, string.format, math.max, math.min

local M = {}

M.objects = {}
M.objectCollisionIds = {}

local mapData, mapBuildSerial, edgeKdTree, maxRadius, customMap
local lastSimTime = -1

local function updateDrivabilities()
  -- Dynamically Change edge Drivability for the Navgraph
  if not (mapData and mapData.graph) then return end

  local changeSet = obj:getLastMailbox('updateDrivabilities')
  if changeSet == "" then return end -- mailbox is empty
  changeSet = lpack.decode(changeSet) -- changeSet format: {nodeA1, nodeB1, driv1, nodeA2, nodeB2, driv2, ...}

  local graph = mapData.graph
  for i = 1, #changeSet, 3 do
    if graph[changeSet[i]] then
      local edge = graph[changeSet[i]][changeSet[i+1]]
      local newDrivability = max(1e-30, changeSet[i+2])
      if edge and edge.drivability ~= newDrivability then
        edge.len = max(0, (edge.len - edge.gated) * edge.drivability / newDrivability + edge.gated)
        changeSet[i+2] = newDrivability - edge.drivability -- keep track of whether an edge had its drivability reduced or increased
        edge.drivability = newDrivability
      end
    end
  end

  M.changeSet = changeSet
end

local function setMap(newbuildSerial)
  if newbuildSerial and newbuildSerial == mapBuildSerial then return end
  mapBuildSerial = newbuildSerial

  local _map = lpack.decode(obj:getLastMailbox('mapData'))
  if not (_map and _map.graphData and _map.edgeKdTree and _map.maxRadius and _map.nodeAliases) then return end

  maxRadius = _map.maxRadius
  M.nodeAliases = _map.nodeAliases

  mapData = graphpath.newGraphpath()
  mapData:import(_map.graphData)
  updateDrivabilities()
  M.mapData = mapData

  edgeKdTree = kdTreeBox2D.new()
  edgeKdTree:import(_map.edgeKdTree)

  M.rules = _map.rules

  obj:queueGameEngineLua("extensions.hook('onVehicleMapmgrUpdate', "..tostring(objectId)..")")
end

local function requestMap()
  obj:queueGameEngineLua(string.format('map.request(%s,%s)', objectId, mapBuildSerial))
end

local function setCustomMap(map)
  mapData = map
  M.mapData = mapData
  customMap = true
  mapBuildSerial = nil
end

local function clearCustomMap()
  if customMap then
    mapData = nil
    M.mapData = nil
    customMap = nil
  end
end

local function setSignals(data)
  M.signalsData = data
end

local function updateSignals(data)
  if M.signalsData then
    for i = 1, #data, 4 do
      M.signalsData[data[i]] = M.signalsData[data[i]] or {}
      M.signalsData[data[i]][data[i + 1]] = M.signalsData[data[i]][data[i + 1]] or {}
      M.signalsData[data[i]][data[i + 1]][data[i + 2]] = M.signalsData[data[i]][data[i + 1]][data[i + 2]] or {action = 0}
      M.signalsData[data[i]][data[i + 1]][data[i + 2]].action = tonumber(data[i + 3]) or 0
    end
  end
end

local buf = buffer.new() -- https://luajit.org/ext_buffer.html
local states = {}
local currentMailboxVersion = nil
local function sendTracking()
  if M.signalsData then
    local lastMailboxVersion = obj:getLastMailboxVersion("trafficSignalUpdates")
    if currentMailboxVersion ~= lastMailboxVersion then
      currentMailboxVersion = lastMailboxVersion
      updateSignals(lpack.decode(obj:getLastMailbox("trafficSignalUpdates")))
    end
  end

  local objCols = M.objectCollisionIds
  table.clear(objCols)
  obj:getObjectCollisionIds(objCols)

  if electrics.values.horn ~= 0 then states.horn = electrics.values.horn end
  if electrics.values.lightbar ~= 0 then states.lightbar = electrics.values.lightbar end
  if electrics.values.hazard_enabled ~= 0 then states.hazard_enabled = electrics.values.hazard_enabled end
  if electrics.values.ignitionLevel == 0 or electrics.values.ignitionLevel == 1 then states.ignitionLevel = electrics.values.ignitionLevel end

  buf:reset():putf('map.objectData(%s,%s,%s,', objectId, playerInfo.anyPlayerSeated, math.floor(beamstate.damage))

  -- add states to the buffer if they exist
  if next(states) then
    buf:put('{')
    for k, v in pairs(states) do
      buf:putf('[%q]=%s,', k, v)
    end
    buf:put('}')
  else
    buf:put('nil')
  end

  -- add object collisions to the buffer if they exist
  if objCols[1] then
    buf:put(',{')
    for i = 1, #objCols do buf:putf('[%s]=1,', objCols[i]) end
    buf:put('}')
  end

  buf:put(')')

  obj:queueGameEngineLua(buf)

  table.clear(states)
end

M.sendTracking = nop
local function enableTracking(name)
  obj:queueGameEngineLua(stringFormat('map.setNameForId(%s, %s)', name and '"'..name..'"' or objectId, objectId))
  M.sendTracking = sendTracking
end

local function disableTracking(forceDisable)
  if forceDisable or not playerInfo.anyPlayerSeated then
    M.sendTracking = nop
  end
end

local function reset()
  M.objects = {}
end

local function init()
  if wheels.wheelCount > 0 or (v.data.general and v.data.general.enableTracking) then
    enableTracking()
  end
end

local function getObjects()
  local simTime = obj:getSimTime()
  if simTime ~= lastSimTime then
    local objData = obj:getLastMailbox("objUpdate")
    M.objects = #objData == 0 and {} or lpack.decode(objData)
    lastSimTime = simTime
  end
  return M.objects
end

local p1, p2 = vec3(), vec3()
local function surfaceNormalBelow(p, r)
  --   p3
  --     \ r
  --      \    r
  --       p - - - p1     + > y
  --      /               v
  --     / r              x
  --   p2

  r = r or 2
  local hr = 1.2 * r -- calculation is guaranteed to be accurate up to ~ 50 deg (argtan(1.2)) inclination

  p1:set(p.x, p.y + r, p.z + hr)
  p2:set(p.x + 0.8660254037844386 * r, p.y - 0.5 * r, p1.z) -- sin(60) => 0.8660254037844386, cos(60) => 0.5
  local p3 = p2:copy()
  p3.x = p.x - 0.8660254037844386 * r

  p1.z = obj:getSurfaceHeightBelow(p1)
  p2.z = obj:getSurfaceHeightBelow(p2)
  p3.z = obj:getSurfaceHeightBelow(p3)

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

local function sqDistToLineSegmentZBias(s, a, b, wZ)
  -- squaredDistanceToLineSegment with a bias against height (z) differences
  wZ = wZ or 1.85
  local abx, aby, abz, asx, asy, asz = a.x-b.x, a.y-b.y, a.z-b.z, a.x-s.x, a.y-s.y, a.z-s.z
  local xnormC = min(max((abx*asx + aby*asy + abz*asz) / (abx*abx + aby*aby + abz*abz + 1e-30), 0), 1)
  return square(asx - abx*xnormC) + square(asy - aby*xnormC) + wZ * square(asz - abz*xnormC)
end

-- the same function is also located in ge/map.lua
local function findClosestRoad(pos, wZ)
  --log('A','mapmgr', 'findClosestRoad called with '..pos.x..','..pos.y..','..pos.z)
  pos = pos or obj:getPosition()

  local bestRoad1, bestRoad2, bestDist
  local searchRadius = maxRadius
  repeat
    local searchRadiusSq = searchRadius * searchRadius
    local minCurDist = searchRadiusSq * 4
    bestDist = searchRadiusSq
    for item_id in edgeKdTree:queryNotNested(pointBBox(pos.x, pos.y, searchRadius)) do
      local i = stringFind(item_id, '\0')
      local n1id = stringSub(item_id, 1, i-1)
      local n2id = stringSub(item_id, i+1, #item_id)
      local n1Pos, n2Pos = mapData:getEdgePositions(n1id, n2id)
      local curDist = sqDistToLineSegmentZBias(pos, n1Pos, n2Pos, wZ)

      if curDist <= bestDist then
        bestDist = curDist
        bestRoad1 = n1id
        bestRoad2 = n2id
      else
        minCurDist = min(minCurDist, curDist) -- this is the smallest curDist that is larger than bestDist
      end
    end

    searchRadius = math.sqrt(max(minCurDist, searchRadiusSq * 4))
  until bestRoad1 or searchRadius > 200

  return bestRoad1, bestRoad2, math.sqrt(bestDist)
end

local function findBestRoad(pos, dir, wZ)
  -- searches for best road with respect to position and direction, with a fallback to the generic findClosestRoad function
  pos = pos or obj:getPosition()
  dir = dir or obj:getDirectionVector()

  local nodePositions = mapData.positions
  local nodeRadius = mapData.radius
  local bestRoad1, bestRoad2, bestDist
  local currRoads = {}

  for item_id in edgeKdTree:queryNotNested(pointBBox(pos.x, pos.y, 20)) do -- assuming that no roads would have a radius greater than 20 m
    local i = stringFind(item_id, '\0')
    local n1id = stringSub(item_id, 1, i-1)
    local n2id = stringSub(item_id, i+1, #item_id)
    local curDist = sqDistToLineSegmentZBias(pos, nodePositions[n1id], nodePositions[n2id], wZ)

    if curDist <= square(math.max(nodeRadius[n1id], nodeRadius[n2id])) then
      local xnorm = pos:xnormOnLine(nodePositions[n1id], nodePositions[n2id])
      if xnorm >= 0 and xnorm <= 1 then -- insert result if it is within road boundaries
        table.insert(currRoads, {n1id, n2id, curDist})
      end
    end
  end

  if not currRoads[1] then
    --log('W', 'mapmgr', 'no results for findBestRoad, now using findClosestRoad')
    return findClosestRoad(pos, wZ) -- fallback
  elseif not currRoads[2] then -- only one entry in the table
    return currRoads[1][1], currRoads[1][2], math.sqrt(currRoads[1][3])
  end

  local bestDot = 0
  for _, v in ipairs(currRoads) do
    local dirDot = math.abs(dir:dot((nodePositions[v[1]] - nodePositions[v[2]]):normalized()))
    if dirDot >= bestDot then -- best direction
      bestDot = dirDot
      bestRoad1, bestRoad2, bestDist = v[1], v[2], v[3]
    end
  end

  return bestRoad1, bestRoad2, math.sqrt(bestDist)
end

local function startPosLinks(position, wZ)
  wZ = wZ or 1 -- zbias
  local nodePositions = mapData.positions
  local nodeRadius = mapData.radius
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
            edgeVec:set(nodePositions[n2id])
            edgeVec:setSub(n1Pos)
            tmpVec:set(position)
            tmpVec:setSub(n1Pos) -- node1ToPosVec
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

local function getPointToPointPath(startPos, targetPos, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
  -- startPos: path source position
  -- targetPos: target position (vec3)
  -- cutOffDrivability: penalize roads with drivability <= cutOffDrivability
  -- dirMult: amount of penalty to impose to path if it does not respect road legal directions (should be larger than 1 typically >= 10e4).
  --          If equal to nil or 1 then it means no penalty.
  -- penaltyAboveCutoff: penalty multiplier for roads above the drivability cutoff
  -- penaltyBelowCutoff: penalty multiplier for roads below the drivability cutoff
  -- wZ: number (typically >= 1). When higher than 1 destination node of optimum path will be biased towards minimizing height difference to targetPos.

  if mapData == nil or edgeKdTree == nil then return {} end
  wZ = wZ or 4
  local iter = startPosLinks(startPos, wZ)
  return mapData:getPointToPointPath(startPos, iter, targetPos, cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wZ)
end

M.init = init
M.reset = reset
M.requestMap = requestMap
M.setMap = setMap
M.setSignals = setSignals
M.updateSignals = updateSignals
M.enableTracking = enableTracking
M.disableTracking = disableTracking
M.getObjects = getObjects
M.updateDrivabilities = updateDrivabilities
M.surfaceNormalBelow = surfaceNormalBelow
M.findClosestRoad = findClosestRoad
M.findBestRoad = findBestRoad
M.getPointToPointPath = getPointToPointPath
M.setCustomMap = setCustomMap
M.clearCustomMap = clearCustomMap

return M
