-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This class is the 2nd version of the driveline.
-- It uses route.lua under the hood.

-- steps
-- 1. loads a race path from race.json
-- 2. calculates the race's ai path
-- 3. loads a notebook from notebook.json
-- 4. merges the pacenote waypoints from the notebook with the ai path
-- 5. creates a route from the resulting path

local kdTreeP3d = require('kdtreepoint3d')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local Route = require('/lua/ge/extensions/gameplay/route/raceRoute')
local WaypointTypes = require('/lua/ge/extensions/gameplay/rally/notebook/waypointTypes')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')

local logTag = ''

local C = {}

function C:init()
  -- callbacks. not all may be used.
  self.onPacenoteCsDynamicHit = nil
  self.onPacenoteCsImmediateHit = nil
  self.onPacenoteCsStaticHit = nil
  self.onPacenoteCeStaticHit = nil
  self.onPacenoteCsOffsetHit = nil
  self.onPacenoteCeOffsetHit = nil
  self.onPacenoteCornerPercentHit = nil

  -- stuff for debugging helpers
  self.trackMouseLikeVehicle = false
  self.trackMouseLikeVehicleEnableMovement = false

  self.racePath = nil
  self.notebookPath = nil

  self.raceCompletionData = {
    distKm = nil,
    distM = nil,
    distPct = nil
  }

  self:_reset()
end

function C:enableTriggerLogging()
  if gameplay_rally and gameplay_rally.getDebugLogging() then
    return true
  else
    return false
  end
end

function C:isLoaded()
  return self.loaded
end

function C:_reset()
  -- log('D', logTag, '_reset')

  self.loaded = false

  -- stuff for loading the route
  self.route = nil
  self.routeStatic = nil
  self.kdTreeRouteStatic = nil
  self.stableIdIndexRouteStatic = nil
  self.vehicleTracker = nil

  -- stuff for driving the route
  self.nextPacenoteIdxForEval = 1
  self.nextPacenoteWpFromRecalc = nil
  self.nextRacePathnodeFromRecalc = nil
  self.pacenoteLookbehindWindow = 5
  -- self.pacenoteLookaheadWindow = 0
  -- events
  self.events = {}
  self.eventLog = {}
  -- resetting
  self.recalcNeeded = false
  self.dtSimSumSinceRecalc = nil
  -- self.disableTracking = false

  -- audio triggering
  self.staticDistanceThresholdMeters = 0.1
  self.defaultCodriverTiming = 3.0
  self.minEvalSpeedMph = 1.0
  self.minWaitTimeSinceRecalc = 2.0

  -- v1 - result: timing still too late for high speed straights into tight turns.
  -- self.audioLenScaler = 0.75
  -- self.scaleMinSpeedMph = 30
  -- self.scaleMaxSpeedMph = 60
  -- self.minMultiplier = 1.0
  -- self.maxMultiplier = 1.2
  -- v2
  self.audioLenScaler = 0.75
  self.scaleMinSpeedMph = 50 -- changed from 30 to 50 to make scaling more pronounced at higher speeds.
  self.scaleMaxSpeedMph = 90 -- changed from 60 to 90
  self.minMultiplier = 1.0
  self.maxMultiplier = 1.5 -- changed from 1.2 to 1.5 to make speed scaling more aggressive.

  -- mouse-based positioning for faster debugging
  self.mousePos = nil
  self.mouseSpeedMs = 0
  self.smoothedPos = nil
end

function C:baseCodriverTiming()
  local baseCodriverTimingSetting = settings.getValue("rallyCodriverTiming")
  if not baseCodriverTimingSetting then
    return self.defaultCodriverTiming
  end
  return baseCodriverTimingSetting
end

function C:setVehicleTracker(vehicleTracker)
  self.vehicleTracker = vehicleTracker
end

function C:getPacenotes()
  if not self.notebookPath then
    -- log('E', logTag, "expected notebookPath")
    return {}
  end
  return self.notebookPath.pacenotes.sorted
end

function C:getNextPacenote()
  return self:getPacenotes()[self.nextPacenoteIdxForEval]
end

local buildKdTreePoints3D = function(points)
  local kdT = kdTreeP3d.new(#points)
  local stableIdIndex = {}

  -- Preload items: Populates the self.items table
  for i,point in ipairs(points) do
    local stableId = point.metadata and point.metadata.stableId
    if not stableId then
      stableId = point.wp
    end
    if not stableId then
      stableId = "point_" .. tostring(i)
      -- log('E', logTag, "expected stableId for point: " .. i .. ". using '" .. stableId .. "' instead.")
      point.wp = stableId
    end
    kdT:preLoad(stableId, point.pos.x, point.pos.y, point.pos.z)
    stableIdIndex[stableId] = {idx = i, point = point}
  end

  -- Build the tree: creates the tree from the preloaded items, i.e. it populates the self.tree table
  kdT:build()
  return kdT, stableIdIndex
end

-- updateStableIdIndexes: fixes the indices in the stableId->index mapping for all entries
-- with indices greater than or equal to insertIdx.
local updateStableIdIndexes = function(stableIdIndex, insertItem)
  for k, item in pairs(stableIdIndex) do
    if item.idx >= insertItem.idx then
      item.idx = item.idx + 1
    end
  end
end

local function calcClosestLineSegmentKD(points, kdTree, stableIdIndex, queryPoint)
  local stableId, dist = kdTree:findNearest(queryPoint.x, queryPoint.y, queryPoint.z)

  local item = stableIdIndex[stableId]
  if not item then error("expected item for stableId: " .. stableId) end
  -- log('D', logTag, string.format("dist: %s, stableId: %s", dist, stableId))
  -- log('D', logTag, "item: " .. dumps(item))
  local itemIdx = item.idx

  local minDistSq = math.huge
  local fromPoint = nil
  local toPoint = nil

  -- Perform a small search of segments forward and backward from the closest point.
  -- The min search window that makes sense is 1.
  -- IE, check the segment on either side of the closest point.
  -- If the search window is not large enough, the point may be inserted in the wrong place.
  -- If the search window is too large, performance will degrade.
  local searchWindow = 4
  local searchStart = itemIdx - searchWindow
  local searchEnd = (itemIdx + searchWindow) - 1 -- subtract 1 because we add 1 below to get the 2nd point.
  local searchMin = math.max(1, searchStart)
  local searchMax = math.min(#points, searchEnd)
  local bestSegmentIdx = nil

  for i = searchMin, searchMax do
    local p1 = points[i]
    local p2 = points[i+1]
    if p1 and p2 then
      local dist = queryPoint:squaredDistanceToLineSegment(p1.pos, p2.pos)
      -- log('D', logTag, string.format("itemIdx=%d, i=%d, squaredDist: %0.2f", itemIdx, i, dist))
      if dist < minDistSq then
        minDistSq = dist
        fromPoint = p1
        toPoint = p2
        bestSegmentIdx = i
      end
    end
  end

  if bestSegmentIdx == searchMin and searchMin > 1 then
    log('W', logTag, string.format("first searched segment (%d) was closest for queryPoint=%s. consider inspecting the segments in the vicinity to ensure the closest segment is found. potentially increase the search window (currently %d).", searchMin, stableId, searchWindow))
  elseif bestSegmentIdx == searchMax then
    log('W', logTag, string.format("last searched segment (%d) was closest for queryPoint=%s. consider inspecting the segments in the vicinity to ensure the closest segment is found. potentially increase the search window (currently %d).", searchMax, stableId, searchWindow))
  end

  return fromPoint, toPoint, minDistSq
end

-- Core function that encapsulates the common insertion logic.
local function insertRoutePointCore(routePath, kdTree, stableIdIndex, pos, metadata, nodeName)
  -- log('D', logTag, "----- insertPathnode for '" .. nodeName .. "' --------------")
  local fromPoint, toPoint, minDistSq = calcClosestLineSegmentKD(routePath, kdTree, stableIdIndex, pos)
  if fromPoint and toPoint then
    local insertPoint = fromPoint
    -- log('D', logTag, "insertPoint:")
    -- dumpz(insertPoint, 3)
    local insertItem = stableIdIndex[insertPoint.metadata.stableId]
    if not insertItem then
      error("expected insertItem in stableIdIndex for " .. insertPoint.metadata.stableId)
    end
    updateStableIdIndexes(stableIdIndex, insertItem)
    local newPoint = { pos = pos, metadata = metadata }
    table.insert(routePath, insertItem.idx, newPoint)
    -- Update the stableIdIndex with the new node for future insertions.
    stableIdIndex[metadata.stableId] = { idx = insertItem.idx, point = newPoint }
  else
    log('E', logTag, "No closest line segment found for " .. nodeName)
  end
end

-- Wrapper for inserting a race pathnode.
local function insertRacePathnode(racePath, routePathRaceAi, pathnode, kdTreeRaceAi, stableIdIndex, pnTypes)
  local pos = pathnode.pos
  local metadata = {
    racePathnode = pathnode,
    stableId = pathnode.name,
    racePathnodeType = pnTypes[pathnode.name],
    source = "race"
  }
  insertRoutePointCore(routePathRaceAi, kdTreeRaceAi, stableIdIndex, pos, metadata, pathnode.name)
end

-- Wrapper for inserting a pacenote waypoint.
local function insertPacenoteWaypoint(routePathRaceAi, pacenoteWaypoint, kdTreeRaceAi, stableIdIndex)
  local pos = pacenoteWaypoint.pos
  local pacenote = pacenoteWaypoint.pacenote
  local shortWpType = WaypointTypes.shortenWaypointType(pacenoteWaypoint.waypointType)
  local wpKey = string.format("wp%s", shortWpType)
  local metadata = {
    -- pacenoteWaypoint = pacenoteWaypoint,
    [wpKey] = pacenoteWaypoint,
    stableId = string.format("%s[%s]", pacenote.name, shortWpType),
    -- source = string.format("%s[%s]", pacenote.name, shortWpType),
    source = "pacenote_" .. shortWpType
  }
  insertRoutePointCore(routePathRaceAi, kdTreeRaceAi, stableIdIndex, pos, metadata, pacenote.name .. ' ' .. shortWpType)
end

local function getRacePathnodeTypes(racePath)
  local firstPnName = racePath.pathnodes.sorted[1].name
  local lastPnName = racePath.pathnodes.sorted[#racePath.pathnodes.sorted].name

  local pnTypes = {
    [firstPnName] = "first",
    [lastPnName] = "finish",
  }

  local splitCount = 1
  for _, pn in ipairs(racePath.pathnodes.sorted) do
    local pnType = pn.name
    if pn.visible and pnType ~= firstPnName and pnType ~= lastPnName then
      pnTypes[pnType] = "split_"..splitCount
      splitCount = splitCount + 1
    end
  end

  return pnTypes
end

function C:_abortLoadRoute()
  log('E', logTag, "aborting loadRoute")
  self:_reset()
end

function C:pointsForSnaproad(racePath, notebookPath)
  log('D', logTag, 'pointsForSnaproad')
  self:_reset()

  -- the goal is to populate this list with points that will be used to create
  -- the routes.
  local preRoutePoints = {}

  -- Step 1: Get the race setup.
  racePath:autoConfig()
  local _aiPath, aiDetailedPath = racePath:getAiPath(true)

  -- Step 2: Add startPos to the points list.
  local startPosId = racePath.defaultStartPosition
  local startPos = racePath.startPositions.objects[startPosId]
  if not startPos then
    log('E', logTag, "expected startPos")
    -- self:_abortLoadRoute()
    return false
  end
  local metadata = {
    stableId = "startPos",
    source = "race",
    racePathnode = startPos,
    racePathnodeType = "spStart",
  }
  table.insert(preRoutePoints, {pos = startPos.pos, metadata = metadata })

  -- Step 2.5: route between startPos and first pathnode.
  if #aiDetailedPath > 0 then
    local tmpRouteStart = Route()
    tmpRouteStart:setupPathMultiWithMetadata({
      { pos = vec3(preRoutePoints[1].pos), metadata = { stableId = "tmp1" } },
      -- { pos = vec3(racePath.pathnodes.sorted[1].pos) , metadata = { stableId = "tmp2" } }
      { pos = vec3(aiDetailedPath[1].pos) , metadata = { stableId = "tmp2" } }
    })
    for i, point in ipairs(tmpRouteStart.path) do
      table.insert(preRoutePoints, { pos = point.pos , metadata = {stableId = point.wp or "tmp_"..i} })
    end
  end

  -- Step 3: Add aiDetailedPath to the points list.
  for i, point in ipairs(aiDetailedPath) do
    local wp = point.wp
    if not wp then error("expected wp") end
    metadata = {
      stableId = wp,
      source = 'aiDetailedPath'
    }
    local point = {pos = point.pos, metadata = metadata}
    point.noFixed = true
    table.insert(preRoutePoints, point)
  end

  -- Step 4: Add spStopZone to the points list.
  local spStopZone = nil
  for _, sp in ipairs(racePath.startPositions.sorted) do
    if sp.name == "STOP_ZONE" then
      spStopZone = sp
      break
    end
  end
  if not spStopZone then
    log('E', logTag, "expected Race Start Position with name 'STOP_ZONE'")
    -- self:_abortLoadRoute()
    return false
  end
  metadata = {
    stableId = "stopZone",
    source = "race",
    racePathnode = spStopZone,
    racePathnodeType = "spStopZone",
  }

  -- Step 4.5: route between final point and stopZone
  local tmpRoute = Route()
  tmpRoute:setupPathMultiWithMetadata({
    { pos = vec3(preRoutePoints[#preRoutePoints].pos), metadata = { stableId = "tmp1" } },
    { pos = vec3(spStopZone.pos) , metadata = { stableId = "tmp2" } }
  })
  for i, point in ipairs(tmpRoute.path) do
    table.insert(preRoutePoints, { pos = point.pos , metadata = {stableId = point.wp or "tmp_"..i} })
  end

  -- insert the stopZone
  table.insert(preRoutePoints, {pos = spStopZone.pos, metadata = metadata })

  -- self.debugPreRoutePoints = shallowcopy(preRoutePoints)

  -- Step 5: Build kdTree for the points, before inserting any new points, for
  -- fast lookup of closest point/line segment during insertions. And return
  -- stableIdIndex mapping to track the index of the point in the routePathRaceAi
  -- even after insertions.
  local kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 6: Insert race pathnodes into the points list.
  local pnTypes = getRacePathnodeTypes(racePath)
  for i,pathnode in ipairs(racePath.pathnodes.sorted) do
    insertRacePathnode(racePath, preRoutePoints, pathnode, kdTreePreRoute, stableIdIndexPreRoute, pnTypes)
  end

  -- rebuild points index before inserting pacenotes.
  kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 7: Insert pacenote waypoints into the points list.
  -- for _,pn in ipairs(notebookPath.pacenotes.sorted) do
  --   insertPacenoteWaypoint(preRoutePoints, pn:getCornerStartWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
  --   insertPacenoteWaypoint(preRoutePoints, pn:getCornerEndWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
  -- end

  -- Step 8: Clear noFixed points from the points list.
  local routePathRaceAiWithoutFixed = {}
  for i, point in ipairs(preRoutePoints) do
    if not point.noFixed then
      table.insert(routePathRaceAiWithoutFixed, point)
    end
  end

  -- Step 9: Create the routes.
  -- local closeDistSquared = 5
  -- local removeFirst = true
  -- local useMapPathfinding = true
  -- self.route = Route(removeFirst, useMapPathfinding, closeDistSquared)
  -- self.route.callbacks.onPointProcessed = function(point)
  --   -- if point.metadata and point.metadata.pacenoteWaypoint then
  --   --   point.metadata.pacenoteWaypoint:setRoutePoint(point)
  --   -- end
  --   if point.metadata then
  --     if point.metadata.wpCs then
  --       point.metadata.wpCs:setRoutePoint(point)
  --     end
  --     if point.metadata.wpCe then
  --       point.metadata.wpCe:setRoutePoint(point)
  --     end
  --     if point.metadata.racePathnode then
  --       point.metadata.racePathnode:setRoutePoint(point)
  --     end
  --   end
  -- end

  -- self.route.callbacks.onRouteShortened = function(point)
  --   if point.metadata and point.metadata.racePathnode then
  --     -- log('D', logTag, 'onRouteShortened')
  --     -- dumpz(point, 2)
  --   end
  -- end


  -- self.route.callbacks.shouldPointBeFixedForRecalc = function(point)
  --   -- log('D', logTag, "--------------------------------")
  --   if not self.nextPacenoteWpFromRecalc or not self.nextRacePathnodeFromRecalc then
  --     -- log('E', logTag, "no nextPacenoteWpFromRecalc or nextRacePathnodeFromRecalc")
  --     return true
  --   end

  --   -- use the static route point for distance comparison because the *FromRecalc
  --   -- points are set using the staticRoute.
  --   local nextWpRoutePoint = self.nextPacenoteWpFromRecalc:getStaticRoutePoint()
  --   -- if not nextWpRoutePoint then
  --     -- log('E', logTag, "no nextWpRoutePoint")
  --   -- end
  --   local nextRacePathnodeRoutePoint = self.nextRacePathnodeFromRecalc:getStaticRoutePoint()
  --   -- if not nextRacePathnodeRoutePoint then
  --     -- log('E', logTag, "no nextRacePathnodeRoutePoint")
  --   -- end

  --   -- if point.metadata then
  --   --   local rpn = point.metadata.racePathnode
  --   --   local cs = point.metadata.wpCs
  --   --   local ce = point.metadata.wpCe
  --   --   log('D', logTag, string.format("rpn.name='%s' cs.name='%s' ce.name='%s'", rpn and rpn.name or "nil", cs and cs:nameWithPacenote() or "nil", ce and ce:nameWithPacenote() or "nil"))
  --   -- else
  --   --   log('E', logTag, "no point.metadata")
  --   -- end

  --   -- handle non race or pacenote points
  --   local actualNextFixedRoutePoint = nil
  --   if nextWpRoutePoint.distToTarget > nextRacePathnodeRoutePoint.distToTarget then
  --     -- log('D', logTag, "nextWpRoutePoint is farther")
  --     actualNextFixedRoutePoint = nextWpRoutePoint
  --   else
  --     -- log('D', logTag, "nextRacePathnodeRoutePoint is farther")
  --     actualNextFixedRoutePoint = nextRacePathnodeRoutePoint
  --   end

  --   if point.distToTarget > actualNextFixedRoutePoint.distToTarget then
  --     -- log('D', logTag, string.format("false point.distToTarget=%f nextWpRoutePoint.distToTarget=%f nextRacePathnodeRoutePoint.distToTarget=%f", point.distToTarget, nextWpRoutePoint.distToTarget, nextRacePathnodeRoutePoint.distToTarget))
  --     return false
  --   else
  --     -- local pointHasWpCs = point.metadata and point.metadata.wpCs
  --     -- local pointHasWpCe = point.metadata and point.metadata.wpCe
  --     -- local pointHasRacePathnode = point.metadata and point.metadata.racePathnode
  --     -- return pointHasWpCs or pointHasWpCe or pointHasRacePathnode
  --   end

  --   -- shouldnt hit this?
  --   -- log('D', logTag, "final true means fixed")
  --   return true
  -- end

  -- -- local removeFirst = true
  -- -- if recordedDriveline then
  -- --   local useMapPathfinding = false
  -- --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- -- else
  -- --   local useMapPathfinding = true
  -- --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- -- end
  -- self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)

  --
  -- Step 10: Build static route
  ---

  local closeDistSquared = 5
  local removeFirst = false
  local useMapPathfinding = true
  local routeStatic = Route(removeFirst, useMapPathfinding, closeDistSquared)
  -- routeStatic.callbacks.onPointProcessed = function(point)
  --   -- if point.metadata and point.metadata.racePathnode then
  --   --   point.metadata.racePathnode:setStaticRoutePoint(point)
  --   -- end

  --   if point.metadata then
  --     if point.metadata.wpCs then
  --       point.metadata.wpCs:setStaticRoutePoint(point)
  --     end
  --     if point.metadata.wpCe then
  --       point.metadata.wpCe:setStaticRoutePoint(point)
  --     end
  --     if point.metadata.racePathnode then
  --       point.metadata.racePathnode:setStaticRoutePoint(point)
  --     end
  --   end
  -- end

  routeStatic.callbacks.onMetadataMerge = function(pointI, last, cur)
    local maxI = 10

    local lastPointRacePn = nil
    local lastPointWpCs = nil
    local lastPointWpCe = nil
    local lastMetadataHasRacingData = false
    if last.metadata then
      lastPointRacePn = last.metadata and last.metadata.racePathnode
      lastPointWpCs = last.metadata and last.metadata.wpCs
      lastPointWpCe = last.metadata and last.metadata.wpCe
      lastMetadataHasRacingData = lastPointRacePn or lastPointWpCs or lastPointWpCe
    end

    local curPointRacePn = nil
    local curPointWpCs = nil
    local curPointWpCe = nil
    local curMetadataHasRacingData = false
    if cur.metadata then
      curPointRacePn = cur.metadata and cur.metadata.racePathnode
      curPointWpCs = cur.metadata and cur.metadata.wpCs
      curPointWpCe = cur.metadata and cur.metadata.wpCe
      curMetadataHasRacingData = curPointRacePn or curPointWpCs or curPointWpCe
    end

    local newPoint = nil
    local newMetadata = nil

    if lastMetadataHasRacingData and curMetadataHasRacingData then
      -- reject the merge if both points have racing data
      return nil, nil
    elseif lastMetadataHasRacingData then
      -- prioritize last point for the merge
      newPoint = last
      newMetadata = last.metadata
    elseif curMetadataHasRacingData then
      -- prioritize cur point for the merge
      newPoint = cur
      newMetadata = cur.metadata
    else
      newPoint = last
      newMetadata = last.metadata
    end

    return newPoint, newMetadata
  end

  routeStatic:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)
  -- Step 11: Build kdTree for the static route
  -- self.kdTreeRouteStatic, self.stableIdIndexRouteStatic = buildKdTreePoints3D(self.routeStatic.path)

  -- Step 12: calculate cached lengths for pacenotes
  -- for _,pn in ipairs(self:getPacenotes()) do
  --   local wpCS = pn:getCornerStartWaypoint()
  --   local wpCE = pn:getCornerEndWaypoint()

  --   if not wpCS or not wpCE then
  --     log('E', logTag, "No wpCS or wpCE found for " .. pn.name)
  --   end

  --   local rpCS = wpCS:getRoutePoint()
  --   local rpCE = wpCE:getRoutePoint()

  --   if not rpCS or not rpCE then
  --     log('E', logTag, "No route point found for " .. pn.name..". rpCS="..tostring(not not rpCS).." rpCE="..tostring(not not rpCE))
  --     pn:setCachedLength(0.0)
  --   else
  --     local len = rpCS.distToTarget - rpCE.distToTarget
  --     pn:setCachedLength(len)
  --   end
  -- end

  -- Finally done!

  log('I', logTag, "loaded DrivelineRoute with type=snaproad and " .. #routeStatic.path .. " points")
  -- self.loaded = true
  return routeStatic
end -- end of pointsForSnaproad

function C:_loadRouteCoreRecce(racePath, notebookPath, recordedDriveline)
  -- log('D', logTag, '_loadRouteCoreRecce')
  self:_reset()

  self.racePath = racePath
  self.notebookPath = notebookPath
  self.rawRecordedDriveline = recordedDriveline

  if recordedDriveline then
    -- log('D', logTag, "downsampling recordedDriveline")
    recordedDriveline = self.rawRecordedDriveline:downsample()
    if not recordedDriveline then
      self:_abortLoadRoute()
      return false
    end
    self.recordedDriveline = recordedDriveline
  end

  -- the goal is to populate this list with points that will be used to create
  -- the routes.
  local preRoutePoints = {}

  -- Step 1: Get the race setup.
  racePath:autoConfig()
  local _aiPath, aiDetailedPath = nil, nil

  -- Step 2: Add aiDetailedPath OR recce driveline points to the points list.
  local step3Source = nil

  local step3Points = recordedDriveline.points
  for i, point in ipairs(step3Points) do
    local wp = string.format("driveline_%d", i)
    if not wp then error("expected wp") end
    local metadata = {
      stableId = wp,
      source = step3Source
    }
    local point = {pos = point.pos, wp = wp, metadata = metadata}
    table.insert(preRoutePoints, point)
  end


  -- Step 2: Build kdTree for the points, before inserting any new points, for
  -- fast lookup of closest point/line segment during insertions. And return
  -- stableIdIndex mapping to track the index of the point in the routePathRaceAi
  -- even after insertions.
  local kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 2: Add startPos to the points list.
  local startPosId = racePath.defaultStartPosition
  local startPos = racePath.startPositions.objects[startPosId]
  if not startPos then
    log('E', logTag, "expected startPos")
    self:_abortLoadRoute()
    return false
  end
  local metadata = {
    stableId = "startPos",
    source = "race",
    racePathnode = startPos,
    racePathnodeType = "spStart",
  }
  insertRoutePointCore(preRoutePoints, kdTreePreRoute, stableIdIndexPreRoute, startPos.pos, metadata, "startPos")

  -- Step 4: Add spStopZone to the points list.
  local spStopZone = nil
  for _, sp in ipairs(racePath.startPositions.sorted) do
    if sp.name == "STOP_ZONE" then
      spStopZone = sp
      break
    end
  end
  if not spStopZone then
    log('E', logTag, "expected Race Start Position with name 'STOP_ZONE'")
    self:_abortLoadRoute()
    return false
  end
  metadata = {
    stableId = "stopZone",
    source = "race",
    racePathnode = spStopZone,
    racePathnodeType = "spStopZone",
  }
  insertRoutePointCore(preRoutePoints, kdTreePreRoute, stableIdIndexPreRoute, spStopZone.pos, metadata, "stopZone")

  -- self.debugPreRoutePoints = preRoutePoints

  -- Step 6: Insert race pathnodes into the points list.
  local pnTypes = getRacePathnodeTypes(racePath)
  for i,pathnode in ipairs(racePath.pathnodes.sorted) do
    insertRacePathnode(racePath, preRoutePoints, pathnode, kdTreePreRoute, stableIdIndexPreRoute, pnTypes)
  end

  -- re index the points before inserting pacenotes.
  kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 7: Insert pacenote waypoints into the points list.
  for _,pn in ipairs(notebookPath.pacenotes.sorted) do
    insertPacenoteWaypoint(preRoutePoints, pn:getCornerStartWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
    insertPacenoteWaypoint(preRoutePoints, pn:getCornerEndWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
  end

  -- Step 8: Clear noFixed points from the points list.
  local routePathRaceAiWithoutFixed = {}
  for i, point in ipairs(preRoutePoints) do
    if not point.noFixed then
      table.insert(routePathRaceAiWithoutFixed, point)
    end
  end

  -- Step 8.5: Remove leading points before the start position
  local startIndex = nil
  for i, point in ipairs(routePathRaceAiWithoutFixed) do
    if point.metadata and point.metadata.racePathnodeType == 'spStart' then
      startIndex = i
      break
    end
  end

  if startIndex and startIndex > 1 then
    local newRoutePoints = {}
    for i = startIndex, #routePathRaceAiWithoutFixed do
      table.insert(newRoutePoints, routePathRaceAiWithoutFixed[i])
    end
    routePathRaceAiWithoutFixed = newRoutePoints
  end

  -- Step 8.6: Remove trailing points after the stop position
  local stopIndex = nil
  for i, point in ipairs(routePathRaceAiWithoutFixed) do
    if point.metadata and point.metadata.racePathnodeType == 'spStopZone' then
      stopIndex = i
      break
    end
  end

  if stopIndex and stopIndex < #routePathRaceAiWithoutFixed then
    local newRoutePoints = {}
    for i = 1, stopIndex do
      table.insert(newRoutePoints, routePathRaceAiWithoutFixed[i])
    end
    routePathRaceAiWithoutFixed = newRoutePoints
  end

  self.finalPreRouteInput = {}

  for i, point in ipairs(routePathRaceAiWithoutFixed) do
    table.insert(self.finalPreRouteInput, {
      i = i,
      pos = point.pos,
      metadata = point.metadata,
      wp = point.wp,
    })
  end

  -- Step 9: Create the routes.
  local closeDistSquared = 5
  local removeFirst = true
  local useMapPathfinding = true
  if recordedDriveline then
    useMapPathfinding = false
  end
  self.route = Route(removeFirst, useMapPathfinding, closeDistSquared, 'DynamicRoute')
  self.route.callbacks.onPointProcessed = function(point)
    -- if point.metadata and point.metadata.pacenoteWaypoint then
    --   point.metadata.pacenoteWaypoint:setRoutePoint(point)
    -- end
    if point.metadata then
      if point.metadata.wpCs then
        point.metadata.wpCs:setRoutePoint(point)
      end
      if point.metadata.wpCe then
        point.metadata.wpCe:setRoutePoint(point)
      end
      if point.metadata.racePathnode then
        point.metadata.racePathnode:setRoutePoint(point)
      end
    end
  end

  self.route.callbacks.onRouteShortened = function(point)
    if point.metadata and point.metadata.racePathnode then
      -- log('D', logTag, 'onRouteShortened')
      -- dumpz(point, 2)
    end
  end

  self.route.callbacks.onMetadataMerge = function(pointI, last, cur)
    -- log('D', '', "--------------------------------")
    -- log('D', '', "onMetadataMerge")
    local maxI = 10

    -- if pointI < maxI then
    --   log('I', '', "last:")
    --   dumpz(last, 2)
    --   log('I', '', "cur:")
    --   dumpz(cur, 2)
    -- end

    local lastPointRacePn = nil
    local lastPointWpCs = nil
    local lastPointWpCe = nil
    local lastMetadataHasRacingData = false
    if last.metadata then
      lastPointRacePn = last.metadata and last.metadata.racePathnode
      lastPointWpCs = last.metadata and last.metadata.wpCs
      lastPointWpCe = last.metadata and last.metadata.wpCe
      lastMetadataHasRacingData = lastPointRacePn or lastPointWpCs or lastPointWpCe
    end

    local curPointRacePn = nil
    local curPointWpCs = nil
    local curPointWpCe = nil
    local curMetadataHasRacingData = false
    if cur.metadata then
      curPointRacePn = cur.metadata and cur.metadata.racePathnode
      curPointWpCs = cur.metadata and cur.metadata.wpCs
      curPointWpCe = cur.metadata and cur.metadata.wpCe
      curMetadataHasRacingData = curPointRacePn or curPointWpCs or curPointWpCe
    end

    local newPoint = nil
    local newMetadata = nil

    if lastMetadataHasRacingData and curMetadataHasRacingData then
      -- log('D', '', string.format("last.stableId=%s cur.stableId=%s", last.metadata.stableId, cur.metadata.stableId))
      -- reject the merge if both points have racing data
      return nil, nil

      -- if lastPointRacePn and curPointRacePn then
      --   log('E', '', "two points cant be merged, both have racePathnode")
      --   error("two points cant be merged, both have racePathnode")
      -- end
      -- if lastPointWpCs and curPointWpCs then
      --   log('E', '', "two points cant be merged, both have wpCs")
      --   error("two points cant be merged, both have wpCs")
      -- end
      -- if lastPointWpCe and curPointWpCe then
      --   log('E', '', "two points cant be merged, both have wpCe")
      --   error("two points cant be merged, both have wpCe")
      -- end
    elseif lastMetadataHasRacingData then
      -- prioritize last point for the merge
      newPoint = last
      newMetadata = last.metadata
    elseif curMetadataHasRacingData then
      -- prioritize cur point for the merge
      newPoint = cur
      newMetadata = cur.metadata
    else
      newPoint = last
      newMetadata = last.metadata
    end

    -- if pointI < maxI then
      -- log('I', '', "newPoint:")
      -- dumpz(newPoint, 2)
      -- log('I', '', "newMetadata:")
      -- dumpz(newMetadata, 2)
    -- end

    return newPoint, newMetadata
  end

  self.route.callbacks.shouldPointBeFixedForRecalc = function(point)
    -- log('D', logTag, "--------------------------------")
    if not self.nextPacenoteWpFromRecalc or not self.nextRacePathnodeFromRecalc then
      -- log('E', logTag, "no nextPacenoteWpFromRecalc or nextRacePathnodeFromRecalc")
      return true
    end

    -- use the static route point for distance comparison because the *FromRecalc
    -- points are set using the staticRoute.
    local nextWpRoutePoint = self.nextPacenoteWpFromRecalc:getStaticRoutePoint()
    -- if not nextWpRoutePoint then
      -- log('E', logTag, "no nextWpRoutePoint")
    -- end
    local nextRacePathnodeRoutePoint = self.nextRacePathnodeFromRecalc:getStaticRoutePoint()
    -- if not nextRacePathnodeRoutePoint then
      -- log('E', logTag, "no nextRacePathnodeRoutePoint")
    -- end

    -- if point.metadata then
    --   local rpn = point.metadata.racePathnode
    --   local cs = point.metadata.wpCs
    --   local ce = point.metadata.wpCe
    --   log('D', logTag, string.format("rpn.name='%s' cs.name='%s' ce.name='%s'", rpn and rpn.name or "nil", cs and cs:nameWithPacenote() or "nil", ce and ce:nameWithPacenote() or "nil"))
    -- else
    --   log('E', logTag, "no point.metadata")
    -- end

    -- handle non race or pacenote points
    local actualNextFixedRoutePoint = nil
    if nextWpRoutePoint.distToTarget > nextRacePathnodeRoutePoint.distToTarget then
      -- log('D', logTag, "nextWpRoutePoint is farther")
      actualNextFixedRoutePoint = nextWpRoutePoint
    else
      -- log('D', logTag, "nextRacePathnodeRoutePoint is farther")
      actualNextFixedRoutePoint = nextRacePathnodeRoutePoint
    end

    if point.distToTarget > actualNextFixedRoutePoint.distToTarget then
      -- log('D', logTag, string.format("false point.distToTarget=%f nextWpRoutePoint.distToTarget=%f nextRacePathnodeRoutePoint.distToTarget=%f", point.distToTarget, nextWpRoutePoint.distToTarget, nextRacePathnodeRoutePoint.distToTarget))
      return false
    else
      -- local pointHasWpCs = point.metadata and point.metadata.wpCs
      -- local pointHasWpCe = point.metadata and point.metadata.wpCe
      -- local pointHasRacePathnode = point.metadata and point.metadata.racePathnode
      -- return pointHasWpCs or pointHasWpCe or pointHasRacePathnode
    end

    -- shouldnt hit this?
    -- log('D', logTag, "final true means fixed")
    return true
  end

  -- local removeFirst = true
  -- if recordedDriveline then
  --   local useMapPathfinding = false
  --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- else
  --   local useMapPathfinding = true
  --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- end
  self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)

  --
  -- Step 10: Build static route
  ---

  removeFirst = false
  useMapPathfinding = true
  if recordedDriveline then
    useMapPathfinding = false
  end
  self.routeStatic = Route(removeFirst, useMapPathfinding, closeDistSquared, 'StaticRoute')
  self.routeStatic.callbacks.onPointProcessed = function(point)
    -- if point.metadata and point.metadata.racePathnode then
    --   point.metadata.racePathnode:setStaticRoutePoint(point)
    -- end

    if point.metadata then
      if point.metadata.wpCs then
        point.metadata.wpCs:setStaticRoutePoint(point)
      end
      if point.metadata.wpCe then
        point.metadata.wpCe:setStaticRoutePoint(point)
      end
      if point.metadata.racePathnode then
        point.metadata.racePathnode:setStaticRoutePoint(point)
      end
    end
  end

  self.routeStatic.callbacks.onMetadataMerge = self.route.callbacks.onMetadataMerge

  -- local removeFirst = false
  -- if recordedDriveline then
    -- local useMapPathfinding = false
    self.routeStatic:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)
  -- else
    -- local useMapPathfinding = true
    -- self.routeStatic:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- end

  -- Step 11: Build kdTree for the static route
  self.kdTreeRouteStatic, self.stableIdIndexRouteStatic = buildKdTreePoints3D(self.routeStatic.path)

  -- Step 12: calculate cached lengths for pacenotes
  for _,pn in ipairs(self:getPacenotes()) do
    local wpCS = pn:getCornerStartWaypoint()
    local wpCE = pn:getCornerEndWaypoint()

    if not wpCS or not wpCE then
      log('E', logTag, "No wpCS or wpCE found for " .. pn.name)
    end

    local rpCS = wpCS:getRoutePoint()
    local rpCE = wpCE:getRoutePoint()

    if not rpCS or not rpCE then
      log('E', logTag, "No route point found for " .. pn.name..". rpCS="..tostring(not not rpCS).." rpCE="..tostring(not not rpCE))
      pn:setCachedLength(0.0)
    else
      local len = rpCS.distToTarget - rpCE.distToTarget
      pn:setCachedLength(len)
    end
  end


  -- check if the finishline offset distance can be calculated.
  -- if not, that probably means the recce driveline is not long enough to
  -- reach the finish line.
  local finishLineDistOffset = self:getFinishLineDistOffset()
  if not finishLineDistOffset then
    log('E', logTag, "finishLineDistOffset is nil, probably means the recce driveline is not long enough to reach the finish line")
    return false
  end

  -- Finally done!

  log('I', logTag, "loaded DrivelineRoute with type=recce and " .. #self.route.path .. " points")
  self.loaded = true
  return true
end -- end of loadRoute

function C:_loadRouteCore(racePath, notebookPath)
  -- log('D', logTag, '_loadRouteCore')
  self:_reset()

  self.racePath = racePath
  self.notebookPath = notebookPath

  -- the goal is to populate this list with points that will be used to create
  -- the routes.
  local preRoutePoints = {}

  -- Step 1: Get the race setup.
  racePath:autoConfig()
  local _aiPath, aiDetailedPath = racePath:getAiPath(true)

  -- Step 2: Add startPos to the points list.
  local startPosId = racePath.defaultStartPosition
  local startPos = racePath.startPositions.objects[startPosId]
  if not startPos then
    log('E', logTag, "expected startPos")
    self:_abortLoadRoute()
    return false
  end
  local metadata = {
    stableId = "startPos",
    source = "race",
    racePathnode = startPos,
    racePathnodeType = "spStart",
  }
  table.insert(preRoutePoints, {pos = startPos.pos, metadata = metadata })

  -- Step 2.5: route between startPos and first pathnode.
  if #aiDetailedPath > 0 then
    local tmpRouteStart = Route()
    tmpRouteStart:setupPathMultiWithMetadata({
      { pos = vec3(preRoutePoints[1].pos), metadata = { stableId = "tmp1" } },
      -- { pos = vec3(racePath.pathnodes.sorted[1].pos) , metadata = { stableId = "tmp2" } }
      { pos = vec3(aiDetailedPath[1].pos) , metadata = { stableId = "tmp2" } }
    })
    for i, point in ipairs(tmpRouteStart.path) do
      table.insert(preRoutePoints, { pos = point.pos , metadata = {stableId = point.wp or "tmp_"..i} })
    end
  end

  -- Step 3: Add aiDetailedPath to the points list.

  for i, point in ipairs(aiDetailedPath) do
    local wp = point.wp
    if not wp then error("expected wp") end
    metadata = {
      stableId = wp,
      source = 'aiDetailedPath'
    }
    local point = {pos = point.pos, metadata = metadata}
    point.noFixed = true
    table.insert(preRoutePoints, point)
  end

  -- Step 4: Add spStopZone to the points list.
  local spStopZone = nil
  for _, sp in ipairs(racePath.startPositions.sorted) do
    if sp.name == "STOP_ZONE" then
      spStopZone = sp
      break
    end
  end
  if not spStopZone then
    log('E', logTag, "expected Race Start Position with name 'STOP_ZONE'")
    self:_abortLoadRoute()
    return false
  end
  metadata = {
    stableId = "stopZone",
    source = "race",
    racePathnode = spStopZone,
    racePathnodeType = "spStopZone",
  }

  -- Step 4.5: route between final point and stopZone
  local tmpRoute = Route()
  tmpRoute:setupPathMultiWithMetadata({
    { pos = vec3(preRoutePoints[#preRoutePoints].pos), metadata = { stableId = "tmp1" } },
    { pos = vec3(spStopZone.pos) , metadata = { stableId = "tmp2" } }
  })
  for i, point in ipairs(tmpRoute.path) do
    table.insert(preRoutePoints, { pos = point.pos , metadata = {stableId = point.wp or "tmp_"..i} })
  end

  -- insert the stopZone
  table.insert(preRoutePoints, {pos = spStopZone.pos, metadata = metadata })

  self.debugPreRoutePoints = shallowcopy(preRoutePoints)

  -- Step 5: Build kdTree for the points, before inserting any new points, for
  -- fast lookup of closest point/line segment during insertions. And return
  -- stableIdIndex mapping to track the index of the point in the routePathRaceAi
  -- even after insertions.
  local kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 6: Insert race pathnodes into the points list.
  local pnTypes = getRacePathnodeTypes(racePath)
  for i,pathnode in ipairs(racePath.pathnodes.sorted) do
    insertRacePathnode(racePath, preRoutePoints, pathnode, kdTreePreRoute, stableIdIndexPreRoute, pnTypes)
  end

  -- rebuild points index before inserting pacenotes.
  kdTreePreRoute, stableIdIndexPreRoute = buildKdTreePoints3D(preRoutePoints)

  -- Step 7: Insert pacenote waypoints into the points list.
  for _,pn in ipairs(notebookPath.pacenotes.sorted) do
    insertPacenoteWaypoint(preRoutePoints, pn:getCornerStartWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
    insertPacenoteWaypoint(preRoutePoints, pn:getCornerEndWaypoint(), kdTreePreRoute, stableIdIndexPreRoute)
  end

  -- Step 8: Clear noFixed points from the points list.
  local routePathRaceAiWithoutFixed = {}
  for i, point in ipairs(preRoutePoints) do
    if not point.noFixed then
      table.insert(routePathRaceAiWithoutFixed, point)
    end
  end

  -- Step 9: Create the routes.
  local closeDistSquared = 5
  local removeFirst = true
  local useMapPathfinding = true
  self.route = Route(removeFirst, useMapPathfinding, closeDistSquared)
  self.route.callbacks.onPointProcessed = function(point)
    -- if point.metadata and point.metadata.pacenoteWaypoint then
    --   point.metadata.pacenoteWaypoint:setRoutePoint(point)
    -- end
    if point.metadata then
      if point.metadata.wpCs then
        point.metadata.wpCs:setRoutePoint(point)
      end
      if point.metadata.wpCe then
        point.metadata.wpCe:setRoutePoint(point)
      end
      if point.metadata.racePathnode then
        point.metadata.racePathnode:setRoutePoint(point)
      end
    end
  end

  self.route.callbacks.onRouteShortened = function(point)
    if point.metadata and point.metadata.racePathnode then
      -- log('D', logTag, 'onRouteShortened')
      -- dumpz(point, 2)
    end
  end

  self.route.callbacks.onMetadataMerge = function(pointI, last, cur)
    -- log('D', '', "--------------------------------")
    -- log('D', '', "onMetadataMerge")
    local maxI = 10

    -- if pointI < maxI then
    --   log('I', '', "last:")
    --   dumpz(last, 2)
    --   log('I', '', "cur:")
    --   dumpz(cur, 2)
    -- end

    local lastPointRacePn = nil
    local lastPointWpCs = nil
    local lastPointWpCe = nil
    local lastMetadataHasRacingData = false
    if last.metadata then
      lastPointRacePn = last.metadata and last.metadata.racePathnode
      lastPointWpCs = last.metadata and last.metadata.wpCs
      lastPointWpCe = last.metadata and last.metadata.wpCe
      lastMetadataHasRacingData = lastPointRacePn or lastPointWpCs or lastPointWpCe
    end

    local curPointRacePn = nil
    local curPointWpCs = nil
    local curPointWpCe = nil
    local curMetadataHasRacingData = false
    if cur.metadata then
      curPointRacePn = cur.metadata and cur.metadata.racePathnode
      curPointWpCs = cur.metadata and cur.metadata.wpCs
      curPointWpCe = cur.metadata and cur.metadata.wpCe
      curMetadataHasRacingData = curPointRacePn or curPointWpCs or curPointWpCe
    end

    local newPoint = nil
    local newMetadata = nil

    if lastMetadataHasRacingData and curMetadataHasRacingData then
      -- reject the merge if both points have racing data
      return nil, nil

      -- if lastPointRacePn and curPointRacePn then
      --   log('E', '', "two points cant be merged, both have racePathnode")
      --   error("two points cant be merged, both have racePathnode")
      -- end
      -- if lastPointWpCs and curPointWpCs then
      --   log('E', '', "two points cant be merged, both have wpCs")
      --   error("two points cant be merged, both have wpCs")
      -- end
      -- if lastPointWpCe and curPointWpCe then
      --   log('E', '', "two points cant be merged, both have wpCe")
      --   error("two points cant be merged, both have wpCe")
      -- end
    elseif lastMetadataHasRacingData then
      -- prioritize last point for the merge
      newPoint = last
      newMetadata = last.metadata
    elseif curMetadataHasRacingData then
      -- prioritize cur point for the merge
      newPoint = cur
      newMetadata = cur.metadata
    else
      newPoint = last
      newMetadata = last.metadata
    end

    -- if pointI < maxI then
      -- log('I', '', "newPoint:")
      -- dumpz(newPoint, 2)
      -- log('I', '', "newMetadata:")
      -- dumpz(newMetadata, 2)
    -- end

    return newPoint, newMetadata
  end

  self.route.callbacks.shouldPointBeFixedForRecalc = function(point)
    -- log('D', logTag, "--------------------------------")
    if not self.nextPacenoteWpFromRecalc or not self.nextRacePathnodeFromRecalc then
      -- log('E', logTag, "no nextPacenoteWpFromRecalc or nextRacePathnodeFromRecalc")
      return true
    end

    -- use the static route point for distance comparison because the *FromRecalc
    -- points are set using the staticRoute.
    local nextWpRoutePoint = self.nextPacenoteWpFromRecalc:getStaticRoutePoint()
    -- if not nextWpRoutePoint then
      -- log('E', logTag, "no nextWpRoutePoint")
    -- end
    local nextRacePathnodeRoutePoint = self.nextRacePathnodeFromRecalc:getStaticRoutePoint()
    -- if not nextRacePathnodeRoutePoint then
      -- log('E', logTag, "no nextRacePathnodeRoutePoint")
    -- end

    -- if point.metadata then
    --   local rpn = point.metadata.racePathnode
    --   local cs = point.metadata.wpCs
    --   local ce = point.metadata.wpCe
    --   log('D', logTag, string.format("rpn.name='%s' cs.name='%s' ce.name='%s'", rpn and rpn.name or "nil", cs and cs:nameWithPacenote() or "nil", ce and ce:nameWithPacenote() or "nil"))
    -- else
    --   log('E', logTag, "no point.metadata")
    -- end

    -- handle non race or pacenote points
    local actualNextFixedRoutePoint = nil
    if nextWpRoutePoint.distToTarget > nextRacePathnodeRoutePoint.distToTarget then
      -- log('D', logTag, "nextWpRoutePoint is farther")
      actualNextFixedRoutePoint = nextWpRoutePoint
    else
      -- log('D', logTag, "nextRacePathnodeRoutePoint is farther")
      actualNextFixedRoutePoint = nextRacePathnodeRoutePoint
    end

    if point.distToTarget > actualNextFixedRoutePoint.distToTarget then
      -- log('D', logTag, string.format("false point.distToTarget=%f nextWpRoutePoint.distToTarget=%f nextRacePathnodeRoutePoint.distToTarget=%f", point.distToTarget, nextWpRoutePoint.distToTarget, nextRacePathnodeRoutePoint.distToTarget))
      return false
    else
      -- local pointHasWpCs = point.metadata and point.metadata.wpCs
      -- local pointHasWpCe = point.metadata and point.metadata.wpCe
      -- local pointHasRacePathnode = point.metadata and point.metadata.racePathnode
      -- return pointHasWpCs or pointHasWpCe or pointHasRacePathnode
    end

    -- shouldnt hit this?
    -- log('D', logTag, "final true means fixed")
    return true
  end

  -- local removeFirst = true
  -- if recordedDriveline then
  --   local useMapPathfinding = false
  --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- else
  --   local useMapPathfinding = true
  --   self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- end
  self.route:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)

  --
  -- Step 10: Build static route
  ---

  removeFirst = false
  useMapPathfinding = true
  self.routeStatic = Route(removeFirst, useMapPathfinding, closeDistSquared)
  self.routeStatic.callbacks.onPointProcessed = function(point)
    -- if point.metadata and point.metadata.racePathnode then
    --   point.metadata.racePathnode:setStaticRoutePoint(point)
    -- end

    if point.metadata then
      if point.metadata.wpCs then
        point.metadata.wpCs:setStaticRoutePoint(point)
      end
      if point.metadata.wpCe then
        point.metadata.wpCe:setStaticRoutePoint(point)
      end
      if point.metadata.racePathnode then
        point.metadata.racePathnode:setStaticRoutePoint(point)
      end
    end
  end

  self.routeStatic.callbacks.onMetadataMerge = self.route.callbacks.onMetadataMerge

  -- local removeFirst = false
  -- if recordedDriveline then
    -- local useMapPathfinding = false
    self.routeStatic:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed)
  -- else
    -- local useMapPathfinding = true
    -- self.routeStatic:setupPathMultiWithMetadata(routePathRaceAiWithoutFixed, removeFirst, useMapPathfinding)
  -- end

  -- Step 11: Build kdTree for the static route
  self.kdTreeRouteStatic, self.stableIdIndexRouteStatic = buildKdTreePoints3D(self.routeStatic.path)

  -- Step 12: calculate cached lengths for pacenotes
  for _,pn in ipairs(self:getPacenotes()) do
    local wpCS = pn:getCornerStartWaypoint()
    local wpCE = pn:getCornerEndWaypoint()

    if not wpCS or not wpCE then
      log('E', logTag, "No wpCS or wpCE found for " .. pn.name)
    end

    local rpCS = wpCS:getRoutePoint()
    local rpCE = wpCE:getRoutePoint()

    if not rpCS or not rpCE then
      log('E', logTag, "No route point found for " .. pn.name..". rpCS="..tostring(not not rpCS).." rpCE="..tostring(not not rpCE))
      pn:setCachedLength(0.0)
    else
      local len = rpCS.distToTarget - rpCE.distToTarget
      pn:setCachedLength(len)
    end
  end

  -- Finally done!

  log('I', logTag, "loaded DrivelineRoute with type=route and " .. #self.route.path .. " points")
  self.loaded = true
  return true
end -- end of loadRoute

function C:loadRoute(racePath, notebookPath)
  -- log('D', logTag, 'loadRoute')
  return self:_loadRouteCore(racePath, notebookPath)
end

function C:loadRouteFromRecordedDriveline(racePath, notebookPath, driveline)
  -- log('D', logTag, 'loadRouteFromRecordedDriveline')
  return self:_loadRouteCoreRecce(racePath, notebookPath, driveline)
end

function C:getNearestRoutePoint(pos)
  local fromPoint, toPoint, minDistSq = calcClosestLineSegmentKD(self.routeStatic.path, self.kdTreeRouteStatic, self.stableIdIndexRouteStatic, pos)

  local xnorm = pos:xnormOnLine(fromPoint.pos, toPoint.pos)
  if xnorm > 0 then
    local newpos = lerp(fromPoint.pos, toPoint.pos, xnorm)
    return newpos
  end

  return nil
end

function C:recalculate()
  -- log('D', logTag, 'recalculate')
  local pos = self:getPosition()
  self.lastRecalculateVehiclePos = pos

  local stableId, dist = self.kdTreeRouteStatic:findNearest(pos.x, pos.y, pos.z)
  local item = self.stableIdIndexRouteStatic[stableId]
  local point = item.point
  self.debugNearestRecalcPoint = point

  local nextWp = nil

  -- log('D', logTag, "searching for next pacenote waypoint")
  for i = item.idx, #self.routeStatic.path do
    local pathPoint = self.routeStatic.path[i]
    -- log('D', logTag, string.format("static route point: wp=%s stableId=%s", pathPoint.wp or "nil", pathPoint.metadata and pathPoint.metadata.stableId or "nil"))
    local metadata = pathPoint.metadata
    if metadata then
      if not nextWp then
        if metadata.wpCs then
          nextWp = metadata.wpCs
          break
        -- elseif metadata.wpCe then
        --   nextWp = metadata.wpCe
        --   break
        end
      end
    end
  end
  self.nextPacenoteWpFromRecalc = nextWp

  -- log('D', logTag, "searching for next race pathnode")
  local nextRacePathnode = nil

  for i = item.idx, #self.routeStatic.path do
    local pathPoint = self.routeStatic.path[i]
    -- log('D', logTag, string.format("static route point: wp=%s stableId=%s", pathPoint.wp or "nil", pathPoint.metadata and pathPoint.metadata.stableId or "nil"))
    local metadata = pathPoint.metadata
    if metadata then
      if not nextRacePathnode then
        if metadata.racePathnode then
          nextRacePathnode = metadata.racePathnode
          break
        end
      end
    end
  end
  self.nextRacePathnodeFromRecalc = nextRacePathnode

  -- first clear all events
  for i,pacenote in ipairs(self:getPacenotes()) do
    -- log('D', logTag, string.format('clearing events for %s', pacenote.name))
    self.events[pacenote.id] = nil
  end

  -- clear events for pacenotes after the next pacenote waypoint
  if self.nextPacenoteWpFromRecalc then
    local nextPnId = self.nextPacenoteWpFromRecalc.pacenote.id
    self.nextPacenoteIdxForEval = nil
    -- log('D', logTag, string.format('nextPnId id=%d name=%s', nextPnId, self.nextPacenoteWpFromRecalc.pacenote.name))
    for i, pacenote in ipairs(self:getPacenotes()) do
      if pacenote.id == nextPnId then
        if not self.nextPacenoteIdxForEval then
          self.nextPacenoteIdxForEval = i
          break
        end
      else
        local event = self:getPacenoteEvent(pacenote)
        if event then
          event.disabledByRecalc = true
          self.events[pacenote.id] = event
          -- log('D', logTag, string.format('disabled event for %s', pacenote.name))
        else
          log('W', logTag, string.format('no event found for %s', pacenote.name))
        end
      end
    end
  end

  self.route:recalculateRouteWithOriginalPositions(pos)

  -- if self.nextPacenoteWpFromRecalc then
  --   log('D', logTag, string.format('nextPacenoteWp: %s', self.nextPacenoteWpFromRecalc:nameWithPacenote()))
  -- end

  -- if self.nextRacePathnodeFromRecalc then
  --   log('D', logTag, string.format('nextRacePathnode: %s', self.nextRacePathnodeFromRecalc.name))
  -- end

  return true
end

local function calcSpeedMph(speedMs)
  return speedMs * 2.23694
end

function C:getPacenoteEvent(pacenote)
  -- Add more stops to offsets or percentages to allow for more trigger points.
  -- negative numbers mean before the point, positive means after.
  -- TODO pre-allocate this

  local event = self.events[pacenote.id]
  if not event then
    -- log('D', logTag, string.format('creating event for %s', pacenote.name))
    event = {
      disabledByRecalc = false,
      csDynamicHit = false,
      csImmediateHit = false,
      csStaticHit = false,
      csMeterOffsetHit = {
        -- [-10] = false,
      },
      cornerPercentHit = {
        -- [0.25] = false,
        [0.5] = false, -- halfway between corner start and corner end
        -- [0.75] = false,
      },
      ceStaticHit = false,
      ceMeterOffsetHit = {
        [-5] = false, -- 5m before corner end
      },
      audioTriggerComplete = false,
    }
  end

  return event
end

-- triggers needed
-- - cs - static
-- - ce - static
-- - half - static
--   - or make it % between cs and ce
-- - for hiding distance icon at the end of the straight - ce + N meters
-- - at? - static
function C:evaluatePacenoteEvents(pacenote, speedMs)
  -- print(string.format("evaluating pacenote %s", pacenote.name))

  local wpCs = pacenote:getCornerStartWaypoint()
  local wpCe = pacenote:getCornerEndWaypoint()
  local routePointCs = wpCs:getRoutePoint()
  local routePointCe = wpCe:getRoutePoint()

  local triggerType = pacenote:getTriggerType() or RallyEnums.triggerType.dynamic

  if not RallyEnums.triggerTypeName[triggerType] then
    log('E', logTag, "Invalid trigger type: " .. tostring(triggerType))
    return
  end

  if not routePointCs or not routePointCe then
    log('E', logTag, "No route point found for " .. pacenote.name)
    return
  end

  local vehDistToFinish = self.route.path[1].distToTarget
  local event = self:getPacenoteEvent(pacenote)
  local disableTrackingStr = event.disabledByRecalc and "[disabled]" or ""

  if event.disabledByRecalc then
    -- log('D', logTag, string.format('pacenote %s is disabled by recalc', pacenote.name))
    return false
  end

  local markPacenoteAsCompleteThisTick = false
  local causedCompletionStr = nil

  -- Calculate time until car reaches corner start based on current speed
  local timeToCs = nil
  if speedMs and speedMs > 0 then
    local distanceToCs = vehDistToFinish - wpCs:getOriginalDistToTarget()
    timeToCs = distanceToCs / speedMs
  end

  if not timeToCs then
    log('E', logTag, string.format('expected timeToCs for %s', pacenote.name))
    return false
  end

  local speedMph = calcSpeedMph(speedMs)

  -- Scale the speed multiplier between A and B based on speed, starting at X mph, ending at Y mph.
  local speedMultiplier = 1.0
  if speedMph > self.scaleMinSpeedMph then
    local scaleFactor = math.min((speedMph - self.scaleMinSpeedMph) / (self.maxMultiplier - self.minMultiplier), 1.0)
    speedMultiplier = self.minMultiplier + ((self.maxMultiplier - self.minMultiplier) * scaleFactor)
  end
  local audioLenTotal = pacenote:audioLenTotal()
  local audioLenTotalScaled = audioLenTotal * self.audioLenScaler
  local threshold = (self:baseCodriverTiming() + audioLenTotalScaled) * speedMultiplier
  -- log('D', logTag, string.format('threshold=%.2fs speedMph=%0.2fmph speedMultiplier[%0.1f(%dmph)-%0.1f(%dmph)]=%0.2f audioLenTotal=%.2fs audioLenTotalScaled=%.2fs', threshold, speedMph, self.minMultiplier, self.scaleMinSpeedMph, self.maxMultiplier, self.scaleMaxSpeedMph, speedMultiplier, audioLenTotal, audioLenTotalScaled))

  if not event.csDynamicHit and timeToCs < threshold then
    event.csDynamicHit = true
    local shouldTriggerAudio = false
    if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.dynamic then
      markPacenoteAsCompleteThisTick = true
      event.audioTriggerComplete = true
      shouldTriggerAudio = true
    end
    if self:enableTriggerLogging() then
      if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
      local eventStr = string.format("%s[%s]%s cs dynamic @ %0.2fs < %0.2fs, %0.2fmph", disableTrackingStr, pacenote.name, causedCompletionStr or '', timeToCs, threshold, calcSpeedMph(speedMs))
      log('D', logTag, eventStr)
      table.insert(self.eventLog, eventStr)
    end
    self:callPacenoteEventCallback(self.onPacenoteCsDynamicHit, pacenote, shouldTriggerAudio)
  end

  -- immediate is only evaluated when the pacenote's trigger type is immediate, unlike the other triggers.
  if not event.csImmediateHit and triggerType == RallyEnums.triggerType.csImmediate then
    event.csImmediateHit = true
    local shouldTriggerAudio = false
    if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.csImmediate then
      markPacenoteAsCompleteThisTick = true
      event.audioTriggerComplete = true
      shouldTriggerAudio = true
    end
    if self:enableTriggerLogging() then
      if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
      local eventStr = string.format("%s[%s]%s cs immediate @ %0.2fmph", disableTrackingStr, pacenote.name, causedCompletionStr or '', calcSpeedMph(speedMs))
      log('D', logTag, eventStr)
      table.insert(self.eventLog, eventStr)
    end
    self:callPacenoteEventCallback(self.onPacenoteCsImmediateHit, pacenote, shouldTriggerAudio)
  end

  local relativeDistCs = vehDistToFinish - wpCs:getOriginalDistToTarget()
  if not event.csStaticHit and relativeDistCs < self.staticDistanceThresholdMeters then
    event.csStaticHit = true
    local shouldTriggerAudio = false
    if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.csStatic then
      markPacenoteAsCompleteThisTick = true
      event.audioTriggerComplete = true
      shouldTriggerAudio = true
    end
    if self:enableTriggerLogging() then
      if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
      local eventStr = string.format("%s[%s]%s cs static @ %0.2fm < %0.2fm", disableTrackingStr, pacenote.name, causedCompletionStr or '', relativeDistCs, self.staticDistanceThresholdMeters)
      log('D', logTag, eventStr)
      table.insert(self.eventLog, eventStr)
    end
    self:callPacenoteEventCallback(self.onPacenoteCsStaticHit, pacenote, shouldTriggerAudio)
  end

  for percent, hit in pairs(event.cornerPercentHit) do
    local distPercent = pacenote:getCachedLength() * percent
    local absDistPercent = wpCs:getOriginalDistToTarget() - distPercent
    local relativeDistCsPercent = vehDistToFinish - absDistPercent
    local shouldTriggerAudio = false
    if not hit and relativeDistCsPercent < self.staticDistanceThresholdMeters then
      event.cornerPercentHit[percent] = true
      if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.csHalf and percent == 0.5 then
        markPacenoteAsCompleteThisTick = true
        event.audioTriggerComplete = true
        shouldTriggerAudio = true
      end
      if self:enableTriggerLogging() then
        if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
        local eventStr = string.format("%s[%s]%s cs+%0.0f%% @ %0.2fm < %0.2fm", disableTrackingStr, pacenote.name, causedCompletionStr or '', percent * 100, relativeDistCsPercent, self.staticDistanceThresholdMeters)
        log('D', logTag, eventStr)
        table.insert(self.eventLog, eventStr)
      end
      self:callPacenoteEventCallback(self.onPacenoteCornerPercentHit, pacenote, shouldTriggerAudio, percent)
    end
  end

  for offset, hit in pairs(event.csMeterOffsetHit) do
    local distOffset = offset
    local absDistOffset = wpCs:getOriginalDistToTarget() - distOffset
    local relativeDistCsOffset = vehDistToFinish - absDistOffset
    local shouldTriggerAudio = false
    if not hit and relativeDistCsOffset < self.staticDistanceThresholdMeters then
      event.csMeterOffsetHit[offset] = true
      if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.csMinus5 and offset == -5 then
        markPacenoteAsCompleteThisTick = true
        event.audioTriggerComplete = true
        shouldTriggerAudio = true
      end
      if self:enableTriggerLogging() then
        if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
        local sign = offset < 0 and "-" or "+"
        local eventStr = string.format("%s[%s]%s cs%s%0.0fm @ %0.2fm < %0.2fm", disableTrackingStr, pacenote.name, causedCompletionStr or '', sign, math.abs(offset), relativeDistCsOffset, self.staticDistanceThresholdMeters)
        log('D', logTag, eventStr)
        table.insert(self.eventLog, eventStr)
      end
      self:callPacenoteEventCallback(self.onPacenoteCsOffsetHit, pacenote, shouldTriggerAudio, offset)
    end
  end

  for offset, hit in pairs(event.ceMeterOffsetHit) do
    local distOffset = offset
    local absDistOffset = wpCe:getOriginalDistToTarget() - distOffset
    local relativeDistCeOffset = vehDistToFinish - absDistOffset
    local shouldTriggerAudio = false
    if not hit and relativeDistCeOffset < self.staticDistanceThresholdMeters then
      event.ceMeterOffsetHit[offset] = true
      if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.ceMinus5 and offset == -5 then
        markPacenoteAsCompleteThisTick = true
        event.audioTriggerComplete = true
        shouldTriggerAudio = true
      end
      if self:enableTriggerLogging() then
        if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
        local sign = offset < 0 and "-" or "+"
        local eventStr = string.format("%s[%s]%s ce%s%0.0fm @ %0.2fm < %0.2fm", disableTrackingStr, pacenote.name, causedCompletionStr or '', sign, math.abs(offset), relativeDistCeOffset, self.staticDistanceThresholdMeters)
        log('D', logTag, eventStr)
        table.insert(self.eventLog, eventStr)
      end
      self:callPacenoteEventCallback(self.onPacenoteCeOffsetHit, pacenote, shouldTriggerAudio, offset)
    end
  end

  local relativeDistCe = vehDistToFinish - wpCe:getOriginalDistToTarget()
  if not event.ceStaticHit and relativeDistCe < self.staticDistanceThresholdMeters then
    event.ceStaticHit = true
    local shouldTriggerAudio = false
    if not event.audioTriggerComplete and triggerType == RallyEnums.triggerType.ceStatic then
      markPacenoteAsCompleteThisTick = true
      event.audioTriggerComplete = true
      shouldTriggerAudio = true
    end
    if self:enableTriggerLogging() then
      if markPacenoteAsCompleteThisTick then causedCompletionStr = "[C]" end
      local eventStr = string.format("%s[%s]%s ce static @ %0.2fm < %0.2fm", disableTrackingStr, pacenote.name, causedCompletionStr or '', relativeDistCe, self.staticDistanceThresholdMeters)
      log('D', logTag, eventStr)
      table.insert(self.eventLog, eventStr)
    end
    self:callPacenoteEventCallback(self.onPacenoteCeStaticHit, pacenote, shouldTriggerAudio)
  end

  self.events[pacenote.id] = event

  return markPacenoteAsCompleteThisTick
end

function C:callPacenoteEventCallback(callbackFn, ...)
  if not callbackFn then return end
  if self.recalcNeeded then return end
  callbackFn(...)
end

function C:getSpeed()
  if self.trackMouseLikeVehicle then
    return self.mouseSpeedMs
  else
    if not self.vehicleTracker then
      -- log('E', logTag, 'No vehicle tracker found')
      return 0
    end

    local playerVehicle = self.vehicleTracker:getVehicle()

    -- result: saw up to 0.9m using amateur rally sb2
    -- makes me think static distance threshold should include a velocity multiplier
    -- result: vDist: 1.009m vSpeed: 46.919m/s or 104.9mph
    -- local vPos = playerVehicle:getPosition()
    -- if self.lastVPos then
    --   local vDist = vPos:distance(self.lastVPos)
    --   print(string.format("vDist: %0.3fm vSpeed: %0.3fm/s", vDist, vSpeed))
    -- end
    -- self.lastVPos = vPos
    -- although i think this only matters if abs() is used for distance comparison along route.

    if not playerVehicle then
      log('E', logTag, 'No player vehicle found')
      return nil
    end

    -- local speed = playerVehicle:getVelocity():length()
    local speedMs = self.vehicleTracker:speedMs()
    return speedMs
  end
end

function C:getPosition()
  if self.trackMouseLikeVehicle then
    return self.mousePos
  else
    if not self.vehicleTracker then
      -- log('E', logTag, 'No vehicle tracker found')
      return nil
    end

    local playerVehicle = self.vehicleTracker:getVehicle()
    if not playerVehicle then
      log('E', logTag, 'No player vehicle found')
      return nil
    end
    return playerVehicle:getPosition()
  end
end

function C:evaluatePacenotesWindow(speedMs)
  -- dist is how far the vehicle is from the closest point on the route.
  -- the xnorm distance, if you will.
  -- could be used to adjust the pacenote trigger if the car is on the racing line,
  -- but the distance is along the route's line.
  local pos = self:getPosition()
  if not pos then return end
  local _idx, _dist = self.route:trackPosition(pos)

  -- use a small search window to look for the next pacenote because
  local minPacenoteIdx = math.max(self.nextPacenoteIdxForEval - self.pacenoteLookbehindWindow, 1)
  -- local maxPacenoteIdx = math.min(self.nextPacenoteIdxForEval + self.pacenoteLookaheadWindow, #self:getPacenotes())
  local maxPacenoteIdx = math.min(self.nextPacenoteIdxForEval, #self:getPacenotes())

  -- Search window is needed because pacenotes that have been triggered with dynamic audio may still trigger other types of events like clearning visual notes.
  for i = minPacenoteIdx, maxPacenoteIdx do
    local pacenote = self:getPacenotes()[i]
    if pacenote then
      -- log('D', logTag, string.format('evaluating pacenote idx=%d name=%s', i, pacenote.name))
      local audioWasTriggered = self:evaluatePacenoteEvents(pacenote, speedMs)
      if audioWasTriggered then
        self.nextPacenoteIdxForEval = self.nextPacenoteIdxForEval + 1
      end
    else
      -- log('W', logTag, string.format('no pacenote found for idx=%d', i))
    end
  end
end

function C:onUpdate(dtReal, dtSim, dtRaw)
  if not self.route then return end

  self:updateMouseLikeVehicle(dtSim)

  local speedMs = self:getSpeed()
  if not speedMs then return end
  local speedMph = calcSpeedMph(speedMs)
  -- this can happen when a vehicle is reset.
  if speedMph > 1000 then
    -- bypass this tick if the speed is very high, which indicates a reset.
    -- log('W', logTag, string.format('detected insane speed > 1000mph speed=%0.2fmph recalcNeeded=%s', speedMph, tostring(self.recalcNeeded)))
    return
  end

  if self.recalcNeeded then
    -- log('D', logTag, 'recalcNeeded')
    self:recalculate()
    self.recalcNeeded = false
    self.dtSimSumSinceRecalc = 0
  else
    -- dont evaluate pacenotes if the speed is too low.
    -- wait for a little big after a recalc to allow triggering because the vehicle may be bouncing up and down from the reset.
    -- even if it's bouncing up and down, it will count as speed because the refnode is moving.
    if speedMph > self.minEvalSpeedMph and self.dtSimSumSinceRecalc > self.minWaitTimeSinceRecalc then
      self:evaluatePacenotesWindow(speedMs)
    else
      if self.dtSimSumSinceRecalc then
        self.dtSimSumSinceRecalc = self.dtSimSumSinceRecalc + dtSim
      end
    end
  end
end

function C:setRecalcNeeded()
  self.recalcNeeded = true
end

function C:setTrackMouseLikeVehicle(val)
  self.trackMouseLikeVehicle = val

  if not self.trackMouseLikeVehicle then
    self.mousePos = nil
    self.mouseSpeedMs = nil
    self.trackMouseLikeVehicleEnableMovement = false
  end
end

function C:enableTrackMouseLikeVehicleMovement(val)
  self.trackMouseLikeVehicleEnableMovement = val

  if val then
    local mouseRayCast = cameraMouseRayCast()
    local newPos = mouseRayCast.pos
    self.mousePos = newPos
    self.mouseSpeedMs = 0
    self.smoothedPos = newPos
  end
end

function C:updateMouseLikeVehicle(dtSim)
  if not self.trackMouseLikeVehicle then return end

  if self.trackMouseLikeVehicleEnableMovement then
    local mouseRayCast = cameraMouseRayCast()
    local newPos = mouseRayCast.pos

    if not self.mousePos then
      self.mousePos = newPos
      self.mouseSpeedMs = 0
      self.smoothedPos = newPos
      return
    end

    -- Smooth the position using exponential moving average
    -- Lower alpha = more smoothing (0-1)
    local alpha = 0.008
    self.smoothedPos = self.smoothedPos + (newPos - self.smoothedPos) * alpha
  end

  if self.smoothedPos then
    -- update speed regardless of movement enablement
    -- Calculate speed based on smoothed position change
    if self.mousePos then
      local dist = self.smoothedPos:distance(self.mousePos)
      self.mouseSpeedMs = dist / dtSim
    end

    self.mousePos = self.smoothedPos
  end
end

function C:getFinishLineDistOffset()
  if not self.racePath then
    log('E', logTag, 'no race path')
    return nil
  end
  if not self.racePath.pathnodes then
    log('E', logTag, 'no race path nodes')
    return nil
  end
  if #self.racePath.pathnodes.sorted == 0 then
    log('E', logTag, 'no race path nodes sorted')
    return nil
  end
  local lastPathnode = self.racePath.pathnodes.sorted[#self.racePath.pathnodes.sorted]
  if not lastPathnode then
    log('E', logTag, 'no last race path node')
    return nil
  end
  local staticRoutePoint = lastPathnode:getStaticRoutePoint()
  if not staticRoutePoint then
    -- log('E', logTag, 'no static route point')
    return nil
  end

  return staticRoutePoint.distToTarget
end

function C:getRaceDistanceMeters()
  if not self.routeStatic then return end
  if not self.racePath then return end

  local startDistToFinish = self.routeStatic.path[1].distToTarget
  local finishLineDistOffset = self:getFinishLineDistOffset()
  if not finishLineDistOffset then
    -- log('E', logTag, "finishLineDistOffset is nil, probably means the recce driveline is not long enough to reach the finish line")
    return -1
  end
  return startDistToFinish - finishLineDistOffset
end

function C:getRaceDistanceKmString()
  local dist = self:getRaceDistanceMeters()
  if dist and dist >= 0 then
    return string.format("%.2fkm", dist / 1000)
  else
    return "N/A"
  end
end

function C:getRaceCompletionData()
  if not self.route then return end
  if not self.racePath then return end

  local vehDistToFinish = self.route.path[1].distToTarget
  local finishLineDistOffset = self:getFinishLineDistOffset()
  local raceDistM = self:getRaceDistanceMeters()

  -- Calculate distance completed so far
  local distM = raceDistM - (vehDistToFinish - finishLineDistOffset)
  distM = math.max(0, distM)
  local distPct = distM / raceDistM

  self.raceCompletionData.distM = distM
  self.raceCompletionData.distPct = distPct
  self.raceCompletionData.distKm = string.format("%.2fkm", distM / 1000)

  return self.raceCompletionData
end

local colorWhite = ColorF(1, 1, 1, 1)
local colorIBlack = ColorI(0, 0, 0, 192)

local clrFg = ColorF(0, 0, 0, 1)
local clrIBg = ColorI(255, 255, 255, 255)

local clrCs = cc.waypoint_clr_cs
local clrCe = cc.waypoint_clr_ce

local function drawPacenoteWaypoint(wp, point, zOffset, startDistToFinish, drawPacenoteText)
  local wpType = WaypointTypes.shortenWaypointType(wp.waypointType)
  local pacenoteName = wp.pacenote.name
  local relativeDist = startDistToFinish - point.distToTarget

  if wp:isCs() then
    clrIBg = ColorI(clrCs[1] * 255, clrCs[2] * 255, clrCs[3] * 255, 255)
  elseif wp:isCe() then
    clrIBg = ColorI(clrCe[1] * 255, clrCe[2] * 255, clrCe[3] * 255, 255)
  end

  local pacenoteText = ''
  if drawPacenoteText then
    pacenoteText = ' '..wp.pacenote:noteOutputPreview()
  end

  local txtPos = vec3(point.pos)
  txtPos.z = txtPos.z + zOffset
  debugDrawer:drawTextAdvanced(
    txtPos,
    String(string.format("%0.1fm | %s[%s]%s", relativeDist, pacenoteName, wpType, pacenoteText)),
    clrFg, true, false, clrIBg
  )
end

function C:getPointDistanceFromStartMeters(point)
  if not self.routeStatic then return end
  local startDistToFinish = self.routeStatic.path[1].distToTarget
  local distFromStart = startDistToFinish - point.distToTarget
  if not distFromStart then
    log('E', logTag, 'getPointDistanceFromStart: distFromStart is nil')
    return 0.0
  end
  return distFromStart
end

function C:getPointDistanceFromStartKm(point)
  local distFromStart = self:getPointDistanceFromStartMeters(point)
  return distFromStart / 1000.0
end

function C:drawDebugDrivelineRoute(drawRoute, drawPacenotes, drawLimit, monochrome, static, drawHiddenPathnodes, drawRoutePathnodes, drawPointI, drawPointMetadata, drawPacenoteText)
  if not self.route then return end

  monochrome = monochrome or false
  static = static or false
  drawHiddenPathnodes = drawHiddenPathnodes or false
  drawRoutePathnodes = drawRoutePathnodes or false
  drawRoute = drawRoute or static or false
  drawPointI = drawPointI or false
  drawPointMetadata = drawPointMetadata or false
  drawPacenoteText = drawPacenoteText or false

  local routeToDraw = static and self.routeStatic or self.route

  local clr
  local pos
  local pacenoteName
  local relativeDist
  local zOffset = 0

  local drawLimit = drawLimit or 4
  local drawn = 0
  local startDistToFinish = self.routeStatic.path[1].distToTarget

  -- local finishLineDistOffset = self.racePath.pathnodes.sorted[#self.racePath.pathnodes.sorted]:getRoutePoint().distToTarget

  -- local minI = 1
  local maxI = #routeToDraw.path
  -- local maxI = 15

  for i, point in ipairs(routeToDraw.path) do
    clr = rainbowColor(#routeToDraw.path, i, 1)
    pos = vec3(point.pos)
    pos.z = pos.z + zOffset

    if drawRoute then
      if monochrome then
        clr = {0.1, 0.1, 0.1}
        zOffset = 2
      end
      debugDrawer:drawSphere(pos, 1, ColorF(clr[1], clr[2], clr[3], 0.6))

      if i > 1 then
        local pos2 = vec3(routeToDraw.path[i-1].pos)
        pos2.z = pos2.z + zOffset
        debugDrawer:drawSquarePrism(pos, pos2, Point2F(2, 0.5), Point2F(2, 0.5), ColorF(clr[1], clr[2], clr[3], 0.4))
      end
    end -- end drawRoute

    -- helps debug the order of route points.
    if drawPointI then
      clrIBg = ColorI(255, 255, 255, 255)
      debugDrawer:drawTextAdvanced(pos, string.format("%d", i), clrFg, true, false, clrIBg)
    end

    if drawPointMetadata then
      local pointFields = {}
      pointFields.merges = point.merges or 0
      pointFields.fixed = point.fixed
      pointFields.wp = point.wp
      pointFields.distToTarget = point.distToTarget

      if point.metadata then
        -- helps debug point & metadata
        local md = {}
        for k,v in pairs(point.metadata) do
          if type(v) == "table" then
            if k == "wpCs" or k == "wpCe" then
              v = '<'..v:nameWithPacenote()..'>'
            elseif k == "racePathnode" then
              v = '<'..v.name..'>'
            else
              v = "<table>"
            end
          end
          md[k] = v
        end

        pointFields.metadata = md

        if md.source == "pacenote_Cs" then
          clrIBg = ColorI(0, 200, 0, 255)
        elseif md.source == "pacenote_Ce" then
          clrIBg = ColorI(200, 0, 0, 255)
        elseif md.source == "race" then
          clrIBg = ColorI(100, 150, 255, 255)
        else
          clrIBg = ColorI(150, 150, 150, 255)
        end
      else
        clrIBg = ColorI(150, 150, 150, 255)
      end -- end if md.source

      if i <= maxI then
        debugDrawer:drawTextAdvanced(pos, string.format("i=%d %s", i, dumps(pointFields)), clrFg, true, false, clrIBg)
      end
      -- else
      --   if i <= maxI then
      --     clrIBg = ColorI(150, 150, 150, 255)
      --     debugDrawer:drawTextAdvanced(pos, string.format("i=%d no metadata", i), clrFg, true, false, clrIBg)
      --   end
      -- end
    end -- end drawPointMetadata

    if drawRoutePathnodes then
      local textFg = (clr[1] < 0.1 and clr[2] < 0.4 and clr[3] > 0.8) and ColorF(1, 1, 1, 1) or ColorF(0, 0, 0, 1)
      local textBg = ColorI(clr[1] * 255, clr[2] * 255, clr[3] * 255, 255)
      local fixedStr = point.fixed and "[fixed]" or ""
      local stableId = ""
      local source = ""
      local distStr = ""
      local pnType = ""
      if point.metadata then
        source = point.metadata.source or ""
        if source ~= "" then
          source = "[" .. source .. "]"
        end
        stableId = point.metadata.stableId or point.wp or ""
        if fixedStr ~= "" or stableId ~= "" then
          stableId = " " .. stableId
        end

        if point.metadata.source == "pacenote_Cs" then
          textFg = ColorF(0, 0, 0, 1)
          textBg = ColorI(clrCs[1] * 255, clrCs[2] * 255, clrCs[3] * 255, 255)
        elseif point.metadata.source == "pacenote_Ce" then
          textFg = ColorF(0, 0, 0, 1)
          textBg = ColorI(clrCe[1] * 255, clrCe[2] * 255, clrCe[3] * 255, 255)
        end

        if point.metadata.racePathnodeType then
          pnType = point.metadata.racePathnodeType
          if pnType then
            stableId = stableId .. " {" .. pnType .. "}"
          end
        end
      end

      if fixedStr ~= "" and pnType ~= "spStopZone" then
        local distFromStart = startDistToFinish - point.distToTarget
        local splitDistKm = distFromStart / 1000
        local splitLabel = "split"

        if pnType == "finish" then
          splitLabel = "course"
        end

        distStr = string.format(" | %s=%0.2fkm from_start=%0.1fm abs=%0.1fm", splitLabel, splitDistKm, distFromStart, point.distToTarget)
      end

      if (fixedStr ~= "" and source == "[race]") or pnType == "spStart" then
        local visible = true
        local formatStr = "%s%s%s%s"
        if pnType == "" or pnType == "first" then
          formatStr = "(%s%s%s%s)"
          visible = false
        end
        if visible or drawHiddenPathnodes then
          debugDrawer:drawTextAdvanced(pos, String(string.format(formatStr, fixedStr, source, stableId, distStr)), textFg, true, false, textBg)
        end
      end
    end

    if drawPacenotes then
      -- if drawn < drawLimit and point.metadata and point.metadata.pacenoteWaypoint then
      if drawn < drawLimit and point.metadata then

        if point.metadata.wpCs then
          drawPacenoteWaypoint(point.metadata.wpCs, point, zOffset, startDistToFinish, drawPacenoteText)
          drawn = drawn + 1
        end

        if point.metadata.wpCe then
          drawPacenoteWaypoint(point.metadata.wpCe, point, zOffset, startDistToFinish, drawPacenoteText)
          drawn = drawn + 1
        end



        -- local wp = point.metadata.pacenoteWaypoint
        -- local wpType = WaypointTypes.shortenWaypointType(wp.waypointType)
        -- pacenoteName = wp.pacenote.name
        -- relativeDist = startDistToFinish - point.distToTarget

        -- if wp:isCs() then
        --   clrIBg = ColorI(clrCs[1] * 255, clrCs[2] * 255, clrCs[3] * 255, 255)
        -- elseif wp:isCe() then
        --   clrIBg = ColorI(clrCe[1] * 255, clrCe[2] * 255, clrCe[3] * 255, 255)
        -- end

        -- local txtPos = vec3(point.pos)
        -- txtPos.z = txtPos.z + zOffset
        -- debugDrawer:drawTextAdvanced(txtPos, String(string.format("%0.1fm | %s[%s]", relativeDist, pacenoteName, wpType)), clrFg, true, false, clrIBg)
        -- drawn = drawn + 1
      end -- end draw limit stuff
    end -- end if drawPacenotes
  end -- end for loop
end -- end method

-- local clrShort = ColorF(1, 0, 0, 0.5)
local clrShort = ColorF(0, 1, 0, 0.5)
local h = 0.25
local w = 0.5

function C:drawDebugDrivelineRouteShort(dist)
  if not self.route then return end
  if not self.racePath then return end

  local distLimit = dist or 100
  local distDrawn = 0

  for i, point in ipairs(self.route.path) do
    local pos = vec3(point.pos)
    -- debugDrawer:drawSphere(pos, 1, clrShort)

    if i > 1 then
      local pos2 = vec3(self.route.path[i-1].pos)
      local dist = pos:distance(pos2)
      distDrawn = distDrawn + dist
      debugDrawer:drawSquarePrism(pos, pos2, Point2F(h,w), Point2F(h,w), clrShort)
    end

    if distDrawn > distLimit then
      break
    end
  end

end

return function(...)
  local o = {}

  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end