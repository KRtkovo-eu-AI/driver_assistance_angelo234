-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Control parameters.
local lookAheadDistance = 150.0                                         -- The distance ahead, in metres, to which the navigraph will be scanned to collect road data.
local splineGranularity = 50                                            -- The granularity (discretisation) used when spline fitting (used in computed the distances to road lanes).
local splineSmoothness = 0.5                                            -- The smoothness value, in [0, 1] which is used when fitting a CR-spline to four points.
local timeToWaitForMap = 5000                                           -- The number of physics steps to wait until collecting the navgraph.

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

M.type = "auxiliary"

local logTag = 'roadsSensor'

-- Module constants.
local min, max, abs, sqrt, acos = math.min, math.max, math.abs, math.sqrt, math.acos
local NaN = 0 / 0
local splineGranInv = 1.0 / splineGranularity

-- Module state.
local sensorId = nil                                                    -- The unique Id number for roads sensor.
local GFXUpdateTime = nil                                               -- The GFX step update time (ie how often readings data is available to the user).
local timeSinceLastPoll = 0.0                                           -- The time since this roads sensor was last polled (for graphics step).
local physicsTimer = nil                                                -- A timer used for the physics step, to check if an readings update is required.
local physicsUpdateTime = nil                                           -- How often the physics should be updated, in seconds.
local readings, readingIndex = {}, 1                                    -- Container and counter for the sensor readings.
local isMapDataCached = false                                           -- A flag which indicates if the navigraph map data has been cached, and is ready to access here in vlua.
local initTimer = 0                                                     -- A counter used to wait for the navgraph data to become available in vlua.
local isVisualised = false                                              -- A flag which indicates whether the sensor will draw debug data or not.

-- Navgraph state.
local graph, coords, widths, normals = {}, {}, {}, {}                   -- Initialise a table to store the navigraph map, nodes, widths and normals.

local p1_prev_Key, p2_prev_Key = nil, nil
local p1Key, p2Key
-- The player vehicle state.
local pos, fwd, vel = vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0)                           -- The vehicle's position and forward vector.
local distToCenterline, distToLeft, distToRight = 0.0, 0.0, 0.0         -- The minimum distances between the vehicle's front axle midpoint to each lane.
local headingAngle,headingAngle_prev = 0.0, 0.0                                                -- The heading of the vehicle with respect to the road immediately ahead.
local halfWidth = 0.0                                                   -- The half-width of the road at the vehicle's front axis.
local roadRadius = NaN                                                  -- The radius of the road immediately ahead of the vehicle.
local coeffsCL, coeffsL, coeffsR = {}, {}, {}                           -- Tables to store the polynomial coefficients for each lane spline.
local coordsCL = {}                                                     -- A table to store the road centerline coordinates (of the next four nodes)
local drivability, speedLimit, oneWay = NaN, NaN, NaN                   -- Some extra road meta data values related to the road immediately ahead.
local numlane = 0
local latestReading = {
  time = 0.0, dist2CL = 0.0, dist2Left = 0.0, dist2Right = 0.0,
  halfWidth = 0.0, roadRadius = 0.0, headingAngle = 0.0,
  xP0onCL = 0.0, yP0onCL = 0.0, zP0onCL = 0.0,
  xP1onCL = 0.0, yP1onCL = 0.0, zP1onCL = 0.0,
  xP2onCL = 0.0, yP2onCL = 0.0, zP2onCL = 0.0,
  xP3onCL = 0.0, yP3onCL = 0.0, zP3onCL = 0.0,
  uAofCL = 0.0, uBofCL = 0.0, uCofCL = 0.0, uDofCL = 0.0,
  vAofCL = 0.0, vBofCL = 0.0, vCofCL = 0.0, vDofCL = 0.0,
  uAofLeftRE = 0.0, uBofLeftRE = 0.0, uCofLeftRE = 0.0, uDofLeftRE = 0.0,
  vAofLeftRE = 0.0, vBofLeftRE = 0.0, vCofLeftRE = 0.0, vDofLeftRE = 0.0,
  uAofRightRE = 0.0, uBofRightRE = 0.0, uCofRightRE = 0.0, uDofRightRE = 0.0,
  vAofRightRE = 0.0, vBofRightRE = 0.0, vCofRightRE = 0.0, vDofRightRE = 0.0,
  xStartCL = 0.0, yStartCL = 0.0, zStartCL = 0.0,
  xStartL = 0.0, yStartL = 0.0, zStartL = 0.0,
  xStartR = 0.0, yStartR = 0.0, zStartR = 0.0,
  drivability = 0.0, speedLimit = 0.0, flag1way = 0.0, numlane = 0.0 }

-- Computes the closest point on the given line segment (a, b) to the given point p, in 2D.
local function closestPointBetween2D(p, a, b)
  local u, v = vec3(a.x - p.x, a.y - p.y), vec3(b.x - a.x, b.y - a.y)
  local vu, vv = v:dot(u), v:dot(v)
  local t = -vu / vv
  if t >= 0 and t <= 1 then
    return (1 - t) * a + t * b
  end
  local q0, q1 = a - p, b - p
  local q0x, q0y, q1x, q1y = q0.x, q0.y, q1.x, q1.y
  if q0x * q0x + q0y + q0y <= q1x * q1x + q1y * q1y then
    return a
  end
  return b
end

-- Computes the two CR tangents used when fitting a parametric cubic (from 4 points, we go to 2 inner points and 2 defined CR-based tangents).
local function computeTangents(pn0, pn1, pn2, pn3)
  local d1, d2, d3 = max(sqrt(pn0:distance(pn1)), 1e-12), sqrt(pn1:distance(pn2)), max(sqrt(pn2:distance(pn3)), 1e-12)
  local m = (pn1 - pn0) / d1 + (pn0 - pn2) / (d1 + d2)
  local n = (pn1 - pn3) / (d2 + d3) + (pn3 - pn2) / d3
  local pn12 = pn2 - pn1
  local t1 = d2 * m + pn12
  if t1:length() < 1e-5 then t1 = pn12 * 0.5 end
  local t2 = d2 * n + pn12
  if t2:length() < 1e-5 then t2 = pn12 * 0.5 end
  return t1, t2
end

-- Computes the reference line cubic equations (parametric).
local function computeRefLineCubic(p0, p1, p2, p3)
  local p1_2d = p1
  local pn0_2d, pn1_2d, pn2_2d, pn3_2d = p0 - p1_2d, vec3(0.0, 0.0, 0.0), p2 - p1_2d, p3 - p1_2d
  local t1, t2 = computeTangents(pn0_2d, pn1_2d, pn2_2d, pn3_2d)
  local coeffC, coeffD = (-2.0 * t1) - t2 + (3.0 * pn2_2d), t1 + t2 - (2.0 * pn2_2d)
  return { uA = pn1_2d.x, uB = t1.x, uC = coeffC.x, uD = coeffD.x, vA = pn1_2d.y, vB = t1.y, vC = coeffC.y, vD = coeffD.y }
end

-- For a given navigraph node, width and direction, computes the left and right road edge points.
local function computeRoadEdgePoints(key, dir)
  local coord, n = coords[key], normals[key]
  dir:normalize()
  n:normalize()
  local lateralVec = dir:cross(n) * widths[key]
  return coord - lateralVec, coord + lateralVec
end

local function numOfLanesFromRadius(rad1, rad2)
  return max(1, math.floor(min(rad1, rad2 or math.huge) * 2 / 3.45 + 0.5)) -- math.floor(min(rad1, rad2) / 2.7) + 1
end

local function flipLanes(lanes)
  -- ex. '--+++' becomes '---++'
  local res = ''
  for i = #lanes, 1, -1 do
    local char = lanes:byte(i) == 43 and '-' or lanes:byte(i) == 45 and '+' or '0'
    res = res..char
  end
  return res
end

local function numOfLanesInDirection(lanes, dir)
  dir = dir == '+' and 43 or dir == '-' and 45
  local lanesN = 0
  for i = 1, #lanes, 1 do
    if lanes:byte(i) == dir then
      lanesN = lanesN + 1
    end
  end
  return lanesN
end

local function getEdgeLaneConfig(inNode, outNode)
  local lanes
  local edge = mapmgr.mapData.graph[inNode][outNode]
  if edge.lanes then
    lanes = edge.lanes
  else -- make up some lane data in case they don't exist
    if edge.oneWay then
      local numOfLanes = numOfLanesFromRadius(mapmgr.mapData.radius[inNode], mapmgr.mapData.radius[outNode])
      lanes = string.rep("+", numOfLanes)
    else
      local numOfLanes = max(1, math.floor(numOfLanesFromRadius(mapmgr.mapData.radius[inNode], mapmgr.mapData.radius[outNode]) * 0.5))
      if mapmgr.rules.rightHandDrive then
        lanes = string.rep("+", numOfLanes)..string.rep("-", numOfLanes)
      else
        lanes = string.rep("-", numOfLanes)..string.rep("+", numOfLanes)
      end
    end
  end

  return edge.inNode == inNode and lanes or flipLanes(lanes) -- flip lanes string based on inNode data
end

local function cubicCurvature(coeff,u)
  local dx = coeff.uB + 2*coeff.uC*u + 3*coeff.uD*u*u
  local dy = coeff.vB + 2*coeff.vC*u + 3*coeff.vD*u*u
  local ddx = 2*coeff.uC + 6*coeff.uD*u
  local ddy = 2*coeff.vC + 6*coeff.vD*u
  return abs(dx*ddy - dy*ddx)/max((dx^2 + dy^2)^(3/2),1e-30)
end

-- Computes the curvature between two vectors.
local function inCurvature(vec1, vec2)
  local vec1Sqlen, vec2Sqlen = vec1:squaredLength(), vec2:squaredLength()
  local dot12 = vec1:dot(vec2)
  local cos8sq = min(1, dot12 * dot12 / max(1e-30, vec1Sqlen * vec2Sqlen))
  if dot12 < 0 then
    local minDsq = min(vec1Sqlen, vec2Sqlen)
    local maxDsq = minDsq / max(1e-30, cos8sq)
    if max(vec1Sqlen, vec2Sqlen) > (minDsq + maxDsq) * 0.5 then
      if vec1Sqlen > vec2Sqlen then
        vec1, vec2 = vec2, vec1
        vec1Sqlen, vec2Sqlen = vec2Sqlen, vec1Sqlen
      end
      vec2:setScaled(sqrt(0.5 * (minDsq + maxDsq) / max(1e-30, vec2Sqlen)))
    end
  end
  vec2:setScaled(-1)
  return 2 * sqrt((1 - cos8sq) / max(1e-30, vec1:squaredDistance(vec2)))
end

local function getPointAhead(vpos, vfwd, pl)
  local p_rearKey, p_frontKey, _ = mapmgr.findBestRoad(vpos, vfwd)
  local p_front = coords[p_frontKey]
  local p_prev = coords[p_rearKey]

  if vfwd:dot(p_front) < vfwd:dot(p_prev) then
    p_rearKey, p_frontKey = p_frontKey, p_rearKey
    p_front, p_prev = p_prev, p_front
  end

  local distSq = vpos:squaredDistance(p_front)
  while distSq - pl*pl < 0 do
    local cands, c2Ctr = {}, 1
    for k, _ in pairs(graph[p_frontKey]) do
      cands[c2Ctr] = k
      c2Ctr = c2Ctr + 1
    end

    local len = #cands
    local bestAbsDot = 0.0
    local pbest = p_front
    local pbestKey = p_frontKey
    for i = 1, len do
      local p = coords[cands[i]]
      local Dot = (p_front - p_prev):dot(p - p_front)   --(p - p_front):dot(p_front - p_prev)
      local absDot = abs(Dot)
      if (p - p_prev):squaredLength() > 1e-5 and absDot > bestAbsDot and Dot > 0 then
        pbest, bestAbsDot, pbestKey = p, absDot, cands[i]
      end
    end
    p_prev = p_front
    p_front = pbest
    p_frontKey = pbestKey
    local ds = p_prev:squaredDistance(p_front)
    distSq = distSq + ds
  end

  return p_frontKey, p_rearKey, p_front
end

-- Attempts to find the four point extrusion from two given navgraph keys.
-- The line segment p1->p2 bounds the vehicle position, and we wish to get valid p0 and p3s from the graph.
local function getFourBoundingPoints(p1Key, p2Key)

  --local p1, p2 = coords[p1Key], coords[p2Key]
  local p1,p2 = mapmgr.mapData:getEdgePositions(p1Key, p2Key)
  p1 = p1 or coords[p1Key]
  p2 = p2 or coords[p2Key]

  -- Fetch all the candidate graph keys from each graph node of the given line segment.
  local cands1, c1Ctr = {}, 1
  for k, _ in pairs(graph[p1Key]) do
    cands1[c1Ctr] = k
    c1Ctr = c1Ctr + 1
  end
  local cands2, c2Ctr = {}, 1
  for k, _ in pairs(graph[p2Key]) do
    cands2[c2Ctr] = k
    c2Ctr = c2Ctr + 1
  end

  -- If there are not four points to be taken, then return false immediately.
  local len1, len2 = #cands1, #cands2
  if len1 < 2 or len2 < 2 then
    return nil, nil, nil, nil, false
  end

  -- Find the best-fitting p0 point from the first candidates array.
  local bestAbsDot = 0.0
  local p0 = p1
  local p0Key = p1Key
  for i = 1, len1 do
    local p = coords[cands1[i]]
    local Dot = ((p1 - p2):dot(p - p1))
    local absDot = abs(Dot)
    if (p - p2):squaredLength() > 1e-5 and absDot > bestAbsDot and Dot > 0 then --and  dist < bdist  and Dot > 0
      p0, bestAbsDot, p0Key = p, absDot, cands1[i]
    end
  end

  -- Find the best-fitting p3 point from the second candidates array
  bestAbsDot = 0.0
  local p3 = p2
  local p3Key = p2Key
  for i = 1, len2 do
    local p = coords[cands2[i]]
    local Dot = ((p2 - p1):dot(p - p2))
    local absDot = abs(Dot)
    if (p - p1):squaredLength() > 1e-5 and absDot > bestAbsDot  and Dot > 0 then --and  dist < bdist  and Dot > 0
      p3, bestAbsDot, p3Key = p, absDot, cands2[i]
    end
  end

  return p0, p1, p2, p3, true, p0Key, p3Key
end

local function getSensorData()
  return {
    readings = readings,
    GFXUpdateTime = GFXUpdateTime,
    timeSinceLastPoll = timeSinceLastPoll }
end

local function getLatest() return latestReading end

local function incrementTimer(dtSim) timeSinceLastPoll = timeSinceLastPoll + dtSim end

-- Initialises this roads sensor instance.
local function init(data)
  sensorId = data.sensorId
  GFXUpdateTime = data.GFXUpdateTime
  timeSinceLastPoll = 0.0
  readings = {}
  readingIndex = 1
  physicsTimer = 0.0
  physicsUpdateTime = data.physicsUpdateTime
  isVisualised = data.isVisualised
end

local function reset()
  readings = {}
  readingIndex = 1
  timeSinceLastPoll = timeSinceLastPoll % math.max(GFXUpdateTime, 1e-30)
end

-- The roads sensor physics step update callback.
local function update(dtSim)

  -- After init, we must wait a few frames to let the navgraph data become available in vlua.
  -- This is only done once. Once it is collected we can skip this section - it will not change during execution.
  if not isMapDataCached then
    if initTimer < timeToWaitForMap then
      if initTimer < 1 then
        mapmgr.requestMap()
      end
      initTimer = initTimer + 1
      return
    end

    -- Cache the map data if we do not already have it yet. If it remains unavailable, reset the timer and wait again.
    local mapData = mapmgr.mapData
    if mapData == nil then
      initTimer = 0
      log('W', logTag, 'Navgraph data not yet available. Reseting timer')
      return
    end
    graph, coords, widths = mapData.graph, mapData.positions, mapData.radius
    for k, v in pairs(coords) do
      normals[k] = mapmgr.surfaceNormalBelow(v)
    end
    isMapDataCached = true
  end

  -- Cycle the physics update timer. If we are not ready for a physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Compute the player vehicle pose data.
  pos, fwd, vel = obj:getPosition(), obj:getDirectionVector(), vec3(obj:getSmoothRefVelocityXYZ())
  fwd:normalize()
  local aiSpeed = vel:length()
  -- Compute the player vehicle wheels data.
  local wp, ctr = {}, 1
  for _, wheel in pairs(wheels.wheels) do
    wp[ctr] = obj:getNodePosition(wheel.node1)
    ctr = ctr + 1
  end

  -- Compute the player vehicle axle data.
  local frontAxleMidpoint, rearAxleMidpoint = pos + (wp[min(ctr, 3)] + wp[min(ctr, 4)]) * 0.5, pos + (wp[min(ctr, 1)] + wp[min(ctr, 2)]) * 0.5
  local frontAxleMidpointProjGround = vec3(frontAxleMidpoint.x, frontAxleMidpoint.y, obj:getSurfaceHeightBelow(frontAxleMidpoint))

  -- Compute the distances from the player vehicle front axle midpoint to the road centerline and edges.
  if aiSpeed > 2 then
    p1Key, p2Key = mapmgr.findBestRoad(frontAxleMidpointProjGround, fwd)
  else
    p1Key, p2Key = p1_prev_Key, p2_prev_Key
  end
  p1_prev_Key, p2_prev_Key = p1Key, p2Key
  if p1Key ~= nil and p2Key ~= nil then

    -- Find the four bounding points from the navgraph, such that p1 and p2 are the local bounds.
    -- If we cannot find four bounding points (eg due to a dead end of a road), then skip computing these properties.
    local p0, p1, p2, p3, isFourPointsFound, p0Key, p3Key = getFourBoundingPoints(p1Key, p2Key, coords)
    if isFourPointsFound then

      -- Fit a spline to the bounding line segment, to ensure smoothness.
      -- Also store the linearly-interpolated widths for each discretisation point.
      local w1, w2 = widths[p1Key], widths[p2Key]
      local w0, w3 = widths[p0Key], widths[p3Key]
      local k1 = max(inCurvature(p1-p0,p2-p1),1e-30)
      local k2 = max(inCurvature(p2-p1, p3-p2),1e-30)
      local dk = k2 - k1
      local dw = w2 - w1
      local disc, wds, curv, ctr = {}, {}, {}, 1
      for k = 0, splineGranularity do
        local q = k * splineGranInv
        disc[ctr] = catmullRomCentripetal(p0, p1, p2, p3, q, splineSmoothness)
        --obj.debugDrawProxy:drawSphere(0.1, disc[ctr] + vec3(0, 0, 0.25), color(255, 255, 255, 255))
        wds[ctr] = catmullRom(w0, w1, w2, w3, q, splineSmoothness)
        curv[ctr] = k1 + dk*q
        ctr = ctr + 1
      end

      -- Find the closest line segment from the discretised spline (this is not the closest line segment from the navgraph [p1Key, p2Key], which we computed before).
      local dSqBest, best1, best2, curvBest, numDisc = 1e99, nil, nil, nil, #disc
      for i = 2, numDisc do
        local iMinus1 = i - 1
        local tp1, tp2 = disc[iMinus1], disc[i]
        local dSq = frontAxleMidpointProjGround:squaredDistanceToLineSegment(tp1, tp2)
        if dSq < dSqBest then
          dSqBest, best1, best2, halfWidth, curvBest = dSq, tp1, tp2, (wds[iMinus1] + wds[i]) * 0.5, (curv[iMinus1] + curv[i]) * 0.5
        end
      end
      roadRadius = 1/curvBest
      -- Compute the normalised line segment, and ensure it has the correct direction (it should point closest to the vehicle forward direction).
      local lineSegNorm = best2 - best1
      lineSegNorm:normalize()
      if fwd:dot(best1) > fwd:dot(best2) then
        lineSegNorm = -lineSegNorm
      end

      -- Compute the heading angle when compared the vehicle forward direction.
      local fwdproj = vec3(fwd.x, fwd.y, 0); fwdproj:normalize() -- New
      local lineSegNormproj = vec3(lineSegNorm.x, lineSegNorm.y, 0); lineSegNormproj:normalize() -- New
      local alfa = 0.5
      headingAngle = alfa*acos(max(-1, min(1, fwdproj:dot(lineSegNormproj)))) + (1-alfa)*headingAngle_prev-- New
      if headingAngle < 1.5*math.pi/180 then
        headingAngle = 0
      end
      headingAngle_prev = headingAngle
      -- Compute the shortest distance between the vehicle front axle midpoint and the best-matching line segment (the line segment from the spline, not the navgraph).
      local pInt = closestPointBetween2D(frontAxleMidpointProjGround, best1, best2)
      pInt.z = p1.z +2 --9999
      pInt.z = obj:getSurfaceHeightBelow(pInt)

      -- Extrude outwards along the perpendicular vector to get the local left and right road edge point estimates.
      local perpVector = vec3(lineSegNorm.y, -lineSegNorm.x, lineSegNorm.z)
      --local latVec = halfWidth * lineSegNorm:cross(normals[p2Key])
      local latVec = halfWidth * perpVector
      local pLeftRaw, pRightRaw = pInt - latVec, pInt + latVec
      pLeftRaw.z, pRightRaw.z = p1.z +2, p1.z +2 --9999, 9999
      local pLeft = vec3(pLeftRaw.x, pLeftRaw.y, obj:getSurfaceHeightBelow(pLeftRaw))
      local pRight = vec3(pRightRaw.x, pRightRaw.y, obj:getSurfaceHeightBelow(pRightRaw))

      -- Set the distances from the vehicle front axle midpoint to each estimated point.
      distToCenterline, distToLeft, distToRight = sqrt(dSqBest), (pLeft - frontAxleMidpointProjGround):length(), (pRight - frontAxleMidpointProjGround):length()
      if distToLeft < distToRight then
        distToCenterline = -distToCenterline
      end

      if isVisualised then
        -- For debugging.
        --
        obj.debugDrawProxy:drawSphere(0.1, pLeft + vec3(0, 0, 0.25), color(255, 0, 0, 255))
        obj.debugDrawProxy:drawSphere(0.1, pInt + vec3(0, 0, 0.25), color(0, 255, 0, 255))
        obj.debugDrawProxy:drawSphere(0.1, pRight + vec3(0, 0, 0.25), color(0, 0, 255, 255))

        obj.debugDrawProxy:drawSphere(0.2, p0 + vec3(0, 0, 0.25), color(255, 255, 255,255))
        obj.debugDrawProxy:drawSphere(0.2, p1 + vec3(0, 0, 0.25), color(255, 155, 155,255))
        obj.debugDrawProxy:drawSphere(0.2, p2 + vec3(0, 0, 0.25), color(155, 155, 255,255))
        obj.debugDrawProxy:drawSphere(0.2, p3 + vec3(0, 0, 0.25), color(255, 255, 255,255))
        --
      end
    end
  end

  -- Compute the parametric polynomials for the road centerline (reference line), road left edge, and road right edge.
  --local pointAhead = rearAxleMidpoint + (lookAheadDistance * fwd)
  --local path = mapmgr.getPointToPointPath(rearAxleMidpoint, pointAhead, nil, 1e-4, nil, nil, nil)
  local pFKey, pRKey, pH = getPointAhead(frontAxleMidpointProjGround, fwd, lookAheadDistance)
  local path = mapmgr.mapData:getPath(pRKey, pFKey, nil)
  coeffsCL = { uA = 0, uB = 0, uC = 0, uD = 0, vA = 0, vB = 0, vC = 0, vD = 0 }
  coeffsL = { uA = 0, uB = 0, uC = 0, uD = 0, vA = 0, vB = 0, vC = 0, vD = 0 }
  coeffsR = { uA = 0, uB = 0, uC = 0, uD = 0, vA = 0, vB = 0, vC = 0, vD = 0 }
  coordsCL = { a = vec3(0, 0), b = vec3(0, 0), c = vec3(0, 0), d = vec3(0, 0) }
  local startCL, startL, startR = vec3(0, 0), vec3(0, 0), vec3(0, 0)
  --roadRadius = NaN
  if #path > 3 then
    local p1, p2, p3, p4 = coords[path[1]], coords[path[2]], coords[path[3]], coords[path[4]]
    local inLaneConfig = getEdgeLaneConfig(path[2], path[3])
    numlane = numOfLanesInDirection(inLaneConfig, '+')    local left1, right1 = computeRoadEdgePoints(path[1], p2 - p1)
    local left2, right2 = computeRoadEdgePoints(path[2], p3 - p1)
    local left3, right3 = computeRoadEdgePoints(path[3], p4 - p2)
    local left4, right4 = computeRoadEdgePoints(path[4], p4 - p3)
    coeffsCL = computeRefLineCubic(p1, p2, p3, p4)
    coeffsL, coeffsR = computeRefLineCubic(left1, left2, left3, left4), computeRefLineCubic(right1, right2, right3, right4)
    startCL, startL, startR = p2, left2, right2
    coordsCL = { a = p1, b = p2, c = p3, d = p4 }
    --roadRadius = 1.0 / inCurvature(p2 - p1, p3 - p2)
  elseif #path > 2 then
    local p1, p2, p3 = coords[path[1]], coords[path[2]], coords[path[3]]
    local inLaneConfig = getEdgeLaneConfig(path[2], path[3])
    numlane = numOfLanesInDirection(inLaneConfig, '+')    local left1, right1 = computeRoadEdgePoints(path[1], p2 - p1)
    local left2, right2 = computeRoadEdgePoints(path[2], p3 - p1)
    local left3, right3 = computeRoadEdgePoints(path[3], p3 - p2)
    coeffsCL = computeRefLineCubic(p1, p1, p2, p3)
    coeffsL, coeffsR = computeRefLineCubic(left1, left1, left2, left3), computeRefLineCubic(right1, right1, right2, right3)
    startCL, startL, startR = p1, left1, right1
    coordsCL = { a = p1, b = p2, c = p3, d = vec3(NaN, NaN, NaN) }
    --roadRadius = 1.0 / inCurvature(p2 - p1, p3 - p2)
  elseif #path > 1 then
    local p1, p2 = coords[path[1]], coords[path[2]]
    local inLaneConfig = getEdgeLaneConfig(path[1], path[2])
    numlane = numOfLanesInDirection(inLaneConfig, '+')
    local dir = p2 - p1
    local left1, right1 = computeRoadEdgePoints(path[1], dir)
    local left2, right2 = computeRoadEdgePoints(path[2], dir)
    coeffsCL = computeRefLineCubic(p1, p1, p2, p2)
    coeffsL, coeffsR = computeRefLineCubic(left1, left1, left2, left2), computeRefLineCubic(right1, right1, right2, right2)
    startCL, startL, startR = p1, left1, right1
    coordsCL = { a = p1, b = p2, c = vec3(NaN, NaN, NaN), d = vec3(NaN, NaN, NaN) }
    --roadRadius = NaN
  end

  -- Extract some useful road metadata.
  drivability, speedLimit, oneWay = NaN, NaN, NaN
  if p1Key ~= nil then
    if graph[p1Key][p2Key].drivability ~= nil then
      drivability = graph[p1Key][p2Key].drivability
    end
    if graph[p1Key][p2Key].speedLimit ~= nil then
      speedLimit = graph[p1Key][p2Key].speedLimit
    end
    if graph[p1Key][p2Key].oneWay ~= nil then
      if graph[p1Key][p2Key].oneWay == true then
        oneWay = 1.0
      else
        oneWay = 0.0
      end
    end
  end

  -- Populate the latest readings with the freshly-computed data.
  local aCL, bCL, cCL, dCL = coordsCL.a, coordsCL.b, coordsCL.c, coordsCL.d
  latestReading = {
    time = obj:getSimTime(),                                              -- Time-stamp the sample reading.
    dist2CL = distToCenterline,                                           -- Approx. minimum distance between vehicle front-axle-midpoint and road reference line (center line), in metres.
    dist2Left = distToLeft,                                               -- Approx. minimum distance between vehicle front-axle-midpoint and left road edge, in metres.
    dist2Right = distToRight,                                             -- Approx. minimum distance between vehicle front-axle-midpoint and right road edge, in metres.
    halfWidth = halfWidth,                                                -- The half-width (center to edge) of the road at the front-axle-midpoint, in metres.
    roadRadius = roadRadius,                                              -- The radius of the curvature of the road, in metres.  If road is straight, radius = NaN.
    headingAngle = headingAngle,                                          -- The angle between the vehicle forward direction and the road reference line, in rad.

    xP0onCL = aCL.x,                                                      -- The world-space X coordinate of the closest road point, 'P0', in metres.  Fit with points P0, P1, P2, P3.
    yP0onCL = aCL.y,                                                      -- The world-space Y coordinate of the closest road point, 'P0', in metres.
    zP0onCL = aCL.z,                                                      -- The world-space Z coordinate of the closest road point, 'P0', in metres.
    xP1onCL = bCL.x,                                                      -- The world-space X coordinate of the 2nd-closest road point, 'P1', in metres.
    yP1onCL = bCL.y,                                                      -- The world-space Y coordinate of the 2nd-closest road point, 'P1', in metres.
    zP1onCL = bCL.z,                                                      -- The world-space Z coordinate of the 2nd-closest road point, 'P1', in metres.
    xP2onCL = cCL.x,                                                      -- The world-space X coordinate of the 3rd-closest road point, 'P2', in metres.
    yP2onCL = cCL.y,                                                      -- The world-space Y coordinate of the 3rd-closest road point, 'P2', in metres.
    zP2onCL = cCL.z,                                                      -- The world-space Z coordinate of the 3rd-closest road point, 'P2', in metres.
    xP3onCL = dCL.x,                                                      -- The world-space X coordinate of the 4th-closest road point, 'P3', in metres.
    yP3onCL = dCL.y,                                                      -- The world-space Y coordinate of the 4th-closest road point, 'P3', in metres.
    zP3onCL = dCL.z,                                                      -- The world-space Z coordinate of the 4th-closest road point, 'P3', in metres.

    uAofCL = coeffsCL.uA,                                                 -- Road Reference-Line Parametric Cubic Polynomial U equation, constant term.
    uBofCL = coeffsCL.uB,                                                 -- Road Reference-Line Parametric Cubic Polynomial U equation, linear term.
    uCofCL = coeffsCL.uC,                                                 -- Road Reference-Line Parametric Cubic Polynomial U equation, quadratic term.
    uDofCL = coeffsCL.uD,                                                 -- Road Reference-Line Parametric Cubic Polynomial U equation, cubic term.
    vAofCL = coeffsCL.vA,                                                 -- Road Reference-Line Parametric Cubic Polynomial V equation, constant term.
    vBofCL = coeffsCL.vB,                                                 -- Road Reference-Line Parametric Cubic Polynomial V equation, linear term.
    vCofCL = coeffsCL.vC,                                                 -- Road Reference-Line Parametric Cubic Polynomial V equation, quadratic term.
    vDofCL = coeffsCL.vD,                                                 -- Road Reference-Line Parametric Cubic Polynomial V equation, cubic term.
    uAofLeftRE = coeffsL.uA,                                              -- Road Left Edge Parametric Cubic Polynomial U equation, constant term.
    uBofLeftRE = coeffsL.uB,                                              -- Road Left Edge Parametric Cubic Polynomial U equation, linear term.
    uCofLeftRE = coeffsL.uC,                                              -- Road Left Edge Parametric Cubic Polynomial U equation, quadratic term.
    uDofLeftRE = coeffsL.uD,                                              -- Road Left Edge Parametric Cubic Polynomial U equation, cubic term.
    vAofLeftRE = coeffsL.vA,                                              -- Road Left Edge Parametric Cubic Polynomial V equation, constant term.
    vBofLeftRE = coeffsL.vB,                                              -- Road Left Edge Parametric Cubic Polynomial V equation, linear term.
    vCofLeftRE = coeffsL.vC,                                              -- Road Left Edge Parametric Cubic Polynomial V equation, quadratic term.
    vDofLeftRE = coeffsL.vD,                                              -- Road Left Edge Parametric Cubic Polynomial V equation, cubic term.
    uAofRightRE = coeffsR.uA,                                             -- Road Right Edge Parametric Cubic Polynomial U equation, constant term.
    uBofRightRE = coeffsR.uB,                                             -- Road Right Edge Parametric Cubic Polynomial U equation, linear term.
    uCofRightRE = coeffsR.uC,                                             -- Road Right Edge Parametric Cubic Polynomial U equation, quadratic term.
    uDofRightRE = coeffsR.uD,                                             -- Road Right Edge Parametric Cubic Polynomial U equation, cubic term.
    vAofRightRE = coeffsR.vA,                                             -- Road Right Edge Parametric Cubic Polynomial V equation, constant term.
    vBofRightRE = coeffsR.vB,                                             -- Road Right Edge Parametric Cubic Polynomial V equation, linear term.
    vCofRightRE = coeffsR.vC,                                             -- Road Right Edge Parametric Cubic Polynomial V equation, quadratic term.
    vDofRightRE = coeffsR.vD,                                             -- Road Right Edge Parametric Cubic Polynomial V equation, cubic term.

    xStartCL = startCL.x, yStartCL = startCL.y, zStartCL = startCL.z,     -- The starting point of the road centerline.
    xStartL = startL.x, yStartL = startL.y, zStartL = startL.z,           -- The starting point of the road left edge (estimated).
    xStartR = startR.x, yStartR = startR.y, zStartR = startR.z,           -- The starting point of the road right edge (estimated).

    drivability = drivability,                                            -- The 'drivability' number of the road [smaller = dirt/country roads, larger = highways etc].
    speedLimit = speedLimit,                                              -- The speed limit of the road, in m/s.
    flag1way = oneWay,                                                     -- A flag which indicates if the road is bi-directional (val = 0.0), or one-way (val = 1.0).
    numlane = numlane
  }

  -- Store the latest readings for this roads sensor in the extension. This is used for sending back on the physics step.
  extensions.tech_roadsSensor.cacheLatestReading(sensorId, latestReading)

  -- Add the data to the readings array, for later retrieval. This is used for sending back on the graphics step.
  readings[readingIndex] = latestReading
  readingIndex = readingIndex + 1
end


-- Public interface:
M.getSensorData                                           = getSensorData
M.getLatest                                               = getLatest
M.incrementTimer                                          = incrementTimer
M.init                                                    = init
M.reset                                                   = reset
M.update                                                  = update

return M