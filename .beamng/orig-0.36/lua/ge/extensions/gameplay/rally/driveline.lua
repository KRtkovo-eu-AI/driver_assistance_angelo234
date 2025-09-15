-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local dequeue = require('dequeue')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local snaproadNormals = require('/lua/ge/extensions/gameplay/rally/snaproad/normals')

local C = {}
local logTag = ''

local startingMinDist = 4294967295

function C:init(missionDir)
  self.missionDir = missionDir
  self.points = nil

  self._cached_dist = nil
  self._cached_dist_race = nil

  self.radius = rallyUtil.default_waypoint_intersect_radius
end

function C:_setPoints(points)
  self.points = points
end

local function startsWithDoubleSlash(line)
  return line:match("^//") ~= nil
end

local function readFileToMemory(fname)
  local file = io.open(fname, "r")
  if not file then return nil end
  local content = file:read("*a")
  file:close()
  return content
end

local function splitIntoLines(content)
  local lines = {}
  for line in content:gmatch("[^\r\n]+") do
    table.insert(lines, line)
  end
  return lines
end

local function setPointPrevNext(points)
  for i,point in ipairs(points) do
    point.id = i
    if i > 1 then
      point.prev = points[i-1]
    end
    if i < #points then
      point.next = points[i+1]
    end
  end
end

local function setPointNormals(points)
  for i,point in ipairs(points) do
    point.normal = snaproadNormals.forwardNormalVec(point)
    point.pacenoteDistances = {}
    point.cachedPacenotes = {
      cs = nil,
      ce = nil,
      at = nil,
      auto_at = nil,
      half = nil,
    }
  end
end

function C:downsample(straightnessThreshold, maxDistance)
  if not self.points or #self.points < 3 then
    log('E', logTag, 'failed to downsample driveline, not enough points (points='..tostring(#self.points)..')')
    return nil
  end

  straightnessThreshold = straightnessThreshold or 0.980
  maxDistance = maxDistance or 10 -- Default maximum distance in meters

  local copyPoint = function(point)
    return {
      pos = vec3(point.pos),
      quat = quat(point.quat),
      ts = point.ts,
    }
  end

  local newPoints = {}
  table.insert(newPoints, copyPoint(self.points[1])) -- Always include the first point
  local lastKeptPointPos = self.points[1].pos

  -- Function to drop every N points
  local function dropEveryN(points, n)
    if not n or n <= 1 then return points end

    local result = {}
    for i = 1, #points do
      if i == 1 or i == #points or i % n ~= 0 then
        table.insert(result, points[i])
      end
    end
    return result
  end

  -- Drop every N points before applying the straightness algorithm
  -- This is a simple way to reduce the number of points
  local dropN = 2
  local initialPoints = dropEveryN(self.points, dropN)

  -- If we've dropped too many points, revert to original
  if #initialPoints < 3 then
    initialPoints = self.points
  end


  -- Iterate through points in sets of 3
  for i = 2, #initialPoints - 1 do
    local prev = initialPoints[i-1].pos
    local curr = initialPoints[i].pos
    local next = initialPoints[i+1].pos

    -- Calculate vectors between points
    local v1 = (curr - prev):normalized()
    local v2 = (next - curr):normalized()

    -- Calculate the dot product to determine the angle
    local dotProduct = v1:dot(v2)

    -- Calculate the distance between the previous kept point and the next point
    local distToNext = (next - lastKeptPointPos):length()

    -- If the path is not straight or the distance would be too large, include it
    if dotProduct < straightnessThreshold or distToNext > maxDistance then
      table.insert(newPoints, copyPoint(initialPoints[i]))
      lastKeptPointPos = curr
    end
  end

  table.insert(newPoints, copyPoint(initialPoints[#initialPoints])) -- Always include the last point

  setPointPrevNext(newPoints)
  setPointNormals(newPoints)

  local newDriveline = {}
  setmetatable(newDriveline, getmetatable(self))
  newDriveline:init(self.missionDir)
  -- newDriveline:load()
  newDriveline:_setPoints(newPoints)

  return newDriveline
end

function C:load()
  local t_start = rallyUtil.getTime()

  local fname = rallyUtil.drivelineFile(self.missionDir)
  local points = {}

  if not FS:fileExists(fname) then
    log('E', logTag, 'failed to load driveline file: '..fname)
    self.points = nil
    return false
  end

  local content = readFileToMemory(fname)
  if content then
    local lines = splitIntoLines(content)
    for _, line in ipairs(lines) do
      line = rallyUtil.trimString(line)
      if line == "" then
        -- print('empty line')
      elseif startsWithDoubleSlash(line) then
        -- print('commented line')
      else
        local obj = jsonDecode(line)

        obj.pos = vec3(obj.pos)
        obj.quat = quat(obj.quat)
        obj.prev = nil
        obj.next = nil
        obj.id = nil
        obj.partition = nil
        table.insert(points, obj)
      end
    end
  else
    log("E", logTag, "failed to read driveline file")
    return false
  end

  if #points < 3 then
    log('E', logTag, 'failed to load driveline, not enough points (points='..tostring(#points)..')')
    self.points = nil
    return false
  end

  setPointPrevNext(points)
  setPointNormals(points)

  local t_load = rallyUtil.getTime() - t_start

  log('I', logTag, 'loaded recce driveline in '.. string.format("%.3f", t_load)..'s with '..tostring(#points)..' points')
  self:_setPoints(points)

  return true
end

function C:loadFromRoute(route, N)
  N = N or 4  -- Default spacing of 1 meter if not provided

  -- Build the original list of points from route.path
  local points = {}
  for _, pt in ipairs(route.path) do
    local point = {
      pos = vec3(pt.pos),
      quat = quat(0, 0, 0, 1),
      prev = nil,
      next = nil,
      id = nil,
      partition = nil,
    }
    table.insert(points, point)
  end

  local refinedPoints = {}

  if #points > 0 then
    -- Always start with the first original point
    table.insert(refinedPoints, points[1])
  end

  -- Iterate over each pair of consecutive original points
  for i = 1, #points - 1 do
    local p0 = points[i].pos
    local p1 = points[i+1].pos
    local diff = p1 - p0
    local segmentLength = diff:length()

    if segmentLength == 0 then
      -- If the two points are coincident, just add the endpoint
      table.insert(refinedPoints, points[i+1])
    else
      -- Calculate the number of segments required such that each segment is <= N meters.
      local numSegments = math.ceil(segmentLength / N)
      local spacing = segmentLength / numSegments
      local dir = diff:normalized()

      -- Insert intermediate points evenly spaced between p0 and p1
      for j = 1, numSegments - 1 do
        local interpPos = p0 + dir * (j * spacing)
        local interpPoint = {
          pos = interpPos,
          quat = quat(0, 0, 0, 1),  -- Adjust if quaternion interpolation is required
          prev = nil,
          next = nil,
          id = nil,
          partition = nil,
        }
        table.insert(refinedPoints, interpPoint)
      end

      -- Finally, add the original endpoint of the segment
      table.insert(refinedPoints, points[i+1])
    end
  end

  -- Update id, prev, and next pointers for the refined list
  for i, point in ipairs(refinedPoints) do
    point.id = i
    point.prev = (i > 1) and refinedPoints[i - 1] or nil
    point.next = (i < #refinedPoints) and refinedPoints[i + 1] or nil
  end

  self:_setPoints(refinedPoints)
  return true
end

local prevMouseDown = false

function C:drawDebugDriveline(drawLabels, mouseInfo)
  if not self.points then return end

  -- dump(mouseInfo)

  local mousePos = mouseInfo.valid and mouseInfo.rayCast and mouseInfo.rayCast.pos or nil

  local mouseDown = mouseInfo.down

  local clickTick = false

  if mouseDown and not prevMouseDown then
    clickTick = true
  end

  prevMouseDown = mouseDown

  drawLabels = drawLabels or false

  local clr = cc.recce_driveline_clr
  -- local alpha_shape = cc.recce_alpha
  local alpha_shape = 0.3
  local radius = cc.snaproads_radius_recce

  local clr_shape = cc.clr_red
  local plane_radius = self.radius
  local midWidth = plane_radius * 2
  local clr_txt = cc.clr_black

  for _,point in ipairs(self.points) do
    -- local cached = point.cachedPacenotes
    -- if cached.half then
      -- color = cc.clr_blue
      -- radius = 0.75
    -- end

    local pos = point.pos
    if mousePos then
      local dist = vec3(pos):distance(mousePos)
      -- local dist = 100
      if dist < radius then
        clr = cc.clr_white
        debugDrawer:drawTextAdvanced(
          pos,
          String("Click to copy TS to clipboard"),
          ColorF(clr_txt[1], clr_txt[2], clr_txt[3], 1),
          true,
          false,
          ColorI(clr[1]*255, clr[2]*255, clr[3]*255, 255)
        )
        if clickTick then
          im.SetClipboardText(tostring(point.ts))
        end
      else
        clr = cc.clr_red
      end
    else
      clr = cc.clr_red
    end

    debugDrawer:drawSphere(
      pos,
      radius,
      ColorF(clr[1], clr[2], clr[3], alpha_shape)
    )

    if drawLabels then
      debugDrawer:drawTextAdvanced(
        pos,
        String(dumps(point.ts)),
        ColorF(clr_txt[1], clr_txt[2], clr_txt[3], 1),
        true,
        false,
        ColorI(clr[1]*255, clr[2]*255, clr[3]*255, 255)
      )
    end

    -- local side = point.normal:cross(vec3(0,0,1)) * (plane_radius - (midWidth / 2))
    --
    -- -- this square prism is the intersection "plane" of the point.
    -- debugDrawer:drawSquarePrism(
    --   point.pos + side,
    --   point.pos + 0.25 * point.normal + side,
    --   Point2F(5, midWidth),
    --   Point2F(0, 0),
    --   ColorF(clr_shape[1], clr_shape[2], clr_shape[3], alpha_shape)
    -- )
  end
end

function C:findNearestPoint(srcPos, startPoint_i, reverse, limit)
  startPoint_i = startPoint_i or 1
  reverse = reverse or false
  limit = limit or false

  -- if the consecutive number of points, when compared, have increasing distance, call it quits.
  local searchConfidenceThreshold = 100
  local increasingDistSearches = 0

  local minDist = startingMinDist
  local closestPoint = nil

  -- for _,point in ipairs(self.points) do
  --   local pos = vec3(point.pos)
  --   local dist = (pos - srcPos):length()
  --   if dist < minDist then
  --     minDist = dist
  --     closestPoint = point
  --   end
  -- end

  local incr = (reverse and -1) or 1
  local end_i = (reverse and 1) or #self.points

  for i=startPoint_i,end_i,incr do
    local point = self.points[i]
    local pos = vec3(point.pos)
    local dist = (pos - srcPos):length()
    if dist < minDist then
      minDist = dist
      closestPoint = point
    else
      increasingDistSearches = increasingDistSearches + 1
    end

    if limit and increasingDistSearches > searchConfidenceThreshold then
      break
    end
  end

  return closestPoint
end

function C:calculateDistance(point1, point2)
  local pos1 = vec3(point1.pos)
  local pos2 = vec3(point2.pos)
  return (pos2 - pos1):length()
end

function C:length()
  if self._cached_dist then
    return self._cached_dist
  end

  self._cached_dist = 0
  local prevPoint = self.points[1]

  for _,point in ipairs(self.points) do
    self._cached_dist = self._cached_dist + self:calculateDistance(prevPoint, point)
    prevPoint = point
  end

  return self._cached_dist
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
