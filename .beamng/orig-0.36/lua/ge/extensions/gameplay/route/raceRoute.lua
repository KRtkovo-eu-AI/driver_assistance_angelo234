-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt


-- Experimental version of route.lua for racing.
-- The goal is to merge with the original route.lua as much as possible.

local C = {}
local onPathDist = 15
local logTag = ''

function C:init(removeFirst, useMapPathfinding, closeDistSquared, logTag)
  logTag = logTag or ''

  -- the defaults for these are set like route.lua had them before rally/race related changes.
  if removeFirst == nil then
    self.removeFirst = true
  else
    self.removeFirst = removeFirst
  end

  if useMapPathfinding == nil then
    self.useMapPathfinding = true
  else
    self.useMapPathfinding = useMapPathfinding
  end

  self.closeDistSquared = closeDistSquared or 1

  -- log('D', logTag, string.format("RaceRoute:init with removeFirst=%s useMapPathfinding=%s closeDistSquared=%s", tostring(self.removeFirst), tostring(self.useMapPathfinding), tostring(self.closeDistSquared)))

  self.path = {}
  self.originalFixedPositions = nil
  self.dirMult = 1e3
  self.callbacks = {
    onPointProcessed = nil,
    shouldPointBeFixedForRecalc = nil,
    onRouteShortened = nil,
    onMetadataMerge = nil,
  }
end

-- function C:clone()
--   local clone = {}
--   setmetatable(clone, C)
--   clone.path = deepcopy(self.path)
--   clone.originalPositions = deepcopy(self.originalPositions)

--   clone.cutOffDrivability = self.cutOffDrivability
--   clone.penaltyAboveCutoff = self.penaltyAboveCutoff
--   clone.penaltyBelowCutoff = self.penaltyBelowCutoff
--   clone.wD = self.wD
--   clone.wZ = self.wZ

--   return clone
-- end

function C:setRouteParams(cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wD, wZ)
  self.cutOffDrivability = cutOffDrivability
  self.dirMult = dirMult or 1e3
  self.penaltyAboveCutoff = penaltyAboveCutoff
  self.penaltyBelowCutoff = penaltyBelowCutoff
  self.wD = wD
  self.wZ = wZ
  --log("I","",dumps("setRouteParams", cutOffDrivability, dirMult, penaltyAboveCutoff, penaltyBelowCutoff, wD, wZ))
end

local function fixStartEnd(p, a, b, resetMetadata, debug, theSelf)
  debug = debug or false
  resetMetadata = resetMetadata or false
  profilerPushEvent("Route - fixStartEnd")

  if debug and not theSelf.debugFixStartEnd_a then
    theSelf.debugFixStartEnd_a = a
    theSelf.debugFixStartEnd_b = b
    theSelf.debugFixStartEnd_p = p
  end

  local xnorm = p.pos:xnormOnLine(a.pos, b.pos)
  if debug then
    theSelf.debugFixStartEnd_xnorm = xnorm
  end

  if xnorm > 0 then
    a.pos = lerp(a.pos, b.pos, xnorm)
    a.wp = nil
    if resetMetadata then
      a.metadata = nil
    end
  end

  a.distToTarget = a.pos:distance(b.pos) + (b.distToTarget or 0)

  profilerPopEvent("Route - fixStartEnd")
end

function C:stepAhead(stepDist, reset) -- returns data from a distance along the route (and also saves the last position, to optimize for looping)
  if not self.lastDist or reset then
    self.lastIndexAtDist = 1
    self.lastDist = 0
  end
  if not self.path[2] then return end

  local pathCount = #self.path
  stepDist = stepDist + self.lastDist
  for i = self.lastIndexAtDist, pathCount - 1 do
    local v1, v2 = self.path[i], self.path[i + 1]
    local length = v1.distToTarget - v2.distToTarget

    if stepDist > length then
      stepDist = stepDist - length
    else
      self.lastDist = stepDist
      self.lastIndexAtDist = i
      local xnorm = clamp(stepDist / (length + 1e-30), 0, 1)
      return {n1 = v1.wp, n2 = v2.wp, idx = i, pos = linePointFromXnorm(v1.pos, v2.pos, xnorm), xnorm = xnorm}
    end
  end

  return {n1 = self.path[pathCount - 1].wp, n2 = self.path[pathCount].wp, idx = pathCount - 1, pos = self.path[pathCount].pos, xnorm = 1}
end

function C:calcDistance()
  local dist = 0
  for i = #self.path, 2, -1 do
    self.path[i].distToTarget = dist
    dist = dist + self.path[i].pos:distance(self.path[i - 1].pos)
  end
  self.path[1].distToTarget = dist

  return dist
end

function C:setupPath(fromPos, toPos)
  self:setupPathMulti({fromPos, toPos})
end

function C:setupPathMultiWaypoints(wpList)
  profilerPushEvent("Route - setupPathMultiWaypoints")
  for i = 1, #wpList-1 do
    local path = map.getPath(wpList[i], wpList[i + 1], self.cutOffDrivability, self.dirMult, self.penaltyAboveCutoff, self.penaltyBelowCutoff)
    local pathLen = #self.path
    for j, pwp in ipairs(path) do
      if not self.path[pathLen] or self.path[pathLen].wp ~= pwp then
        table.insert(self.path, {pos = map.getMap().nodes[pwp].pos, wp = pwp, linkCount = map.getNodeLinkCount(pwp)})
      end
    end
  end

  self:calcDistance()
  profilerPopEvent("Route - setupPathMultiWaypoints")
end

function C:setupPathMulti(positions)
  profilerPushEvent("Route - setupPathMulti")
  local positionsWithMetadata = {}
  for i, pos in ipairs(positions) do
    positionsWithMetadata[i] = { pos = pos }
  end
  self:setupPathMultiWithMetadata(positionsWithMetadata)
  profilerPopEvent("Route - setupPathMulti")
end

-- Parameters:
-- * positions (array): table of tables with the following fields:
--   Required fields:
--   * pos (vec3): position of the node
--   Optional fields:
--   * mergePriority (float): used to merge nodes into one. lower values are merged into higher values.
--   * metadata (table): used to store additional data for each node. merging
--                       behavior is defined by the metadata. metadata with same keys will have the
--                       higher mergePriority value kept.
--   Internally added fields: wp, fixed
function C:setupPathMultiWithMetadata(positions)
  -- log('D', '', "===========================================================================")
  -- log('D', '', "setupPathMultiWithMetadata")
  profilerPushEvent("Route - setupPathMultiWithMetadata")
  table.clear(self.path)
  -- Add the first position
  table.insert(self.path, positions[1])

  for i = 1, #positions-1 do
    -- Get the current and next positions
    local from, to = positions[i], positions[i+1]

    if self.useMapPathfinding then
      -- Get the path from the current position to the next position
      profilerPushEvent("Route - get point to point")
      local path = map.getPointToPointPath(from.pos, to.pos, self.cutOffDrivability, self.dirMult, self.penaltyAboveCutoff, self.penaltyBelowCutoff, self.wD, self.wZ)
      profilerPopEvent("Route - get point to point")
      local lastIdx = #self.path
      for i, p in ipairs(path) do
        -- Check if the waypoint ID starts with "DR"
        -- if not string.find(p, "^DR") then
          -- log('E', '', "Waypoint ID '" .. tostring(p) .. "' does not start with 'DR'")
        -- end

        -- table.insert(self.path, {pos = map.getMap().nodes[p].pos, wp = p, metadata = {stableId = p}, linkCount = map.getNodeLinkCount(p)})
        table.insert(self.path, {pos = map.getMap().nodes[p].pos, wp = p, linkCount = map.getNodeLinkCount(p)})
      end

      -- Add the last point of the current path segment
      -- This ensures that the last point of the current path segment is fixed
      table.insert(self.path, {pos = to.pos, wp = nil, fixed = true, metadata = to.metadata })

      -- Adjust the first few points of the current path segment
      -- This ensures smooth transitions between fixed waypoints by adjusting the positions
      -- of the first few nodes after a fixed waypoint
      if #self.path >= 3 and #path >= 2 then
        fixStartEnd(self.path[lastIdx], self.path[lastIdx+1], self.path[lastIdx+2])
      end

      -- Adjust the last few points of the current path segment
      -- This ensures smooth transitions between fixed waypoints by adjusting the positions
      -- of the last few nodes before reaching the next fixed waypoint
      if #self.path >= 3 and #path >= 2 then
        fixStartEnd(self.path[#self.path], self.path[#self.path-1], self.path[#self.path-2])
      end
    else
      table.insert(self.path, {pos = to.pos, wp = to.wp or nil, fixed = true, metadata = to.metadata })
    end
  end

  if not self.removeFirst then
    self.path[1].fixed = true
  end

  -- local function retainDebugInfo(points)
  --   local newPoints = {}
  --   for i, point in ipairs(points) do
  --     table.insert(newPoints, {
  --       pos = vec3(point.pos),
  --     })
  --   end
  --   return newPoints
  -- end

  -- self.debugPath = retainDebugInfo(self.path)

  -- self.debugMergePathSample = {}

  -- merge too-close nodes into one and preserve fields wp, fixed, mergePriority, metadata
  local newPath = {self.path[1]}
  local last = self.path[1]
  for i = 2, #self.path do
    local cur = self.path[i]
    if cur.pos:squaredDistance(last.pos) <= self.closeDistSquared then
      -- log("D", '', 'squaredDistance is close')
      if self.callbacks.onMetadataMerge then
        -- log("D", '', 'has onMetadataMerge')
        -- merge
        last.merges = (last.merges or 0) + 1
        cur.merges = (cur.merges or 0) + 1
        local newPoint, newMetadata = self.callbacks.onMetadataMerge(i, last, cur)

        if newPoint then
          -- log("D", '', 'onMetadataMerge returned newPoint')
          -- if last.metadata and cur.metadata then
          --   log('D', '', string.format("merging last.stableId=%s cur.stableId=%s", last.metadata and last.metadata.stableId or 'nil', cur.metadata and cur.metadata.stableId or 'nil'))
          -- else
          --   log('D', '', string.format("merging last.wp=%s cur.wp=%s", tostring(last.wp), tostring(cur.wp)))
          -- end
          last.pos = newPoint.pos
          last.wp = newPoint.wp
          last.fixed = newPoint.fixed
          last.metadata = newMetadata
        else
          -- log("D", '', 'onMetadataMerge skip merging')
          -- skip merging
          last = cur
          table.insert(newPath, cur)
        end
      else
        -- log("D", '', 'NO onMetadataMerge - default merge behavior')
        -- default merge behavior. basically merges points backwards.
        last.wp = last.wp or cur.wp
        last.fixed = last.fixed or cur.fixed
      end
    else
      -- log("D", '', 'skip merging')
      -- skip merging
      last = cur
      table.insert(newPath, cur)
    end
    -- log('D', '', '----------------------------')
  end

  self.path = newPath
  -- self.debugPostMergePath = retainDebugInfo(self.path)

  if #self.path >= 3 then
    -- fix the start.
    fixStartEnd(self.path[1], self.path[2], self.path[3], false, true, self)
    --merge first two nodes if already on path
    if self.removeFirst and self.path[1].pos:squaredDistance(self.path[2].pos) < onPathDist * onPathDist then
      -- table.remove(self.path, 1)
    end
  end

  -- self.debugPostFixStartEnd1 = retainDebugInfo(self.path)

  -- fix the end.
  if #self.path >= 3 then
    fixStartEnd(self.path[#self.path], self.path[#self.path-1], self.path[#self.path-2])
  end

  -- self.debugPostFixStartEnd2 = retainDebugInfo(self.path)

  self:calcDistance()

  if self.callbacks.onPointProcessed then
    for i, point in ipairs(self.path) do
      self.callbacks.onPointProcessed(point)
    end
  end

  if not self.originalFixedPositions then
    self.originalFixedPositions = {}
    for i, point in ipairs(self.path) do
      if point.fixed then
        -- dumpz(point, 2)
        table.insert(self.originalFixedPositions, {
          pos = vec3(point.pos),
          linkCount = point.linkCount,
          fixed = point.fixed,
          mergePriority = point.mergePriority,
          metadata = point.metadata,
          distToTarget = point.distToTarget,
          wp = point.wp
        })
      end
    end
  end

  profilerPopEvent("Route - setupPathMultiWithMetadata")
end

function C:clear()
  table.clear(self.path)
  -- table.clear(self.originalFixedPositions)
end

function C:getNextFixedWP()
  for _, wp in ipairs(self.path) do
    if wp.fixed then return wp.pos end
  end
end

function C:recalculateRoute(startPos)
  profilerPushEvent("Route - recalculateRoute")
  local fixedWps = {}
  table.insert(fixedWps, { pos = startPos })
  for _, wp in ipairs(self.path) do
    if wp.fixed then
      table.insert(fixedWps, {
        pos = wp.pos,
        metadata = wp.metadata,
        wp = wp.wp
      })
    end
  end
  self:setupPathMultiWithMetadata(fixedWps)
  extensions.hook('onRecalculatedRoute')
  profilerPopEvent("Route - recalculateRoute")
end

-- The difference with recalculateRoute() is that this function uses the
-- original fixed positions, ie the original unshorted self.path points,
-- instead of the remaining points in self.path after repeated shortening.
--
-- It also uses a callback to determine whether a point should be considered fixed for recalculation.
--
-- The overall behavior is that the route is recalculatd along the same path as when the route created.
function C:recalculateRouteWithOriginalPositions(startPos)
  profilerPushEvent("Route - recalculateRouteWithOriginalPositions")
  local fixedWps = {}
  table.insert(fixedWps, { pos = startPos })
  local callbackResult = nil
  for i, wp in ipairs(self.originalFixedPositions) do
    if self.callbacks.shouldPointBeFixedForRecalc then
      -- dumpz(wp, 2)
      callbackResult = self.callbacks.shouldPointBeFixedForRecalc(wp)
      local source = wp.metadata and wp.metadata.source
      local stableId = wp.metadata and wp.metadata.stableId
      -- local msg = ""
      -- if stableId then
        -- msg = string.format('callbackResult=%s source="%s" stableId="%s"', tostring(callbackResult), source, stableId)
      -- else
        -- log('E', '', "no stableId for original point")
      -- end
      -- log('D', '', string.format('recalculateRouteWithOriginalPositions i=%d %s', i, msg))
    end

    -- always add the final original point.
    if i == #self.originalFixedPositions or callbackResult == true then
      table.insert(fixedWps, {
        pos = wp.pos,
        metadata = wp.metadata,
        wp = wp.wp
      })
    end
  end
  self:setupPathMultiWithMetadata(fixedWps)
  extensions.hook('onRecalculatedRoute')
  profilerPopEvent("Route - recalculateRouteWithOriginalPositions")
end

--local offPathDist = 25
function C:getPositionOffset(currentPos)
  profilerPushEvent("Route - getPositionOffset")
  -- go through all segments and check where we are on that line
  local minDistance, totalMinDist = math.huge, math.huge
  local lowIdx = 0
  for i = 1, #self.path-1 do
    local distSq = currentPos:squaredDistanceToLineSegment(self.path[i].pos, self.path[i+1].pos)
    totalMinDist = math.min(totalMinDist, distSq)

    if distSq > minDistance then break end

    if distSq < onPathDist * onPathDist then
      minDistance = distSq
      lowIdx = i
    elseif self.path[i].fixed then
      break
    end
  end
  profilerPopEvent("Route - getPositionOffset")
  return lowIdx, math.sqrt(totalMinDist)
end

function C:shortenPath(idx)
  profilerPushEvent("Route - shortenPath")
  for i = 2, idx do
    local removed = self.path[1]
    table.remove(self.path, 1)
    if self.callbacks.onRouteShortened then
      self.callbacks.onRouteShortened(removed)
    end
  end
  profilerPopEvent("Route - shortenPath")
end

function C:trackVehicle(veh) return self:updatePathForPos(veh:getPosition()) end
function C:trackCamera() return self:updatePathForPos(core_camera.getPosition()) end
function C:trackPosition(pos) return self:updatePathForPos(pos) end

function C:updatePathForPos(pos)
  profilerPushEvent("Route - updatePathForPos")
  -- are we there yet? no path or only one element remaining?
  if not next(self.path) or #self.path < 2 then
    self.done = true
    profilerPopEvent("Route - updatePathForPos")
    return
  end

  -- did we pass any of the first positions, moving forward on the track?
  local idx, minDistance = self:getPositionOffset(pos)
  --
  if idx == 0 and minDistance >= onPathDist then
    -- re-do route
    self:recalculateRoute(pos)
  elseif idx >= 2 then
    -- if we passed the first segment, truncate path accordingly
    self:shortenPath(idx)
  end
  local resetMetadata = true
  fixStartEnd({pos = pos}, self.path[1], self.path[2], resetMetadata)

  profilerPopEvent("Route - updatePathForPos")

  return idx, minDistance
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end