-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for fitting a polyline to an unordered set of points.

local M = {}

-- Module constants.
local huge = math.huge


-- Computes the indices of the best start and end points of the polyline.
-- [Finds the  pair with maximum distance, to identify good start/end.]
local function computeStartEndIndices(remaining)
  local maxDistSq = -1
  local startIdx, endIdx = 1, 1
  for i = 1, #remaining do
    local pi = remaining[i].obj:getPosition()
    for j = i + 1, #remaining do
      local pj = remaining[j].obj:getPosition()
      local distSq = (pj - pi):squaredLength()
      if distSq > maxDistSq then
        maxDistSq = distSq
        startIdx = i
        endIdx = j
      end
    end
  end
  return startIdx, endIdx
end

-- Chooses the "lowest Y then lowest X" of the two endpoints to start.
local function chooseStartPoint(remaining, startIdx, endIdx)
  local ptA = remaining[startIdx].obj:getPosition()
  local ptB = remaining[endIdx].obj:getPosition()
  local initialIdx
  if ptA.y < ptB.y or (ptA.y == ptB.y and ptA.x < ptB.x) then
    initialIdx = startIdx
  else
    initialIdx = endIdx
  end
  return initialIdx
end

-- Nearest-neighbour chaining.
local function chainObjects(current, remaining)
  local ordered = {}
  table.insert(ordered, current)
  while #remaining > 0 do
    local lastPos = current.obj:getPosition()
    local bestIdx, bestDistSq = nil, huge
    for i, cand in ipairs(remaining) do
      local candPos = cand.obj:getPosition()
      local distSq = (candPos - lastPos):squaredLength()
      if distSq < bestDistSq then
        bestIdx = i
        bestDistSq = distSq
      end
    end
    current = table.remove(remaining, bestIdx)
    table.insert(ordered, current)
  end
  return ordered
end

-- Fits a polyline to an unordered set of points.
local function fitPoly(sleeperComponents)
  -- Make a manual copy of components for spatial ordering.
  local remaining = {}
  for i = 1, #sleeperComponents do
    table.insert(remaining, sleeperComponents[i])
  end

  -- Find pair with maximum distance to identify good start/end.
  local startIdx, endIdx = computeStartEndIndices(remaining)

  -- Choose the "lowest Y then lowest X" of the two endpoints to start.
  local initialIdx = chooseStartPoint(remaining, startIdx, endIdx)
  local current = table.remove(remaining, initialIdx)

  -- Nearest-neighbour chaining.
  local ordered = chainObjects(current, remaining)

  -- Assign ordering index.
  for i, comp in ipairs(ordered) do
    comp.order = i
  end

  return ordered
end


-- Public interface.
M.fitPoly =                                             fitPoly

return M
