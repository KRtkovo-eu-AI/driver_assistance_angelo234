-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This module provides two implementations of the Ramer–Douglas–Peucker algorithm, used to simplify polylines with excessive points.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local defaultTol = 1.0 -- The default tolerance used for the RDP algorithms.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module state.
local simplifiedIdxs = {}


-- Performs a Ramer-Douglas-Peucker simplification of the given polyline. Points-based version.
local function rdp(points, epsilonIn)
  if #points <= 2 then
    return points -- No points to simplify.
  end

  -- Find the point with the maximum squared distance from the line between the first and last point.
  local maxSqDist = 0
  local index = 0
  for i = 2, #points - 1 do
    local dist = points[i]:squaredDistanceToLine(points[1], points[#points])
    if dist > maxSqDist then
      index = i
      maxSqDist = dist
    end
  end

  -- If max distance is greater than epsilon, recursively simplify.
  local epsilon = epsilonIn or defaultTol
  if maxSqDist > epsilon then
    local left = rdp({ unpack(points, 1, index) }, epsilon)
    local right = rdp({ unpack(points, index, #points) }, epsilon)
    table.remove(right, 1) -- Remove duplicate point at the junction.
    for _, p in ipairs(right) do
      table.insert(left, p)
    end
    return left
  end
  return { points[1], points[#points] } -- No point is far enough, keep only endpoints.
end

-- Simplifies a path using the recursive Ramer–Douglas–Peucker algorithm. Index-based version.
local function rdpIndexBased(startIdx, endIdx, simplifiedIdxList, nodes, tol)
  if endIdx <= startIdx + 1 then
    return -- No points to simplify.
  end
  local maxDist = -1
  local index = startIdx
  for i = startIdx + 1, endIdx - 1 do
    local dist = nodes[i]:distanceToLineSegment(nodes[startIdx], nodes[endIdx])
    if dist > maxDist then
      maxDist = dist
      index = i
    end
  end
  if maxDist > tol then
    rdpIndexBased(startIdx, index, simplifiedIdxList, nodes, tol)
    table.insert(simplifiedIdxList, index)
    rdpIndexBased(index, endIdx, simplifiedIdxList, nodes, tol)
  end
end

-- Simplifies an ordered polyline using the Ramer–Douglas–Peucker algorithm, along with the corresponding widths.
local function simplify(nodes, widths, tol)
  if #nodes < 3 then
    return nodes, widths
  end
  table.clear(simplifiedIdxs)
  simplifiedIdxs[1] = 1
  rdpIndexBased(1, #nodes, simplifiedIdxs, nodes, tol or defaultTol)
  table.insert(simplifiedIdxs, #nodes)
  table.sort(simplifiedIdxs)
  local newNodes, newWidths = {}, {}
  for _, i in ipairs(simplifiedIdxs) do
    table.insert(newNodes, nodes[i])
    table.insert(newWidths, widths[i])
  end
  return newNodes, newWidths
end

-- Simplifies an ordered polyline using the Ramer–Douglas–Peucker algorithm, along with the corresponding widths and normals.
local function simplifyWithNormals(nodes, widths, normals, tol)
  if #nodes < 3 then
    return nodes, widths, normals
  end
  table.clear(simplifiedIdxs)
  simplifiedIdxs[1] = 1
  rdpIndexBased(1, #nodes, simplifiedIdxs, nodes, tol or defaultTol)
  table.insert(simplifiedIdxs, #nodes)
  table.sort(simplifiedIdxs)
  local newNodes, newWidths, newNormals = {}, {}, {}
  for _, i in ipairs(simplifiedIdxs) do
    table.insert(newNodes, nodes[i])
    table.insert(newWidths, widths[i])
    table.insert(newNormals, normals[i])
  end
  return newNodes, newWidths, newNormals
end


-- Public interface.
M.rdp =                                                 rdp
M.simplify =                                            simplify
M.simplifyWithNormals =                                 simplifyWithNormals

return M