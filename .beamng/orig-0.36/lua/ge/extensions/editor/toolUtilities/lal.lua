-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local minSplineDivisions = 10 -- The minimum number of subdivisions to use for a spline.
local intersectionTol = 1e-3

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require('editor/toolUtilities/geom')

-- Module constants.
local min, max, floor = math.min, math.max, math.floor
local cos, sin, acos = math.cos, math.sin, math.acos
local oneThird, twoThirds = 0.33333333333333333333, 0.66666666666666666667
local up = vec3(0, 0, 1)

-- Module state.
local tmp0, tmp1, tmp2 = vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0)

-- Computes the (small) angle between two vectors of arbitrary length, in radians.
local function angleBetweenVecs(a, b) return acos(a:normalized():dot(b:normalized())) end

-- Rotates vector v around unit axis k, by angle theta (in radians).
-- [This function uses the standard Rodrigues formula].
local function rotateVecAroundAxis(v, k, theta)
  local c = cos(theta)
  return v * c + k:cross(v) * sin(theta) + k * k:dot(v) * (1.0 - c)
end

-- Computes the 2D incircle of a triangle (from the three given triangle vertices v0-v1-v2).
-- Returns the center and radius of the incircle.
local function computeIncircle_2D(v0, v1, v2)
  tmp0:set(v0.x, v0.y, 0.0) -- Ensure the input points are 2D.
  tmp1:set(v1.x, v1.y, 0.0)
  tmp2:set(v2.x, v2.y, 0.0)
  local a, b, c = tmp2:distance(tmp1), tmp2:distance(tmp0), tmp1:distance(tmp0)
  local center = vec3((a * v0.x + b * v1.x + c * v2.x), (a * v0.y + b * v1.y + c * v2.y), 0.0) / (a + b + c)
  return center, center:distanceToLineSegment(v0, v1)
end

-- Finds the intersection between line segment (a->b) and circle (c, r).
-- This function either returns the point of intersection, or nil if there is no intersection.
local function intLineSegAndCircle(a, b, c, r)
  local rayDir = b - a
  rayDir:normalize()
  local q1, q2 = intersectsRay_Sphere(a, rayDir, c, r)
  local isct1, isct2 = a + q1 * rayDir, a + q2 * rayDir
  if isct1:squaredDistanceToLineSegment(a, b) < intersectionTol then
    return isct1
  elseif isct2:squaredDistanceToLineSegment(a, b) < intersectionTol then
    return isct2
  end
  return nil
end

-- Interpolates the points with Line-Arc-Line sequences.
local function LALInterpolate(group)
  local nodes = group.nodes
  if #nodes < 3 then
    geom.catmullRomRaycast(group, minSplineDivisions) -- If there are less than 3 nodes, just fit with the standard method.
    return
  end

  -- First compute the arc discretisations at each intermediate node.
  local userRad = 0.1 -- TODO: make this a slider maybe.
  local numNodes = #nodes
  local arcs = table.new(numNodes - 2, 0)
  for i = 2, numNodes - 1 do
    -- Test the angle between the incoming and outgoing edges.
    local p0, p1, p2 = nodes[i - 1], nodes[i], nodes[i + 1]
    if angleBetweenVecs(p1 - p0, p2 - p1) < 0.5 then
      arcs[i - 1] = { p1 } -- If the angle between the incoming and outgoing edges is less than 1 degree, then we can just use a straight line.
    else
      -- Compute the incircle of the triangle formed by the current node and appropriate points on the incoming and outgoing edges.
      local pS = lerp(p0, p1, userRad)
      local pE = lerp(p2, p1, userRad)
      local center, radius = computeIncircle_2D(pS, p1, pE)
      radius = radius + 1e-2 -- Add a small margin to the radius, to ensure the arc is always intersected.

      -- Compute the two intersection points on each of the triangle edges which touch the node.
      -- [At these intersections, the linear segment tangents match the arc tangent, so continuity exists].
      local pStart, pEnd = intLineSegAndCircle(pS, p1, center, radius), intLineSegAndCircle(p1, pE, center, radius)

      -- Determine the angular domain for the arc.
      -- [The sign of theta depends on the sign of the distance to the lateral plane of the current point.]
      local v1_2D = pStart - center
      local tgt_2D = pE - pS
      tgt_2D.z = 0.0
      tgt_2D:normalize()
      local signFac = -sign2(tgt_2D:cross(up):dot(pEnd - p1))
      local theta = angleBetweenVecs(v1_2D, pEnd - center) * signFac
      local vStart_2D, vEnd_2D = rotateVecAroundAxis(v1_2D, up, oneThird * theta), rotateVecAroundAxis(v1_2D, up, twoThirds * theta)
      local u2_2D, u3_2D = center + vStart_2D, center + vEnd_2D
      local arcAngle = angleBetweenVecs(u2_2D - center, u3_2D - center) * signFac

      -- Compute the arc section.
      -- [This is done before computing the Clothoid sections].
      local splineGran = 10
      local splineGranInv = 1.0 / splineGran
      local arc = table.new(splineGran + 1, 0)
      for j = 0, splineGran do
        arc[j + 1] = center + rotateVecAroundAxis(vStart_2D, up, j * splineGranInv * arcAngle)
        arc[j + 1].z = 0.0
      end
      arcs[i - 1] = arc
    end
  end

  -- Second, compute the discretized line segments which join the arcs.
  local lineSegs = table.new(#arcs + 1, 0)
  for i = 1, #arcs + 1 do
    local p1, p2 = nil, nil
    if i == 1 then
      p1 = nodes[1]
      p2 = arcs[1][1]
    elseif i == #arcs + 1 then
      p1 = arcs[#arcs][#arcs[#arcs]]
      p2 = nodes[#nodes]
    else
      p1 = arcs[i - 1][#arcs[i - 1]]
      p2 = arcs[i][1]
    end
    local lineSeg, ctr = table.new(11, 0), 1
    for j = 0.0, 1.0, 0.1 do
      lineSeg[ctr] = lerp(p1, p2, j)
      ctr = ctr + 1
    end
    lineSegs[i] = lineSeg
  end

  -- Third, create the spline by round-robining through the line segments and arcs, in turn.
  local divPoints, divWidths, discMap = group.divPoints, group.divWidths, group.discMap
  table.clear(divPoints)
  table.clear(divWidths)
  table.clear(discMap)
  discMap[1] = 1
  local ctr = 1
  local nextLineIdx, nextArcIdx, mapCtr = 1, 1, 2
  local i = 1
  while nextLineIdx <= #lineSegs or nextArcIdx <= #arcs do
    if i % 2 == 1 then
      -- Index is odd, so concatenate the next line segment.
      local lineSeg = lineSegs[nextLineIdx]
      for j = 1, #lineSeg do
        divPoints[ctr] = lineSeg[j]
        divWidths[ctr] = 10.0 -- TODO: get the width of the line segment.
        ctr = ctr + 1
      end
      nextLineIdx = nextLineIdx + 1
    else
      -- Index is even, so concatenate the next arc.
      local arc = arcs[nextArcIdx]
      for j = 1, #arc do
        divPoints[ctr] = arc[j]
        divWidths[ctr] = 10.0 -- TODO: get the width of the arc.
        if j == floor(#arc * 0.5) then
          discMap[mapCtr] = ctr -- Use the arc midpoint as the map entry for the next node.
          mapCtr = mapCtr + 1
        end
        ctr = ctr + 1
      end
      nextArcIdx = nextArcIdx + 1
    end
    i = i + 1
  end
  discMap[mapCtr] = ctr - 1

  -- Compute the local Frenet frame at each division point.
  local tangents, binormals, normals = group.tangents, group.binormals, group.normals
  table.clear(tangents)
  table.clear(binormals)
  table.clear(normals)
  for i = 1, #divPoints do
    local p0, p1 = divPoints[max(1, i - 1)], divPoints[min(#divPoints, i + 1)]
    tangents[i] = p1 - p0
    tangents[i]:normalize()
    binormals[i] = tangents[i]:cross(up)
    binormals[i]:normalize()
    normals[i] = binormals[i]:cross(tangents[i])
  end
end


-- Public interface.
M.LALInterpolate =                                      LALInterpolate

return M