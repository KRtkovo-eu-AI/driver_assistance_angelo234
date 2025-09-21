-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class containing geometric functions, for use with spline-editing tools.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local maxRaycastDist = 10000 -- The maximum distance for the camera -> mouse ray, in meters.
local zExtra = 3.0 -- The extra height to add to the points when raycasting to the surface below.
local zRaycastOffset = 0.05 -- The z-offset to add to the points when raycasting to the surface below.
local defaultMinNumDivisions = 10 -- The minimum number of subdivisions used when interpolating the secondary geometry properties of a spline.
local maxDivisionSpacing = 20.0 -- The maximum allowed spacing between consecutive division points, in meters (determines the minimum number of subdivisions).

local endMargin = 10 -- The margin to add to the first and last points of a polygon, in meters.

local minScale = 0.01 -- The minimum scale for the adaptive sampling of a spline.
local kappaSensitivity = 150 -- The sensitivity of the adaptive sampling of a spline to curvature.

local weightLength = 1.0 -- Weights used to score the decal roads when choosing the master road.
local weightWidth  = 5.0 -- prioritise wider roads more than longer roads.

local maxRayDist = 10000 -- The maximum distance for the camera -> mouse ray, in meters.
local mouseToNodetol = 0.84 -- The distance tolerance used when testing if the mouse is close to a node, in meters.                                                                        -- The sq. distance tolerance used when testing if the mouse is close to a spline.
local intermediateTolSq = 1 -- The sq. distance tolerance used when testing if the mouse is close to a spline.

local barScale = 0.2 -- The scale factor for the bar points (height = barScale * velocity).

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local util = require('editor/toolUtilities/util')

-- Module constants.
local min, max, floor = math.min, math.max, math.floor
local sin, cos, acos, pi = math.sin, math.cos, math.acos, math.pi
local random, huge = math.random, math.huge
local splineGranInv = 1.0 / defaultMinNumDivisions
local up, down = vec3(0, 0, 1), vec3(0, 0, -1)
local zeroQuat = quat(0, 0, 0, 1)

-- Module state.
local pThis, pLast, mousePos2D = vec3(), vec3(), vec3()
local tmp0, tmp1, tmp2, tmp3, tmp4, tmpTangent = vec3(), vec3(), vec3(), vec3(), vec3(), vec3()
local tmpPoint2I = Point2I(0, 0)
local intersections, leftPts, rightPts = {}, {}, {}


-- Returns the scale factor for the bar points.
local function getBarScale() return barScale end

-- Checks if the mouse is over a node of any spline in the given collection of splines.
-- [Splines - The collection of splines to check, which must have a member array 'nodes', containing the ordered points of the splines.]
local function isMouseOverNode(splines)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Get the camera-to-mouse ray.
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir

  -- Check each spline in turn.
  for i = 1, #splines do
    local spline = splines[i]
    if spline then
      local nodes = spline.nodes
      for j = 1, #nodes do
        local a, b = intersectsRay_Sphere(rayPos, rayDir, nodes[j], mouseToNodetol) -- Get any intersection points between the ray and node sphere.
        if min(a, b) < maxRayDist then -- If they do exist, the mouse is over this node, so we have found target.
          return true, i, j -- Return the spline/node indices along with the true result.
        end
      end
    end
  end

  -- The mouse is not over any node, so return nil.
  return false, nil, nil
end

-- Checks if the mouse is over the given spline.
-- [Spline - The spline to check, which must have a member array 'nodes', containing the ordered points of the spline.]
local function isMouseOverSpline(spline, mousePos)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Project the mouse position onto the {z = 0} plane.
  mousePos2D:set(mousePos.x, mousePos.y, 0.0)

  -- Check each node, in each spline, in turn.
  pLast:set(huge, huge, huge)
  local points = spline.nodes
  for j = 1, #points - 1 do
    local p0, p1, p2, p3 = points[max(1, j - 1)], points[j], points[j + 1], points[min(#points, j + 2)]
    for k = 0, defaultMinNumDivisions do
      pThis:set(catmullRom(p0, p1, p2, p3, k * splineGranInv, 0.5))
      pThis.z = 0.0 -- Ensure the point is on the {z = 0} plane.
      if mousePos2D:squaredDistanceToLineSegment(pThis, pLast) < intermediateTolSq then
        return true, j
      end
      pLast:set(pThis) -- Update the last mouse position.
    end
  end

  -- The mouse is not over the spline, so return nil.
  return false, nil
end

-- Checks if the mouse is over the given polyline.
local function isMouseOverPolyline(nodes, mousePos)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Project the mouse position onto the {z = 0} plane.
  mousePos2D:set(mousePos.x, mousePos.y, 0.0)

  -- Check each node, in each spline, in turn.
  for j = 1, #nodes - 1 do
    local p0, p1 = nodes[j], nodes[j + 1]
    tmp1:set(p0.x, p0.y, 0.0)
    tmp2:set(p1.x, p1.y, 0.0)
    if mousePos2D:squaredDistanceToLineSegment(tmp1, tmp2) < intermediateTolSq then
      return true, j
    end
  end

  -- The mouse is not over the polyline, so return nil.
  return false, nil
end

-- Checks if the mouse is over a rib.
-- [Splines - The collection of splines to check, which must have a member 1D array 'ribPoints', containing the ordered points of the ribs.]
local function isMouseOverRib(splines)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Get the camera-to-mouse ray.
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir

  -- Check each spline in turn.
  for i = 1, #splines do
    local spline = splines[i]
    if spline and spline.isEnabled then
      local ribPoints = spline.ribPoints
      if ribPoints and #ribPoints > 0 then
        for j = 1, #ribPoints do
          local a, b = intersectsRay_Sphere(rayPos, rayDir, ribPoints[j], mouseToNodetol) -- The intersection points between ray and node sphere.
          if min(a, b) < maxRayDist then -- If one exists, the mouse is over this rib point.
            return true, i, j -- Returns: isOverRib, spline index in give table, rib index in given table.
          end
        end
      end
    end
  end

  -- The mouse is not over any rib, so return nil.
  return false, nil, nil
end

-- Checks if the mouse is over a bar.
-- [Splines - The collection of splines to check, which must have a member 1D array 'barPoints', containing the ordered points of the bars.]
local function isMouseOverBar(splines)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Get the camera-to-mouse ray.
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir

  -- Check each spline in turn.
  for i = 1, #splines do
    local spline = splines[i]
    if spline and spline.isEnabled then
      local barPoints = spline.barPoints
      if barPoints and #barPoints > 0 then
        for j = 1, #barPoints do
          local a, b = intersectsRay_Sphere(rayPos, rayDir, barPoints[j], mouseToNodetol) -- The intersection points between ray and node sphere.
          if min(a, b) < maxRayDist then -- If one exists, the mouse is over this bar point.
            return true, i, j -- Returns: isOverBar, spline index in give table, bar index in given table.
          end
        end
      end
    end
  end

  -- The mouse is not over any bar, so return nil.
  return false, nil, nil
end

-- Checks if the mouse is over a node in the navigation graph.
local function isMouseOverGraphNode(nodes)
  if not util.isMouseHoveringOverTerrain() then
    return false, nil -- If the mouse is not over the terrain, return nil.
  end

  -- Get the camera-to-mouse ray.
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir

  -- Check each graph node in turn.
  for key, node in pairs(nodes) do
    local a, b = intersectsRay_Sphere(rayPos, rayDir, node, mouseToNodetol) -- Get any intersection points between the ray and node sphere.
    if min(a, b) < maxRayDist then -- If they do exist, the mouse is over this node, so we have found target.
      return true, key -- Return the node key along with the true result.
    end
  end

  -- The mouse is not over any graph node, so return nil.
  return false, nil
end

-- Returns the axis-aligned bounding box of the given array of points.
local function getAABB(points)
  local xMin, xMax, yMin, yMax = huge, -huge, huge, -huge
  for i = 1, #points do
    local p = points[i]
    local pX, pY = p.x, p.y
    xMin, xMax, yMin, yMax = min(xMin, pX), max(xMax, pX), min(yMin, pY), max(yMax, pY)
  end
  return { xMin = xMin, xMax = xMax, yMin = yMin, yMax = yMax }
end

-- Computes the length of the given polyline.
local function computePolylineLength(points)
  local length = 0.0
  for i = 1, #points - 1 do
    length = length + points[i]:distance(points[i + 1])
  end
  return length
end

-- Computes a polygonal outline from the given spline. Includes a given margin.
-- [Polygons are fast to deal with using scanline spans.]
-- [Returns: polygon]
local function computeSplinePolygon(spline, margin, polygon)
  -- Create the left and right edge polylines.
  table.clear(leftPts)
  table.clear(rightPts)
  local divPoints, tangents, binormals, divWidths = spline.divPoints, spline.tangents, spline.binormals, spline.divWidths
  local numDiv = #divPoints
  for i = 1, numDiv do
    local p, bin = divPoints[i], binormals[i]
    local halfWidth = divWidths[i] * 0.5 + margin
    local pX, pY, latX, latY = p.x, p.y, bin.x * halfWidth, bin.y * halfWidth
    leftPts[i], rightPts[i] = vec3(pX - latX, pY - latY, 0), vec3(pX + latX, pY + latY, 0)
  end

  -- Move the first and last points inwards by some margin.
  leftPts[1], rightPts[1] = leftPts[1] + tangents[1] * endMargin, rightPts[1] + tangents[1] * endMargin
  leftPts[numDiv], rightPts[numDiv] = leftPts[numDiv] - tangents[numDiv] * endMargin, rightPts[numDiv] - tangents[numDiv] * endMargin

  -- Create the polygon from the left and right edge polylines.
  table.clear(polygon)
  local ctr = 1
  for i = 1, numDiv do
    polygon[ctr] = leftPts[i]
    ctr = ctr + 1
  end
  for i = numDiv, 1, -1 do
    polygon[ctr] = rightPts[i]
    ctr = ctr + 1
  end
end

-- Checks if the given point (p) is inside the given triangle (a, b, c).
local function isPointInTriangle(p, a, b, c)
  local v0, v1, v2 = c - a, b - a, p - a
  local dot00, dot01, dot02, dot11, dot12 = v0:dot(v0), v0:dot(v1), v0:dot(v2), v1:dot(v1), v1:dot(v2)
  local denom = dot00 * dot11 - dot01 * dot01
  if denom == 0 then
    return false
  end
  local invDenom = 1.0 / denom
  local u = (dot11 * dot02 - dot01 * dot12) * invDenom
  local v = (dot00 * dot12 - dot01 * dot02) * invDenom
  return (u >= 0.0 and v >= 0.0 and (u + v) <= 1.0)
end

-- Returns the normal of the terrain at the given world-space point.
local function getTerrainNormal(p)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  te:worldToGridByPoint2I(p, tmpPoint2I, tb) -- World point to grid.
  local gx, gy = tmpPoint2I.x, tmpPoint2I.y
  local dzdx = (tb:getHeightGrid(gx + 1, gy) - tb:getHeightGrid(gx - 1, gy)) * 0.5 -- Sample neighbourhood.
  local dzdy = (tb:getHeightGrid(gx, gy + 1) - tb:getHeightGrid(gx, gy - 1)) * 0.5
  local normal = vec3(-dzdx, -dzdy, 1)
  normal:normalize()
  return normal
end

-- Rotates vector v around unit axis k, by angle theta (in radians).
-- [Uses the standard Rodrigues formula].
local function rotateVecAroundAxis(v, k, theta)
  local c = cos(theta)
  return v * c + k:cross(v) * sin(theta) + k * k:dot(v) * (1.0 - c)
end

-- Checks if the given line segment (a, b) intersects with the given line segment (c, d).
local function isLineSegIntersect(a, b, c, d)
  local xnorm, xnorm2 = closestLinePoints(a, b, c, d)
  return xnorm >= 0.0 and xnorm <= 1.0 and xnorm2 >= 0.0 and xnorm2 <= 1.0
end

-- Compute the signed angle between two vectors around a given axis.
local function signedAngleAroundAxis(fromVec, toVec, axis)
  local fromProj = fromVec:projectToOriginPlane(axis)
  local toProj = toVec:projectToOriginPlane(axis)
  local dot = max(-1, min(1, fromProj:dot(toProj)))
  local angle = acos(dot) -- The unsigned angle.
  local sign = (fromProj:cross(toProj)):dot(axis) >= 0 and 1 or -1 -- Determine the sign of the angle.
  return angle * sign * (180 / pi) -- Signed angle in degrees.
end

-- Compute the closest point on the given ribbon segment, to the given point.
local function closestRibbonSegPointToPoint(segIdx, ribbon, p)
  if segIdx <= ribbon.numSegs then
    local twoSegIdx = segIdx * 2
    local i1, i2, i3, i4 = twoSegIdx - 1, twoSegIdx, twoSegIdx + 1, twoSegIdx + 2
    local nodes = ribbon.nodes
    local c0, c1, c2, c3 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local pT1 = p:triangleClosestPoint(c0, c1, c2)
    local pT2 = p:triangleClosestPoint(c1, c2, c3)
    local dSq1, dSq2 = p:squaredDistance(pT1), p:squaredDistance(pT2)
    if dSq2 < dSq1 then
      return pT2, dSq2
    end
    return pT1, dSq1
  end
  return nil, nil
end

-- Given a polyline and a polygon, returns sequences of consecutive indices that are inside the polygon.
local function getNodeSpansInsidePolygon(nodes, polygon)
  local spans, ctr = {}, 1
  local currentSpan = nil
  for i = 1, #nodes do
    if nodes[i]:inPolygon(polygon) then
      if not currentSpan then
        currentSpan = { i, i } -- start a new span.
      else
        currentSpan[2] = i -- extend current span.
      end
    else
      if currentSpan then
        spans[ctr] = currentSpan
        ctr = ctr + 1
        currentSpan = nil
      end
    end
  end
  if currentSpan then
    spans[ctr] = currentSpan -- Capture final span if list ends while inside.
    ctr = ctr + 1
  end
  return spans
end

-- Computes the mean lateral offset of a decal road from the master road.
local function computeMeanLateralOffset(nodes, masterDivPoints, masterWidths)
  local total, count = 0, 0
  for _, p in ipairs(nodes) do
    -- Find the best projection and binormal of the point onto the master road.
    local bestDistSqr, bestProj, bestIdx, bestBinormal = huge, nil, nil, nil
    for i = 1, #masterDivPoints - 1 do
      local a, b = masterDivPoints[i], masterDivPoints[i + 1]
      local ab = b - a
      local t = clamp((p - a):dot(ab) / ab:squaredLength(), 0, 1)
      local proj = a + ab * t
      local distSqr = (p - proj):squaredLength()

      if distSqr < bestDistSqr then
        bestDistSqr = distSqr
        bestProj = proj
        bestIdx = i
        bestBinormal = (b - a):cross(up):normalized()
      end
    end

    -- If the best projection and binormal are valid, add contribution to the subtotal.
    if bestProj and bestBinormal then
      local lateralVec = p - bestProj
      local lateral = lateralVec:dot(bestBinormal)
      local halfWidth = (masterWidths[bestIdx] or 10) * 0.5
      total = total + (lateral / halfWidth)
      count = count + 1
    end
  end

  if count == 0 then
    return 0.0 -- If no valid projection and binormal were found, return 0.0.
  end

  -- Return the mean lateral offset, clamped to [-2.0, 2.0].
  return clamp(total / count, -2.0, 2.0)
end

-- Chooses the best master road: longest and widest.
local function getBestMasterDecalRoadIndex(decalRoads)
  local bestIdx, bestScore = nil, -1
  for i, road in ipairs(decalRoads) do
    local nodes = editor.getNodes(road)
    local numNodes = #nodes
    local numNodesInv = 1.0 / numNodes
    if numNodes >= 2 then
      -- Compute total length.
      local length = 0
      for j = 1, numNodes - 1 do
        length = length + (nodes[j + 1].pos - nodes[j].pos):length()
      end

      -- Compute mean width.
      local totalWidth = 0
      for _, node in ipairs(nodes) do
        totalWidth = totalWidth + (node.width or 0)
      end
      local avgWidth = totalWidth * numNodesInv

      -- Final composite score. Store if best yet.
      local score = (length * weightLength) + (avgWidth * weightWidth)
      if score > bestScore then
        bestScore = score
        bestIdx = i
      end
    end
  end
  return bestIdx
end

-- Returns position, tangent, binormal and width, at a normalised distance along the given Frenet polyline.
local function getDecalTransformAt(divPoints, tangents, binormals, divWidths, q)
  local numDivPoints = #divPoints
  if numDivPoints < 2 then
    return nil -- We can not place a decal if there are less than 2 points, since (s, t) is not defined.
  end

  -- Compute cumulative segment lengths.
  local segLengths, totalLength = table.new(numDivPoints, 0), 0.0
  for i = 1, numDivPoints - 1 do
    local len = divPoints[i]:distance(divPoints[i + 1])
    segLengths[i] = len
    totalLength = totalLength + len
  end

  -- Find the segment which contains the given normalised distance.
  local targetDist = q * totalLength
  local accumulated = 0.0
  for i = 1, #segLengths do
    local nextAccum = accumulated + segLengths[i]
    if targetDist <= nextAccum then
      local t = (targetDist - accumulated) / segLengths[i]
      local pos = divPoints[i] + (divPoints[i + 1] - divPoints[i]) * t -- Position.
      local tangent = tangents[i] + (tangents[i + 1] - tangents[i]) * t -- Tangent.
      tangent:normalize()
      local binormal = binormals[i] + (binormals[i + 1] - binormals[i]) * t -- Binormal.
      binormal:normalize()
      local width = divWidths[i] + (divWidths[i + 1] - divWidths[i]) * t -- Width.
      return pos, tangent, binormal, width
    end
    accumulated = nextAccum
  end

  -- Fallback: return last point.
  local fallbackTangent = tangents[#tangents]
  local fallbackBinormal = binormals[#binormals]
  local fallbackNormal = fallbackTangent:cross(fallbackBinormal)
  fallbackNormal:normalize()
  return divPoints[#divPoints], fallbackTangent, fallbackBinormal, divWidths[#divWidths]
end

-- Projects the given point onto the given spline.
-- [Returns: closest point (p), s in [0,1], t in [-inf,inf]; or nil if outside segment bounds.]
local function projectPointToSpline(pos, spline)
  local divPoints, binormals, widths = spline.divPoints, spline.binormals, spline.divWidths
  local bestDistSq, bestProj, bestS = huge, nil, 0.0
  local bestIdx, bestT, bestSegment = 1, 0.0, 1

  -- Precompute segment lengths and total
  local segLengths, totalLength = table.new(#divPoints, 0), 0.0
  for i = 1, #divPoints - 1 do
    local len = divPoints[i]:distance(divPoints[i + 1])
    segLengths[i] = len
    totalLength = totalLength + len
  end

  -- Find closest projection
  local accumulated = 0.0
  for i = 1, #divPoints - 1 do
    local p0, p1 = divPoints[i], divPoints[i + 1]
    local seg = p1 - p0
    local lenSq = seg:squaredLength()
    if lenSq > 0 then
      local t = clamp((pos - p0):dot(seg) / lenSq, 0, 1)
      local proj = p0 + seg * t
      local distSq = (proj - pos):squaredLength()
      if distSq < bestDistSq then
        bestDistSq = distSq
        bestProj = proj
        bestS = (accumulated + segLengths[i] * t) / totalLength
        bestIdx = i
        bestT = t
        bestSegment = i
      end
    end
    accumulated = accumulated + segLengths[i]
  end

  -- Reject if projected onto the first or last segment near their outer ends
  local numSegs = #divPoints - 1
  local edgeTol = 1e-3
  if (bestSegment == 1     and bestT < edgeTol) or
     (bestSegment == numSegs and bestT > 1.0 - edgeTol) then
    return nil, nil, nil
  end

  -- Interpolate binormal and width
  local bin = lerp(binormals[bestIdx], binormals[bestIdx + 1], bestT)
  bin:normalize()
  local width = widths[bestIdx] * (1 - bestT) + widths[bestIdx + 1] * bestT

  local t = 2.0 * (pos - bestProj):dot(bin) / width
  return bestProj, bestS, t
end

-- Sample along true arc-length of the given polyline, to compute the positions and Frenet frame at regular intervals.
-- [Returns: outPosns, outTans, outNormals]
local function sampleSpline(divPoints, tangents, normals, spacing, outPosns, outTans, outNormals)
  table.clear(outPosns)
  table.clear(outTans)
  table.clear(outNormals)
  local nextDist, accDist, ctr, i = 0.0, 0.0, 1, 2
  while i <= #divPoints do
    local iMinusOne = i - 1
    local p1, p2 = divPoints[iMinusOne], divPoints[i]
    tmpTangent:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
    local segLen = tmpTangent:length()
    if accDist + segLen < nextDist then -- If the current segment can't reach the next sample point, move on.
      accDist = accDist + segLen
      i = i + 1
    else
      local t = (nextDist - accDist) / segLen
      outPosns[ctr] = vec3(p1.x + tmpTangent.x * t, p1.y + tmpTangent.y * t, p1.z + tmpTangent.z * t)
      outTans[ctr] = lerp(tangents[iMinusOne], tangents[i], t)
      outTans[ctr]:normalize()
      outNormals[ctr] = lerp(normals[iMinusOne], normals[i], t)
      outNormals[ctr]:normalize()
      ctr = ctr + 1
      nextDist = nextDist + spacing
    end
  end
end

-- Estimate local curvature at a point on a polyline of vec3s.
-- Returns 0 for endpoints or degenerate segments.
local function estimateCurvatureAt(points, i)
  if i <= 1 or i >= #points then
    return 0.0 -- Endpoints: undefined curvature.
  end
  local pPrev = points[i - 1]
  local pCurr = points[i]
  local pNext = points[i + 1]
  local v1 = pCurr - pPrev
  local v2 = pNext - pCurr
  local len1, len2 = v1:length(), v2:length()
  if len1 < 1e-6 or len2 < 1e-6 then
    return 0.0 -- Degenerate segment.
  end
  v1:normalize()
  v2:normalize()
  local dot = max(-1, min(1, v1:dot(v2)))
  local angle = acos(dot)
  local avgLen = 0.5 * (len1 + len2)
  if avgLen < 1e-6 then
    return 0.0
  end
  return angle / avgLen -- angle / arc length.
end

-- Adaptive sampling along the given spline, to compute the positions and Frenet frame at regular intervals.
-- [Adaptive in the sense that small curvature segments are sampled more densely, and large curvature segments are sampled more sparsely.]
-- [Returns: outPosns, outTans, outNormals, outScales]
local function sampleSplineAdaptive(divPoints, tangents, normals, meshLength, outPosns, outTans, outNormals, outScales)
  table.clear(outPosns)
  table.clear(outTans)
  table.clear(outNormals)
  table.clear(outScales)
  local nextDist, accDist, ctr, i = 0.0, 0.0, 1, 2
  while i <= #divPoints do
    local iMinusOne = i - 1
    local p1, p2 = divPoints[iMinusOne], divPoints[i]
    tmpTangent:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
    local segLen = tmpTangent:length()
    if accDist + segLen < nextDist then -- If the current segment can't reach the next sample point, move on.
      accDist = accDist + segLen
      i = i + 1
    else
      local t = (nextDist - accDist) / segLen
      outPosns[ctr] = vec3(p1.x + tmpTangent.x * t, p1.y + tmpTangent.y * t, p1.z + tmpTangent.z * t)
      outTans[ctr] = lerp(tangents[iMinusOne], tangents[i], t)
      outTans[ctr]:normalize()
      outNormals[ctr] = lerp(normals[iMinusOne], normals[i], t)
      outNormals[ctr]:normalize()
      local curvature = estimateCurvatureAt(divPoints, i)
      outScales[ctr] = max(minScale, 1.0 / (1.0 + kappaSensitivity * curvature)) -- Maps curvature to a normalised scale [minScale, 1].
      nextDist = nextDist + meshLength * outScales[ctr]
      ctr = ctr + 1
    end
  end
end

-- Translates the given spline by the given amount on the binormal (lateral) direction.
local function translateSpline(pts, binormals, t, out)
  table.clear(out)
  for i = 1, #pts do
    out[i] = pts[i] + binormals[i] * t
  end
end

-- Computes a random jitter quaternion, for Z-only jitter.
local function computeRandomJitterQuat_ZOnly(jitter, nmlVec) return quatFromAxisAngle(nmlVec, (random() * 2 - 1) * jitter) end

-- Computes a random jitter quaternion, for component-wise jitter.
local function computeRandomJitterQuat(spline, tgt, rightVec, nmlVec)
  local jitterQuat = zeroQuat -- Initialize jitter quaternions as identity.
  local jitterForward, jitterRight, jitterUp = spline.jitterForward, spline.jitterRight, spline.jitterUp
  if jitterForward and jitterForward > 0.0 then -- Apply forward (tangent) jitter if requested.
    local angleF = (random() * 2 - 1) * jitterForward
    local jitterF = quatFromAxisAngle(tgt, angleF)
    jitterQuat = jitterF * jitterQuat
  end
  if jitterRight and jitterRight > 0.0 then -- Apply right jitter if requested.
    local angleR = (random() * 2 - 1) * jitterRight
    local jitterR = quatFromAxisAngle(rightVec, angleR)
    jitterQuat = jitterR * jitterQuat
  end
  if jitterUp and jitterUp > 0.0 then -- Apply up (normal) jitter requested.
    local angleU = (random() * 2 - 1) * jitterUp
    local jitterU = quatFromAxisAngle(nmlVec, angleU)
    jitterQuat = jitterU * jitterQuat
  end
  return jitterQuat
end

-- Update the rib points.
local function updateRibPoints(spline)
  local nodes, widths, ribPoints = spline.nodes, spline.widths, spline.ribPoints
  local numNodes, ctr = #nodes, 1
  table.clear(ribPoints)
  for i = 1, numNodes do
    local p1, p2 = nodes[min(numNodes, i + 1)], nodes[max(1, i - 1)]
    tmp0:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z) -- Tangent.
    tmp0:normalize()
    tmp1:set(tmp0:cross(up)) -- Binormal.
    local latVec = tmp1 * widths[i] * 0.5
    local node = nodes[i]
    local pL, pR = node - latVec, node + latVec
    tmp1:set(pL.x, pL.y, pL.z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist)
    pL.z = tmp1.z - d + zRaycastOffset
    tmp2:set(pR.x, pR.y, pR.z + zExtra)
    d = castRayStatic(tmp2, down, maxRaycastDist)
    pR.z = tmp2.z - d + zRaycastOffset
    ribPoints[ctr], ribPoints[ctr + 1] = pL, pR
    ctr = ctr + 2
  end
end

-- Update the bar points.
local function updateBarPoints(spline, isBarsLimits)
  local nodes, barPoints = spline.nodes, spline.barPoints
  local vals = isBarsLimits and spline.velLimits or spline.vels
  table.clear(barPoints)
  for i = 1, #nodes do
    tmp1:set(nodes[i].x, nodes[i].y, nodes[i].z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp1.z = tmp1.z - d + vals[i] * barScale
    barPoints[i] = vec3(tmp1)
  end
end

-- Update the bar points for a graph path.
local function updateBarPointsGraph(spline, graphData)
  local graphDataNodes = graphData.nodes
  local graphNodes, barPoints, vels = spline.graphNodes, spline.barPoints, spline.vels
  table.clear(barPoints)
  for i = 1, #graphNodes do
    local node = graphDataNodes[graphNodes[i]]
    tmp1:set(node.x, node.y, node.z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp1.z = tmp1.z - d + vels[i] * barScale
    barPoints[i] = vec3(tmp1)
  end
end

-- Computes the scanline spans for the given polygon.
-- [The scanline spans are the spans of the polygon on the Y-axis, in grid space.]
-- [The spans are stored in the given spansXMin, spansXMax, and spansY tables.]
local function getScanlineSpans(gMinY, gMaxY, te, tb, polygon, spansXMin, spansXMax, spansY)
  table.clear(spansXMin)
  table.clear(spansXMax)
  table.clear(spansY)
  local numPolygonNodes, ctr = #polygon, 1
  for y = gMinY, gMaxY do
    table.clear(intersections)
    tmpPoint2I.x, tmpPoint2I.y = 0, y
    local yWorld = te:gridToWorldByPoint2I(tmpPoint2I, tb).y
    local iCtr = 1
    for i = 1, numPolygonNodes do
      local a, b = polygon[i], polygon[i % numPolygonNodes + 1] -- Wrapped line segment [a, b] on polygon.
      local aX, aY, bX, bY = a.x, a.y, b.x, b.y
      if (aY <= yWorld and bY > yWorld) or (bY <= yWorld and aY > yWorld) then -- Note: Ignores odd numbers of intersections.
        local t = (yWorld - aY) / (bY - aY)
        local x = aX + t * (bX - aX)
        intersections[iCtr] = x
        iCtr = iCtr + 1
      end
    end
    table.sort(intersections)
    for i = 1, #intersections - 1, 2 do
      local x1, x2 = intersections[i], intersections[i + 1]
      tmp1:set(x1, yWorld, 0)
      te:worldToGridByPoint2I(tmp1, tmpPoint2I, tb)
      local xStart = tmpPoint2I.x
      tmp1:set(x2, yWorld, 0)
      te:worldToGridByPoint2I(tmp1, tmpPoint2I, tb)
      local xEnd = tmpPoint2I.x
      spansXMin[ctr], spansXMax[ctr], spansY[ctr] = xStart, xEnd, y
      ctr = ctr + 1
    end
  end
end

-- Computes the signed distance field (SDF) of the given mask.
local function computeSDF(mask)
  local w, h = #mask[1], #mask
  local maxDist = w + h
  local sdf = {}

  -- 1. Init distance map
  for y = 1, h do
    sdf[y] = {}
    for x = 1, w do
      sdf[y][x] = mask[y][x] == 0 and 0 or maxDist
    end
  end

  -- 2. Forward pass
  for y = 1, h do
    for x = 1, w do
      local d = sdf[y][x]
      if x > 1 then d = min(d, sdf[y][x - 1] + 1) end
      if y > 1 then d = min(d, sdf[y - 1][x] + 1) end
      if x > 1 and y > 1 then d = min(d, sdf[y - 1][x - 1] + 1.414) end
      if x < w and y > 1 then d = min(d, sdf[y - 1][x + 1] + 1.414) end
      sdf[y][x] = d
    end
  end

  -- 3. Backward pass
  for y = h, 1, -1 do
    for x = w, 1, -1 do
      local d = sdf[y][x]
      if x < w then d = min(d, sdf[y][x + 1] + 1) end
      if y < h then d = min(d, sdf[y + 1][x] + 1) end
      if x < w and y < h then d = min(d, sdf[y + 1][x + 1] + 1.414) end
      if x > 1 and y < h then d = min(d, sdf[y + 1][x - 1] + 1.414) end
      sdf[y][x] = d
    end
  end

  return sdf
end

-- Interpolates the given points with Catmull-Rom splines.
-- [Only works for a single array of nodes.]
local function catmullRomNodesOnly(nodes, gran)
  local granInv = 1.0 / gran
  local pts, ctr, startIdx, numNodes = {}, 1, 0, #nodes
  for j = 1, numNodes - 1 do
    local p0, p1, p2, p3 = nodes[max(1, j - 1)], nodes[j], nodes[j + 1], nodes[min(numNodes, j + 2)]
    local z0, z1, z2, z3 = p0.z, p1.z, p2.z, p3.z
    for k = startIdx, gran do
      local t = k * granInv
      pts[ctr] = catmullRom(p0, p1, p2, p3, t, 0.5)
      pts[ctr].z = monotonicSteffen(z0, z1, z2, z3, 0, 1, 2, 3, t + 1) -- Use a monotonic spline for Z, to stop overshooting under the surface.
      ctr = ctr + 1
    end
    startIdx = 1
  end
  return pts
end

-- Interpolates the given points with Catmull-Rom splines.
-- [Only works for a single array of nodes.]
local function catmullRomNodesWidthsOnly(nodes, widths, gran)
  local granInv = 1.0 / gran
  local pts, wds, ctr, startIdx, numNodes = {}, {}, 1, 0, #nodes
  for j = 1, numNodes - 1 do
    local i1, i2, i3, i4 = max(1, j - 1), j, j + 1, min(numNodes, j + 2)
    local p0, p1, p2, p3 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local z0, z1, z2, z3 = p0.z, p1.z, p2.z, p3.z
    local w0, w1, w2, w3 = widths[i1], widths[i2], widths[i3], widths[i4]
    for k = startIdx, gran do
      local t = k * granInv
      pts[ctr] = catmullRom(p0, p1, p2, p3, t, 0.5)
      pts[ctr].z = monotonicSteffen(z0, z1, z2, z3, 0, 1, 2, 3, t + 1) -- Use a monotonic spline for Z, to stop overshooting under the surface.
      wds[ctr] = monotonicSteffen(w0, w1, w2, w3, 0, 1, 2, 3, t + 1) -- Use a monotonic spline for the width interpolation.
      ctr = ctr + 1
    end
    startIdx = 1
  end
  return pts, wds
end

-- Compute the local Frenet frame at each division point.
local function computeFrenetFrame(spline)
  local divPoints, tangents, binormals, normals = spline.divPoints, spline.tangents, spline.binormals, spline.normals
  table.clear(tangents)
  table.clear(binormals)
  local numDivPoints = #divPoints
  for i = 1, numDivPoints do
    tangents[i] = divPoints[min(numDivPoints, i + 1)] - divPoints[max(1, i - 1)]
    tangents[i]:normalize()
    binormals[i] = tangents[i]:cross(normals[i])
  end
end

-- Interpolates the nodes and widths of the given spline with 'standard' Catmull-Rom splines to generate the secondary geometry properties.
-- [Points are conformed to the terrain.]
-- [The secondary geometry data is stored in the given spline object.]
local function catmullRomConformToTerrain(spline, minNumDivIn)
  local terrain = core_terrain.getTerrain()
  local minNumDivisionsFinal = minNumDivIn or defaultMinNumDivisions
  local divPoints, divWidths, normals, discMap = spline.divPoints, spline.divWidths, spline.normals, spline.discMap
  table.clear(divPoints)
  table.clear(divWidths)
  table.clear(normals)
  table.clear(discMap)
  local nodes, widths, nmls = spline.nodes, spline.widths, spline.nmls
  local numNodes, startIdx, ctr = #nodes, 0, 1
  for j = 1, numNodes - 1 do
    discMap[j] = max(1, ctr - 1) -- Create an entry in the map of node index to the division point index.
    local i1, i2, i3, i4 = max(1, j - 1), j, j + 1, min(numNodes, j + 2)
    local p0, p1, p2, p3 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local n0, n1, n2, n3 = nmls[i1], nmls[i2], nmls[i3], nmls[i4]
    local w0, w1, w2, w3 = widths[i1], widths[i2], widths[i3], widths[i4]
    tmp0:set(p0.x, p0.y, w0)
    tmp1:set(p1.x, p1.y, w1)
    tmp2:set(p2.x, p2.y, w2)
    tmp3:set(p3.x, p3.y, w3)
    local numDivisions = max(minNumDivisionsFinal, floor(p1:distance(p2) / maxDivisionSpacing + 0.5)) -- The number of divisions is dynamic based on node-to-node distance.
    local step = 1.0 / numDivisions
    for k = startIdx, numDivisions do
      local t = k * step
      divPoints[ctr] = catmullRom(p0, p1, p2, p3, t, 0.5)
      divPoints[ctr].z = terrain:getHeight(divPoints[ctr]) -- Conform to the terrain.
      normals[ctr] = catmullRom(n0, n1, n2, n3, t, 0.5) -- Interpolate the normals.
      divWidths[ctr] = monotonicSteffen(w0, w1, w2, w3, 0, 1, 2, 3, t + 1) -- Use a monotonic spline for the width interpolation.
      ctr = ctr + 1
    end
    startIdx = 1 -- Set the start index to 1 for all [2, .., n] iterations, so as to avoid duplicates in subsequent iterations. Avoids branch.
  end
  discMap[#nodes] = ctr - 1

  computeFrenetFrame(spline) -- Compute the local Frenet frame at each division point, and store it in the spline.
end

-- Interpolates the nodes and widths of the given spline with 'standard' Catmull-Rom splines to generate the secondary geometry properties.
-- [Points are raycast to the surface below.]
-- [The secondary geometry data is stored in the given spline object.]
local function catmullRomRaycast(spline, minNumDivIn)
  local minNumDivisionsFinal = minNumDivIn or defaultMinNumDivisions
  local divPoints, divWidths, normals, discMap = spline.divPoints, spline.divWidths, spline.normals, spline.discMap
  table.clear(divPoints)
  table.clear(divWidths)
  table.clear(normals)
  table.clear(discMap)
  local nodes, widths, nmls = spline.nodes, spline.widths, spline.nmls
  local numNodes, startIdx, ctr = #nodes, 0, 1
  for j = 1, numNodes - 1 do
    discMap[j] = max(1, ctr - 1) -- Create an entry in the map of node index to the division point index.
    local i1, i2, i3, i4 = max(1, j - 1), j, j + 1, min(numNodes, j + 2)
    local p0, p1, p2, p3 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local n0, n1, n2, n3 = nmls[i1], nmls[i2], nmls[i3], nmls[i4]
    local w0, w1, w2, w3 = widths[i1], widths[i2], widths[i3], widths[i4]
    tmp0:set(p0.x, p0.y, w0)
    tmp1:set(p1.x, p1.y, w1)
    tmp2:set(p2.x, p2.y, w2)
    tmp3:set(p3.x, p3.y, w3)
    local numDivisions = max(minNumDivisionsFinal, floor(p1:distance(p2) / maxDivisionSpacing + 0.5)) -- The number of divisions is dynamic based on node-to-node distance.
    local step = 1.0 / numDivisions
    for k = startIdx, numDivisions do
      local t = k * step
      divPoints[ctr] = catmullRom(p0, p1, p2, p3, t, 0.5)
      tmp4:set(divPoints[ctr].x, divPoints[ctr].y, divPoints[ctr].z + zExtra)
      local d = castRayStatic(tmp4, down, maxRaycastDist) -- Raycast to the surface below.
      divPoints[ctr].z = tmp4.z - d + zRaycastOffset
      normals[ctr] = catmullRom(n0, n1, n2, n3, t, 0.5) -- Interpolate the normals.
      divWidths[ctr] = monotonicSteffen(w0, w1, w2, w3, 0, 1, 2, 3, t + 1) -- Use a monotonic spline for the width interpolation.
      ctr = ctr + 1
    end
    startIdx = 1 -- Set the start index to 1 for all [2, .., n] iterations, so as to avoid duplicates in subsequent iterations. Avoids branch.
  end
  discMap[#nodes] = ctr - 1

  computeFrenetFrame(spline) -- Compute the local Frenet frame at each division point, and store it in the spline.
end

-- Interpolates the nodes and widths of the given spline with 'standard' Catmull-Rom splines to generate the secondary geometry properties.
-- [The secondary geometry data is stored in the given spline object.]
local function catmullRomFree(spline, minNumDivIn)
  local minNumDivisionsFinal = minNumDivIn or defaultMinNumDivisions
  local divPoints, divWidths, normals, discMap = spline.divPoints, spline.divWidths, spline.normals, spline.discMap
  table.clear(divPoints)
  table.clear(divWidths)
  table.clear(normals)
  table.clear(discMap)
  local nodes, widths, nmls = spline.nodes, spline.widths, spline.nmls
  local numNodes, startIdx, ctr = #nodes, 0, 1
  for j = 1, numNodes - 1 do
    discMap[j] = max(1, ctr - 1) -- Create an entry in the map of node index to the division point index.
    local i1, i2, i3, i4 = max(1, j - 1), j, j + 1, min(numNodes, j + 2)
    local p0, p1, p2, p3 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local n0, n1, n2, n3 = nmls[i1], nmls[i2], nmls[i3], nmls[i4]
    local z0, z1, z2, z3 = p0.z, p1.z, p2.z, p3.z
    local w0, w1, w2, w3 = widths[i1], widths[i2], widths[i3], widths[i4]
    local numDivisions = max(minNumDivisionsFinal, floor(p1:distance(p2) / maxDivisionSpacing + 0.5)) -- The number of divisions is dynamic based on node-to-node distance.
    local step = 1.0 / numDivisions
    for k = startIdx, numDivisions do
      local t = k * step
      local tPlus1 = t + 1
      divPoints[ctr] = catmullRom(p0, p1, p2, p3, t, 0.5)
      divPoints[ctr].z = monotonicSteffen(z0, z1, z2, z3, 0, 1, 2, 3, tPlus1) -- Use a monotonic spline for Z, to stop overshooting under the surface.
      normals[ctr] = catmullRom(n0, n1, n2, n3, t, 0.5) -- Interpolate the normals.
      divWidths[ctr] = monotonicSteffen(w0, w1, w2, w3, 0, 1, 2, 3, tPlus1) -- Use a monotonic spline for the width interpolation.
      ctr = ctr + 1
    end
    startIdx = 1 -- Set the start index to 1 for all [2, .., n] iterations, so as to avoid duplicates in subsequent iterations. Avoids branch.
  end
  discMap[#nodes] = ctr - 1

  computeFrenetFrame(spline) -- Compute the local Frenet frame at each division point, and store it in the spline.
end

-- Computes the graph path from the given nodes. Data is placed in the given spline's divPoints.
local function computeGraphPathFromNodes(spline)
  local nodes, path = spline.graphNodes, spline.graphPath
  table.clear(path) -- Clear the path no matter what.
  if #nodes < 2 then
    return -- Not enough nodes to compute a path.
  end

  -- Compute the path.
  local ctr, startIdx = 1, 1
  for i = 1, #nodes - 1 do
    local section = map.getPath(nodes[i], nodes[i + 1])
    for j = startIdx, #section do
      path[ctr] = section[j]
      ctr = ctr + 1
    end
    startIdx = 2 -- Set the start index to 2 for all [2, .., n] iterations, so as to avoid duplicates in subsequent iterations. Avoids branch.
  end
end


-- Public interface.
M.getBarScale =                                         getBarScale

M.isMouseOverNode =                                     isMouseOverNode
M.isMouseOverSpline =                                   isMouseOverSpline
M.isMouseOverPolyline =                                 isMouseOverPolyline
M.isMouseOverRib =                                      isMouseOverRib
M.isMouseOverBar =                                      isMouseOverBar
M.isMouseOverGraphNode =                                isMouseOverGraphNode

M.getAABB =                                             getAABB
M.computePolylineLength =                               computePolylineLength
M.computeSplinePolygon =                                computeSplinePolygon
M.isPointInTriangle =                                   isPointInTriangle
M.getTerrainNormal =                                    getTerrainNormal
M.rotateVecAroundAxis =                                 rotateVecAroundAxis
M.isLineSegIntersect =                                  isLineSegIntersect
M.signedAngleAroundAxis =                               signedAngleAroundAxis
M.closestRibbonSegPointToPoint =                        closestRibbonSegPointToPoint
M.getNodeSpansInsidePolygon =                           getNodeSpansInsidePolygon
M.computeMeanLateralOffset =                            computeMeanLateralOffset
M.getBestMasterDecalRoadIndex =                         getBestMasterDecalRoadIndex
M.getDecalTransformAt =                                 getDecalTransformAt
M.projectPointToSpline =                                projectPointToSpline
M.sampleSpline =                                        sampleSpline
M.sampleSplineAdaptive =                                sampleSplineAdaptive
M.translateSpline =                                     translateSpline

M.computeRandomJitterQuat_ZOnly =                       computeRandomJitterQuat_ZOnly
M.computeRandomJitterQuat =                             computeRandomJitterQuat

M.updateRibPoints =                                     updateRibPoints
M.updateBarPoints =                                     updateBarPoints
M.updateBarPointsGraph =                                updateBarPointsGraph

M.computeSDF =                                          computeSDF
M.getScanlineSpans =                                    getScanlineSpans

M.catmullRomNodesOnly =                                 catmullRomNodesOnly
M.catmullRomNodesWidthsOnly =                           catmullRomNodesWidthsOnly
M.catmullRomConformToTerrain =                          catmullRomConformToTerrain
M.catmullRomRaycast =                                   catmullRomRaycast
M.catmullRomFree =                                      catmullRomFree

M.computeGraphPathFromNodes =                           computeGraphPathFromNodes

return M