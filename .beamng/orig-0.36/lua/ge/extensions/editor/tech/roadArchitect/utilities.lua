-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local maxRayDist = 1000                                                                             -- The maximum distance for the camera -> mouse ray, in metres.
local mouseToNodetol = 0.84                                                                         -- The distance tolerance used when testing if the mouse is close to a node.
local nodeCloseDist = 10.0                                                                          -- The distance used when checking if the mouse is close to a node.
local highlightMargin = 0.2                                                                         -- The margin by which a highlight surrounds an object (multiplicative factor).
local distTolSq = 0.01                                                                              -- The distance tolerance used when determining which nodes relates to which div pt.

local sphereColor = color(255, 0, 0, 127)                                                           -- The colour used for drawing red spheres.
local groupSphereColor = color(255, 0, 255, 127)                                                    -- The colour used for drawing group polygon spheres.
local highlightColor = color(127, 127, 127, 127)                                                    -- The colour used for drawing transparent highlights.
local highlightColorPurple = color(255, 0, 255, 255)
local redColor = color(255, 0, 0, 255)                                                              -- The colour used for highlight selected auto junction road end nodes.

local textA = color(0, 0, 0, 255)                                                                   -- The markup text foreground colour.
local textB = color(255, 255, 255, 255)                                                             -- The markup text background colour.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}


-- External modules used.
local dbgDraw = require('utils/debugDraw')


-- Private constants.
local im = ui_imgui
local floor, abs, min, max, sqrt = math.floor, math.abs, math.min, math.max, math.sqrt
local sin, cos, acos = math.sin, math.cos, math.acos
local twoPi = math.pi * 2.0
local random = math.random
local xAxis, yAxis = vec3(1, 0, 0), vec3(0, 1, 0)
local downVec = vec3(0, 0, -1)
local tmp = vec3(0, 0)
local nodeCloseDistSq = nodeCloseDist * nodeCloseDist


-- Draws spheres (for various purposes).
-- [The size of the sphere varies with the camera distance, so as to keep the sphere about the same size].
local function drawSphere(pos) dbgDraw.drawSphere(pos, sqrt(pos:distance(core_camera.getPosition())) * 0.1, sphereColor) end
local function drawGroupSphere(pos) dbgDraw.drawSphere(pos, sqrt(pos:distance(core_camera.getPosition())) * 0.1, groupSphereColor) end
local function drawSphereHighlight(pos) dbgDraw.drawSphere(pos, sqrt(pos:distance(core_camera.getPosition())) * highlightMargin, highlightColor) end
local function drawSphereHighlightRed(pos) dbgDraw.drawSphere(pos, sqrt(pos:distance(core_camera.getPosition())) * highlightMargin, redColor) end
local function drawSphereHighlightPurple(pos) dbgDraw.drawSphere(pos, sqrt(pos:distance(core_camera.getPosition())) * highlightMargin, highlightColorPurple) end
local function drawHighlightSphereRad(pos, rad) dbgDraw.drawSphere(pos, rad, highlightColor) end

-- Draw lines.
local function drawPurpleLine(p1, p2) dbgDraw.drawLineInstance_MinArg(p1, p2, 5, highlightColorPurple) end

-- Draw text markup for auto junctions.
local function drawAutoJctMarkup(pos, rName, rLie)
  dbgDraw.drawTextAdvanced(pos, rName, textA, true, false, textB)
  dbgDraw.drawTextAdvanced(pos, rLie, textA, true, false, textB)
end

-- Tests if mouse is hovering over the terrain (as opposed to any windows, etc).
local function isMouseHoveringOverTerrain() return not im.IsAnyItemHovered() and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not editor.isAxisGizmoHovered() end

-- Computes the position on the map at which the mouse points.
local function mouseOnMapPos()
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir
  return rayPos + rayDir * castRayStatic(rayPos, rayDir, 1000)
end

-- Checks if the mouse is hovering over any existing reference node. If so, returns the relevant data.
-- [The road and node indices are returned].
local function isMouseOverNode(roads)
  if not isMouseHoveringOverTerrain() then
    return nil, nil, nil
  end
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir
  local numRoads = #roads
  for i = 1, numRoads do
    local r = roads[i]
    if r.isDisplayNodeSpheres[0] and r.isVis[0] and not r.treatAsInvisibleInEdit then               -- Only consider a road if the node spheres are currently visible.
      local nodes = r.nodes
      local numNodes = #nodes
      for j = 1, numNodes do
        local node = nodes[j]
        if not node.isLocked then                                                                   -- Do not consider locked nodes.
          local a, b = intersectsRay_Sphere(rayPos, rayDir, node.p, mouseToNodetol)                 -- Get the two intersection points between the ray and sphere, if any exist.
          if min(a, b) < maxRayDist then                                                            -- If they do exist, the mouse is over this node, so we have found target.
            return true, i, j                                                                       -- Return the road and node indices, along with the relevant flag combo.
          end
        end
      end
    end
  end
  return nil, nil, nil                                                                              -- The mouse is not over any node of any road, so return nil.
end

-- Computes the 2D axis-aligned bounding box of the given group.
-- [Only nodes in the group are included - not all nodes in each group road].
local function computeAABB2DGroup(group, roads, map)
  local gList = group.list
  local xMin, xMax, yMin, yMax = 1e24, -1e24, 1e24, -1e24
  for i = 1, #gList do
    local gL = gList[i]
    local road = roads[map[gL.r]]
    local p = road.nodes[gL.n].p
    local x, y = p.x, p.y
    xMin, xMax, yMin, yMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y)
  end
  return { xMin = xMin, xMax = xMax, yMin = yMin, yMax = yMax }
end

-- Computes the 2D axis-aligned bounding box of all roads in the session.
local function computeAABB2DAllRoads(roads)
  local xMin, xMax, yMin, yMax = 1e24, -1e24, 1e24, -1e24
  for i = 1, #roads do
    local road = roads[i]
    for j = 1, #road.nodes do
      local p = road.nodes[j].p
      local x, y = p.x, p.y
      xMin, xMax, yMin, yMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y)
    end
  end
  return { xMin = xMin, xMax = xMax, yMin = yMin, yMax = yMax }
end

-- Adds the given placed group index to the given road, if it is not already there.
local function tryAddGroupIdxToRoad(r, placedGroupIdx)
  local currPGIs = r.groupIdx
  local isFound = false
  for i = 1, #currPGIs do
    if currPGIs[i] == placedGroupIdx then
      isFound = true
      break
    end
  end
  if not isFound then
    currPGIs[#currPGIs + 1] = placedGroupIdx
  end
end

-- Rounds the given floating-point number to two decimal places.
local function round2(num) return floor(num * 100 + 0.5) / 100.0 end

-- Computes the squared 2D distance between two points (which may be 3D-valued).
local function sqDist2D(a, b)
  local dx, dy = b.x - a.x, b.y - a.y
  return dx * dx + dy * dy
end

-- Computes the 2D distance between two points, given component-wise.
local function distance2D(x1, y1, x2, y2) return sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2) end

-- Checks if the mouse is close to any existing node, excluding a given avoid road.
-- [The road and node indices are returned for every road/node combo which is found].
local function isMouseCloseToNode(roads, selectedRoadIdx, selectedNodeIdx)
  local pSel = roads[selectedRoadIdx].nodes[selectedNodeIdx].p
  local rIdxs, nIdxs, ctr = {}, {}, 1
  local rMids, nMids, mCtr = {}, {}, 1
  for i = 1,  #roads do
    if i ~= selectedRoadIdx then
      local r = roads[i]
      if r.isDisplayNodeSpheres[0] and r.isVis[0] and not r.treatAsInvisibleInEdit then             -- Only consider a road if the node spheres are currently visible.

        -- Check the road start node.
        local nodes = r.nodes
        local node = nodes[1]
        if node then
          if not node.isLocked then                                                                 -- Do not consider locked nodes.
            if sqDist2D(pSel, node.p) < nodeCloseDistSq then
              rIdxs[ctr], nIdxs[ctr] = i, 1
              ctr = ctr + 1
            end
          end
        end

        -- Check the road end node.
        local numNodes = #nodes
        local node = nodes[numNodes]
        if node then
          if not node.isLocked then                                                                 -- Do not consider locked nodes.
            if sqDist2D(pSel, node.p) < nodeCloseDistSq then
              rIdxs[ctr], nIdxs[ctr] = i, numNodes
              ctr = ctr + 1
            end
          end
        end

        -- Check the road intemediate nodes, if there are any.
        if numNodes > 2 then                                                                        -- Only consider if the road middle is linkable.
          for j = 2, numNodes - 1 do
            local node = nodes[j]
            if node then
              if not node.isLocked then                                                             -- Do not consider locked nodes.
                if sqDist2D(pSel, node.p) < nodeCloseDistSq then
                  rMids[mCtr], nMids[mCtr] = i, j
                  mCtr = mCtr + 1
                end
              end
            end
          end
        end

      end
    end
  end
  return ctr > 1 or mCtr > 1, rIdxs, nIdxs, rMids, nMids
end

-- Function to extract filename and extension from a path.
local function getFilenameFromPath(path) return path:match("([^/]+)$") end

-- Function to remove the three-letter extension (and dot) from a filename.
local function removeExtension(filename) return filename:match("(.+)%.[^.]+$") or filename end

-- Function to remove the filename and extension from the given path
local function removeFileNameFromPath(path)
  local lastSeparator = path:match("^.*()[/\\]") 
  if lastSeparator then
    return path:sub(1, lastSeparator)
  end
  return ""                                                                                         -- If no separator is found, return an empty string.
end

-- Computes the lengths (from road start) of each div point in the given render data.
local function computeRoadLength(rD)
  local cenIdx1, cenIdx2 = -1, 2
  if rD[1][1] then
    cenIdx1, cenIdx2 = 1, 1
  end

  local total = 0.0
  local lengths = { 0.0 }
  for i = 2, #rD do
    local d = rD[i][cenIdx1][cenIdx2]:distance(rD[i - 1][cenIdx1][cenIdx2])
    total = total + d
    lengths[i] = total
  end
  return lengths
end

-- Gets the minimum and maximum lane key values, from a given collection of lanes.
local function getMinMaxLaneKeys(profile)
  local l, u = 100, -100
  for i = -20, 20 do
    if profile[i] then
      l, u = min(l, i), max(u, i)
    end
  end
  return l, u
end

-- Computes the half width of the given road (at the first node - assumes width is equal throughout).
local function getHalfWidthAtNode(r)
  local rData = r.renderData
  local idx1, idx2 = -1, 3
  if rData[1][1] then
    idx1, idx2 = 1, 4
  end
  local _, lMax = getMinMaxLaneKeys(r.profile)
  return rData[1][lMax][3]:distance(rData[1][idx1][idx2])
end

-- Finds the lower and upper bounding div point index of a given length along a road.
local function findBounds(pEval, lengths)
  for i = 2, #lengths do
    local lTest, rTest = lengths[i - 1], lengths[i]
    if pEval >= lTest and pEval <= rTest then
      return i - 1, i
    end
  end
  if pEval < 0 then
    return 1, 2
  end
  if pEval > lengths[#lengths] then
    return #lengths - 1, #lengths
  end
end

-- For a given node, find the corresponding div index in the road render data.
local function computeDivIndicesFromNode(nIdx, road)
  if not road or not road.nodes or #road.nodes < 2 then
    return 1
  end
  local p = road.nodes[nIdx].p
  local p_2D = vec3(p.x, p.y, 0)
  local rData = road.renderData
  local cenIdx1, cenIdx2 = -1, 2
  if rData[1][1] then
    cenIdx1, cenIdx2 = 1, 1
  end
  for i = 1, #rData do
    local q = rData[i][cenIdx1][cenIdx2]
    tmp:set(q.x, q.y, 0.0)
    if p_2D:squaredDistance(tmp) < distTolSq then
      return i
    end
  end
  return 1
end

-- Determines if a given div point index is inside a tunnel or not.
local function isInTunnel(idx, tunnels, extraS, extraE)
  for i = 1, #tunnels do
    local t = tunnels[i]
    if t.s + extraS <= idx and idx <= t.e - extraE - 1 then
      return true
    end
  end
  return false
end

-- Linearly interpolates into a given polyline, including the orthonormal frame.
local function polyLerp(pos, nml, lens, q)
  local l, u = 1, #pos
  for i = 2, #lens do
    if lens[i - 1] <= q and lens[i] >= q then
      l, u = i - 1, i
      break
    end
  end
  if q < lens[1] then
    return nil, nil
  end
  if q > lens[#lens] then
    return nil, nil
  end
  local rat = (q - lens[l]) / (lens[u] - lens[l])
  local p = pos[l] + rat * (pos[u] - pos[l])
  local fN = vec3(0, 0)
  fN:setLerp(nml[l], nml[u], rat)
  fN:normalize()
  return p, fN
end

-- Finds the intersection between two lines (p1 -> p2), (p3 -> p4).
local function intersection2Lines(p1, p2, p3, p4)
  local x1, y1, x2, y2 = p1.x, p1.y, p2.x, p2.y
  local x3, y3, x4, y4 = p3.x, p3.y, p4.x, p4.y
  local denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
  if abs(denom) < 1e-7 then
    return nil                                                                                      -- The lines are parallel or coincident.
  end
  local invDenom = 1.0 / denom
  return vec3(
    ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) * invDenom,
    ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) * invDenom)
end

local function orientation(px, py, qx, qy, rx, ry)
  local val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy)
  if val == 0 then return 0 end
  return (val > 0) and 1 or 2
end

-- Function to check if point q lies on line segment pr.
local function onSegment(px, py, qx, qy, rx, ry) return qx <= max(px, rx) and qx >= min(px, rx) and qy <= max(py, ry) and qy >= min(py, ry) end

-- Function to determine if two line segments intersect.
local function segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)
  local o1 = orientation(x1, y1, x2, y2, x3, y3)                                                    -- Find the four orientations needed for the general and special cases.
  local o2 = orientation(x1, y1, x2, y2, x4, y4)
  local o3 = orientation(x3, y3, x4, y4, x1, y1)
  local o4 = orientation(x3, y3, x4, y4, x2, y2)
  if o1 ~= o2 and o3 ~= o4 then                                                                     -- General case.
    return true
  end
  if o1 == 0 and onSegment(x1, y1, x3, y3, x2, y2) then return true end                             -- x1, y1, x2, y2 and x3, y3 are collinear and x3, y3 lies on segment x1, y1 -> x2, y2.
  if o2 == 0 and onSegment(x1, y1, x4, y4, x2, y2) then return true end                             -- x1, y1, x2, y2 and x4, y4 are collinear and x4, y4 lies on segment x1, y1 -> x2, y2.
  if o3 == 0 and onSegment(x3, y3, x1, y1, x4, y4) then return true end                             -- x3, y3, x4, y4 and x1, y1 are collinear and x1, y1 lies on segment x3, y3 -> x4, y4.
  if o4 == 0 and onSegment(x3, y3, x2, y2, x4, y4) then return true end                             -- x3, y3, x4, y4 and x2, y2 are collinear and x2, y2 lies on segment x3, y3 -> x4, y4
  return false                                                                                      -- No intersection.
end

-- Function to find the intersection point between two line segments.
local function intersection2LineSegs(p1, p2, q1, q2)
  local x1, y1, x2, y2, x3, y3, x4, y4 = p1.x, p1.y, p2.x, p2.y, q1.x, q1.y, q2.x, q2.y
  if not segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) then
    return nil                                                                                      -- No intersection.
  end
  local num1 = x1 * y2 - y1 * x2
  local num2 = x3 * y4 - y3 * x4
  local invDenom = 1.0 / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
  return vec3((num1 * (x3 - x4) - (x1 - x2) * num2) * invDenom, (num1 * (y3 - y4) - (y1 - y2) * num2) * invDenom )
end

-- Function to find the intersection between a 2D line segment and a circle.
local function intersectionLineSegmentCircle(x1, y1, x2, y2, cx, cy, r)
  local dx = x2 - x1
  local dy = y2 - y1
  local a = dx * dx + dy * dy
  local b = 2.0 * (dx * (x1 - cx) + dy * (y1 - cy))
  local c = (x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy) - r * r

  -- Calculate the discriminant.
  local discriminant = b * b - 4.0 * a * c
  if discriminant < 0 then
    return nil
  end

  -- Calculate the two possible values of t.
  local sqrtDiscriminant = sqrt(discriminant)
  local denom = 1.0 / (2.0 * a)
  local t1 = (-b + sqrtDiscriminant) * denom
  local t2 = (-b - sqrtDiscriminant) * denom

  local intersectionPoints = {}

  -- Check if the solutions are within the line segment (0 <= t <= 1).
  if t1 >= 0.0 and t1 <= 1.0 then
    table.insert(intersectionPoints, { x1 + t1 * dx, y1 + t1 * dy })
  end
  if t2 >= 0.0 and t2 <= 1.0 then
    table.insert(intersectionPoints, { x1 + t2 * dx, y1 + t2 * dy })
  end

  -- Return the intersection points (if any).
  if #intersectionPoints == 0 then
    return nil
  else
    return intersectionPoints
  end
end

-- Function to project point p onto the line defined by points a and b (in 2D).
local function projectPointToLine(p, a, b)

  local px, py, x1, y1, x2, y2 = p.x, p.y, a.x, a.y, b.x, b.y

  -- Calculate the direction vector AB.
  local ABx = x2 - x1
  local ABy = y2 - y1

  -- Calculate the vector AP.
  local APx = px - x1
  local APy = py - y1

  -- Calculate the dot product of AB and AP.
  local AB_AB = ABx * ABx + ABy * ABy
  local AB_AP = ABx * APx + ABy * APy

  -- Calculate the projection scalar.
  local t = AB_AP / AB_AB

  -- Calculate the projected point's coordinates.
  local projx = x1 + t * ABx
  local projy = y1 + t * ABy

  return vec3(projx, projy, p.z)
end

-- Attemps to fit a circle (2D) to three given points.
local function circle2DFrom3Points(p1, p2, p3)
  local p1x, p1y, p2x, p2y, p3x, p3y = p1.x, p1.y, p2.x, p2.y, p3.x, p3.y
  local dot22 = p2x * p2x + p2y * p2y
  local bc = (p1x * p1x + p1y * p1y - dot22) * 0.5
  local cd = (dot22 - p3x * p3x - p3y * p3y) * 0.5
  local det = (p1x - p2x) * (p2y - p3y) - (p2x - p3x) * (p1y - p2y)
  if abs(det) < 1e-12 then
    return nil, nil
  end
  local detInv = 1.0 / det
  local cx = (bc * (p2y - p3y) - cd * (p1y - p2y)) * detInv
  local cy = ((p1x - p2x) * cd - (p2x - p3x) * bc) * detInv
  return vec3(cx, cy)
end

-- Performs spherical linear interpolation between two vectors.
local function slerp(v1, v2, t)
  local dot = v1:dot(v2)
  if dot > 1.0 then dot = 1.0 end
  if dot < -1.0 then dot = -1.0 end

  local theta = acos(dot) * t
  local sin_theta_1_minus_t = sin((1 - t) * theta)
  local sin_theta_t = sin(t * theta)

  local v2_proj_on_v1 = v1 * dot
  local relative_vec = v2 - v2_proj_on_v1
  relative_vec:normalize()
  return v1 * sin_theta_1_minus_t + relative_vec * sin_theta_t
end

-- Computes the (small) angle between two unit vectors, in radians.
local function angleBetweenVecsNorm(a, b) return acos(a:dot(b)) end

-- Computes the (small) angle between two vectors of arbitrary length, in radians.
local function angleBetweenVecs(a, b) return angleBetweenVecsNorm(a:normalized(), b:normalized()) end

-- Computes the (small) angle between two vectors (converted to 2D first).
local function angleBetweenVecs2D(a, b) return angleBetweenVecs(vec3(a.x, a.y, 0.0), vec3(b.x, b.y, 0.0)) end

-- Computes the clockwise angle between the two given vectors, in 2D
local function clockwiseAngleBetweenVecs2D(v1, v2)
  local v1x, v1y, v2x, v2y = v1.x, v1.y, v2.x, v2.y
  local mag_v1 = sqrt(v1x^2 + v1y^2)
  local mag_v2 = sqrt(v2x^2 + v2y^2)
  local dot_product = (v1x * v2x) + (v1y * v2y)
  local angle = acos(dot_product / (mag_v1 * mag_v2))
  local cross_product = (v1x * v2y) - (v1y * v2x)
  if cross_product < 0.0 then
    return angle
  else
    return twoPi - angle
  end
end

-- Rotates vector v around unit axis k, by angle theta (in radians).
-- [This function uses the standard Rodrigues formula].
local function rotateVecAroundAxis(v, k, theta)
  local c = cos(theta)
  return v * c + k:cross(v) * sin(theta) + k * k:dot(v) * (1.0 - c)
end

-- Computes the rotation between two vectors.
local function getRotationBetweenVecs(v1, v2)
  v1:normalize()
  v2:normalize()
  local dot = v1:dot(v2)
  if dot > 0.999999 then
      return quat(0, 0, 0, 1)
  end
  if dot < -0.999999 then
    local orthogonal = xAxis:cross(v1)
    if sqrt(orthogonal.x^2 + orthogonal.y^2 + orthogonal.z^2) < 0.0001 then
        orthogonal = yAxis:cross(v1)
    end
    orthogonal:normalize()
    return quat(orthogonal.x, orthogonal.y, orthogonal.z, 0):normalized()
  end
  local cross = v1:cross(v2)
  local w = sqrt((1 + dot) * 2)
  local inv_w = 1.0 / w
  return quat(cross.x * inv_w, cross.y * inv_w, cross.z * inv_w, w * 0.5):normalized()
end

-- Function to rotate a vector by a quaternion.
local function rotateVecByQuaternion(v, q)
  q:normalize()
  local qOut = (q * quat(v.x, v.y, v.z, 0)) * quat(-q.x, -q.y, -q.z, q.w)
  return vec3(qOut.x, qOut.y, qOut.z)
end

local function isPointOnLeftSideOfLine(p, p1, p2) return (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x) > 0 end

-- Determines if the given point is inside the given 2D Axis-Aligned bounding box.
local function isInBox(p, box)
  local x, y = p.x, p.y
  return x >= box.xMin and x <= box.xMax and y >= box.yMin and y <= box.yMax
end

-- Determines if the given point is directly over the terrain (rather than over a bridge).
local function isOverTerrain(p)
  local zTerrain = core_terrain.getTerrainHeight(p)
  tmp:set(p.x, p.y, p.z + 4.5)
  local zCast = tmp.z - castRayStatic(tmp, downVec, 1000)
  return abs(zTerrain - zCast) < 0.25
end

-- Returns a random integer in range [a, b] inclusive.
local function randomInRange(a, b)
  if a > b then
    a, b = b, a
  end
  return random(a, b)
end


-- Public interface.
M.drawSphere =                                            drawSphere
M.drawGroupSphere =                                       drawGroupSphere
M.drawSphereHighlight =                                   drawSphereHighlight
M.drawSphereHighlightRed =                                drawSphereHighlightRed
M.drawSphereHighlightPurple =                             drawSphereHighlightPurple
M.drawHighlightSphereRad =                                drawHighlightSphereRad

M.drawPurpleLine =                                        drawPurpleLine

M.drawAutoJctMarkup =                                     drawAutoJctMarkup

M.isMouseHoveringOverTerrain =                            isMouseHoveringOverTerrain
M.mouseOnMapPos =                                         mouseOnMapPos
M.isMouseOverNode =                                       isMouseOverNode
M.isMouseCloseToNode =                                    isMouseCloseToNode

M.getFilenameFromPath =                                   getFilenameFromPath
M.removeExtension =                                       removeExtension
M.removeFileNameFromPath =                                removeFileNameFromPath

M.computeRoadLength =                                     computeRoadLength
M.getHalfWidthAtNode =                                    getHalfWidthAtNode
M.findBounds =                                            findBounds
M.computeDivIndicesFromNode =                             computeDivIndicesFromNode
M.isInTunnel =                                            isInTunnel

M.computeAABB2DGroup =                                    computeAABB2DGroup
M.computeAABB2DAllRoads =                                 computeAABB2DAllRoads

M.tryAddGroupIdxToRoad =                                  tryAddGroupIdxToRoad

M.round2 =                                                round2

M.polyLerp =                                              polyLerp

M.sqDist2D =                                              sqDist2D
M.distance2D =                                            distance2D
M.intersection2Lines =                                    intersection2Lines
M.intersection2LineSegs =                                 intersection2LineSegs
M.intersectionLineSegmentCircle =                         intersectionLineSegmentCircle
M.projectPointToLine =                                    projectPointToLine
M.circle2DFrom3Points =                                   circle2DFrom3Points

M.slerp =                                                 slerp
M.angleBetweenVecs =                                      angleBetweenVecs
M.angleBetweenVecs2D =                                    angleBetweenVecs2D
M.clockwiseAngleBetweenVecs2D =                           clockwiseAngleBetweenVecs2D

M.rotateVecAroundAxis =                                   rotateVecAroundAxis
M.getRotationBetweenVecs =                                getRotationBetweenVecs
M.rotateVecByQuaternion =                                 rotateVecByQuaternion

M.isPointOnLeftSideOfLine =                               isPointOnLeftSideOfLine

M.isInBox =                                               isInBox
M.isOverTerrain =                                         isOverTerrain

M.randomInRange =                                         randomInRange

return M