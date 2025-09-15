-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local laneInfoDivStep = 10                                                                          -- The number of lateral division between each lane number marking.

local groupPolygonElevVec = vec3(0, 0, 5)                                                           -- A vector used when rendering group creation 'fences' (polygons).

local refLineThickness = 4                                                                          -- The thickness of the road reference line.
local roadThickness = 2                                                                             -- The thickness of the road outline mesh.
local guidelineThickness = 3                                                                        -- The thickness of the road guidelines.
local guidelineThickness2 = 5                                                                       -- The thickness of the selected road guidelines.
local linkThickness = 11                                                                            -- The thickness of the link proposal.
local overlayThickness = 5                                                                          -- The thickness of the overlay road polylines.
local groupPolygonThickness = 7                                                                     -- The thickness of the group polygon polyline.
local groupPolygonThicknessTr = 3                                                                   -- The thickness of the group polygon polyline closing line segment.

local guidelineLength = 200.0                                                                       -- The length of the road guidelines, in meters.

local cylRaised = vec3(0, 0, 5.0)                                                                   -- The length of the cylinder used to render terraform range visualisation spheres.

local jctVisBoxSize = 7.0                                                                           -- The box (half) width/height when drawing a proposed junction.

local roadColor = color(12, 63, 211, 255)                                                           -- The colour of road visualisations.
local fullWhite = color(255, 255, 255, 255)                                                         -- The colour of SELECTED road visualisations.
local linkColor = color(0, 120, 207, 255)                                                           -- The colour of proposed link visualisation lines.
local meshOutlineColor = color(255, 255, 255, 64)                                                   -- The colour of (non-selected) static mesh outlines.
local selectedMeshOutlineColor = color(0, 0, 0, 255)                                                -- The colour of (selected) static mesh outlines.
local overlayColor = color(0, 0, 0, 255)                                                            -- The colour of overlay polylines/nodes.
local overlayHighlightColor = color(240, 240, 240, 255)                                             -- The colour of the selected overlay polyline/nodes.
local tunnelColor = color(150, 150, 150, 255)                                                       -- The colour of auto tunnel visualisations.
local layerColor = color(255, 255, 255, 64)                                                         -- The colour of (non-selected) layer visualisations.
local selectedLayerColor = color(255, 255, 255, 168)                                                -- The colour of (selected) layer visualisations.
local selLayerRed = color(255, 50, 50, 255)                                                         -- The red colour of (selected) layer internal details.
local selLayerGreen = color(50, 255, 50, 255)                                                       -- The green colour of (selected) layer internal details.
local selLayerBlue = color(50, 50, 255, 255)                                                        -- The blue colour of (selected) layer internal details.
local refLineColor = color(249, 224, 62, 255)                                                       -- The colour of the road road reference line.
local guidelineColor = color(66, 70, 81, 255)                                                       -- The colour of the road guidelines.
local guidelineColor2 = color(21, 24, 32, 255)                                                      -- The colour of the selected road guidelines.
local nodeHighlightColor = color(2, 29, 118, 255)                                                   -- The colour of the highlight at each road reference node.
local groupPolygonColorTr = color(127, 127, 127, 255)                                               -- The colour of the closing line segment of the group polygon polyline.
local terraVisColour = ColorF(0.5, 0, 0, 0.01)                                                      -- The colour of the terraforming range visualisation spheres.
local terraVisColourSingle = ColorF(0.5, 0, 0, 0.1)                                                 -- The colour of the terraforming range visualisation spheres.
local textA = color(0, 0, 0, 255)                                                                   -- The markup text foreground colour.
local textB = color(255, 255, 255, 255)                                                             -- The markup text background colour.
local laneDirColor = color(35, 147, 184, 255)                                                       -- The lane direction markup colour.
local laneColours = {                                                                               -- The lane type colours, in the road surface visualisation.
  road_lane = color(96, 101, 112, 255),
  sidewalk = color(167, 174, 191, 255),
  shoulder = color(40, 43, 51, 255),
  island = color(200, 200, 200, 255) }

local innerCullLimit = 150.0                                                                        -- The max distance at which to draw the secondary details, in meters.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A utilities module.
local dbgDraw = require('utils/debugDraw')

-- Private constants.
local min, max, abs, floor, ceil = math.min, math.max, math.abs, math.floor, math.ceil
local sqrt, twoPi, deg = math.sqrt, math.pi * 2.0, math.deg
local random, randomseed = math.random, math.randomseed
local tmp0, tmp1 = vec3(0, 0), vec3(0, 0)

local guidelines, selectedGuidelines, isInMulti = {}, {}, {}
local innerCullLimSq = innerCullLimit * innerCullLimit

local clRaise = vec3(0, 0, 0.01)                                                                    -- A vector used to raise the road centerline slightly.
local xAxis, yAxis, zAxis = vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1)                             -- The unit axis vectors.
local downVec, raised = -zAxis, zAxis * 10.0
local raisedBig = zAxis * 800.0

local root2Over2 = sqrt(2) * 0.5
local rot_q0 = quat(0, 0, 0, 1)                                                                     -- Some common rotations (as quaternions).
local rot_q90 = quat(0, 0, root2Over2, root2Over2)
local rot_q180 = quat(0, 0, 1, 0)
local rot_q270 = quat(0, 0, -root2Over2, root2Over2)


-- Render the surface mesh.
-- [This consists of a triangulated covering of the road surface, bottom and sides].
local function renderRoadSurface(rData, laneKeys, camPos)
  local rD1, numDivs = rData[1], #rData
  for k = 1, #laneKeys do
    local laneKey = laneKeys[k]
    local col = laneColours[rD1[laneKey][8]]
    for j = 2, numDivs do
      local ld1 = rData[j - 1][laneKey]
      local b1 = ld1[1]
      if b1:squaredDistance(camPos) < innerCullLimSq then
        local ld2 = rData[j][laneKey]
        local b2, f1 = ld1[2], ld2[1]
        dbgDraw.drawTriSolid(b1, b2, f1, col, true)                                                 -- Top quad (the primary faces).
        dbgDraw.drawTriSolid(b2, ld2[2], f1, col, true)
      end
    end
  end
end

-- Renders the road outline mesh (wireframe).
local function renderRoadOutline(renderData, laneKeys, isSelectedRoad, camPos)
  local col, thickness = roadColor, roadThickness
  if isSelectedRoad then
    col, thickness = fullWhite, roadThickness * 2
  end

  -- The line segment cross sectional lines, and lines joining to previous cross section.
  local numLaneKeys = #laneKeys
  for j = 2, #renderData do
    local div1, div2 = renderData[j - 1], renderData[j]
    for k = 1, numLaneKeys do
      local laneKey = laneKeys[k]
      local ld1, ld2 = div1[laneKey], div2[laneKey]
      local ld1_1, ld1_2, ld2_1, ld2_2 = ld1[1], ld1[2], ld2[1], ld2[2]
      dbgDraw.drawLineInstance_MinArg(ld1_1, ld1_2, thickness, col)                                 -- Draw the primary cross section quadrilateral lines for div1.
      dbgDraw.drawLineInstance_MinArg(ld2_1, ld2_2, thickness, col)                                 -- Draw the primary cross section quadrilateral lines for div2.
      dbgDraw.drawLineInstance_MinArg(ld1_1, ld2_1, thickness, col)                                 -- Draw the primary joining lines from div1 to div2 (top lines).
      dbgDraw.drawLineInstance_MinArg(ld1_2, ld2_2, thickness, col)
    end
  end
end

-- Renders the road reference line/centerline.
-- [A line going down the center of the road, between the left and right sides].
local function renderRoadCenterline(renderData, camPos, isGroupMode)
  for j = 2, #renderData do
    local div1, div2 = renderData[j - 1], renderData[j]
    if div1[-1] then                                                                                -- Either lane '-1' or lane '1' must exist, so use whichever the road has.
      local p1 = div1[-1][2]
      if isGroupMode or (p1:squaredDistance(camPos) < innerCullLimSq) then
        dbgDraw.drawLineInstance_MinArg(p1 + clRaise, div2[-1][2] + clRaise, refLineThickness, refLineColor)
      end
    else
      local p1 = div1[1][1]
      if isGroupMode or (p1:squaredDistance(camPos) < innerCullLimSq) then
        dbgDraw.drawLineInstance_MinArg(p1 + clRaise, div2[1][1] + clRaise, refLineThickness, refLineColor)
      end
    end
  end
end

-- Add to the road guidelines.
-- [These are lines which extend from the start and end of the road, and help the user to line up roads to a grid].
local function addThisRoadsGuidelines(r, renderData, guidelines, isSelectedRoad)
  local idx1, idx2 = -1, 2
  if renderData[1][1] then
    idx1, idx2 = 1, 1
  end

  local ctr = #guidelines + 1
  if not r.nodes[1].isLocked then
    local cp1, cp2 = renderData[1][idx1][idx2], renderData[2][idx1][idx2]
    local pStart, p2 = vec3(cp1.x, cp1.y, cp1.z), vec3(cp2.x, cp2.y, cp2.z)
    local sTgt = pStart - p2
    sTgt:normalize()
    if isSelectedRoad then
      selectedGuidelines[#selectedGuidelines + 1] = { pStart, pStart + sTgt * guidelineLength }
    else
      guidelines[ctr] = { pStart, pStart + sTgt * guidelineLength }
      ctr = ctr + 1
    end
  end

  if not r.nodes[#r.nodes].isLocked then
    local cp1, cp2 = renderData[#renderData][idx1][idx2], renderData[#renderData - 1][idx1][idx2]
    local pEnd, p3 = vec3(cp1.x, cp1.y, cp1.z), vec3(cp2.x, cp2.y, cp2.z)
    local eTgt = pEnd - p3
    eTgt:normalize()
    if isSelectedRoad then
      selectedGuidelines[#selectedGuidelines + 1] = { pEnd, pEnd + eTgt * guidelineLength }
    else
      guidelines[ctr] = { pEnd, pEnd + eTgt * guidelineLength }
      ctr = ctr + 1
    end
  end
end

-- Renders the road guidelines/measurement markups.
local function renderGuidelines(camPos)
  -- Draw all the guidelines, regardless of road.
  for i = 1, #guidelines do
    local gL = guidelines[i]
    tmp0:set(gL[1].x, gL[1].y, 0)
    tmp1:set(gL[2].x, gL[2].y, 0)
    local vec = tmp1 - tmp0
    for j = 1, 100 do
      local q1, q2 = (j - 1) * 0.01, j * 0.01
      local p1, p2 = tmp0 + vec * q1 + raisedBig, tmp0 + vec * q2 + raisedBig
      p1.z = p1.z - castRayStatic(p1, downVec, 1000) + 0.02
      p2.z = p2.z - castRayStatic(p2, downVec, 1000) + 0.02
      dbgDraw.drawLineInstance_MinArg(p1, p2, guidelineThickness, guidelineColor)
    end
  end
  for i = 1, #selectedGuidelines do
    local gL = selectedGuidelines[i]
    tmp0:set(gL[1].x, gL[1].y, 0)
    tmp1:set(gL[2].x, gL[2].y, 0)
    local vec = tmp1 - tmp0
    for j = 1, 100 do
      local q1, q2 = (j - 1) * 0.01, j * 0.01
      local p1, p2 = tmp0 + vec * q1 + raisedBig, tmp0 + vec * q2 + raisedBig
      p1.z = p1.z - castRayStatic(p1, downVec, 1000) + 0.02
      p2.z = p2.z - castRayStatic(p2, downVec, 1000) + 0.02
      dbgDraw.drawLineInstance_MinArg(p1, p2, guidelineThickness2, guidelineColor2)
    end
  end

  -- Draw spheres at the intersection points.
  for i = 1, #selectedGuidelines do
    local sGL = selectedGuidelines[i]
    local p1, p2 = sGL[1] + raisedBig, sGL[2] + raisedBig
    p1.z = p1.z - castRayStatic(p1, downVec, 1000) + 0.02
    p2.z = p2.z - castRayStatic(p2, downVec, 1000) + 0.02
    for j = 1, #guidelines do
      local tGL = guidelines[j]
      local q1, q2 = tGL[1] + raisedBig, tGL[2] + raisedBig
      q1.z = q1.z - castRayStatic(q1, downVec, 1000) + 0.02
      q2.z = q2.z - castRayStatic(q2, downVec, 1000) + 0.02
      local pInt2D = util.intersection2LineSegs(p1, p2, q1, q2)
      if pInt2D then
        pInt2D = pInt2D + raisedBig
        pInt2D.z = pInt2D.z - castRayStatic(pInt2D, downVec, 1000) + 0.02
        dbgDraw.drawSphere(pInt2D, 0.1 * sqrt(pInt2D:distance(camPos)), guidelineColor2)
        local dist = pInt2D:distance(p1)
        local v1 = p2 - p1
        v1:normalize()
        local v2 = q2 - q1
        v2:normalize()
        local v2Alt = -v2
        local theta1, theta2 = util.angleBetweenVecs2D(v1, v2), util.angleBetweenVecs2D(v1, v2Alt)
        if theta1 <= theta2 then
          dbgDraw.drawTextAdvanced(pInt2D, tostring(util.round2(dist)) .. ' m ', textA, true, false, textB)
          dbgDraw.drawTextAdvanced(pInt2D, tostring(util.round2(deg(theta1))) .. ' ° ', textA, true, false, textB)
          local vec5 = v1 * 5.0
          local dSq1 = (pInt2D + util.rotateVecAroundAxis(vec5, zAxis, theta1)):squaredDistanceToLineSegment(q1, q2)
          local dSq2 = (pInt2D + util.rotateVecAroundAxis(vec5, zAxis, -theta1)):squaredDistanceToLineSegment(q1, q2)
          local angleSign = 1
          if dSq2 < dSq1 then
            angleSign = -1
          end
          local pts = {}
          for t = 0, 20 do
            pts[t + 1] = pInt2D + util.rotateVecAroundAxis(vec5, zAxis, t * 0.05 * theta1 * angleSign) + raisedBig
            pts[t + 1].z = pts[t + 1].z - castRayStatic(pts[t + 1], downVec, 1000) + 0.02
          end
          for k = 2, #pts do
            dbgDraw.drawLineInstance_MinArg(pts[k - 1], pts[k], guidelineThickness, guidelineColor2)
          end
        else
          dbgDraw.drawTextAdvanced(pInt2D, tostring(util.round2(dist)) .. ' m ', textA, true, false, textB)
          dbgDraw.drawTextAdvanced(pInt2D, tostring(util.round2(deg(theta2))) .. ' ° ', textA, true, false, textB)
          local vec5 = v1 * 5.0
          local dSq1 = (pInt2D + util.rotateVecAroundAxis(vec5, zAxis, theta2)):squaredDistanceToLineSegment(q1, q2)
          local dSq2 = (pInt2D + util.rotateVecAroundAxis(vec5, zAxis, -theta2)):squaredDistanceToLineSegment(q1, q2)
          local angleSign = 1
          if dSq2 < dSq1 then
            angleSign = -1
          end
          local pts = {}
          for t = 0, 20 do
            pts[t + 1] = pInt2D + util.rotateVecAroundAxis(vec5, zAxis, t * 0.05 * theta2 * angleSign) + raisedBig
            pts[t + 1].z = pts[t + 1].z - castRayStatic(pts[t + 1], downVec, 1000) + 0.02
          end
          for k = 2, #pts do
            dbgDraw.drawLineInstance_MinArg(pts[k - 1], pts[k], guidelineThickness, guidelineColor2)
          end
        end
      end
    end
  end
end

-- Renders the node spheres (or square prisms for locked nodes).
local function renderNodeSpheres(nodes, camPos, isSelectedRoad, selectedNodeIdx)
  local numNodes = #nodes
  for j = 1, numNodes do
    local col = nodeHighlightColor
    if isSelectedRoad and j == selectedNodeIdx then
      col = fullWhite
    end
    local node = nodes[j]
    local pos = node.p
    local cam2Pos = sqrt(pos:distance(camPos))
    if node.isLocked then
      local nDist = 0.175 * cam2Pos
      local sqC = Point2F(nDist, nDist)
      tmp0:set(0, 0, nDist)
      dbgDraw.drawSquarePrism(pos - tmp0, pos + tmp0, sqC, sqC, col)
    else
      dbgDraw.drawSphere(pos, 0.1 * cam2Pos, col)
    end
  end
end

-- Renders the node numbering.
local function renderNodeNumbering(nodes, camPos)
  local numNodes = #nodes
  for j = 1, numNodes do
    local pos = nodes[j].p
    if pos:squaredDistance(camPos) < innerCullLimSq then
      dbgDraw.drawTextAdvanced(pos, tostring(j), textA, true, false, textB)
    end
  end
end

-- Renders the lane numbering.
local function renderLaneNumbering(renderData, laneKeys, isGroupMode, camPos)
  local numDivs, numLaneKeys = #renderData, #laneKeys
  if not isGroupMode then                                                                           -- Do not display lane numbering in group auditions.
    for j = 1, numDivs, laneInfoDivStep do                                                          -- Lane numbers will appear at every n lateral divisions.
      local div = renderData[j]
      for k = 1, numLaneKeys do
        local lane = laneKeys[k]
        local pos = div[lane][7]
        if pos:squaredDistance(camPos) < innerCullLimSq then
          dbgDraw.drawTextAdvanced(pos, tostring(lane), textA, true, false, textB)
        end
      end
    end
  end
end

-- Renders the lane direction triangles.
local function renderLaneDirectionTriangles(renderData, laneKeys, isGroupMode, camPos)
  local lIdx = -1
  if renderData[1][1] then
    lIdx = 1
  end
  local roadLanes = {}                                                                              -- Store the indices of all the road lanes.
  for k = 1, #laneKeys do
    local lIdx = laneKeys[k]
    if renderData[1][lIdx][8] == 'road_lane' then
      roadLanes[lIdx] = true
    end
  end
  local numDivs = #renderData
  for j = 1, numDivs do
    local div = renderData[j]
    if isGroupMode or (div[lIdx][7]:squaredDistance(camPos) < innerCullLimSq) then
      for lane, _ in pairs(roadLanes) do
        local dL = div[lane]
        if lane > 0 then
          local dPL = renderData[min(numDivs, j + 1)][lane]
          dbgDraw.drawTriSolid(dPL[2], dPL[1], (dL[1] + dL[2]) * 0.5, laneDirColor, true)
        else
          local dML = renderData[max(1, j - 1)][lane]
          dbgDraw.drawTriSolid(dML[1], dML[2], (dL[1] + dL[2]) * 0.5, laneDirColor, true)
        end
      end
    end
  end
end

-- Renders the group polygon/fence (when user is creating a group).
local function renderGroupPolygonFence(gPolygon)
  local gPolygonLen = #gPolygon
  for i = 1, gPolygonLen do
    local p = gPolygon[i]
    util.drawGroupSphere(p)
    dbgDraw.drawTextAdvanced(p, tostring(i), textA, true, false, textB)
  end
  for i = 2, gPolygonLen do
    local p1, p2 = gPolygon[i - 1], gPolygon[i]
    local p3, p4 = p1 + groupPolygonElevVec, p2 + groupPolygonElevVec
    dbgDraw.drawLineInstance_MinArg(p1, p2, groupPolygonThickness, linkColor)
    dbgDraw.drawLineInstance_MinArg(p3, p4, groupPolygonThickness, linkColor)
    dbgDraw.drawLineInstance_MinArg(p1, p3, groupPolygonThickness, linkColor)
    dbgDraw.drawLineInstance_MinArg(p2, p4, groupPolygonThickness, linkColor)
  end
  if gPolygonLen > 2 then
    local p1, p2 = gPolygon[1], gPolygon[gPolygonLen]
    local p3, p4 = p1 + groupPolygonElevVec, p2 + groupPolygonElevVec
    dbgDraw.drawLineInstance_MinArg(p1, p2, groupPolygonThicknessTr, groupPolygonColorTr)
    dbgDraw.drawLineInstance_MinArg(p3, p4, groupPolygonThicknessTr, groupPolygonColorTr)
    dbgDraw.drawLineInstance_MinArg(p1, p3, groupPolygonThicknessTr, groupPolygonColorTr)
    dbgDraw.drawLineInstance_MinArg(p2, p4, groupPolygonThicknessTr, groupPolygonColorTr)
  end
end

-- Renders the node highlights for a multi-selection.
local function renderMultiSelectNodes(roads, multi)
  for i = 1, #multi do
    local m = multi[i]
    local r = roads[m.r]
    if r and roads[m.r].nodes[m.n] and not r.isJctRoad then
      util.drawSphereHighlight(roads[m.r].nodes[m.n].p)
    end
  end
end

-- Renders a road auto tunnel.
local function renderAutoTunnel(rData, tnl, camPos)

  -- Determine the pipe center and lateral edge indices.
  local rD1, cIdx1, cIdx2, lIdx, rIdx = rData[1], -1, 3, nil, nil
  if not rD1[-1] then
    cIdx1, cIdx2 = 1, 4
  end
  for i = -20, 20 do
    if rD1[i] then
      lIdx = i
      break
    end
  end
  for i = 20, -20, -1 do
    if rD1[i] then
      rIdx = i
      break
    end
  end

  -- Compute the ring at each longitudinal point in the given section.
  local iStart, iEnd, radGran, radOffset = tnl.s, tnl.e, tnl.radGran, tnl.radOffset
  local thickness, zOffsetFromRoad = tnl.thickness, tnl.zOffsetFromRoad
  local protrudeS, protrudeE = tnl.protrudeS, tnl.protrudeE
  local theta, zVec, roadLen, rGranP1 = twoPi / radGran, zAxis * zOffsetFromRoad, #rData, radGran + 1
  local ringsI, ringsO = {}, {}
  local numRings = iEnd - iStart + 1
  for i = 1, numRings do
    local idx = iStart - 1 + i
    local rD = rData[idx]
    local rTan = rData[min(roadLen, idx + 1)][cIdx1][cIdx2] - rData[max(1, idx - 1)][cIdx1][cIdx2]
    rTan:normalize()                                                                                -- The road unit tangent vector.
    local rDLeft = rD[lIdx]
    local rNorm = rDLeft[5]                                                                         -- The road unit normal vector.
    local rLeft, rRight = rDLeft[4], rD[rIdx][3]                                                    -- The left-most and right-most lateral road points.
    local rCen = (rLeft + rRight) * 0.5                                                             -- The road lateral center.
    if i == 1 then
      rCen = rCen - rTan * protrudeS                                                                -- Protrude the start point, if required.
    elseif i == numRings then
      rCen = rCen + rTan * protrudeE                                                                -- Protrude the end point, if required.
    end
    local rWidth = rLeft:distance(rRight)                                                           -- The (lateral) road width at this longitudinal point.
    local nOff = rWidth + radOffset                                                                 -- The normal offset, based on the radii.
    local normWithRInner, normWithROuter = rNorm * (nOff - thickness), rNorm * (nOff + thickness)   -- The full normal offset vectors.
    local pCen = rCen + zVec                                                                        -- The pipe center.
    local ringI, ringO = {}, {}
    local zI = min(rLeft.z, rRight.z)
    local zO = zI - thickness
    for j = 1, rGranP1 do                                                                           -- Compute the cross-sectional rings of points aroung this lon. point.
      local angle = (j - 1) * theta
      ringI[j] = pCen + util.rotateVecAroundAxis(normWithRInner, rTan, angle)
      ringO[j] = pCen + util.rotateVecAroundAxis(normWithROuter, rTan, angle)
      ringI[j].z = max(ringI[j].z, zI)
      ringO[j].z = max(ringO[j].z, zO)
    end
    ringsI[i], ringsO[i] = ringI, ringO
  end

  -- Create the vertex arrays.
  local numRings = #ringsI
  local vertsO, vertsI, ctr = {}, {}, 1
  for i = 1, numRings do
    local ringI, ringO = ringsI[i], ringsO[i]
    for j = 1, rGranP1 do
      vertsI[ctr], vertsO[ctr] = ringI[j], ringO[j]
      ctr = ctr + 1
    end
  end

  -- Draw the faces.
  local numRingsMinus1, rGranP1, numVerts = numRings - 1, radGran + 1, #vertsO
  for i = 1, numRingsMinus1 do
    local i1 = min(numVerts, (i - 1) * rGranP1 + 1)
    local i2 = min(numVerts, i1 + 1)
    local i3 = min(numVerts, i1 + rGranP1)
    local i4 = min(numVerts, i2 + rGranP1)
    for j = 1, radGran do
      local tA, tB, tC, tD = vertsO[i1], vertsO[i2], vertsO[i3], vertsO[i4]
      dbgDraw.drawTriSolid(tA, tC, tB, tunnelColor, true)                                           -- Outer faces.
      dbgDraw.drawTriSolid(tA, tB, tC, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tD, tB, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tB, tD, tunnelColor, true)
      dbgDraw.drawLineInstance_MinArg(tA, tB, roadThickness, roadColor)
      dbgDraw.drawLineInstance_MinArg(tC, tD, roadThickness, roadColor)
      dbgDraw.drawLineInstance_MinArg(tA, tC, roadThickness, roadColor)
      dbgDraw.drawLineInstance_MinArg(tB, tD, roadThickness, roadColor)
      if tA:squaredDistance(camPos) < innerCullLimSq then
        tA, tB, tC, tD = vertsI[i1], vertsI[i2], vertsI[i3], vertsI[i4]
        dbgDraw.drawTriSolid(tA, tC, tB, tunnelColor, true)                                         -- Inner faces.
        dbgDraw.drawTriSolid(tA, tB, tC, tunnelColor, true)
        dbgDraw.drawTriSolid(tC, tD, tB, tunnelColor, true)
        dbgDraw.drawTriSolid(tC, tB, tD, tunnelColor, true)
        dbgDraw.drawLineInstance_MinArg(tA, tB, roadThickness, roadColor)
        dbgDraw.drawLineInstance_MinArg(tC, tD, roadThickness, roadColor)
        dbgDraw.drawLineInstance_MinArg(tA, tC, roadThickness, roadColor)
        dbgDraw.drawLineInstance_MinArg(tB, tD, roadThickness, roadColor)
      end
      i1, i2, i3, i4 = min(numVerts, i1 + 1), min(numVerts, i2 + 1), min(numVerts, i3 + 1), min(numVerts, i4 + 1)
    end
  end

  -- Start cap vertices.
  local capVertsS, ctr = {}, 1
  for i = 1, rGranP1 do
    capVertsS[ctr] = { x = vertsO[i].x, y = vertsO[i].y, z = vertsO[i].z }
    capVertsS[ctr + 1] = { x = vertsI[i].x, y = vertsI[i].y, z = vertsI[i].z }
    ctr = ctr + 2
  end

  -- Start cap faces.
  if vertsO[1]:squaredDistance(camPos) < innerCullLimSq then
    local vIdx = 1
    local numVerts = #capVertsS
    for i = 1, radGran do
      local tA, tB, tC, tD = capVertsS[min(numVerts, vIdx)], capVertsS[min(numVerts, vIdx + 2)], capVertsS[min(numVerts, vIdx + 3)], capVertsS[min(numVerts, vIdx + 1)]
      dbgDraw.drawTriSolid(tB, tD, tA, tunnelColor, true)
      dbgDraw.drawTriSolid(tB, tA, tD, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tD, tB, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tB, tD, tunnelColor, true)
      dbgDraw.drawLineInstance_MinArg(tA, tD, roadThickness, roadColor)
      vIdx = vIdx + 2
    end
  end

  -- End cap vertices
  local capVertsE, ctr, numVertsO = {}, 1, #vertsO
  for i = 1, rGranP1 do
    local idx = numVertsO - i + 1
    capVertsE[ctr] = { x = vertsO[idx].x, y = vertsO[idx].y, z = vertsO[idx].z }
    capVertsE[ctr + 1] = { x = vertsI[idx].x, y = vertsI[idx].y, z = vertsI[idx].z }
    ctr = ctr + 2
  end

  -- End cap faces.
  if vertsO[#vertsO]:squaredDistance(camPos) < innerCullLimSq then
    local vIdx = 1
    local numVerts = #capVertsE
    for i = 1, radGran do
      local tA, tB, tC, tD = capVertsE[min(numVerts, vIdx)], capVertsE[min(numVerts, vIdx + 2)], capVertsE[min(numVerts, vIdx + 3)], capVertsE[min(numVerts, vIdx + 1)]
      dbgDraw.drawTriSolid(tB, tD, tA, tunnelColor, true)
      dbgDraw.drawTriSolid(tB, tA, tD, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tD, tB, tunnelColor, true)
      dbgDraw.drawTriSolid(tC, tB, tD, tunnelColor, true)
      dbgDraw.drawLineInstance_MinArg(tA, tD, roadThickness, roadColor)
      vIdx = vIdx + 2
    end
  end
end

-- Computes the length of the given polyline.
local function computePolylineLengths(posns)
  local lens = { 0.0 }
  for i = 2, #posns do
    local iMinus1 = i - 1
    lens[i] = lens[iMinus1] + posns[iMinus1]:distance(posns[i])
  end
  return lens
end

-- Renders any layers which the user has requested to display.
local function renderLayer(road, layer, renderData, camPos, isSelectedLayer)
  local col = layerColor
  if isSelectedLayer then
    col = selectedLayerColor
  end

  local layerType = layer.type[0]
  if layerType == 0 then                                                                            -- TYPE: [SPAN LANE].
    local startDivIdx, endDivIdx = 1, #renderData
    if not layer.isSpanLong[0] then
      startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], road)
      endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], road)
    end

    local laneKeyL, laneKeyR = layer.laneMin[0], layer.laneMax[0]
    for j = startDivIdx + 1, endDivIdx do
      local div1, div2 = renderData[j - 1], renderData[j]
      local ld1L, ld2L = div1[laneKeyL], div2[laneKeyL]
      local ld1R, ld2R = div1[laneKeyR], div2[laneKeyR]
      local b1, b2, f1, f2 = nil, nil, nil, nil
      local offVec = ld1L[6] * layer.off[0]
      b1, b2 = ld1L[1] + offVec, ld1R[2] + offVec
      if b1:squaredDistance(camPos) < innerCullLimSq then
        f1, f2 = ld2L[1] + offVec, ld2R[2] + offVec
        dbgDraw.drawTriSolid(b1, b2, f1, col, true)
        dbgDraw.drawTriSolid(b1, f1, b2, col, true)
        dbgDraw.drawTriSolid(b2, f1, f2, col, true)
        dbgDraw.drawTriSolid(b2, f2, f1, col, true)
      end
    end
  elseif layerType == 1 then                                                                        -- TYPE: [OFFSET FROM LANE].
    local startDivIdx, endDivIdx = 1, #renderData
    if not layer.isSpanLong[0] then
      startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], road)
      endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], road)
    end

    local laneKey = layer.lane[0]
    local isLeftIdx = 2
    if layer.isLeft[0] then
      isLeftIdx = 1
    end
    local halfWidth = layer.width[0] * 0.5
    for j = startDivIdx + 1, endDivIdx do
      local div1, div2 = renderData[j - 1], renderData[j]
      local ld1, ld2 = div1[laneKey], div2[laneKey]
      local b1, b2, f1, f2 = nil, nil, nil, nil
      local p1, p2 = ld1[isLeftIdx] + ld1[6] * layer.off[0], ld2[isLeftIdx] + ld2[6] * layer.off[0]
      local latVec1, latVec2 = ld1[6] * halfWidth, ld2[6] * halfWidth
      b1, b2 = p1 - latVec1, p1 + latVec1
      f1, f2 = p2 - latVec2, p2 + latVec2
      dbgDraw.drawTriSolid(b1, b2, f1, col, true)
      dbgDraw.drawTriSolid(b1, f1, b2, col, true)
      dbgDraw.drawTriSolid(b2, f1, f2, col, true)
      dbgDraw.drawTriSolid(b2, f2, f1, col, true)
    end
  elseif layerType == 2 then                                                                        -- TYPE: [UNIQUE LATERAL PATCH - (NON-DECAL)].
    -- Compute the relevant points, using linear interpolation.
    local lMin, lMax = layer.laneMin[0], layer.laneMax[0]
    local qq = layer.off[0]
    local lengths = util.computeRoadLength(renderData)
    local pEval = qq * lengths[#lengths]                                                            -- The longitudinal evaluation position on the road, in meters.
    local lower, upper = util.findBounds(pEval, lengths)
    local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                          -- The q in [0, 1] between div points (linear interpolation).
    local pL1 = renderData[lower][lMin][1]
    local pL2 = renderData[upper][lMin][1]
    local pL = pL1 + q * (pL2 - pL1)                                                                -- The evaluation on the left edge.
    local pR1 = renderData[lower][lMax][2]
    local pR2 = renderData[upper][lMax][2]
    local pR = pR1 + q * (pR2 - pR1)                                                                -- The evaluation on the right edge.

    -- Compute the local tangent at the road centerline, along which to protrude the patch.
    local cenIdx1, cenIdx2 = 1, 1
    if renderData[1][-1] then
      cenIdx1, cenIdx2 = -1, 2
    end
    local tgt = renderData[upper][cenIdx1][cenIdx2] - renderData[lower][cenIdx1][cenIdx2]
    tgt:normalize()

    -- Compute the four quadrilateral vertices of the patch.
    local tgtVec = tgt * layer.width[0]
    local b1, b2 = pL, pR
    local f1, f2 = pL + tgtVec, pR + tgtVec
    dbgDraw.drawTriSolid(b1, b2, f1, col, true)
    dbgDraw.drawTriSolid(b1, f1, b2, col, true)
    dbgDraw.drawTriSolid(b2, f1, f2, col, true)
    dbgDraw.drawTriSolid(b2, f2, f1, col, true)
  elseif layerType == 3 then                                                                        -- TYPE: [UNIQUE DECAL PATCH].
    local lIdx = layer.lane[0]
    local lengths = util.computeRoadLength(renderData)
    local pEval = layer.off[0] * lengths[#lengths]                                                  -- The longitudinal evaluation position on the road, in meters.
    local lower, upper = util.findBounds(pEval, lengths)
    local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                          -- The q in [0, 1] between div points (linear interpolation).
    local pL = nil
    if layer.isLeft[0] then
      local pL1 = renderData[lower][lIdx][1]
      local pL2 = renderData[upper][lIdx][1]
      pL = pL1 + q * (pL2 - pL1)
    else
      local pL1 = renderData[lower][lIdx][2]
      local pL2 = renderData[upper][lIdx][2]
      pL = pL1 + q * (pL2 - pL1)
    end
    local n1 = renderData[lower][lIdx][5]
    local n2 = renderData[upper][lIdx][5]
    local nml = n1 + q * (n2 - n1)
    nml:normalize()
    local l1 = renderData[lower][lIdx][6]
    local l2 = renderData[upper][lIdx][6]
    local lat = l1 + q * (l2 - l1)
    lat:normalize()
    local tgt = nml:cross(lat)
    local pos = pL + lat * layer.pos[0]

    -- Compute the corner points for this decal patch.
    local extraSize = layer.size[0]
    local tgtHalf, latHalf = tgt * extraSize, lat * extraSize
    local p1 = pos - tgtHalf - latHalf
    local p2 = pos - tgtHalf + latHalf
    local p3 = pos + tgtHalf - latHalf
    local p4 = pos + tgtHalf + latHalf

    -- Render the patch as a filled square.
    dbgDraw.drawTriSolid(p1, p2, p3, col, true)
    dbgDraw.drawTriSolid(p1, p3, p2, col, true)
    dbgDraw.drawTriSolid(p2, p3, p4, col, true)
    dbgDraw.drawTriSolid(p2, p4, p3, col, true)
    dbgDraw.drawLineInstance_MinArg(p1, p2, roadThickness, fullWhite)
    dbgDraw.drawLineInstance_MinArg(p3, p4, roadThickness, fullWhite)
    dbgDraw.drawLineInstance_MinArg(p1, p3, roadThickness, fullWhite)
    dbgDraw.drawLineInstance_MinArg(p2, p4, roadThickness, fullWhite)

  elseif layerType == 4 then                                                                        -- TYPE: [ROAD-SPANNING STATIC MESH].

    randomseed(41225)

    local colLines = meshOutlineColor
    if isSelectedLayer then
      colLines = selectedMeshOutlineColor
    end

    local length = layer.extentsL
    local vertOffset = layer.vertOffset[0]
    local latOffset = layer.latOffset[0]
    local lIdx = layer.lane[0]
    local pIdx = 2
    if layer.isLeft[0] then
      pIdx = 1
    end
    local preRot = rot_q0
    local isCentered = abs(layer.boxXLeft - layer.boxXRight) < 0.1 and max(layer.boxXLeft, layer.boxXRight) > 1.0
    if layer.rot[0] == 1 then
      preRot = rot_q90
      length = layer.extentsW
      isCentered = abs(layer.boxYLeft - layer.boxYRight) < 0.1 and max(layer.boxYLeft, layer.boxYRight) > 1.0
    elseif layer.rot[0] == 2 then
      preRot = rot_q180
    elseif layer.rot[0] == 3 then
      preRot = rot_q270
      length = layer.extentsW
      isCentered = abs(layer.boxYLeft - layer.boxYRight) < 0.1 and max(layer.boxYLeft, layer.boxYRight) > 1.0
    end
    local extraSpacing = layer.spacing[0]
    local jitter = layer.jitter[0]
    local isGlobalZ = layer.useWorldZ[0]

    if not length then
      return
    end

    local rData = road.renderData
    local startDivIdx, endDivIdx = 1, #rData
    if not layer.isSpanLong[0] then
      startDivIdx = util.computeDivIndicesFromNode(layer.nMin[0], road)
      endDivIdx = util.computeDivIndicesFromNode(layer.nMax[0], road)
    end

    local posns, nmls, ctr = {}, {}, 1
    for j = startDivIdx, endDivIdx do
      local lData = rData[j][lIdx]
      local tgt = rData[min(#rData, j + 1)][lIdx][1] - rData[max(1, j - 1)][lIdx][1]
      tgt:normalize()
      local lat = lData[6]
      local nml = tgt:cross(lat)
      local orig = lData[pIdx]
      posns[ctr] = orig + (vertOffset * nml) + (latOffset * lat)
      nmls[ctr] = nml
      ctr = ctr + 1
    end

    -- Compute the positions and normal vectors for each mesh unit on the lane.
    local lens = computePolylineLengths(posns)
    local numMeshes = ceil(lens[#lens] / (length + extraSpacing))
    local q = 0.0
    local fPosns, fNmls = {}, {}
    local ctr = 1
    for _ = 1, numMeshes do
      local pos, nml = util.polyLerp(posns, nmls, lens, q)
      if pos then
        fPosns[ctr], fNmls[ctr] = pos, nml
        ctr = ctr + 1
      end
      q = q + length + extraSpacing
    end

    local q = 0.0
    numMeshes = #fPosns
    for j = 1, numMeshes do
      local p1, p2 = fPosns[j], fPosns[j + 1]
      local posOrig = p1
      if j == numMeshes then
        p1, p2 = fPosns[j - 1], fPosns[j]
        posOrig = p2
      end
      if p1 and p2 then
        local v = p2 - p1
        local pos = nil
        if isCentered then                                                                            -- If the static mesh has its origin at the center of mesh.
          pos = posOrig + v * 0.5
        else
          pos = posOrig
        end

        local tgt = v:normalized()
        local rot = quat()
        if isGlobalZ then
          tgt.z = 0.0
          rot:setFromDir(tgt, zAxis)
        else
          rot:setFromDir(tgt, fNmls[j])
        end
        rot = preRot * rot_q90 * rot

        rot.x = rot.x + (random() * 2 - 1) * jitter                                               -- Apply any random jittering, if requested.
        rot.y = rot.y + (random() * 2 - 1) * jitter
        rot.z = rot.z + (random() * 2 - 1) * jitter

        local fwd = xAxis:rotated(rot)
        local right = yAxis:rotated(rot)
        local up = zAxis:rotated(rot)
        local xLeft, yLeft, zLeft = -layer.boxXLeft * fwd, -layer.boxYLeft * right, -layer.boxZLeft * up
        local xRight, yRight, zRight = layer.boxXRight * fwd, layer.boxYRight * right, layer.boxZRight * up
        local b1 = pos + xLeft + yLeft + zLeft
        local b2 = pos + xLeft + yRight + zLeft
        local b3 = pos + xLeft + yLeft + zRight
        local b4 = pos + xLeft + yRight + zRight
        local f1 = pos + xRight + yLeft + zLeft
        local f2 = pos + xRight + yRight + zLeft
        local f3 = pos + xRight + yLeft + zRight
        local f4 = pos + xRight + yRight + zRight

        dbgDraw.drawLineInstance_MinArg(b1, b2, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b3, b4, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b1, b3, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b2, b4, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(f1, f2, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(f3, f4, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(f1, f3, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(f2, f4, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b1, f1, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b2, f2, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b3, f3, roadThickness, colLines)
        dbgDraw.drawLineInstance_MinArg(b4, f4, roadThickness, colLines)

        dbgDraw.drawTriSolid(b1, b2, b3, col, true)
        dbgDraw.drawTriSolid(b2, b3, b4, col, true)
        dbgDraw.drawTriSolid(f1, f2, f3, col, true)
        dbgDraw.drawTriSolid(f2, f3, f4, col, true)

        dbgDraw.drawTriSolid(b1, b3, b2, col, true)
        dbgDraw.drawTriSolid(b2, b4, b3, col, true)
        dbgDraw.drawTriSolid(f1, f3, f2, col, true)
        dbgDraw.drawTriSolid(f2, f4, f3, col, true)

        dbgDraw.drawTriSolid(b2, f2, b4, col, true)
        dbgDraw.drawTriSolid(f2, b4, f4, col, true)
        dbgDraw.drawTriSolid(b1, f1, b3, col, true)
        dbgDraw.drawTriSolid(f1, b3, f3, col, true)

        dbgDraw.drawTriSolid(b2, b4, f2, col, true)
        dbgDraw.drawTriSolid(f2, f4, b4, col, true)
        dbgDraw.drawTriSolid(b1, b3, f1, col, true)
        dbgDraw.drawTriSolid(f1, f3, b3, col, true)

        dbgDraw.drawTriSolid(b3, f3, b4, col, true)
        dbgDraw.drawTriSolid(f3, b4, f4, col, true)
        dbgDraw.drawTriSolid(b1, f1, f2, col, true)
        dbgDraw.drawTriSolid(b1, f2, b2, col, true)

        dbgDraw.drawTriSolid(b3, b4, f3, col, true)
        dbgDraw.drawTriSolid(f3, f4, b4, col, true)
        dbgDraw.drawTriSolid(b1, f2, f1, col, true)
        dbgDraw.drawTriSolid(b1, b2, f2, col, true)

        if isSelectedLayer then
          dbgDraw.drawSphere(pos, 0.02 * sqrt(pos:distance(camPos)), selLayerRed)
          dbgDraw.drawLineInstance_MinArg(pos + xLeft, pos + xRight, roadThickness, selLayerRed)
          dbgDraw.drawLineInstance_MinArg(pos + yLeft, pos + yRight, roadThickness, selLayerGreen)
          dbgDraw.drawLineInstance_MinArg(pos + zLeft, pos + zRight, roadThickness, selLayerBlue)
        end
      end
      q = q + length + extraSpacing
    end

  elseif layerType == 5 then                                                                        -- TYPE: [SINGLE STATIC MESH PATCH].

    local colLines = meshOutlineColor
    if isSelectedLayer then
      colLines = selectedMeshOutlineColor
    end

    -- Compute the position on the road.
    local rData = road.renderData
    local lIdx = layer.lane[0]
    local lengths = util.computeRoadLength(rData)
    local pEval = layer.pos[0] * lengths[#lengths]                                                  -- The longitudinal evaluation position on the road, in meters.
    local lower, upper = util.findBounds(pEval, lengths)
    local q = (pEval - lengths[lower]) / (lengths[upper] - lengths[lower])                          -- The q in [0, 1] between div points (linear interpolation).
    local pL = nil
    if layer.isLeft[0] then
      local pL1 = rData[lower][lIdx][1]
      local pL2 = rData[upper][lIdx][1]
      pL = pL1 + q * (pL2 - pL1)
    else
      local pL1 = rData[lower][lIdx][2]
      local pL2 = rData[upper][lIdx][2]
      pL = pL1 + q * (pL2 - pL1)
    end
    local n1 = rData[lower][lIdx][5]
    local n2 = rData[upper][lIdx][5]
    local nml = n1 + q * (n2 - n1)
    local l1 = rData[lower][lIdx][6]
    local l2 = rData[upper][lIdx][6]
    local lat = l1 + q * (l2 - l1)
    local pos = pL + nml * (layer.vertOffset[0] + 0.001) + lat * layer.latOffset[0]

    -- Compute the rotation.
    local preRot = rot_q0
    local lRot = layer.rot[0]
    if lRot == 1 then
      preRot = rot_q90
    elseif lRot == 2 then
      preRot = rot_q180
    elseif lRot == 3 then
      preRot = rot_q270
    end
    local rot = quat()
    local tgt = lat:cross(nml)
    rot:setFromDir(tgt, nml)
    rot = preRot * rot_q90 * rot

    local fwd = xAxis:rotated(rot)
    local right = yAxis:rotated(rot)
    local up = zAxis:rotated(rot)
    local xLeft, yLeft, zLeft = -layer.boxXLeft * fwd, -layer.boxYLeft * right, -layer.boxZLeft * up
    local xRight, yRight, zRight = layer.boxXRight * fwd, layer.boxYRight * right, layer.boxZRight * up
    local b1 = pos + xLeft + yLeft + zLeft
    local b2 = pos + xLeft + yRight + zLeft
    local b3 = pos + xLeft + yLeft + zRight
    local b4 = pos + xLeft + yRight + zRight
    local f1 = pos + xRight + yLeft + zLeft
    local f2 = pos + xRight + yRight + zLeft
    local f3 = pos + xRight + yLeft + zRight
    local f4 = pos + xRight + yRight + zRight

    dbgDraw.drawLineInstance_MinArg(b1, b2, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b3, b4, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b1, b3, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b2, b4, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(f1, f2, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(f3, f4, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(f1, f3, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(f2, f4, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b1, f1, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b2, f2, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b3, f3, roadThickness, colLines)
    dbgDraw.drawLineInstance_MinArg(b4, f4, roadThickness, colLines)

    dbgDraw.drawTriSolid(b1, b2, b3, col, true)
    dbgDraw.drawTriSolid(b2, b3, b4, col, true)
    dbgDraw.drawTriSolid(f1, f2, f3, col, true)
    dbgDraw.drawTriSolid(f2, f3, f4, col, true)

    dbgDraw.drawTriSolid(b1, b3, b2, col, true)
    dbgDraw.drawTriSolid(b2, b4, b3, col, true)
    dbgDraw.drawTriSolid(f1, f3, f2, col, true)
    dbgDraw.drawTriSolid(f2, f4, f3, col, true)

    dbgDraw.drawTriSolid(b2, f2, b4, col, true)
    dbgDraw.drawTriSolid(f2, b4, f4, col, true)
    dbgDraw.drawTriSolid(b1, f1, b3, col, true)
    dbgDraw.drawTriSolid(f1, b3, f3, col, true)

    dbgDraw.drawTriSolid(b2, b4, f2, col, true)
    dbgDraw.drawTriSolid(f2, f4, b4, col, true)
    dbgDraw.drawTriSolid(b1, b3, f1, col, true)
    dbgDraw.drawTriSolid(f1, f3, b3, col, true)

    dbgDraw.drawTriSolid(b3, f3, b4, col, true)
    dbgDraw.drawTriSolid(f3, b4, f4, col, true)
    dbgDraw.drawTriSolid(b1, f1, f2, col, true)
    dbgDraw.drawTriSolid(b1, f2, b2, col, true)

    dbgDraw.drawTriSolid(b3, b4, f3, col, true)
    dbgDraw.drawTriSolid(f3, f4, b4, col, true)
    dbgDraw.drawTriSolid(b1, f2, f1, col, true)
    dbgDraw.drawTriSolid(b1, b2, f2, col, true)

    if isSelectedLayer then
      dbgDraw.drawSphere(pos, 0.02 * sqrt(pos:distance(camPos)), selLayerRed)
      dbgDraw.drawLineInstance_MinArg(pos + xLeft, pos + xRight, roadThickness, selLayerRed)
      dbgDraw.drawLineInstance_MinArg(pos + yLeft, pos + yRight, roadThickness, selLayerGreen)
      dbgDraw.drawLineInstance_MinArg(pos + zLeft, pos + zRight, roadThickness, selLayerBlue)
    end
  end
end

-- Renders the proposed join (when linking from start/end of one road, to start/end of another road).
local function renderProposedJoin_End2End(selectedLink, roads, map)
  local r1 = roads[map[selectedLink.r1Name]]
  local r2 = roads[map[selectedLink.r2Name]]
  local p1, p2 = nil, nil
  if selectedLink.r1Lie == 'start' then
    p1 = r1.nodes[2].p
    p2 = r1.nodes[1].p
  else
    p1 = r1.nodes[#r1.nodes - 1].p
    p2 = r1.nodes[#r1.nodes].p
  end
  local p3, p4 = nil, nil
  if selectedLink.r2Lie == 'start' then
    p3 = r2.nodes[1].p
    p4 = r2.nodes[2].p
  else
    p3 = r2.nodes[#r2.nodes].p
    p4 = r2.nodes[#r2.nodes - 1].p
  end
  local pts = {}
  for j = 0, 10 do
    pts[j + 1] = catmullRomChordal(p1, p2, p3, p4, j * 0.1, 0.5)
  end
  for j = 2, #pts do
    dbgDraw.drawLineInstance_MinArg(pts[j - 1], pts[j], linkThickness, linkColor)
  end
  dbgDraw.drawTextAdvanced(p2, 'Join: [' .. ffi.string(r1.displayName) .. ' <-> ' .. ffi.string(r2.displayName) .. ']', textA, true, false, textB)
  dbgDraw.drawTextAdvanced(p2, selectedLink.class, textA, true, false, textB)
end

-- Renders the proposed junction (when linking from start/end of one road, to the middle of another road).
local function renderProposedJoin_End2Mid(selectedJct, roads, map)
  local jRoad = roads[map[selectedJct.jName]]
  local jNodeIdx = selectedJct.jNode
  local r = roads[map[selectedJct.tName]]
  local p = r.nodes[selectedJct.tNode].p
  local xVec, yVec = xAxis * jctVisBoxSize, yAxis * jctVisBoxSize
  local tl = p - xVec - yVec
  local tr = p + xVec - yVec
  local bl = p - xVec + yVec
  local br = p + xVec + yVec
  dbgDraw.drawLineInstance_MinArg(tl, tr, linkThickness, linkColor)
  dbgDraw.drawLineInstance_MinArg(bl, br, linkThickness, linkColor)
  dbgDraw.drawLineInstance_MinArg(tl, bl, linkThickness, linkColor)
  dbgDraw.drawLineInstance_MinArg(br, tr, linkThickness, linkColor)

  dbgDraw.drawLineInstance_MinArg(p, jRoad.nodes[jNodeIdx].p, linkThickness, refLineColor)

  local pos = p + (jRoad.nodes[jNodeIdx].p - p) * 0.5
  dbgDraw.drawTextAdvanced(pos, 'Join: [' .. ffi.string(r.displayName) .. ' <-> ' .. ffi.string(jRoad.displayName) .. ']', textA, true, false, textB)
  dbgDraw.drawTextAdvanced(pos, selectedJct.class, textA, true, false, textB)
end

-- Render overlays.
local function renderOverlay(r, isSelectedRoad, camPos)
  local col = overlayColor
  if isSelectedRoad then
    col = overlayHighlightColor
  end
  local nodes = r.nodes
  local numNodes = #nodes
  local last = nodes[1].p
  for i = 2, numNodes do
    local pos = nodes[i].p
    dbgDraw.drawLineInstance_MinArg(last, pos, overlayThickness, col)
    last = pos
  end
  for i = 1, numNodes do                                                                            -- Draw spheres at each node of this overlay.
    local pos = nodes[i].p
    dbgDraw.drawSphere(pos, 0.05 * sqrt(pos:distance(camPos)), col)
  end
end

-- Renders the terraforming range visualisation for the selected road.
local function renderTerraSingle(roads, rIdx, terraParams)
  local r = roads[rIdx]
  if r and not r.isBridge and not r.isOverlay and #r.nodes > 1 and r.renderData then
    local rData = r.renderData
    local radius = terraParams.domainOfInfluence[0] + terraParams.terraMargin[0]
    local lIdx = -1
    if rData[1][1] then
      lIdx = 1
    end
    for i = 1, #rData do
      local pos = rData[i][lIdx][7]
      debugDrawer:drawCylinder(pos, pos + cylRaised, radius, terraVisColourSingle)
    end
  end
end

-- Renders the terraforming range visualisation for the selected group.
local function renderTerraGroup(roads, map, group, terraParams)
  local radius = terraParams.domainOfInfluence[0] + terraParams.terraMargin[0]
  local box = util.computeAABB2DGroup(group, roads, map)
  local gList = group.list
  for i = 1, #gList do
    local gL = gList[i]
    local r = roads[map[gL.r]]
    if r then
      local rData = r.renderData
      if not r.isOverlay and rData then
        local lIdx = -1
        if rData[1][1] then
          lIdx = 1
        end
        for i = 1, #rData do
          local pos = rData[i][lIdx][7]
          if util.isInBox(pos, box) then
            debugDrawer:drawCylinder(pos, pos + cylRaised, radius, terraVisColour)
          end
        end
      end
    end
  end
end

-- Renders all appropriate markups for roads which request them.
local function drawRoadMarkups(
  roads, map, tree,
  selectedRoadIdx, selectedNodeIdx, selectedLayerIdx,
  isGroupMode, isProfileMode, isCreateGroup, gPolygon, multi, selectedLink, selectedCandidateJct,
  isGuidelines, isShowTerraSingle,
  isShowTerraGroup, selectedPlacedGroup, terraParams)

  if not tree then
    return
  end

  -- Get the camera position and a 2D visibility bounding box around this position.
  local camPos = core_camera.getPosition()
  local camXMin, camYMin, camXMax, camYMax = camPos.x - 350, camPos.y - 350, camPos.x + 350, camPos.y + 350

  -- Prepare the necessary structures for guidelines mode.
  table.clear(guidelines)
  table.clear(selectedGuidelines)
  table.clear(isInMulti)
  if isGuidelines then
    for i = 1, #multi do
      isInMulti[multi[i].r] = true
    end
  end

  -- Render the road-specific details.
  for rName in tree:queryNotNested(camXMin, camYMin, camXMax, camYMax) do
    local rIdx = map[rName]
    if rIdx then
      local isSelectedRoad = rIdx == selectedRoadIdx
      local r = roads[rIdx]
      local laneKeys = r.laneKeys
      local renderData = r.renderData
      local isBridge = r.isBridge
      if not r.treatAsInvisibleInEdit and r.isVis[0] and not r.isOverlay then
        local nodes = r.nodes
        if (not r.isHidden or isGroupMode or isProfileMode) and #nodes > 0 then
          -- Only render the following, if there exists at least two nodes on the road.
          if #nodes > 1 then

            -- Draw the surface mesh, if requested.
            if r.isDisplayRoadSurface[0] and not isBridge then
              renderRoadSurface(renderData, laneKeys, camPos)
            end

            -- Draw the road outline, if requested.
            if r.isDisplayRoadOutline[0] then
              renderRoadOutline(renderData, laneKeys, isSelectedRoad, camPos)
            end

            -- Draw the road reference line (centre line), if requested.
            if r.isDisplayRefLine[0] then
              renderRoadCenterline(renderData, camPos, isGroupMode)
            end

            -- Draw the road guidelines, if requested.
            if isGuidelines then
              addThisRoadsGuidelines(r, renderData, guidelines, isSelectedRoad or isInMulti[rIdx])
            end

            -- Draw the lane markups, if requested.
            if r.isDisplayLaneInfo[0] and renderData and not isBridge then
              renderLaneNumbering(renderData, laneKeys, isGroupMode, camPos)                        -- Display the lane numbering.
              renderLaneDirectionTriangles(renderData, laneKeys, isGroupMode, camPos)               -- Display the lane direction arrows.
            end

            -- Draw the road auto tunnels, if required.
            local tunnels = r.tunnels
            for j = 1, #tunnels do
              renderAutoTunnel(renderData, tunnels[j], camPos)
            end

          end

          if not r.isJctRoad then                                                                   -- Do not show nodes for junction-under-edit roads.
            -- Draw spheres at each node road, if requested to do so.
            if r.isDisplayNodeSpheres[0] then
              renderNodeSpheres(nodes, camPos, isSelectedRoad, selectedNodeIdx)
            end

            -- Draw the node numbering, if requested to do so.
            if r.isDisplayNodeNumbers[0] then
              renderNodeNumbering(nodes, camPos)
            end
          end
        end

        -- Render the layers of the selected road(s).
        if isSelectedRoad and not r.treatAsInvisibleInEdit and r.isVis[0] and not isBridge then
          local nodes = r.nodes
          if (not r.isHidden or isGroupMode or isProfileMode) and not r.isJctRoad and #nodes > 0 then
            if #nodes > 1 then
              local layers = r.profile.layers
              if layers then
                for j = 1, #layers do
                  local lay = layers[j]
                  if lay.isDisplay[0] then
                    renderLayer(r, lay, renderData, camPos, j == selectedLayerIdx)
                  end
                end
              end

            end
          end
        end

      elseif r.isOverlay and r.isVis[0] then
        renderOverlay(r, isSelectedRoad, camPos)

      end
    end
  end

  -- Draw the proposed link, if requested.
  if selectedLink then
    renderProposedJoin_End2End(selectedLink, roads, map)
  end

  -- Draw the proposed junction, if requested.
  if selectedCandidateJct then
    renderProposedJoin_End2Mid(selectedCandidateJct, roads, map)
  end

  -- If the user is creating a group (drawing a perimeter polygon with the mouse).
  if isCreateGroup then
    renderGroupPolygonFence(gPolygon)
  end

  -- If the user has created a multi-selection, highlight all relevant nodes.
  if #multi > 0 then
    renderMultiSelectNodes(roads, multi)
  end

  -- Render the road guidelines.
  if #guidelines > 0 or #selectedGuidelines > 0 then
    renderGuidelines(camPos)
  end

  -- Render the terraforming range edit visualisation.
  if isShowTerraSingle then
    renderTerraSingle(roads, selectedRoadIdx, terraParams)
  elseif isShowTerraGroup and selectedPlacedGroup then
    renderTerraGroup(roads, map, selectedPlacedGroup, terraParams)
  end
end


-- Public interface.
M.drawRoadMarkups =                                       drawRoadMarkups

return M