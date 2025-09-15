-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local intersectionTol = 1e-3                                                                        -- A tolerance used when determining if line segments intersect circles.
local splineSmoothingVal = 0.5                                                                      -- The smoothing value used when fitting splines to polylines, in [0, 1].

local extraHairpinWidth = 3.0                                                                       -- The extra width added to hairpin corners.
local lonDistMin, lonDistMax = 1, 20

local tolUG, tolOG = 1.0, 0.5                                                                       -- The tolerances used for determining if a road is underground or overground.
local minTunnelLen = 3                                                                              -- The minimum number of div points which can comprise a tunnel length.
local minBridgeLen = 3                                                                              -- The minimum number of div points which can comprise a bridge.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local im = ui_imgui
local min, max, abs, floor, ceil = math.min, math.max, math.abs, math.floor, math.ceil
local atan2, pi, sqrt, rad = math.atan2, math.pi, math.sqrt, math.rad
local twoPi, halfPi, deg2Rad = pi * 2.0, pi * 0.5, pi / 180.0
local tmp1, tmp2, tmp3 = vec3(0, 0), vec3(0, 0), vec3(0, 0)
local tgt_2D, tmpLat = vec3(0, 0), vec3(0, 0)
local pStart_2D, pMid_2D, pEnd_2D = vec3(0, 0), vec3(0, 0), vec3(0, 0)
local vertical = vec3(0, 0, 1)
local oneThird, twoThirds = 0.3333333333333333333333333333, 0.666666666666666666666666666667
local splSm = splineSmoothingVal * 2


-- Linearly interpolates between two sets of lane width values and (left and right) relative height offset values.
local function lerpWAndH(w1, w2, hL1, hL2, hR1, hR2, q)
  local w_q, hL_q, hR_q = {}, {}, {}
  for k, _ in pairs(w1) do
    local w1Key, hL1Key, hR1Key = w1[k][0], hL1[k][0], hR1[k][0]
    w_q[k], hL_q[k], hR_q[k] = im.FloatPtr(w1Key + q * (w2[k][0] - w1Key)), im.FloatPtr(hL1Key + q * (hL2[k][0] - hL1Key)), im.FloatPtr(hR1Key + q * (hR2[k][0] - hR1Key))
  end
  return w_q, hL_q, hR_q
end

-- Computes the 2D incircle of a triangle (from the three given triangle vertices v0-v1-v2).
-- Returns the center and radius of the incircle.
local function computeIncircle2D(v0, v1, v2)
  local a, b, c = v2:distance(v1), v2:distance(v0), v1:distance(v0)
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

-- Identifies all the tunnel sections within the given road.
local function identifyTunnelSections(rData, extraS, extraE)

  -- Determine the left and right lateral edge indices.
  local rD1, lIdx, rIdx = rData[1], nil, nil
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

  -- Iterate over the road longitudinally, and identify all the individual tunnelled sections.
  local tSections, sCtr, rDataSize, isInside, sTun = {}, 1, #rData, false, nil
  for i = 1, rDataSize do
    local rD = rData[i]
    local rDLeft = rD[lIdx]
    local rLeft, rRight = rDLeft[4], rD[rIdx][3]                                                    -- Test both the left-most and right-most lateral road points.
    local zLeft = core_terrain.getTerrainHeight(rLeft)
    local zRight = core_terrain.getTerrainHeight(rRight)

    if zLeft > rLeft.z + tolUG and zRight > rRight.z + tolUG then
      if not isInside then
        sTun, isInside = i, true
      end
    else
      if isInside then
        if i - 1 > sTun + minTunnelLen then                                                         -- Only include sections which span a sufficient number of div points.
          tSections[sCtr] = { s = max(1, sTun - extraS), e = min(rDataSize, i + extraE) }
          sCtr = sCtr + 1
        end
      end
      isInside = false
    end
  end

  if isInside then
    if rDataSize > sTun + minTunnelLen then                                                         -- Only include sections which span a sufficient number of div points.
      tSections[sCtr] = { s = max(1, sTun - extraS), e = rDataSize }
      sCtr = sCtr + 1
    end
  end

  return tSections
end

-- Computes the normal vectors from the discretised points and rotation angles.
local function normalsFromPAndRot(pDisc, rDisc)
  local nDisc = {}
  for i = 1, #pDisc do
    local tgt = pDisc[min(#pDisc, i + 1)] - pDisc[max(1, i - 1)]                                    -- Compute the sparse tangent vector (the axis of rotation for this node).
    tgt:normalize()
    nDisc[i] = util.rotateVecAroundAxis(vertical, tgt, rDisc[i] * deg2Rad)                          -- Rotate the vertical around the axis of rotation by the selected angle.
  end
  return nDisc
end

-- Adaptively computes the best granularity for the given spline section, using curvature and length.
local function computeAdaptiveGran(road, i1, i2, i3, i4, numNodes, length, isFirstEval)
  local radius, cen = 1e5, nil
  if numNodes > 2 then
    if isFirstEval then
      tmp1:set(i2.x, i2.y, 0)
      tmp2:set(i3.x, i3.y, 0)
      tmp3:set(i4.x, i4.y, 0)
      cen = util.circle2DFrom3Points(tmp1, tmp2, tmp3)
    else
      local radius1, radius2 = nil, nil
      tmp1:set(i1.x, i1.y, 0)
      tmp2:set(i2.x, i2.y, 0)
      tmp3:set(i3.x, i3.y, 0)
      local cen1 = util.circle2DFrom3Points(tmp1, tmp2, tmp3)
      if cen1 then radius1 = cen1:distance(tmp1) end
      tmp1:set(i2.x, i2.y, 0)
      tmp2:set(i3.x, i3.y, 0)
      tmp3:set(i4.x, i4.y, 0)
      local cen2 = util.circle2DFrom3Points(tmp1, tmp2, tmp3)
      if cen2 then radius2 = cen2:distance(tmp1) end
      radius = min(radius1 or 1e5, radius2 or 1e5)
    end
  end
  if cen then radius = cen:distance(tmp1) end
  local lonDistMin, lonDistMax = 1, 10
  local dx = min(lonDistMax, max(lonDistMin, radius * (lonDistMax - lonDistMin) / 190.0 + (20.0 * lonDistMin - lonDistMax) / 19.0))
  return ceil(length / dx) * road.granFactor[0]
end

-- Deep copies a node.
local function copyNode(n)
  local wC, hLC, hRC, w, hL, hR = {}, {}, {}, n.widths, n.heightsL, n.heightsR
  for i = -20, 20 do
    if w[i] then
      wC[i], hLC[i], hRC[i] = im.FloatPtr(w[i][0]), im.FloatPtr(hL[i][0]), im.FloatPtr(hR[i][0])
    end
  end
  local pos = n.p
  return {
    p = vec3(pos.x, pos.y, pos.z),
    isLocked = n.isLocked,
    rot = im.FloatPtr(n.rot[0]),
    widths = wC, heightsL = hLC, heightsR = hRC,
    incircleRad = im.FloatPtr(n.incircleRad[0]),
    isAutoBanked = n.isAutoBanked,
    offset = n.offset }
end

-- Fits a spline through a standard (user) road.
-- [Catmull-Rom is fitted through X and Y, monotonic Steffen preconditioning is applied for Z].
local function fitSplineStandardRoad(road)

  local pDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc, ctr = {}, {}, {}, {}, {}, {}, 1
  local poly = road.nodes
  local numNodes = #poly
  for i = 2, numNodes do
    local iMinus1 = i - 1
    local p1, p2, p3, p4 = poly[max(1, i - 2)], poly[iMinus1], poly[i], poly[min(numNodes, i + 1)]
    local i1, i2, i3, i4 = p1.p, p2.p, p3.p, p4.p

    local i1z, i2z, i3z, i4z, i2i3 = i1.z, i2.z, i3.z, i4.z, i3 - i2
    local delta0, delta1, delta2 = i2z - i1z, i3z - i2z, i4z - i3z
    local signDelta1, absDelta1 = sign2(delta1), abs(delta1)
    local n1 = (sign2(delta0) + signDelta1) * min(abs(delta0), absDelta1, 0.5 * abs((delta0 + delta1) * 0.5))
    local n2 = (signDelta1 + sign2(delta2)) * min(absDelta1, abs(delta2), 0.5 * abs((delta1 + delta2) * 0.5))
    local n1PlusN2 = n1 + n2
    local length = i2:distance(i3)
    local d1, d2, d3 = max(1e-30, sqrt(i1:distance(i2))), sqrt(length), max(1e-30, sqrt(i3:distance(i4)))
    local m1, m2, sd2 = (i2 - i1) / d1 + (i1 - i3) / (d1 + d2), (i2 - i4) / (d2 + d3) + (i4 - i3) / d3, splSm * d2

    -- Compute the granularity to be used with this spline section.
    local splineGran = computeAdaptiveGran(road, i1, i2, i3, i4, numNodes, length, i == 2)
    local splineGranInv = 1.0 / splineGran

    -- Fit the spline.
    local startIdx = 1
    if i == 2 then startIdx = 0 end
    for j = startIdx, splineGran do
      local q = j * splineGranInv
      local tt, t_1 = q * q, q - 1
      local t_1sq= t_1 * t_1
      local p = q * t_1sq * sd2 * m1 + tt * t_1 * sd2 * m2 + t_1sq * (2.0 * q + 1) * i2 - tt * (2.0 * q - 3.0) * i3 + splSm * t_1 * (q * t_1 + tt) * i2i3
      p.z = i2z + q * (n1 + q * (delta1 - n1 + (q - 1.0) * (n1PlusN2 - 2.0 * delta1)))
      pDisc[ctr] = p
      local qPlus1 = q + 1
      rDisc[ctr] = monotonicSteffen(p1.rot[0], p2.rot[0], p3.rot[0], p4.rot[0], 0, 1, 2, 3, qPlus1)
      oDisc[ctr] = monotonicSteffen(p1.offset, p2.offset, p3.offset, p4.offset, 0, 1, 2, 3, qPlus1)
      wDisc[ctr], hLDisc[ctr], hRDisc[ctr] = {}, {}, {}
      for k, _ in pairs(p1.widths) do
        wDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.widths[k][0], p2.widths[k][0], p3.widths[k][0], p4.widths[k][0], 0, 1, 2, 3, qPlus1))
        hLDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.heightsL[k][0], p2.heightsL[k][0], p3.heightsL[k][0], p4.heightsL[k][0], 0, 1, 2, 3, qPlus1))
        hRDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.heightsR[k][0], p2.heightsR[k][0], p3.heightsR[k][0], p4.heightsR[k][0], 0, 1, 2, 3, qPlus1))
      end
      ctr = ctr + 1
    end
  end

  -- Compute the normal vectors.
  local nDisc = normalsFromPAndRot(pDisc, rDisc)

  return pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc
end

-- Computes the full angle between two vectors, using a direction.
local function calcSpanAngle(center, rotDir, p1, p2)
  local n2, n3 = p1 - center, p2 - center
  return (twoPi + atan2(n2:dot(n3:cross(rotDir)), n2:dot(n3))) % twoPi
end

-- Fits a circular arc segment through a road.
local function fitArc(road)

  -- Determine if an arc can be fitted through the given data.
  local nodes = road.nodes
  local numNodes, cen, p1, p2, p3 = #nodes, nil, nil, nil, nil
  if numNodes > 2 then
    p1, p2, p3 = nodes[1].p, nodes[2].p, nodes[3].p
    cen = util.circle2DFrom3Points(p1, p2, p3)                                                      -- Find the (2D) circle which fits through the (three) road nodes.
  end
  if not cen then
    return fitSplineStandardRoad(road)                                                              -- If an arc cannot be fitted through the given nodes, fit a spline instead.
  end

  -- Discretise the arc to produce a fitted polyline.
  local pDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc = {}, {}, {}, {}, {}, {}
  pStart_2D:set(p1.x, p1.y, 0.0)
  pMid_2D:set(p2.x, p2.y, 0.0)
  pEnd_2D:set(p3.x, p3.y, 0.0)
  local v1 = pStart_2D - cen
  local theta, signFac = 0.0, -sign2((p2 - p1):cross(vertical):dot(p3 - p1))
  local rotDir = (pStart_2D - pEnd_2D):cross(pMid_2D - pEnd_2D):normalized()
  theta = signFac * calcSpanAngle(cen, rotDir, pStart_2D, pEnd_2D)
  local z1, z2 = p1.z, p3.z
  local nd1, nd3 = nodes[1], nodes[3]
  local rot1, rot2, off1, off2 = nd1.rot[0], nd3.rot[0], nd1.offset, nd3.offset
  local w1, w2 = nd1.widths, nd3.widths
  local hL1, hL2, hR1, hR2 = nd1.heightsL, nd3.heightsL, nd1.heightsR, nd3.heightsR

  local radius = cen:distance(pStart_2D)
  local dx = min(lonDistMax, max(lonDistMin, radius * (lonDistMax - lonDistMin) / 190.0 + (20.0 * lonDistMin - lonDistMax) / 19.0))
  local length = pStart_2D:distance(pMid_2D) + pMid_2D:distance(pEnd_2D)
  local splineGran = (ceil(length / dx) + 1) * road.granFactor[0]
  local splineGranInv = 1.0 / splineGran

  for i = 0, splineGran do
    local q, idx = i * splineGranInv, i + 1
    local p = cen + util.rotateVecAroundAxis(v1, vertical, q * theta)
    p.z = lerp(z1, z2, q)
    pDisc[idx] = p
    rDisc[idx], oDisc[idx] = lerp(rot1, rot2, q), lerp(off1, off2, q)
    wDisc[idx], hLDisc[idx], hRDisc[idx] = lerpWAndH(w1, w2, hL1, hL2, hR1, hR2, q)
  end

  -- Compute the normal vectors.
  local nDisc = normalsFromPAndRot(pDisc, rDisc)

  return pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc
end

-- Fits civil engineering style splines through a given road (line-spline-arc-spline-line sequences),
-- and interpolates a local orthonormal frame across this discretisation.
local function fitCivEngAndFrame(road)

  -- Compute linking points and tangents, if applicable.
  local poly = road.nodes

  -- If there are only two nodes, fit a spline instead.
  if #poly < 3 then
    return fitSplineStandardRoad(road)
  end

  -- Attempt to fit civil-engineering style splines at each corner.
  local map = { 1 }
  local poly1 = poly[1]
  local pDisc = { vec3(poly1.p.x, poly1.p.y, 0) }
  local ctr = 2
  for i = 2, #poly - 1 do

    -- Compute the three triangle points which will define the circular arc.
    -- [This is the node position and a point on each connecting line segment, which depend on the selected incircle radius].
    local iMinus1, iPlus1 = i - 1, i + 1
    local node, nM, nP = poly[i], poly[iMinus1], poly[iPlus1]
    local pI, pLast, pNext, icRad = node.p, nM.p, nP.p, node.incircleRad[0]
    local vLast, vNext = pLast - pI, pNext - pI
    local pIm, pIp = pI + icRad * vLast, pI + icRad * vNext

    -- Compute the incircle.
    tmp1:set(pI.x, pI.y, 0.0)
    tmp2:set(pIm.x, pIm.y, 0.0)
    tmp3:set(pIp.x, pIp.y, 0.0)
    local iCen_2D, iRad = computeIncircle2D(tmp1, tmp2, tmp3)
    iRad = iRad + 1e-5

    -- Compute the two intersection points on each of the triangle edges which touch the node.
    -- [At these intersections, the linear segment tangents match the arc tangent, so continuity exists].
    local p1_2D, p3_2D = intLineSegAndCircle(tmp1, tmp2, iCen_2D, iRad), intLineSegAndCircle(tmp1, tmp3, iCen_2D, iRad)

    -- Determine the angular domain for the arc.
    -- [The sign of theta depends on the sign of the distance to the lateral plane of the current point.]
    local v1_2D = p1_2D - iCen_2D
    local tgt_3D = pIp - pIm
    tgt_2D:set(tgt_3D.x, tgt_3D.y, 0.0)
    tgt_2D:normalize()
    local signFac = -sign2(tgt_2D:cross(vertical):dot(pIp - pI))
    local theta = util.angleBetweenVecs(v1_2D, p3_2D - iCen_2D) * signFac
    local vStart_2D, vEnd_2D = util.rotateVecAroundAxis(v1_2D, vertical, oneThird * theta), util.rotateVecAroundAxis(v1_2D, vertical, twoThirds * theta)
    local u1_2D, u2_2D, u3_2D = p1_2D, iCen_2D + vStart_2D, iCen_2D + vEnd_2D
    local arcAngle = util.angleBetweenVecs(u2_2D - iCen_2D, u3_2D - iCen_2D) * signFac

    -- Fit a discretised line.
    -- [The points and width values are linearly-interpolated, and the normals are spherically-interpolated].
    local lastIdx = ctr - 1
    local pLastIdx = max(1, lastIdx - 1)
    local pPrev = vec3(pDisc[pLastIdx].x, pDisc[pLastIdx].y, 0)
    local pStart_3D = vec3(pDisc[lastIdx].x, pDisc[lastIdx].y, 0)
    pStart_2D:set(pStart_3D.x, pStart_3D.y, 0.0)
    local i1, i2, i3, i4 = pPrev, pStart_2D, vec3(u1_2D.x, u1_2D.y, 0), vec3(u2_2D.x, u2_2D.y, 0)

    local i2i3 = i3 - i2
    local d23 = i2:distance(i3)
    local d1, d2, d3 = max(1e-30, sqrt(i1:distance(i2))), sqrt(d23), max(1e-30, sqrt(i3:distance(i4)))
    local m1, m2, sd2 = (i2 - i1) / d1 + (i1 - i3) / (d1 + d2), (i2 - i4) / (d2 + d3) + (i4 - i3) / d3, splSm * d2
    local splineGran = computeAdaptiveGran(road, i1, i2, i3, i4, #poly, d23, i == 2)
    local splineGranInv = 1.0 / splineGran
    local startIdx = 1
    if i == 2 then
      startIdx = 0
    end
    for j = startIdx, splineGran do
      local q = j * splineGranInv
      local tt, t_1 = q * q, q - 1
      local t_1sq = t_1 * t_1
      pDisc[ctr] = q * t_1sq * sd2 * m1 + tt * t_1 * sd2 * m2 + t_1sq * (2.0 * q + 1) * i2 - tt * (2.0 * q - 3.0) * i3 + splSm * t_1 * (q * t_1 + tt) * i2i3
      pDisc[ctr].z = 0.0
      ctr = ctr + 1
    end

    -- Compute the arc section.
    -- [This is done before computing the Clothoid sections].
    local dx = 5.0--min(lonDistMax, max(lonDistMin, iRad * (lonDistMax - lonDistMin) / 190.0 + (20.0 * lonDistMin - lonDistMax) / 19.0))
    local length = 10--u1_2D:distance(u2_2D) + u2_2D:distance(u3_2D)
    local splineGran = ceil(length / dx) * road.granFactor[0]
    local splineGranInv = 1.0 / splineGran
    local pArc = {}
    for j = 0, splineGran do
      pArc[j + 1] = iCen_2D + util.rotateVecAroundAxis(vStart_2D, vertical, j * splineGranInv * arcAngle)
      pArc[j + 1].z = 0.0
    end

    -- Fit a spline between the first line and the arc.
    local pClo1 = {}
    i1 = vec3(pIm.x, pIm.y, 0)
    i2 = vec3(pDisc[ctr - 1].x, pDisc[ctr - 1].y, 0)
    i3 = vec3(pArc[1].x, pArc[1].y, 0)
    i4 = vec3(pArc[2].x, pArc[2].y, 0)
    i2i3 = i3 - i2
    d23 = i2:distance(i3)
    d1, d2, d3 = max(1e-30, sqrt(i1:distance(i2))), sqrt(d23), max(1e-30, sqrt(i3:distance(i4)))
    m1, m2, sd2 = (i2 - i1) / d1 + (i1 - i3) / (d1 + d2), (i2 - i4) / (d2 + d3) + (i4 - i3) / d3, splSm * d2
    for j = 1, splineGran - 1 do
      local q = j * splineGranInv
      local tt, t_1 = q * q, q - 1
      local t_1sq = t_1 * t_1
      pClo1[j] = q * t_1sq * sd2 * m1 + tt * t_1 * sd2 * m2 + t_1sq * (2.0 * q + 1) * i2 - tt * (2.0 * q - 3.0) * i3 + splSm * t_1 * (q * t_1 + tt) * i2i3
      pClo1[j].z = 0.0
    end

    -- Fit a spline between the arc and the second line.
    local pClo2 = {}
    local pArcLen = #pArc
    i1 = vec3(pArc[pArcLen - 1].x, pArc[pArcLen - 1].y, 0)
    i2 = vec3(pArc[pArcLen].x, pArc[pArcLen].y, 0)
    i3 = vec3(p3_2D.x, p3_2D.y, 0)
    i4 = vec3(pIp.x, pIp.y, 0)
    i2i3 = i3 - i2
    d23 = i2:distance(i3)
    d1, d2, d3 = max(1e-30, sqrt(i1:distance(i2))), sqrt(d23), max(1e-30, sqrt(i3:distance(i4)))
    m1, m2, sd2 = (i2 - i1) / d1 + (i1 - i3) / (d1 + d2), (i2 - i4) / (d2 + d3) + (i4 - i3) / d3, splSm * d2
    for j = 1, splineGran - 1 do
      local q = j * splineGranInv
      local tt, t_1 = q * q, q - 1
      local t_1sq = t_1 * t_1
      pClo2[j] = q * t_1sq * sd2 * m1 + tt * t_1 * sd2 * m2 + t_1sq * (2.0 * q + 1) * i2 - tt * (2.0 * q - 3.0) * i3 + splSm * t_1 * (q * t_1 + tt) * i2i3
      pClo2[j].z = 0.0
    end

    -- Append [Clothoid 1 - Arc - Clothoid 2] multi-section to the discretised points array.
    if abs(iRad) > 1.7 then
      for j = 1, #pClo1 do
        pDisc[ctr] = pClo1[j]
        ctr = ctr + 1
      end
      map[#map + 1] = ctr + floor(#pArc * 0.5)                                                      -- For the general case, add the arc midpoint to the map.
      for j = 1, #pArc do
        pDisc[ctr] = pArc[j]
        ctr = ctr + 1
      end
      for j = 1, #pClo2 do
        pDisc[ctr] = pClo2[j]
        ctr = ctr + 1
      end
    else
      map[#map + 1] = ctr + 1                                                                       -- Use only a single point in the near-linear case (avoids rendering issues/folding).
      pDisc[ctr] = pArc[floor(#pArc * 0.5)]
      ctr = ctr + 1
    end

  end

  -- Fit a final discretised line between the last multi-section and the very last point.
  -- [The points and lane widths are linearly-interpolated, and the normals are spherically-interpolated].
  local ctrLast = ctr - 1
  local nLast = poly[#poly]
  local i1 = vec3(pDisc[ctr - 2].x, pDisc[ctr - 2].y, 0)
  local i2 = vec3(pDisc[ctrLast].x, pDisc[ctrLast].y, 0)
  local i3 = vec3(nLast.p.x, nLast.p.y, 0)
  local i4 = i3
  local i2i3 = i3 - i2
  local d23 = i2:distance(i3)
  local d1, d2, d3 = max(1e-30, sqrt(i1:distance(i2))), sqrt(d23), max(1e-30, sqrt(i3:distance(i4)))
  local m1, m2, sd2 = (i2 - i1) / d1 + (i1 - i3) / (d1 + d2), (i2 - i4) / (d2 + d3) + (i4 - i3) / d3, splSm * d2
  local splineGran = computeAdaptiveGran(road, i1, i2, i3, i3, #poly, d23, false)
  local splineGranInv = 1.0 / splineGran
  for j = 1, splineGran do
    local q = j * splineGranInv
    local tt, t_1 = q * q, q - 1
    local t_1sq = t_1 * t_1
    pDisc[ctr] = q * t_1sq * sd2 * m1 + tt * t_1 * sd2 * m2 + t_1sq * (2.0 * q + 1) * i2 - tt * (2.0 * q - 3.0) * i3 + splSm * t_1 * (q * t_1 + tt) * i2i3
    pDisc[ctr].z = 0.0
    ctr = ctr + 1
  end

  map[#map + 1] = #pDisc

  -- Interpolate the other quantities using Steffen.
  local rDisc, oDisc, wDisc, hLDisc, hRDisc = { poly[1].rot[0] }, { poly[1].offset }, { poly[1].widths }, { poly[1].heightsL }, { poly[1].heightsR }
  local ctr = 2
  for i = 2, #poly do
    local i1, i2, i3, i4 = max(1, i - 2), i - 1, i, min(#poly, i + 1)
    local p1, p2, p3, p4 = poly[i1], poly[i2], poly[i3], poly[i4]
    local div1, div2 = map[i2], map[i3]
    tmp1:set(pDisc[div1].x, pDisc[div1].y, 0)
    tmp2:set(pDisc[div2].x, pDisc[div2].y, 0)
    local dAll = tmp1:distance(tmp2)
    local size = map[i3] - map[i2]
    for _ = 1, size do
      tmp3:set(pDisc[ctr].x, pDisc[ctr].y, 0)
      local d1 = tmp3:distance(tmp1)
      local q = d1 / dAll
      local qPlus1 = q + 1
      pDisc[ctr].z = monotonicSteffen(p1.p.z, p2.p.z, p3.p.z, p4.p.z, 0, 1, 2, 3, qPlus1)
      rDisc[ctr] = monotonicSteffen(p1.rot[0], p2.rot[0], p3.rot[0], p4.rot[0], 0, 1, 2, 3, qPlus1)
      oDisc[ctr] = monotonicSteffen(p1.offset, p2.offset, p3.offset, p4.offset, 0, 1, 2, 3, qPlus1)
      wDisc[ctr], hLDisc[ctr], hRDisc[ctr] = {}, {}, {}
      for k, _ in pairs(p1.widths) do
        wDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.widths[k][0], p2.widths[k][0], p3.widths[k][0], p4.widths[k][0], 0, 1, 2, 3, qPlus1))
        hLDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.heightsL[k][0], p2.heightsL[k][0], p3.heightsL[k][0], p4.heightsL[k][0], 0, 1, 2, 3, qPlus1))
        hRDisc[ctr][k] = im.FloatPtr(monotonicSteffen(p1.heightsR[k][0], p2.heightsR[k][0], p3.heightsR[k][0], p4.heightsR[k][0], 0, 1, 2, 3, qPlus1))
      end
      ctr = ctr + 1
    end
  end
  pDisc[1].z = poly[1].p.z
  table.remove(pDisc, 1)
  table.remove(rDisc, 1)
  table.remove(oDisc, 1)
  table.remove(wDisc, 1)
  table.remove(hLDisc, 1)
  table.remove(hRDisc, 1)

  -- Remove any duplicates.
  local pDiscA, rDiscA, oDiscA, wDiscA, hLDiscA, hRDiscA = { pDisc[1] }, { rDisc[1] }, { oDisc[1] }, { wDisc[1] }, { hLDisc[1] }, { hRDisc[1] }
  local ctr = 2
  for i = 2, #pDisc do
    local p1, p2 = pDisc[i - 1], pDisc[i]
    local dSq = p1:squaredDistance(p2)
    if dSq > 0.01 then
      pDiscA[ctr], rDiscA[ctr], oDiscA[ctr], wDiscA[ctr], hLDiscA[ctr], hRDiscA[ctr] = pDisc[i], rDisc[i], oDisc[i], wDisc[i], hLDisc[i], hRDisc[i]
      ctr = ctr + 1
    end
  end

  -- Compute the normal vectors.
  local nDiscA = normalsFromPAndRot(pDiscA, rDiscA)

  return pDiscA, nDiscA, rDiscA, oDiscA, wDiscA, hLDiscA, hRDiscA
end

-- Computes the geometric data for the given road, ready for rendering.
-- This function performs either CR-spline-fitting or line-spline-arc-spline-line (civil engineering style) fitting to the reference nodes.
-- The structure is indexed as [point][lane id][1:12] where:
--  [1:4] are the cross sectional points, clockwise from top-left,
--  [5:6] are the unit normal/lateral vectors (same for all lanes),
--  [7] is the lane midpoint position at the point,
--  [8] is the lane type (string).
--  [9:11] is the lane width and left/right relative heights.
--  [12] is the signed lateral rotation angle (super-elevation), same for all lanes.
--  [13] The lateral lane offset value (same for every lane, at this disc. pt).
local function computeRoadRenderData(road)

  -- If the road is being conformed to the terrain, project the nodes to the terrain.
  local isConformToTerrain = road.isConformRoadToTerrain[0]
  if isConformToTerrain then
    for i = 1, #road.nodes do
      local p = road.nodes[i].p
      tmp1:set(p.x, p.y, 0)
      p.z = core_terrain.getTerrainHeight(tmp1)
    end
  end

  -- Apply auto banking, if selected.
  local profile = road.profile
  if profile.isAutoBanking[0] then
    local autoBankingFactor = profile.autoBankingFactor[0]
    local nodes = road.nodes
    nodes[1].rot = im.FloatPtr(0.0)
    nodes[#nodes].rot = im.FloatPtr(0.0)
    for i = 2, #nodes - 1 do
      if nodes[i].isAutoBanked then
        local p1, p2, p3 = nodes[i - 1].p, nodes[i].p, nodes[i + 1].p
        tmp1:set(p1.x, p1.y, 0)
        tmp2:set(p2.x, p2.y, 0)
        tmp3:set(p3.x, p3.y, 0)
        local cen = util.circle2DFrom3Points(tmp1, tmp2, tmp3)
        if cen then
          local radius = cen:distance(tmp1)
          local angDeg = max(0.0, min(8.0, -0.015 * radius + 8.5))
          local bankingSign = sign2((p2 - p1):cross(vertical):dot(p3 - p1))
          nodes[i].rot = im.FloatPtr(bankingSign * autoBankingFactor * angDeg)
        end
      end
    end
  end

  -- Apply extra width at hairpins, if required.
  local nodes = road.nodes
  local origWidths = {}
  if profile.isExtraWidth[0] then
    for i = 2, #nodes - 1 do
      origWidths[i] = {}
      local v1, v2 = nodes[i].p - nodes[i - 1].p, nodes[i + 1].p - nodes[i].p
      if abs(util.angleBetweenVecs(v1, v2)) > halfPi then
        local numLanes = 0
        for k, _ in pairs(nodes[i].widths) do
          if nodes[i].widths[k][0] > 2.0 then
            numLanes = numLanes + 1
          end
        end
        local extraHairpinLaneWidth = extraHairpinWidth / numLanes
        for k, _ in pairs(nodes[i].widths) do
          if nodes[i].widths[k][0] > 2.0 then
            origWidths[i][k] = nodes[i].widths[k][0]
            nodes[i].widths[k] = im.FloatPtr(nodes[i].widths[k][0] + extraHairpinLaneWidth)
          end
        end
      end
    end
  end

  -- Filter by case:
  -- i) Standard roads: if the user has requested a civil engineering style spline, this is handled separately.
  -- ii) Arc roads: circular-arc fitted splines comprising of exactly three nodes.
  -- iii) Standard roads: the default is to use centripetal CR spline fitting through the nodes.
  local pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc = nil, nil, nil, nil, nil, nil, nil
  if road.isCivilEngRoads[0] then
    pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc = fitCivEngAndFrame(road)
  elseif road.isArc then
    pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc = fitArc(road)
  else
    pDisc, nDisc, rDisc, oDisc, wDisc, hLDisc, hRDisc = fitSplineStandardRoad(road)
  end

  -- Recover the old widths (after hairpin computations above).
  if profile.isExtraWidth[0] then
    for i = 1, #origWidths do
      if origWidths[i] then
        for k, _ in pairs(origWidths[i]) do
          nodes[i].widths[k] = im.FloatPtr(origWidths[i][k])
        end
      end
    end
  end

  -- Compute the render data for this road (sub node data at each discretisation point on the fitted polyline).
  local laneKeys, leftKeys, rightKeys = road.laneKeys, road.leftKeys, road.rightKeys
  local numLaneKeys, numLeftKeys, numRightKeys = #laneKeys, #leftKeys, #rightKeys
  local numDiscPoints = #pDisc

  -- Fetch the existing render data table, if it exists, or allocate a new table (new roads will require this).
  local renderData = road.renderData or {}
  if #renderData > numDiscPoints then
    table.clear(renderData)                                                                         -- If there are less points than in last frame, clear all existing render data.
  end

  for i = 1, numDiscPoints do

    -- Compute the unit tangent and lateral vectors, at this disc. point.
    local tgt = pDisc[min(numDiscPoints, i + 1)] - pDisc[max(1, i - 1)]
    tgt:normalize()
    local nml = nDisc[i]
    nml:normalize()
    tmpLat:set(nml.y * tgt.z - nml.z * tgt.y, nml.z * tgt.x - nml.x * tgt.z, nml.x * tgt.y - nml.y * tgt.x)
    tmpLat:normalize()
    nml = tgt:cross(tmpLat)

    -- Fetch the render data for this discretisation point, if it exists, or allocate a new table (new roads will require this).
    local rData = renderData[i] or {}
    if #rData > numLaneKeys then
      table.clear(rData)                                                                            -- If the number of lanes have changed, clear the table for this disc. point.
    end

    -- First, do the left lanes first, from inner to outer.
    -- [Splitting road into two sides ensures the reference line is rendered at the center, without needing an offset].
    local p, w, hL, hR = pDisc[i], wDisc[i], hLDisc[i], hRDisc[i]
    local rotRad, off, pWork = rad(rDisc[i]), oDisc[i], p
    for j = numLeftKeys, 1, -1 do
      local k = leftKeys[j]
      local laneWidth, heightL, heightR = w[k][0], hL[k][0], hR[k][0]
      local laneWidthVec = laneWidth * tmpLat
      local nHL, nHR = nml * heightL, nml * heightR
      local lu, ru = pWork + nHL, pWork - laneWidthVec + nHR                                        -- The lane inner-most/outer-most points.
      pWork = pWork - laneWidthVec
      local rdk = rData[k]
      if not rdk then
        rdk = {}
        rdk[6] = vec3(0, 0)
      end
      local oVec = tmpLat * off                                                                     -- The lateral offset vector.
      local p1, p2, p3, p4 = ru + oVec, lu + oVec, lu - nHL + oVec, ru - nHR + oVec                 -- The four cross-sectional points from top-left, clockwise, to bottom-right.
      rdk[1], rdk[2], rdk[3], rdk[4] = p1, p2, p3, p4                                               -- [1:4] The four cross-sectional points, clockwise from top left.
      rdk[5] = nml                                                                                  -- [5] The road section unit normal vector (not quad local).
      rdk[6]:set(tmpLat.x, tmpLat.y, tmpLat.z)                                                      -- [6] The road section unit lateral vectors (not quad local).
      rdk[7] = (p1 + p2) * 0.5                                                                      -- [7] The lane midpoint (on top face).
      rdk[8] = profile[k].type                                                                      -- [8] The lane type ('road_lane', 'sidewalk', 'island', etc).
      rdk[9], rdk[10], rdk[11] = laneWidth, heightL, heightR                                        -- [9:11] The lane width, left/right relative heights.
      rdk[12] = rotRad                                                                              -- [12] The signed lateral rotation angle (super-elevation).
      rdk[13] = off                                                                                 -- [13] The lateral lane offset value (same for every lane, at this disc. pt).
      rData[k] = rdk
    end

    -- Next, do the right lanes, from inner to outer.
    -- [Note: points [1:4] are flipped here, to ensure they follow clockwise from left-top on both sides].
    pWork = p
    for j = 1, numRightKeys do
      local k = rightKeys[j]
      local laneWidth, heightL, heightR = w[k][0], hL[k][0], hR[k][0]
      local laneWidthVec = laneWidth * tmpLat
      local nHL, nHR = nml * heightL, nml * heightR
      local lu, ru = pWork + nHL, pWork + laneWidthVec + nHR                                        -- The lane inner-most/outer-most points.
      pWork = pWork + laneWidthVec
      local rdk = rData[k]
      if not rdk then
        rdk = {}
        rdk[6] = vec3(0, 0)
      end
      local oVec = tmpLat * off                                                                     -- The lateral offset vector.
      local p1, p2, p3, p4 = lu + oVec, ru + oVec, ru - nHR + oVec, lu - nHL + oVec                 -- The four cross-sectional points from top-left, clockwise, to bottom-right.
      rdk[1], rdk[2], rdk[3], rdk[4] = p1, p2, p3, p4                                               -- [1:4] The four cross-sectional points, clockwise from top left.
      rdk[5] = nml                                                                                  -- [5] The road section unit normal vector (not quad local).
      rdk[6]:set(tmpLat.x, tmpLat.y, tmpLat.z)                                                      -- [6] The road section unit lateral vectors (not quad local).
      rdk[7] = (p1 + p2) * 0.5                                                                      -- [7] The lane midpoint (on top face).
      rdk[8] = profile[k].type                                                                      -- [8] The lane type ('road_lane', 'sidewalk', 'island', etc).
      rdk[9], rdk[10], rdk[11] = laneWidth, heightL, heightR                                        -- [9:11] The lane width, left/right relative heights.
      rdk[12] = rotRad                                                                              -- [12] The signed lateral rotation angle (super-elevation).
      rdk[13] = off                                                                                 -- [13] The lateral lane offset value (same for every lane, at this disc. pt).
      rData[k] = rdk
    end
    renderData[i] = rData
  end

  -- If the road is set to conform to the terrain, offset the profile vertically, as required.
  -- [The terrain height under the left and right points are sampled, and the four quad points are offset].
  if isConformToTerrain then
    for i = 1, numDiscPoints do
      local rD = renderData[i]
      for j = 1, numLaneKeys do
        local rDL = rD[laneKeys[j]]
        local l1, l2, l3, l4 = rDL[1], rDL[2], rDL[3], rDL[4]
        tmp1:set(l1.x, l1.y, 0)
        tmp2:set(l2.x, l2.y, 0)
        local gL = core_terrain.getTerrainHeight(tmp1)                                              -- Get the terrain height value under the left and right points.
        local gR = core_terrain.getTerrainHeight(tmp2)
        tmp1:set(0, 0, l4.z - gL)                                                                   -- The left and right vertical offset vectors, required to conform the road.
        tmp2:set(0, 0, l3.z - gR)
        rDL[1], rDL[2], rDL[3], rDL[4] = l1 - tmp1, l2 - tmp2, l3 - tmp2, l4 - tmp1
        rDL[7] = rDL[7] - (tmp1 + tmp2) * 0.5                                                       -- The lane midpoint.
      end
    end
  end

  -- Set the road render data on the road.
  road.renderData = renderData

  -- Handle any tunneled/bridged sections for this road.
  if extensions.editor_terrainEditor.getTerrainBlock() then
    if road.isAllowTunnels[0] then
      local tSections = identifyTunnelSections(renderData, road.extraS[0], road.extraE[0])
      table.clear(road.tunnels)
      for i = 1, #tSections do
        local sec = tSections[i]
        road.tunnels[i] = {
          name = 'tunnel_' .. tostring(i),
          s = sec.s, e = sec.e,
          radGran = road.radGran[0], radOffset = road.radOffset[0],
          thickness = road.thickness[0], zOffsetFromRoad = road.zOffsetFromRoad[0],
          protrudeS = road.protrudeS[0], protrudeE = road.protrudeE[0] }
      end
    end
  end
end


-- Public interface.
M.computeRoadRenderData =                                 computeRoadRenderData

return M