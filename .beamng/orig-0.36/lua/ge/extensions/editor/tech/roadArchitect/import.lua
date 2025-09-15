-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local verticalNodeOffset = 0.01                                                                     -- A positive vertical offset, added on to every imported road node.
local duplicateNodeTol = 1e-3                                                                       -- A tolerance used when checking if two nodes are sufficiently distant.
local targetLonRes = 5.0                                                                            -- The default target longitudinal resolution for the road.
local targetArcRes = 2.0                                                                            -- The default target circular arc resolution for the road.
local defaultImportWidth = 2.7                                                                      -- The default lane width to use, when width is not specified in the file.
local defaultImportHeight = 0.1                                                                     -- The default lane rel. height to use, when height is not specified in the file.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}


-- External modules used.
local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing the road structure/handling road calculations.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- A module for managing the profiles structure/handling profile calculations.
local geom = require('editor/tech/roadArchitect/geometry')                                          -- A module for performing geometric calculations.
local terra = require('editor/tech/roadArchitect/terraform')                                        -- A module for handling terraforming operations.
local clothoid = require('editor/tech/roadArchitect/clothoid')                                      -- A module for evaluating Clothoid (Euler) spirals.

-- Module constants.
local im = ui_imgui
local min, max, floor, ceil = math.min, math.max, math.floor, math.ceil
local abs = math.abs
local sin, cos = math.sin, math.cos
local targetLonResInv, targetArcResInv = 1.0 / targetLonRes, 1.0 / targetArcRes
local tmp0, tmp1 = vec3(0, 0), vec3(0, 0)


-- Evaluates an explicit cubic polynomial at the given p in [0, segment length].
local function evalExpCubic(a, b, c, d, s)
  if s then
    local s2 = s * s
    return a + (s * b) + (s2 * c) + (s2 * s * d)
  end
  return 0.0
end

-- Converts an OpenDRIVE 'line' primitive, to a 2D polyline.
local function lineTo2DPoly(start, length, hdg)
  tmp0:set(cos(hdg), sin(hdg), 0.0)
  tmp0:normalize()
  local splineGran = ceil(length * targetLonResInv)
  local poly, f = {}, length * tmp0 / splineGran
  for i = 0, splineGran do
    poly[i + 1] = start + i * f                                                                         -- Parameter q (implicit) in [0, geodesic length].
  end
  return poly
end

-- Converts an OpenDRIVE 'arc' primitive, to 2D polyline.
local function arcTo2DPoly(start, length, k, hdg)
  local xStart, yStart, hCos, hSin, r = start.x, start.y, cos(hdg), sin(hdg), abs(1.0 / k)
  local theta = length / r
  local kSign = sign2(k)
  local kSignR = kSign * r
  local kSROpp = -kSignR
  local arcGran = max(3, ceil(length * 0.1 * targetArcResInv))
  local poly, f = {}, theta / arcGran
  for i = 0, arcGran do
    local q = i * f
    local pX, pY = r * sin(q), kSROpp * cos(q) + kSignR
    poly[i + 1] = vec3(hCos * pX - hSin * pY + xStart, hSin * pX + hCos * pY + yStart)
  end
  return poly
end

-- Converts an OpenDRIVE 'spiral' primitive, to a 2D polyline.
local function spiralTo2DPoly(start, length, k1, k2, hdg)
  local arcGran = max(3, ceil(length * 0.1 * targetArcResInv))
  local poly, f, dK = {}, length / arcGran, (k2 - k1) / length
  for i = 0, arcGran do
    poly[i + 1] = clothoid.evaluate(start, hdg, k1, dK, i * f)                                          -- Parameter q (implicit) in [0, geodesic length].
  end
  return poly
end

-- Converts an OpenDRIVE 'poly3' primitive (explicit cubic), to a 2D polyline.
local function poly3To2DPoly(start, length, hdg, A, B, C, D)

  -- Compute the (u, v) orthonormal frame.
  local hCos, hSin = cos(hdg), sin(hdg)
  tmp0:set(hCos, hSin, 0.0)
  tmp0:normalize()
  tmp1:set(-hSin, hCos, 0.0)
  tmp1:normalize()

  -- Fit the polyline to the primitive.
  local splineGran = ceil(length * targetLonResInv)
  local poly, f, q, q2, v = {}, length / splineGran, nil, nil, nil
  for i = 1, splineGran do
    q = i * f                                                                                       -- Parameter q in [0, geodesic length].
    q2 = q * q
    v = A + (q * B) + (q2 * C) + (q2 * q * D)                                                       -- v(q): (the lateral deviation from the reference line).
    poly[i] = start + (tmp0 * q) + (tmp1 * v)                                                       -- Project from (s, t) space, to world space (x, y).
  end
  return poly
end

-- Converts an OpenDRIVE 'paramPoly3' primitive (parametric cubic), to a 2D polyline.
local function paramPoly3To2DPoly(start, length, hdg, isArcLength, uA, uB, uC, uD, vA, vB, vC, vD)

  -- Compute the (u, v) orthonormal frame.
  local hCos, hSin = cos(hdg), sin(hdg)
  tmp0:set(hCos, hSin, 0.0)
  tmp0:normalize()
  tmp1:set(-hSin, hCos, 0.0)
  tmp1:normalize()

  -- If the primitive is using 'arc length' parametrisation, handle with q in [0, geodesic length].
  local splineGran = ceil(length * targetLonResInv)
  local splineGranInv = 1.0 / splineGran
  local poly, q, q2, q3, u, v = {}, nil, nil, nil, nil, nil
  if isArcLength then
    local f = length * splineGranInv
    for i = 1, splineGran do
      q = i * f                                                                                     -- Parameter q in [0, geodesic length].
      q2 = q * q
      q3 = q2 * q
      u = uA + (q * uB) + (q2 * uC) + (q3 * uD)
      v = vA + (q * vB) + (q2 * vC) + (q3 * vD)
      poly[i] = start + (tmp0 * u) + (tmp1 * v)                                                     -- Project from (s, t) space, to world space (x, y).
    end
    return poly
  end

  -- The primitive is not using 'arc length' parametrisation, so handle with q in [0, 1].
  for i = 1, splineGran do
    q = i * splineGranInv                                                                           -- Parameter q in [0, 1].
    q2 = q * q
    q3 = q2 * q
    u = uA + (q * uB) + (q2 * uC) + (q3 * uD)
    v = vA + (q * vB) + (q2 * vC) + (q3 * vD)
    poly[i] = start + (tmp0 * u) + (tmp1 * v)                                                       -- Project from (s, t) space, to world space (x, y).
  end

  return poly
end

-- Removes duplicate points from a given polyline.
local function removeDuplicates(poly)
  local polyPost, ctr, numPoints = { poly[1] }, 2, #poly
  for i = 2, numPoints do
    if poly[i]:squaredDistance(poly[i - 1]) > duplicateNodeTol then
      polyPost[ctr] = poly[i]
      ctr = ctr + 1
    end
  end
  return polyPost
end

-- Appends an array of points to another array of points.
local function appendPointsToPoly(pts, poly)
  local outPoly, polyLen = {}, #poly
  for i = 1, polyLen do
    outPoly[i] = poly[i]
  end
  local ctr, numPts = polyLen + 1, #pts
  for i = 1, numPts do
    outPoly[ctr] = pts[i]
    ctr = ctr + 1
  end
  return outPoly
end

-- Computes a 2D road reference polyline from given OpenDRIVE data.
local function compute2DRefPolyLine(refLineData)
  local poly, numRef = {}, #refLineData
  for i = 1, numRef do
    local ref = refLineData[i]
    local geom = ref.geom
    local type, start, length, hdg, pts = geom.type, ref.start, ref.length, ref.hdg, nil
    if type == 'line' then
      pts = lineTo2DPoly(start, length, hdg)
    elseif type == 'arc' then
      pts = arcTo2DPoly(start, length, geom.k, hdg)
    elseif type == 'spiral' then
      pts = spiralTo2DPoly(start, length, geom.k1, geom.k2, hdg)
    elseif type == 'poly3' then
      pts = poly3To2DPoly(start, length, hdg, geom.a, geom.b, geom.c, geom.d)
    elseif type == 'paramPoly3' then
      pts = paramPoly3To2DPoly(start, length, hdg, geom.isArcLength, geom.aU, geom.bU, geom.cU, geom.dU, geom.aV, geom.bV, geom.cV, geom.dV)
    end
    poly = appendPointsToPoly(pts, poly)
  end
  return removeDuplicates(poly)
end

-- Used in the parsing of .xml files.
local function parseargs(s)
  local arg = {}
  string.gsub(s, "([%-%w]+)=([\"'])(.-)%2", function (w, _, a) arg[w] = a end)
  return arg
end

-- Used in the parsing of .xml files.
local function collect(s)
  local stack, top = {}, {}
  table.insert(stack, top)
  local ni, c, label, xarg, empty
  local i, j = 1, 1
  while true do
    ni, j, c, label, xarg, empty = string.find(s, "<(%/?)([%w:]+)(.-)(%/?)>", i)
    if not ni then break end
    local text = string.sub(s, i, ni - 1)
    if not string.find(text, "^%s*$") then
      table.insert(top, text)
    end
    if empty == "/" then
      table.insert(top, { label = label, xarg = parseargs(xarg), empty = 1 })
    elseif c == "" then
      top = { label = label, xarg=parseargs(xarg) }
      table.insert(stack, top)
    else
      local toclose = table.remove(stack)
      top = stack[#stack]
      if #stack < 1 then
        error("nothing to close with "..label)
      end
      if toclose.label ~= label then
        error("trying to close " .. toclose.label .. " with " .. label)
      end
      table.insert(top, toclose)
    end
    i = j + 1
  end
  local text = string.sub(s, i)
  if not string.find(text, "^%s*$") then
    table.insert(stack[#stack], text)
  end
  if #stack > 1 then
    error("unclosed " .. stack[#stack].label)
  end
  return stack[1]
end

-- Converts the 3D reference polyline to the editor node format polyline.
local function format3DPoly(raw)
  local poly, numNodes = {}, #raw
  for i = 1, numNodes do
    local p = raw[i]
    poly[i] = {
      p = p,
      isLocked = false,
      rot = im.FloatPtr(0.0),
      height = im.FloatPtr(p.z),
      widths = {}, heightsL = {}, heightsR = {},
      incircleRad = im.FloatPtr(1.0),
      isAutoBanked = false,
      offset = 0.0 }
  end
  return poly
end

-- Approximates the arc length (from start) at every node in the reference polyline.
local function computeArcLengthsNodes(poly)
  local lengths, sum, numNodes = { 0.0 }, 0.0, #poly
  for i = 2, numNodes do
    sum = sum + poly[i]:distance(poly[i - 1])
    lengths[i] = sum
  end
  return lengths
end

-- Finds the section in a profile which is appropriate for the given s-value, and returns the cubic.
local function getRelevantCubic(sNode, sections)

  -- Attempt to find the s-value from the section which covers the given node region.
  local numSections = #sections
  for i = 2, numSections do
    local s = sections[i].s
    if s > sNode then
      local iMinus1 = i - 1
      return iMinus1, sNode - sections[iMinus1].s
    end
  end

  -- There is no section s-value which is above the given node s-value, so use the last available section.
  return numSections, sNode - sections[numSections].s
end

-- Applies the lane offset sections to the 3D reference polyline.
local function applyLaneOffsets(poly, laneOffsets, lengths)
  local numNodes = #poly
  if laneOffsets and #laneOffsets > 0 then
    for i = 1, numNodes do
      local lOIdx, sLocal = getRelevantCubic(lengths[i], laneOffsets)                               -- Find the appropriate cubic which matches the s-value at this node.
      local lOP = laneOffsets[lOIdx]
      poly[i].offset = evalExpCubic(lOP.a, lOP.b, lOP.c, lOP.d, sLocal)
    end
  end
end

-- Applies the elevation sections to the 2D reference polyline, to get the 3D reference polyline.
local function applyElevation(poly2D, elevs, lengths)
  local poly3D, numNodes = {}, #poly2D
  if elevs and #elevs > 0 then
    for i = 1, numNodes do
      local p2D = poly2D[i]
      local eIdx, sLocal = getRelevantCubic(lengths[i], elevs)
      local eP = elevs[eIdx]
      poly3D[i] = vec3(p2D.x, p2D.y, evalExpCubic(eP.a, eP.b, eP.c, eP.d, sLocal) + verticalNodeOffset)
    end
  else
    for i = 1, numNodes do
      local p2D = poly2D[i]
      poly3D[i] = vec3(p2D.x, p2D.y, 0.0)
    end
  end
  return poly3D
end

-- Applies the super-elevation sections to the native 3D polyline.
local function applySuperElevation(poly, superElevs, lengths)
  local numNodes = #poly
  for i = 1, numNodes do
    local eIdx, sLocal = getRelevantCubic(lengths[i], superElevs)
    local eP = superElevs[eIdx]
    poly[i].rot = im.FloatPtr(evalExpCubic(eP.a, eP.b, eP.c, eP.d, sLocal))
  end
end

-- Get the appropriate cubic container from a collection of candidates, based on s-value.
-- [This is used for lane widths and heights, where an sOffset can be provided].
local function getApprCubic(cands, sNode, sLaneSec)

  -- If there are multiple candidates, choose the one with the correct s-value.
  local numCands = #cands
  for i = 2, numCands do
    if sLaneSec + cands[i].sOffset > sNode then
      return cands[i - 1]
    end
  end

  -- If we have not found a candidate, return the last candidate by default.
  return cands[numCands]
end

-- Adds lane width and height offset data to the road.
local function addLaneWAndH(poly, sec, laneKeys, lengths)
  local numNodes = #poly
  for i = 1, numNodes do
    local sNode = lengths[i]
    local sLocal = sNode - lengths[1]
    local sLaneSec = sec.s
    for j = -20, 20 do
      local lane = sec[j]
      if lane and laneKeys[j] then                                                                  -- If this lane does exist in this section, set the data.

        -- Add the lane width offsets, if they are provided
        local lWidth = getApprCubic(lane.widths, sNode, sLaneSec)
        if lWidth then
          poly[i].widths[j] = im.FloatPtr(evalExpCubic(lWidth.a, lWidth.b, lWidth.c, lWidth.d, sLocal - lWidth.sOffset))
        else
          poly[i].widths[j] = im.FloatPtr(defaultImportWidth)                                       -- Width is not present, so use defaults.
        end
        if poly[i].widths[j][0] > 10.0 then                                                         -- If width has blown up, collapse the lane here.  This is a bit hacky.
          poly[i].widths[j] = im.FloatPtr(0.0)
        end

        -- Add the lane height offsets, if they are provided.
        local lHeight = getApprCubic(lane.heights, sNode, sLaneSec)
        if lHeight and lHeight.inner and lHeight.outer then
          if j < 0 then
            poly[i].heightsL[j] = im.FloatPtr(lHeight.inner)
            poly[i].heightsR[j] = im.FloatPtr(lHeight.outer)
          else
            poly[i].heightsL[j] = im.FloatPtr(lHeight.outer)
            poly[i].heightsR[j] = im.FloatPtr(lHeight.inner)
          end
        else
          poly[i].heightsL[j] = im.FloatPtr(defaultImportHeight)                                    -- Width is present, but not heights, so use defaults.
          poly[i].heightsR[j] = im.FloatPtr(defaultImportHeight)
        end
      elseif laneKeys[j] then
        poly[i].widths[j] = im.FloatPtr(0.0)                                                        -- If this lane does not exist in this section, use zero width and default height.
        poly[i].heightsL[j] = im.FloatPtr(defaultImportHeight)
        poly[i].heightsR[j] = im.FloatPtr(defaultImportHeight)
      end
    end
  end
end

-- Computes the split location on a road, given an s-value.
-- [Returns the lower and upper node indices and interpolation parameter in [0, 1], at which to split the road].
local function getSplitLocation(poly2D, arcLengths, sSec)
  local numNodes = #poly2D
  for i = 2, numNodes do
    local iM = i - 1
    local sL, sU = arcLengths[iM], arcLengths[i]
    if sL <= sSec and sSec <= sU then
      return true, iM, i, (sSec - sL) / (sU - sL)
    end
  end
  return false, nil, nil, nil
end

-- Inserts nodes at the points where each laneSection transitions into the next.
local function insertNodesAtSectionTransitions(refPoly2D, arcLengths, lanes)

  -- If there is only one laneSection, do nothing.
  local numSections = #lanes
  if numSections < 2 then
    return {}, arcLengths
  end

  -- Insert the relevant nodes.
  local splitPoints, sCtr = {}, 1
  for i = 2, numSections do
    local sec = lanes[i]
    local isFound, iL, iU, q = getSplitLocation(refPoly2D, arcLengths, sec.s)
    if isFound then
      local lNode = refPoly2D[iL]
      local p = lNode + q * (refPoly2D[iU] - lNode)                                                 -- The point to be added.
      table.insert(refPoly2D, iU, p)
      arcLengths = computeArcLengthsNodes(refPoly2D)                                                -- Re-compute the arc lengths, now that there is an inserted node in the polyline.
      splitPoints[sCtr] = p
      sCtr = sCtr + 1
    end
  end

  -- Get the indices of each split point, in the final poly line.
  local splitIndices = {}
  local numSplitPoints, numNodes = #splitPoints, #refPoly2D
  for i = 1, numSplitPoints do
    local sp = splitPoints[i]
    for j = 1, numNodes do
      if sp:squaredDistance(refPoly2D[j]) < 1e-5 then
        splitIndices[j] = true
        break
      end
    end
  end

  return splitIndices, arcLengths
end

-- Splits the given poly line at the given mark indices.
local function splitPolyAtJoins(poly2D, splitIndices, arcLengths)
  if #splitIndices == 1 then
    return { poly2D }, { arcLengths }
  end

  local splits, LS, sCtr, numNodes = {}, {}, 1, #poly2D
  local build, bLS, bCtr = {}, {}, 1
  for i = 1, numNodes do
    local p = poly2D[i]
    if splitIndices[i] then
      build[bCtr] = vec3(p.x, p.y)
      bLS[bCtr] = arcLengths[i]
      splits[sCtr] = build
      LS[sCtr] = bLS
      sCtr = sCtr + 1
      build, bCtr = { vec3(p.x, p.y) }, 2
      bLS = { arcLengths[i] }
    else
      build[bCtr] = vec3(p.x, p.y)
      bLS[bCtr] = arcLengths[i]
      bCtr = bCtr + 1
    end
  end
  if #build > 1 then
    splits[sCtr] = build
    LS[sCtr] = bLS
  end

  return splits, LS
end

-- Removes any nodes which are within some tolerance to other nodes of the same polyline.
local function removeCloseNeighbours(splits, lengths)
  local numSplits = #splits
  for i = 1, numSplits do
    local split, length = splits[i], lengths[i]
    local splitsPP, lengthsPP, iCtr = { split[1] }, { length[1] }, 2
    local numNodes = #split
    for j = 2, numNodes do
      if split[j - 1]:squaredDistance(split[j]) > 0.1 then
        splitsPP[iCtr] = split[j]
        lengthsPP[iCtr] = length[j]
        iCtr = iCtr + 1
      end
    end
    splits[i], lengths[i] = splitsPP, lengthsPP
  end
  return splits, lengths
end

-- Convert an OpenDRIVE road representation to the editor format.
-- [Each input data type is an ordered array, by OpenDRIVE s-value].
local function convertRoad(data)

  -- Convert the core geometric data to the native road structure.
  local refPoly2D = compute2DRefPolyLine(data.geom)                                                   -- Fit a 2D polyline through the given road reference line data.
  local arcLengths = computeArcLengthsNodes(refPoly2D)                                                -- Approximate the arc length (from start) at every point in the ref polyline.

  local splitIndices, arcLengths = insertNodesAtSectionTransitions(refPoly2D, arcLengths, data.lanes) -- Insert nodes at the places where one laneSection transitions into the next.
  local splits, arcLengthsLS = splitPolyAtJoins(refPoly2D, splitIndices, arcLengths)                  -- Now split the road at the marked positions (where laneSections transition).
  splits, arcLengthsLS = removeCloseNeighbours(splits, arcLengthsLS)                                  -- Remove any nodes which are too close to other nodes.

  -- Create the new roads and add to the roads container.
  local numSplits = #splits
  for i = 1, numSplits do
    if #splits[i] > 1 then

      -- Apply the elevation sections to the 2D ref polyline, to get a 3D ref polyline.
      local LSAL = arcLengthsLS[i]
      local refPoly3D = applyElevation(splits[i], data.elev, LSAL)

      -- Convert the 3D reference polyline to the editor node format.
      local rEPoly = format3DPoly(refPoly3D)

      -- Create a new, custom road profile for the imported road.
      local newProfile, laneKeys = profileMgr.createCustomImportProfile(data.lanes[i])

      -- Apply the super-elevation sections to the native 3D polyline.
      local sElev = data.sElev
      if sElev and #sElev > 0 then
        applySuperElevation(rEPoly, data.sElev, LSAL)
      end

      -- Convert the lane width and height data, and add to the new road.
      addLaneWAndH(rEPoly, data.lanes[i], laneKeys, LSAL)

      -- Apply lane offsets to the road, in order to ensure road centerline is in the correct position.
      applyLaneOffsets(rEPoly, data.laneOffsets, LSAL)

      local road = roadMgr.createRoadFromProfile(newProfile)
      road.nodes = rEPoly

      geom.computeRoadRenderData(road, roadMgr.roads, roadMgr.map)

      local rIdx = #roadMgr.roads + 1
      roadMgr.roads[rIdx] = road
      roadMgr.map[road.name] = rIdx
      roadMgr.setDirty(road)
    end
  end
end

-- Imports an .xodr file from disk, and creates a Road Architect network from the data.
local function import(importO2T, importCO, importTT2I, importCustomOffset, domainOfInfluence, margin)

  extensions.editor_fileDialog.openFile(
    function(data)

      local rPrims = {}                                                                             -- The imported roads data structure.
      local jPrims = {}                                                                             -- The imported junctions data structure.

      -- Collect all the .xml data.
      local d = collect(readFile(data.filepath))

      -- Iterate over the first children.
      for _, v1 in pairs(d[2]) do

        -- Collect all road data.
        if v1.label == 'road' then

          -- Iterate over the second children.
          local pred, succ, geom, elev, sElev, lanes, laneOffsets = nil, nil, {}, {}, {}, {}, {}
          for _, v2 in pairs(v1) do

            -- Collect the road connectivity data, if any exists.
            if v2.label == 'link' then
              for _, v3 in pairs(v2) do
                if v3.label == 'predecessor' then
                  local dP = v3.xarg
                  pred = { type = dP.elementType, id = tonumber(dP.elementId), contactPoint = dP.contactPoint }
                elseif v3.label == 'successor' then
                  local dS = v3.xarg
                  succ = { type = dS.elementType, id = tonumber(dS.elementId), contactPoint = dS.contactPoint }
                end
              end

            -- Collect the road geometry data.
            elseif v2.label == 'planView' then
              for _, v3 in pairs(v2) do
                local gInner = {}
                if v3.label == 'geometry' then
                  for _, v4 in pairs(v3) do
                    if v4.label == 'line' then
                      gInner.type = 'line'
                    elseif v4.label == 'arc' then
                      gInner.type, gInner.k = 'arc', tonumber(v4.xarg.curvature)
                    elseif v4.label == 'spiral' then
                      local dS = v4.xarg
                      gInner.type, gInner.k1, gInner.k2 = 'spiral', tonumber(dS.curvStart), tonumber(dS.curvEnd)
                    elseif v4.label == 'poly3' then
                      local dP = v4.xarg
                      gInner.type, gInner.a, gInner.b, gInner.c, gInner.d = 'poly3', tonumber(dP.a), tonumber(dP.b), tonumber(dP.c), tonumber(dP.d)
                    elseif v4.label == 'paramPoly3' then
                      local dP = v4.xarg
                      gInner.type = 'paramPoly3'
                      gInner.aU, gInner.bU, gInner.cU, gInner.dU = tonumber(dP.aU), tonumber(dP.bU), tonumber(dP.cU), tonumber(dP.dU)
                      gInner.aV, gInner.bV, gInner.cV, gInner.dV = tonumber(dP.aV), tonumber(dP.bV), tonumber(dP.cV), tonumber(dP.dV)
                    end
                    break                                                                           -- We assume only one primitive per geometry tag.
                  end
                  local dG = v3.xarg
                  geom[#geom + 1] = { s = tonumber(dG.s), start = vec3(tonumber(dG.x), tonumber(dG.y)), hdg = tonumber(dG.hdg), length = tonumber(dG.length), geom = gInner }
                end
              end

            -- Collect the road elevation data, if it exists.
            elseif v2.label == 'elevationProfile' then
              for _, v3 in pairs(v2) do
                if v3.label == 'elevation' then
                  local dE = v3.xarg
                  elev[#elev + 1] = { s = tonumber(dE.s), a = tonumber(dE.a), b = tonumber(dE.b), c = tonumber(dE.c), d = tonumber(dE.d) }
                end
              end

            -- Collect the road lateral profile data, if it exists.
            elseif v2.label == 'lateralProfile' then
              for _, v3 in pairs(v2) do
                if v3.label == 'superelevation' then
                  local dSE = v3.xarg
                  sElev[#sElev + 1] = { s = tonumber(dSE.s), a = tonumber(dSE.a), b = tonumber(dSE.b), c = tonumber(dSE.c), d = tonumber(dSE.d) }
                end
              end

            -- Collect the road lanes data.
            elseif v2.label == 'lanes' then
              for _, v3 in pairs(v2) do
                if v3.label == 'laneSection' then
                  local lSecData = { s = tonumber(v3.xarg.s) }
                  for _, v4 in pairs(v3) do
                    if v4.label == 'left' or v4.label == 'right' then
                      for _, v5 in pairs(v4) do
                        if v5.label == 'lane' then
                          local lD = v5.xarg
                          local laneId = tonumber(lD.id)
                          lSecData[laneId] = { type = tonumber(lD.type), dir = lD.direction, widths = {}, heights = {} }
                          for _, v6 in pairs(v5) do
                            local lA = v6.xarg
                            if v6.label == 'width' then
                              local numWidths = #lSecData[laneId].widths
                              lSecData[laneId].widths[numWidths + 1] = { sOffset = tonumber(lA.sOffset), a = tonumber(lA.a), b = tonumber(lA.b), c = tonumber(lA.c), d = tonumber(lA.d) }
                            elseif v6.label == 'height' then
                              local numHeights = #lSecData[laneId].heights
                              lSecData[laneId].heights[numHeights + 1] = { sOffset = tonumber(lA.sOffset), inner = tonumber(lA.inner), outer = tonumber(lA.outer) }
                            elseif v6.label == 'link' then
                              for _, v7 in pairs(v6) do
                                if v7.label == 'predecessor' then
                                  lSecData[laneId].pred = tonumber(v7.xarg.id)
                                elseif v7.label == 'successor' then
                                  lSecData[laneId].succ = tonumber(v7.xarg.id)
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                  lanes[#lanes + 1] = lSecData
                elseif v3.label == 'laneOffset' then
                  local lOD = v3.xarg
                  laneOffsets[#laneOffsets + 1] = { s = tonumber(lOD.s), a = tonumber(lOD.a), b = tonumber(lOD.b), c = tonumber(lOD.c), d = tonumber(lOD.d) }
                end
              end
            end
          end

          -- Store the collected data for this single road.
          local xArg = v1.xarg
          rPrims[#rPrims + 1] = {
            id = tonumber(xArg.id), name = tonumber(xArg.name),
            rule = tonumber(xArg.rule), length = tonumber(xArg.length), junction = tonumber(xArg.junction),
            pred = pred, succ = succ,
            geom = geom, elev = elev, sElev = sElev, lanes = lanes, laneOffsets = laneOffsets }

        -- Collect all junction data.
        elseif v1.label == 'junction' then
          local jId, jType, conn = tonumber(v1.xarg.id), v1.xarg.type, {}
          for _, v2 in pairs(v1) do
            if v2.label == 'connection' then
              local rIn, rOut, cp = tonumber(v2.xarg.incomingRoad), tonumber(v2.xarg.linkedRoad), v2.xarg.contactPoint
              local links = {}
              for _, v3 in pairs(v2) do
                if v3.label == 'laneLink' then
                  links[#links + 1] = { from = tonumber(v3.xarg.from), to = tonumber(v3.xarg.to) }
                end
              end
              conn[#conn + 1] = { rIn = rIn, rOut = rOut, contactPoint = cp, links = links }
            end
          end
          jPrims[#jPrims + 1] = { id = jId, type = jType, connections = conn }
        end
      end

      -- Convert the collected OpenDRIVE primitive-based roads into independent, native roads.
      -- [Also creates a map from OpenDRIVE road ids to native road id in roads container].
      local numRoadPrims = #rPrims
      for i = 1, numRoadPrims do
        convertRoad(rPrims[i])
      end

      -- Perform any request post-processing on the import.
      if importO2T then
        roadMgr.offsetRoads2Terrain()
      elseif importCO then
        roadMgr.offsetByValue(importCustomOffset)
      end
      if importTT2I then
        roadMgr.computeAllRoadRenderData()
        terra.terraformMultiRoads(domainOfInfluence, margin, nil)
      end

    end,
    {{"xodr",".xodr"}},
    false,
    "/")
end


-- Public interface.
M.import =                                                import

return M