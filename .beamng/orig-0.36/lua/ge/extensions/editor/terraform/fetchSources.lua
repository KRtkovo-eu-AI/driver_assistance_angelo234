-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local defaultWidthForMeshSplines = 3.0

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules.
--local meshSplines = require("editor/meshSpline/splineMgr")
--local stitchSplines = require("editor/stitchSpline/splineMgr")
--local railSplines = require("editor/railSpline/splineMgr")
local ratRoads = require("editor/tech/roadArchitect/roads")
local geom = require("editor/toolUtilities/geom")

-- Module constants.
local min, max = math.min, math.max
local up = vec3(0, 0, 1)

-- Module state.
local tmp1, tmp2, tmp3, tmp4 = vec3(), vec3(), vec3(), vec3()


-- Fits a frenet spline to the given mesh road nodes.
local function fitFrenetSplineToMeshRoadNodes(rawNodes)
  local nodes, widths, normals = {}, {}, {}
  local numRawNodes = #rawNodes
  local startIdx = 0.0
  for i = 1, numRawNodes - 1 do
    local i1, i2, i3, i4 = rawNodes[max(1, i - 1)], rawNodes[i], rawNodes[min(numRawNodes, i + 1)], rawNodes[min(numRawNodes, i + 2)]
    local p1, p2, p3, p4 = i1.pos, i2.pos, i3.pos, i4.pos
    local n1, n2, n3, n4 = i1.normal, i2.normal, i3.normal, i4.normal
    tmp1:set(p1.x, p1.y, i1.width)
    tmp2:set(p2.x, p2.y, i2.width)
    tmp3:set(p3.x, p3.y, i3.width)
    tmp4:set(p4.x, p4.y, i4.width)
    for q = startIdx, 1, 0.1 do
      table.insert(nodes, catmullRom(p1, p2, p3, p4, q, 0.5))
      table.insert(normals, catmullRom(n1, n2, n3, n4, q, 0.5))
      table.insert(widths, catmullRom(tmp1, tmp2, tmp3, tmp4, q, 0.5).z)
    end
    startIdx = 0.1 -- In any interation other than the first, we start at 0.1 to avoid duplicating the first node.
  end
  local binormals = {}
  for i = 1, #nodes do
    local tgt = nodes[min(#nodes, i + 1)] - nodes[max(1, i - 1)]
    tgt:normalize()
    table.insert(binormals, -tgt:cross(normals[i]))
  end
  return nodes, widths, binormals
end

-- Fetches from the scene tree, all the mesh roads (and part-roads) in the given polygon.
local function getAllMeshRoadSourcesInPolygon(polygon, polygonAABB)
  local sources = {}
  for _, meshRoadName in ipairs(scenetree.findClassObjects("MeshRoad")) do
    local road = scenetree.findObject(meshRoadName)
    local center = road:getWorldBox():getCenter()
    local centerX, centerY = center.x, center.y
    local extents = road:getWorldBox():getExtents()
    local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
    local rXMin, rXMax, rYMin, rYMax = centerX - xHalf, centerX + xHalf, centerY - yHalf, centerY + yHalf
    if not (rXMax < polygonAABB.xMin or rXMin > polygonAABB.xMax or rYMax < polygonAABB.yMin or rYMin > polygonAABB.yMax) then
      local nodes, widths, binormals = fitFrenetSplineToMeshRoadNodes(editor.getNodes(road))
      local spans = geom.getNodeSpansInsidePolygon(nodes, polygon)
      for j = 1, #spans do
      local span = spans[j]
      local sIdx, eIdx = span[1], span[2]
      if eIdx > sIdx then
        local inner = {}
        for k = sIdx, eIdx do
          table.insert(inner, { pos = nodes[k], width = widths[k], binormal = binormals[k] })
        end
          table.insert(sources, inner)
        end
      end
    end
  end
  return sources
end

-- Fits a frenet spline to the given nodes.
local function fitFrenetSplineToSplineNodes(rawNodes)
  local nodes, widths = {}, {}
  local numRawNodes = #rawNodes
  local startIdx = 0.0
  for i = 1, numRawNodes - 1 do
    local p1, p2, p3, p4 = rawNodes[max(1, i - 1)], rawNodes[i], rawNodes[min(numRawNodes, i + 1)], rawNodes[min(numRawNodes, i + 2)]
    for q = startIdx, 1, 0.1 do
      table.insert(nodes, catmullRomCentripetal(p1, p2, p3, p4, q, 0.5))
      table.insert(widths, defaultWidthForMeshSplines)
    end
    startIdx = 0.1 -- In any interation other than the first, we start at 0.1 to avoid duplicating the first node.
  end
  local binormals = {}
  for i = 1, #nodes do
    local tgt = nodes[min(#nodes, i + 1)] - nodes[max(1, i - 1)]
    tgt:normalize()
    table.insert(binormals, -tgt:cross(up))
  end
  return nodes, widths, binormals
end

-- Fetches from the scene tree, all the splines which are in the given polygon.
local function getAllSplinesInPolygon(polygon, polygonAABB, splines)
  local numSplines = #splines
  local sources = table.new(numSplines, 0)
  local sCtr = 1
  for i = 1, numSplines do
    local spline = splines[i]
    local aabb = geom.getAABB(spline.nodes)
    local rXMin, rXMax, rYMin, rYMax = aabb.xMin, aabb.xMax, aabb.yMin, aabb.yMax
    if not (rXMax < polygonAABB.xMin or rXMin > polygonAABB.xMax or rYMax < polygonAABB.yMin or rYMin > polygonAABB.yMax) then
      local nodes, widths, binormals = fitFrenetSplineToSplineNodes(spline.nodes)
      local spans = geom.getNodeSpansInsidePolygon(nodes, polygon)
      for j = 1, #spans do
        local span = spans[j]
        local sIdx, eIdx = span[1], span[2]
        if eIdx > sIdx then
          local inner, ctr = table.new(eIdx - sIdx + 1, 0), 1
          for k = sIdx, eIdx do
            inner[ctr] = { pos = nodes[k], width = widths[k], binormal = binormals[k] }
            ctr = ctr + 1
          end
          sources[sCtr] = inner
          sCtr = sCtr + 1
        end
      end
    end
  end
  return sources
end

-- Computes a source from a given mesh road.
local function computeSourceFromMeshRoad(road)
  local nodes, widths, binormals = fitFrenetSplineToMeshRoadNodes(editor.getNodes(road))
  local numNodes = #nodes
  local source = table.new(numNodes, 0)
  for i = 1, numNodes do
    source[i] = { pos = nodes[i], width = widths[i], binormal = binormals[i] }
  end
  return source
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

-- Computes the center line of a given RAT road.
local function computeCenterLine(rData)
  local laneId, pId = -1, 2
  if rData[1] then
    laneId, pId = 1, 1
  end
  local numNodes = #rData
  local centerLine = table.new(numNodes, 0)
  for i = 1, numNodes do
    centerLine[i] = rData[i][laneId][pId]
  end
  return centerLine
end

-- Fetches from the scene tree, all the RAT roads (and part-roads) in the given polygon.
local function getAllRATRoadSourcesInPolygon(polygon)
  ratRoads.computeAllRoadRenderData()
  local allObjects = ratRoads.roads
  if not allObjects then
    return {}
  end
  local numObjects = #allObjects
  local sources = table.new(numObjects, 0)
  local sCtr = 1
  for i = 1, numObjects do
    local obj = allObjects[i]
    local lMin, lMax = getMinMaxLaneKeys(obj.profile)
    local rData = obj.renderData
    local centerLine = computeCenterLine(rData)
    local spans = geom.getNodeSpansInsidePolygon(centerLine, polygon)
    for j = 1, #spans do
      local span = spans[j]
      local sIdx, eIdx = span[1], span[2]
      if eIdx > sIdx then
        local inner, ctr = table.new(eIdx - sIdx + 1, 0), 1
        for k = sIdx, eIdx do
          local pos = centerLine[k]
          local pL, pR = rData[k][lMin][1], rData[k][lMax][2]
          local binormal = pR - pL
          binormal:normalize()
          local width = pL:distance(pR)
          inner[ctr] = { pos = pos, width = width, binormal = binormal }
          ctr = ctr + 1
        end
        sources[sCtr] = inner
        sCtr = sCtr + 1
      end
    end
  end
  return sources
end

-- Returns all the sources within the given polygon, which are supported by the terraform tool.
-- [If eg a road goes in and out of the polygon, then it is split and only the inner parts of the road are included, as separate sources.]
local function getAllSourcesInPolygon(polygon)
  local sources, ctr = {}, 1
  local polygonAABB = geom.getAABB(polygon)

  -- Convert all mesh roads in the polygon to sources.
  local meshRoadsInPoly = getAllMeshRoadSourcesInPolygon(polygon, polygonAABB)
  for i = 1, #meshRoadsInPoly do
    sources[ctr] = meshRoadsInPoly[i]
    ctr = ctr + 1
  end

  -- Convert all RAT roads in the polygon to sources.
  local ratRoadsInPoly = getAllRATRoadSourcesInPolygon(polygon)
  for i = 1, #ratRoadsInPoly do
    sources[ctr] = ratRoadsInPoly[i]
    ctr = ctr + 1
  end
--[[
  -- Convert all mesh splines in the polygon to sources.
  local meshSplinesInPoly = getAllSplinesInPolygon(polygon, polygonAABB, meshSplines.getMeshSplines())
  for i = 1, #meshSplinesInPoly do
    sources[ctr] = meshSplinesInPoly[i]
    ctr = ctr + 1
  end

  -- Convert all stitch splines in the polygon to sources.
  local stitchSplinesInPoly = getAllSplinesInPolygon(polygon, polygonAABB, stitchSplines.getStitchSplines())
  for i = 1, #stitchSplinesInPoly do
    sources[ctr] = stitchSplinesInPoly[i]
    ctr = ctr + 1
  end

  -- Convert all rail splines in the polygon to sources.
  local railSplinesInPoly = getAllSplinesInPolygon(polygon, polygonAABB, railSplines.getRailSplines())
  for i = 1, #railSplinesInPoly do
    sources[ctr] = railSplinesInPoly[i]
    ctr = ctr + 1
  end
--]]
  return sources
end

local function getAllSourcesInSelection()
  local sources = {}

  -- Convert all mesh roads in the selection to sources.
  local meshRoadsInSelection = {}
  for _, v in pairs(editor.selection.object) do
    local sel = scenetree.findObjectById(v)
    if sel and sel:getClassName() == "MeshRoad" then
      table.insert(meshRoadsInSelection, computeSourceFromMeshRoad(sel))
    end
  end
  for i = 1, #meshRoadsInSelection do
    table.insert(sources, meshRoadsInSelection[i])
  end

  return sources
end


-- Public interface.
M.getAllSourcesInPolygon =                              getAllSourcesInPolygon
M.getAllSourcesInSelection =                            getAllSourcesInSelection

return M