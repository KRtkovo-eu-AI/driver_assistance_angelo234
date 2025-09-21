-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local averagingMargin = 2.0                                                                         -- Used when averaging the mask.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'RoadArchitect'

-- External modules used.
local kdTreeB2d = require('kdtreebox2d')
local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- For managing the road structure/handling road calculations.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- For managing the profiles structure.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local min, max, floor, ceil, sqrt = math.min, math.max, math.floor, math.ceil, math.sqrt


-- Undo callback for terraforming operations.
local function terraformUndo(data)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  if not tb then
    return
  end

  -- Recover the heightmap.
  local xMin, xMax, yMin, yMax = 1e99, -1e99, 1e99, -1e99
  for i = 1, #data do
    local d = data[i]
    tb:setHeight(d.x, d.y, max(0, d.old))
    xMin, xMax, yMin, yMax = min(xMin, d.x), max(xMax, d.x), min(yMin, d.y), max(yMax, d.y)
  end

  -- Update the grid after the changes.
  tb:updateGrid(vec3(xMin, yMin), vec3(xMax, yMax))
end

-- Redo callback for terraforming operations.
local function terraformRedo(data)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  if not tb then return end

  -- Recover the heightmap.
  local xMin, xMax, yMin, yMax = 1e99, -1e99, 1e99, -1e99
  for i = 1, #data do
    local d = data[i]
    tb:setHeight(d.x, d.y, max(0, d.new))
    xMin, xMax, yMin, yMax = min(xMin, d.x), max(xMax, d.x), min(yMin, d.y), max(yMax, d.y)
  end

  -- Update the grid after the changes.
  tb:updateGrid(vec3(xMin, yMin), vec3(xMax, yMax))
end

local function intersectsUp_Quad_Barycentric(p, q)
  local u, v = p:invBilinear2D(q[1], q[2], q[3], q[4])
  if u >= 0 and u <= 1 and v >= 0 and v <= 1 then
    return lerp(lerp(q[1].z, q[2].z, u), lerp(q[3].z, q[4].z, u), v)
  end
  return false
end

-- Averages the height with neighbouring points.
local function averageMask(height, mod, fixedMask, xSize, ySize)
  local xTop, yTop = xSize - 1, ySize - 1
  for x = 1, xTop do
    local modX, fixedMaskX = mod[x], fixedMask[x]
    for y = 1, yTop do
      if modX[y] > 0.5 and fixedMaskX[y] < 0.5 then
        local sum, count = 0.0, 0
        for dx = -averagingMargin, averagingMargin do
          for dy = -averagingMargin, averagingMargin do
            local nx, ny = x + dx, y + dy
            if nx >= 0.0 and nx <= xSize and ny >= 0.0 and ny <= ySize then
              sum = sum + height[nx][ny]
              count = count + 1
            end
          end
        end
        height[x][y] = sum / count
      end
    end
  end
end

-- Gets the road quads. Bloats the geometry by the given amount [top surface only], if required.
local function getQuads(road)
  local rData = road.renderData
  if #rData < 2 then
    return
  end

  -- Bloat the outermost points laterally, using the local binormal (lateral) vector.
  local lMin, lMax = profileMgr.getMinMaxLaneKeys(road.profile)
  local left, right = {}, {}
  local lastIdx = #rData
  for i = 1, lastIdx do
    local rD = rData[i]
    left[i] = rD[lMin][4]
    right[i] = rD[lMax][3]
  end

  -- Now form the quadrilaterals.
  -- [Do not include any inside tunnel sections].
  local tunnels, extraS, extraE = road.tunnels, road.extraS[0], road.extraE[0]
  local quads = {}
  for i = 2, lastIdx do
    if not util.isInTunnel(i, tunnels, extraS, extraE) then
      local iMinus1 = i - 1
      quads[iMinus1] = { left[i], right[i], left[iMinus1], right[iMinus1] }
    end
  end

  return quads
end

-- Gets the quads from the relevant roads.
local function getQuadsMulti(roads, box)
  local quads, ctr = {}, 1
  for _, road in pairs(roads) do
    local rData = road.renderData
    if #rData > 1 and not road.isBridge then

      -- Bloat the outermost points laterally, using the local binormal (lateral) vector.
      local lMin, lMax = profileMgr.getMinMaxLaneKeys(road.profile)
      local left, right = {}, {}
      local lastIdx = #rData
      for i = 1, lastIdx do
        local rD = rData[i]
        left[i] = rD[lMin][4]
        right[i] = rD[lMax][3]
      end

      -- Now form the quads.
      -- [Do not include any inside tunnel sections].
      local tunnels, extraS, extraE = road.tunnels, road.extraS[0], road.extraE[0]
      for i = 2, lastIdx do
        if util.isInBox(left[i], box) and not util.isInTunnel(i, tunnels, extraS, extraE) then
          local iMinus1 = i - 1
          quads[ctr] = { left[i], right[i], left[iMinus1], right[iMinus1] }
          ctr = ctr + 1
        end
      end
    end
  end

  return quads
end

-- Creates and populates a kd-tree containing the given quadrilaterals.
local function populateTreeQuads(quads)
  local tree = kdTreeB2d.new(#quads)
  for i = 1, #quads do
    local q = quads[i]
    local qA, qB, qC, qD = q[1], q[2], q[3], q[4]
    tree:preLoad(i, min(qA.x, qB.x, qC.x, qD.x), min(qA.y, qB.y, qC.y, qD.y), max(qA.x, qB.x, qC.x, qD.x), max(qA.y, qB.y, qC.y, qD.y))
  end
  tree:build()
  return tree
end

-- Terraforms the heightmap using the given terraforming data.
-- [Also commits the modification to support undo/redo].
local function modifyTerrainFromHeightArray(height, mod, xSize, ySize, bXMin, bXMax, bYMin, bYMax, tb)
  local history, hCtr = {}, 1
  for x = 0, xSize do
    local heightX, modX, rx = height[x], mod[x], x + bXMin
    for y = 0, ySize do
      if modX[y] > 0.5 then
        local ry = y + bYMin
        local z = heightX[y]
        local zOld = max(0, tb:getHeightGrid(rx, ry))
        tb:setHeightGrid(rx, ry, max(0, z))
        history[hCtr] = { old = zOld, new = z, x = rx, y = ry }
        hCtr = hCtr + 1
      end
    end
  end
  editor.history:commitAction("Terraform", history, terraformUndo, terraformRedo)

  -- Update the terrain block.
  tb:updateGrid(vec3(bXMin, bYMin), vec3(bXMax, bYMax))
end

-- Dilates the mask with the given radius, and returns the dilated mask and corresponding heights.
local function dilateMaskWithHeights(mask, heights, xSize, ySize, radius)
  local newMask = {}
  local newHeights = {}
  local countMap = {}

  local radiusCeil = ceil(radius)
  local radiusSquared = radius * radius

  -- Initialize outputs
  for x = -radiusCeil, xSize + radiusCeil do
    newMask[x] = {}
    newHeights[x] = {}
    countMap[x] = {}
    for y = -radiusCeil, ySize + radiusCeil do
      newMask[x][y] = mask[x] and mask[x][y] or 0
      newHeights[x][y] = heights[x] and heights[x][y] or 0
      countMap[x][y] = newMask[x][y] > 0 and 1 or 0
    end
  end

  -- Perform circular dilation with blending
  for x = 0, xSize do
    for y = 0, ySize do
      if mask[x] and mask[x][y] == 1 then
        local h = heights[x][y] or 0
        for dx = -radiusCeil, radiusCeil do
          for dy = -radiusCeil, radiusCeil do
            if dx * dx + dy * dy <= radiusSquared then
              local nx, ny = x + dx, y + dy
              if nx >= 0 and nx <= xSize and ny >= 0 and ny <= ySize then
                newMask[nx][ny] = 1
                newHeights[nx][ny] = (newHeights[nx][ny] * countMap[nx][ny] + h) / (countMap[nx][ny] + 1)
                countMap[nx][ny] = countMap[nx][ny] + 1
              end
            end
          end
        end
      end
    end
  end

  return newMask, newHeights
end

-- Conforms the local terrain to the road.
local function conformTerrainToRoad(rIdx, DOI, margin)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf
  local zMin, zMax = tb:getPosition().z, tb.maxHeight

  -- Compute the AABB.
  local box = roadMgr.computeAABB2D(rIdx)
  DOI = max(5.0, DOI)
  box.xMin, box.xMax, box.yMin, box.yMax = box.xMin - DOI, box.xMax + DOI, box.yMin - DOI, box.yMax + DOI

  -- Initialize the mask.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(max(tXMin, box.xMin)), floor(max(tYMin, box.yMin))), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(min(tXMax, box.xMax)), ceil(min(tYMax, box.yMax))), gMax, tb)
  local bXMin, bXMax, bYMin, bYMax = gMin.x, gMax.x, gMin.y, gMax.y
  local xSize, ySize = bXMax - bXMin, bYMax - bYMin

  -- Collect the coarse triangles and bloat them up to the inner margin, then populate a kd-tree with them.
  local quads = getQuads(roadMgr.roads[rIdx])
  local tree = populateTreeQuads(quads)

  -- Iterate over the grid bounding box, and add contributions to the road mask.
  local fixedMask, fixedHeights = {}, {}
  for x = 0, xSize do
    local fixedMaskX, fixedHeightsX = {}, {}
    local gX = bXMin + x
    for y = 0, ySize do
      local gY = bYMin + y
      local pWS = te:gridToWorldByPoint2I(Point2I(gX, gY), tb)
      fixedMaskX[y] = 0
      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local z = intersectsUp_Quad_Barycentric(pWS, quads[tIdx])
        if z then
          fixedMaskX[y] = 1
          fixedHeightsX[y] = min(z - zMin, fixedHeightsX[y] or math.huge)
        end
      end
      if not fixedHeightsX[y] then
        fixedHeightsX[y] = max(0, tb:getHeightGrid(gX, gY))
      end
    end
    fixedMask[x] = fixedMaskX
    fixedHeights[x] = fixedHeightsX
  end

  local fixedMask, fixedHeights = dilateMaskWithHeights(fixedMask, fixedHeights, xSize, ySize, 1) -- dilate the mask by 1 to avoid edge effects
  local mask, height = dilateMaskWithHeights(fixedMask, fixedHeights, xSize, ySize, margin)

  -- Create the mod structure.
  -- [A structure which stores the increasing domain of influence].
  local mod = {}
  local chMod = {}
  for x = 0, xSize do
    local maskX =  mask[x]
    mod[x], chMod[x] = {}, {}
    for y = 0, ySize do
      mod[x][y] = maskX[y]
      chMod[x][y] = maskX[y]
    end
  end

  -- Allocate the changes structure.
  local changes = {}
  for x = 0, xSize do
    changes[x] = {}
    local chCol = changes[x]
    for y = 0, ySize do
      chCol[y] = 0.0
    end
  end

  local numIter = ceil(0.5 * sqrt(8 * DOI + 1) - 1)

  -- Iteratively process the mask.
  for i = numIter, 1, -1 do
    local halfkernSizeL = i
    local kernSizeL = halfkernSizeL * 2 + 1
    local invI = 1 / kernSizeL
    local xStart, xEnd = halfkernSizeL + 1, xSize - halfkernSizeL - 1
    local yStart, yEnd = halfkernSizeL + 1, ySize - halfkernSizeL - 1

    -- X.
    for y = yStart, yEnd do
      local numerS, denomS = 0, 0
      for s = 1, kernSizeL do
        numerS, denomS = numerS + height[s][y], denomS + mod[s][y]
      end

      for x = xStart, xEnd do
        if denomS == 0 then
          changes[x][y] = height[x][y]
        else
          changes[x][y] = numerS * invI
          chMod[x][y] = 1
        end
        local frontEdge, backEdge = x + xStart, x - halfkernSizeL
        numerS = numerS + height[frontEdge][y] - height[backEdge][y]
        denomS = denomS + mod[frontEdge][y] - mod[backEdge][y]
      end
    end

    -- Y.
    for x = xStart, xEnd do
      local numerS, denomS = 0, 0
      local heightX, modX, chModX, chX = height[x], mod[x], chMod[x], changes[x]
      for s = 1, kernSizeL do
        numerS, denomS = numerS + heightX[s], denomS + modX[s]
      end

      for y = yStart, yEnd do
        if denomS ~= 0 then
          chX[y] = (chX[y] + numerS * invI) * 0.5
          chModX[y] = 1
        end
        local frontEdge, backEdge = y + xStart, y - halfkernSizeL
        numerS = numerS + heightX[frontEdge] - heightX[backEdge]
        denomS = denomS + modX[frontEdge] - modX[backEdge]
      end
    end

    -- Copy the changes onto the mask, reset the fixed mask points and reset the changes array.
    for x = xStart, xEnd do
      local maskX, heightX, modX, chModX, chX = mask[x], height[x], mod[x], chMod[x], changes[x]
      for y = yStart, yEnd do
        local m = maskX[y]
        heightX[y] = (1 - m) * chX[y] + m * heightX[y]
        modX[y] = chModX[y]
      end
    end
  end

  -- Average the height with neighbouring points.
  averageMask(height, mod, fixedMask, xSize, ySize)

  -- Terraform the heightmap from the processed mask.
  modifyTerrainFromHeightArray(height, mod, xSize, ySize, bXMin, bXMax, bYMin, bYMax, tb)
end

-- Terraforms the whole terrain block to the existing road network (or the group with the given index).
-- [Does not include overlays or bridges].
local function terraformMultiRoads(DOI, margin, group)

  roadMgr.computeAllRoadRenderData()

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf
  local zMin, zMax = tb:getPosition().z, tb.maxHeight

  -- Compute the bounding box of the whole road network or group (if given).
  local roads = roadMgr.getRoadsFromGroup(group)
  local box = util.computeAABB2DGroup(group, roadMgr.roads, roadMgr.map)
  DOI = max(5.0, DOI)
  box.xMin = box.xMin - DOI
  box.xMax = box.xMax + DOI
  box.yMin = box.yMin - DOI
  box.yMax = box.yMax + DOI

  -- Initialize the mask.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(max(tXMin, box.xMin)), floor(max(tYMin, box.yMin))), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(min(tXMax, box.xMax)), ceil(min(tYMax, box.yMax))), gMax, tb)
  local bXMin, bXMax, bYMin, bYMax = gMin.x, gMax.x, gMin.y, gMax.y
  local xSize, ySize = bXMax - bXMin, bYMax - bYMin

  -- Compute the quads, expanded up to the inner margin, then populate a kd-tree with them.
  local quads = getQuadsMulti(roads, box)
  local tree = populateTreeQuads(quads)

  -- Iterate over the grid bounding box, and add contributions to the road mask.
  local fixedMask, fixedHeights = {}, {}
  for x = 0, xSize do
    local fixedMaskX, fixedHeightsX = {}, {}
    local gX = bXMin + x
    for y = 0, ySize do
      local gY = bYMin + y
      local pWS = te:gridToWorldByPoint2I(Point2I(gX, gY), tb)
      fixedMaskX[y] = 0
      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local z = intersectsUp_Quad_Barycentric(pWS, quads[tIdx])
        if z then
          fixedMaskX[y] = 1
          fixedHeightsX[y] = min(z - zMin, fixedHeightsX[y] or math.huge)
        end
      end
      if not fixedHeightsX[y] then
        fixedHeightsX[y] = max(0, tb:getHeightGrid(gX, gY))
      end
    end
    fixedMask[x] = fixedMaskX
    fixedHeights[x] = fixedHeightsX
  end

  local fixedMask, fixedHeights = dilateMaskWithHeights(fixedMask, fixedHeights, xSize, ySize, 1) -- dilate the mask by 1 to avoid edge effects
  local mask, height = dilateMaskWithHeights(fixedMask, fixedHeights, xSize, ySize, margin)

  -- Create the mod structure.
  -- [A structure which stores the increasing domain of influence].
  local mod = {}
  local chMod = {}
  for x = 0, xSize do
    local maskX =  mask[x]
    mod[x], chMod[x] = {}, {}
    for y = 0, ySize do
      mod[x][y] = maskX[y]
      chMod[x][y] = maskX[y]
    end
  end

  -- Allocate the changes structure.
  local changes = {}
  for x = 0, xSize do
    changes[x] = {}
    local chCol = changes[x]
    for y = 0, ySize do
      chCol[y] = 0.0
    end
  end

  local numIter = ceil(0.5 * sqrt(8 * DOI + 1) - 1)

  -- Iteratively process the mask.
  for i = numIter, 1, -1 do
    local halfkernSizeL = i
    local kernSizeL = halfkernSizeL * 2 + 1
    local invI = 1 / kernSizeL
    local xStart, xEnd = halfkernSizeL + 1, xSize - halfkernSizeL - 1
    local yStart, yEnd = halfkernSizeL + 1, ySize - halfkernSizeL - 1

    -- X.
    for y = yStart, yEnd do
      local numerS, denomS = 0, 0
      for s = 1, kernSizeL do
        numerS, denomS = numerS + height[s][y], denomS + mod[s][y]
      end

      for x = xStart, xEnd do
        if denomS == 0 then
          changes[x][y] = height[x][y]
        else
          changes[x][y] = numerS * invI
          chMod[x][y] = 1
        end
        local frontEdge, backEdge = x + xStart, x - halfkernSizeL
        numerS = numerS + height[frontEdge][y] - height[backEdge][y]
        denomS = denomS + mod[frontEdge][y] - mod[backEdge][y]
      end
    end

    -- Y.
    for x = xStart, xEnd do
      local numerS, denomS = 0, 0
      local heightX, modX, chModX, chX = height[x], mod[x], chMod[x], changes[x]
      for s = 1, kernSizeL do
        numerS, denomS = numerS + heightX[s], denomS + modX[s]
      end

      for y = yStart, yEnd do
        if denomS ~= 0 then
          chX[y] = (chX[y] + numerS * invI) * 0.5
          chModX[y] = 1
        end
        local frontEdge, backEdge = y + xStart, y - halfkernSizeL
        numerS = numerS + heightX[frontEdge] - heightX[backEdge]
        denomS = denomS + modX[frontEdge] - modX[backEdge]
      end
    end

    -- Copy the changes onto the mask, reset the fixed mask points and reset the changes array.
    for x = xStart, xEnd do
      local maskX, heightX, modX, chModX, chX = mask[x], height[x], mod[x], chMod[x], changes[x]
      for y = yStart, yEnd do
        local m = maskX[y]
        heightX[y] = (1 - m) * chX[y] + m * heightX[y]
        modX[y] = chModX[y]
      end
    end
  end

  -- Average the height with neighbouring points.
  averageMask(height, mod, fixedMask, xSize, ySize)

  -- Terraform the heightmap from the processed mask.
  modifyTerrainFromHeightArray(height, mod, xSize, ySize, bXMin, bXMax, bYMin, bYMax, tb)
end

-- Saves the heightmap to the given .png file.
local function writeHeightmapToPng(path)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return {}
  end
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf
  local xSize, ySize = tXMax - tXMin, tYMax - tYMin

  local zMin, zMax = tb:getPosition().z, tb.maxHeight
  local uint16Scale = 65535 / (zMax - zMin)

  local bmp = GBitmap()
  bmp:init(xSize, ySize)
  bmp:allocateBitmap(xSize, ySize, false, "GFXFormatR16")
  for x = 0, xSize do
    for y = 0, ySize do
      local val = (max(0, max(tb:getHeightGrid(x, y))) - zMin) * uint16Scale
      bmp:setTexel(x, y, val, val, val, val)
    end
  end
  bmp:saveFile(path)
end

-- Imports a terrain from the given path.
local function setHeightmapFromPng(path)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf
  local xSize, ySize = tXMax - tXMin, tYMax - tYMin

  local zMin, zMax = tb:getPosition().z, tb.maxHeight
  local uint16Scale = 65535 / (zMax - zMin)
  local uint16ScaleInv = 1.0 / uint16Scale

  -- Load the terrain.
  local bmp = GBitmap()
  if not bmp:loadFile(path) then
    log('E', logTag, 'Failed to load heightmap (.png) file [from path: ' .. path .. ']')
  end

  -- Apply the bitmap to the heightmap.
  for x = 0, xSize do
    local rx = x + tXMin
    for y = 0, ySize do
      tb:setHeightGrid(x, y, max(0, bmp:getTexel(x, y) * uint16ScaleInv) + zMin)
    end
  end

  -- Update the terrain block.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(tXMin), floor(tYMin)), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(tXMax), ceil(tYMax)), gMax, tb)
  tb:updateGrid(vec3(gMin.x, gMin.y), vec3(gMax.x, gMax.y))
end

-- Returns the road masks for the given road group (roads only and roads + margins).
local function getRoadMasks(DOI, margin, group)

  roadMgr.computeAllRoadRenderData()

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf

  -- Compute the bounding box of the whole road network or group (if given).
  local roads = roadMgr.getRoadsFromGroup(group)
  local box = util.computeAABB2DGroup(group, roadMgr.roads, roadMgr.map)
  DOI = max(5.0, DOI)
  box.xMin = box.xMin - DOI
  box.xMax = box.xMax + DOI
  box.yMin = box.yMin - DOI
  box.yMax = box.yMax + DOI

  -- Initialize the mask.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(tXMin, tYMin), gMin, tb)
  te:worldToGridByPoint2I(vec3(tXMax, tYMax), gMax, tb)
  local bXMin, bXMax, bYMin, bYMax = gMin.x, gMax.x, gMin.y, gMax.y
  local xSize, ySize = bXMax - bXMin, bYMax - bYMin

  -- Compute the quads, expanded up to the inner margin, then populate a kd-tree with them.
  local quads = getQuadsMulti(roads, box)
  local tree = populateTreeQuads(quads)

  local roadMask, fixedHeights = {}, {}
  for x = 0, xSize do
    local roadMaskX, fixedHeightsX = {}, {}
    for y = 0, ySize do
      roadMaskX[y] = 0
      local pWS = te:gridToWorldByPoint2I(Point2I(x, y), tb)
      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local z = intersectsUp_Quad_Barycentric(pWS, quads[tIdx])
        if z then
          roadMaskX[y] = 65535
          fixedHeightsX[y] = 0.0 -- TODO: heights are not used. refactor it out perhaps with separate function or flag.
        end
      end
    end
    roadMask[x] = roadMaskX
    fixedHeights[x] = fixedHeightsX
  end

  local roadMask, fixedHeights = dilateMaskWithHeights(roadMask, fixedHeights, xSize, ySize, 1) -- dilate the mask by 1 to avoid edge effects
  local marginMask, _ = dilateMaskWithHeights(roadMask, fixedHeights, xSize, ySize, margin)

  return roadMask, marginMask
end


-- Public interface.
M.conformTerrainToRoad =                                  conformTerrainToRoad
M.terraformMultiRoads =                                   terraformMultiRoads

M.writeHeightmapToPng =                                   writeHeightmapToPng
M.setHeightmapFromPng =                                   setHeightmapFromPng

M.getRoadMasks =                                          getRoadMasks

return M