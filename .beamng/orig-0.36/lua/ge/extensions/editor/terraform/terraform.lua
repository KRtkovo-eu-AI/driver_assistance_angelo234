-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local averagingMargin = 2.0                                                                         -- Used when averaging the mask.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


local M = {}

-- External modules.
local kdTreeB2d = require('kdtreebox2d')

-- Module constants.
local min, max, floor, ceil, sqrt, huge = math.min, math.max, math.floor, math.ceil, math.sqrt, math.huge

-- Module state.
local tmp1, tmp2 = vec3(0, 0, 0), vec3(0, 0, 0)


-- Undo callback for terraforming operations.
local function terraformUndo(data)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  if not tb then
    return
  end

  -- Recover the heightmap.
  local xMin, xMax, yMin, yMax = huge, -huge, huge, -huge
  for i = 1, #data do
    local d = data[i]
    tb:setHeight(d.x, d.y, max(0.0, d.old))
    xMin, xMax, yMin, yMax = min(xMin, d.x), max(xMax, d.x), min(yMin, d.y), max(yMax, d.y)
  end

  -- Update the grid after the changes.
  tmp1:set(xMin, yMin, 0)
  tmp2:set(xMax, yMax, 0)
  tb:updateGrid(tmp1, tmp2)
end

-- Redo callback for terraforming operations.
local function terraformRedo(data)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  if not tb then return end

  -- Recover the heightmap.
  local xMin, xMax, yMin, yMax = huge, -huge, huge, -huge
  for i = 1, #data do
    local d = data[i]
    tb:setHeight(d.x, d.y, max(0.0, d.new))
    xMin, xMax, yMin, yMax = min(xMin, d.x), max(xMax, d.x), min(yMin, d.y), max(yMax, d.y)
  end

  -- Update the grid after the changes.
  tmp1:set(xMin, yMin, 0)
  tmp2:set(xMax, yMax, 0)
  tb:updateGrid(tmp1, tmp2)
end

-- Intersects the given point with the given quadrilateral, and returns the height at the intersection point.
local function intersectsUpQuadBarycentric(p, q)
  local u, v = p:invBilinear2D(q[1], q[2], q[3], q[4])
  if u >= 0.0 and u <= 1.0 and v >= 0.0 and v <= 1.0 then
    return lerp(lerp(q[1].z, q[2].z, u), lerp(q[3].z, q[4].z, u), v)
  end
  return false
end

-- Computes the bounding box of the given sources.
local function computeSourcesAABB(sources)
  local xMin, xMax, yMin, yMax = huge, -huge, huge, -huge
  for i = 1, #sources do
    local source = sources[i]
    for j = 1, #source do
      local p = source[j].pos
      local x, y = p.x, p.y
      xMin = min(xMin, x)
      xMax = max(xMax, x)
      yMin = min(yMin, y)
      yMax = max(yMax, y)
    end
  end
  return { xMin = xMin, xMax = xMax, yMin = yMin, yMax = yMax }
end

-- Averages the height with neighbouring points.
local function averageMask(height, mod, fixedMask, xSize, ySize)
  local xTop, yTop = xSize - 1, ySize - 1
  for x = 1, xTop do
    local modX, fixedMaskX = mod[x], fixedMask[x]
    for y = 1, yTop do
      if modX[y] > 0.5 and fixedMaskX[y] < 0.5 then
        local sum, ctr = 0.0, 0
        for dx = -averagingMargin, averagingMargin do
          for dy = -averagingMargin, averagingMargin do
            local nx, ny = x + dx, y + dy
            if nx >= 0.0 and nx <= xSize and ny >= 0.0 and ny <= ySize then
              sum = sum + height[nx][ny]
              ctr = ctr + 1
            end
          end
        end
        height[x][y] = sum / ctr
      end
    end
  end
end

-- Gets the quads from the relevant sources.
local function getAllQuadrilaterals(sources)
  local quads, ctr = {}, 1
  for i = 1, #sources do
    local source = sources[i]
    for j = 2, #source do
      local s1, s2 = source[j - 1], source[j]
      local p1, p2 = s1.pos, s2.pos
      local b1, b2 = s1.binormal, s2.binormal
      local lateral1, lateral2 = b1 * s1.width * 0.5, b2 * s2.width * 0.5
      quads[ctr] = { p2 - lateral2, p2 + lateral2, p1 - lateral1, p1 + lateral1 }
      ctr = ctr + 1
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
    local qAx, qAy, qBx, qBy, qCx, qCy, qDx, qDy = qA.x, qA.y, qB.x, qB.y, qC.x, qC.y, qD.x, qD.y
    tree:preLoad(i, min(qAx, qBx, qCx, qDx), min(qAy, qBy, qCy, qDy), max(qAx, qBx, qCx, qDx), max(qAy, qBy, qCy, qDy))
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

  -- Update the terrain block.
  tmp1:set(bXMin, bYMin, 0)
  tmp2:set(bXMax, bYMax, 0)
  tb:updateGrid(tmp1, tmp2)
  editor_terrainEditor.setTerrainDirty()

  -- Commit the terraforming action to the undo/redo history.
  editor.history:commitAction("Terraform", history, terraformUndo, terraformRedo)
end

-- Dilates the mask with the given radius, and returns the dilated mask and corresponding heights.
local function dilateMaskWithHeights(mask, heights, xSize, ySize, radius)
  local newMask, newHeights, countMap = {}, {}, {}

  local radiusCeil = ceil(radius)
  local radiusSquared = radius * radius

  -- Initialise outputs.
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

  -- Perform circular dilation with blending.
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

-- Terraforms the current heightmap using the given terraforming data.
-- The sources structure is an array containing each source element (eg a road).
-- Each source element is an array containing ordered polyline points with the following structure:
-- { pos = vec3(x, y, z), width = f, binormal = vec3(x, y, z) }.
-- 'DOI' (Domain of Influence) is the max distance at which the terraforming will affect, in meters.
-- 'margin' is the distance which the terraforming will affect the outer edge of the sources, in meters.
local function terraformToSources(DOI, margin, sources)
  -- If there are no sources then leave immediately.
  if not sources then
    return
  end

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  -- Fetch the terrain block extents.
  local extents = tb:getWorldBox():getExtents()
  local center = tb:getWorldBox():getCenter()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = center.x - xHalf, center.x + xHalf, center.y - yHalf, center.y + yHalf
  local zMin = tb:getPosition().z

  -- Compute the bounding box of the sources as a whole.
  local box = computeSourcesAABB(sources)
  DOI = max(5.0, DOI)
  box.xMin = box.xMin - DOI
  box.xMax = box.xMax + DOI
  box.yMin = box.yMin - DOI
  box.yMax = box.yMax + DOI

  -- Initialize the mask.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  tmp1:set(floor(max(tXMin, box.xMin)), floor(max(tYMin, box.yMin)), 0)
  te:worldToGridByPoint2I(tmp1, gMin, tb)
  tmp2:set(ceil(min(tXMax, box.xMax)), ceil(min(tYMax, box.yMax)), 0)
  te:worldToGridByPoint2I(tmp2, gMax, tb)
  local bXMin, bXMax, bYMin, bYMax = gMin.x, gMax.x, gMin.y, gMax.y
  local xSize, ySize = bXMax - bXMin, bYMax - bYMin

  -- Compute the quads, expanded up to the inner margin.
  local quads = getAllQuadrilaterals(sources)

  -- Populate a kd-tree with the quads.
  local tree = populateTreeQuads(quads)

  -- Iterate over the grid bounding box, and add contributions to the road mask.
  local innerMask, innerHeights = {}, {}
  for x = 0, xSize do
    local innerMaskX, innerHeightsX = {}, {}
    local gX = bXMin + x
    for y = 0, ySize do
      local gY = bYMin + y
      local pWS = te:gridToWorldByPoint2I(Point2I(gX, gY), tb)
      innerMaskX[y] = 0
      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local z = intersectsUpQuadBarycentric(pWS, quads[tIdx])
        if z then
          innerMaskX[y] = 1
          innerHeightsX[y] = min(z - zMin, innerHeightsX[y] or huge)
        end
      end
      if not innerHeightsX[y] then
        innerHeightsX[y] = max(0, tb:getHeightGrid(gX, gY))
      end
    end
    innerMask[x] = innerMaskX
    innerHeights[x] = innerHeightsX
  end

  -- Dilate the masks. The inner mask is dilated by 1 to avoid edge effects, and the outer mask is dilated by the given margin.
  innerMask, innerHeights = dilateMaskWithHeights(innerMask, innerHeights, xSize, ySize, 1)
  local outerMask, outerHeights = dilateMaskWithHeights(innerMask, innerHeights, xSize, ySize, margin)

  -- Create the mod structure. [A structure which stores the increasing domain of influence].
  local mod = {}
  local chMod = {}
  for x = 0, xSize do
    local maskX =  outerMask[x]
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

  -- Calculate the number of iterations needed.
  local numIter = ceil(0.5 * sqrt(8 * DOI + 1) - 1)

  -- Iteratively process the mask.
  for i = numIter, 1, -1 do
    local halfkernSizeL = i
    local kernSizeL = halfkernSizeL * 2 + 1
    local invI = 1 / kernSizeL
    local xStart, xEnd = halfkernSizeL + 1, xSize - halfkernSizeL - 1
    local yStart, yEnd = halfkernSizeL + 1, ySize - halfkernSizeL - 1

    -- X pass.
    for y = yStart, yEnd do
      local numerS, denomS = 0, 0
      for s = 1, kernSizeL do
        numerS, denomS = numerS + outerHeights[s][y], denomS + mod[s][y]
      end

      for x = xStart, xEnd do
        if denomS == 0 then
          changes[x][y] = outerHeights[x][y]
        else
          changes[x][y] = numerS * invI
          chMod[x][y] = 1
        end
        local frontEdge, backEdge = x + xStart, x - halfkernSizeL
        numerS = numerS + outerHeights[frontEdge][y] - outerHeights[backEdge][y]
        denomS = denomS + mod[frontEdge][y] - mod[backEdge][y]
      end
    end

    -- Y pass.
    for x = xStart, xEnd do
      local numerS, denomS = 0, 0
      local heightX, modX, chModX, chX = outerHeights[x], mod[x], chMod[x], changes[x]
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
      local maskX, heightX, modX, chModX, chX = outerMask[x], outerHeights[x], mod[x], chMod[x], changes[x]
      for y = yStart, yEnd do
        local m = maskX[y]
        heightX[y] = (1 - m) * chX[y] + m * heightX[y]
        modX[y] = chModX[y]
      end
    end
  end

  -- Average the height with neighbouring points.
  averageMask(outerHeights, mod, innerMask, xSize, ySize)

  -- Terraform the heightmap from the processed mask.
  modifyTerrainFromHeightArray(outerHeights, mod, xSize, ySize, bXMin, bXMax, bYMin, bYMax, tb)
end


-- Public interface.
M.terraformToSources =                                  terraformToSources

return M