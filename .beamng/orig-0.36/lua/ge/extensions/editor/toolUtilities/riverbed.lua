-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for terraforming the terrain under a spline to create a riverbed

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local safeMargin = 1 -- Extra margin to add to the grid, to ensure all terraformed points are included in the grid.
local bucketSize = 4.0 -- Fast spline search: World-space bucket size for spatial hashing.
local bucketRadius = 4 -- Fast spline search: The radius of the bucket.
local endTaperFac = 250.0

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require("editor/toolUtilities/geom")
local util = require("editor/toolUtilities/util")

-- Module constants.
local min, max, floor, huge = math.min, math.max, math.floor, math.huge
local sqrt, sin, pi = math.sqrt, math.sin, math.pi
local bucketSizeInv = 1.0 / bucketSize
local endTaperFacInv = 1.0 / endTaperFac

-- Module state.
local tmp1, tmp2, posWS = vec3(), vec3(), vec3()
local gMin, gMax, tmpPoint2I = Point2I(0, 0), Point2I(0, 0), Point2I(0, 0)
local polygon, spansXMin, spansXMax, spansY = {}, {}, {}, {}
local mask, tmpHeight = {}, {}
local setMask, tmpZ, newZ = {}, {}, {}
local buckets = {}


-- Builds a spatial hash table of the division points.
local function buildDivPointBuckets(divPoints)
  table.clear(buckets)
  for i = 1, #divPoints do
    local p = divPoints[i]
    local bx, by = floor(p.x * bucketSizeInv), floor(p.y * bucketSizeInv)
    local key = bx * 65536 + by
    local bucket = buckets[key] or {}
    bucket[#bucket + 1] = i
  end
end

-- Finds the nearest division point to a given point.
local function findNearestDivPoint(px, py, divPoints)
  -- Compute the bucket coordinates for the given point.
  local bx, by = floor(px * bucketSizeInv), floor(py * bucketSizeInv)

  -- Search the local buckets for the nearest division point.
  local nearestIdx, minDistSq = 1, huge
  local found = false
  for ox = -bucketRadius, bucketRadius do
    local fac1 = (bx + ox) * 65536
    for oy = -bucketRadius, bucketRadius do
      local key = fac1 + (by + oy)
      local bucket = buckets[key]
      if bucket then
        found = true
        for i = 1, #bucket do
          local j = bucket[i]
          local dp = divPoints[j]
          local dx2, dy2 = px - dp.x, py - dp.y
          local distSq = dx2 * dx2 + dy2 * dy2
          if distSq < minDistSq then
            minDistSq = distSq
            nearestIdx = j
          end
        end
      end
    end
  end

  -- Fallback: scan all divPoints if none found in local buckets.
  if not found then
    for j = 1, #divPoints do
      local dp = divPoints[j]
      local dx2, dy2 = px - dp.x, py - dp.y
      local distSq = dx2 * dx2 + dy2 * dy2
      if distSq < minDistSq then
        minDistSq = distSq
        nearestIdx = j
      end
    end
  end

  return nearestIdx, minDistSq
end

-- Smooths the riverbed for a given spline.
local function blur(gMinX, gMaxX, gMinY, gMaxY, tb, smoothingLevel)
  -- Load current terrain heights into the temporary arrays.
  table.clear(tmpZ)
  table.clear(newZ)
  local width, height = gMaxX - gMinX + 1, gMaxY - gMinY + 1
  for i = 1, #spansY do
    local y = spansY[i]
    local yOff = (y - gMinY) * width
    for x = spansXMin[i], spansXMax[i] do
      local idx = yOff + (x - gMinX) + 1
      tmpZ[idx] = tb:getHeightGrid(x, y)
      newZ[idx] = 0
    end
  end

  -- Perform the blurring passes.
  for _ = 1, smoothingLevel do
    -- Horizontal pass (fixed 3-tap: 0.25, 0.5, 0.25).
    for i = 1, #spansY do
      local y = spansY[i]
      local yFac = (y - gMinY) * width + 1
      local xMin, xMax = spansXMin[i], spansXMax[i]
      for x = xMin, xMax do
        local localX = x - gMinX
        local idx = yFac + localX
        if tmpZ[idx] ~= nil then
          local localXMinus1, localXPlus1 = localX - 1, localX + 1
          local zl = (localXMinus1 >= 0) and tmpZ[yFac + localXMinus1] or tmpZ[idx]
          local zc = tmpZ[idx]
          local zr = (localXPlus1 < width) and tmpZ[yFac + localXPlus1] or tmpZ[idx]
          newZ[idx] = 0.25 * zl + 0.5 * zc + 0.25 * zr
        end
      end
    end

    -- Vertical pass (fixed 3-tap: 0.25, 0.5, 0.25).
    for i = 1, #spansY do
      local y = spansY[i]
      local yFac = (y - gMinY) * width
      local xMin, xMax = spansXMin[i], spansXMax[i]
      local row = y - gMinY
      local rowMinus1, rowPlus1 = row - 1, row + 1
      local rowMinus1WidthPlus1, rowWidthPlus1, rowPlus1WidthPlus1 = (rowMinus1 * width) + 1, (row * width) + 1, (rowPlus1 * width) + 1
      for x = xMin, xMax do
        local localX = x - gMinX
        local idx = yFac + localX + 1
        if tmpZ[idx] ~= nil then
          local up = (rowMinus1 >= 0) and newZ[rowMinus1WidthPlus1 + localX] or newZ[idx]
          local mid = newZ[rowWidthPlus1 + localX]
          local dn = (rowPlus1 < height) and newZ[rowPlus1WidthPlus1 + localX] or newZ[idx]
          tmpZ[idx] = 0.25 * up + 0.5 * mid + 0.25 * dn
        end
      end
    end

    -- Write the changes to the terrain, with lateral fade weight.
    for i = 1, #spansY do
      local y = spansY[i]
      local yFac = (y - gMinY) * width + 1
      local xMin, xMax = spansXMin[i], spansXMax[i]
      local xSpan = xMax - xMin
      local xSpanInv = 1.0 / xSpan
      for x = xMin, xMax do
        local localX = x - gMinX
        local idx = yFac + localX
        if tmpZ[idx] ~= nil then
          local fadeWeight = 1.0
          if xSpan > 0 then
            local rel = (x - xMin) * xSpanInv
            fadeWeight = sin(rel * pi) -- smooth 0 -> 1 -> 0 across scanline.
          end
          local oldZ = tb:getHeightGrid(x, y)
          local blendedZ = oldZ * (1.0 - fadeWeight) + tmpZ[idx] * fadeWeight -- Lerp between original and changed height as we approach scanline edge.
          tb:setHeightGrid(x, y, max(0, blendedZ))
        end
      end
    end
  end
end

-- Terraforms the riverbed for a given spline.
local function terraform(spline)
  if #spline.divPoints < 2 then
    return -- Not enough points to create a riverbed.
  end

  -- Get the AABB of the spline and add a margin.
  local bedDepth, bankSharpness, excess = spline.bedDepth, spline.bankSharpness, spline.excess
  local box = geom.getAABB(spline.divPoints)
  local _, wMax = util.getMinMaxWidth(spline)
  local fullMargin = wMax * 0.5 + excess
  box.xMin, box.xMax = box.xMin - fullMargin, box.xMax + fullMargin
  box.yMin, box.yMax = box.yMin - fullMargin, box.yMax + fullMargin

  -- Convert the AABB from world space to grid space.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  tmp1:set(box.xMin, box.yMin, 0) te:worldToGridByPoint2I(tmp1, gMin, tb)
  tmp1:set(box.xMax, box.yMax, 0) te:worldToGridByPoint2I(tmp1, gMax, tb)
  local gMinX, gMinY, gMaxX, gMaxY = gMin.x, gMin.y, gMax.x, gMax.y

  -- Compute the polygon which outlines the spline, and includes a margin.
  geom.computeSplinePolygon(spline, excess, polygon)

  -- Get the scanline spans for the polygon.
  geom.getScanlineSpans(gMinY, gMaxY, te, tb, polygon, spansXMin, spansXMax, spansY)

  -- Build the spatial hash table of the division points.
  buildDivPointBuckets(spline.divPoints)

  -- Main terraforming loop.
  table.clear(setMask)
  table.clear(spline.riverbedDataX)
  table.clear(spline.riverbedDataY)
  table.clear(spline.riverbedDataVals)
  local xVals, yVals, zOldVals = spline.riverbedDataX, spline.riverbedDataY, spline.riverbedDataVals
  local divPoints, divWidths = spline.divPoints, spline.divWidths
  local startPoint, endPoint = divPoints[1], divPoints[#divPoints]
  local width, ctr = gMaxX - gMinX + 1, 1
  for i = 1, #spansY do
    local y = spansY[i]
    for x = spansXMin[i] - 4, spansXMax[i] + 4 do
      tmpPoint2I.x, tmpPoint2I.y = x, y
      posWS:set(te:gridToWorldByPoint2I(tmpPoint2I, tb))
      local idx = (y - gMinY) * width + (x - gMinX) + 1 -- 2D -> 1D index used for the mask.
      if not setMask[idx] then -- The mask ensures we only write to a grid point once, to keep the history revertible.
        -- For this grid point, find the nearest point on the spline, and its distance.
        local nearestIdx, minDistSq = findNearestDivPoint(posWS.x, posWS.y, divPoints)

        -- Compute a multiplier to reduce the offset at the ends of the spline.
        local distToEnd = min(posWS:squaredDistance(startPoint), posWS:squaredDistance(endPoint))
        local normDist = clamp(distToEnd * endTaperFacInv, 0.0, 1.0)
        local fac = normDist * normDist * (3.0 - 2.0 * normDist) -- Stays low near the ends, then smoothly ramps up

        -- Compute the vertical (Z) offset for the current grid point.
        -- [This is done by first computing a normalised distance (t) from the nearest point on the spline, to approximate lateral distance,
        -- then using this to compute the offset in the cross-sectional band.]
        local t = min(1.0, sqrt(minDistSq) / (divWidths[nearestIdx] * 0.5 + excess))
        local u = t * 2.0
        local ramp = max(0.0, 0.5 - t) * 2.0 -- 1 when t = 0, 0 when t >= 0.5.
        local shape = (1.0 - u ^ bankSharpness) * ramp
        local offsetZ = -bedDepth * shape * fac

        -- Apply the change for this grid point to the terrain.
        local gridX, gridY = tmpPoint2I.x, tmpPoint2I.y
        local oldZ = tb:getHeightGrid(gridX, gridY)
        tb:setHeightGrid(gridX, gridY, max(0, oldZ + offsetZ))
        xVals[ctr], yVals[ctr], zOldVals[ctr] = gridX, gridY, oldZ
        ctr = ctr + 1
        setMask[idx] = true -- Mark this grid point as terraformed, so we do not re-visit it in later iterations.
      end
    end
  end

  -- Smoothing pass to reduce jagginess inside the terraformed region.
  blur(gMinX, gMaxX, gMinY, gMaxY, tb, spline.smoothingLevel)

  -- Apply the smoothed heights to terrain
  for x = gMinX, gMaxX do
    local xOff = x - gMinX
    for y = gMinY, gMaxY do
      local yOff = y - gMinY
      local idx = yOff * width + xOff + 1
      if mask[idx] == 1 then
        tb:setHeightGrid(x, y, max(0, tmpHeight[idx]))
      end
    end
  end

  -- Expand bounding box to include smoothed fringe.
  -- [This ensures that when reverted, we catch all the terraformed points.]
  gMin.x, gMin.y, gMax.x, gMax.y = gMin.x - safeMargin, gMin.y - safeMargin, gMax.x + safeMargin, gMax.y + safeMargin

  -- Update the grid.
  tmp1:set(gMinX, gMinY, 0)
  tmp2:set(gMaxX, gMaxY, 0)
  tb:updateGrid(tmp1, tmp2)

  -- Store the riverbed data in the spline.
  spline.riverbedDataX = xVals
  spline.riverbedDataY = yVals
  spline.riverbedDataVals = zOldVals
  spline.riverbedDataBoxXMin = gMinX
  spline.riverbedDataBoxXMax = gMaxX
  spline.riverbedDataBoxYMin = gMinY
  spline.riverbedDataBoxYMax = gMaxY
end

-- Reverts the riverbed terraforming for a given spline.
local function revert(spline)
  if not spline.riverbedDataVals then
    return -- No riverbed data to revert. Leave immediately.
  end

  -- Revert the riverbed.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local stateX, stateY, stateVals = spline.riverbedDataX, spline.riverbedDataY, spline.riverbedDataVals
  for i = 1, #stateVals do
    tb:setHeightGrid(stateX[i], stateY[i], max(0, stateVals[i]))
  end

  -- Update the grid.
  tmp1:set(spline.riverbedDataBoxXMin, spline.riverbedDataBoxYMin, 0)
  tmp2:set(spline.riverbedDataBoxXMax, spline.riverbedDataBoxYMax, 0)
  tb:updateGrid(tmp1, tmp2)

  -- Clear the riverbed data from the spline.
  table.clear(spline.riverbedDataX)
  table.clear(spline.riverbedDataY)
  table.clear(spline.riverbedDataVals)
  spline.riverbedDataBoxXMin, spline.riverbedDataBoxXMax, spline.riverbedDataBoxYMin, spline.riverbedDataBoxYMax = 0, 0, 0, 0
end

-- Reverts all riverbed terraforming for all splines.
local function revertAll(splines)
  for i = 1, #splines do
    revert(splines[i])
  end
end


-- Public interface.
M.terraform =                                           terraform

M.revert =                                              revert
M.revertAll =                                           revertAll

return M