-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for managing the painting of the terrain underneath a spline.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local defaultMaterialIdx = 1  -- The default index of the painting material, in the material table.
local margin = 2  -- The margin to add to the spline bounds, in meters.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require("editor/toolUtilities/geom")
local util = require("editor/toolUtilities/util")

-- Module state.
local tmp1, tmp2, posWS = vec3(), vec3(), vec3()
local gMin, gMax, tmpPoint2I = Point2I(0, 0), Point2I(0, 0), Point2I(0, 0)
local leftPts, rightPts, polygon = {}, {}, {}
local spansXMin, spansXMax, spansY = {}, {}, {}
local setMask = {}


-- Paints the given spline.
local function paint(group)
  if #group.paintedDataVals > 0 or #group.nodes < 2 or #group.divPoints < 2 then
    return -- If no painted data or spline invalid, then leave immediately.
  end

  -- Compute the AABB of the spline and add a margin.
  local box = geom.getAABB(group.divPoints)
  local _, wMax = util.getMinMaxWidth(group)
  local fullMargin = wMax + margin + group.paintMargin
  box.xMin, box.xMax = box.xMin - fullMargin, box.xMax + fullMargin
  box.yMin, box.yMax = box.yMin - fullMargin, box.yMax + fullMargin

  -- Convert the AABB from world space to grid space.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  tmp1:set(box.xMin, box.yMin, 0) te:worldToGridByPoint2I(tmp1, gMin, tb)
  tmp1:set(box.xMax, box.yMax, 0) te:worldToGridByPoint2I(tmp1, gMax, tb)
  local gMinX, gMinY, gMaxX, gMaxY = gMin.x, gMin.y, gMax.x, gMax.y

  -- Create the left and right edge points.
  local divPoints, binormals, divWidths = group.divPoints, group.binormals, group.divWidths
  local numDivPoints = #divPoints
  table.clear(leftPts)
  table.clear(rightPts)
  for i = 1, numDivPoints do
    local p, bin = divPoints[i], binormals[i]
    local binX, binY, halfWidth = bin.x, bin.y, divWidths[i] * 0.5 + group.paintMargin
    leftPts[i]  = vec3(p.x - binX * halfWidth, p.y - binY * halfWidth, 0)
    rightPts[i] = vec3(p.x + binX * halfWidth, p.y + binY * halfWidth, 0)
  end

  -- Create the polygon.
  table.clear(polygon)
  local numLeftPts, numRightPts, ctr = #leftPts, #rightPts, 1
  for i = 1, numLeftPts do
    polygon[ctr] = leftPts[i]
    ctr = ctr + 1
  end
  for i = numRightPts, 1, -1 do
    polygon[ctr] = rightPts[i]
    ctr = ctr + 1
  end

  -- Get the scanline spans for the polygon.
  geom.getScanlineSpans(gMinY, gMaxY, te, tb, polygon, spansXMin, spansXMax, spansY)

  -- Paint the scanline spans.
  table.clear(setMask)
  local paintMaterialIdx = group.paintMaterialIdx or defaultMaterialIdx
  local capEstimate = #spansY * (gMaxX - gMinX + 1)
  local xVals, yVals, prevVals = table.new(capEstimate, 0), table.new(capEstimate, 0), table.new(capEstimate, 0)
  local width = gMaxX - gMinX + 1
  ctr = 1
  for i = 1, #spansY do
    local y = spansY[i]
    for x = spansXMin[i], spansXMax[i] do
      tmpPoint2I.x, tmpPoint2I.y = x, spansY[i]
      posWS:set(te:gridToWorldByPoint2I(tmpPoint2I, tb))
      local idx = (y - gMinY) * width + (x - gMinX) + 1 -- 2D -> 1D index used for the mask.
      if not setMask[idx] then -- Only write to a point once, to keep the history clean.
        prevVals[ctr] = tb:getMaterialIdxWs(posWS)
        tb:setMaterialIdxWs(posWS, paintMaterialIdx)
        xVals[ctr], yVals[ctr] = posWS.x, posWS.y
        ctr = ctr + 1
        setMask[idx] = true
      end
    end
  end

  -- Update the grid after changes.
  tmp1:set(gMinX, gMinY, 0)
  tmp2:set(gMaxX, gMaxY, 0)
  tb:updateGridMaterials(tmp1, tmp2)
  tb:updateGrid(tmp1, tmp2)

  -- Store the painted data in the spline, so it can be reverted later.
  group.paintedDataX = xVals
  group.paintedDataY = yVals
  group.paintedDataVals = prevVals
  group.paintedDataBoxXMin = gMinX
  group.paintedDataBoxXMax = gMaxX
  group.paintedDataBoxYMin = gMinY
  group.paintedDataBoxYMax = gMaxY
end

-- Reverts the terrain underneath the given spline to the state it had before it was painted.
local function revert(group)
  if not group.paintedDataVals or #group.paintedDataVals < 1 then
    return -- If no painted data, there is nothing to revert.
  end

  -- Restore the materials.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local stateX, stateY, stateVals = group.paintedDataX, group.paintedDataY, group.paintedDataVals
  for i = 1, #stateVals do
    tmp1:set(stateX[i], stateY[i], 0)
    tb:setMaterialIdxWs(tmp1, stateVals[i]) -- Restore the original material index (before painting).
  end

  -- Update the grid after changes.
  tmp1:set(group.paintedDataBoxXMin, group.paintedDataBoxYMin, 0)
  tmp2:set(group.paintedDataBoxXMax, group.paintedDataBoxYMax, 0)
  tb:updateGridMaterials(tmp1, tmp2)
  tb:updateGrid(tmp1, tmp2)

  -- Remove the painted data from the group.
  table.clear(group.paintedDataX)
  table.clear(group.paintedDataY)
  table.clear(group.paintedDataVals)
  group.paintedDataBoxXMin, group.paintedDataBoxXMax, group.paintedDataBoxYMin, group.paintedDataBoxYMax = 0, 0, 0, 0
end

-- Reverts all terrain painting for all groups.
local function revertAll(groups)
  for i = 1, #groups do
    revert(groups[i])
  end
end


-- Public interface.
M.paint =                                               paint

M.revert =                                              revert
M.revertAll =                                           revertAll

return M