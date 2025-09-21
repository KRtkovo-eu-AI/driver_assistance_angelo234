-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a general utility class contain various common functions used across various spline-editing tools.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local maxRayDist = 10000 -- The maximum distance for the camera -> mouse ray, in meters.
local defaultMinWidth, defaultMaxWidth = 10.0, 10.0 -- The default min and max widths for a spline.
local fixedWidthTolerance = 0.01 -- The tolerance for the width of a node to be considered fixed.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'toolUtilities'

-- Module constants.
local im = ui_imgui
local abs, min, max, floor = math.abs, math.min, math.max, math.floor
local sqrt, tan, huge = math.sqrt, math.tan, math.huge
local camLookDownRot = quatFromDir(vec3(0, 0, -1))

-- Module state.
local tmp1, tmp2 = vec3(), vec3()


-- Tests if mouse is hovering over the terrain (as opposed to any windows, etc).
local function isMouseHoveringOverTerrain()
  return not im.IsAnyItemHovered() and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not editor.isAxisGizmoHovered()
end

-- Computes the position on the map at which the mouse points.
local function mouseOnMapPos()
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir
  return rayPos + rayDir * castRayStatic(rayPos, rayDir, maxRayDist)
end

-- Conversion functions for velocities in meters per second.
local function msToMph(ms) return ms * 2.236936 end
local function msToKph(ms) return ms * 3.6 end

-- Computes a blue to red color by interpolating between blue [minValue] and red [maxValue].
local function getBlueToRedColour(value, minValue, maxValue)
  local t = max(0, min(1, (value - minValue) / (maxValue - minValue))) -- Clamp and normalise.
  return color(floor(t * 255 + 0.5), 0, floor((1.0 - t) * 255 + 0.5), 255)
end

-- Generates a unique name, so it will not clash with any existing names in the scene tree.
local function generateUniqueName(baseName, prefixIn)
  local prefix = prefixIn .. " - "
  local name = baseName
  local fullName = prefix .. name
  local i = 1
  while scenetree.findObject(fullName) do
    name = baseName .. " - " .. i
    fullName = prefix .. name
    i = i + 1
  end
  return name
end

-- Computes a map of spline IDs to their indices.
local function computeIdToIdxMap(splines, map)
  table.clear(map)
  for i = 1, #splines do
    map[splines[i].id] = i
  end
end

-- Returns the number of materials in the terrain block.
local function getNumMaterials()
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local mtls = tb:getMaterials()
  return #mtls
end

-- Returns the minimum and maximum widths of the given spline.
local function getMinMaxWidth(spline)
  local widths = spline.widths
  if not widths or #widths < 1 then
    return defaultMinWidth, defaultMaxWidth -- If no nodes in group spline, return the default min and max widths.
  end
  local wMin, wMax = huge, -huge
  for i = 1, #widths do
    local w = widths[i]
    wMin, wMax = min(wMin, w), max(wMax, w)
  end
  return wMin, wMax
end

-- Returns true if the width of the nodes is fixed, to some tolerance.
local function isWidthFixed(nodes)
  local wMin, wMax = huge, -huge
  for _, node in ipairs(nodes) do
    local width = node.width or 0
    wMin, wMax = min(wMin, width), max(wMax, width)
  end
  return abs(wMax - wMin) < fixedWidthTolerance, wMin
end

-- Removes consecutive points that are closer than minDist in XY-plane.
local function filterClosePointsXY(points, minDist)
  if #points < 2 then
    return points -- No points to filter.
  end

  -- Create a new table to store the filtered points.
  local numPoints = #points
  local filtered, ctr = table.new(numPoints, 0), 2
  filtered[1] = points[1]

  -- Iterate through the points, and filter out consecutive points that are closer than minDist in XY-plane.
  local minDistSq = minDist * minDist
  for i = 2, numPoints do
    local prev, curr = filtered[#filtered], points[i]
    local dx, dy = curr.x - prev.x, curr.y - prev.y
    if (dx * dx + dy * dy) >= minDistSq then
      filtered[ctr] = curr
      ctr = ctr + 1
    end
  end

  return filtered
end

-- Estimates average XY-plane spacing between nodes, simulating Catmull-Rom interpolation but excluding mesh extents.
local function calculateAverageSpacingXY(positions, minSpacing)
  if #positions < 2 then
    return minSpacing or 1.0
  end

  -- Simulate Catmull-Rom interpolation to estimate the path length.
  local totalLength = 0
  for i = 1, #positions - 1 do
    local p0 = positions[max(1, i - 1)]
    local p1 = positions[i]
    local p2 = positions[i + 1]
    local p3 = positions[min(#positions, i + 2)]

    -- Sample intermediate points for more accurate length.
    local lastPt = p1
    for t = 0.1, 1.0, 0.1 do
      local pt = catmullRomCentripetal(p0, p1, p2, p3, t, 0.5)
      local dx, dy = pt.x - lastPt.x, pt.y - lastPt.y  -- XY-plane only.
      totalLength = totalLength + sqrt(dx * dx + dy * dy)
      lastPt = pt
    end
  end

  -- Estimate average spacing between nodes based on total path length.
  local avgSpacing = totalLength / (#positions - 1)
  if minSpacing then
    avgSpacing = max(avgSpacing, minSpacing)
  end
  return avgSpacing
end

-- Returns true/idx if the path contains the given node key, otherwise false/nil.
local function doesPathContainNode(path, nodeKey)
  for i = 1, #path do
    if path[i] == nodeKey then
      return true, i
    end
  end
  return false, nil
end

-- Moves the camera directly above the spline with the given index.
local function goToSpline(points)
  -- Compute the 2D AABB of the spline, and the max height.
  local xMin, xMax, yMin, yMax, zMax = huge, -huge, huge, -huge, -huge
  for i = 1, #points do
    local p = points[i]
    local px, py = p.x, p.y
    xMin, xMax = min(xMin, px), max(xMax, px)
    yMin, yMax = min(yMin, py), max(yMax, py)
    zMax = max(zMax, p.z)
  end

  -- If the spline is too small (eg one node), do nothing and leave immediately.
  if max(abs(xMax - xMin), abs(yMax - yMin)) < 0.1 then
    return
  end

  -- Compute the mid-point of the 2D AABB.
  local midX, midY = (xMin + xMax) * 0.5, (yMin + yMax) * 0.5
  tmp1:set(midX, midY, 0.0)
  tmp2:set(xMax, yMax, 0.0)

  -- Determine the required distance at which the camera should be positioned.
  local groundDist = tmp1:distance(tmp2) -- The largest distance from the center of the AABB to the outside.
  local halfFov = core_camera.getFovRad() * 0.5 -- Half the camera field-of-view (in radians).
  local height = groundDist / tan(halfFov) + zMax + 5.0 -- The height at which the camera should be positioned, to fit the spline in view.

  -- Move the camera to the appropriate pose.
  commands.setFreeCamera()
  core_camera.setPosRot(0, midX, midY, height, camLookDownRot.x, camLookDownRot.y, camLookDownRot.z, camLookDownRot.w)
end

-- Converts HSV to RGB (all in range [0,1]).
local function hsvToRgb(h, s, v)
  local r, g, b
  local i = floor(h * 6)
  local f = h * 6 - i
  local p = v * (1 - s)
  local q = v * (1 - f * s)
  local t = v * (1 - (1 - f) * s)
  i = i % 6

  if i == 0 then r, g, b = v, t, p
  elseif i == 1 then r, g, b = q, v, p
  elseif i == 2 then r, g, b = p, v, t
  elseif i == 3 then r, g, b = p, q, v
  elseif i == 4 then r, g, b = t, p, v
  elseif i == 5 then r, g, b = v, p, q end

  return r, g, b
end

-- Flip the bitmap vertically, in-place.
local function flipBitmapY(bmpIn)
  local width, height = bmpIn:getWidth(), bmpIn:getHeight()
  local bmp = GBitmap()
  bmp:allocateBitmap(width, height, false, "GFXFormatR16")
  local heightMinus1 = height - 1
  for x = 0, width - 1 do
    for y = 0, heightMinus1 do
      local val = bmpIn:getTexel(x, heightMinus1 - y)
      bmp:setTexel(x, y, val, val, val, val)
    end
  end
  return bmp
end

-- Writes a binary mask to a .png file in RGBA format. Debug utility.
local function writeMaskToPng(mask, path)
  local height = #mask
  if height == 0 then return end
  local width = #mask[1]

  local bmp = GBitmap()
  bmp:allocateBitmap(width, height, false, "GFXFormatR16") -- 16-bit greyscale.

  for y = 1, height do
    for x = 1, width do
      local v = mask[y][x] == 1 and 65535 or 0
      bmp:setTexel(x - 1, y - 1, v, v, v, 65535) -- only white if mask is 1
    end
  end

  if bmp:saveFile(path) then
    log('I', logTag, 'Wrote RGBA mask PNG to: ' .. tostring(path))
  else
    log('E', logTag, 'Failed to write mask PNG to: ' .. tostring(path))
  end
end

-- Writes a set of vectorized paths to a .png file in 16-bit greyscale format.
-- Each path gets a distinct greyscale intensity for visual differentiation.
local function writePathsToPng(paths, width, height, path)
  local bmp = GBitmap()
  bmp:allocateBitmap(width, height, false, "GFXFormatR16") -- 16-bit grayscale

  -- Clear to black
  for y = 0, height - 1 do
    for x = 0, width - 1 do
      bmp:setTexel(x, y, 0, 0, 0, 65535)
    end
  end

  -- Use repeating high-contrast intensities for each path
  local levels = { 10000, 20000, 30000, 40000, 50000, 60000 }
  local nLevels = #levels

  for i, path in ipairs(paths) do
    local intensity = levels[(i - 1) % nLevels + 1] -- Cycle if too many

    for j = 1, #path do
      local pt = path[j]
      local x, y = floor(pt.x + 0.5), floor(pt.y + 0.5)
      if x >= 0 and x < width and y >= 0 and y < height then
        bmp:setTexel(x, y, intensity, intensity, intensity, 65535)
      end
    end
  end

  if bmp:saveFile(path) then
    log('I', logTag, 'writePathsToPng(): wrote to ' .. tostring(path))
  else
    log('E', logTag, 'writePathsToPng(): failed to save image to ' .. tostring(path))
  end
end

-- Updated: writeWidthsToPng using continuous thick strokes.
local function writeWidthsToPng(paths, widths, width, height, outPath)
  local bmp = GBitmap()
  bmp:allocateBitmap(width, height, false, "GFXFormatR16")

  -- Clear to black
  for y = 0, height - 1 do
    for x = 0, width - 1 do
      bmp:setTexel(x, y, 0, 0, 0, 65535)
    end
  end

  -- Draw each path using filled circles based on computed width
  for i, path in ipairs(paths) do
    local wPath = widths[i]
    for j = 1, #path do
      local p = path[j]
      local w = wPath[j] or 1
      local radius = max(1, floor(w * 0.5))

      -- Draw a filled circle centered at (p.x, p.y)
      local cx = floor(p.x + 0.5)
      local cy = floor(p.y + 0.5)
      for y = cy - radius, cy + radius do
        for x = cx - radius, cx + radius do
          if x >= 0 and x < width and y >= 0 and y < height then
            local dx, dy = x - cx, y - cy
            if dx * dx + dy * dy <= radius * radius then
              bmp:setTexel(x, y, 65535, 65535, 65535, 65535)
            end
          end
        end
      end
    end
  end

  -- Save
  if bmp:saveFile(outPath) then
    log('I', logTag, 'writeWidthsToPng(): wrote to ' .. tostring(outPath))
  else
    log('E', logTag, 'writeWidthsToPng(): failed to save image to ' .. tostring(outPath))
  end
end


-- Public interface.
M.isMouseHoveringOverTerrain =                          isMouseHoveringOverTerrain
M.mouseOnMapPos =                                       mouseOnMapPos

M.msToMph =                                             msToMph
M.msToKph =                                             msToKph

M.getBlueToRedColour =                                  getBlueToRedColour
M.generateUniqueName =                                  generateUniqueName
M.computeIdToIdxMap =                                   computeIdToIdxMap
M.getNumMaterials =                                     getNumMaterials
M.getMinMaxWidth =                                      getMinMaxWidth
M.isWidthFixed =                                        isWidthFixed
M.filterClosePointsXY =                                 filterClosePointsXY
M.calculateAverageSpacingXY =                           calculateAverageSpacingXY
M.doesPathContainNode =                                 doesPathContainNode

M.goToSpline =                                          goToSpline

M.hsvToRgb =                                            hsvToRgb
M.flipBitmapY =                                         flipBitmapY
M.writeMaskToPng =                                      writeMaskToPng
M.writePathsToPng =                                     writePathsToPng
M.writeWidthsToPng =                                    writeWidthsToPng

return M