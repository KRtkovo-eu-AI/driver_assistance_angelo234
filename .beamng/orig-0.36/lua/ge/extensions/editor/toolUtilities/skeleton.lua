-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility suite of functions for skeletonising and vectorising binary masks.
-- The main function getPathsFromPng(), converts a .png file to a skeletonised mask and then vectorises it into a series of polylines.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local defaultThreshold = 0.5 -- The default threshold for the bitmap to mask conversion.
local rdpTolerance = 6.0 -- The tolerance used for the RDP algorithm.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'skeleton'

-- Module dependencies.
local rdp = require('editor/toolUtilities/rdp')
local util = require('editor/toolUtilities/util')

-- Module constants.
local abs, min, floor, sqrt, huge = math.abs, math.min, math.floor, math.sqrt, math.huge

-- Module state.
local neighbours, visited = {}, {}
local toDeleteX, toDeleteY = {}, {}
local stackX, stackY = {}, {}
local outX, outY = {}, {}
local tangents = {}


-- Determines if a pixel is black.
local function isBlack(y, x, mask) return mask[y] and mask[y][x] == 1 end

-- Counts the number of black neighbours of a pixel.
local function countBlackNeighbours(y, x, mask)
  local count = 0
  for dy = -1, 1 do
    local yPlusDy = y + dy
    for dx = -1, 1 do
      if not (dx == 0 and dy == 0) and isBlack(yPlusDy, x + dx, mask) then
        count = count + 1
      end
    end
  end
  return count
end

-- Gets unvisited black neighbours of a pixel.
-- [Returns data via the outX, outY arrays.]
local function getUnvisitedNeighbours(y, x, mask, width)
  table.clear(outX)
  table.clear(outY)
  local ctr = 1
  for dy = -1, 1 do
    local ny = y + dy
    local nyTimesWidth = ny * width
    for dx = -1, 1 do
      if not (dx == 0 and dy == 0) then
        local nx = x + dx
        if isBlack(ny, nx, mask) and not visited[nyTimesWidth + nx] then
          outX[ctr], outY[ctr] = nx, ny
          ctr = ctr + 1
        end
      end
    end
  end
end

-- Counts the number of transitions in the neighbourhood of a pixel.
local function countTransitions(y, x, mask)
  -- Examine the neighbourhood of the pixel.
  neighbours[1] = isBlack(y - 1, x, mask) and 1 or 0 -- P2
  neighbours[2] = isBlack(y - 1, x + 1, mask) and 1 or 0 -- P3
  neighbours[3] = isBlack(y,     x + 1, mask) and 1 or 0 -- P4
  neighbours[4] = isBlack(y + 1, x + 1, mask) and 1 or 0 -- P5
  neighbours[5] = isBlack(y + 1, x, mask) and 1 or 0 -- P6
  neighbours[6] = isBlack(y + 1, x - 1, mask) and 1 or 0 -- P7
  neighbours[7] = isBlack(y,     x - 1, mask) and 1 or 0 -- P8
  neighbours[8] = isBlack(y - 1, x - 1, mask) and 1 or 0 -- P9

  -- Count the number of transitions in the neighbourhood.
  local transitions = 0
  for i = 1, #neighbours do
    local nCurr = neighbours[i]
    local nNext = neighbours[(i % 8) + 1]
    if nCurr == 0 and nNext == 1 then
      transitions = transitions + 1
    end
  end
  return transitions
end

-- Performs a single iteration of the Guo-Hall skeletonisation algorithm.
local function guoHallIteration(mask, phase)
  table.clear(toDeleteX)
  table.clear(toDeleteY)
  local w, h = #mask[1], #mask
  local ctr = 1

  for y = 2, h - 1 do
    for x = 2, w - 1 do
      if mask[y][x] == 1 then
        local p2 = mask[y - 1][x]
        local p3 = mask[y - 1][x + 1]
        local p4 = mask[y][x + 1]
        local p5 = mask[y + 1][x + 1]
        local p6 = mask[y + 1][x]
        local p7 = mask[y + 1][x - 1]
        local p8 = mask[y][x - 1]
        local p9 = mask[y - 1][x - 1]

        local A = 0
        if p2 == 0 and p3 == 1 then A = A + 1 end
        if p3 == 0 and p4 == 1 then A = A + 1 end
        if p4 == 0 and p5 == 1 then A = A + 1 end
        if p5 == 0 and p6 == 1 then A = A + 1 end
        if p6 == 0 and p7 == 1 then A = A + 1 end
        if p7 == 0 and p8 == 1 then A = A + 1 end
        if p8 == 0 and p9 == 1 then A = A + 1 end
        if p9 == 0 and p2 == 1 then A = A + 1 end

        local B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9

        if A == 1 and B >= 2 and B <= 6 then
          if phase == 0 then
            if p2 * p4 * p6 == 0 and p4 * p6 * p8 == 0 then
              toDeleteX[ctr] = x
              toDeleteY[ctr] = y
              ctr = ctr + 1
            end
          else
            if p2 * p4 * p8 == 0 and p2 * p6 * p8 == 0 then
              toDeleteX[ctr] = x
              toDeleteY[ctr] = y
              ctr = ctr + 1
            end
          end
        end
      end
    end
  end

  for i = 1, #toDeleteX do
    mask[toDeleteY[i]][toDeleteX[i]] = 0
  end

  return #toDeleteX > 0
end

-- Skeletonises a mask using the Guo-Hall algorithm.
-- [The given mask is processed in place, and returned.]
local function skeletonise(mask)
  local changed = true
  while changed do
    changed = guoHallIteration(mask, 0) or guoHallIteration(mask, 1)
  end
  return mask
end

-- Scans the skeleton mask to identify endpoints and junctions.
-- Returns a list of {x, y} tables and a map of idx → 'end' or 'junction'.
local function detectKeypoints(mask)
  local endpoints, junctions, classifications = {}, {}, {}
  local w, h = #mask[1], #mask

  for y = 2, h - 1 do
    for x = 2, w - 1 do
      if mask[y][x] == 1 then
        local n = countBlackNeighbours(y, x, mask)
        local t = countTransitions(y, x, mask)
        local idx = y * w + x

        if n == 1 then
          endpoints[#endpoints+1] = {x=x, y=y}
          classifications[idx] = "end"
        elseif t >= 3 and n >= 3 then
          junctions[#junctions+1] = {x=x, y=y}
          classifications[idx] = "junction"
        end
      end
    end
  end

  return endpoints, junctions, classifications
end

-- Converts 2D coordinates to a flat index.
local function flatIdx(x, y, width)
  return y * width + x
end

-- Checks if the coordinates are within the valid bounds of the mask.
local function isValidCoord(x, y, width, height)
  return x >= 1 and x <= width and y >= 1 and y <= height
end

-- Returns unvisited black (1-valued) 8-connected neighbors of a pixel.
local function getBlackUnvisitedNeighbours(x, y, mask, visited, width, height)
  local nbs = {}
  for dy = -1, 1 do
    for dx = -1, 1 do
      if not (dx == 0 and dy == 0) then
        local nx, ny = x + dx, y + dy
        if isValidCoord(nx, ny, width, height) and mask[ny][nx] == 1 and not visited[flatIdx(nx, ny, width)] then
          table.insert(nbs, {x = nx, y = ny})
        end
      end
    end
  end
  return nbs
end

-- Extracts all paths from endpoints and junctions on a 1-pixel-wide skeleton.
-- Each path starts and ends at a junction or endpoint.
local function extractPathsFromEndpointsAndJunctions(mask, endpoints, junctions, classifications)
  local width, height = #mask[1], #mask
  local visited = {}
  local paths = {}
  local walkedArms = {} -- key: srcFlatIdx, value: {dstFlatIdx=true, ...}

  -- Converts a pair of (x, y) to flat index.
  local function idx(x, y)
    return flatIdx(x, y, width)
  end

  -- Tracks arms walked from a given control point.
  local function markArmWalked(fromIdx, toIdx)
    if not walkedArms[fromIdx] then walkedArms[fromIdx] = {} end
    walkedArms[fromIdx][toIdx] = true
  end
  local function hasArmBeenWalked(fromIdx, toIdx)
    return walkedArms[fromIdx] and walkedArms[fromIdx][toIdx]
  end

  -- Gather all control points and potential isolated fragments
  local controlPoints = {}
  for y = 2, height - 1 do
    for x = 2, width - 1 do
      if mask[y][x] == 1 then
        local idx = flatIdx(x, y, width)
        if not visited[idx] then
          table.insert(controlPoints, { x = x, y = y })
        end
      end
    end
  end

  for _, pt in ipairs(controlPoints) do
    local x, y = pt.x, pt.y
    local fromIdx = idx(x, y)

    local nbs = getBlackUnvisitedNeighbours(x, y, mask, visited, width, height)
    for _, nb in ipairs(nbs) do
      local toIdx = idx(nb.x, nb.y)
      if not hasArmBeenWalked(fromIdx, toIdx) then
        -- Walk in this direction
        local localVisited = {} -- local visited for this trace
        local function walkArm(x0, y0)
          local path = {}
          local x, y = x0, y0
          while true do
            local i = idx(x, y)
            if visited[i] or localVisited[i] then break end

            table.insert(path, vec3(x, y))
            visited[i] = true
            localVisited[i] = true

            -- Stop if this is a control point and not the first pixel
            if #path > 1 and (classifications[i] == "junction" or classifications[i] == "end") then
              break
            end

            local nbs2 = getBlackUnvisitedNeighbours(x, y, mask, visited, width, height)
            if #nbs2 == 0 then break end
            x, y = nbs2[1].x, nbs2[1].y
          end
          return path
        end

        local forward = walkArm(nb.x, nb.y)

        -- Reverse walk toward control point
        local backPath = { vec3(x, y) } -- start at control point
        if #forward > 0 then
          -- Reverse prepend (back to control point, then rest of forward)
          for _, p in ipairs(forward) do table.insert(backPath, p) end
        end

        -- Register arm walked
        markArmWalked(fromIdx, toIdx)
        markArmWalked(toIdx, fromIdx)

        if #backPath > 1 then
          table.insert(paths, backPath)
        end
      end
    end
  end

  return paths
end

-- Traces a single path starting at (y, x).
local function tracePath(y, x, mask, width)
  local path, pathCtr = {}, 1
  table.clear(stackX)
  table.clear(stackY)
  stackX[1], stackY[1] = x, y
  while #stackX > 0 do
    local pX, pY = table.remove(stackX), table.remove(stackY) -- Pop the last (y, x) element from the stack.
    local idx = pY * width + pX
    if not visited[idx] then -- If the current pixel has not been visited, add it to the path and mark it as visited.
      path[pathCtr] = vec3(pX, pY) -- Add a deep copy of the current pixel to the path.
      pathCtr = pathCtr + 1
      visited[idx] = true -- Mark the current pixel as visited.
      getUnvisitedNeighbours(pY, pX, mask, width) -- Push all unvisited neighbours onto the stack.
      for i = 1, #outX do
        local stackIdx = #stackX + 1
        stackX[stackIdx], stackY[stackIdx] = outX[i], outY[i]
      end
    end
  end
  return path
end

-- Extracts all paths from given skeletonised mask.
local function extractPaths(mask)
  -- Clear the visited array.
  table.clear(visited)
  local w, h = #mask[1], #mask
  for y = 1, #mask do
    local yw = y * w
    for x = 1, w do
      visited[yw + x] = false
    end
  end

  -- Extract all paths from the mask.
  local paths, wMinus1, ctr = {}, w - 1, 1
  for y = 2, h - 1 do
    local yw = y * w
    for x = 2, wMinus1 do
      if isBlack(y, x, mask) and not visited[yw + x] then
        if countBlackNeighbours(y, x, mask) ~= 2 then -- Endpoint or junction.
          local path = tracePath(y, x, mask, w)
          if #path > 1 then
            paths[ctr] = path
            ctr = ctr + 1
          end
        end
      end
    end
  end

  return paths
end

-- Joins nearby paths.
local function joinClosePaths(paths, joinThreshold)
  local thresholdSq = joinThreshold * joinThreshold

  local function distSq(p1, p2)
    local dx, dy = p1.x - p2.x, p1.y - p2.y
    return dx * dx + dy * dy
  end

  local didMerge = true
  while didMerge do
    didMerge = false
    local i = 1
    while i <= #paths do
      local path1 = paths[i]
      local bestJ, bestCase, bestDistSq = nil, nil, huge

      for j = i + 1, #paths do
        local path2 = paths[j]

        local p1s, p1e = path1[1], path1[#path1]
        local p2s, p2e = path2[1], path2[#path2]

        local cases = {
          {distSq(p1e, p2s), "e-s"},
          {distSq(p1e, p2e), "e-e"},
          {distSq(p1s, p2e), "s-e"},
          {distSq(p1s, p2s), "s-s"}
        }

        for _, case in ipairs(cases) do
          local d, name = case[1], case[2]
          if d < bestDistSq and d < thresholdSq then
            bestJ, bestCase, bestDistSq = j, name, d
          end
        end
      end

      if bestJ then
        local path2 = paths[bestJ]

        -- Perform merge according to best case
        if bestCase == "e-s" then
          for k = 2, #path2 do table.insert(path1, path2[k]) end
        elseif bestCase == "e-e" then
          for k = #path2 - 1, 1, -1 do table.insert(path1, path2[k]) end
        elseif bestCase == "s-e" then
          local newPath = {}
          for k = 1, #path2 do table.insert(newPath, path2[k]) end
          for k = 2, #path1 do table.insert(newPath, path1[k]) end
          path1 = newPath
          paths[i] = path1
        elseif bestCase == "s-s" then
          local newPath = {}
          for k = #path2, 1, -1 do table.insert(newPath, path2[k]) end
          for k = 2, #path1 do table.insert(newPath, path1[k]) end
          path1 = newPath
          paths[i] = path1
        end

        table.remove(paths, bestJ)
        didMerge = true
      else
        i = i + 1
      end
    end
  end

  return paths
end

-- Filters out short paths by length.
local function filterShortPaths(paths, minLength)
  local filtered = {}
  for _, path in ipairs(paths) do
    local len = 0
    for i = 2, #path do
      local dx = path[i].x - path[i-1].x
      local dy = path[i].y - path[i-1].y
      len = len + sqrt(dx*dx + dy*dy)
    end
    if len >= minLength then
      table.insert(filtered, path)
    end
  end
  return filtered
end

-- Estimates the widths from the original .png file.
local function estimateWidths(paths, mask)
  -- Compute tangents
  table.clear(tangents)
  for i, path in ipairs(paths) do
    local t = {}
    for j = 1, #path do
      local p0 = path[math.max(1, j - 1)]
      local p1 = path[math.min(#path, j + 1)]
      local dx, dy = p1.x - p0.x, p1.y - p0.y
      local len = sqrt(dx * dx + dy * dy)
      t[j] = len > 0 and vec3(dx / len, dy / len) or vec3(0, 0)
    end
    tangents[i] = t
  end

  -- Estimate widths.
  local height, width = #mask, #mask[1]
  local outWidths = {}

  for i, path in ipairs(paths) do
    local widthsInner = {}
    local tPath = tangents[i]

    for j, p in ipairs(path) do
      local pX, pY = p.x, p.y
      local t = tPath[j]
      local nx, ny = -t.y, t.x -- binormal
      local maxRadius = 30
      local maxDeltaXY = 15

      local function walk(dx, dy)
        for d = 1, maxRadius do
          local ox = dx * (d + 0.5)
          local oy = dy * (d + 0.5)
          local x = floor(pX + ox + 0.5)
          local y = floor(pY + oy + 0.5)

          if x < 1 or x > width or y < 1 or y > height then return d - 1 end
          if mask[y][x] ~= 1 then return d - 1 end
          if abs(x - pX) > maxDeltaXY or abs(y - pY) > maxDeltaXY then return d - 1 end
        end
        return maxRadius
      end

      local w1 = walk(nx, ny)
      local w2 = walk(-nx, -ny)

      local combined = 0
      if w1 > 0 and w2 > 0 then
        combined = w1 + w2
      elseif w1 > 0 then
        combined = w1 * 2
      elseif w2 > 0 then
        combined = w2 * 2
      end

      widthsInner[j] = combined
    end

    outWidths[i] = widthsInner
  end

  return outWidths
end

-- Taper the widths at path endpoints to reduce 'ball' artifacts.
local function taperWidths(widths, taperLength)
  for _, wList in ipairs(widths) do
    local len = #wList
    for i = 1, taperLength do
      local headIdx = i
      local tailIdx = len - i + 1
      local factor = i / (taperLength + 1)

      if wList[headIdx] then
        wList[headIdx] = wList[headIdx] * factor
      end
      if wList[tailIdx] and tailIdx ~= headIdx then -- prevent double-multiplication
        wList[tailIdx] = wList[tailIdx] * factor
      end
    end
  end
end

local function smoothWidths(widths, radius)
  for _, wList in ipairs(widths) do
    local len = #wList
    local smoothed = {}
    for i = 1, len do
      local acc, count = 0, 0
      for j = i - radius, i + radius do
        if j >= 1 and j <= len then
          acc = acc + wList[j]
          count = count + 1
        end
      end
      smoothed[i] = acc / count
    end
    for i = 1, len do
      wList[i] = smoothed[i]
    end
  end
end

local function clampEndpointWidths(widths, maxWidth)
  for _, wList in ipairs(widths) do
    wList[1] = min(wList[1], maxWidth)
    wList[#wList] = min(wList[#wList], maxWidth)
  end
end

local function fillZeroWidths(widths)
  for _, wList in ipairs(widths) do
    local len = #wList
    for i = 1, len do
      if wList[i] == 0 then
        local sum, count = 0, 0
        for j = i - 2, i + 2 do
          if j >= 1 and j <= len and wList[j] > 0 then
            sum = sum + wList[j]
            count = count + 1
          end
        end
        if count > 0 then
          wList[i] = sum / count
        end
      end
    end
  end
end

-- Converts a .png to a mask.
-- [The mask is thresholded depending on the format of the .png.]
local function bitmapToMask(bmp, threshold)
  -- Assume image is 32-bit grayscale integer format.
  -- We dynamically normalize based on observed value range.
  local width, height = bmp:getWidth(), bmp:getHeight()
  local minVal, maxVal = huge, -huge

  -- First pass: find min and max values (used for normalization)
  for y = 0, height - 1 do
    for x = 0, width - 1 do
      local texel = bmp:getTexel(x, y)
      if texel < minVal then minVal = texel end
      if texel > maxVal then maxVal = texel end
    end
  end

  if maxVal == minVal then
    log('E', logTag, 'Bitmap appears to be uniform (no variation).')
    return nil
  end

  local rangeInv = 1.0 / (maxVal - minVal)
  threshold = threshold or defaultThreshold

  -- Second pass: create binary mask based on normalized value
  local mask = {}
  for y = 0, height - 1 do
    local row = {}
    for x = 0, width - 1 do
      local texel = bmp:getTexel(x, y)
      local normVal = (texel - minVal) * rangeInv
      row[x + 1] = normVal >= threshold and 1 or 0
    end
    mask[y + 1] = row
  end

  log('I', logTag, string.format("bitmapToMask() complete. Range: [%g, %g], threshold: %.3f", minVal, maxVal, threshold))
  return mask
end

-- Expands the black regions outward by a fixed radius.
local function dilateMask(mask, radius)
  local w, h = #mask[1], #mask
  local out = {}

  for y = 1, h do
    out[y] = {}
    for x = 1, w do
      local found = false
      for dy = -radius, radius do
        for dx = -radius, radius do
          local nx, ny = x + dx, y + dy
          if nx >= 1 and nx <= w and ny >= 1 and ny <= h and mask[ny][nx] == 1 then
            found = true
            break
          end
        end
        if found then break end
      end
      out[y][x] = found and 1 or 0
    end
  end

  return out
end

-- Converts a .png file to a skeletonised mask and then vectorises it into a series of polylines.
-- [filepath should direct to a .png file of a supported format.]
-- [Returns an array of paths, where each contains a table of 2D points as vec3() and a corresponding widths table.]
local function getPathsFromPng(filepath)
  -- Load the .png at the given path.
  local bmpRaw = GBitmap()
  if not bmpRaw:loadFile(filepath) then
    log('E', logTag, 'Failed to load image at path: ' .. filepath)
    return nil
  end

  -- Flip the bitmap vertically, to orient it correctly.
  local bmp = util.flipBitmapY(bmpRaw)

  -- Convert the .png to a mask.
  local rawMask = bitmapToMask(bmp)
  if not rawMask then
    log('E', logTag, 'Failed to convert .png to mask. Aborting .png to skeleton extraction.')
    return nil
  end

  local maskForWidths = deepcopy(rawMask)

  -- Dilate the mask to expand the black regions outward by a fixed radius.
  rawMask = dilateMask(rawMask, 3) -- Try 2 or 3 if width is still too narrow

  -- First, skeletonise the given mask.
  local skeletonMask = skeletonise(rawMask)
  --local height = #skeletonMask
  --local width = height > 0 and #skeletonMask[1] or 0
  --util.writeMaskToPng(skeletonMask, filepath .. "_skeleton.png") -- Debug.

  -- Detect the endpoints and junctions in the skeletonised mask.
  local endpoints, junctions, classifications = detectKeypoints(skeletonMask)

  -- Extract the paths from the skeletonised mask, and re-join path fragments.
  local paths = extractPathsFromEndpointsAndJunctions(skeletonMask, endpoints, junctions, classifications)
  paths = filterShortPaths(paths, 20)
  paths = joinClosePaths(paths, 40)
  --util.writePathsToPng(paths, width, height, filepath .. "_paths.png")

  -- Estimate the widths of the paths.
  local widths = estimateWidths(paths, maskForWidths)
  fillZeroWidths(widths)
  taperWidths(widths, 3) -- Taper ends to suppress ball artifacts
  smoothWidths(widths, 2) -- Try radius=2 first; increase for smoother, decrease for crisp
  clampEndpointWidths(widths, 15.0) -- adjust as needed
  --util.writeWidthsToPng(paths, widths, width, height, filepath .. "_widths.png")

  -- Simplify the paths using the RDP process.
  local cap = #paths
  local finalPaths = table.new(cap, 0)
  for i = 1, cap do
    local filteredPoints, filteredWidths = rdp.simplify(paths[i], widths[i], rdpTolerance)
    finalPaths[i] = { points = filteredPoints, widths = filteredWidths }
  end

  return finalPaths
end


-- Public interface.
M.skeletonise =                                         skeletonise
M.extractPaths =                                        extractPaths
M.estimateWidths =                                      estimateWidths

M.getPathsFromPng =                                     getPathsFromPng

return M