-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local decalRoadMaterial = "road_asphalt_2lane"                                                      -- The material to be used with decal road surfaces.
local folderName = "Terrain And Road Importer"                                                      -- The name of the folder which will appear in the scene tree, containing the roads.
local secSize = 1.0                                                                                 -- The longitudinal section size to be used when discretising the decal road spline.
local edgeSmoothingWidth = 2

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

local logTag = 'terrainAndRoadImporter'

-- External modules used.
local kdTreeB2d = require('kdtreebox2d')

-- Module constants.
local im = ui_imgui
local min, max, floor, ceil = math.min, math.max, math.floor, math.ceil
local abs, sqrt, exp, pi = math.abs, math.sqrt, math.exp, math.pi
local vert = vec3(0, 0, 1.0)
local zOffsetDecal = vert * 10
local convFac = 1.0 / 65535.0

local tmp1, tmp2, tmp3, tmp4 = vec3(0, 0), vec3(0, 0), vec3(0, 0), vec3(0, 0)

local toolWinName, toolWinSize = 'terrainAndRoadImporter', im.ImVec2(200, 200)                      -- The main tool window of the feature.
local isToolActive = false

local zMax = im.FloatPtr(400.0)
local DOI = im.IntPtr(150)
local margin = im.FloatPtr(7.0)


-- Function to calculate the distance between two points.
local function distance(x1, y1, x2, y2)
  return math.sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2)
end

-- Function to perform IDW interpolation.
local function idwInterpolate(grid, peaks, power)
  local width = #grid[1]
  local height = #grid

  for y = 1, height do
      for x = 1, width do
          local numerator = 0
          local denominator = 0

          for _, peak in ipairs(peaks) do
              local px, py, pz = peak.x, peak.y, peak.z
              local dist = distance(x, y, px, py)
              if dist == 0 then
                  -- If the point is exactly at the peak location
                  numerator = pz
                  denominator = 1
                  break
              end
              local weight = 1 / (dist ^ power)
              numerator = numerator + weight * pz
              denominator = denominator + weight
          end

          grid[y][x] = numerator / denominator
      end
  end
end

-- Conforms the local terrain to some given peaks
local function conformTerrainToPeaks(filepath)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  -- Get the peaks from the .json file.
  local gW = Point2I(0, 0)
  local jsonFull = jsonReadFile(filepath)
  local peaks = {}
  for i = 1, #jsonFull do
    te:worldToGridByPoint2I(vec3(jsonFull[i].x, jsonFull[i].y, jsonFull[i].z), gW, tb)
    peaks[i] = vec3(gW.x, gW.y, jsonFull[i].z)
  end

  -- Initialise the grid.
  local xSize, ySize = 2000, 2000
  local grid = {}
  for x = 0, xSize do
    local gridX = {}
    for y = 0, ySize do
      gridX[y] = 0.0
    end
    grid[x] = gridX
  end

  local power = 2
  idwInterpolate(grid, peaks, power)

  -- Terraform the heightmap from the processed mask.
  for x = 0, xSize do
    local gridX = grid[x]
    for y = 0, ySize do
        tb:setHeight(x, y, max(0, min(250.0, gridX[y])))
    end
  end

  -- Update the terrain block.
  tb:updateGrid(vec3(0, 0), vec3(xSize, ySize))
end

-- Creates a decal road.
local function createDecalRoad(nodes, widths, material, folder)
  local dRoad = createObject("DecalRoad")
  --dRoad:setField("improvedSpline", 0, "true")
  --dRoad:setField("overObjects", 0, "true")
  dRoad:setField("material", 0, material)
  dRoad:setField("drivability", 0, 1.0)
  dRoad:registerObject("")
  folder:addObject(dRoad)
  local decalId = dRoad:getID()
  for i = 1, #nodes do
    editor.addRoadNode(decalId, { pos = nodes[i] + zOffsetDecal, width = widths[i], index = i - 1 })
  end
end

local function intersectsUp_Triangle(rpos, ca, bc, c, norm, normSq)
  local rposc = rpos - c
  local pOnTri = rposc:dot(norm) / norm.z
  rposc.z = rposc.z - pOnTri
  local pacnorm = rposc:cross(norm)
  local bx, by = bc:dot(pacnorm), ca:dot(pacnorm)
  if min(bx, by) >= 0 and bx + by <= normSq then
    return -pOnTri
  end
  return false
end

-- Function to create a Gaussian kernel.
local function gaussianKernel(size, sigma)
  local kernel = {}
  local sum = 0
  local center = floor(size / 2)

  for i = 0, size - 1 do
    kernel[i] = {}
    for j = 0, size - 1 do
      local x = i - center
      local y = j - center
      kernel[i][j] = exp(-(x*x + y*y) / (2 * sigma * sigma)) / (2 * pi * sigma * sigma)
      sum = sum + kernel[i][j]
    end
  end

  -- Normalize the kernel.
  for i = 0, size - 1 do
    for j = 0, size - 1 do
      kernel[i][j] = kernel[i][j] / sum
    end
  end

  return kernel
end

-- Function to apply the Gaussian blur to a 2D table with specified range and a mask table.
local function gaussianBlur2D(inputTable, kernel, maskTable)
  local height = #inputTable
  local width = #inputTable[0]
  local kSize = #kernel
  local kCenter = floor(kSize / 2)
  local result = {}

  for i = 0, height do
    result[i] = {}
    for j = 0, width do
      if maskTable[i] and maskTable[i][j] > 0.5 then
        result[i][j] = inputTable[i][j]
      else
        local sum = 0
        local weightSum = 0
        for ki = 0, kSize - 1 do
          for kj = 0, kSize - 1 do
            local x = j + kj - kCenter
            local y = i + ki - kCenter
            if x >= 0 and x <= width and y >= 0 and y <= height then
              sum = sum + inputTable[y][x] * kernel[ki][kj]
              weightSum = weightSum + kernel[ki][kj]
            end
          end
        end
        if weightSum > 0 then
          result[i][j] = sum / weightSum
        else
          result[i][j] = sum
        end
      end
    end
  end

  return result
end

-- Averages the height with neighbouring points.
local function averageMask(height, mod, fixedMask, xSize, ySize)
  local xTop, yTop = xSize - 1, ySize - 1
  for x = 1, xTop do
    for y = 1, yTop do
      if mod[x][y] > 0.5 and fixedMask[x][y] < 0.5 then
        local sum, count = 0, 0
        for dx = -edgeSmoothingWidth, edgeSmoothingWidth do
          for dy = -edgeSmoothingWidth, edgeSmoothingWidth do
            local nx, ny = x + dx, y + dy
            if nx >= 0 and nx <= xSize and ny >= 0 and ny <= ySize then
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

-- Conforms the local terrain to the road.
local function conformTerrainToRoad(DOI, tris, trisBloated, box)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  local extents = tb:getObjectBox():getExtents()
  local tDX, tDY = extents.x, extents.y
  local xHalf, yHalf = tDX * 0.5, tDY * 0.5
  local tXMin, tXMax, tYMin, tYMax = -xHalf, xHalf, -yHalf, yHalf
  local zOff = tb:getTransform():getPosition().z

  -- Initialize the mask.
  local mask, height = {}, {}
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(max(tXMin, box.xMin - DOI)), floor(max(tYMin, box.yMin - DOI))), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(min(tXMax, box.xMax + DOI)), ceil(min(tYMax, box.yMax + DOI))), gMax, tb)
  local bXMin, bXMax, bYMin, bYMax = gMin.x, gMax.x, gMin.y, gMax.y
  local xSize, ySize = bXMax - bXMin, bYMax - bYMin
  for x = 0, xSize do
    local xPos = bXMin + x
    local innerM, innerH = {}, {}
    for y = 0, ySize do
      innerM[y], innerH[y] = 0, max(0, tb:getHeightGrid(xPos, bYMin + y))
    end
    mask[x], height[x] = innerM, innerH
  end

  -- Create and populate a kd-tree.
  local tree = kdTreeB2d.new(#trisBloated)
  for i = 1, #trisBloated do
    local t = trisBloated[i]
    local tA, tB, tC = t.a, t.b, t.c
    local xMin, xMax = min(tA.x, min(tB.x, tC.x)), max(tA.x, max(tB.x, tC.x))
    local yMin, yMax = min(tA.y, min(tB.y, tC.y)), max(tA.y, max(tB.y, tC.y))
    tree:preLoad(i, xMin, yMin, xMax, yMax)
  end
  tree:build()

  -- Iterate over the grid bounding box, and add contributions to the road mask.
  local pWS = vec3(0, 0)
  for x = 0, xSize do
    local maskX, heightX = mask[x], height[x]
    local gX = bXMin + x
    for y = 0, ySize do
      local gY = bYMin + y
      local pWS_3F = te:gridToWorldByPoint2I(Point2I(gX, gY), tb)
      pWS:set(pWS_3F.x, pWS_3F.y, 0)

      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local tri = trisBloated[tIdx]
        local tA, tB, tC = tri.a, tri.b, tri.c
        local tCA = tC - tA
        local tBC = tB - tC
        local triNorm = tCA:cross(tBC)
        local sqTriNorm = triNorm:squaredLength()
        local z = intersectsUp_Triangle(pWS, tCA, tBC, tC, triNorm, sqTriNorm)
        if z then
          heightX[y] = (maskX[y] == 0 and z - zOff) or min(z - zOff, heightX[y])
          maskX[y] = 1
        end
      end
    end
  end

  -- Create and populate a kd-tree for the non-bloated (thin) road mesh.
  local tree = kdTreeB2d.new(#tris)
  for i = 1, #tris do
    local t = tris[i]
    local tA, tB, tC = t.a, t.b, t.c
    local xMin, xMax = min(tA.x, min(tB.x, tC.x)), max(tA.x, max(tB.x, tC.x))
    local yMin, yMax = min(tA.y, min(tB.y, tC.y)), max(tA.y, max(tB.y, tC.y))
    tree:preLoad(i, xMin, yMin, xMax, yMax)
  end
  tree:build()

  -- Iterate over the grid bounding box, and add contributions to the road mask.
  local pWS = vec3(0, 0)
  local fixedMask = {}
  for x = 0, xSize do
    local fixedMaskX = {}
    local gX = bXMin + x
    for y = 0, ySize do
      fixedMaskX[y] = 0
      local gY = bYMin + y
      local pWS_3F = te:gridToWorldByPoint2I(Point2I(gX, gY), tb)
      pWS:set(pWS_3F.x, pWS_3F.y, 0)

      for tIdx in tree:queryNotNested(pWS.x, pWS.y, pWS.x, pWS.y) do
        local tri = tris[tIdx]
        local tA, tB, tC = tri.a, tri.b, tri.c
        local tCA = tC - tA
        local tBC = tB - tC
        local triNorm = tCA:cross(tBC)
        local sqTriNorm = triNorm:squaredLength()
        local z = intersectsUp_Triangle(pWS, tCA, tBC, tC, triNorm, sqTriNorm)
        if z then
          fixedMaskX[y] = 1
        end
      end
    end
    fixedMask[x] = fixedMaskX
  end

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

  -- Perform Gaussian blur on the mask, using the fixed mask.
  for _ = 1, 3 do
    height = gaussianBlur2D(height, gaussianKernel(7, 1.0), fixedMask)
  end

  -- Average the height with neighbouring points.
  averageMask(height, mod, fixedMask, xSize, ySize)

  -- Terraform the heightmap from the processed mask.
  for x = 0, xSize do
    local heightX, modX, rx = height[x], mod[x], x + bXMin
    for y = 0, ySize do
      if modX[y] > 0.5 then
        local ry = y + bYMin
        local z = heightX[y]
        tb:setHeight(rx, ry, max(0, z))
      end
    end
  end

  -- Update the terrain block.
  tb:updateGrid(vec3(bXMin, bYMin), vec3(bXMax, bYMax))
end

-- Triangulates a decal road.  The height is chosen along the road centerline.
local function triangulateAroundCenter(nodes, widths, DOI, margin, tXMin, tXMax, tYMin, tYMax, tris, trisBloated, box)
  local pDiv, wDiv, ctr = {}, {}, 1
  local secSizeInv = 1.0 / secSize
  local numNodes = #nodes

  -- Discretise the decal road spline.
  for i = 2, numNodes do
    local i1, i2, i3, i4 = max(1, i - 2), i - 1, i, min(numNodes, i + 1)
    local p1, p2, p3, p4 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
    local dx = p2:distance(p3)
    local numDivs = max(3, ceil(dx * secSizeInv))
    local numDivsInv = 1.0 / numDivs
    local startIdx = 1
    if i == 2 then
      startIdx = 0
    end
    for j = startIdx, numDivs do
      local q = j * numDivsInv
      pDiv[ctr] = catmullRom(p1, p2, p3, p4, q)
      tmp1:set(p1.x, p1.y, widths[i1] * 0.5)
      tmp2:set(p2.x, p2.y, widths[i2] * 0.5)
      tmp3:set(p3.x, p3.y, widths[i3] * 0.5)
      tmp4:set(p4.x, p4.y, widths[i4] * 0.5)
      wDiv[ctr] = catmullRom(tmp1, tmp2, tmp3, tmp4, q).z
      ctr = ctr + 1
    end
  end

  -- Compute the corresponding road edge points.
  local left, right, leftB, rightB = {}, {}, {}, {}
  local numDivs = #pDiv
  for i = 1, numDivs do
    local p1, p2 = pDiv[max(1, i - 1)], pDiv[min(numDivs, i + 1)]
    local tgt = p2 - p1
    tgt:normalize()
    local lat = vert:cross(tgt)
    local dw = wDiv[i]
    local vLat, vLatB = lat * dw, lat * (dw + margin)
    left[i], right[i] = pDiv[i] + vLat, pDiv[i] - vLat
    leftB[i], rightB[i] = pDiv[i] + vLatB, pDiv[i] - vLatB
  end

  -- Create the two triangle sets and axis-aligned bounding box.
  local ctr = #tris + 1
  for i = 2, numDivs do
    local iM = i - 1
    local b1, b2, b3, f1, f2, f3 = left[iM], pDiv[iM], right[iM], left[i], pDiv[i], right[i]
    local b1B, b3B, f1B, f3B = leftB[iM], rightB[iM], leftB[i], rightB[i]
    tris[ctr] = { a = b1, b = b2, c = f1 }
    tris[ctr + 1] = { a = f2, b = b2, c = f1 }
    tris[ctr + 2] = { a = b2, b = b3, c = f2 }
    tris[ctr + 3] = { a = f2, b = b3, c = f3 }
    trisBloated[ctr] = { a = b1B, b = b2, c = f1B }
    trisBloated[ctr + 1] = { a = f2, b = b2, c = f1B }
    trisBloated[ctr + 2] = { a = b2, b = b3B, c = f2 }
    trisBloated[ctr + 3] = { a = f2, b = b3B, c = f3B }
    ctr = ctr + 4
  end

  -- Compute the AABB.
  local xMin, xMax, yMin, yMax = 1e99, -1e99, 1e99, -1e99
  for i = 1, #trisBloated do
    local t = trisBloated[i]
    local a, b, c = t.a, t.b, t.c
    xMin = min(xMin, a.x)
    xMin = min(xMin, b.x)
    xMin = min(xMin, c.x)
    xMax = max(xMax, a.x)
    xMax = max(xMax, b.x)
    xMax = max(xMax, c.x)
    yMin = min(yMin, a.y)
    yMin = min(yMin, b.y)
    yMin = min(yMin, c.y)
    yMax = max(yMax, a.y)
    yMax = max(yMax, b.y)
    yMax = max(yMax, c.y)
  end
  local newBox = {
    xMin = max(tXMin, xMin - DOI), xMax = min(tXMax, xMax + DOI),
    yMin = max(tYMin, yMin - DOI), yMax = min(tYMax, yMax + DOI) }
  box.xMin = min(box.xMin, newBox.xMin)
  box.xMax = max(box.xMax, newBox.xMax)
  box.yMin = min(box.yMin, newBox.yMin)
  box.yMax = max(box.yMax, newBox.yMax)
end

-- Imports a terrain from the given path.
local function importTerrain(terrPath, zMax)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  -- Fetch the extents of the terrain block.
  local extents = tb:getObjectBox():getExtents()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = -xHalf, xHalf, -yHalf, yHalf
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(tXMin), floor(tYMin)), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(tXMax), ceil(tYMax)), gMax, tb)

  -- Load the terrain.
  local bmp = GBitmap()
  if not bmp:loadFile(terrPath) then
    log('E', logTag, 'Failed to load terrain (.png) file.')
  end

  -- Apply the bitmap to the heightmap.
  local gMinX, gMaxX, gMinY, gMaxY = gMin.x, gMax.x, gMin.y, gMax.y
  local fac = convFac * zMax
  for x = gMinX, gMaxX do
    local xBmp = x - gMinX
    if xBmp <= 2000 then
      for y = gMinY, gMaxY do
        local yBmp = y - gMinY
        if yBmp <= 2000 then
          tb:setHeight(x, y, max(0, bmp:getTexel(xBmp, yBmp) * fac))
        end
      end
    end
  end

  -- Update the terrain block.
  tb:updateGrid(vec3(gMin.x, gMin.y), vec3(gMax.x, gMax.y))

  -- Now that the meshes exist in the scene, re-compute the collision mesh to take them into account.
  be:reloadCollision()
end

-- Imports a set of roads from the given paths.
local function importRoads(roadPath, DOI, margin)

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  -- Fetch the extents of the terrain block.
  local extents = tb:getObjectBox():getExtents()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = -xHalf, xHalf, -yHalf, yHalf
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(tXMin), floor(tYMin)), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(tXMax), ceil(tYMax)), gMax, tb)

  -- Read the roads file.
  local jsonFull = jsonReadFile(roadPath)

  -- Import the roads.
  local sceneTreeFolder = createObject("SimGroup")
  sceneTreeFolder:registerObject(folderName)
  scenetree.MissionGroup:addObject(sceneTreeFolder)
  local numRoads = #jsonFull
  local n, w, tris, trisBloated, box = {}, {}, {}, {}, { xMin = 1e99, xMax = -1e99, yMin = 1e99, yMax = -1e99 }
  for i = 1, numRoads do
    local nodes, widths = {}, {}
    local json = jsonFull[i]
    for j = 1, #json do
      local d = json[j]
      tmp1:set(d.x, d.y, 0)
      nodes[j] = vec3(d.x, d.y, core_terrain.getTerrainHeight(tmp1))
      widths[j] = d.width
    end
    triangulateAroundCenter(nodes, widths, DOI, margin, tXMin, tXMax, tYMin, tYMax, tris, trisBloated, box)
    n[i], w[i] = nodes, widths
  end

  conformTerrainToRoad(DOI, tris, trisBloated, box)
  for i = 1, numRoads do
    createDecalRoad(n[i], w[i], decalRoadMaterial, sceneTreeFolder)
  end

  -- Reload the navgraph.
  map.reset()
end

-- Removes all decal roads and resets the heightmap/terrain (sets it flat everywhere ie z=0).
local function reset()

  -- Remove any folders.
  local folder = scenetree.findObject(folderName)
  if folder then
    folder:delete()
  end

  -- If there is no terrain block (eg smallgrid) then leave immediately.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  if not tb or not te then
    return
  end

  -- Fetch the extents of the terrain block.
  local extents = tb:getObjectBox():getExtents()
  local xHalf, yHalf = extents.x * 0.5, extents.y * 0.5
  local tXMin, tXMax, tYMin, tYMax = -xHalf, xHalf, -yHalf, yHalf
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  te:worldToGridByPoint2I(vec3(floor(tXMin), floor(tYMin)), gMin, tb)
  te:worldToGridByPoint2I(vec3(ceil(tXMax), ceil(tYMax)), gMax, tb)

  -- Set the heightmap to zero everywhere.
  local gMinX, gMaxX, gMinY, gMaxY = gMin.x, gMax.x, gMin.y, gMax.y
  for x = gMinX, gMaxX do
    for y = gMinY, gMaxY do
      tb:setHeight(x, y, 0.0)
    end
  end

  -- Update the terrain block.
  tb:updateGrid(vec3(gMin.x, gMin.y), vec3(gMax.x, gMax.y))
end

-- Opens/closes the world editor.
local function toggleWorldEditor(isOpen) editor.setEditorActive(isOpen) end

-- World editor main callback for rendering the UI.
local function onEditorGui()

  if not isToolActive then
    return
  end

  if editor.beginWindow(toolWinName, "Terrain And Road Importer###1") then

    -- 'Conform To Peaks' button.
    if editor.uiIconImageButton(editor.icons.lineToTerrain, im.ImVec2(34, 34), nil, nil, nil, 'conformToPeaksBtn') then
      extensions.editor_fileDialog.openFile(
        function(data)
          conformTerrainToPeaks(data.filepath)
        end,
        {{"JSON",".json"}},
        false,
        "/")
    end
    im.tooltip('Terraforms the terrain to some given peaks (from a .json file).')
    im.SameLine()

    -- 'Import Terrain' button.
    if editor.uiIconImageButton(editor.icons.simobject_terrainblock, im.ImVec2(34, 34), nil, nil, nil, 'importTerrainBtn') then
      extensions.editor_fileDialog.openFile(
        function(data)
          importTerrain(data.filepath, zMax[0])
        end,
        {{"PNG",".png"}},
        false,
        "/")
    end
    im.tooltip('Import a terrain (from 16-bit greyscale .png file).')
    im.SameLine()

    -- 'Import Roads' button.
    if editor.uiIconImageButton(editor.icons.autobahn, im.ImVec2(34, 34), nil, nil, nil, 'importRoadsBtn') then
      extensions.editor_fileDialog.openFile(
        function(data)
          importRoads(data.filepath, DOI[0], margin[0])
        end,
        {{"JSON",".json"}},
        false,
        "/")
    end
    im.tooltip('Import roads (from .json file).')
    im.SameLine()

    -- 'Reset' button.
    if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(34, 34), nil, nil, nil, 'resetBtn') then
      reset()
    end
    im.tooltip('Reset (remove imported terrain and all imported roads).')

    -- 'zMax' slider.
    im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
    im.PushItemWidth(200)
    im.SliderFloat("###2", zMax, 0.0, 500.0, "zMax (m) = %.3f")
    im.PopItemWidth()
    im.PopStyleVar()
    im.tooltip('Set the terrain prominence - ie terrain will map to [0, zMax].')

    -- 'Domain Of Influence' slider.
    im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
    im.PushItemWidth(200)
    im.SliderInt("###3", DOI, 1, 500, "Domain Of Influence (m) %d")
    im.PopItemWidth()
    im.PopStyleVar()
    im.tooltip('Set the domain of influence of the terraforming.')

    -- 'Margin' slider.
    im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
    im.PushItemWidth(200)
    im.SliderFloat("###4", margin, 0.0, 20.0, "Margin (m) = %.3f")
    im.PopItemWidth()
    im.PopStyleVar()
    im.tooltip('Set the terraforming margin (around roads).')
  end
end

-- Called when the 'Terrain And Roads Importer' feature icon is pressed.
local function onActivate()
  editor.clearObjectSelection()
  editor.showWindow(toolWinName)
  isToolActive = true
end

-- Called when the 'Terrain And Roads Importer' feature is exited.
local function onDeactivate()
  editor.hideWindow(toolWinName)
  isToolActive = false
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  if tech_license.isValid() then
    editor.editModes.terrainAndRoadImporterEditMode = {
      displayName = "Terrain And Road Importer",
      onUpdate = nop,
      onActivate = onActivate,
      onDeactivate = onDeactivate,
      icon = editor.icons.terrainToLine,
      iconTooltip = "Terrain And Road Importer",
      auxShortcuts = {},
      hideObjectIcons = true }

    editor.registerWindow(toolWinName, toolWinSize)
  end
end


-- Public interface.
M.conformTerrainToPeaks =                                 conformTerrainToPeaks
M.importTerrain =                                         importTerrain
M.importRoads =                                           importRoads
M.reset =                                                 reset
M.toggleWorldEditor =                                     toggleWorldEditor

M.onEditorGui =                                           onEditorGui
M.onEditorInitialized =                                   onEditorInitialized

return M