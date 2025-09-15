-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.load('util_maptiles')

local M = {}

local im = ui_imgui
local imUtils = require('ui/imguiUtils')

local windowOpen = im.BoolPtr(false)
local show3DWorld = im.BoolPtr(false)
local showDebugPanel = im.BoolPtr(false)
local runGenerator = im.BoolPtr(true)
local draw3DDebug = im.BoolPtr(false)
local initialWindowSize = im.ImVec2(500, 500)

local renderView

-- Variables for tile generation
local baseTileSize = 1000 -- Base tile size in world units
local basePixelSize = 512 -- Base pixel size for tiles
local maxZoomLevel = 5 -- Maximum zoom level to generate
local currentZoomLevel = 0
local tilePositions = {}
local currentTileIndex = 1
local isGeneratingTiles = false
local outputDirectory = "tiles" -- Directory to save tiles
local metadata = {
  baseTileSize = baseTileSize,
  basePixelSize = basePixelSize,
  maxZoomLevel = maxZoomLevel,
  terrainSizeUnits = 0, -- Will be calculated
  tileGrid = {} -- Will store tile counts per zoom level
}
local currentLevelName

-- Variables for frame delay
local framesPerTile = 2 -- Number of frames to wait per tile
local frameCounter = 0

-- Variables to save original rendering settings
local originalSettings = {}

-- Variables for grid preview and ETA
local startTime = 0
local totalTilesX = 0
local totalTilesY = 0

-- For selecting zoom level in debug panel
local selectedZoomLevel = im.IntPtr(currentZoomLevel)

-- Function to format time in hh:mm:ss
local function formatTime(seconds)
  local hours = math.floor(seconds / 3600)
  seconds = seconds % 3600
  local minutes = math.floor(seconds / 60)
  seconds = seconds % 60
  if hours > 0 then
      return string.format("%dh %dm %ds", hours, minutes, math.floor(seconds))
  elseif minutes > 0 then
      return string.format("%dm %ds", minutes, math.floor(seconds))
  else
      return string.format("%ds", math.floor(seconds))
  end
end

-- Function to adjust rendering settings for high-quality rendering
local function adjustRenderingSettings()
  -- Save current settings
  originalSettings.detailAdjust = TorqueScriptLua.getVar("$pref::TS::detailAdjust")
  originalSettings.lodScale = TorqueScriptLua.getVar("$pref::Terrain::lodScale")
  originalSettings.groundCoverScale = getGroundCoverScale()

  local sunsky = scenetree.findObject("sunsky")
  if sunsky then
    originalSettings.sunskyTexSize = sunsky.texSize
    originalSettings.sunskyShadowDistance = sunsky.shadowDistance
    -- Adjust sunsky settings for better shadows
    sunsky.texSize = 64
    sunsky.shadowDistance = 0 -- no shadows here
    sunsky.castShadows = false
  end

  -- Set new rendering settings for high-quality rendering
  TorqueScriptLua.setVar("$pref::TS::detailAdjust", 20000)
  TorqueScriptLua.setVar("$pref::Terrain::lodScale", 0.00001)
  setGroundCoverScale(8)
  flushGroundCoverGrids()
end

-- Function to restore original rendering settings
local function restoreRenderingSettings()
  -- Restore saved settings
  TorqueScriptLua.setVar("$pref::TS::detailAdjust", originalSettings.detailAdjust)
  TorqueScriptLua.setVar("$pref::Terrain::lodScale", originalSettings.lodScale)
  setGroundCoverScale(originalSettings.groundCoverScale)

  local sunsky = scenetree.findObject("sunsky")
  if sunsky then
    sunsky.texSize = originalSettings.sunskyTexSize
    sunsky.shadowDistance = originalSettings.sunskyShadowDistance
    sunsky.castShadows = true
  end
end


local minX, minY, maxX, maxY
local function calcBounds()
  -- Get terrain info
  local tb = scenetree.findObject(scenetree.findClassObjects('TerrainBlock')[1])
  local terrainSize = tb:getWorldBlockSize()
  local terrainPosition = tb:getPosition()

  -- Calculate bounds
  minX = terrainPosition.x
  maxX = terrainPosition.x + terrainSize
  minY = terrainPosition.y
  maxY = terrainPosition.y + terrainSize
  dump(minX, maxX, minY, maxY)
  -- Calculate bounding box for all map nodes
  local m = map
  if m then
    for nid, n in pairs(m.getMap().nodes) do
      local pos = n.pos
      minX = math.min(minX, pos.x)
      minY = math.min(minY, pos.y)
      maxX = math.max(maxX, pos.x)
      maxY = math.max(maxY, pos.y)
    end
  end
  -- Add a margin for link widths
  local margin = 250
  minX = math.floor(minX - margin - baseTileSize)
  minY = math.floor(minY - margin - baseTileSize)
  maxX = math.ceil( maxX + margin)
  maxY = math.ceil( maxY + margin)
  dump(minX, maxX, minY, maxY)

  minX = math.ceil(math.abs(minX/baseTileSize)) * sign(minX) * baseTileSize
  minY = math.ceil(math.abs(minY/baseTileSize)) * sign(minY) * baseTileSize
  maxX = math.ceil(math.abs(maxX/baseTileSize)) * sign(maxX) * baseTileSize
  maxY = math.ceil(math.abs(maxY/baseTileSize)) * sign(maxY) * baseTileSize


 dump(minX, maxX, minY, maxY)
end

--use this function to map game coordinates to the leaflet map coordinates
local function gameCoordsToExportCoords(p)
  p = vec3(-p.y - minX, p.x - minY)
  local f = basePixelSize / (baseTileSize*2)
  return {p.x*f , p.y*f}
end

-- exports the navgraph into a file
local function exportNavgraph()
  local navgraph = {}
  for nid, n in pairs(map.getMap().nodes) do -- remember edges are now single sided
    local a = map.getMap().nodes[nid]
    for lid, link in pairs(n.links) do
      local b = map.getMap().nodes[lid]
      local t = (a.pos - b.pos):normalized()
      t = vec3(t.y, -t.x, 0)
      local h, i, j, k = a.pos + t*a.radius, b.pos + t*b.radius, b.pos - t*b.radius, a.pos - t*a.radius
      table.insert(navgraph, {
        points = {
          gameCoordsToExportCoords(h),
          gameCoordsToExportCoords(i),
          gameCoordsToExportCoords(j),
          gameCoordsToExportCoords(k)
        },
        drivability = link.drivability,
        color = link.drivability > 0.9 and "#ff8800" or (link.drivability > 0.25 and "#aa6600" or "#884400")
      })
    end
  end
  jsonWriteFile(outputDirectory.."/navgraph.json", {list = navgraph}, true)
end

-- Function to generate tile positions across the terrain for a given zoom level
local function generateTilePositions(zoomLevel)
  tilePositions = {}
  currentTileIndex = 1

  -- Adjust tileSize and pixelSize based on zoom level
  local tileSize = baseTileSize / (2 ^ zoomLevel)
  local pixelSize = basePixelSize



  -- Generate grid positions with integer indices
  totalTilesX = 0
  totalTilesY = 0
  local yIndex = 0
  for y = maxY-tileSize, minY, -tileSize do
    local xIndex = 0
    for x = minX, maxX-tileSize, tileSize do
      table.insert(tilePositions, {
        x = x + tileSize / 2, -- Center of the tile
        y = y + tileSize / 2, -- Center of the tile
        xIndex = xIndex,
        yIndex = yIndex,
        status = 'pending',
        tileSize = tileSize,
        pixelSize = pixelSize,
        topLeft = vec3(x, y+tileSize),
      })
      xIndex = xIndex + 1
    end
    totalTilesX = xIndex -- The last xIndex
    yIndex = yIndex + 1
  end
  totalTilesY = yIndex

  -- Update metadata for the current zoom level
  metadata.tileGrid[zoomLevel] = {
    tileSizeUnits = tileSize,
    pixelSize = pixelSize,
    tilesX = totalTilesX,
    tilesY = totalTilesY
  }

  -- Set terrain size units if not already set
  if metadata.terrainSizeUnits == 0 then
    metadata.terrainSizeUnits = (maxX - minX) -- Updated to include borders
  end
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if not renderView then
    renderView = RenderViewManagerInstance:getOrCreateView('mapTilePreview')
    renderView.namedTexTargetColor = 'mapTilePreview'
    renderView.luaOwned = true -- Ensure the view is deleted properly if the GC collects it
  end

  im.SetNextWindowSize(initialWindowSize, im.Cond_FirstUseEver)

  if im.Begin("Map Tile Utility", windowOpen) then
    if im.Checkbox("Show Debug Panel", showDebugPanel) then
    end
    if im.Button("Export Navgraph") then
      calcBounds()
      exportNavgraph()
    end

    if isGeneratingTiles then
      im.Text(string.format("Generating tiles... Zoom Level %d, Tile %d / %d", currentZoomLevel, currentTileIndex, #tilePositions))
      if im.Button('Stop Tile Generation') then
        isGeneratingTiles = false
        restoreRenderingSettings()
        setRenderWorldMain(true)
        print("Tile generation stopped by user.")
      end
    else
      if im.Button('Start Tile Generation') then
        currentLevelName = core_levels.getLevelName(getMissionFilename())
        calcBounds()
        if not isGeneratingTiles then
          currentZoomLevel = 0
          metadata.tileGrid = {} -- Reset tile grid metadata
          generateTilePositions(currentZoomLevel)
          isGeneratingTiles = true
          frameCounter = 0 -- Reset frame counter
          adjustRenderingSettings() -- Adjust rendering settings before starting
          startTime = os.clock() -- Record start time
          if not show3DWorld[0] then
            setRenderWorldMain(false)
          end
          print("Tile generation started...")
        end
      end
    end

    if isGeneratingTiles and currentTileIndex > 1 then
      local tilesDone = (currentTileIndex - 1) + (currentZoomLevel * #tilePositions)
      local totalTiles = (#tilePositions) * (maxZoomLevel + 1)
      local elapsedTime = os.clock() - startTime
      local averageTimePerTile = elapsedTime / tilesDone
      local tilesRemaining = totalTiles - tilesDone
      local timeRemaining = averageTimePerTile * tilesRemaining
      im.Text(string.format("ETA: %s", formatTime(timeRemaining)))
    end

    -- Draw tile status grid
    local hoverTooltip = ""
    do
      local drawList = im.GetWindowDrawList()
      local cursorPos = im.GetCursorScreenPos()
      local gridWidth = 300
      local gridHeight = gridWidth * (totalTilesY / totalTilesX)
      local cellWidth = gridWidth / totalTilesX
      local cellHeight = gridHeight / totalTilesY

      for _, tile in ipairs(tilePositions) do
        local x = cursorPos.x + tile.xIndex * cellWidth
        local y = cursorPos.y + tile.yIndex * cellHeight
        local color

        if tile.status == 'done' then
          color = im.GetColorU322(im.ImVec4(0, 1, 0, 1)) -- Green
        elseif tile.status == 'current' then
          color = im.GetColorU322(im.ImVec4(1, 1, 0, 1)) -- Yellow
        else
          color = im.GetColorU322(im.ImVec4(0.5, 0.5, 0.5, 1)) -- Grey
        end

        -- Draw the tile rectangle
        im.ImDrawList_AddRectFilled(drawList, im.ImVec2(x, y), im.ImVec2(x + cellWidth - 1, y + cellHeight - 1), color)

        -- Handle mouse clicks on the tile
        local mousePos = im.GetMousePos()
        if im.IsMouseHoveringRect(im.ImVec2(x, y), im.ImVec2(x + cellWidth, y + cellHeight)) then
          if im.IsMouseClicked(0) then
            -- Jump to the tile when clicked
            currentTileIndex = tile.yIndex * totalTilesX + tile.xIndex + 1
            dump(tile)
            frameCounter = 0 -- Reset frame counter
          else
            hoverTooltip = dumps(tile)
          end
        end
      end
      -- Draw grid border
      im.ImDrawList_AddRect(drawList, cursorPos, im.ImVec2(cursorPos.x + gridWidth, cursorPos.y + gridHeight), im.GetColorU322(im.ImVec4(1, 1, 1, 1)))

      -- Advance cursor so that the next UI elements don't overlap
      im.Dummy(im.ImVec2(gridWidth, gridHeight))

    end
    im.SameLine()
    local texObj = imUtils.texObj('#mapTilePreview')
    im.Image(texObj.texId, im.ImVec2(300, 300))
    im.TextWrapped(hoverTooltip)
  end
  im.End()

  -- Debug Panel
  if showDebugPanel[0] then
    if im.Begin("Debug Panel", showDebugPanel) then
      if im.Checkbox("Show 3D world", show3DWorld) then
        setRenderWorldMain(show3DWorld[0])
      end
      if im.Checkbox("Run", runGenerator) then
      end
      if im.Checkbox("3D Debug", draw3DDebug) then
      end

      -- Zoom Level Selector
      if im.SliderInt("Zoom Level", selectedZoomLevel, 0, maxZoomLevel) then
        if selectedZoomLevel[0] ~= currentZoomLevel then
          currentZoomLevel = selectedZoomLevel[0]
          generateTilePositions(currentZoomLevel)
          currentTileIndex = 1
          frameCounter = 0
          print(string.format("Switched to Zoom Level %d", currentZoomLevel))
        end
      end

      if im.Button('Advance to Next Tile') then
        currentTileIndex = currentTileIndex + 1
        if currentTileIndex > #tilePositions then
          currentTileIndex = 1
        end
        frameCounter = 0 -- Reset frame counter for the next tile
      end
    end
    im.End()
  end

  if isGeneratingTiles and currentTileIndex <= #tilePositions then
    local tile = tilePositions[currentTileIndex]

    -- Use the same camera orientation as before
    local q = quatFromDir(vec3(0, 0, -1), vec3(0, 1, 0))
    q = QuatF(q.x, q.y, q.z, q.w)
    local mat = q:getMatrix()

    -- Use raycasting to find tile center height
    if not tile.decalPosition then
      local rayStart = vec3(tile.x, tile.y, 1000)
      local rayEnd = vec3(tile.x, tile.y, -1000)
      local hit = Engine.castRay(rayStart, rayEnd, true, true)
      if hit then
        tile.tileCenterPos = vec3(tile.x, tile.y, hit.pt.z)
      end
    end
    mat:setPosition(vec3(tile.x, tile.y, tile.tileCenterPos.z + 250))

    renderView.cameraMatrix = mat -- Determines where the virtual camera is in 3D space

    renderView.renderCubemap = false
    renderView.resolution = Point2I(tile.pixelSize, tile.pixelSize) -- Set desired resolution for tiles
    renderView.viewPort = RectI(0, 0, tile.pixelSize, tile.pixelSize)

    -- Adjust the orthographic projection parameters to cover the tile size
    local halfTileSize = tile.tileSize / 2
    local left = -halfTileSize
    local right = halfTileSize
    local bottom = halfTileSize
    local top = -halfTileSize

    if show3DWorld[0] then
      setRenderWorldMain(true)
    end

    -- Set up the orthographic projection
    renderView.frustum = Frustum.constructOrtho(left, right, bottom, top, 0.1, 2000)
    renderView.fov = 75 -- FOV is not used in orthographic projection but may be required by the API
    renderView.renderEditorIcons = false

    -- Increment frame counter
    frameCounter = frameCounter + 1

    if frameCounter >= framesPerTile then
      if tile.status ~= 'done' then
        local filename = string.format("%s/%s/%d/%d/%d.png", outputDirectory, currentLevelName, currentZoomLevel, tile.xIndex, tile.yIndex)
        renderView:saveToDisk(filename)
        tile.status = 'done'
      end

      -- not working :(
      --[[
      local data = {}
      data.texture = '/marker.png' -- Replace with your decal texture
      data.position = tile.tileCenterPos
      data.color = ColorF(1, 0, 0, 0.75)
      data.forwardVec = vec3(0, 1, 0)
      data.scale = vec3(10, 10, 1)
      Engine.Render.DynamicDecalMgr.addDecal(data)
      ]]

      -- Move to the next tile
      if runGenerator[0] then
        currentTileIndex = currentTileIndex + 1
        if currentTileIndex > #tilePositions then
          currentTileIndex = 1
          currentZoomLevel = currentZoomLevel + 1
          if currentZoomLevel > maxZoomLevel then
            -- Tile generation completed
            isGeneratingTiles = false
            restoreRenderingSettings()
            setRenderWorldMain(true)
            print("Tile generation completed!")

            -- Write the metadata JSON file
            local jsonData = {
              [currentLevelName] = metadata
            }
            jsonWriteFile('/tiles/metadata.json', jsonData, true)
          else
            generateTilePositions(currentZoomLevel)
            print(string.format("Starting Zoom Level %d...", currentZoomLevel))
          end
        end
        frameCounter = 0 -- Reset frame counter for the next tile
      end
    end


    if draw3DDebug[0] then
      -- Draw 3D world box to show the position of the tile
      local halfTileSize = tile.tileSize / 2
      local minPoint = vec3(tile.x - halfTileSize, tile.y - halfTileSize, tile.tileCenterPos.z + 100)
      local maxPoint = vec3(tile.x + halfTileSize, tile.y + halfTileSize, tile.tileCenterPos.z - 100)
      local color = ColorF(1, 0, 0, 0.2) -- Red color

      -- Draw the box in the 3D world
      --debugDrawer:drawBox(minPoint, maxPoint, color)
      debugDrawer:drawSphere(tile.tileCenterPos, 5, ColorF(0, 1, 1, 0.8))
      debugDrawer:drawSphere(minPoint, 5, ColorF(0, 1, 1, 0.8))
      local filename = string.format("%s/%s/%d/%d/%d.png", outputDirectory, currentLevelName, currentZoomLevel, tile.xIndex, tile.yIndex)
      simpleDebugText3d(filename, tile.tileCenterPos)
    end
  elseif isGeneratingTiles then
    -- Finished current zoom level, check if more zoom levels are left
    if currentZoomLevel < maxZoomLevel then
      currentZoomLevel = currentZoomLevel + 1
      generateTilePositions(currentZoomLevel)
      currentTileIndex = 1
      print(string.format("Starting Zoom Level %d...", currentZoomLevel))
    else
      -- Tile generation completed
      isGeneratingTiles = false
      currentTileIndex = 1
      tilePositions = {}
      frameCounter = 0
      restoreRenderingSettings() -- Restore original rendering settings after completion
      startTime = 0
      print("Tile generation completed!")

      -- Write the metadata JSON file
      local jsonData = {
        [currentLevelName] = metadata
      }
      jsonWriteFile('/tiles/metadata.json', jsonData, true)
      if not show3DWorld[0] then
        setRenderWorldMain(true)
      end
    end
  end
end

M.onUpdate = onUpdate

return M
