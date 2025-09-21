-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- Internal storage for tile-based scene data.
-- We'll only expose them via accessor functions below.
local tileDataCache      = nil
local tileDataCacheMeta  = nil

local debugMode = true

--------------------------------------------------------------------------------
-- Utility: get tile indices for a given position
--------------------------------------------------------------------------------
local function getTileIndices(x, y, tileSize)
  local tx = math.floor(x / tileSize)
  local ty = math.floor(y / tileSize)
  return tx, ty
end

--------------------------------------------------------------------------------
-- Create the tile in tileMap if it doesn't exist, then return the tile entry
--------------------------------------------------------------------------------
local function ensureTileEntry(tileMap, tx, ty, tileSize)
  if not tileMap[tx] then
    tileMap[tx] = {}
  end

  if not tileMap[tx][ty] then
    local now = os.time()
    tileMap[tx][ty] = {
      objects     = {},
      roads       = {},
      forestItems = {},
      metadata    = {
        tileX         = tx,
        tileY         = ty,
        tileSize      = tileSize,
        uniqueId      = string.format("Tile_%d_%d", tx, ty),
        creationDate  = now,
        expireDate    = now + 3600, -- example: expires in 1 hour
        objectCount   = 0,
        roadCount     = 0,
        roadNodeCount = 0,
        forestItemCount = 0
      }
    }
  end

  return tileMap[tx][ty]
end

--------------------------------------------------------------------------------
-- Build the tile data: ties normal objects, DecalRoad nodes, and forest items
-- to respective tiles. tileSize defaults to 100 in many usage scenarios.
--------------------------------------------------------------------------------
local function buildTileData(tileSize)
  -- Quick check on tileSize
  if not tileSize or tileSize <= 0 then
    log("E", "c2_simScene",
      string.format("Invalid tileSize (%s). Must be a positive number. Aborting build.", tostring(tileSize)))
    return
  end

  local tileMap = {}
  local decalRoadCount = 0
  local totalDecalNodes = 0
  local normalObjCount = 0

  -- Keep track of min and max tile indices:
  local minX, maxX = math.huge, -math.huge
  local minY, maxY = math.huge, -math.huge

  -------------------------------------------------------------------------------
  -- 1) Gather normal objects, DecalRoad, and Forest objects
  -------------------------------------------------------------------------------
  local allObjectNames = scenetree.getAllObjects()
  if allObjectNames and #allObjectNames > 0 then
    for _, objName in ipairs(allObjectNames) do
      local obj = scenetree.findObject(objName)
      if obj and obj.getClassName then
        local className = obj:getClassName()

        ----------------------------------------------------------------------
        -- DecalRoad objects
        ----------------------------------------------------------------------
        if className == "DecalRoad" and obj.getNodeCount and obj.getMiddleEdgePosition and obj.getPosition and obj.drivability > 0 then
          local nodeCount = obj:getEdgeCount()
          decalRoadCount = decalRoadCount + 1
          totalDecalNodes = totalDecalNodes + nodeCount

          local roadPos = obj:getPosition()
          -- If roadPos is nil or missing x/y, skip this object
          if not (roadPos and roadPos.x and roadPos.y) then
            log("W", "c2_simScene",
              string.format("Skipping DecalRoad '%s' with invalid position data (cannot determine tile).", objName))
            goto continueObject
          end

          -- Determine the tile based on the object's overall position
          local objTx = math.floor(roadPos.x / tileSize)
          local objTy = math.floor(roadPos.y / tileSize)

          if not (objTx and objTy) then
            log("W", "c2_simScene",
              string.format("Skipping DecalRoad '%s'. Computed invalid tile indices for the object's overall position.", objName))
            goto continueObject
          end

          -- Ensure the tile for the object's position
          local tile = ensureTileEntry(tileMap, objTx, objTy, tileSize)

          -- Gather node data, but store everything in this single tile
          local roadNodes = {}
          for i = 0, nodeCount - 1 do
            local pos = obj:getMiddleEdgePosition(i)
            if not (pos and pos.x and pos.y) then
              log("W", "c2_simScene",
                string.format("Skipping invalid road node %d for DecalRoad '%s' (pos.x or pos.y is nil)", i, objName))
              goto continueRoadNode
            end

            -- Position is converted to local tile coordinates based on the object's tile
            local localX = pos.x - (objTx * tileSize)
            local localY = pos.y - (objTy * tileSize)
            local radius = 1

            if obj.getLeftEdgePosition and obj.getRightEdgePosition then
              local leftPos  = obj:getLeftEdgePosition(i)
              local rightPos = obj:getRightEdgePosition(i)
              if leftPos and rightPos then
                local distLeft  = pos:squaredDistance(leftPos)
                local distRight = pos:squaredDistance(rightPos)
                radius = math.sqrt(math.min(distLeft, distRight))
              end
            end

            table.insert(roadNodes, {
              pos    = { localX, localY, pos.z },
              radius = radius
            })
            ::continueRoadNode::
          end

          -- Only add the road object if we had at least one valid node
          if #roadNodes > 0 then
            table.insert(tile.roads, {
              objName = objName,
              nodes   = roadNodes
            })
            tile.metadata.roadCount     = tile.metadata.roadCount + 1
            tile.metadata.roadNodeCount = tile.metadata.roadNodeCount + nodeCount
          end
          ::continueObject::

        ----------------------------------------------------------------------
        -- Forest objects
        ----------------------------------------------------------------------
        elseif className == "Forest" and obj.getData then
          local fData = obj:getData()
          if fData and fData.getItems then
            local forestItemsIn = fData:getItems()

            for i = 1, #forestItemsIn do
              local forestItem = forestItemsIn[i]
              local itemPos = forestItem:getPosition()

              -- Skip invalid positions
              if not (itemPos and itemPos.x and itemPos.y) then
                log("W", "c2_simScene",
                  string.format("Skipping invalid forest item %d for Forest '%s' (itemPos.x or itemPos.y is nil)", i, objName))
                goto continueForestItem
              end

              local tx = math.floor(itemPos.x / tileSize)
              local ty = math.floor(itemPos.y / tileSize)
              if not (tx and ty) then
                log("W", "c2_simScene",
                  string.format("Skipping forest item %d for Forest '%s'. getTileIndices returned nil tx or ty.", i, objName))
                goto continueForestItem
              end

              local tile = ensureTileEntry(tileMap, tx, ty, tileSize)

              -- Make the position relative to this tile
              local localX = itemPos.x - tx * tileSize
              local localY = itemPos.y - ty * tileSize
              local itemRot = quat(forestItem:getTransform():toQuatF()):toEulerYXZ()
              local itemScale = forestItem:getScale()
              local itemRadius = forestItem:getRadius()
              local itemBox = forestItem:getObjBox()
              local itemMesh = forestItem:getData() and forestItem:getData():getShapeFile()

              table.insert(tile.forestItems, {
                objName = objName,
                itemIdx = i,
                pos    = { localX, localY, itemPos.z },
                rot    = { itemRot.x, itemRot.y, itemRot.z },
                scale  = itemScale,
                radius = itemRadius,
                bb     = {itemBox.minExtents.x, itemBox.minExtents.y, itemBox.minExtents.z, itemBox.maxExtents.x, itemBox.maxExtents.y, itemBox.maxExtents.z },
                mesh   = itemMesh
              })
              tile.metadata.forestItemCount = tile.metadata.forestItemCount + 1
              ::continueForestItem::
            end
          end

        ----------------------------------------------------------------------
        -- Regular (non-road, non-forest) objects
        ----------------------------------------------------------------------
        elseif obj.getPosition and obj.getRotation and obj.getScale and obj.getWorldBox then
          local pos = obj:getPosition()
          local centerPos = obj:getWorldBox():getCenter()
          local offsetPivot = centerPos - pos

          if not (pos and pos.x and pos.y) then
            log("W", "c2_simScene",
              string.format("Skipping object '%s' with invalid position data.", objName))
            goto continueObject
          end

          local tx = math.floor(pos.x / tileSize)
          local ty = math.floor(pos.y / tileSize)
          if not (tx and ty) then
            log("W", "c2_simScene",
              string.format("Skipping object '%s'. getTileIndices returned nil tx or ty.", objName))
            goto continueObject
          end

          local tile = ensureTileEntry(tileMap, tx, ty, tileSize)

          -- Convert to local position
          local localX = pos.x - tx * tileSize
          local localY = pos.y - ty * tileSize

          local rot = quat(obj:getRotation()):toEulerYXZ()
          local scale = obj:getScale()
          local box = obj:getObjBox()

          table.insert(tile.objects, {
            name  = objName,
            pos   = { localX, localY, pos.z },
            rot   = { rot.x, rot.y, rot.z },
            scale = { scale.x, scale.y, scale.z },
            bb    = { box.minExtents.x, box.minExtents.y, box.minExtents.z, box.maxExtents.x, box.maxExtents.y, box.maxExtents.z },
            offsetPivot = { offsetPivot.x, offsetPivot.y, offsetPivot.z },
            type  = className
          })

          tile.metadata.objectCount = tile.metadata.objectCount + 1
          normalObjCount = normalObjCount + 1
          ::continueObject::
        end
      end
    end
  end

  ------------------------------------------------------------------------------
  -- Build meta stats
  ------------------------------------------------------------------------------
  local now = os.time()
  local totalTiles = 0
  for x, col in pairs(tileMap) do
    -- Update min/max X
    if x < minX then minX = x end
    if x > maxX then maxX = x end

    for y, _ in pairs(col) do
      totalTiles = totalTiles + 1
      -- Update min/max Y
      if y < minY then minY = y end
      if y > maxY then maxY = y end
    end
  end

  local meta = {
    tileSize        = tileSize,
    totalTiles      = totalTiles,
    creationDate    = now,
    expireDate      = now + 3600,  -- e.g. 1 hour validity
    decalRoadCount  = decalRoadCount,
    totalDecalNodes = totalDecalNodes,
    normalObjCount  = normalObjCount,

    -- Record min & max tile indices
    minTileX = (minX == math.huge) and nil or minX,
    maxTileX = (maxX == -math.huge) and nil or maxX,
    minTileY = (minY == math.huge) and nil or minY,
    maxTileY = (maxY == -math.huge) and nil or maxY
  }

  if debugMode then
    for x, col in pairs(tileMap) do
      for y, tileEntry in pairs(col) do
        log("I", "c2_simScene",
          string.format("[debug] tile (%d,%d) has %d objects, %d roads (%d nodes), %d forest items",
            x, y, tileEntry.metadata.objectCount, tileEntry.metadata.roadCount,
            tileEntry.metadata.roadNodeCount, tileEntry.metadata.forestItemCount))
      end
    end
  end

  -- Store results internally
  tileDataCache     = tileMap
  tileDataCacheMeta = meta
end

--------------------------------------------------------------------------------
-- Accessor for tile cache
--------------------------------------------------------------------------------
local function getTileCache()
  return tileDataCache, tileDataCacheMeta
end


-- Hook function: called when the extension loads
  local function onExtensionLoaded()
    log("I", "tileManager", "Extension loaded; building tile data...")
    buildTileData(100)  -- Example tileSize
  end

  M.onExtensionLoaded = onExtensionLoaded

  -- Called when a WebSocket message for "c2" is received
  local function onC2WebSocketHandlerMessage(args)
    local msg    = args.message
    local server = args.server -- The server object
    local evt    = args.event  -- The event data (like peerId)

    if not msg or type(msg) ~= 'table' then return end

    -- "buildTileData" request:
    if msg.type == "buildTileData" then
      local tileSize = msg.tileSize or 300
      log("I", "tileManager", string.format("Received buildTileData message; size=%d", tileSize))
      buildTileData(tileSize)

    -- "getTileCacheInfo" request:
    elseif msg.type == "getTileCacheInfo" then
      local _, meta = getTileCache()
      if meta then
        server:sendData(evt.peerId, jsonEncode({
          type         = "tileCacheInfo",
          tileSize     = meta.tileSize,
          totalTiles   = meta.totalTiles,
          creationDate = meta.creationDate,
          expireDate   = meta.expireDate,
          minTileX     = meta.minTileX,
          maxTileX     = meta.maxTileX,
          minTileY     = meta.minTileY,
          maxTileY     = meta.maxTileY
        }))
      else
        server:sendData(evt.peerId, jsonEncode({
          type = "error",
          msg  = "No tile cache metadata available."
        }))
      end

    -- "getSceneTile" request:
    elseif msg.type == "getSceneTile" then
      local tileMap, meta = getTileCache()
      if not tileMap or not meta then
        server:sendData(evt.peerId, jsonEncode({
          type = "error",
          msg  = "No tile cache available."
        }))
        return
      end

      local tx = tonumber(msg.tileX) or 0
      local ty = tonumber(msg.tileY) or 0
      local tile = tileMap[tx] and tileMap[tx][ty]
      if not tile then
        server:sendData(evt.peerId, jsonEncode({
          type = "error",
          msg  = string.format("Tile (%d,%d) not found", tx, ty)
        }))
        return
      end

      server:sendData(evt.peerId, jsonEncode({
        type  = "sceneTileData",
        tileX = tx,
        tileY = ty,
        data  = tile
      }))

    else
      log("E", "tileManager", "Unknown data type requested: " .. tostring(msg.type))
    end
  end


--------------------------------------------------------------------------------
-- Public Interface
--------------------------------------------------------------------------------
M.buildTileData = buildTileData
M.getTileCache  = getTileCache
M.onC2WebSocketHandlerMessage = onC2WebSocketHandlerMessage

return M