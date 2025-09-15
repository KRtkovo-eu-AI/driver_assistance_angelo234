-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This utility generates a terrain block from input data; it is a work in progress
-- Sample usage (assuming that smallgrid level is loaded):

--[[
  extensions.load('util/terrainGenerator')
  obj = util_terrainGenerator.new()

  arr = {}
  for x = 1, 256 do
    arr[x] = {}
    for y = 1, 256 do
      arr[x][y] = randomGauss3() / 5
    end
  end

  obj:setBitmapFromArray(arr)
  obj:createTerrain()
  obj:centerTerrain()
]]--

-- You can also save and load PNG heightmaps

-- To set a project directory and auto load maps, use: obj:setUserDir(dir)
-- It will auto process these files, if found: "*_heightMap.png", "*_holeMap.png", "*_layerMap_*.png", "*materials.json"
-- Then, you should be able to proceed: obj:createTerrain()

-- TODO: array as C ffi array instead of Lua array

local M = {}

local logTag = 'terrainGenerator'

local C = {}

function C:init(data)
  data = data or {}
  self.terrainScale = data.terrainScale or 1 -- grid size of terrain
  self.terrainHeight = data.terrainHeight or self:getMaxHeight() -- maximum height of terrain
  self.bitmap = GBitmap()
  self.bitmapWidth = 0
  self.name = data.name or 'temp' -- generic name
  self.terrainName = data.terrainName or 'theTerrain' -- terrain object name in scenetree
  self.userDir = data.userDir or 'temp/' -- project directory; if temp, files get automatically cleared
  self.defaultMaterial = 'Grass' -- default material to use as base
  self.heightMap = data.heightMap or 'temp/temp_heightMap.png' -- heightmap png file path
  self.holeMap = data.holeMap or '' -- holemap png file path
  self.textureMaps = data.textureMaps -- table of terrain material maps
  self.materials = data.materials -- table of terrain materials
  self.flipYAxis = false -- flips Y axis of image maps
  self:resetUserDir('temp/')
end

function C:saveBitmap(path) -- saves heightmap array as PNG file
  path = path or self.heightMap
  if not path then return end

  if not string.endswith(path, '.png') then
    path = path..'.png'
  end

  if not self.bitmap:saveFile(path) then
    log('E', logTag, 'Bitmap save fail: '..path)
    return
  end

  self.heightMap = path
end

function C:loadBitmap(path) -- loads heightmap array from PNG file
  path = path or self.heightMap
  if not path then return end

  if not string.endswith(path, '.png') then
    path = path..'.png'
  end

  if not self.bitmap:loadFile(path) then
    log('E', logTag, 'Bitmap load fail: '..path)
    return
  end

  self.heightMap = path
  self.bitmapWidth = self.bitmap:getWidth()
end

function C:getMaxHeight() -- returns the maximum height of the current terrain
  local height = 0
  if core_terrain.getTerrain() then
    height = core_terrain.getTerrain():getHeightScaleUser()
  end
  return height
end

function C:setTerrainOffset(vec) -- sets a new position for the terrain block (vec3(0, 0, 0) is the corner of the terrain block)
  if core_terrain.getTerrain() then
    core_terrain.getTerrain():setPosition(vec)
    be:reloadCollision()
  end
end

function C:centerTerrain() -- makes the origin the center of the terrain block
  if core_terrain.getTerrain() then
    local n = core_terrain.getTerrain():getWorldBlockSize() * -0.5
    self:setTerrainOffset(vec3(n, n, 0))
  end
end

function C:setPoint(x, y, z) -- sets a pixel of the bitmap (min x and min y are zero)
  local n = clamp(z / self.terrainHeight, 0, 1) * 65535
  --self.bitmap:setColor(x, y, ColorF(n, n, n, 1))
  self.bitmap:setTexel(x, y, n, n, n, 65535)
end

function C:setBitmapFromArray(array) -- creates bitmap data from an array
  if type(array) ~= 'table' or not array[1] or not array[1][1] then --or type(array) ~= 'cdata'
    log('E', logTag, 'Error with processing bitmap from array')
    return
  end
  local size = math.max(#array, #array[1])
  size = 128 * math.ceil(size / 128) -- dimensions need to be divisible by 128

  self.terrainHeight = -math.huge
  for x = 1, size do
    if not array[x] then
      array[x] = {}
      for y = 1, size do
        table.insert(array[x], 0)
      end
    end

    for y = 1, size do
      if not array[x][y] then
        array[x][y] = 0
      end

      if array[x][y] > self.terrainHeight then
        self.terrainHeight = array[x][y]
      end
    end
  end

  self.bitmapWidth = size
  self.bitmap:allocateBitmap(self.bitmapWidth, self.bitmapWidth, false, 'GFXFormatR16')

  for x = 1, size do
    for y = 1, size do
      self:setPoint(x - 1, y - 1, array[x][y])
    end
  end

  log('I', logTag, 'Bitmap created, with dimensions: '..self.bitmapWidth..' * '..self.bitmapWidth)
  self:saveBitmap()
end

function C:resetUserDir(dir) -- clears all matching files in the directory
  dir = dir or self.userDir
  for _, map in ipairs(FS:findFiles(dir, '*heightMap.png', -1, false, true)) do
    FS:removeFile(map)
  end
  for _, map in ipairs(FS:findFiles(dir, '*holeMap.png', -1, false, true)) do
    FS:removeFile(map)
  end
  for _, map in ipairs(FS:findFiles(dir, '*layerMap_*.png', -1, false, true)) do
    FS:removeFile(map)
  end
end

function C:resetTerrain(deleteOnly) -- resets the terrain block
  local terrain = core_terrain.getTerrain()
  if terrain then terrain:delete() end
  if deleteOnly then return end

  local tb = TerrainBlock()
  tb:setName(self.terrainName)
  tb:registerObject(self.terrainName)
  tb:setTerrFileLvlFolder('/levels/'..getCurrentLevelIdentifier())
  scenetree.MissionGroup:addObject(tb)
end

function C:getTextureMap(path, matName) -- returns texture map table data for a given material
  return {path = path, selected = false, material = matName or '', materialId = #self.textureMaps, channel = 'R', channelId = 0}
end

function C:exportTerrainMaps(dir, prefix) -- exports current terrain PNG maps to file
  -- this can be used on any existing terrain!
  dir = dir or self.userDir
  prefix = prefix or self.name

  local terrain = core_terrain.getTerrain()
  if terrain then
    prefix = dir..prefix..'_'
    terrain:exportHeightMap(prefix..'heightMap.png', 'png')
    terrain:exportHoleMaps(prefix..'holeMap', 'png')
    terrain:exportLayerMaps(prefix..'layerMap', 'png')
    log('I', logTag, 'Exported current terrain layers: '..dir)
  end
end

function C:setPngData(path, size, height, scale, flipY) -- sets the heightmap file, width, height, and scale
  self.heightMap = path
  self.bitmapWidth = size
  self.terrainHeight = height
  self.terrainScale = scale
  self.flipYAxis = flipY
end

function C:setUserDir(dir) -- sets the user directory, and automatically sets file data if found
  self.userDir = dir or 'temp/'
  if not string.endswith(self.userDir, '/') then self.userDir = self.userDir..'/' end
  if FS:directoryExists(self.userDir) then
    local heightMapFiles = FS:findFiles(self.userDir, '*heightMap.png', -1, false, true)
    local holeMapFiles = FS:findFiles(self.userDir, '*holeMap.png', -1, false, true)
    if heightMapFiles[1] then
      self.heightMap = heightMapFiles[1]
      log('I', logTag, 'Heightmap file found: '..self.heightMap)
    end
    if holeMapFiles[1] then
      self.holeMap = holeMapFiles[1]
      log('I', logTag, 'Holemap file found: '..self.holeMap)
    end
    local materialFiles = FS:findFiles(self.userDir, '*materials.json', -1, false, true)
    if materialFiles[1] then
      self:setMaterials({filePath = materialFiles[1]})
      log('I', logTag, 'Materials file found: '..materialFiles[1])
    end
  end
end

function C:setDefaultMaterial() -- grid material from smallgrid level
  local name = 'default'
  local terrainMat = TerrainMaterial.findOrCreate(name)
  terrainMat:setInternalName(name)
  terrainMat:setDetailMap('levels/smallgrid/grid_10_diff.dds')
  terrainMat:setDetailSize(10)
  terrainMat:setDetailDistance((self.terrainWidth or 1024) * self.terrainScale)
  terrainMat:setDiffuseMap('levels/smallgrid/art/terrains/Overlay_02')
  terrainMat:setDiffuseSize(256)
  terrainMat:setMacroMap('levels/smallgrid/grid_10_diff.dds')
  terrainMat:setMacroStrength(0)
  self.materials = {name}
end

local materialTextureSetMaps = {'baseColor', 'normal', 'roughness', 'ao', 'height'}
local materialTextureProperties = {'%sBaseTex', '%sBaseTexSize', '%sMacroTex', '%sMacroTexSize', '%sMacroStrength', '%sDetailTex', '%sDetailTexSize', '%sDetailStrength'}
function C:setMaterials(matData) -- sets terrain material info, to be used before creating terrain
  self.materials = {}
  self._tempMaterialTextureSet = nil

  if not matData then -- if no material data exists, try to get it from the existing terrain
    if core_terrain.getTerrain() then
      for _, m in ipairs(core_terrain.getTerrain():getMaterials()) do -- get all existing materials from terrain
        if m:getInternalName() ~= 'warning_material' then -- always ignore warning material
          table.insert(self.materials, m:getInternalName())
        end
      end
    end
  else
    if matData.filePath then
      local json = jsonReadFile(matData.filePath)
      if not json then
        log('W', logTag, 'Materials json not found, now using default material')
        self:setDefaultMaterial()
        return
      end

      local keysSorted = tableKeysSorted(json) -- material name alphabetical order; may not be needed

      for _, key in ipairs(keysSorted) do
        local data = json[key]
        local matName = data.internalName or key
        if not arrayFindValueIndex(self.materials, matName) then -- prevent duplicates?
          if string.find(key, 'TerrainMaterialTextureSet') then
            self._tempMaterialTextureSet = {
              baseTexSize = data.baseTexSize[1],
              detailTexSize = data.detailTexSize[1],
              macroTexSize = data.macroTexSize[1]
            }
          else
            local terrainMat = TerrainMaterial.findOrCreate(key)
            terrainMat:setInternalName(matName)

            -- v1 materials
            if data.diffuseMap then terrainMat:setDiffuseMap(data.diffuseMap) end
            if data.diffuseSize then terrainMat:setDiffuseSize(data.diffuseSize) end
            if data.normalMap then terrainMat:setNormalMap(data.normalMap) end
            if data.detailMap then terrainMat:setDetailMap(data.detailMap) end
            if data.macroMap then terrainMat:setMacroMap(data.macroMap) end
            if data.detailSize then terrainMat:setDetailSize(data.detailSize) end
            if data.detailStrength then terrainMat:setDetailStrength(data.detailStrength) end
            if data.detailDistance then terrainMat:setDetailDistance(data.detailDistance) end
            if data.macroSize then terrainMat:setMacroSize(data.macroSize) end
            if data.macroDistance then terrainMat:setMacroDistance(data.macroDistance) end
            if data.macroStrength then terrainMat:setMacroStrength(data.macroStrength) end
            if data.useSideProjection then terrainMat:setUseSideProjection(data.useSideProjection) end
            if data.parallaxScale then terrainMat:setParallaxScale(data.parallaxScale) end

            -- v1.5 materials
            if data.macroDistAtten then
              if type(data.macroDistAtten) == 'table' then data.macroDistAtten = table.concat(data.macroDistAtten, ' ') end
              terrainMat:setField('macroDistAtten', 0, data.macroDistAtten)
            end
            if data.detailDistAtten then
              if type(data.detailDistAtten) == 'table' then data.detailDistAtten = table.concat(data.detailDistAtten, ' ') end
              terrainMat:setField('detailDistAtten', 0, data.detailDistAtten)
            end

            if data.groundmodelName then terrainMat:setGroundmodelName(data.groundmodelName) end
            if data.annotation then terrainMat:setField('annotation', 0, data.annotation) end

            for _, map in ipairs(materialTextureSetMaps) do
              for _, prop in ipairs(materialTextureProperties) do
                local field = string.format(prop, map)
                if data[field] then
                  if type(data[field]) == 'table' then data[field] = table.concat(data[field], ' ') end
                  terrainMat:setField(field, 0, data[field])
                end
              end
            end

            table.insert(self.materials, matName)
          end
        end
      end
      log('I', logTag, 'Processed materials from json: '..#self.materials)
    elseif type(matData.materials) == 'table' then -- directly set materials array (risky)
      self.materials = matData.materials
    end
  end

  if not self.materials[1] then -- if no materials exist at all, create a default one
    self:setDefaultMaterial()
  end
end

function C:setTextureMaps() -- sets texture maps data
  self.textureMaps = {}
  self.materialsToTextureMaps = {}
  for i, key in ipairs(self.materials) do
    self.materialsToTextureMaps[key] = false
  end

  for _, map in ipairs(FS:findFiles(self.userDir, '*_layerMap_*.png', -1, false, true)) do
    local matName = string.match(map, 'layerMap_%d*_(.*)%.(%w*)$')
    if self.materialsToTextureMaps[matName] == false then
      self.materialsToTextureMaps[matName] = map
      table.insert(self.textureMaps, self:getTextureMap(map, matName))
    end
  end
  log('I', logTag, 'Matched texture maps with materials: '..#self.textureMaps)

  local count = #self.textureMaps
  local blanks = 0
  for k, v in pairs(self.materialsToTextureMaps) do
    if not v then -- if material to texture map entry is still false
      -- creates a blank PNG for safety
      local n = k == self.defaultMaterial and 255 or 0 -- white if no previous entries exist, otherwise black
      local b = GBitmap()
      b:init(self.terrainWidth, self.terrainWidth)
      b:fillColor(ColorI(n, n, n, 255))

      local path = self.userDir..self.name..'_layerMap_'..count..'_'..k..'.png'
      b:saveFile(path)
      table.insert(self.textureMaps, self:getTextureMap(path, k))
      count = count + 1
      blanks = blanks + 1
    end
  end

  if blanks > 0 then
    log('I', logTag, 'Generated blank texture maps: '..blanks)
  end

  --table.sort(self.textureMaps, function(a, b) return a.materialId < b.materialId end)
end

function C:createTerrain() -- creates a terrain block from the saved parameters (heightmap, materials, height, scale, etc.)
  if not self.heightMap or (string.find(self.heightMap, '/') and not FS:fileExists(self.heightMap)) then
    log('E', logTag, 'Heightmap file not found: '..tostring(self.heightMap))
    return
  end

  if not self.terrainId and core_terrain.getTerrain() then
    log('I', logTag, 'Base terrain found, now deleting it in order to create custom terrain')
  end

  self:resetTerrain()
  self.terrainId = nil
  local terrain = core_terrain.getTerrain()
  local terrainBitmap = GBitmap()
  terrainBitmap:loadFile(self.heightMap)
  self.terrainWidth = terrainBitmap:getWidth()

  if self._tempMaterialTextureSet then
    local mts = self._tempMaterialTextureSet
    local filename = getMissionPath() .. 'terrains/main.materials.json'
    local name = string.match(getMissionPath(), "/(.[^/]+)/$") .. "TerrainMaterialTextureSet"

    if scenetree.findObject(name) then -- delete old TerrainMaterialTextureSet; not sure if this causes a crash
      scenetree.findObject(name):delete()
    end

    local textureSet = scenetree.findObject(name)
    if not textureSet then
     textureSet = createObject('TerrainMaterialTextureSet')
    end

    textureSet:setFileName(filename)
    textureSet:setField('name', 0, name)
    textureSet:setField('baseTexSize', 0, mts.baseTexSize..' '..mts.baseTexSize)
    textureSet:setField('detailTexSize', 0, mts.detailTexSize..' '..mts.detailTexSize)
    textureSet:setField('macroTexSize', 0, mts.macroTexSize..' '..mts.macroTexSize)
    textureSet.canSave = true
    textureSet:registerObject(name)
    terrain:setField('materialTextureSet', 0, name)
  end

  if self.terrainScale <= 0 then
    self.terrainScale = 0.01
    log('W', logTag, 'Warning, terrain scale is less than or equal to zero')
  end

  if self.terrainHeight <= 0 then
    self.terrainHeight = 0
    log('W', logTag, 'Warning, terrain height is less than or equal to zero')
  end

  if not self.holeMap or (string.find(self.holeMap, '/') and not FS:fileExists(self.holeMap)) then
    self.holeMap = ''
  end

  if not self.materials or not self.materials[1] then
    self:setMaterials()
  end

  if not self.textureMaps then
    self:setTextureMaps()
  end

  local processedMaterials = {}
  for _, v in ipairs(self.textureMaps) do -- proper material order
    table.insert(processedMaterials, v.material)
  end

  local materialsTblSize = tableSize(self.materials)
  local textureMapsTblSize = tableSize(processedMaterials)
  self.materials = deepcopy(processedMaterials)
  for i, mat in ipairs(self.materials) do
    terrain:addMaterial(mat, -1)
    terrain:updateMaterial(i - 1, mat)
  end

  if materialsTblSize ~= textureMapsTblSize then
    log('E', logTag, 'Terrain import error: Materials table size is not equal to texture maps table size')
    log('I', logTag, 'Materials: '..materialsTblSize..', texture maps: '..textureMapsTblSize)
    terrain:delete()
    return
  end

  terrain:importMaps(self.heightMap, self.terrainScale, self.terrainHeight, self.holeMap, self.materials, self.textureMaps, self.flipYAxis)
  self.terrainId = terrain:getID() -- this id only gets saved if a new terrain was created here
  terrain:save(terrain:getTerrFileName())
end

local function new(data)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(data)
  return o
end

M.new = new

return M