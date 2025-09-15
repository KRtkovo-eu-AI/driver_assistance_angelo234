-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'forestGenerator'

local forestData
local forestItemDict = {}
local forestItemTypes = {}

local function newMatrix(pos, rotEuler)
  pos = pos or vec3()
  rotEuler = rotEuler or vec3()
  local mtx = MatrixF(true)
  mtx:setFromEuler(rotEuler)
  mtx:setPosition(pos)
  return mtx
end

-- initialize forest data
local function initForest()
  if not scenetree.MissionGroup then
    log('W', logTag, 'Level not loaded!')
    return
  end

  if not core_forest or not core_forest.getForestObject() then
    log('W', logTag, 'Forest object does not exist!')
    return
  end

  forestData = core_forest.getForestObject():getData()
  table.clear(forestItemDict)
  for i, v in ipairs(forestData:getItems()) do
    local internalName = v:getData():getInternalName()
    forestItemDict[internalName] = v
  end
  forestItemTypes = tableKeysSorted(forestItemDict)
end
initForest()

-- TODO: function for creating template items

-- returns an array of all forest item types
local function getForestItemTypes()
  return forestItemTypes
end

-- returns the forest item template
local function getForestItemDict()
  return forestItemDict
end

-- creates a single forest item
local function createForestItem(itemType, pos, rotDeg, scl)
  scl = scl or 1
  local baseItem = forestItemDict[itemType]
  if not baseItem then
    log('E', logTag, 'Forest item template not found: '..itemType)
    return
  end
  return forestData:createNewItem(baseItem:getData(), newMatrix(pos, vec3(0, 0, rotDeg or 0)), scl)
end

-- creates a forest with some randomized parameters (within rectangle bounds)
local function createRandomForestRect(itemTypeArray, amount, pos, minX, maxX, minY, maxY, rotSteps, minScl, maxScl)
  itemTypeArray = itemTypeArray or forestItemTypes
  pos = pos or vec3(0, 0, 0)
  minX = minX or -100
  maxX = maxX or 100
  minY = minY or -100
  maxY = maxY or 100
  rotSteps = math.max(1, rotSteps or 360)
  minScl = minScl or 0.75
  maxScl = maxScl or 1.25

  -- absolute x and y bounds
  minX = pos.x + minX
  maxX = pos.x + maxX
  minY = pos.y + minY
  maxY = pos.y + maxY

  local items = {}
  local arraySize = #itemTypeArray
  local bluePos = vec3(math.random(), math.random(), 0)
  for i = 1, amount do
    local data = {}
    local randomRot = math.random(math.ceil(rotSteps))
    data.type = itemTypeArray[math.random(arraySize)]
    data.pos = vec3(bluePos:getBlueNoise2d())
    data.pos.x = lerp(minX, maxX, data.pos.x)
    data.pos.y = lerp(minY, maxY, data.pos.y)
    data.rotDeg = lerp(0, 360, randomRot / rotSteps)
    data.scl = lerp(minScl, maxScl, math.random())

    if core_terrain.getTerrain() then
      data.pos.z = core_terrain.getTerrainHeight(data.pos)
    end

    table.insert(items, createForestItem(data.type, data.pos, data.rotDeg, data.scl))
  end

  return items
end

-- creates a forest with some randomized parameters (within a radius)
local function createRandomForestRadial(itemTypeArray, amount, pos, radius, rotSteps, minScl, maxScl)
  itemTypeArray = itemTypeArray or forestItemTypes
  pos = pos or vec3(0, 0, 0)
  radius = radius or 50
  rotSteps = math.max(1, rotSteps or 360)
  minScl = minScl or 0.75
  maxScl = maxScl or 1.25

  local items = {}
  local arraySize = #itemTypeArray
  local bluePos = vec3(pos)
  for i = 1, amount do
    local data = {}
    local randomRot = math.random(math.ceil(rotSteps))
    data.type = itemTypeArray[math.random(arraySize)]
    data.pos = vec3(bluePos:getRandomPointInCircle(radius))
    data.rotDeg = lerp(0, 360, randomRot / rotSteps)
    data.scl = lerp(minScl, maxScl, math.random())

    if core_terrain.getTerrain() then
      data.pos.z = core_terrain.getTerrainHeight(data.pos)
    end

    table.insert(items, createForestItem(data.type, data.pos, data.rotDeg, data.scl))
  end

  return items
end

-- TODO: function for creating a forest in a poly area

-- updates an existing forest item
local function updateForestItem(item, pos, rotDeg, scl)
  scl = scl or 1
  local mtx = newMatrix(pos, vec3(0, 0, rotDeg or 0))
  return forestData:updateItem(item:getKey(), item:getPosition(), item:getData(), mtx, scl, item:getUid())
end

-- returns an array of forest items found within a given polygon (or if none given, returns all items)
local function getForestItemsPolygon(polygon)
  if not polygon then
    return forestData:getItems()
  end

  local lassoNodes2D = {}

  -- the polygon is an array of vec3 positions
  for i, v in ipairs(polygon) do
    table.insert(lassoNodes2D, Point2F(v.x, v.y))
  end

  return forestData:getItemsPolygon(lassoNodes2D)
end

-- returns an array of forest items found within a given position and radius
local function getForestItemsRadius(pos, radius)
  pos = pos or core_camera.getPosition()
  radius = radius or 20
  return forestData:getItemsCircle(pos, radius)
end

-- deletes a single forest item
local function deleteForestItem(item)
  forestData:removeItem(item)
end

-- deletes all forest items (except for template items)
local function clearForest()
  for i, item in ipairs(forestData:getItems()) do
    local itemType = item:getData():getInternalName()
    if forestItemDict[itemType] and forestItemDict[itemType]:getKey() ~= item:getKey() then
      deleteForestItem(item)
    end
  end
  initForest() -- just in case
end

M.initForest = initForest
M.createForestItem = createForestItem
M.createRandomForestRect = createRandomForestRect
M.createRandomForestRadial = createRandomForestRadial
M.updateForestItem = updateForestItem
M.getForestItemTypes = getForestItemTypes
M.getForestItemDict = getForestItemDict
M.getForestItemsPolygon = getForestItemsPolygon
M.getForestItemsRadius = getForestItemsRadius
M.deleteForestItem = deleteForestItem
M.clearForest = clearForest

return M