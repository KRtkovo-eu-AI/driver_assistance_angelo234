-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

if not scenetree.dynamicDecals_PersistMan then
  local persistenceMgr = PersistenceManager()
  persistenceMgr:registerObject('dynamicDecals_PersistMan')
end

local history = require("editor/api/history")()

local M = {}

local logTag = "editor_api_dynamicDecals"

local app = nil
local decalProjection = nil

local sizeStep = 0.025
local rotationStep = math.pi / 180 * 15

local layerStack = {}
local layerCount = 0

local doPath = false
local currentPathLayer = nil

local highlightedDecal = {}

local mirrorPlaneOffset = 0

-- data we wanna reconstruct after deserialization
local reconstructData = {materialIdx = 0}

local layerNameBuildString = "$type Layer"

local randomStringChars = "0123456789ABCDEF"
local function getRandomString(length)
  local res = ""
  for i = 1, length do
    local rdnNumber = math.random(1, #randomStringChars)
    res = res .. string.sub(randomStringChars, rdnNumber, rdnNumber)
  end
  return res
end

-- pseudo unique, should be enough though
local function getRandomUid()
  return getRandomString(4) .. "-" .. getRandomString(4)
end

local function getChildrenCountRec(children)
  if not children then return 0 end
  local res = 0
  for k, v in ipairs(children) do
    res = res + 1 + getChildrenCountRec(v.children)
  end
  return res
end

local function createLayerName(layerData)
  local function splitLayerTypeIdentifier(s)
    local function split(char)
      return " " .. char
    end
    return (s:gsub("[A-Z]", split):gsub("^.", string.upper))
  end

  local name = layerNameBuildString
  -- layer type
  local layerTypeString = splitLayerTypeIdentifier(M.layerTypesMap[layerData.type])

  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@type)([^{]*)})")
  while startIndex do
    if layerTypeString == "" then
      name = name:gsub("{[^}]*@type[^{]*}", layerTypeString, 1)
    else
      local newString = groupA .. layerTypeString .. groupC
      name = name:gsub("{[^}]*@type[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@type)([^{]*)})")
  end
  name = name:gsub('@type', layerTypeString)

  -- texture path filename
  -- color
  local colorMapTextureString = ""
  if layerData.type == M.layerTypes.decal or layerData.type == M.layerTypes.brushStroke or layerData.type == M.layerTypes.path then
    local _, filename, extension = path.split(layerData.decalColorTexturePath)
    colorMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
  elseif layerData.type == M.layerTypes.textureFill then
    local _, filename, extension = path.split(layerData.fillTexturePath)
    if filename ~= "" then
      colorMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
    end
  end
  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@colormap)([^{]*)})")
  while startIndex do
    if colorMapTextureString == "" then
      name = name:gsub("{[^}]*@colormap[^{]*}", colorMapTextureString, 1)
    else
      local newString = groupA .. colorMapTextureString .. groupC
      name = name:gsub("{[^}]*@colormap[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@colormap)([^{]*)})")
  end
  name = name:gsub('@colormap', colorMapTextureString)

  -- normal
  local normalMapTextureString = ""
  if layerData.type == M.layerTypes.decal or layerData.type == M.layerTypes.brushStroke or layerData.type == M.layerTypes.path then
    local _, filename, extension = path.split(layerData.decalNormalTexturePath)
    if filename ~= "" then
      normalMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
    end
  end
  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@normalmap)([^{]*)})")
  while startIndex do
    if normalMapTextureString == "" then
      name = name:gsub("{[^}]*@normalmap[^{]*}", normalMapTextureString, 1)
    else
      local newString = groupA .. normalMapTextureString .. groupC
      name = name:gsub("{[^}]*@normalmap[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@normalmap)([^{]*)})")
  end
  name = name:gsub('@normalmap', normalMapTextureString)

  -- metallic
  local metallicMapTextureString = ""
  if layerData.type == M.layerTypes.decal or layerData.type == M.layerTypes.brushStroke or layerData.type == M.layerTypes.path then
    local _, filename, extension = path.split(layerData.decalMetallicTexturePath)
    if filename ~= "" then
      metallicMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
    end
  end
  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@metallicmap)([^{]*)})")
  while startIndex do
    if metallicMapTextureString == "" then
      name = name:gsub("{[^}]*@metallicmap[^{]*}", metallicMapTextureString, 1)
    else
      local newString = groupA .. metallicMapTextureString .. groupC
      name = name:gsub("{[^}]*@metallicmap[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@metallicmap)([^{]*)})")
  end
  name = name:gsub('@metallicmap', metallicMapTextureString)

  -- roughness
  local roughnessMapTextureString = ""
  if layerData.type == M.layerTypes.decal or layerData.type == M.layerTypes.brushStroke or layerData.type == M.layerTypes.path then
    local _, filename, extension = path.split(layerData.decalRoughnessTexturePath)
    if filename ~= "" then
      roughnessMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
    end
  end
  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@roughnessmap)([^{]*)})")
  while startIndex do
    if roughnessMapTextureString == "" then
      name = name:gsub("{[^}]*@roughnessmap[^{]*}", roughnessMapTextureString, 1)
    else
      local newString = groupA .. roughnessMapTextureString .. groupC
      name = name:gsub("{[^}]*@roughnessmap[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@roughnessmap)([^{]*)})")
  end
  name = name:gsub('@roughnessmap', roughnessMapTextureString)

  -- alpha
  local alphaMapTextureString = ""
  if layerData.type == M.layerTypes.decal or layerData.type == M.layerTypes.brushStroke or layerData.type == M.layerTypes.path then
    local _, filename, extension = path.split(layerData.decalAlphaTexturePath)
    if filename ~= "" then
      alphaMapTextureString = string.sub(filename, 1, #filename - (#extension + 1))
    end
  end
  local startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@alphamap)([^{]*)})")
  while startIndex do
    if alphaMapTextureString == "" then
      name = name:gsub("{[^}]*@alphamap[^{]*}", alphaMapTextureString, 1)
    else
      local newString = groupA .. alphaMapTextureString .. groupC
      name = name:gsub("{[^}]*@alphamap[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@alphamap)([^{]*)})")
  end
  name = name:gsub('@alphamap', alphaMapTextureString)

  -- uid
  while startIndex do
    if layer.uid == "" then
      name = name:gsub("{[^}]*@uid[^{]*}", layer.uid, 1)
    else
      local newString = groupA .. layer.uid .. groupC
      name = name:gsub("{[^}]*@uid[^{]*}", newString, 1)
    end
    startIndex, endIndex, capture, groupA, groupB, groupC = name:find("({([^}]*)(@uid)([^{]*)})")
  end
  name = name:gsub('@uid', layerData.uid)

  return name
end

local function addLayer(layerData, pos, parentUid, ignoreHook)
  local pos = pos or (parentUid and #M.getLayerByUid(parentUid).children + 1 or #layerStack + 1)
  table.insert(parentUid and M.getLayerByUid(parentUid).children or layerStack, pos or #layerStack + 1, layerData)
  layerCount = layerCount + 1

  if ignoreHook == nil or ignoreHook == false then
    extensions.hook("dynamicDecals_onLayerAdded", layerData.uid)
  end
end

local function deleteLayer(id, parentUid, ignoreHook)
  local layer = M.getLayerById(id, parentUid)
  local layerUid = layer.uid
  layerCount = layerCount - 1 - getChildrenCountRec(layer.children)
  local tableToRemoveFrom = parentUid and M.getLayerByUid(parentUid).children or layerStack
  if id > #tableToRemoveFrom then
    print(string.format("%s.deleteLayer(): Can't delete layer. Id [%d] is out of bounds!", logTag, id))
    return
  end
  table.remove(tableToRemoveFrom, id)

  if ignoreHook == nil or ignoreHook == false then
    extensions.hook("dynamicDecals_onLayerDeleted", layerUid)
  end
end

local function serializeLayer(layer)
  local lyr = {}
  if layer.type == M.layerTypes.decal then
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation
    lyr["alphaMaskScale"] = layer.alphaMaskScale:toTable()
    lyr["alphaMaskOffset"] = layer.alphaMaskOffset:toTable()
    lyr["blendMode"] = layer.blendMode
    lyr["decalPos"] = layer.decalPos
    lyr["decalNorm"] = layer.decalNorm
    lyr["camDirection"] = layer.camDirection
    lyr["camPosition"] = layer.camPosition
    lyr["color"] = layer.color:toTable()
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId
    lyr["colorTextureScale"] = layer.colorTextureScale:toTable()
    lyr["cursorPosScreenUv"] = layer.cursorPosScreenUv
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = layer.decalScale
    lyr["decalSkew"] = layer.decalSkew:toTable()
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = layer.decalGradientColorTopLeft:toTable()
    lyr["decalGradientColorTopRight"] = layer.decalGradientColorTopRight:toTable()
    lyr["decalGradientColorBottomLeft"] = layer.decalGradientColorBottomLeft:toTable()
    lyr["decalGradientColorBottomRight"] = layer.decalGradientColorBottomRight:toTable()
    lyr["decalFontPath"] = layer.decalFontPath
    lyr["decalFontCharacter"] = layer.decalFontCharacter
    lyr["decalUv"] = layer.decalUv:toTable()
    lyr["meshes"] = layer.meshes
    lyr["metallicIntensity"] = layer.metallicIntensity
    lyr["mirrored"] = layer.mirrored
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal
    lyr["name"] = layer.name
    lyr["normalIntensity"] = layer.normalIntensity
    lyr["roughnessIntensity"] = layer.roughnessIntensity
    lyr["type"] = layer.type
    lyr["useSurfaceNormal"] = layer.useSurfaceNormal
    lyr["viewToScreen"] = {
      layer.viewToScreen:getColumn4F(0):toTable(),
      layer.viewToScreen:getColumn4F(1):toTable(),
      layer.viewToScreen:getColumn4F(2):toTable(),
      layer.viewToScreen:getColumn4F(3):toTable()
    }
    lyr["worldToViewToScreen"] = {
      layer.worldToViewToScreen:getColumn4F(0):toTable(),
      layer.worldToViewToScreen:getColumn4F(1):toTable(),
      layer.worldToViewToScreen:getColumn4F(2):toTable(),
      layer.worldToViewToScreen:getColumn4F(3):toTable()
    }
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY
    lyr["useZBufferDepth"] = layer.useZBufferDepth
    lyr["zBufferDepth"] = layer.zBufferDepth
    lyr["useLockedSurfaceNormal"] = layer.useLockedSurfaceNormal
    lyr["surfaceNormal"] = layer.surfaceNormal
    -- sdf
    lyr["sdfEnabled"] = layer.sdfEnabled
    lyr["sdfThickness"] = layer.sdfThickness
    lyr["sdfSoftness"] = layer.sdfSoftness
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness
    lyr["sdfOutlineColor"] = layer.sdfOutlineColor:toTable()
  elseif layer.type == M.layerTypes.fill then
    lyr["name"] = layer.name
    lyr["blendMode"] = layer.blendMode
    lyr["color"] = layer.color:toTable()
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId
    lyr["type"] = layer.type
  elseif layer.type == M.layerTypes.textureFill then
    lyr["name"] = layer.name
    lyr["blendMode"] = layer.blendMode
    lyr["color"] = layer.color:toTable()
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId
    lyr["scale"] = layer.scale:toTable()
    lyr["offset"] = layer.offset:toTable()
    lyr["type"] = layer.type
    lyr["fillTexturePath"] = layer.fillTexturePath
  elseif layer.type == M.layerTypes.group then
    lyr["name"] = layer.name
    lyr["type"] = layer.type
  elseif layer.type == M.layerTypes.brushStroke then
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation
    lyr["alphaMaskScale"] = layer.alphaMaskScale:toTable()
    lyr["alphaMaskOffset"] = layer.alphaMaskOffset:toTable()
    lyr["blendMode"] = layer.blendMode
    lyr["camDirection"] = layer.camDirection
    lyr["camPosition"] = layer.camPosition
    lyr["color"] = layer.color:toTable()
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId
    lyr["colorTextureScale"] = layer.colorTextureScale:toTable()
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = layer.decalScale
    lyr["decalSkew"] = layer.decalSkew:toTable()
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = layer.decalGradientColorTopLeft:toTable()
    lyr["decalGradientColorTopRight"] = layer.decalGradientColorTopRight:toTable()
    lyr["decalGradientColorBottomLeft"] = layer.decalGradientColorBottomLeft:toTable()
    lyr["decalGradientColorBottomRight"] = layer.decalGradientColorBottomRight:toTable()
    lyr["decalFontPath"] = layer.decalFontPath
    lyr["decalFontCharacter"] = layer.decalFontCharacter
    lyr["decalUv"] = layer.decalUv:toTable()
    lyr["interpolationSteps"] = layer.interpolationSteps
    lyr["metallicIntensity"] = layer.metallicIntensity
    lyr["mirrored"] = layer.mirrored
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal
    lyr["name"] = layer.name
    lyr["normalIntensity"] = layer.normalIntensity
    lyr["roughnessIntensity"] = layer.roughnessIntensity
    lyr["type"] = layer.type
    lyr["viewToScreen"] = {
      layer.viewToScreen:getColumn4F(0):toTable(),
      layer.viewToScreen:getColumn4F(1):toTable(),
      layer.viewToScreen:getColumn4F(2):toTable(),
      layer.viewToScreen:getColumn4F(3):toTable()
    }
    lyr["worldToViewToScreen"] = {
      layer.worldToViewToScreen:getColumn4F(0):toTable(),
      layer.worldToViewToScreen:getColumn4F(1):toTable(),
      layer.worldToViewToScreen:getColumn4F(2):toTable(),
      layer.worldToViewToScreen:getColumn4F(3):toTable()
    }
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY
    lyr["zBufferDepth"] = layer.zBufferDepth
    lyr["dataPoints"] = layer.dataPoints
    -- sdf
    lyr["sdfEnabled"] = layer.sdfEnabled
    lyr["sdfThickness"] = layer.sdfThickness
    lyr["sdfSoftness"] = layer.sdfSoftness
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness
    lyr["sdfOutlineColor"] = layer.sdfOutlineColor:toTable()
  elseif layer.type == M.layerTypes.path then
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation
    lyr["alphaMaskScale"] = layer.alphaMaskScale:toTable()
    lyr["alphaMaskOffset"] = layer.alphaMaskOffset:toTable()
    lyr["blendMode"] = layer.blendMode
    lyr["camDirection"] = layer.camDirection
    lyr["camPosition"] = layer.camPosition
    lyr["color"] = layer.color:toTable()
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId
    lyr["colorTextureScale"] = layer.colorTextureScale:toTable()
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = layer.decalScale
    lyr["decalSkew"] = layer.decalSkew:toTable()
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = layer.decalGradientColorTopLeft:toTable()
    lyr["decalGradientColorTopRight"] = layer.decalGradientColorTopRight:toTable()
    lyr["decalGradientColorBottomLeft"] = layer.decalGradientColorBottomLeft:toTable()
    lyr["decalGradientColorBottomRight"] = layer.decalGradientColorBottomRight:toTable()
    lyr["decalFontPath"] = layer.decalFontPath
    lyr["decalFontCharacter"] = layer.decalFontCharacter
    lyr["decalUv"] = layer.decalUv:toTable()
    lyr["interpolationSteps"] = layer.interpolationSteps
    lyr["metallicIntensity"] = layer.metallicIntensity
    lyr["mirrored"] = layer.mirrored
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal
    lyr["name"] = layer.name
    lyr["normalIntensity"] = layer.normalIntensity
    lyr["orientDecals"] = layer.orientDecals
    lyr["pathType"] = layer.pathType
    lyr["roughnessIntensity"] = layer.roughnessIntensity
    lyr["text"] = layer.text
    lyr["fontPath"] = layer.fontPath
    lyr["type"] = layer.type
    lyr["viewToScreen"] = {
      layer.viewToScreen:getColumn4F(0):toTable(),
      layer.viewToScreen:getColumn4F(1):toTable(),
      layer.viewToScreen:getColumn4F(2):toTable(),
      layer.viewToScreen:getColumn4F(3):toTable()
    }
    lyr["worldToViewToScreen"] = {
      layer.worldToViewToScreen:getColumn4F(0):toTable(),
      layer.worldToViewToScreen:getColumn4F(1):toTable(),
      layer.worldToViewToScreen:getColumn4F(2):toTable(),
      layer.worldToViewToScreen:getColumn4F(3):toTable()
    }
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY
    lyr["zBufferDepth"] = layer.zBufferDepth
    lyr["textCharacterPositions"] = layer.textCharacterPositions
    lyr["dataPoints"] = layer.dataPoints
    -- sdf
    lyr["sdfEnabled"] = layer.sdfEnabled
    lyr["sdfThickness"] = layer.sdfThickness
    lyr["sdfSoftness"] = layer.sdfSoftness
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness
    lyr["sdfOutlineColor"] = layer.sdfOutlineColor:toTable()
  elseif layer.type == M.layerTypes.linkedSet then
    lyr["name"] = layer.name
    lyr["type"] = layer.type
    lyr["properties"] = layer.properties
  end

  -- shared properties
  lyr["uid"] = nil
  lyr["enabled"] = layer.enabled
  lyr["locked"] = layer.locked

  lyr["children"] = {}
  if layer.children and #layer.children then
    for _, childLayer in ipairs(layer.children) do
      local childData = serializeLayer(childLayer)
      table.insert(lyr["children"], childData)
    end
  end

  if layer.mask then
    lyr["mask"] = {}
    lyr["mask"].enabled = layer.mask.enabled
    lyr["mask"]["layers"] = {}

    for _, maskLayer in ipairs(layer.mask.layers) do
      local maskData = serializeLayer(maskLayer)
      table.insert(lyr["mask"]["layers"], maskData)
    end
  end

  return lyr
end

local function deserializeLayer(layer)
  local lyr = {}
  if layer.type == M.layerTypes.decal then
    local mat = MatrixF(true)
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode or 0
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel or 3
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity or 1.0
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert or false
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation or 0.0
    lyr["alphaMaskScale"] = (layer.alphaMaskScale and Point2F.fromTable(layer.alphaMaskScale) or Point2F(1,1))
    lyr["alphaMaskOffset"] = (layer.alphaMaskOffset and Point2F.fromTable(layer.alphaMaskOffset) or Point2F(0,0))
    lyr["blendMode"] = layer.blendMode
    lyr["decalPos"] = layer.decalPos and vec3(layer.decalPos.x, layer.decalPos.y, layer.decalPos.z) or nil
    lyr["decalNorm"] = layer.decalNorm and vec3(layer.decalNorm.x, layer.decalNorm.y, layer.decalNorm.z) or nil
    lyr["camDirection"] = vec3(layer.camDirection.x, layer.camDirection.y, layer.camDirection.z)
    lyr["camPosition"] = vec3(layer.camPosition.x, layer.camPosition.y, layer.camPosition.z)
    lyr["color"] = Point4F.fromTable(layer.color)
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId or 0
    lyr["colorTextureScale"] = (layer.colorTextureScale and Point2F.fromTable(layer.colorTextureScale) or Point2F(1,1))
    lyr["cursorPosScreenUv"] = layer.cursorPosScreenUv
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = vec3(layer.decalScale.x, layer.decalScale.y, layer.decalScale.z)
    lyr["decalSkew"] = Point2F.fromTable(layer.decalSkew)
    lyr["decalUv"] = Point2F.fromTable(layer.decalUv)
    lyr["decalFontPath"] = layer.decalFontPath or "/"
    lyr["decalFontCharacter"] = layer.decalFontCharacter or "A"
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = ColorI.fromTable(layer.decalGradientColorTopLeft)
    lyr["decalGradientColorTopRight"] = ColorI.fromTable(layer.decalGradientColorTopRight)
    lyr["decalGradientColorBottomLeft"] = ColorI.fromTable(layer.decalGradientColorBottomLeft)
    lyr["decalGradientColorBottomRight"] = ColorI.fromTable(layer.decalGradientColorBottomRight)
    lyr["metallicIntensity"] = layer.metallicIntensity or 1.0
    lyr["mirrored"] = layer.mirrored or false
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal or false
    lyr["meshes"] = layer.meshes or nil
    lyr["name"] = (layer.name or string.format("%s", "Decal Layer"))
    lyr["normalIntensity"] = layer.normalIntensity or 1.0
    lyr["roughnessIntensity"] = layer.roughnessIntensity or 1.0
    lyr["type"] = layer.type
    lyr["useSurfaceNormal"] = layer.useSurfaceNormal == nil and true or layer.useSurfaceNormal
    mat:setColumn4F(0, Point4F.fromTable(layer.viewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.viewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.viewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.viewToScreen[4]))
    lyr["viewToScreen"] = mat:copy()
    mat:setColumn4F(0, Point4F.fromTable(layer.worldToViewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.worldToViewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.worldToViewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.worldToViewToScreen[4]))
    lyr["worldToViewToScreen"] = mat:copy()
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX or false
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY or false
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX == nil and true or layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY == nil and true or layer.wrapColorTextureY
    lyr["useZBufferDepth"] = layer.useZBufferDepth or false
    lyr["zBufferDepth"] = layer.zBufferDepth or -1.0
    lyr["useLockedSurfaceNormal"] = layer.useLockedSurfaceNormal or false
    lyr["surfaceNormal"] = layer.surfaceNormal and vec3(layer.decalPos.x, layer.decalPos.y, layer.decalPos.z) or vec3(0, 0, 0)
    -- sdf
    if layer.sdfEnabled or (layer.sdfEnabled == nil and layer.sdfThickness) then
      lyr["sdfEnabled"] = true
    else
      lyr["sdfEnabled"] = false
    end
    lyr["sdfThickness"] = layer.sdfThickness or 0.75
    lyr["sdfSoftness"] = layer.sdfSoftness or 0.05
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness or 0.4
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness or 0.1
    lyr["sdfOutlineColor"] = ColorI.fromTable(layer.sdfOutlineColor or {255,0,0,255})
  elseif layer.type == M.layerTypes.fill then
    lyr["name"] = (layer.name or string.format("%s", "Fill Layer"))
    lyr["blendMode"] = layer.blendMode
    lyr["color"] = Point4F.fromTable(layer.color)
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId or 1
    lyr["type"] = layer.type
  elseif layer.type == M.layerTypes.textureFill then
    lyr["name"] = (layer.name or string.format("%s", "Texture Fill Layer"))
    lyr["blendMode"] = layer.blendMode
    lyr["color"] = Point4F.fromTable(layer.color)
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId or 1
    lyr["scale"] = (layer.scale and Point2F.fromTable(layer.scale) or Point2F(1,1))
    lyr["offset"] = (layer.offset and Point2F.fromTable(layer.offset) or Point2F(0,0))
    lyr["type"] = layer.type
    lyr["fillTexturePath"] = layer.fillTexturePath
  elseif layer.type == M.layerTypes.group then
    lyr["name"] = (layer.name or string.format("%s", "Group Layer"))
    lyr["type"] = layer.type
  elseif layer.type == M.layerTypes.brushStroke then
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode or 0
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel or 3
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity or 1.0
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert or false
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation or 0.0
    lyr["alphaMaskScale"] = (layer.alphaMaskScale and Point2F.fromTable(layer.alphaMaskScale) or Point2F(1,1))
    lyr["alphaMaskOffset"] = (layer.alphaMaskOffset and Point2F.fromTable(layer.alphaMaskOffset) or Point2F(0,0))
    lyr["blendMode"] = layer.blendMode
    lyr["camDirection"] = vec3(layer.camDirection.x, layer.camDirection.y, layer.camDirection.z)
    lyr["camPosition"] = vec3(layer.camPosition.x, layer.camPosition.y, layer.camPosition.z)
    lyr["color"] = Point4F.fromTable(layer.color)
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId or 0
    lyr["colorTextureScale"] = (layer.colorTextureScale and Point2F.fromTable(layer.colorTextureScale) or Point2F(1,1))
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = vec3(layer.decalScale.x, layer.decalScale.y, layer.decalScale.z)
    lyr["decalSkew"] = Point2F.fromTable(layer.decalSkew)
    lyr["decalUv"] = Point2F.fromTable(layer.decalUv)
    lyr["decalFontPath"] = layer.decalFontPath or "/"
    lyr["decalFontCharacter"] = layer.decalFontCharacter or "A"
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = ColorI.fromTable(layer.decalGradientColorTopLeft)
    lyr["decalGradientColorTopRight"] = ColorI.fromTable(layer.decalGradientColorTopRight)
    lyr["decalGradientColorBottomLeft"] = ColorI.fromTable(layer.decalGradientColorBottomLeft)
    lyr["decalGradientColorBottomRight"] = ColorI.fromTable(layer.decalGradientColorBottomRight)
    lyr["interpolationSteps"] = layer.interpolationSteps or 2
    lyr["metallicIntensity"] = layer.metallicIntensity or 1.0
    lyr["mirrored"] = layer.mirrored or false
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal or false
    lyr["name"] = (layer.name or string.format("%s", "Group Layer"))
    lyr["normalIntensity"] = layer.normalIntensity or 1.0
    lyr["roughnessIntensity"] = layer.roughnessIntensity or 1.0
    lyr["type"] = layer.type
    local mat = MatrixF(true)
    mat:setColumn4F(0, Point4F.fromTable(layer.viewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.viewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.viewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.viewToScreen[4]))
    lyr["viewToScreen"] = mat
    mat = MatrixF(true)
    mat:setColumn4F(0, Point4F.fromTable(layer.worldToViewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.worldToViewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.worldToViewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.worldToViewToScreen[4]))
    lyr["worldToViewToScreen"] = mat
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX or false
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY or false
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX == nil and true or layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY == nil and true or layer.wrapColorTextureY
    lyr["zBufferDepth"] = layer.zBufferDepth or -1.0
    lyr["dataPoints"] = layer.dataPoints
    -- sdf
    if layer.sdfEnabled or (layer.sdfEnabled == nil and layer.sdfThickness) then
      lyr["sdfEnabled"] = true
    else
      lyr["sdfEnabled"] = false
    end
    lyr["sdfThickness"] = layer.sdfThickness or 0.75
    lyr["sdfSoftness"] = layer.sdfSoftness or 0.05
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness or 0.4
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness or 0.1
    lyr["sdfOutlineColor"] = ColorI.fromTable(layer.sdfOutlineColor or {255,0,0,255})
  elseif layer.type == M.layerTypes.path then
    lyr["alphaMaskBlendMode"] = layer.alphaMaskBlendMode or 0
    lyr["alphaMaskChannel"] = layer.alphaMaskChannel or 3
    lyr["alphaMaskIntensity"] = layer.alphaMaskIntensity or 1.0
    lyr["alphaMaskInvert"] = layer.alphaMaskInvert or false
    lyr["alphaMaskRotation"] = layer.alphaMaskRotation or 0.0
    lyr["alphaMaskScale"] = (layer.alphaMaskScale and Point2F.fromTable(layer.alphaMaskScale) or Point2F(1,1))
    lyr["alphaMaskOffset"] = (layer.alphaMaskOffset and Point2F.fromTable(layer.alphaMaskOffset) or Point2F(0,0))
    lyr["blendMode"] = layer.blendMode
    lyr["camDirection"] = vec3(layer.camDirection.x, layer.camDirection.y, layer.camDirection.z)
    lyr["camPosition"] = vec3(layer.camPosition.x, layer.camPosition.y, layer.camPosition.z)
    lyr["color"] = Point4F.fromTable(layer.color)
    lyr["colorPaletteMapId"] = layer.colorPaletteMapId or 0
    lyr["colorTextureScale"] = (layer.colorTextureScale and Point2F.fromTable(layer.colorTextureScale) or Point2F(1,1))
    lyr["decalAlphaTexturePath"] = layer.decalAlphaTexturePath
    lyr["decalColorTexturePath"] = layer.decalColorTexturePath
    lyr["decalMetallicTexturePath"] = layer.decalMetallicTexturePath
    lyr["decalNormalTexturePath"] = layer.decalNormalTexturePath
    lyr["decalRotation"] = layer.decalRotation
    lyr["decalRoughnessTexturePath"] = layer.decalRoughnessTexturePath
    lyr["decalScale"] = vec3(layer.decalScale.x, layer.decalScale.y, layer.decalScale.z)
    lyr["decalSkew"] = Point2F.fromTable(layer.decalSkew)
    lyr["decalUv"] = Point2F.fromTable(layer.decalUv)
    lyr["decalFontPath"] = layer.decalFontPath or "/"
    lyr["decalFontCharacter"] = layer.decalFontCharacter or "A"
    lyr["decalUseGradientColor"] = layer.decalUseGradientColor
    lyr["decalGradientColorTopLeft"] = ColorI.fromTable(layer.decalGradientColorTopLeft)
    lyr["decalGradientColorTopRight"] = ColorI.fromTable(layer.decalGradientColorTopRight)
    lyr["decalGradientColorBottomLeft"] = ColorI.fromTable(layer.decalGradientColorBottomLeft)
    lyr["decalGradientColorBottomRight"] = ColorI.fromTable(layer.decalGradientColorBottomRight)
    lyr["interpolationSteps"] = layer.interpolationSteps or 5
    lyr["metallicIntensity"] = layer.metallicIntensity or 1.0
    lyr["mirrored"] = layer.mirrored or false
    lyr["flipMirroredDecal"] = layer.flipMirroredDecal or false
    lyr["name"] = (layer.name or string.format("%s", "Group Layer"))
    lyr["normalIntensity"] = layer.normalIntensity or 1.0
    lyr["orientDecals"] = layer.orientDecals == nil and true or layer.orientDecals
    lyr["pathType"] = layer.pathType or 1
    lyr["roughnessIntensity"] = layer.roughnessIntensity or 1.0
    lyr["text"] = layer.text or ""
    lyr["fontPath"] = layer.fontPath or ""
    lyr["type"] = layer.type
    local mat = MatrixF(true)
    mat:setColumn4F(0, Point4F.fromTable(layer.viewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.viewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.viewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.viewToScreen[4]))
    lyr["viewToScreen"] = mat
    mat = MatrixF(true)
    mat:setColumn4F(0, Point4F.fromTable(layer.worldToViewToScreen[1]))
    mat:setColumn4F(1, Point4F.fromTable(layer.worldToViewToScreen[2]))
    mat:setColumn4F(2, Point4F.fromTable(layer.worldToViewToScreen[3]))
    mat:setColumn4F(3, Point4F.fromTable(layer.worldToViewToScreen[4]))
    lyr["worldToViewToScreen"] = mat
    lyr["wrapAlphaMaskX"] = layer.wrapAlphaMaskX or false
    lyr["wrapAlphaMaskY"] = layer.wrapAlphaMaskY or false
    lyr["wrapColorTextureX"] = layer.wrapColorTextureX == nil and true or layer.wrapColorTextureX
    lyr["wrapColorTextureY"] = layer.wrapColorTextureY == nil and true or layer.wrapColorTextureY
    lyr["zBufferDepth"] = layer.zBufferDepth or -1.0
    lyr["textCharacterPositions"] = layer.textCharacterPositions
    lyr["dataPoints"] = layer.dataPoints
    -- sdf
    if layer.sdfEnabled or (layer.sdfEnabled == nil and layer.sdfThickness) then
      lyr["sdfEnabled"] = true
    else
      lyr["sdfEnabled"] = false
    end
    lyr["sdfThickness"] = layer.sdfThickness or 0.75
    lyr["sdfSoftness"] = layer.sdfSoftness or 0.05
    lyr["sdfOutlineThickness"] = layer.sdfOutlineThickness or 0.4
    lyr["sdfOutlineSoftness"] = layer.sdfOutlineSoftness or 0.1
    lyr["sdfOutlineColor"] = ColorI.fromTable(layer.sdfOutlineColor or {255,0,0,255})
  elseif layer.type == M.layerTypes.linkedSet then
    lyr["name"] = (layer.name or string.format("%s", "Linked Set Layer"))
    lyr["type"] = layer.type
    lyr["properties"] = layer.properties or {}
  end

  --shared properties
  lyr["uid"] = getRandomUid()
  lyr["enabled"] = layer.enabled
  lyr["locked"] = layer.locked or false

  lyr["children"] = {}
  if layer.children and #layer.children > 0 then
    for _, childData in ipairs(layer.children) do
      local childLayer = deserializeLayer(childData)
      table.insert(lyr["children"], childLayer)
    end
  end

  if layer.mask then
    lyr["mask"] = {}
    lyr["mask"].enabled = layer.mask.enabled
    lyr["mask"]["layers"] = {}

    for _, maskLayer in ipairs(layer.mask.layers) do
      local maskData = deserializeLayer(maskLayer)
      table.insert(lyr["mask"]["layers"], maskData)
    end
  end

  layerCount = layerCount + 1

  return lyr
end

-- decal projection : public interface
M.debug = false
M.ready = false
M.blendModes = {
  {name = "Clear", value = 0},
  {name = "Normal", value = 1},
  {name = "Screen / Add", value = 2},
  {name = "Subtract", value = 3},
  {name = "Multiply", value = 4},
  {name = "Eraser", value = 5}
}
M.blendModesMap = {}
for k, v in pairs(M.blendModes) do M.blendModesMap[v.name] = v.value end
M.textureResolutions = {
  {name = "512", value = 512},
  {name = "1024", value = 1024},
  {name = "2048", value = 2048},
  {name = "4096", value = 4096},
}
M.settingsFlags = {
  None = {name = "None", value = 0},
  UseMousePos = {name = "Use Mouse Pos", value = bit.lshift(1,2), description = "Sets the cursor position for the decal projection either at the mouse cursor position or at the center of the screen."},
  UseGradientColor = {name = "Use Gradient Color", value = bit.lshift(1,3)},
  AlphaMaskInvert = {name = "Invert Alpha Mask", value = bit.lshift(1,4)},
  WrapAlphaMaskX = {name = "Wrap Alpha Mask Horizontally", value = bit.lshift(1,5)},
  WrapAlphaMaskY = {name = "Wrap Alpha Mask Vertically", value = bit.lshift(1,6)},
  WrapColorTextureX = {name = "Wrap Color Texture Horizontally", value = bit.lshift(1,7)},
  WrapColorTextureY = {name = "Wrap Color Texture Vertically", value = bit.lshift(1,8)}
}
M.loadingModes = {
  {key="Overwrite", value = 0},
  {key="Append", value = 1},
}
local loadingMode = {}
for _, mode in ipairs(M.loadingModes) do loadingMode[mode.key] = mode.value end
M.layerTypes = {
  decal = 0,
  fill = 1,
  textureFill = 2,
  group = 3,
  brushStroke = 4,
  path = 5,
  linkedSet = 6,
}
M.layerTypesMap = {}
for k, v in pairs(M.layerTypes) do M.layerTypesMap[v] = k end
M.layerBakingStatusCode = {
  Ok                          = 0,
  Warning                     = bit.lshift(1,0),
  Error                       = bit.lshift(1,1),
  LayerStackDataTypeIncorrect = bit.lshift(1,2),
  LayerDataTypeIncorrect      = bit.lshift(1,3),
  TextureFileMissing          = bit.lshift(1,4),
  FontFileMissing             = bit.lshift(1,5),
}
M.pathTypes = {
  Linear = 0,
  Bezier = 1
}
M.projectDynamicDecals = true
M.types = {
  bool = 0,
  int = 1,
  float = 2,
  Point2F = 3,
  Point3F = 4,
  Point4F = 5,
  string = 6,
  Texture = 7,
  File = 8,
  ColorI = 9,
  MultiColor = 10,
}
M.typesMap = {}
for k, v in pairs(M.types) do M.typesMap[v] = k end
M.widgetTypes = {
  [M.types.bool] = {
    Default = 0,
    Checkbox = 1,
  },
  [M.types.int] = {
    Default = 0,
    Input = 1,
    Slider = 2,
    Drag = 3,
    Combo = 4,
  },
  [M.types.float] = {
    Default = 0,
    Input = 1,
    Slider = 2,
    Drag = 3,
  },
  [M.types.Point2F] = {
    Default = 0,
    Input = 1,
    Slider = 2,
    Drag = 3,
  },
  [M.types.Point3F] = {
    Default = 0,
    Slider = 1,
    Color = 2,
  },
  [M.types.Point4F] = {
    Default = 0,
    Slider = 1,
    Color = 2,
  },
  [M.types.string] = {
    Default = 0,
    Input = 1,
    InputMultiline = 2,
  },
  [M.types.Texture] = {
    Default = 0,
    File = 1,
    ImageButton = 2,
  },
  [M.types.File] = {
    Default = 0,
    File = 1,
  },
  [M.types.ColorI] = {
    Default = 0,
    Color = 1,
  },
  [M.types.MultiColor] = {
    Default = 0,
    ColorGradient = 1,
  },
}
-- id, name, description, type, default, min (for widget), max(for widget) [, format (for widget)] [,getMod (fn)] [, setMod (fn)] [, widget type]
M.properties = {
  Decal = {
    {id = "alphaMaskBlendMode", name = "Alpha Mask Blend Mode", description = "", type = M.types.int, default = 0, min = 0, max = 1, widgetType = M.widgetTypes[M.types.int].Combo, options = {"multiply", "add"}},
    {id = "alphaMaskChannel", name = "Alpha Mask Channel", description = "", type = M.types.int, default = 3, min = 0, max = 3, widgetType = M.widgetTypes[M.types.int].Combo, options = {"red", "green", "blue", "alpha"}},
    {id = "alphaMaskIntensity", name = "Alpha Mask Intensity", description = "", type = M.types.float, default = 1, min = 0, max = 2, widgetType = M.widgetTypes[M.types.float].Slider},
    {id = "alphaMaskInvert", name = "Alpha Mask Invert", description = "", type = M.types.bool, default = false, widgetType = M.widgetTypes[M.types.bool].Checkbox},
    {id = "alphaMaskRotation", name = "Alpha Mask Rotation", description = "", type = M.types.float, default = 0, min = 0, max = 360, dragSpeed = 0.5, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.1f deg", getMod = function(val) return val * 180 / math.pi end, setMod = function(val) return val / 180 * math.pi end},
    {id = "alphaMaskScale", name = "Alpha Mask Scale", description = "", type = M.types.Point2F, default = {1,1}, min = {0.01,0.01}, max = {6,6}, format = "%.2f", lockRatio = true, widgetType = M.widgetTypes[M.types.Point2F].Slider},
    {id = "alphaMaskOffset", name = "Alpha Mask Offset", description = "", type = M.types.Point2F, default = {0,0}, min = {-1, -1}, max = {1,1}, format = "%.2f", lockRatio = false, widgetType = M.widgetTypes[M.types.Point2F].Slider},
    -- disabled for the time being
    -- {id = "blendMode", name = "Blend Mode", description = "", type = M.types.int, default = 1, min = 0, max = 5, widgetType = M.widgetTypes[M.types.int].Combo, options = {"Clear", "Normal", "Screen / Add", "Subtract", "Multiply", "Erase"}},
    -- {id = "camDirection", name = "camDirection", description = "", type = M.types.int, default = 0, min = 0, max = 10}, = layer.camDirection
    {id = "camPosition", name = "Camera Position", description = "", type = M.types.Point3F, default = {0,0,0}, min = {-5, -5, -5}, max = {5, 5, 5}, widgetType = M.widgetTypes[M.types.Point3F].Slider},
    {id = "color", name = "Color", description = "", type = M.types.Point4F, default = {1,1,1,1}, min = {0, 0, 0, 0}, max = {1, 1, 1, 1}, widgetType = M.widgetTypes[M.types.Point4F].Color},
    {id = "colorPaletteMapId", name = "Color Palette Map Id", description = "", type = M.types.int, default = 0, min = 0, max = 3, widgetType = M.widgetTypes[M.types.int].Combo, options = {"zero", "one", "two", "three"}},
    {id = "colorTextureScale", name = "Color Texture Scale", description = "", type = M.types.Point2F, default = {1, 1}, min = {0.01, 0.01}, max = {6, 6}, widgetType = M.widgetTypes[M.types.Point2F].Slider, format = "%.2f", lockRatio = true},
    {id = "cursorPosScreenUv", name = "Cursor Pos Screen Space", description = "", type = M.types.Point2F, default = {0.5, 0.5}, min = {0.0, 0.0}, max = {1, 1}, widgetType = M.widgetTypes[M.types.Point2F].Slider, format = "%.2f"},
    {id = "decalAlphaTexturePath", name = "Alpha Mask Texture", description = "", type = M.types.Texture, default = "/art/dynamicDecals/textures/_one.png", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    {id = "decalColorTexturePath", name = "Color Texture", description = "", type = M.types.Texture, default = "/art/dynamicDecals/textures/_one.png", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    -- disabled for time being
    -- {id = "decalMetallicTexturePath", name = "Metallic Texture", description = "", type = M.types.Texture, default = "/", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    -- disabled for time being
    -- {id = "decalNormalTexturePath", name = "Normal Texture", description = "", type = M.types.Texture, default = "/art/dynamicDecals/textures/_normal.png", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    {id = "decalRotation", name = "Rotation", description = "", type = M.types.float, default = 0, min = 0, max = 360, dragSpeed = 0.5, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.1f deg", getMod = function(val) return val * 180 / math.pi end, setMod = function(val) return val / 180 * math.pi end},
    -- disabled for time being
    -- {id = "decalRoughnessTexturePath", name = "Roughness Texture", description = "", type = M.types.Texture, default = "/art/dynamicDecals/textures/_one.png", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    {id = "decalScale", name = "Scale", description = "Scale of the decal. The y-component is for debug purposes only (WIP).", type = M.types.Point3F, default = {0.5, 1.0, 0.5}, min = {0.0001, 0.0001, 0.0001}, max = {6, 6, 6}, widgetType = M.widgetTypes[M.types.Point3F].Slider, format = "%.3f", lockRatio = true},
    {id = "decalSkew", name = "Skew", description = "", type = M.types.Point2F, default = {0, 0}, min = {-2, -2}, max = {2, 2}, format = "%.2f", lockRatio = false},
    {id = "decalUseGradientColor", name = "Use Gradient Color", description = "", type = M.types.bool, default = false, widgetType = M.widgetTypes[M.types.bool].Checkbox},
    {id = "decalGradientColorTopLeft", name = "Gradient Color Top Left", description = "", type = M.types.ColorI, default = {255, 255, 255, 255}, min = {0,0,0,0}, max = {255, 255, 255, 255}, widgetType = M.widgetTypes[M.types.ColorI].Color},
    {id = "decalGradientColorTopRight", name = "Gradient Color Top Right", description = "", type = M.types.ColorI, default = {255, 255, 255, 255}, min = {0,0,0,0}, max = {255, 255, 255, 255}, widgetType = M.widgetTypes[M.types.ColorI].Color},
    {id = "decalGradientColorBottomLeft", name = "Gradient Color Bottom Left", description = "", type = M.types.ColorI, default = {255, 255, 255, 255}, min = {0,0,0,0}, max = {255, 255, 255, 255}, widgetType = M.widgetTypes[M.types.ColorI].Color},
    {id = "decalGradientColorBottomRight", name = "Gradient Color Bottom Right", description = "", type = M.types.ColorI, default = {255, 255, 255, 255}, min = {0,0,0,0}, max = {255, 255, 255, 255}, widgetType = M.widgetTypes[M.types.ColorI].Color},
    {
      id = "decalGradientColor",
      name = "Gradient Color",
      description = "",
      type = M.types.MultiColor,
      default = {{255,102,0,255}, {60,179,113, 255}, {90,234,255,255}, {255,0,0,255}},
      min = {0,0,0,0},
      max = {255, 255, 255, 255},
      widgetType = M.widgetTypes[M.types.MultiColor].ColorGradient
    },
    {id = "decalUv", name = "Decal UV", description = "", type = M.types.Point2F, default = {1, 1}, min = {-1, -1}, max = {1, 1}, format = "%.1f", lockRatio = false},
    {id = "enabled", name = "Enabled", description = "", type = M.types.bool, default = true, widgetType = M.widgetTypes[M.types.bool].Checkbox},
    {id = "fontPath", mapId = "decal_fontPath", name = "Font Path", description = "", type = M.types.File, default = "/", defaultDir = "/ui/common/", fileTypes = {{"TTF files", {".ttf", ".TTF"}}}},
    {id = "metallicIntensity", name = "Metallic Intensity", description = "", type = M.types.float, default = 1, min = 0, max = 1, format = "%.2f", widgetType = M.widgetTypes[M.types.float].Slider},
    {id = "mirrored", name = "Mirrored", description = "", type = M.types.bool, default = false},
    {id = "flipMirroredDecal", name = "Flip Mirrored Decal", description = "", type = M.types.bool, default = false},
    {id = "name", name = "Name", description = "", type = M.types.string, default = "Layer"},
    -- disabled for time being
    -- {id = "normalIntensity", name = "Normal Intensity", description = "", type = M.types.float, default = 1, min = 0, max = 1, format = "%.2f", widgetType = M.widgetTypes[M.types.float].Slider},
    {id = "roughnessIntensity", name = "Roughness Intensity", description = "", type = M.types.float, default = 1, min = 0, max = 1, format = "%.2f", widgetType = M.widgetTypes[M.types.float].Slider},
    -- {id = "type", name = "type", description = "", type = M.types.int, default = 0, min = 0, max = 10}, = layer.type
    -- {id = "viewToScreen", name = "viewToScreen", description = "", type = M.types.int, default = 0, min = 0, max = 10}, = {
    --   layer.viewToScreen:getColumn4F(0):toTable(),
    --   layer.viewToScreen:getColumn4F(1):toTable(),
    --   layer.viewToScreen:getColumn4F(2):toTable(),
    --   layer.viewToScreen:getColumn4F(3):toTable()
    -- }
    -- {id = "worldToViewToScreen", name = "worldToViewToScreen", description = "", type = M.types.int, default = 0, min = 0, max = 10}, = {
    --   layer.worldToViewToScreen:getColumn4F(0):toTable(),
    --   layer.worldToViewToScreen:getColumn4F(1):toTable(),
    --   layer.worldToViewToScreen:getColumn4F(2):toTable(),
    --   layer.worldToViewToScreen:getColumn4F(3):toTable()
    -- }
    {id = "useSurfaceNormal", name = "Use Surface Normal", description = "", type = M.types.bool, default = true},
    {id = "wrapAlphaMaskX", name = "Wrap Alpha Mask X", description = "", type = M.types.bool, default = false},
    {id = "wrapAlphaMaskY", name = "Wrap Alpha Mask Y", description = "", type = M.types.bool, default = false},
    {id = "wrapColorTextureX", name = "Wrap Color Texture X", description = "", type = M.types.bool, default = true},
    {id = "wrapColorTextureY", name = "Wrap Color Texture Y", description = "", type = M.types.bool, default = true},
    {id = "useZBufferDepth", name = "Use Z-Buffer Depth", description = "", type = M.types.bool, default = false, widgetType = M.widgetTypes[M.types.bool].Checkbox},
    -- {id = "zBufferDepth", name = "zBufferDepth", description = "", type = M.types.int, default = 0, min = 0, max = 10}, = layer.zBufferDepth
    {id = "sdfEnabled", name = "SDF Enabled", description = "", type = M.types.bool, default = false},
    {id = "sdfThickness", name = "SDF Thickness", description = "", type = M.types.float, default = 0.75, min = 0, max = 1, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.2f"},
    {id = "sdfSoftness", name = "SDF Softness", description = "", type = M.types.float, default = 0.05, min = 0, max = 1, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.2f"},
    {id = "sdfOutlineThickness", name = "SDF Outline Thickness", description = "", type = M.types.float, default = 0.4, min = 0, max = 1, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.2f"},
    {id = "sdfOutlineSoftness", name = "SDF Outline Softness", description = "", type = M.types.float, default = 0.1, min = 0, max = 1, widgetType = M.widgetTypes[M.types.float].Slider, format = "%.2f"},
    {id = "sdfOutlineColor", name = "SDF Outline Color", description = "", type = M.types.ColorI, default = {255, 0, 0, 255}, min = {0,0,0,0}, max = {255, 255, 255, 255}, widgetType = M.widgetTypes[M.types.ColorI].Color},

  },
  ["Path Layer"] = {
    {id = "pathType", name = "Path Type", description = "Curve type", type = M.types.int, default = 1, min = 0, max = 1, widgetType = M.widgetTypes[M.types.int].Combo, options = {"Linear", "Bezier"}},
    {id = "text", name = "Text", description = "The characters of the text property replace the decal color texture.", type = M.types.string, default = ""},
    {id = "fontPath", mapId = "path_fontPath", name = "Font Path", description = "", type = M.types.File, default = "/", defaultDir = "/ui/common/", fileTypes = {{"TTF files", {".ttf", ".TTF"}}}},
    {id = "orientDecals", name = "Orient Decals", description = "If enabled decals are oriented towards the next decal in the path. Overrides 'decal rotation'.", type = M.types.bool, default = true},
    {id = "interpolationSteps", mapId = "path_interpolationSteps", name = "Interpolation Steps", description = "Linear path type: Number of decals in-between control points.\nBezier path type: Number of decals in-between the first and last control point.\n\nDisabled while a text is set since the number of characters determines the number of decals.", type = M.types.int, default = 3, min = 0, max = 10, widgetType = M.widgetTypes[M.types.int].Input},
  },
  ["Brush Stroke Layer"] = {
    {id = "interpolationSteps", mapId = "brushStroke_interpolationSteps", name = "Interpolation Steps", description = "Number of decals placed in-between decals of the brush stroke in order to make the stroke appear smoother.", type = M.types.int, default = 3, min = 0, max = 10, widgetType = M.widgetTypes[M.types.int].Input},
  },
  ["Fill Layer"] = {
    -- disabled for the time being
    -- {id = "blendMode", mapId = "fill_blendMode", name = "Blend Mode", description = "", type = M.types.int, default = 1, min = 0, max = 5, widgetType = M.widgetTypes[M.types.int].Combo, options = {"Clear", "Normal", "Screen / Add", "Subtract", "Multiply", "Erase"}},
    {id = "color", mapId = "fill_color", name = "Color", description = "", type = M.types.Point4F, default = {1,1,1,1}, min = {0, 0, 0, 0}, max = {1, 1, 1, 1}, widgetType = M.widgetTypes[M.types.Point4F].Color},
    {id = "colorPaletteMapId", mapId = "fill_colorPaletteMapId", name = "Color Palette Map Id", description = "", type = M.types.int, default = 0, min = 0, max = 3, widgetType = M.widgetTypes[M.types.int].Combo, options = {"zero", "one", "two", "three"}},
  },
  ["Texture Fill Layer"] = {
    -- disabled for the time being
    -- {id = "blendMode", mapId = "textureFill_blendMode", name = "Blend Mode", description = "", type = M.types.int, default = 1, min = 0, max = 5, widgetType = M.widgetTypes[M.types.int].Combo, options = {"Clear", "Normal", "Screen / Add", "Subtract", "Multiply", "Erase"}},
    {id = "color", mapId = "textureFill_color", name = "Color", description = "", type = M.types.Point4F, default = {1,1,1,1}, min = {0, 0, 0, 0}, max = {1, 1, 1, 1}, widgetType = M.widgetTypes[M.types.Point4F].Color},
    {id = "colorPaletteMapId", mapId = "textureFill_colorPaletteMapId", name = "Color Palette Map Id", description = "", type = M.types.int, default = 0, min = 0, max = 3, widgetType = M.widgetTypes[M.types.int].Combo, options = {"zero", "one", "two", "three"}},
    {id = "fillTexturePath", name = "Fill Texture", description = "", type = M.types.Texture, default = "/art/dynamicDecals/textures/_one.png", defaultDir = "/art/dynamicDecals/textures/", widgetType = M.widgetTypes[M.types.Texture].ImageButton, fileTypes = {{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}}},
    {id = "scale", name = "Scale", description = "", type = M.types.Point2F, default = {1,1}, min = {0.01, 0.01}, max = {6, 6}, format = "%.2f", lockRatio = true, widgetType = M.widgetTypes[M.types.Point2F].Slider},
    {id = "offset", name = "Offset", description = "", type = M.types.Point2F, default = {0,0}, min = {-1.0, -1.0}, max = {1.0, 1.0}, format = "%.4f", widgetType = M.widgetTypes[M.types.Point2F].Slider},
  },
}
M.propertiesMap = {}
for _, cat in pairs(M.properties) do
  for _, prop in ipairs(cat) do
    if not prop.value then prop.value = shallowcopy(prop.default) end
    M.propertiesMap[prop.id] = prop

    if prop.mapId then
      M.propertiesMap[prop.mapId] = prop
    end
  end
end

M.getRandomUid = function()
  return getRandomUid()
end

M.setLayerNameBuildString = function(value_string)
  layerNameBuildString = value_string
end

M.getLayerNameBuildString = function()
  return layerNameBuildString
end

M.setProjectDynamicDecalsState = function(value)
  M.projectDynamicDecals = value
end

local function deepcopyLayer(layer)
  local a = serializeLayer(layer)
  return deserializeLayer(a)
end

local function addMaskDecal_Undo(actionData)
  local layer = M.getLayerByUid(actionData.baseLayerUid)
  if not layer then
    print(string.format("%s.addMaskDecal(): Couldn't find layer for layerUid '%s'. Couldn't undo 'addMaskDecal' action.", logTag, actionData.baseLayerUid))
    return
  end

  if layer.mask and layer.mask.layers then
    table.remove(layer.mask.layers, #layer.mask.layers)

    if #layer.mask.layers == 0 then
      layer.mask = nil
    end
  end

  M.reprojectLayers()
end

local function addMaskDecal_Redo(actionData)
  local layer = M.getLayerByUid(actionData.baseLayerUid)
  if not layer then
    print(string.format("%s.addMaskDecal(): Couldn't find layer for layerUid '%s'. No layer mask decal has been added.", logTag, actionData.baseLayerUid))
    return
  end

  if not layer.mask then layer.mask = {enabled = true, layers = {}} end
  table.insert(layer.mask.layers, actionData.decalData)

  M.reprojectLayers()
end

M.addMaskDecal = function(layerUid_string)
  if not layerUid_string then
    print(string.format("%s.addMaskDecal(): 'layerUid' must be set. No layer mask decal has been added.", logTag))
    return
  end
  local layer = M.getLayerByUid(layerUid_string)
  if not layer then
    print(string.format("%s.addMaskDecal(): Couldn't find layer for layerUid '%s'. No layer mask decal has been added.", logTag, layerUid_string))
    return
  end

  local decalData = decalProjection:getDecalData(app:getCameraM(), app:getProjM())
  decalData.uid = getRandomUid()
  decalData.name = string.format("%s", "Decal Mask Layer")
  decalData.enabled = true
  decalData.locked = false
  decalData.children = {}
  if M.debug then
    print(string.format("%s.addMaskDecal()\ndecalData:\n%s\n### ######## ###", logTag, dumps(decalData)))
  end

  local actionData = {
    baseLayerUid = layerUid_string,
    decalData = decalData
  }
  history:commitAction(
    "Add Decal Mask Layer",
    actionData,
    addMaskDecal_Undo,
    addMaskDecal_Redo
  )
end

local function addDecal_Undo(actionData)
  deleteLayer(#layerStack, actionData.parentUid)
  M.reprojectLayers()
end

local function addDecal_Redo(actionData)
  local decals = {}
  table.insert(decals, actionData)
  M.bakeLayers(decals)
  addLayer(actionData, nil, actionData.parentUid)
end

M.addDecal = function(params)
  params = params or {}

  local decalData = decalProjection:getDecalData(app:getCameraM(), app:getProjM())
  decalData.uid = getRandomUid()
  decalData.name = params.name or createLayerName(decalData)
  decalData.enabled = true
  decalData.locked = false
  decalData.parentUid = params.parentUid or nil
  decalData.children = {}

  -- add all enabled meshes to decalData.meshes
  if M.areAllMeshesEnabled() == false then
    local vehicleObj = getPlayerVehicle(0)
    if vehicleObj then

      local sMeshes = M.getShapeMeshes()
      local meshNames = {}
      for name, enabled in pairs(sMeshes) do
        if enabled then
          table.insert(meshNames, name)
        end
      end

      decalData.meshes = {
        [vehicleObj.jbeam] = meshNames
      }
    end
  end

  if M.debug then
    print(string.format("%s.addDecal()\ndecalData:\n%s\n### ######## ###", logTag, dumps(decalData)))
  end

  history:commitAction(
    "Add Decal",
    decalData,
    addDecal_Undo,
    addDecal_Redo
  )

  return decalData
end

M.addPathDataPoint = function()
  local layerData = decalProjection:getDecalData(app:getCameraM(), app:getProjM())
  if not currentPathLayer then
    -- TODO: Not sure if it's a hack but this fixes path layers for the time being
    layerData.decalPos = nil
    layerData.decalNorm = nil
    layerData.uid = getRandomUid()
    layerData.type = M.layerTypes.path
    layerData.children = {}
    layerData.dataPoints = {{x = layerData.cursorPosScreenUv.x, y = layerData.cursorPosScreenUv.y}}
    layerData.cursorPosScreenUv = nil
    layerData.orientDecals = M.getOrientPathDecals()
    layerData.pathType = M.getPathType()
    layerData.interpolationSteps = M.getPathLayerInterpolationSteps()
    layerData.text = M.getPathLayerText()
    layerData.fontPath = M.getPathLayerFontPath()
    layerData.name = createLayerName(layerData)
    layerData.enabled = true
    layerData.locked = false
    if M.debug then
      print(string.format("%s.addPathDataPoint()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
    end
    addLayer(layerData)

    currentPathLayer = layerData.uid
  else
    table.insert(M.getLayerByUid(currentPathLayer).dataPoints, {x = layerData.cursorPosScreenUv.x, y = layerData.cursorPosScreenUv.y})
  end

  M.reprojectLayers()
  if layerData.name then return layerData end
end

local function addFillLayer_Undo(actionData)
  deleteLayer(#layerStack, actionData.parentUid)
  M.reprojectLayers()
end

local function addFillLayer_Redo(actionData)
  local decals = {}
  table.insert(decals, actionData)
  M.bakeLayers(decals)
  addLayer(actionData, nil, actionData.parentUid)
end

M.addFillLayer = function(params)
  params = params or {}

  local layerData = {
    blendMode = M.blendModesMap.Normal,
    color = M.getFillLayerColor(),
    colorPaletteMapId = M.getFillLayerColorPaletteMapId(),
    enabled = true,
    locked = false,
    uid = getRandomUid(),
    type = M.layerTypes.fill,
    children = {},
  }
  layerData.name = params.name or createLayerName(layerData)
  layerData.parentUid = params.parentUid or nil

  if M.debug then
    print(string.format("%s.addFillLayer()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
  end

  history:commitAction(
    string.format("Add Fill Layer (%s)", layerData.uid),
    layerData,
    addFillLayer_Undo,
    addFillLayer_Redo
  )

  return layerData
end

local function addTextureFillLayer_Undo(actionData)
  deleteLayer(#layerStack)
  M.reprojectLayers()
end

local function addTextureFillLayer_Redo(actionData)
  local decals = {}
  table.insert(decals, actionData)
  M.bakeLayers(decals)
  addLayer(actionData)
end

M.addTextureFillLayer = function(params)
  params = params or {}

  local layerData = {
    blendMode = M.blendModesMap.Normal,
    color = M.getFillLayerColor(),
    colorPaletteMapId = M.getFillLayerColorPaletteMapId(),
    fillTexturePath = M.getFillTexturePath(),
    scale = M.getTextureFillLayerScale(),
    offset = M.getTextureFillOffset(),
    enabled = true,
    locked = false,
    uid = getRandomUid(),
    type = M.layerTypes.textureFill,
    children = {},
  }
  layerData.name = params.name or createLayerName(layerData)
  layerData.parentUid = params.parentUid or nil

  if M.debug then
    print(string.format("%s.addTextureFillLayer()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
  end

  history:commitAction(
    "Add Texture Fill Layer",
    layerData,
    addTextureFillLayer_Undo,
    addTextureFillLayer_Redo
  )

  return layerData
end

local function addGroup_Undo(actionData)
  deleteLayer(#layerStack, actionData.parentUid)
end

local function addGroup_Redo(actionData)
  addLayer(actionData, nil, actionData.parentUid)
end

M.addGroup = function(params)
  params = params or {}

  local layerData = {
    enabled = true,
    locked = false,
    uid = getRandomUid(),
    type = M.layerTypes.group,
    children = {},
  }
  layerData.name = params.name or createLayerName(layerData)
  layerData.parentUid = params.parentUid or nil

  if M.debug then
    print(string.format("%s.addGroup()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
  end

  history:commitAction(
    "Add Group Layer",
    layerData,
    addGroup_Undo,
    addGroup_Redo
  )

  return layerData
end

local function addLinkedSet_Undo(actionData)
  deleteLayer(#layerStack)
end

local function addLinkedSet_Redo(actionData)
  addLayer(actionData)
end

M.addLinkedSet = function(params)
  params = params or {}

  local layerData = {
    enabled = true,
    locked = false,
    uid = getRandomUid(),
    type = M.layerTypes.linkedSet,
    properties = {},
    children = {},
  }
  layerData.name = params.name or createLayerName(layerData)
  layerData.parentUid = params.parentUid or nil

  if M.debug then
    print(string.format("%s.addLinkedSet()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
  end

  history:commitAction(
    "Add Linked Set Layer",
    layerData,
    addLinkedSet_Undo,
    addLinkedSet_Redo
  )

  return layerData
end

local function addBrushStrokeLayer_Undo(actionData)
  deleteLayer(#layerStack)
  M.reprojectLayers()
end

local function addBrushStrokeLayer_Redo(actionData)
  addLayer(actionData)
end

M.addBrushStrokeLayer = function(params)
  params = params or {}

  local layerData = M.getBrushStrokeLayerData()
  layerData.uid = getRandomUid()
  layerData.locked = false
  layerData.type = M.layerTypes.brushStroke
  layerData.children = {}
  layerData.name = params.name or createLayerName(layerData)
  layerData.parentUid = params.parentUid or nil

  if M.debug then
    print(string.format("%s.addBrushStrokeLayer()\nlayerData:\n%s\n### ######## ###", logTag, dumps(layerData)))
  end

  history:commitAction(
    "Add Brush Stroke Layer",
    layerData,
    addBrushStrokeLayer_Undo,
    addBrushStrokeLayer_Redo
  )

  return layerData
end

local function findPartMaterials()
  local data = core_vehicle_manager.getPlayerVehicleData()
  if not data then return nil end
  local paintDesignSlot = data.config.partsTree.children["paint_design"]
  if not paintDesignSlot then return nil end

  local id = paintDesignSlot.chosenPartName
  local part = id and data.vdata.activePartsData[id]
  if part and part.dynDecalMaterials then
    return part.dynDecalMaterials
  end
  return nil
end

local function setVehicleMaterialJob(job)
  if not decalProjection then return end
  coroutine.yield()

  -- set default material based on the current vehicle
  local vehicleObj = getPlayerVehicle(0)
  if vehicleObj then

    local partMaterials = findPartMaterials()
    if partMaterials and #partMaterials > 0 then
      local materials = {}
      for _, mat in ipairs(partMaterials) do
        materials[mat] = true
      end

      M.clearMaterialIdx()
      local mNames = M.getShapeMaterialNames()
      for k, materialName in pairs(mNames) do
        if materials[materialName] then
          M.addMaterialIdx(k)
          if M.debug then
            print(string.format("%s - Material '%s' has been added", logTag, materialName))
          end
        end
      end
    else
      local vehicleName = (vehicleObj and vehicleObj.jbeam or "")
      local mat0, mat1, mat2 = nil, nil, nil
      local mNames = M.getShapeMaterialNames()
      for k, materialName in pairs(mNames) do
        if materialName == vehicleName then
          mat0 = k
          M.setMaterialIdx(mat0)
          if M.debug then
            print(string.format("%s - Material set to '%s'", logTag, materialName))
          end
          return
        end
        if string.endswith(materialName, "main") then
          mat1 = {k, materialName}
        end
        if string.endswith(materialName, "body") then
          mat2 = {k, materialName}
        end
      end

      if mat1 then
        M.setMaterialIdx(mat1[1])
        if M.debug then
          print(string.format("%s - Material set to '%s'", logTag, mat1[2]))
        end
        return
      end
      if mat2 then
        M.setMaterialIdx(mat2[1])
        if M.debug then
          print(string.format("%s - Material set to '%s'", logTag, mat2[2]))
        end
        return
      end
      print(string.format("%s - Not able to set the default material", logTag))

    end
  end
end

M.updateVehicleMaterials = function()
  if not decalProjection then return end
  core_jobsystem.create(setVehicleMaterialJob, 1)
end

M.setup = function()
  -- has been setup before already
  if decalProjection then return end

  if DecalShapeRenderApp and DecalShapeRenderApp.getActiveApp and DecalShapeRenderApp:getActiveApp() then
    local appPtr = DecalShapeRenderApp:getActiveApp()
    decalProjection = appPtr:getDecalProjection()
    app = {}
    app.getCameraM = function(self)
      return appPtr:getCameraM()
    end
    app.getProjM = function(self)
      return appPtr:getProjM()
    end
    app.setTextureSet = function(self, id, set)
      appPtr:setTextureSet(set)
    end
    app.onUpdate = function(self)
    end

    M.ready = true
  else
    -- set shape transform
    decalProjection = DecalProjection("", Point2I(4096, 2048), 1, reconstructData.materialIdx or 0)
    app = {}
    app.getCameraM = function(self)
      local veh = getPlayerVehicle(0)
      -- convert camera from world to vehicle space
      if veh then
        return veh:getRefNodeMatrix():fullInverse():mul(getCameraTransform())
      else
        return MatrixF(true)
      end
    end
    app.getProjM = function(self)
      return getCameraProjMatrix()
    end
    app.setTextureSet = function(self, id, set)
      local veh = getPlayerVehicle(0)
      if veh then
        veh:setTextureSet(id, set)
        decalProjection:setShape(veh:getDecalProjectionShape())
      end
    end
    app.onUpdate = function(self)
      -- we need to update all pointers to be sure old shape is alive
      local veh = getPlayerVehicle(0)
      if veh then
        app:setTextureSet("@DynamicTexture", app.textureSet)
        decalProjection:setShape(veh:getDecalProjectionShape())
        decalProjection:setWorldTransform(veh:getRefNodeMatrix())
        decalProjection:setRenderTransform(veh:getRefNodeMatrix():copy():setPosition(vec3(0, 0, 0)):inverse())
        decalProjection:setMirrorOffset(veh:getSpawnLocalAABB():getCenter().x + mirrorPlaneOffset)
      end
    end

    M.ready = true
  end

  app.textureSet = decalProjection:getTextureSet()
  app:onUpdate()

  M.updateVehicleMaterials()

  -- Add a base layer if there's none yet.
  -- TODO: Do this in the tool, this shouldn't be an api thing.
  if #layerStack == 0 then
    M.addFillLayer()
  end
end

M.disableDecalHighlighting = function()
  local decal = {uid = ""}
  highlightedDecal = decal
  decalProjection:setHighlightedDecal(decal)
end

M.setLayerVisibility = function(layerUid_string, visibility_bool)
  if visibility_bool == nil then print(string.format("%s.setLayerVisibility(): 'visibility_bool' argument must be given.", logTag)) return end
  local layer = M.getLayerByUid(layerUid_string)
  layer.enabled = visibility_bool == true and true or false
  if layer.uid == highlightedDecal.uid then
    M.disableDecalHighlighting()
  end
  M.reprojectLayers()
end

local function toggleLayerVisibility_UndoRedo(actionData)
  local layer = M.getLayerByUid(actionData.layerUid)
  layer.enabled = not layer.enabled
  if layer.uid == highlightedDecal.uid then
    M.disableDecalHighlighting()
  end
  M.reprojectLayers()
end

M.toggleLayerVisibility = function(layerUid_string)
  local layerData = M.getLayerByUid(layerUid_string)
  local res = not layerData.enabled

  history:commitAction(
    string.format("Toggle Layer Visibility: ", layerUid_string),
    {layerUid = layerUid_string},
    toggleLayerVisibility_UndoRedo,
    toggleLayerVisibility_UndoRedo
  )

  return res
end

M.getLayerStack = function()
  return layerStack
end

M.getLayerCount = function()
  return layerCount
end
---

-- Decal projection - public interface
M.changeDecalSize = function(increase_bool, step_number)
  local scale = decalProjection.decalScale
  local change = (step_number or sizeStep) * (increase_bool and 1 or -1)
  decalProjection.decalScale = vec3(scale.x + change, scale.y, scale.z + change)
end

M.changeDecalRotation = function(clockwise_bool, step_radian_number)
  local change = (step_radian_number or rotationStep) * (clockwise_bool and 1 or -1)
  decalProjection.decalRotation = decalProjection.decalRotation + change
end

M.bakeLayers = function(layers)
  local vehicleObj = getPlayerVehicle(0)
  if not vehicleObj then
    print(string.format("%s.bakeLayers(layers): Can't bake layers, vehicle's missing.", logTag))
    return
  end
  local res = decalProjection:bakeLayers(layers, (vehicleObj and vehicleObj.jbeam or nil))
  if M.debug then
    print(string.format("%s.bakeLayers(layers)\nresult:\n%s\n### ######## ###", logTag, dumps(res)))
  end
  return res
end

-- returns the highlightedDecal, returns nil if no decal is highlighted
M.getHighlightedLayer = function()
  -- highlightedDecal is never nil but a table with just an empty string for the name property - most probably a hack =)
  return (highlightedDecal.name and highlightedDecal or nil)
end

M.highlightLayer = function(layer_table)
  if layer_table.uid == highlightedDecal.uid then return end
  if M.debug then
    print(string.format("%s.highlightLayer(decal)\ndecal:\n%s\n### ######## ###", logTag, dumps(layer_table)))
  end
  highlightedDecal = layer_table
  decalProjection:setHighlightedDecal(layer_table)
end

M.highlightLayerByUid = function(layerUid_string)
  if layerUid_string == highlightedDecal.uid then return end
  local layer = M.getLayerByUid(layerUid_string)
  if M.debug then
    print(string.format("%s.highlightLayerByUid(decal)\ndecal:\n%s\n### ######## ###", logTag, dumps(layer)))
  end
  highlightedDecal = layer
  decalProjection:setHighlightedDecal(layer)
end

local function clearLayerStack_Undo(actionData)
  layerStack = actionData.layerStack
  layerCount = actionData.layerCount

  M.disableDecalHighlighting()
  M.reprojectLayers()
end

local function clearLayerStack_Redo(actionData)
  layerStack = {}
  layerCount = 0

  M.disableDecalHighlighting()
  decalProjection:clearBakedTextures()
end

M.clearLayerStack = function()
  history:commitAction(
    "Clear Baked Textures",
    {layerStack = layerStack, layerCount = layerCount},
    clearLayerStack_Undo,
    clearLayerStack_Redo
  )
end

M.reprojectLayers = function()
  local timer = hptimer()
  timer:stopAndReset()
  decalProjection:clearBakedTextures()
  local res = M.bakeLayers(layerStack)
  if not res then
    print(string.format("%s.reprojectLayers(): Failed", logTag))
    return
  end
  -- todo: check if there were issues during baking
  if res.status ~= M.layerBakingStatusCode.Ok then

  end

  local function checkLayers(layers)
    for k, layer in ipairs(layers) do
      if res.layers[layer.uid] then
        layer.status = res.layers[layer.uid]
      end
      if layer.children and #layer.children > 0 then
        checkLayers(layer.children)
      end
    end
  end

  checkLayers(layerStack)

  local reprojectionTime = timer:stopAndReset()
  res["reprojectionTime"] = (reprojectionTime / 1000)
  return res
end

local function getLayerByUidRec(layerUid, layers)
  local res
  for k, layer in ipairs(layers) do
    if layer.uid == layerUid then return layer end

    -- TODO: Added this to be able to edit mask layers as well. There should be a better way to do it e.g. only search mask layers as well when a flag is set.
    if layer.mask then
      for kk, maskLayer in ipairs(layer.mask.layers) do
        if maskLayer.uid == layerUid then return maskLayer end
      end
    end

    if layer.children then
      res = getLayerByUidRec(layerUid, layer.children)
      if res then return res end
    end
  end
  return res
end

M.getLayerByUid = function(layerUid_string)
  local res = getLayerByUidRec(layerUid_string, layerStack)
  if not res then
    print(string.format("%s.getLayerByUid(layerUid_string): No layer found with given uid '%s'", logTag, layerUid_string))
  end

  return res
end

M.getLayerById = function(layerId_number, parentUid_string)
  return (parentUid_string and M.getLayerByUid(parentUid_string).children[layerId_number] or layerStack[layerId_number])
end

local function _setLayerInCollection(collection, layerUid, layerData)
  for k, layer in ipairs(collection) do
    if layer.uid == layerUid then
      collection[k] = layerData
      extensions.hook("dynamicDecals_onLayerUpdated", layerUid)
      return
    end

    -- TODO: Added this to be able to edit mask layers as well. There should be a better way to do it e.g. only search mask layers as well when a flag is set.
    if layer.mask then
      for kk, maskLayer in ipairs(layer.mask.layers) do
        if maskLayer.uid == layerUid then
          layer.mask.layers[kk] = layerData
          extensions.hook("dynamicDecals_onLayerUpdated", layerUid)
          return
        end
      end
    end

    if layer.children then
      _setLayerInCollection(layer.children, layerUid, layerData)
    end

  end
end

local function setLayer_Undo(actionData)
  _setLayerInCollection(layerStack, actionData.layerUid, actionData.fromLayerData)

  if actionData.doReproject then
    M.reprojectLayers()
  end
end

local function setLayer_Redo(actionData)
  _setLayerInCollection(layerStack, actionData.layerUid, actionData.toLayerData)

  if actionData.doReproject then
    M.reprojectLayers()
  end
end

M.setLayer = function(layerData_table, doReproject_bool)
  local fromLayer = M.getLayerByUid(layerData_table.uid)
  if not fromLayer then
    print(string.format("%s.setLayer(layerData_table, doReproject_bool): Couldn't find layer for layerUid '%s'. Can't update layer.", logTag, layerData_table.uid))
    return
  end

  local fromLayerData = deepcopy(fromLayer)
  local toLayerData = deepcopy(layerData_table)

  history:commitAction(
    "Set Layer",
    {
      layerUid = layerData_table.uid,
      fromLayerData = fromLayerData,
      toLayerData = toLayerData,
      doReproject = doReproject_bool
    },
    setLayer_Undo,
    setLayer_Redo
  )

  extensions.hook("dynamicDecals_setLayer", layerData_table.uid)
end

local function moveLayer_Undo(actionData)
  local item = layerStack[actionData.to]
  deleteLayer(actionData.to, actionData.toParentUid, true)
  addLayer(item, actionData.from, actionData.fromParentUid, true)
  M.reprojectLayers()
end

local function moveLayer_Redo(actionData)
  local layer = (actionData.fromParentUid and M.getLayerByUid(actionData.fromParentUid).children[actionData.from] or layerStack[actionData.from])
  deleteLayer(actionData.from, actionData.fromParentUid, true)
  addLayer(layer, actionData.to, actionData.toParentUid, true)
  M.reprojectLayers()
  extensions.hook("dynamicDecals_moveLayer", actionData.from, actionData.fromParentUid, actionData.to, actionData.toParentUid)
end

-- params
-- number from
-- string fromParentUid
-- number to
-- string toParentUid
M.moveLayer = function(from_number, fromParentUid_string, to_number, toParentUid_string)
  if M.debug then
    print(string.format("moveLayer, from: %d fromParentUid: %s, to: %d toParentUid: %s", from_number or -1, fromParentUid_string or "nil", to_number or -1, toParentUid_string or "nil"))
  end

  local layer = M.getLayerById(from_number, fromParentUid_string)
  -- Chech if user tries to make the layer its own child layer
  if layer.uid == toParentUid_string then
    print(string.format("Can't make a layer its own child. Aborting moving layer '%s'", layer.name))
    return
  end

  history:commitAction(
    "Move Layer",
    {from = from_number, fromParentUid = fromParentUid_string, to = to_number, toParentUid = toParentUid_string},
    moveLayer_Undo,
    moveLayer_Redo
  )
end

local function removeLayer(index_number, parentUid_string)
  local layerUid = M.getLayerById(index_number, parentUid_string).uid
  -- User manually deletes the current active path layer; it no longer exists now and hence can't bethe active one any longer
  if layerUid == currentPathLayer then
    currentPathLayer = nil
  end
  deleteLayer(index_number, parentUid_string)
  M.reprojectLayers()
end

local function removeLayer_Undo(actionData)
  addLayer(actionData.decalData, actionData.layerStackId, actionData.parentUid)
  M.reprojectLayers()
end

local function removeLayer_Redo(actionData)
  if highlightedDecal and (highlightedDecal.uid == M.getLayerById(actionData.layerStackId, actionData.parentUid).uid) then
    M.disableDecalHighlighting()
  end
  removeLayer(actionData.layerStackId, actionData.parentUid)
end

M.removeLayer = function(index_number, parentUid_string)
  history:commitAction(
    "Remove Layer",
    {
      decalData = M.getLayerById(index_number, parentUid_string),
      layerStackId = index_number,
      parentUid = parentUid_string
    },
    removeLayer_Undo,
    removeLayer_Redo
  )
end

-- TODO: Take mask layers into account
local function setRandomLayerUidRec(layer)
  layer.uid = getRandomUid()
  if layer.mask then
    for _, maskLayer in ipairs(layer.mask.layers) do
      maskLayer.uid = getRandomUid()
    end
  end
  if layer.children and #layer.children > 0 then
    for _, childLayer in ipairs(layer.children) do
      setRandomLayerUidRec(childLayer)
    end
  end
end

local function duplicateLayer_Undo(actionData)
  deleteLayer(actionData.layerStackId + 1, actionData.parentUid)
  M.reprojectLayers()
end

local function duplicateLayer_Redo(actionData)
  addLayer(actionData.layerData, actionData.layerStackId + 1, actionData.parentUid)
  layerCount = layerCount + getChildrenCountRec(actionData.layerData.children)
  M.reprojectLayers()
end

M.duplicateLayer = function(layerId_number, parentUid_string)
  local newLayerData = deepcopyLayer(M.getLayerById(layerId_number, parentUid_string))
  setRandomLayerUidRec(newLayerData)
  newLayerData.name = string.format("%s %s", newLayerData.name, "Copy")

  history:commitAction(
    "Duplicate Layer",
    {
      layerData = newLayerData,
      layerStackId = layerId_number,
      parentUid = parentUid_string
    },
    duplicateLayer_Undo,
    duplicateLayer_Redo
  )
end

local function duplicateAndMirrorLayer_Undo(actionData)
  deleteLayer(actionData.layerStackId + 1, actionData.parentUid)
  M.reprojectLayers()
end

local function duplicateAndMirrorLayer_Redo(actionData)
  addLayer(actionData.layerData, actionData.layerStackId + 1, actionData.parentUid)
  layerCount = layerCount + getChildrenCountRec(actionData.layerData.children)
  M.reprojectLayers()
end

M.duplicateAndMirrorLayer = function(layerId_number, parentUid_string, mirrorChildren)
  local newLayerData = deepcopyLayer(M.getLayerById(layerId_number, parentUid_string))
  newLayerData = decalProjection:mirrorLayerData(newLayerData, mirrorChildren)
  setRandomLayerUidRec(newLayerData)
  newLayerData.name = string.format("%s %s", newLayerData.name, "Mirrored Copy")

  history:commitAction(
    "Duplicate Layer",
    {
      layerData = newLayerData,
      layerStackId = layerId_number,
      parentUid = parentUid_string
    },
    duplicateAndMirrorLayer_Undo,
    duplicateAndMirrorLayer_Redo
  )
end

M.getSettings = function()
  return decalProjection:getSettings()
end

M.isDecalGradientColorEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.UseGradientColor.value) == M.settingsFlags.UseGradientColor.value)
end

M.isAlphaMaskInvertEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.AlphaMaskInvert.value) == M.settingsFlags.AlphaMaskInvert.value)
end

M.isWrapAlphaMaskXEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.WrapAlphaMaskX.value) == M.settingsFlags.WrapAlphaMaskX.value)
end

M.isWrapAlphaMaskYEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.WrapAlphaMaskY.value) == M.settingsFlags.WrapAlphaMaskY.value)
end

M.isWrapColorTextureXEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.WrapColorTextureX.value) == M.settingsFlags.WrapColorTextureX.value)
end

M.isWrapColorTextureYEnabled = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.WrapColorTextureY.value) == M.settingsFlags.WrapColorTextureY.value)
end

M.isUseMousePos = function()
  return (bit.band(decalProjection:getSettings(), M.settingsFlags.UseMousePos.value) == M.settingsFlags.UseMousePos.value)
end

M.toggleSetting = function(value_int)
  return decalProjection:toggleSetting(value_int)
end

M.getBlendMode = function()
  return decalProjection.blendMode
end

M.setBlendMode = function(blendMode_int)
  decalProjection.blendMode = blendMode_int
end

M.exportTextures = function(directoryPath, exportName, extension)
  return decalProjection:exportTextures(directoryPath, exportName, extension)
end

M.exportLayerMask = function(layer, directory, filename, extension)
  decalProjection:exportLayerMask(layer, directory, filename, extension)
end

M.exportSkin = function(vehicleName, skinName)
  -- local directory = "/vehicles/" .. vehicleName .. "/" .. skinName .. "/"
  -- Export in mods folder for the time being
  local modDirectory = "/mods/unpacked/" .. skinName .. "/vehicles/" .. vehicleName .. "/" .. skinName .. "/"
  local directory = "/vehicles/" .. vehicleName .. "/" .. skinName .. "/"
  decalProjection:exportTextures(modDirectory, skinName, "png")

  local uvLayer = decalProjection:getUvLayer()
  local mNames = M.getShapeMaterialNames()
  local materials = {}
  for materialId, materialName in pairs(mNames) do
    materials[materialId] = materialName
  end

  local materialsUsed = {}
  for _, id in ipairs(M.getMaterialIndices()) do
    table.insert(materialsUsed, materials[id])
  end

  local found = 0
  for _, matName in ipairs(materialsUsed) do
    local matSkinPresetName = matName .. ".skin.dynamicTextures"
    local dynDecalMaterial = scenetree.findObject(matSkinPresetName)
    if dynDecalMaterial then
      found = found + 1

      local mat = createObject('Material')
      mat:assignFieldsFromObject(dynDecalMaterial)
      mat:setFilename(modDirectory .. vehicleName .. "_" .. skinName .. "_skin.materials.json")

      local skin = matName .. ".skin." .. skinName
      mat:setField('name', 0, skin)
      mat:setField('mapTo', 0, skin)

      for stage = 0, 3 do
        local value = mat:getField("baseColorMap", stage)
        if value and value ~= "" and value:startswith("@DynamicTexture") then
          mat:setField("baseColorMap", stage, directory .. skinName .. "_d.data.png")
          mat:setField("diffuseMapUseUV", stage, uvLayer)
        end
        -- value = mat:getField("normalMap", stage)
        -- if value ~= "" and value:startswith("@DynamicTexture") then
        --   mat:setField("normalMap", stage, directory .. skinName .. "_n.normal.png")
        --   mat:setField("normalMapUseUV", stage, uvLayer)
        -- end
        value = mat:getField("metallicMap", stage)
        if value and value ~= "" and value:startswith("@DynamicTexture") then
          mat:setField("metallicMap", stage, directory .. skinName .. "_m.data.png")
          mat:setField("metallicMapUseUV", stage, uvLayer)
        end
        value = mat:getField("roughnessMap", stage)
        if value and value ~= "" and value:startswith("@DynamicTexture") then
          mat:setField("roughnessMap", stage, directory .. skinName .. "_r.data.png")
          mat:setField("roughnessMapUseUV", stage, uvLayer)
        end
        value = mat:getField("colorPaletteMap", stage)
        if value and value ~= "" and value:startswith("@DynamicTexture") then
          mat:setField("colorPaletteMap", stage, directory .. skinName .. "_cp.data.png")
          mat:setField("colorPaletteMapUseUV", stage, uvLayer)
        end
      end

      mat.canSave = true
      mat:registerObject(skin)
      scenetree.dynamicDecals_PersistMan:setDirty(mat, '')
      scenetree.dynamicDecals_PersistMan:saveDirty()

    else
      print(string.format("%s : Can't find Dynamic Decals preset material: %s", logTag, matSkinPresetName))
    end
  end

  if found == 0 then
    print(string.format("%s : No materials skin preset has been found", logTag))
    return
  end

  local jbeamData = {
    [vehicleName .. "_skin_" .. skinName] = {
      information = {
        authors = "Dynamic Decals",
        name = skinName,
        value = 1337
      },
      slotType = "paint_design",
      globalSkin = skinName
    }
  }
  jsonWriteFile(modDirectory .. vehicleName .. "_" .. skinName .. "_skin.jbeam", jbeamData, true)

  print(string.format("%s : Skin files exported to '%s'", logTag, modDirectory))

  -- ToDo: Replace this when the mod manager can see the skin mod automatically
  core_modmanager.checkUpdate()
end

M.getDecalTexturePath = function(type_string)
  return decalProjection:getDecalTexturePath(type_string)
end

M.setDecalTexturePath = function(type_string, path_string)
  decalProjection:setDecalTexturePath(type_string, path_string)
end

M.getFillTexturePath = function()
  return decalProjection:getFillTexturePath()
end

M.setFillTexturePath = function(path_string)
  decalProjection:setFillTexturePath(path_string)
end

M.getDecalColor = function()
  return decalProjection.decalColor
end

M.setDecalColor = function(value_Point4F)
  decalProjection.decalColor = value_Point4F
end

M.getDecalScale = function()
  return decalProjection.decalScale
end

M.setDecalScale = function(value_vec3)
  decalProjection.decalScale = value_vec3
end

M.getDecalRotation = function()
  return decalProjection.decalRotation
end

M.setDecalRotation = function(value_radian_number)
  decalProjection.decalRotation = value_radian_number
end

M.drawDynamicTextures = function(maxImageWidth)
  decalProjection:drawDynamicTexturesInGui(maxImageWidth)
end

M.drawBakedTextures = function(maxImageWidth)
  decalProjection:drawBakedTexturesInGui(maxImageWidth)
end

M.drawHighlightTextures = function(maxImageWidth)
  decalProjection:drawHighlightTexturesInGui(maxImageWidth)
end

M.drawBrushInputTextures = function(maxImageWidth)
  decalProjection:drawBrushInputTexturesInGui(maxImageWidth)
end

M.drawMaskTextures = function(maxImageWidth)
  decalProjection:drawMaskTexturesInGui(maxImageWidth)
end

M.drawTextureSet = function(textureSet, name, maxImageWidth)
  decalProjection:drawTextureSetInGui(textureSet, name, maxImageWidth)
end

M.getEnabled = function()
  return decalProjection:getEnabled()
end

M.setEnabled = function(enabled)
  decalProjection:setEnabled(enabled)
  if enabled == false then
    decalProjection:flushDynamicTextures()
    decalProjection:combineTextures(app.textureSet)
  end
end

M.onEditorDeactivated = function()
  decalProjection:flushDynamicTextures()
  decalProjection:combineTextures(app.textureSet)
end

M.toggleEnabled = function()
  decalProjection:setEnabled(not decalProjection:getEnabled())
  if decalProjection:getEnabled() == false then
    decalProjection:flushDynamicTextures()
    decalProjection:combineTextures(app.textureSet)
  end
end

M.getFontTextureAtlasPath = function()
  return decalProjection:getFontTextureAtlasPath()
end

M.setFontTextureAtlasPath = function(path_string)
  decalProjection:setFontTextureAtlasPath(path_string)
end

M.getDecalUv = function()
  return decalProjection.decalUv
end

M.setDecalUv = function(value_Point2F)
  decalProjection.decalUv = value_Point2F
end

M.getDecalSkew = function()
  return decalProjection.decalSkew
end

M.setDecalSkew = function(value_Point2F)
  decalProjection.decalSkew = value_Point2F
end

M.getColorTextureScale = function()
  return decalProjection.colorTextureScale
end

M.setColorTextureScale = function(value_Point2F)
  decalProjection.colorTextureScale = value_Point2F
end

M.getAlphaMaskScale = function()
  return decalProjection.alphaMaskScale
end

M.setAlphaMaskScale = function(value_Point2F)
  decalProjection.alphaMaskScale = value_Point2F
end

M.getAlphaMaskOffset = function()
  return decalProjection.alphaMaskOffset
end

M.setAlphaMaskOffset = function(value_Point2F)
  decalProjection.alphaMaskOffset = value_Point2F
end

M.getAlphaMaskIntensity = function()
  return decalProjection.alphaMaskIntensity
end

M.setAlphaMaskIntensity = function(value_float)
  decalProjection.alphaMaskIntensity = value_float
end

M.getNormalIntensity = function()
  return decalProjection.normalIntensity
end

M.setNormalIntensity = function(value_float)
  decalProjection.normalIntensity = value_float
end

M.getMetallicIntensity = function()
  return decalProjection.metallicIntensity
end

M.setMetallicIntensity = function(value_float)
  decalProjection.metallicIntensity = value_float
end

M.getRoughnessIntensity = function()
  return decalProjection.roughnessIntensity
end

M.setRoughnessIntensity = function(value_float)
  decalProjection.roughnessIntensity = value_float
end

M.getGradientColorTopLeft = function()
  return decalProjection.gradientColorTopLeft
end

M.setGradientColorTopLeft = function(value_ColorI)
  decalProjection.gradientColorTopLeft = value_ColorI
end

M.getGradientColorTopRight = function()
  return decalProjection.gradientColorTopRight
end

M.setGradientColorTopRight = function(value_ColorI)
  decalProjection.gradientColorTopRight = value_ColorI
end

M.getGradientColorBottomLeft = function()
  return decalProjection.gradientColorBottomLeft
end

M.setGradientColorBottomLeft = function(value_ColorI)
  decalProjection.gradientColorBottomLeft = value_ColorI
end

M.getGradientColorBottomRight = function()
  return decalProjection.gradientColorBottomRight
end

M.setGradientColorBottomRight = function(value_ColorI)
  decalProjection.gradientColorBottomRight = value_ColorI
end

M.getColorPaletteMapId = function()
  return decalProjection.colorPaletteMapId
end

M.setColorPaletteMapId = function(value_int)
  decalProjection.colorPaletteMapId = value_int
end
-- END Decal projection - public interface END

-- decal shape render app : public interface
M.getShapePath = function()
  return decalProjection:getShapePath()
end

M.setShapePath = function(path)
  decalProjection:setShapePath(path)
  M.reprojectLayers()
end

M.getTextureResolution = function()
  return decalProjection:getTextureResolution()
end

M.setTextureResolution = function(resolution_point2i)
  if decalProjection:setTextureResolution(resolution_point2i) then
    M.reprojectLayers()
    app.textureSet = decalProjection:getTextureSet()
    app:setTextureSet("@DynamicTexture", app.textureSet)
  end
end

M.getUvLayer = function()
  return decalProjection:getUvLayer()
end

M.setUvLayer = function(uvLayerId_int)
  decalProjection:setUvLayer(uvLayerId_int)
  M.reprojectLayers()
end

M.clearMaterialIdx = function()
  decalProjection:clearMaterialIdx()
end

M.getMaterialIndices = function()
  return decalProjection:getMaterialIdx()
end

M.setMaterialIdx = function(materialIdx_int)
  decalProjection:setMaterialIdx(materialIdx_int)
end

M.addMaterialIdx = function(materialIdx_int)
  decalProjection:addMaterialIdx(materialIdx_int)
end

M.removeMaterialIdx = function(materialIdx_int)
  decalProjection:removeMaterialIdx(materialIdx_int)
end

M.getMeshObjectCount = function()
  return decalProjection:getMeshObjectCount()
end

M.getFillLayerColor = function()
  return decalProjection.fillLayerColor
end

M.setFillLayerColor = function(value_Point4F)
  decalProjection.fillLayerColor = value_Point4F
end

M.getFillLayerColorPaletteMapId = function()
  return decalProjection.fillLayerColorPaletteMapId
end

M.setFillLayerColorPaletteMapId = function(value_int)
  decalProjection.fillLayerColorPaletteMapId = value_int
end

M.getAlphaMaskChannel = function()
  return decalProjection.alphaMaskChannel
end

M.setAlphaMaskChannel = function(value_int)
  decalProjection.alphaMaskChannel = value_int
end

M.getAlphaMaskBlendMode = function()
  return decalProjection.alphaMaskBlendMode
end

M.setAlphaMaskBlendMode = function(value_int)
  decalProjection.alphaMaskBlendMode = value_int
end

M.getAlphaMaskRotation = function()
  return decalProjection.alphaMaskRotation
end

M.setAlphaMaskRotation = function(value_radian_float)
  decalProjection.alphaMaskRotation = value_radian_float
end

M.getMirrored = function()
  return decalProjection.mirrored
end

M.setMirrored = function(value_bool)
  decalProjection.mirrored = value_bool
end

M.getMirrorOffset = function()
  return mirrorPlaneOffset
end

M.setMirrorOffset = function(value_float)
  mirrorPlaneOffset = value_float
end

M.getMirrorDebug = function()
  return decalProjection.mirroredDebug
end

M.setMirrorDebug = function(value_bool)
  decalProjection.mirroredDebug = value_bool
end

M.getTextureFillLayerScale = function()
  return decalProjection.textureFillLayerScale
end

M.setTextureFillLayerScale = function(value_Point2F)
  decalProjection.textureFillLayerScale = value_Point2F
end

M.getTextureFillOffset = function()
  return decalProjection.textureFillOffset
end

M.setTextureFillOffset = function(value_Point2F)
  decalProjection.textureFillOffset = value_Point2F
end

M.getBrushStrokeInterpolationSteps = function()
  return decalProjection.brushStrokeInterpolationSteps
end

M.setBrushStrokeInterpolationSteps = function(value_u32)
  decalProjection.brushStrokeInterpolationSteps = value_u32
end

M.getPathLayerInterpolationSteps = function()
  return decalProjection.pathLayerInterpolationSteps
end

M.setPathLayerInterpolationSteps = function(value_u32)
  decalProjection.pathLayerInterpolationSteps = value_u32
end

M.getShapeMaterialNames = function()
  return decalProjection:getShapeMaterialNames()
end

M.getLockDepth = function()
  return decalProjection.lockDecalMatrixZBuffer
end

M.setLockDepth = function(value_bool)
  decalProjection.lockDecalMatrixZBuffer = value_bool
end

M.getDepth = function()
  return decalProjection:getDecalMatrixZBufferValue()
end

M.getLockSurfaceNormal = function()
  return decalProjection.lockSurfaceNormal
end

M.setLockSurfaceNormal = function(value_bool)
  decalProjection.lockSurfaceNormal = value_bool
end

M.getSurfaceNormal = function()
  return decalProjection:getDecalMatrixSurfaceNormalValue()
end

M.getUseSurfaceNormal = function()
  return decalProjection.useSurfaceNormal
end

M.setUseSurfaceNormal = function(value_bool)
  decalProjection.useSurfaceNormal = value_bool
end

M.setEnableBrushStroke = function(value_bool)
  decalProjection:setEnableBrushStroke(value_bool, getRandomUid())
end

M.setEnablePathLayer = function(value_bool)
  if value_bool == false then
    currentPathLayer = nil
  end
  doPath = value_bool
end

M.getEnablePathLayer = function()
  return doPath
end

M.hasActivePathLayer = function()
  return (currentPathLayer ~= nil)
end

M.finishPathLayer = function()
  currentPathLayer = nil
end

M.removeLastPathLayerPoint = function()
  if (currentPathLayer) then
    local dataPoints = M.getLayerByUid(currentPathLayer).dataPoints
    if #dataPoints == 1 then
      removeLayer(#layerStack, nil)
      currentPathLayer = nil
    elseif #dataPoints > 1 then
      table.remove(dataPoints)
      M.reprojectLayers()
    end
  end
end

M.getOrientPathDecals = function()
  return decalProjection.orientPathDecals
end

M.setOrientPathDecals = function(value_bool)
  decalProjection.orientPathDecals = value_bool
end

M.getPathType = function()
  return decalProjection.pathType
end

M.setPathType = function(value_int)
  decalProjection.pathType = value_int
end

M.getBrushStrokeLayerData = function()
  return decalProjection:getBrushStrokeLayerData()
end

M.getShapeMeshes = function()
  return decalProjection:getShapeMeshes()
end

M.enableAllMeshes = function()
  decalProjection:enableAllMeshes()
end

M.disableAllMeshes = function()
  decalProjection:disableAllMeshes()
end

M.areAllMeshesEnabled = function()
  return decalProjection:areAllMeshesEnabled()
end

M.setMeshEnable = function(name_string, value_bool)
  decalProjection:setMeshEnable(name_string, value_bool)
end

M.getDecalWorldPos = function(decal)
  return decalProjection:getDecalWorldPos(decal)
end

M.getDecalLocalPos = function(decal)
  return decalProjection:getDecalLocalPos(decal)
end

M.setDecalLocalPos = function(decal, worldPos)
  decalProjection:setDecalLocalPos(decal, worldPos)
end

M.getDecalWorldTransform = function(decal)
  return decalProjection:getDecalWorldTransform(decal)
end

M.getPathLayerText = function()
  return decalProjection.pathLayerText
end

M.setPathLayerText = function(value_string)
  decalProjection.pathLayerText = value_string
end

M.getDecalLayerFontCharacter = function()
  return decalProjection:getDecalLayerFontCharacter()
end

M.setDecalLayerFontCharacter = function(value_char)
  decalProjection:setDecalLayerFontCharacter(value_char)
end

M.getDecalLayerFontPath = function()
  return decalProjection:getDecalLayerFontPath()
end

M.setDecalLayerFontPath = function(value_string)
  decalProjection:setDecalLayerFontPath(value_string)
end

M.getPathLayerFontPath = function()
  return decalProjection:getPathLayerFontPath()
end

M.setPathLayerFontPath = function(value_string)
  decalProjection:setPathLayerFontPath(value_string)
end

M.getFlipMirroredDecal = function()
  return decalProjection.flipMirroredDecal
end

M.setFlipMirroredDecal = function(value_bool)
  decalProjection.flipMirroredDecal = value_bool
end

M.getSdfOutlineColor = function()
  return decalProjection.sdfOutlineColor
end

M.setSdfOutlineColor = function(value_ColorI)
  decalProjection.sdfOutlineColor = value_ColorI
end

M.getSdfEnabled = function()
  return decalProjection:getSdfEnabled()
end

M.setSdfEnabled = function(value_bool)
  decalProjection:setSdfEnabled(value_bool)
end

M.getSdfThickness = function()
  return decalProjection.sdfThickness
end

M.setSdfThickness = function(value_f32)
  decalProjection.sdfThickness = value_f32
end

M.getSdfSoftness = function()
  return decalProjection.sdfSoftness
end

M.setSdfSoftness = function(value_f32)
  decalProjection.sdfSoftness = value_f32
end

M.getSdfOutlineThickness = function()
  return decalProjection.sdfOutlineThickness
end

M.setSdfOutlineThickness = function(value_f32)
  decalProjection.sdfOutlineThickness = value_f32
end

M.getSdfOutlineSoftness = function()
  return decalProjection.sdfOutlineSoftness
end

M.setSdfOutlineSoftness = function(value_f32)
  decalProjection.sdfOutlineSoftness = value_f32
end

M.bakeBrush = function()
  decalProjection:bakeBrush()
end

M.getTextureSet = function()
  return (app and app.textureSet) and app.textureSet or nil
end

M.getCursorPosition = function()
  return decalProjection.cursorPosition
end

M.setCursorPosition = function(value_Point2F)
  decalProjection.cursorPosition = value_Point2F
end

M.renderSdfTextureImgui = function(sizeX, sizeY)
  decalProjection:renderSdfTextureImgui(sizeX, sizeY)
end

local function moveLayerCursorPos(layer, cursorPosOffset)
  if layer.type == M.layerTypes.decal then
    local layerCopy = deepcopy(layer)
    layerCopy.cursorPosScreenUv.x = layer.cursorPosScreenUv.x + cursorPosOffset.x
    layerCopy.cursorPosScreenUv.y = layer.cursorPosScreenUv.y + cursorPosOffset.y
    M.setLayer(layerCopy, false)
  elseif layer.type == M.layerTypes.brushStroke then
    local layerCopy = deepcopy(layer)
    for _, cpos in ipairs(layerCopy.dataPoints) do
      cpos.x = cpos.x + cursorPosOffset.x
      cpos.y = cpos.y + cursorPosOffset.y
    end
    M.setLayer(layerCopy, false)
  elseif layer.type == M.layerTypes.path then
    local layerCopy = deepcopy(layer)
    for _, cpos in ipairs(layerCopy.dataPoints) do
      cpos.x = cpos.x + cursorPosOffset.x
      cpos.y = cpos.y + cursorPosOffset.y
    end
    M.setLayer(layerCopy, false)
  end
end

local function moveLayerChildrenCursorPosRec(layer, cursorPosOffset)
  if layer.children then
    for _, child in ipairs(layer.children) do
      moveLayerCursorPos(child, cursorPosOffset)
      moveLayerChildrenCursorPosRec(child, cursorPosOffset)
    end
  end
end

local function transformLayerChildren(layer, parentTransformOld, parentTransformNew)
  if layer.children then
    local invParent = parentTransformOld:copy():inverse()
    for _, child in ipairs(layer.children) do
      if child.type == M.layerTypes.decal then
        local transformRel = invParent:copy():mul(decalProjection:getDecalWorldTransform(child))
        local transform = parentTransformNew:copy():mul(transformRel)
        local layerCopy = deepcopy(M.getLayerByUid(child.uid))
        decalProjection:setDecalWorldTransform(layerCopy, transform)
        M.setLayer(layerCopy, false)
      end
    end
  end
end

local function moveLayerLocalPos_group(layer, newLocalPosition, includeChildren, referenceLayerUid)
  local layerDataCopy = deepcopy(M.getLayerByUid(referenceLayerUid))

  local cursorPosOffset = {x = 0.0, y = 0.0}
  if layerDataCopy.type == M.layerTypes.decal then
    local cursorPosStart = {x = layerDataCopy.cursorPosScreenUv.x, y = layerDataCopy.cursorPosScreenUv.y}
    M.setDecalLocalPos(layerDataCopy, newLocalPosition)
    cursorPosOffset.x = layerDataCopy.cursorPosScreenUv.x - cursorPosStart.x
    cursorPosOffset.y = layerDataCopy.cursorPosScreenUv.y - cursorPosStart.y
  elseif layerDataCopy.type == M.layerTypes.path then
    layerDataCopy.cursorPosScreenUv = {x = layerDataCopy.dataPoints[1].x, y = layerDataCopy.dataPoints[1].y}
    local cursorPosStart = {x = layerDataCopy.cursorPosScreenUv.x, y = layerDataCopy.cursorPosScreenUv.y}
    M.setDecalLocalPos(layerDataCopy, newLocalPosition)
    cursorPosOffset.x = layerDataCopy.cursorPosScreenUv.x - cursorPosStart.x
    cursorPosOffset.y = layerDataCopy.cursorPosScreenUv.y - cursorPosStart.y
  elseif layerDataCopy.type == M.layerTypes.brushStroke then
    layerDataCopy.cursorPosScreenUv = {x = layerDataCopy.dataPoints[1].x, y = layerDataCopy.dataPoints[1].y}
    local cursorPosStart = {x = layerDataCopy.cursorPosScreenUv.x, y = layerDataCopy.cursorPosScreenUv.y}
    M.setDecalLocalPos(layerDataCopy, newLocalPosition)
    cursorPosOffset.x = layerDataCopy.cursorPosScreenUv.x - cursorPosStart.x
    cursorPosOffset.y = layerDataCopy.cursorPosScreenUv.y - cursorPosStart.y
  end

  if includeChildren then
    moveLayerChildrenCursorPosRec(layer, cursorPosOffset)
  end
  M.reprojectLayers()
end

local function moveLayerLocalPos_path(layer, newLocalPosition, includeChildren)
  local data = deepcopy(M.getLayerByUid(layer.uid))
  local layerDataCopy = shallowcopy(data)
  data.cursorPosScreenUv = {x = data.dataPoints[1].x, y = data.dataPoints[1].y}
  layerDataCopy.cursorPosScreenUv = {x = data.dataPoints[1].x, y = data.dataPoints[1].y}
  local curLocalPos = M.getDecalLocalPos(layerDataCopy)
  local localOffset = newLocalPosition - curLocalPos
  M.setDecalLocalPos(layerDataCopy, newLocalPosition)

  local newDataPoints = {}
  local xOffset = layerDataCopy.cursorPosScreenUv.x - data.cursorPosScreenUv.x
  local yOffset = layerDataCopy.cursorPosScreenUv.y - data.cursorPosScreenUv.y
  newDataPoints[1] = {x = layerDataCopy.cursorPosScreenUv.x, y = layerDataCopy.cursorPosScreenUv.y}

  if #data.dataPoints > 1 then
    for i = 2, #data.dataPoints do
      newDataPoints[i] = {x = data.dataPoints[i].x + xOffset, y = data.dataPoints[i].y + yOffset}
    end
  end
  data.dataPoints = newDataPoints
  M.setLayer(data, false)

  if includeChildren then
    moveLayerChildrenCursorPosRec(layer, {x = data.dataPoints[1].x - layer.dataPoints[1].x, y = data.dataPoints[1].y - layer.dataPoints[1].y})
  end
  M.reprojectLayers()
end

local function moveLayerLocalPos_brushStroke(layer, newLocalPosition, includeChildren)
  local data = deepcopy(M.getLayerByUid(layer.uid))
  local layerDataCopy = shallowcopy(data)
  data.cursorPosScreenUv = {x = data.dataPoints[1].x, y = data.dataPoints[1].y}
  layerDataCopy.cursorPosScreenUv = {x = data.dataPoints[1].x, y = data.dataPoints[1].y}
  local curLocalPos = M.getDecalLocalPos(layerDataCopy)
  local localOffset = newLocalPosition - curLocalPos
  M.setDecalLocalPos(layerDataCopy, newLocalPosition)

  local newDataPoints = {}
  local xOffset = layerDataCopy.cursorPosScreenUv.x - data.cursorPosScreenUv.x
  local yOffset = layerDataCopy.cursorPosScreenUv.y - data.cursorPosScreenUv.y
  newDataPoints[1] = {x = layerDataCopy.cursorPosScreenUv.x, y = layerDataCopy.cursorPosScreenUv.y}

  if #data.dataPoints > 1 then
    for i = 2, #data.dataPoints do
      newDataPoints[i] = {x = data.dataPoints[i].x + xOffset, y = data.dataPoints[i].y + yOffset}
    end
  end
  data.dataPoints = newDataPoints
  M.setLayer(data, false)

  if includeChildren then
    moveLayerChildrenCursorPosRec(layer, {x = data.dataPoints[1].x - layer.dataPoints[1].x, y = data.dataPoints[1].y - layer.dataPoints[1].y})
  end
  M.reprojectLayers()
end

local function moveLayerLocalPos_decal(layer, newPosition, includeChildren)
  local layer = deepcopy(M.getLayerByUid(layer.uid))
  local oldTransform = decalProjection:getDecalWorldTransform(layer)
  local offsetM = MatrixF(true)
  offsetM:setPosition(newPosition - oldTransform:mulP3F(Point3F(0, 0, 0)))
  --local newTransform = oldTransform:copy():mul(offsetM)
  local newTransform = offsetM:mul(oldTransform)
  local newLayer = deepcopy(M.getLayerByUid(layer.uid))
  decalProjection:setDecalWorldTransform(newLayer, newTransform)
  M.setLayer(newLayer, false)

  if includeChildren then
    transformLayerChildren(newLayer, oldTransform, newTransform)
  end

  M.reprojectLayers()
end

M.moveLayerLocalPos = function(layerUid, newLocalPosition, includeChildren, referenceLayerUid)
  local layer = M.getLayerByUid(layerUid)
  if layer.type == M.layerTypes.decal then
    moveLayerLocalPos_decal(layer, newLocalPosition, includeChildren)
  elseif layer.type == M.layerTypes.brushStroke then
    moveLayerLocalPos_brushStroke(layer, newLocalPosition, includeChildren)
  elseif layer.type == M.layerTypes.path then
    moveLayerLocalPos_path(layer, newLocalPosition, includeChildren)
  elseif layer.type == M.layerTypes.group then
    if not referenceLayerUid then
      print("'referenceLayerUid' is needed in order to be able to move the layers")
      return
    end
    moveLayerLocalPos_group(layer, newLocalPosition, includeChildren, referenceLayerUid)
  end
end

M.rotateLayer = function(layer, delta_vec3)
  layer = deepcopy(M.getLayerByUid(layer.uid))
  local dotVal = -delta_vec3.y
  local rotUpvec = vec3(math.sin(dotVal), 0, math.cos(dotVal)):normalized()
  local oldTransform = decalProjection:getDecalWorldTransform(layer)
  local newTransform = MatrixF(true)
  newTransform:setColumn(0, vec3(0, 1, 0):cross(rotUpvec):normalized())
  newTransform:setColumn(1, vec3(0, 1, 0))
  newTransform:setColumn(2, rotUpvec)
  newTransform = oldTransform:copy():mul(newTransform)
  local newLayer = deepcopy(M.getLayerByUid(layer.uid))
  decalProjection:setDecalWorldTransform(newLayer, newTransform)
  M.setLayer(newLayer, false)

  local includeChildren = true
  if includeChildren then
    transformLayerChildren(newLayer, oldTransform, newTransform)
  end

  M.reprojectLayers()
end

M.scaleLayer = function(layer, delta_vec3)
  layer = deepcopy(M.getLayerByUid(layer.uid))
  local newLayer = deepcopy(M.getLayerByUid(layer.uid))
  local oldTransform = decalProjection:getDecalWorldTransform(layer)
  local newTransform = oldTransform:copy()
  newTransform:scale(vec3(1, 1, 1) + delta_vec3)
  decalProjection:setDecalWorldTransform(newLayer, newTransform)
  M.setLayer(newLayer, false)

  local includeChildren = true
  if includeChildren then
    transformLayerChildren(newLayer, oldTransform, newTransform)
  end

  M.reprojectLayers()
end

local function serializeLayerStack()
  local layerData = {}
  for _, layer in pairs(layerStack) do
    table.insert(layerData, serializeLayer(layer))
  end

  return layerData
end

local function deserializeLayerStack(layerStackData)
  for _, layer in ipairs(layerStackData) do
    local lyr = deserializeLayer(layer)
    table.insert(layerStack, lyr)
  end
end

-- serialization
M.onSerialize = function()
  if not decalProjection then return end
  local materialIndices = decalProjection:getMaterialIdx()
  local materialIdx = #materialIndices > 0 and materialIndices[1] or nil
  return {
    layerStackData = serializeLayerStack(),
    materialIdx = materialIdx,
  }
end

M.onDeserialized = function(data)
  if data.layerStackData then
    deserializeLayerStack(data.layerStackData)
  end
  reconstructData.materialIdx = data.materialIdx or 0
end

M.saveLayerStackToFile = function(path_string)
  local data = M.onSerialize()
  data.materialIdx = nil
  jsonWriteFile(path_string, data, true)
end

M.loadLayerStackFromFile = function(path_string, mode_number)
  mode_number = mode_number or loadingMode.Overwrite
  if mode_number == loadingMode.Overwrite then
    layerStack = {}
    layerCount = 0
  end
  local data = jsonReadFile(path_string)
  M.onDeserialized(data)
  return M.reprojectLayers()
end

M.onUpdate_ = function()
  if app then
    profilerPushEvent('dynamicDecals/app:onUpdate_()')
    app:onUpdate()
    profilerPopEvent('dynamicDecals/app:onUpdate_()')
    if app.textureSet then
      if M.projectDynamicDecals == true then
        profilerPushEvent('dynamicDecals/decalProjection:projectDynamicDecals()')
        decalProjection:projectDynamicDecals(app:getCameraM(), app:getProjM())
        profilerPopEvent('dynamicDecals/decalProjection:projectDynamicDecals()')
      end
      profilerPushEvent('dynamicDecals/decalProjection:combineTextures()')
      decalProjection:combineTextures(app.textureSet)
      profilerPopEvent('dynamicDecals/decalProjection:combineTextures()')
    end
  end
end

M.getHistory = function()
  return history
end

M.undo = function()
  history:undo()
end

M.redo = function()
  history:redo()
end

M.onExtensionUnloaded = function()
  decalProjection = nil
  app = nil
  M.ready = false
end

M.onVehicleSwitched  = function()
  -- force to refresh the new vehicle shape
  if(app) then app.onUpdate() end
  M.updateVehicleMaterials()
end

M.getRandomUid = getRandomUid
M.serializeLayer = serializeLayer
M.deserializeLayer = deserializeLayer

return M
