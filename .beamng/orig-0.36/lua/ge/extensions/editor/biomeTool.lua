-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local u_32_max_int = 4294967295
local logTag = 'biome_tool'
local toolWindowName = "biomeTool"
local editModeName = "Edit Biome"
local imgui = ui_imgui
local ffi = require('ffi')

local roadRiverGui = extensions.editor_roadRiverGui
local exclusionZoneIndices = {}
local isDrawingLassoArea = false
local valueInspector = require("editor/api/valueInspector")()

local forest
local layerCreateMtlComboItemsTbl = {}
local layerCreateMtlComboItems = imgui.ArrayCharPtrByTbl(layerCreateMtlComboItemsTbl)

local layerBlendingComboItemsTbl = {"Add", "Replace", "Delete"}
local layerBlendingComboItems = imgui.ArrayCharPtrByTbl(layerBlendingComboItemsTbl)
local terrainBlock = nil
local maskFilePath = imgui.ArrayChar(256,"")
local randomBaseMaskFilePath = imgui.ArrayChar(256,"")
local editAreaIndex = nil
local renameEditEnded = imgui.BoolPtr(false)
local areaMaterialIndex = 0

local levelBiomeToolbarHeight = 50
local levelBiomeLevelsListHeight = 500
local levelBiomeLevelPropsHeight = 700
local biomeAreasToolbarHeight = 50
local biomeAreasLevelsListHeight = 500
local biomeAreasLevelPropsHeight = 500
local seperatorHeight            = 6
local createForestPopupShown = false

local areaPaneSeparatorHeight = 6
local editLayerPaneHeight = 160
local editEnded = imgui.BoolPtr(false)
local input2FloatValue = imgui.ArrayFloat(2)
local inputTextValue = imgui.ArrayChar(500)

local highlighAnimation =  {
  isHighligthing = false,
  layerType = nil,
  layerID = nil,
  zoneType = nil,
  zoneID = nil,
  duration = 3.0,
  elapsed = 0.0
}

local areaType_enum = {
  lasso = 0,
  terrain_material = 1
}

local layerType_enum = {
  terrain = 0,
  area = 1,
  any = 2
}

local blending_enum = {
  add = 0,
  replace = 1,
  delete = 2,
}

local enum_forestObjType = {
  forestBrush = 1,
  forestBrushElement = 2,
  forestItemData = 3
}

local enum_forestBrushItemZone = {
  central = 1,
  falloff = 2,
  edge = 3
}

local enum_biomeProcType = {
  Material = 0,
  Lasso = 1,
  Field = 2
}

local enum_tabType = {
  LevelBiome = 0,
  BiomeAreas = 1
}

local noneBrushItemName = "- NONE -"

local fieldInfoTemplate = {
  { name = "LayerName", label = "Layer name", val = "", type = "string", layerType = layerType_enum.any},
  { name = "TerrainMaterial", label = "Terrain material", val = 0, type = "int", layerType = layerType_enum.terrain},
  { name = "TerrainMask", label = "Terrain mask", val = "", type = "string", layerType = layerType_enum.terrain},
  { name = "ForestDensity", label = "Forest density (0 to 1)", val = 1, minValue = 0, maxValue = 1, type = "float", layerType = layerType_enum.any},
  { name = "SlopeInfluence", label = "Slope influence (-1 to 1)", val = 0, minValue = -1, maxValue = 1, type = "int", layerType = layerType_enum.any},
  { name = "SlopeRange", label = "Slope range (0 to 90)", val = {0, 90}, minValue = 0, maxValue = 90, type = "float", layerType = layerType_enum.any},
  { name = "BordersFalloff", label = "Border falloff (-10 to 10)", val = 0, minValue = -10, maxValue = 10, type = "int", layerType = layerType_enum.any},
  { name = "BordersDensity", label = "Border Density (0 to 1)", val = 1, minValue = 0, maxValue = 1, type = "float", layerType = layerType_enum.any},
  { name = "BlendingMethod", label = "Blending method", val = blending_enum.add, type = "int", layerType = layerType_enum.any},
  { name = "EdgePlacement", label = "Place Edges", val = false, type = "bool", layerType = layerType_enum.any},

  --lasso only
  { name = "FieldPlacement", label = "Place Field", val = false, type = "bool", layerType = layerType_enum.area},
  { name = "FieldItemDistance", label = "Items Distance", val = 1, minValue = 0, maxValue = 100, type = "float", layerType = layerType_enum.area},
  { name = "FieldRowDistance", label = "Rows Distance", val = 1, minValue = 0, maxValue = 100, type = "float", layerType = layerType_enum.area},
  { name = "FieldRowOrientation", label = "Rows Orientation", val = 0, minValue = 0, maxValue = 360, type = "float", layerType = layerType_enum.area},

  --random area params
  { name = "RA_Map", label = "Random area map", val = "", type = "filename", typeName = "TypeImageFilename", layerType = layerType_enum.any},
  { name = "RA_Seed", label = "Seed", val = 176312589, minValue = 0, maxValue = 276447231, type = "int", layerType = layerType_enum.any},
  { name = "RA_Freq", label = "Frequency", val = 1, minValue = 0.001, maxValue = 2.0, type = "float", layerType = layerType_enum.any},
  { name = "RA_Amp", label = "Size Coeff.", val = 1, minValue = 1, maxValue = 100.0, type = "float", layerType = layerType_enum.any},
  { name = "RA_Thr", label = "Threshold", val = 0.0, minValue = -1.0, maxValue = 1.0, type = "float", layerType = layerType_enum.any},
  { name = "RA_Oct", label = "Octave", val = 5, minValue = 1, maxValue = 10, type = "int", layerType = layerType_enum.any},
  { name = "RA_Mask", label = "Mask file", val = "", type = "string", layerType = layerType_enum.any},
  { name = "RA_Material", label = "Material", val = "", type = "string", layerType = layerType_enum.any},
  { name = "RA_LassoAreas", label = "Material", val = "", type = "", layerType = layerType_enum.area},

  --edge placement params
  { name = "EP_ItemDistance", label = "Item Distance", val = 1, minValue = -10, maxValue = 10, type = "int", layerType = layerType_enum.any},
  { name = "EP_RandomTilt", label = "Random Tilt Range", val = {-5, 5}, minValue = -40, maxValue = 40, type = "float", layerType = layerType_enum.any},
  { name = "EP_BorderFalloff", label = "Border Falloff", val = 0.0, minValue = -10, maxValue = 10, type = "float", layerType = layerType_enum.any}
}

local rowPlacementFields = { "FieldItemDistance", "FieldRowDistance", "FieldRowOrientation" }
local areaListFilter = imgui.ImGuiTextFilter()
local shouldUpdateAreasScroll = false
local var = {}
var.layers = {}
var.layers.layerInfoTbl = {}
var.layers.layerGlobalIndices = {}
var.layers.lassoAreaGlobalIndices = {}
var.layers.exZoneGlobalIndices = {}
var.layers.fieldInfoTbl = {}
var.layers.selectedLayerIDs = {}
var.layers.forestBrushSelectedItems = {}
var.layers.forestBrushTempSelectedItems = {}
var.layers.forestItemsTbl = {}
var.layers.layerBlendingComboIndexTbl = {}
var.layers.PopupOpenMousePos = {x = 200, y = 200}

var.areas = {}
var.areas.layerBlendingComboIndexTbl = {}
var.areas.areaGlobalIndex = 1
var.areas.areaInfoTbl = {}
var.areas.fieldInfoTbl = {}
var.areas.forestItemsTbl = {}
var.areas.forestBrushSelectedItems = {}
var.forestBrushTempSelectedItems = {}
var.forestBrushes = {}
var.lassoAreas = {}
var.lassoPLNodes = {}
var.lassoPLLineSegments = {}
var.lassoHoveredNode = {}
var.lassoActionHoveredNodeIndex = nil
var.lassoActionSelectedNodeIndex = nil
var.lassoSelectionEnded = false
var.lassoSelectionItemsCalculated = false
var.mouseButtonHeldOnLassoNode = false
var.lassoNodeDragInitialNodes = nil
var.shouldRenderCompletionSphere = false
var.lassoDrawActionCompleted = false
var.lassoAreaSelectedNode = {}
var.selectedAreaID = nil
var.lassoAreasGroupedMeshTable = {}
var.isGroupMeshesEnabled = true
var.groupSelectedIndices_Modify = {}
var.itemsSelectedIndices_Modify = {}
var.groupSelectedIndices_Generate = {}
var.itemsSelectedIndices_Generate = {}
var.selectAllEnabled = false
var.selectedLayerID = nil
var.forestBrushGroup = {}
var.buttonColor_active = imgui.GetStyleColorVec4(imgui.Col_ButtonActive)
var.buttonColor_inactive = imgui.GetStyleColorVec4(imgui.Col_Button)
var.forestBrushTool = nil
var.itemsToDelete = {}
var.enum_lassoDrawType = {inclusionZone = 0, exclusionZone = 1}
var.lassoDrawInfo = {type = var.enum_lassoDrawType.inclusionZone, areaID = nil, layerID = nil}
var.areas.exclusionZones = {}
var.enum_hoveredNodeAreaType = {lassoAction = 0, inclusionZone = 1, exclusionZone = 2}
var.hoveredNodeAreaType = var.enum_hoveredNodeAreaType.lassoAction
var.randImgScaleCoeff = 1.0
var.selectedTab = enum_tabType.LevelBiome
var.askedToOpenProject = false
var.fieldPlacementBoolPtr = imgui.BoolPtr(0)
var.edgePlacementBoolPtr = imgui.BoolPtr(0)
var.randomAreaBoolPtr = imgui.BoolPtr(0)

local function getLayersWithType(layerType)
  local layers = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerType == layerType then
      table.insert(layers, layer)
    end
  end
  return layers
end

local function getLayer(layerType, layerID)
  local ret = nil
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerType == layerType and layer.layerID == layerID then
      ret = layer
      break
    end
  end
  return ret
end

local function getLassoNodeUnderCursor()
  local camPos = core_camera.getPosition()
  local ray = getCameraMouseRay()
  local rayDir = ray.dir
  local minNodeDist = u_32_max_int
  local hoveredNodeIndex = nil
  local hoveredNodeLayerID = nil
  local hoveredNodeAreaID = nil
  local hoveredNodeAreaType = var.enum_hoveredNodeAreaType.lassoAction

  if isDrawingLassoArea then
    for nodeIndex, node in ipairs(var.lassoPLNodes) do
      local distNodeToCam = (node.pos - camPos):length()
      if distNodeToCam < minNodeDist then
        local nodeRayDistance = (node.pos - camPos):cross(rayDir):length() / rayDir:length()
        local sphereRadius = (camPos - node.pos):length() * roadRiverGui.nodeSizeFactor
        if nodeRayDistance <= sphereRadius then
          hoveredNodeLayerID = -1
          hoveredNodeIndex = nodeIndex
          minNodeDist = distNodeToCam
        end
      end
    end
    if hoveredNodeIndex then
      hoveredNodeAreaType = var.enum_hoveredNodeAreaType.lassoAction
      return {index = hoveredNodeIndex, layerID = hoveredNodeLayerID, areaType = hoveredNodeAreaType}
    end
  end

  local selectedLassoAreaID = nil
  for _, selection in ipairs(var.layers.selectedLayerIDs) do
    if selection.layerType == layerType_enum.area then
      selectedLassoAreaID = selection.selectedLayerID
    end
  end

  local lassoAreas = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == selectedLassoAreaID and layer.layerType == layerType_enum.area then
      --local lassoArea = {lassoAreaID = getLassoAreaGlobalIdx(layerID, zoneType) + 1, zoneType = zoneType, nodes = deepcopy(nodes)}
      for _, lassoArea in ipairs(layer.lassoAreas) do
        for nodeIndex, node in ipairs(lassoArea.nodes) do
          local distNodeToCam = (node.pos - camPos):length()
          if distNodeToCam < minNodeDist then
            local nodeRayDistance = (node.pos - camPos):cross(rayDir):length() / rayDir:length()
            local sphereRadius = (camPos - node.pos):length() * roadRiverGui.nodeSizeFactor
            if nodeRayDistance <= sphereRadius then
              hoveredNodeLayerID = layer.layerID
              hoveredNodeAreaID = lassoArea.lassoAreaID
              hoveredNodeIndex = nodeIndex
              minNodeDist = distNodeToCam
              hoveredNodeAreaType = lassoArea.zoneType
              break
            end
          end
        end
        if hoveredNodeLayerID then
          break
        end
      end
      break
    end
  end
  if hoveredNodeIndex then
    return {index = hoveredNodeIndex, layerID = hoveredNodeLayerID, areaID = hoveredNodeAreaID, areaType = hoveredNodeAreaType}
  end
  return nil
end

local function castRayDown(startPoint, endPoint)
  if not endPoint then
    endPoint = startPoint - vec3(0,0,100)
  end
  local res = Engine.castRay((startPoint + vec3(0,0,1)), endPoint, true, false)
  if not res then
    res = Engine.castRay((startPoint + vec3(0,0,100)), (startPoint - vec3(0,0,1000)), true, false)
  end
  return res
end

local function drawLassoLineSegmented(originNode, targetNode, lassoAreaType, color)
  originNode.pos = vec3(originNode.pos.x, originNode.pos.y, originNode.pos.z)
  targetNode.pos = vec3(targetNode.pos.x, targetNode.pos.y, targetNode.pos.z)
  local length = (originNode.pos - targetNode.pos):length()
  local lineWidth = editor.getPreference("gizmos.general.lineThicknessScale") * 4
  local lineColor = ColorF(0,0,1,0.5)
  local renderColor = (lassoAreaType == var.enum_lassoDrawType.inclusionZone) and ColorF(0,0,1,0.5) or ColorF(1,0,0,0.5)
  if color then renderColor = color end
  debugDrawer:drawLineInstance(originNode.pos, targetNode.pos, lineWidth, renderColor, false)
end

local function incExZoneGlobalIdx(areaID, layerID)
  local indexFound = false
  for _, zoneIndexInfo  in ipairs(exclusionZoneIndices) do
    if zoneIndexInfo.areaID == areaID and zoneIndexInfo.layerID == layerID  then
      zoneIndexInfo.zoneIndex = zoneIndexInfo.zoneIndex + 1
      indexFound = true
      break
    end
  end
end

local function deleteLayer(layerType, layerID)
  local removeIndex = -1
  for index, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerType == layerType and layer.layerID == layerID then
      removeIndex = index
      break
    end
  end

  if removeIndex > 0 then
    table.remove(var.layers.layerInfoTbl, removeIndex)
  end
end

local function insertFieldInfo(layerType, layerID, fieldData)
  for _, fieldInfo in ipairs(var.layers.fieldInfoTbl) do
    if fieldInfo.layerType == layerType and fieldInfo.layerID == layerID then
      table.insert(fieldInfo.fieldsData, fieldData)
    end
  end
end

local function getLayerWithType(layerType, layerID)
  local layerInfo = nil
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerType == layerType and layer.layerID == layerID then
      layerInfo = layer
      break
    end
  end
  return layerInfo
end

local function populateForestBrushes()
  var.forestBrushGroup = scenetree.findObject("ForestBrushGroup")
  var.forestBrushes = {}
  if var.forestBrushGroup then
    local forestBrushGroupSize = var.forestBrushGroup:size() - 1
    for i = 0, forestBrushGroupSize do
      local obj = var.forestBrushGroup:at(i)
      local internalName = obj:getInternalName()
      if internalName then
        local item = {
          id = obj:getId(),
          internalName = internalName,
          type = (obj:getClassName() == "ForestBrush") and enum_forestObjType.forestBrush or enum_forestObjType.forestBrushElement,
          elements = {},
          open = false,
          selected = false
        }
        table.insert(var.forestBrushes, item)
      end
    end
    local compareFunc = function(a,b)
      return string.lower(a.internalName) < string.lower(b.internalName)
    end

    table.sort(var.forestBrushes, compareFunc)
  end
end

local function getLayerType(areaID, layerID)
  local layerType = nil
  for _, area in ipairs(var.areas.areaInfoTbl) do
    if area.areaID == areaID then
      for _, layer in ipairs(area.layers) do
        if layer.layerID == layerID then
          layerType = layer.layerType
          break
        end
      end
      break
    end
  end
  return layerType
end

local function getBlendingMethodStr(blendingEnum)
  local blendingMethodStr = ""
  if blendingEnum == blending_enum.add then
    blendingMethodStr = "Add"
  elseif blendingEnum == blending_enum.delete then
    blendingMethodStr = "Delete"
  else
    blendingMethodStr = "Replace"
  end
  return blendingMethodStr
end

local function selectForestBrush(layerType, layerID, internalName, zoneType)
  local itemFound = false
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      table.insert(selectedItemsInfo.selectedItems, internalName)
      itemFound = true
    end
  end

  if not itemFound then
    local brushZoneType = zoneType or enum_forestBrushItemZone.central
    local selectionData = {layerType = layerType, layerID = layerID, zoneType = brushZoneType, selectedItems = {internalName}}
    table.insert(var.layers.forestBrushSelectedItems, selectionData)
  end
end

local function selectForestTempBrush(layerType, layerID, internalName, zoneType)
  local itemFound = false
  for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      table.insert(selectedItemsInfo.selectedItems, internalName)
      itemFound = true
    end
  end
  if not itemFound then
    local brushZoneType = zoneType or enum_forestBrushItemZone.central
    local selectionData = {layerType = layerType, layerID = layerID, zoneType = brushZoneType, selectedItems = {internalName}}
    table.insert(var.forestBrushTempSelectedItems, selectionData)
  end
end

local function clearForestBrushTempSelection()
  var.forestBrushTempSelectedItems = {}
end

local function indexOf(table, value)
  if not table then return -1 end
  for i,v in ipairs(table) do
    if v == value then return i end
  end
  return -1
end

local function isForestBrushTempSelected(layerType, layerID, internalName, zoneType)
  local selected = false
  for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      selected = (indexOf(selectedItemsInfo.selectedItems, internalName) ~= -1)
      break
    end
  end
  return selected
end

local function getForestBrushTempSelection(areaID, layerID, zoneType)
  local selectionInfo = {}
  if isForestBrushTempSelected(layerType, layerID, noneBrushItemName, zoneType) then
    return selectionInfo
  end
  for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
       selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
        selectionInfo = selectedItemsInfo.selectedItems
      break
    end
  end
  return selectionInfo
end

local function deselectForestTempBrush(layerType, layerID, internalName, zoneType)
  for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      local index = indexOf(selectedItemsInfo.selectedItems, internalName)
      if index ~= -1 then
        table.remove(selectedItemsInfo.selectedItems, index)
      end
      break
    end
  end
end

local function getFieldsData(layerType, layerID)
  local data = {}
  for _, info in ipairs(var.layers.fieldInfoTbl) do
    if (info.layerType == layerType or info.layerType == layerType_enum.any) and info.layerID == layerID then
      data = info.fieldsData
    end
  end
  return data
end

local function getRandomLayerMapFile(layerType, layerID)
  local mapFile = ""
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Map" then
      mapFile = fieldData.val
    end
  end
  return mapFile
end

local function setRandomLayerMapFile(layerType, layerID, filename)
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Map" then
      fieldData.val = filename
      break
    end
  end
end

local function getRandomLayerFrequency(layerType, layerID)
  local freq = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Freq" then
      freq = fieldData.val
    end
  end
  return freq
end

local function getRandomLayerSeed(layerType, layerID)
  local seed = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Seed" then
      seed = fieldData.val
    end
  end
  return seed
end

local function getRandomLayerAmplitude(layerType, layerID)
  local amp = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Amp" then
      amp = fieldData.val
    end
  end
  return amp
end

local function getRandomLayerThreshold(layerType, layerID)
  local threshold = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Thr" then
      threshold = fieldData.val
    end
  end
  return threshold
end

local function getRandomLayerOctave(layerType, layerID)
  local octave = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Oct" then
      octave = fieldData.val
    end
  end
  return octave
end

local function getRandomLayerBaseMaskFile(layerType, layerID)
  local mapFile = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Mask" then
      mapFile = fieldData.val
    end
  end
  return mapFile
end

local function setRandomLayerBaseMaskFile(layerType, layerID, filename)
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Mask" then
      fieldData.val = filename
    end
  end
end

local function getRandomLayerMaterialIndex(layerType, layerID)
  local materialName = nil
  local materialIndex = 0
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_Material" then
      materialName = fieldData.val
      break
    end
  end

  if terrainBlock then
    local mtls = terrainBlock:getMaterials()
    for index, mtl in ipairs(mtls) do
      if materialName == mtl.internalName then
        materialIndex = index-1
      end
    end
  end
  return materialIndex
end

local function getRandomLayerLassoAreas(layerType, layerID)
  local lassoAreas = nil
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "RA_LassoAreas" then
      lassoAreas = fieldData.val
      break
    end
  end
  return lassoAreas
end

local function getLayerTypeGlobalIdx(layerType)
  local index = 0
  local found = false
  for _, indexData in ipairs(var.layers.layerGlobalIndices) do
    if indexData.layerType == layerType then
      index = indexData.index
      found = true
    end
  end
  if not found then
    table.insert(var.layers.layerGlobalIndices, {layerType = layerType, index = 0})
  end
  return index
end

local function incLayerTypeGlobalIdx(layerType)
  local found = false
  for _, indexData in ipairs(var.layers.layerGlobalIndices) do
    if indexData.layerType == layerType then
      indexData.index = indexData.index + 1
      found = true
    end
  end
  if not found then
    table.insert(var.layers.layerGlobalIndices, {layerType = layerType, index = 1})
  end
end

local function getFieldMinMax(name)
  local fieldMinMaxPair = {}
  for _, fieldInfo in ipairs(fieldInfoTemplate) do
    if fieldInfo.name == name then
      fieldMinMaxPair = {fieldInfo.minValue, fieldInfo.maxValue}
      break
    end
  end
  return fieldMinMaxPair
end

local function addLayerWithType(layerType)
  local layerID = nil
  local layerName
  local layerIndex =  getLayerTypeGlobalIdx(layerType) + 1
  incLayerTypeGlobalIdx(layerType)
  local prefix = (layerType == layerType_enum.area) and "Area" or "Terrain"
  local layer = {
    layerType = layerType,
    layerID = layerIndex,
    layerName = prefix.." ".."Layer "..tostring(layerIndex),
    lassoAreas = {}
  }
  layerName = layer.layerName
  layerID = layerIndex
  table.insert(var.layers.layerInfoTbl, layer)
  table.insert(var.layers.fieldInfoTbl, {layerType = layerType, layerID = layerID, fieldsData = {}})

  for _, fieldInfo in ipairs(var.layers.fieldInfoTbl) do
    if fieldInfo.layerType == layerType and fieldInfo.layerID == layerID then
      table.insert(fieldInfo.fieldsData, fieldData)
    end
  end

  insertFieldInfo(layerType, layerID, fieldData)

  for _, fieldInfo in ipairs(fieldInfoTemplate) do
    if fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType then
      local fieldData = {
        name = fieldInfo.name,
        label = fieldInfo.label,
        val = fieldInfo.val,
        minValue = fieldInfo.minValue,
        maxValue = fieldInfo.maxValue,
        type = fieldInfo.type,
        layerType = fieldInfo.layerType
      }
      if fieldInfo.name == "BlendingMethod" then
        fieldData.val = getBlendingMethodStr(fieldData.val)
      end
      insertFieldInfo(layerType, layerID, fieldData)
    end
  end

  populateForestBrushes()
  return layer
end

local function resetDrawActionVariables()
  var.shouldRenderCompletionSphere = false
  var.lassoSelectionEnded = false
  var.lassoSelectionItemsCalculated = false
  var.lassoPLNodes = {}
end

local function drawLassoPolylineAction()
  local numNodes = #var.lassoPLNodes
  var.shouldRenderCompletionSphere = false
  if var.lassoActionHoveredNodeIndex == 1 and numNodes > 2 then
    if var.lassoSelectionEnded then
      var.shouldRenderCompletionSphere = false;
    else
      var.shouldRenderCompletionSphere = true;
    end
  end

  -- draw cursor sphere
  if not var.shouldRenderCompletionSphere then
    local hit
    if imgui.GetIO().WantCaptureMouse == false then
      hit = cameraMouseRayCast(false, imgui.flags(SOTTerrain))
    end
    if hit then
      local sphereRadius = (core_camera.getPosition() - hit.pos):length() * roadRiverGui.nodeSizeFactor
      debugDrawer:drawSphere(hit.pos, sphereRadius, roadRiverGui.highlightColors.node, false)
      if not tableIsEmpty(var.lassoPLNodes) then
        local tempNode = {pos = hit.pos, isUpdated = true}
        drawLassoLineSegmented(var.lassoPLNodes[numNodes], tempNode, var.lassoDrawInfo.type)
      end
    end
  end

  if tableIsEmpty(var.lassoPLNodes) then return end

  for index, node in ipairs(var.lassoPLNodes) do
    local nodeColor = roadRiverGui.highlightColors.node
    if var.lassoActionHoveredNodeIndex == index then
      nodeColor = roadRiverGui.highlightColors.hoveredNode
    elseif var.lassoActionSelectedNodeIndex == index then
      nodeColor = roadRiverGui.highlightColors.selectedNode
    end
    -- Skip first node if we should render completion sphere
    if index == 1 and var.shouldRenderCompletionSphere then
      goto continue
    else
      local sphereRadius = (core_camera.getPosition() - node.pos):length() * roadRiverGui.nodeSizeFactor
      debugDrawer:drawSphere(node.pos, sphereRadius, nodeColor, false)
    end
    if index > 1 then
      drawLassoLineSegmented(var.lassoPLNodes[index - 1], node, var.lassoDrawInfo.type)
    end
    ::continue::
  end

  -- finally draw the closing line if selection ended
  if var.lassoSelectionEnded then
    drawLassoLineSegmented(var.lassoPLNodes[numNodes], var.lassoPLNodes[1], var.lassoDrawInfo.type)
  end

  -- draw completion line and sphere
  if var.lassoSelectionEnded == false then
    if var.shouldRenderCompletionSphere then
      local sphereRadius = (core_camera.getPosition() - var.lassoPLNodes[1].pos):length() * roadRiverGui.nodeSizeFactor * 2
      debugDrawer:drawSphere(var.lassoPLNodes[1].pos, sphereRadius,  ColorF(0,1,0,0.5), false)
      var.lassoPLNodes[1].isUpdated = true
      drawLassoLineSegmented(var.lassoPLNodes[numNodes], var.lassoPLNodes[1], var.lassoDrawInfo.type)
    end
  end

  for _, node in ipairs(var.lassoPLNodes) do
    node.isUpdated = false
  end
end

local function getLassoAreas(layerID)
  local lassoAreas = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == var.enum_lassoDrawType.inclusionZone then
          table.insert(lassoAreas, lassoArea)
        end
      end
    break
    end
  end
  return lassoAreas
end

local function getAllLassoAreas(layerID)
  local lassoAreas = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        table.insert(lassoAreas, lassoArea)
      end
    break
    end
  end
  return lassoAreas
end

local function getLassoNodes(layerID)
  local lassoAreas = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == var.enum_lassoDrawType.inclusionZone then
          local lassoNodes = {}
          for _, node in ipairs(lassoArea.nodes) do
            table.insert(lassoNodes, node.pos)
          end
          table.insert(lassoAreas, lassoNodes)
        end
      end
      break
    end
  end
  return lassoAreas
end

local function getLassoNodesWithAreaID(layerID, lassoAreaID)
  local lassoAreas = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == var.enum_lassoDrawType.inclusionZone and lassoArea.lassoAreaID == lassoAreaID then
          local lassoNodes = {}
          for _, node in ipairs(lassoArea.nodes) do
            table.insert(lassoNodes, node.pos)
          end
          table.insert(lassoAreas, deepcopy(lassoNodes))
        end
      end
      break
    end
  end
  return lassoAreas
end

local function drawLassoAreas(layerID)
  for index, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID then
      for index, lassoArea in ipairs(layer.lassoAreas) do
        if highlighAnimation.isHighligthing and highlighAnimation.elapsed < highlighAnimation.duration then
          if layer.layerType == highlighAnimation.layerType and layer.layerID == highlighAnimation.layerID and
            lassoArea.zoneType == highlighAnimation.zoneType and lassoArea.lassoAreaID == highlighAnimation.zoneID then
            highlighAnimation.elapsed = highlighAnimation.elapsed + editor.getDeltaTime()
              if math.ceil(highlighAnimation.elapsed / 0.5) % 2 ~= 0 then
                goto continue
              end
          end
        end
        if highlighAnimation.elapsed > highlighAnimation.duration then
          highlighAnimation.isHighligthing = false
          highlighAnimation.elapsed = 0.0
        end

        local lineColor = nil
        if lassoArea.lassoAreaID == var.lassoHoveredNode.areaID and lassoArea.zoneType == var.lassoHoveredNode.areaType then
          lineColor = ColorF(0,1,0,0.5)
        end

        local numNodes = tableSize(lassoArea.nodes)
        for index, node in ipairs(lassoArea.nodes) do
          local nodeColor = roadRiverGui.highlightColors.node
          if var.lassoHoveredNode.index == index and var.lassoHoveredNode.layerID == layer.layerID and
             var.lassoHoveredNode.areaID == lassoArea.lassoAreaID and var.lassoHoveredNode.areaType == lassoArea.zoneType then
            nodeColor = roadRiverGui.highlightColors.hoveredNode
          end
          local sphereRadius = (core_camera.getPosition() - node.pos):length() * roadRiverGui.nodeSizeFactor
          debugDrawer:drawSphere(node.pos, sphereRadius, nodeColor, false)
          if index > 1 then
            drawLassoLineSegmented(lassoArea.nodes[index - 1], lassoArea.nodes[index], lassoArea.zoneType, lineColor)
          end
        end
        -- finally draw the closing line
        drawLassoLineSegmented(lassoArea.nodes[numNodes], lassoArea.nodes[1], lassoArea.zoneType, lineColor)
        ::continue::
      end
    end
  end
end

local function getAvailableLayerType(areaID)
  local layerType = nil
  local layers = getLayers(areaID)
  for _, layer in ipairs(layers) do
    layerType = layer.layerType
    if layerType == layerType_enum.terrain_material or layerType == layerType_enum.terrain_mask then
      break
    end
  end
  return layerType
end

local function getForestItemUIDsInLayer(layerType, layerID)
  if var.layers.forestItemsTbl == nil then var.layers.forestItemsTbl = {} end
  local forestItemsTbl = {}
  for _, layerItemsEntry in ipairs(var.layers.forestItemsTbl) do
    if layerItemsEntry.layerType == layerType and layerItemsEntry.layerID == layerID then
      forestItemsTbl = layerItemsEntry.forestItems
    end
  end
  return forestItemsTbl
end

local function deleteItemsInLayer(layerType, layerID)
  local itemUIDs = getForestItemUIDsInLayer(layerType, layerID)
  local forestData = forest:getData()
  for _, itemUID in ipairs(itemUIDs) do
    local item = forestData:findItemByUid(itemUID)
    editor.removeForestItem(forestData, item)
  end
end

local function setLayerName(layerType, layerID, name)
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerType == layerType and layer.layerID == layerID then
      layer.layerName = name
    end
  end
end

local function isForestBrushSelected(layerType, layerID, internalName, zoneType)
  local selected = false
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      selected = (indexOf(selectedItemsInfo.selectedItems, internalName) ~= -1)
      break
    end
  end
  return selected
end

local function getForestBrushSelection(layerType, layerID, zoneType)
  local selectionInfo = {}
  if isForestBrushSelected(layerType, layerID, noneBrushItemName, zoneType) then
    return selectionInfo
  end
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
       selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
        selectionInfo = selectedItemsInfo.selectedItems
      break
    end
  end
  if selectionInfo == nil then selectionInfo = {} end
  return selectionInfo
end

local function deselectForestBrush(layerType, layerID, internalName, zoneType)
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      local index = indexOf(selectedItemsInfo.selectedItems, internalName)
      if index ~= -1 then
        table.remove(selectedItemsInfo.selectedItems, index)
      end
      break
    end
  end
end

local function clearForestBrushSelection(layerType, layerID, zoneType)
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == (zoneType or enum_forestBrushItemZone.central) then
      selectedItemsInfo.selectedItems = {}
      break
    end
  end
end

local function getElementsForBrush(brushName)
  local forestBrushElements = {}
  local forestBrushElementIds = scenetree.findClassObjects("ForestBrushElement")
  for _, id in ipairs(forestBrushElementIds) do
    local fbe = scenetree.findObject(id)
    if fbe then
      local groupName = fbe:getGroup():getInternalName()
      if groupName == brushName then
        local fbeName = fbe:getInternalName()
        forestBrushElements[fbe:getId()] = fbeName
      end
    else
      editor.logWarn("Missing forest brush element ID: " .. tostring(id))
    end
  end
  return forestBrushElements
end

local function getForestBrushElementsFromSelection(layerType, layerID, zoneType)
  local forestBrushElements = {}
  local brushSelection = getForestBrushSelection(layerType, layerID, zoneType)
  for _, brushName in ipairs(brushSelection) do
    local elements = getElementsForBrush(brushName)
    for id, elementName in pairs(elements) do
      forestBrushElements[id] = elementName
    end
  end
  return forestBrushElements
end

local function getForestDensity(layerType, layerID)
  local density = nil
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "ForestDensity"then
          density = fieldData.val
          break
        end
      end
      break
    end
  end
  return density
end

local function setForestDensity(layerType, layerID, value)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "ForestDensity"then
          fieldData.val = value
          break
        end
      end
      break
    end
  end
end

local function getBorderDensity(layerType, layerID)
  local density = nil
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "BordersDensity"then
          density = fieldData.val
          break
        end
      end
      break
    end
  end
  return density
end

local function setBorderDensity(layerType, layerID, value)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "BordersDensity"then
          fieldData.val = value
          break
        end
      end
      break
    end
  end
end

local function getForestBorderFallOff(layerType, layerID)
  local density = nil
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "BordersFalloff"then
          density = fieldData.val
          break
        end
      end
      break
    end
  end
  return density
end

local function setForestBorderFallOff(layerType, layerID, value)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "BordersFalloff"then
          fieldData.val = value
          break
        end
      end
      break
    end
  end
end

local function removeItemsActionUndo(actionData)
  for _, item in pairs(actionData.items) do
    editor.addForestItem(var.forestData, item)
  end
end

local function removeItemsActionRedo(actionData)
  for _, item in pairs(actionData.items) do
    editor.removeForestItem(var.forestData, item)
  end
end

local function removeItems(items)
  if tableIsEmpty(items) then return end
  editor.history:commitAction("RemoveBiomeItems", {items = items}, removeItemsActionUndo, removeItemsActionRedo, true)
end

local function setFieldValue(fieldName, fieldValue, customData)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == customData.layerType and item.layerID == customData.layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == fieldName then
          if fieldData.name == "LayerName" then
            fieldData.val = fieldValue
            setLayerName(item.areaID, item.layerID, fieldValue)
          elseif fieldData.name == "FieldPlacement" then
            fieldData.val = fieldValue
          else
            local clampedValue = clamp(fieldValue, fieldData.minValue or -math.huge, fieldData.maxValue or math.huge)
            fieldData.val = clampedValue
          end
        end
      end
    end
  end
end

local function pasteLayerFieldValue(fieldName, copiedValue, arrayIndex, customData)
  local clampedValue = clamp(copiedValue, customData.minValue or -math.huge, customData.maxValue or math.huge)
  setFieldValue(fieldName, clampedValue, customData)
end

local function isExclusionZoneSelected(areaID, layerID, zoneID)
  for _, zone in ipairs(var.areas.exclusionZones) do
    if zone.areaID == areaID and zone.layerID == layerID then
      for _, zoneData in ipairs(zone.zoneData) do
        if zoneData.ID == zoneID then
          return zoneData.isSelected
        end
      end
    end
  end
  return false
end

local function isAnyZoneSelected(areaID, layerID)
  for _, zone in ipairs(var.areas.exclusionZones) do
    if zone.areaID == areaID and zone.layerID == layerID then
      for _, zoneData in ipairs(zone.zoneData) do
        if zoneData.isSelected then
          return zoneData.isSelected
        end
      end
    end
  end
  return false
end

local function setZoneSelected(areaID, layerID, zoneID, select)
  for _, zone in ipairs(var.areas.exclusionZones) do
    if zone.areaID == areaID and zone.layerID == layerID then
      for _, zoneData in ipairs(zone.zoneData) do
        if zoneData.ID == zoneID then
          zoneData.isSelected = select
        end
      end
    end
  end
end

local function deselectExclusionZone(areaID, layerID, zoneID)
  setZoneSelected(areaID, layerID, zoneID, false)
end

local function selectExclusionZone(areaID, layerID, zoneID)
  setZoneSelected(areaID, layerID, zoneID, true)
end

local function clearExZoneSelection(areaID, layerID)
  for _, zone in ipairs(var.areas.exclusionZones) do
    if zone.areaID == areaID and zone.layerID == layerID then
      for _, zoneData in ipairs(zone.zoneData) do
        zoneData.isSelected = false
      end
    end
  end
end

-- Add Items
local function addItemsActionUndo(actionData)
  for _, item in pairs(actionData.items) do
    editor.removeForestItem(var.forestData, item)
  end
end

local function addItemsActionRedo(actionData)
  for _, item in pairs(actionData.items) do
    editor.addForestItem(var.forestData, item)
  end
end

local function replaceItemsActionUndo(actionData)
  for i, item in pairs(actionData.newItems) do
    editor.removeForestItem(var.forestData, item)
  end
  for i, item in pairs(actionData.oldItems) do
    editor.addForestItem(var.forestData, item)
  end
end

local function replaceItemsActionRedo(actionData)
  for _, item in pairs(actionData.oldItems) do
    editor.removeForestItem(var.forestData, item)
  end
  for _, item in pairs(actionData.newItems) do
    editor.addForestItem(var.forestData, item)
  end
end

local function getBlendingMethod(layerType, layerID)
  local blendingMethodPtr = 0
  local itemFound = false
  for _, blendingData in ipairs(var.layers.layerBlendingComboIndexTbl) do
    if blendingData.layerType == layerType and blendingData.layerID == layerID then
      blendingMethodPtr = blendingData.blendingMethod
      itemFound = true
      break
    end
  end
  if not itemFound then
    table.insert(var.layers.layerBlendingComboIndexTbl, {layerType = layerType, layerID = layerID, blendingMethod = blending_enum.add})
    blendingMethodPtr = blending_enum.add
  end
  return blendingMethodPtr
end

local function setBlendingMethod(layerType, layerID, method)
  local itemFound = false
  for _, blendingData in ipairs(var.layers.layerBlendingComboIndexTbl) do
    if blendingData.layerType == layerType and blendingData.layerID == layerID then
      blendingData.blendingMethod = method
      itemFound = true
      break
    end
  end
  if not itemFound then
    table.insert(var.layers.layerBlendingComboIndexTbl, {layerType = layerType, layerID = layerID, blendingMethod = method})
  end
end

local function initEmptyFieldInfo()
  local isFieldAvailableFunc = function(fieldsTbl, fieldName)
    local available = false
    for _, field in ipairs(fieldsTbl) do
      if field.name == fieldName then
        available = true
        break
      end
    end
    return available
  end

  local layerType, layerID, fieldName, layerType
  for _, fieldInfos in ipairs(var.layers.fieldInfoTbl) do
    layerType = fieldInfos.layerType
    layerID = fieldInfos.layerID
    for _, fieldData in ipairs(fieldInfos.fieldsData) do
      for _, fieldInfo in ipairs(fieldInfoTemplate) do
        if layerType == fieldInfo.layerType then
          for _, rowPlFieldName in ipairs(rowPlacementFields) do
            if fieldInfo.name == rowPlFieldName then
              local fieldData = {
                name = fieldInfo.name,
                label = fieldInfo.label,
                val = fieldInfo.val,
                minValue = fieldInfo.minValue,
                maxValue = fieldInfo.maxValue,
                type = fieldInfo.type,
                layerType = fieldInfo.layerType
              }
              local fieldAvailable = isFieldAvailableFunc(getFieldsData(areaID, layerID), fieldInfo.name)
              if not fieldAvailable then
                insertFieldInfo(areaID, layerID, fieldData)
              end
            end
          end
        end
        ::continue::
      end
    end
  end
end

local function getFieldPlacement(layerType, layerID)
  local fieldPlacement = false
  local fieldName = "FieldPlacement"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      fieldPlacement = fieldData.val == "true"
      break
    end
  end
  return fieldPlacement
end

local function setFieldPlacement(layerType, layerID, val)
  local itemFound = false
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "FieldPlacement" then
      fieldData.val = (val and "true" or "false")
      break
    end
  end
end

local function getFieldRowDistance(layerType, layerID)
  local rowDist = false
  local fieldName = "FieldRowDistance"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      rowDist = fieldData.val
      break
    end
  end
  return rowDist
end

local function getFieldItemDistance(layerType, layerID)
  local itemDist = false
  local fieldName = "FieldItemDistance"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      itemDist = fieldData.val
      break
    end
  end
  return itemDist
end

local function getFieldRowOrientation(layerType, layerID)
  local rowOrientation = false
  local fieldName = "FieldRowOrientation"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      rowOrientation = fieldData.val
      break
    end
  end
  return rowOrientation
end

local function getSlopeRange(layerType, layerID)
  local slopeRange = {0, 90}
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "SlopeRange"then
          slopeRange = fieldData.val
          break
        end
      end
      break
    end
  end
  return slopeRange
end

local function setSlopeRange(layerType, layerID, range)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "SlopeRange"then
          local slopeRangeValid = range
          local slopeRange = getSlopeRange(layerType, layerID)
          slopeRangeValid[1] = math.min(range[1], slopeRange[2])
          slopeRangeValid[2] = math.max(range[1], range[2])
          fieldData.val = slopeRangeValid
          break
        end
      end
      break
    end
  end
end

var.getFieldMinMax = function(layerType, layerID, fieldName)
  local minMax = {0, 0}
  for _, fieldInfo in ipairs(fieldInfoTemplate) do
    if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
      minMax = {fieldInfo.minValue, fieldInfo.maxValue}
    end
  end
  return minMax
end

var.getEPRandomTiltRange = function (layerType, layerID)
  local slopeRange = {-5, 5}
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "EP_RandomTilt"then
          slopeRange = fieldData.val
          break
        end
      end
      break
    end
  end
  return slopeRange
end

var.setEPRandomTiltRange = function (layerType, layerID, range)
  local itemFound = false
  local fieldsData = getFieldsData(layerType, layerID)
  local slopeRangeValid = range
  local fieldName = "EP_RandomTilt"
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      local slopeRange = var.getEPRandomTiltRange(layerType, layerID)
      local minMax = var.getFieldMinMax(layerType, layerID, fieldName)
      slopeRangeValid[1] = clamp(math.min(range[1], slopeRange[2]), minMax[1], minMax[2])
      slopeRangeValid[2] = clamp(math.max(range[1], range[2]), minMax[1], minMax[2])
      fieldData.val = slopeRangeValid
      itemFound = true
      break
    end
  end
  if not itemFound then
    for _, fieldInfo in ipairs(fieldInfoTemplate) do
      if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
        table.insert(fieldsData, {name = fieldName, val = slopeRangeValid, minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
        return fieldInfo.val
      end
    end
  end
end

var.getEPFalloff = function(layerType, layerID)
  local itemFound = false
  local fieldName = "EP_BorderFalloff"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      itemFound = true
      return fieldData.val
    end
  end
  if not itemFound then
    for _, fieldInfo in ipairs(fieldInfoTemplate) do
      if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
        table.insert(fieldsData, {name = fieldName, val = fieldInfo.val, minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
        return fieldInfo.val
      end
    end
  end
  return nil
end

var.setEPFalloff = function(layerType, layerID, val)
  local itemFound = false
  local fieldsData = getFieldsData(layerType, layerID)
  local fieldName = "EP_BorderFalloff"
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      itemFound = true
      fieldData.val = clamp(val, fieldData.minValue, fieldData.maxValue)
      break
    end
  end
  if not itemFound then
    for _, fieldInfo in ipairs(fieldInfoTemplate) do
      if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
        table.insert(fieldsData, {name = fieldName, val = clamp(val, fieldInfo.minValue, fieldInfo.maxValue), minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
        return fieldInfo.val
      end
    end
  end
  return nil
end

var.getEPItemDistance = function(layerType, layerID)
  local itemFound = false
  local fieldsData = getFieldsData(layerType, layerID)
  local fieldName = "EP_ItemDistance"
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      itemFound = true
      return fieldData.val
    end
  end

  if not itemFound then
    for _, fieldInfo in ipairs(fieldInfoTemplate) do
      if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
        table.insert(fieldsData, {name = fieldName, val = fieldInfo.val, minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
        return fieldInfo.val
      end
    end
  end
end

var.setEPItemDistance = function(layerType, layerID, val)
  local itemFound = false
  local fieldsData = getFieldsData(layerType, layerID)
  local fieldName = "EP_ItemDistance"
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      local minMax = var.getFieldMinMax(layerType, layerID, fieldName)
      itemFound = true
      fieldData.val = clamp(val, minMax[1], minMax[2])
      break
    end
  end
  if not itemFound then
    for _, fieldInfo in ipairs(fieldInfoTemplate) do
      if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
        table.insert(fieldsData, {name = fieldName, val = clamp(val, fieldInfo.minValue, fieldInfo.maxValue), minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
        return fieldInfo.val
      end
    end
  end
  return nil
end

local function getSlopeInfluence(layerType, layerID)
  local slopeInfluence = 0
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "SlopeInfluence"then
          slopeInfluence = fieldData.val
          break
        end
      end
      break
    end
  end
  return slopeInfluence
end

local function setSlopeInfluence(layerType, layerID, value)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "SlopeInfluence"then
          fieldData.val = value
          break
        end
      end
      break
    end
  end
end

local function addForestItemUIDToLayerTable(layerType, layerID, itemUIDs)
  if var.layers.forestItemsTbl == nil then var.layers.forestItemsTbl = {} end
  local forestItemsTbl = nil
  for _, layerItemsEntry in ipairs(var.layers.forestItemsTbl) do
    if layerItemsEntry.layerType == layerType and layerItemsEntry.layerID == layerID then
      forestItemsTbl = layerItemsEntry.forestItems
      break
    end
  end
  if forestItemsTbl == nil then
    forestItemsTbl = {}
    table.insert(var.layers.forestItemsTbl, {layerType = layerType, layerID = layerID, forestItems = forestItemsTbl})
  end

  for _, forestItemUid in ipairs(itemUIDs) do
    table.insert(forestItemsTbl, forestItemUid)
  end
end

local function getTerrLayerMaterialIndex(layerID)
  local materialIndex = 0
  local fieldsData = getFieldsData(layerType_enum.terrain, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "TerrainMaterial" then
      materialIndex = fieldData.val
      break
    end
  end
  return materialIndex
end

local function setTerrLayerMaterial(layerID, materialIndex)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType_enum.terrain and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "TerrainMaterial" then
          fieldData.val = materialIndex
        end
      end
    end
  end
end

local function getTerrLayerMask(layerID)
  local maskFilePath = ""
  local fieldsData = getFieldsData(layerType_enum.terrain, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "TerrainMask" then
      maskFilePath = fieldData.val
      break
    end
  end
  return maskFilePath
end

local function setTerrLayerMask(layerID, maskFile)
  for _, item in ipairs(var.layers.fieldInfoTbl) do
    if item.layerType == layerType_enum.terrain and item.layerID == layerID then
      for _, fieldData in ipairs(item.fieldsData) do
        if fieldData.name == "TerrainMask" then
          fieldData.val = maskFile
        end
      end
    end
  end
end

local function getSelectedLayer(layerType)
  local layer = nil
  for _, selection in ipairs(var.layers.selectedLayerIDs) do
    if selection.layerType == layerType then
      layer = getLayer(layerType, selection.selectedLayerID)
      break
    end
  end
  return layer
end

local function biomeProcFunc(isDeletingLayer, layer, lassoAreaID, isResizingLasso)
  local layerID = layer.layerID
  local layerType = layer.layerType
  local randomMask = getRandomLayerMapFile(layerType, layerID)
  local maskFile = getTerrLayerMask(layerID)
  local materialIndex = getTerrLayerMaterialIndex(layerID)
  local centralElements = getForestBrushElementsFromSelection(layerType, layerID)
  local falloffElements = getForestBrushElementsFromSelection(layerType, layerID, enum_forestBrushItemZone.falloff)
  local lassoNodes = getLassoNodes(layerID)
  local forestDensity = getForestDensity(layerType, layerID) or 1.0
  local borderFallOff = getForestBorderFallOff(layerType, layerID) or 1.0
  local borderDensity = getBorderDensity(layerType, layerID) or 1.0
  local vegetationFalloff = 0
  local slopeInfluence = getSlopeInfluence(layerType, layerID) or 0
  local slopeRange = getSlopeRange(layerType, layerID)
  local slopeVal = {slopeInfluence, slopeRange[1], slopeRange[2]}
  local exclusionZones = {}
  local eraseExistingItems = false
  local blendingMethod = getBlendingMethod(layerType, layerID)
  local dragNodes = {}

  if isDeletingLayer then
    blendingMethod = blending_enum.delete
    forestDensity = 1.0
    borderDensity = 1.0
  elseif lassoAreaID ~= nil and layerType == layerType_enum.area and not isResizingLasso then
    blendingMethod = blending_enum.delete
    lassoNodes = getLassoNodesWithAreaID(layerID, lassoAreaID)
    forestDensity = 1.0
    borderDensity = 1.0
  elseif lassoAreaID ~= nil and isResizingLasso and layerType == layerType_enum.area then
    blendingMethod = blending_enum.replace
    lassoNodes = getLassoNodesWithAreaID(layerID, lassoAreaID)
    dragNodes = var.lassoNodeDragInitialNodes
  end

  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == var.enum_lassoDrawType.exclusionZone then
          local lassoNodes = {}
          for _, node in ipairs(lassoArea.nodes) do
            table.insert(lassoNodes, node.pos)
          end
          table.insert(exclusionZones, lassoNodes)
        end
      end
      break
    end
  end

  local forestItemUIDs = getForestItemUIDsInLayer(layerType, layerID)
  local matIndex = -1
  if layerType == layerType_enum.terrain then
    if randomMask ~= "" then
      maskFile = randomMask
    end
    if maskFile == "" then
      matIndex = materialIndex
    end
    var.forestBrushTool:initBiomeMatProc(maskFile, matIndex, {}, centralElements, falloffElements, exclusionZones or {}, forestDensity, borderFallOff, vegetationFalloff, borderDensity, slopeVal, blendingMethod, forestItemUIDs)
  elseif layerType == layerType_enum.area then
    eraseExistingItems = false
    local shouldPlaceField = getFieldPlacement(layerType, layerID)
    if shouldPlaceField then
      local rowDistance = getFieldRowDistance(layerType, layerID)
      local itemDistance = getFieldItemDistance(layerType, layerID)
      local rowOrientation = getFieldRowOrientation(layerType, layerID)
      var.forestBrushTool:initBiomeFieldProc(centralElements, lassoNodes, dragNodes, rowDistance, itemDistance, rowOrientation, blendingMethod, forestItemUIDs)
    else
      if randomMask ~= "" then
        var.forestBrushTool:initBiomeMatProc(randomMask, matIndex, {}, centralElements, falloffElements, exclusionZones or {}, forestDensity, borderFallOff, vegetationFalloff, borderDensity, slopeVal, blendingMethod, forestItemUIDs)
      else
        var.forestBrushTool:initBiomeLassoProc(lassoNodes, dragNodes, centralElements, falloffElements, exclusionZones or {}, forestDensity, borderFallOff, vegetationFalloff,
                                                borderDensity, blendingMethod, slopeVal, forestItemUIDs)
      end
    end
  end
end

local function drawLayerProperties(layer)
  imgui.Text(layer.layerName)
  local layerType = layer.layerType
  local layerID = layer.layerID
  local layer = getLayerWithType(layerType, layerID)
  local buttonSize = imgui.ImVec2(150, 30)

  imgui.BeginChild1("MainPanel#"..layer.layerType..layer.layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y-(imgui.GetStyle().FramePadding.y)), true)
  imgui.BeginChild1("LayerActionsPanel"..layer.layerType..layer.layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, 40 * imgui.uiscale[0]), nil)
  imgui.SameLine()

  local errCode = var.forestBrushTool:getBiomeError()
  if var.forestBrushTool:isBiomeProcCompleted() and errCode == 0 then
    var.forestBrushTool:insertBiomeItems()
    local itemsTbl = var.forestBrushTool:getBiomeItems()
    local itemsToAdd = {}
    local itemsToDel = {}
    itemsToAdd = itemsTbl["items"]
    itemsToDel = itemsTbl["itemsToDel"]
    local blendingMethod = getBlendingMethod(areaID, layerID)
    if blendingMethod == blending_enum.replace then
      if not tableIsEmpty(itemsToAdd) or not tableIsEmpty(itemsToDel) then
        local delItems = {}
        for _, item in pairs(itemsToDel) do
          table.insert(delItems, item)
        end
        editor.history:commitAction("ReplaceBiomeItems", {oldItems = delItems, newItems = itemsToAdd}, replaceItemsActionUndo, replaceItemsActionRedo, true)
      end
    elseif blendingMethod == blending_enum.add then
      if not tableIsEmpty(itemsToAdd) then
        local itemUIDs = {}
        local itemKeys = {}
        for _, key in ipairs(itemsToAdd) do
          table.insert(itemKeys, key)
        end

        ---local forestData = forest:getData()
        ---for _, item in pairs(itemsToAdd) do
          --local uids = forestData:generateAndSetItemUid(itemKeys)
          --table.insert(itemUIDs, uid)
        ---end
        addForestItemUIDToLayerTable(layerType, layerID, itemUIDs)
        editor.history:commitAction("AddBiomeItems", {items = itemsToAdd}, addItemsActionUndo, addItemsActionRedo, true)
      end
    elseif blendingMethod == blending_enum.delete then
      if not tableIsEmpty(itemsToDel) then
        local delItems = {}
        for _, item in pairs(itemsToDel) do
          table.insert(delItems, item)
        end
        editor.history:commitAction("RemoveBiomeItems", {items = delItems}, removeItemsActionUndo, removeItemsActionRedo, true)
      end
    end
    var.forestBrushTool:resetBiomeProcState()
  end

  imgui.SetNextWindowSize(imgui.ImVec2(300, 100), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("No Forest Item Selected") then
    imgui.TextUnformatted("No Central Forest Item Selected!")
    if imgui.Button("OK") then
      imgui.CloseCurrentPopup()
    end
    imgui.EndPopup()
  end

  imgui.EndChild()

  local getFieldByNameFunc = function (fieldName, layerType, layerID)
    local field = nil
    for index, item in ipairs(var.layers.fieldInfoTbl) do
      if item.layerType == layerType and item.layerID == layerID then
        for _, fieldData in ipairs(item.fieldsData) do
          if fieldData.name == fieldName then
            field = fieldData
          end
        end
      end
    end
    return field
  end

  local panelWidth = imgui.GetContentRegionAvail().x
  imgui.BeginChild1("LayerMainPanel##"..tostring(layer.layerType)..tostring(layer.layerID), imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y), nil)

  local syncSelectedBrushListFunc = function (zoneType)
    local selectionList = {}
    for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
      if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
        selectedItemsInfo.zoneType == zoneType then
        selectionList = selectedItemsInfo.selectedItems
        break
      end
    end
    local itemFound = false
    for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
      if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
        selectedItemsInfo.zoneType == zoneType then
        selectedItemsInfo.selectedItems = deepcopy(selectionList)
        itemFound = true
        break
      end
    end

    if not itemFound then
      local brushZoneType = zoneType or enum_forestBrushItemZone.central
      local selectionData = {layerType = layerType, layerID = layerID, zoneType = brushZoneType, selectedItems = selectionList}
      table.insert(var.layers.forestBrushSelectedItems, selectionData)
    end
  end

  local syncSelectedTempBrushListFunc = function (zoneType)
    local selectionList = {}
    local itemFound = false
    for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
      if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
        selectedItemsInfo.zoneType == zoneType then
        selectionList = selectedItemsInfo.selectedItems
        break
      end
    end

    for _, selectedItemsInfo in ipairs(var.forestBrushTempSelectedItems) do
      if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID and
        selectedItemsInfo.zoneType == zoneType then
        selectedItemsInfo.selectedItems = deepcopy(selectionList)
        itemFound = true
        break
      end
    end

    if not itemFound then
      local selectionData = {layerType = layerType, layerID = layerID, zoneType = zoneType, selectedItems = deepcopy(selectionList)}
      table.insert(var.forestBrushTempSelectedItems, selectionData)
    end
  end

  --imgui.SetNextWindowPos(imgui.ImVec2(imgui.GetWindowViewport().Size.x * 0.5, imgui.GetWindowViewport().Size.y * 0.5))
  imgui.SetNextWindowPos(imgui.ImVec2(imgui.GetWindowPos().x + imgui.GetWindowSize().x * 0.5 - 150, imgui.GetWindowPos().y + imgui.GetWindowSize().y * 0.5))
  local framePadding =  imgui.ImVec2(3, 3) --imgui.GetStyle().FramePadding
  imgui.SetNextWindowSize(imgui.ImVec2(300, 100), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("Biome Work Progress") then
    if errCode == 0 then
      var.forestBrushTool:runBiomeProcess()
    end
    local progressStr = var.forestBrushTool:getBiomeWorkName()
    local progressPercent = var.forestBrushTool:getBiomeWorkProgress()
    local buttonText = "Cancel"
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      progressStr = "Error: " ..var.forestBrushTool:getBiomeErrorStr()
      imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
      buttonText = "Ok"
    end
    imgui.TextUnformatted(progressStr)
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      imgui.PopStyleColor()
    end

    if errCode == 0 then
      imgui.ProgressBar(progressPercent, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0), string.format("%d%%", progressPercent * 100))
    end

    if imgui.Button(buttonText) then
      var.forestBrushTool:quitBiomeProcess()
      imgui.CloseCurrentPopup()
    end
    if var.forestBrushTool:isBiomeProcCompleted() then
      if errCode == 0 then
        imgui.CloseCurrentPopup()
      end
    end
    imgui.EndPopup()
  end

  imgui.SetNextWindowPos(imgui.ImVec2(imgui.GetWindowPos().x + imgui.GetWindowSize().x * 0.5 - 150, imgui.GetWindowPos().y + imgui.GetWindowSize().y * 0.5))
  local framePadding =  imgui.ImVec2(3, 3)
  imgui.SetNextWindowSize(imgui.ImVec2(300, 100), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("Biome Edge Placement Progress") then
    if errCode == 0 then
      local edgeElements = getForestBrushElementsFromSelection(layer.layerType, layer.layerID, enum_forestBrushItemZone.edge)
      var.forestBrushTool:runBiomeEdgeProc();
    end
    local progressStr = var.forestBrushTool:getBiomeWorkName()
    local progressPercent = var.forestBrushTool:getBiomeWorkProgress()
    local buttonText = "Cancel"
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      progressStr = "Error: " ..var.forestBrushTool:getBiomeErrorStr()
      imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
      buttonText = "Ok"
    end
    imgui.TextUnformatted(progressStr)
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      imgui.PopStyleColor()
    end

    if errCode == 0 then
      imgui.ProgressBar(progressPercent, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0), string.format("%d%%", progressPercent * 100))
    end

    if imgui.Button(buttonText) then
      var.forestBrushTool:quitBiomeProcess()
      imgui.CloseCurrentPopup()
    end
    if var.forestBrushTool:isBiomeProcCompleted() then
      var.forestBrushTool:insertBiomeItems()
      local itemsTbl = var.forestBrushTool:getBiomeItems()
      local itemsToAdd = {}
      local itemsToDel = {}
      itemsToAdd = itemsTbl["items"]
      itemsToDel = itemsTbl["itemsToDel"]
      if not tableIsEmpty(itemsToAdd) then
        editor.history:commitAction("AddBiomeItems", {items = itemsToAdd}, addItemsActionUndo, addItemsActionRedo, true)
      end
      var.forestBrushTool:resetBiomeProcState()
      if errCode == 0 then
        imgui.CloseCurrentPopup()
      end
    end
    imgui.EndPopup()
  end

  imgui.SetNextWindowPos(imgui.ImVec2(imgui.GetWindowPos().x + imgui.GetWindowSize().x * 0.5, imgui.GetWindowPos().y + imgui.GetWindowSize().y * 0.5))
  if imgui.BeginPopupModal("No Brush Selected!") then
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
    imgui.TextUnformatted("Please make brush selection!")
    imgui.PopStyleColor()

    if imgui.Button("Ok") then
      imgui.CloseCurrentPopup()
    end
    imgui.EndPopup()
  end
  imgui.SetNextWindowPos(imgui.ImVec2(imgui.GetWindowPos().x + imgui.GetWindowSize().x * 0.5, imgui.GetWindowPos().y + imgui.GetWindowSize().y * 0.5))
  if imgui.BeginPopupModal("No Lasso Areas!") then
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
    imgui.TextUnformatted("Please create at least one lasso area!")
    imgui.PopStyleColor()

    if imgui.Button("Ok") then
      imgui.CloseCurrentPopup()
    end
    imgui.EndPopup()
  end
  local fieldData = getFieldByNameFunc("LayerName", layerType, layerID)
  local textSize = imgui.CalcTextSize("Blending Method:").x
  local posX = imgui.GetCursorPosX()
  imgui.Text("Layer Name:")
  imgui.SameLine()
  imgui.SetCursorPosX(posX + textSize + 5)
  imgui.SetNextItemWidth(200)
  editor.uiInputText('##LayerName'..layerType..layerID, editor.getTempCharPtr(layer.layerName), nil, imgui.InputTextFlags_ReadOnly)
  imgui.SameLine()
  imgui.SetCursorPosX(panelWidth - 180)

  if var.forestBrushTool:isBiomeProcRunning() then
    imgui.BeginDisabled()
  end
  if imgui.Button("Generate Layer", imgui.ImVec2(150, 30)) then
    local isAddBlending = (getBlendingMethod(layerType, layerID) == blending_enum.add)
    local brushCentral = getForestBrushSelection(layerType, layerID, enum_forestBrushItemZone.central)
    local brushBorder = getForestBrushSelection(layerType, layerID, enum_forestBrushItemZone.falloff)
    local bothBrushesEmpty = tableIsEmpty(brushCentral) and tableIsEmpty(brushBorder)
    local noLasso = tableIsEmpty(getLassoAreas(layerID))

    if isAddBlending and bothBrushesEmpty then
      imgui.OpenPopup("No Brush Selected!")
    elseif noLasso and layerType == layerType_enum.area then
      imgui.OpenPopup("No Lasso Areas!")
    else
      local isDeletingLayer = false
      biomeProcFunc(isDeletingLayer, layer)
      imgui.OpenPopup("Biome Work Progress")
    end
  end
  if var.forestBrushTool:isBiomeProcRunning() then
    imgui.EndDisabled()
  end

  imgui.Text("Blending Method:")
  imgui.SameLine()
  imgui.SetCursorPosX(posX + textSize + 5)
  imgui.SetNextItemWidth(200)
  local blendingMethodPtr = imgui.IntPtr(0)
  blendingMethodPtr[0] = getBlendingMethod(layerType, layerID)
  if imgui.Combo1("##layersBlendingMth"..layerType..layerID, blendingMethodPtr, layerBlendingComboItems) then
    setBlendingMethod(layerType, layerID, blendingMethodPtr[0])
  end

  if layer.layerType == layerType_enum.terrain then
    imgui.Text("Layer Material:")
    imgui.SameLine()
    imgui.SetCursorPosX(posX + textSize + 5)
    imgui.SetNextItemWidth(200)
    local mtlPtr = imgui.IntPtr(0)
    mtlPtr[0] = getTerrLayerMaterialIndex(layerID)
    if imgui.Combo1("##terrainLayerMaterialCombo"..layerType..layerID, mtlPtr, imgui.ArrayCharPtrByTbl(layerCreateMtlComboItemsTbl)) then
      setTerrLayerMaterial(layerID, mtlPtr[0])
    end

    imgui.Dummy(imgui.ImVec2(5,10))
    imgui.Text("Layer Mask:")
    imgui.SameLine()
    imgui.SetCursorPosX(posX + textSize + 5)
    imgui.SetNextItemWidth(200)
    local maskFile = getTerrLayerMask(layerID)
    editor.uiInputText('##LayerMaskFile'..layerType..layerID, editor.getTempCharPtr(maskFile), nil, imgui.InputTextFlags_ReadOnly)
    if imgui.IsItemHovered() and maskFile ~= "" then
      imgui.SetTooltip(maskFile)
    end
    imgui.SameLine()
    if editor.uiIconImageButton(
      editor.icons.folder,
      imgui.ImVec2(22, 22)
    ) then
      local levelPath, levelName, _ = path.split(getMissionFilename())
      editor_fileDialog.openFile(
        function(data)
          maskFilePath = imgui.ArrayChar(256, data.filepath)
          setTerrLayerMask(layerID, ffi.string(maskFilePath))
        end,
        {{"Images",{".png", ".jpg"}},{"PNG", ".png"}, {"JPG", ".jpg"}},
        false, levelPath, true)
    end
    imgui.SameLine()
    if imgui.Button("Clear", imgui.ImVec2(40, 30)) then
      setTerrLayerMask(layerID, "")
    end

    imgui.SetCursorPosX(posX + textSize + 5)
    local imgPosStart = imgui.GetCursorPos()
    local texture = editor.getTempTextureObj(getTerrLayerMask(layerID))
    imgui.Image(texture.tex:getID(), imgui.ImVec2(200, 200), nil, nil, nil, editor.color.white.Value)
    if getTerrLayerMask(layerID) == "" then
      local textSize = imgui.CalcTextSize("No Mask Selected!")
      local posX = 100 - textSize.x/2
      local posY = 100 - textSize.y/2
      imgui.SetCursorPos(imgui.ImVec2(imgPosStart.x + posX, imgPosStart.y + posY))
      imgui.Text("No Mask Selected!")
    end
    imgui.SetCursorPos(imgui.ImVec2(0, imgPosStart.y + 205))
  end

  imgui.SetCursorPosY(imgui.GetCursorPosY() + 10)
  imgui.Text("Layer Brush:")
  local brush = getForestBrushSelection(layerType, layerID, enum_forestBrushItemZone.central)
  local brushName = tableIsEmpty(brush) and "None" or brush[1]
  imgui.SameLine()
  imgui.SetCursorPosX(posX + textSize + 5)
  local firstWidgetPos = imgui.GetCursorPosX()
  imgui.SetCursorPosX(firstWidgetPos)
  imgui.BeginChild1("CentralBrushesSelected"..layerType..layerID, imgui.ImVec2(180, 100), imgui.WindowFlags_ChildWindow)

  local noBrushSelected = true
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == enum_forestBrushItemZone.central then
      for index, item in ipairs(var.forestBrushes) do
        if (indexOf(selectedItemsInfo.selectedItems, item.internalName) ~= -1) then
          noBrushSelected = false
          break
        end
      end
    end
  end
  if noBrushSelected then
    local noBrushTextSize = imgui.CalcTextSize("No Brush Selected!")
    imgui.SetCursorPos(imgui.ImVec2(90 - noBrushTextSize.x/2, 50 - noBrushTextSize.y/2))
    imgui.Text("No Brush Selected!")
  end

  for index, item in ipairs(var.forestBrushes) do
    if not isForestBrushSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.central) then goto continue end
    --imgui.PushStyleColor2(imgui.Col_Button, (isForestBrushTempSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.central)) and var.buttonColor_active or var.buttonColor_inactive)
    editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
    imgui.SameLine()
    local textPos = imgui.GetCursorPos()
    if imgui.Button("##CentralBrush"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
      selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.central)
      deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.central)
    end
    imgui.tooltip(item.internalName)
    imgui.SetCursorPos(textPos)
    imgui.Text(item.internalName)
    ::continue::
  end
  imgui.EndChild()

  imgui.SameLine()
  if imgui.Button("Select Brush##Central", imgui.ImVec2(100, 30)) then
    imgui.OpenPopup("Select Forest Brush (Central)")
    var.layers.PopupOpenMousePos.x = imgui.GetMousePos().x
    var.layers.PopupOpenMousePos.y = imgui.GetMousePos().y
    syncSelectedTempBrushListFunc(enum_forestBrushItemZone.central)
  end

  local shouldWrap = false
  if imgui.GetContentRegionAvailWidth() < 720 then
    shouldWrap = true
  end

  if not shouldWrap then
    imgui.SameLine()
  end

  local posXCol2 = imgui.GetCursorPosX()
  imgui.SetCursorPosX(posXCol2 + 20)
  imgui.Text("Density:")
  imgui.SameLine()
  if shouldWrap then
    imgui.SetCursorPosX(firstWidgetPos)
  else
    imgui.SetCursorPosX(posXCol2 + imgui.CalcTextSize("Density:").x + 25)
  end
  imgui.SetNextItemWidth(120)

  local layerDensityPtr = imgui.FloatPtr(getForestDensity(layerType, layerID))
  editor.uiInputFloat("##LayerDensity"..layerType..layerID, layerDensityPtr, 0.1, 1.0, nil, nil, editEnded)
  if editEnded[0] then
    local minMaxPair = getFieldMinMax("ForestDensity")
    local value = layerDensityPtr[0]
    if not tableIsEmpty(minMaxPair) then
      value = clamp(layerDensityPtr[0], minMaxPair[1], minMaxPair[2])
    end
    setForestDensity(layerType, layerID, value)
  end

  imgui.Separator()

  imgui.Text("Border Brush:")
  local borderBrush = getForestBrushSelection(layerType, layerID, enum_forestBrushItemZone.falloff)
  local borderBrushName = tableIsEmpty(borderBrush) and "None" or borderBrush[1]
  imgui.SameLine()
  imgui.SetCursorPosX(posX + textSize + 5)

  imgui.SetCursorPosX(firstWidgetPos)
  imgui.BeginChild1("FalloffBrushesSelected"..layerType..layerID, imgui.ImVec2(180, 100), imgui.WindowFlags_ChildWindow)

  local noBrushSelected = true
  for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
    if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
      selectedItemsInfo.zoneType == enum_forestBrushItemZone.falloff then
      for index, item in ipairs(var.forestBrushes) do
        if (indexOf(selectedItemsInfo.selectedItems, item.internalName) ~= -1) then
          noBrushSelected = false
          break
        end
      end
    end
  end
  if noBrushSelected then
    local noBrushTextSize = imgui.CalcTextSize("No Brush Selected!")
    imgui.SetCursorPos(imgui.ImVec2(90 - noBrushTextSize.x/2, 50 - noBrushTextSize.y/2))
    imgui.Text("No Brush Selected!")
  end

  for index, item in ipairs(var.forestBrushes) do
    if not isForestBrushSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.falloff) then goto continue end
    editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
    imgui.SameLine()
    local textPos = imgui.GetCursorPos()
    if imgui.Button("##FalloffBrush"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
      selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.falloff)
      deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.falloff)
    end
    imgui.tooltip(item.internalName)
    imgui.SetCursorPos(textPos)
    imgui.Text(item.internalName)
    ::continue::
  end
  imgui.EndChild()

  imgui.SameLine()
  if imgui.Button("Select Brush##Falloff", imgui.ImVec2(100, 30)) then
    imgui.OpenPopup("Select Forest Brush (Falloff)")
    var.layers.PopupOpenMousePos.x = imgui.GetMousePos().x
    var.layers.PopupOpenMousePos.y = imgui.GetMousePos().y
    syncSelectedTempBrushListFunc(enum_forestBrushItemZone.falloff)
  end

  local forestBrPopupPos = imgui.ImVec2(var.layers.PopupOpenMousePos.x + 20, var.layers.PopupOpenMousePos.y - 250)
  imgui.SetNextWindowPos(forestBrPopupPos)
  imgui.SetNextWindowSize(imgui.ImVec2(300, 500), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("Select Forest Brush (Central)") then
    imgui.BeginChild1("CentralBrushesCentralPopup"..layerType..layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y - 50), imgui.WindowFlags_ChildWindow)
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
    local isNoneBrushSelected = isForestBrushTempSelected(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.central)
    imgui.PushStyleColor2(imgui.Col_Button, (isNoneBrushSelected) and var.buttonColor_active or var.buttonColor_inactive)
    editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
    imgui.SameLine()
    local textPos = imgui.GetCursorPos()
    if imgui.Button("##NoneCentral", imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
      clearForestBrushTempSelection()
      selectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.central)
    end
    imgui.SetCursorPos(textPos)
    imgui.Text("- NONE -")
    imgui.PopStyleColor()
    imgui.PopStyleColor()

    for index, item in ipairs(var.forestBrushes) do
      local isBrushSelected = isForestBrushTempSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.central)
      imgui.PushStyleColor2(imgui.Col_Button, (isBrushSelected) and var.buttonColor_active or var.buttonColor_inactive)
      editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
      imgui.SameLine()
      local textPos = imgui.GetCursorPos()
      if imgui.Button("##Falloff"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
        if isBrushSelected then
          deselectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.central)
        else
          selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.central)
        end
        deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.central)
      end
      imgui.SetCursorPos(textPos)
      imgui.Text(item.internalName)
      imgui.PopStyleColor()
    end
    imgui.EndChild()
    if imgui.Button("OK") then
      imgui.CloseCurrentPopup()
      syncSelectedBrushListFunc(enum_forestBrushItemZone.central)
    end
    imgui.SameLine()
    if imgui.Button("Cancel") then
      imgui.CloseCurrentPopup()
    end
    if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Escape)) then
      imgui.CloseCurrentPopup()
    end

    if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Enter)) then
      syncSelectedBrushListFunc(enum_forestBrushItemZone.central)
      imgui.CloseCurrentPopup()
    end
    imgui.EndPopup()
  end

  local forestBrPopupPos = imgui.ImVec2(var.layers.PopupOpenMousePos.x + 20, var.layers.PopupOpenMousePos.y - 250)
  imgui.SetNextWindowPos(forestBrPopupPos)
  imgui.SetNextWindowSize(imgui.ImVec2(300, 500), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("Select Forest Brush (Falloff)") then
    imgui.BeginChild1("FalloffBrushesCentralPopup"..layerType..layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y - 50), imgui.WindowFlags_ChildWindow)
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
    imgui.PushStyleColor2(imgui.Col_Button, (isForestBrushTempSelected(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.falloff)) and var.buttonColor_active or var.buttonColor_inactive)
    editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
    imgui.SameLine()
    local textPos = imgui.GetCursorPos()
    if imgui.Button("##NoneFalloff", imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
      clearForestBrushTempSelection()
      selectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.falloff)
    end
    imgui.SetCursorPos(textPos)
    imgui.Text("- NONE -")
    imgui.PopStyleColor()
    imgui.PopStyleColor()

    for index, item in ipairs(var.forestBrushes) do
      local isBrushSelected = isForestBrushTempSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.falloff)
      imgui.PushStyleColor2(imgui.Col_Button, isBrushSelected and var.buttonColor_active or var.buttonColor_inactive)
      editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
      imgui.SameLine()
      local textPos = imgui.GetCursorPos()
      if imgui.Button("##Falloff"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
        if isBrushSelected then
          deselectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.falloff)
        else
          selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.falloff)
        end
        deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.falloff)
      end
      imgui.SetCursorPos(textPos)
      imgui.Text(item.internalName)
      imgui.PopStyleColor()
    end
    imgui.EndChild()
    if imgui.Button("OK") then
      imgui.CloseCurrentPopup()
      syncSelectedBrushListFunc(enum_forestBrushItemZone.falloff)
    end
    imgui.SameLine()
    if imgui.Button("Cancel") then
      imgui.CloseCurrentPopup()
    end
    if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Escape)) then
      imgui.CloseCurrentPopup()
    end
    if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Enter)) then
      syncSelectedBrushListFunc(enum_forestBrushItemZone.falloff)
      imgui.CloseCurrentPopup()
    end
    imgui.EndPopup()
  end

  if not shouldWrap then
    imgui.SameLine()
  end
  imgui.SetCursorPosX(posXCol2 + 20)
  imgui.Text("Density:")
  local borderBrush =  "Oak Trees"
  imgui.SameLine()
  if not shouldWrap then
    imgui.SetCursorPosX(posXCol2 + imgui.CalcTextSize("Density:").x + 25)
  else
    imgui.SetCursorPosX(firstWidgetPos)
  end
  imgui.SetNextItemWidth(120)
  local borderDensityPtr = imgui.FloatPtr(getBorderDensity(layerType, layerID))

  editor.uiInputFloat("##BorderDensity"..layerType..layerID, borderDensityPtr, 0.1, 1.0, nil, nil, editEnded)
  if editEnded[0] then
    local minMaxPair = getFieldMinMax("BordersDensity")
    local value = borderDensityPtr[0]
    if not tableIsEmpty(minMaxPair) then
      value = clamp(value, minMaxPair[1], minMaxPair[2])
    end
    setBorderDensity(layerType, layerID, value)
  end

  imgui.SameLine()
  imgui.SetCursorPosX(imgui.GetCursorPos().x + 20)
  imgui.Text("Border Width:")
  imgui.SameLine()
  imgui.SetCursorPosX(imgui.GetCursorPos().x + 5)
  imgui.SetNextItemWidth(120)
  local widthEditEnded = imgui.BoolPtr(false)
  local borderFalloffPtr = imgui.FloatPtr(getForestBorderFallOff(layerType, layerID))
  editor.uiInputFloat("##BorderWidth"..layerType..layerID, borderFalloffPtr, 1.0, 1.0, nil, nil, editEnded)
  if editEnded[0] then
    local minMaxPair = getFieldMinMax("BordersFalloff")
    local value = borderFalloffPtr[0]
    if not tableIsEmpty(minMaxPair) then
      value = clamp(borderFalloffPtr[0], minMaxPair[1], minMaxPair[2])
    end
    setForestBorderFallOff(layerType, layerID, value)
  end



  imgui.Separator()

  imgui.Text("Slope Influence:")
  imgui.SameLine()
  imgui.SetCursorPosX(posX + textSize + 5)
  imgui.SetNextItemWidth(120)

  local slopeInfluencePtr = imgui.FloatPtr(getSlopeInfluence(layerType, layerID))
  editor.uiInputFloat("##SlopeInfluence"..layerType..layerID, slopeInfluencePtr, 1.0, 1.0, nil, nil, editEnded)
  if editEnded[0] then
    local minMaxPair = getFieldMinMax("SlopeInfluence")
    local value = slopeInfluencePtr[0]
    if not tableIsEmpty(minMaxPair) then
      value = clamp(slopeInfluencePtr[0], minMaxPair[1], minMaxPair[2])
    end
    setSlopeInfluence(layerType, layerID, value)
  end

  imgui.SameLine()
  local tempBoolPtr = imgui.BoolPtr(true)

  local slopeRange = getSlopeRange(layerType, layerID)
  input2FloatValue[0] = slopeRange[1]
  input2FloatValue[1] = slopeRange[2]
  imgui.SetNextItemWidth(120)
  editor.uiInputFloat2("##SlopeRange" .. layerType .. layerID, input2FloatValue, "%.2f", nil, editEnded)
  imgui.tooltip("Slope Inf. From-To")
  if editEnded[0] then
    local minMaxPair = getFieldMinMax("SlopeRange")
    local minVal = input2FloatValue[0]
    local maxVal = input2FloatValue[1]
    --[[if not tableIsEmpty(minMaxPair) then
      minVal = clamp(input2FloatValue[0], minMaxPair[1], math.min(maxVal, minMaxPair[2]))
      maxVal = clamp(input2FloatValue[1], math.max(minVal, minMaxPair[1]), minMaxPair[2])
    end]]
    setSlopeRange(layerType, layerID, {minVal, maxVal})
  end

  if layer.layerType == layerType_enum.area then
    imgui.Separator()
    imgui.Text("Place Field:")
    imgui.SameLine()
    var.fieldPlacementBoolPtr[0] = getFieldPlacement(layerType, layerID)
    imgui.SetCursorPosX(posX + textSize + 5)
    if imgui.Checkbox("##fieldPlacementEnabled", var.fieldPlacementBoolPtr) then
      setFieldPlacement(layerType, layerID, var.fieldPlacementBoolPtr[0])
    end

    if var.fieldPlacementBoolPtr[0] then
      imgui.Text("Item Distance:")
      imgui.SameLine()
      imgui.SetCursorPosX(posX + textSize + 5)
      local fieldItemDistPtr = imgui.FloatPtr(getFieldItemDistance(layerType, layerID))
      imgui.SetNextItemWidth(120)
      editor.uiInputFloat("##fieldItemDistance"..layerType..layerID, fieldItemDistPtr, 1.0, 1.0, nil, nil, editEnded)
      if editEnded[0] then
        local fieldName = "FieldItemDistance"
        local fieldsData = getFieldsData(layerType, layerID)
        for _, fieldData in ipairs(fieldsData) do
          if fieldData.name == fieldName then
            fieldData.val = fieldItemDistPtr[0]
            break
          end
        end
      end

      imgui.Text("Row Distance:")
      imgui.SameLine()
      imgui.SetCursorPosX(posX + textSize + 5)
      local fieldRowDistPtr = imgui.FloatPtr(getFieldRowDistance(layerType, layerID))
      imgui.SetNextItemWidth(120)
      editor.uiInputFloat("##fieldRowDistance"..layerType..layerID, fieldRowDistPtr, 1.0, 1.0, nil, nil, editEnded)
      if editEnded[0] then
        local fieldName = "FieldRowDistance"
        local fieldsData = getFieldsData(layerType, layerID)
        for _, fieldData in ipairs(fieldsData) do
          if fieldData.name == fieldName then
            fieldData.val = fieldRowDistPtr[0]
            break
          end
        end
      end

      imgui.Text("Row Orientation:")
      imgui.SameLine()
      imgui.SetCursorPosX(posX + textSize + 5)
      local fieldRowOriPtr = imgui.FloatPtr(getFieldRowOrientation(layerType, layerID))
      imgui.SetNextItemWidth(120)
      editor.uiInputFloat("##fieldRowOrientation"..layerType..layerID, fieldRowOriPtr, 1.0, 1.0, nil, nil, editEnded)
      if editEnded[0] then
        local fieldName = "FieldRowOrientation"
        local fieldsData = getFieldsData(layerType, layerID)
        for _, fieldData in ipairs(fieldsData) do
          if fieldData.name == fieldName then
            fieldData.val = fieldRowOriPtr[0]
            break
          end
        end
      end
    end
  end

  imgui.Separator()
  imgui.Text("Place Edges:")
  imgui.SameLine()

  local getFieldValue = function(fieldName)
    local itemFound = false
    local fieldsData = getFieldsData(layerType, layerID)
    for _, fieldData in ipairs(fieldsData) do
      if fieldData.name == fieldName then
        itemFound = true
        return fieldData.val
      end
    end
    if not itemFound then
      for _, fieldInfo in ipairs(fieldInfoTemplate) do
        if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
          table.insert(fieldsData, {name = fieldName, val = fieldInfo.val, minValue = fieldInfo.minValue, maxValue = fieldInfo.maxValue})
          return fieldInfo.val
        end
      end
    end
    return nil
  end

  local setFieldValue = function(fieldName, value)
    local itemFound = false
    local fieldsData = getFieldsData(layerType, layerID)
    for _, fieldData in ipairs(fieldsData) do
      if fieldData.name == fieldName then
        fieldData.val = clamp(value, fieldData.minValue, fieldData.maxValue)
        itemFound = true
        break
      end
    end
    if not itemFound then
      for _, fieldInfo in ipairs(fieldInfoTemplate) do
        if (fieldInfo.layerType == layerType_enum.any or layerType == fieldInfo.layerType) and fieldInfo.name == fieldName then
          table.insert(fieldsData, {name = fieldName, val = clamp(fieldInfo.val, fieldInfo.minValue, fieldInfo.maxValue)})
          return fieldInfo.val
        end
      end
    end
  end

  local edgePlacement = false
  local fieldName = "EdgePlacement"
  local fieldsData = getFieldsData(layerType, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == fieldName then
      edgePlacement = (fieldData.val == "true")
      break
    end
  end
  var.edgePlacementBoolPtr[0] = edgePlacement
  local cursorPosY = imgui.GetCursorPosY()
  posX = imgui.GetCursorPosX()
  imgui.SetCursorPosX(firstWidgetPos)
  if imgui.Checkbox("##edgePlacementEnabled", var.edgePlacementBoolPtr) then
    local itemFound = false
    for _, fieldData in ipairs(fieldsData) do
      if fieldData.name == fieldName then
        fieldData.val = (var.edgePlacementBoolPtr[0] and "true" or "false")
        itemFound = true
        break
      end
    end
    if not itemFound then
      table.insert(fieldsData, {name = fieldName, val = (var.edgePlacementBoolPtr[0] and "true" or "false")})
    end
  end
  if var.edgePlacementBoolPtr[0] then
    if imgui.BeginPopupModal("No Brush Selected!##Edge") then
      imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
      imgui.TextUnformatted("You need to select a border brush element!")
      imgui.PopStyleColor()

      if imgui.Button("Ok") then
        imgui.CloseCurrentPopup()
      end
      imgui.EndPopup()
    end
    imgui.SetCursorPos(imgui.ImVec2(panelWidth - 180, cursorPosY))
    if imgui.Button("Create Edges", imgui.ImVec2(150, 30)) then
      local edgeElements = getForestBrushElementsFromSelection(layerType, layerID, enum_forestBrushItemZone.edge)
      if tableIsEmpty(edgeElements) then
        imgui.OpenPopup("No Brush Selected!##Edge")
      else
        local layerType = layer.layerType
        local layerID = layer.layerID
        local maskFile = getTerrLayerMask(layerID)
        local matIndex = getTerrLayerMaterialIndex(layerID)
        local falloff = var.getEPFalloff(layerType, layerID) or 1.0
        local randomTilt = var.getEPRandomTiltRange(layerType, layerID) or 1.0
        local itemDistance = var.getEPItemDistance(layerType, layerID) or 1.0
        local blendingMethod = getBlendingMethod(layerType, layerID)
        local fileSuffix = tostring(layerType).."_"..tostring(layerID)
        local edgeElements = getForestBrushElementsFromSelection(layerType, layerID, enum_forestBrushItemZone.edge)
        local lassoAreas = {}
        if layerType == layerType_enum.area then
          lassoAreas = getLassoNodes(layerID)
        maskFile = ""
          matIndex = -1
        end
        var.forestBrushTool:initBiomeEdgeProc(edgeElements, maskFile, matIndex, falloff, blendingMethod, fileSuffix, randomTilt[1], randomTilt[2], itemDistance, lassoAreas)
        imgui.OpenPopup("Biome Edge Placement Progress")
      end
    end

    imgui.Text("Edge Brush:")
    local borderBrush = getForestBrushSelection(layerType, layerID, enum_forestBrushItemZone.edge)
    local borderBrushName = tableIsEmpty(borderBrush) and "None" or borderBrush[1]
    imgui.SameLine()

    imgui.SetCursorPosX(firstWidgetPos)
    imgui.BeginChild1("EdgeBrushesSelected"..layerType..layerID, imgui.ImVec2(180, 100), imgui.WindowFlags_ChildWindow)

    local noBrushSelected = true
    for _, selectedItemsInfo in ipairs(var.layers.forestBrushSelectedItems) do
      if selectedItemsInfo.layerType == layerType and selectedItemsInfo.layerID == layerID  and
        selectedItemsInfo.zoneType == enum_forestBrushItemZone.edge then
        for index, item in ipairs(var.forestBrushes) do
          if (indexOf(selectedItemsInfo.selectedItems, item.internalName) ~= -1) then
            noBrushSelected = false
            break
          end
        end
      end
    end
    if noBrushSelected then
      local noBrushTextSize = imgui.CalcTextSize("No Brush Selected!")
      imgui.SetCursorPos(imgui.ImVec2(90 - noBrushTextSize.x/2, 50 - noBrushTextSize.y/2))
      imgui.Text("No Brush Selected!")
    end

    for index, item in ipairs(var.forestBrushes) do
      if not isForestBrushSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.edge) then goto continue end
      editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
      imgui.SameLine()
      local textPos = imgui.GetCursorPos()
      if imgui.Button("##EdgeBrush"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
        selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.edge)
        deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.edge)
      end
      imgui.tooltip(item.internalName)
      imgui.SetCursorPos(textPos)
      imgui.Text(item.internalName)
      ::continue::
    end
    imgui.EndChild()

    local forestBrPopupPos = imgui.ImVec2(var.layers.PopupOpenMousePos.x + 20, var.layers.PopupOpenMousePos.y - 250)
    imgui.SetNextWindowPos(forestBrPopupPos)
    imgui.SetNextWindowSize(imgui.ImVec2(300, 500), imgui.Cond_FirstUseEver)
    if imgui.BeginPopupModal("Select Forest Brush (Edge)") then
      imgui.BeginChild1("EdgeBrushesPopup"..layerType..layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y - 50), imgui.WindowFlags_ChildWindow)
      imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
      local isNoneBrushSelected = isForestBrushTempSelected(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.central)
      imgui.PushStyleColor2(imgui.Col_Button, (isNoneBrushSelected) and var.buttonColor_active or var.buttonColor_inactive)
      editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
      imgui.SameLine()
      local textPos = imgui.GetCursorPos()
      if imgui.Button("##NoneEdge", imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
        clearForestBrushTempSelection()
        selectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.edge)
      end
      imgui.SetCursorPos(textPos)
      imgui.Text("- NONE -")
      imgui.PopStyleColor()
      imgui.PopStyleColor()

      for index, item in ipairs(var.forestBrushes) do
        local isBrushSelected = isForestBrushTempSelected(layerType, layerID, item.internalName, enum_forestBrushItemZone.edge)
        imgui.PushStyleColor2(imgui.Col_Button, (isBrushSelected) and var.buttonColor_active or var.buttonColor_inactive)
        editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(math.ceil(imgui.GetFontSize()), math.ceil(imgui.GetFontSize())))
        imgui.SameLine()
        local textPos = imgui.GetCursorPos()
        if imgui.Button("##Edge"..item.id, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), math.ceil(imgui.GetFontSize()))) then
          if isBrushSelected then
            deselectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.edge)
          else
            selectForestTempBrush(layerType, layerID, item.internalName, enum_forestBrushItemZone.edge)
          end
          deselectForestTempBrush(layerType, layerID, noneBrushItemName, enum_forestBrushItemZone.edge)
        end
        imgui.SetCursorPos(textPos)
        imgui.Text(item.internalName)
        imgui.PopStyleColor()
      end
      imgui.EndChild()
      if imgui.Button("OK") then
        imgui.CloseCurrentPopup()
        syncSelectedBrushListFunc(enum_forestBrushItemZone.edge)
      end
      imgui.SameLine()
      if imgui.Button("Cancel") then
        imgui.CloseCurrentPopup()
      end
      if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Escape)) then
        imgui.CloseCurrentPopup()
      end

      if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Enter)) then
        syncSelectedBrushListFunc(enum_forestBrushItemZone.edge)
        imgui.CloseCurrentPopup()
      end
      imgui.EndPopup()
    end

    imgui.SameLine()
    if imgui.Button("Select Brush##Edge", imgui.ImVec2(100, 30)) then
      imgui.OpenPopup("Select Forest Brush (Edge)")
      var.layers.PopupOpenMousePos.x = imgui.GetMousePos().x
      var.layers.PopupOpenMousePos.y = imgui.GetMousePos().y
      syncSelectedTempBrushListFunc(enum_forestBrushItemZone.edge)
    end
    imgui.Text("Item Distance:")
    imgui.SameLine()
    imgui.SetCursorPosX(firstWidgetPos)
    local fieldName = "EP_ItemDistance"
    local itemDistance = getFieldValue(fieldName)
    local itemDistancePtr = imgui.FloatPtr(itemDistance)
    imgui.SetNextItemWidth(120)
    editor.uiInputFloat("##EP_ItemDistance"..layerType..layerID, itemDistancePtr, 1.0, 1.0, nil, nil, editEnded)
    if editEnded[0] then
      var.setEPItemDistance(layerType, layerID, itemDistancePtr[0])
    end

    imgui.Text("Random Tilt:")
    imgui.SameLine()
    imgui.SetCursorPosX(firstWidgetPos)
    local randomTiltRange = var.getEPRandomTiltRange(layerType, layerID)
    input2FloatValue[0] = randomTiltRange[1]
    input2FloatValue[1] = randomTiltRange[2]
    imgui.SetNextItemWidth(120)
    editor.uiInputFloat2("##EP_RandomTilt" .. layerType .. layerID, input2FloatValue, "%.2f", nil, editEnded)
    imgui.tooltip("Random Tilt Range")
    if editEnded[0] then
      var.setEPRandomTiltRange(layerType, layerID, {input2FloatValue[0], input2FloatValue[1]})
    end

    imgui.Text("Border Falloff:")
    imgui.SameLine()
    imgui.SetCursorPosX(firstWidgetPos)
    fieldName = "EP_BorderFalloff"
    local borderFalloff = getFieldValue(fieldName)
    local borderFalloffPtr = imgui.FloatPtr(borderFalloff)
    imgui.SetNextItemWidth(120)
    editor.uiInputFloat("##EP_BorderFalloff"..layerType..layerID, borderFalloffPtr, 1.0, 1.0, nil, nil, editEnded)
    if editEnded[0] then
      var.setEPFalloff(layerType, layerID, borderFalloffPtr[0])
    end
  end

  imgui.EndChild()
  imgui.EndChild()
end

local function populateMaterialsList()
  layerCreateMtlComboItemsTbl = {}
  if terrainBlock then
    local mtls = terrainBlock:getMaterials()
    for index, mtl in ipairs(mtls) do
      table.insert(layerCreateMtlComboItemsTbl, mtl.internalName)
    end
  end
end

local function getMaterialName(index)
  local materialName = ""
  if terrainBlock then
    local mtls = terrainBlock:getMaterials()
    for matIndex, mtl in ipairs(mtls) do
      if matIndex == index then
        materialName = mtl.internalName
        break
      end
    end
  end
  return materialName
end

local function setItemTransformUndo(actionData)
  for index, item in ipairs(actionData.items) do
    actionData.items[index] = editor.updateForestItem(var.forestData, item:getKey(), item:getPosition(), item:getData(), editor.tableToMatrix(actionData.oldTransforms[index]), item:getScale())
  end
end

local function setItemTransformRedo(actionData)
  for index, item in ipairs(actionData.items) do
    actionData.items[index] = editor.updateForestItem(var.forestData, item:getKey(), item:getPosition(), item:getData(), editor.tableToMatrix(actionData.newTransforms[index]), item:getScale())
  end
end

local function selectLayer(layer)
  local found = false
  for _, selection in ipairs(var.layers.selectedLayerIDs) do
    if selection.layerType == layer.layerType then
      selection.selectedLayerID = layer.layerID
      found = true
    end
  end
  if not found then
    table.insert(var.layers.selectedLayerIDs, {layerType = layer.layerType, selectedLayerID = layer.layerID})
  end
end

local function deselectLayer(layer)
  local found = false
  for _, selection in ipairs(var.layers.selectedLayerIDs) do
    if selection.layerType == layer.layerType and selection.selectedLayerID == layer.layerID then
      selection.selectedLayerID = -1
    end
  end
end

local function isLayerSelected(layer)
  local selected = false
  for _, selection in ipairs(var.layers.selectedLayerIDs) do
    if selection.layerType == layer.layerType and selection.selectedLayerID == layer.layerID then
      selected = true
      break
    end
  end
  return selected
end

local function deleteLassoArea(layerID, areaID, zoneType)
  local index = -1
  local delLayer = nil
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for lassoIndex, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.lassoAreaID == areaID and lassoArea.zoneType == zoneType then
          index = lassoIndex
          delLayer = layer
          break
        end
      end
    end
  end
  if index ~= -1 then
    table.remove(delLayer.lassoAreas, index)
  end
end

local function deleteItemsInArea(layerID, lassoAreaID)
  local lassoNodes2D = {}
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == var.enum_lassoDrawType.inclusionZone and lassoArea.lassoAreaID == lassoAreaID then
          for _, node in ipairs(lassoArea.nodes) do
            table.insert(lassoNodes2D, Point2F(node.pos.x, node.pos.y))
          end
          break
        end
      end
    end
  end

  local forestItems = var.forestData:getItemsPolygon(lassoNodes2D)
  if not forestItems then return end

  --local itemUIDs = getForestItemUIDsInLayer(layerType_enum.area, layerID)
  local forestData = forest:getData()
  --for _, itemUID in ipairs(itemUIDs) do
    for _, item in ipairs(forestItems) do
      --if item:getUid() == itemUID then
        editor.removeForestItem(forestData, item)
      --end
    end
  --end
end

local function drawLassoAreasList(layer)
  local panelHeight = math.max(imgui.GetContentRegionAvail().y, 200)
  imgui.BeginChild1("AreasList##"..layer.layerType..layer.layerID, imgui.ImVec2(imgui.GetContentRegionAvail().x, panelHeight - 60), imgui.WindowFlags_ChildWindow)
  local lassoAreas = getAllLassoAreas(layer.layerID)
  if tableIsEmpty(lassoAreas) then
    local noAreaText = "N O   L A S S O   A R E A   A V A I L A B L E !"
    imgui.SetCursorPos(imgui.ImVec2(imgui.GetContentRegionAvail().x/2 - imgui.CalcTextSize(noAreaText).x/2, imgui.GetContentRegionAvail().y/2))
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    editor.uiTextColoredWithFont(imgui.ImVec4(1, 0, 0, 1), noAreaText, "cairo_bold")
    imgui.PopStyleColor()
  else
    imgui.SetCursorPosX(imgui.GetContentRegionAvail().x/2 - 150)
    local arealistHeight = math.max(imgui.GetContentRegionAvail().y, 200)
    imgui.BeginChild1("AreasListContainer##"..layer.layerType..layer.layerID, imgui.ImVec2(300, arealistHeight - 10), imgui.WindowFlags_ChildWindow)
    imgui.Text("Lasso Areas")
    local buttonSize = imgui.ImVec2(180, 30)
    local framePadding = imgui.ImVec2(3, 3)

    local area = nil
    for index = #lassoAreas, 1, -1 do
      area = lassoAreas[index]
      local p1 = imgui.ImVec2(imgui.GetCursorPos().x - 32, imgui.GetCursorPos().y - 32)
      local p2 = imgui.ImVec2(imgui.GetCursorPos().x + 150 + 32, imgui.GetCursorPos().y + 30 + 32)
      imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1, p2, imgui.GetColorU322(imgui.ImVec4(1, 0, 0, 1)))
      ---imgui.BeginDisabled()
      if area.zoneType == var.enum_lassoDrawType.inclusionZone then
        if imgui.Button("Area "..area.lassoAreaID, buttonSize) then
          highlighAnimation.isHighligthing = true
          highlighAnimation.layerType = layer.layerType
          highlighAnimation.layerID = layer.layerID
          highlighAnimation.zoneType = var.enum_lassoDrawType.inclusionZone
          highlighAnimation.zoneID = area.lassoAreaID
          highlighAnimation.elapsed = 0.0
        end
      elseif area.zoneType == var.enum_lassoDrawType.exclusionZone then
        if imgui.Button("Exclusion Zone "..area.lassoAreaID, buttonSize) then
          highlighAnimation.isHighligthing = true
          highlighAnimation.layerType = layer.layerType
          highlighAnimation.layerID = layer.layerID
          highlighAnimation.zoneType = var.enum_lassoDrawType.exclusionZone
          highlighAnimation.zoneID = area.lassoAreaID
          highlighAnimation.elapsed = 0.0
        end
      end
      --imgui.EndDisabled()
      imgui.SameLine()
      imgui.Dummy(imgui.ImVec2(5,1))
      imgui.SameLine()

      imgui.SetNextWindowSize(imgui.ImVec2(300, 100), imgui.Cond_FirstUseEver)
      if imgui.BeginPopupModal("Delete Lasso Area##"..area.lassoAreaID) then
        imgui.TextUnformatted("Are you sure you want to delete \"".."Area "..area.lassoAreaID.."\"?")
        if imgui.Button("Cancel") then
          imgui.CloseCurrentPopup()
        end
        imgui.SameLine()
        if imgui.Button("OK") then
          deleteItemsInArea(layerType_enum.area, layer.layerID)
          deleteLassoArea(layer.layerID, area.lassoAreaID, area.zoneType)
          imgui.CloseCurrentPopup()
        end
        imgui.EndPopup()
      end

      if imgui.Button("Delete##Area"..layer.layerID..area.lassoAreaID, imgui.ImVec2(50, 30)) then
        if area.zoneType == var.enum_lassoDrawType.inclusionZone then
          local isDeletingLayer = false
          biomeProcFunc(isDeletingLayer, layer, area.lassoAreaID)

          var.forestBrushTool:runBiomeProcess()
          var.forestBrushTool:insertBiomeItems()
          local itemsTbl = var.forestBrushTool:getBiomeItems()
          local itemsToAdd = {}
          local itemsToDel = {}
          itemsToAdd = itemsTbl["items"]
          itemsToDel = itemsTbl["itemsToDel"]
          if not tableIsEmpty(itemsToDel) then
            local delItems = {}
            for _, item in pairs(itemsToDel) do
              table.insert(delItems, item)
            end
            editor.history:commitAction("RemoveBiomeItems", {items = delItems}, removeItemsActionUndo, removeItemsActionRedo, true)
          end
        end
        deleteLassoArea(layer.layerID, area.lassoAreaID, area.zoneType)
      end
    end
    imgui.EndChild()
  end
  imgui.EndChild()

  local buttonSize = imgui.ImVec2(150, 30)
  imgui.SetCursorPosX(imgui.GetContentRegionAvail().x/2 - buttonSize.x - 20)
  local cursorPosMsg = imgui.GetCursorPosX()
  if imgui.Button("Add New Lasso Area", buttonSize) then
    var.lassoDrawInfo.layerType = layerType
    var.lassoDrawInfo.layerID = layerID
    var.lassoDrawInfo.type = var.enum_lassoDrawType.inclusionZone
    isDrawingLassoArea = true
  end
  imgui.SameLine()
  imgui.Dummy(imgui.ImVec2(20,1))
  imgui.SameLine()
  if imgui.Button("Add Exclusion Zone", buttonSize) then
    var.lassoDrawInfo.layerType = layerType
    var.lassoDrawInfo.layerID = layerID
    var.lassoDrawInfo.type = var.enum_lassoDrawType.exclusionZone
    isDrawingLassoArea = true
  end

  if isDrawingLassoArea then
    local txtSize = imgui.CalcTextSize("Please draw the area on map")
    imgui.SetCursorPosX(cursorPosMsg - math.abs(buttonSize.x - txtSize.x)/2)
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    imgui.Text("Please draw the area on map")
    imgui.PopStyleColor()
  end
end

local function getLassoAreaCount(layerType, layerID)
  local count = 0
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      count = tableSize(layer.lassoAreas)
      break
    end
  end
  return count
end

local function setHeaderState(label, state)
  local context = imgui.GetCurrentContext()
  local id = imgui.GetID1(label)
  imgui.ImGuiStorage_SetInt(imgui.GetStateStorage(), id, state)
end

local isRenamingLayer = false
local function drawLayersListWithType(layerType)
  local layers = getLayersWithType(layerType)
  imgui.BeginChild1("LayersList", imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y - 4), imgui.WindowFlags_ChildWindow)
  for _, layer in ipairs(layers) do
    if imgui.CollapsingHeader1(layer.layerName..'##'..layer.layerID, setHeaderState(layer.layerName..'##'..layer.layerID, isLayerSelected(layer))) then
      if shouldUpdateAreasScroll then
        shouldUpdateAreasScroll = false
        imgui.SetScrollHereY(-20)
      end
      local renamingState = isRenamingLayer
      if not renamingState then
        imgui.BeginDisabled()
      end
      imgui.SetNextItemWidth(150)
      inputTextValue = editor.getTempCharPtr(layer.layerName)
      editor.uiInputText("", inputTextValue, imgui.ArraySize(inputTextValue), imgui.InputTextFlags_AutoSelectAll, nil, nil, editEnded)
      if editEnded[0] then
        isRenamingLayer = false
        setLayerName(layer.layerType, layer.layerID,  ffi.string(inputTextValue))
      end
      if not renamingState then
        imgui.EndDisabled()
      end

      imgui.SameLine()

      local buttonSize = imgui.ImVec2(150, 30)
      local cursorPosX = imgui.GetCursorPosX()
      if imgui.Button("Rename Layer", buttonSize) then
        isRenamingLayer = true
      end

      imgui.SameLine()
      if imgui.Button("Delete Layer", buttonSize) then
        imgui.OpenPopup("Delete Layer")
      end

      imgui.SetNextWindowSize(imgui.ImVec2(200, 100), imgui.Cond_FirstUseEver)
      if imgui.BeginPopupModal("Delete Layer") then
        imgui.TextUnformatted("Are you sure you want to delete \""..layer.layerName.."\"?")
        if imgui.Button("Cancel") then
          imgui.CloseCurrentPopup()
        end
        imgui.SameLine()
        if imgui.Button("OK") then
          local isDeletingLayer = true
          biomeProcFunc(isDeletingLayer, layer)

          var.forestBrushTool:runBiomeProcess()
          var.forestBrushTool:insertBiomeItems()
          local itemsTbl = var.forestBrushTool:getBiomeItems()
          local itemsToAdd = {}
          local itemsToDel = {}
          itemsToAdd = itemsTbl["items"]
          itemsToDel = itemsTbl["itemsToDel"]
          if not tableIsEmpty(itemsToDel) then
            local delItems = {}
            for _, item in pairs(itemsToDel) do
              table.insert(delItems, item)
            end
            editor.history:commitAction("RemoveBiomeItems", {items = delItems}, removeItemsActionUndo, removeItemsActionRedo, true)
          end

          deleteLayer(layer.layerType, layer.layerID)
          imgui.CloseCurrentPopup()
        end
        imgui.EndPopup()
      end

      local brush = getForestBrushSelection(layer.layerType, layer.layerID, enum_forestBrushItemZone.central)
      local brushName = tableIsEmpty(brush) and "None" or brush[1]
      imgui.Dummy(imgui.ImVec2(10,1))
      imgui.SameLine()
      local widgetStartPosX = imgui.GetCursorPosX()
      imgui.Text("Layer Brush:")
      imgui.SameLine()
      imgui.SetNextItemWidth(150)

      imgui.BeginDisabled()
      editor.uiInputText("##inputTextBrushName#"..layer.layerType..layer.layerID, editor.getTempCharPtr(brushName))
      imgui.EndDisabled()
      imgui.SameLine()
      imgui.Dummy(imgui.ImVec2(10,1))
      imgui.SameLine()
      imgui.Text("Brush Density:")
      imgui.SameLine()
      imgui.SetNextItemWidth(150)
      imgui.BeginDisabled()
      editor.uiInputText("##inputTextDensity##"..layer.layerType..layer.layerID, editor.getTempCharPtr(string.format("%.1f", getForestDensity(layer.layerType, layer.layerID))))
      imgui.EndDisabled()

      if layer.layerType == layerType_enum.area then
        local shouldWrap = false
        if imgui.GetContentRegionAvailWidth() < 720 then
          shouldWrap = true
        end

        if not shouldWrap then
          imgui.SameLine()
        else
          imgui.SetCursorPosX(widgetStartPosX)
        end
        imgui.Text("Area Count:")
        imgui.SameLine()
        imgui.SetNextItemWidth(150)
        imgui.BeginDisabled()
        editor.uiInputText("##inputTextDensity##"..layer.layerType..layer.layerID, editor.getTempCharPtr(tostring(getLassoAreaCount(layer.layerType, layer.layerID))))
        imgui.EndDisabled()
      end

      --imgui.SetCursorPosX(cursorPosX)
      --if imgui.Button("Duplicate Layer", buttonSize) then
      --end

      if layer.layerType == layerType_enum.area then
        drawLassoAreasList(layer)
      end
    end
    if imgui.IsItemClicked() then
      if not isLayerSelected(layer) then
        selectLayer(layer)
      --else
      --  deselectLayer(layer)
      end
    end
  end
  imgui.EndChild()
end

local function drawLevelBiomeToolbar()
  local buttonSize = imgui.ImVec2(150, 30)
  imgui.BeginChild1("MainToolbar", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), levelBiomeToolbarHeight), true)
  if imgui.Button("(Re)generate all layers", buttonSize) then
  end
  imgui.SameLine()
  if imgui.Button("Undo", buttonSize) then
  end
  imgui.SameLine()
  if imgui.Button("Redo", buttonSize) then
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonLBToolbar", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = levelBiomeToolbarHeight + imgui.GetIO().MouseDelta.y
      levelBiomeToolbarHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawLevelBiomeLayersList()
  imgui.BeginChild1("LevelBiomeLayers", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), levelBiomeLevelsListHeight), true)
  imgui.Text("Terrain Layers:")
  local buttonSize = imgui.ImVec2(150, 30)
  imgui.BeginChild1("MainPanelLevelBiome", imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y-(buttonSize.y+imgui.GetStyle().FramePadding.y*6)), true)
  local layers = getLayersWithType(layerType_enum.terrain)
  if tableIsEmpty(layers) then
    local noAreaText = "N O   T E R R A I N   L A Y E R   A V A I L A B L E !"
    imgui.SetCursorPos(imgui.ImVec2(imgui.GetContentRegionAvail().x/2 - imgui.CalcTextSize(noAreaText).x/2, imgui.GetContentRegionAvail().y/2))
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    editor.uiTextColoredWithFont(imgui.ImVec4(1, 0, 0, 1), noAreaText, "cairo_bold")
    imgui.PopStyleColor()
  else
    drawLayersListWithType(layerType_enum.terrain)
  end
  imgui.EndChild()

  imgui.SetCursorPosX(imgui.GetContentRegionAvail().x/2 - buttonSize.x/2)
  imgui.SetCursorPosY(imgui.GetCursorPosY()+imgui.GetStyle().FramePadding.y*2)

  if imgui.Button("New Terrain Layer", buttonSize) then
    local layer = addLayerWithType(layerType_enum.terrain)
    selectLayer(layer)
    shouldUpdateAreasScroll = true
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonLBLevels", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = levelBiomeLevelsListHeight + imgui.GetIO().MouseDelta.y
      levelBiomeLevelsListHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawLevelBiomeLayerProperties()
  imgui.BeginChild1("LevelBiomeLayerProps", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), levelBiomeLevelPropsHeight), true)
  local selectedLayer = getSelectedLayer(layerType_enum.terrain)
  if selectedLayer == nil then
    local noAreaText = "N O   L A Y E R   S E L E C T E D !"
    imgui.SetCursorPos(imgui.ImVec2(imgui.GetContentRegionAvail().x/2 - imgui.CalcTextSize(noAreaText).x/2, imgui.GetContentRegionAvail().y/2))
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    editor.uiTextColoredWithFont(imgui.ImVec4(1, 0, 0, 1), noAreaText, "cairo_bold")
    imgui.PopStyleColor()
  else
    drawLayerProperties(selectedLayer)
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonLBLevelsProps", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = levelBiomeLevelPropsHeight + imgui.GetIO().MouseDelta.y
      levelBiomeLevelPropsHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawBiomeAreasToolbar()
  local buttonSize = imgui.ImVec2(150, 30)
  imgui.BeginChild1("MainToolbar", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), biomeAreasToolbarHeight), true)
  if imgui.Button("(Re)generate all layers", buttonSize) then
  end
  imgui.SameLine()
  if imgui.Button("Undo", buttonSize) then
  end
  imgui.SameLine()
  if imgui.Button("Redo", buttonSize) then
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonLBToolbar", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = biomeAreasToolbarHeight + imgui.GetIO().MouseDelta.y
      biomeAreasToolbarHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawBiomeAreasLayersList()
  imgui.BeginChild1("AreaLayers", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), biomeAreasLevelsListHeight), true)
  imgui.Text("Area Layers:")
  local buttonSize = imgui.ImVec2(150, 30)
  imgui.BeginChild1("MainPanelAL", imgui.ImVec2(imgui.GetContentRegionAvail().x, imgui.GetContentRegionAvail().y-(buttonSize.y+imgui.GetStyle().FramePadding.y*6)), true)

  local layers = getLayersWithType(layerType_enum.area)
  if tableIsEmpty(layers) then
    local noAreaText = "N O   A R E A   L A Y E R   A V A I L A B L E !"
    imgui.SetCursorPos(imgui.ImVec2(imgui.GetContentRegionAvail().x/2 - imgui.CalcTextSize(noAreaText).x/2, imgui.GetContentRegionAvail().y/2))
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    editor.uiTextColoredWithFont(imgui.ImVec4(1, 0, 0, 1), noAreaText, "cairo_bold")
    imgui.PopStyleColor()
  else
    drawLayersListWithType(layerType_enum.area)
  end
  imgui.EndChild()

  imgui.SetCursorPosX(imgui.GetContentRegionAvail().x/2 - buttonSize.x/2)
  imgui.SetCursorPosY(imgui.GetCursorPosY()+imgui.GetStyle().FramePadding.y*2)

  if imgui.Button("New Area Layer", buttonSize) then
    local layer = addLayerWithType(layerType_enum.area)
    shouldUpdateAreasScroll = true
    selectLayer(layer)
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonALLevels", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = biomeAreasLevelsListHeight + imgui.GetIO().MouseDelta.y
      biomeAreasLevelsListHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawBiomeAreasLayerProperties()

  imgui.BeginChild1("BiomeAreaLayerProps", imgui.ImVec2((imgui.GetContentRegionAvail().x - 2), biomeAreasLevelPropsHeight), true)
  local selectedLayer = getSelectedLayer(layerType_enum.area)
  if selectedLayer == nil then
    local noAreaText = "N O   L A Y E R   S E L E C T E D !"
    imgui.SetCursorPos(imgui.ImVec2(imgui.GetContentRegionAvail().x/2 - imgui.CalcTextSize(noAreaText).x/2, imgui.GetContentRegionAvail().y/2))
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1));
    editor.uiTextColoredWithFont(imgui.ImVec4(1, 0, 0, 1), noAreaText, "cairo_bold")
    imgui.PopStyleColor()
  else
    drawLayerProperties(selectedLayer)
  end
  imgui.EndChild()

  local separatorPos = imgui.GetCursorPos()
  imgui.InvisibleButton("SeparatorButtonALLevelsProps", imgui.ImVec2(imgui.GetContentRegionAvail().x, seperatorHeight))
  if imgui.IsItemActive() then
    if imgui.GetIO().MouseDelta.y ~= 0 and imgui.IsMouseDown(0)then
      local newHeightVal = biomeAreasLevelPropsHeight + imgui.GetIO().MouseDelta.y
      biomeAreasLevelPropsHeight = clamp(newHeightVal, 50, 1000)
    end
  end
  if imgui.IsItemActive() or imgui.IsItemHovered() then
    imgui.SetMouseCursor(3)
  end
  imgui.SetCursorPosY(separatorPos.y + seperatorHeight / 2)
  imgui.Separator()
end

local function drawMainToolbar()
  local buttonSize = imgui.ImVec2(150, 30)
  imgui.BeginChild1("MainToolbar", imgui.ImVec2((imgui.GetContentRegionAvail().x - 6), 50), true)
  if imgui.Button("Open Project", buttonSize) then
  end
  imgui.SameLine()
  imgui.Text("levelPath")
  imgui.SameLine()
  imgui.Text("LevelName")
  imgui.SameLine()
  if imgui.Button("Save", buttonSize) then
  end
  imgui.SameLine()
  if imgui.Button("Exit", buttonSize) then
    imgui.OpenPopup("Exit Confirmation")
  end
  imgui.EndChild()
end

local function drawMainPanel()
  var.forestEditorWindowSize = imgui.GetWindowSize()
  local cursorPos = imgui.GetCursorPos()
  local tabIconWidth = imgui.GetFontSize() + 6
  local tabIconSize = imgui.ImVec2(tabIconWidth, tabIconWidth)

  local framePadding =  imgui.ImVec2(6, 6) --imgui.GetStyle().FramePadding
  imgui.PushStyleVar2(imgui.StyleVar_FramePadding, framePadding)--imgui.ImVec2(imgui.GetFontSize(), imgui.GetFontSize()))
  if imgui.BeginTabBar("BiomeToolTabBar") then
    if imgui.BeginTabItem("Level Biome##Tab") then
      imgui.PushStyleVar2(imgui.StyleVar_FramePadding, framePadding)
      var.selectedTab = enum_tabType.LevelBiome
      if imgui.BeginChild1("LevelBiomeChild", imgui.ImVec2((imgui.GetContentRegionAvail().x - 6), (imgui.GetContentRegionAvail().y - 6)), true) then
        drawLevelBiomeLayersList()
        drawLevelBiomeLayerProperties()
      end
      imgui.EndChild()
      imgui.PopStyleVar()
      imgui.EndTabItem()
    end
    if imgui.BeginTabItem("Biome Areas##Tab") then
      imgui.PushStyleVar2(imgui.StyleVar_FramePadding, framePadding)
      var.selectedTab = enum_tabType.BiomeAreas
      if imgui.BeginChild1("BiomeAreasChild", imgui.ImVec2((imgui.GetContentRegionAvail().x - 6), (imgui.GetContentRegionAvail().y - 6)), true) then
        drawBiomeAreasLayersList()
        drawBiomeAreasLayerProperties()
      end
      imgui.EndChild()
      imgui.PopStyleVar()
      imgui.EndTabItem()
    end
    imgui.EndTabBar()
  end
  imgui.PopStyleVar()
end

local function drawWindow()
  if editor.beginWindow(toolWindowName, "Biome Tool") then
    drawMainPanel()
  end
  editor.endWindow()
end

local function updateNodePosInArea(layerID, areaID, areaType, nodeIndex, pos)
  for index, layer in ipairs(var.layers.layerInfoTbl) do
    if layerID == layer.layerID then
      for index, lassoArea in ipairs(layer.lassoAreas) do
        if areaID == lassoArea.lassoAreaID and areaType == lassoArea.zoneType then
          for index, node in ipairs(lassoArea.nodes) do
            if nodeIndex == index then
              node.pos = pos
            end
          end
          break
        end
      end
    end
  end
end

local function getLassoAreaGlobalIdx(layerID, zoneType)
  local numAreas = 0
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      for _, lassoArea in ipairs(layer.lassoAreas) do
        if lassoArea.zoneType == zoneType then
          numAreas = numAreas + 1
        end
      end
      break
    end
  end

  local index = 0
  local found = false
  if zoneType == var.enum_lassoDrawType.inclusionZone then
    for _, indexData in ipairs(var.layers.lassoAreaGlobalIndices) do
      if indexData.layerID == layerID then
        index = indexData.index
        found = true
        if numAreas == 0 then
          index = 0
          indexData.index = 0
        end
      end
    end
    if not found then
      table.insert(var.layers.lassoAreaGlobalIndices, {layerID = layerID, index = 0})
    end
  elseif zoneType == var.enum_lassoDrawType.exclusionZone then
    found = false
    for _, indexData in ipairs(var.layers.exZoneGlobalIndices) do
      if indexData.layerID == layerID then
        index = indexData.index
        found = true
        if numAreas == 0 then
          index = 0
          indexData.index = 0
        end
      end
    end
    if not found then
      table.insert(var.layers.exZoneGlobalIndices, {layerID = layerID, index = 0})
    end
  end
  return index
end

local function incLassoAreaGlobalIdx(layerID, zoneType)
  local found = false
  if zoneType == var.enum_lassoDrawType.inclusionZone then
    for _, indexData in ipairs(var.layers.lassoAreaGlobalIndices) do
      if indexData.layerID == layerID then
        indexData.index = indexData.index + 1
        found = true
      end
    end
    if not found then
      table.insert(var.layers.lassoAreaGlobalIndices, {layerID = layerID, index = 1})
    end
  elseif zoneType == var.enum_lassoDrawType.exclusionZone then
    for _, indexData in ipairs(var.layers.exZoneGlobalIndices) do
      if indexData.layerID == layerID then
        indexData.index = indexData.index + 1
        found = true
      end
    end
    if not found then
      table.insert(var.layers.exZoneGlobalIndices, {layerID = layerID, index = 1})
    end
  end
end

local function addLassoArea(layerID, zoneType, nodes)
  for _, layer in ipairs(var.layers.layerInfoTbl) do
    if layer.layerID == layerID and layer.layerType == layerType_enum.area then
      local lassoArea = {lassoAreaID = getLassoAreaGlobalIdx(layerID, zoneType) + 1, zoneType = zoneType, nodes = deepcopy(nodes)}
      table.insert(layer.lassoAreas, lassoArea)
      break
    end
  end
  incLassoAreaGlobalIdx(layerID, zoneType)
end

local function createForestObject()
  local forest = core_forest and core_forest.getForestObject()
  if not forest then
    forest = worldEditorCppApi.createObject("Forest")
    forest:registerObject("")
    forest:setName("theForest")
    scenetree.MissionGroup:addObject(forest)
    createForestBrushGroup()
    editor.setDirty()
  end
end

local function onEditorGui()
  if not editor.editMode or (editor.editMode.displayName ~= editModeName) then
    return
  end

  local forest = core_forest and core_forest.getForestObject()
  if not forest and not createForestPopupShown then
    editor.openModalWindow("NoForestObjDialog")
    createForestPopupShown = true
  end

  if editor.beginModalWindow("NoForestObjDialog", "No Forest Object") then
    imgui.Spacing()
    imgui.Text("There is no Forest object, please create one.")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()
    imgui.Spacing()
    imgui.Spacing()
    if imgui.Button("OK") then
      editor.closeModalWindow("NoForestObjDialog")
    end
  end
  editor.endModalWindow()
  if not forest then
    return
  end

  if not var.askedToOpenProject then
    --imgui.OpenPopup("Ask Project")
  end
  drawWindow()
  if isDrawingLassoArea then
    drawLassoPolylineAction()
  end

  if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Escape)) then
    isDrawingLassoArea = false
    resetDrawActionVariables()
  end

  local hit
  if imgui.GetIO().WantCaptureMouse == false then
    hit = cameraMouseRayCast(false, imgui.flags(SOTTerrain))
  end

  local layer = getSelectedLayer(layerType_enum.area)
  if layer ~= nil and var.selectedTab == enum_tabType.BiomeAreas then
    drawLassoAreas(layer.layerID)
  end

  if not imgui.IsMouseDown(0) then
    local hoveredNodeInfo = getLassoNodeUnderCursor()
    if hoveredNodeInfo then
      var.lassoHoveredNode = {}
      var.lassoActionHoveredNodeIndex = nil
      if hoveredNodeInfo.layerID == -1 then
        var.lassoActionHoveredNodeIndex = hoveredNodeInfo.index
        var.lassoHoveredNode.areaID = -1
        var.lassoHoveredNode.areaType = hoveredNodeInfo.areaType
      else
        var.lassoHoveredNode.index = hoveredNodeInfo.index
        var.lassoHoveredNode.layerID = hoveredNodeInfo.layerID
        var.lassoHoveredNode.areaID = hoveredNodeInfo.areaID
        var.lassoHoveredNode.areaType = hoveredNodeInfo.areaType
      end
    else
      var.lassoHoveredNode.index = nil
      var.lassoHoveredNode.layerID = -2
      var.lassoHoveredNode.areaID = -1
      var.lassoHoveredNode.areaType = nil
    end
  end

  local wasDrawingLassoArea = false
  if imgui.IsMouseClicked(0) and isDrawingLassoArea
      and editor.isViewportHovered()
      and not editor.isAxisGizmoHovered() then
    if var.lassoActionHoveredNodeIndex == 1 and #var.lassoPLNodes > 2 then
      var.lassoSelectionEnded = true
      isDrawingLassoArea = false
      wasDrawingLassoArea = true
      addLassoArea(layer.layerID, var.lassoDrawInfo.type, var.lassoPLNodes)
      resetDrawActionVariables()
    elseif hit then
      local node = {
        nodeID    = #var.lassoPLNodes + 1,
        pos       = hit.pos,
        isUpdated = false
      }
      table.insert(var.lassoPLNodes, node)
    end
  end

  local errCode = var.forestBrushTool:getBiomeError()
  imgui.SetNextWindowSize(imgui.ImVec2(300, 100), imgui.Cond_FirstUseEver)
  if imgui.BeginPopupModal("Biome Lasso Work Progress") then
    if errCode == 0 then
      var.forestBrushTool:runBiomeProcess()
    end
    local progressStr = var.forestBrushTool:getBiomeWorkName()
    local progressPercent = var.forestBrushTool:getBiomeWorkProgress()
    local buttonText = "Cancel"
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      progressStr = "Error: " ..var.forestBrushTool:getBiomeErrorStr()
      imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0, 0, 1))
      buttonText = "Ok"
    end
    imgui.TextUnformatted(progressStr)
    if var.forestBrushTool:isBiomeProcCompleted() and errCode ~= 0 then
      imgui.PopStyleColor()
    end

    if errCode == 0 then
      imgui.ProgressBar(progressPercent, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0), string.format("%d%%", progressPercent * 100))
    end

    if imgui.Button(buttonText) then
      var.forestBrushTool:quitBiomeProcess()
      imgui.CloseCurrentPopup()
    end
    if var.forestBrushTool:isBiomeProcCompleted() then
      if errCode == 0 then
        imgui.CloseCurrentPopup()
      end
    end
    imgui.EndPopup()
  end

  if hit and not isDrawingLassoArea then
    if imgui.IsMouseClicked(0)
        and editor.isViewportHovered()
        and not editor.isAxisGizmoHovered() then
      if var.lassoHoveredNode.index ~= nil then
        var.mouseButtonHeldOnLassoNode = true
        if var.lassoHoveredNode.layerID == -1 then
          var.lassoActionSelectedNodeIndex = var.lassoHoveredNode.index
        else
          var.lassoAreaSelectedNode = {}
          var.lassoAreaSelectedNode.index = var.lassoHoveredNode.index
          var.lassoAreaSelectedNode.layerID = var.lassoHoveredNode.layerID
          var.lassoNodeDragInitialNodes = getLassoNodesWithAreaID(var.lassoHoveredNode.layerID, var.lassoHoveredNode.areaID)
        end
      end
    end
    if var.mouseButtonHeldOnLassoNode and imgui.IsMouseReleased(0) then
      var.mouseButtonHeldOnLassoNode = false
      var.lassoAreaSelectedNode = {}
      if not wasDrawingLassoArea and var.lassoHoveredNode.index ~= nil and var.lassoHoveredNode.areaType == var.enum_lassoDrawType.inclusionZone then
        local layer = getSelectedLayer(layerType_enum.area)
        local isDeletingLayer = false
        local isResizingLasso = true
        biomeProcFunc(isDeletingLayer, layer, var.lassoHoveredNode.areaID, isResizingLasso)
        imgui.OpenPopup("Biome Lasso Work Progress")
      end
    end
    if var.mouseButtonHeldOnLassoNode and imgui.IsMouseDragging(0) then
      updateNodePosInArea(var.lassoHoveredNode.layerID, var.lassoHoveredNode.areaID, var.lassoHoveredNode.areaType, var.lassoHoveredNode.index, hit.pos)
    end
  end
end

local function show()
  editor.clearObjectSelection()
  editor.selectEditMode(editor.editModes.biomeEditMode)
  editor.showWindow(toolWindowName)
end

local function onActivate()
  local forest = core_forest and core_forest.getForestObject()
  if forest then
    editor.clearObjectSelection()
    editor_terrainEditor.updateMaterialLibrary()
    populateMaterialsList()
    editor.showWindow(toolWindowName)
  end
end

local function onDeactivate()
  createForestPopupShown = false
end

local function initialize()
  -- ForestItemData
  local forestItemDataNames = scenetree.findClassObjects("TSForestItemData")
  var.forestItemData = {}
  for k, forestItemDataId in ipairs(forestItemDataNames) do
    local cobj = scenetree.findObject(forestItemDataId)
    if cobj then
      local item = {
        pos = k,
        id = cobj:getId(),
        dirty = false,
        selected = false
      }
      table.insert(var.forestItemData, item)
    end
  end
  var.forestBrushTool = ForestBrushTool()
  local forest = core_forest.getForestObject()
  if forest then
    var.forestData = forest:getData()
    var.forestBrushTool:setActiveForest(forest)
  else
    log('I', '', "There's no Forest object.")
  end
end

local function getLayerTerrainMaterial(areaID, layerID)
  local materialName = nil
  local fieldsData = getFieldsData(areaID, layerID)
  for _, fieldData in ipairs(fieldsData) do
    if fieldData.name == "TerrainMaterial" then
      materialName = fieldData.val
    end
  end
  return materialName
end

local editingPos = false
local range = imgui.ArrayFloat(2)
local fieldPlacementBoolPtr = imgui.BoolPtr(0)
local function biomeToolCustomFieldEditor(objectIds, fieldValue, fieldName, fieldLabel, fieldDesc, fieldType, fieldTypeName, customData, pasteCallback, contextMenuUI)
  local fieldVal = fieldValue
  if fieldName == "TerrainMaterial" then
    fieldVal = getLayerTerrainMaterial(customData.layerType, customData.layerID)
    imgui.BeginDisabled()
    editor.uiInputText('', editor.getTempCharPtr(fieldVal))
    imgui.EndDisabled()
  elseif fieldName == "BlendingMethod" then
    local blendingMethodPtr = imgui.IntPtr(0)
    blendingMethodPtr[0] = getBlendingMethod(customData.layerType, customData.layerID)
    if imgui.Combo1("##layersDelete", blendingMethodPtr, layerBlendingComboItems) then
      setBlendingMethod(customData.layerType, customData.layerID, blendingMethodPtr[0])
    end
  elseif fieldName == "FieldPlacement" then
    fieldPlacementBoolPtr[0] = getFieldPlacement(customData.layerType, customData.layerID)
    if imgui.Checkbox("##fieldPlacementEnabled", fieldPlacementBoolPtr) then
      setFieldPlacement(customData.layerType, customData.layerID, fieldPlacementBoolPtr[0])
    end
  end

  if fieldName == "SlopeRange" then
    local shouldDisableRange = (getSlopeInfluence(customData.areaID, customData.layerID) == 0.0)
    local slopeRange = getSlopeRange(customData.areaID, customData.layerID)
    if shouldDisableRange then
      imgui.BeginDisabled()
    end
    if not editingPos then
      range = imgui.TableToArrayFloat(slopeRange)
    end
    local positionSliderEditEnded = imgui.BoolPtr(false)
    if editor.uiDragFloat2("##" .."SlopeRange"..tostring(customData.areaID)..tostring(customData.layerID),
      range, 0.2, 0, 90, "%0." .. editor.getPreference("ui.general.floatDigitCount") .. "f", 1, positionSliderEditEnded) then
        editingPos = true
    end
    if positionSliderEditEnded[0] == true then
      setSlopeRange(customData.areaID, customData.layerID, {range[0], range[1]})
      editingPos = false
    end
    if shouldDisableRange then
      imgui.EndDisabled()
    end
  end
end

local function getLevelPathAndName()
  local path = '/levels/'
  local name = ""
  local i = 1
  for str in string.gmatch(getMissionFilename(),"([^/]+)") do
    if i == 2 then
      path = path .. str
      name = str
    end
    i = i + 1
  end
  return path, name
end

local function onEditorInitialized()
  local levelPath, levelName = getLevelPathAndName()
  local levelDataPath = string.format("%s%s", levelPath, "/art/biomeTool/biomeTool.json")

  if FS:fileExists(levelDataPath) then
    var.layers = jsonReadFile(levelDataPath)
    --initEmptyFieldInfo()
  end
  populateForestBrushes()

  editor.editModes.biomeEditMode =
  {
    displayName = editModeName,
    onUpdate = nop,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    icon = editor.icons.biome_tool,
    iconTooltip = "Biome Tool",
    sortOrder = 6,
    auxShortcuts = {},
  }
  editor.registerCustomFieldInspectorEditor("BiomeTool", "TerrainMaterial", biomeToolCustomFieldEditor)
  editor.registerCustomFieldInspectorEditor("BiomeTool", "BlendingMethod", biomeToolCustomFieldEditor)
  editor.registerCustomFieldInspectorEditor("BiomeTool", "SlopeRange", biomeToolCustomFieldEditor)
  editor.registerCustomFieldInspectorEditor("BiomeTool", "VegetationFalloff", biomeToolCustomFieldEditor)
  editor.registerCustomFieldInspectorEditor("BiomeTool", "FieldPlacement", biomeToolCustomFieldEditor)
  editor.registerCustomFieldInspectorEditor("BiomeTool", "RA_Map", biomeToolCustomFieldEditor)
  editor.registerWindow(toolWindowName, imgui.ImVec2(400, 400))
  editor.registerModalWindow("NoForestObjDialog")

  valueInspector.selectionClassName = "BiomeTool"
  valueInspector.setValueCallback = function(fieldName, fieldValue, arrayIndex, customData, editEnded)
    if customData then
      setFieldValue(fieldName, fieldValue, customData)
    end
  end

  forest = core_forest and core_forest.getForestObject()
  if forest then
    var.forestData = forest:getData()
  end

  terrainBlock = getObjectByClass("TerrainBlock")
  populateMaterialsList()
  initialize()
end

local function onEditorToolWindowHide(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.objectSelect)
  end
end

local function onWindowGotFocus(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.biomeEditMode)
  end
end

local function onEditorAfterSaveLevel()
  if tableIsEmpty(var.layers) then return end
  local levelPath, levelName = getLevelPathAndName()
  local biomeDataPath = string.format("%s/%s", levelPath, "/art/biomeTool/biomeTool.json")
  local areaLayers = getLayersWithType(layerType_enum.area)
  local terrLayers = getLayersWithType(layerType_enum.terrain)
  local layerAvailable = not tableIsEmpty(areaLayers) or not tableIsEmpty(terrLayers)
  if FS:fileExists(biomeDataPath) then
    FS:removeFile(biomeDataPath)
  end

  if layerAvailable then
    jsonWriteFile(biomeDataPath, var.layers, true)
  end
  local forest = core_forest.getForestObject()
  if forest then
    forest:saveForest()
  end
end

M.onEditorAfterSaveLevel = onEditorAfterSaveLevel
M.onEditorGui = onEditorGui
M.onEditorToolWindowHide = onEditorToolWindowHide
M.onEditorToolWindowGotFocus = onWindowGotFocus

M.onEditorInitialized = onEditorInitialized
M.onExtensionLoaded = onExtensionLoaded

return M