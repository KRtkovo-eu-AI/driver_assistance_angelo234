-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local utils = extensions.ui_liveryEditor_utils
local uiCameraApi = extensions.ui_liveryEditor_camera
local uiFillLayer = extensions.ui_liveryEditor_layers_fill

-- Layer Cached Data
-- Id
-- Path - path from root
local layerMap = {}
local layers = {}

local transformLayerUiFormat = function(layer, coordinates)
  local formattedLayer = {
    uid = layer.uid,
    id = layer.uid,
    name = layer.name,
    type = layer.type,
    enabled = layer.enabled,
    locked = layer.locked
  }

  if layer.type == api.layerTypes.decal then
    formattedLayer.preview = layer.decalColorTexturePath
    formattedLayer.rotation = utils.convertRadiansToDegrees(layer.decalRotation)
    formattedLayer.mirrored = layer.mirrored
    formattedLayer.mirrorFlipped = layer.flipMirroredDecal
    formattedLayer.scale = {
      x = layer.decalScale.x,
      y = layer.decalScale.z
    }
    formattedLayer.skew = {
      x = utils.roundAndTruncateDecimal(layer.decalSkew.x),
      y = utils.roundAndTruncateDecimal(layer.decalSkew.y)
    }
    -- formattedLayer.position = utils.getXYCoordinates(layer.decalPos, uiCameraApi.getCoordinates())
    formattedLayer.color = layer.color:toTable()
    formattedLayer.metallicIntensity = layer.metallicIntensity
    formattedLayer.normalIntensity = layer.normalIntensity
    formattedLayer.roughnessIntensity = layer.roughnessIntensity
  elseif layer.type == api.layerTypes.fill then
    formattedLayer.colorPaletteMapId = layer.colorPaletteMapId
    formattedLayer.color = layer.color:toTable()
  elseif layer.type == api.layerTypes.linkedSet then
    for k, property in ipairs(layer.properties) do
      if property.id == "color" then
        formattedLayer.color = property.value:toTable()
      end
    end
  end

  return formattedLayer
end

M.parseLayersData = function(layersData, parentLayer)
  dump("parseLayerData", parentLayer)
  local uiLayers = {}

  for key in ipairs(layersData) do
    local layer = layersData[key]
    local uiLayer = transformLayerUiFormat(layer, uiCameraApi.getCoordinates())

    uiLayer.order = key
    uiLayer.parentUid = parentLayer and parentLayer.uid
    uiLayer.childrenCount = layer.children and #layer.children or 0

    -- set layer hidden for now if type is not decal or group
    local hidden = layer.type ~= api.layerTypes.decal and layer.type ~= api.layerTypes.linkedSet
    uiLayer.hidden = hidden

    if parentLayer then
      uiLayer.path = shallowcopy(parentLayer.path)
      table.insert(uiLayer.path, layer.uid)

      uiLayer.pathIndices = shallowcopy(parentLayer.pathIndices)
      table.insert(uiLayer.pathIndices, key)
    else
      uiLayer.path = {layer.uid}
      uiLayer.pathIndices = {key}
    end

    uiLayer.siblingCount = #layersData

    if layer.children and #layer.children > 0 then
      uiLayer.children = M.parseLayersData(layer.children, uiLayer)
    end

    table.insert(uiLayers, 1, uiLayer)

    M.layerMap[uiLayer.uid] = {
      order = key,
      parentUid = parentLayer and parentLayer.uid,
      layer = shallowcopy(uiLayer),
      hidden = hidden
    }
  end

  return uiLayers
end

M.getVisibleLayersCount = function()
  local count = 0
  if M.layerMap then
    for k, layer in pairs(M.layerMap) do
      if not layer.hidden then
        count = count + 1
      end
    end
  end

  return count
end

M.rebuildLayerData = function()
  M.layerMap = {}
  M.layers = M.parseLayersData(api.getLayerStack())
  guihooks.trigger("liveryEditor_OnLayersUpdated", M.layers)
  guihooks.trigger("liveryEditor_Layers_OnVisibleCountChanged", M.getVisibleLayersCount())
end

M.getLayers = function()
  return layers
end

M.getLayerByUid = function(layerUid)
  return M.layerMap[layerUid].layer
end

M.getLayerByOrder = function(order, parentUid)
  for k, layerMapItem in pairs(M.layerMap) do
    if (not parentUid or parentUid == layerMapItem.parentUid) and layerMapItem.order == order then
      return layerMapItem.layer
    end
  end
end

M.getChildrenCount = function(layerUid)
  if not layerUid then
    return #M.layers
  end

  local count = 0
  for k, layer in ipairs(M.layerMap) do
    if layer.parentUid == layerUid then
      count = count + 1
    end
  end

  return count
end

M.requestInitialData = function()
  M.rebuildLayerData()
end

M.layers = layers
M.layerMap = layerMap
-- M.notifyUiListeners

-- External hooks. Do not call!
M.dynamicDecals_onLayerAdded = function(layerUid)
  dump("dynamicDecals_onLayerAdded", layerUid)
  M.rebuildLayerData()
  extensions.hook("liveryEditor_OnLayerAdded", M.getLayerByUid(layerUid))
end

M.dynamicDecals_onLayerDeleted = function(layerUid)
  dump("dynamicDecals_onLayerDeleted", layerUid)
  M.rebuildLayerData()
  extensions.hook("liveryEditor_OnLayerDeleted", layerUid)
end

M.dynamicDecals_onLayerUpdated = function(layerUid)
  dump("dynamicDecals_onLayerUpdated", layerUid)
  M.rebuildLayerData()
  extensions.hook("liveryEditor_onLayersUpdated", layerUid)
end

M.dynamicDecals_moveLayer = function(from, fromParentUid, to, toParentUid)
  dump("dynamicDecals_moveLayer",
      "from: " .. from .. ", fromParent: " .. (fromParentUid or "nil") .. ", to: " .. to .. ", toParent: " ..
          (toParentUid or "nil"))
  M.rebuildLayerData()
  local layer = M.getLayerByOrder(to, toParentUid)
  extensions.hook("liveryEditor_onLayersUpdated", layer.uid)
end

return M
