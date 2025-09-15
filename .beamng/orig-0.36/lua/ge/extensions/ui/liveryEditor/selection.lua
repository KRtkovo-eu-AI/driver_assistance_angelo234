-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local API = extensions.editor_api_dynamicDecals
local utils = extensions.ui_liveryEditor_utils
local uiCameraApi = extensions.ui_liveryEditor_camera
local uiLayersApi = extensions.ui_liveryEditor_layers
local uiCursor = extensions.ui_liveryEditor_layers_cursor
local uiDecals = extensions.ui_liveryEditor_layers_decals
local uiDecal = extensions.ui_liveryEditor_layers_decal

local MULTI_TYPES_LAYER_ACTIONS = {"visibility"}
local MULTI_DECAL_LAYER_ACTIONS = {"group", "material", "duplicate", "mirror", "visibility", "delete"}
local MULTI_FILL_LAYER_ACTIONS = {"duplicate", "visibility", "delete"}

local selectedLayers = nil

local getUiFormattedSelectedLayers = function()
  if not M.selectedLayers then
    return nil
  end

  local highlightedLayer = API.getHighlightedLayer()
  local uiFormattedLayers = {}
  for key, layerUid in ipairs(M.selectedLayers) do
    local layer = uiLayersApi.getLayerByUid(layerUid)
    layer.highlighted = highlightedLayer and highlightedLayer.uid == layerUid
    table.insert(uiFormattedLayers, layer)
  end
  return uiFormattedLayers
end

local addToSelection = function(layerUid)
  if not M.selectedLayers then
    M.selectedLayers = {}
  end
  table.insert(M.selectedLayers, layerUid)
end

local removeFromSelection = function(layerUid)
  if M.selectedLayers then
    for i, selectedLayerUid in ipairs(M.selectedLayers) do
      if selectedLayerUid == layerUid then
        table.remove(M.selectedLayers, i)
        return true
      end
    end
  end

  return false
end

local getFirstSelectedLayer = function()
  if M.selectedLayers and #M.selectedLayers > 0 then
    for _, value in ipairs(M.selectedLayers) do
      return value
    end
  end
end

local getAvailableActions = function()
  if not M.selectedLayers then
    return nil
  end

  if #M.selectedLayers == 1 then
    local selectedLayer = uiLayersApi.getLayerByUid(M.getFirstSelectedLayer())
    local selectedLayerType = selectedLayer.type

    if selectedLayerType == API.layerTypes.decal then
      return extensions.ui_liveryEditor_layers_decal.getLayerActions(selectedLayer.uid)
    elseif selectedLayerType == API.layerTypes.fill then
      return extensions.ui_liveryEditor_layers_fill.getLayerActions(selectedLayer)
    elseif selectedLayerType == API.layerTypes.linkedSet then
      return extensions.ui_liveryEditor_layers_group.getLayerActions()
    else
      log("D", "", "Layer actions not implemented for " .. selectedLayerType)
    end
  else
    -- do filtering here
    local layerTypes = {}

    for _, v in ipairs(M.selectedLayers) do
      local layer = uiLayersApi.getLayerByUid(v)

      if not layerTypes[layer.type] then
        layerTypes[layer.type] = true
      end
    end

    local layerType
    local layerTypeCount = 0
    for k, _ in pairs(layerTypes) do
      layerType = k
      layerTypeCount = layerTypeCount + 1
    end

    if layerTypeCount > 1 then
      return MULTI_TYPES_LAYER_ACTIONS
    elseif layerType == API.layerTypes.decal then
      return MULTI_DECAL_LAYER_ACTIONS
    elseif layerType == API.layerTypes.fill then
      return MULTI_FILL_LAYER_ACTIONS
    end
  end
end

local notifyUiListeners = function()
  guihooks.trigger("liveryEditor_selection_actionsUpdated", M.getAvailableActions())
  guihooks.trigger("liveryEditor_selection_selectedChanged", M.getSelectedLayersData())

  -- cleanup and delete this events to have properly formatted names
  guihooks.trigger("LiveryEditor_SelectedLayersChanged", M.selectedLayers)
  guihooks.trigger("LiverEditorLayerActionsUpdated", M.getAvailableActions())
  guihooks.trigger("LiveryEditor_SelectedLayersDataUpdated", M.getSelectedLayersData())

  local first = M.getFirstSelectedLayer()
  if first then
    local layer = API.getLayerByUid(first)
    uiDecals.notifyListeners(layer)
  end
end

local clearSelection = function()
  M.selectedLayers = nil
  API.disableDecalHighlighting()
  M.notifyUiListeners()
end

local getSelectedLayers = function()
  return M.selectedLayers
end

local isLayerSelected = function(layerUid)
  if M.selectedLayers then
    for _, value in ipairs(M.selectedLayers) do
      if layerUid == value then
        return true
      end
    end
  end

  return false
end

M.toggleHighlightSelectedLayer = function()
  API.setEnabled(true)
  local layerId = M.getFirstSelectedLayer()
  local highlighted = API.getHighlightedLayer()

  if highlighted and highlighted.uid == layerId then
    API.disableDecalHighlighting()
  else
    API.highlightLayerByUid(layerId)
  end

  M.notifyUiListeners()
end

local select = function(layerIds, highlight)
  if type(layerIds) == "table" then
    M.selectedLayers = layerIds
  else
    M.selectedLayers = {layerIds}
    if highlight then
      API.highlightLayerByUid(layerIds)
    end
  end

  M.notifyUiListeners()
end

local setSelected = function(layerUid)
  selectedLayers = {layerUid}
  local layer = API.getLayerByUid(layerUid)
  M.notifyUiListeners()
end

local setMultipleSelected = function(layerUids)
  selectedLayers = layerUids

  M.notifyUiListeners()
end

M.toggleSelection = function(layerUid)
  if M.isLayerSelected(layerUid) then
    M.removeFromSelection(layerUid)
  else
    M.addToSelection(layerUid)
  end
  M.notifyUiListeners()
end

M.duplicateSelectedLayer = function()
  M.duplicateLayerActive = true
  local uid = M.getFirstSelectedLayer()
  local layer = uiLayersApi.getLayerByUid(uid)
  API.duplicateLayer(layer.order, layer.parentUid)
end

M.reapplySelectedLayer = function()
  M.reapplyLayerActive = true
  local uid = M.getFirstSelectedLayer()
  local layer = API.getLayerById(uid)
  layer.enabled = false
  API.setLayer(layer, true)

  API.setEnabled(true)
  uiCursor.setCursorProperties(layer)
  uiCursor.notifyListeners()
end

M.cancelReapplySelectedLayer = function()
  local uid = M.getFirstSelectedLayer()
  local layer = API.getLayerById(uid)
  layer.enabled = true
  API.setLayer(layer, true)

  API.setEnabled(false)
  uiDecals.notifyListeners(layer)

  M.reapplyLayerActive = false
end

M.setup = function()
  M.selectedLayers = nil
  notifyUiListeners()
end

M.duplicateLayerActive = false
M.reapplyLayerActive = false
M.select = select
M.selectedLayers = selectedLayers
M.clearSelection = clearSelection
M.getSelectedLayers = getSelectedLayers
M.getFirstSelectedLayer = getFirstSelectedLayer
M.addToSelection = addToSelection
M.removeFromSelection = removeFromSelection
M.getSelectedLayersData = getUiFormattedSelectedLayers
M.isLayerSelected = isLayerSelected
M.getAvailableActions = getAvailableActions
M.notifyUiListeners = notifyUiListeners
M.requestInitialData = notifyUiListeners

-- External hooks. Do not call!
M.liveryEditor_OnLayerAdded = function(layer)
  if M.duplicateLayerActive then
    M.select(layer.uid, false)
    M.duplicateLayerActive = false
  end
end

M.dynamicDecals_onLayerDeleted = function(layerUid)
  local success = M.removeFromSelection(layerUid)
  if success then
    if #M.selectedLayers then
      M.selectedLayers = nil
    end
    M.notifyUiListeners()
  end
end

M.liveryEditor_onLayersUpdated = function(layerUid)
  if M.isLayerSelected(layerUid) then
    guihooks.trigger("LiveryEditor_SelectedLayersDataUpdated", M.getSelectedLayersData())
    -- M.notifyUiListeners()
  end
end

return M
