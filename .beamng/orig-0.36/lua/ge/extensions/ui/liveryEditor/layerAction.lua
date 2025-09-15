-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiSelection = extensions.ui_liveryEditor_selection
local uiLayerEdit = extensions.ui_liveryEditor_layerEdit
local uiLayers = extensions.ui_liveryEditor_layers

local actionTypes = {
  requestReproject = "requestReproject",
  cancelReproject = "cancelReproject",
  reproject = "reproject",
  transform = "transform",
  materials = "materials",
  highlight = "highlight",
  enabled = "enabled",
  delete = "delete",
  duplicate = "duplicate",
  mirror = "mirror",
  flipMirrored = "flipMirrored"
}

local useMouse = function()
  local isUseMouse = api.isUseMousePos()
  if not isUseMouse then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
end

local useCursor = function()
  local isUseMouse = api.isUseMousePos()
  if isUseMouse then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
end

local showCursor = function(show)
  local r, g, b = unpack(api.getDecalColor():toTable())
  api.setDecalColor(Point4F.fromTable({r, g, b, show and 1 or 0}))
end

local resetCursorPosition = function()
  local cursorPosition = api.getCursorPosition()
  cursorPosition.x = 0.5
  cursorPosition.y = 0.5
  api.setCursorPosition(cursorPosition)
end

local showLayer = function(layer, show)
  -- hide layer by setting alpha to 0. Using enabled property will update UI styling and component rerenders
  local r, g, b = unpack(layer.color:toTable())
  layer.color = Point4F.fromTable({r, g, b, show and 1 or 0})
  api.setLayer(layer, true)
end

-- show cursor and set its properties to the specified layer
local performRequestReproject = function(layer)
  -- set cursor properties
  api.setDecalTexturePath("color", layer.decalColorTexturePath)
  api.setDecalScale(layer.decalScale)
  api.setDecalSkew(layer.decalSkew)
  api.setDecalRotation(layer.decalRotation)
  api.setMetallicIntensity(layer.metallicIntensity)
  api.setRoughnessIntensity(layer.metallicIntensity)
  api.setDecalColor(layer.color)
  api.setMirrored(layer.mirrored)
  api.setFlipMirroredDecal(layer.flipMirroredDecal)
  api.setMirrorOffset(layer.mirrorOffset and layer.mirrorOffset or 0)

  api.disableDecalHighlighting()

  useCursor()
  resetCursorPosition()

  showCursor(true)
  showLayer(layer, false)
end

local performCancelReproject = function(layer)
  showCursor(false)
  showLayer(layer, true)
  api.highlightLayerByUid(layer.uid)
end

local performReproject = function(layer)
  dump("PERFORM REPROJECT", layer)
  local lastIndex = uiLayers.getChildrenCount()
  local referenceDecal = api.addDecal()
  local newDecal = deepcopy(referenceDecal)
  newDecal.uid = layer.uid
  newDecal.name = layer.name
  api.setLayer(newDecal, true)
  api.removeLayer(lastIndex + 1)
  showCursor(false)
end

-- transform currently only supports single decal layer
local performTransform = function(layer)
  -- diable decal highlighting
  local highlightedLayer = api.getHighlightedLayer()
  if highlightedLayer and layer.uid == highlightedLayer.uid then
    api.disableDecalHighlighting()
  end

  uiLayerEdit.editExistingLayer(layer.uid, true)
  guihooks.trigger("liveryEditor_changeView", "LayerTransform")
end

local performMaterials = function(layer)
  uiLayerEdit.editExistingLayer(layer.uid, false)
  api.disableDecalHighlighting()
  guihooks.trigger("liveryEditor_changeView", "LayerMaterials")
end

local performEnabled = function(layerUids)
  for _, uid in ipairs(layerUids) do
    local layer = api.getLayerByUid(uid)
    layer.enabled = not layer.enabled
    api.setLayer(layer, true)
    if layer.enabled then
      api.highlightLayerByUid(uid)
      return true
    else
      api.disableDecalHighlighting()
      return false
    end
  end
end

local performDelete = function(layerUid)
  local layer = uiLayers.getLayerByUid(layerUid)
  api.removeLayer(layer.order, layer.parentUid)
end

local performDuplicate = function(layerUid)
  local layer = uiLayers.getLayerByUid(layerUid)
  api.duplicateLayer(layer.order, layer.parentUid)
end

local performToggleHighlight = function(layerUid)
  local highlightedLayer = api.getHighlightedLayer()
  if highlightedLayer and highlightedLayer.uid == layerUid then
    api.disableDecalHighlighting()
    return false
  else
    api.highlightLayerByUid(layerUid)
    return true
  end
end

local performToggleMirrored = function(layer)
  layer.mirrored = not layer.mirrored
  api.setLayer(layer, true)
  return layer.mirrored
end

local performToggleFlipMirrored = function(layer)
  layer.flipMirroredDecal = not layer.flipMirroredDecal
  api.setLayer(layer, true)
  return layer.flipMirroredDecal
end

M.toggleEnabledByLayerUid = function(layerUid)
  local layer = api.getLayerByUid(layerUid)
  layer.enabled = not layer.enabled
  api.setLayer(layer, true)

  if not layer.enabled then
    local highlighted = api.getHighlightedLayer()
    if highlighted and highlighted.uid == layerUid then
      api.disableDecalHighlighting()
    end
  end
end

M.performAction = function(action)
  if action == actionTypes.requestReproject then
    local firstUid = uiSelection.getFirstSelectedLayer()
    local first = firstUid and api.getLayerByUid(firstUid)
    performRequestReproject(first)
  elseif action == actionTypes.cancelReproject then
    local firstUid = uiSelection.getFirstSelectedLayer()
    local first = firstUid and api.getLayerByUid(firstUid)
    performCancelReproject(first)
  elseif action == actionTypes.reproject then
    local firstUid = uiSelection.getFirstSelectedLayer()
    local first = firstUid and api.getLayerByUid(firstUid)
    performReproject(first)
  elseif action == actionTypes.transform then
    local firstUid = uiSelection.getFirstSelectedLayer()
    local first = firstUid and api.getLayerByUid(firstUid)
    performTransform(first)
  elseif action == actionTypes.materials then
    local layerUid = uiSelection.getFirstSelectedLayer()
    local layer = layerUid and api.getLayerByUid(layerUid)
    performMaterials(layer)
  elseif action == actionTypes.enabled then
    return performEnabled(uiSelection.getSelectedLayers())
  elseif action == actionTypes.delete then
    local layerUid = uiSelection.getFirstSelectedLayer()
    performDelete(layerUid)
  elseif action == actionTypes.duplicate then
    local layerUid = uiSelection.getFirstSelectedLayer()
    performDuplicate(layerUid)
  elseif action == actionTypes.highlight then
    local layerUid = uiSelection.getFirstSelectedLayer()
    return performToggleHighlight(layerUid)
  elseif action == actionTypes.mirror then
    local layerUid = uiSelection.getFirstSelectedLayer()
    local layer = layerUid and api.getLayerByUid(layerUid)
    return performToggleMirrored(layer)
  elseif action == actionTypes.flipMirrored then
    local layerUid = uiSelection.getFirstSelectedLayer()
    local layer = layerUid and api.getLayerByUid(layerUid)
    return performToggleFlipMirrored(layer)
  end
end

return M
