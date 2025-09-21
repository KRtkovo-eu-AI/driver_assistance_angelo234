-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiTools = extensions.ui_liveryEditor_tools
local uiLayers = extensions.ui_liveryEditor_layers
local uiCursorApi = extensions.ui_liveryEditor_layers_cursor
local uiDecals = extensions.ui_liveryEditor_layers_decals

M.setVisibility = function(show)
  uiTools.doOperation(function(layer, show)
    layer.enabled = show
    api.setLayer(layer, true)
  end, show)
end

M.toggleVisibility = function()
  uiTools.doOperation(function(layer)
    local apiEnabled = api.getEnabled()
    if not apiEnabled then
      api.setEnabled(true)
    end
    layer.enabled = not layer.enabled
    api.setLayer(layer, true)
    if apiEnabled ~= api.getEnabled() then
      api.setEnabled(apiEnabled)
    end
  end)
end

M.toggleLock = function()
  uiTools.doOperation(function(layer)
    layer.locked = not layer.locked
    api.setLayer(layer, false)
  end)
end

M.toggleVisibilityById = function(layerUid)
  local layer = api.getLayerByUid(layerUid)
  layer.enabled = not layer.enabled
  api.setLayer(layer, true)
end

M.toggleLockById = function(layerUid)
  local layer = api.getLayerByUid(layerUid)
  layer.locked = not layer.locked
  api.setLayer(layer, false)
end

M.rename = function(name)
  uiTools.doOperation(function(layer)
    layer.name = name
    api.setLayer(layer, false)
  end)
end

M.deleteLayer = function()
  uiTools.doOperation(function(layer)
    local uiLayer = uiLayers.getLayerByUid(layer.uid)
    api.removeLayer(uiLayer.order, uiLayer.parentUid)
  end)
end

M.setMirrored = function(mirror, flip)
  uiTools.doOperation(function(layer)
    if not layer then
      uiCursorApi.setMirrored(mirror, flip)
    elseif layer.type == api.layerTypes.decal then
      uiDecals.setMirrored(layer, mirror, flip)
    end
  end)
end

M.setMirrorOffset = function(offset)
  uiTools.doOperation(function(layer)
    if not layer then
      uiCursorApi.setMirrorOffset(offset)
    end
  end)
end

M.setUseMousePos = function(value)
  if api.isUseMousePos() ~= value then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
    guihooks.trigger("liveryEditor_OnSettingsChanged_UseMousePos", value)
    extensions.hook("liveryEditor_OnUseMousePosChanged", value)
  end
end

M.setProjectSurfaceNormal = function(value)
  uiTools.doOperation(function(layer)
    if not layer then
      uiCursorApi.setProjectSurfaceNormal(value)
    end
  end)
end

return M
