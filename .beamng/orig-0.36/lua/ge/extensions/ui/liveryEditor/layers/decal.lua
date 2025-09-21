-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiUtils = extensions.ui_liveryEditor_utils

local ACTIONS = {{
  label = "Reproject",
  value = "requestReproject",
  lazyLoadItems = true,
  allowOpenDrawer = false
}, {
  label = "Transform",
  value = "transform"
}, {
  label = "Materials",
  value = "materials"
}, {
  label = "Mirror",
  value = "requestMirror",
  lazyLoadItems = true,
  allowOpenDrawer = false
}, {
  label = "Order",
  value = "order"
}, {
  label = "Highlight",
  value = "highlight",
  isSwitch = true
}, {
  label = "Enable",
  value = "enabled"
}, {
  label = "Duplicate",
  value = "duplicate"
}, {
  label = "Delete",
  value = "delete"
}}

M.layerUid = nil

M.setLayer = function(layerUid)
  M.layerUid = layerUid
  -- notify ui and extensions
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

local resetCursor = function()
  resetCursorPosition()
  api.setDecalColor(Point4F(1, 1, 1, 1))
  api.setDecalScale(vec3(0.5, 1, 0.5))
  api.setDecalRotation(0)
  api.setMetallicIntensity(0.5)
  api.setRoughnessIntensity(0.5)
  api.setMirrored(false)
  api.setFlipMirroredDecal(false)
  api.setMirrorOffset(0)
end

M.addLayer = function(params)
  resetCursor()

  local color = params and params.color and Point4F.fromTable(params.color) or Point4F(1, 1, 1, 1)
  api.setDecalColor(color)

  if params.texturePath then
    api.setDecalTexturePath("color", params.texturePath)
  end
  return api.addDecal()
end

M.addLayerCentered = function(params)
  local isUseMouse = api.isUseMousePos()
  if isUseMouse then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
  -- resetCursorPosition()
  local layer = M.addLayer(params)
  resetCursor()
  showCursor(false)
  return layer
end

M.updateLayer = function(params)
end

M.getLayerActions = function(layerUid)
  layerUid = layerUid and layerUid or M.layerUid
  local layer = layerUid and api.getLayerByUid(layerUid)

  if layer then
    local data = {}

    for _, action in ipairs(ACTIONS) do
      if action.value == "enabled" then
        local actionCopy = deepcopy(action)
        actionCopy.data = {
          value = layer.enabled,
          label = layer.enabled and "Enabled" or "Disabled"
        }
        table.insert(data, actionCopy)
      else
        table.insert(data, action)
      end
    end

    return data
  else
    log("W", "", "Layer " .. (layerUid and layerUid or "nil") .. " not found. Unable to retrieve layer actions")
  end
end

return M
