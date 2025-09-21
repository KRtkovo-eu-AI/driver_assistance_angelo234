-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiSelectionApi = extensions.ui_liveryEditor_selection
local uiDecalsApi = extensions.ui_liveryEditor_layers_decals
local uiLayersApi = extensions.ui_liveryEditor_layers
local uiCursorApi = extensions.ui_liveryEditor_layers_cursor
local uiTools = extensions.ui_liveryEditor_tools
local uiControlsApi = extensions.ui_liveryEditor_controls
local uiUtils = extensions.ui_liveryEditor_utils

-- Stamp assumes there is only one layer selected
M.useStamp = function()
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  local layer = api.getLayerByUid(layerUid)

  -- match decal project properties to selected layer
  api.setDecalRotation(layer.decalRotation)
  api.setDecalScale(layer.decalScale)
  api.setDecalSkew(layer.decalSkew)
  api.setDecalColor(layer.color)

  -- hide selected layer
  layer.color = Point4F(layer.color.x, layer.color.y, layer.color.z, 0)
  api.setLayer(layer, true)

  uiDecalsApi.showCursor(true)
  uiControlsApi.useMouseProjection()

  guihooks.trigger("LiveryEditor_ToolDataUpdated", {
    mode = "stamp"
  })
end

M.cancelStamp = function()
  dump("cancelStamp")
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  local layer = api.getLayerByUid(layerUid)

  -- show selected layer
  layer.color = Point4F(layer.color.x, layer.color.y, layer.color.z, 1)
  api.setLayer(layer, true)

  uiDecalsApi.showCursor(false)
  uiControlsApi.useCursorProjection()
  guihooks.trigger("LiveryEditor_ToolDataUpdated", {
    mode = "default"
  })
end

M.translate = function(steps_x, steps_y)
  uiTools.doOperation(function(layer, steps_x, steps_y)
    if not layer then
      uiCursorApi.translate(steps_x, steps_y)
    end
  end, steps_x, steps_y)
end

M.setPosition = function(posX, posY)
  uiTools.doOperation(function(layer, posX, posY)
    if not layer then
      uiCursorApi.setPosition(posX, posY)
    end
  end, posX, posY)
end

-- Increment/decrement the rotation by a number of degrees
M.rotate = function(degrees, counterClockwise)
  uiTools.doOperation(function(layer, degrees, counterClockwise)
    if not layer then
      uiCursorApi.rotate(degrees, counterClockwise)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.rotate(layer, degrees, counterClockwise)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, degrees, counterClockwise)
end

-- Set degree value as the rotation
M.setRotation = function(degrees)
  uiTools.doOperation(function(layer, degrees)
    if not layer then
      uiCursorApi.setRotation(degrees)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.setRotation(layer, degrees)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, degrees)
end

local MEASUREMENTS = {
  TRANSLATE_STEP_UNIT = 0.1,
  ROTATE_STEP_UNIT = 1,
  SCALE_STEP_UNIT = 0.1,
  SKEW_STEP_UNIT = 0.1
}

local scaleX = function(scaleX, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  return scaleX + diff
end

local scaleY = function(scaleY, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  return scaleY + diff
end

M.scale = function(steps_x, steps_y)
  uiTools.doOperation(function(layer, steps_x, steps_y)
    if not layer then
      uiCursorApi.scale(steps_x, steps_y)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.scale(layer, steps_x, steps_y)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, steps_x, steps_y)
end

M.setScale = function(scaleX, scaleY)
  dump("tool_transform_setScale", scaleX, scaleY)
  uiTools.doOperation(function(layer, scaleX, scaleY)
    if not layer then
      uiCursorApi.setScale(scaleX, scaleY)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.setScale(layer, scaleX, scaleY)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, scaleX, scaleY)
end

M.skew = function(steps_x, steps_y)
  uiTools.doOperation(function(layer, steps_x, steps_y)
    if not layer then
      uiCursorApi.skew(steps_x, steps_y)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.skew(layer, steps_x, steps_y)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, steps_x, steps_y)
end

M.setSkew = function(skewX, skewY)
  uiTools.doOperation(function(layer, skewX, skewY)
    if not layer then
      uiCursorApi.setSkew(skewX, skewY)
    elseif layer.type == api.layerTypes.decal then
      uiDecalsApi.setSkew(layer, skewX, skewY)
    elseif layer.type == api.layerTypes.group then
      log("D", "", "Translate Group not implemented yet")
    else
      log("W", "", "Cannot move unsupported layer " .. layer.uid .. " of type " .. (layer.type or "nil"))
    end
  end, skewX, skewY)
end

return M
