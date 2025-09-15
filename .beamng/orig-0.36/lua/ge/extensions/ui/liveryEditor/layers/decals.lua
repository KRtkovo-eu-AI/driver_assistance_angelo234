-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local camera = extensions.ui_liveryEditor_camera
local uiCursor = extensions.ui_liveryEditor_layers_cursor
local uiUtils = extensions.ui_liveryEditor_utils

local ACTIONS = {"transform", "material", "order", "duplicate", "mirror", "rename", "visibility", "delete"}
local MEASUREMENTS = {
  ROTATE_STEP_UNIT = 0.1,
  SCALE_STEP_UNIT = 0.01,
  SKEW_STEP_UNIT = 0.01
}

local getLayerActions = function()
  return ACTIONS
end

local scaleX = function(layer, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  local oldScale = layer.decalScale
  layer.decalScale = vec3(oldScale.x + diff, oldScale.y, oldScale.z)
  api.setLayer(layer, true)
end

local scaleY = function(layer, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  local oldScale = layer.decalScale
  layer.decalScale = vec3(oldScale.x, 1, oldScale.z + diff)
  api.setLayer(layer, true)
end

local skewX = function(layer, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  layer.decalSkew = Point2F(layer.decalSkew.x + diff, layer.decalSkew.y)
  api.setLayer(layer, true)
end

local skewY = function(layer, steps)
  local diff = steps * MEASUREMENTS.SCALE_STEP_UNIT
  layer.decalSkew = Point2F(layer.decalSkew.x, layer.decalSkew.y + diff)
  api.setLayer(layer, true)
end

local getData = function(layer)
  local cursorPos = layer.cursorPosScreenUv
  local skew = layer.decalSkew
  local scale = layer.decalScale

  return {
    uid = layer.uid,
    decalTexturePath = layer.decalColorTexturePath,
    scale = {
      x = uiUtils.roundAndTruncateDecimal(scale.x, 2),
      y = uiUtils.roundAndTruncateDecimal(scale.z, 2)
    },
    skew = {
      x = uiUtils.roundAndTruncateDecimal(skew.x, 2),
      y = uiUtils.roundAndTruncateDecimal(skew.y, 2)
    },
    cursorPosition = {
      x = uiUtils.roundAndTruncateDecimal(cursorPos.x, 3),
      y = uiUtils.roundAndTruncateDecimal(cursorPos.y, 3)
    },
    rotation = uiUtils.roundAndTruncateDecimal(uiUtils.convertRadiansToDegrees(layer.decalRotation)),
    color = layer.color:toTable(),
    metallicIntensity = uiUtils.roundAndTruncateDecimal(layer.metallicIntensity, 2),
    roughnessIntensity = uiUtils.roundAndTruncateDecimal(layer.roughnessIntensity, 2),
    mirrored = layer.mirrored,
    flipMirroredDecal = layer.flipMirroredDecal,
    mirrorOffset = api.mirrorOffset,
    isUseMousePos = api.isUseMousePos(),
    isProjectSurfaceNormal = api.getUseSurfaceNormal(),
    applied = true
  }
end

local notifyListeners = function(layer)
  guihooks.trigger("LiveryEditor_CursorUpdated", M.getData(layer))
end

local showDecalCursor = function(show)
  local r, g, b = unpack(api.getDecalColor():toTable())
  api.setDecalColor(Point4F.fromTable({r, g, b, show and 1 or 0}))
end

M.layerUid = nil
M.layerData = nil

M.setLayer = function(layerUid)
  M.layerUid = layerUid
  M.layerData = deepcopy(api.getLayerByUid(M.layerUid))
  -- notify ui and extensions
end

M.addLayer = function(params)
  -- use cursor
  api.toggleSetting(api.settingsFlags.UseMousePos.value)
  api.setCursorPosition(Point2F(0.5, 0.5))

  showDecalCursor(true)
  api.setDecalTexturePath("color", params.texturePath)
  return api.addDecal()
end

M.setColor = function(layer, color)
  layer.color = Point4F.fromTable(color)
  api.setLayer(layer, true)
  M.notifyListeners(layer)
end

M.setMetallicIntensity = function(layer, value)
  layer.metallicIntensity = value
  api.setLayer(layer, true)
  M.notifyListeners(layer)
end

M.setRoughnessIntensity = function(layer, value)
  layer.roughnessIntensity = value
  api.setLayer(layer, true)
  M.notifyListeners(layer)
end

M.rotate = function(layer, degrees, counterClockwise)
  local steps = degrees * MEASUREMENTS.ROTATE_STEP_UNIT * (counterClockwise and 1 or -1)
  local layerRotation = uiUtils.convertRadiansToDegrees(layer.decalRotation)
  local newRotation = layerRotation - steps

  newRotation = uiUtils.roundAndTruncateDecimal(newRotation, 1)
  newRotation = uiUtils.cycleRange(newRotation, 0, 360)

  layer.decalRotation = uiUtils.convertDegreesToRadians(newRotation)
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.setRotation = function(layer, degrees)
  layer.decalRotation = uiUtils.convertDegreesToRadians(degrees)
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.scale = function(layer, steps_x, steps_y)
  if steps_x and steps_x ~= 0 then
    scaleX(layer, steps_x)
    notifyListeners(layer)
  end

  if steps_y and not steps_y ~= 0 then
    scaleY(layer, steps_y)
    notifyListeners(layer)
  end
end

M.setScale = function(layer, scaleX, scaleY)
  layer.decalScale = vec3(scaleX, layer.decalScale.y, scaleY)
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.skew = function(layer, stepsX, stepsY)
  if stepsX and stepsX ~= 0 then
    skewX(layer, stepsX)
    notifyListeners(layer)
  end

  if stepsY and not stepsY ~= 0 then
    skewY(layer, stepsY)
    notifyListeners(layer)
  end
end

M.setSkew = function(layer, skewX, skewY)
  layer.decalSkew = Point2F(skewX, skewY)
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.setMirrored = function(layer, mirrored, flipped)
  layer.mirrored = mirrored
  layer.flipMirroredDecal = flipped or false
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.setDecal = function(layer, texture)
  layer.decalColorTexturePath = texture
  api.setLayer(layer, true)
  notifyListeners(layer)
end

M.getLayerActions = getLayerActions
M.getData = getData
M.requestData = notifyListeners
M.notifyListeners = notifyListeners

return M
