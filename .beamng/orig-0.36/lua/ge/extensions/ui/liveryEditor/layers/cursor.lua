-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals
local uiUtils = extensions.ui_liveryEditor_utils

local MEASUREMENTS = {
  TRANSLATE_STEP_UNIT = 0.001,
  ROTATE_STEP_UNIT = 0.1,
  SCALE_STEP_UNIT = 0.01,
  SKEW_STEP_UNIT = 0.01
}

local getData = function()
  local position = api.getCursorPosition()
  local skew = api.getDecalSkew()
  local scale = api.getDecalScale()
  return {
    decalTexturePath = api.getDecalTexturePath("color"),
    position = {
      x = uiUtils.roundAndTruncateDecimal(position.x, 3),
      y = uiUtils.roundAndTruncateDecimal(position.y, 3),
      maxX = 1,
      maxY = 1
    },
    cursorPosition = {
      x = uiUtils.roundAndTruncateDecimal(position.x, 3),
      y = uiUtils.roundAndTruncateDecimal(position.y, 3)
    },
    scale = {
      x = uiUtils.roundAndTruncateDecimal(scale.x, 2),
      y = uiUtils.roundAndTruncateDecimal(scale.z, 2)
    },
    skew = {
      x = uiUtils.roundAndTruncateDecimal(skew.x, 2),
      y = uiUtils.roundAndTruncateDecimal(skew.y, 2)
    },
    rotation = uiUtils.roundAndTruncateDecimal(uiUtils.convertRadiansToDegrees(api.getDecalRotation()), 1),
    color = api.getDecalColor():toTable(),
    metallicIntensity = uiUtils.roundAndTruncateDecimal(api.getMetallicIntensity(), 2),
    roughnessIntensity = uiUtils.roundAndTruncateDecimal(api.getRoughnessIntensity(), 2),
    mirrored = api.getMirrored(),
    flipMirroredDecal = api.getFlipMirroredDecal(),
    mirrorOffset = api.getMirrorOffset(),
    isUseMousePos = api.isUseMousePos(),
    isProjectSurfaceNormal = api.getUseSurfaceNormal(),
    applied = false
  }
end

local notifyListeners = function()
  guihooks.trigger("LiveryEditor_CursorUpdated", getData())
end

M.translate = function(steps_x, steps_y)
  local cursorPos = api.getCursorPosition()
  local translateX = cursorPos.x + (steps_x * MEASUREMENTS.TRANSLATE_STEP_UNIT)
  local translateY = cursorPos.y + (steps_y * MEASUREMENTS.TRANSLATE_STEP_UNIT * -1)
  api.setCursorPosition(Point2F(translateX, translateY))
  notifyListeners()
end

M.setPosition = function(posX, posY)
  api.setCursorPosition(Point2F(posX, posY))
  notifyListeners()
end

M.scale = function(steps_x, steps_y)
  local cursorScale = api.getDecalScale()
  local scaleX = cursorScale.x + (steps_x * MEASUREMENTS.SCALE_STEP_UNIT)
  local scaleY = cursorScale.z + (steps_y * MEASUREMENTS.SCALE_STEP_UNIT)
  api.setDecalScale(vec3(scaleX, cursorScale.y, scaleY))
  notifyListeners()
end

M.skew = function(steps_x, steps_y)
  local cursorSkew = api.getDecalSkew()
  local skewX = cursorSkew.x + (steps_x * MEASUREMENTS.SKEW_STEP_UNIT)
  local skewY = cursorSkew.y + (steps_y * MEASUREMENTS.SKEW_STEP_UNIT)
  api.setDecalSkew(Point2F(skewX, skewY))
  notifyListeners()
end

M.rotate = function(degrees, counterClockwise)
  local steps = degrees * MEASUREMENTS.ROTATE_STEP_UNIT * (counterClockwise and -1 or 1)
  local layerRotation = uiUtils.convertRadiansToDegrees(api.getDecalRotation())
  local newRotation = layerRotation + steps

  newRotation = uiUtils.roundAndTruncateDecimal(newRotation, 1)
  newRotation = uiUtils.cycleRange(newRotation, 0, 360)

  api.setDecalRotation(uiUtils.convertDegreesToRadians(newRotation))
  notifyListeners()
end

M.setColor = function(color)
  api.setDecalColor(Point4F.fromTable(color))
  notifyListeners()
end

M.setMetallicIntensity = function(value)
  api.setMetallicIntensity(value)
  notifyListeners()
end

M.setRoughnessIntensity = function(value)
  api.setRoughnessIntensity(value)
  notifyListeners()
end

M.setMirrored = function(mirrored, flipped)
  api.setMirrored(mirrored)
  api.setFlipMirroredDecal(flipped)
  notifyListeners()
end

M.setMirrorOffset = function(offset)
  api.setMirrorOffset(offset)
  notifyListeners()
end

M.setUseMousePos = function(value)
  if api.isUseMousePos() ~= value then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
    extensions.hook("liveryEditor_OnUseMousePosChanged", value)
    notifyListeners()
  end
end

M.setProjectSurfaceNormal = function(value)
  api.setUseSurfaceNormal(value)
  notifyListeners()
end

M.setScale = function(scaleX, scaleY)
  local cursorScale = api.getDecalScale()
  api.setDecalScale(vec3(scaleX, cursorScale.y, scaleY))
  notifyListeners()
end

M.setSkew = function(skewX, skewY)
  api.setDecalSkew(Point2F(skewX, skewY))
  notifyListeners()
end

M.setRotation = function(degrees)
  local rads = uiUtils.convertDegreesToRadians(degrees)
  api.setDecalRotation(rads)
  notifyListeners()
end

M.setDecal = function(texture)
  api.setDecalTexturePath("color", texture)
  local color = api.getDecalColor():toTable()

  if color[4] == 0 then
    color[4] = 1
    api.setDecalColor(Point4F.fromTable(color))
  end
end

M.setCursorProperties = function(params)
  if params.decalColorTexturePath then
    api.setDecalTexturePath("color", params.decalColorTexturePath)
  end
  if params.color then
    api.setDecalColor(params.color)
  end
  if params.cursorPosScreenUv then
    api.setCursorPosition(Point2F(params.cursorPosScreenUv.x, params.cursorPosScreenUv.y))
  end
  if params.decalScale then
    api.setDecalScale(params.decalScale)
  end
  if params.decalSkew then
    api.setDecalSkew(params.decalSkew)
  end
  if params.decalRotation then
    api.setDecalRotation(params.decalRotation)
  end
  if params.metallicIntensity then
    api.setMetallicIntensity(params.metallicIntensity)
  end
  if params.roughnessIntensity then
    api.setRoughnessIntensity(params.roughnessIntensity)
  end
  if params.mirrored then
    api.setMirrored(params.mirrored)
  end
  if params.flipMirroredDecal then
    api.setFlipMirroredDecal(params.flipMirroredDecal)
  end
  if params.mirrorOffset then
    api.setMirrorOffset(params.mirrorOffset)
  end
end

M.resetProperties = function(params)
  local data = {}
  local paramsEmpty = not params or #params == 0

  if paramsEmpty then
    api.setCursorPosition(M.DEFAULT_POSITION)
    data["decalScale"] = M.DEFAULT_SCALE
    data["decalSkew"] = M.DEFAULT_SKEW
    data["decalRotation"] = M.DEFAULT_ROTATION
    data["color"] = M.DEFAULT_COLOR
    data["metallicIntensity"] = M.DEFAULT_METALLICINTENSITY
    data["roughnessIntensity"] = M.DEFAULT_ROUGHNESSINTENSITY
    api.setMirrored(M.DEFAULT_MIRRORED)
    api.setFlipMirroredDecal(M.DEFAULT_FLIP_MIRRORED)
    api.setMirrorOffset(M.DEFAULT_MIRROR_OFFSET)
  else
    for _, value in ipairs(params) do
      if value == "position" or value == "transform" then
        api.setCursorPosition(M.DEFAULT_POSITION)
      elseif value == "scale" then
        data["decalScale"] = M.DEFAULT_SCALE
      elseif value == "skew" then
        data["decalSkew"] = M.DEFAULT_SKEW
      elseif value == "rotation" or value == "rotate" then
        data["decalRotation"] = M.DEFAULT_ROTATION
      elseif value == "color" then
        data["color"] = M.DEFAULT_COLOR
      elseif value == "metallicIntensity" then
        data["metallicIntensity"] = M.DEFAULT_METALLICINTENSITY
      elseif value == "roughnessIntensity" then
        data["roughnessIntensity"] = M.DEFAULT_ROUGHNESSINTENSITY
      elseif value == "material" then
        data["color"] = M.DEFAULT_COLOR
        data["metallicIntensity"] = M.DEFAULT_METALLICINTENSITY
        data["roughnessIntensity"] = M.DEFAULT_ROUGHNESSINTENSITY
      elseif value == "mirror" then
        api.setMirrored(M.DEFAULT_MIRRORED)
        api.setFlipMirroredDecal(M.DEFAULT_FLIP_MIRRORED)
        api.setMirrorOffset(M.DEFAULT_MIRROR_OFFSET)
      end
    end
  end

  M.setCursorProperties(data)
end

M.DEFAULT_POSITION = Point2F(0.5, 0.5)
M.DEFAULT_SCALE = vec3(0.5, 1, 0.5)
M.DEFAULT_SKEW = Point2F(0, 0)
M.DEFAULT_ROTATION = 0
M.DEFAULT_COLOR = Point4F(1, 1, 1, 1)
M.DEFAULT_METALLICINTENSITY = 0
M.DEFAULT_ROUGHNESSINTENSITY = 0
M.DEFAULT_MIRRORED = false
M.DEFAULT_FLIP_MIRRORED = false
M.DEFAULT_MIRROR_OFFSET = 0

M.getData = getData
M.requestData = notifyListeners
M.notifyListeners = notifyListeners

return M
