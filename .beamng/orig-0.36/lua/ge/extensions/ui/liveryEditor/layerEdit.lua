-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local uiLayers = extensions.ui_liveryEditor_layers
local uiDecal = extensions.ui_liveryEditor_layers_decal
local utils = extensions.ui_liveryEditor_utils
local uiCamera = extensions.ui_liveryEditor_camera

local MEASUREMENTS = {
  TRANSLATE_STEP_UNIT = 0.001,
  ROTATE_STEP_UNIT = 0.01,
  SCALE_STEP_UNIT = 0.01,
  SKEW_STEP_UNIT = 0.01
}

local useCursorForTransform = true

local notifyListeners = function(hookName, hookData)
  local fullHookName = "liveryEditor_layerEdit"
  if hookName then
    fullHookName = fullHookName .. "_" .. hookName
  end

  guihooks.trigger(fullHookName, hookData)
end

local showCursor = function(show)
  local r, g, b = unpack(api.getDecalColor():toTable())
  api.setDecalColor(Point4F.fromTable({r, g, b, show and 1 or 0}))
end

M.editState = {}

M.setup = function()
  M.resetEditState()
end

M.editNewDecal = function(params)
  M.editState.layerType = api.layerTypes.decal
  M.editState.isAdd = true

  if not api.isUseMousePos() then
    api.setCursorPosition(Point2F(0.5, 0.5))
  end

  api.setDecalTexturePath("color", params.texturePath)
end

M.editExistingLayer = function(layerUid, replaceOnSave)
  M.resetEditState()
  M.editState.layerUid = layerUid
  M.editState.isAdd = false
  M.editState.layerType = api.layerTypes.decal
  M.editState.replaceOnSave = replaceOnSave

  local layer = api.getLayerByUid(layerUid)
  if layer then
    -- switch to freecam and set camera to decal camera properties
    uiCamera.setCameraByLayer(layer)

    -- keep original copy to restore original on cancel changes
    M.editState.layer = deepcopy(layer)
    M.editState.layerType = layer.type

    -- hide layer and show cursor
    if replaceOnSave then
      M.setCursorProperties(layer)
      layer.enabled = false
      api.setLayer(layer, true)
      showCursor(true)
    end

    pushActionMap("LiveryEditorTransform")
  else
    log("W", "", "Layer not found. Unable to set layer " .. layerUid)
  end
end

M.setCursorProperties = function(layer)
  api.setCursorPosition(Point2F(layer.cursorPosScreenUv.x, layer.cursorPosScreenUv.y))
  api.setDecalTexturePath("color", layer.decalColorTexturePath)
  api.setDecalColor(layer.color)
  api.setDecalScale(layer.decalScale)
  api.setDecalRotation(layer.decalRotation)
  api.setMetallicIntensity(layer.metallicIntensity)
  api.setRoughnessIntensity(layer.roughnessIntensity)
  api.setMirrored(layer.mirrored)
  api.setFlipMirroredDecal(layer.flipMirroredDecal)
  if layer.mirrorOffset then
    api.setMirrorOffset(layer.mirrorOffset)
  end
end

M.translateLayer = function(steps_x, steps_y)
  if api.isUseMousePos() then
    return
  end

  M.isPositionChanged = true

  local pos
  if useCursorForTransform then
    local cursorPos = api.getCursorPosition()
    local translateX = cursorPos.x + (steps_x * MEASUREMENTS.TRANSLATE_STEP_UNIT)
    local translateY = cursorPos.y + (steps_y * MEASUREMENTS.TRANSLATE_STEP_UNIT * -1)
    pos = Point2F(translateX, translateY)
    api.setCursorPosition(pos)
  end

  return {
    x = utils.roundAndTruncateDecimal(pos.x, 3),
    y = utils.roundAndTruncateDecimal(pos.y, 3)
  }
end

M.setPosition = function(x, y)
  if api.isUseMousePos() then
    return
  end

  M.isPositionChanged = true

  local pos = Point2F(x, y)

  api.setCursorPosition(pos)

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.cursorPosScreenUv = pos
  api.setLayer(layer, true)
end

M.scaleLayer = function(steps_x, steps_y)
  local calculateScale = function(scaleVec, stepsX, stepsY)
    local scaleX = scaleVec.x + steps_x * MEASUREMENTS.SCALE_STEP_UNIT
    local scaleY = scaleVec.z + steps_y * MEASUREMENTS.SCALE_STEP_UNIT
    -- prevent negative scale
    if scaleX < 0 or scaleY < 0 then
      return scaleVec
    end
    return vec3(scaleX, scaleVec.y, scaleY)
  end

  local scale
  if useCursorForTransform then
    scale = calculateScale(api.getDecalScale(), steps_x, steps_y)
    api.setDecalScale(scale)
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalScale = scale
  api.setLayer(layer, true)

  return {
    x = utils.roundAndTruncateDecimal(scale.x, 2),
    y = utils.roundAndTruncateDecimal(scale.z, 2)
  }
end

M.setScale = function(x, y, lock)
  local scale = vec3(x, 1, y)
  if useCursorForTransform then
    api.setDecalScale(scale)
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalScale = scale
  api.setLayer(layer, true)
end

M.skewLayer = function(x, y)
  local calculateSkew = function(skew, x, y)
    local skewX = skew.x + x * MEASUREMENTS.SKEW_STEP_UNIT
    local skewY = skew.y + y * MEASUREMENTS.SKEW_STEP_UNIT
    return Point2F(skewX, skewY)
  end

  local skew

  if useCursorForTransform then
    skew = calculateSkew(api.getDecalSkew(), x, y)
    api.setDecalSkew(skew)
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalSkew = skew
  api.setLayer(layer, true)

  return {
    x = utils.roundAndTruncateDecimal(skew.x, 2),
    y = utils.roundAndTruncateDecimal(skew.y, 2)
  }
end

M.setSkew = function(x, y)
  local skew = Point2F(x, y)

  if useCursorForTransform then
    api.setDecalSkew(skew)
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalSkew = skew
  api.setLayer(layer, true)
end

M.rotateLayer = function(steps, counterClockwise)
  local processSteps = function(rotation, steps, counterClockwise)
    local addDeg = steps * MEASUREMENTS.ROTATE_STEP_UNIT * (counterClockwise and -1 or 1)
    local rotDeg = utils.convertRadiansToDegrees(rotation)
    local newRotDeg = rotDeg + addDeg

    newRotDeg = utils.roundAndTruncateDecimal(newRotDeg, 1)
    newRotDeg = utils.cycleRange(newRotDeg, 0, 360)
    return newRotDeg
  end

  local rotation

  if useCursorForTransform then
    rotation = processSteps(api.getDecalRotation(), steps, counterClockwise)
    api.setDecalRotation(utils.convertDegreesToRadians(rotation))
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalRotation = utils.convertDegreesToRadians(rotation)
  api.setLayer(layer, true)

  return rotation
end

M.setRotation = function(degrees)
  local rads = utils.convertDegreesToRadians(degrees)

  if useCursorForTransform then
    api.setDecalRotation(rads)
  end

  local layer = api.getLayerByUid(M.editState.layerUid)
  layer.decalRotation = rads
  api.setLayer(layer, true)
end

M.setLayerMaterials = function(properties)
  local layer = api.getLayerByUid(M.editState.layerUid)

  if properties.metallicIntensity then
    -- ui value ranges from 0 - 100
    local metallicIntensity = properties.metallicIntensity / 100
    if M.editState.replaceOnSave then
      api.setMetallicIntensity(metallicIntensity)
    else
      layer.metallicIntensity = metallicIntensity
    end
  end

  if properties.roughnessIntensity then
    -- ui value ranges from 0 - 100
    local roughnessIntensity = properties.roughnessIntensity / 100
    if M.editState.replaceOnSave then
      api.setRoughnessIntensity(roughnessIntensity)
    else
      layer.roughnessIntensity = roughnessIntensity
    end
  end

  if properties.color then
    local r, g, b = unpack(properties.color)
    local color = Point4F(r, g, b, 1)

    if M.editState.replaceOnSave then
      api.setDecalColor(color)
    else
      layer.color = color
    end
  end

  if not M.editState.replaceOnSave then
    api.setLayer(layer, true)
  end
end

M.stampDecal = function()
  popActionMap("LiveryEditorTransformStamp")
  M.applyReposition()
  M.toggleUseMouseOrCursor()
end

M.requestStateData = function()
  notifyListeners("state", M.editState)
end

M.requestLayerMaterials = function()
  local layerMaterials

  if M.editState.layerType == api.layerTypes.decal then
    if M.editState.isAdd then
      layerMaterials = {
        color = api.getDecalColor():toTable(),
        metallicIntensity = api.getMetallicIntensity() * 100,
        roughnessIntensity = api.getRoughnessIntensity() * 100
      }
    else
      -- get rendered layer instead of cached M.editState.layer because this is the updated one. latter will be used as reference for restore
      local layer = api.getLayerByUid(M.editState.layerUid)
      layerMaterials = {
        color = layer.color:toTable(),
        metallicIntensity = layer.metallicIntensity * 100,
        roughnessIntensity = layer.roughnessIntensity * 100
      }
    end
  end

  notifyListeners("layerMaterialsData", layerMaterials)
end

M.cancelChanges = function()
  if M.editState.replaceOnSave then
    local layer = api.getLayerByUid(M.editState.layerUid)
    layer.enabled = true
    api.setLayer(layer, true)
    showCursor(false)
  else
    -- restore original layer properties
    api.setLayer(M.editState.layer, true)
  end
  -- api.setLayer(M.editState.layer, true)
  M.resetEditState()
  -- uiCamera.setOrthographicView("default")
  uiCamera.switchToOrbit()
end

M.saveChanges = function()
  if M.editState.replaceOnSave then
    if M.isPositionChanged then
      local lastIndex = uiLayers.getChildrenCount()
      local referenceDecal = api.addDecal()
      local layer = deepcopy(referenceDecal)
      layer.uid = M.editState.layerUid
      layer.name = M.editState.layer.name
      layer.enabled = true

      -- set color alpha to 1
      layer.color = Point4F(layer.color.x, layer.color.y, layer.color.z, 1)

      api.setLayer(layer, true)
      api.removeLayer(lastIndex + 1)
      showCursor(false)
    else
      -- local layer = api.getLayerByUid(M.editState.layerUid)
      -- layer.enabled = true
      -- api.setLayer(layer, true)
    end
  end

  M.resetEditState()

  -- uiCamera.setOrthographicView("default")
  uiCamera.switchToOrbit()
end

M.requestReposition = function()
  if not useCursorForTransform then
    local layer = api.getLayerByUid(M.editState.layerUid)
    M.setCursorProperties(layer)
    showCursor(true)
    layer.enabled = false
    api.setLayer(layer, true)
  end
  -- uiCamera.setOrthographicView("default")
  uiCamera.switchToOrbit(true)
end

M.cancelReposition = function()
  local layer = api.getLayerByUid(M.editState.layerUid)
  if not useCursorForTransform then
    layer.enabled = true
    api.setLayer(layer, true)
    showCursor(false)
  end
  uiCamera.setCameraByLayer(layer)
end

M.applyReposition = function()
  local layer
  local lastIndex = uiLayers.getChildrenCount()
  local referenceDecal = api.addDecal()
  layer = deepcopy(referenceDecal)
  layer.uid = M.editState.layerUid
  layer.name = M.editState.layer.name
  layer.enabled = not useCursorForTransform
  api.setLayer(layer, true)
  api.removeLayer(lastIndex + 1)

  uiCamera.setCameraByLayer(layer)

  if useCursorForTransform then
    M.setCursorProperties(layer)
  else
    showCursor(false)
  end

  notifyListeners("repositionSuccess")
  M.requestInitialLayerData()
end

M.toggleUseMouseOrCursor = function()
  if M.editState.useMouseRealValue ~= api.isUseMousePos() then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
  local isUseMouse = api.isUseMousePos()
  if isUseMouse then
    pushActionMap("LiveryEditorTransformStamp")
  else
    popActionMap("LiveryEditorTransformStamp")
  end

  return {
    isUseMouse = isUseMouse
  }
end

M.requestTransform = function()
  if M.editState.layerUid then
    local layer = api.getLayerByUid(M.editState.layerUid)
    layer.enabled = false
    api.setLayer(layer, true)
    M.setCursorProperties(layer)

    if api.isUseMousePos() then
      M.editState.isStampReapplying = true
    end
  end

  if api.isUseMousePos() then
    pushActionMap("LiveryEditorTransformStamp")
  end
  pushActionMap("LiveryEditorTransform")
  showCursor(true)
end

M.endTransform = function()
  if api.isUseMousePos() then
    popActionMap("LiveryEditorTransformStamp")
    M.editState.isStampReapplying = false
  end
  popActionMap("LiveryEditorTransform")
  showCursor(false)
end

M.showCursorOrLayer = function(show)
  if not M.editState.isAdd or (M.editState.isAdd and M.editState.layerUid) then
    return
  end
  if M.editState.isAdd then
    if show then
      local useMouse = api.isUseMousePos()
      M.editState.useMouseRealValue = useMouse
      if useMouse then
        api.toggleSetting(api.settingsFlags.UseMousePos.value)
      end
    else
      if M.editState.useMouseRealValue ~= api.isUseMousePos() then
        api.toggleSetting(api.settingsFlags.UseMousePos.value)
      end
      M.editState.useMouseRealValue = api.isUseMousePos()
    end

    showCursor(show)
  else
    local layer = api.getLayerByUid(M.editState.layerUid)
    if layer then
      layer.enabled = show
      api.setLayer(layer, true)
    end
  end
end

M.requestInitialLayerData = function()
  local initData = {}

  local pos = api.getCursorPosition()
  initData["position"] = {
    x = utils.roundAndTruncateDecimal(pos.x, 3),
    y = utils.roundAndTruncateDecimal(pos.y, 3)
  }
  initData.useSurfaceNormal = api.getUseSurfaceNormal()

  if M.editState and M.editState.layer then
    initData["scale"] = {
      x = utils.roundAndTruncateDecimal(M.editState.layer.decalScale.x, 2),
      y = utils.roundAndTruncateDecimal(M.editState.layer.decalScale.z, 2)
    }

    initData["skew"] = {
      x = utils.roundAndTruncateDecimal(M.editState.layer.decalSkew.x, 2),
      y = utils.roundAndTruncateDecimal(M.editState.layer.decalSkew.y, 2)
    }

    initData.rotation = utils.roundAndTruncateDecimal(utils.convertRadiansToDegrees(M.editState.layer.decalRotation), 1)
    initData.metallicIntensity = utils.roundAndTruncateDecimal(M.editState.layer.metallicIntensity, 2)
    initData.roughnessIntensity = utils.roundAndTruncateDecimal(M.editState.layer.roughnessIntensity, 2)
    initData.color = M.editState.layer.color:toTable()
    initData.mirrored = M.editState.layer.mirrored
    initData.mirrorFlipped = M.editState.layer.mirrorFlippedDecal
    initData.mirrorOffset = M.editState.layer.mirrorOffset
  else
    local scale = api.getDecalScale()
    initData["scale"] = {
      x = utils.roundAndTruncateDecimal(scale.x, 2),
      y = utils.roundAndTruncateDecimal(scale.z, 2)
    }

    local skew = api.getDecalSkew()
    initData["skew"] = {
      x = utils.roundAndTruncateDecimal(skew.x, 2),
      y = utils.roundAndTruncateDecimal(skew.y, 2)
    }

    initData.rotation = utils.roundAndTruncateDecimal(utils.convertRadiansToDegrees(api.getDecalRotation()), 1)
    initData.metallicIntensity = utils.roundAndTruncateDecimal(api.getMetallicIntensity(), 2)
    initData.roughnessIntensity = utils.roundAndTruncateDecimal(api.getRoughnessIntensity(), 2)
    initData.color = api.getDecalColor():toTable()
    initData.mirrored = api.getMirrored()
    initData.mirrorFlipped = api.getFlipMirroredDecal()
    initData.mirrorOffset = api.getMirrorOffset()
  end

  notifyListeners("initialLayerData", initData)
end

M.resetEditState = function()
  M.editState = {
    layerUid = nil,
    layerType = nil,
    isAdd = nil,
    layer = nil,
    replaceOnSave = nil,
    isStampReapplying = nil,
    useMouseRealValue = nil
  }
  M.isPositionChanged = false
end

M.allowRotationAction = true
M.isRotationPrecise = false
M.isPositionChanged = false
M.showCursor = showCursor

M.setIsRotationPrecise = function(value)
  M.isRotationPrecise = value
end

M.setAllowRotationAction = function(value)
  M.allowRotationAction = value
end

-- controller translate actions
M.holdAction = nil
M.holdValue = nil
M.holdAxis = nil
M.precise = false
M.holdTime = 0

local resetHoldState = function()
  M.holdTime = 0
  M.holdAction = nil
  M.holdAxis = nil
  M.holdValue = nil
end

local startHold = function(action)
  M.holdAction = action
  M.holdTime = 0
end

M.holdPrecise = function(enable)
  M.precise = enable
end

M.holdRotate = function(direction, value)
  if (M.holdAction and M.holdAction ~= "rotate") or (M.holdValue and M.holdValue ~= direction) then
    return
  end

  if value == 1 then
    M.holdValue = direction
    startHold("rotate")
  else
    resetHoldState()
  end
end

-- axis: x or y, value: -1, 0, 1
M.holdTranslate = function(axis, value)
  if (M.holdAction and M.holdAction ~= "translate") or (M.holdAxis and M.holdAxis ~= axis) then
    return
  end

  -- reset hold if value is 0
  if value == 0 then
    resetHoldState()
  else
    M.holdAxis = axis
    M.holdValue = value
    startHold("translate")
  end
end

M.holdTranslateScalar = function(axis, value)
  local threshold = 0.5

  if (M.holdAction and M.holdAction ~= "translate_scalar") or (M.holdAxis and M.holdAxis ~= axis) then
    return
  end

  if math.abs(value) < threshold or (M.holdValue and (value < M.holdValue * 0.9 or value > M.holdValue * 1.1)) then
    resetHoldState()
  else
    M.holdAxis = axis
    M.holdValue = value < 0 and -1 or 1
    startHold("translate_scalar")
  end
end

M.holdScale = function(axis, value)
  local threshold = 0.5

  if (M.holdAction and M.holdAction ~= "scale") or (M.holdAxis and M.holdAxis ~= axis) then
    return
  end

  if math.abs(value) < threshold or (M.holdValue and value < M.holdValue * 0.9) then
    resetHoldState()
  else
    M.holdAxis = axis
    M.holdValue = value < 0 and -1 or 1
    startHold("scale")
  end
end

M.holdSkew = function(axis, value)
  local threshold = 0.5

  if (M.holdAction and M.holdAction ~= "skew") or (M.holdAxis and M.holdAxis ~= axis) then
    return
  end

  if math.abs(value) < threshold or (M.holdValue and value < M.holdValue * 0.9) then
    resetHoldState()
  else
    M.holdAxis = axis
    M.holdValue = value < 0 and -1 or 1
    startHold("skew")
  end
end

M.holdTranslateAction = function()
  local x = 0
  local y = 0
  local multiplier = M.precise and 0.1 or 1

  if M.holdAxis == "x" then
    x = M.holdValue * multiplier
  elseif M.holdAxis == "y" then
    y = M.holdValue * multiplier
  end

  local res = M.translateLayer(x, y)
  notifyListeners("positionChanged", res)
end
-- end controller translate actions

M.holdRotateAction = function()
  if not M.allowRotationAction then
    return
  end

  local multiplier = M.precise and 1 or 10
  local counterClockwise = M.holdValue == -1
  local rotation = M.rotateLayer(multiplier, counterClockwise)
  notifyListeners("rotationChanged", rotation)
end

M.holdScaleAction = function()
  local x = 0
  local y = 0
  local multiplier = M.precise and 0.1 or 1

  if M.holdAxis == "x" then
    x = M.holdValue * multiplier
  elseif M.holdAxis == "y" then
    y = M.holdValue * multiplier
  end

  local res = M.scaleLayer(x, y)
  notifyListeners("scaleChanged", res)
end

M.holdSkewAction = function()
  local x = 0
  local y = 0
  local multiplier = M.precise and 0.1 or 1

  if M.holdAxis == "x" then
    x = M.holdValue * multiplier
  elseif M.holdAxis == "y" then
    y = M.holdValue * multiplier
  end

  local res = M.skewLayer(x, y)
  notifyListeners("skewChanged", res)
end

local actionFns = function()
  if M.holdAction == "translate" or M.holdAction == "translate_scalar" then
    M.holdTranslateAction()
  elseif M.holdAction == "scale" then
    M.holdScaleAction()
  elseif M.holdAction == "skew" then
    M.holdSkewAction()
  else
    -- needs catch all since rotate is not being triggered by UI, but by actionmap instead
    M.holdRotateAction()
  end
end

M.onUpdate = function(dtReal, dtSim, dtRaw)
  if not M.holdAction then
    return
  end

  if M.holdTime == 0 or M.holdTime > 0.5 then
    actionFns()
  end

  M.holdTime = M.holdTime + dtSim
end

return M
