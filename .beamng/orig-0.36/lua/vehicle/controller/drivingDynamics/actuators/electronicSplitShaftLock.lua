-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

M.isActive = false
M.isActing = false

M.overrideMin = 0
M.overrideMax = 1

local max = math.max
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local configPacket = {sourceType = "electronicSplitShaftLock", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "electronicSplitShaftLock"}

local relevantSplitShaft = nil

local inputAVSmoother = newExponentialSmoothing(200)
local outputAVSmoother = newExponentialSmoothing(500)
local clutchRatioSmoother = newTemporalSmoothingNonLinear(20, 5)

local inputAV = 0
local outputAV = 0
local avDiff = 0
local clutchRatio = 0
local preOverrideClutchRatio = 0
local clutchRatioPID
local applyMinimumLockOnlyWithThrottle = true

local function resetOverride()
  M.overrideMin = 0
  M.overrideMax = 1
end

local function updateWheelsIntermediate(dt)
  local params = controlParameters
  local lastClutchRatio = clutchRatio
  clutchRatio = 0

  inputAV = inputAVSmoother:get(relevantSplitShaft.inputAV)
  outputAV = outputAVSmoother:get(relevantSplitShaft[relevantSplitShaft.secondaryOutputAVName])

  if outputAV * inputAV < 0 then --when ouput turns the other way than the input, set it to 0
    outputAV = 0
  end

  avDiff = (inputAV - outputAV) * sign(inputAV)

  if params.isEnabled then
    local avHighEnough = abs(inputAV) > params.avThreshold

    if avHighEnough then
      local protectedAVDiff = max(abs(inputAV) - abs(outputAV), 0)
      clutchRatio = clutchRatioPID:get(-protectedAVDiff, -params.avDiffThreshold, dt)
    end

    local isBraking = electrics.values.brake > 0
    local isShifting = electrics.values.isShifting
    local isUsingParkingBrake = electrics.values.parkingbrake > 0
    local isESCActive = false

    if isBraking or isESCActive or electrics.values.throttle <= 0 then
      clutchRatio = 0
    end

    if isShifting and not isBraking then
      clutchRatio = lastClutchRatio
    end

    local minimumLockCoef = applyMinimumLockOnlyWithThrottle and electrics.values.throttle or 1
    if isBraking and applyMinimumLockOnlyWithThrottle then
      minimumLockCoef = 0
    end
    clutchRatio = max(clutchRatio, params.minimumLock * minimumLockCoef)

    if isUsingParkingBrake then
      clutchRatio = 0
    end
    clutchRatio = clutchRatioSmoother:get(clutchRatio, dt)
  end

  preOverrideClutchRatio = clutchRatio --used for debugging purposes in the UI
  clutchRatio = clamp(clutchRatio, M.overrideMin, M.overrideMax) --depending on powertrain layout either min or max is used for the override

  relevantSplitShaft.clutchRatio = clutchRatio
  M.isActing = clutchRatio > 0.01
end

local function updateGFX(dt)
  if not controlParameters.isEnabled then
    return
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.inputAV = inputAV
  debugPacket.outputAV = outputAV
  debugPacket.avDiff = avDiff

  debugPacket.clutchRatio = clutchRatio
  debugPacket.preOverrideClutchRatio = preOverrideClutchRatio
  debugPacket.overrideMin = M.overrideMin
  debugPacket.overrideMax = M.overrideMax

  debugPacket.isEnabled = controlParameters.isEnabled
  debugPacket.avDiffThreshold = controlParameters.avDiffThreshold
  debugPacket.avThreshold = controlParameters.avThreshold

  debugPacket.isActing = M.isActing

  CMU.sendDebugPacket(debugPacket)
end

local function shutdown()
  M.isActive = false
  M.isActing = false
  M.updateGFX = nil
  M.updateWheelsIntermediate = nil
end

local function reset()
  M.isActing = false
  M.overrideMin = 0
  M.overrideMax = 1
end

local function init(jbeamData)
  M.isActing = false
  controlParameters.isEnabled = true

  controlParameters.avDiffThreshold = jbeamData.avDiffThreshold or 4
  controlParameters.avThreshold = jbeamData.avThreshold or 1
  controlParameters.minimumLock = jbeamData.minimumLock or 0
  controlParameters.clutchPIDkP = jbeamData.clutchPIDkP or 0.1
  controlParameters.clutchPIDkI = jbeamData.clutchPIDkI or 1
  controlParameters.clutchPIDkD = jbeamData.clutchPIDkD or 0

  initialControlParameters = deepcopy(controlParameters)

  clutchRatioPID = newPIDParallel(controlParameters.clutchPIDkP, controlParameters.clutchPIDkI, controlParameters.clutchPIDkD, 0, 1, nil, nil, 0)

  if jbeamData.applyMinimumLockOnlyWithThrottle ~= nil then
    applyMinimumLockOnlyWithThrottle = jbeamData.applyMinimumLockOnlyWithThrottle
  else
    applyMinimumLockOnlyWithThrottle = true
  end

  local nameString = jbeamData.name
  local slashPos = nameString:find("/", -nameString:len())
  if slashPos then
    nameString = nameString:sub(slashPos + 1)
  end
  debugPacket.sourceName = nameString
end

local function initSecondStage(jbeamData)
  if not CMU then
    log("W", "electronicSplitShaftLock.initSecondStage", "No CMU present, disabling system...")
    shutdown()
    return
  end

  local splitShaftName = jbeamData.splitShaftName
  if not splitShaftName then
    log("E", "electronicSplitShaftLock.initSecondStage", "No splitShaftName configured, disabling system...")
    return
  end
  relevantSplitShaft = powertrain.getDevice(splitShaftName)

  if not relevantSplitShaft then
    log("E", "electronicSplitShaftLock.initSecondStage", string.format("Can't find configured splitShaft (%q), disabling system...", splitShaftName))
    return
  end

  M.isActive = true
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function setParameters(parameters)
  if not CMU then
    return
  end

  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "avDiffThreshold")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "avThreshold")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "minimumLock")
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "isEnabled")
  local newP = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "clutchPIDkP")
  local newI = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "clutchPIDkI")
  local newD = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "clutchPIDkD")

  if newP or newI or newD then
    clutchRatioPID:setConfig(controlParameters.clutchPIDkP, controlParameters.clutchPIDkI, controlParameters.clutchPIDkD, 0, 1, nil, nil, 0)
  end
end

local function setConfig(configTable)
  controlParameters = configTable
end

local function getConfig()
  return deepcopy(controlParameters)
end

local function sendConfigData()
  configPacket.config = controlParameters
  CMU.sendDebugPacket(configPacket)
end

M.init = init
M.initSecondStage = initSecondStage

M.reset = reset

M.updateGFX = updateGFX
M.updateWheelsIntermediate = updateWheelsIntermediate

M.resetOverride = resetOverride

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

return M
