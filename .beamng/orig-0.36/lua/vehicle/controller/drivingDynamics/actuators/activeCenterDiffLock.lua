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

local configPacket = {sourceType = "electronicCenterDiffLock", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "electronicCenterDiffLock"}

local relevantDiff = nil

local outputAV1Smoother = newExponentialSmoothing(500)
local outputAV2Smoother = newExponentialSmoothing(500)
local lockCoefSmoother = newTemporalSmoothingNonLinear(8, 20)

local outputAV1 = 0
local outputAV2 = 0
local avDiff = 0
local lockCoef = 0
local preOverrideLockCoef = 0
local lockCoefPID
local applyMinimumLockOnlyWithThrottle = true

local function resetOverride()
  M.overrideMin = 0
  M.overrideMax = 1
end

local function updateWheelsIntermediate(dt)
  local params = controlParameters
  local lastLockCoef = lockCoef
  lockCoef = 0

  outputAV1 = outputAV1Smoother:get(relevantDiff.outputAV1)
  outputAV2 = outputAV2Smoother:get(relevantDiff.outputAV2)

  avDiff = (outputAV1 - outputAV2) * sign(outputAV1)

  if params.isEnabled then
    local avHighEnough = abs((outputAV1 + outputAV2) * 0.5) > params.avThreshold

    if avHighEnough then
      local protectedAVDiff = abs(abs(outputAV1) - abs(outputAV2))
      lockCoef = lockCoefPID:get(-protectedAVDiff, -params.avDiffThreshold, dt)
    end

    local isBraking = electrics.values.brake > 0
    local isShifting = electrics.values.isShifting
    local isUsingParkingBrake = electrics.values.parkingbrake > 0
    local isESCActive = false

    if isBraking or isESCActive or electrics.values.throttle <= 0 then
      lockCoef = 0
    end

    if isShifting and not isBraking then
      lockCoef = lastLockCoef
    end

    local minimumLockCoef = applyMinimumLockOnlyWithThrottle and electrics.values.throttle or 1
    if isBraking and applyMinimumLockOnlyWithThrottle then
      minimumLockCoef = 0
    end
    lockCoef = max(lockCoef, params.minimumLock * minimumLockCoef)

    if isUsingParkingBrake then
      lockCoef = 0
    end
    lockCoef = lockCoefSmoother:get(lockCoef, dt)
  end

  preOverrideLockCoef = lockCoef --used for debugging purposes in the UI
  lockCoef = clamp(lockCoef, M.overrideMin, M.overrideMax) --depending on powertrain layout either min or max is used for the override

  relevantDiff.activeLockCoef = lockCoef
  M.isActing = lockCoef > 0.01
end

local function updateGFX(dt)
  if not controlParameters.isEnabled then
    return
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.outputAV1 = outputAV1
  debugPacket.outputAV2 = outputAV2
  debugPacket.avDiff = avDiff

  debugPacket.lockCoef = lockCoef
  debugPacket.preOverrideLockCoef = preOverrideLockCoef
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
  if relevantDiff then
    relevantDiff.activeLockCoef = 0
  end
  outputAV1 = 0
  outputAV2 = 0
  avDiff = 0
  lockCoef = 0
  outputAV1Smoother:reset()
  outputAV2Smoother:reset()
end

local function init(jbeamData)
  M.isActing = false
  controlParameters.isEnabled = true

  controlParameters.avDiffThreshold = jbeamData.avDiffThreshold or 4
  controlParameters.avThreshold = jbeamData.avThreshold or 1
  controlParameters.minimumLock = jbeamData.minimumLock or 0
  controlParameters.clutchPIDkP = jbeamData.clutchPIDkP or 0.15
  controlParameters.clutchPIDkI = jbeamData.clutchPIDkI or 0.001
  controlParameters.clutchPIDkD = jbeamData.clutchPIDkD or 0

  initialControlParameters = deepcopy(controlParameters)

  lockCoefPID = newPIDParallel(controlParameters.clutchPIDkP, controlParameters.clutchPIDkI, controlParameters.clutchPIDkD, 0, 1, nil, nil, 0)

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
    log("W", "electronicCenterDiffLock.initSecondStage", "No CMU present, disabling system...")
    shutdown()
    return
  end

  local centerDiffName = jbeamData.centerDiffName
  if not centerDiffName then
    log("E", "electronicCenterDiffLock.initSecondStage", "No centerDiffName configured, disabling system...")
    return
  end
  relevantDiff = powertrain.getDevice(centerDiffName)

  if not relevantDiff then
    log("E", "electronicCenterDiffLock.initSecondStage", string.format("Can't find configured centerDiff (%q), disabling system...", centerDiffName))
    return
  end

  relevantDiff.activeLockCoef = 0

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
    lockCoefPID:setConfig(controlParameters.clutchPIDkP, controlParameters.clutchPIDkI, controlParameters.clutchPIDkD, 0, 1, nil, nil, 0)
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
