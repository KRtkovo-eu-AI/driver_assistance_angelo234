-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

M.isActive = false
M.isActing = false

M.overrideMax = 1

local min = math.min
local max = math.max
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local configPacket = {sourceType = "electronicDiffLock", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "electronicDiffLock"}

local relevantDifferential = nil
local relevantWheels = {}

local inputAVSmoother = newExponentialSmoothing(50)
local outputAV1Smoother = newExponentialSmoothing(50)
local outputAV2Smoother = newExponentialSmoothing(50)

local inputAV = 0
local outputAV1Corrected = 0
local outputAV2Corrected = 0
local avDiff = 0
local outputBrakingCoef = {}
local turningCircleRatioOutput1 = 0
local turningCircleRatioOutput2 = 0

local brakingPID
local side1BrakingSmoother = newTemporalSmoothingNonLinear(10, 10)
local side2BrakingSmoother = newTemporalSmoothingNonLinear(10, 10)

local function resetOverride()
  M.overrideMin = 0
  M.overrideMax = 1
end

local function updateWheelsIntermediate(dt)
  local params = controlParameters
  outputBrakingCoef[-1] = 0
  outputBrakingCoef[1] = 0

  local turningCircleSpeedRatios = CMU.vehicleData.turningCircleSpeedRatios

  turningCircleRatioOutput1 = 0
  for _, wheel in ipairs(relevantWheels[-1].wheels) do
    turningCircleRatioOutput1 = turningCircleRatioOutput1 + turningCircleSpeedRatios[wheel.name]
  end
  turningCircleRatioOutput1 = turningCircleRatioOutput1 * relevantWheels[-1].invWheelCount

  turningCircleRatioOutput2 = 0
  for _, wheel in ipairs(relevantWheels[1].wheels) do
    turningCircleRatioOutput2 = turningCircleRatioOutput2 + turningCircleSpeedRatios[wheel.name]
  end
  turningCircleRatioOutput2 = turningCircleRatioOutput2 * relevantWheels[1].invWheelCount

  inputAV = inputAVSmoother:get(relevantDifferential.inputAV * relevantDifferential.invGearRatio)
  outputAV1Corrected = outputAV1Smoother:get(relevantDifferential.outputAV1 * turningCircleRatioOutput1)
  outputAV2Corrected = outputAV2Smoother:get(relevantDifferential.outputAV2 * turningCircleRatioOutput2)

  if outputAV1Corrected * inputAV < 0 then --when one wheel turns the other way than the input, set it to 0
    outputAV1Corrected = 0
  end
  if outputAV2Corrected * inputAV < 0 then
    outputAV2Corrected = 0
  end
  avDiff = (outputAV2Corrected - outputAV1Corrected) * sign(inputAV)

  M.isActing = false
  if params.isEnabled then
    local avDiffCorrected = avDiff
    local brakingCoef = brakingPID:get(avDiffCorrected, 0, dt)
    local diffSideToBrake = sign(avDiffCorrected)
    if diffSideToBrake ~= 0 and electrics.values.throttle > 0.05 then
      local shouldBrake = abs(avDiffCorrected) > params.avDiffThreshold and abs(outputAV1Corrected) + abs(outputAV2Corrected) > params.avThreshold
      brakingCoef = shouldBrake and abs(brakingCoef) or 0
      local brakingCoefSmooth = relevantWheels[diffSideToBrake].brakeSmoother:get(brakingCoef, dt)
      brakingCoefSmooth = min(brakingCoefSmooth, M.overrideMax)
      for _, wd in ipairs(relevantWheels[diffSideToBrake].wheels) do --todo simple loop
        if not wd.isBroken then
          local brakingTorque = brakingCoefSmooth * wd.brakeTorque
          wd.desiredBrakingTorque = min(wd.desiredBrakingTorque + brakingTorque, wd.brakeTorque)
          outputBrakingCoef[diffSideToBrake] = brakingCoefSmooth
        end
      end
      M.isActing = shouldBrake
    else
      brakingPID:reset()
      side1BrakingSmoother:reset()
      side2BrakingSmoother:reset()
    end
  end
end

local function updateGFX(dt)
  if not controlParameters.isEnabled then
    return
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.inputAV = inputAV
  debugPacket.outputAV1 = outputAV1Corrected
  debugPacket.outputAV2 = outputAV2Corrected
  debugPacket.avDiff = avDiff

  debugPacket.turningCircleRatioOutput1 = turningCircleRatioOutput1
  debugPacket.turningCircleRatioOutput2 = turningCircleRatioOutput2

  debugPacket.side1BrakingCoef = outputBrakingCoef[-1]
  debugPacket.side2BrakingCoef = outputBrakingCoef[1]

  debugPacket.avDiffThreshold = controlParameters.avDiffThreshold
  debugPacket.avThreshold = controlParameters.avThreshold

  debugPacket.overrideMax = M.overrideMax

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
  M.overrideMax = 1
end

local function init(jbeamData)
  M.isActing = false
  controlParameters.isEnabled = true

  controlParameters.avDiffThreshold = jbeamData.avDiffThreshold or 5
  controlParameters.avThreshold = jbeamData.avThreshold or 1
  controlParameters.brakingPIDkP = jbeamData.brakingPIDkP or 0
  controlParameters.brakingPIDkI = jbeamData.brakingPIDkI or 0.1
  controlParameters.brakingPIDkD = jbeamData.brakingPIDkD or 0

  brakingPID = newPIDParallel(controlParameters.brakingPIDkP, controlParameters.brakingPIDkI, controlParameters.brakingPIDkD, -1, 1)

  initialControlParameters = deepcopy(controlParameters)

  local nameString = jbeamData.name
  local slashPos = nameString:find("/", -nameString:len())
  if slashPos then
    nameString = nameString:sub(slashPos + 1)
  end
  debugPacket.sourceName = nameString
  configPacket.sourceName = nameString
end

local function initSecondStage(jbeamData)
  if not CMU then
    log("W", "electronicDiffLock.initSecondStage", "No CMU present, disabling system...")
    shutdown()
    return
  end

  local diffName = jbeamData.differentialName
  if not diffName then
    log("E", "electronicDiffLock.initSecondStage", "No differentialName configured, disabling system...")
    return
  end
  relevantDifferential = powertrain.getDevice(diffName)

  if not relevantDifferential then
    log("E", "electronicDiffLock.initSecondStage", string.format("Can't find configured differential (%q), disabling system...", diffName))
    return
  end

  relevantWheels = {}
  relevantWheels[-1] = {}
  relevantWheels[1] = {}
  relevantWheels[-1].wheels = powertrain.getChildWheels(relevantDifferential, 1)
  relevantWheels[1].wheels = powertrain.getChildWheels(relevantDifferential, 2)
  relevantWheels[-1].invWheelCount = #relevantWheels[-1].wheels > 0 and #relevantWheels[-1].wheels or 1
  relevantWheels[1].invWheelCount = #relevantWheels[1].wheels > 0 and #relevantWheels[1].wheels or 1
  relevantWheels[-1].brakeSmoother = side1BrakingSmoother
  relevantWheels[1].brakeSmoother = side2BrakingSmoother

  if relevantWheels[-1].wheels and relevantWheels[1].wheels then
    M.isActive = true
  end
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
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "isEnabled")

  local newP = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkP")
  local newI = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkI")
  local newD = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkD")

  if newP or newI or newD then
    brakingPID:setConfig(controlParameters.brakingPIDkP, controlParameters.brakingPIDkI, controlParameters.brakingPIDkD, -1, 1)
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
