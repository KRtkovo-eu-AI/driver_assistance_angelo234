-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

M.isActive = false
M.isActing = false

M.overrideMin = -1
M.overrideMax = 1

M.wheelSides = {} --used to communicate which diff output is left/right

local max = math.max
local abs = math.abs

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local configPacket = {sourceType = "activeDiffBias", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "activeDiffBias"}

local relevantDifferential = nil
local relevantWheels = {}

local inputAVSmoother = newExponentialSmoothing(50)
local outputAV1Smoother = newExponentialSmoothing(50)
local outputAV2Smoother = newExponentialSmoothing(50)

local inputAV = 0
local outputAV1Corrected = 0
local outputAV2Corrected = 0
local avDiff = 0
local avDiffBiasCoef = 0
local steeringBiasCoef = 0
local biasOffset = 0
local turningCircleRatioOutput1 = 0
local turningCircleRatioOutput2 = 0

local biasPID
local biasSmoother = newTemporalSmoothingNonLinear(10, 10)

local function resetOverride()
  M.overrideMin = -1
  M.overrideMax = 1
end

local function updateFixedStep(dt)
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

  avDiffBiasCoef = 0
  steeringBiasCoef = 0
  biasOffset = 0

  if controlParameters.isEnabled then
    local avDiffCorrected = max(abs(avDiff) - controlParameters.avDiffThreshold, 0) * sign(avDiff)
    if electrics.values.throttle > 0 then
      if abs(avDiffCorrected) == 0 then
        biasPID:reset()
      end
      avDiffBiasCoef = biasPID:get(avDiffCorrected, 0, dt)
      local gForceCoef = linearScale(abs(CMU.sensorHub.accelerationXSmooth), 7, 10, 0, 1)
      local throttleCoef = linearScale(electrics.values.throttle or 0, 0, 0.5, 0, 1)
      steeringBiasCoef = linearScale(abs(electrics.values.steering), 0.1, 0.3, 0, 1) * sign(electrics.values.steering) * gForceCoef * throttleCoef
    else
      biasPID:reset()
      biasSmoother:reset()
    end
  end

  biasOffset = linearScale(avDiffBiasCoef + steeringBiasCoef, -1, 1, -controlParameters.maxBiasOffset, controlParameters.maxBiasOffset)
  biasOffset = clamp(biasOffset, M.overrideMin, M.overrideMax)
  local biasA = 0.5 - biasOffset
  local biasB = 1 - biasA
  relevantDifferential.diffTorqueSplitA = biasA
  relevantDifferential.diffTorqueSplitB = biasB
  M.isActing = abs(biasOffset) > 0
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

  debugPacket.avDiffBiasCoef = avDiffBiasCoef
  debugPacket.steeringBiasCoef = steeringBiasCoef
  debugPacket.biasOffset = biasOffset

  debugPacket.overrideMin = M.overrideMin
  debugPacket.overrideMax = M.overrideMax

  debugPacket.avDiffThreshold = controlParameters.avDiffThreshold

  debugPacket.isActing = M.isActing

  CMU.sendDebugPacket(debugPacket)
end

local function shutdown()
  M.isActive = false
  M.isActing = false
  M.updateGFX = nil
  M.updateFixedStep = nil
end

local function reset()
  M.isActing = false
  M.overrideMin = -1
  M.overrideMax = 1
end

local function init(jbeamData)
  M.isActing = false
  controlParameters.isEnabled = true

  controlParameters.avDiffThreshold = jbeamData.avDiffThreshold or 5
  controlParameters.brakingPIDkP = jbeamData.brakingPIDkP or 0
  controlParameters.brakingPIDkI = jbeamData.brakingPIDkI or 0.1
  controlParameters.brakingPIDkD = jbeamData.brakingPIDkD or 0

  controlParameters.maxBiasOffset = jbeamData.maxBiasOffset or 0.2

  biasPID = newPIDParallel(controlParameters.brakingPIDkP, controlParameters.brakingPIDkI, controlParameters.brakingPIDkD, -1, 1, 1000, 100)

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
    log("W", "activeDiffBias.initSecondStage", "No CMU present, disabling system...")
    shutdown()
    return
  end

  local diffName = jbeamData.differentialName
  if not diffName then
    log("E", "activeDiffBias.initSecondStage", "No differentialName configured, disabling system...")
    return
  end
  relevantDifferential = powertrain.getDevice(diffName)

  if not relevantDifferential then
    log("E", "activeDiffBias.initSecondStage", string.format("Can't find configured differential (%q), disabling system...", diffName))
    return
  end

  relevantWheels = {}
  relevantWheels[-1] = {}
  relevantWheels[1] = {}
  relevantWheels[-1].wheels = powertrain.getChildWheels(relevantDifferential, 1)
  relevantWheels[1].wheels = powertrain.getChildWheels(relevantDifferential, 2)
  relevantWheels[-1].invWheelCount = (#relevantWheels[-1].wheels > 0) and (1 / #relevantWheels[-1].wheels) or 1
  relevantWheels[1].invWheelCount = (#relevantWheels[1].wheels > 0) and (1 / #relevantWheels[1].wheels) or 1

  --calculate avg wheel positions for both diff sides (a diff could have more than one wheel attached per side)
  local avgWheelPositions = {[-1] = vec3(), [1] = vec3()}
  for wheelSideIndex, wheelSide in pairs(relevantWheels) do
    for _, wheel in ipairs(wheelSide.wheels) do
      avgWheelPositions[wheelSideIndex] = avgWheelPositions[wheelSideIndex] + vec3(v.data.nodes[wheel.node1].pos)
    end
    avgWheelPositions[wheelSideIndex] = avgWheelPositions[wheelSideIndex] / #wheelSide.wheels
  end

  local vectorForward = vec3(v.data.nodes[v.data.refNodes[0].ref].pos) - vec3(v.data.nodes[v.data.refNodes[0].back].pos)
  local vectorUp = vec3(v.data.nodes[v.data.refNodes[0].up].pos) - vec3(v.data.nodes[v.data.refNodes[0].ref].pos)

  local vectorRight = vectorForward:cross(vectorUp) --vector facing to the right
  local avgDiffPosition = (avgWheelPositions[-1] + avgWheelPositions[1]) * 0.5

  for wheelSideIndex, avgWheelPosition in pairs(avgWheelPositions) do
    local wheelVector = avgWheelPosition - avgDiffPosition --create a vector from our "center" to the wheel
    local dotForward = vectorForward:dot(wheelVector) --calculate dot product of said vector and forward vector
    local dotLeft = vectorRight:dot(wheelVector) --calculate dot product of said vector and left vector
    --print(string.format("Side: %d, Name: %q, Front/Back: %.2f, LeftRight: %.2f", wheelSideIndex, relevantWheels[wheelSideIndex].wheels[1].name, dotForward, dotLeft))

    if dotForward ~= 0 then
      --wheel sides are not on the same plane front to back, probably a center diff?
    else
      if dotLeft < 0 then
        M.wheelSides.left = wheelSideIndex
      elseif dotLeft > 0 then
        M.wheelSides.right = wheelSideIndex
      else
        --no left/right info found, probably a center diff?
      end
    end
  end
  --dump(M.wheelSides)

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
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "maxBiasOffset")

  local newP = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkP")
  local newI = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkI")
  local newD = CMU.applyParameter(controlParameters, initialControlParameters, parameters, "brakingPIDkD")

  if newP or newI or newD then
    biasPID:setConfig(controlParameters.brakingPIDkP, controlParameters.brakingPIDkI, controlParameters.brakingPIDkD, -1, 1)
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
M.updateFixedStep = updateFixedStep

M.resetOverride = resetOverride

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

return M
