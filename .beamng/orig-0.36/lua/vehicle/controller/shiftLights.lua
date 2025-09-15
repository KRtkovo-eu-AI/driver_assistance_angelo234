-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local max = math.max

local lightSteps = nil
local flashingLEDsOn = nil
local flashingLEDsOff = nil
local minRPM = 0
local maxRPM = 0
local inputElectricsName = nil
local flashDuration = 0
local flashTimer = 0
local flashSwitch = false
local keepLowerLEDsOn = true
local stallWarningEngine = nil
local hasStallWarning = true

local function updateGFX(dt)
  local rpm = electrics.values[inputElectricsName]
  for _, v in ipairs(lightSteps) do
    electrics.values[v.electricsName] = 0
  end
  local isOverMaxRPM = rpm >= maxRPM

  for _, v in ipairs(lightSteps) do
    if rpm >= v.startRPM and (rpm < v.endRPM or keepLowerLEDsOn) and not isOverMaxRPM then
      electrics.values[v.electricsName] = 1
    end
  end

  electrics.values.shouldShift = isOverMaxRPM

  flashTimer = max(flashTimer - dt, 0)
  if isOverMaxRPM or (hasStallWarning and stallWarningEngine.isStalled) then
    if flashTimer <= 0 then
      flashSwitch = not flashSwitch
      flashTimer = flashDuration
    end

    local flashValue = flashSwitch and 1 or 0
    for _, v in ipairs(flashingLEDsOn) do
      electrics.values[v] = flashValue
    end
    for _, v in ipairs(flashingLEDsOff) do
      electrics.values[v] = 1 - flashValue
    end
  else
    flashSwitch = false
  end
end

local function init(jbeamData)
  flashTimer = 0
  flashSwitch = false
  lightSteps = {}

  maxRPM = 7000
  local rpmRange = jbeamData.rpmRange or 3000
  hasStallWarning = false

  if jbeamData.engineName and jbeamData.maxEngineRPMOffset then
    local engine = powertrain.getDevice(jbeamData.engineName)
    if engine then
      maxRPM = engine.maxRPM - jbeamData.maxEngineRPMOffset
      stallWarningEngine = engine
      hasStallWarning = jbeamData.hasStallWarning or false
    end
  end

  maxRPM = jbeamData.maxRPM or maxRPM
  minRPM = maxRPM - rpmRange
  minRPM = jbeamData.minRPM or minRPM

  inputElectricsName = jbeamData.inputElectricsName or "rpm"
  keepLowerLEDsOn = jbeamData.keepLEDsActive == nil and true or jbeamData.keepLEDsActive
  flashDuration = jbeamData.flashDuration or 0.1

  local steps = jbeamData.outputElectrics or {}
  local stepCount = #steps
  local rpmStepRange = maxRPM - minRPM
  local rpmStep = rpmStepRange / stepCount
  for i = 1, stepCount do
    local step = steps[i]
    local stepData = {startRPM = minRPM + rpmStep * (i - 1), endRPM = minRPM + rpmStep * i, electricsName = step}
    table.insert(lightSteps, stepData)
  end
  flashingLEDsOn = {}
  flashingLEDsOff = {}
  for _, v in pairs(jbeamData.flashingOutputElectrics or {}) do
    table.insert(flashingLEDsOn, v)
  end
  for _, v in pairs(jbeamData.flashingAlternateOutputElectrics or {}) do
    table.insert(flashingLEDsOff, v)
  end

  --dump(lightSteps)
end

M.init = init
M.updateGFX = updateGFX

return M
