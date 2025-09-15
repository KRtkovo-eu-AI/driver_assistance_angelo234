-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local rpmToAV = 0.104719755

local raisedIdleAV
local controlledEngine
local relevantPump
local relevantElectrics
local electricActivationSigns
local manualIdleRaise
local raisedIdleElectricsName

local function updateGFXDynamicRaise(dt)
  local activateAutoIdleRaise = false
  local connectPump = false
  local electricActive = false

  for _, electric in ipairs(relevantElectrics) do
    if electrics.values[electric] and electricActivationSigns[sign(electrics.values[electric] or 0)] then
      activateAutoIdleRaise = true
      electricActive = true
    end
  end

  if electrics.values.wheelspeed > 2 then
    activateAutoIdleRaise = false
  end

  if activateAutoIdleRaise or manualIdleRaise then
    controlledEngine.idleAVOverwrite = raisedIdleAV
    controlledEngine.maxIdleThrottleOverwrite = 1
  else
    controlledEngine.idleAVOverwrite = 0
    controlledEngine.maxIdleThrottleOverwrite = 0
  end

  if electricActive then
    if controlledEngine.outputAV1 >= raisedIdleAV * 0.7 then
      connectPump = true
    end
  end

  if relevantPump then
    relevantPump:setConnected(connectPump)
  end

  if raisedIdleElectricsName then
    electrics.values[raisedIdleElectricsName] = activateAutoIdleRaise or manualIdleRaise
  end
end

local function setIdleRaise(enabled)
  manualIdleRaise = enabled
end

local function toggleIdleRaise()
  setIdleRaise(not manualIdleRaise)
end

local function reset(jbeamData)
  manualIdleRaise = false
end

local function init(jbeamData)
  local mode = jbeamData.mode
  if mode == "electricsRaiseAndConnect" then
    local relevantEngineName = jbeamData.controlledEngine or "mainEngine"
    controlledEngine = powertrain.getDevice(relevantEngineName)
    if not controlledEngine then
      log("E", "hydraulicsIdleRaise.init", "Can't find relevant engine with name: " .. dumps(relevantEngineName))
      return
    end

    local relevantPumpName = jbeamData.relevantPump or "pump1"
    relevantPump = powertrain.getDevice(relevantPumpName)
    if not relevantPump then
      log("D", "hydraulicsIdleRaise.init", "Can't find relevant pump with name: " .. dumps(relevantPumpName))
    end

    relevantElectrics = {}
    local relevantElectricsNames = jbeamData.relevantElectrics or {}
    if type(relevantElectricsNames) == "table" then
      for _, electricsName in pairs(relevantElectricsNames) do
        table.insert(relevantElectrics, electricsName)
      end
    elseif relevantElectricsNames then
      log("E", "hydraulicsIdleRaise.init", "Found wrong type for relevantElectrics, expected table, actual data: " .. dumps(jbeamData.relevantElectrics))
      return
    end

    local actOnPositiveElectric = jbeamData.actOnPositiveElectric == nil and true or jbeamData.actOnPositiveElectric
    local actOnNegativeElectric = jbeamData.actOnNegativeElectric == nil and true or jbeamData.actOnNegativeElectric
    electricActivationSigns = {[-1] = actOnNegativeElectric, [1] = actOnPositiveElectric, [0] = false}

    raisedIdleAV = (jbeamData.raisedIdleRPM or 1800) * rpmToAV
    raisedIdleElectricsName = jbeamData.raisedIdleElectricsName or "raisedIdle"
    manualIdleRaise = false
    M.updateGFX = updateGFXDynamicRaise
  end
end

M.init = init
M.reset = reset
M.updateGFX = nop
M.toggleIdleRaise = toggleIdleRaise
M.setIdleRaise = setIdleRaise

return M
