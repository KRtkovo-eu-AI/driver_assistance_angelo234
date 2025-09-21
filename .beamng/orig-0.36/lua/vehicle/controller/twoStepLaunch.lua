-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local engine = nil
local electricsName = nil
local hasBuiltPie = false

local rpmToAV = 0.104719755

local tempRevLimiterAV = 0
local tempRevLimiterRPM = 0
local twoStepState = "deactivated"

local function displayState()
  guihooks.message(string.format("Two-Step: %s (%d RPM)", (twoStepState ~= "deactivated" and "Active" or "Inactive"), tempRevLimiterRPM), 2, "vehicle.twoStep.status")
end

local function setTwoStep(enabled)
  if engine then
    if enabled then
      twoStepState = "idle"
    else
      twoStepState = "deactivated"
      engine:resetTempRevLimiter()
    end
    displayState()
  end
end

local function toggleTwoStep()
  if engine then
    setTwoStep(twoStepState == "deactivated")
  end
end

local function setTwoStepRPM(rpm)
  tempRevLimiterRPM = rpm
  tempRevLimiterAV = tempRevLimiterRPM * rpmToAV
  displayState()
end

local function changeTwoStepRPM(amount)
  setTwoStepRPM(tempRevLimiterRPM + amount)
end

local function updateGFX(dt)
  if twoStepState == "idle" then
    local usesKeyboard = input.state.throttle.filter == FILTER_KBD or input.state.throttle.filter == FILTER_KBD2
    local isSpeedLowEnough = usesKeyboard and (electrics.values.wheelspeed <= 2) or (electrics.values.wheelspeed <= 0.5)
    local isThrottleHighEnough = usesKeyboard and (electrics.values.throttle >= 0.2) or (electrics.values.throttle >= 0.8)
    local isBrakeHighEnough = usesKeyboard and (electrics.values.brake >= 0.2) or (electrics.values.brake >= 0.8)

    if isSpeedLowEnough and isThrottleHighEnough and isBrakeHighEnough then
      twoStepState = "armed"
    end
  elseif twoStepState == "armed" then
    engine:setTempRevLimiter(tempRevLimiterAV)

    if electrics.values.brake <= 0 or electrics.values.throttle <= 0 then
      engine:resetTempRevLimiter()
      twoStepState = "idle"
    end
  end

  electrics.values[electricsName] = twoStepState ~= "deactivated"
end

local function reset()
  if engine then
    engine:resetTempRevLimiter()
  end
  if twoStepState ~= "deactivated" then
    twoStepState = "idle"
  end
end

local function init(jbeamData)
  local engineName = jbeamData.engineName or "mainEngine"
  electricsName = jbeamData.electricsName or "twoStep"
  engine = powertrain.getDevice(engineName)
  M.updateGFX = engine and updateGFX or nop
  twoStepState = "deactivated"

  setTwoStepRPM(jbeamData.rpmLimit or 2000)

  if not hasBuiltPie then
    if engine then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          generator = function(entries)
            table.insert(entries, {title = "Two-Step", priority = 40, ["goto"] = "/root/playerVehicle/vehicleFeatures/twoStep/", icon = "raceFlag", uniqueID = "twoStep"})
          end
        }
      )

      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/twoStep",
          generator = function(entries)
            local enableEntry = {
              title = "Toggle",
              priority = 30,
              icon = "powerOnOff",
              startSlot = 0.5,
              endSlot = 1.5,
              originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "twoStep"},
              onSelect = function()
                controller.getController("twoStepLaunch").toggleTwoStep()
                return {"reload"}
              end
            }
            if twoStepState ~= "deactivated" then
              enableEntry.color = "#ff6600"
            end
            local upEntry = {
              title = "RPM Up",
              priority = 10,
              icon = "arrowLargeUp",
              startSlot = 2.5,
              endSlot = 3.5,
              originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "twoStep"},
              onSelect = function()
                controller.getController("twoStepLaunch").changeTwoStepRPM(100)
                return {"reload"}
              end
            }
            local downEntry = {
              title = "RPM Down",
              priority = 20,
              icon = "arrowLargeDown",
              startSlot = 6.5,
              endSlot = 7.5,
              originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "twoStep"},
              onSelect = function()
                controller.getController("twoStepLaunch").changeTwoStepRPM(-100)
                return {"reload"}
              end
            }
            table.insert(entries, enableEntry)
            table.insert(entries, upEntry)
            table.insert(entries, downEntry)
          end
        }
      )
    end
    hasBuiltPie = true
  end
end

local function serialize()
  return {
    state = twoStepState,
    rpm = tempRevLimiterRPM
  }
end

local function deserialize(data)
  if data and data.state and data.rpm then
    twoStepState = data.state
    setTwoStepRPM(data.rpm)
  end
end

local function setParameters(parameters)
  if parameters.isEnabled ~= nil then
    setTwoStep(parameters.isEnabled)
  end
  if parameters.launchRPM then
    setTwoStepRPM(parameters.launchRPM)
  end
end

M.init = init
M.reset = reset
M.updateGFX = nop
M.setTwoStep = setTwoStep
M.toggleTwoStep = toggleTwoStep
M.changeTwoStepRPM = changeTwoStepRPM
M.serialize = serialize
M.deserialize = deserialize

M.setParameters = setParameters

return M
