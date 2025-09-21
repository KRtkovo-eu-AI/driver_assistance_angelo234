-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local ceil = math.ceil
local min = math.min
local max = math.max

local customValueParser = require("electricsCustomValueParser")

M.values = {
  throttle = 0,
  brake = 0,
  steering = 0,
  clutch = 0,
  wheelspeed = 0,
  odometer = 0,
  avgWheelAV = 0,
  airspeed = 0,
  horn = false,
  rpmspin = 0,
  rpm = 0,
  boost = 0,
  boostMax = 0
}

M.disabledState = {}

local smoothers = {}

local rpmSmoother = newTemporalSigmoidSmoothing(50000, 75000, 50000, 75000, 0)

local lightsState = 0
local lightsSavedState = 0

local signalRightState = false
local signalLeftState = false
local signalWarnState = false
local lightbarState = 0
local hornState = false

local fogLightsState = false

local blinkPulse = false
local blinkTimerThreshold = 0.4
local blinkTimer = 0

-- sounds
local hornSound1
local hornSound2
local sirenSound
local indStartSnd
local indStopSnd
local indLoopSnd1
local indLoopSnd2
local lightOn
local lightOff
local hasSteered = false -- used to see whether right/left-turn has been finished

-- set to nop in the beginning - this avoids conflict with the warn signal
local automaticIndicatorStopHandler = nop
local generateBlinkPulseHandler = nop
local spawnVehicleIgnitionLevel
local ignitionLevelSanitization
local allowedIgnitionLevels
local allowedIgnitionLevelsLookup = {}
local previousIgnitionLevel
local updateIgnitionStarterHandler = nop
local ignitionHoldingStarterTimer = 0
local ignitionWasStartingEngine = false
local ignitionMessageLookup = {
  [0] = "ui.common.vehicleOff",
  [1] = "ui.common.vehicleAccessoryOn",
  [2] = "ui.common.vehicleOn"
}

local ignitionLevelOverrideType
local updateElectricsWithIgnitionLevelHandler = nop --used to alter the electrics output based on ignition state, made to support multiple different countries/regulations
local updateElectricsWithIgnitionLevelHandlers

local hasRegisteredQuickAccessMenu = false

local function generateBlinkPulseFun(dt)
  blinkTimer = blinkTimer + dt
  if blinkTimer > blinkTimerThreshold then
    if blinkPulse then
      indLoopSnd1 = indLoopSnd1 or sounds.createSoundscapeSound("indLoop1")
      sounds.playSoundSkipAI(indLoopSnd1)
    else
      indLoopSnd2 = indLoopSnd2 or sounds.createSoundscapeSound("indLoop2")
      sounds.playSoundSkipAI(indLoopSnd2)
    end
    blinkPulse = not blinkPulse
    blinkTimer = 0
  end
end

local function updateSignals()
  generateBlinkPulseHandler = (signalLeftState or signalRightState) and generateBlinkPulseFun or nop
end

-- stops automatically indicator if turn has been finished or if wheel is steered in opposite direction
local function manageAutomaticIndicatorStop()
  local controlPoint = 100
  local steering = M.values.steering
  if steering == nil then
    return
  end

  --check whether user has steered in the desired direction
  if signalLeftState and steering > controlPoint then
    hasSteered = true
  elseif signalRightState and steering < -controlPoint then
    hasSteered = true
  end

  --if the wheel has returned to the neutral position, turn indicator off
  if signalLeftState and hasSteered and steering <= 0 then
    signalLeftState = false
    hasSteered = false
    sounds.playSoundSkipAI(indStopSnd)
    automaticIndicatorStopHandler = nop
  elseif signalRightState and hasSteered and steering >= 0 then
    signalRightState = false
    hasSteered = false
    sounds.playSoundSkipAI(indStopSnd)
    automaticIndicatorStopHandler = nop
  end

  updateSignals()
end

local function stop_turn_signal()
  if not signalWarnState then
    if signalLeftState or signalRightState then
      signalLeftState = false
      signalRightState = false
      hasSteered = false
      sounds.playSoundSkipAI(indStopSnd)
      automaticIndicatorStopHandler = nop
    end

    updateSignals()
  end
end

-- user input functions
local function toggle_left_signal()
  if not signalWarnState then
    signalLeftState = not signalLeftState
  else
    signalLeftState = true
  end
  if signalLeftState then
    signalRightState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    automaticIndicatorStopHandler = manageAutomaticIndicatorStop
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not signalLeftState then
    sounds.playSoundSkipAI(indStopSnd)
    automaticIndicatorStopHandler = nop
    hasSteered = false
  end

  updateSignals()
end

local function set_left_signal(state, autoCancel)
  if state and not signalLeftState then
    signalLeftState = true
    signalRightState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    if autoCancel then
      automaticIndicatorStopHandler = manageAutomaticIndicatorStop
    end
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not state and signalLeftState then
    signalLeftState = false
    sounds.playSoundSkipAI(indStopSnd)
    automaticIndicatorStopHandler = nop
    hasSteered = false
  end

  updateSignals()
end

local function toggle_right_signal()
  if not signalWarnState then
    signalRightState = not signalRightState
  else
    signalRightState = true
  end
  if signalRightState then
    signalLeftState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    automaticIndicatorStopHandler = manageAutomaticIndicatorStop
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not signalRightState then
    automaticIndicatorStopHandler = nop
    sounds.playSoundSkipAI(indStopSnd)
    hasSteered = false
  end

  updateSignals()
end

local function set_right_signal(state, autoCancel)
  if state and not signalRightState then
    signalRightState = true
    signalLeftState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    if autoCancel then
      automaticIndicatorStopHandler = manageAutomaticIndicatorStop
    end
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not state and signalRightState then
    signalRightState = false
    sounds.playSoundSkipAI(indStopSnd)
    automaticIndicatorStopHandler = nop
    hasSteered = false
  end

  updateSignals()
end

local function toggleSound(val, snd)
  if not snd then
    return
  end
  if val then
    obj:setVolume(snd, 1)
    obj:playSFX(snd)
  else
    obj:stopSFX(snd)
  end
end

local function sanitizeIgnitionLevel(desiredIgnitionLevel)
  --return the _actual_ ignition level for a desired one so that certain levels can be "forbidden"
  return ignitionLevelSanitization[desiredIgnitionLevel] or 0
end

local function updateElectricsWithIgnitionLevelEuropean()
  local values = M.values
  --adjustments in "off" mode
  values.brakelights = 0
  values.parkingbrakelight = 0
  values.brakelight_signal_R = 0
  values.brakelight_signal_L = 0

  values.parking = 0
  values.reverse = 0

  values.lights = 0
  values.lights_state = 0

  values.turnsignal = 0
  values.signal_L = 0
  values.signal_R = 0
  values.hazard = 0
  values.hazard_enabled = 0
  values.lightbar = 0
  lightbarState = 0
  values.lowpressure = 0
  values.oil = 0
  values.lowhighbeam = 0
  values.lowbeam = 0
  values.highbeam = 0
  values.fog = 0
  values.horn = 0
  values.lowhighbeam_signal_R = 0
  values.lowhighbeam_signal_L = 0
  values.highbeam_wigwag_R = 0
  values.highbeam_wigwag_L = 0
  values.reverse_wigwag_R = 0
  values.reverse_wigwag_L = 0
  values.signal_right_input = 0
  values.signal_left_input = 0

  signalLeftState = false
  signalRightState = false
  signalWarnState = false

  automaticIndicatorStopHandler = nop
  generateBlinkPulseHandler = nop

  values.fuel = 0
  values.lowfuel = 0

  values.abs = 0
  values.esc = 0
  values.tcs = 0

  values.oiltemp = 0
  values.watertemp = 0
  values.checkengine = 0
  values.ignition = 0
  values.running = 0
  values.engineRunning = 0
end

-- use to update the handler method that overwrites electrics values based on ignition state
local function updateIgnitionLevelElectricsOverrideHandler()
  updateElectricsWithIgnitionLevelHandler = M.values.ignitionLevel == 0 and updateElectricsWithIgnitionLevelHandlers[ignitionLevelOverrideType] or nop
end

local function setIgnitionLevel(ignitionLevel)
  --print("Ignition Level: " .. ignitionLevel)
  previousIgnitionLevel = M.values.ignitionLevel
  local newIgnitionLevel = sanitizeIgnitionLevel(ignitionLevel)
  M.values.ignitionLevel = newIgnitionLevel

  if ignitionMessageLookup[newIgnitionLevel] then
    guihooks.message({txt = ignitionMessageLookup[newIgnitionLevel], context = {}}, 4, "vehicle.ignition.ignitionLevel")
  end

  if newIgnitionLevel == 0 or newIgnitionLevel == 1 then
    controller.mainController.setEngineIgnition(false)
    controller.mainController.setStarter(false)
  elseif newIgnitionLevel == 2 then
    controller.mainController.setEngineIgnition(true)
    controller.mainController.setStarter(false)
  elseif newIgnitionLevel == 3 then
    controller.mainController.setEngineIgnition(true)
    controller.mainController.setStarter(true)
  end

  updateIgnitionLevelElectricsOverrideHandler()
end

local function updateIgnitionStarter(dt)
  ignitionHoldingStarterTimer = ignitionHoldingStarterTimer + dt
  if ignitionHoldingStarterTimer > 0.5 then
    if not ignitionWasStartingEngine then
      ignitionWasStartingEngine = true
      M.setIgnitionLevel(3)
    end
  else
    ignitionWasStartingEngine = false
  end
end

local function updateGFX(dt)
  updateIgnitionStarterHandler(dt)
  generateBlinkPulseHandler(dt)

  local vals = M.values
  -- the primary source values

  automaticIndicatorStopHandler()

  vals.accXSmooth = sensors.gx2
  vals.accYSmooth = sensors.gy2
  vals.accZSmooth = sensors.gz2

  vals.odometer = partCondition.getRootPartOdometerValue()
  vals.trip = partCondition.getRootPartTripValue()

  vals.brakelights = nil
  vals.nop = 0 --nop electrics for not yet working things
  vals.parkingbrake = vals.parkingbrake_input
  vals.parkingbrakelight = vals.parkingbrake > 0
  vals.lights = lightsState
  vals.lights_state = lightsState
  if signalWarnState then
    vals.turnsignal = 0
  elseif signalRightState then
    vals.turnsignal = 1
  elseif signalLeftState then
    vals.turnsignal = -1
  else
    vals.turnsignal = 0
  end

  vals.airspeed = obj:getGroundSpeed()
  vals.airflowspeed = obj:getAirflowSpeed()
  vals.altitude = obj:getAltitude()
  vals.parking = 0 -- TODO: input.parkinglights
  vals.reverse = (vals.gearIndex or 0) < 0

  -- and then the derived values
  vals.signal_L = vals.signal_left_input == 1 and blinkPulse
  vals.signal_R = vals.signal_right_input == 1 and blinkPulse

  vals.hazard = (signalWarnState and blinkPulse)
  vals.hazard_enabled = signalWarnState
  vals.lightbar = lightbarState
  vals.lowpressure = (beamstate.lowpressure)
  vals.oil = (vals.oiltemp or 0) >= 130
  vals.lowhighbeam = (lightsState == 1 or lightsState == 2)
  vals.lowbeam = (lightsState == 1)
  vals.highbeam = (lightsState == 2)
  vals.fog = fogLightsState
  vals.horn = hornState

  --mixed values for american style indicators/lights
  vals.lowhighbeam_signal_R = vals.signal_right_input == 1 and (blinkPulse and 1 or 0) or ceil(vals.lowhighbeam and 1 or 0)
  vals.lowhighbeam_signal_L = vals.signal_left_input == 1 and (blinkPulse and 1 or 0) or ceil(vals.lowhighbeam and 1 or 0)
  --wigwag lights
  --desired behavior: highbeam is controlled by normal highbeam values if lightbar is OFF
  --if it's on, only the wigwag signal has control over the highbeam
  local lightbarActive = vals.lightbar > 0
  local wigwagRActive = vals.wigwag_R == 1
  local wigwagLActive = vals.wigwag_L == 1
  local highbeamActive = vals.highbeam
  vals.highbeam_wigwag_R = ((highbeamActive and not lightbarActive) or (wigwagRActive)) and 1 or 0
  vals.highbeam_wigwag_L = ((highbeamActive and not lightbarActive) or (wigwagLActive)) and 1 or 0
  vals.reverse_wigwag_R = vals.wigwag_R == 1 or ceil(vals.reverse and 1 or 0)
  vals.reverse_wigwag_L = vals.wigwag_L == 1 or ceil(vals.reverse and 1 or 0)

  local rpm = vals.rpm
  vals.rpmTacho = rpmSmoother:get(rpm, dt)
  vals.rpmspin = (vals.rpmspin + dt * rpm * 6) % 360 --make sure to convert properly between the units here

  vals.signal_right_input = (signalRightState)
  vals.signal_left_input = (signalLeftState)

  vals.boost = (vals.turboBoost or 0) + (vals.superchargerBoost or 0)
  vals.boostMax = max((vals.turboBoostMax or 0), (vals.superchargerBoostMax or 0))

  -- inject imported electrics events first time, this needs to happen twice overall so that code between gfx first step and gfx second step can see these updated electrics
  beamstate.updateRemoteElectrics(true)

  for f, v in pairs(vals) do
    if M.disabledState[f] ~= nil then
      vals[f] = nil
    else
      if type(v) == "boolean" then
        vals[f] = vals[f] and 1 or 0
      end
    end
  end
end

local function updateGFXSecondStep(dt)
  local values = M.values
  --make sure to update the brakelights value based on the brake value
  values.brakelights = values.brakelights or ceil(values.brake)
  values.brakelight_signal_R = values.signal_right_input == 1 and (blinkPulse and 1 or 0) or ceil(values.brakelights or values.brake or 0)
  values.brakelight_signal_L = values.signal_left_input == 1 and (blinkPulse and 1 or 0) or ceil(values.brakelights or values.brake or 0)

  -- inject imported electrics events second time, this should be the last step before update electrics by ignition state so that we can override everything if needed
  beamstate.updateRemoteElectrics(false)

  updateElectricsWithIgnitionLevelHandler()

  customValueParser.updateGFX(dt)

  --apply the smoothers as the last thing
  for f, s in pairs(smoothers) do
    if values[f] ~= nil then
      values[f] = s:get(values[f], dt)
    end
  end
end

local function registerQuickAccessMenu()
  if not core_quickAccess or hasRegisteredQuickAccessMenu then
    return
  end
  hasRegisteredQuickAccessMenu = true

  if #allowedIgnitionLevels > 1 then
    -- headlights
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/",
        generator = function(entries)
          local e = {title = "ui.radialmenu2.electrics.headlights", ["goto"] = "/root/playerVehicle/lights/headlights/", icon = "lowBeam", action = "toggle_headlights", uniqueID = "headlights"}
          if electrics.values.lights_state == 1 then
            e.color = "#33ff33"
            e.icon = "lowBeam"
          end
          if electrics.values.lights_state == 2 then
            e.color = "#3333ff"
            e.icon = "highBeam"
          end
          table.insert(entries, e)
        end
      }
    )

    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/headlights/",
        uniqueID = "headlights",
        generator = function(entries)
          local e = {
            title = "ui.radialmenu2.electrics.headlights.off",
            icon = "lowBeam",
            originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "headlights"},
            uniqueID = "headlights_off",
            onSelect = function()
              M.setLightsState(0)
              return {"hide"}
            end
          }
          if M.values.lights_state == 0 then
            e.color = "#ff6600"
          end
          table.insert(entries, e)

          e = {
            title = "ui.radialmenu2.electrics.headlights.low",
            icon = "lowBeam",
            originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "headlights"},
            uniqueID = "headlights_low",
            onSelect = function()
              M.setLightsState(1)
              return {"hide"}
            end
          }
          if M.values.lights_state == 1 then
            e.color = "#33ff33"
          end
          table.insert(entries, e)

          e = {
            title = "ui.radialmenu2.electrics.headlights.high",
            icon = "highBeam",
            originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "headlights"},
            uniqueID = "headlights_high",
            onSelect = function()
              M.setLightsState(2)
              return {"hide"}
            end
          }
          if M.values.lights_state == 2 then
            e.color = "#3333ff"
          end
          table.insert(entries, e)
        end
      }
    )

    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/",
        generator = function(entries)
          -- warning lights
          local e = {
            title = "ui.radialmenu2.electrics.hazard_lights",
            icon = "hazardLights",
            action = "toggle_hazard_signal",
            uniqueID = "hazard_lights",
            onSelect = function()
              M.toggle_warn_signal()
              return {"hide"}
            end
          }
          if M.values.hazard_enabled == 1 then
            e.color = "#ff0000"
          end
          table.insert(entries, e)

          -- fog lights
          e = {
            title = "ui.radialmenu2.electrics.fog_lights",
            icon = "fogLight",
            action = "toggle_foglights",
            uniqueID = "fog_lights",
            onSelect = function()
              M.toggle_fog_lights()
              return {"hide"}
            end
          }
          if M.values.fog == 1 then
            e.color = "#ff6600"
          end
          table.insert(entries, e)

          -- lightbar
          e = {
            title = "ui.radialmenu2.electrics.lightbar",
            icon = "wigwags",
            action = "toggle_lightbar_signal",
            uniqueID = "lightbar",
            onSelect = function()
              M.toggle_lightbar_signal()
              return {"hide"}
            end
          }
          if M.values.lightbar == 1 then
            e.color = "#ff6600"
          end
          if M.values.lightbar == 2 then
            e.color = "#ff0000"
          end
          table.insert(entries, e)

          -- signals
          e = {title = "ui.radialmenu2.electrics.signals", icon = "twoArrowsHorizontal", ["goto"] = "/root/playerVehicle/lights/signals/", uniqueID = "signals"}
          if M.values.hazard_enabled == 0 and (M.values.signal_left_input == 1 or M.values.signal_right_input == 1) then
            e.color = "#33ff33"
          end
          table.insert(entries, e)
        end
      }
    )

    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/signals/",
        generator = function(entries)
          local e = {
            startSlot = 0.5,
            endSlot = 1.5,
            title = "ui.radialmenu2.electrics.signals.left",
            priority = 2,
            icon = "arrowSolidLeft",
            action = "toggle_left_signal",
            uniqueID = "toggle_left_signal",
            originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "signals"},
            ignoreAsRecentActionForCategory = "playerVehicle",
            onSelect = function()
              M.toggle_left_signal()
              return {"hide"}
            end
          }
          if M.values.hazard_enabled == 0 and M.values.signal_left_input == 1 then
            e.color = "#33ff33"
          end
          table.insert(entries, e)

          e = {
            startSlot = 4.5,
            endSlot = 5.5,
            title = "ui.radialmenu2.electrics.signals.right",
            priority = 1,
            icon = "arrowSolidRight",
            action = "toggle_right_signal",
            uniqueID = "toggle_right_signal",
            originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "signals"},
            ignoreAsRecentActionForCategory = "playerVehicle",
            onSelect = function()
              M.toggle_right_signal()
              return {"hide"}
            end
          }
          if M.values.hazard_enabled == 0 and M.values.signal_right_input == 1 then
            e.color = "#33ff33"
          end
          table.insert(entries, e)
        end
      }
    )

    -- ignition level 0-2
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/vehicleFeatures/",
        generator = function(entries)
          local currentIgnitionLevel = M.values.ignitionLevel
          local e = {
            title = "ui.radialmenu2.powertrain.toggleIgnitionLevel",
            desc = {txt = "ui.radialmenu2.currentValue", context = {value = ignitionMessageLookup[currentIgnitionLevel]}},
            ignoreAsRecentActionForCategory = "playerVehicle",
            action = "",
            icon = "keys2",
            uniqueID = "toggleIgnitionLevel",
            onSelect = function()
              --this toggles through 0 to 2 and skips 3 because there is not time between onDown and onUp
              --TODO implement "hold" functionality like the normal input system
              M.toggleIgnitionLevelOnDown()
              M.toggleIgnitionLevelOnUp()
            end
          }
          table.insert(entries, e)
        end
      }
    )
  end

  --check if level 3 is allowed
  if allowedIgnitionLevelsLookup[3] then
    --starter button (ignition level 3)
    --this is a bandaid for the missing "hold" functionality of the radial menu
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/vehicleFeatures/",
        generator = function(entries)
          local e = {
            title = "ui.radialmenu2.powertrain.engine",
            desc = "ui.radialmenu2.powertrain.engine.desc",
            action = "activateStarterMotor",
            icon = "keys2",
            uniqueID = "activateStarterMotor",
            onSelect = function()
              M.setIgnitionLevel(3)
            end
          }
          table.insert(entries, e)
        end
      }
    )
  end
end

local function setAllowedIgnitionLevels(newAllowedIgnitionLevels)
  if #newAllowedIgnitionLevels <= 0 then --make sure that we always have at least one allowed ignition level
    newAllowedIgnitionLevels = {0}
  --print warning
  end
  allowedIgnitionLevels = newAllowedIgnitionLevels
  allowedIgnitionLevelsLookup = {}
  for _, level in ipairs(newAllowedIgnitionLevels) do
    allowedIgnitionLevelsLookup[level] = true
  end

  ignitionLevelSanitization = {[0] = 0, [1] = 1, [2] = 2, [3] = 3}

  for i = 0, 3, 1 do --for every possible ignition level
    if not allowedIgnitionLevelsLookup[i] then --check if it's currently allowed
      for j = min(i + 1, 3), 3, 1 do --if not, first try to find anything _higher_ that is allowed
        if allowedIgnitionLevelsLookup[j] then
          ignitionLevelSanitization[i] = j
          break
        end
      end

      if not allowedIgnitionLevelsLookup[i] then --if we couldn't find anything higher allowed
        for j = max(i - 1, 0), 0, -1 do --try to find anything lower that is allowed...
          if allowedIgnitionLevelsLookup[j] then
            ignitionLevelSanitization[i] = j
            break
          end
        end
      end
    end
  end
end

local function reset()
  M.disabledState = {}

  for _, s in pairs(smoothers) do
    s:reset()
  end

  M.values.throttle = 0
  M.values.brake = 0
  M.values.steering = 0
  M.values.clutch = 0
  M.values.wheelspeed = 0
  M.values.odometer = 0
  M.values.avgWheelAV = 0
  M.values.airspeed = 0
  M.values.airflowspeed = 0
  M.values.horn = false
  M.values.boost = 0
  M.values.boostMax = 0
  M.values.electricalLoadCoef = 1

  customValueParser.resetCustomValues()

  --lightbarState = 0
  lightsSavedState = 0

  toggleSound(lightbarState == 2, sirenSound)

  updateIgnitionStarterHandler = nop
  ignitionHoldingStarterTimer = 0
  ignitionWasStartingEngine = false

  --set these without the setter because it has additional runtime logic that is not desired upon reset
  M.values.ignitionLevel = spawnVehicleIgnitionLevel
  previousIgnitionLevel = spawnVehicleIgnitionLevel - 1
  updateIgnitionLevelElectricsOverrideHandler()
end

--used for creating smoother from jbeam settings
local function newSmoother(smootherType, params)
  if smootherType == "exponential" then
    return newExponentialSmoothing(unpack(params))
  elseif smootherType == "temporal" then
    return newTemporalSmoothing(unpack(params))
  elseif smootherType == "temporalNonLinear" then
    return newTemporalSmoothingNonLinear(unpack(params))
  end
end

local function init()
  M.disabledState = {}

  M.values.throttle = 0
  M.values.brake = 0
  M.values.steering = 0
  M.values.clutch = 0
  M.values.wheelspeed = 0
  M.values.odometer = 0
  M.values.avgWheelAV = 0
  M.values.airspeed = 0
  M.values.airflowspeed = 0
  M.values.horn = false
  M.values.boost = 0
  M.values.boostMax = 0
  M.values.electricalLoadCoef = 1

  --lightbarState = 0
  lightsSavedState = 0

  toggleSound(lightbarState == 2, sirenSound)

  --look at both the regular electrics data as well as the components version for deep merging
  local jbeamData = tableMergeRecursiveArray(v.data.electrics or {}, v.data.components.electrics or {})

  --default smoothers for backwards compat (we've always used them)
  local defaultSmoothersSettings = {
    {electricsName = "wheelspeed", smootherType = "exponential", params = {10}},
    {electricsName = "gear_A", smootherType = "exponential", params = {10}},
    {electricsName = "rpm", smootherType = "exponential", params = {10}},
    {electricsName = "lights", smootherType = "exponential", params = {10}},
    {electricsName = "fuel", smootherType = "exponential", params = {100}},
    {electricsName = "oiltemp", smootherType = "exponential", params = {100}},
    {electricsName = "watertemp", smootherType = "exponential", params = {100}},
    {electricsName = "turnsignal", smootherType = "exponential", params = {10}},
    {electricsName = "airspeed", smootherType = "exponential", params = {10}},
    {electricsName = "airflowspeed", smootherType = "exponential", params = {10}},
    {electricsName = "altitude", smootherType = "exponential", params = {10}}
  }

  smoothers = {}

  --load jbeam smoother data
  local jbeamSmootherSettings = jbeamData.smoothers or {}
  --convert data into a usable table
  jbeamSmootherSettings = tableFromHeaderTable(jbeamSmootherSettings)
  --merge default and jbeam data for final settings
  local smootherSettings = tableMergeRecursiveArray(defaultSmoothersSettings, jbeamSmootherSettings)
  --dump(smootherSettings)

  --iterate over all desired smoothers and create them with the correct settings
  for _, smootherSetting in ipairs(smootherSettings) do
    smoothers[smootherSetting.electricsName] = newSmoother(smootherSetting.smootherType, smootherSetting.params)
  end

  local defaultCustomValues = {}

  --this can come from components which do not offer proper table merging and might therefore contain multiple table headers
  --this is dealt with in the customValueParser by ignoring the table headers
  local jbeamCustomValues = jbeamData.customValues or {}
  jbeamCustomValues = tableFromHeaderTable(jbeamCustomValues)
  --merge default and jbeam data for final settings
  local customValues = {}
  local customValueNames = {}
  for _, value in ipairs(jbeamCustomValues) do
    if not customValueNames[value.electricsName] then
      customValueNames[value.electricsName] = true
      table.insert(customValues, value)
    end
  end

  for _, value in ipairs(defaultCustomValues) do
    if not customValueNames[value.electricsName] then
      table.insert(customValues, value)
    end
  end

  --dump(customValues)

  customValueParser.compileCustomValueUpdates(customValues)

  --set all smoothers to the starting value of their respective electrics value
  for electricsName, smoother in pairs(smoothers) do
    if M.values[electricsName] ~= nil then
      smoother:set(M.values[electricsName])
    end
  end

  --build the table of supported electrics override schemes
  updateElectricsWithIgnitionLevelHandlers = {
    european = updateElectricsWithIgnitionLevelEuropean
  }
  ignitionLevelOverrideType = jbeamData.ignitionLevelOverrideType or "european"

  setAllowedIgnitionLevels(jbeamData.allowedIgnitionLevels or {0, 1, 2, 3}) --read allowed ignition levels from jbeam or use all of them by default

  updateIgnitionStarterHandler = nop
  ignitionHoldingStarterTimer = 0
  ignitionWasStartingEngine = false

  spawnVehicleIgnitionLevel = settings.getValue("spawnVehicleIgnitionLevel") or 3
  if v.config.additionalVehicleData and v.config.additionalVehicleData.spawnWithEngineRunning ~= nil then
    spawnVehicleIgnitionLevel = v.config.additionalVehicleData.spawnWithEngineRunning and 3 or 0
  end
  spawnVehicleIgnitionLevel = sanitizeIgnitionLevel(spawnVehicleIgnitionLevel)

  --these are set without the setter intentionally because the setter has added runtime logic that is not desired here
  M.values.ignitionLevel = spawnVehicleIgnitionLevel
  previousIgnitionLevel = spawnVehicleIgnitionLevel - 1

  registerQuickAccessMenu()

  updateIgnitionLevelElectricsOverrideHandler()
end

local function initLastStage()
  if M.values.ignitionLevel == 3 then
    setIgnitionLevel(2)
    previousIgnitionLevel = 3
  end
end

local function set_warn_signal(value)
  signalWarnState = value
  signalRightState = signalWarnState
  signalLeftState = signalWarnState
  automaticIndicatorStopHandler = nop
  updateSignals()
end

local function toggle_warn_signal()
  set_warn_signal(not signalWarnState)
end

local function toggle_lights()
  lightsState = lightsState + 1
  if lightsState == 1 then
    lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
    sounds.playSoundSkipAI(lightOn)
  elseif lightsState == 2 then
    lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
    sounds.playSoundSkipAI(lightOn)
  elseif lightsState == 3 then
    lightsState = 0
    lightOff = lightOff or sounds.createSoundscapeSound("LightOff")
    sounds.playSoundSkipAI(lightOff)
  end
end

local function toggle_highbeams()
  --no function if state is 0/off
  if lightsState == 1 then
    lightsState = 2
    lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
    sounds.playSoundSkipAI(lightOn)
  elseif lightsState == 2 then
    lightsState = 1
    lightOff = lightOff or sounds.createSoundscapeSound("LightOff")
    sounds.playSoundSkipAI(lightOff)
  end
end

local function light_flash_highbeams(enabled)
  if enabled then
    lightsSavedState = lightsState
    lightsState = 2
  else
    lightsState = lightsSavedState
  end
end

local function set_lightbar_signal(state)
  if state == lightbarState then
    return
  end

  sirenSound = sirenSound or sounds.createSoundscapeSound("siren")
  lightbarState = state % (sirenSound ~= nil and 3 or 2)

  -- 1 = lights, no sound
  -- 2 = lights + sound
  toggleSound(lightbarState == 2, sirenSound)
end

local function toggle_lightbar_signal()
  set_lightbar_signal(lightbarState + 1)
end

local function setLightsState(newval)
  lightsState = newval
end

local function toggle_fog_lights()
  fogLightsState = not fogLightsState
end

local function set_fog_lights(state)
  fogLightsState = state
end

local function horn(state)
  hornState = state
  if hornState then
    hornSound1 = hornSound1 or sounds.createSoundscapeSound("horn")
    if not hornSound1 then
      return
    end
    if hornSound2 then
      obj:setVolume(hornSound2, 0)
    end
    obj:setVolume(hornSound1, 1)
    obj:playSFX(hornSound1)
  else
    if hornSound1 then
      obj:stopSFX(hornSound1)
    end
    hornSound1, hornSound2 = hornSound2, hornSound1
  end
end

local function toggleIgnitionLevelOnDown()
  updateIgnitionStarterHandler = updateIgnitionStarter
end

local function toggleIgnitionLevelOnUp()
  if ignitionWasStartingEngine and M.values.ignitionLevel < 3 then
    --this case happens when the vehicle controller already switched to level 2 by itself after the engine fired up
    --make sure to pretend that we are still in level 3 so that the following logic works. Not nice, but couldn't find a better solution that actually works
    --this logic is also needed so that we can set level 3 from external (eg GE) and have the engine start autoamtically and go back to level 2 after doing so
    --(vehicle controller switches to level 2 after engine fired up)
    setIgnitionLevel(3)
  end
  local currentIgnitionLevel = M.values.ignitionLevel
  updateIgnitionStarterHandler = nop
  ignitionHoldingStarterTimer = 0
  ignitionWasStartingEngine = false

  if currentIgnitionLevel == 0 then
    setIgnitionLevel(1)
  elseif currentIgnitionLevel == 1 then
    if previousIgnitionLevel < currentIgnitionLevel then
      setIgnitionLevel(2)
    else
      setIgnitionLevel(0)
    end
  elseif currentIgnitionLevel == 2 then
    setIgnitionLevel(1)
  elseif currentIgnitionLevel == 3 then
    setIgnitionLevel(2)
  end
end

-- public interface
M.updateGFX = updateGFX
M.updateGFXSecondStep = updateGFXSecondStep
M.toggle_left_signal = toggle_left_signal
M.toggle_right_signal = toggle_right_signal
M.set_left_signal = set_left_signal
M.set_right_signal = set_right_signal
M.stop_turn_signal = stop_turn_signal
M.toggle_warn_signal = toggle_warn_signal
M.set_warn_signal = set_warn_signal
M.toggle_lightbar_signal = toggle_lightbar_signal
M.set_lightbar_signal = set_lightbar_signal
M.toggle_fog_lights = toggle_fog_lights
M.set_fog_lights = set_fog_lights
M.toggle_lights = toggle_lights
M.toggle_highbeams = toggle_highbeams
M.light_flash_highbeams = light_flash_highbeams
M.setLightsState = setLightsState
M.horn = horn
M.setIgnitionLevel = setIgnitionLevel
M.toggleIgnitionLevelOnDown = toggleIgnitionLevelOnDown
M.toggleIgnitionLevelOnUp = toggleIgnitionLevelOnUp
M.setAllowedIgnitionLevels = setAllowedIgnitionLevels
M.resetLastStage = initLastStage
M.reset = reset
M.init = init
M.initLastStage = initLastStage
return M
