-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local max = math.max

local rpmToAV = 0.10471971768
local avToRPM = 9.5493

local states = {off = "off", idle = "idle", armed = "armed", manualOverride = "manualOverride", timeout = "timeout"}
local state

local controlledEngine
local controlledTurbocharger
local turboTargetAV
local minActiveEngineAV
local minActiveWheelSpeed
local maxActiveThrottleInput
local minActiveBrakeInput

local timeoutTime
local timeoutTimer
local timeoutResetTimer
local timeoutResetTime

local autoArmMinAV
local autoArmMinThrottle
local autoArmTime
local autoArmTimer

local autoDisarmMaxAV
local autoDisarmMaxThrottle
local autoDisarmTime
local autoDisarmTimer

local turboAVPIDController

local function setAntilagState(newState)
  state = newState
end

local function updateGFX(dt)
  local throttle = electrics.values.throttle
  local engineAV = controlledEngine.outputAV1
  local antilagCoef = 0

  -- IDLE STATE: System is ready but waiting for activation conditions
  if state == states.idle then
    -- ARMED STATE: System is active and can engage antilag
    -- Check if throttle and engine speed are high enough to arm the system
    if throttle >= autoArmMinThrottle and engineAV >= autoArmMinAV then
      autoArmTimer = autoArmTimer + dt
      if autoArmTimer >= autoArmTime then
        state = states.armed
        autoArmTimer = 0
      end
    else
      autoArmTimer = 0
    end
  elseif state == states.armed then
    -- Check all conditions required for antilag operation
    local wheelSpeedHighEnough = electrics.values.wheelspeed >= minActiveWheelSpeed
    local throttleLowEnough = electrics.values.throttle <= maxActiveThrottleInput
    local brakeHighEnough = electrics.values.brake >= minActiveBrakeInput
    local engineAVHighEnough = controlledEngine.outputAV1 >= minActiveEngineAV

    -- If all conditions are met, engage antilag
    if wheelSpeedHighEnough and (throttleLowEnough or brakeHighEnough) and engineAVHighEnough then
      -- Calculate how much antilag is needed based on turbo speed difference
      local currentTurboAV = controlledTurbocharger.turboAV

      antilagCoef = turboAVPIDController:get(currentTurboAV, turboTargetAV, dt)
      --print("antilagCoef: " .. antilagCoef)

      -- Monitor continuous usage time to prevent overheating
      timeoutTimer = timeoutTimer + dt
      if timeoutTimer > timeoutTime then
        state = states.timeout
        timeoutResetTimer = timeoutResetTime
      end
    else
      -- Gradually reduce timeout timer when conditions aren't met
      timeoutTimer = max(timeoutTimer - dt * 3, 0)
    end

    -- Check if conditions for auto-disarm are met
    if antilagCoef <= 0 and throttle < autoDisarmMaxThrottle and engineAV < autoDisarmMaxAV then
      autoDisarmTimer = autoDisarmTimer + dt
      if autoDisarmTimer >= autoDisarmTime then
        state = states.idle
        autoDisarmTimer = 0
      end
    else
      autoDisarmTimer = 0
    end
  elseif state == states.manualOverride then
    -- TIMEOUT STATE: System is cooling down
    antilagCoef = 1
  elseif state == states.timeout then
    antilagCoef = 0
    timeoutResetTimer = timeoutResetTimer - dt
    if timeoutResetTimer <= 0 then
      state = states.idle
      timeoutTimer = 0
      timeoutTimer = 0
    end
  end

  electrics.values.alsActive = antilagCoef > 0
  electrics.values.alsState = state

  -- Apply the calculated antilag coefficient to the engine
  controlledEngine:setAntilagCoef(antilagCoef)
end

local function reset(jbeamData)
  timeoutTimer = 0
  state = states.idle
  turboAVPIDController:reset()
  electrics.values.alsActive = false
  electrics.values.alsState = state
end

local function init(jbeamData)
  local engineName = jbeamData.controlledEngine or "mainEngine"
  controlledEngine = powertrain.getDevice(engineName)
  if not controlledEngine then
    log("E", "anitlag.init", string.format("Can't find requested engine with name: %q, antilag won't work!", engineName))
    M.updateGFX = nop
    return
  end

  if not controlledEngine.turbocharger then
    log("E", "anitlag.init", string.format("Engine %q does not have a turbocharger, antilag won't work!", engineName))
    M.updateGFX = nop
    return
  end

  -- Target turbo speed [RPM]
  turboTargetAV = (jbeamData.turboTargetRPM or 90000) * rpmToAV -- Desired turbo speed to maintain during antilag

  -- Minimum engine speed for activation [RPM]
  minActiveEngineAV = (jbeamData.minActiveEngineRPM or 2000) * rpmToAV -- Engine must be above this speed for antilag
  minActiveWheelSpeed = jbeamData.minActiveWheelSpeed or 1 -- Vehicle must be moving faster than this [m/s]
  maxActiveThrottleInput = jbeamData.maxActiveThrottleInput or 0.2 -- Antilag only works below this throttle [0-1]
  minActiveBrakeInput = jbeamData.minActiveBrakeInput or 0.2 -- Brake must be pressed harder than this [0-1]

  controlledTurbocharger = controlledEngine.turbocharger

  timeoutTime = jbeamData.timeoutTime or 30 --Maximum continuous runtime [s], prevents overheating by limiting active duration
  timeoutResetTime = jbeamData.timeoutResetTime or 2 -- Cooldown period after timeout [s]
  timeoutTimer = 0
  timeoutResetTimer = 0

  -- Auto-arm parameters
  autoArmMinAV = (jbeamData.autoArmMinRPM or 5000) * rpmToAV -- Engine speed needed to arm antilag
  autoArmMinThrottle = jbeamData.autoArmMinThrottle or 0.5 -- Heavy throttle needed to arm antilag
  autoArmTime = jbeamData.autoArmTime or 1 -- Time conditions must be met to arm [s]
  autoArmTimer = 0

  -- Auto-disarm parameters
  autoDisarmMaxAV = (jbeamData.autoDisarmMaxRPM or 3000) * rpmToAV -- Disarms when engine drops below this
  autoDisarmMaxThrottle = jbeamData.autoDisarmMaxThrottle or 0.3 -- Disarms when throttle drops below this
  autoDisarmTime = jbeamData.autoDisarmTime or 3 -- Time before system disarms [s]
  autoDisarmTimer = 0

  turboAVPIDController = newPIDParallel(0.01, 0, 0, 0, 1, -10, 10)

  state = states.idle
end

M.init = init
M.reset = reset

M.updateGFX = updateGFX

M.setAntilagState = setAntilagState

return M
