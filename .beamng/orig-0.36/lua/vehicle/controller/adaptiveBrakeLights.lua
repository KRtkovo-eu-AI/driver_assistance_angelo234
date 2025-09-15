-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 900

local min = math.min
local floor = math.floor
local ceil = math.ceil

local isEnabled
local electricsName = nil
local absBlinkTimer = 0
local absBlinkOffTimer = 0
local absBlinkTime = 0
local absBlinkOffTime = 0
local blinkPulse = 1
local absActiveSmoother = nil
local escActiveSmoother = nil
local indicateESCUsageWithBrakelights = true
local activateHazardsAfterEmergencyBraking = true
local hazardArmSpeed = 10
local hazardActivateSpeed = 3
local hazardDeactivateThrottle = 0.3
local hazardDeactivateSpeed = 3
local emergencyBrakingHazardsArmed = false
local emergencyBrakingHazardsActive = false
local activateBrakeLightsFromDecel = false
local decelThreshold = 0
local prevWheelspeed = 0
local accelSmoother = nil
local decelBrakeLightsLatch = false

local function boolToNumber(bool)
  if not bool then
    return
  end
  if type(bool) == "boolean" then
    return bool and 1 or 0
  end
  return bool
end

local function updateGFX(dt)
  local brakeValue = electrics.values.brake or 0
  if isEnabled then
    local absActiveCoef = boolToNumber(electrics.values.absActive) or 0
    local absActive = absActiveSmoother:getUncapped(absActiveCoef, dt)
    local escActive = 0
    if indicateESCUsageWithBrakelights then
      local isESCActive = electrics.values.escActive or 0
      if electrics.values.isYCBrakeActive ~= nil then
        isESCActive = electrics.values.isYCBrakeActive
      end
      escActive = escActiveSmoother:getUncapped(isESCActive, dt)
    end

    if blinkPulse > 0 then
      absBlinkTimer = absBlinkTimer + dt * absActive
      if absBlinkTimer > absBlinkTime then
        absBlinkTimer = 0
        blinkPulse = 0
      end
    end

    if blinkPulse <= 0 then
      absBlinkOffTimer = absBlinkOffTimer + dt
      if absBlinkOffTimer > absBlinkOffTime then
        absBlinkOffTimer = 0
        blinkPulse = 1
      end
    end

    local wheelspeed = electrics.values.wheelspeed

    if wheelspeed >= hazardArmSpeed then
      emergencyBrakingHazardsArmed = true
    elseif absActive <= 0 then
      emergencyBrakingHazardsArmed = false
    end

    if emergencyBrakingHazardsArmed and absActive > 0 and wheelspeed < hazardActivateSpeed and activateHazardsAfterEmergencyBraking then
      electrics.set_warn_signal(true)
      emergencyBrakingHazardsActive = true
    end

    if emergencyBrakingHazardsActive and electrics.values.throttle > hazardDeactivateThrottle and wheelspeed > hazardDeactivateSpeed then
      electrics.set_warn_signal(false)
      emergencyBrakingHazardsActive = false
    end

    if activateBrakeLightsFromDecel and (electrics.values.regenFromOnePedal or 0) > 0.1 then
      -- regen-while-coasting is active; activate brake lights when decelerating quickly enough
      local acceleration = accelSmoother:get((wheelspeed - prevWheelspeed) / dt, dt)

      if -acceleration > decelThreshold then
        decelBrakeLightsLatch = true
      elseif -acceleration < decelThreshold * 0.8 then
        decelBrakeLightsLatch = false
      end
    else
      decelBrakeLightsLatch = false
    end

    local escBrakeValue = floor(escActive) * (wheelspeed > 14 and 1 or 0)
    local decelValue = decelBrakeLightsLatch and 1 or 0
    -- turn on brake lights when using regen via the brake pedal (in case electrics.values.brake is still 0)
    local regenBrakeValue = electrics.values.regenFromBrake or 0

    brakeValue = min(electrics.values.brake + escBrakeValue + decelValue + regenBrakeValue, 1)
    prevWheelspeed = wheelspeed
  else
    blinkPulse = 1
  end
  electrics.values[electricsName] = ceil(brakeValue * blinkPulse)
end

local function reset()
  if emergencyBrakingHazardsActive then
    electrics.set_warn_signal(false)
  end

  absBlinkTimer = 0
  absBlinkOffTimer = 0
  blinkPulse = 1
  absActiveSmoother:reset()
  escActiveSmoother:reset()
  emergencyBrakingHazardsArmed = false
  emergencyBrakingHazardsActive = false
  accelSmoother:reset()
  prevWheelspeed = electrics.values.wheelspeed or 0
  decelBrakeLightsLatch = false
end

local function init(jbeamData)
  electricsName = jbeamData.electricsName or "brakelights"
  indicateESCUsageWithBrakelights = jbeamData.indicateESCUsageWithBrakelights == nil and true or jbeamData.indicateESCUsageWithBrakelights
  activateHazardsAfterEmergencyBraking = jbeamData.activateHazardsAfterEmergencyBraking == nil and true or jbeamData.activateHazardsAfterEmergencyBraking
  hazardArmSpeed = jbeamData.hazardArmSpeed or 10
  hazardActivateSpeed = jbeamData.hazardActivateSpeed or 3
  hazardDeactivateThrottle = jbeamData.hazardDeactivateThrottle or 0.3
  hazardDeactivateSpeed = jbeamData.hazardDeactivateSpeed or 3
  absBlinkTime = jbeamData.blinkOnTime or 0.1
  absBlinkOffTime = jbeamData.blinkOffTime or 0.1
  activateBrakeLightsFromDecel = jbeamData.activateBrakeLightsFromDecel == nil and true or jbeamData.activateBrakeLightsFromDecel
  decelThreshold = jbeamData.decelThreshold or 1.3 -- m/s/s (default value is the legal threshold in the U.K.)
  accelSmoother = newTemporalSmoothingNonLinear(5)
  prevWheelspeed = 0
  decelBrakeLightsLatch = false

  isEnabled = true
  absBlinkTimer = 0
  absBlinkOffTimer = 0
  blinkPulse = 1
  absActiveSmoother = newTemporalSmoothing(2, 2)
  escActiveSmoother = newTemporalSmoothing(2, 2)
  emergencyBrakingHazardsArmed = false
  emergencyBrakingHazardsActive = false
end

local function setParameters(parameters)
  if parameters.isEnabled ~= nil then
    isEnabled = parameters.isEnabled
  end
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.setParameters = setParameters

return M
