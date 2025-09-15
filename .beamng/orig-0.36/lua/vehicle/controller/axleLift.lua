-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.relevantDevice = "transfercase"

local min = math.min
local abs = math.abs

local hasBuiltPie

local currentMode
local modes = { auto = "auto", manual = "manual", off = "off" }

local velocityThresholdDisable
local velocityThresholdEnable
local accelerationThreshold

local frontLoweredPosition
local rearLoweredPosition
local frontRaisedPosition
local rearRaisedPosition
local velocitySmoother = newTemporalSmoothing(2, 10)

local function updateGFX(dt)
  local frontPos = frontRaisedPosition
  local rearPos = rearRaisedPosition
  local velocity = velocitySmoother:getUncapped(min(electrics.values.wheelspeed or 0, velocityThresholdDisable + 2), dt)
  if currentMode == modes.auto then
    local speedTooHigh = velocity > velocityThresholdDisable
    local speedTooLow = (electrics.values.wheelspeed or 0) < velocityThresholdEnable
    local parkingBrakeActive = electrics.values.parkingbrake ~= 0
    local accelerationTooHigh = abs(sensors.gy2) > accelerationThreshold

    if speedTooHigh or speedTooLow or parkingBrakeActive or accelerationTooHigh then
      frontPos = frontLoweredPosition
      rearPos = rearLoweredPosition
    end
  elseif currentMode == modes.manual then
    frontPos = frontRaisedPosition
    rearPos = rearRaisedPosition
  elseif currentMode == modes.off then
    frontPos = frontLoweredPosition
    rearPos = rearLoweredPosition
  end

  electrics.values.strut_F_axleLift = frontPos
  electrics.values.strut_R_axleLift = rearPos
end

local function setMode(mode)
  currentMode = mode
  guihooks.message("Axlelift: " .. string.sentenceCase(currentMode), 5, "vehicle.axleLift")
end

local function getNextMode()
  if currentMode == modes.auto then
    return modes.manual
  elseif currentMode == modes.manual then
    return modes.off
  else
    return modes.auto
  end
end

local function toggleMode()
  setMode(getNextMode())
end

local function init(jbeamData)
  velocityThresholdDisable = jbeamData.velocityThresholdDisable or 14
  velocityThresholdEnable = jbeamData.velocityThresholdEnable or 0.1
  accelerationThreshold = jbeamData.accelerationThreshold or 3
  frontLoweredPosition = jbeamData.frontLoweredPosition or 0
  rearLoweredPosition = jbeamData.rearLoweredPosition or 0
  frontRaisedPosition = jbeamData.frontRaisedPosition or 1
  rearRaisedPosition = jbeamData.rearRaisedPosition or 1

  electrics.values.strut_F_axleLift = 0
  electrics.values.strut_R_axleLift = 0

  velocitySmoother:reset()
  setMode(modes.auto)

  if not hasBuiltPie then
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/vehicleFeatures/",
        generator = function(entries)
          local noEntry = {
            title = "Axle Lift",
            priority = 40,
            icon = "axleLift",
            uniqueID = "axleLiftToggleMode",
            onSelect = function()
              controller.getControllerSafe("axleLift").toggleMode()
              return { "reload" }
            end
          }
          table.insert(entries, noEntry)
        end
      }
    )
  end
  hasBuiltPie = true
end

local function setParameters(parameters)
  if parameters.mode ~= nil then
    setMode(parameters.mode)
  end
  if parameters.accelerationThreshold ~= nil then
    accelerationThreshold = parameters.accelerationThreshold
  end
  if parameters.velocityThresholdDisable ~= nil then
    velocityThresholdDisable = parameters.velocityThresholdDisable
  end
  if parameters.velocityThresholdEnable ~= nil then
    velocityThresholdEnable = parameters.velocityThresholdEnable
  end
end

M.init = init
M.updateGFX = updateGFX
M.toggleMode = toggleMode
M.setMode = setMode

M.setParameters = setParameters

return M
