-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local max = math.max

local powerSteeringStrengthSlow = 0
local powerSteeringStrengthFast = 0
local powerSteeringSpeedSlow = 0
local powerSteeringSpeedFast = 0

local constantOffset = 0
local strengthRange = 0
local speedRange = 0

local function updateGFXVariableCoef(dt)
  local speed = electrics.values.wheelspeed or 0
  local strengthCoef = max(strengthRange * max(min(speed, powerSteeringSpeedFast) - powerSteeringSpeedSlow, 0) / speedRange + powerSteeringStrengthSlow, 0)
  strengthCoef = max(strengthCoef + constantOffset, 0)

  hydros.wheelPowerSteeringCoef = strengthCoef
end

local function reset()
  hydros.wheelPowerSteeringCoef = 1 + constantOffset
end

local function init(jbeamData)
  powerSteeringStrengthFast = jbeamData.strengthFast or 1
  powerSteeringStrengthSlow = jbeamData.strengthSlow or 1
  powerSteeringSpeedSlow = jbeamData.speedSlow or 0
  powerSteeringSpeedFast = jbeamData.speedFast or 0

  constantOffset = jbeamData.constantOffset or -0.2

  strengthRange = powerSteeringStrengthFast - powerSteeringStrengthSlow
  speedRange = powerSteeringSpeedFast - powerSteeringSpeedSlow

  local hasCoefScaling = speedRange > 0 and strengthRange > 0

  hydros.wheelPowerSteeringCoef = 1 + constantOffset
  M.updateGFX = hasCoefScaling and updateGFXVariableCoef or nil
end

M.init = init
M.reset = reset
M.updateGFX = nil

return M
