-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local abs = math.abs
local max = math.max

local throttleLockMap
local allowedSlipMap
local slipLockMap
local brakeLockMap
local leftFootBrakeLockMap
local steeringLockMap
local handbrakeLockMap

local centerDiff
local axleDiff1
local axleDiff2

local biasOversteerVsTraction = 0.5
local desiredLock = 0

local function updateFixedStep(dt)
  local vehicleSpeed = electrics.values.wheelspeed
  local throttleInput = electrics.values.throttle_input
  local brakeInput = electrics.values.brake_input
  local handbrakeInput = electrics.values.parkingbrake
  local frontAxleAV = axleDiff1.inputAV
  local rearAxleAV = axleDiff2.inputAV
  local steeringInput = abs(electrics.values.steering_input)
  --dump(electrics.values)

  --print("--------------------------------")

  local oversteerControl = throttleLockMap:get(throttleInput, vehicleSpeed)
  --print("oversteerControl: " .. oversteerControl)
  --calculate slip between axle1 and axle2
  local slip = abs((frontAxleAV - rearAxleAV) / max(abs(frontAxleAV), abs(rearAxleAV), 0.1)) --Calculate normalized slip ratio between axles
  --print("slip: " .. slip)
  local allowedSlip = allowedSlipMap:get(throttleInput, vehicleSpeed)
  --print("allowedSlip: " .. allowedSlip)
  local slipControl = slipLockMap:get(max(slip - allowedSlip, 0), vehicleSpeed)
  --print("slipControl: " .. slipControl)
  local stabilityControl = brakeLockMap:get(brakeInput, vehicleSpeed)
  --print("stabilityControl: " .. stabilityControl)
  local leftFootBrakeControl = leftFootBrakeLockMap:get(brakeInput, vehicleSpeed)
  --print("leftFootBrakeControl: " .. leftFootBrakeControl)
  local steeringControl = steeringLockMap:get(steeringInput, vehicleSpeed)
  --print("steeringControl: " .. steeringControl)

  local accelerationLock = oversteerControl * biasOversteerVsTraction + slipControl * (1 - biasOversteerVsTraction) - leftFootBrakeControl * throttleInput
  local decelerationLock = stabilityControl - steeringControl
  --print("accelerationLock: " .. accelerationLock)
  --print("decelerationLock: " .. decelerationLock)

  local modeSelection = clamp(throttleInput - brakeInput, -1, 1)
  --print("modeSelection: " .. modeSelection)

  desiredLock = modeSelection < -0.01 and decelerationLock or accelerationLock
  --print("desiredLock: " .. desiredLock)

  local handbrakeLockCoef = handbrakeLockMap:get(handbrakeInput, vehicleSpeed)
  --print("handbrakeLockCoef: " .. handbrakeLockCoef)
  local absActiveLockCoef = 1 - (electrics.values.absActive or 0)

  desiredLock = clamp(desiredLock,0,1) * handbrakeLockCoef * absActiveLockCoef

  --print(string.format("Desired Lock: %.2f ", desiredLock) .. string.format("Slip: %.2f ", slip))
  centerDiff.activeLockCoef = desiredLock
end

local function updateGFX(dt)
end

local function reset(jbeamData)
end

-- local function resetLastStage(jbeamData)
-- end

local function init(jbeamData)
  biasOversteerVsTraction = jbeamData.biasOversteerVsTraction or 0.5

  local interpolatedMap = rerequire("interpolatedMap")
  throttleLockMap = interpolatedMap.new()
  throttleLockMap.clampToDataRange = true
  throttleLockMap:loadData(jbeamData.throttleLockMap)

  allowedSlipMap = interpolatedMap.new()
  allowedSlipMap.clampToDataRange = true
  allowedSlipMap:loadData(jbeamData.allowedSlipMap)

  slipLockMap = interpolatedMap.new()
  slipLockMap.clampToDataRange = true
  slipLockMap:loadData(jbeamData.slipLockMap)

  brakeLockMap = interpolatedMap.new()
  brakeLockMap.clampToDataRange = true
  brakeLockMap:loadData(jbeamData.brakeLockMap)

  leftFootBrakeLockMap = interpolatedMap.new()
  leftFootBrakeLockMap.clampToDataRange = true
  leftFootBrakeLockMap:loadData(jbeamData.leftFootBrakeLockMap)

  steeringLockMap = interpolatedMap.new()
  steeringLockMap.clampToDataRange = true
  steeringLockMap:loadData(jbeamData.steeringLockMap)

  handbrakeLockMap = interpolatedMap.new()
  handbrakeLockMap.clampToDataRange = true
  handbrakeLockMap:loadData(jbeamData.handbrakeLockMap)

  local diffName = jbeamData.centerDiffName
  centerDiff = powertrain.getDevice(diffName)

  local axleDiff1Name = jbeamData.axleDiff1Name
  axleDiff1 = powertrain.getDevice(axleDiff1Name)

  local axleDiff2Name = jbeamData.axleDiff2Name
  axleDiff2 = powertrain.getDevice(axleDiff2Name)

  if not centerDiff or not axleDiff1 or not axleDiff2 then
    print("Diff(s) not found")
    M.updateFixedStep = nil
    M.updateGFX = nil
    return
  end
end

-- local function initLastStage(jbeamData)
-- end

M.init = init
--M.initLastStage = initLastStage
M.reset = reset
--M.resetLastStage = resetLastStage
M.updateFixedStep = updateFixedStep
M.updateGFX = updateGFX

return M
