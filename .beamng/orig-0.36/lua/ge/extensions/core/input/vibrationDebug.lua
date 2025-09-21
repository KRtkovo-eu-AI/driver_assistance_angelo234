-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local imgui = ui_imgui

local wheelSlipForceMult = imgui.FloatPtr(1.0)
local jerkForceMult = imgui.FloatPtr(1.0)
local wheelSlipMin = imgui.FloatPtr(2.0)
local jerkMin = imgui.FloatPtr(30.0)

local function onUpdate()
  imgui.Begin("Vibration Debug")
  local veh = getPlayerVehicle(0)
  if veh then
    if imgui.InputFloat("Wheel Slip Force Multiplier", wheelSlipForceMult, 0.1, 1.0, "%.2f") then
      veh:queueLuaCommand("hydros.setWheelSlipForceMultiplier("..wheelSlipForceMult[0]..")")
    end
    if imgui.InputFloat("Jerk Force Multiplier", jerkForceMult, 0.1, 1.0, "%.2f") then
      veh:queueLuaCommand("hydros.setJerkForceMultiplier("..jerkForceMult[0]..")")
    end
    if imgui.InputFloat("Wheel Slip Min", wheelSlipMin, 0.1, 1.0, "%.2f") then
      veh:queueLuaCommand("hydros.setWheelSlipMin("..wheelSlipMin[0]..")")
    end
    if imgui.InputFloat("Jerk Min", jerkMin, 0.1, 1.0, "%.2f") then
      veh:queueLuaCommand("hydros.setJerkMin("..jerkMin[0]..")")
    end
  end
  imgui.End()
end

local function onVehicleSwitched()
  local veh = getPlayerVehicle(0)
  if veh then
    wheelSlipForceMult[0] = 1
    jerkForceMult[0] = 1
    wheelSlipMin[0] = 4
    jerkMin[0] = 30
  end
end

M.onUpdate = onUpdate
M.onVehicleSwitched = onVehicleSwitched

return M
