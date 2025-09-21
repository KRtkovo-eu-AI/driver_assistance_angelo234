-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
--Mandatory controller parameters
M.type = "main"
M.engineInfo = {
  0,
  0,
  1000,
  1000,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "manual",
  obj:getId(),
  0,
  0,
  1,
  0,
  0,
  0
}
M.fireEngineTemperature = 0
M.throttle = 0
M.brake = 0
M.clutchRatio = 0
M.drivingAggression = 0
M.shiftingAggression = 0

-----

local function updateGFX(dt)
  electrics.values.throttle = math.min(math.max(input.throttle or 0, 0), 1)
  electrics.values.brake = math.min(math.max(input.brake or 0, 0), 1)
  electrics.values.clutch = math.min(math.max(input.clutch or 0, 0), 1)
  electrics.values.clutchRatio = 1 - electrics.values.clutch
  electrics.values.gear = "N"
  electrics.values.gearIndex = 0
  electrics.values.rpm = 0
  electrics.values.oiltemp = 0
  electrics.values.watertemp = 0
  electrics.values.fuel = 0
  electrics.values.lowfuel = false
  electrics.values.checkengine = false
  electrics.values.ignition = false
  electrics.values.engineThrottle = 0
  electrics.values.running = electrics.values.ignition

  electrics.values.gearboxMode = "none"
  electrics.values.freezeState = false

  if streams.willSend("engineInfo") then
    M.engineInfo[11] = obj:getGroundSpeed()
  end
end

M.init = nop
M.updateGFX = updateGFX

--Mandatory main controller API
M.shiftUp = nop
M.shiftDown = nop
M.shiftToGearIndex = nop
M.cycleGearboxModes = nop
M.setGearboxMode = nop
M.setStarter = nop
M.setEngineIgnition = nop
M.setFreeze = nop
M.sendTorqueData = nop
M.vehicleActivated = nop
-------------------------------

return M
