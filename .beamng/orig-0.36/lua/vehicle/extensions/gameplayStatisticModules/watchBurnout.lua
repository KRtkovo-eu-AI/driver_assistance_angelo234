-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local wd
local velocity
local currently = false
local initialWheelCount = 0
local wheelsBurnout = {}
local max, min = math.max, math.min

local function watchBurnout()

  velocity = obj:getGroundSpeed() * 1.5
  currently = false
  if initialWheelCount ~= wheels.wheelCount then return end --if we lose a wheel
  for _, wIndex in ipairs(wheelsBurnout) do
    wd = wheels.wheels[wIndex]
    if wd.isBroken == false and wd.wheelSpeed > 2 and wd.wheelSpeed > velocity and (wd.lastSlip * min(wd.downForce * 0.1, 1))>4 and wd.contactMaterialID1 ~=-1 and wd.contactMaterialID2 ~=-1 then
      currently = true
    end
  end
  gameplayStatistic.refreshTimer("vehicle/burnout.time", currently, 0.1, true)
end

local function onExtensionLoaded()
  if controller.mainController.typeName ~= "vehicleController/vehicleController" then
    return false --unload
  end

  initialWheelCount = wheels.wheelCount
  local i
  if initialWheelCount>0 then
    for i = 0, initialWheelCount-1 do
      local wd = wheels.wheels[i]
      if wd.isPropulsed then
        wheelsBurnout[#wheelsBurnout +1] = i
      end
    end
    if #wheelsBurnout>0 then
      -- gameplayStatistic.addSchedule(watchBurnout)
      return
    end
  end
  return false
end

M.onExtensionLoaded = onExtensionLoaded
M.workload = watchBurnout

return M