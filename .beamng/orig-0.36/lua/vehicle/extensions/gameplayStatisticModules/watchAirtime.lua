-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local initialWheelCount = 0
local wd
local currently
local tmpVec = vec3(0,0,0)
local upVector = vec3(0,0,1)

local function watchAirtime()
  if initialWheelCount ~= wheels.wheelCount then return end --if we lose a wheel
  currently = obj:getGroundSpeed() > 6

  for i = 0, initialWheelCount-1 do
    wd = wheels.wheels[i]
    currently = currently and wd.contactMaterialID1 ==-1 and wd.contactMaterialID2 ==-1
    if not currently then break end
  end
  --todo redo this
  tmpVec:set(obj:getDirectionVectorUpXYZ())
  currently = currently and (upVector:dot(tmpVec) > 0.707 ) --45deg

  gameplayStatistic.refreshTimer("vehicle/airtime.time", currently, 0.1, true)
end

local function onExtensionLoaded()
  if controller.mainController.typeName ~= "vehicleController/vehicleController" then
    return false --unload
  end

  initialWheelCount = wheels.wheelCount
  if initialWheelCount==0 then
    return false
  end

end

M.onExtensionLoaded = onExtensionLoaded
M.workload = watchAirtime

return M