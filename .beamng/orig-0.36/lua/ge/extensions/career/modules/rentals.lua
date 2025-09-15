-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local fileName = "insurance"

local baseAbandonTimer = 30
local currAbandonTimer = 0

local abandonDist = 300
local currRentalVeh

local function resetAbandonTimer()
  currAbandonTimer = baseAbandonTimer
end

local function resetValues()
  resetAbandonTimer()
end

local function vehReturn(veh)
  --calculate damage tier
  --refund totally, partially or not
  --pay for kms driven
end

local function automaticVehReturn(veh)
  --pay for recollection fee based on distance to nearest rental agency
  vehReturn(veh)
end

local function findNearestRentalAgency()
  --return distance from player and position
end

local function returnVehicleRecoveryButton()
  --tp at findNearestRentalAgency()
  --automaticVehReturn()
end

local function isRenting()
  return currRentalVeh ~= nil
end

local function beginRenting(vehInfo)
  --Spawn vehicle
  local vehicleInfo = career_modules_vehicleShopping.getVehiclesInShop()[shopId]
  local spawnOptions = {}
  spawnOptions.config = vehicleInfo.key
  spawnOptions.autoEnterVehicle = false
  local currRentalVeh = core_vehicles.spawnNewVehicle(vehicleInfo.model_key, spawnOptions)
  --Start tracking kms
  --Add the "stop renting veh" custom recovery prompt button
end

local function playAbandonTimer(dtReal)
  currAbandonTime = currAbandonTime - dtReal
end

local function trackAbandonVeh(dtReal)
  local rentalVehData = map.objects[currRentalVeh.id]
  local plVehData = map.objects[be:getPlayerVehicleID(0)]

  if not rentalVehData or not plVehData then return end

  local playerDist = plVehData.pos:distance(rentalVehData.pos)
  if playerDist > abandonDist then
    playAbandonTimer(dtReal)
    if currAbandonTime <= 0 then
      automaticVehReturn()
      resetAbandonTimer()
    end
  end --don't reset timer on "else" so we are certain rental vehicle will despawn
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if isRenting() then
    trackAbandonVeh(dtReal)
  end
end

local function loadData()
end

local function saveData(currentSavePath)
end

local function onExtensionLoaded()
  resetValues()
  loadData()
end

local function onSaveCurrentSaveSlot()
  saveData(saveData)
end

--UI api
M.isRenting = isRenting

-- M.onExtensionLoaded = onExtensionLoaded
-- M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot
-- M.onUpdate = onUpdate

return M
