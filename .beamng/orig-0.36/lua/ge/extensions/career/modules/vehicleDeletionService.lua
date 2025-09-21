-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'career_career'}

local deleteDistance = 100

local flaggedVehicles = {}

local function onExtensionLoaded()
  if not career_career.isActive() then return false end
end

local function deleteVehicle(vehId)
  local inventoryId = career_modules_inventory.getInventoryIdFromVehicleId(vehId)
  if inventoryId then
    career_modules_inventory.removeVehicleObject(inventoryId)
  else
    local obj = getObjectByID(vehId)
    if obj then
      obj:delete()
    end
  end
end

local vehPos = vec3()
local camPos = vec3()
local camDir = vec3()
local function onUpdate()
  for vehId, data in pairs(flaggedVehicles) do
    local obj = getObjectByID(vehId)
    camPos:set(core_camera.getPositionXYZ())
    vehPos:set(obj:getPositionXYZ())

    -- check if vehicle is far enough away
    if camPos:distance(vehPos) > deleteDistance then
      camDir:set(core_camera.getForwardXYZ())
      local camToVeh = vehPos - camPos

      -- check if camera is looking away from vehicle
      if camDir:dot(camToVeh) < 0 then
        data.callback()
        deleteVehicle(vehId)
      end
    end
  end
end

local function checkOnUpdateFunction()
  if tableIsEmpty(flaggedVehicles) then
    if M.onUpdate then
      M.onUpdate = nil
      extensions.hookUpdate("onUpdate")
    end
  else
    if not M.onUpdate then
      M.onUpdate = onUpdate
      extensions.hookUpdate("onUpdate")
    end
  end
end

local function deleteFlaggedVehicles()
  for vehId, _ in pairs(flaggedVehicles) do
    deleteVehicle(vehId)
  end
end

local function flagForDeletion(vehId, callback)
  flaggedVehicles[vehId] = flaggedVehicles[vehId] or {}
  flaggedVehicles[vehId].delete = true
  flaggedVehicles[vehId].callback = callback or nop
  checkOnUpdateFunction()
end

local function clearFlags(vehId)
  flaggedVehicles[vehId] = nil
  checkOnUpdateFunction()
end

local function onClientStartMission()
  table.clear(flaggedVehicles)
  checkOnUpdateFunction()
end

local function onAnyMissionChanged(state, mission)
  if not (career_career and career_career.isActive()) then return end
  if mission then
    if state == "started" then
      deleteFlaggedVehicles()
    end
  end
end

local function onVehicleDestroyed(vehId)
  clearFlags(vehId)
end

M.flagForDeletion = flagForDeletion
M.clearFlags = clearFlags
M.deleteFlaggedVehicles = deleteFlaggedVehicles

M.onExtensionLoaded = onExtensionLoaded
M.onAnyMissionChanged = onAnyMissionChanged
M.onClientStartMission = onClientStartMission
M.onVehicleDestroyed = onVehicleDestroyed

return M