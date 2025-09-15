-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- Tags:
local permissionTags = {
  vehicleModification = "allowed",-- Slow and Fast Repairing, Changing and buying parts, tuning, painting
  vehicleSelling = "allowed", --selling a vehicle
  vehicleStorage = "allowed", --put vehicles into storage
  vehicleRetrieval = "allowed", --retrieve vehicles from storage

  vehicleShopping = "allowed",

  interactRefuel = "allowed", --use the refueling POI to refuel vehicle
  interactMission = "allowed", --use the mission POI to start a mission
  interactDelivery = "allowed", --use any delivery POI to start delivery mode

  recoveryFlipUpright = "allowed", --flip upright
  recoveryTowToRoad = "allowed", --tow to road
  recoveryTowToGarage = "allowed", --tow to garage
}

-- permission can be:
-- "allowed" - normal behaviour, no restriction
-- "warning" - action can be done, but a warning is displayed. will probably have some effect on the current activity (ie ending it, penalty, etc)
-- "forbidden" - action is visible, but actively disabled. showing that the action exists, but cannot be performated at the moment
-- "hidden" - action is not visible at all

-- it is assumed that during an activity, the permissions don't change.
-- if permission do change, you can split it into two activites

local permissionPriorities = {
  allowed = 0,
  warning = 1,
  forbidden = 2,
  hidden = 3
}

local function getStatusForTag(tags, additionalData)
  local permissions = {}
  if type(tags) ~= "table" then
    tags = {tags}
  end
  extensions.hook("onCheckPermission", tags, permissions, additionalData)

  local result
  for _, permissionData in ipairs(permissions) do
    if not result or permissionPriorities[permissionData.permission] > permissionPriorities[result.permission] then
      result = permissionData
    end
  end
  if not result then
    return {permission = "allowed", allow = true}
  end
  local ret = {
    allow = result.permission == "warning",
    permission = result.permission,
    label = result.label,
    type = "text",
    penalty = result.penalty 
  }
  return ret
end

M.getStatusForTag = getStatusForTag

return M