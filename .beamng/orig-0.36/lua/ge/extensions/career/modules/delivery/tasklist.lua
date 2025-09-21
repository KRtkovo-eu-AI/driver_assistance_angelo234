-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"core_vehicleBridge"}
local dParcelManager, dCargoScreen, dGeneral, dGenerator, dProgress, dVehicleTasks
M.onCareerActivated = function()
  dParcelManager = career_modules_delivery_parcelManager
  dCargoScreen = career_modules_delivery_cargoScreen
  dGeneral = career_modules_delivery_general
  dGenerator = career_modules_delivery_generator
  dProgress = career_modules_delivery_progress
  dVehicleTasks = career_modules_delivery_vehicleTasks
end


local cargoIdToTasklistElementId = {}
local offerIdToTasklistElementId = {}
local tasklistElements = {}
local function sendCargoToTasklist()
  --log("I","","sendCargoToTasklist")

  dGeneral.getNearbyVehicleCargoContainers(function(containers)
    cargoIdToTasklistElementId = {}
    for _, elem in pairs(tasklistElements) do
      elem.clear = true
    end

    -- cargo already loaded
    local cargoCount = 0
    local cargoGrouped = {}
    for _, con in ipairs(containers) do
      for _, cargo in ipairs(con.rawCargo) do
        local gId = string.format("%s-%s", dParcelManager.getLocationLabelShort(cargo.destination),
          #cargo.modifiers == 0 and "noMods"
          or string.format("%s-%0.2f", cargo.groupId, cargo.loadedAtTimeStamp or -1)
          )
        cargoCount = cargoCount +1
        cargoGrouped[gId] = (cargoGrouped[gId] or {})
        table.insert(cargoGrouped[gId], cargo.id)
      end
    end
    for _, gId in ipairs(tableKeysSorted(cargoGrouped)) do
      local tasklistElement = {
        type = "parcels",
        cargoIds = cargoGrouped[gId],
        id = gId,
        update = true,
      }
      tasklistElements[gId] = tasklistElement
      for _, cId in ipairs(cargoGrouped[gId]) do
        cargoIdToTasklistElementId[cId] = gId
      end
    end

    -- cargo to be loaded
    local transientCount = 0
    local transientPickupLocationGrouped = {}
    for _, con in ipairs(containers) do
      for _, cargo in ipairs(con.transientCargo) do
        local dId = string.format("%s", dParcelManager.getLocationLabelLong(cargo.location))
        transientCount = transientCount +1
        transientPickupLocationGrouped[dId] = (transientPickupLocationGrouped[dId] or {})
        table.insert(transientPickupLocationGrouped[dId], cargo.id)
      end
    end
    for _, dId in ipairs(tableKeysSorted(transientPickupLocationGrouped)) do
      local tasklistElement = {
        type = "pickup",
        cargoIds = transientPickupLocationGrouped[dId],
        id = dId,
        update = true,
      }
      tasklistElements[dId] = tasklistElement
      for _, cId in ipairs(transientPickupLocationGrouped[dId]) do
        cargoIdToTasklistElementId[cId] = dId
      end
    end


    -- trailer tasks
    local vehicleTaskCount = 0
    for _, taskData in ipairs(dVehicleTasks.getVehicleTasks()) do
      local tasklistElement = {
        type = taskData.offer.data.type,
        id = "trailer"..taskData.offer.id,
        update = true,
        taskData = taskData
      }
      tasklistElements[tasklistElement.id] = tasklistElement
      offerIdToTasklistElementId[taskData.offer.id] = tasklistElement.id
      vehicleTaskCount = vehicleTaskCount + 1
    end

    -- only include header if there is a task remaining (that is not to be deleted)
    local hasNonClearElement = false
    for _, elem in pairs(tasklistElements) do
      hasNonClearElement = hasNonClearElement or not elem.clear
    end
    if hasNonClearElement then
      local subtext = ""
      if cargoCount > 0 and vehicleTaskCount == 0 then
        subtext = string.format("%d item%s loaded.", cargoCount, cargoCount > 1 and "s" or "")
      else
        subtext = string.format("%d ongoing task%s.", cargoCount + vehicleTaskCount + transientCount, (cargoCount + vehicleTaskCount + transientCount) > 1 and "s" or "")
      end
      tasklistElements.header = {
        id = "header",
        update = true,
        type = "header",
        label = "Active Deliveries",
        subtext = subtext
      }
    end

  end)
end
M.sendCargoToTasklist = sendCargoToTasklist

local function updateTasklistForCargoId(cargoId)
  if cargoIdToTasklistElementId[cargoId] and tasklistElements[cargoIdToTasklistElementId[cargoId]] then
    tasklistElements[cargoIdToTasklistElementId[cargoId]].update = true
  end
end
M.updateTasklistForCargoId = updateTasklistForCargoId

local function updateTasklistForOfferId(offerId)
  if offerIdToTasklistElementId[offerId] and tasklistElements[offerIdToTasklistElementId[offerId]] then
    tasklistElements[offerIdToTasklistElementId[offerId]].update = true
  end
end
M.updateTasklistForOfferId = updateTasklistForOfferId

local function clearTasklistForOfferId(offerId)
  if offerIdToTasklistElementId[offerId] and tasklistElements[offerIdToTasklistElementId[offerId]] then
    tasklistElements[offerIdToTasklistElementId[offerId]].clear = true
  end
end
M.clearTasklistForOfferId = clearTasklistForOfferId

local function clearAll()
  for _, elem in pairs(tasklistElements) do
    elem.clear = true
  end
end
M.clearAll = clearAll

local anyUpdated = false
local elementsToClearById = {}
local function updateCargoTasklistElements()
  anyUpdated = false

  table.clear(elementsToClearById)
  for _, tasklistId in ipairs(tableKeysSorted(tasklistElements)) do
    -- clearing of no longer used tasks.
    local elem = tasklistElements[tasklistId]
    if elem.clear then
      if elem.type == "parcels" then
        guihooks.trigger("DiscardTasklistItem", elem.id)
      end
      if elem.type == "pickup" then
        guihooks.trigger("DiscardTasklistItem", elem.id)
      end
      if elem.type == "header" then
        guihooks.trigger("SetTasklistHeader",nil)
      end
      if elem.type == "trailer" or elem.type == "vehicle" then
        guihooks.trigger("DiscardTasklistItem", elem.id)
      end
      elementsToClearById[elem.id] = true
      --log("I","",string.format("Deleting %s - %s", elem.type, tasklistId))
    end

    -- updating tasks that still exist
    if elem.update then
      --log("I","",string.format("Updating %s - %s", elem.type, tasklistId))
      anyUpdated = true
      -- parcel tasks. these usually consist of multiple parcels grouped by destination.
      if elem.type == "parcels" then
        local first = dParcelManager.getCargoById(elem.cargoIds[1])
        local modifierStrings = {string.format("%d Item%s", #elem.cargoIds, #elem.cargoIds~=1 and "s" or "")}

        for _, mod in ipairs(first.modifiers) do
          if mod.type == "timed" then
            local expiredTime = dGeneral.time() - first.loadedAtTimeStamp
            if expiredTime <= mod.timeUntilDelayed then
              table.insert(modifierStrings, string.format("Time: %ds", mod.timeUntilDelayed - expiredTime))
            elseif expiredTime <= mod.timeUntilLate then
              table.insert(modifierStrings, string.format("Delayed: %ds", mod.timeUntilLate - expiredTime))
            else
              table.insert(modifierStrings, "Late")
            end

          end
        end
        guihooks.trigger("SetTasklistTask", {
            id = tasklistId,
            label = string.format("Deliver to %s",  dParcelManager.getLocationLabelShort(first.destination)),
            subtext = table.concat(modifierStrings, ", "),
            active = true,
            type = "message"
          }
        )
      end

      -- pickup
      if elem.type == "pickup" then
        local first = dParcelManager.getCargoById(elem.cargoIds[1])
        guihooks.trigger("SetTasklistTask", {
            id = tasklistId,
            label = string.format("Pick up %d items from %s", #elem.cargoIds, dParcelManager.getLocationLabelLong(first.location)),
            active = true,
            type = "message"
          }
        )
      end


      -- trailer tasks are often a singular tasks with multiple stages.
      if elem.type == "trailer" or elem.type == "vehicle" then
        local task = elem.taskData.tasks[elem.taskData.activeTaskIndex]
        if task then
          local label
          if task.type == "coupleTrailer" then
            label = string.format("Couple the trailer.")
          elseif task.type == "enterVehicle" then
            label = string.format("Enter the vehicle.")
          elseif task.type == "bringToDestination" or task.type == "confirmDropOff" then
            label = string.format("Drop off %s at %s.",elem.type == "trailer" and "the trailer" or "the vehicle", dParcelManager.getLocationLabelShort(task.destination))
          elseif task.type == "putIntoParkingSpot" then
            local forwardString = ""
            if task.forwardOn == "uncouple" then forwardString = "decouple" end
            if task.forwardOn == "exitVehicle" then forwardString = "exit the vehicle" end
            label = string.format("Park Trailer in %s and %s.", dParcelManager.getLocationLabelShort(task.destination), forwardString)
          --elseif task.type == "confirmDropOff" then
          --label = string.format("Confirm the dropoff at %s.", dParcelManager.getLocationLabelShort(task.destination))
          end
          local subtext = string.format("%s",elem.taskData.offer.vehicle.name)
          if elem.type == "vehicle" then
            subtext = string.format("%s %s",elem.taskData.offer.vehicle.brand, elem.taskData.offer.vehicle.name)
          end
          guihooks.trigger("SetTasklistTask", {
              id = tasklistId,
              label = label,
              subtext = subtext,
              active = true,
              type = "message"
            }
          )
        end
      end


      if elem.type == "header" then
        guihooks.trigger("SetTasklistHeader", {
          label = elem.label,
          subtext = elem.subtext
        })
      end
      elem.update = false
    end
  end

  for id, _ in pairs(elementsToClearById) do
    tasklistElements[id] = nil
  end

end
M.updateCargoTasklistElements = updateCargoTasklistElements

M.onUpdate = function(dtReal, dtSim, dtRaw)
  M.updateCargoTasklistElements()
end

return M