-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {'career_career'}

local repairExceptions = {
  "bumper", "door", "mirror", "fascia"
}

local lossPerKmRelative = 0.0000025
local scrapValueRelative = 0.05

-- vehicle damage related variables
local repairTimePerPart = 20 -- amount of seconds needed to repair one part
local brokenPartsThreshold = 3 -- a vehicle is considered to need repair after x broken parts
local minimumCarValue = 500
local minimumCarValueRelativeToNew = 0.05

local function getVehicleMileage(vehicle)
  if not vehicle.config.partsTree then return 0 end
  local partCondition = vehicle.partConditions[vehicle.config.partsTree.partPath]
  if not partCondition then
    log("E", "valueCalculator", "Couldnt find partCondition for " .. vehicle.config.partsTree.partPath .. " in vehicle " .. vehicle.id)
    return 0
  end
  return partCondition["odometer"]
end

local function getVehicleMileageById(inventoryId)
  return getVehicleMileage(career_modules_inventory.getVehicles()[inventoryId])
end

local depreciationByYear = {-0.20, -0.15, -0.10, -0.10, -0.07, -0.06, -0.05, -0.05, -0.04, -0.04, -0.03, -0.03, -0.02, -0.02, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.005, 0.0, 0.0, 0.005, 0.005, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.012, 0.012, 0.012, 0.012, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.020, 0.020, 0.020, 0.020, 0.020, 0.020, 0.020, 0.020, 0.020, 0.020, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025}

local function getValueByAge(value, age)
  local tempValue = value
  for i=1, age do
    tempValue = tempValue + tempValue * (depreciationByYear[i] or 0)
  end
  return tempValue
end

local function getAdjustedVehicleBaseValue(value, vehicleCondition)
  local valueByAge = getValueByAge(value, vehicleCondition.age)
  local scrapValue = valueByAge * scrapValueRelative
  local valueLossFromMileage = valueByAge * vehicleCondition.mileage/1000 * lossPerKmRelative
  local valueTemp = math.max(0, valueByAge - valueLossFromMileage)
  return valueTemp + scrapValue
end

local function getPartDifference(originalParts, newParts, changedSlots)
  local addedParts = {}
  local removedParts = {}
  for slotName, oldPart in pairs(originalParts) do
    local newPart = newParts[slotName]
    if newPart ~= oldPart.name then
      if oldPart.name ~= "" then
        -- part was removed
        removedParts[slotName] = oldPart.name
      end
      if newPart ~= "" then
        -- part was added
        addedParts[slotName] = newPart
      end
    end
  end

  for slotName, newPart in pairs(newParts) do
    local oldPart = originalParts[slotName]
    if newPart ~= "" then
      if not oldPart then
        -- part was added
        addedParts[slotName] = newPart
      end

      -- using part condition to see if there was another of the same part installed
      if changedSlots[slotName] and oldPart and newPart == oldPart.name then
        addedParts[slotName] = newPart
        removedParts[slotName] = originalParts[slotName]
      end
    end
  end

  return addedParts, removedParts
end

local function getPartValue(part)
  local value = getAdjustedVehicleBaseValue(part.value, {age = 2023 - part.year, mileage = part.partCondition["odometer"]})

  if part.primered then
    value = value * 0.95
  end

  if part.repairCount then
    value = value - value * (part.repairCount/(part.repairCount + 1)) * 0.2
  end
  return value
end

-- for now every damaged part needs to be replaced
local function getDamagedParts(vehInfo)
  local damagedParts = {
    partsToBeReplaced = {}
  }

  local function traversePartsTree(node)
    if not node.partPath then return end

    local partCondition = vehInfo.partConditions[node.partPath]
    if not partCondition then
      log("E", "valueCalculator", "Couldnt find partCondition for " .. node.partPath .. " in vehicle " .. vehInfo.id)
      return
    end

    if partCondition.integrityValue and partCondition.integrityValue == 0 then
      local part = career_modules_partInventory.getPart(vehInfo.id, node.path)
      table.insert(damagedParts.partsToBeReplaced, part)
    end

    if node.children then
      for childSlotName, childNode in pairs(node.children) do
        traversePartsTree(childNode)
      end
    end
  end

  if vehInfo.config.partsTree then
    traversePartsTree(vehInfo.config.partsTree)
  end

  return damagedParts
end

local function getRepairDetails(invVehInfo)
  local details = {
    price = 0,
    repairTime = 0
  }

  local damagedParts = getDamagedParts(invVehInfo)
  for _, part in pairs(damagedParts.partsToBeReplaced) do
    local price = part.value or 700
    details.price = details.price + price * 0.6-- lower the price a bit..
    details.repairTime = details.repairTime + repairTimePerPart
  end

  return details
end

-- IMPORTANT the pc file of a config does not contain the correct list of parts in the vehicle. there might be old unused slots/parts there and there might be slots/parts missing that are in the vehicle
-- the empty strings in the pc file are important, because otherwise the game will use the default part

local function getVehicleValue(configBaseValue, vehicle, ignoreDamage)
  local mileage = getVehicleMileage(vehicle)

  local partInventory = career_modules_partInventory.getInventory()

  local newParts = {}
  -- Loop through partInventory to find parts belonging to this vehicle
  for _, part in pairs(partInventory) do
    if part.location == vehicle.id then
      newParts[part.containingSlot] = part.name
    end
  end
  local originalParts = vehicle.originalParts
  local changedSlots = vehicle.changedSlots
  local addedParts, removedParts = getPartDifference(originalParts, newParts, changedSlots)
  local sumPartValues = 0
  for slot, partName in pairs(addedParts) do
    local part = career_modules_partInventory.getPart(vehicle.id, slot)
    if not part then
      log("E", "valueCalculator", "Couldnt find part " .. partName .. ", in slot " .. slot .. " of vehicle " .. vehicle.id)
    else
      sumPartValues = sumPartValues + 0.5 * getPartValue(part)
    end
  end

  for slot, partName in pairs(removedParts) do
    local part = {value = vehicle.originalParts[slot].value, year = vehicle.year, partCondition = {odometer = mileage}} -- use vehicle mileage to calculate the value of the removed part
    sumPartValues = sumPartValues - 0.5 * getPartValue(part)
  end

  local repairDetails = getRepairDetails(vehicle)
  if ignoreDamage then
    repairDetails.price = 0
  end

  local adjustedBaseValue = getAdjustedVehicleBaseValue(configBaseValue, {mileage = mileage, age = 2023 - (vehicle.year or 2023)})
  local minValue = math.min(minimumCarValue, configBaseValue * minimumCarValueRelativeToNew)
  return math.max(adjustedBaseValue + sumPartValues - repairDetails.price, minValue)
end

local function getInventoryVehicleValue(inventoryId, ignoreDamage)
  local vehicle = career_modules_inventory.getVehicles()[inventoryId]
  if not vehicle then return end
  return getVehicleValue(vehicle.configBaseValue, vehicle, ignoreDamage)
end

local function getNumberOfBrokenParts(partConditions)
  local counter = 0
  for partPath, info in pairs(partConditions) do
    if info.integrityValue and info.integrityValue == 0 then
      counter = counter + 1
    end
  end
  return counter
end

local function isPartException(partPath)
  for _, exception in ipairs(repairExceptions) do
    if string.find(partPath, exception) then
      return true
    end
  end
end

local function partConditionsNeedRepair(partConditions)
  return getNumberOfBrokenParts(partConditions) >= brokenPartsThreshold
  --[[ for partPath, info in pairs(partConditions) do
    if info.integrityValue and info.integrityValue == 0 and not isPartException(partPath) then
      return true
    end
  end
  return false ]]
end

local function getBrokenPartsThreshold()
  return brokenPartsThreshold
end

M.getPartDifference = getPartDifference

M.getInventoryVehicleValue = getInventoryVehicleValue
M.getPartValue = getPartValue
M.getAdjustedVehicleBaseValue = getAdjustedVehicleBaseValue
M.getVehicleMileageById = getVehicleMileageById
M.getBrokenPartsThreshold = getBrokenPartsThreshold

-- Vehicle damage related API
M.getRepairDetails = getRepairDetails
M.getNumberOfBrokenParts = getNumberOfBrokenParts
M.partConditionsNeedRepair = partConditionsNeedRepair
return M