-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui

local listedVehicles = {}

local timeBetweenOffersBase = 95
local offerTTL = 500
local offerTTLVariance = 0.5
local valueLossLimit = 0.95

local offerMenuOpen = false

local function findVehicleListing(inventoryId)
  for _, listing in ipairs(listedVehicles) do
    if listing.id == inventoryId then
      return listing
    end
  end
end

local function listVehicles(vehicles)
  local timestamp = os.time()
  for _, inventoryId in ipairs(vehicles) do
    local veh = career_modules_inventory.getVehicles()[inventoryId]
    if veh and not findVehicleListing(inventoryId) then
      local listingData = {
        id = veh.id,
        timestamp = timestamp,
        offers = {},
        value = career_modules_valueCalculator.getInventoryVehicleValue(inventoryId),
        timeOfNextOffer = nil,
        niceName = veh.niceName,
        thumbnail = career_modules_inventory.getVehicleThumbnail(inventoryId)
      }
      table.insert(listedVehicles, listingData)
    end
  end
end

local function removeVehicleListing(inventoryId)
  for i, listing in ipairs(listedVehicles) do
    if listing.id == inventoryId then
      table.remove(listedVehicles, i)
    end
  end
end

local function generateOffer(inventoryId)
  local listing = inventoryId and findVehicleListing(inventoryId) or listedVehicles[math.random(1, #listedVehicles)]
  local offer = {
    timestamp = os.time(),
    value = round(listing.value * (biasGainFun(math.random(), 0.5, 0.03) * 0.5 + 0.73)),
    ttl = offerTTL + ((math.random() * offerTTLVariance*2) - offerTTLVariance) * offerTTL
  }
  table.insert(listing.offers, offer)
  return offer
end

local function acceptOffer(inventoryId, offerIndex)
  for i, listing in ipairs(listedVehicles) do
    if listing.id == inventoryId then
      local offer = listing.offers[offerIndex]
      table.remove(listing.offers, offerIndex)
      career_modules_inventory.sellVehicle(inventoryId, offer.value)
      return
    end
  end
end

local function deleteOffer(inventoryId, offerIndex)
  for i, listing in ipairs(listedVehicles) do
    if listing.id == inventoryId then
      table.remove(listing.offers, offerIndex)
      return
    end
  end
end

local function getOfferCount()
  local count = 0
  for _, listing in ipairs(listedVehicles) do
    count = count + #listing.offers
  end
  return count
end

local function updateListings()
  local timeNow = os.time()
  local offerCountDiff = 0

  for _, listing in ipairs(listedVehicles) do
    if not listing.timeOfNextOffer then
      listing.timeOfNextOffer = timeNow + timeBetweenOffersBase + (math.random(-60, 60) / 100 * timeBetweenOffersBase)
    end

    if timeNow >= listing.timeOfNextOffer then
      listing.timeOfNextOffer = nil
      generateOffer(listing.id)
      local offerValue = listing.offers[#listing.offers].value
      guihooks.trigger("toastrMsg", {type="info", title="New offer for your listed vehicle", msg = listing.niceName .. ": $" .. string.format("%.2f", offerValue) .. " ( " .. (offerValue > listing.value and "+ " or "- ") .. string.format("%.2f", math.abs(offerValue - listing.value)) .. "$ )"})
      offerCountDiff = offerCountDiff + 1
    end

    for offerIndex = #listing.offers, 1, -1 do
      local offer = listing.offers[offerIndex]
      if not offer.expiredViewCounter and timeNow - offer.timestamp > (offer.ttl or offerTTL) then
        offer.expiredViewCounter = 1
        offerCountDiff = offerCountDiff - 1
      end
    end
  end

  return offerCountDiff
end

local timeSinceUpdate = 0
local function onUpdate(dtReal, dtSim, dtRaw)
  if tableIsEmpty(listedVehicles) or offerMenuOpen then
    return
  end

  timeSinceUpdate = timeSinceUpdate + dtSim
  if timeSinceUpdate < 10 then return end
  timeSinceUpdate = 0

  updateListings()
end

local function onVehicleRemoved(inventoryId)
  removeVehicleListing(inventoryId)
end

local function getListings()
  local listingsCopy = deepcopy(listedVehicles)
  for i, listing in ipairs(listingsCopy) do
    local currentValue = career_modules_valueCalculator.getInventoryVehicleValue(listing.id)
    if currentValue < listing.value * valueLossLimit then
      listing.disabled = true
      listing.disableReason = "Cant sell the vehicle because value has dropped below " .. valueLossLimit * 100 .. "% of the initial listing value"
    end

    for _, offer in ipairs(listing.offers) do
      if offer.expiredViewCounter then
        offer.disabled = true
        offer.disableReason = "Cant sell the vehicle because the offer has expired"
      end
    end
  end
  return listingsCopy
end

local function menuOpened(open)
  offerMenuOpen = open

  -- generate offers as if they have been generated while the menu was closed
  if open then
    for i, listing in ipairs(listedVehicles) do
      for offerIndex = #listing.offers, 1, -1 do
        local offer = listing.offers[offerIndex]
        if offer.expiredViewCounter then
          offer.expiredViewCounter = offer.expiredViewCounter + 1
          if offer.expiredViewCounter > 3 then
            table.remove(listing.offers, offerIndex)
          end
        end
      end
    end
  else
    local offerCountDiff = updateListings()
    if offerCountDiff < 0 then
      for i = 1, math.abs(offerCountDiff) do
        local offer = generateOffer()
        -- randomize the offer timestamp
        offer.timestamp = offer.timestamp + math.random(1, offerTTL)
      end
    end
  end
end

local function openMenu(computerId)
  career_modules_vehicleShopping.openShop(nil, computerId, "marketplace")
end

local function onSaveCurrentSaveSlot(currentSavePath, oldSaveDate, vehiclesThumbnailUpdate)
  career_saveSystem.jsonWriteFileSafe(currentSavePath .. "/career/marketplace.json", {
    listedVehicles = listedVehicles
  }, true)
end

local function onExtensionLoaded()
  if not career_career.isActive() then return false end

  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
  if not saveSlot or not savePath then return end

  local data = jsonReadFile(savePath .. "/career/marketplace.json")
  if data then
    listedVehicles = data.listedVehicles
  end
end

M.onUpdate = onUpdate
M.onVehicleRemoved = onVehicleRemoved
M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot
M.onExtensionLoaded = onExtensionLoaded

M.getListings = getListings
M.menuOpened = menuOpened
M.acceptOffer = acceptOffer
M.declineOffer = deleteOffer
M.listVehicles = listVehicles
M.findVehicleListing = findVehicleListing
M.openMenu = openMenu
M.removeVehicleListing = removeVehicleListing
M.generateOffer = generateOffer

return M