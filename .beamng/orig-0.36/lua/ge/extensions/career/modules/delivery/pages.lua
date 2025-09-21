-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local dParcelManager, dCargoScreen, dGeneral, dGenerator, dPages, dProgress
M.onCareerActivated = function()
  dParcelManager = career_modules_delivery_parcelManager
  dCargoScreen = career_modules_delivery_cargoScreen
  dGeneral = career_modules_delivery_general
  dGenerator = career_modules_delivery_generator
  dPages = career_modules_delivery_pages
  dProgress = career_modules_delivery_progress
end

-- logbook integration
local function onLogbookGetEntries(list)
  local progress = dProgress.getProgress()

  local facTable = {
    headers = {'Name','Delivered from here','Delivered to here'},
    rows = {}
  }

  local unlockedCount, providingFacilitiesCount = 0, 0
  local totalItems, totalMoney
  for _, fac in ipairs(dGenerator.getFacilities()) do
    if dProgress.isFacilityUnlocked(fac.id) then
      unlockedCount = unlockedCount +1
    end
    if next(fac.logisticTypesProvided) then
      providingFacilitiesCount = providingFacilitiesCount + 1
    end
    if dProgress.isFacilityVisible(fac.id) and (fac.progress.itemsDeliveredFromHere.count > 0 or fac.progress.itemsDeliveredToHere.count > 0 or (dProgress.isFacilityUnlocked(fac.id) and next(fac.logisticTypesProvided))) then
      table.insert(facTable.rows,{
        fac.name,
        fac.progress.itemsDeliveredFromHere.count > 0
          and string.format("%d Items, %0.2f$",fac.progress.itemsDeliveredFromHere.count, fac.progress.itemsDeliveredFromHere.moneySum)
          or "-",

        fac.progress.itemsDeliveredToHere.count > 0
        and string.format("%d Items, %0.2f$",fac.progress.itemsDeliveredToHere.count, fac.progress.itemsDeliveredToHere.moneySum)
        or "-"
      }
      )
    end
  end

  local facText = string.format('<span>Below is an overview of all facilities you have delivered an item to or from.</span><ul><li>You have <b>unlocked %d/%d facilities</b> that send out cargo. To unlock a facility and be able to deliver items for them, first deliver an item there.</li><li>You delivered a total of <b>%d items</b> and earned a total of <b>%0.2f$</b> with deliveries. You can see a more detailled list of delivered items in the Delivery History.</li></ul>',unlockedCount, providingFacilitiesCount, progress.itemsDeliveredTotal or 0, progress.rewardFromAllDeliveries.money or 0 )

  local formattedFacilities = {
    entryId = "deliveryFacilities",
    type = "progress",
    cardTypeLabel = "ui.career.poiCard.generic",
    title = "Delivery Facilities",
    text = facText,
    time = os.time()-2,
    hideInRecent = true,
    tables = {facTable}
  }
  table.insert(list, formattedFacilities)


  local gameplayText = '<span>Below is an overview of all rewards earned from Deliveries.</span><ul><li><b>Money</b> can be used to purchase vehicles, vehicle parts, repairs, towing and more.</li><li><b>Beam XP</b> is a measure of your overall general progress, but has no use in game currently.</li><li><b>Labourer XP</b> accumulates as you progress through labourer branch gameplay challenges.</li><li><b>Delivery XP</b> is a skill-specific points system within the laborer branch, not yet in use, it will be used to track progression and unlock gameplay.</li></ul>'
  local gameplayTable = {
    headers = {'Reason','Change','Time'},
    rows = {}
  }
  for _, change in ipairs(arrayReverse(deepcopy(career_modules_playerAttributes.getAttributeLog()))) do
    if change.reason.delivery then
      local changeText = ""
      for _, key in ipairs(career_branches.orderAttributeKeysByBranchOrder(tableKeys(change.attributeChange))) do
        changeText = changeText .. string.format('<span><b>%s</b>: %s%0.2f</span><br>', key, change.attributeChange[key] > 0 and "+" or "", change.attributeChange[key])
      end
      table.insert(gameplayTable.rows,
        {change.reason.label, changeText, os.date("%c",change.time)}
      )
    end
  end

  local formattedGameplay = {
    entryId = "deliveryHistory",
    type = "progress",
    cardTypeLabel = "ui.career.poiCard.generic",
    title = "Delivery History",
    text = gameplayText,
    time = os.time()-3,
    hideInRecent = true,
    tables = {gameplayTable}
  }
  table.insert(list, formattedGameplay)


  local deliveriesText = "<span>Cargo items can have <strong>modifiers</strong> that will change how you need to handle the cargo. Each modifier will increase the potential rewards you receive upon delivery, but can also reduce rewards if you fail to meet the requirements.</span>"

  -- urgent deliveries
  deliveriesText = deliveriesText.. "<h3>Urgent Cargo</h3><span><strong>Urgent Cargo</strong> has a time limit that starts once you exit the cargo screen after picking it up. Deliver the item <b>before the time runs out</b> for the full reward. After the time runs out, you will have a few more minutes, but the delivery will be considered <i>delayed</i> and your rewards are reduced. Once that time runs out, your delivery is <i>late</i> and you will receive substantially less rewards.</span><br><br><span>This modifier increases the potential rewards by about <b>30%</b>.</span><br><br>"

  if not progress.timedFlag then
    deliveriesText = deliveriesText .. string.format("<span>Urgent Delivieries are still locked. Deliver <b>%d items in total</b> to unlock them. (<b>%d / %d</b>).</span>", dProgress.getModifierRequirements().itemsDeliveredTotalToUnlockTimed, progress.itemsDeliveredTotal, dProgress.getModifierRequirements().itemsDeliveredTotalToUnlockTimed)
  else
    deliveriesText = deliveriesText .. string.format("<span>You have delivered a total of %d Urgent Cargo items. Of those, %d were delivered on time, %d were delayed and %d were late.</span>",progress.timedDeliveries, progress.onTimeDeliveries, progress.delayedDeliveries, progress.lateDeliveries)
  end

  --precious cargo
  deliveriesText = deliveriesText.. "<h3>Precious Cargo</h3><span><strong>Precious Cargo</strong> needs to be handled delicately. <b>Strong acceleration or sharp turns will damage it</b> and you will receive less and less rewards. If the health of the item falls below 90%, it is considered <i>damaged</i> and your rewards will be reduced. If it reaches 0%, it is considered <i>destroyed</i> and you will receive substantially less rewards.</span><br><br><span>This modifier increases the potential rewards by about <b>70%</b>.</span><br><br>"

  if not progress.fragileFlag then
    deliveriesText = deliveriesText .. string.format("<span>Precious Cargo Deliveries are still locked. Deliver <b>%d items with the Urgent Cargo modifier on time</b> to unlock them. (<b>%d / %d</b>)</span>", dProgress.getModifierRequirements().onTimeDeliveriesToUnlockFragile, progress.onTimeDeliveries, dProgress.getModifierRequirements().onTimeDeliveriesToUnlockFragile)
  else
    deliveriesText = deliveriesText .. string.format("<span>You have delivered a total of %d Precious Cargo items. Of those, %d were delivered intact, %d were damaged and %d were destroyed.</span>",progress.fragileDeliveries, progress.noDamageDeliveries, progress.damagedDeliveries, progress.brokenDeliveries)
  end

  -- combo
  deliveriesText = deliveriesText.. "<h3>Urgent and Precious Cargo</h3><span>Cargo items can have multiple modifiers, combining their properties.</span><br><br>"

  if not progress.timedFragileFlag then
    deliveriesText = deliveriesText .. string.format("<span>Urgent and Precious Cargo Deliveries are still locked. Deliver <b>%d items with the Precious Cargo modifier intact</b> to unlock them. (<b>%d / %d</b>)</span>", dProgress.getModifierRequirements().noDamageDeliveriesToUnlockTimedFragile, progress.noDamageDeliveries, dProgress.getModifierRequirements().noDamageDeliveriesToUnlockTimedFragile)
  else
    deliveriesText = deliveriesText .. "<span>Urgent and Precious Cargo is unlocked.</span>"
  end

  local formattedDeliveries = {
    entryId = "deliveryProgress",
    type = "progress",
    cardTypeLabel = "ui.career.poiCard.generic",
    title = "Cargo Modifiers",
    text = deliveriesText,
    time = os.time()-4,
    hideInRecent = true,
    tables = {}
  }

  table.insert(list, formattedDeliveries)
end
--M.onLogbookGetEntries = onLogbookGetEntries


--[[
local function onGetMilestones(list)
  local elem = {
    label = "Delivey Person",
    description = "colelct x" ,
    progress = {{
      type = "progressBar",
      minValue = 0,
      currValue = 5,
      maxValue = 10,
      label = "5 / 10",
      done = false,
    }},
    rewards = {},
    claimable = false,

  }
  table.insert(list, elem)
end
M.onGetMilestones = onGetMilestones
]]

return M
