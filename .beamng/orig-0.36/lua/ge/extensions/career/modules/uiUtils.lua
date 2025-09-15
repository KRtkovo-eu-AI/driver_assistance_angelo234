-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"career_career"}

-- CareerStatus vue component
local function getCareerStatusData()
  local data = {}
  data.money = career_modules_playerAttributes.getAttributeValue("money")
  data.beamXP = career_modules_playerAttributes.getAttributeValue("beamXP")
  data.vouchers = career_modules_playerAttributes.getAttributeValue("vouchers")
  return data
end
M.getCareerStatusData = getCareerStatusData

--CareerSimpleStats vue component
local function getCareerSimpleStats()
  local currentSaveSlot, _ = career_saveSystem.getCurrentSaveSlot()
  local data = {
    saveSlotName = currentSaveSlot,
    branches = {}
  }

  for _, br in pairs(career_branches.getSortedBranches()) do
    if not br.isSkill then
      local branchInfo = {
        name = br.name,
      }
      local attKey = br.attributeKey
      local value = career_modules_playerAttributes.getAttributeValue(attKey)
      local level, _, _, min, max = career_branches.calcBranchLevelFromValue(value, br.id)
      table.insert(data.branches, {
        name = br.name,
        levelLabel = {txt='ui.career.lvlLabel', context={lvl=level}},
        min = min,
        value = value,
        max = max,
      })
    end
  end
  return data
end
M.getCareerSimpleStats = getCareerSimpleStats

-- Career Pause Context Buttons
local careerPauseContextButtonFunctions = {}
local function storeCareerPauseContextButtons(data)
  table.clear(careerPauseContextButtonFunctions)
  for i, btn in ipairs(data.buttons) do
    btn.functionId = i
    careerPauseContextButtonFunctions[i] = btn.fun
  end
end
local function callCareerPauseContextButtons(functionId)
  local fun = careerPauseContextButtonFunctions[functionId]
  if fun then fun() end
end
local function getCareerPauseContextButtons()
  local data = {buttons = {}}

  if career_modules_delivery_general.isDeliveryModeActive() then
    table.insert(data.buttons, {
      label = "Map (My Cargo)",
      icon = "map",
      fun = function() career_modules_delivery_cargoScreen.enterMyCargo() end
    })
  else
    table.insert(data.buttons, {
      label = "Map",
      icon = "map",
      fun = function() freeroam_bigMapMode.enterBigMap({instant=true}) end
    })
  end

  table.insert(data.buttons, {
    label = "Logbook",
    icon = "book",
    fun = function() guihooks.trigger('ChangeState', {state = 'logbook'}) end
  })

  if not career_modules_linearTutorial.isLinearTutorialActive() and career_career.hasBoughtStarterVehicle() then
    table.insert(data.buttons, {
      label = "ui.career.landingPage.name",
      icon = "progress",
      fun = function() guihooks.trigger('ChangeState', {state = 'domainSelection'}) end,
      showIndicator = career_modules_milestones_milestones.unclaimedMilestonesCount() > 0
    })
  end

  if career_modules_vehiclePerformance.isTestInProgress() then
    table.insert(data.buttons, {
      label = "Cancel Certification",
      icon = "cancel",
      fun = function() career_modules_vehiclePerformance.cancelTest() end,
      showIndicator = true
    })
  end

  storeCareerPauseContextButtons(data)
  return data
end
M.storeCareerPauseContextButtons = storeCareerPauseContextButtons
M.callCareerPauseContextButtons = callCareerPauseContextButtons
M.getCareerPauseContextButtons = getCareerPauseContextButtons

-- Career pause Preview Cards

local function getCareerCurrentLevelName()
  for _, lvl in ipairs(core_levels.getList()) do
    if string.lower(lvl.levelName) == getCurrentLevelIdentifier() then
      return lvl
    end
  end
end
M.getCareerCurrentLevelName = getCareerCurrentLevelName


return M