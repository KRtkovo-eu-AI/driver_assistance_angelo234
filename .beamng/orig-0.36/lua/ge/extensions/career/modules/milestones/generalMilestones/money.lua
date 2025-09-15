-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local milestones
local moneyMilestones = {}
local moneyValues = {}
for _, k in ipairs({1000, 10000, 100000}) do
  for _, i in ipairs({1,2,5}) do
    table.insert(moneyValues, i*k)
  end
end
table.insert(moneyValues,1000000)

local modeConfigs = {
  {
    mode = "gains",
    tags = {"reward"},
    filter = {general=true},
    values = moneyValues,
    label = "Moneymaker",
    description = "Earn %0.2f from rewards.",
  },
  {
    mode = "losses",
    label = "Big Spender",
    tags = {"buying"},
    filter = {general=true},
    values = moneyValues,
    description = "Spend %0.2f on goods and services.",
  },
  {
    mode = "losses",
    label = "New Parts cost Money",
    tags = {"partsBought"},
    filter = {},
    values = {5000,15000,50000},
    description = "Spend %0.2f on vehicle parts.",
  },
  {
    mode = "losses",
    label = "It's an investment!",
    tags = {"vehicleBought"},
    filter = {},
    values = {10000,20000,500000},
    description = "Spend %0.2f on new vehicles.",
  },
  {
    mode = "gains",
    label = "I dont want it anymore",
    tags = {"selling"},
    filter = {},
    values = {5000,15000,50000},
    description = "Sell vehicles and parts worth %0.2f.",
  },

  {
    mode = "gains",
    tags = {"delivery"},
    filter = {},
    values = {5000,15000,50000,100000},
    label = "Delivery Earner",
    description = "Earn %0.2f through deliveries.",
  },
  {
    mode = "losses",
    label = "It's fine.",
    tags = {"fine"},
    filter = {},
    values = {500,1500,5000},
    description = "Lose %0.2f through fines.",
  },
}
M.onGeneralMilestonesCollect = function(milestonesList)
  milestones = career_modules_milestones_milestones

  for _, modeConfig in ipairs(modeConfigs) do
    local mode = modeConfig.mode
    local tags = modeConfig.tags
    local values = modeConfig.values
    local milestoneConfig = {
      id = "money_"..modeConfig.mode.."--"..table.concat(tags,"-"),
      sortIndex = 100,
      mode = modeConfig.mode,
      filter = modeConfig.filter or {},
      icon="beamCurrency",
      color=milestones.colorOrange,
      getValue = function()
        local sum = 0
        for _, tag in ipairs(tags) do
          sum = sum + math.abs(career_modules_playerAttributes.getAttribute("money")[modeConfig.mode][tag] or 0)
        end
        return sum
      end,
      getLabel = function(step, displayValue, target) return modeConfig.label end,
      getDescription = function(step, displayValue, target) return string.format(modeConfig.description, target) end,
      getProgressLabel = function(step, current, target) return string.format("%0.2f / %0.2f", current, target) end,
      getTarget = function(step) return values[step] end,
      getRewards = milestones.minorLinear,
      maxStep = #values
    }
    milestoneConfig.filter.money = true
    milestones.saveData.general[milestoneConfig.id] = milestones.saveData.general[milestoneConfig.id] or {claimedStep = 0, notificationStep = 0}
    table.insert(milestonesList, milestoneConfig)
  end
end

M.onGeneralMilestonesSetupCallbacks = function()
  for _, milestoneConfig in ipairs(moneyMilestones) do
    M.setNotificationTarget(milestoneConfig)
  end
end


-- branch related updates
local function setNotificationTarget(milestoneConfig)
  local step = milestones.saveData.general[milestoneConfig.id].notificationStep +1
  -- check if completed
  if milestoneConfig.maxStep and step > milestoneConfig.maxStep then return end
  local target = milestoneConfig.getTarget(step)
  if target then
    milestoneConfig._target = target
  end
end

local function onPlayerAttributesChanged(change)
  if change.money then
    for _, milestoneConfig in ipairs(moneyMilestones) do
      local step = milestones.saveData.general[milestoneConfig.id].notificationStep +1
      if milestoneConfig._target and milestoneConfig.getValue() >= milestoneConfig._target then
        milestones.milestoneReached(milestoneConfig.getLabel(step))
        milestoneConfig._target = nil
        milestones.saveData.general[milestoneConfig.id].notificationStep = step
        M.setNotificationTarget(milestoneConfig)
      end
    end
  end
end
M.setNotificationTarget = setNotificationTarget
M.onPlayerAttributesChanged = onPlayerAttributesChanged

return M