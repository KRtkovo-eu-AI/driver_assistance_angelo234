-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local eventName
local eventVolume
local triggerElectrics
local breakgroups
local couplers
local hasTriggered = false
local eventNodeId
local uiName

local function trigger()
  sounds.playSoundOnceFollowNode(eventName, eventNodeId, eventVolume, 1, 1, 1)
  for _, breakgroup in ipairs(breakgroups) do
    beamstate.breakBreakGroup(breakgroup)
  end

  for _, coupler in ipairs(couplers) do
    beamstate.detachCouplers(coupler, true, true)
  end

  guihooks.message("Pyrotechnic charge triggered: " .. uiName, 5, "vehicle.pyrotechnicCharge." .. uiName)
end

local function updateGFX(dt)
  if not hasTriggered then
    local doTrigger = false
    for _, triggerElectric in ipairs(triggerElectrics) do
      if electrics.values[triggerElectric] == 1 or electrics.values[triggerElectric] == true then
        doTrigger = true
      end
    end

    if doTrigger then
      trigger()
      hasTriggered = true
    end
  end
end

local function reset()
  hasTriggered = false
end

local function init(jbeamData)
  hasTriggered = false

  triggerElectrics = jbeamData.triggerElectrics or {}

  breakgroups = jbeamData.breakgroups or {}
  couplers = jbeamData.couplers or {}

  eventName = jbeamData.eventName or "event:>Vehicle>Failures>pyrotechnic_charge"
  eventVolume = jbeamData.eventVolume or 1
  uiName = jbeamData.uiName or ""

  local eventNodeName = jbeamData.eventNode
  eventNodeId = 0
  if eventNodeName then
    for cid, node in pairs(v.data.nodes) do
      if node.name == eventNodeName then
        eventNodeId = cid
        break
      end
    end
  end

  if eventNodeId then
    bdebug.setNodeDebugText("Pyrotechnic Charges", eventNodeId, M.name .. ": " .. (eventName or "no event"))
  end
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.trigger = trigger

return M
