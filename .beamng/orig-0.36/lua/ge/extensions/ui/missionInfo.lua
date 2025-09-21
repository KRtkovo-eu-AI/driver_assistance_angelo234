-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'MissionInfo'
M.buttonsTable = nil
M.openState = nil

M.performAction = function(actionName)
  -- log('I', logTag, tostring(actionName) .. " action triggered. Looking for " .. tostring(actionName) .. " action in "..dumps(M.buttonsTable))
  if M.buttonsTable then
    for i,button in ipairs(M.buttonsTable) do
      if button.action == actionName then
        loadstring(button.cmd)()
        return
      end
    end
  end
end

M.openDialogue = function(content)
  content = content or {}
  -- do not push the actionmap if content says so
  if content.actionMap ~= false then
    local am = scenetree.findObject("MissionUIActionMap")
    if am then am:push() end
  end
  M.buttonsTable = content.buttons or {}
  guihooks.trigger('MissionInfoUpdate', content)
  Engine.Audio.playOnce('AudioGui','event:>UI>Missions>Info_Open')
end


M.performActivityAction = function(id)
  log('I', logTag, tostring(actionName) .. " action triggered. Looking for " .. tostring(actionName) .. " action in "..dumps(M.buttonsTable))
  if M.buttonsTable then
    (M.buttonsTable[id] or nop)()
  end
end

M.openActivityAcceptDialogue = function(content)
  content = content or {}
  -- do not push the actionmap if content says so
  --if content.actionMap ~= false then
  --  local am = scenetree.findObject("MissionUIActionMap")
  --  if am then am:push() end
  --end
  M.buttonsTable = {}
  for i, elem in ipairs(content) do
    M.buttonsTable[i] = elem.buttonFun
    elem.missionInfoPerformActionIndex = i
  end
  extensions.hook('onActivityAcceptUpdate', content)
  guihooks.trigger('ActivityAcceptUpdate', content)
  Engine.Audio.playOnce('AudioGui','event:>UI>Missions>Info_Open')

  local oldState = M.openState
  M.openState = "opened"
  extensions.hook('onMissionInfoChangedState', oldState, M.openState, content)
end

M.closeDialogue = function()
  local am = scenetree.findObject("MissionUIActionMap")
  if am then am:pop() end
  M.buttonsTable = nil
  guihooks.trigger('MissionInfoUpdate', nil)

  local oldState = M.openState
  M.openState = "closed"
  extensions.hook('onMissionInfoChangedState', oldState, M.openState)
end

return M
