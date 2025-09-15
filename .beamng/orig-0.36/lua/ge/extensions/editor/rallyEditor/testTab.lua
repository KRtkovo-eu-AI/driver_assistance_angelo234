-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
C.windowDescription = 'Test'

function C:init(rallyEditor)
  self.path = nil
  self.rallyEditor = rallyEditor

  self.driveline = nil
end

function C:setPath(path)
  self.path = path
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end

  local missionDir = self.path:getMissionDir()
  self.driveline = require('/lua/ge/extensions/gameplay/rally/driveline')(missionDir)
  if not self.driveline:load() then
    self.driveline = nil
    return
  end

  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

-- called by RallyEditor when this tab is unselected.
function C:unselect()
  if not self.path then return end
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

function C:draw()
  if not self.path then return end

  if im.Button("Test 1") then
    self:test1()
  end

  if im.Button("Load Recce Mission") then
    local mid = self.path:missionId()
    local missionDir = self.path:getMissionDir()
    extensions.unload("gameplay_rally_recceApp")

    extensions.load("gameplay_rally_recceApp")
    gameplay_rally_recceApp.loadMission(mid, missionDir)
    gameplay_rally_recceApp.setDrawDebug(true)
  end

  if im.Button("Unload Recce Mission") then
    extensions.unload("gameplay_rally_recceApp")
  end
end

function C:test1()
  print('-- test1 --------------------------------------------------------')

  local pnName = self.path:getRandomSystemPacenote('firstnoteintro')
  print(tostring(pnName))

  pnName = self.path:getRandomSystemPacenote('firstnoteoutro')
  print(tostring(pnName))

  pnName = self.path:getRandomSystemPacenote('finish')
  print(tostring(pnName))
end

function C:drawDebugEntrypoint()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end


