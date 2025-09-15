-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im  = ui_imgui
local detailedTimes = false
local C = {}
C.windowDescription = 'General'

function C:init(raceEditor)
  self.raceEditor = raceEditor
  self.startIndex = -1
  self.state = 'setup'
end

function C:setPath(path)
  self.path = path
end

function C:setupRace()
  local oldLap = self.race and self.race.lapCount or 1
  self.race = require('/lua/ge/extensions/gameplay/race/race')()
  self.race:setPath(self.path)
  self.race.lapCount = oldLap
  self.race.useDebugDraw = true
  self.race:setVehicleIds({be:getPlayerVehicleID(0)})
  self.state = 'setup'
end

function C:startRace(vehIds, enableAi, rollingStart)
  vehIds = vehIds or {be:getPlayerVehicleID(0)}

  self.race.path.config.rollingStart = rollingStart and true or false
  self.race:setVehicleIds(vehIds)
  self.race:startRace()
  self.state = 'race'

  if enableAi then
    for _, id in ipairs(vehIds) do
      self.race:startAiVehicle(id, 0.8)
    end
  end
end

function C:draw(dt)
  if self.state == 'setup' then
    self:drawSetup()
  elseif self.state == 'race' then
    self.race:onUpdate(dt)
    self:drawRace()
  elseif self.state == 'stopped' then
    self:drawStopped()
  end
end

function C:drawSetup()
  local lapCount = im.IntPtr(self.race.lapCount or 1)
  im.PushItemWidth(im.GetContentRegionAvailWidth() * 0.5)
  if im.InputInt("Lap Count", lapCount) then
    self.race.lapCount = math.max(1, lapCount[0])
  end
  im.PopItemWidth()

  for i, sp in ipairs(self.path.startPositions.sorted) do
    if im.SmallButton("Move to " .. sp.name) then
      sp:moveResetVehicleTo(be:getPlayerVehicleID(0))
    end
  end

  if im.Button("Start") then
    editor.setEditorActive(false)
    self:startRace(nil, false, false)
  end
  if im.Button("Start Rolling") then
    editor.setEditorActive(false)
    self:startRace(nil, false, true)
  end

  im.Separator()

  if im.Button("Move All Vehicles to Starting Positions") then
    for i, veh in ipairs(getAllVehiclesByType()) do
      local sp = self.path.startPositions.sorted[i]
      if sp and not sp.missing then
        sp:moveResetVehicleTo(veh:getId())
      end
    end
  end

  if im.Button("AI Drive Test Current Vehicle") then
    self:startRace(nil, true, false)
  end

  if im.Button("AI Drive Test All Vehicles") then
    local vehIds = {}

    for _, veh in ipairs(getAllVehiclesByType()) do
      if self.path.defaultStartPosition and not self.path.startPositions.objects[self.path.defaultStartPosition].missing then
        if self.path.startPositions.objects[self.path.defaultStartPosition].pos:squaredDistance(veh:getPosition()) <= 10000 then
          table.insert(vehIds, veh:getId())
        end
      else
        table.insert(vehIds, veh:getId())
      end
    end

    self:startRace(vehIds, true, false)
  end
end

function C:drawRace(dt)
  if im.Button("Stop") then
    for _, id in ipairs(self.race.vehIds) do
      self.race:abortRace(id)
    end
    editor.setEditorActive(true)
    self.state = 'stopped'
    self.raceEditor.show()
  end
  im.SameLine()
  if im.Button("State") then
    dump(self.race.states[self.race.vehIds[1]])
  end
  im.SameLine()
  if im.Button("Recover") then
    self.race:requestRecover(self.race.vehIds[1])
  end
  self:drawTimes()
  self:drawEventLog()
end

function C:drawStopped()
  if im.Button("Restart") then
    self:setupRace()
    self.state = 'setup'
  end
  self:drawEventLog()
  self:drawTimes()
end

function C:drawTimes()
  if not self.race.vehIds[1] or not self.race.states[self.race.vehIds[1]] then return end

  local avail = im.GetContentRegionAvail()
  im.BeginChild1("Times", im.ImVec2(avail.x, avail.y/2-5), 0, im.WindowFlags_AlwaysVerticalScrollbar)

  if self.race:inDrawTimes(self.race.vehIds[1], im, detailedTimes) then
    detailedTimes = not detailedTimes
  end
  im.EndChild()
end

function C:drawEventLog()
  if not self.race.vehIds[1] or not self.race.states[self.race.vehIds[1]] then return end

  local avail = im.GetContentRegionAvail()
  im.BeginChild1("EventLog", im.ImVec2(avail.x, avail.y-5), 0, im.WindowFlags_AlwaysVerticalScrollbar)
  self.race:inDrawEventlog(self.race.vehIds[1], im)
  im.EndChild()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
