-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui
local DrivelineRoute = require('/lua/ge/extensions/gameplay/rally/driveline/drivelineRoute')
local TextCompositor = require('/lua/ge/extensions/gameplay/rally/notebook/structured/textCompositor')

local logTag = ''

local C = {}

function C:init()
  -- self:_reset()
  self:loadCompositors()
end

-- function C:_reset()
  -- self.reachedPacenotes = {}
  -- self.selectedWaypoint = nil
  -- self.drivelineRoute = DrivelineRoute()
-- end

-- function C:loadRoute()
--   if not editor_raceEditor then
--     log('E', 'Rally Debug', 'No race editor found')
--     return
--   end

--   local racePath = editor_raceEditor.getCurrentPath()
--   if not racePath then
--     log('E', 'Rally Debug', 'No race path found')
--     return
--   end

--   if not editor_rallyEditor then
--     log('E', 'Rally Debug', 'No rally editor found')
--     return
--   end

--   local notebookPath = editor_rallyEditor.getCurrentPath()
--   if not notebookPath then
--     log('E', 'Rally Debug', 'No notebook path found')
--     return
--   end

--   if self.drivelineRoute then
--     self.drivelineRoute:loadRoute(racePath, notebookPath)
--   else
--     log('E', 'Rally Debug', 'No driveline route found')
--   end
-- end

function C:loadCompositors()
  local compositorFiles = FS:findFiles("/lua/ge/extensions/gameplay/rally/compositors/", "compositor.lua", -1, true, false)
  local compositors = {}

  for _, file in ipairs(compositorFiles) do
    -- Extract compositor name from path
    local compositorName = string.match(file, "/compositors/([^/]+)/compositor%.lua$")
    if compositorName then
      table.insert(compositors, compositorName)
    end
  end

  table.sort(compositors)
  log('I', logTag, 'loaded textCompositors: ' .. dumps(compositors))
  self.compositors = compositors
end

function C:enumerate()
  if not self.selectedCompositor then
    log('E', logTag, 'No text compositor selected')
    return
  end
  local selectedCompositor = self.selectedCompositor
  local compositor = TextCompositor(selectedCompositor)
  if not compositor:load() then
    log('E', logTag, 'failed to load text compositor')
  end
  local enumerated = compositor:enumerateAll()
  compositor:writeEnumerated(self:enumeratorOutFname(), enumerated)
end

function C:enumeratorOutFname()
  if not self.selectedCompositor then return '' end
  return string.format('/temp/rally/enumerated_%s.json', self.selectedCompositor)
end

function C:drawGameSettings()
  im.Text('rallyTextCompositorVoice='..settings.getValue("rallyTextCompositorVoice"))
end

function C:draw(dt)
  local collapaseFlags = im.TreeNodeFlags_DefaultClosed

  if im.CollapsingHeader1("Enumerate Text Compositors", im.TreeNodeFlags_DefaultOpen) then
    if not self.selectedCompositor then
      -- Initialize with the first compositor
      self.selectedCompositor = self.compositors and self.compositors[1]
    end

    im.Text("Text Compositor")
    im.SameLine()

    im.SetNextItemWidth(100)
    if im.BeginCombo("##CompositorSelector", self.selectedCompositor or "None") then
      if self.compositors then
        for _, compositorName in ipairs(self.compositors) do
          if im.Selectable1(compositorName, compositorName == self.selectedCompositor) then
            self.selectedCompositor = compositorName
          end
        end
      else
        im.Text("No compositors loaded")
      end
      im.EndCombo()
    end
    im.SameLine()
    if im.Button("Enumerate") then
      self:enumerate()
    end
    im.Text("Output file: " .. self:enumeratorOutFname())
  end

  -- im.Separator()

  if im.CollapsingHeader1("Game Settings", collapaseFlags) then
    self:drawGameSettings()
  end

  -- if im.Button("Load Route") then
  --   self:loadRoute()
  -- end
  -- if self.drivelineRoute then
  --   im.Text(string.format("Next Pacenote Idx: %d", self.drivelineRoute.nextPacenoteIdx))

  --   local nextPacenote = self.drivelineRoute.pacenotes[self.drivelineRoute.nextPacenoteIdx]
  --   if nextPacenote then
  --     im.Text(string.format("Next Pacenote: %s(%d) length=%0.1f", nextPacenote.name, nextPacenote.id, nextPacenote:getCachedLength() or 0.0))
  --   else
  --     im.Text("Next Pacenote: none")
  --   end

  --   if im.BeginTable("eventLog", 1, im.TableFlags_Borders) then
  --     im.TableSetupColumn("Event")
  --     im.TableHeadersRow()

  --     for i = #self.drivelineRoute.eventLog, 1, -1 do
  --       local event = self.drivelineRoute.eventLog[i]
  --       im.TableNextRow()
  --       im.TableNextColumn()
  --       im.Text(event:gsub("%%", "%%%%"))
  --     end

  --     im.EndTable()
  --   end

  --   self.drivelineRoute:onUpdate()
  --   self.drivelineRoute:drawDebugDrivelineRoute()
  -- end
end

function C:onVehicleResetted()
  -- self:loadRoute()
  -- self.drivelineRoute:onVehicleResetted()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
