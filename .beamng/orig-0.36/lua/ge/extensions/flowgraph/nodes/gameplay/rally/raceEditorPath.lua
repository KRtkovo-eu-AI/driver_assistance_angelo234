-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local RallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}

C.name = 'Rally Mode Race Editor Path'
C.description = 'Returns the appropriate race.json file path, either from the raceEditor, or the mission.'
C.category = 'once_instant'
C.color = RallyUtil.rally_flowgraph_color
C.tags = {'rally'}

C.pinSchema = {
  {dir = 'out', type = 'string', name = 'fname', description = 'The race filename.'},
}

local function loadCurrentRaceFileState()
  local missionManagerMissionId = RallyUtil.detectMissionManagerMissionId()
  if missionManagerMissionId then
    -- log('D', '', string.format('RallyMode using missionManager race file missionId=%s raceFileFname=%s raceFileSource=%s', missionManagerMissionId, raceFileFname, raceFileSource))
    return 'race.race.json', 'Mission'
  elseif editor_raceEditor then
    local fname = editor_raceEditor.getCurrentFilename()
    if fname and fname ~= '/gameplay/races/new.path.json' and fname ~= '/gameplay/races/NewRace.race.json' then
      -- log('D', '', string.format('RallyMode using editor race file fname=%s raceFileFname=%s raceFileSource=%s', fname, fname, 'Editor'))
      return fname, 'Editor'
    end
  else
    return nil, nil
  end
end

-- function C:_executionStarted()
  -- self:loadCurrentRaceFileState()
-- end

-- function C:onNodeReset()
  -- self:loadCurrentRaceFileState()
-- end

function C:drawMiddle(builder, style)
  local raceFileFname, raceFileSource = loadCurrentRaceFileState()
  builder:Middle()
  im.Text("Source: " .. tostring(raceFileSource))
end

function C:drawCustomProperties()
  local raceFileFname, raceFileSource = loadCurrentRaceFileState()
  im.Text("Race file:")
  im.Text(tostring(raceFileFname))
  im.Text("Source: " .. tostring(raceFileSource))
end

function C:workOnce()
  local raceFileFname, raceFileSource = loadCurrentRaceFileState()
  log('D', 'raceEditorPath.workOnce', string.format('RallyMode using race file from %s: %s', raceFileSource, raceFileFname))
  self.pinOut.fname.value = raceFileFname
end

return _flowgraph_createNode(C)
