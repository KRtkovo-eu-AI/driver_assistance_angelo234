-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}
C.moduleOrder = 0 -- low first, high later
C.hooks = {"onReplayStateChanged", "onReplayRecordingValueRequested", "onMissionAutoReplayRecordingSettingChanged", "onMissionAttemptAggregated"}
local dir

local function invertList(x)
  local n, m = #x, #x/2
  for i=1, m do
    x[i], x[n-i+1] = x[n-i+1], x[i]
  end
  return x
end

function C:init()
  self.replayRecording = false
  self.currentReplayFile = nil
end

function C:executionStopped()
  self.stopIfRec()
end

function C:executionStarted()
  dir = core_replay.getMissionReplaysPath()
end

function C:getMissionReplayFiles()
  local orderedReplays = {}
  local result = {}
  for i, file in ipairs(FS:findFiles(dir, '*.rpl', 0, false, false)) do
    table.insert(orderedReplays, {date = FS:stat(file).createtime, file = file})
  end
  table.sort(orderedReplays, function(a,b) return a.date < b.date end)
  for i, file in ipairs(orderedReplays) do
    table.insert(result, file.file)
  end
  return result
end

function C:deleteReplayFile(file)
  FS:removeFile(file)
end

function C:deleteByCount()
  local files = self:getMissionReplayFiles()
  local diff = #files - settings.getValue('countReplayCapMode')
  if diff > 0 then
    for i = 1, diff + 1, 1 do
      self:deleteReplayFile(files[i])
    end
  end
end

function C:deleteByMaxSize()
  local files = self:getMissionReplayFiles()
  local totalSize = 0
  for i, file in ipairs(files) do
    totalSize = totalSize + getFileSize(file) / 1000000
  end
  for i, file in ipairs(files) do
    if totalSize < settings.getValue('maxSizeReplayCapMode') then return end

    totalSize = totalSize - getFileSize(file) / 1000000
    self:deleteReplayFile(file)
  end
end

function C:deleteOlderReplays()
  if settings.getValue('enableMissionReplayCapModes') == "maxSize" then
    self:deleteByMaxSize()
  elseif settings.getValue('enableMissionReplayCapModes') == "count" then
    self:deleteByCount()
  end
end

function C:stopIfRec()
  if core_replay.getState() == "recording" then
    core_replay.toggleMissionRecording()
  end
end

function C:startNewRec()
  self:stopIfRec()
  self:deleteOlderReplays()

  if core_replay.getState() == "inactive" and self.replayRecording then
    self.currentReplayFile = core_replay.toggleMissionRecording()
  end
end

function C:buildMetaInfo(attempt, mission, progressKey)
  if self.currentReplayFile == nil then
    return
  end

  local currentSaveSlot, _ = career_saveSystem.getCurrentSaveSlot()
  local dir, fn, ext = path.split(self.currentReplayFile)
  local metaFileName = dir .. fn .. ".rplMeta.json"
  local metaInfo = {
    missionId = self.mgr.activity.id,
    missionName = self.mgr.activity.name,
    time = os.time(),
    humanTime = os.date("%Y-%m-%d %H:%M:%S"),
    context = {
      isInCareer = career_career and career_career.isActive(),
      saveSlot = career_career and career_career.isActive() and currentSaveSlot or nil,
    }
  }

  if attempt ~= nil and progressKey ~= nil then
    metaInfo.attempt = {
      progressKey = progressKey,
      attemptNumber = attempt.attemptNumber,
    }
  end

  jsonWriteFile(metaFileName, metaInfo)
end

function C:saveMetaInfoWithoutAttempt()
  self:buildMetaInfo()
  self.currentReplayFile = nil
end

function C:onMissionAttemptAggregated(attempt, mission, progressKey)
  self:buildMetaInfo(attempt, mission, progressKey)
  self.currentReplayFile = nil
end

function C:onReplayRecordingValueRequested()
  guihooks.trigger("onReplayRecordingValueRequested", self.replayRecording)
end

function C:onMissionAutoReplayRecordingSettingChanged(newValue)
  self.replayRecording = newValue
end


local originalCamMode = nil
local originalPathId = nil
local originalPathLoop = nil
function C:missionPlaybackStarted()
  originalCamMode = core_camera.getActiveCamName()
  if core_camera.getActiveCamName() == "path" then
    originalPathId = self.mgr.modules.camera.activePathId
    originalPathLoop = self.mgr.modules.camera.loopActivePath
  end

  core_camera.setByName(0, "orbit")

  core_jobsystem.create(function(job)
    job.sleep(0.1)
    simTimeAuthority.pause(false)
  end)
end

function C:missionPlaybackEnded()
  if originalCamMode == "path" then
    self.mgr.modules.camera:startPath(originalPathId, originalPathLoop)
  else
    core_camera.setByName(0, originalCamMode)
  end
end

-- this is to handle the case where the playback is played in the start screen with a camera path that needs to be played again when stopping the playback
local lastFrameState = nil
local currentMissionReplayFiles = nil
function C:onReplayStateChanged(state)
  if lastFrameState ~= "playback" and state.state == "playback" then
    currentMissionReplayFiles = core_replay.getMissionReplayFiles(self.mgr.activity)

    -- Check if current replay file exists in mission replays
    local replayFileFound = false
    for _, replayInfo in ipairs(currentMissionReplayFiles) do
      if replayInfo.replayFile == state.loadedFile then
        self:missionPlaybackStarted()
        break
      end
    end
  elseif lastFrameState == "playback" and lastFrameState ~= nil and state.state == "inactive" then
    self:missionPlaybackEnded()
  end
  lastFrameState = state.state
end

return _flowgraph_createModule(C)