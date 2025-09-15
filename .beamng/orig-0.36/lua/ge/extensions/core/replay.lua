-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local min = math.min
local max = math.max

local M = { state = {} }
M.state.speed = 1 -- playback speed indicator
local speeds = {1/1000, 1/500, 1/200, 1/100, 1/50, 1/32, 1/16, 1/8, 1/4, 1/2, 3/4, 1.0, 1.5, 2, 4, 8}
local missionReplayRecording = false
local missionReplaysPath = "replays/missionReplays/"
local userSavedMissionReplays = "replays/userSavedMissionReplays/"

local function getFileStream()
  if not be then return end
  return be:getFileStream()
end


local function onInit()
  local stream = getFileStream()
  if stream then
    stream:requestState()
  end
end

local function getRecordings()
  local result = {}
  for i,file in ipairs(FS:findFiles('replays', '*.rpl', 1, false, false)) do
    file = string.gsub(file, "/(.*)", "%1") -- strip leading /
    table.insert(result, {filename=file, size=FS:fileSize(file)})
  end
  return result
end

local function stateChanged(loadedFile, positionSeconds, totalSeconds, speed, paused, fpsPlay, fpsRec, statestr, framePositionSeconds)
  if M.state.state ~= statestr then
    if statestr == 'playback' then -- we are in playback now
      local o = scenetree.findObject("VehicleCommonActionMap")
      if o then o:setEnabled(false) end
      o = scenetree.findObject("VehicleSpecificActionMap")
      if o then o:setEnabled(false) end
      o = scenetree.findObject("ReplayPlaybackActionMap")
      if o then o:push() end
    else -- we are not in playback now (start of game, just exited playback, etc)
      local o = scenetree.findObject("ReplayPlaybackActionMap")
      if o then o:pop() end
      o = scenetree.findObject("VehicleSpecificActionMap")
      if o then o:setEnabled(true) end
      o = scenetree.findObject("VehicleCommonActionMap")
      if o then o:setEnabled(true) end
    end
  end
  -- speed: we lose some precission on the way to C++ and back, round it a bit
  M.state = {loadedFile = loadedFile, positionSeconds = positionSeconds, totalSeconds = totalSeconds, speed = round(speed*1000)/1000, paused = paused, fpsPlay = fpsPlay, fpsRec = fpsRec, state = statestr, framePositionSeconds = framePositionSeconds}
  guihooks.trigger('replayStateChanged', M.state)
  extensions.hook("onReplayStateChanged", M.state)
end

local function getPositionSeconds()
  return M.state.positionSeconds
end

local function getTotalSeconds()
  return M.state.totalSeconds
end

local function getState()
  return M.state.state
end

local function isPaused()
  return M.state.paused
end

local function getLoadedFile()
  return M.state.loadedFile
end

local function setSpeed(speed)
  local stream = getFileStream()
  if not stream then return end
  if M.state.speed ~= speed then
    stream:setSpeed(speed)
  end
end

local togglingSpeed = 1/8
local function toggleSpeed(val)
  local newSpeed = M.state.speed
  if val == "realtime" then
    if M.state.speed == 1 then
      newSpeed = togglingSpeed
    else
      togglingSpeed = M.state.speed
      newSpeed = 1
    end
  elseif val == "slowmotion" then
    newSpeed = 1/8
  else
    local speedId = -1
    for i,speed in ipairs(speeds) do
      if speed == M.state.speed then
        speedId = i
        break
      end
    end
    if speedId == -1 and val < 0 then
      for i=#speeds,1,-1 do
        if speeds[i] <= M.state.speed then
          speedId = i
          break
        end
      end
    end
    if speedId == -1 and val > 0 then
      for i,speed in ipairs(speeds) do
        if speed >= M.state.speed then
          speedId = i
          break
        end
      end
    end
    speedId = min(#speeds, max(1, speedId+val))
    newSpeed = speeds[speedId]
  end
  setSpeed(newSpeed)
  simTimeAuthority.reportSpeed(newSpeed)
end

local function pause(v)
  local stream = getFileStream()
  if not stream then return end

  if M.state.state ~= 'playback' then return end
  stream:setPaused(v)
end

local function displayMsg(level, msg, context)
  -- level is a toastr category name ("error", "info", "warning"...)
  guihooks.trigger("toastrMsg", {type=level, title="Replay "..level, msg=msg, context=context})
  log(string.gsub(level, "^(.).*", string.upper), "", "Replay msg: "..dumps(level, msg, context))
end

local function togglePlay()
  if not M.state.loadedFile or M.state.loadedFile == "" then
    return
  end
  local stream = getFileStream()
  if not stream then return end

  if M.state.state == 'inactive' then
    stream:setPaused(false)
    local ret = stream:play(M.state.loadedFile)
    if ret ~= 0 then displayMsg("error", "replay.playError", {filename=M.state.loadedFile}) end
  elseif M.state.state == 'playback' then
    stream:setPaused(not M.state.paused)
  else
    log("E","",'Will not toggle play from state: '..dumps(M.state.state))
  end
end

local function loadFile(filename)
  local stream = getFileStream()
  if not stream then return end
  log("D","", "Loading: "..filename)

  stream:stop()
  stream:setPaused(true)
  local ret = stream:play(filename)
  if ret ~= 0 then displayMsg("error", "replay.playError", {filename=filename}) end
end

local function stop()
  local stream = getFileStream()
  if not stream then return end

  log("D","", 'Stopping from state: '..M.state.state);
  stream:stop()
end

local function cancelRecording()
  local stream = getFileStream()
  if not stream then return end

  log("D","",'Cancelling recording from state: '..M.state.state)
  if M.state.state == 'recording' then
    ui_message("replay.cancelRecording", 5, "replay", "local_movies")
    local file = M.state.loadedFile
    stream:stop()
    FS:removeFile(file)
  end
end

local function toggleRecording(autoplayAfterStopping, isMissionReplay)
  isMissionReplay = isMissionReplay or false

  local stream = getFileStream()
  if not stream then return end
  log("D","",'Toggle recording from state: '..M.state.state)

  if M.state.state == 'recording' then
    if not isMissionReplay and missionReplayRecording then
      ui_message("replay.cantManualRecording", 5, "replay", "local_movies")
      return
    end
    if isMissionReplay then missionReplayRecording = false end
    if autoplayAfterStopping then
      ui_message(isMissionReplay and "stopAutoReplayRecordingAutoplay" or "replay.stopRecordingAutoplay", 5, "replay", "local_movies")
      loadFile(M.state.loadedFile)
    else
      ui_message(isMissionReplay and "stopAutoReplayRecording" or "replay.stopRecording", 5, "replay", "local_movies")
      stream:stop()
    end
  elseif M.state.state == 'playback' then
    stop()
  else
    if not isMissionReplay and missionReplayRecording then
      ui_message("replay.cantManualRecording", 5, "replay", "local_movies")
      return
    end

    local date = os.date("%Y-%m-%d_%H-%M-%S")

    local map = core_levels.getLevelName(getMissionFilename())

    if map == nil then
      log("E", "", "Cannot start recording replay. Map filename: "..dumps(getMissionFilename()))
    else
      if isMissionReplay then missionReplayRecording = true end
      local dir = isMissionReplay and missionReplaysPath or "replays/"
      local filename = dir..date.." "..map..".rpl"
      log("D","",'record to: '..filename)
      ui_message(isMissionReplay and "replay.startRecordingAutoReplay" or "replay.startRecording", 5, "replay", "local_movies")
      stream:record(filename)
      return filename
    end
  end
end

local function toggleMissionRecording()
  return toggleRecording(false, true)
end

local function seek(percent)
  local stream = getFileStream()
  if not stream then return end

  if M.state.state ~= 'playback' then return end
  stream:seek(max(0, min(1, percent)))
end

local function jumpFrames(offset)
  --dump{'jumpFrames', offset}
  local stream = getFileStream()
  if not stream then return end
  stream:stepFrames(offset)
  ui_message({txt="replay.jumpFrames", context={frameCount=offset}}, 2, "replay", "local_movies")
end

local function jumpTime(timeDiffInSeconds)
  --dump{'jumpTime', timeDiffInSeconds}
  local stream = getFileStream()
  if not stream then return end
  stream:stepTime(timeDiffInSeconds)
  ui_message({txt="replay.jump", context={seconds=timeDiffInSeconds}}, 2, "replay", "local_movies")
end

local function openReplayFolderInExplorer()
  if not fileExistsOrNil('/replays/') then  -- create dir if it doesnt exist
    FS:directoryCreate("/replay/", true)
  end
  Engine.Platform.exploreFolder("/replays/")
end

local function onClientEndMission(levelPath)
  if M.state.state == 'playback' and not M.requestedStartLevel then
    log("I", "", string.format("Stopping replay playback. Reason: level changed from \"%s\" to \"%s\"", getLoadedFile(), levelPath))
    displayMsg("info", "replay.stopPlayback")
    stop()
  end
  M.requestedStartLevel = nil
end

-- TODO:
--local function onDrawDebug(lastDebugFocusPos, dtReal, dtSim, dtRaw)
  -- 1) get the current replay frame (current VehicleState)
  --    > getCurrentStateReplayNodeCount(vehId)
  --    > getCurrentStateReplayNodePosition(vehId, x)
  -- local vdata = extensions.core_vehicle_manager.getVehicleData(vid).vdata
  -- -> iterate through the nodes and beams and debug draw them
  -- 2) figure out the bdebug mode? beams? nodes?
  -- 3) custom draw the things
--end

local function startLevel(levelPath)
  M.requestedStartLevel = true
  local levelName = core_levels.getLevelName(levelPath)
  local spawnVehicle = false  -- don't spawn a vehicle by default
  freeroam_freeroam.startFreeroamByName(levelName, nil, nil, spawnVehicle)  -- don't spawn a vehicle by default
end

local function acceptRename(oldFilename, newFilename)
  be:getFileStream():stop()
  FS:renameFile(oldFilename, newFilename)
  FS:removeFile(oldFilename)
  loadFile(newFilename)
end

local function getCurrentUserSavedReplayFilesPath()
  local finalPath = userSavedMissionReplays

  -- save the replay in the proper folder
  if career_career and career_career.isActive() then
    local currentSaveSlot, _ = career_saveSystem.getCurrentSaveSlot()
    finalPath = finalPath .. "career/" .. currentSaveSlot .. "/"
  else -- we are in freeroam
    finalPath = finalPath .. "freeroam/"
  end

  return finalPath
end

local function getMissionReplayFiles(mission, returnOnlyWithAttempt)
  local files = {}
  local filesWithUserSaved = {}

  if returnOnlyWithAttempt == nil then
    returnOnlyWithAttempt = false
  end

  -- add all the missions replays that have been automatically recorded
  for i, file in ipairs(FS:findFiles(M.getMissionReplaysPath(), '*.rpl', 0, false, false)) do
    table.insert(filesWithUserSaved, {file = file})
  end
  -- add all the user saved replays that correspong to the current environment. Ie freeroam or career, save slot
  for i, file in ipairs(FS:findFiles(getCurrentUserSavedReplayFilesPath(), '*.rpl', 0, false, false)) do
    table.insert(filesWithUserSaved, {file = file, userSaved = true})
  end

  local currentSaveSlot, _ = career_saveSystem.getCurrentSaveSlot()
  local isCurrentlyInCareer = career_career and career_career.isActive()

  local ret = {}
  for _, file in ipairs(filesWithUserSaved) do
    local dir, fn, ext = path.split(file.file)
    local metaFileName = dir .. fn .. ".rplMeta.json"
    local meta = jsonReadFile(metaFileName)
    if meta and meta.missionId == mission.id and ((returnOnlyWithAttempt and meta.attempt) or not returnOnlyWithAttempt) and ((isCurrentlyInCareer and meta.context.saveSlot == currentSaveSlot) or not isCurrentlyInCareer) then
      table.insert(ret, {
        replayFile = file.file,
        replayFileName = fn,
        meta = meta,
        userSaved = file.userSaved
      })
    end
  end


  table.sort(ret, function(a,b) return a.meta.time < b.meta.time end)
  return ret
end

local function saveMissionReplay(replayFileName)
  local finalPath = getCurrentUserSavedReplayFilesPath()

  if not FS:directoryExists(finalPath) then FS:directoryCreate(finalPath) end
  local dir, fn, ext = path.split(replayFileName)

  FS:copyFile(replayFileName, finalPath..fn)
  FS:copyFile(replayFileName..".rplMeta.json", finalPath..fn..".rplMeta.json")

  FS:removeFile(replayFileName)
  FS:removeFile(replayFileName..".rplMeta.json")

  local meta = jsonReadFile(finalPath..fn..".rplMeta.json")
  local recordingFiles = getMissionReplayFiles(gameplay_missions_missions.getMissionById(meta.missionId))
  guihooks.trigger("recordingFilesUpdated", recordingFiles)
end

local function removeMissionSavedReplay(replayFileName)
  local dir, fn, ext = path.split(replayFileName)

  if not FS:directoryExists(missionReplaysPath) then FS:directoryCreate(missionReplaysPath) end

  FS:copyFile(replayFileName, missionReplaysPath..fn)
  FS:copyFile(replayFileName..".rplMeta.json", missionReplaysPath..fn..".rplMeta.json")

  FS:removeFile(replayFileName)
  FS:removeFile(replayFileName..".rplMeta.json")

  local meta = jsonReadFile(missionReplaysPath..fn..".rplMeta.json")
  local recordingFiles = getMissionReplayFiles(gameplay_missions_missions.getMissionById(meta.missionId))
  guihooks.trigger("recordingFilesUpdated", recordingFiles)
end

-- public interface
M.onInit = onInit
M.onClientEndMission = onClientEndMission
M.startLevel = startLevel

M.stateChanged = stateChanged
M.getRecordings = getRecordings
M.setSpeed = setSpeed -- 1=realtime, 0.5=slowmo, 2=fastmotion (the change will be instantaneous, without any smoothing)
M.toggleSpeed = toggleSpeed
M.togglePlay = togglePlay
M.toggleRecording = toggleRecording
M.cancelRecording = cancelRecording
M.loadFile = loadFile
M.stop = stop
M.pause = pause
M.seek = seek -- [0..1] normalized position to seek to
M.jumpTime = jumpTime
-- M.jump is replaced by M.jumpTime and M.jumpFrames
M.jump = jumpFrames
M.jumpFrames = jumpFrames
M.openReplayFolderInExplorer = openReplayFolderInExplorer
M.displayMsg = displayMsg
M.getPositionSeconds = getPositionSeconds
M.getTotalSeconds = getTotalSeconds
M.getState = getState
M.isPaused = isPaused
M.getLoadedFile = getLoadedFile
M.acceptRename = acceptRename
--M.onDrawDebug = onDrawDebug -- TODO

-- Mission / Automatic replay (they're the same thing eh)
M.saveMissionReplay = saveMissionReplay
M.getMissionReplayFiles = getMissionReplayFiles
M.getMissionReplaysPath = function() return missionReplaysPath end
M.toggleMissionRecording = toggleMissionRecording
M.removeMissionSavedReplay = removeMissionSavedReplay
return M
