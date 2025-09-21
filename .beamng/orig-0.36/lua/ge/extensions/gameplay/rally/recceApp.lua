-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local Recce = require('/lua/ge/extensions/gameplay/rally/recce')
local RecceSettings = require('/lua/ge/extensions/gameplay/rally/recceSettings')
local VehicleCapture = require('/lua/ge/extensions/gameplay/rally/vehicleCapture')
local CutCapture = require('/lua/ge/extensions/gameplay/rally/cutCapture')
local kdTreeP3d = require('kdtreepoint3d')

local logTag = ''

local M = {}

local enabled = false
-- loaded extension state
local recceSettings = nil
local cornerStartsKdTree = nil
local cornerStartsIndex = nil
local showNotes = true

-- mission list state
local missionList = nil
local lastMissionId = nil
local lastLoadState = nil
local cornerAnglesStyle = nil

-- recording state
local vehicleCapture = nil
local cutCapture = nil
-- recording settings state
local _isRecordingDriveline = false
local shouldRecordDriveline = false
local shouldRecordVoice = false

local function getPlayerVehicleForRecce()
  return getPlayerVehicle(0)
end

local function getRallyManager()
  return gameplay_rally.getRallyManager()
end

local function isFreeroam()
  return core_gamestate.state and core_gamestate.state.state == "freeroam"
end

local function ensureRecceDirs()
  if not getRallyManager() then return end

  local missionDir = getRallyManager():getMissionDir()
  local dirname = rallyUtil.missionRecceRecordDir(missionDir)

  if not FS:directoryExists(dirname) then
    log('D', logTag, 'creating recce dirs: '..dirname)
    FS:directoryCreate(dirname, true)
  end
end

local function setEnabled(val)
  enabled = val
end

local function isEnabled()
  return enabled
end

local function initCaptures()
  if not getRallyManager() then return end

  log('D', logTag, 'initCaptures')

  vehicleCapture = nil
  cutCapture = nil
  local missionDir = getRallyManager():getMissionDir()

  if isFreeroam() and missionDir then
    local veh = getPlayerVehicleForRecce()
    ensureRecceDirs()
    vehicleCapture = VehicleCapture(veh, missionDir)
    cutCapture = CutCapture(veh, missionDir)
  end
end

local function setLastMissionId(mid)
  if recceSettings then
    recceSettings:setLastMissionId(getCurrentLevelIdentifier(), mid)
  end
end

local function setLastLoadState(state)
  if recceSettings then
    recceSettings:setLastLoadState(getCurrentLevelIdentifier(), state)
  end
end

local function loadMission(missionId, missionDir)
  -- log('D', logTag, 'loadMission')

  cornerStartsKdTree = nil
  cornerStartsIndex = nil

  gameplay_rally.loadMission(missionId, missionDir)

  local rm = gameplay_rally.getRallyManager()
  if not rm then
    log('E', logTag, 'loadMission failed, no rally manager')
    guihooks.trigger('rally.recceApp.missionLoaded', false, gameplay_rally.getErrorMsgForUser())
    return
  end

  if rm and rm:getNotebookPath() then
    cornerStartsIndex = {}
    for _, pacenote in ipairs(rm:getNotebookPath().pacenotes.sorted) do
      local wp = pacenote:getCornerStartWaypoint()
      cornerStartsIndex[wp:nameWithPacenote()] = wp
      table.insert(cornerStartsIndex, wp)
    end

    cornerStartsKdTree = kdTreeP3d.new(#cornerStartsIndex)
    for _, wp in ipairs(cornerStartsIndex) do
      cornerStartsKdTree:preLoad(wp:nameWithPacenote(), wp.pos.x, wp.pos.y, wp.pos.z)
    end

    cornerStartsKdTree:build()

    if gameplay_rally.getRallyToolbox() then
      gameplay_rally.getRallyToolbox():setDrivelineMode(rm:getDrivelineMode())
    end
  end

  setLastMissionId(missionId)
  setLastLoadState(true)
  guihooks.trigger('rally.recceApp.missionLoaded', true, nil)
end

local function unloadMission()
  -- log('D', logTag, 'unloadMission')
  setLastLoadState(false)
  gameplay_rally.unloadMission()
end

local function updateVehicleCapture()
  if not vehicleCapture then return end
  if shouldRecordDriveline then
    vehicleCapture:capture()
  end
end

local function draw()
  if showNotes and not (editor and editor.isEditorActive()) then
    local rm = gameplay_rally.getRallyManager()
    if rm and isFreeroam() and not gameplay_rally.isRecceTooboxVisible() then
      rm:drawPacenotesForDriving()
    end
  end
end

local function moveVehicleToStart()
  local racePath = gameplay_rally.getRallyManager():getRacePath()
  local startPosId = racePath.defaultStartPosition
  local startPos = racePath.startPositions.objects[startPosId]

  local pos = startPos.pos
  local rot = startPos.rot

  -- local firstPacenote = racePath.pacenotes.sorted[1]
  -- if not firstPacenote then return end

  -- local pos, rot = firstPacenote:vehiclePlacementPosAndRot()

  if pos and rot then
    local playerVehicle = getPlayerVehicleForRecce()
    if playerVehicle then
      spawn.safeTeleport(playerVehicle, pos, rot)
    end
  end
end

local function moveVehicleToPacenoteV2(forward)
  local playerVehicle = getPlayerVehicleForRecce()
  if not playerVehicle then return end
  local rm = getRallyManager()
  if not rm then return end
  if not rm:getNotebookPath() then return end
  if cornerStartsKdTree and cornerStartsKdTree.itemCount == 0 then return end
  -- if not cornerStartsKdTree then return end

  local vPos = playerVehicle:getPosition()
  local nearestCsName, dist = cornerStartsKdTree:findNearest(vPos.x, vPos.y, vPos.z)
  local wp = cornerStartsIndex[nearestCsName]

  if not wp then
    log('E', logTag, 'moveVehicleToPacenoteV2 failed, no nearest wp found')
    return
  end

  local pacenoteForMove = nil
  local distToMove = 5

  -- log('D', logTag, 'moveVehicleToPacenoteV2: closest pacenote('..wp.pacenote.name..') dist='..dist)

  if dist < distToMove+2 then
    -- since the vehicle is close to a pacenote, let's move to the next or previous pacenote.
    if forward then
      -- log('D', logTag, 'moveVehicleToPacenoteV2: moving to next pacenote')
      pacenoteForMove = wp.pacenote.nextNote
    else
      -- log('D', logTag, 'moveVehicleToPacenoteV2: moving to previous pacenote')
      pacenoteForMove = wp.pacenote.prevNote
    end
  else
    -- log('D', logTag, 'moveVehicleToPacenoteV2: moving to current pacenote')
    -- since the vehicle is not close to a pacenote, let's move to the pacenote.
    pacenoteForMove = wp.pacenote
  end

  if pacenoteForMove then
    -- log('D', logTag, 'moveVehicleToPacenoteV2: moving to pacenote '..pacenoteForMove.name)
    local pos, rot = pacenoteForMove:vehiclePlacementPosAndRot(distToMove)
    spawn.safeTeleport(playerVehicle, pos, rot)

    if rm then
      if not rm:reload() then
        log('E', logTag, 'moveVehicleToPacenoteV2 failed, failed to reload rally manager')
        return
      end
    else
      log('E', logTag, 'moveVehicleToPacenoteV2 failed, no rally manager found')
      return
    end
  else
    -- log('E', logTag, 'moveVehicleToPacenoteV2 failed, no pacenoteForMove found')
  end
end

local function moveVehicleForward()
  moveVehicleToPacenoteV2(true)
end

local function moveVehicleBackward()
  moveVehicleToPacenoteV2(false)
end

local function moveVehicleToMission()
  local missionStartTrigger = gameplay_rally.getRallyManager():getMissionStartTrigger()
  if not missionStartTrigger then
    log('E', logTag, 'moveVehicleToMission: mission start trigger not found')
    return
  end

  local pos = missionStartTrigger.pos
  local rot = missionStartTrigger.rot

  local playerVehicle = getPlayerVehicleForRecce()
  if playerVehicle then
    spawn.safeTeleport(playerVehicle, vec3(pos), quat(rot))
  end
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if not enabled then return end

  draw()
  updateVehicleCapture()
end

local function setShowNotes(val)
  showNotes = val
end

local function recordDrivelineCut()
  log('I', logTag, 'recordDrivelineCut')

  if cutCapture then
    local cutId = cutCapture:capture()
    if shouldRecordVoice then
      local request = {
        -- vehicle_data = getVehiclePosForCut(),
        cut_id = cutId,
      }
      local resp = extensions.gameplay_rally_client.transcribe_recording_cut(request)
      if not resp.ok then
        guihooks.trigger('rallyInputActionDesktopCallNotOk', resp.client_msg)
      end
    end
  end
end

local function recordDrivelineStart(recordVoice)
  log('I', logTag, 'recordDrivelineStart')

  _isRecordingDriveline = true
  shouldRecordDriveline = true
  shouldRecordVoice = recordVoice

  initCaptures()
end

local function recordDrivelineStop()
  log('I', logTag, 'recordDrivelineStop')

  _isRecordingDriveline = false

  if vehicleCapture and shouldRecordDriveline then
    vehicleCapture:writeCaptures(true)
  end

  vehicleCapture = nil
  cutCapture = nil
  shouldRecordDriveline = false
  shouldRecordVoice = false
end

local function recordDrivelineClearAll()
  _isRecordingDriveline = false
  initCaptures()
  vehicleCapture:truncateCapturesFile()
  cutCapture:truncateCapturesFile()
  cutCapture:truncateTranscriptsFile()

  vehicleCapture = nil
  cutCapture = nil
end

-- local function onVehicleSwitched()
--   log('D', 'rally', 'onVehicleSwitched')
-- end
--
-- local function onVehicleSpawned()
--   log('D', 'rally', 'onVehicleSpawned')
-- end

local function toggleRallyToolbox()
  gameplay_rally.toggleRallyToolbox()
end

local function refreshMissionState()
  local level = getCurrentLevelIdentifier()

  local filterFn = function (mission)
    return mission.startTrigger.level == level and mission.missionType == 'rallyStage'
  end

  missionList = {}

  for _, mission in ipairs(gameplay_missions_missions.getFilesData() or {}) do
    if filterFn(mission) then
      local missionData = {
        missionId = mission.id,
        missionDir = mission.missionFolder,
        missionName = rallyUtil.translatedMissionName(mission.name),
      }
      table.insert(missionList, missionData)
    end
  end
  recceSettings:load()

  if recceSettings then
    lastMissionId = recceSettings:getLastMissionId(level)
    lastLoadState = recceSettings:getLastLoadState(level)
    cornerAnglesStyle = recceSettings:getCornerCallStyle()
  else
    lastMissionId = nil
    lastLoadState = false
    cornerAnglesStyle = nil
  end
end

local function reload(rallyExt)
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'reload') end

  recceSettings = RecceSettings()
  recceSettings:load()

  refreshMissionState()

  local resp = {
    missions = missionList,
    last_mission_id = lastMissionId,
    last_load_state = lastLoadState,
    corner_angles_style = cornerAnglesStyle,
  }

  guihooks.trigger('rally.recceApp.refreshed', resp)
end

local function toggleMouseLikeVehicle()
  -- log('I', logTag, 'toggleMouseLikeVehicle')
  if not gameplay_rally.getRallyToolbox() then return end
  gameplay_rally.getRallyToolbox():toggleMouseMovementCheckbox()
end

-- was considering using this for full-imgui recce app
  -- if im.BeginCombo("Missions", self.selectedMission and self.selectedMission.missionName or "Select Mission") then

  --   for _, mission in ipairs(self.missions or {}) do

  --     local isSelected = self.selectedMission and self.selectedMission.missionID == mission.missionID
  --     if im.Selectable1(mission.missionName, isSelected) then
  --       self.selectedMission = mission
  --     end
  --     if isSelected then
  --       im.SetItemDefaultFocus()
  --     end
  --   end
  --   im.EndCombo()
  -- end

  -- im.SameLine()

  -- if im.Button('Refresh') then
  --   self:refresh()
  -- end

  -- if im.Button('Load Mission') then
  --   self:loadMission()
  -- end
  -- im.SameLine()
  -- if im.Button('Unload Mission') then
  --   self:unloadMission()
  -- end


  -- im.Text("Vehicle Controls")
  -- if im.Button('<-') then
  -- end
  -- im.SameLine()
  -- if im.Button('->') then
  -- end

-- local function getCurrentState()
--   local resp = {
--     missions = self.missions,
--     last_mission_id = self.lastMissionId,
--     last_load_state = self.lastLoadState,
--     corner_angles_style = self.cornerAnglesStyle,
--   }
--   return resp
-- end

-- was considering using this for a full-imgui recce app
-- function C:loadMission()
--   if not self.selectedMission then return end

--   local missionId = self.selectedMission.missionId
--   local missionDir = self.selectedMission.missionDir
--   local missionName = self.selectedMission.missionName


--   -- log('D', 'loadMission: ' .. missionId)

--   self.rallyExt.loadMission(missionId, missionDir, missionName)
--   -- local rallyManager = self.rallyExt.getRallyManager()

--   -- if rallyManager then
--     -- self.missionDir = missionDir
--     -- self.missionId = missionId
--     -- self.missionName = missionName

--     -- local selectedPacenote = rallyManager:closestPacenoteToVehicle()
--     -- if selectedPacenote then
--     --   self.selectedPacenote = selectedPacenote
--     --   log('D', logTag, 'closest pacenote to vehicle: '..selectedPacenote.name)
--     -- end

--     -- if missionDir then
--       -- local recce = Recce(missionDir)
--       -- recce:load()
--       -- self.snaproad = Snaproad(recce)
--     -- end
--   -- end
-- end

-- was considering using this for a full-imgui recce app
-- function C:unloadMission()
--   log('D', 'recceApp.unloadMission')

--   self.rallyExt.unloadMission()

--   -- self.selectedMission = nil
--   -- self.missionDir = nil
--   -- self.missionId = nil
--   -- self.missionName = nil
--   -- self.snaproad = nil
--   -- self.selectedPacenote = nil


--   -- self.rallyExt.clearRallyManager()
-- end

-- function C:translatedMissionName()
--   local rm = gameplay_rally.getRallyManager()
--   if rm then
--     local missionName = rm:translatedMissionName()
--     return missionName
--   else
--     return '<none>'
--   end
-- end


M.onUpdate = onUpdate
M.reload = reload
M.loadMission = loadMission
M.unloadMission = unloadMission
M.setLastMissionId = setLastMissionId
M.setLastLoadState = setLastLoadState
M.setShowNotes = setShowNotes
M.moveVehicleBackward = moveVehicleBackward
M.moveVehicleForward = moveVehicleForward
M.moveVehicleToStart = moveVehicleToStart
M.moveVehicleToMission = moveVehicleToMission
M.recordDrivelineCut = recordDrivelineCut
M.recordDrivelineStart = recordDrivelineStart
M.recordDrivelineStop = recordDrivelineStop
M.recordDrivelineClearAll = recordDrivelineClearAll
M.isRecording = function() return _isRecordingDriveline end
M.setEnabled = setEnabled
M.isEnabled = isEnabled
M.toggleRallyToolbox = toggleRallyToolbox
M.toggleMouseLikeVehicle = toggleMouseLikeVehicle

return M
