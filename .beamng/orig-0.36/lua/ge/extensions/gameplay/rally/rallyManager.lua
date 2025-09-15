-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local MissionSettings = require('/lua/ge/extensions/gameplay/rally/notebook/missionSettings')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local VehicleTracker  = require('/lua/ge/extensions/gameplay/rally/vehicleTracker')
local AudioManager = require('/lua/ge/extensions/gameplay/rally/audioManager')
local DrivelineRoute = require('/lua/ge/extensions/gameplay/rally/driveline/drivelineRoute')
local Recce = require('/lua/ge/extensions/gameplay/rally/recce')
local dequeue = require('dequeue')

local C = {}
local logTag = ''

local holdValue = 'HOLD'

function C:init(missionDir, missionId, damageThresh, closestPacenotes_n)
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, '<<<<< RallyManager2 init >>>>>') end
  self.damageThresh = damageThresh
  self.closestPacenotes_n = closestPacenotes_n

  self.pacenoteQueue = dequeue.new()

  self.missionId = missionId
  self.missionDir = missionDir
  self.missionName = nil
  self:getMissionName()

  self.notebook = nil
  self.race = nil
  self.splitPathnodes = nil

  self.audioManager = nil
  self.vehicleTracker = nil
  self.drivelineMode = nil
  self.drivelineRoute = nil

  self.errorMsgForUser = nil
end

function C:getErrorMsgForUser()
  return self.errorMsgForUser
end

function C:setDrivelineMode(drivelineMode)
  self.drivelineMode = drivelineMode
end

function C:getDrivelineMode()
  return self.drivelineMode
end

function C:getMissionDir()
  return self.missionDir
end

function C:getStartPosition()
  return self.race.startPositions.objects[self.race.defaultStartPosition]
end

function C:getNotebookPath()
  return self.notebook
end

function C:getRacePath()
  return self.race
end

function C:getMissionName()
  if self.missionName then
    return self.missionName
  else
    self.missionName = rallyUtil.translatedMissionNameFromId(self.missionId)
    return self.missionName
  end
end

function C:getMissionStartTrigger()
  local mission = gameplay_missions_missions.getMissionById(self.missionId)
  if not mission then
    log('E', logTag, 'getMissionStartTriggerPos: mission not found')
    return nil
  end
  return mission.startTrigger
end

function C:triggerPacenote(pacenote)
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'triggerPacenote name='..tostring(pacenote.name)) end
  if self:_playbackAllowed(pacenote) then
    local settingVisualPacenotes = settings.getValue('rallyVisualPacenotes')
    local settingAudioPacenotes = settings.getValue('rallyAudioPacenotes')

    if settingAudioPacenotes then
      self:sendPacenoteToAudioManager(pacenote)
    end

    if settingVisualPacenotes then
      self:triggerShowVisualPacenote(pacenote)
    end
  end
end

function C:setupDrivelineRouteHooks()
  local rallyManager = self

  self.drivelineRoute.onPacenoteCsDynamicHit = function(pacenote, shouldTriggerAudio)
    if shouldTriggerAudio then
      -- rallyManager:triggerPacenote(pacenote)
      rallyManager:enqueuePacenote(pacenote)
      if pacenote.slowCorner then
        rallyManager:enqueueHold()
      end
    end
  end

  self.drivelineRoute.onPacenoteCsImmediateHit = function(pacenote, shouldTriggerAudio)
    if shouldTriggerAudio then
      -- rallyManager:triggerPacenote(pacenote)
      rallyManager:enqueuePacenote(pacenote)
      if pacenote.slowCorner then
        rallyManager:enqueueHold()
      end
    end
  end

  self.drivelineRoute.onPacenoteCsStaticHit = function(pacenote, shouldTriggerAudio)
    rallyManager:triggerClearVisualPacenote(pacenote)
    if shouldTriggerAudio then
      -- rallyManager:triggerPacenote(pacenote)
      rallyManager:enqueuePacenote(pacenote)
      if pacenote.slowCorner then
        rallyManager:enqueueHold()
      end
    end
    if pacenote.slowCorner and pacenote:isSlowCornerReleaseCsStatic() then
      rallyManager:clearQueueHold()
    end
  end

  self.drivelineRoute.onPacenoteCeStaticHit = function(pacenote, shouldTriggerAudio)
    if shouldTriggerAudio then
      -- rallyManager:triggerPacenote(pacenote)
      rallyManager:enqueuePacenote(pacenote)
      if pacenote.slowCorner then
        rallyManager:enqueueHold()
      end
    end
    if pacenote.slowCorner and pacenote:isSlowCornerReleaseCeStatic() then
      rallyManager:clearQueueHold()
    end
  end

  self.drivelineRoute.onPacenoteCsOffsetHit = function(pacenote, shouldTriggerAudio, offset)
    -- offset has to be manually checked by the programmer here against what trigger points the driveline route is tracking.
  end

  self.drivelineRoute.onPacenoteCeOffsetHit = function(pacenote, shouldTriggerAudio, offset)
    -- offset has to be manually checked by the programmer here against what trigger points the driveline route is tracking.
    if offset == -5 then
      if shouldTriggerAudio then
        -- rallyManager:triggerPacenote(pacenote)
        rallyManager:enqueuePacenote(pacenote)
        if pacenote.slowCorner then
          rallyManager:enqueueHold()
        end
      end

      if pacenote.slowCorner and pacenote:isSlowCornerReleaseCeMinus5() then
        rallyManager:clearQueueHold()
      end
    end
  end

  self.drivelineRoute.onPacenoteCornerPercentHit = function(pacenote, shouldTriggerAudio, percent)
    -- percent has to be manually checked by the programmer here against what trigger points the driveline route is tracking.
    if percent == 0.5 then
      if shouldTriggerAudio then
        -- rallyManager:triggerPacenote(pacenote)
        rallyManager:enqueuePacenote(pacenote)
        if pacenote.slowCorner then
          rallyManager:enqueueHold()
        end
      end

      if pacenote.slowCorner and pacenote:isSlowCornerReleaseCsHalf() then
        rallyManager:clearQueueHold()
      end
    end
  end
end

function C:codriver()
  return self.notebook:selectedCodriver()
end

function C:getSnaproadPointsFromRoute()
  local dr = DrivelineRoute()
  local points = dr:pointsForSnaproad(self.race, self.notebook)
  if not points then
    log('E', logTag, 'failed to load snaproad points')
    return false
  end

  return points
end

function C:indexSplits()
  self.splitPathnodes = {}
  local pathnodes = self.race.pathnodes.sorted
  for _, pathnode in ipairs(pathnodes) do
    local name = pathnode.name
    local visible = pathnode.visible
    if visible then
      local rp = pathnode:getRoutePoint()
      if rp then
        self.splitPathnodes[name] = pathnode
      end
    end
  end
end

function C:getPointDistanceFromStartMeters(point)
  return self.drivelineRoute:getPointDistanceFromStartMeters(point)
end

function C:getPointDistanceFromStartKm(point)
  return self.drivelineRoute:getPointDistanceFromStartKm(point)
end

function C:getSplitPathnode(name)
  if not self.splitPathnodes then
    log('E', logTag, 'getSplitPathnode: splitPathnodes not indexed')
    return nil
  end
  return self.splitPathnodes[name]
end

function C:reload()
  log('I', logTag, 'RallyManager reload')
  -- log('D', logTag, 'reload')

  self.errorMsgForUser = nil

  local ms = MissionSettings(self.missionDir)
  ms:load()

  self.vehicleTracker = VehicleTracker(self.damageThresh)

  self.race = ms:loadRace()
  if not self.race then
    log('E', logTag, 'RallyManager setup no racePath')
    self.errorMsgForUser = 'race.json missing'
    return false
  end

  self.notebook = ms:loadNotebook()
  if not self.notebook then
    log('W', logTag, 'RallyManager setup with no notebook')
    return true
  end

  self.notebook:cacheCompiledPacenotes()
  -- must load audioManager after notebook is loaded
  self.audioManager = AudioManager(self)

  self.drivelineMode = self.drivelineMode or ms:getDrivelineMode()
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'drivelineMode=' .. tostring(RallyEnums.drivelineModeNames[self.drivelineMode])) end

  self.drivelineRoute = DrivelineRoute()
  self:setupDrivelineRouteHooks()

  if self.drivelineRoute then
    if self.drivelineMode == RallyEnums.drivelineMode.route then
      if not self.drivelineRoute:loadRoute(self.race, self.notebook) then
        log('E', logTag, 'failed to load driveline route')
        self.errorMsgForUser = 'failed to load route driveline'
        return false
      end
    elseif self.drivelineMode == RallyEnums.drivelineMode.recce then
      local recce = Recce(self.missionDir)
      if not recce:loadDrivelineAndCuts() then
        log('E', logTag, 'failed to load recce driveline and cuts for refresh')
        self.errorMsgForUser = 'failed to load recce'
        return false
      end

      if not self.drivelineRoute:loadRouteFromRecordedDriveline(self.race, self.notebook, recce.driveline) then
        log('E', logTag, 'failed to load driveline route')
        self.errorMsgForUser = 'failed to load recorded driveline'
        return false
      end
    end
    self.drivelineRoute:setVehicleTracker(self.vehicleTracker)
    self:indexSplits()
  else
    log('E', logTag, 'failed to initialize driveline route')
    self.errorMsgForUser = 'failed to initialize DrivelineRoute'
    return false
  end

  if not self:softReload() then
    log('E', logTag, 'softReload failed')
    self.errorMsgForUser = 'softReload failed'
    return false
  end

  return true
end

function C:softReload()
  log('I', logTag, 'RallyManager softReload')
  -- log('D', logTag, 'softReload')

  self:triggerClearAllVisualPacenotes()
  self:resetAudioQueue()
  self.pacenoteQueue = dequeue.new()
  if self.drivelineRoute then
    self.drivelineRoute:setRecalcNeeded()
  end

  return true
end

function C:onVehicleResetted()
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'onVehicleResetted') end
  if not self:softReload() then
    log('E', logTag, 'softReload failed')
  end
end

function C:_playbackAllowed(pacenote)
  local allowed, err = pacenote:playbackAllowed()
  if err then
    log('E', logTag, 'error in pacenote:playbackAllowed(): '..err)
    allowed = true
  end

  if not allowed then
    log('I', logTag, '['..pacenote.name..'] playbackAllowed: false')
  end

  return allowed
end

function C:triggerShowVisualPacenote(pacenote)
  if not pacenote then return end
  local compiledPacenote = pacenote:asCompiled()
  local visualPacenotes = compiledPacenote.visualPacenotes2

  local visualPacenoteEvent = {
    pacenoteId = pacenote.id,
    pacenoteName = pacenote.name,
    visualPacenotes = visualPacenotes,
    serialNo = pacenote.visualSerialNo,
  }

  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, string.format('triggerShowVisualPacenote name=%s, serialNo=%s', pacenote.name, pacenote.visualSerialNo)) end

  guihooks.trigger('showVisualPacenote2', visualPacenoteEvent)
end

function C:triggerClearVisualPacenote(pacenote)
  if not pacenote then return end
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, string.format('triggerClearVisualPacenote name=%s, serialNo=%s', pacenote.name, pacenote.visualSerialNo)) end
  guihooks.trigger('clearOneVisualPacenote', pacenote.visualSerialNo)
end

function C:triggerClearAllVisualPacenotes()
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'triggerClearAllVisualPacenotes') end
  guihooks.trigger('clearAllVisualPacenotes')
end

function C:drawPacenotesForDriving()
  local nextPacenotes = self:getNextPacenotes()
  local pacenote = nextPacenotes[1]
  if pacenote then
    local wp_cs = pacenote:getCornerStartWaypoint()
    local render_wp = wp_cs
    local codriver = self:codriver()
    local compiledPacenote = pacenote:asCompiled(codriver)
    if compiledPacenote then
      local noteText = compiledPacenote.noteText
      render_wp:drawDebugRecce(1, noteText)
    end
  end
end

function C:onUpdate(dtReal, dtSim, dtRaw)
  if self.vehicleTracker then
    self.vehicleTracker:onUpdate(dtReal, dtSim, dtRaw)
    -- if self.audioManager and self.vehicleTracker:didJustHaveDamage() then
    --   self.audioManager:handleDamage()
    -- end
  end

  if self.drivelineRoute then
    self.drivelineRoute:onUpdate(dtReal, dtSim, dtRaw)
    -- self:handleDrivelineEvents()
  end

  if self.audioManager then
    self.audioManager:onUpdate(dtReal, dtSim, dtRaw)
  end

  self:processPacenoteQueue()
end

function C:clearQueueHold()
  local peekValue = self.pacenoteQueue:peek_left()
  if peekValue == holdValue then
    self.pacenoteQueue:pop_left()
  end
end

function C:enqueuePacenote(pacenote)
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'enqueuePacenote name='..tostring(pacenote.name)) end
  self.pacenoteQueue:push_right(pacenote)
end

function C:enqueueHold()
  if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'enqueueHold') end
  self.pacenoteQueue:push_right(holdValue)
end

function C:processPacenoteQueue()
  local peekValue = self.pacenoteQueue:peek_left()
  if peekValue == holdValue then
    -- log('D', logTag, 'processPacenoteQueue: '..tostring(peekValue)) -- spams the log
    -- do nothing
  elseif self.pacenoteQueue:length() > 0 then
    local pacenote = self.pacenoteQueue:pop_left()
    if pacenote then
      if gameplay_rally and gameplay_rally.getDebugLogging() then log('D', logTag, 'processPacenoteQueue: got pacenote name='..tostring(pacenote.name)) end
      self:triggerPacenote(pacenote)
    else
      log('E', logTag, 'processPacenoteQueue: expected pacenote in queue')
    end
  elseif peekValue ~= nil then
    if gameplay_rally and gameplay_rally.getDebugLogging() then log('E', logTag, 'processPacenoteQueue: unknown peek value: '..tostring(peekValue)) end
  end
end

function C:closestPacenoteToVehicle()
  local pacenotes = self.notebook:findNClosestPacenotes(self.vehicleTracker:pos(), 1)
  if pacenotes and pacenotes[1] then
    return pacenotes[1]
  else
    return nil
  end
end

function C:getNextPacenotes()
  if not self.drivelineRoute then return {} end
  return { self.drivelineRoute:getNextPacenote() }
end

function C:enqueueRandomSystemPacenote(name)
  local pacenote, metadata = self.notebook:getRandomSystemPacenote(name)
  if pacenote and self.audioManager and metadata then
    self.audioManager:enqueueSystemPacenote(pacenote, nil, metadata)
  else
    log('E', logTag, "enqueueRandomSystemPacenote: couldnt find system pacenote with name '"..name.."'")
  end
end

function C:enqueuePauseSecs(secs)
  if not self.audioManager then return end
  self.audioManager:enqueuePauseSecs(secs)
end

function C:sendPacenoteToAudioManager(pacenote)
  if not self.audioManager then return end
  self.audioManager:enqueuePacenote(pacenote)
end

function C:playFirstPacenote()
  if not self.audioManager then return end
  local pacenote = self.notebook.pacenotes.sorted[1]
  self.audioManager:enqueuePacenote(pacenote)
end

function C:getDrivelineRoute()
  return self.drivelineRoute
end

function C:resetAudioQueue()
  if not self.audioManager then return end
  self.audioManager:resetQueue()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
