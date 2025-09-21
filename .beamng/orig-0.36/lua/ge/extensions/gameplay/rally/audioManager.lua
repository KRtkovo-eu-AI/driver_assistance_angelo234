-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local dequeue = require('dequeue')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = ''

function C:init(rallyManager)
  self.rallyManager = rallyManager
  self.pacenoteMetadataOfflineStructured = nil
  self.pacenoteMetadataOnlineStructuredAndFreeform = nil

  self.queue = dequeue.new()
  self.currAudioObj = nil

  self:_loadPacenoteMetadata()
  -- self:resetQueue()

  -- self.damageAudioPlayedAt = nil
  -- self.damageTimeoutSecs = 1.5
end


function C:_loadPacenoteMetadata()
end

function C:resetQueue()
  -- log('D', logTag, "resetQueue")
  self.queue = dequeue.new()

  if self.currAudioObj then
    self:_stopAudio()
  end

  self.currAudioObj = nil
end

-- function C:stopSfxSource(sourceId)
--   local sfxSource = scenetree.findObjectById(sourceId)
--   if sfxSource then
--     sfxSource:stop(-1)
--   end
-- end

function C:_stopAudio()
  -- self:stopSfxSource(self.currAudioObj.sourceId)
  Engine.Audio.intercomStopPacenote()
end

function C:handleDamage()
  -- if self.currAudioObj and self.currAudioObj.sourceId then
  --   self:stopSfxSource(self.currAudioObj.sourceId)
  --   self.currAudioObj = nil
  -- end

  if self.currAudioObj then
    if not self.currAudioObj.damage then
      if self.currAudioObj.sourceId then
        -- immediately stop playing and clear currAudioObj so isPlaying will return false.
        -- the note that was playing wont be played again.
        self:_stopAudio()
        self.currAudioObj = nil
        self.queue = dequeue.new()
        -- self:enqueueDamage()
      end
    end
  else
    -- self:enqueueDamage()
  end
end

function C:enqueueDamage()
  -- local ao = self:enqueueSystemPacenote('damage_1', true)
  -- ao.damage = true
  -- ao.breathSuffixTime = 1.0
  -- ao = self:enqueuePauseSecs(0.5, true)
  -- ao.damage = true
end

function C:enqueuePauseSecs(secs, addToFront)
  addToFront = addToFront or false
  -- log('I', logTag, 'pause='..secs..'s front='..tostring(addToFront))
  local audioObj = rallyUtil.buildAudioObjPause(secs)
  if addToFront then
    self.queue:push_left(audioObj)
  else
    self.queue:push_right(audioObj)
  end
  return audioObj
end

function C:enqueuePacenote(pacenote, addToFront)
  local compiledPacenote = pacenote:asCompiled()
  if compiledPacenote then
    log('I', logTag, "RallyMode: playing pacenote: name='"..pacenote.name.."' audioMode='"..tostring(pacenote:getAudioModeString()).."' note='"..compiledPacenote.noteText.."'")

    local audioMetadata = nil
    if pacenote:isAudioModeStructuredOnline() then
      audioMetadata = self.rallyManager:getNotebookPath():loadOnlineStructuredPacenoteMetadata()
    elseif pacenote:isAudioModeStructuredOffline() then
      audioMetadata = self.rallyManager:getNotebookPath():loadOfflineStructuredPacenoteMetadata()
    elseif pacenote:isAudioModeFreeform() then
      audioMetadata = self.rallyManager:getNotebookPath():loadFreeformPacenoteMetadata()
    elseif pacenote:isAudioModeCustom() then
      audioMetadata = self.rallyManager:getNotebookPath():loadCustomPacenoteMetadata()
    else
      log('E', logTag, "enqueuePacenote: unknown audio mode")
      return nil
    end

    if pacenote:useStructured() then
      local noteStrs = nil

      if pacenote:isAudioModeStructuredOnline() then
        noteStrs = compiledPacenote.audioFnamesStructuredOnline
      elseif pacenote:isAudioModeStructuredOffline() then
        noteStrs = compiledPacenote.audioFnamesStructuredOffline
      end

      local breathSuffixTime = nil
      local breathConfig = pacenote:getBreathConfig()
      for i,fname in ipairs(noteStrs) do
        if i == #noteStrs then
          -- the last note may take longer breaths to represent the co-driver taking a breath after reading a pacenote.
          breathSuffixTime = rallyUtil.breathSuffixTime(breathConfig.lastSubphrase[1], breathConfig.lastSubphrase[2])
        else
          breathSuffixTime = rallyUtil.breathSuffixTime(breathConfig.default[1], breathConfig.default[2])
        end
        self:_enqueueFile(pacenote.name, compiledPacenote.noteText, fname, addToFront, breathSuffixTime, audioMetadata)
      end
    elseif pacenote:isAudioModeCustom() then
      local breathSuffixTime = nil
      return self:_enqueueFile(pacenote.name, compiledPacenote.noteText, compiledPacenote.audioFnameCustom, addToFront, breathSuffixTime, audioMetadata)
    elseif pacenote:isAudioModeFreeform() then
      local breathSuffixTime = nil
      return self:_enqueueFile(pacenote.name, compiledPacenote.noteText, compiledPacenote.audioFnameFreeform, addToFront, breathSuffixTime, audioMetadata)
    else
      log('E', logTag, "enqueuePacenote: unknown audio mode")
      return nil
    end
  else
    log('E', logTag, "enqueuePacenote: compiled pacenote is missing")
    return nil
  end
end

function C:enqueueSystemPacenote(pacenote, addToFront, audioMetadata)
  if pacenote then
    log('I', logTag, "RallyMode: playing system pacenote: '"..pacenote.text.."'")
    self:_enqueueFile(pacenote.name, pacenote.text, pacenote.audioFname, addToFront, nil, audioMetadata)
  else
    log('E', logTag, "enqueueSystemPacenote: couldnt find static pacenote with name '"..pacenote.name.."'")
  end
end

function C:_enqueueFile(name, noteText, fname, addToFront, breathSuffixTime, audioMetadata)
  addToFront = addToFront or false
  local audioObj = rallyUtil.buildAudioObjPacenote(fname)
  if breathSuffixTime then
    audioObj.breathSuffixTime = clamp(breathSuffixTime, 0.0, 2.0)
  end
  audioObj.note_name = name

  local _, basename, _ = path.split(fname)
  if audioMetadata then
    local metadataVal = audioMetadata[basename]
    if not metadataVal then
      log('E', logTag, "_enqueueFile: cant find metadata entry for basename=" .. basename)
      guihooks.message("Can't get audio length for pacenote '".. noteText .."'.", 5)
      return nil
    end
    audioObj.audioLen = tonumber(metadataVal.audioLen)
  else
    log('E', logTag, "_enqueueFile: no audioMetadata")
    guihooks.message("Can't get metadata for pacenotes.", 5)
    return nil
  end

  if FS:fileExists(fname) then
    -- log('D', logTag, "_enqueueFile: exists=yes front="..tostring(addToFront) .." fname=" .. fname)
    if addToFront then
      self.queue:push_left(audioObj)
    else
      self.queue:push_right(audioObj)
    end
    return audioObj
  else
    log('E', logTag, "_enqueueFile: exists=no fname=" .. fname)
    guihooks.message("Can't find audio file for pacenote '".. noteText .."'.", 5)
    return nil
  end
end

function C:doPause(audioObj)
  audioObj.time = rallyUtil.getTime()
  audioObj.audioLen = audioObj.pauseTime
  audioObj.timeout = audioObj.time + audioObj.audioLen
  -- log('D', logTag, 'doPause: '..dumps(audioObj))
end

function C:isPlaying()
  if self.currAudioObj then
    return rallyUtil.getTime() < self.currAudioObj.timeout
  else
    return false
  end
end

local queueInfo = {}
function C:getQueueInfo()
  queueInfo.queueSize = self.queue:length()
  queueInfo.paused = not self:isPlaying()
  return queueInfo
end

function C:playNextInQueue()
  if not self:isPlaying() then
    self.currAudioObj = self.queue:pop_left()
    if self.currAudioObj then
      if self.currAudioObj.audioType == 'pacenote' then
        rallyUtil.playPacenote(self.currAudioObj)
      elseif self.currAudioObj.audioType == 'pause' then
        self:doPause(self.currAudioObj)
      else
        log('E', logTag, 'unknown audioType: '..self.currAudioObj.audioType)
      end
    end
  end
end

function C:onUpdate(dtReal, dtSim, dtRaw)
  self:playNextInQueue()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
