-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local im  = ui_imgui

local C = {}

local defaultDuration = 3
local defaultMaxAnnounced = 3

C.name = 'Rally Mode Countdown'
C.icon = "timer"
C.description = 'Manages a Countdown. Displays it on the screen as well, using both Message and FlashMessage.'
C.color = rallyUtil.rally_flowgraph_color
--C.category = 'repeat'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = 'Inflow for this node.' },
  { dir = 'in', type = 'flow', name = 'reset', description = 'Reset the countdown.', impulse = true },
  -- { dir = 'in', type = 'number', name = 'duration', default = 3, description = 'Duration of countdown.' },
  -- { dir = 'in', type = 'number', name = 'maxAnnounced', default = 3, description = 'Maximum number of announcements to make.' },
  { dir = 'in', type = 'string', name = 'countdownMsg', hardcoded = true, hidden = true, default = '%d', description = 'Message to show before the countdown message; %d is the number.' },
  { dir = 'in', type = 'string', name = 'finishMsg', default = 'Go!', description = 'Message to flash at the end of countdown; leave blank to use default translation string.' },
  { dir = 'in', type = 'number', name = 'finishMsgDuration', hardcoded = true, hidden = true, default = 1, description = 'Duration of finish message.' },
  -- { dir = 'in', type = 'bool', name = 'useAudio', description = "If true, the pre-countdown will be played.", hardcoded = true, default = true },

  { dir = 'out', type = 'flow', name = 'flow', description = 'Outflow for this node.' },
  { dir = 'out', type = 'flow', name = 'finished', description = 'Triggers when countdown has finished.', impulse = true },
  { dir = 'out', type = 'flow', name = 'ongoing', description = 'Triggers when countdown is in progress.' },
}
C.tags = {'scenario'}

function C:init(mgr, ...)
  self.data.useImgui = false
  self.data.useMessages = false
  -- self.data.playSoundEffects = true
  self.data.bigFinishMsg = true

  -- self.data.visualCountdown = true
  -- self.data.voiceCountdown = false


  self.maxAnnounced = defaultDuration
  self.duration = 1
  self.timer = 1
  self.msg = "Go!"
  self.done = false
  self.running = false
  self.flags = {
    finished = false
  }
end

function C:onExecutionStarted()
  self.duration = 1
  self.timer = 1
  self.msg = "Go!"
end

function C:_executionStopped()
  self:stopTimer()
end

function C:reset()
  self:stopTimer()
  self.pinOut.flow.value = false
  self.pinOut.finished.value = false
  self.pinOut.ongoing.value = false
end

function C:stopTimer()
  self.done = false
  self.running = false
  self.flags = {
    finished = false
  }
end

function C:shouldPlayVoiceCountdown()
  return self.playVoiceCountdown
end

function C:shouldPlaySoundEffects()
  return self.playSoundEffects
end

function C:shouldShowVisualCountdown()
  return self.showVisualCountdown
end

function C:startTimer()
  local settingVisualPacenotes = settings.getValue('rallyVisualPacenotes')
  local settingAudioPacenotes = settings.getValue('rallyAudioPacenotes')
  local settingCountdownStyle = settings.getValue('rallyCountdownStyle')

  self.showVisualCountdown = nil
  self.playSoundEffects = nil
  self.playVoiceCountdown = nil

  if settingAudioPacenotes then
    self.playSoundEffects = false
    self.playVoiceCountdown = true
    if settingCountdownStyle == 'countdown_style_3' then
      self.duration = 4
      self.maxAnnounced = 3
    elseif settingCountdownStyle == 'countdown_style_5' then
      self.duration = 6
      self.maxAnnounced = 5
    else
      log("E", "Rally", "Invalid countdown style: " .. settingCountdownStyle)
    end
  else
    self.duration = defaultDuration
    self.maxAnnounced = defaultMaxAnnounced
    self.playSoundEffects = true
    self.playVoiceCountdown = false
  end

  if settingVisualPacenotes then
    self.showVisualCountdown = true
  end

  -- self.duration = self.pinIn.duration.value or defaultDuration
  -- self.maxAnnounced = self.pinIn.maxAnnounced.value or defaultMaxAnnounced

  self.timer = self.duration
  self.running = true
  self.msg = self.pinIn.finishMsg.value or "ui.scenarios.go"
  guihooks.trigger('ScenarioFlashMessageClear')
  self.flags.finished = false
  self.pinOut.flow.value = false
end

function C:enqueueSystemPacenote(pacenote_name)
  local rm = gameplay_rally.getRallyManager()
  if rm then
    rm:enqueueRandomSystemPacenote(pacenote_name)
  end
end

function C:show(msg, big, duration)
  --ui_message(msg, 1, "")
  duration = duration or (big and 1.4 or 0.95)
  guihooks.trigger('ScenarioFlashMessage', {{msg, duration , "", big}})
  if self.data.useMessages then
    guihooks.trigger('Message', {
      ttl = 1,
      msg = tostring(msg),
      category =  ("countdown__"..self.id),
      icon = 'timer'}
    )
  end
end

function C:countdown()
  if not self.running then return end

  local old = math.floor(self.timer)
  self.timer = self.timer - self.mgr.dtSim
  if self.timer <= 0 then

    if self:shouldShowVisualCountdown() then
      self:show(self.msg, self.data.bigFinishMsg, self.pinIn.finishMsgDuration.value)
    end

    if self:shouldPlayVoiceCountdown() then
      self:enqueueSystemPacenote('countdowngo')
    elseif self:shouldPlaySoundEffects() then
      Engine.Audio.playOnce('AudioGui', 'event:UI_CountdownGo')
    end

    self.flags.finished = true
    self.running = false
    self.pinOut.flow.value = true
    self.done = true
  else
    if old ~= math.floor(self.timer) and old <= self.maxAnnounced then
      self.countdownMsg = self.pinIn.countdownMsg.value or "%d"
      local countdownMsg = string.format(self.countdownMsg, old)
      local bigMsg = self.countdownMsg == "%d"

      if self:shouldShowVisualCountdown() then
        self:show(countdownMsg, bigMsg, 0.95)
      end

      if self:shouldPlayVoiceCountdown() then
        self:enqueueSystemPacenote('countdown'..countdownMsg)
      elseif self:shouldPlaySoundEffects() then
        Engine.Audio.playOnce('AudioGui', 'event:UI_Countdown1')
      end
    end
    if self.data.useImgui then
      local avail = im.GetContentRegionAvail()
      local txt = " - " .. (math.ceil(old)+1) .. " - "
      local tWidth = im.CalcTextSize(txt)
      if tWidth.x < avail.x then
        im.Dummy(im.ImVec2((avail.x-tWidth.x)/2 -10,0))
        im.SameLine()
      end
      im.Text(txt)
    end
  end
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.ProgressBar((self.duration - self.timer) / self.duration, im.ImVec2(100,0))
  if not self.running then
    if self.done then
      im.Text("Done")
    else
      im.Text("Stopped")
    end
  else
    im.Text("Running")
  end
end

function C:work(args)
  if self.pinIn.reset.value then
    self:reset()
  end
  if self.pinIn.flow.value and not self.running and not self.done then
    self:startTimer()
  end
  self:countdown()
  self.pinOut.flow.value = self.done
  self.pinOut.ongoing.value = self.running
  -- set out pins according to flags and reset flags
  for pName, val in pairs(self.flags) do
    self.pinOut[pName].value = val
    self.flags[pName] = false
  end

end

return _flowgraph_createNode(C)
