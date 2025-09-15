-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Audio Channel Fade'
C.icon = 'audiotrack'
C.description = 'Fades the volume for the given channel, with more controls.'
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow' },
  { dir = 'in', type = 'flow', name = 'fadeDown', description = 'Start fading down to silence.' },
  { dir = 'in', type = 'flow', name = 'fadeUp', description = 'Start fading up to previous volume.' },
  { dir = 'in', type = 'number', name = 'duration', default = 1, description = 'Duration of fade.' },
  { dir = 'in', type = 'string', name = 'channel', default = 'AudioChannelMaster', description = '(Optional) The audio channel to fade.' },
  { dir = 'in', type = 'number', name = 'volumeMin', hidden = true, default = 0, description = '(Optional) Minimum volume (from 0 to 1).' },
  { dir = 'in', type = 'number', name = 'volumeMax', hidden = true, default = 1, description = '(Optional) Maximum volume (from 0 to 1).' },

  { dir = 'out', type = 'flow', name = 'fadeDone' },
  { dir = 'out', type = 'number', name = 'volume', description = 'Current volume.' }
}

C.legacyPins = {
  _in = {
    fadeTime = 'duration',
    fadeIn = 'fadeDown',
    fadeOut = 'fadeUp'
  }
}

C.tags = {'sound', 'audio', 'volume'}

-- this node uses the original behavior of manipulating the channel volume via use of a self timer, for backwards compatibility reasons

function C:reset()
  self.timer = 0
  self.mode = 0
  self.origVolume = 0
  self.targetVolume = 0
  self.channel = nil
end

function C:init()
  self:reset()
end

function C:_executionStopped()
  if self.channel then
    Engine.Audio.setChannelVolume(self.channel, Engine.Audio.getChannelVolume(self.channel, false), true)
  end
  self:reset()
end

function C:work()
  if not self.pinIn.flow.value then return end

  self.channel = self.pinIn.channel.value or 'AudioChannelMaster'
  local duration = math.max(0, self.pinIn.duration.value or 1)
  self.pinOut.volume.value = Engine.Audio.getChannelVolume(self.channel, true) -- actual volume value

  if self.pinIn.fadeDown.value or self.pinIn.fadeUp.value then
    local cmode = 0
    if self.pinIn.fadeDown.value and not self.pinIn.fadeUp.value then
      cmode = -1
    elseif self.pinIn.fadeUp.value and not self.pinIn.fadeDown.value then
      cmode = 1
    end
    if cmode ~= self.mode then
      self.timer = 0
      self.mode = cmode
      self.pinOut.fadeDone.value = false

      if self.mode ~= 0 then
        self.origVolume = Engine.Audio.getChannelVolume(self.channel, true)
        if self.mode == -1 then
          self.targetVolume = self.pinIn.volumeMin.value or 0
        else
          self.targetVolume = self.pinIn.volumeMax.value or Engine.Audio.getChannelVolume(self.channel, false)
        end
      end
    end
  end

  if self.mode == 0 then return end

  if not self.pinOut.fadeDone.value then
    self.timer = self.timer + self.mgr.dtReal
    if self.timer >= duration then
      Engine.Audio.setChannelVolume(self.channel, self.targetVolume, true)
      self.pinOut.fadeDone.value = true
    else
      Engine.Audio.setChannelVolume(self.channel, lerp(self.origVolume, self.targetVolume, self.timer / duration), true)
    end
  end
end

return _flowgraph_createNode(C)
