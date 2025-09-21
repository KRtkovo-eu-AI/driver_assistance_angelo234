-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Audio Fade In'
C.icon = 'audiotrack'
C.description = 'Fades the global volume up.'
C.category = 'once_f_duration'
C.pinSchema = {
  { dir = 'in', type = 'number', name = 'duration', default = 1, description = 'Duration of fade.' }
}

C.tags = {'sound', 'audio', 'volume'}

function C:init()
  self.timer = 0
end

function C:_executionStopped()
  self:reset()
end

function C:onNodeReset()
  self:reset()
end

function C:reset()
  self.timer = 0
  self:setDurationState('inactive')
  SFXSystem.setGlobalParameter("g_FadeTimeMS", 0)
  SFXSystem.setGlobalParameter("g_GameLoading", 0)
end

function C:workOnce()
  SFXSystem.setGlobalParameter("g_FadeTimeMS", (self.pinIn.duration.value or 0) * 1000)
  SFXSystem.setGlobalParameter("g_GameLoading", 0)
  self:setDurationState('started')
end

function C:work()
  local duration = self.pinIn.duration.value or 0
  if self.durationState ~= 'finished' and self.timer >= duration then
    self:setDurationState('finished')
  else
    self.timer = self.timer + self.mgr.dtReal
  end
end

return _flowgraph_createNode(C)
