-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = 'rally-mode-flowgraph'

C.name = 'Rally Mode Stage Start'
C.description = 'Plays pre-countdown audio.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'once_f_duration'

function C:_executionStarted()
  self:onNodeReset()
end

function C:_executionStopped()
  self:onNodeReset()
end

function C:onNodeReset()
  self:setDurationState('inactive')
end

function C:workOnce()
  extensions.hook('onRallyStageStart')

  if settings.getValue('rallyAudioPacenotes') then
    local rm = gameplay_rally.getRallyManager()
    rm:enqueuePauseSecs(0.5)
    rm:enqueueRandomSystemPacenote('firstnoteintro')
    rm:playFirstPacenote()
    rm:enqueueRandomSystemPacenote('firstnoteoutro')
  end
end

function C:work()
  local rm = gameplay_rally.getRallyManager()
  local am = rm.audioManager
  local qi = am:getQueueInfo()
  if qi.queueSize == 0 and qi.paused then
    self:setDurationState('finished')
  end
end

return _flowgraph_createNode(C)
