-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = 'rally-mode-flowgraph'

C.name = 'Rally Mode Flying Finish'
C.description = 'Plays audio after crossing the finish line.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'once_instant'

C.pinSchema = {
}

function C:workOnce()
  local settingAudioPacenotes = settings.getValue('rallyAudioPacenotes')
  local settingVisualPacenotes = settings.getValue('rallyVisualPacenotes')

  if settingVisualPacenotes then
    guihooks.trigger('ScenarioFlashMessageClear')
    local big = false
    require('scenario/scenariohelper').flashUiMessage('ui.rally.finish', 5, big)
  end

  if settingAudioPacenotes then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      rm:enqueuePauseSecs(0.75)
      rm:enqueueRandomSystemPacenote('finish')
    end
  end

  extensions.hook('onRallyStageFlyingFinish')
end

return _flowgraph_createNode(C)
