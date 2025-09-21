-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = 'rally-mode-flowgraph'

C.name = 'Rally Mode Reset Next Pacenote'
C.description = 'Resets the next pacenote to the vehicle position.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'repeat_instant'

function C:work()
  log('D', logTag, 'FG softReload')
  local rm = gameplay_rally.getRallyManager()
  if rm then
    log('D', logTag, 'FG softReload doing the softReload')
    rm:softReload()
  else
    log('E', logTag, 'FG softReload failed, rallyManager not found')
  end

  self.pinOut.flow.value = self.pinIn.flow.value
end

return _flowgraph_createNode(C)
