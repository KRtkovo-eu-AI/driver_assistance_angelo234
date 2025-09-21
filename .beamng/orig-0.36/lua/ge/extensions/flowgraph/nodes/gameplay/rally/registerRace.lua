-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = ''

C.name = 'Rally Mode Register Race'
C.description = 'Keep track of the race data.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'table', name = 'raceData', tableType = 'raceData', description = 'Race data.'},
}

function C:workOnce()
  extensions.hook('onRallyRegisterRace', self.pinIn.raceData.value)
end

return _flowgraph_createNode(C)
