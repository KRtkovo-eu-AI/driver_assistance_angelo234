-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = 'rally-mode-flowgraph'

C.name = 'Rally Mode Handle Lap Change'
C.description = 'Updates with latest lap info.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}

C.pinSchema = {
  -- { dir = 'in', type = 'flow',   name = 'flow', description = 'Inflow for this node.' },
  -- { dir = 'in', type = 'flow',   name = 'reset', description = 'Inflow for this node.', impulse = true },
  -- { dir = 'in', type = 'table',  name = 'raceData', tableType = 'raceData', description = 'Race data.'},
  -- { dir = 'in', type = 'flow',   name = 'noteSearch', description = 'Reset completed pacenotes to near car.', impulse = true },
  { dir = 'in', type = 'flow',   name = 'lapChange', description = 'When a lap changes.', impulse = true },
  { dir = 'in', type = 'number', name = 'currLap', description = 'Current lap number.'},
  { dir = 'in', type = 'number', name = 'maxLap', description = 'Maximum lap number.'},

  -- { dir = 'out', type = 'flow', name = 'flow', description = 'Outflow from this node.' },
}

-- function C:init()
-- end

function C:work()
  if self.pinIn.lapChange.value then
    -- gameplay_rally.getRallyManager():handleLapChange(self.pinIn.currLap.value, self.pinIn.maxLap.value)
  end
end

return _flowgraph_createNode(C)
