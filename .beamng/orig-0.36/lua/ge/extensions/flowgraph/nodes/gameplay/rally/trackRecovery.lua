-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = 'rally-mode-flowgraph'

C.name = 'Rally Mode Track Recovery'
C.description = 'Track flips and recoveries.'
C.color = rallyUtil.rally_flowgraph_color
C.tags = {'rally'}
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flip', description = 'A flip'},
  { dir = 'in', type = 'flow', name = 'recovery', description = 'A recovery'},
  { dir = 'in', type = 'flow', name = 'restart', description = 'A recovery'},

  { dir = 'in', type = 'number', name = 'vehId', description = 'Vehicle id.'},
  { dir = 'in', type = 'table',  name = 'raceData', tableType = 'raceData', description = 'Race data.'},
}

function C:work()
  local recoveryType = nil

  print('flip: '     .. tostring(self.pinIn.flip.value))
  print('recovery: ' .. tostring(self.pinIn.recovery.value))
  print('restart: '  .. tostring(self.pinIn.restart.value))

  if self.pinIn.flip.value then
    recoveryType = 'flip'
  end

  if self.pinIn.recovery.value then
    recoveryType = 'recovery'
  end

  if self.pinIn.restart.value then
    recoveryType = 'restart'
  end

  print('recoveryType: '..tostring(recoveryType))

  extensions.hook('onRallyVehicleRecovery', recoveryType)
end

return _flowgraph_createNode(C)
