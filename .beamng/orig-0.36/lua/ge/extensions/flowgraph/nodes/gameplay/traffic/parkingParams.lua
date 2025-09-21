-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Parking System Parameters'
C.description = 'Sets variables for the parking system.'
C.color = ui_flowgraph_editor.nodeColors.traffic
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'
C.tags = {'traffic', 'parking', 'parameters'}

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'precision', description = 'Precision required to validate a vehicle in a parking spot (from 0 to 1).' },
  { dir = 'in', type = 'number', name = 'neatness', description = 'Parking neatness of other parked vehicles used by the parking system (from 0 to 1).' },
  { dir = 'in', type = 'number', name = 'parkingDelay', description = 'Delay, in seconds, until a stopped vehicle is considered parked in a parking spot.' },
  { dir = 'in', type = 'number', name = 'respawnProbability', description = 'Base probability to use for finding and moving to parking spots.' },
  { dir = 'in', type = 'number', name = 'poolActiveAmount', hidden = true, description = 'Amount of active and visible vehicles in the vehicle pooling system.' },
  { dir = 'in', type = 'number', name = 'debugLevel', hidden = true, description = 'Debug mode level to use (from 0 to 3).' }
}

function C:init()
  self.vars = {}
  self.data.usePoolInactiveAmount = false
end

function C:workOnce()
  table.clear(self.vars)

  if self.pinIn.precision.value ~= nil then
    self.vars.precision = clamp(self.pinIn.precision.value, 0, 1)
  end
  if self.pinIn.neatness.value ~= nil then
    self.vars.neatness = clamp(self.pinIn.neatness.value, 0, 1)
  end
  if self.pinIn.parkingDelay.value ~= nil then
    self.vars.parkingDelay = self.pinIn.parkingDelay.value
  end
  if self.pinIn.respawnProbability.value ~= nil then
    self.vars.baseProbability = self.pinIn.respawnProbability.value
  end
  if self.pinIn.poolActiveAmount.value ~= nil then
    if not self.data.usePoolInactiveAmount then
      self.vars.activeAmount = self.pinIn.poolActiveAmount.value
    else
      self.vars.activeAmount = #gameplay_parking.getParkedCarsList() - self.pinIn.poolActiveAmount.value
    end
  end

  if self.pinIn.debugLevel.value ~= nil then
    gameplay_parking.debugLevel = self.pinIn.debugLevel.value
  end

  if next(self.vars) then
    gameplay_parking.setParkingVars(self.vars)
  end
end

return _flowgraph_createNode(C)