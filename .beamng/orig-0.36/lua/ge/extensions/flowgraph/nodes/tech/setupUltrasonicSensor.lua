-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Create Ultrasonic Sensor'
C.color = ui_flowgraph_editor.nodeColors.tech
C.icon = ui_flowgraph_editor.nodeIcons.tech

C.description = 'Setus up an ultrasonic sensor.'
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = 'ID of the vehicle to set the sensor up for.' },
  { dir = 'in', type = 'number', name = 'width', description = 'Width of the sensor.' },
  { dir = 'in', type = 'number', name = 'height', description = 'Height of the sensor.' },
  { dir = 'out', type = 'number', name = 'sensorId', description = 'ID created sensor.' },
}

C.tags = {}

function C:init()

end

function C:workOnce()
  local vehId = self.pinIn.vehId.value or be:getPlayerVehicleID(0)
  local args = {
    size = {
      self.pinIn.width.value or 50,
      self.pinIn.height.value or 50,
    }
  }
  self.pinOut.sensorId.value = extensions.tech_sensors.createUltrasonic(vehId, args)
end


return _flowgraph_createNode(C)
