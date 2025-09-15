-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Get Ultrasonic Readings'
C.color = ui_flowgraph_editor.nodeColors.tech
C.icon = ui_flowgraph_editor.nodeIcons.tech

C.description = 'Gets Ultrasonic sensor readins'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'sensorId', description = 'ID of the vehicle to set the sensor up for.' },
}

C.tags = {}

function C:init()

end

function C:work()
  local data = extensions.tech_sensors.getUltrasonicReadings(self.pinIn.sensorId.value)
  dump(data)
end


return _flowgraph_createNode(C)
