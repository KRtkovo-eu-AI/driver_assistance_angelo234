-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'AI Arrive'
C.color = ui_flowgraph_editor.nodeColors.ai
C.icon = ui_flowgraph_editor.nodeIcons.ai
C.description = 'Sets a vehicle to drive towards a target waypoint.'
C.category = 'once_f_duration'
C.behaviour = { duration = true }


C.pinSchema = {
  { dir = 'in', type = 'number', name = 'aiVehId', description = 'Vehicle id to set AI mode for.' },
  { dir = 'in', type = 'string', name = 'waypointName', description = 'Name of waypoint to be driven to.' },
  { dir = 'in', type = 'number', name = 'checkDistance', hidden = true, default = 1, description = 'If the vehicle is closer than this distance, it is considered arrived. Keep empty for default waypoint width.'
  },
  { dir = 'in', type = 'number', name = 'checkVelocity', hidden = true, default = 0.1, hardcoded = true, description = 'If given, vehicle has to be slower than this to be considered arrived.' },
  { dir = 'out', type = 'number', name = 'distance', hidden = true, description = 'Distance to the center of the target waypoint.' }
}


C.legacyPins = {
  _in = {
    inRadius = 'complete'
  }
}

C.tags = {'manual', 'driveTo'}

function C:init()
  self.sentCommand = false
  self.data.autoDisableOnArrive = true
  self:setDurationState('inactive')
end

function C:onNodeReset()
  self.sentCommand = false
  self:setDurationState('inactive')
end

function C:_executionStopped()
  self:onNodeReset()
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.Text("State: "..self.durationState)
  im.Text("Command sent: "..(self.sentCommand and 'yes' or 'no'))
end

function C:getVeh()
  local veh
  if self.pinIn.aiVehId.value then
    veh = getObjectByID(self.pinIn.aiVehId.value)
  else
    veh = getPlayerVehicle(0)
  end
  return veh
end

function C:work()
  if self.pinIn.flow.value then
    if self.durationState == 'finished' then return end

    if self.pinIn.waypointName.value then
      local node = map.getMap().nodes[self.pinIn.waypointName.value]
      if not node then
        self:__setNodeError("work", "No target waypoint of name " .. self.pinIn.waypointName.value .. " found!")
        return
      end

      local veh = self:getVeh()
      if not veh then
        self:__setNodeError("work", "No vehicle found!")
        return
      end

      local radius = self.pinIn.checkDistance.value
      if not radius or radius == 0 then
        radius = node.radius
      end
      local frontPos = linePointFromXnorm(vec3(veh:getCornerPosition(0)), vec3(veh:getCornerPosition(1)), 0.5)
      local dist = (frontPos - node.pos):length()
      self.pinOut.distance.value = dist

      if dist < radius and map.objects[veh:getID()].vel:length() < (self.pinIn.checkVelocity.value or 10000) then
        self:setDurationState('finished')
        if self.data.autoDisableOnArrive then
          veh:queueLuaCommand('ai.setState({mode = "disabled"})')
        end
      end
      if not self.sentCommand then
        veh:queueLuaCommand('ai.setState({mode = "manual"})')
        veh:queueLuaCommand('ai.setTarget("'..self.pinIn.waypointName.value..'")')
        self.sentCommand = true
        self:setDurationState('started')
      end
    else
      self:__setNodeError("work", "No target waypoint name given!")
    end
  end
end

return _flowgraph_createNode(C)
