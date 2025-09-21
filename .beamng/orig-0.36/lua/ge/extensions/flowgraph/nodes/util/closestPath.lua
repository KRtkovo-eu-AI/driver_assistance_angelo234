-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}
local route
C.name = 'Navgraph Distance'

C.description = 'Finds the approximate length between positions along the navgraph.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', impulse = true, name = 'setRoute', description = "Sets the route." },
  { dir = 'in', type = 'vec3', name = 'trackedPos', description = "The tracked position." },
  { dir = 'in', type = 'vec3', name = 'posA', description = "The first endpoint position." },
  { dir = 'in', type = 'vec3', name = 'posB', description = "The second endpoint position." },
  { dir = 'out', type = 'number', name = 'dist', description = "Distance to the end of the route." },
}

C.color = ui_flowgraph_editor.nodeColors.default

function C:init()
  self.waypoint = im.BoolPtr(false)
end

function C:_executionStarted()
  self.lastTrackedPosValue = vec3()
end

function C:_onSerialize(res)
  res.waypoint = self.waypoint[0]
end

function C:_onDeserialized(nodeData)
  self.waypoint = im.BoolPtr(nodeData.waypoint or false)
  self:updatePins()
end

function C:drawCustomProperties()
  if im.Checkbox("Waypoints", self.waypoint) then
    self:updatePins()
  end
end

function C:updatePins()
  if self.waypoint[0] then
    self:removePin(self.pinInLocal["posA"])
    self:removePin(self.pinInLocal["posB"])
    self:createPin('in', 'table', "waypoints", nil, 'The route.')
  else
    self:removePin(self.pinInLocal["waypoints"])
    self:createPin('in', 'vec3', "posA", nil, 'The first endpoint position.')
    self:createPin('in', 'vec3', "posB", nil, 'The second endpoint position.')
  end
end

function C:work()
  self.pinOut.dist.value = -1

  if (self.waypoint[0] and not self.pinIn.waypoints.value) or (not self.waypoint[0] and (not self.pinIn.posA.value or not self.pinIn.posB.value)) then return end

  if self.pinIn.setRoute.value then
    route = require('/lua/ge/extensions/gameplay/route/route')()
    if self.waypoint[0] then
      route:setupPathMulti(self.pinIn.waypoints.value)
    else
      route:setupPathMulti({vec3(self.pinIn.posB.value), vec3(self.pinIn.posA.value)})
    end
  end

  if route ~= nil and self.pinIn.flow.value and self.pinIn.trackedPos.value ~= self.lastTrackedPosValue then
    route:trackPosition(vec3(self.pinIn.trackedPos.value))
    self.lastTrackedPosValue = self.pinIn.trackedPos.value
  end

  if route ~= nil then
    self.pinOut.dist.value = route.path[1].distToTarget
  end
end

return _flowgraph_createNode(C)
