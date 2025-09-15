-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Detect Acceleration At Start'
C.description = 'Used to start a mission when the player moves instead of a countdown.'
C.color = im.ImVec4(1, 1, 0, 0.75)
C.icon = "timer"
C.category = 'repeat_instant'
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'reset', description = "Resets the node", impulse = true },
  { dir = 'in', type = 'number', name = 'vehId', description = "The veh id to track"},
  { dir = 'out', type = 'flow', name = 'success', description = "When the player moves" },
}

function C:init()
  self.initialPos = nil
  self.msgSent = true
  self.data = {
    useScenarioRealtimeDisplay = true
  }
end

function C:_executionStarted()
  self.initialPos = nil
  self.msgSent = true
end

function C:work(args)
  if self.pinIn.reset.value then
    self.initialPos = nil
  end
  self.pinOut.success.value = false
  if self.pinIn.flow.value then
    local vehId
    if self.pinIn.vehId.value == nil then
      vehId = be:getPlayerVehicleID(0)
    else
      vehId = self.pinIn.vehId.value
    end

    local vehData = map.objects[vehId]
    if not vehData then return end

    if not self.initialPos then
      self.initialPos = vec3(vehData.pos)
      self.msgSent = true
    end

    local hasMoved = vehData.pos:distance(self.initialPos) > 0.1

    -- display start message
    if self.data.useScenarioRealtimeDisplay then
      if self.msgSent and not hasMoved then
        guihooks.trigger('ScenarioRealtimeDisplay', {msg = translateLanguage("missions.general.moveToStart", "missions.general.moveToStart", true), context = "accelerateToBegin"})
      end

      if hasMoved and self.msgSent then
        self.initialPosIsSet = false
        guihooks.trigger('ScenarioRealtimeDisplay', {msg = "", context = "accelerateToBegin"})
      end
    end

    if hasMoved then
      self.pinOut.success.value = true
    end
  end
end

return _flowgraph_createNode(C)
