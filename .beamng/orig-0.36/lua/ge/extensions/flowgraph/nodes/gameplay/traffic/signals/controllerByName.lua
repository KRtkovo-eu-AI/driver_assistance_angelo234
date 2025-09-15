-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Controller by Name'
C.description = 'Finds a signal controller (signal type) by name.'
C.color = ui_flowgraph_editor.nodeColors.signals
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'once_instant'
C.pinSchema = {
  {dir = 'in', type = 'table', name = 'signalsData', tableType = 'signalsData', description = 'Table of traffic signals data; use the File Traffic Signals node.'},
  {dir = 'in', type = 'string', name = 'name', description = 'Name of the signal.'},
  {dir = 'out', type = 'bool', name = 'exists', hidden = true, description = 'True if data exists.'},
  {dir = 'out', type = 'table', name = 'controllerData', tableType = 'signalControllerData', description = 'Signal controller data.'}
}

C.tags = {'traffic', 'signals'}

function C:onNodeReset()
  self.pinOut.controllerData.value = nil
end

function C:init()
  self:onNodeReset()
end

function C:_executionStopped()
  self:onNodeReset()
end

function C:work(args)
  if not self.pinOut.controllerData.value then
    if self.pinIn.signalsData.value and self.pinIn.name.value then
      for _, ctrl in ipairs(self.pinIn.signalsData.value.controllers) do
        if ctrl.name == self.pinIn.name.value then
          self.pinOut.controllerData.value = ctrl
          break
        end
      end
    end
  end

  self.pinOut.exists.value = self.pinOut.controllerData.value and true or false
end

return _flowgraph_createNode(C)