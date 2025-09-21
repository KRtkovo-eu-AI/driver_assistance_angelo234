-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Popup'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.description = "Shows a Popup that can only be dismissed using buttons."
C.todo = "Showing two of these at the same time will queue them after another"
C.behaviour = { once = true, singleActive = true}
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = 'Inflow for this node.' },
  { dir = 'in', type = 'flow', name = 'reset', description = 'Resets this node.', impulse = true },
  { dir = 'in', type = {'string', 'table'}, name = 'title', description = 'Title.' },
  { dir = 'in', type = {'string', 'table'}, name = 'text', description = 'Text.' },
  { dir = 'in', type = 'string', name = 'buttonText', hardcoded = true, hidden = true, default = 'ui.scenarios.start.start', description = 'Text to display on the button.' },
  { dir = 'out', type = 'flow', name = 'flow', description = 'Outflow for this node.' },
}
C.dependencies = {'core_input_bindings'}

function C:init()
  self.open = false
  self.done = false
end

function C:_executionStarted()
  for _, p in pairs(self.pinOut) do
    p.value = false
  end
  self.open = false
  self.done = false
  self._active = false
end

function C:postInit()

end

function C:_executionStarted()
  for _, p in pairs(self.pinOut) do
    p.value = false
  end
  self._active = false
end

function C:_executionStopped()
  if self.open then
    self:closeDialogue()
  end
  self:reset()
end

function C:reset()
  self.done = false
  self.open = false
  self._active = false
end

function C:buttonPushed(action)
  for nm, pn in pairs(self.pinOut) do
    self.pinOut[nm].value = nm == action
  end
end

function C:getCmd(action)
  return 'local n = core_flowgraphManager.getManagerGraphNode('..self.mgr.id..', '..self.graph.id..', '..self.id..') if n then n:buttonPushed("'..action..'") end'
end

function C:closeDialogue()
  self.open = false
  self._active = false
end

function C:openDialogue()
  self.open = true
  -- dump("opening dialogue!")
  local data = {}

  data.title = self.pinIn.title.value or ""
  data.text = self.pinIn.text.value
  data.buttons = {
    {
      default = true,
      class = "main",
      label = self.pinIn.buttonText.value or "ui.scenarios.start.start",
      clickLua = 'local n = core_flowgraphManager.getManagerGraphNode('..self.mgr.id..', '..self.graph.id..', '..self.id..') if n then n:started() end'
    }
  }

  self._active = true
  guihooks.trigger('introPopupMission', data)
end


function C:closed()
  self.done = true
  self._active = false
end

function C:onFilteredInputChanged(devName, action, value)

end

function C:started()
  self:closeDialogue()
  self.pinOut.flow.value = true
  self.done = true
  self._active = false
end

function C:onClientEndMission()
  self.open = false
  self._active = false
end

function C:work()
  if self.pinIn.reset.value == true then
    if self.open then
      self:closeDialogue()
    end
    self:reset()
    for _,pn in pairs(self.pinOut) do
      pn.value = false
    end
    return
  else
    if self.done then return end
    if self.pinIn.flow.value and not self.open then
      self:openDialogue()
    end
  end
end

return _flowgraph_createNode(C)
