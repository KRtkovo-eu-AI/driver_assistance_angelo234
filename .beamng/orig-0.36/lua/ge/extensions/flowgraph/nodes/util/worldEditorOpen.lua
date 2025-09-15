-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'World Editor Open'
C.description = "Checks if the world editor is open"
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', impulse = true, name = 'reset', description = 'Resets the impulse output pins' },

  { dir = 'out', type = 'flow', name = 'open', description = 'The world editor is open' },
  { dir = 'out', type = 'flow', name = 'close', description = 'The world editor is closed' },
  { dir = 'out', type = 'flow', name = 'onOpened', impulse = true, hidden = true, description = 'The world editor just opened' },
  { dir = 'out', type = 'flow', name = 'onClosed', impulse = true, hidden = true, description = 'The world editor just closed' },
  { dir = 'out', type = 'bool', name = 'openOrClosed',hidden = true, description = 'A boolean reflecting the open state of the world editor' },
}

C.tags = {}

function C:reset()
  self.lastState = "none"
end

function C:_executionStarted()
  self:reset()
end

function C:work()
  self.pinOut.onClosed.value = false
  self.pinOut.onOpened.value = false

  if self.pinIn.reset.value then
    self:reset()
  end

  if editor and editor.active then
    self.pinOut.open.value = true
    self.pinOut.close.value = false
    self.pinOut.openOrClosed.value = true
  else
    self.pinOut.open.value = false
    self.pinOut.close.value = true
    self.pinOut.openOrClosed.value = false
  end

  if self.lastState ~= "none" then
    if self.lastState == "opened" and not self.pinOut.open.value then
      self.pinOut.onClosed.value = true
    elseif self.lastState == "closed" and self.pinOut.open.value then
      self.pinOut.onOpened.value = true
    end
  end

  self.lastState = self.pinOut.open.value and "opened" or "closed"
end


return _flowgraph_createNode(C)
