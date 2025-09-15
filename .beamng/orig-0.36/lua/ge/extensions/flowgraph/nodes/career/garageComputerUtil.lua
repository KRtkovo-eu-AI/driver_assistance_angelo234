-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'Garage Computer Util'
C.description = 'Util Helper for Garage tutorial'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'computerOpened', description = "Outflow once if computer was opened", impulse = true },
  { dir = 'out', type = 'flow', name = 'tuningStarted', description = "Outflow once if tuning was opened", impulse = true },
  { dir = 'out', type = 'flow', name = 'tuningApplied', description = "Outflow once if tuning was applied", impulse = true },
  { dir = 'out', type = 'flow', name = 'shoppingStarted', description = "Outflow once if part shopping opened", impulse = true },
  { dir = 'out', type = 'flow', name = 'partInstalled', description = "Outflow once if part shopping opened", impulse = true },
  { dir = 'out', type = 'flow', name = 'transactionComplete', description = "Outflow once if part shopping opened", impulse = true },




}
C.dependencies = {}

function C:init()
  self.flags = {}
end


function C:work(args)
  self.pinOut.computerOpened.value = false
  self.pinOut.tuningStarted.value = false
  self.pinOut.tuningApplied.value = false
  self.pinOut.shoppingStarted.value = false
  self.pinOut.partInstalled.value = false
  self.pinOut.transactionComplete.value = false

  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end
  table.clear(self.flags)
end

function C:onComputerMenuOpened()
  self.flags.computerOpened = true
end

function C:onCareerTuningStarted()
  self.flags.tuningStarted = true
end

function C:onCareerTuningApplied()
  self.flags.tuningApplied = true
end

function C:onPartShoppingStarted()
  self.flags.shoppingStarted = true
end

function C:onPartShoppingPartInstalled(data)
  self.flags.partInstalled = true
end

function C:onPartShoppingTransactionComplete()
  self.flags.transactionComplete = true
end

return _flowgraph_createNode(C)