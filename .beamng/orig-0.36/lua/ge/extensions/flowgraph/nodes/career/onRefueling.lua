-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'On Refueling'
C.description = 'Detects various events associated with refueling.'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'out', type = 'flow', name = 'openedMenu', description = "Outflow once when refueling menu is opened.", impulse = true },
  { dir = 'out', type = 'flow', name = 'tankFilled', description = "Outflow once when a tank is completely filled", impulse = true },
  { dir = 'out', type = 'flow', name = 'currentlyFueling', description = "Outflow if fueling is currentl active" },
  { dir = 'out', type = 'number', name = 'tankPercent', description = "Filled percentage of the tank after filling"},
  { dir = 'out', type = 'flow', name = 'paidPrice', description = "Outflow once when the player paid the price", impulse = true },
  { dir = 'out', type = 'flow', name = 'closedMenu', description = "Outflow once when refueling menu is closed.", impulse = true },
}
C.dependencies = {}


function C:init()
  self.flags = {}
end

function C:work(args)
  for _, pin in pairs(self.pinOut) do
    if pin.type == "flow" then
      pin.value = false
    end
  end
  for flag, act in pairs(self.flags) do
    self.pinOut[flag].value = act
  end

  self.pinOut.currentlyFueling.value = career_modules_fuel.isCurrentlyFueling()
  local fuelData = career_modules_fuel.getFuelData()
  if fuelData then
    local cur, max = 0,0
    for index, data in ipairs(fuelData or {} ) do
      cur, max = cur + data.currentEnergy, max + data.maxEnergy
    end
    self.pinOut.tankPercent.value = cur / max
  else
    --self.pinOut.tankPercent.value = 0
  end
  table.clear(self.flags)
end

function C:onRefuelingStartTransaction(data)
  self.flags.openedMenu = true
end

function C:onRefuelingStopFueling(data)
  self.flags.tankFilled = true
  --self.flags.tankPercent = 0
  --if data and data.currentEnergy and data.maxEnergy then
  --  self.flags.tankPercent = data.currentEnergy / data.maxEnergy
  --end
end

function C:onPaidRefuelling(data)
  self.flags.paidPrice = true
end

function C:onRefuelingEndTransaction()
  self.flags.closedMenu = true
end

return _flowgraph_createNode(C)