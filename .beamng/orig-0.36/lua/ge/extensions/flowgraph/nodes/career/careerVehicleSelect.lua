-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Career Vehicle Select'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career
C.description = "Lets the player pick between 3 vehicles.."
C.behaviour = { once = true, singleActive = true}
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = 'Inflow for this node.' },
  { dir = 'in', type = 'flow', name = 'reset', description = 'Resets this node.', impulse = true },
  { dir = 'in', type = {'string', 'table'}, tableType = 'multiTranslationObject', name = 'description', description = 'Subtext of the menu.' },

  { dir = 'in', type = 'string', name = 'veh1image', description = '' },
  { dir = 'in', type = 'string', name = 'veh1name', description = '' },
  { dir = 'in', type = 'string', name = 'veh1brand', description = '' },
  { dir = 'in', type = 'string', name = 'veh1desc', description = '' },
  { dir = 'in', type = 'bool'  , name = 'veh1testDisabled', description = '' },

  { dir = 'in', type = 'string', name = 'veh2image', description = '' },
  { dir = 'in', type = 'string', name = 'veh2name', description = '' },
  { dir = 'in', type = 'string', name = 'veh2brand', description = '' },
  { dir = 'in', type = 'string', name = 'veh2desc', description = '' },
  { dir = 'in', type = 'bool'  , name = 'veh2testDisabled', description = '' },

  { dir = 'in', type = 'string', name = 'veh3image', description = '' },
  { dir = 'in', type = 'string', name = 'veh3name', description = '' },
  { dir = 'in', type = 'string', name = 'veh3brand', description = '' },
  { dir = 'in', type = 'string', name = 'veh3desc', description = '' },
  { dir = 'in', type = 'bool'  , name = 'veh3testDisabled', description = '' },


  { dir = 'out', type = 'flow', name = 'test1', description = 'When vehicle 1 has been selected for test.' },
  { dir = 'out', type = 'flow', name = 'pick1', description = 'when vehicle 1 has been selected for pick.' },
  { dir = 'out', type = 'flow', name = 'test2', description = 'When vehicle 2 has been selected for test.' },
  { dir = 'out', type = 'flow', name = 'pick2', description = 'when vehicle 2 has been selected for pick.' },
  { dir = 'out', type = 'flow', name = 'test3', description = 'When vehicle 3 has been selected for test.' },
  { dir = 'out', type = 'flow', name = 'pick3', description = 'when vehicle 3 has been selected for pick.' },

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

  local data = {
    description = self.pinIn.description.value,
    vehicles = {}
  }
  for _, i in ipairs({1,2,3}) do
    local veh = {}
    for _, key in ipairs({"image","brand","name","desc"}) do
      veh[key] = self.pinIn["veh"..i..key].value
    end
    veh.testButtonCode = self:getCmd('test'..i)
    veh.pickButtonCode = self:getCmd('pick'..i)
    veh.testDisabled = self.pinIn["veh"..i.."testDisabled"].value or false
    data.vehicles[i] = veh

  end

  self._storedData = data
  self._active = true
  guihooks.trigger('ChangeState', {state = 'menu.careerVehicleSelect', params = {data = data}})
end


function C:closed()
  self.done = true
  self._active = false
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
