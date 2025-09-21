-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Generate Vehicle Group'
C.description = 'Generates vehicle group data, to be used with the Spawn Vehicle Group node.'
C.color = ui_flowgraph_editor.nodeColors.traffic
C.icon = ui_flowgraph_editor.nodeIcons.traffic
C.category = 'provider'
C.tags = {'spawn', 'vehicle', 'group', 'traffic', 'multispawn'}

C.pinSchema = {
  {dir = 'out', type = 'table', name = 'group', tableType = 'vehicleGroupData', description = 'Vehicle group data.'}
}

local modes = {settings = 'Parameters', custom = 'Custom'}

function C:init()
  self.mode = 'settings'
  self.count = 0
  self:resetData()
end

function C:resetData()
  self.params = {auto = true, allMods = false, allConfigs = false, simpleVehs = false}
end

function C:drawCustomProperties()
  -- mode select
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.BeginCombo('Mode', modes[self.mode]) then
    if im.Selectable1('Parameters', self.mode == 'settings') then
      self.mode = 'settings'
      self:updatePins(self.count, 0)
      self:resetData()
    end
    if im.Selectable1('Custom', self.mode == 'custom') then
      self.mode = 'custom'
      self:updatePins(self.count, 1)
      self:resetData()
    end
    im.EndCombo()
  end
  im.PopItemWidth()

  if self.mode == 'settings' then
    local var = im.BoolPtr(self.params.auto)
    if im.Checkbox('Use Game Options Only##groupGenerator', var) then
      self.params.auto = var[0]
    end

    if self.params.auto then
      im.BeginDisabled()
    end
    var = im.BoolPtr(self.params.allMods)
    if im.Checkbox('Allow Mods##groupGenerator', var) then
      self.params.allMods = var[0]
    end
    var = im.BoolPtr(self.params.allConfigs)
    if im.Checkbox('Use All Configs##groupGenerator', var) then
      self.params.allConfigs = var[0]
    end
    var = im.BoolPtr(self.params.simpleVehs)
    if im.Checkbox('Use Simple Vehicles##groupGenerator', var) then
      self.params.simpleVehs = var[0]
    end
    if self.params.auto then
      im.EndDisabled()
    end
  else
    -- select amount of pins
    local count = im.IntPtr(self.count)
    if im.InputInt('Count', count) then
      self:updatePins(self.count, math.max(1, count[0]))
    end
  end
end

function C:onStateStarted()
  self.done = false
end

function C:updatePins(old, new)
  if new < old then
    for i = old, new + 1, -1 do
      for _, link in pairs(self.graph.links) do
        if link.targetPin == self.pinInLocal['model_'..i]then
          self.graph:deleteLink(link)
        end
        if link.targetPin == self.pinInLocal['config_'..i] then
          self.graph:deleteLink(link)
        end
      end
      self:removePin(self.pinInLocal['model_'..i])
      self:removePin(self.pinInLocal['config_'..i])
    end
  else
    for i = old + 1, new do
      -- direction, type, name, default, description, autoNumber
      self:createPin('in', 'string', 'model_'..i)
      self:createPin('in', {'string', 'table'}, 'config_'..i)
    end
  end
  self.count = new
end

function C:_executionStarted()
  self.done = false
end

function C:buildGroup() -- builds vehicle group from pin inputs
  self.group.name = 'Custom Group'
  self.group.type = 'custom'
  self.group.data = {}
  for i = 1, self.count do
    local model, config = self.pinIn['model_'..i].value, self.pinIn['config_'..i].value
    if model then
      table.insert(self.group.data, {model = model, config = config})
    end
  end
end

function C:generateGroup() -- builds vehicle group from settings
  self.group.name = 'Custom Group'
  self.group.type = 'generator'
  self.group.generator = deepcopy(self.params)
  if self.group.generator.auto then
    self.group.data = gameplay_traffic_trafficUtils.createTrafficGroup(20)
  else
    self.group.data = gameplay_traffic_trafficUtils.createTrafficGroup(20, self.group.generator.allMods, self.group.generator.allConfigs, self.group.generator.simpleVehs)
  end
end

function C:work()
  -- only set out pin once per execution.
  if not self.done then
    self.group = {}
    if self.mode == 'settings' then
      self:generateGroup()
    else
      if not self.pinIn['model_'..self.count].value then return end -- delay until value is ready, just in case
      self:buildGroup()
    end
    if not self.group.data or not self.group.data[1] then -- group data needs to exist for output
      log('W', 'trafficCustomGroup', 'Invalid vehicle group data, now creating default data')
      self.group.data = core_multiSpawn.createGroup(20)
    end
    self.pinOut.group.value = self.group
    self.done = true
  end
end

function C:_onSerialize(res)
  res.mode = self.mode

  if self.mode == 'settings' then
    res.params = self.params
  else
    res.count = self.count
  end
end

function C:_onDeserialized(data)
  self.mode = data.mode
  if self.mode == 'file' then
    log('W', 'trafficCustomGroup', 'File mode is no longer supported for this node; please use File Vehicle Group')
    self.mode = 'settings'
  end
  if self.mode == 'settings' then
    self.params = data.params
    if not self.params then
      self:resetData()
    end
  else
    self:updatePins(self.count, data.count)
  end
end

return _flowgraph_createNode(C)