-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Generate Drag Race Opponents'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.description = "Gives you a random amount of vehicles configurations determinated by the player vehicle."
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', default = 0, description = "Vehicle ID. If not present, player vehicle will be used." },
  { dir = 'in', type = 'number', name = 'numberOfOpponents', default = 1, description = 'Number of generated vehicles.' },
  { dir = 'in', type = 'number', name = 'playerDial', default = -1, description = '' },
  { dir = 'out', type = 'table', name = 'vehicleGroup', description = '' },
}

C.tags = {}

function C:init()
  self.selectedOpponents = {}
  self.playerId = -1
end

function C:drawCustomProperties()
  local reason = nil
  return reason
end

function C:drawMiddle(builder, style)
  builder:Middle()
end

function C:selectVehicle()
  local randomVehicles = {}
  local configs = core_vehicles.getConfigList()
  local dial = self.pinIn.playerDial.value or -1

  --Get the possible vehicle configurations
  local vehConfigs = {}
  for i,v in pairs(configs.configs) do
    local model = core_vehicles.getModel(v.model_key).model

    local passType = true
    passType = passType and (model.Type == 'Car' or model.Type == 'Truck') and model['isAuxiliary'] ~= true -- always only use cars or trucks
    if passType and not string.match(i, 'simple_traffic') then
      table.insert(vehConfigs, v)
    end
  end
  if gameplay_drag_general.getData().dragType == "bracketRace" then
    for i = 1, self.pinIn.numberOfOpponents.value do
      math.randomseed(os.time())
      local selectedConfig = vehConfigs[math.random(#vehConfigs)]
      local m = selectedConfig.model_key
      local c = selectedConfig.key
      local p = tableKeys(tableValuesAsLookupDict(core_vehicles.getModel(selectedConfig.model_key).model.paints or {}))
      local n = selectedConfig.Name
      table.insert(randomVehicles, {
            model = m,
            config = c,
            paint = p[math.random(#p)],
            dial = selectedConfig["Drag Times"].time_1_4 - 0.5
          })
    end
    return randomVehicles
  end

  local currentVeh = core_vehicles.getCurrentVehicleDetails()
  if dial < 0 then
    --Get the data from the savefile just to know the timesTable
    local configTimes = gameplay_drag_general.getDialTimes()

    local currentConfig = gameplay_drag_general.generateHashFromFile()

    if configTimes[currentConfig] then
      dial = configTimes[currentConfig].time_1_4
    else
      if currentVeh.configs then
        dial = currentVeh.configs["Drag Times"] and  currentVeh.configs["Drag Times"].time_1_4 or 12
      end
    end
    log("I","","Player dial time: " .. dial)
  end

  local similarVehicles = {}
  local similarVehicleCount = 0


  for i,v in pairs(vehConfigs) do
    if (v["Drag Times"])then
      if v["Drag Times"].time_1_4 >= dial - 0.5 and v["Drag Times"].time_1_4 < dial + 0.1 then
        table.insert(similarVehicles, v)
        similarVehicleCount = similarVehicleCount + 1
      end
    end
  end

  if similarVehicleCount == 0 then
    table.insert(similarVehicles, currentVeh.configs)
    similarVehicleCount = similarVehicleCount + 1
  end

  --Add a random selection of vehicles
  for i = 1, self.pinIn.numberOfOpponents.value do
    local selectedConfig = similarVehicles[math.random(similarVehicleCount)]
    local m = selectedConfig.model_key
    local c = selectedConfig.key
    local p = tableKeys(tableValuesAsLookupDict(core_vehicles.getModel(selectedConfig.model_key).model.paints or {}))
    local n = selectedConfig.Name
    table.insert(randomVehicles, {
          model = m,
          config = c,
          paint = p[math.random(#p)],
          dial = selectedConfig["Drag Times"].time_1_4
        })
  end
  return randomVehicles
end

function C:_executionStarted()
  self.selectedOpponents = {}
  self.playerId = 0
end

function C:workOnce()
  math.randomseed(os.time())
  local group = self:selectVehicle()
  self.pinOut.vehicleGroup.value = group
end

return _flowgraph_createNode(C)
