-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Spawn Vehicle'
C.description = 'Spawns a vehicle.'
C.color = ui_flowgraph_editor.nodeColors.vehicle
C.icon = ui_flowgraph_editor.nodeIcons.vehicle
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = 'In flow pin to trigger this node' },
  { dir = 'in', type = 'flow', name = 'reset', hidden = true, description = 'Removes the vehicle and resets this node.', impulse = true },
  { dir = 'in', type = 'string', name = 'model', default = 'pickup', description = 'The model of the object to use' },
  { dir = 'in', type = { 'string', 'table' }, name = 'config',  tableType = 'vehicleConfig', default = 'vehicles/pickup/d15_4wd_A.pc', description = 'The configuration of the object to use. Check VehicleConfigProvider node.' },
  { dir = 'in', type = 'vec3', name = 'pos', description = '(Optional) Where the vehicle is spawned in the world.' },
  { dir = 'in', type = 'quat', name = 'rot', description = '(Optional) How the vehicle will be rotated in the world.' },
  { dir = 'in', type = 'string', name = 'licenseText', default = 'Flow', hidden = true, description = '(Optional) The text to use for the license plate. Check VehicleConfigProvider node.' },
  { dir = 'in', type = {'color', 'string'}, name = 'color', default = { 0.25, 0.25, 0.25, 0.5 }, hidden = true, description = '(Optional) The color to assign to the object.' },
  { dir = 'in', type = 'bool', name = 'randomColor', hidden = true, description = 'If true, the vehicles color will be randomly picked from the available colors.' },
  { dir = 'in', type = 'string', name = 'name', default = nil, hidden = true, description = '(Optional) The name to assign the object.' },
  { dir = 'in', type = 'bool', name = 'replacePlayer', hidden = true, default = false, description = '(Optional) Set to true to replace current player vehicle.' },
  { dir = 'in', type = 'bool', name = 'keepCamera', hidden = true, default = true, description = 'If true, the camera will remain in position or on the current vehicle.' },
  { dir = 'in', type = 'bool', name = 'keepVehicle', hidden = true, default = true, description = 'If true, the vehicle will not be deleted when you stop the project.' },
  { dir = 'in', type = 'bool', name = 'keepOnReset', hidden = true, default = true, description = 'If true, the vehicle will not be deleted when you reset the node.' },

  {dir = 'out', type = 'flow', name = 'flow', default = false, description = 'Continues flow only when vehicle is loaded.'},
  {dir = 'out', type = 'flow', name = 'loaded', default = false, description = 'Triggers once after the vehicle is loaded or found in the scenetree.', impulse = true},
  {dir = 'out', type = 'number', name = 'vehId', default = nil, description = 'The id of the spawned object.'},
}
C.legacyPins = {
  out = {
    objectId = 'vehId'
  },
  _in = {
    replaceCurrent = 'replacePlayer',
    keepCurrent = 'keepCamera',
    dontDelete = 'keepVehicle',
  }
}

C.tags = {'gameplay', 'utils'}

function C:init()
  self.state = 1
  self.pinOut.flow.value = false
end

function C:postInit()
  self.pinInLocal.color.colorSetup = {
    vehicleColor = true
  }
end

function C:_executionStopped()
  self.spawnedObjectId = nil
  self.state = 1
  self.pinOut.flow.value = false
  self.veh = nil
end

function C:_executionStarted()
  self.spawningOptions = nil
  self.spawnedObjectId = nil
  self.state = 1
  self.pinOut.flow.value = false
  self.pinOut.loaded.value = false
  self._dontDelete = false
end

function C:work()
  if self.pinIn.reset.value then
    if self.spawnedObjectId and not self.pinIn.replacePlayer.value then -- if replacing the player vehicle, we don't need to delete the current vehicle
      local obj = scenetree.findObjectById(self.spawnedObjectId)
      if obj and not self.pinIn.keepOnReset.value then
        if editor and editor.onRemoveSceneTreeObjects then
          editor.onRemoveSceneTreeObjects({obj:getId()})
        end
        obj:delete()
      end
    end

    self.spawningOptions = nil
    self.spawnedObjectId = nil
    self.state = 1
    self.pinOut.flow.value = false
    self.pinOut.loaded.value = false
    self.veh = nil
    return
  end
  if self.pinIn.flow.value then
    -- state 1: spawning state
    if self.state == 1 then
      -- create a name if we have none.
      local name = self.pinIn.name.value or generateObjectNameForClass('BeamNGVehicle', 'object_')

      -- load model and config. maybe read config from file, if it's not the table.
      local model = self.pinIn.model.value
      local config = self.pinIn.config.value
      local configKey = type(config) == 'string' and config or 'base' -- this is required for function core_vehicles.getConfig
      if string.find(configKey, '.pc') then
        configKey = string.match(configKey, '/*([%w_%-]+).pc')
      end

      --various vehicle data
      local paint
      local paintValue = self.pinIn.color.value
      if self.pinIn.randomColor.value or type(paintValue) == 'string' then
        local allPaints = nil
        local modelData = core_vehicles.getModel(self.pinIn.model.value)
        local configData = core_vehicles.getConfig(model, configKey)
        if modelData and modelData.model then
          if type(paintValue) == 'string' then
            paint = modelData.model.paints[paintValue] -- if paint value is string
          else
            allPaints = tableKeys(tableValuesAsLookupDict(modelData.model.paints or {})) -- if random paint is true
            if configData and configData.aggregates and configData.aggregates["Config Type"] then
              local cType = configData.aggregates["Config Type"]
              if cType["Police"] or cType["Service"] then -- random paint disabled
                allPaints = nil
                self.mgr:logEvent("Vehicle Spawned ".. dumps(self.pinIn.model.value), "I", "Vehicle " ..dumps(self.pinIn.model.value) .. ": ignoring random color.", {type = "node", node = self})
              end
            end
          end
        end

        if allPaints then
          paint = allPaints[math.random(#allPaints)]
        end
      elseif type(paintValue) == 'table' and (paintValue.baseColor or paintValue[4]) then
        paint = paintValue
      end

      if paint and not paint.baseColor then
        paint = createVehiclePaint({x=paint[1], y=paint[2], z=paint[3], w=paint[4]}, {paint[5], paint[6], paint[7], paint[8]})
      end

      local licenseText = self.pinIn.licenseText.value
      local spawnPos = self.pinIn.pos.value and vec3(self.pinIn.pos.value)
      local spawnRot = self.pinIn.rot.value and quat(self.pinIn.rot.value)
      if spawnRot then
        spawnRot = quat(0,0,1,0) * spawnRot -- rotate 180 degrees
      end

      -- create actual spawning data
      local options = {config = config, paint = paint, licenseText = licenseText, vehicleName = name, pos = spawnPos, rot = spawnRot}
      self.spawningOptions = sanitizeVehicleSpawnOptions(model, options)
      --dumpz(self.spawningOptions,2)

      -- set the flag for camera switching
      self.spawningOptions.autoEnterVehicle = not (self.pinIn.keepCamera.value or false)

      -- do the actual spawning/replacing.
      if self.pinIn.replacePlayer.value then
        self.veh = core_vehicles.replaceVehicle(self.spawningOptions.model, self.spawningOptions)
      else
        self.veh = core_vehicles.spawnNewVehicle(self.spawningOptions.model, self.spawningOptions)
        self.mgr:logEvent("Vehicle Spawned ".. dumps(self.pinIn.model.value), "I", "A Vehicle " ..dumps(self.pinIn.model.value) .. " was spawned.", {type = "node", node = self})
      end
      self.mgr.modules.vehicle:addVehicle(self.veh, {dontDelete = self.pinIn.keepVehicle.value})
      self.spawnedObjectId = self.veh:getId()

      --dump("id  is : "..self.veh:getId())
      self._dontDelete = self.pinIn.keepVehicle.value or false
      self.state = 2

      -- state 2: waiting
    elseif self.state == 2 then
      --dump(self.id .. dumps(self.veh))
      if self.mgr.modules.vehicle:getVehicle(self.spawnedObjectId).ready then
        --self.spawnedObjectId = self.veh:getId()
        --print("node id " .. self.id .. " gained vehicle id: " .. self.spawnedObjectId)
        self.state =  3
      end

    elseif self.state == 3 then
      if not (self.pinIn.keepCamera.value or false) and commands.isFreeCamera() then commands.setGameCamera() end -- if free camera was somehow still active

      self.pinOut.loaded.value = true
      self.pinOut.flow.value = true
      self.pinOut.vehId.value = self.spawnedObjectId
      self.state = 4
    elseif self.state == 4 then
      self.pinOut.loaded.value = false
      self.pinOut.flow.value = true
    end
    --print("final state: " .. self.id .. " is " .. dumps(self.state))
  end
end

function C:getCmd(action)
  return 'local n = core_flowgraphManager.getManagerGraphNode('..self.mgr.id..', '..self.graph.id..', '..self.id..') if n then n:buttonPushed("'..action..'") end'
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.Text("State:" .. tostring(self.state))
  im.Text("Veh Id:" .. tostring(self.spawnedObjectId))
end

return _flowgraph_createNode(C)
