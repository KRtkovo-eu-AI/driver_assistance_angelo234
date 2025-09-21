-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"util_stepHandler"}
local active
local vehicleCount = 1
local setupSteps = nil
local state = "none"
local vehicleIds = {}

local function activate()
  if active then return end
  log("I","","Activating g2g backrgound activity")
  active = true
  local stepper = util_stepHandler

  local sequence = {stepper.makeStepFadeToBlack()}
  local vehOptions = M.getVehicleOptions()
  math.randomseed(os.time())
  for i = 1, vehicleCount do
    local opt = vehOptions[math.random(#vehOptions)]
    local cnf = opt.configs[math.random(#opt.configs)]
    table.insert(sequence,stepper.makeStepSpawnVehicleSimple(opt.model, cnf.config,function(step, vehId) table.insert(vehicleIds, {vehId = vehId, idx = i}) end))
  end
  table.insert(sequence,stepper.makeStepFadeFromBlack())
  stepper.startStepSequence(sequence, M.activateFinished)
end
M.activateFinished = function()
  log("I","","Finished loading g2g background activity")
  active = true
end

local function deactivate()
  if not active then return end
  log("I","","deactivating g2g backrgound activity")
  for _, data in ipairs(vehicleIds) do
    local obj = scenetree.findObjectById(data.vehId)
    if obj then
      if editor and editor.onRemoveSceneTreeObjects then
        editor.onRemoveSceneTreeObjects({data.vehId})
      end
      obj:delete()
    end
  end
  table.clear(vehicleIds)
end

local function toggleActive()
  if active then
    deactivate()
  else
    activate()
  end
end

local lastUpdateTimer = updateTime
local function onUpdate(dtReal, dtSim, dtRaw)
end

local function onClientEndMission()
  deactivate()
end

local function onVehicleSwitched()
  --deactivate()
end

local function isActive()
  return active
end

local function onSerialize()
  deactivate()
end

local function onDeserialized()
end

local function onCareerActive(enabled)
  if enabled then
    deactivate()
  end
end

-- helper functions
local vehicleOptions
local function getVehicleOptions()
  if vehicleOptions == nil then
    local vehs =  core_vehicles.getVehicleList().vehicles
    local mode = 'Default'
    vehicleOptions = {}
    for _, v in ipairs(vehs) do
      local passType = true
      passType = passType and (v.model.Type == 'Car' or v.model.Type == 'Truck') and v.model['Body Style'] ~= 'Bus' and v.model['isAuxiliary'] ~= true -- always only use cars or trucks
      if mode == "Old Cars" then passType = passType and v.model.Years and v.model.Years.max and v.model.Years.max <= 1985 end
      if mode == "New Cars" then passType = passType and v.model.Years and v.model.Years.min and v.model.Years.min > 1985 end
      if mode == "Mod Cars Only" then passType = passType and not v.model.Author end
      if mode == "Mod Cars Only" then passType = passType and v.model.Author and v.model.Author ~= "BeamNG" end
      -- always only use cars.
      if passType then
        local model = {
          model = v.model.key,
          configs = {},
          paints = tableKeys(tableValuesAsLookupDict(v.model.paints or {}))
        }
        for _, c in pairs(v.configs) do
          local passConfig = true
          passConfig = passConfig and c["Top Speed"] and c["Top Speed"] > 10 and c['isAuxiliary'] ~= true -- always have some minimum speed
          if mode == "Race and Rally" then passConfig = passConfig and (c['Config Type'] == 'Race' or c['Config Type'] == 'Rally') end
          if mode == "RWD Only" then passConfig = passConfig and c["Drivetrain"] == 'RWD' end
          --Offroad cars
          if mode == "Off-road" then passConfig = passConfig and c["Off-Road Score"] >= 50 end
          if passConfig then
            table.insert(model.configs, {
              config = c.key,
              name = c.Name,
            })
          end
        end
        if #model.configs > 0 then
          table.insert(vehicleOptions, model)
        end
      end
    end
  end
  return vehicleOptions
end
M.getVehicleOptions = getVehicleOptions

M.activate = activate
M.deactivate = deactivate
M.toggleActive = toggleActive
M.isActive = isActive


M.onUpdate = onUpdate
M.onClientEndMission = onClientEndMission
M.onVehicleSwitched = onVehicleSwitched
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onCareerActive = onCareerActive


return M