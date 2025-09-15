-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = 'auxiliary'
local logTag = 'vehicleSystemsCoupling'

local couplingController = nil
local previousConfig = nil

local function startCoupling(config)
  previousConfig = config
  if couplingController ~= nil then
    log('E', logTag, 'Coupling controller already exists. Please call `tech_vehicleSystemsCoupling.stopCoupling()` first.')
    return
  end

  if config and config.skipControllerLoad then
    couplingController = 'coupling'
    return
  end

  if controller.getController('vehicleSystemsCoupling') == nil then
    controller.loadControllerExternal('tech/vehicleSystemsCoupling', 'vehicleSystemsCoupling', {loadedByExtension = true})
  end

  couplingController = controller.getController('vehicleSystemsCoupling')
  couplingController.initialSetup(config)
end

local function stopCoupling()
  if couplingController == nil then
    log('E', logTag, 'Coupling controller does not exist.')
    return
  end
  if couplingController == 'coupling' then
    couplingController = controller.getController('vehicleSystemsCoupling')
  end
  couplingController.stopCoupling()
  couplingController = nil
  controller.unloadControllerExternal('vehicleSystemsCoupling')
end

local function onExtensionUnloaded()
  if couplingController == nil then return end

  stopCoupling()
end

local function onReset()
  log('I', logTag, 'Reset extension.')
  if couplingController == nil then return end

  stopCoupling()
  startCoupling(previousConfig)
end

local function onSerialize()
  return {
    previousConfig = previousConfig
  }
end

local function onDeserialized(data)
  previousConfig = data.previousConfig
end


M.startCoupling = startCoupling
M.stopCoupling = stopCoupling
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onExtensionUnloaded = onExtensionUnloaded
M.onReset = onReset

return M