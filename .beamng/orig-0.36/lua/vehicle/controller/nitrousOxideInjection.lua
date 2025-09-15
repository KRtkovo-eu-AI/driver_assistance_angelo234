-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.relevantDevice = "mainEngine"

local name = nil
local hasBuiltPie = false
local armElectricsName = nil
local overrideElectricsName
local engine = nil
local purgeTime = 0
local n2oData = {}
local simpleButtonColorInactive = "FFFFFF"
local simpleButtonColorArmed = "98FB00"
local simpleButtonColorActive = "3096F1"

local function updateSimpleControlButtons()
  local color = simpleButtonColorInactive
  if n2oData.isArmed then
    color = simpleButtonColorArmed
  end
  if n2oData.isActive then
    color = simpleButtonColorActive
  end

  extensions.ui_simplePowertrainControl.setButton(M.name, "N2O", "powertrain_n2o", color, n2oData.tankRatio, string.format("controller.getController(%q).toggleActive()", M.name))
end

local function updateGFX(dt)
  local tankRatio = engine.nitrousOxideInjection.getTankRatio()
  local isArmed = engine.nitrousOxideInjection.isArmed
  local isActive = engine.nitrousOxideInjection.isActive
  if tankRatio ~= n2oData.tankRatio or isArmed ~= n2oData.isArmed or isActive ~= n2oData.isActive then
    n2oData.tankRatio = tankRatio
    n2oData.isArmed = isArmed
    n2oData.isActive = isActive
    updateSimpleControlButtons()
  end
end

local function displayState()
  guihooks.message("Nitrous Oxide Injection: " .. ((electrics.values[armElectricsName] or 0) >= 1 and "Armed" or "Disarmed"), 5, "vehicle.powertrain.nitrousOxideInjection")
end

local function setOverride(active)
  electrics.values[overrideElectricsName] = active and 1 or 0
end

local function toggleActive()
  if electrics.values[armElectricsName] == 0 then
    engine.nitrousOxideInjection.purgeLines(purgeTime)
  end
  electrics.values[armElectricsName] = 1 - (electrics.values[armElectricsName] or 0)
  displayState()
end

local function serialize()
  return {
    isArmed = electrics.values[armElectricsName]
  }
end

local function deserialize(data)
  if data and data.isArmed then
    electrics.values[armElectricsName] = data.isArmed
  end
end

local function reset(jbeamData)
  n2oData = {}
end

local function init(jbeamData)
  M.updateGFX = nop

  name = jbeamData.name
  armElectricsName = jbeamData.electricsArmName or "nitrousOxideArm"
  overrideElectricsName = jbeamData.electricsOverrideName or "nitrousOxideOverride"
  purgeTime = jbeamData.purgeTime or 1
  local engineName = jbeamData.engineName or "mainEngine"
  electrics.values[armElectricsName] = electrics.values[armElectricsName] or 0

  engine = powertrain.getDevice(engineName)
  local hasNitrousOxideInjection = engine and engine.nitrousOxideInjection and engine.nitrousOxideInjection.isExisting
  if hasNitrousOxideInjection then
    M.updateGFX = updateGFX

    if not hasBuiltPie then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          generator = function(entries)
            local noEntry = {
              title = "Nitrous Oxide",
              priority = 40,
              icon = "N2OHoriz",
              uniqueID = "n2oToggle",
              onSelect = function()
                controller.getController(name).toggleActive()
                return {"reload"}
              end
            }
            if electrics.values[armElectricsName] >= 1 then
              noEntry.color = "#ff6600"
            end
            table.insert(entries, noEntry)
          end
        }
      )
    end
    hasBuiltPie = true
  end

  displayState()
end

M.init = init
M.reset = reset
M.updateGFX = nop
M.setOverride = setOverride
M.toggleActive = toggleActive
M.serialize = serialize
M.deserialize = deserialize

M.updateSimpleControlButtons = updateSimpleControlButtons

return M
