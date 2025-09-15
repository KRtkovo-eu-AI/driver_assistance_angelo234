-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local max = math.max

M.controlModeName = nil
M.controlModeIndex = nil
local hasRegisteredQuickAccess = false

local controlModes = nil
local inputs = {}

local configurations = {}

local function updateGFX(dt)
  inputs.steering = input.steering
  inputs.throttle = input.throttle
  inputs.brake = input.brake
  inputs.throttlebrake = input.throttle - input.brake

  local control = controlModes[M.controlModeIndex].electrics
  for _, v in pairs(control) do
    local config = configurations[v]
    electrics.values[v] = min(max(electrics.values[v] + config.smoother:get(inputs[config.input], dt) * config.speed * dt, config.min), config.max)
  end
end

local function setControlModeIndex(index)
  M.controlModeIndex = index
  M.controlModeName = controlModes[M.controlModeIndex].name
  guihooks.message("Control Mode: " .. controlModes[M.controlModeIndex].name, 5, "vehicle.controls.mode")
end

local function toggleControlMode(value)
  M.controlModeIndex = M.controlModeIndex + value
  if M.controlModeIndex > #controlModes then
    M.controlModeIndex = 1
  end
  if M.controlModeIndex <= 0 then
    M.controlModeIndex = #controlModes
  end
  setControlModeIndex(M.controlModeIndex)
end

local function setInputValue(name, value)
  inputs[name] = value
end

local function init(jbeamData)
  controlModes = tableFromHeaderTable(jbeamData.modes)

  local config = tableFromHeaderTable(jbeamData.config)
  configurations = {}
  inputs = {}
  for _, v in pairs(config) do
    configurations[v.electric] = v
    configurations[v.electric].smoother = newTemporalSmoothingNonLinear(configurations[v.electric].smoothIn, configurations[v.electric].smoothOut)
    electrics.values[v.electric] = 0
    inputs[v.input] = 0

    configurations[v.electric].electric = nil
  end

  M.controlModeIndex = jbeamData.defaultControlModeIndex or 1
  M.controlModeName = controlModes[M.controlModeIndex].name

  if not hasRegisteredQuickAccess then
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/general/",
        generator = function(entries)
          if controller.getController("controlModes") then
            table.insert(entries, {title = "Modes", priority = 40, ["goto"] = "/root/playerVehicle/general/controlmodes/", icon = "settings", uniqueID = "controlModes"})
          end
        end
      }
    )

    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/general/controlmodes/",
        generator = function(entries)
          for k, v in pairs(controlModes) do
            local entry = {
              title = v.name,
              originalActionInfo = {level = "/root/playerVehicle/general/", uniqueID = "controlModes"},
              onSelect = function()
                setControlModeIndex(k)
                return {"reload"}
              end
            }
            if M.controlModeIndex == k then
              entry.color = "#ff6600"
            end
            table.insert(entries, entry)
          end
        end
      }
    )
    hasRegisteredQuickAccess = true
  end
end

M.init = init
M.updateGFX = updateGFX
M.toggleControlMode = toggleControlMode
M.setControlModeIndex = setControlModeIndex
M.setInputValue = setInputValue

return M
