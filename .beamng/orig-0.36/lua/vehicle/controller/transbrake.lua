-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local gearbox = nil
local electricsName = nil
local hasBuiltPie = false

local function updateGFX(dt)
  electrics.values[electricsName] = gearbox.lockCoef == 0
end

local function setTransbrake(value)
  if gearbox then
    gearbox:setLock(value)
    guihooks.message("Transbrake is " .. (value and "enabled" or "disabled"), 2, "vehicle.transbrake.status")
  end
end

local function toggleTransbrake()
  if gearbox then
    local enabled = gearbox.lockCoef == 0
    setTransbrake(not enabled)
  end
end

local function init(jbeamData)
  local gearboxName = jbeamData.gearboxName or "gearbox"
  electricsName = jbeamData.electricsName or "transbrake"
  gearbox = powertrain.getDevice(gearboxName)
  M.updateGFX = gearbox and updateGFX or nop

  if not hasBuiltPie then
    if gearbox then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          generator = function(entries)
            local noEntry = {
              title = "Transbrake",
              priority = 40,
              icon = "powerOnOff",
              uniqueID = "transbrakeToggle",
              onSelect = function()
                controller.getController("transbrake").toggleTransbrake()
                return {"reload"}
              end
            }
            if gearbox.lockCoef == 0 then
              noEntry.color = "#ff6600"
            end
            table.insert(entries, noEntry)
          end
        }
      )
    end
    hasBuiltPie = true
  end
end

--example: onGameplayEvent("controller:transbrake", "setTransbrake", true)
--example: onGameplayEvent("controller:myTransbrakeName", "setTransbrake", false)
local function onGameplayEvent(event, ...)
  local splits = split(event, ":")
  if splits[1] == "controller" and (splits[2] == M.typeName or splits[2] == M.name) then
    local params = {...}
    local dataTypeCheck, dataTypeError = checkTableDataTypes(params, {"string", "boolean"})
    if not dataTypeCheck then
      log("E", "transbrake.onGameplayEvent", dataTypeError)
      return
    end
    if params[1] == "setTransbrake" then
      setTransbrake(params[2])
    end
  end
end

M.init = init
M.updateGFX = nop
M.toggleTransbrake = toggleTransbrake
M.setTransbrake = setTransbrake
M.onGameplayEvent = onGameplayEvent

return M
