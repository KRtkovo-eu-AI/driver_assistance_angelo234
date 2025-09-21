-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 40

local abs = math.abs
local ceil = math.ceil

local name = nil
local lockedLines = nil
local lockedBrakingTorques = nil
local lineLockActive = 0
local electricsName = nil
local hasBuiltPie = false

local wheelRotatorCountDec = 0

local wheelNamesString

local function updateWheelsIntermediate(dt)
  if lineLockActive > 0 then
    for i = 0, wheelRotatorCountDec, 1 do
      local wd = wheels.wheelRotators[i]
      wd.desiredBrakingTorque = lockedLines[wd.name] and lockedBrakingTorques[wd.name] or wd.desiredBrakingTorque
    end
  end
end

local function updateGFX(dt)
  electrics.values[electricsName] = lineLockActive
end

local function setLineLock(value)
  lineLockActive = value

  for i = 0, wheels.wheelRotatorCount - 1, 1 do
    local wd = wheels.wheelRotators[i]
    if lockedLines[wd.name] then
      lockedBrakingTorques[wd.name] = lineLockActive > 0 and abs(wd.desiredMainBrakingTorque) or 0
    end
  end

  if value >= 1 then
    local inputPercentage = ceil(electrics.values.brake * 100)
    guihooks.message(string.format("Linelock: Enabled (%s brake lines locked at %d%%)", wheelNamesString, inputPercentage), 3, "vehicle.linelock.status")
  else
    guihooks.message("Linelock: Disabled", 2, "vehicle.linelock.status")
  end
end

local function toggleLineLock()
  lineLockActive = 1 - lineLockActive
  setLineLock(lineLockActive)
end

local function init(jbeamData)
  name = jbeamData.name

  lockedLines = {}
  lockedBrakingTorques = {}
  wheelNamesString = ""
  for _, v in pairs(jbeamData.lockedLines or {}) do
    lockedLines[v] = true
    lockedBrakingTorques[v] = 0
    wheelNamesString = wheelNamesString .. v .. ", "
  end
  wheelNamesString = wheelNamesString:sub(0, wheelNamesString:len() - 2)

  wheelRotatorCountDec = wheels.wheelRotatorCount - 1

  electricsName = jbeamData.electricsName or "linelock"

  if not hasBuiltPie then
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/vehicleFeatures/",
        generator = function(entries)
          local noEntry = {
            title = "Line Lock",
            priority = 40,
            icon = "drag01",
            uniqueID = "lineLockToggle",
            onSelect = function()
              controller.getController(name).toggleLineLock()
              return {"reload"}
            end
          }
          if electrics.values[electricsName] >= 1 then
            noEntry.color = "#ff6600"
          end
          table.insert(entries, noEntry)
        end
      }
    )
  end
  hasBuiltPie = true

  lineLockActive = 0
end

M.init = init
M.updateGFX = updateGFX
M.updateWheelsIntermediate = updateWheelsIntermediate
M.setLineLock = setLineLock
M.toggleLineLock = toggleLineLock

return M
