-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- this extension helps scenario creators prevent the execution of stuff before the scenario has started (during intro screen/camera)
-- e.g. if you want the cannon not to fire the ball, while user is reading the introduction screen

local functionNames = {} -- stuff the scenario creator doesn't want to execute. e.g. { "hydro_input.liftUp", "controller.shiftDown" }
local originalFreezeFunction = nil -- used to revert back situation when unloading this extension

-- enables or disables the freezing (the ignore) of functions listed in functionNames
local function setFreeze(mode)
  if mode == 1 then -- freeze everything
    for _,funcString in pairs(functionNames) do
      if loadstring("return "..funcString.."_freeze_backup")() == nil then
        loadstring(funcString.."_freeze_backup = "..funcString)() -- backup the original function code
        loadstring(funcString.." = nop")() -- now replace function with no operation
      else
        log("D", "functionFreezer", "Attempted to freeze a function that is already frozen. Ignoring... "..dumps(funcString))
      end
    end
  else -- revert function to its original code
    for _,funcString in pairs(functionNames) do
      if loadstring("return "..funcString.."_freeze_backup")() == nil then
        log("E", "functionFreezer", "Attempted to unfreeze a function that was never frozen. Ignoring... "..dumps(funcString))
      else
        loadstring(funcString.." = "..funcString.."_freeze_backup")() -- restore backup code back to its original place
        loadstring(funcString.."_freeze_backup".. " = nil")() -- mark as unfrozen
      end
    end
  end
end


-- start overloading controller.freeze with our own replacement, so we can ignore scenario-defined functions
local function onExtensionLoaded()
  originalFreezeFunction = controller.setFreeze
  controller.setFreeze = function(mode)
    -- our replacement function calls the original freeze function, if present:
    if originalFreezeFunction then
      originalFreezeFunction(mode)
    end
    -- then we actually call our custom freezing code
    setFreeze(mode)
  end
end

-- stop overloading controller.freeze, let it work as originally intended from now on
local function onExtensionUnloaded()
  setFreeze(0)
  controller.setFreeze = originalFreezeFunction
end

-- parse scenario configuration json, which lists all what we want to freeze
local function onVehicleScenarioData(data)
  setFreeze(0) -- revert all functions' code before starting from scratch, just in case
  functionNames = {}
  -- parse function names, so we know what must be frozen/unfrozen in the future
  for _,functionString in ipairs(data) do
    local func = loadstring("return "..functionString)() -- func is only used for sanitization checks
    if func then
      if type(func) == "function" then
        table.insert(functionNames, functionString)
      else
        log("E", "functionFreezer", "Cannot add action to freeze list (\""..dumps(functionString).."\"), it's not a function: "..dumps(func))
      end
    else
      log("E", "functionFreezer", "Cannot add action to freeze list, it doesn't exist: "..dumps(functionString))
    end
  end
  log("D", "functionFreezer", "Added "..dumps(tableSize(functionNames)).." functions to freeze list")
end

M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onVehicleScenarioData = onVehicleScenarioData

return M
