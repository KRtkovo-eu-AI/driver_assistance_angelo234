-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local nameVidMap = {}

local function executeCommand(context, cmd)
  extensions.hook('onConsoleExecuteCommand', context, cmd)
  --log('D', 'winConsole', 'executeCommand: ' .. tostring(context) .. ' - ' .. tostring(cmd))
  if context == 'GE-Lua' then
    -- executed in c++, it does not reach here
  elseif context == 'GE-TorqueScript' then
    TorqueScript.eval(cmd)
  elseif context == 'CEF/UI - JS' then
    be:queueJS(cmd)
  else if nameVidMap[context] then
    local veh = getObjectByID(nameVidMap[context])
    if veh then
      veh:queueLuaCommand(cmd)
    end
  end
  end
end

local function getNameForVid(vid)
  local vdata = extensions.core_vehicle_manager.getVehicleData(vid)
  local res = vid
  if vdata and vdata.mainPartName then
    res = vdata.mainPartName
  end
  return res
end

local function refreshCombo()
  consoleClearAvailableContexts()
  consoleAddAvailableContext('GE-Lua')
  local vehCount = be:getObjectCount()
  local playerVid = nil
  if vehCount > 0  then
    local playerVehicle = getPlayerVehicle(0)
    if playerVehicle and playerVehicle:getActive() then
      local name = "Current Vehicle - Lua"
      consoleAddAvailableContext(name)
      playerVid = playerVehicle:getID()
      nameVidMap[name] = playerVid
    end
    for i=0,vehCount-1 do
      local v = be:getObject(i)
      if v:getActive() then
        local vid = v:getID()
        if vid ~= playerVid then
          local name = getNameForVid(vid)
          consoleAddAvailableContext(name)
          nameVidMap[name] = v:getID()
        end
      end
    end
  end
  consoleAddAvailableContext('GE-TorqueScript')
  consoleAddAvailableContext('CEF/UI - JS')
end

local function onExtensionLoaded()
  if type(consoleClearAvailableContexts) == 'nil' then
    -- sorry, not available on your platform :(
    return false
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.configure = refreshCombo -- callback used by C++ when the console window is activated
M.executeCommand = executeCommand -- callback used by C++

M.onVehicleDestroyed = refreshCombo
M.onVehicleSwitched = refreshCombo
M.onVehicleSpawned = refreshCombo
M.onVehicleActiveChanged = refreshCombo

return M
