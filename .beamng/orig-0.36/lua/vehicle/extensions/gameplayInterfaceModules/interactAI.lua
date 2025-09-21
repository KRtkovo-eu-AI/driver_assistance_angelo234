-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local moduleName = "interactAI"
M.moduleActions = {}
M.moduleLookups = {}

local function setAIMode(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local mode = params[1]
  ai.setMode(mode)
end

local function setOtherVehiclesAIMode(params)
  local dataTypeCheck, dataTypeError = checkTableDataTypes(params, { "string" })
  if not dataTypeCheck then
    return { failReason = dataTypeError }
  end
  local mode = params[1]

  if mode == "stop" then
    BeamEngine:queueAllObjectLua('ai.setMode("stop")')
    obj:queueGameEngineLua("extensions.gameplay_traffic.deactivate()")
    obj:queueGameEngineLua('extensions.hook("trackAIAllVeh", "disabled")')
    BeamEngine:queueAllObjectLuaExcept('ai.setRacing(false)', objectId)

  elseif mode == "random" then
    BeamEngine:queueAllObjectLuaExcept('ai.setSpeedMode("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.driveInLane("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setState({mode = "random", extAggression = 1, targetObjectID = ' .. tostring(objectId) .. "})", objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setRacing(false)', objectId)
    obj:queueGameEngineLua('extensions.hook("trackAIAllVeh", "random")')

  elseif mode == "flee" then
    BeamEngine:queueAllObjectLuaExcept('ai.setSpeedMode("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.driveInLane("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setState({mode = "flee", targetObjectID = ' .. tostring(objectId) .. "})", objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setRacing(false)', objectId)
    obj:queueGameEngineLua('extensions.hook("trackAIAllVeh", "flee")')

  elseif mode == "chase" then
    BeamEngine:queueAllObjectLuaExcept('ai.setSpeedMode("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.driveInLane("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setState({mode = "chase", targetObjectID = ' .. tostring(objectId) .. "})", objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setRacing(false)', objectId)
    obj:queueGameEngineLua('extensions.hook("trackAIAllVeh", "chase")')

  elseif mode == "follow" then
    BeamEngine:queueAllObjectLuaExcept('ai.setSpeedMode("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.driveInLane("off")', objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setState({mode = "follow", targetObjectID = ' .. tostring(objectId) .. "})", objectId)
    BeamEngine:queueAllObjectLuaExcept('ai.setRacing(false)', objectId)
    obj:queueGameEngineLua('extensions.hook("trackAIAllVeh", "follow")')

  else
    log("W", "interactAI", "unknown ai command: " .. mode)
  end
end

local function requestRegistration(gi)
  gi.registerModule(moduleName, M.moduleActions, M.moduleLookups)
end

local function onExtensionLoaded()
  M.moduleActions.setAIMode = setAIMode
  M.moduleActions.setOtherVehiclesAIMode = setOtherVehiclesAIMode
end

M.onExtensionLoaded = onExtensionLoaded
M.requestRegistration = requestRegistration

return M
