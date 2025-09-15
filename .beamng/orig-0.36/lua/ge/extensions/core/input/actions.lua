-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

require("utils")

local actionsCache = {} -- both vehicle-specific and normal actions, [true] for active ones, [false] for inactive ones
local normalActionsCache = {}                     -- normal actions, [true] for active ones, [false] for inactive ones

local M = {}
M.dependencies = { "core_input_categories", "core_input_deprecatedActions", "tech_license" }
M.menuIndependentPrefix = "MenuIndependent_"
M.menuActionMapNames = {}

-- mangle the action name, needed to prevent collisions with other vehicles' action names
local function nameToUniqueName(actionName, vehicleName)
  return vehicleName and (vehicleName.."__"..actionName) or actionName
end

local function uniqueNameToName(uniqueActionName, vehicleName)
  local actionName = uniqueActionName
  if vehicleName then
    local prefix = vehicleName.."__"
    if string.startswith(actionName, prefix) then
      actionName = string.sub(actionName, 1+string.len(prefix))
    else
      log("E", "", "Unable to convert uniqueName to name: "..dumps(uniqueActionName))
    end
  end
  return actionName
end

-- convert actions to unique action names, and set appropriate default values
-- e.g. convert a pickup action from "myAction" to "pickup__myAction", then set category, context, etc
local function actionsToUniqueActions(actions, vehicleName)
  local result = {}
  for actionName,v in pairs(actions) do
    if vehicleName and v.namespace ~= "common" then
      v.vehicle = vehicleName
      v.cat = v.cat or "vehicle_specific"
      v.ctx = v.ctx or "vlua"
    end
    result[nameToUniqueName(actionName, vehicleName)] = v
  end
  return result
end

-- read the actions for the specified vehicle, or normal actions otherwise
-- if active is true, only active actions are returned
-- if active is false, only inactive actions are returned (due to e.g. beamng.tech license)
local function readFromActionsFile(vehicleName, active)
  local directory = vehicleName and ("vehicles/"..vehicleName) or "lua/ge/extensions/core/input/actions/"
  local pattern = vehicleName and "input_actions*.json" or "*.json"
  local result = {}
  for _,path in pairs(FS:findFiles(directory, pattern, 0, false, false)) do
    if active == tech_license.isAllowedActionsPath(path) then
      local vehicleActions = jsonReadFile(path)
      if vehicleActions == nil then
        log("E", "input_actions", 'unable to read json file: ' .. tostring(path))
      end
      for k,action in pairs(vehicleActions or {}) do
        action.source = path
      end
      tableMerge(result, actionsToUniqueActions(vehicleActions or {}, vehicleName))
    end
  end
  return result
end

-- actions that are not vehicle-specific
local function getNormalActions(active)
  normalActionsCache[active] = normalActionsCache[active] or readFromActionsFile(nil, active)
  return normalActionsCache[active]
end

local function readActionsFromDisk(active)
  local result = {}
  -- read all vehicle-specific actions
  for vid, vehicle in vehiclesIterator() do
    local vehicleName = vehicle:getJBeamFilename()

    -- add actions from vehicle action files
    for uniqueActionName,action in pairs(readFromActionsFile(vehicleName, active)) do
      if result[uniqueActionName] and result[uniqueActionName].source ~= action.source then log("E", "", "Detected duplication of action: "..dumps(uniqueActionName)..", vid: "..dumps(vid)..", source A: "..dumps(result[uniqueActionName].source)..", source B: "..dumps(action.source)) end
      result[uniqueActionName] = action
    end

    -- add actions from vehicle jbeam files (interaction support)
    local vd = extensions.core_vehicle_manager.getVehicleData(vehicle:getID())
    for uniqueActionName,action in pairs(actionsToUniqueActions(vd and vd.vdata and vd.vdata.inputActions or {}, vehicleName)) do
      if result[uniqueActionName] and result[uniqueActionName].source ~= action.source then log("E", "", "Detected duplication of action: "..dumps(uniqueActionName)..", vid: "..dumps(vid)..", source A: "..dumps(result[uniqueActionName].source)..", source B: "..dumps(action.source)) end
      result[uniqueActionName] = action
    end
  end

  local playerVehicleId = be:getPlayerVehicleID(0)
  if playerVehicleId then
    local playerVehicle = getObjectByID(playerVehicleId)
    if playerVehicle then
      local playerVehicleName = playerVehicle:getJBeamFilename()
      local vd = extensions.core_vehicle_manager.getVehicleData(playerVehicleId)
      if vd.vdata.actionsEnabled then
        for _, action in pairs(vd.vdata.actionsEnabled or {}) do
          for resultActionName, resultAction in pairs(result) do
            if resultAction.vehicle and resultActionName == playerVehicleName .. "__" .. action.name then
              resultAction.usable = true
            end
          end
        end
      else
        -- If the vehicle has no actionsEnabled table, mark all vehicle-specific actions as usable
        for resultActionName, resultAction in pairs(result) do
          resultAction.usable = true
        end
      end
    end
  end

  for resultActionName, resultAction in pairs(result) do
    if resultAction.usable then
      resultAction.cat = "vehicle_specific"
    else
      resultAction.cat = "vehicle_specific_unusable"
    end
    resultAction.usable = nil
  end

  -- add actions from normal action files
  for uniqueActionName,action in pairs(getNormalActions(active)) do
    if result[uniqueActionName] and result[uniqueActionName].source ~= action.source then log("E", "", "Detected duplication of action: "..dumps(uniqueActionName)..", vid: "..dumps(vid)..", source A: "..dumps(result[uniqueActionName].source)..", source B: "..dumps(action.source)) end
    result[uniqueActionName] = action
  end

  -- build the list of active actionmaps that are related to the menu actionmap system
  if active then
    table.clear(M.menuActionMapNames)
    for actionName, action in pairs(result) do
      local success, actionMap = M.actionToCommands(actionName, action)
      if not success then goto continue end
      actionMap = actionMap.."ActionMap"

      -- catalog menu vs non-menu actionmaps
      local isMenuAction = string.startswith(actionMap, M.menuIndependentPrefix)
      if isMenuAction then M.menuActionMapNames[actionMap] = true end

      -- create any new/missing actionmaps in c++ side
      local am = scenetree.findObject(actionMap)
      if not am then
        am = ActionMap(actionMap)
        if isMenuAction then am:push() end
        --log('D', '', "Registered new action map: "..actionMap)
      end
      ::continue::
    end
  end
  return result
end

-- actions that are either active, or inactive, at the current time (for example, including BeamNG.tech actions if the tech license is valid)
local function getActions(active)
  actionsCache[active] = actionsCache[active] or readActionsFromDisk(active)
  return actionsCache[active]
end

-- check if an action has been deprecated or replaced, and return the new version when possible
local function upgradeAction(action)
  if action == nil then
    log('E', 'bindings', "Cannot parse null action")
    return
  end
  if getActions(true)[action] == nil then
    if getActions(false)[action] ~= nil then
      -- ignoring action, as it's currently inactive
      return
    end

    if core_input_deprecatedActions[action] == nil then
      log('E', 'bindings', "Couldn't find action "..tostring(action).." in actions lookup table")
      return
    end

    if core_input_deprecatedActions[action]["replacement"] ~= nil then
      log('D', 'bindings', "Replacing deprecated action "..action.." with new action "..core_input_deprecatedActions[action]["replacement"]);
      return upgradeAction(core_input_deprecatedActions[action]["replacement"])
    end
    if core_input_deprecatedActions[action]["obsolete"] == true then
      log('D', 'bindings', "Ignoring deprecated action: "..action)
      return
    end
    log('E', 'bindings', "Couldn't process deprecated action "..action..": "..dumps(core_input_deprecatedActions[action]))
    return
  end
  return action
end

-- TLDR: actionCache only needs to be used when the caller is inside getActions
-- NTLDR: actionCache allows to avoid an infinite recursion loop when the caller is getActions (while the cache is being generated, before it's been made public, but we have this particular action's cache internally computed already)
local function actionToCommands(actionName, actionCache)
  -- retrieve the code/parameters that will be used by GameEngine when a binding triggers this action
  local actionMap       = "Normal"
  local actsOnChange    = false
  local     onChange    = ""
  local actsOnDown      = false
  local     onDown      = ""
  local actsOnUp        = false
  local     onUp        = ""
  local isRelative      = false
  local ctx             = ActionMapCommandContext()
  ctx.type = COMMAND_CONTEXT_TLUA
  local isCentered      = false

  local ctxStr = 'tlua'

  local c = actionCache or getActions(true)[actionName]
  if c["cat"]      =='menu' then actionMap = "Menu" end
  if c["ctx"]      =='vlua' then actionMap = "VehicleCommon" end
  if c["actionMap"]  ~= nil then actionMap = c["actionMap"]; end
  if actionMap  == "MenuIndependent" then actionMap = M.menuIndependentPrefix..actionName end
  if c["vehicle"]    ~= nil then actionMap = "VehicleSpecific" end
  if c["onChange"]   ~= nil then onChange = c["onChange"]; actsOnChange = true; end
  if c["onRelative"] ~= nil then onChange = c["onRelative"]; actsOnChange = true; isRelative = true; end
  if c["onDown"]     ~= nil then onDown = c["onDown"]; actsOnDown = true; end
  if c["onUp"]       ~= nil then onUp = c["onUp"]; actsOnUp = true; end
  if c["isCentered"] ~= nil then isCentered= c["isCentered"]; end
  if c["ctx"]        ~= nil then ctxStr = c["ctx"]; end

  if core_input_categories[c.cat] == nil then
    log('W', 'bindings', 'Invalid category '..dumps(c.cat)..' defined for '..dumps(actionName).."' action. Suggested fix: modify the 'cat' action field to a valid category, or add the category to categories.lua")
  end
  if ctxStr == 'slua' then
    ctxStr = 'elua'
    log('E', 'bindings', 'Replacing deprecated "slua" context with "elua", for action: '..dumps(actionName))
  end
  if     ctxStr == 'ts'    then
    log("E", "", "The 'ts' context is deprecated. please use tlua. Action: "..dumps(actionName))
    ctx.type = COMMAND_CONTEXT_TS
  elseif ctxStr == 'ui'    then
    log("E", "", "The ui context is deprecated. please use tlua and guihooks. Action: "..dumps(actionName))
  elseif ctxStr == 'vlua'  then ctx.type = COMMAND_CONTEXT_VLUA
  elseif ctxStr == 'elua'  then ctx.type = COMMAND_CONTEXT_ELUA
  elseif ctxStr == 'tlua'  then ctx.type = COMMAND_CONTEXT_TLUA
  elseif ctxStr == 'bvlua' then ctx.type = COMMAND_CONTEXT_BVLUA
  end

  if actsOnUp and not actsOnDown then
    log("W", "", "Action "..dumps(actionName).." uses 'onUp' instead of 'onDown', which adds input lag. If you're the author of this action, please double check and see if you can switch to 'onDown' instead.")
  end

  return true, actionMap, actsOnChange, onChange, actsOnDown, onDown, actsOnUp, onUp, isRelative, ctx, isCentered
end

local function onFirstUpdate()
  table.clear(actionsCache)
end

local function onVehicleSwitched(oldId, newId, player)
  local oldVehicle = getObjectByID(oldId)
  local newVehicle = getObjectByID(newId)
  local oldName = oldVehicle and oldVehicle:getJBeamFilename() or "<none>"
  local newName = newVehicle and newVehicle:getJBeamFilename() or "<none>"
  table.clear(actionsCache)
end

local function onFileChanged(filename)
  local actionsModified = string.startswith(filename, "/lua/ge/extensions/core/input/actions") and string.endswith(filename, ".json")
  if actionsModified then
    table.clear(normalActionsCache)
    table.clear(actionsCache)
  end
end

local function triggerDown(actionName)
  ActionMap.triggerBindingByNameDigital(actionName, true, os.clockhp())
end
local function triggerUp(actionName)
  ActionMap.triggerBindingByNameDigital(actionName, false, os.clockhp())
end
local function triggerDownUp(actionName)
  triggerDown(actionName)
  triggerUp(actionName)
end


M.onFirstUpdate = onFirstUpdate
M.onVehicleSwitched = onVehicleSwitched
M.onFileChanged = onFileChanged

M.getActiveActions = function() return getActions(true) end
M.upgradeAction = upgradeAction
M.actionToCommands = actionToCommands
M.uniqueNameToName = uniqueNameToName
M.nameToUniqueName = nameToUniqueName

-- API typically used by command handler (such as 'beamng:' URL protocol)
M.triggerDown   = triggerDown
M.triggerUp     = triggerUp
M.triggerDownUp = triggerDownUp

return M
