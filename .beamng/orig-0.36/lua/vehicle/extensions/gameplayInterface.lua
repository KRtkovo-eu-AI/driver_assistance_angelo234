-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local registeredActions = {}
local registeredLookups = {}

local callbacks = {}
local registeredValueChangeNotifications = {}

local checkValueChangeNotificationMethod = nop

local function debugCallback(requestId, data)
  print("gameplayInterface debug callback data:")
  dump(data)
end

local function vLuaCallback(callback, requestId, data)
  callback(requestId, data)
end

local function geLuaCallback(callbackString, requestId, data)
  local cmdString = string.format("%s(%d, %d, %q)", callbackString, objectId, requestId, serialize(data))
  obj:queueGameEngineLua(cmdString)
end

local function triggerCallback(callbackId, requestId, data)
  --get callback data for the relevant Id
  local callbackData = callbacks[callbackId]
  if not callbackData then
    print(string.format("Can't find requested callback id %d", callbackId))
    dump(callbacks)
    return
  end

  --check which VM the callback belongs to
  if callbackData.vm == "gelua" then
    geLuaCallback(callbackData.callback, requestId, data)
  elseif callbackData.vm == "vlua" then
    vLuaCallback(callbackData.callback, requestId, data)
  end
end

local function checkValueChangeNotifications(dt)
  for _, v in ipairs(registeredValueChangeNotifications) do
    local currentValue = electrics.values[v.electricsKey]
    if currentValue and currentValue ~= v.lastValue then
      v.lastValue = currentValue
      triggerCallback(v.callbackId, -2, {[v.electricsKey] = currentValue})
    end
  end
end

local function updateGFX(dt)
  checkValueChangeNotificationMethod(dt)
end

local function ensureExtensionModuleLoaded(extensionName)
  if not extensions["gameplayInterfaceModules_" .. extensionName] then
    extensions.load("gameplayInterfaceModules/" .. extensionName)
  end
end

local function getSystemData(callbackId, id, system, ...)
  local result
  local params = {...}

  if id and system then
    local systemMethod
    local extensionModule = registeredLookups[system]
    if extensionModule then
      ensureExtensionModuleLoaded(extensionModule)
      systemMethod = extensions["gameplayInterfaceModules_" .. extensionModule].moduleLookups[system]
    else
      result = {failReason = string.format("Can't find module for system %q with request id %d", system, id)}
      print(result.failReason)
    end

    if systemMethod then
      result = systemMethod(params)
    else
      result = {failReason = string.format("Received unknown system %q with request id %d", system, id)}
      print(result.failReason)
    end
  else
    result = {failReason = string.format("Received invalid request data, request id: %d, system: %q", id, system)}
    print(result.failReason)
  end

  if result ~= nil then
    --tell gelua about this result
    triggerCallback(callbackId, id, result)
  end
end

local function executeAction(callbackId, id, action, ...)
  local result
  local params = {...}
  --print(string.format("Doing thing %q with data %q", action, dumps(params)))
  if id and action then
    local actionMethod
    local extensionModule = registeredActions[action]
    if extensionModule then
      ensureExtensionModuleLoaded(extensionModule)
      actionMethod = extensions["gameplayInterfaceModules_" .. extensionModule].moduleActions[action]
    else
      result = {failReason = string.format("Can't find module for action %q with id %d", action, id)}
      print(result.failReason)
    end
    if actionMethod then
      result = actionMethod(params)
    else
      result = {failReason = string.format("Received unknown action %q with id %d", action, id)}
      print(result.failReason)
    end
  else
    result = {failReason = string.format("Received invalid request data, id: %d, action: %q", id, action)}
    print(result.failReason)
  end

  if result ~= nil then
    --tell gelua about this result
    triggerCallback(callbackId, id, result)
  end
end

local function registerValueChangeNotification(callbackId, id, electricsKey)
  table.insert(registeredValueChangeNotifications, {callbackId = callbackId, electricsKey = electricsKey, lastValue = nil})
  triggerCallback(callbackId, id, {registerResult = true})

  --we just added a notif, so we need to call the gfx update as well
  checkValueChangeNotificationMethod = checkValueChangeNotifications
end

local function unregisterValueChangeNotification(callbackId, electricsKey)
  local indexToRemove
  for k, v in ipairs(registeredValueChangeNotifications) do
    if v.callbackId == callbackId and v.electricsKey == electricsKey then
      indexToRemove = k
      break
    end
  end

  if indexToRemove then
    table.remove(registeredValueChangeNotifications, indexToRemove)
    --we just removed an element from the notif table, check if we still need to call the GFX update
    if tableIsEmpty(registeredValueChangeNotifications) then
      checkValueChangeNotificationMethod = nop
    end
  end
end

local function triggerValueChangeNotification(valueChangeNotificationId, data)
  local callbackId = registeredValueChangeNotifications[valueChangeNotificationId].callbackId
  local valueChangeData = {data = data, valueChangeNotificationId = valueChangeNotificationId}
  triggerCallback(callbackId, -2, valueChangeData)
end

local function registerCallback(vm, callback)
  table.insert(callbacks, {vm = vm, callback = callback})
  local currentCallbackId = #callbacks
  triggerCallback(currentCallbackId, -1, {callbackId = currentCallbackId})
end

local function registerModule(name, actions, lookups)
  for k, _ in pairs(actions) do
    if registeredActions[k] then
      print(string.format("duplicate action: %q from %q and %q", k, registeredActions[k], name))
    end
    registeredActions[k] = name
  end
  for k, _ in pairs(lookups) do
    if registeredLookups[k] then
      print(string.format("duplicate lookup: %q from %q and %q", k, registeredLookups[k], name))
    end
    registeredLookups[k] = name
  end
end

local function onReset()
  for _, v in ipairs(registeredValueChangeNotifications) do
    v.lastValue = nil
  end
end

local function onInit()
end

local function onExtensionLoaded()
  --iterate over all files within subdir: gameplayInterfaceModule
  --load each of them, wait for registerModule call, then unload them
  local moduleDir = "lua/vehicle/extensions/gameplayInterfaceModules"
  local moduleFiles = FS:findFiles(moduleDir, "*.lua", -1, true, false)
  if moduleFiles then
    for _, filePath in ipairs(moduleFiles) do
      local _, file, _ = path.split(filePath)
      local fileName = file:sub(1, -5)
      local extensionPath = "gameplayInterfaceModules/" .. fileName
      extensions.load(extensionPath)
      local extensionName = "gameplayInterfaceModules_" .. fileName
      extensions[extensionName].requestRegistration(M)
      extensions.unload(extensionName)
    end
  end

  callbacks = {
    [0] = {vm = "gelua", callback = "extensions.core_vehicleBridge.callbackFromVlua"},
    [1] = {vm = "vlua", callback = M.debugCallback}
  }
end

-- public interface
M.onInit = onInit
M.onReset = onReset
M.updateGFX = updateGFX
M.onExtensionLoaded = onExtensionLoaded

M.registerModule = registerModule

M.registerCallback = registerCallback
M.getSystemData = getSystemData
M.executeAction = executeAction

M.registerValueChangeNotification = registerValueChangeNotification
M.unregisterValueChangeNotification = unregisterValueChangeNotification
M.triggerValueChangeNotification = triggerValueChangeNotification

M.debugCallback = debugCallback

return M
