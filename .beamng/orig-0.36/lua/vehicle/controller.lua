-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.mainController = nil
M.isFrozen = false

M.nilController = nil

local blacklist = {"shiftLogic-automaticGearbox", "shiftLogic-cvtGearbox", "shiftLogic-dctGearbox", "shiftLogic-manualGearbox", "shiftLogic-sequentialGearbox"}
local blacklistLookup = nil

local loadedControllers = {}
local sortedControllers = {}
local physicsUpdates = {}
local physicsUpdateCount = 0
local wheelsIntermediateUpdates = {}
local wheelsIntermediateUpdateCount = 0
local gfxUpdates = {}
local gfxUpdateCount = 0
local fixedStepUpdates = {}
local fixedStepUpdateCount = 0
local debugDraws = {}
local debugDrawCount = 0
local beamBrokens = {}
local beamBrokenCount = 0
local beamDeformeds = {}
local beamDeformedCount = 0
local nodeCollisions = {}
local nodeCollisionCount = 0
local couplerFoundEvents = {}
local couplerFoundEventCount = 0
local couplerAttachedEvents = {}
local couplerAttachedEventCount = 0
local couplerDetachedEvents = {}
local couplerDetachedEventCount = 0
local gameplayEvents = {}
local gameplayEventCount = 0
local controllerJbeamData = {}

local fixedStepTimer = 0
local fixedStepTime = 1 / 100

local controllerNameLookup = {
  updateGFXStep = {},
  updateFixedStep = {},
  updatePhysicsStep = {},
  updateWheelsIntermediate = {},
  beamBroke = {},
  beamDeformed = {},
  nodeCollision = {},
  onCouplerFound = {},
  onCouplerAttached = {},
  onCouplerDetached = {},
  onGameplayEvent = {},
  debugDraw = {}
}

local relocatedControllers = {}

local function registerRelocatedController(originalPath, newPath)
  relocatedControllers[originalPath] = newPath
end

--These debug methods are here to be used in conjunction with controller.printDebugMethodCalls() to generate a hardcoded list of controller updates.
--Check the comments in updateFunctionCounts() for more info

-- local function updateFixedStepDebug(dt)
-- end

-- local function updateDebug(dt)
--   fixedStepTimer = fixedStepTimer + dt
--   if fixedStepTimer >= fixedStepTime then
--     updateFixedStepDebug(fixedStepTimer)
--     fixedStepTimer = fixedStepTimer - fixedStepTime
--   end
-- end

-- local function updateGFXDebug(dt)
-- end

-- local function updateWheelsIntermediateDebug(dt)
-- end

-- local function beamBrokeDebug(id, energy)
-- end

-- local function beamDeformedDebug(id, ratio)
-- end

-- local function nodeCollisionDebug(p)
-- end

-- local function onCouplerFoundDebug(nodeId, obj2id, obj2nodeId)
-- end

-- local function onCouplerAttachedDebug(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
-- end

-- local function onCouplerDetachedDebug(nodeId, obj2id, obj2nodeId, breakForce)
-- end

-- local function onGameplayEventDebug(eventName, ...)
-- end

-- local function debugDrawDebug(focusPos)
-- end

-- local function settingsChangedDebug()
-- end

local function updateGFX(dt)
  for i = 1, gfxUpdateCount, 1 do
    --profilerPushEvent(controllerNameLookup.updateGFXStep[i] .. ":updateGFXStep")
    gfxUpdates[i](dt)
    --profilerPopEvent(controllerNameLookup.updateGFXStep[i] .. ":updateGFXStep")
  end
end

local function updateFixedStep(dt)
  --profilerPushEvent("controller:updateFixedStep")
  for i = 1, fixedStepUpdateCount, 1 do
    --profilerPushEvent(controllerNameLookup.updateFixedStep[i] .. ":updateFixedStep")
    fixedStepUpdates[i](dt)
    --profilerPopEvent(controllerNameLookup.updateFixedStep[i] .. ":updateFixedStep")
  end
  --profilerPopEvent("controller:updateFixedStep")
end

local function updateWithFixedStep(dt)
  --profilerPushEvent("controller:updatePhysicsStep")
  for i = 1, physicsUpdateCount, 1 do
    --profilerPushEvent(controllerNameLookup.updatePhysicsStep[i] .. ":updatePhysicsStep")
    physicsUpdates[i](dt)
    --profilerPopEvent(controllerNameLookup.updatePhysicsStep[i] .. ":updatePhysicsStep")
  end

  --if below code needs to change, make sure to copy the changes to the hardcoded export version at the end of this file as well
  fixedStepTimer = fixedStepTimer + dt
  if fixedStepTimer >= fixedStepTime then
    updateFixedStep(fixedStepTimer)
    fixedStepTimer = fixedStepTimer - fixedStepTime
  end
  --profilerPopEvent("controller:updatePhysicsStep")
end

local function updateWithoutFixedStep(dt)
  for i = 1, physicsUpdateCount, 1 do
    --profilerPushEvent(controllerNameLookup.updatePhysicsStep[i] .. ":updatePhysicsStep")
    physicsUpdates[i](dt)
    --profilerPopEvent(controllerNameLookup.updatePhysicsStep[i] .. ":updatePhysicsStep")
  end
end

local function updateWheelsIntermediate(dt)
  for i = 1, wheelsIntermediateUpdateCount, 1 do
    --profilerPushEvent(controllerNameLookup.updateWheelsIntermediate[i] .. ":updateWheelsIntermediate")
    wheelsIntermediateUpdates[i](dt)
    --profilerPopEvent(controllerNameLookup.updateWheelsIntermediate[i] .. ":updateWheelsIntermediate")
  end
end

local function beamBroke(id, energy)
  for i = 1, beamBrokenCount, 1 do
    beamBrokens[i](id, energy)
  end
end

local function beamDeformed(id, ratio)
  for i = 1, beamDeformedCount, 1 do
    beamDeformeds[i](id, ratio)
  end
end

local function nodeCollision(p)
  for i = 1, nodeCollisionCount, 1 do
    nodeCollisions[i](p)
  end
end

local function onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  for i = 1, couplerFoundEventCount, 1 do
    couplerFoundEvents[i](nodeId, obj2id, obj2nodeId, nodeDist)
  end
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  for i = 1, couplerAttachedEventCount, 1 do
    couplerAttachedEvents[i](nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  end
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  for i = 1, couplerDetachedEventCount, 1 do
    couplerDetachedEvents[i](nodeId, obj2id, obj2nodeId, breakForce)
  end
end

local function onGameplayEvent(eventName, ...)
  for i = 1, gameplayEventCount, 1 do
    gameplayEvents[i](eventName, ...)
  end
end

local function debugDraw(focusPos)
  for i = 1, debugDrawCount, 1 do
    debugDraws[i](focusPos)
  end
end

local function settingsChanged()
  for _, v in pairs(loadedControllers) do
    if v.settingsChanged then
      v.settingsChanged()
    end
  end
end

local function getAllControllers(name)
  return loadedControllers
end

local function getController(name)
  return loadedControllers[name]
end

local function getControllerSafe(name)
  local controller = loadedControllers[name]
  if controller then
    return controller
  else
    log("D", "controller.getControllerSafe", string.format("Didn't find controller '%s', returning nilController.", name))
    --log("D", "controller.getControllerSafe", debug.traceback())
    --return our nilController that accepts all indexes and can be called without errors
    return M.nilController
  end
end

local function getControllersByType(typeName)
  local controllers = {}
  for _, v in pairs(loadedControllers) do
    if v.typeName == typeName then
      table.insert(controllers, v)
    end
  end
  return controllers
end

local function getControllersFromPath(path)
  local controllers = {}
  for _, v in pairs(loadedControllers) do
    if v.typeName:sub(1, #path) == path then
      table.insert(controllers, v)
    end
  end
  return controllers
end

local function setFreeze(mode)
  M.isFrozen = mode == 1
  if M.mainController then
    M.mainController.setFreeze(mode)
  end
end

local function updateSimpleControlButtons()
  for _, v in pairs(loadedControllers) do
    if v.updateSimpleControlButtons then
      v.updateSimpleControlButtons()
    end
  end
end

local function loadControllerExternal(fileName, controllerName, controllerData)
  local directory = "controller/"

  local filePath = directory .. fileName
  --adjust for relocated controllers by using the new path if one exists
  if relocatedControllers[filePath] then
    log("D", "controller.loadControllerExternal", string.format("Using relocated controller controller '%s' at '/%s.lua', original file path: '/%s.lua'", fileName, relocatedControllers[filePath], filePath))
    filePath = relocatedControllers[filePath]
  end
  controllerName = controllerName or fileName
  local c
  local loadFunc = function()
    if loadedControllers[controllerName] then
      error(string.format("Controller with same name is already existing, can't load duplicate controller. Name: %q", controllerName))
    end
    c = rerequire(filePath)
    if c then
      local data = controllerData or {}
      controllerJbeamData[controllerName] = data
      c.name = controllerName
      c.typeName = fileName
      c.init(data)
      c.manualOrder = data.manualOrder
      loadedControllers[controllerName] = c

      if c.type == "main" then
        error(string.format("Can't load mainController at runtime! FileName: %q, ControllerName: %q", fileName, c.name))
        return nil
      end
    end
  end
  local result, errorStr = pcall(loadFunc)
  if not result then
    log("E", "controller.loadControllerExternal", string.format("Can't load controller '%s' at '/%s.lua', further info below:", fileName, filePath))
    log("E", "controller.loadControllerExternal", errorStr)
    log("E", "controller.loadControllerExternal", debug.traceback())
  end

  if c.initSecondStage then
    c.initSecondStage(controllerJbeamData[controller.name])
  end
  if c.initSounds then
    c.initSounds(controllerJbeamData[controller.name])
  end
  if c.initLasttage then
    c.initLasttage(controllerJbeamData[controller.name])
  end

  table.clear(sortedControllers)
  for _, v in pairs(loadedControllers) do
    table.insert(sortedControllers, v)
  end

  local ranks = {}
  for k, v in ipairs(powertrain.getOrderedDevices()) do
    ranks[v.name] = k * 100
  end
  table.sort(
    sortedControllers,
    function(a, b)
      local ra, rb = ranks[a.relevantDevice or ""] or a.manualOrder or a.defaultOrder or 100000, ranks[b.relevantDevice or ""] or b.manualOrder or b.defaultOrder or 100000
      a.order = ra
      b.order = rb
      if ra == rb then
        return a.name < b.name
      else
        return ra < rb
      end
    end
  )

  M.cacheAllControllerFunctions()

  return c
end

local function unloadControllerExternal(controllerName)
  if not loadedControllers[controllerName] then
    log("E", "controller.unloadControllerExternal", string.format("Can't unload controller with name '%s', no matching controller found.", controllerName))
    return
  end

  loadedControllers[controllerName] = nil

  table.clear(sortedControllers)
  for _, v in pairs(loadedControllers) do
    table.insert(sortedControllers, v)
  end

  local ranks = {}
  for k, v in ipairs(powertrain.getOrderedDevices()) do
    ranks[v.name] = k * 100
  end
  table.sort(
    sortedControllers,
    function(a, b)
      local ra, rb = ranks[a.relevantDevice or ""] or a.manualOrder or a.defaultOrder or 100000, ranks[b.relevantDevice or ""] or b.manualOrder or b.defaultOrder or 100000
      a.order = ra
      b.order = rb
      if ra == rb then
        return a.name < b.name
      else
        return ra < rb
      end
    end
  )

  M.cacheAllControllerFunctions()
end

local function adjustControllersPreInit(controllers)
  -- local escBehavior = settings.getValue("escBehavior") or "realistic"
  -- if escBehavior ~= "realistic" then
  --   if escBehavior == "arcade" and controllers.esc == nil then --only add arcade esc if we don't have a factory esc
  --     --we want arcade esc so we add that controller
  --     controllers.escArcade = {fileName = "escArcade"}
  --   end
  -- end
  return controllers
end

local function registerRelocatedControllers()
  registerRelocatedController("vehicleController", "vehicleController/vehicleController")
  registerRelocatedController("AVAS", "sound/AVAS")
  registerRelocatedController("airbrakes", "sound/airbrakes")
  registerRelocatedController("reverseWarn", "sound/reverseWarn")
end

local function init()
  loadedControllers = {}
  sortedControllers = {}
  controllerJbeamData = {}

  M.mainController = nil

  --Here we create a bit of special magic to deal with fire and forget controller access
  --for example: controller.getController("abc").doSomething()
  --in this case an error is thrown if "abc" is not a valid controller.
  --Using controller.getControllerSafe() instead returns a magic table that
  --happily accepts all indexes and can be called without throwing errors (if no real controller is found)
  M.nilController = {}
  local mt = {
    __index = function(t, _)
      return t
    end, --return self when indexing
    __call = function(t, ...)
      return t
    end, --return self when being called
    __newindex = function(_, _, _)
    end, --prevent any write access
    __metatable = false --hide metatable to prevent any changes to it
  }
  setmetatable(M.nilController, mt)

  registerRelocatedControllers()

  local jbeamControllers = v.data.controller
  if not jbeamControllers then
    jbeamControllers = {{fileName = "dummy"}}
    log("D", "controller.init", "No controllers found, adding a dummy controller!")
  end

  blacklistLookup = {}
  for _, v in pairs(blacklist) do
    blacklistLookup[v] = true
  end

  local controllers = {}
  for _, v in pairs(jbeamControllers) do
    if v.fileName and not blacklistLookup[v.fileName] then
      local name = v.name or v.fileName
      if controllers[name] then
        log("E", "controller.init", string.format("Found duplicate controller of name %q, please make sure there are no name overlaps.", name))
        log("E", "controller.init", "By default controller names are the type, specify unique names if you use multiple controllers of the same type.")
      end
      if relocatedControllers[v.fileName] then
        log("D", "controller.init", string.format("Using relocated controller controller '%s' at '/%s.lua', original file path: '/%s.lua'", name, relocatedControllers[v.fileName], v.fileName))
        v.fileName = relocatedControllers[v.fileName]
      end
      controllers[name] = v
    end
  end

  controllers = adjustControllersPreInit(controllers)

  local directory = "controller/"
  for k, c in pairs(controllers) do
    local filePath = directory .. c.fileName
    local loadFunc = function()
      local controller = rerequire(filePath)
      if controller then
        local data = tableMergeRecursive(c, v.data[k] or {})
        c.name = c.name or k
        controllerJbeamData[c.name] = data
        controller.name = c.name
        controller.typeName = c.fileName
        controller.init(data)
        controller.manualOrder = data.manualOrder
        loadedControllers[c.name] = controller

        if controller.type == "main" then
          if not M.mainController then
            M.mainController = controller
          else
            log("W", "controller.init", string.format("Found more than one main controller, 1: '%s', 2: '%s', unloading the first one...", M.mainController.name, controller.name))
            loadedControllers[M.mainController.name] = nil
            M.mainController = controller
          end
        end
      end
    end
    local result, errorStr = pcall(loadFunc)
    if not result then
      log("E", "controller.init", string.format("Can't load controller '%s' at '/%s.lua', further info below:", c.fileName, filePath))
      log("E", "controller.init", errorStr)
      log("E", "controller.init", debug.traceback())
    end
  end

  if not M.mainController then
    log("D", "controller.init", "No main controller found, adding a dummy controller!")
    local dummyName = "dummy"
    local controller = require(directory .. dummyName)
    if controller then
      loadedControllers[dummyName] = controller
      controller.init()
      controller.name = dummyName
      M.mainController = controller
    end
  end

  for _, v in pairs(loadedControllers) do
    table.insert(sortedControllers, v)
  end

  local ranks = {}
  for k, v in ipairs(powertrain.getOrderedDevices()) do
    ranks[v.name] = k * 100
  end
  table.sort(
    sortedControllers,
    function(a, b)
      local ra, rb = ranks[a.relevantDevice or ""] or a.manualOrder or a.defaultOrder or 100000, ranks[b.relevantDevice or ""] or b.manualOrder or b.defaultOrder or 100000
      a.order = ra
      b.order = rb
      if ra == rb then
        return a.name < b.name
      else
        return ra < rb
      end
    end
  )

  --  for k,v in pairs(sortedControllers) do
  --    print(string.format("%s -> %d", v.name, v.order))
  --  end

  --backwards compatiblity for old scenario.lua:freeze(), we don't know if any mod ever used this, just here as a precaution
  scenario = {
    freeze = function(mode)
      log("W", "controller", "scenario.freeze(mode) is deprecated. Please switch to controller.setFreeze(mode)")
      setFreeze(mode)
    end
  }
end

local function cacheControllerFunctions(controller)
  if controller.update then
    table.insert(physicsUpdates, controller.update)
    table.insert(controllerNameLookup.updatePhysicsStep, controller.typeName)
  end
  if controller.updateWheelsIntermediate then
    table.insert(wheelsIntermediateUpdates, controller.updateWheelsIntermediate)
    table.insert(controllerNameLookup.updateWheelsIntermediate, controller.typeName)
  end
  if controller.updateGFX then
    table.insert(gfxUpdates, controller.updateGFX)
    table.insert(controllerNameLookup.updateGFXStep, controller.typeName)
  end
  if controller.updateFixedStep then
    table.insert(fixedStepUpdates, controller.updateFixedStep)
    table.insert(controllerNameLookup.updateFixedStep, controller.typeName)
  end
  if controller.debugDraw then
    table.insert(debugDraws, controller.debugDraw)
    table.insert(controllerNameLookup.debugDraw, controller.typeName)
  end
  if controller.beamBroken then
    table.insert(beamBrokens, controller.beamBroken)
    table.insert(controllerNameLookup.beamBroke, controller.typeName)
  end
  if controller.beamDeformed then
    table.insert(beamDeformeds, controller.beamDeformed)
    table.insert(controllerNameLookup.beamDeformed, controller.typeName)
  end
  if controller.nodeCollision then
    table.insert(nodeCollisions, controller.nodeCollision)
    table.insert(controllerNameLookup.nodeCollision, controller.typeName)
  end
  if controller.onCouplerFound then
    table.insert(couplerFoundEvents, controller.onCouplerFound)
    table.insert(controllerNameLookup.onCouplerFound, controller.typeName)
  end
  if controller.onCouplerAttached then
    table.insert(couplerAttachedEvents, controller.onCouplerAttached)
    table.insert(controllerNameLookup.onCouplerAttached, controller.typeName)
  end
  if controller.onCouplerDetached then
    table.insert(couplerDetachedEvents, controller.onCouplerDetached)
    table.insert(controllerNameLookup.onCouplerDetached, controller.typeName)
  end
  if controller.onGameplayEvent then
    table.insert(gameplayEvents, controller.onGameplayEvent)
    table.insert(controllerNameLookup.onGameplayEvent, controller.typeName)
  end
end

local function updateFunctionCounts()
  physicsUpdateCount = #physicsUpdates
  wheelsIntermediateUpdateCount = #wheelsIntermediateUpdates
  fixedStepUpdateCount = #fixedStepUpdates
  gfxUpdateCount = #gfxUpdates
  debugDrawCount = #debugDraws
  beamBrokenCount = #beamBrokens
  beamDeformedCount = #beamDeformeds
  nodeCollisionCount = #nodeCollisions
  couplerFoundEventCount = #couplerFoundEvents
  couplerAttachedEventCount = #couplerAttachedEvents
  couplerDetachedEventCount = #couplerDetachedEvents
  gameplayEventCount = #gameplayEvents

  local physicsUpdate = fixedStepUpdateCount > 0 and updateWithFixedStep or updateWithoutFixedStep

  M.update = (physicsUpdateCount > 0 or fixedStepUpdateCount > 0) and physicsUpdate or nop
  M.updateWheelsIntermediate = wheelsIntermediateUpdateCount > 0 and updateWheelsIntermediate or nop
  M.updateGFX = gfxUpdateCount > 0 and updateGFX or nop
  M.debugDraw = debugDrawCount > 0 and debugDraw or nop
  M.beamBroke = beamBrokenCount > 0 and beamBroke or nop
  M.beamDeformed = beamDeformedCount > 0 and beamDeformed or nop
  M.nodeCollision = nodeCollisionCount > 0 and nodeCollision or nop
  M.onCouplerFound = couplerFoundEventCount > 0 and onCouplerFound or nop
  M.onCouplerAttached = couplerAttachedEventCount > 0 and onCouplerAttached or nop
  M.onCouplerDetached = couplerDetachedEventCount > 0 and onCouplerDetached or nop
  M.onGameplayEvent = gameplayEventCount > 0 and onGameplayEvent or nop

  --check if a hardcoded debug version of various methods exists, if so, execute that instead of the normal loop based version
  --attention: these hardcoded methods are normally commented and need to be generated from a given vehicle via: "controller.printDebugMethodCalls()"
  --copy the contents of that print into the relevant methods at the top of this file. Make sure (!!!) to remove it again when done
  --if you switch vehicles you MUST (!!!) regenerate the methods, otherwise things break!
  if updateDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: updateDebug")
    M.update = updateDebug
  end
  if updateWheelsIntermediateDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: updateWheelsIntermediateDebug")
    M.updateWheelsIntermediate = updateWheelsIntermediateDebug
  end
  if updateGFXDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: updateGFXDebug")
    M.updateGFX = updateGFXDebug
  end
  if debugDrawDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: debugDrawDebug")
    M.debugDraw = debugDrawDebug
  end
  if beamBrokeDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: beamBrokeDebug")
    M.beamBroke = beamBrokeDebug
  end
  if beamDeformedDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: beamDeformedDebug")
    M.beamDeformed = beamDeformedDebug
  end
  if nodeCollisionDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: nodeCollisionDebug")
    M.nodeCollision = nodeCollisionDebug
  end
  if onCouplerFoundDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: onCouplerFoundDebug")
    M.onCouplerFound = onCouplerFoundDebug
  end
  if onCouplerAttachedDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: onCouplerAttachedDebug")
    M.onCouplerAttached = onCouplerAttachedDebug
  end
  if onCouplerDetachedDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: onCouplerDetachedDebug")
    M.onCouplerDetached = onCouplerDetachedDebug
  end
  if onGameplayEventDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: onGameplayEventDebug")
    M.onGameplayEvent = onGameplayEventDebug
  end
  if settingsChangedDebug then
    log("W", "controller.updateFunctionCounts", "ATTENTION !!! Using hardcoded update method: settingsChangedDebug")
    M.settingsChanged = settingsChangedDebug
  end
end

local function cacheAllControllerFunctions()
  physicsUpdates = {}
  wheelsIntermediateUpdates = {}
  gfxUpdates = {}
  fixedStepUpdates = {}
  beamBrokens = {}
  nodeCollisions = {}
  couplerAttachedEvents = {}
  couplerDetachedEvents = {}
  couplerFoundEvents = {}
  gameplayEvents = {}
  debugDraws = {}

  for name, _ in pairs(controllerNameLookup) do
    controllerNameLookup[name] = {}
  end

  for _, controller in ipairs(sortedControllers) do
    cacheControllerFunctions(controller)
  end

  updateFunctionCounts()
end

local function initSecondStage()
  for _, v in pairs(sortedControllers) do
    if v.initSecondStage then
      v.initSecondStage(controllerJbeamData[v.name])
    end
  end

  cacheAllControllerFunctions()
end

local function initLastStage()
  for _, v in pairs(sortedControllers) do
    if v.initLastStage then
      v.initLastStage(controllerJbeamData[v.name])
    end
  end

  cacheAllControllerFunctions()
end

local function initSounds()
  for _, v in pairs(sortedControllers) do
    if v.initSounds then
      v.initSounds(controllerJbeamData[v.name])
    end
  end

  cacheAllControllerFunctions()
end

local function reset()
  fixedStepTimer = 0

  for _, v in pairs(sortedControllers) do
    if not v.reset then
      v.init(controllerJbeamData[v.name])
    end
  end
end

local function resetSecondStage()
  for _, v in pairs(sortedControllers) do
    if v.reset then
      v.reset(controllerJbeamData[v.name])
    elseif v.initSecondStage then
      v.initSecondStage(controllerJbeamData[v.name])
    end
  end

  cacheAllControllerFunctions()
end

local function resetLastStage()
  for _, v in pairs(sortedControllers) do
    if v.resetLastStage then
      v.resetLastStage()
    elseif v.initLastStage then
      v.initLastStage(controllerJbeamData[v.name])
    end
  end

  cacheAllControllerFunctions()
end

local function resetSounds()
  for _, v in pairs(sortedControllers) do
    if v.resetSounds then
      v.resetSounds()
    end
  end

  cacheAllControllerFunctions()
end

local function printDebugMethodCalls(callType)
  local disclaimer = "\r\n\r\nCopy the contents of the method(s) below into their respective analogs in controller.lua and make sure the methods are not commented.\r\ncontroller.lua will automatically execute these hardcoded versions rather than using its internal loops\r\n \r\n "
  print(disclaimer)
  if callType == "update" or callType == nil then
    print("local function updateFixedStepDebug(dt)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.updateFixedStep ~= nil then
        print("  sortedControllers[" .. i .. "].updateFixedStep(dt) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")

    print("local function updateDebug(dt)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.update ~= nil then
        print("  sortedControllers[" .. i .. "].update(dt) -- " .. controller.typeName)
      end
    end

    print(" ")
    print("  fixedStepTimer = fixedStepTimer + dt")
    print("  if fixedStepTimer >= fixedStepTime then")
    print("    updateFixedStepDebug(fixedStepTimer)")
    print("    fixedStepTimer = fixedStepTimer - fixedStepTime")
    print("  end")
    print("end")
    print(" ")
  end
  if callType == "updateGFX" or callType == nil then
    print("local function updateGFXDebug(dt)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.updateGFX ~= nil then
        print("  sortedControllers[" .. i .. "].updateGFX(dt) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "updateWheelsIntermediate" or callType == nil then
    print("local function updateWheelsIntermediateDebug(dt)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.updateWheelsIntermediate ~= nil then
        print("  sortedControllers[" .. i .. "].updateWheelsIntermediate(dt) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "beamBroke" or callType == nil then
    print("local function beamBrokeDebug(id, energy)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.beamBroke ~= nil then
        print("  sortedControllers[" .. i .. "].beamBroke(id, energy) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "beamDeformed" or callType == nil then
    print("local function beamDeformedDebug(id, ratio)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.beamDeformed ~= nil then
        print("  sortedControllers[" .. i .. "].beamDeformed(id, ratio) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "nodeCollision" or callType == nil then
    print("local function nodeCollisionDebug(p)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.nodeCollision ~= nil then
        print("  sortedControllers[" .. i .. "].nodeCollision(p) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "onCouplerFound" or callType == nil then
    print("local function onCouplerFoundDebug(nodeId, obj2id, obj2nodeId)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.onCouplerFound ~= nil then
        print("  sortedControllers[" .. i .. "].onCouplerFound(nodeId, obj2id, obj2nodeId) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "onCouplerAttached" or callType == nil then
    print("local function onCouplerAttachedDebug(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.onCouplerAttached ~= nil then
        print("  sortedControllers[" .. i .. "].onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "onCouplerDetached" or callType == nil then
    print("local function onCouplerDetachedDebug(nodeId, obj2id, obj2nodeId, breakForce)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.onCouplerDetached ~= nil then
        print("  sortedControllers[" .. i .. "].onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "onGameplayEvent" or callType == nil then
    print("local function onGameplayEventDebug(eventName, ...)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.onGameplayEvent ~= nil then
        print("  sortedControllers[" .. i .. "].onGameplayEvent(eventName, ...) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "debugDraw" or callType == nil then
    print("local function debugDrawDebug(focusPos)")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.debugDraw ~= nil then
        print("  sortedControllers[" .. i .. "].debugDraw(focusPos) -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
  if callType == "settingsChanged" or callType == nil then
    print("local function settingsChangedDebug()")
    print("  --Controllers not in this list do not have a matching method to call")
    for i, controller in ipairs(sortedControllers) do
      if controller.settingsChanged ~= nil then
        print("  sortedControllers[" .. i .. "].settingsChanged() -- " .. tostring(controller.typeName))
      end
    end
    print("end")
    print(" ")
  end
end

local function onDeserialize(data)
  if not data or type(data) ~= "table" then
    return
  end

  for name, controllerData in pairs(data) do
    if name and loadedControllers[name] and loadedControllers[name].deserialize then
      loadedControllers[name].deserialize(controllerData)
    end
  end
end

local function onSerialize()
  local data = {}
  for _, controller in ipairs(sortedControllers) do
    if controller.serialize then
      data[controller.name] = controller.serialize()
    end
  end
  return data
end

local function setState(data)
  if not data then
    return
  end

  for _, controller in ipairs(sortedControllers) do
    if controller.setState then
      controller.setState(data)
    end
  end
end

local function getState()
  local data = {}
  for _, controller in ipairs(sortedControllers) do
    if controller.getState then
      tableMergeRecursive(data, controller.getState())
    end
  end

  return tableIsEmpty(data) and nil or data
end

local stateEvents = {}

local function publishStateEvent(controllerName, ...)
  local c = getController(controllerName)
  if not c then
    log("E", "controller.stateEvent", string.format("Can't find controller '%s', ignoring state event.", controllerName))
    return
  end
  if not c.stateEvent then
    log("E", "controller.stateEvent", string.format("Controller '%s' does not support state events, ignoring state event.", controllerName))
    return
  end
  c.stateEvent(...)
end

local function triggerStateEvent(controller, ...)
  table.insert(stateEvents, controller.name)
  table.insert(stateEvents, {...})
  dump(stateEvents)
end

local function getStateEvents()
  return stateEvents
end

local function isPhysicsStepUsed()
  --Check if any controller uses a function relevant to physics step
  return physicsUpdateCount > 0 or fixedStepUpdateCount > 0 or wheelsIntermediateUpdateCount > 0
end

M.init = init
M.reset = reset
M.resetSecondStage = resetSecondStage
M.initSecondStage = initSecondStage
M.resetLastStage = resetLastStage
M.initLastStage = initLastStage
M.initSounds = initSounds
M.resetSounds = resetSounds

M.registerRelocatedController = registerRelocatedController
M.cacheAllControllerFunctions = cacheAllControllerFunctions
M.loadControllerExternal = loadControllerExternal
M.unloadControllerExternal = unloadControllerExternal

M.setFreeze = setFreeze --TBD in the future, use onGameplayEvent with freeze param instead

M.update = nop
M.updateWheelsIntermediate = nop
M.updateGFX = nop
M.beamBroke = nop
M.beamDeformed = nop
M.nodeCollision = nop
M.debugDraw = nop

M.onCouplerFound = nop
M.onCouplerAttached = nop
M.onCouplerDetached = nop

M.onGameplayEvent = nop

M.getController = getController
M.getAllControllers = getAllControllers
M.getControllerSafe = getControllerSafe
M.getControllersByType = getControllersByType
M.getControllersFromPath = getControllersFromPath

M.updateSimpleControlButtons = updateSimpleControlButtons
M.settingsChanged = settingsChanged
M.onDeserialize = onDeserialize
M.onSerialize = onSerialize
M.printDebugMethodCalls = printDebugMethodCalls

M.getState = getState
M.setState = setState
M.publishStateEvent = publishStateEvent
M.triggerStateEvent = triggerStateEvent
M.getStateEvents = getStateEvents

M.isPhysicsStepUsed = isPhysicsStepUsed

return M
