-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- jit.opt.start(3,'minstitch=10000000','fma')
vmType = "vehicle"

package.path = "lua/vehicle/?.lua;?.lua;lua/common/?.lua;lua/common/libs/luasocket/?.lua;lua/?.lua;?.lua"
package.cpath = ""
require("luaCore")
require('common/cdefMath')

print = function(...)
  log("A", "print", tostring(...))
  -- log('A', "print", debug.traceback()) -- find where print is used
end

require("utils")
require("devUtils")
require("ve_utils")
require("mathlib")
float3 = vec3
require("controlSystems")
local STP = require "libs/StackTracePlus/StackTracePlus"
debug.traceback = STP.stacktrace
debug.tracesimple = STP.stacktraceSimple

-- nop's for the profiler functions if not present
profilerPushEvent = profilerPushEvent or nop
profilerPopEvent = profilerPopEvent or nop

extensions = require("extensions")
extensions.addModulePath("lua/vehicle/extensions/")
extensions.addModulePath("lua/common/extensions/")

profilerPushEvent("lua init")

settings = require("settings")
backwardsCompatibility = require("backwardsCompatibility")
objectId = obj:getId() -- also set by c++
vehiclePath = nil

playerInfo = {
  -- seatedPlayers = nil, -- list of players seated in this vehicle; players are indexed from 0 to N (e.g. { [1]=true, [4]=true } for 2nd and 5th players)
  firstPlayerSeated = false,
  anyPlayerSeated = false
}
lastDt = 1 / 20
physicsDt = obj:getPhysicsDt()
ghostOnReset, ghostOnTp = nil, nil

local extensionsHook = nop

function updateCorePhysicsStepEnabled()
  -- print("Controller: " .. tostring(controller.isPhysicsStepUsed()))
  -- print("Powertrain: " .. tostring(powertrain.isPhysicsStepUsed()))
  -- print("Wheels: " .. tostring(wheels.isPhysicsStepUsed()))
  -- print("Thrusters: " .. tostring(thrusters.isPhysicsStepUsed()))
  -- print("Hydros: " .. tostring(hydros.isPhysicsStepUsed()))
  -- print("Beamstate: " .. tostring(beamstate.isPhysicsStepUsed()))
  -- print("---")
  obj:setPhysicsStepEnabled(controller.isPhysicsStepUsed() or powertrain.isPhysicsStepUsed() or wheels.isPhysicsStepUsed() or thrusters.isPhysicsStepUsed() or hydros.isPhysicsStepUsed() or beamstate.isPhysicsStepUsed() or protocols.isPhysicsStepUsed() or extensionsHook ~= nop)
end

function enablePhysicsStepHook()
  extensionsHook = extensions.hook
  updateCorePhysicsStepEnabled()
end

--!!! PERFORMANCE HEAVY, DEBUG ONLY !!!
--function onUnstableNode(nodeId)
--  print("onUnstableNode: " .. nodeId)
--end

-- step functions
function onPhysicsStep(dtPhys)
  wheels.updateWheelVelocities(dtPhys)
  powertrain.update(dtPhys)
  controller.updateWheelsIntermediate(dtPhys)
  wheels.updateWheelTorques(dtPhys)
  controller.update(dtPhys)
  thrusters.update()
  hydros.update(dtPhys)
  beamstate.update(dtPhys)
  protocols.update(dtPhys)
  extensionsHook("onPhysicsStep", dtPhys)
end

-- This is called in the local scope, so it is NOT safe to do things that contact things outside the vehicle
function onGraphicsStep(dtSim)
  lastDt = dtSim
  sensors.updateGFX(dtSim) -- must be before input and ai
  mapmgr.sendTracking() -- must be before ai
  wheels.updateGFX(dtSim)
  ai.updateGFX(dtSim) -- must be before input and after wheels
  input.updateGFX(dtSim) -- must be as early as possible
  electrics.updateGFX(dtSim)
  controller.updateGFX(dtSim)
  electrics.updateGFXSecondStep(dtSim)
  extensions.hook("updateGFX", dtSim) -- must be before drivetrain, hydros and after electrics
  hydros.updateGFX(dtSim) -- must be early for FFB, but after (input, electrics) and before props
  powertrain.updateGFX(dtSim)
  energyStorage.updateGFX(dtSim)
  drivetrain.updateGFX(dtSim)
  beamstate.updateGFX(dtSim) -- must be after drivetrain
  protocols.updateGFX(dtSim)
  sounds.updateGFX(dtSim)
  thrusters.updateGFX() -- should be after extensions.hook

  if streams.hasActiveStreams() and obj:getUpdateUIflag() then
    guihooks.updateStreams = true
    guihooks.sendStreams()
  else
    guihooks.updateStreams = false
  end

  if playerInfo.firstPlayerSeated then
    damageTracker.updateGFX(dtSim)
  end

  props.updateGFX() -- must be after hydros
  material.updateGFX()
  fire.updateGFX(dtSim)
  recovery.updateGFX(dtSim)
  powertrain.updateGFXLastStage(dtSim)
end

-- debug rendering
local focusPos = vec3(0, 0, 0)
function onDebugDraw(x, y, z)
  focusPos.x, focusPos.y, focusPos.z = x, y, z
  bdebug.debugDraw(focusPos)
  ai.debugDraw(focusPos)
  beamstate.debugDraw(focusPos)
  controller.debugDraw(focusPos)
  hydros.debugDraw()
  extensions.hook("onDebugDraw", focusPos)
  if playerInfo.anyPlayerSeated then
    extensions.hook("onDebugDrawActive", focusPos)
  end
end

function initSystems()
  profilerPushEvent("3.1 init - compat")
  backwardsCompatibility.init()
  profilerPopEvent("3.1 init - compat")

  profilerPushEvent("3.2.X init - materials (sum)")
  material.init()
  profilerPopEvent("3.2.X init - materials (sum)")

  profilerPushEvent("3.2 init - first stage")
  bdebug.init()
  electrics.init()
  damageTracker.init()
  beamstate.init() -- needs to go before powertrain and first controller init, needs to go after damageTracker
  protocols.init()
  wheels.init()
  powertrain.init()
  energyStorage.init()
  input.init()
  controller.init() -- needs to go after input first stage
  profilerPopEvent("3.2 init - first stage")

  profilerPushEvent("3.3 init - second stage")
  wheels.initSecondStage()
  controller.initSecondStage()
  drivetrain.init()
  profilerPopEvent("3.3 init - second stage")

  profilerPushEvent("3.4 init - groupA")
  sensors.reset()
  thrusters.init()
  hydros.init()
  profilerPopEvent("3.4 init - groupA")

  profilerPushEvent("3.5 init - audio")
  sounds.init()
  profilerPopEvent("3.5 init - audio")

  profilerPushEvent("3.6 init - groupB")
  props.init()
  input.initSecondStage() -- needs to go after sounds & electrics
  recovery.init()
  sensors.init()
  fire.init()
  wheels.initSounds()
  powertrain.initSounds()
  controller.initSounds()
  guihooks.message("", 0, "^vehicle\\.") -- clear damage messages on vehicle restart
  profilerPopEvent("3.6 init - groupB")

  profilerPushEvent("3.7 init - extensions")
  extensions.hook("onInit")
  profilerPopEvent("3.7 init - extensions")

  profilerPushEvent("3.8 init - last stage")
  mapmgr.init()

  electrics.initLastStage()
  controller.initLastStage() --meant to be last in init
  powertrain.sendTorqueData()

  -- be sensitive about global writes from now on
  detectGlobalWrites()
  updateCorePhysicsStepEnabled()
  profilerPopEvent("3.8 init - last stage")
end

function init(path, initData)
  profilerPushEvent("4.X.X.X total (sum)")

  profilerPushEvent("0 startup")

  if not obj then
    log("W", "default.init", "Error getting main object: unable to spawn")
    return
  end
  log("D", "default.init", "spawning vehicle " .. tostring(path))

  -- we change the lookup path here, so it prefers the vehicle lua
  package.path = path .. "/lua/?.lua;" .. package.path
  vehiclePath = path
  extensions.loadModulesInDirectory(path .. "/lua", {"controller", "powertrain", "energyStorage"})

  extensions.load("core_quickAccess")

  damageTracker = require("damageTracker")
  drivetrain = require("drivetrain")
  powertrain = require("powertrain")
  powertrain.setVehiclePath(path)
  energyStorage = require("energyStorage")
  controller = require("controller")

  wheels = require("wheels")
  sounds = require("sounds")
  -- vehedit = require('vehicleEditor/veMain')
  bdebug = require("bdebug")
  input = require("input")
  props = require("props")

  particles = require("particles")
  particlefilter = require("particlefilter")
  material = require("material")
  v = require("jbeam/stage2")
  electrics = require("electrics")
  beamstate = require("beamstate")
  protocols = require("protocols")
  sensors = require("sensors")
  bullettime = require("bullettime") -- to be deprecated
  thrusters = require("thrusters")
  hydros = require("hydros")
  guihooks = require("guihooks") -- do not change its name, the GUI callback will break otherwise
  streams = require("guistreams")
  gui = guihooks -- backward compatibility
  ai = require("ai")
  recovery = require("recovery")
  mapmgr = require("mapmgr")
  fire = require("fire")
  partCondition = require("partCondition")

  profilerPopEvent("0 startup")

  profilerPushEvent("loadVehicleStage2 (sum)")

  -- care about the config before pushing to the physics
  local vehicle
  if type(initData) == "string" and string.len(initData) > 0 then
    profilerPushEvent("deserialize")
    local state, initData = pcall(lpack.decode, initData)
    profilerPopEvent("deserialize")
    if state and type(initData) == "table" then
      if initData.vdata then
        vehicle = v.loadVehicleStage2(initData)
      else
        log("E", "vehicle", "unable to load vehicle: invalid spawn data")
      end
    end
  else
    log("E", "vehicle", "invalid initData: " .. tostring(type(initData)) .. ": " .. tostring(initData))
  end

  if not vehicle then
    log("E", "loader", "vehicle loading failed fatally")
    return false -- return false = unload lua
  end
  profilerPopEvent("loadVehicleStage2 (sum)")

  -- you can change the data in here before it gets submitted to the physics

  if v.data == nil then
    v.data = {}
  end

  -- disable lua for simple vehicles
  if v.data.information and v.data.information.simpleObject == true then
    log("I", "", "lua disabled!")
    return false -- return false = unload lua
  end

  profilerPushEvent("3.X init systems (sum)")
  initSystems()
  profilerPopEvent("3.X init systems (sum)")

  -- temporary tire mark setting
  obj:setSlipTireMarkThreshold(10)

  --Load skeleton extension that draws nice(r) beams and nodes if there are no meshes, unloads itself immediately otherwise
  extensions.load("skeleton")

  profilerPushEvent("5 postspawn")

  -- load the extensions at this point in time, so the whole jbeam is parsed already
  extensions.loadModulesInDirectory("lua/vehicle/extensions/auto")

  -- extensions that always load

  extensions.hook("onVehicleLoaded", retainDebug)

  --extensions.load('vehicleEditor_veMain')
  extensions.load("gameplayStatistic")

  profilerPopEvent("5 postspawn")
  profilerPopEvent("4.X.X.X total (sum)")

  return true -- false = unload Lua
end

-- various callbacks
function onCallEvent(funName, data)
  if type(funName) ~= "string" then
    return
  end

  local f
  local _, j = string.find(funName, ".", 1, true)
  if j then
    local m = _G[string.sub(funName, 1, j - 1)]
    if type(m) == "table" then
      f = m[string.sub(funName, j + 1)]
    end
  else
    f = _G[funName]
  end

  if type(f) == "function" then
    f(data)
  end
end

function onBeamBroke(id, energy)
  beamstate.beamBroken(id, energy)
  wheels.beamBroke(id)
  powertrain.beamBroke(id)
  energyStorage.beamBroke(id)
  controller.beamBroke(id, energy)
  bdebug.beamBroke(id, energy)
  extensions.hook("onBeamBroke", id, energy)
end

-- only being called if the beam has deform triggers
function onBeamDeformed(id, ratio)
  beamstate.onBeamDeformed(id, ratio)
  controller.beamDeformed(id, ratio)
  extensions.hook("onBeamDeformed", id, ratio)
end

function onTorsionbarBroken(id, energy)
  beamstate.torsionBarBroken(id, energy)
  extensions.hook("onTorsionbarBroken", id, energy)
end

function onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  -- print('couplerFound'..','..nodeId..','..obj2nodeId..','..obj2id)
  beamstate.couplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  controller.onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  powertrain.onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  energyStorage.onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  extensions.hook("onCouplerFound", nodeId, obj2id, obj2nodeId, nodeDist)
end

function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  -- print('couplerAttached'..','..nodeId..','..obj2nodeId..','..obj2id..','..attachSpeed)
  beamstate.onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  controller.onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  powertrain.onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  energyStorage.onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  extensions.hook("onCouplerAttached", nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
end

function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  -- print('couplerDetached'..','..nodeId..','..obj2nodeId..','..obj2id..','..breakForce)
  beamstate.onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  controller.onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  powertrain.onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  energyStorage.onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  extensions.hook("onCouplerDetached", nodeId, obj2id, obj2nodeId, breakForce)
end

function onDynamicBeamAdded(dbId, nodeId, tag)
end

function onDynamicBeamDeleted(dbId)
end

function onDynamicBeamBroke(dbId, energy)
end

-- called when vehicle is removed
function onDespawnObject()
  --log('D', "default.vehicleDestroy", "vehicleDestroy()")
  hydros.destroy()
  protocols.destroy()
  if odometer then
    odometer.submitStatistic()
  end
end

function onSetClusterPosRelRot(cNodeId, pos, relRot)
  if ghostOnTp ~= nil then
    obj:setGhostEnabled(ghostOnTp)
  end

  extensions.hook("onSetClusterPosRelRot", cNodeId, pos, relRot)
end

-- called when the user pressed I
function onVehicleReset(retainDebug)
  if ghostOnReset ~= nil then
    obj:setGhostEnabled(ghostOnReset)
  end

  guihooks.reset()
  extensions.hook("onReset", retainDebug)
  ai.reset()
  mapmgr.reset()

  --log('D', "default.vehicleResetted", "vehicleResetted()")
  damageTracker.reset()
  beamstate.reset() --needs to be before any calls to beamnstate.registerExternalCouplerBreakGroup(), for example controller.lua
  protocols.reset()
  wheels.reset()
  electrics.reset()
  powertrain.reset()
  energyStorage.reset()
  controller.reset()
  wheels.resetSecondStage()
  controller.resetSecondStage()
  drivetrain.reset()
  props.reset()
  sensors.reset()
  bdebug.reset()
  thrusters.reset()
  input.reset()
  hydros.reset()
  material.reset()
  fire.reset()
  powertrain.resetSounds()
  controller.resetSounds()
  sounds.reset()
  partCondition.reset()

  electrics.resetLastStage()
  controller.resetLastStage() --meant to be last in reset
  powertrain.sendTorqueData()
  updateCorePhysicsStepEnabled()

  guihooks.message("", 0, "^vehicle\\.") -- clear damage messages on vehicle restart
end

function onNodeCollision(id1, pos, normal, nodeVel, perpendicularVel, slipVec, slipVel, slipForce, normalForce, depth, materialId1, materialId2)
  local p = particlefilter.particleData
  p.id1, p.pos, p.normal, p.nodeVel, p.perpendicularVel = id1, pos, normal, nodeVel, perpendicularVel
  p.slipVec, p.slipVel, p.slipForce = slipVec, slipVel, slipForce
  p.normalForce, p.depth, p.materialID1, p.materialID2 = normalForce, depth, materialId1, materialId2

  wheels.nodeCollision(p)
  fire.nodeCollision(p)
  controller.nodeCollision(p)
  particlefilter.nodeCollision(p)
  bdebug.nodeCollision(p)
end

function setControllingPlayers(players)
  playerInfo.seatedPlayers = players
  playerInfo.anyPlayerSeated = not (tableIsEmpty(players))
  playerInfo.firstPlayerSeated = players[0] ~= nil

  if playerInfo.anyPlayerSeated then
    if controller and controller.mainController then
      if controller.mainController.vehicleActivated then --TBD, only vehicleActivated should be there
        controller.mainController.vehicleActivated()
      else
        controller.mainController.sendTorqueData()
      end
    end

    powertrain.sendTorqueData()
    damageTracker.sendNow() --send over damage data of (now) active vehicle
  end

  bdebug.onPlayersChanged(playerInfo.anyPlayerSeated)
  protocols.onPlayersChanged()
  ai.stateChanged()
  sounds.updateObjType()
  extensions.hook("onPlayersChanged", playerInfo.anyPlayerSeated) -- backward compatibility
end

function exportPersistentData()
  local d = serializePackages("reload")
  --log('D', "default.exportPersistentData", d)
  obj:setPersistentData(serialize(d))
end

function importPersistentData(s)
  --log('D', "default.importPersistentData", s)
  -- deserialize extensions first, so the extensions are loaded before they are trying to get deserialized
  deserializePackages(deserialize(s))
end

function onSettingsChanged()
  settings.settingsChanged()
  extensions.hook("onSettingsChanged")
  controller.settingsChanged()
  input.settingsChanged()
  wheels.settingsChanged()
  protocols.settingsChanged()
end

profilerPopEvent("lua init")
