--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local particles = require("particles")

local jbeamIO = require('jbeam/io')
local jbeamTableSchema = require('jbeam/tableSchema')
local jbeamLinks = require('jbeam/links')
local jbeamOptimization = require('jbeam/optimization')
local sectionMerger = require('jbeam/sectionMerger')
local jbeamSlotSystem = require('jbeam/slotSystem')
local jbeamGroups = require('jbeam/groups')
local jbeamScaling = require('jbeam/scaling')
local jbeamInteraction = require('jbeam/interaction')
local jbeamVariables = require('jbeam/variables')
local jbeamCamera = require('jbeam/sections/camera')
local jbeamWheels = require('jbeam/sections/wheels')
local jbeamNodeBeam = require('jbeam/sections/nodeBeam')
local jbeamLicensePlatesSkins = require('jbeam/sections/licenseplatesSkins')
local jbeamAssorted = require('jbeam/sections/assorted')
local jbeamMeshs = require('jbeam/sections/meshs')
local jbeamEvents = require('jbeam/sections/events')
local jbeamColors = require('jbeam/sections/colors')
local jbeamPaints = require('jbeam/sections/paints')
local jbeamMirrors = require('jbeam/sections/mirror')
local jbeamPartColors =  require_optional('jbeam/sections/partColors') or { process=nop }
local jbeamCondition = require_optional('jbeam/sections/condition') or { process=nop }
local jbeamMaterials = require('jbeam/materials')

local M = {}

-- this is intentionally here for the doc sync system
M.defaultBeamSpring = 4300000
M.defaultBeamDeform = 220000
M.defaultBeamDamp   = 580
M.defaultNodeWeight = 25
M.defaultBeamStrength = math.huge

M.data = {}
M.materials, M.materialsMap = particles.getMaterialsParticlesTable()

-- this will inject frame renders during loading
-- if you want to just profile the loading time, you can disable it
local refreshScreenWhileLoading = true

--local cache = {}

-- load all the jbeam and construct the thing in memory
local function loadJbeam(objID, loadingProgress, vehicleDirectories, vehicleConfig, debugMgrContext)
  profilerPushEvent('jbeam/loader')
  if loadingProgress then loadingProgress:update(0.1, 'Reading files...') end
  local ioCtx = jbeamIO.startLoading(vehicleDirectories)

  -- figure out the model name based on the directory given
  local modelName = vehicleDirectories[1]:match('/vehicles/([^/]+)')

  if vehicleConfig == nil then vehicleConfig = {} end
  local debugEnabled = nil
  if vehicleConfig.additionalVehicleData then
    debugEnabled = vehicleConfig.additionalVehicleData.debugEnabled
  end

  vehicleConfig.model = modelName

  if not vehicleConfig.paints and vehicleConfig.colors then
    vehicleConfig.paints = convertVehicleColorsToPaints(vehicleConfig.colors)
    vehicleConfig.colors = nil
  end
  if not vehicleConfig.mainPartName then
    vehicleConfig.mainPartName = jbeamIO.getMainPartName(ioCtx)
  end
  if vehicleConfig.mainPartName then
    vehicleConfig.mainPartPath = '/' .. vehicleConfig.mainPartName
  end

  --log('D', 'loadVehicle', 'spawn config: ' .. dumps(vehicleConfig))

  if loadingProgress then loadingProgress:update(0.2, 'Finding parts...') end
  local vehicle, unifyJournal, unifyJournalC, chosenPartsTree, slotPartMap, activePartsData, activeParts = jbeamSlotSystem.findParts(ioCtx, vehicleConfig)
  if not vehicle then return end

  vehicle.partOrigin = vehicle.partName
  vehicle.partPath = '/' .. vehicle.partName

  local vars = jbeamVariables.getAllVariables(vehicle, unifyJournal, vehicleConfig)

  -- we process all components before processing variables so the variables can use them
  jbeamVariables.processComponents(vehicle, unifyJournalC, vehicleConfig, vars)

  if loadingProgress then loadingProgress:update(0.21, 'Applying variables...') end
  local allVariables = jbeamVariables.processParts(vehicle, unifyJournal, vehicleConfig, vars)

  if loadingProgress then loadingProgress:update(0.22, 'Unifying parts...') end
  if not jbeamSlotSystem.unifyPartJournal(ioCtx, unifyJournal) then return end

  jbeamVariables.postProcessVariables(vehicle, allVariables)

  -- cleanup everything that should not be send over to the other side that is not serializeable
  jbeamVariables.componentsCleanup(vehicle)

  if debugMgrContext and debugMgrContext.dumpDebug[0] then
    table.insert(debugMgrContext.debugTexts, 'vehicleDebug_preTable.json')
    require('jbeamWriter').writeFile('vehicleDebug_preTable.json', vehicle)
  end
  --dump({'chosenPartsTree = ', chosenPartsTree})

  vehicleConfig.parts = nil -- obsolete info, replaced with partsTree
  vehicleConfig.partsTree = chosenPartsTree

  --jsonWriteFile('chosenPartsTree.json', chosenPartsTree, true)
  --jsonWriteFile('vehicle.json', vehicle, true)
  --jsonWriteFile('activePartsData.json', activePartsData, true)

  if loadingProgress then loadingProgress:update(0.3, 'Assembling tables ...') end
  if not jbeamTableSchema.process(vehicle) then
    log('W', "jbeam.compile", "*** preparation error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  -- 0) merge sections together properly. This is primarily for usability of the jbeam
  local sectionRenames = {}
  if not sectionMerger.process(vehicle, sectionRenames) then
    log('W', "jbeam.compile", "*** sectionMerger error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  if loadingProgress then loadingProgress:update(0.4, 'Linking things together ...') end
  -- a) this creates a list of things to be linked AND deletes unlinkable things
  local linksToResolve = jbeamLinks.prepareLinksDestructive(vehicle, sectionRenames)
  if linksToResolve == nil then
    log('W', "jbeam.compile", "*** link preparation error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  -- b) this assigns upcounting continouus IDs for the physics.
  --    Items cannot be added or deleted afterwards
  if not jbeamOptimization.assignCIDs(vehicle) then
    log('W', "jbeam.compile", "*** numbering error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  -- c) This resolves the links with the cids assigned now
  if not jbeamLinks.resolveLinks(vehicle, linksToResolve) then
    log('W', "jbeam.compile", "*** link resolving error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  if loadingProgress then loadingProgress:update(0.5, 'Inspecting variables ...') end
  jbeamNodeBeam.process(vehicle)
  if vmType == 'game' then
    jbeamCamera.process(objID, vehicle)
  end

  if loadingProgress then loadingProgress:update(0.6, 'Adding wheels ...') end
  jbeamWheels.processWheels(vehicle)

  if not jbeamLinks.resolveGroupLinks(vehicle) then
    log('W', "jbeam.postProcess","*** group link resolving error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  if loadingProgress then loadingProgress:update(0.7, 'Doing some things.') end
  jbeamAssorted.process(vehicle) -- after resolveGroupLinks
  jbeamWheels.processRotators(vehicle)
  jbeamGroups.process(vehicle) -- after processWheels, processRotators
  jbeamScaling.process(vehicle) -- after jbeamGroups

  -- add default options
  if vehicle.options.beamSpring   == nil then vehicle.options.beamSpring   = M.defaultBeamSpring end
  if vehicle.options.beamDeform   == nil then vehicle.options.beamDeform   = M.defaultBeamDeform end
  if vehicle.options.beamDamp     == nil then vehicle.options.beamDamp     = M.defaultBeamDamp end
  if vehicle.options.beamStrength == nil then vehicle.options.beamStrength = M.defaultBeamStrength end
  if vehicle.options.nodeWeight   == nil then vehicle.options.nodeWeight   = M.defaultNodeWeight end

  vehicle.vehicleDirectory = vehicleDirectories[1]
  vehicle.directoriesLoaded = vehicleDirectories
  vehicle.activePartsData = activePartsData
  vehicle.activeParts = activeParts
  vehicle.slotPartMap = slotPartMap
  vehicle.model = modelName


  if loadingProgress then loadingProgress:update(0.9, 'Optimizing result') end
  if not jbeamOptimization.process(vehicle, debugEnabled) then
    log('W', "jbeam.compile", "*** optimization error")
    profilerPopEvent('jbeam/loader')
    return nil
  end

  jbeamIO.finishLoading() -- clears some caches

  -- for the UI, after all the jbeam loading is done
  jbeamInteraction.process(vehicle)
  profilerPopEvent('jbeam/loader')
  return {
    id               = objID,
    vehicleDirectory = vehicle.vehicleDirectory,
    vdata            = vehicle,
    config           = vehicleConfig,
    mainPartName     = vehicleConfig.mainPartName,
    ioCtx            = ioCtx,
  }
end

local function loadBundle(objID, vehicleBundle, loadingProgress)
  profilerPushEvent('loadBundle')
  if not vehicleBundle then return end

  local vehicleObj
  if vmType == 'game' then
    vehicleObj = scenetree.findObject(objID)
    if not vehicleObj then
      log('E', 'loader', 'unable to find object with it: ' .. tostring(objID))
    else
      vehicleObj = vehicleObj.obj
    end
  end
  -- everything that needs the object to be working or 3D meshes goes from here:
  if vehicleObj then

    if loadingProgress then loadingProgress:update(0.8, 'Loading input events...') end
    jbeamEvents.process(objID, vehicleObj, vehicleBundle.vdata)

    if loadingProgress then loadingProgress:update(0.8, 'Adding the 3D meshes ...') end
    jbeamLicensePlatesSkins.process(objID, vehicleObj, vehicleBundle.config, vehicleBundle.vdata.activePartsData)
    jbeamColors.process(vehicleObj, vehicleBundle.config, vehicleBundle.vdata)
    jbeamPaints.process(vehicleObj, vehicleBundle.config, vehicleBundle.vdata)
    jbeamPartColors.process(vehicleObj, vehicleBundle.config, vehicleBundle.vdata)
    jbeamCondition.process(vehicleObj, vehicleBundle.config, vehicleBundle.vdata, vehicleBundle)

    -- set initial node positions
    if vehicleBundle.vdata.maxIDs.nodes then
      vehicleObj:setInitialNodePositionCount(vehicleBundle.vdata.maxIDs.nodes)
      local nodes = vehicleBundle.vdata.nodes
      for i = 0, tableSizeC(vehicleBundle.vdata.nodes) - 1 do
        local n = nodes[i]
        vehicleObj:setInitialNodePosition(n.pos.x, n.pos.y, n.pos.z)
        local staticCollision = n.staticCollision
        if staticCollision == nil then staticCollision = true end
        local collision = n.collision
        if collision == nil then collision = true end
        vehicleObj:setInitialNodeCollision(collision, staticCollision)
      end
      vehicleObj:initialNodePositionsDone()
    end
    local refNodes = vehicleBundle.vdata.refNodes[0]
    vehicleObj:setRefNodes(refNodes.ref or 0, refNodes.back or 0, refNodes.left or 0, refNodes.up or 0)

    jbeamMeshs.process(objID, vehicleObj, vehicleBundle.vdata)
    jbeamMaterials.process(vehicleObj, vehicleBundle.vdata)
    jbeamMirrors.process(objID, vehicleObj, vehicleBundle.vdata)

    --some configs hardcode their colors in the info json rather than referencing a color name from the main json
    --only look for that if we don't have any other color information already, otherwise things go wrong
    if not vehicleBundle.config.paints then
      vehicleBundle.config.paints = deserialize(vehicleObj.paints or '{}')
    end

    if vehicleBundle.vdata.animation then
      vehicleObj:queueLuaCommand('extensions.load("test_animationViz")')
    end

  else
    vehicleBundle.vdata.props = {}
    vehicleBundle.vdata.flexbodies = {}
  end
  profilerPopEvent('loadBundle')
end

-- be aware this code runs on vehicle and ge lua
local function loadVehicleStage1(objID, vehicleDir, vehicleConfig, debugMgrContext)
  profilerPushEvent('loadVehicleStage1')
  local loadingProgress

  if debugMgrContext and debugMgrContext.dumpDebug[0] then
    debugMgrContext.debugTexts = {'Creating debug output:'}
  end

  if refreshScreenWhileLoading and vmType == 'game' then
    loadingProgress = LoadingManager:push('beamng')
  end

  -- the directory needs a leading and trailing slash
  if vehicleDir:sub(1, 1) ~= '/' then
    vehicleDir = '/' .. vehicleDir
  end
  if vehicleDir:sub(-1, -1) ~= '/' then
    vehicleDir = vehicleDir .. '/'
  end

  local vehicleDirectories = {vehicleDir, '/vehicles/common/'}
  --local spawnHash = nil
  --if hashStringSHA256 then
  --  spawnHash = hashStringSHA256(dumps(vehicleDirectories) .. '#' .. jsonEncode(vehicleConfig))
  --end
  --print(">>> spawnHash = " .. tostring(spawnHash))

  local vehicleBundle = nil -- cache[spawnHash]

  if not vehicleBundle then
    vehicleBundle = loadJbeam(objID, loadingProgress, vehicleDirectories, vehicleConfig, debugMgrContext)
    --if spawnHash then cache[spawnHash] = vehicleBundle end
  end

  --log('D', 'loader', 'jbeam LOADING TOOK: ' .. tostring(t:stopAndReset()) .. ' ms')

  --- for debug purposes:
  if vehicleBundle and debugMgrContext and debugMgrContext.dumpDebug[0] then
    local activePartsData = vehicleBundle.vdata.activePartsData
    vehicleBundle.vdata.activePartsData = nil
    jsonWriteFile('vehicleDebug_data.json', vehicleBundle.vdata, true)
    table.insert(debugMgrContext.debugTexts, 'vehicleDebug_data.json')
    jsonWriteFile('vehicleDebug_activeParts.json', activePartsData, true)
    vehicleBundle.vdata.activePartsData = activePartsData
    table.insert(debugMgrContext.debugTexts, 'vehicleDebug_activeParts.json')
    jsonWriteFile('vehicleDebug_config.json', vehicleBundle.config, true)
    table.insert(debugMgrContext.debugTexts, 'vehicleDebug_config.json')
    jsonWriteFile('vehicleDebug_Tree.json', vehicleBundle.config.partsTree, true)
    table.insert(debugMgrContext.debugTexts, 'vehicleDebug_Tree.json')
  end


  --jsonWriteFile('vehicleBundle.json', vehicleBundle, true)

  loadBundle(objID, vehicleBundle, loadingProgress)

  --log('D', 'loader', '3D LOADING TOOK: ' .. tostring(t:stopAndReset()) .. ' ms')

  profilerPopEvent('loadVehicleStage1')

  if loadingProgress then
    loadingProgress:update(1, 'Vehicle loading done')
    LoadingManager:pop(loadingProgress)
  end

  return vehicleBundle
end

-- public interface
M._noSerialize = true
M.loadVehicleStage1 = loadVehicleStage1
M.loadBundle = loadBundle

return M
