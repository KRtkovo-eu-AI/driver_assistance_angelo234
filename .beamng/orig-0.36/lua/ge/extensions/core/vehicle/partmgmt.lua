-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local json = require("json")
local jbeamIO = require('jbeam/io')

local vehManager = extensions.core_vehicle_manager
local vehsPartsData = {}

-- If inVehID is nil, it uses player vehicle
local function getVehData(inVehID)
  local vehObj = inVehID and getObjectByID(inVehID) or getPlayerVehicle(0)
  if not vehObj then return end
  local vehID = vehObj:getID()

  local vehData = vehManager.getVehicleData(vehID)
  if not vehData then return end

  if not vehsPartsData[vehID] then
    local partsHighlighted = {}
    local partsHighlightedIdxs = {}
    local partNameToIdx = {}

    -- A local helper function to recurse through the 'children'.
    local function recGetPart(node, outPartsHighlighted, outPartsHighlightedIdxs, outPartNameToIdx)
      if node.partPath then
        outPartsHighlighted[node.partPath] = true
      end
      -- Recurse on any further children.
      if node.children then
        for _, childNode in pairs(node.children) do
          recGetPart(childNode, outPartsHighlighted, outPartsHighlightedIdxs, outPartNameToIdx)
        end
      end
    end

    recGetPart(vehData.config.partsTree or {}, partsHighlighted, partsHighlightedIdxs, partNameToIdx)
    local partsSorted = tableKeysSorted(partsHighlighted)

    for k, partName in ipairs(partsSorted) do
      partsHighlighted[partName] = true
      table.insert(partsHighlightedIdxs, k)
      partNameToIdx[partName] = k
    end

    vehsPartsData[vehID] = {
      vehName = vehObj:getJBeamFilename(),
      alpha = 1,
      partsSorted = partsSorted,
      partsHighlighted = partsHighlighted,
      partsHighlightedIdxs = partsHighlightedIdxs,
      partNameToIdx = partNameToIdx,
    }
  end

  return vehObj, vehData, vehID, vehsPartsData[vehID]
end

local function getDefaultConfigFileFromDir(vehicleDir, configData)
  local vehicleInfo = jsonReadFile(vehicleDir .. '/info.json')
  if not vehicleInfo then return end
  if not vehicleInfo.default_pc then return end
  log('W', 'main', "Supplied config file: " .. dumps(configData) .. " not found. Using default config instead.")
  return vehicleDir .. vehicleInfo.default_pc .. ".pc"
end

local function buildConfigFromString(vehicleDir, configData, onlyReturnChosenConfig)
  local function preprocessPartConfig(configData)
    -- If the config data format is 4, then we need to preprocess it
    -- Replace references to part config files with the actual part config data
    if configData.format == 4 then
      for _, vehData in ipairs(configData.vehicles) do
        if vehData.linkedPCFile then
          local data, isChosenConfigReturned = buildConfigFromString(vehData.linkedPCFile, nil, true)
          if isChosenConfigReturned then
            tableMerge(vehData, data)
          else
            return false, nil
          end
        end
      end
    else
      return true, configData
    end
  end

  local dataType = type(configData)
  local fileData
  local isChosenConfigReturned = false

  if dataType == 'table' then
    local res, newConfigData = preprocessPartConfig(configData)
    if res then
      isChosenConfigReturned = true
      return newConfigData, isChosenConfigReturned
    end
  elseif dataType == 'string' and configData:sub(1, 1) == '{' then
    local res, newConfigData = preprocessPartConfig(deserialize(configData))
    if res then
      isChosenConfigReturned = true
      return newConfigData, isChosenConfigReturned
    end
  elseif configData ~= nil and configData ~= "" then
    fileData = jsonReadFile(configData)
    if fileData then
      local res, newConfigData = preprocessPartConfig(fileData)
      if res then
        isChosenConfigReturned = true
        fileData = newConfigData
      else
        fileData = nil
      end
    else
      log("W", "", "Unable to read json contents for configData file path: "..dumps(configData))
    end
  end

  if onlyReturnChosenConfig and not isChosenConfigReturned then
    return nil, false
  end

  -- Default to default config if config not found
  if not fileData then
    log("W", "", "Problems reading requested configuration: "..dumps(configData))
    configData = getDefaultConfigFileFromDir(vehicleDir, configData)
    if configData then
      fileData = jsonReadFile(configData)
    end
  end

  local res = {}
  res.partConfigFilename = configData
  if fileData and fileData.format == 2 then
    fileData.format = nil
    tableMerge(res, fileData)
  else
    res.parts = fileData or {}
  end

  return res, isChosenConfigReturned
end

local function saveVehicle(vehEntry)
  local vehId = vehEntry.vehId
  local vehicle = getObjectByID(vehId)
  local vehicleData = vehManager.getVehicleData(vehId)
  if not vehicle or not vehicleData then
    log('E', 'partmgmt', 'vehicle ' .. tostring(vehId) .. ' not found')
    return
  end

  local data

  -- Check if vehicle originates from a part config file
  local partConfigFilename = vehicle.partConfig
  if partConfigFilename and string.endswith(partConfigFilename, '.pc') then
    -- Vehicle originates from a part config file
    data = {}
    data.linkedPCFile = partConfigFilename
  else
    -- Vehicle doesn't originate from a part config file
    data = deepcopy(vehicleData.config)
    data.linkedPCFile = nil
    data.model = vehicleData.model or vehicleData.vehicleDirectory:gsub("vehicles/", ""):gsub("/", "")
    data.partsCondition = partsCondition
    if not data.paints or data.colors then
      data.paints = {}
      local colorTable = vehicle:getColorFTable()
      local colorTableSize = tableSize(colorTable)
      for i = 1, colorTableSize do
        local metallicPaintData = stringToTable(vehicle:getField('metallicPaintData', i - 1))
        local paint = createVehiclePaint({x = colorTable[i].r, y = colorTable[i].g, z = colorTable[i].b, w = colorTable[i].a}, metallicPaintData)
        validateVehiclePaint(paint)
        table.insert(data.paints, paint)
      end

      if #data.paints > 0 then
        data.colors = nil
      end
    end
    data.licenseName = extensions.core_vehicles.makeVehicleLicenseText()

    local legacySlotMap = {}
    local legacySlotMapSimple = {}

    -- we want to have a simple key: value structure for the parts

    local function  flattenPartsTreeRecursive(node)
      if not node then return end
      for slotId, child in pairs(node.children or {}) do
        legacySlotMap[child.path] = {
          slotId = child.id,
          path = child.path,
          chosenPartName = child.chosenPartName,
        }
        flattenPartsTreeRecursive(child)
      end
    end
    flattenPartsTreeRecursive(data.partsTree)
    data.partsTree = nil
    -- now simplify it
    for path, slotData in pairs(legacySlotMap) do
      if not legacySlotMapSimple[slotData.slotId] then
        legacySlotMapSimple[slotData.slotId] = slotData
      else
        -- we have a collision, so we need to save both slots with the full path
        local tmp = legacySlotMapSimple[slotData.slotId]
        if tmp ~= "COLLISION" then
          legacySlotMapSimple[tmp.path] = tmp
          legacySlotMapSimple[slotData.slotId] = "COLLISION"
        end
        legacySlotMapSimple[slotData.path] = slotData
      end
    end
    -- now discard the complex data
    for key, slotData in pairs(legacySlotMapSimple) do
      if slotData ~= "COLLISION" then
        legacySlotMapSimple[key] = slotData.chosenPartName
      else
        legacySlotMapSimple[key] = nil
      end
    end
    -- save to the resulting data table
    data.parts = legacySlotMapSimple
  end

  data.id = vehEntry.id
  data.offsetData = vehEntry.offsetData

  --dump{'coupledNodes = ', vehId, coupledNodes}
  data.format = nil -- remove obsolete key
  data.mainPartPath = nil

  return data
end

-- Compile the vehicle collection tree into a list of vehicles, resolve offsets, and assign pre-runtime ids
local function compileVehicleCollection(collectionTree)
  local collectionTreeCopy = deepcopy(collectionTree)

  -- Calculate offset between parent and child
  local function calculateOffsetRec(entry, parentEntry)
    if parentEntry then
      local offsetData = entry.offsetData
      local veh, parentVeh = getObjectByID(entry.vehId), getObjectByID(parentEntry.vehId)
      if not veh then
        log('E', 'partmgmt', 'vehicle not found for entry: ' .. tostring(entry.vehId))
        return
      end
      if not parentVeh then
        log('E', 'partmgmt', 'parent vehicle not found for entry: ' .. tostring(entry.vehId))
        return
      end

      local vehicleMat, parentVehMat = veh:getRefNodeMatrix(), parentVeh:getRefNodeMatrix()
      local offsetMat = parentVehMat:inverse() * vehicleMat
      local offsetEuler = offsetMat:toEuler()

      if offsetData.type == 'default' then
        local offsetPos = offsetMat:getPosition()
        entry.offsetData.offset = {
          x = roundNear(offsetPos.x, 0.001),
          y = roundNear(offsetPos.y, 0.001),
          z = roundNear(offsetPos.z, 0.001),
          rx = roundNear(math.deg(offsetEuler.x), 0.1),
          ry = roundNear(math.deg(offsetEuler.y), 0.1),
          rz = roundNear(math.deg(offsetEuler.z), 0.1),
        }

      elseif offsetData.type == 'coupledNodes' then
        entry.offsetData.offset = {
          rx = roundNear(math.deg(offsetEuler.x), 0.1),
          ry = roundNear(math.deg(offsetEuler.y), 0.1),
          rz = roundNear(math.deg(offsetEuler.z), 0.1),
        }
      end
    end

    for _, child in ipairs(entry.children) do
      calculateOffsetRec(child, entry)
    end
  end
  calculateOffsetRec(collectionTreeCopy, nil)

  local vehicles = {}
  local vehIdMap = {}

  local function createVehIdMapRec(entry)
    vehIdMap[entry.vehId] = tableSize(vehIdMap) + 1
    for _, child in ipairs(entry.children) do
      createVehIdMapRec(child)
    end
  end

  createVehIdMapRec(collectionTreeCopy)

  local function convertVehIdToIdx(entry, parentEntry)
    local offsetData = entry.offsetData
    if offsetData then
      local parentVehId = parentEntry.vehId
      local parentId = vehIdMap[parentVehId]
      if parentId then
        offsetData.parentId = parentId
      end
    end

    local data = saveVehicle(entry)
    if data then
      table.insert(vehicles, data)
    end

    for _, child in ipairs(entry.children) do
      convertVehIdToIdx(child, entry)
    end
  end

  convertVehIdToIdx(collectionTreeCopy)

  return vehicles
end

local function saveVehicleCollection(collectionTree, filename)
  log('D', 'partmgmt', 'saving vehicle collection... ' .. filename)

  local vehicles = compileVehicleCollection(collectionTree)

  local res = {}
  res.format = 4
  res.vehicles = vehicles

  local writeRes = jsonWriteFile(filename, res, true)
  if writeRes then
    log('D', 'partmgmt', 'vehicle collection saved to ' .. filename)
    guihooks.trigger("VehicleconfigSaved", {})
  else
    log('W', "vehicles.save", "unable to save config: "..filename)
  end
  guihooks.trigger('Message', {ttl = 15, msg = 'Configuration saved', icon = 'directions_car'})
end

local function savePartConfigFileStage2(partsCondition, filename)
  local playerVehicle = getPlayerVehicle(0)
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  local playerVehicleId = playerVehicle:getID()
  local collectionTree = core_vehicles.generateAttachedVehiclesTree(playerVehicleId)
  local vehicles = compileVehicleCollection(collectionTree)

  local res = {}
  res.format = 4
  res.vehicles = vehicles

  local writeRes = jsonWriteFile(filename, res, true)
  if writeRes then
    guihooks.trigger("VehicleconfigSaved", {})
  else
    log('W', "vehicles.save", "unable to save config: "..filename)
  end
  guihooks.trigger('Message', {ttl = 15, msg = 'Configuration saved', icon = 'directions_car'})
end

-- TODO: remove this later
local function savePartConfigFileStage2_Format2(partsCondition, filename)
  local playerVehicle = getPlayerVehicle(0)
  local playerVehicleData = vehManager.getPlayerVehicleData()
  if not playerVehicle or not playerVehicleData then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end

  local data = deepcopy(playerVehicleData.config)
  local prevPCFilename = data.partConfigFilename
  data.partConfigFilename = nil
  data.format = 2
  data.model = playerVehicleData.model or playerVehicleData.vehicleDirectory:gsub("vehicles/", ""):gsub("/", "")
  data.partsCondition = partsCondition
  if not data.paints or data.colors then
    data.paints = {}
    local colorTable = playerVehicle:getColorFTable()
    local colorTableSize = tableSize(colorTable)
    for i = 1, colorTableSize do
      local metallicPaintData = stringToTable(playerVehicle:getField('metallicPaintData', i - 1))
      local paint = createVehiclePaint({x = colorTable[i].r, y = colorTable[i].g, z = colorTable[i].b, w = colorTable[i].a}, metallicPaintData)
      validateVehiclePaint(paint)
      table.insert(data.paints, paint)
    end

    if #data.paints > 0 then
      data.colors = nil
    end
  end
  data.licenseName = extensions.core_vehicles.makeVehicleLicenseText()

  local legacySlotMap = {}
  local legacySlotMapSimple = {}

  -- we want to have a simple key: value structure for the parts

  local function  flattenPartsTreeRecursive(node)
    if not node then return end
    for slotId, child in pairs(node.children or {}) do
      legacySlotMap[child.path] = {
        slotId = child.id,
        path = child.path,
        chosenPartName = child.chosenPartName,
      }
      flattenPartsTreeRecursive(child)
    end
  end
  flattenPartsTreeRecursive(data.partsTree)
  data.partsTree = nil
  -- now simplify it
  for path, slotData in pairs(legacySlotMap) do
    if not legacySlotMapSimple[slotData.slotId] then
      legacySlotMapSimple[slotData.slotId] = slotData
    else
      -- we have a collision, so we need to save both slots with the full path
      local tmp = legacySlotMapSimple[slotData.slotId]
      if tmp ~= "COLLISION" then
        legacySlotMapSimple[tmp.path] = tmp
        legacySlotMapSimple[slotData.slotId] = "COLLISION"
      end
      legacySlotMapSimple[slotData.path] = slotData
    end
  end
  -- now discard the complex data
  for key, slotData in pairs(legacySlotMapSimple) do
    if slotData ~= "COLLISION" then
      legacySlotMapSimple[key] = slotData.chosenPartName
    else
      legacySlotMapSimple[key] = nil
    end
  end
  -- save to the resulting data table
  data.parts = legacySlotMapSimple

  local res = jsonWriteFile(filename, data, true)
  if res then
    data.partConfigFilename = filename
    guihooks.trigger("VehicleconfigSaved", {})
  else
    data.partConfigFilename = prevPCFilename
    log('W', "vehicles.save", "unable to save config: "..filename)
  end
  guihooks.trigger('Message', {ttl = 15, msg = 'Configuration saved', icon = 'directions_car'})
end

local function savePartConfigFile(filename)
  local savePartsCondition = false
  if savePartsCondition then
    local playerVehicle = getPlayerVehicle(0)
    if playerVehicle then
      queueCallbackInVehicle(playerVehicle, "extensions.core_vehicle_partmgmt.savePartConfigFileStage2", "partCondition.getConditions("..serialize(filename)..")")
    end
  else
    -- TODO: CHANGE THIS LATER TO FORMAT 4
    savePartConfigFileStage2_Format2(nil, filename)
  end
end

local function saveLocal(fn)
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  savePartConfigFile(playerVehicle.vehicleDirectory .. fn)
end

local function saveLocalScreenshot(fn)
  -- See ui/modules/vehicleconfig/vehicleconfig.js (line 420)
  -- Set up camera
  commands.setFreeCamera()
  core_camera.setFOV(0, 35)
  -- Stage 1 happens on JS side for timing reasons
  guihooks.trigger("saveLocalScreenshot_stage1", {})
end

-- Stage 2
local function saveLocalScreenshot_stage2(fn)
  -- Take screenshot
  local playerVehicle = vehManager.getPlayerVehicleData()
  local screenshotName = (playerVehicle.vehicleDirectory .. fn)
  screenshot.doScreenshot(nil, nil, screenshotName, 'jpg')
  -- Stage 3 on JS side
  guihooks.trigger('saveLocalScreenshot_stage3', {})
end


local function savedefault()
  guihooks.trigger('Message', {ttl = 5, msg = 'New default vehicle has been set', icon = 'directions_car'})
  savePartConfigFile('settings/default.pc')
end

local function buildRichPartInfo(ioCtx)
  local availableParts = jbeamIO.getAvailableParts(ioCtx)
  local res = {}
  -- enrich the data a bit for the UI
  for partName, uiPartInfo in pairs(availableParts) do
    local richPartInfo = {}
    richPartInfo.information = deepcopy(uiPartInfo or {})
    if uiPartInfo.modName then
      local mod = core_modmanager.getModDB(uiPartInfo.modName)
      if mod and mod.modData then
        richPartInfo.modTagLine    = mod.modData.tag_line
        richPartInfo.modTitle      = mod.modData.title
        richPartInfo.modLastUpdate = mod.modData.last_update
      end
    end
    res[partName] = richPartInfo
  end
  return res
end

local function sendDataToUI()
  local playerVehID = be:getPlayerVehicleID(0)
  if playerVehID == -1 then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  local vehObj, vehData, vehID, partsData = getVehData(playerVehID)

  local pcFilename = vehData.config.partConfigFilename
  local configDefaults = nil
  if pcFilename then
    local data = buildConfigFromString(vehData.vehicleDirectory, pcFilename)
    if data ~= nil then
      configDefaults = data
      configDefaults.parts = configDefaults.parts or {}
      configDefaults.vars = configDefaults.vars or {}
    end
  end
  if configDefaults == nil then
    configDefaults = {parts = {}, vars = {}}
  end

  local data = {
    vehID                = vehID,
    mainPartName         = vehData.mainPartName,
    chosenPartsTree      = vehData.config.partsTree,
    variables            = vehData.vdata.variables,
    defaults             = configDefaults,
    partsHighlighted     = partsData.partsHighlighted,
  }

  data.richPartInfo = buildRichPartInfo(vehData.ioCtx)

  --dump{'UI part info = ', data}

  guihooks.trigger("VehicleConfigChange", data)
end

local function hasAvailablePart(partName)
  if not partName or partName == "" then return end
  local playerVehicleData = core_vehicle_manager.getPlayerVehicleData()
  local parts = jbeamIO.getAvailableParts(playerVehicleData.ioCtx)

  if parts[partName] then
    return true
  end

  return false
end

local function setSkin(skin)
  local vehicle = getPlayerVehicle(0)
  local playerVehicleData = core_vehicle_manager.getPlayerVehicleData()

  if not vehicle or not playerVehicleData then return end

  local partName = nil

  if skin and skin ~= "" then
    partName = vehicle.JBeam .. "_skin_" .. skin
    local parts = jbeamIO.getAvailableParts(playerVehicleData.ioCtx)
    if not parts[partName] then return end
  end

  local carConfigToLoad = playerVehicleData.config
  if carConfigToLoad.partsTree.children and carConfigToLoad.partsTree.children["paint_design"] then
    local paintDesignSlot = carConfigToLoad.partsTree.children["paint_design"]
    paintDesignSlot.chosenPartName = partName

    local carModelToLoad = vehicle.JBeam
    local vehicleData = {}
    vehicleData.config = carConfigToLoad
    core_vehicles.replaceVehicle(carModelToLoad, vehicleData)
  else
    log('E', 'setSkin', '"paint_design" slot not found in main part')
  end
end

local function reset()
  sendDataToUI()
end

local function mergeConfig(inData, respawn)
  --dump{"mergeConfig> ", inData, respawn}
  local veh = getPlayerVehicle(0)
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not veh or not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end

  if respawn == nil then respawn = true end -- respawn is required all the time except when loading the vehicle

  if not inData or type(inData) ~= 'table' then
    log('W', "partmgmt.mergeConfig", "invalid argument [" .. type(inData) .. '] = '..dumps(inData))
    return
  end

  tableMerge(playerVehicle.config, inData)

  if respawn then
    --dump{"RESPAWN: ", playerVehicle.config}
    veh:respawn(serialize(playerVehicle.config))
  else
    local paintCount = tableSize(inData.paints)
    for i = 1, paintCount do
      vehManager.liveUpdateVehicleColors(veh:getId(), veh, i, inData.paints[i])
    end
    veh:setField('partConfig', '', serialize(playerVehicle.config))
  end
end

local function setConfigPaints (data, respawn)
  mergeConfig({paints = data}, respawn)
end

local function setConfigVars (data, respawn)
  mergeConfig({vars = data}, respawn)
end

local function setPartsConfig (data, respawn)
  log('E', 'partmgmt', 'please use the new function setPartsTreeConfig instead')
end

local function setPartsTreeConfig (dataTree, respawn)
  mergeConfig({partsTree = dataTree}, respawn)
end

local function getConfig()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  return playerVehicle.config
end

local function loadLocal(filename, respawn)
  local veh = getPlayerVehicle(0)
  if not veh then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  core_vehicles.replaceVehicle(veh.JBeam, {config = filename})
end

local function removeLocal(filename)
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  FS:removeFile(playerVehicle.vehicleDirectory .. filename .. ".pc")
  FS:removeFile(playerVehicle.vehicleDirectory .. filename .. ".jpg") -- remove generated thumbnail
  guihooks.trigger("VehicleconfigRemoved", {})
  log('I', 'partmgmt', "deleted user configuration: " .. playerVehicle.vehicleDirectory .. filename .. ".pc")
end

local function isOfficialConfig(filename)
  local isOfficial
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  isOfficial = isOfficialContentVPath(playerVehicle.vehicleDirectory .. filename)
  return isOfficial
end

local function isPlayerConfig(filename)
  local isPlayerConfig
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  isPlayerConfig = isPlayerVehConfig(playerVehicle.vehicleDirectory .. filename)
  return isPlayerConfig
end


local function getConfigList()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end

  local files = FS:findFiles(playerVehicle.vehicleDirectory, "*.pc", -1, true, false) or {}
  local result = {}

  for _, file in pairs(files) do
    local basename = string.sub(file, string.len(playerVehicle.vehicleDirectory) + 1, -1)
    table.insert(result,
    {
      fileName = basename,
      name = string.sub(basename,0, -4),
      official = isOfficialConfig(basename),
      player = isPlayerConfig(basename)
    })
  end
  return result
end

local function openConfigFolderInExplorer()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end

  if not fileExistsOrNil(playerVehicle.vehicleDirectory) then  -- create dir if it doesnt exist
    FS:directoryCreate(playerVehicle.vehicleDirectory, true)
  end
   Engine.Platform.exploreFolder(playerVehicle.vehicleDirectory)
end

-- Actually sets transparency of meshes related to parts
local function setPartsMeshesAlpha(vehObj, vdata, partNames, alpha, notSelectedAlpha)
  vehObj:setMeshAlpha(notSelectedAlpha or 0, "", false)

  if vdata.flexbodies then
    for _, flexbody in pairs(vdata.flexbodies) do
      if flexbody.mesh and flexbody.mesh ~= 'SPOTLIGHT' and flexbody.mesh ~= 'POINTLIGHT' and flexbody.meshLoaded then
        if partNames[flexbody.partPath] then
          if not vehObj:setMeshAlpha(alpha, flexbody.mesh, false) then
            log('W', 'mesh', 'unable to set mesh alpha: ' ..  dumps{'mesh: ', flexbody.mesh, 'alpha: ', alpha, 'existing alpha: ', vehObj:getMeshAlpha(flexbody.mesh)})
          end
        else
          --log('W', '', 'part not highlighted: ' .. tostring(flexbody.partPath))
        end
      end
    end
  end
  if vdata.props then
    for _, prop in pairs(vdata.props) do
      if prop.mesh and partNames[prop.partPath] then
        vehObj:setMeshAlpha(alpha, prop.mesh, false)
      end
    end
  end
  vehObj:queueLuaCommand(string.format('bdebug.setPartsSelected(%s)', serialize(partNames)))
end

-- Sets transparency of highlighted parts
-- If inVehID is nil, it uses player vehicle
local function setHighlightedPartsVisiblity(alpha, inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  partsData.alpha = alpha

  setPartsMeshesAlpha(vehObj, vehData.vdata, partsData.partsHighlighted, alpha)
end

-- Changes transparency of highlighted parts by delta value
-- If inVehID is nil, it uses player vehicle
local function changeHighlightedPartsVisiblity(deltaAlpha, inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  partsData.alpha = clamp(partsData.alpha + deltaAlpha, 0, 1)

  setPartsMeshesAlpha(vehObj, vehData.vdata, partsData.partsHighlighted, partsData.alpha)
end

-- Just shows parts highlighted
-- If inVehID is nil, it uses player vehicle
local function showHighlightedParts(inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  setPartsMeshesAlpha(vehObj, vehData.vdata, partsData.partsHighlighted, partsData.alpha)
end

-- Highlighting refers to clicking on the "eye" icon
-- If inVehID is nil, it uses player vehicle
local function highlightParts(parts, inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  table.clear(partsData.partsHighlightedIdxs)

  -- A local helper function to recurse through the 'children'.
  local function highlightNode(node)
    if node.chosenPartName and node.chosenPartName ~= '' then
      if parts[node.partPath] then
        partsData.partsHighlighted[node.partPath] = parts[node.partPath]
        table.insert(partsData.partsHighlightedIdxs, partsData.partNameToIdx[node.partPath])
      else
        partsData.partsHighlighted[node.partPath] = false
      end
    end

    -- Recurse on any further children.
    if node.children then
      for _, childNode in pairs(node.children) do
        highlightNode(childNode)
      end
    end
  end

  highlightNode(vehData.config.partsTree or {})
  setPartsMeshesAlpha(vehObj, vehData.vdata, parts, partsData.alpha)
end

-- selecting refers to hovering over a part in the UI (only temporary)
-- If inVehID is nil, it uses player vehicle
local function selectParts(parts, inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end
  setPartsMeshesAlpha(vehObj, vehData.vdata, parts, partsData.alpha, 0.2)
end

-- Merge old part highlights with new vehicle parts
-- When new part added, its visiblity is set to the parent part visiblity,
-- as well as its children to prevent weirdness
-- If inVehID is nil, it uses player vehicle
local function setNewParts(inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  local partsFlattened = {}
  local newHighlightedParts = {}
  local oldPartsHighlighted = partsData.partsHighlighted

  local function recHighlightNode(node, parentHighlight)
    local highlight = nil
    if node.partPath then
      partsFlattened[node.partPath] = true
      if oldPartsHighlighted then
        if oldPartsHighlighted[node.partPath] ~= nil then
          -- Existing part uses old highlight
          highlight = oldPartsHighlighted[node.partPath]
        else
          -- New part uses parent part highlight
          highlight = parentHighlight
        end
      else
        highlight = parentHighlight
      end
      newHighlightedParts[node.partPath] = highlight
    end
    if node.children then
      for _, childNode in pairs(node.children) do
        recHighlightNode(childNode, highlight)
      end
    end
  end

  recHighlightNode(vehData.config.partsTree or {}, true)

  local partsSorted = tableKeysSorted(partsFlattened)
  partsData.partsHighlighted = newHighlightedParts
  partsData.partsSorted = partsSorted

  table.clear(partsData.partsHighlightedIdxs)
  table.clear(partsData.partNameToIdx)

  for k, partName in ipairs(partsSorted) do
    if newHighlightedParts[partName] then
      table.insert(partsData.partsHighlightedIdxs, k)
    end
    partsData.partNameToIdx[partName] = k
  end
end

local function sendPartsSelectorStateToUI()
  local vehObj, vehData, vehID, partsData = getVehData()
  if not vehObj then return end

  local uiData = {
    vehID = vehID,
    partsSorted = partsData.partsSorted,
    partsHighlightedIdxs = partsData.partsHighlightedIdxs,
  }
  guihooks.trigger("PartsSelectorUpdate", uiData)
end

local function partsSelectorChangedDebounced(state)
  local vehObj, vehData, vehID, partsData = getVehData(state.vehID)
  if not vehObj then return end

  local partsSelected = {}
  for _, idx in ipairs(state.partsHighlightedIdxs) do
    partsSelected[state.partsSorted[idx]] = true
  end
  highlightParts(partsSelected, vehID)
end

local partsSelectorChangedTime = nil
local partsSelectorChangedState = nil

local function partsSelectorChanged(state)
  partsSelectorChangedState = state
  partsSelectorChangedTime = os.clockhp()
end

-- Clears out all highlights data (parts highlighted, mesh transparency, and vehicle)
-- If inVehID is nil, it uses player vehicle
local function resetVehicleHighlights(onlyIfVehChanged, inVehID)
  local vehObj, vehData, vehID, partsData = getVehData(inVehID)
  if not vehObj then return end

  local clear = true

  if onlyIfVehChanged then
    local name = getObjectByID(vehID):getJBeamFilename()
    local oldName = partsData.vehName

    if name == oldName then
      clear = false
    end
  end

  if clear then
    vehsPartsData[vehID] = nil
  end
end

local function resetConfig()
  mergeConfig({parts = {}, vars = {}}, true)
end

local function resetAllToLoadedConfig()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  loadLocal(playerVehicle.config.partConfigFilename)
end

-- Used by Vehicle Config -> Parts "Reset" button to reset parts back to loaded config
local function resetPartsToLoadedConfig()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  local pcFilename = playerVehicle.config.partConfigFilename
  local parts = nil
  if pcFilename then
    local data = buildConfigFromString(playerVehicle.vehicleDirectory, pcFilename)
    if data ~= nil then
      parts = data.parts
    end
  end
  if parts == nil then
    parts = {}
  end
  setPartsConfig(parts, true)
end

local function resetVarsToLoadedConfig()
  local playerVehicle = vehManager.getPlayerVehicleData()
  if not playerVehicle then
    log('E', 'partmgmt', 'no active vehicle')
    return
  end
  local pcFilename = playerVehicle.config.partConfigFilename
  local vars = nil
  if pcFilename then
    local data = buildConfigFromString(playerVehicle.vehicleDirectory, pcFilename)
    if data ~= nil then
      vars = data.vars
    end
  end
  if vars == nil then
    vars = {}
  end
  setConfigVars(vars, true)
end

local function onUpdate(dt)
  if partsSelectorChangedTime then
    if os.clockhp() - partsSelectorChangedTime > 0.1 then
      partsSelectorChangedDebounced(partsSelectorChangedState)
      partsSelectorChangedState = nil
      partsSelectorChangedTime = nil
    end
  end
end

local function onSerialize()
  return {
    vehsPartsData = vehsPartsData,
  }
end

local function onDeserialized(data)
  vehsPartsData = data.vehsPartsData
end

-- public interface
M.save = savePartConfigFile
M.compileVehicleCollection = compileVehicleCollection
M.saveVehicleCollection = saveVehicleCollection

-- TODO: CHANGE THIS LATER TO FORMAT 4
M.savePartConfigFileStage2 = savePartConfigFileStage2_Format2 --savePartConfigFileStage2

M.setHighlightedPartsVisiblity = setHighlightedPartsVisiblity
M.changeHighlightedPartsVisiblity = changeHighlightedPartsVisiblity
M.highlightParts = highlightParts
M.selectParts = selectParts
M.setNewParts = setNewParts
M.showHighlightedParts = showHighlightedParts
M.resetVehicleHighlights = resetVehicleHighlights
M.setConfig = mergeConfig
M.setConfigPaints = setConfigPaints
M.setConfigVars = setConfigVars
M.setPartsConfig = setPartsConfig
M.setPartsTreeConfig = setPartsTreeConfig
M.getConfig = getConfig
M.resetConfig = resetConfig
M.reset = reset
M.sendDataToUI = sendDataToUI
M.sendPartsSelectorStateToUI = sendPartsSelectorStateToUI
M.partsSelectorChanged = partsSelectorChanged
M.vehicleResetted = reset
M.getConfigSource = getConfigSource
M.getConfigList = getConfigList
M.openConfigFolderInExplorer = openConfigFolderInExplorer
M.loadLocal = loadLocal
M.removeLocal = removeLocal
M.resetAllToLoadedConfig = resetAllToLoadedConfig
M.resetPartsToLoadedConfig = resetPartsToLoadedConfig
M.resetVarsToLoadedConfig = resetVarsToLoadedConfig
M.saveLocal = saveLocal
M.saveLocalScreenshot = saveLocalScreenshot
M.saveLocalScreenshot_stage2 = saveLocalScreenshot_stage2
M.savedefault = savedefault
M.hasAvailablePart = hasAvailablePart
M.setSkin = setSkin
M.onUpdate = onUpdate

M.buildConfigFromString = buildConfigFromString

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M
