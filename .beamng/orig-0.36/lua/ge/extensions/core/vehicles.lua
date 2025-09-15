-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local min, max, random = math.min, math.max, math.random
local pathDefaultConfig = "settings/default.pc"
local vehManager = extensions.core_vehicle_manager

local M = {}

--M.forceLicenceStr = 'BeamNG' -- enables to force a license plate text

M.defaultVehicleModel = string.match(beamng_appname, 'tech') and 'etk800' or 'pickup'

-- coupler tags + a tag for the auto couple function
M.couplerTagsOptions = {
  tow_hitch = "autoCouple",
  fifthwheel = "autoCouple",
  gooseneck_hitch = "autoCouple",
  pintle = "autoCouple",
  fifthwheel_v2 = true
}

M.resetVehCollectionEnabled = true

M.vehCollections = {}
M.vehIdToVehCollection = {}

M.initVehCollections = {}
M.initVehIdToVehCollection = {}

M.attachedCouplers = {}
M.vehsCouplerCache = {}
M.vehsCouplerTags = {}
M.vehsCouplerOffset = {}

local filtersWhiteList = { "Drivetrain", "Type", "Config Type", "Transmission", "Country", "Derby Class", "Performance Class",
 "Value", "Brand", "Body Style", "Source", "Weight", "Top Speed", "0-100 km/h", "0-60 mph", "Weight/Power", "Off-Road Score", "Years", 'Propulsion', 'Fuel Type', 'Induction Type', 'Commercial Class' }

local range = {'Years'}
local beamStats = {}

-- agregates only, attribute stays the same
local convertToRange = { 'Value', 'Weight', 'Top Speed', '0-100 km/h', '0-60 mph', 'Weight/Power', "Off-Road Score" }

-- so the ui knows when to interpret the data as range
local finalRanges = {}
arrayConcat(finalRanges, range)
arrayConcat(finalRanges, convertToRange)

local displayInfo = {
  ranges = {
    all = finalRanges,
    real = range
  },
  units = {
    Weight = {type = 'weight', dec = 0},
    ['Top Speed'] = {type = 'speed', dec = 0},
    ['Torque'] = {type = 'torque', dec = 0},
    ['Power'] = {type = 'power', dec = 0},
    ['Weight/Power'] = {type = 'weightPower', dec = 2},
  },
  predefinedUnits = {
    ['0-60 mph'] = {unit = 's', type = 'speed', ifIs = 'mph', dec = 1},
    ['0-100 mph'] = {unit = 's', type = 'speed', ifIs = 'mph', dec = 1},
    ['0-200 mph'] = {unit = 's', type = 'speed', ifIs = 'mph', dec = 1},
    ['60-100 mph'] = {unit = 's', type = 'speed', ifIs = 'mph', dec = 1},
    ['0-100 km/h'] = {unit = 's', type = 'speed', ifIs = 'km/h', dec = 1},
    ['0-200 km/h'] = {unit = 's', type = 'speed', ifIs = 'km/h', dec = 1},
    ['0-300 km/h'] = {unit = 's', type = 'speed', ifIs = 'km/h', dec = 1},
    ['100-200 km/h'] = {unit = 's', type = 'speed', ifIs = 'km/h', dec = 1},
    ['100-0 km/h'] = {unit = 'm', type = 'length', ifIs = 'm', dec = 1},
    ['60-0 mph'] = {unit = 'ft', type = 'length', ifIs = 'ft', dec = 1}
  },
  dontShowInDetails = { 'Type', 'Config Type' },
  perfData = { '0-60 mph', '0-100 mph', '0-200 mph', '60-100 mph', '60-0 mph', '0-100 km/h', '0-200 km/h', '0-300 km/h', '100-200 km/h', '100-0 km/h', 'Braking G', 'Top Speed', 'Weight/Power', 'Off-Road Score', 'Propulsion', 'Fuel Type', 'Drivetrain', 'Transmission', 'Induction Type' },
  filterData = filtersWhiteList
}

-- TODO: Think about only operating on cache and not cache + local variable in function
local showStandalonePcs = settings.getValue('showStandalonePcs')
local SteamLicensePlateVehicleId
local cache = {}
local anyCacheFileModified = false

local vehsSpawningFlag = {}
local vehsPreventCollectionResetFlag = {}

local function _parseVehicleNameBackwardCompatibility(vehicleName)
  -- try to read the name.cs
  local nameCS = "/vehicles/" .. vehicleName .. "/name.cs"
  local res = { configs = {} }
  local f = io.open(nameCS, "r")
  if f then
    for line in f:lines() do
      local key, value = line:match("^%%(%w+)%s-=%s-\"(.+)\";")
      if key ~= nil and key == "vehicleName" and value ~= nil then
        res.Name = value
      end
    end
    f:close()
  end

  -- get .pc files and fix them up for the new system
  local pcfiles = FS:findFiles("/vehicles/" .. vehicleName .. "/", "*.pc", 0, true, false)
  for _, fn in pairs(pcfiles) do
    local dir, filename, ext = path.split(fn)
    if dir and filename and ext and string.lower(ext) == "pc" then
      local pcfn = filename:sub(1, #filename - 3)
      res.configs[pcfn] = { Configuration = pcfn}
    end
  end

  -- no name fallback
  if res.Name == nil then
    res.Name = vehicleName
  end

  -- type fallback
  res.Type = "Car"
  return res
end

local function _fillAggregates(data, destination)
  for key, value in pairs(data) do
    if tableContains(range, key) then
      if not destination[key] then
        destination[key] = deepcopy(data[key])
      else
        destination[key].min = min(data[key].min, destination[key].min)
        destination[key].max = max(data[key].max, destination[key].max)
      end
    elseif tableContains(convertToRange, key) then
      if type(data[key]) == 'number' then
        if not destination[key] then
          destination[key] = {min = data[key], max = data[key]}
        end
        destination[key].min = min(data[key], destination[key].min)
        destination[key].max = max(data[key], destination[key].max)
      end
    elseif tableContains(filtersWhiteList, key) then
      if not destination[key] then
        destination[key] = {}
      end

      destination[key][value] = true
    end
  end
end

local function _mergeAggregates(data, destination)
  for key, value in pairs(data) do
    if tableContains(range, key) or tableContains(convertToRange, key) then
      if not destination[key] then
        destination[key] = deepcopy(data[key])
      else
        destination[key].min = min(data[key].min, destination[key].min)
        destination[key].max = max(data[key].max, destination[key].max)
      end
    elseif tableContains(filtersWhiteList, key) then
      if not destination[key] then
        destination[key] = deepcopy(data[key])
      else
        for key2, _ in pairs(value) do
          destination[key][key2] = true
        end
      end
    end
  end
end

-- gets all files related to vehicle info
local profilerEnabled = true
local p
local function computeFilesCaches()
  local jfiles = FS:findFiles("/vehicles/", "info*.json\t*.pc\t*.png\t*.jpg", -1, true, true)
  if p then p:add("find") end
  local filesJson, filesPC, filesImages, filesParsed = {}, {}, {}, {}
  for _, filename in ipairs(jfiles) do
    if shouldHideUNRELIABLE("/vehicles/", filename) then
      --log('W', '', 'Skipping hidden file: '.. filename)
    else
      if string.lower(filename:sub(-5)) == '.json' then
        table.insert(filesJson, filename)
      elseif string.lower(filename:sub(-3)) == '.pc' then
        table.insert(filesPC, filename)
      elseif string.lower(filename:sub(-4)) == '.png' or string.lower(filename:sub(-4)) == '.jpg' then
        filesImages[filename] = true
      else
        log('E', 'vehicles', 'Bug in vehicles.lua code (unrecognized file: '.. filename..')')
      end
    end
  end
  if p then p:add("classify") end
  for _, fn in ipairs(filesJson) do
    local data = readFile(fn)
    if p then p:add("json read") end
    if data then
      data = jsonDecode(data)
      if p then p:add("json decode") end
      if not data then
        log('E', 'vehicles', 'unable to read info file, ignoring: '.. fn)
      else
        filesParsed[fn] = data
      end
    else
      log('E', 'vehicles', 'unable to read file, ignoring: '.. fn)
    end
    if p then p:add("json end") end
  end
  for _, fn in ipairs(filesPC) do
    local data = readFile(fn)
    if p then p:add("pc read") end
    if data then
      data = jsonDecode(data)
      if p then p:add("pc decode") end
      if not data then
        log('E', 'vehicles', 'unable to read PC file, ignoring: '.. fn)
      else
        filesParsed[fn] = data
      end
    else
      log('E', 'vehicles', 'unable to read file, ignoring: '.. fn)
    end
    if p then p:add("pc end") end
  end
  return filesJson, filesPC, filesImages, filesParsed
end

local filesJsonCache, filesPCCache, filesImagesCache, filesParsedCache
local function getFilesJson()
  if not filesJsonCache then filesJsonCache, filesPCCache, filesImagesCache, filesParsedCache = computeFilesCaches() end
  return filesJsonCache
end

local function getFilesPC()
  if not filesPCCache then filesJsonCache, filesPCCache, filesImagesCache, filesParsedCache = computeFilesCaches() end
  return filesPCCache
end

local function getFilesImages()
  if not filesImagesCache then filesJsonCache, filesPCCache, filesImagesCache, filesParsedCache = computeFilesCaches() end
  return filesImagesCache
end

local function getFilesParsed()
  if not filesParsedCache then filesJsonCache, filesPCCache, filesImagesCache, filesParsedCache = computeFilesCaches() end
  return filesParsedCache
end

-- returns all found model names
local modelsDataCache
local function getModelsData()
  if not modelsDataCache then
    local modelsData = {}

    local modelRegex  = "^/vehicles/([%w|_|%-|%s]+)/info[_]?(.*)%.json"
    local modelRegexPC  = "^/vehicles/([%w|_|%-|%s]+)/(.*)%.pc"
    local modelRegexDir  = "^/vehicles/([%w|_|%-|%s]+)"

    -- Get the models. They are the directories one level under vehicles folder
    for _, path in ipairs(getFilesJson()) do
      -- name of a file or directory can have alphanumerics, hyphens and underscores.
      local model, configName = string.match(path, modelRegex)
      if model then
        if not modelsData[model] then modelsData[model] = { info = {}, configs = {}} end
        modelsData[model]['info'][path] = configName
      else
        log("E", "", string.format("Cannot parse path %s with regex %s. Can be caused by uncommon characters, subfolders...", dumps(path), dumps(modelRegex)))
      end
    end

    for _, path in ipairs(getFilesPC()) do
      local model, configName = string.match(path, modelRegexPC)
      if model then
        if not modelsData[model] then
          if showStandalonePcs then
            modelsData[model] = { info = {}, configs = {}}
          else
            log('W', '', 'standalone pc file without info file ignored: ' .. tostring(path))
          end
        end
        if modelsData[model] then
          local addIt = true
          if not showStandalonePcs then
            local infoFilename = "/vehicles/" .. model .. "/info_" .. configName .. ".json"
            if not modelsData[model]['info'][infoFilename] then
              --log('W', '', 'Vehicle config does not have an info file: ' .. tostring(path) .. '. Ignoring the file.')
              addIt = false
            end
          end
          if addIt then
            modelsData[model]['configs'][path] = configName
          end
        end
      else
        log("E", "", string.format("Cannot parse path %s with regex %s. Can be caused by uncommon characters, subfolders...", dumps(path), dumps(modelRegexPC)))
      end
    end

    -- find any vehicles without configurations
    local vehicleDirs = FS:directoryList('/vehicles/', false, true)
    for _, path in ipairs(vehicleDirs) do
      if shouldHideUNRELIABLE("/vehicles/", path) then
        --log("W", "", "Skipping hidden vehicleDir: "..dumps(path))
        goto continue
      end
      local model = string.match(path, modelRegexDir)
      if not model then
        log("E", "", string.format("Cannot parse path %s with regex %s. Can be caused by uncommon characters, subfolders...", dumps(path), dumps(modelRegexDir)))
        goto continue
      end
      if model == "common" then
        goto continue
      end
      if modelsData[model] then
        goto continue
      end
      -- ok, we don't know about this vehicle, lets figure out if there are jbeam files in there
      local jbeamFiles = FS:findFiles(path .. '/', "*.jbeam", 0, false, false)
      if #jbeamFiles == 0 then
        log('W', '', 'Warning: vehicle folder does not contain any jbeam files: ' .. tostring(path) .. '. Ignored.')
        goto continue
      end
      -- ok, look if the mainpart is somewhere in there ...
      local mainPartFound = false
      for _, fn in ipairs(jbeamFiles) do
        local fileData = jsonReadFile(fn)
        if fileData then
          for partName, part in pairs(fileData) do
            if part.slotType == 'main' then
              mainPartFound = true
              break
            end
          end
          if mainPartFound then break end
        end
      end
      if not mainPartFound then
        log('W', '', 'Warning: vehicle folder does not contain a configuration or a valid main part: ' .. tostring(path))
        goto continue
      end
      log('W', '', 'Warning: vehicle folder containing main part but no info or config: ' .. tostring(path))
      -- adding the model anyways so the default configuration is spawn-able
      modelsData[model] = { info = {}, configs = {}}
      ::continue::
    end
    modelsDataCache = modelsData
  end
  return modelsDataCache
end

local _cachedGamePath = FS:getGamePath()
local function _isOfficialContentVPathFast(vpath)
  return string.startswith(FS:getFileRealPath(vpath), _cachedGamePath)
end

local function getSourceAttr(path)
  if _isOfficialContentVPathFast(path) then
    return 'BeamNG - Official'
  elseif string.sub(path, -3) == '.pc' then
    return 'Custom'
  else
    return 'Mod'
  end
end

local function convertVehicleInfo(info)
  -- log('I', 'convert', 'Convert vehicle info: '..dump(info))
  local color = {x=0,y=0,z=0,w=0}
  local metallicData = {}
  local paints = {}
  for name, data in pairs(info.colors or {}) do
    if type(data) == 'string' then
      local colorTable = stringToTable(data)
      color.x = tonumber(colorTable[1])
      color.y = tonumber(colorTable[2])
      color.z = tonumber(colorTable[3])
      color.w = tonumber(colorTable[4])
      metallicData[1] = tonumber(colorTable[5])
      metallicData[2] = tonumber(colorTable[6])
      metallicData[3] = tonumber(colorTable[7])
      metallicData[4] = tonumber(colorTable[8])
      -- log('I','convert', name..' colorTable: '..dumps(colorTable)..' color: '..dumps(color)..' metallicData: '..dumps(metallicData))
      local paint = createVehiclePaint(color, metallicData)
      paints[name] = paint
    end
  end
  if not tableIsEmpty(paints) then
    info.paints = paints
    info.colors = nil
  end
  info.defaultPaintName1 = info.default_color
  info.default_color = nil

  info.defaultPaintName2 = info.default_color_2
  info.default_color_2 = nil

  info.defaultPaintName3 = info.default_color_3
  info.default_color_3 = nil
  return info
end

local function  _imageExistsDefault(...)
  for _, path in ipairs({...}) do
    if getFilesImages()[path] then
      return path
    end
  end
  return '/ui/images/appDefault.png'
end

local function _modelConfigsHelper(key, model, ignoreCache)
  --dump{'_modelConfigsHelper', key, model}
  if type(key) ~= 'string' then return nil end
  local vehFiles = getModelsData()[key]
  if not vehFiles then
    log('E', '', 'Vehicle not available: ' .. tostring(key))
    return nil
  end

  if cache[key].configs and not ignoreCache then
    return cache[key].configs
  end

  if not cache[key].configs then
    cache[key].configs = {}
  end

  local configs = {}

  for configFilename, configName in pairs(vehFiles.configs) do
    local contentSource = getSourceAttr(configFilename)
    local infoFilename = "/vehicles/" .. key .. "/info_" .. configName .. ".json"
    local readData = {}
    if vehFiles.info[infoFilename] and getFilesParsed()[infoFilename] then
      readData = getFilesParsed()[infoFilename]
    else
      if FS:fileExists(infoFilename) then
        log('E', 'vehicles', 'unable to read info file, ignoring: '.. infoFilename)
      end
      infoFilename = nil
    end

    if not readData and contentSource ~= 'Custom' then
      log('W', 'vehicles', 'unable to find info file: '.. infoFilename)
    end

    if readData.colors or readData.default_color or readData.default_color_2 or readData.default_color_3 then
      convertVehicleInfo(readData)
    end


    local configData = readData
    --configData.infoFilename = infoFilename
    --configData.pcFilename = configFilename

    configData.Source = contentSource

    if model.default_pc == nil then
      model.default_pc = configName
    end

    -- makes life easier
    configData.model_key = key
    configData.key = configName
    configData.aggregates = {}

    if not configData.Configuration then
      configData.Configuration = configName
    end
    configData.Name = model.Name .. " " .. configData.Configuration

    configData.preview = _imageExistsDefault('/vehicles/' .. key .. '/' .. configName .. '.png', '/vehicles/' .. key .. '/' .. configName .. '.jpg')

    if configData.defaultPaintName1 ~= nil and configData.defaultPaintName1 ~= '' then
      if not model.paints then
        model.paints = model.paints or {}
        log('E', 'vehicles', key..':'..configName..': cannot set default paint for model with no paints data.')
      end

      configData.defaultPaint = model.paints[configData.defaultPaintName1]
    end

    if configData.Value then --if we have a value number
      configData.Value = tonumber(configData.Value) or configData.Value --make sure it's actually a NUMBER and not a string
    end

    configData.is_default_config = (configName == model.default_pc)

    if readData then
      _fillAggregates(readData, configData.aggregates)
    end

    --configData.mod, configData.modFingerprint = extensions.core_modmanager.getModFromPath(configFilename, true) -- TODO: FIXME: SUPER SLOW

    configs[configName] = configData
  end

  return configs
end

-- get all info to one model
local function getModel(key)
  if type(key) ~= 'string' then return {} end
  if not getModelsData()[key] then
    log('E', '', 'Vehicle not available: ' .. dumps(key))
    return {}
  end

  if cache[key] then
    return cache[key]
  end

  local infoFilename = "/vehicles/"..key.."/info.json"
  local data = getFilesParsed()[infoFilename]
  if data and (data.colors or data.default_color or data.default_color_2 or data.default_color_3) then
    convertVehicleInfo(data)
  end

  local fixedVehicle = false
  if data == nil then
    data = _parseVehicleNameBackwardCompatibility(key)
    fixedVehicle = true
  end

  -- Patch up old vehicles for new System
  local missingInfoConfigs = nil
  if data.configs then
    missingInfoConfigs = data.configs
    for mConfigName, mConfig in pairs(missingInfoConfigs) do
      mConfig.is_default_config = false
      if not data.default_pc then
        data.default_pc = mConfigName
        mConfig.is_default_config = true
      end
      mConfig.aggregates = {}
      mConfig.Configuration = mConfigName
      mConfig.Name = data.Name .. ' ' .. mConfigName
      mConfig.key = mConfigName
      mConfig.model_key = key
      mConfig.preview = _imageExistsDefault('/vehicles/' .. key .. '/' .. mConfigName .. '.png', '/vehicles/' .. key .. '/' .. mConfigName .. '.jpg')
    end
    data.configs = nil
  end

  local model = {}
  if data then
    model = deepcopy(data)

    if not data.Type then
      model.Type = "Unknown"
      --log('E', 'vehicles', "model" .. dumps(model) .. "has type \"Unknown\"")
    end

    model.aggregates = {} -- values for filtering
  end

  -- get preview if it exists
  model.preview = _imageExistsDefault('/vehicles/' .. key .. '/default.png', '/vehicles/' .. key .. '/default.jpg')

  --model.infoFilename = infoFilename
  --model.pcFilename = ''

  model.logo = _imageExistsDefault('/vehicles/' .. key .. '/logo.png', '/vehicles/' .. key .. '/logo.jpg')

  if model.defaultPaintName1 then
    if model.paints then
      model.defaultPaint = model.paints[model.defaultPaintName1]
    end
  else
    model.defaultPaint = {}
    model.defaultPaintName1 = ""
  end

  model.key = key -- redundant but makes life easy

  -- figure out the mod this belongs to
  --model.mod, model.modFingerprint = extensions.core_modmanager.getModFromPath(infoFilename, true) -- TODO: FIXME: SUPER SLOW

  cache[key] = {}
  cache[key].model = model

  cache[key].configs = missingInfoConfigs or _modelConfigsHelper(key, model, ignoreCache)

  if cache[key].configs and tableSize(cache[key].configs) < 1 then
    cache[key].configs[key] = deepcopy(model)
    if cache[key].configs[key].model_key == nil then
      cache[key].configs[key].model_key = cache[key].configs[key].key
    end
    if cache[key].model.default_pc == nil then
      cache[key].model.default_pc = key
    end
  end

  if data then
    data.Source = getSourceAttr(infoFilename)

    if fixedVehicle then
      data.Source = 'Mod'
    end

    _fillAggregates(data, model.aggregates)
  end

  -- all configs should have the same aggregates as the base model
  -- the model should have all aggregates of the configs
  local aggHelper = {}
  for _, config in pairs(cache[key].configs) do
    _mergeAggregates(config.aggregates, aggHelper)
    -- I remember removing this in rev 40591 but i cannot tell why that was. The only difference now is that the merge function is "fixed" and never should overwrite values
    _mergeAggregates(cache[key].model.aggregates, config.aggregates)
  end
  _mergeAggregates(aggHelper, cache[key].model.aggregates)

  return cache[key]
end

-- returns the key of the current vehicle (of player one)
-- one could also use: playerVehicle:getJBeamFilename()
local function getVehicleDetails(id)
  local res = {}
  local vehicle = getObjectByID(id)
  if vehicle then
    res.key       = vehicle.JBeam
    res.pc_file   = vehicle.partConfig
    res.position  = vehicle:getPosition()
    res.color     = vehicle.color
  end

  local model = res.key and getModel(res.key) or {}
  local config = {}
  local default = res.pc_file == pathDefaultConfig
  if res.pc_file ~= nil then
    res.config_key = string.match(res.pc_file, "vehicles/".. res.key .."/(.*).pc")
    config = model.configs[res.config_key] or model.model
  end
  return {current = res, model = model.model, configs = config, userDefault = default}
end

-- returns the key of the current vehicle (of player one)
-- one could also use: playerVehicle:getJBeamFilename()
local function getCurrentVehicleDetails()
  return getVehicleDetails(be:getPlayerVehicleID(0))
end


local function createFilters(list)
  local filter = {}

  if list then
    for _, value in pairs(list) do
      for propName, propVal in pairs(value.aggregates) do
        if tableContains(finalRanges, propName) then
          if filter[propName] then
            filter[propName].min = min(value.aggregates[propName].min, filter[propName].min)
            filter[propName].max = max(value.aggregates[propName].max, filter[propName].max)
          else
            filter[propName] = deepcopy(value.aggregates[propName])
          end
        else
          if not filter[propName] then
            filter[propName] = {}
          end
          for key,_ in pairs(propVal) do
            filter[propName][key .. ''] = true
          end
        end
      end
    end
  end

  return filter
end

-- get the list of all available models
local function getModelList(array)
  local models = {}
  for modelName, _ in pairs(getModelsData()) do
    local model = getModel(modelName)
    if array then
      table.insert(models, model.model)
    else
      models[model.model.key] = model.model
    end
  end
  return {models = models, filters = createFilters(models), displayInfo = displayInfo}
end

-- get the list of all available configurations
local function getConfigList(array)
  local configList = {}
  for modelName, _ in pairs(getModelsData()) do
    local model = getModel(modelName)
    -- dump(model.configs)
    if model.configs and not tableIsEmpty(model.configs) then
      for _, config in pairs(model.configs) do
        if array then
          table.insert(configList, config)
        else
          -- dump(config)
          configList[config.model_key .. '_' .. config.key] = config
        end
      end
    end
  end
  return {configs = configList, filters = createFilters(configList), displayInfo = displayInfo}
end

local function getConfig(modelName, configKey)
  local model = getModel(modelName)
  if not model.configs then return end
  for _, config in pairs(model.configs) do
    if configKey == config.key then
      return config
    end
  end
end

local function openSelectorUI()
  if getMissionFilename() == '' then
    -- no vehicle selector in main menu
    return
  end
  log("I", "", "Vehicle selector triggered")
  if profilerEnabled then p = LuaProfiler("Vehicle Selector menu (triggered by a binding)") end
  if p then p:start() end
  guihooks.trigger('MenuOpenModule','vehicleselect')
  if p then p:add("guihook trigger") end
end

-- get the list of all available vehicles for ui
local function notifyUI()
  if p then
    p:add("CEF response to guihook")
  else
    if profilerEnabled then p = LuaProfiler("Vehicle Selector menu WITHOUT profiling the first CEF stages (because the menu was triggered programmatically, not through binding)") end
    if p then p:start() end
  end
  local modelList, configList = {}, {}
  for modelName, _ in pairs(getModelsData()) do
    if p then p:add("model begin") end
    local model = getModel(modelName)
    if p then p:add("model get") end
    table.insert(modelList, model.model)
    if p then p:add("model insert") end
    for _, config in pairs(model.configs or {}) do
      table.insert(configList, config)
    end
    if p then p:add("model configs") end
  end

  if career_career.isActive() and gameplay_garageMode.getGarageMenuState() == "myCars" then
    local vehicles = career_modules_inventory.getVehicles()
    configList = {}
    for id, vehicle in pairs(vehicles) do
      -- configList
      local configInfo = {}
      configInfo.Name = id .. " - " .. (vehicle.niceName or vehicle.model)
      configInfo.model_key = vehicle.model
      configInfo.preview = "/vehicles/" .. vehicle.model .. "/default.jpg"
      configInfo.is_default_config = true
      configInfo.key = nil
      configInfo.aggregates = {Source = {
        ["Career"] = true
      },
      Type = {
        Car = true
      }}
      configInfo.Source = "Career"
      configInfo.spawnFunction = "career_modules_inventory.enterVehicle(" .. id .. ")"
      table.insert(configList, configInfo)
    end
  end

  guihooks.trigger('sendVehicleList', {models = modelList, configs = configList, filters = createFilters(modelList), displayInfo = displayInfo})
  if p then p:add("CEF request") end
end

local function notifyUIEnd()
  if p then p:add("CEF side") end
  if p then p:finish() end
  p = nil
end

local function getVehicleList()
  local models = getModelList(true).models
  local vehicles = {}
  for i,m in pairs(models) do
    local vehicle = getModel(m.key)
    if vehicle ~= nil then
      table.insert(vehicles, vehicle)
    end
  end
  return {vehicles = vehicles, filters = createFilters(models)}
end

local function finalizeSpawn(options, _vehicle)
  local firstVehicle = be:getObjectCount() == 0
  if firstVehicle then
    local player = 0
    be:enterNextVehicle(player, 0) -- enter any vehicle
  end

  local vehicle = _vehicle or getPlayerVehicle(0)
  if vehicle then
    if options.licenseText then
      vehicle:setDynDataFieldbyName("licenseText", 0, options.licenseText)
    end

    if options.vehicleName then
      vehicle:setField('name', '', options.vehicleName)
    end

    if be:getObjectCount() > 1 then
      if vehicle:getField('JBeam','0') ~= "unicycle" then
        ui_message("ui.hints.switchVehicle", 10, "spawn")
      end
    end
  end
end

local function resetVehicleCollection(collection)
  collection.resetState = {
    reset = {},
    coupled = {},
  }
end

local function initVehicleCollection()
  local collection = {}
  return collection
end

local function buildInitVehCollectionCache(initCollection, collection)
  local function buildInitVehCollectionCacheRec(initParentVehData, initVehData, parentVehData, vehData)
    local vehId = initVehData.vehId
    initVehData.parent, vehData.parent = parentVehData, parentVehData
    initCollection.vehsData[vehId], collection.vehsData[vehId] = initVehData, vehData
    M.initVehIdToVehCollection[vehId], M.vehIdToVehCollection[vehId] = initCollection, collection

    for childVehId, initChildVehData in pairs(initVehData.children or {}) do
      buildInitVehCollectionCacheRec(initVehData, initChildVehData, vehData, vehData.children[childVehId])
    end
  end
  initCollection.vehsData, collection.vehsData = {}, {}
  local initMainVehData, mainVehData = initCollection.mainVehData, collection.mainVehData
  M.vehCollections[initMainVehData.vehId] = collection
  buildInitVehCollectionCacheRec(nil, initMainVehData, nil, mainVehData)
end

-- Builds the vehicle collection cache for a given collection
-- Sets collection.vehsData and M.vehIdToVehCollection for all vehicles in the collection
-- Warning: This only adds vehicles to M.vehIdToVehCollection. If vehicles are to be removed, you need to manually remove vehicles from M.vehIdToVehCollection first
local function buildVehCollectionCache(collection)
  local function buildVehCollectionCacheRec(parentVehData, vehData)
    local vehId = vehData.vehId
    vehData.parent = parentVehData
    collection.vehsData[vehId] = vehData
    M.vehIdToVehCollection[vehId] = collection

    for childVehId, childVehData in pairs(vehData.children or {}) do
      buildVehCollectionCacheRec(vehData, childVehData)
    end
  end
  collection.vehsData = {}
  buildVehCollectionCacheRec(nil, collection.mainVehData)
end

local function removeVehicleCollection(collection)
  for vehId, vehData in pairs(collection.vehsData) do
    M.vehIdToVehCollection[vehId] = nil
  end
  M.vehCollections[collection.mainVehData.vehId] = nil
end

local function addVehicleToNewCollectionRec(collection, vehData)
  M.vehIdToVehCollection[vehData.vehId] = collection
  collection.vehsData[vehData.vehId] = vehData
  for childVehId, childVehData in pairs(vehData.children or {}) do
    addVehicleToNewCollectionRec(collection, childVehData)
  end
end

local function addVehicleToCollectionViaCoupling(collection, parentVehId, parentNode, vehId, vehNode)
  local oldCollection = M.vehIdToVehCollection[vehId]
  if not oldCollection then return end

  local vehData = oldCollection.vehsData[vehId]

  -- Remove the vehicle from the old collection
  removeVehicleCollection(oldCollection)

  -- Add the vehicle to the new collection
  local parentVehData = collection.vehsData[parentVehId]
  if parentVehData then
    parentVehData.children = parentVehData.children or {}
    parentVehData.children[vehData.vehId] = vehData
    vehData.parent = parentVehData
    vehData.offsetData = {
      type = "coupledNodes",
      parentNode = parentNode,
      node = vehNode,
      offset = {rx = 0, ry = 0, rz = 0},
    }
  end
  addVehicleToNewCollectionRec(collection, vehData)
end

-- Remove the target vehicle from the collection and add the remaining vehicles to new collections
local function removeVehicleFromCollection(targetVehId)
  local collection = M.vehIdToVehCollection[targetVehId]
  if not collection then return end

  local vehData = collection.vehsData[targetVehId]

  -- Remove the target vehicle from the parent vehicle
  collection.vehsData[targetVehId] = nil
  M.vehIdToVehCollection[targetVehId] = nil

  local parentVehData = vehData.parent
  if parentVehData then
    parentVehData.children[targetVehId] = nil
    buildVehCollectionCache(collection)
  else
    removeVehicleCollection(collection)
  end

  -- Add the remaining vehicles to new collections
  for childVehId, childVehData in pairs(vehData.children or {}) do
    local newCollection = initVehicleCollection()
    newCollection.mainVehData = childVehData
    childVehData.parent = nil
    childVehData.offsetData = nil
    M.vehCollections[childVehData.vehId] = newCollection
    buildVehCollectionCache(newCollection)
  end
end

-- Splits the vehicle collection into two separate collections:
-- - The first half of the vehicles before the target vehicle
-- - The target vehicle and its children
local function splitVehicleCollection(targetVehId)
  local function removeVehicleFromCollectionRec(collection, vehData)
    local vehId = vehData.vehId
    collection.vehsData[vehId] = nil
    M.vehIdToVehCollection[vehId] = nil
    for childVehId, childVehData in pairs(vehData.children or {}) do
      removeVehicleFromCollectionRec(collection, childVehData)
    end
  end

  local collection = M.vehIdToVehCollection[targetVehId]
  if not collection then return end

  local vehData = collection.vehsData[targetVehId]
  if not vehData then return end

  local parentVehData = vehData.parent
  if not parentVehData then return end

  -- Remove the child vehicle from the parent vehicle and the whole collection
  parentVehData.children[targetVehId] = nil
  removeVehicleFromCollectionRec(collection, vehData)
  vehData.offsetData = nil

  -- Create a new collection for the child vehicles
  local newCollection = initVehicleCollection()
  newCollection.mainVehData = vehData
  M.vehCollections[vehData.vehId] = newCollection
  buildVehCollectionCache(newCollection)
end

-- Checks if the vehicle collection is fully reset and vehicles are connected to each other
-- If so, it broadcasts the event "onVehicleCollectionReset"
local function checkVehicleCollectionReset(collection)
  if not collection then return false end
  if not collection.resetState then return false end

  local mainVehId = collection.mainVehData.vehId

  local function checkResetRec(collection, currVid)
    -- Vehicle is part of a vehicle collection
    local veh = getObjectByID(currVid)
    if not veh then return false end

    local vehsData = collection.vehsData
    local vehData = vehsData[currVid]

    for childVehId, childVehData in pairs(vehData.children or {}) do
      local childVeh = getObjectByID(childVehId)
      if not childVeh then
        return false
      end
      local offsetData = childVehData.offsetData
      if offsetData.type == "default" then
        return checkResetRec(collection, childVehId)
      elseif offsetData.type == "coupledNodes" then
        -- Check if the child vehicle is connected to the parent vehicle
        local coupledData = collection.resetState.coupled
        if coupledData[currVid] and coupledData[currVid][childVehId] then
          return checkResetRec(collection, childVehId)
        else
          --log('W', 'checkVehicleCollectionReset', "Vehicle " .. currVid .. " and " .. childVehId .. " are not connected to each other")
          return false
        end
      end
    end
    return true
  end

  local res = checkResetRec(collection, mainVehId)
  if res then
    collection.resetState = nil
    log('D', 'checkVehicleCollectionReset', "Vehicle collection " .. collection.mainVehData.vehId .. " reset")
    extensions.hook("onVehicleCollectionReset", collection)
  end
end

local function findAttachedVehicles(vehId)
  local visited = {}
  local connected = {}

  local function search(vehicle)
    visited[vehicle] = true
    for _, cdata in ipairs(M.attachedCouplers) do
      if cdata[1] == vehicle and not visited[cdata[2]] then
        table.insert(connected, cdata[2])
        search(cdata[2])
      elseif cdata[2] == vehicle and not visited[cdata[1]] then
        table.insert(connected, cdata[1])
        search(cdata[1])
      end
    end
  end
  search(vehId)
  return connected
end

local function getNodeByCid(vehicleData, nodeCid)
  for nodeId, node in pairs(vehicleData.vdata.nodes) do
    if node.cid == nodeCid then
      return node
    end
  end
end

local function getNodeFromName(vdata, nodeName)
  if not vdata or not vdata.nodes then return end
  for nodeId, node in pairs(vdata.nodes) do
    if node.name == nodeName then
      return node
    end
  end
end

local function _generateAttachedVehiclesTree(visitedVehs, vehId, parentCouplerData)
  local entry = {vehId = vehId, children = {}}

  local veh = getObjectByID(vehId)
  if not veh then
    return
  end
  local vehData = vehManager.getVehicleData(vehId)

  -- If attached to another vehicle, record the coupled nodes and offset
  if parentCouplerData then
    local parentVehicle = getObjectByID(parentCouplerData.parentVehId)
    if parentVehicle then
      entry.offsetData = {
        type = "coupledNodes",
        parentNode = parentCouplerData.parentNode.name or parentCouplerData.parentNode.cid,
        node = parentCouplerData.node.name or parentCouplerData.node.cid,
      }
    end
  end

  for _, coupler in ipairs(M.attachedCouplers) do
    -- objId1, objId2, obj1nodeId, obj2nodeId
    local childVehId, nodeId, childNodeId = nil, nil, nil

    if coupler[1] == vehId and coupler[2] then
      childVehId, nodeId, childNodeId = coupler[2], coupler[3], coupler[4]
    elseif coupler[2] == vehId and coupler[1] then
      childVehId, nodeId, childNodeId = coupler[1], coupler[4], coupler[3]
    else
      goto continue
    end

    local childVeh = getObjectByID(childVehId)
    if not childVeh then
      goto continue
    end
    local childVehData = vehManager.getVehicleData(childVehId)

    local node = getNodeByCid(vehData, nodeId)
    local childNode = getNodeByCid(childVehData, childNodeId)

    if not node or not childNode then
      goto continue
    end

    if node.couplerTag then
      local couplerData = {parentVehId = vehId, parentNode = node, node = childNode}
      visitedVehs[childVehId] = true
      local childEntry = _generateAttachedVehiclesTree(visitedVehs, childVehId, couplerData)
      if childEntry then
        table.insert(entry.children, childEntry)
      end
    end

    ::continue::
  end

  return entry
end

local function generateAttachedVehiclesTree(vehId)
  local visitedVehs = {}
  local entry = _generateAttachedVehiclesTree(visitedVehs, vehId, nil)
  return entry
end

local function getCouplerOffset(vehId, couplerTag)
  local veh = getObjectByID(vehId)
  if not veh then return end

  local vdata = core_vehicle_manager.getVehicleData(vehId).vdata
  if not vdata.nodes then return end

  local couplerCache = M.vehsCouplerCache[vehId]

  local refPos = vdata.nodes[vdata.refNodes[0].ref].pos
  local couplerOffset = {}
  for _, c in pairs(couplerCache) do
    if c.couplerTag == couplerTag or c.tag == couplerTag or couplerTag == "" or not couplerTag then
      local pos = vdata.nodes[c.cid].pos
      couplerOffset[c.cid] = {x = pos.x - refPos.x, y = pos.y - refPos.y, z = pos.z - refPos.z, couplerTag = c.couplerTag, tag = c.tag}
    end
  end

  return couplerOffset
end

local function positionChildVehicle(vehId, vehTransform, childVehId, childVehOffset)
  local veh = getObjectByID(vehId)
  if not veh then return end
  local veh2 = getObjectByID(childVehId)
  if not veh2 then return end

  local mat = vehTransform * childVehOffset

  veh2:setTransform(mat)
  --veh2:queueLuaCommand('obj:requestReset(RESET_PHYSICS)')
  veh2:resetBrokenFlexMesh()

  return mat
end

local function placeChildVeh(vehId, vehTransform, childVehId, offsetData)
  local rotOffset = offsetData.offset
  local trailerOffset = MatrixF(true)
  local rotOffsetVec = vec3(math.rad(rotOffset.rx or 0), math.rad(rotOffset.ry or 0), math.rad(rotOffset.rz or 0))
  trailerOffset:setFromEuler(rotOffsetVec)

  local vehCouplerOffset = M.vehsCouplerOffset[vehId][offsetData.parentNode]
  local trailerCouplerOffset = M.vehsCouplerOffset[childVehId][offsetData.node]
  local vehCouplerTag = M.vehsCouplerTags[vehId][offsetData.parentNode]

  local veh = getObjectByID(vehId)
  if not veh then return end
  local veh2 = getObjectByID(childVehId)
  if not veh2 then return end

  local mat = spawn.calculateRelativeVehiclePlacement(vehTransform, vehCouplerOffset, trailerCouplerOffset, trailerOffset)

  veh2:setTransform(mat)
  --veh2:queueLuaCommand('obj:requestReset(RESET_PHYSICS)')
  veh2:resetBrokenFlexMesh()

  if M.couplerTagsOptions[vehCouplerTag] == "autoCouple" then
    veh:queueLuaCommand(string.format('beamstate.activateAutoCoupling("%s")', vehCouplerTag))
  end

  return mat
end

local spawnMultipleVehicles -- forward declaration

local function buildVehicleOffsettingDependencyTree(initVehsData)
  local function getFirstVehData(vehData)
    if not vehData.vehId then
      return getFirstVehData(vehData[1])
    end
    return vehData
  end

  -- build the dependency tree
  for id, vehData in ipairs(initVehsData) do
    vehData = getFirstVehData(vehData)
    if vehData then
      local vehId = vehData.vehId
      local offsetData = vehData.offsetData
      if offsetData and (offsetData.type == "default" or offsetData.type == "coupledNodes") then
        local parentVehData = initVehsData[offsetData.parentId]
        if parentVehData then
          local parentVehId = parentVehData.vehId
          parentVehData.children = parentVehData.children or {}
          parentVehData.children[vehId] = vehData

          if offsetData.type == "coupledNodes" then
            local vdata, parentVdata = core_vehicle_manager.getVehicleData(vehId).vdata, core_vehicle_manager.getVehicleData(parentVehId).vdata
            if vdata and parentVdata then
              local node, parentNode = getNodeFromName(vdata, offsetData.node), getNodeFromName(parentVdata, offsetData.parentNode)
              if node and parentNode then
                offsetData.node = node.cid
                offsetData.parentNode = parentNode.cid
              end
            end
          end
          offsetData.parentId = nil
        end
      end
    end
  end
end

local function prepareConfigData(modelName, opts)
  local model = core_vehicles.getModel(modelName)

  if not opts.config then
    opts.config = 'vehicles/' .. modelName .. '/' .. model.model.default_pc .. '.pc'
  elseif type(opts.config) == 'string' and not string.find(opts.config, '.pc') and FS:fileExists('/vehicles/' .. modelName .. '/' .. opts.config .. '.pc') then
    opts.config = 'vehicles/' .. modelName .. '/' .. opts.config .. '.pc'
  end

  if type(opts.config) == 'string' and opts.config ~= "" then
    -- if we provide a pc file: can be a basename or a full path
    local filename = opts.config
    if not FS:fileExists(filename) then
      filename = '/vehicles/' .. tostring(modelName) .. '/' .. tostring(opts.config)
    end
    if not string.endswith(filename, '.pc') then filename = filename .. '.pc' end
    local data = jsonReadFile(filename)
    -- If the pc file format is 2, keep the config as the filename
    if data then
      if data.format == 4 then
        opts.config = data
      end
    end

    --return data

  elseif type(opts.config) == 'table' and opts.config.format == 4 then
    --return
  end

  --return nil
end

local function prepareMultiVehConfig(modelName, config)
  if config.linkedPCFile then
    local data = jsonReadFile(config.linkedPCFile)
    if data and data.format == 4 then
      return true, data
    end
  elseif config.format == 4 then
    return true, config
  end

  return false
end

local function _spawnNewVehicle(modelName, config, state, localOptions, level)
  if level == 1 then
    local isMultiVehConfig, multiVehConfig = prepareMultiVehConfig(modelName, config)
    if isMultiVehConfig then
      -- spawn multi
      local vehs, vehsData = spawnMultipleVehicles(multiVehConfig, state, localOptions, level)
      if vehs then
        return vehs, vehsData
      end
      return nil
    end
  end

  -- spawn single
  if state.initVehCollection.mainVehData then
    if localOptions.autoEnterVehicle == nil then
      localOptions.autoEnterVehicle = false
    end
  else
    if localOptions.autoEnterVehicle == nil then
      localOptions.autoEnterVehicle = true
    end
  end

  localOptions.model = modelName

  if type(config) == 'table' and config.linkedPCFile then
    localOptions.config = config.linkedPCFile
  else
    localOptions.config = config
  end

  local opt = sanitizeVehicleSpawnOptions(modelName, localOptions)
  local veh = spawn.spawnVehicle(modelName, opt.config, opt.pos, opt.rot, opt)
  local vehId = veh:getID()
  finalizeSpawn(opt, veh)
  vehsSpawningFlag[vehId] = true

  local initVehData = {vehId = vehId, offsetData = config.offsetData}

  if not state.initVehCollection.mainVehData then
    state.initVehCollection.mainVehData = initVehData
    M.initVehCollections[vehId] = state.initVehCollection
  end

  return veh, initVehData
end

local function replaceOtherVehicle(model, opt, otherVeh)
  opt.model = model
  opt.licenseText = otherVeh.licenseText -- copy the current license text
  local options = sanitizeVehicleSpawnOptions(model, opt)
  if options.cling == nil then
    options.cling = true
  end
  spawn.setVehicleObject(otherVeh, options)
  finalizeSpawn(options, otherVeh)
  return otherVeh
end

local function _replaceVehicle(modelName, config, state, localOptions, level)
  --dump("replaceVehicle: ", model, opt, otherVeh)
  -- Get the other collection's vehicles

  if level == 1 then
    local isMultiVehConfig, multiVehConfig = prepareMultiVehConfig(modelName, config)
    if isMultiVehConfig then
      -- spawn multi
      local vehs, vehsData = spawnMultipleVehicles(multiVehConfig, state, localOptions, level)
      if vehs then
        return vehs, vehsData
      end
      return nil
    end
  end

  -- spawn single
  local vehId = table.remove(state.vehsToReuse)
  local veh = getObjectByID(vehId or -1)

  -- when no vehicle is spawned, spawn a new one instead
  if not veh then
    return _spawnNewVehicle(modelName, config, state, localOptions, level)
  else -- spawn new vehicle in place and remove other
    local enterVehicle = vehId == be:getPlayerVehicleID(0)

    localOptions.model = modelName

    if type(config) == 'table' and config.linkedPCFile then
      localOptions.config = config.linkedPCFile
    else
      localOptions.config = config
    end

    localOptions.pos = veh:getPosition()
    if localOptions.keepOtherVehRotation then
      localOptions.rot = quat(0,0,1,0) * quat(veh:getRefNodeRotation())
    end
    localOptions.vehicleName = veh:getField('name', '')

    if state.initVehCollection.mainVehData then
      if localOptions.autoEnterVehicle == nil then
        localOptions.autoEnterVehicle = false
      end
    else
      if localOptions.autoEnterVehicle == nil then
        localOptions.autoEnterVehicle = true
      end
    end

    local opt = sanitizeVehicleSpawnOptions(modelName, localOptions)

    veh:setDynDataFieldbyName("autoEnterVehicle", 0, "false")
    veh = replaceOtherVehicle(modelName, opt, veh)
    veh:setDynDataFieldbyName("autoEnterVehicle", 0, "true")
    vehsSpawningFlag[vehId] = true

    local initVehData = {vehId = vehId, offsetData = config.offsetData}

    if not state.initVehCollection.mainVehData then
      state.initVehCollection.mainVehData = initVehData
      M.initVehCollections[vehId] = state.initVehCollection
    end

    if enterVehicle then be:enterVehicle(0, veh) end
    extensions.hook("onVehicleReplaced", vehId)

    return veh, initVehData
  end
end

spawnMultipleVehicles = function (configs, state, localOptions, level)
  local vehs = {}
  local initVehsData = {}

  -- spawn the main vehicle and all its vehicles
  for i, config in ipairs(configs.vehicles) do
    if level > 1 and i > 1 then
      log('E', 'spawnMultipleVehicles', 'Cannot reference a pc file containing more than one vehicle. Only the first vehicle will be spawned.')
      break
    end

    if level > 2 then
      log('E', 'spawnMultipleVehicles', 'Cannot reference a pc file from a pc file from a pc file.')
      break
    end

    local opt, modelName
    if i == 1 then
      opt = localOptions and localOptions[i] or localOptions or {}
    else
      opt = localOptions and localOptions[i] or {}
    end

    if config.model then
      modelName = config.model
      opt.config = config
    elseif config.linkedPCFile then
      modelName = string.match(config.linkedPCFile, 'vehicles/([%w|_|%-|%s]+)')
      opt.config = config.linkedPCFile
    end

    if modelName then
      --config = sanitizeVehicleSpawnOptions(modelName, opt)

      local newVeh, initVehData = _replaceVehicle(modelName, config, state, opt, level + 1)
      if newVeh then
        if type(newVeh) == 'table' then
          for _, veh in ipairs(newVeh) do
            table.insert(vehs, veh)
          end
        else
          table.insert(vehs, newVeh)
        end
        table.insert(initVehsData, initVehData)
      end
    end
  end

  buildVehicleOffsettingDependencyTree(initVehsData)

  return vehs, initVehsData
end

-- called by the UI directly
local function spawnNewVehicle(modelName, opt)
  local state = {
    initVehCollection = initVehicleCollection(),
    vehsToReuse = {},
  }
  resetVehicleCollection(state.initVehCollection)

  -- Get the config data
  opt = deepcopy(opt) or {}
  prepareConfigData(modelName, opt)

  local vehs = _spawnNewVehicle(modelName, opt.config, state, opt, 1)
  local vehCollection = deepcopy(state.initVehCollection)
  buildInitVehCollectionCache(state.initVehCollection, vehCollection)

  if type(vehs) == 'table' then
    return vehs[1], vehs
  else
    return vehs, {vehs}
  end
end

local function removeCurrent()
  local vehicle = getPlayerVehicle(0)
  if vehicle then
    vehicle:delete()
    if be:getEnterableObjectCount() == 0 then
      commands.setFreeCamera() -- reuse current vehicle camera position for free camera, before removing vehicle
    end
  end
end

-- called by the UI directly
local function replaceVehicle(modelName, opt, otherVeh, replaceWholeCollection)
  local state = {
    initVehCollection = initVehicleCollection(),
    vehsToReuse = {},
  }
  resetVehicleCollection(state.initVehCollection)

  local other
  if otherVeh then
    other = otherVeh
  else
    other = getPlayerVehicle(0)
  end

  local otherVehCollection, otherVehId = nil, nil
  if other then
    otherVehId = other:getID()
    otherVehCollection = M.vehIdToVehCollection[otherVehId]
  end

  if replaceWholeCollection then
    -- get all vehicles to reuse from the other vehicle collection
    for vehId, _ in pairs(otherVehCollection.vehsData) do
      table.insert(state.vehsToReuse, vehId)
    end
  else
    table.insert(state.vehsToReuse, otherVehId)
    splitVehicleCollection(otherVehId)
    removeVehicleFromCollection(otherVehId)
  end

  -- Get the config data
  opt = deepcopy(opt) or {}
  prepareConfigData(modelName, opt)

  local vehs = _replaceVehicle(modelName, opt.config, state, opt, 1)

  -- remove remaining vehicles
  for _, vehId in ipairs(state.vehsToReuse) do
    M.vehCollections[vehId] = nil
    M.vehIdToVehCollection[vehId] = nil
    local veh = getObjectByID(vehId)
    if veh then
      veh:delete()
    end
  end

  local vehCollection = deepcopy(state.initVehCollection)
  buildInitVehCollectionCache(state.initVehCollection, vehCollection)

  if type(vehs) == 'table' then
    return vehs[1], vehs
  else
    return vehs, {vehs}
  end
end

local function removeAllExceptCurrent()
  local vid = be:getPlayerVehicleID(0)
  for i = be:getObjectCount()-1, 0, -1 do
    local veh = be:getObject(i)
    if veh:getId() ~= vid then
      veh:delete()
    end
  end
end

local function cloneCurrent()
  local veh = getPlayerVehicle(0)
  if not veh then
    log('E', 'vehicles', 'unable to clone vehicle: player 0 vehicle not found')
    return false
  end

  -- we get the current vehicles parameters and feed it into the spawning function
  local metallicPaintData = veh:getMetallicPaintData()
  local options = {
    model  = veh.JBeam,
    config = veh.partConfig,
    paint  = createVehiclePaint(veh.color, metallicPaintData[1]),
    paint2 = createVehiclePaint(veh.colorPalette0, metallicPaintData[2]),
    paint3 = createVehiclePaint(veh.colorPalette1, metallicPaintData[3])
  }

  spawnNewVehicle(veh.JBeam, options)
end

local function removeAll()
  local vehicle = getPlayerVehicle(0)
  if vehicle then
    commands.setFreeCamera() -- reuse current vehicle camera position for free camera, before removing vehicles
  end
  for i = be:getObjectCount()-1, 0, -1 do
    be:getObject(i):delete()
  end
end

local function clearCache()
  anyCacheFileModified = false
  filesJsonCache = nil
  filesPCCache = nil
  filesImagesCache = nil
  filesParsedCache = nil
  table.clear(cache)
  modelsDataCache = nil
end

-- local function resetToInitState(vid)
--   local collection = deepcopy(M.initVehIdToVehCollection[vid])
--   if collection then
--     local mainVehData = collection.mainVehData
--     if vid == mainVehData.vehId then
--       -- Remove the vehicles from the other collections
--       for otherMainVehId, otherCollection in pairs(M.vehCollections) do
--         for vehId, vehData in pairs(collection.vehsData) do
--           if otherCollection.vehsData[vehId] then
--             removeVehicleFromCollection(vehId)
--           end
--         end
--       end

--       M.vehCollections[vid] = collection
--       buildVehCollectionCache(collection)
--       local veh = getObjectByID(vid)
--       if veh then
--         veh:requestReset()
--         veh:resetBrokenFlexMesh()
--       end
--     end
--   end
-- end

-- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
-- local function resetAllToInitState()
--   M.vehCollections = deepcopy(M.initVehCollections)
--   M.vehIdToVehCollection = deepcopy(M.initVehIdToVehCollection)

--   for mainVehId, vehCollection in pairs(M.vehCollections) do
--     buildVehCollectionCache(vehCollection)
--     local mainVeh = getObjectByID(mainVehId)
--     if mainVeh then
--       mainVeh:requestReset()
--       mainVeh:resetBrokenFlexMesh()
--     end
--   end
-- end

local function onFileChanged(filename, type)
  if string.find(filename, '/vehicles/') == 1 or string.find(filename, '/mods/') == 1 then
    local fLower = string.lower(filename)
    if string.sub(fLower, -5) == '.json'
    or string.sub(fLower, -6) == '.jbeam'
    or string.sub(fLower, -3) == '.pc'
    or string.sub(fLower, -4) == '.jpg'
    or string.sub(fLower, -4) == '.png' then
      anyCacheFileModified = true
    end
  end
end

local function onFileChangedEnd()
  if anyCacheFileModified then
    clearCache()
  end
end

local function onSettingsChanged()
  local showStandalonePcsNew = settings.getValue('showStandalonePcs')
  if showStandalonePcsNew ~= showStandalonePcs  then
    clearCache()
    showStandalonePcs = showStandalonePcsNew
  end
end

local function generateLicenceText(designData,veh)
  local T = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'}
  if not designData then
    --default_gen
    return T[random(1, #T)] .. T[random(1, #T)] .. T[random(1, #T)] ..'-'..random(0, 9)..random(0, 9)..random(0, 9)..random(0, 9)
  else
    if not designData.gen or not designData.gen.pattern then
      return generateLicenceText() --go back to default
    end

    local formattxt = veh:getDynDataFieldbyName("licenseFormats", 0)
    local formats = {}
    if formattxt and string.len(formattxt) >0 then
      formats = jsonDecode(formattxt )
    else
      formats = {"30-15"}
    end
    for _,v in ipairs(formats)do
      if designData.format[v] and designData.format[v].gen then
        if designData.format[v].gen.pattern then
          designData.gen.pattern = designData.format[v].gen.pattern
        end
        if designData.format[v].gen.patternData then
          tableMerge(designData.gen.patternData, designData.format[v].gen.patternData )
        end
      end
    end

    if not designData.gen.patternData then
      designData.gen.patternData = {}
    end

    local strtmp = designData.gen.pattern
    if type(strtmp) == "table" then
      strtmp = strtmp[random(1, #strtmp)]
    end
    designData.gen.patternData.c = function() return T[random(1, #T)] end
    designData.gen.patternData.D = function() return random(1, 9) end
    designData.gen.patternData.d = function() return random(0, 9) end
    designData.gen.patternData.vid = function() if veh then return veh:getId() else return 0 end end
    designData.gen.patternData.vname = function() if veh then return veh:getJBeamFilename() else return "" end end

    for k,fn in pairs(designData.gen.patternData) do
      if type(fn) == "table" then
        local tmpfn = function() return fn[random(1, #fn)]  end --return one random for each use instead of same
        strtmp = string.gsub(strtmp, "%%"..k, tmpfn)
      else
        strtmp = string.gsub(strtmp, "%%"..k, fn)
      end
    end
    strtmp = string.gsub(strtmp, "%%%%", "%%")
    return strtmp

    -- return string.gsub(designData.gen.pattern, "%%([^%%])", designData.gen.patternData)
  end
end

local function makeVehicleLicenseText(veh, designPath)
  if forceLicenceStr then
    return forceLicenceStr
  end

  if FS:fileExists("settings/cloud/forceLicencePlate.txt") then
    local content = readFile("settings/cloud/forceLicencePlate.txt")
    if content ~= nil then
      log("D","makeVehicleLicenseText","forced to used LicencePlate.txt = '"..tostring(content).."'")
      return content
    end
  end

  veh = veh or getPlayerVehicle(0)
  if type(veh) == 'number' then
    veh = getObjectByID(veh)
  end
  if not veh then return '' end

  local txt = veh:getDynDataFieldbyName("licenseText", 0)
  if txt and txt:len() > 0 then
    return txt
  end

  local design = nil
  if designPath then
    design = jsonReadFile(designPath)
  end

  if settings.getValue("useSteamName") and core_gamestate.state.state == "freeroam" and veh.autoEnterVehicle and SteamLicensePlateVehicleId == nil and Steam and Steam.isWorking and Steam.accountLoggedIn then
    SteamLicensePlateVehicleId = veh:getId()
    txt = Steam.playerName
    txt = txt:gsub('"', "'") -- replace " with '
    -- more cleaning up required?
  elseif not design or not design.data or not design.version or (design.version and design.version == 1) then
    txt = generateLicenceText()
  elseif design.version and design.version > 1 then
    txt = generateLicenceText(design.data or nil, veh)
  end

  return txt
end

local function regenerateVehicleLicenseText(veh)
  local generated_txt = ""
  if veh then
    local current_txt = veh:getDynDataFieldbyName("licenseText", 0)
    veh:setDynDataFieldbyName("licenseText", 0, "")
    local designPath = veh:getDynDataFieldbyName("licenseDesign", 0) or ''
    generated_txt = makeVehicleLicenseText(veh, designPath)
    veh:setDynDataFieldbyName("licenseText", 0, current_txt)
  end
  return generated_txt
end

local function getVehicleLicenseText(veh)
  local licenseText = ""
  if veh then
    licenseText = veh:getDynDataFieldbyName("licenseText", 0)
  end
  return licenseText
end

local function isLicensePlateValid(text)
  return not text or not text:find('"')
end

local DEFAULT_DESIGN_PATH = 'vehicles/common/licenseplates/default/licensePlate-default.json'

-- nil values are equal last values
local function setPlateText(txt, vehId, designPath, formats)
  if not isLicensePlateValid(txt) then
    return
  end

  local veh = nil
  if vehId then
    veh = getObjectByID(vehId)
  else
    veh = getPlayerVehicle(0)
  end
  if not veh then return end
  if not formats then
    local formattxt = veh:getDynDataFieldbyName("licenseFormats", 0)
    if formattxt and string.len(formattxt) >0 then
      formats = jsonDecode(formattxt )
    else
      formats = {"30-15"}
    end
  end

  if not designPath then
    designPath = veh:getDynDataFieldbyName("licenseDesign", 0) or ''
  end

  local design
  if designPath and designPath~="" and FS:fileExists(designPath) then
    design = jsonReadFile(designPath)
  end
  -- dump(design)
  if not design or not design.data then
    if designPath:len() > 0 then
      log('E', 'setPlateText', "License plate "..designPath.." not existing")
    end
    local levelName = core_levels.getLevelName(getMissionFilename())
    if levelName then
      -- log('E', 'setPlateText', "levelName = "..tostring(levelName))
      designPath =  'vehicles/common/licenseplates/'..levelName..'/licensePlate-default.json'
      if FS:fileExists(designPath) then
        design = jsonReadFile(designPath)
      end
    end
  end

  if not design or not design.data then
    designPath = DEFAULT_DESIGN_PATH
    design = jsonReadFile(designPath)
  end

  if not txt then
    txt = makeVehicleLicenseText(veh, designPath)
  end

  local currentFormat = veh:getDynDataFieldbyName("licenseFormats", 0)
  local currentDesign = veh:getDynDataFieldbyName("licenseDesign", 0)
  local currentText = veh:getDynDataFieldbyName("licenseText", 0)

  if (currentFormat == formats and designPath == currentDesign and txt == currentText) then return end;

  veh:setDynDataFieldbyName("licenseFormats", 0, jsonEncode(formats))
  veh:setDynDataFieldbyName("licenseDesign", 0, designPath)
  veh:setDynDataFieldbyName("licenseText", 0, txt)

  ----adding licenseplate html generator and characterlayout to Json file

  local skipGen = settings.getValue('SkipGenerateLicencePlate')
                  or veh:getDynDataFieldbyName("licenseNoGen", 0)
                  or headless_mode

  if design then
    local designData = nil
    if design.version == 1 then
      local dtmp = {}; dtmp.data={};dtmp.data.format={}
      dtmp.data.format["30-15"] = design.data
      design = dtmp;
    end
    for _,curFormat in pairs(formats) do
      designData={}; designData.data=design.data.format[curFormat]
      local textureTagPrefix = "@licenseplate-default"
      if curFormat ~= "30-15" then
        textureTagPrefix = string.format("@licenseplate-%s", curFormat)
      end
      if skipGen then
        veh:setTaggedTexture(textureTagPrefix, "/vehicles/common/licenseplates/premade"..textureTagPrefix..".dds")
        veh:setTaggedTexture(textureTagPrefix.."-normal", "/vehicles/common/licenseplates/premade"..textureTagPrefix.."-normal"..".dds")
        veh:setTaggedTexture(textureTagPrefix.."-specular", "/vehicles/common/licenseplates/premade"..textureTagPrefix.."-specular"..".dds")
        goto continue
      end
      if not designData.data then
        if curFormat ~= "52-11" or designPath ~= DEFAULT_DESIGN_PATH then
          log("W", "setPlateText", "license plate format not found '"..tostring(curFormat).."' in style '"..tostring(designPath).."'")
        end
        local defaultDesignFallBackPath = 'vehicles/common/licenseplates/default/licensePlate-default-'..curFormat..'.json'
        if FS:fileExists(defaultDesignFallBackPath) then
          local defaultDesign = jsonReadFile(defaultDesignFallBackPath)
          if defaultDesign then
            designData.data = defaultDesign.data.format[curFormat]
            log("I", "setPlateText", "license plate fallback used '"..tostring(defaultDesignFallBackPath).."'")
          else
            log('E',tostring(defaultDesignFallBackPath) , 'Json error')
            goto continue
          end
        else
          log('E', "setPlateText", '[NO TEXTURE] No fallback for this licence plate format. Please create a default file here : "'..tostring(defaultDesignFallBackPath)..'"')
          goto continue
        end
      end
      if designData.data.characterLayout then
        if FS:fileExists(designData.data.characterLayout) then
          designData.data.characterLayout = jsonReadFile(designData.data.characterLayout)
        else
          log('E',tostring(designData.data.characterLayout) , ' File not existing')
        end
      else
        designData.data.characterLayout= "vehicles/common/licenseplates/default/platefont.json"
        designData.data.characterLayout= jsonReadFile(designData.data.characterLayout)
      end

      if designData.data.generator then
        if FS:fileExists(designData.data.generator) then
          designData.data.generator = "local://local/" .. designData.data.generator
        else
          log('E',tostring(designData.data.generator) , ' File not existing')
        end
      else
        designData.data.generator = "local://local/vehicles/common/licenseplates/default/licenseplate-default.html"
      end

      if TextureDrawPrimitiveRegistry and veh.setTaggedTextureDrawPrim and
      designData.data.generator == "local://local/vehicles/common/licenseplates/default/licenseplate-default.html" then
        designData.data.generator = "extensions/test/texrdrlic2"
      else
        --log("E", "setPlateText", "veh.setTaggedTextureDrawPrim "..dumps(veh.setTaggedTextureDrawPrim))
      end
      if veh.setTaggedTextureDrawPrim then
        log("E", "setPlateText", "designData.data.generator = "..tostring(designData.data.generator))
      end
      designData.data.format = curFormat

      if TextureDrawPrimitiveRegistry and not designData.data.generator:find(".html") then
        for i,val in ipairs(designData.data.characterLayout.chars.char) do
          for key, value in pairs(val) do
            designData.data.characterLayout.chars.char[i][key] = tonumber(value)
          end
        end
        local moduleFound, module = pcall(require, designData.data.generator)
        if not moduleFound then
          log('E',tostring(designData.data.generator) , ' File not existing')
        else
          local vehPrefix = "VehicleTex-"..veh:getId()..textureTagPrefix
          local td = TextureDrawPrimitiveRegistry:getOrCreate(vehPrefix,Point2I(designData.data.size.x, designData.data.size.y), false, ColorF(0,0,0,0))
          veh:setTaggedTextureDrawPrim(textureTagPrefix, td)
          local tn = TextureDrawPrimitiveRegistry:getOrCreate(vehPrefix.."-normal",Point2I(designData.data.size.x, designData.data.size.y), false, ColorF(0,0,0,0))
          veh:setTaggedTextureDrawPrim(textureTagPrefix.."-normal", tn)
          local ts = TextureDrawPrimitiveRegistry:getOrCreate(vehPrefix.."-specular",Point2I(designData.data.size.x, designData.data.size.y), false, ColorF(0,0,0,0))
          veh:setTaggedTextureDrawPrim(textureTagPrefix.."-specular", ts)

          local errHandler = function(err)
            log('E',tostring(designData.data.generator) , ' error '..tostring(err)..'\n'..(debug.traceback(nil, 2)))
            return err
          end
          local ok, err = xpcall(function()
            module.renderLicensePlate(td, designData.data, txt, "diffuse")
          end, errHandler)
          if not ok then
            log('E',tostring(designData.data.generator) , ' error in Diffuse pass\n'..dumps(err))
          end
          ok, err = xpcall(function()
            module.renderLicensePlate(tn, designData.data, txt, "bump")
          end, errHandler)
          if not ok then
            log('E',tostring(designData.data.generator) , ' error in bump pass\n'..dumps(err))
          end
          ok, err = xpcall(function()
            module.renderLicensePlate(ts, designData.data, txt, "specular")
          end, errHandler)
          if not ok then
            log('E',tostring(designData.data.generator) , ' error in specular pass\n'..dumps(err))
          end
          goto continue
        end

      end

      -- log('D', "setPlateText", "cef tex :"..tostring(curFormat).. "   gen="..tostring(designData.data.generator) .. "prefix="..tostring(textureTagPrefix) )
      veh:createUITexture(textureTagPrefix, designData.data.generator, designData.data.size.x, designData.data.size.y, UI_TEXTURE_USAGE_AUTOMATIC, 1) --UI_TEXTURE_USAGE_MANUAL
      veh:queueJSUITexture(textureTagPrefix, 'init("diffuse","' .. txt .. '", '.. jsonEncode(designData) .. ');')

      veh:createUITexture(textureTagPrefix.."-normal", designData.data.generator, designData.data.size.x, designData.data.size.y, UI_TEXTURE_USAGE_AUTOMATIC, 1)
      veh:queueJSUITexture(textureTagPrefix.."-normal", 'init("bump","' .. txt .. '", '.. jsonEncode(designData) .. ');')

      veh:createUITexture(textureTagPrefix.."-specular", designData.data.generator, designData.data.size.x, designData.data.size.y, UI_TEXTURE_USAGE_AUTOMATIC, 1)
      veh:queueJSUITexture(textureTagPrefix.."-specular", 'init("specular","' .. txt .. '", '.. jsonEncode(designData) .. ');')

      ::continue::
    end
  end
  extensions.hook("onLicensePlateChanged", txt, veh:getID(), designPath, formats)
end

local function loadDefaultPickup()
  local modelName = M.defaultVehicleModel
  log('D', 'main', "Loading the default vehicle " .. modelName)

  local vehicleInfo = jsonReadFile('vehicles/' .. modelName .. '/info.json')
  if not vehicleInfo then
    log('E', 'main', "No info.json for default pickup found.")
    return
  end

  if vehicleInfo.colors or vehicleInfo.default_color or vehicleInfo.default_color2 or vehicleInfo.default_color_3 then
    convertVehicleInfo(vehicleInfo)
  end

  local defaultPC = vehicleInfo.default_pc
  local paint = vehicleInfo.paints and vehicleInfo.paints[vehicleInfo.defaultPaintName1]
  paint = validateVehiclePaint(paint)
  local color = string.format("%s %s %s %s", paint.baseColor[1], paint.baseColor[2], paint.baseColor[3], paint.baseColor[4])
  local metallicPaintData = vehicleMetallicPaintString(paint.metallic, paint.roughness, paint.clearcoat, paint.clearcoatRoughness)
  TorqueScriptLua.setVar( '$beamngVehicle', modelName)
  TorqueScriptLua.setVar( '$beamngVehicleColor', color)
  TorqueScriptLua.setVar( '$beamngVehicleMetallicPaintData', metallicPaintData)
  TorqueScriptLua.setVar( '$beamngVehicleConfig', 'vehicles/' .. modelName .. '/' .. defaultPC .. '.pc' )
end

local function loadCustomVehicle(modelName, data)
  TorqueScriptLua.setVar( '$beamngVehicle', modelName)
  TorqueScriptLua.setVar( '$beamngVehicleConfig', data.config)
end

--check if there is default vehicle or not
--if not then use the defaultVehicleModel
local function loadDefaultVehicle()
  log('D', 'main', 'Loading default vehicle')
  local arg = parseArgs.args.vehicleConfig
  if arg then
    local pathConfig = "vehicles/"..arg
    local modelRegexPC  = "^([%w|_|%-|%s]+)/(.*)%.pc"
    local model, config = string.match(arg, modelRegexPC)
    TorqueScriptLua.setVar('$beamngVehicle', model)
    TorqueScriptLua.setVar('$beamngVehicleConfig', pathConfig)
    return
  end
  local myveh = TorqueScriptLua.getVar('$beamngVehicleArgs')
  if myveh ~= ""  then
    TorqueScriptLua.setVar( '$beamngVehicle', myveh )
    local mycolor = getVehicleColor()
    log('I', 'main', 'myColor = '..dumps(mycolor))
    TorqueScriptLua.setVar( '$beamngVehicleColor', mycolor )
    return
  end

  local dir = FS:directoryExists('vehicles')
  if not dir then
    log('E', 'main', '"/vehicles/" directory not found!')
    return
  end

  local data = jsonReadFile(pathDefaultConfig)
  if not data then
    loadDefaultPickup() -- If there is no pathDefaultConfig file, load the default pickup
    return
  end

  if not data.format or data.format == 2 then
    if data.model then
      if next(FS:findFiles('/vehicles/'..data.model..'/', '*.jbeam', 1, false, false)) then
        TorqueScriptLua.setVar( '$beamngVehicle', data.model ) -- Set the model
        TorqueScriptLua.setVar( '$beamngVehicleConfig', pathDefaultConfig ) -- Set the parts and color
        TorqueScriptLua.setVar( '$beamngVehicleLicenseName', data.licenseName and data.licenseName or "") -- Set the license plate
      else
        log('E', 'main', "Model of default vehicle doesn't exist. Loading default pickup.")
        loadDefaultPickup()
      end
    else
      loadDefaultPickup()
    end
  elseif data.format == 4 then
    local allVehDirsExist = false

    if type(data.vehicles) == "table" and next(data.vehicles) then
      local visitedDirs = {}
      allVehDirsExist = true
      for vehIdx, vehData in ipairs(data.vehicles) do
        local modelName
        if vehData.model then
          modelName = vehData.model
        elseif vehData.linkedPCFile then
          modelName = string.match(vehData.linkedPCFile, 'vehicles/([%w|_|%-|%s]+)')
        end

        if not modelName then
          allVehDirsExist = false
          break
        end

        if not visitedDirs[modelName] then
          if not next(FS:findFiles('/vehicles/'..modelName..'/', '*.jbeam', 1, false, false)) then
            allVehDirsExist = false
            break
          end
          visitedDirs[modelName] = true
        end
      end
    else
      log('E', 'main', "No vehicles exist in default vehicle collection. Loading default pickup.")
      loadDefaultPickup()
      return
    end

    if allVehDirsExist then
      local pcData = data.vehicles[1]
      if not pcData then
        log('E', 'main', "No vehicles exist in default vehicle collection. Loading default pickup.")
        loadDefaultPickup()
        return
      end

      local modelName
      if pcData.model then
        modelName = pcData.model
      elseif pcData.linkedPCFile then
        modelName = string.match(pcData.linkedPCFile, 'vehicles/([%w|_|%-|%s]+)')
      end

      if modelName then
        TorqueScriptLua.setVar( '$beamngVehicle', modelName ) -- Set the model
        TorqueScriptLua.setVar( '$beamngVehicleConfig', pathDefaultConfig ) -- Set the parts and color
        --TorqueScriptLua.setVar( '$beamngVehicleLicenseName', data.licenseName and data.licenseName or "") -- Set the license plate
      else
        log('E', 'main', "Model of first vehicle in default vehicle collection couldn't be determined. Loading default pickup.")
        loadDefaultPickup()
      end
    else
      log('E', 'main', "Model(s) of default vehicle collection don't exist. Loading default pickup.")
      loadDefaultPickup()
    end
  end
end

local function loadMaybeVehicle(maybeVehicle)
  if maybeVehicle == nil then
    loadDefaultVehicle()
    return true
  elseif maybeVehicle == false then
    -- do nothing
    return true
  elseif type(maybeVehicle) == "table" then
    loadCustomVehicle(unpack(maybeVehicle))
    return true
  else
    return false
  end
end

local function spawnDefault()
  local vehicle = getPlayerVehicle(0)
  if FS:fileExists(pathDefaultConfig) then
    local data = jsonReadFile(pathDefaultConfig)
    if vehicle then
      replaceVehicle(data.model, {config = pathDefaultConfig, licenseText = data.licenseName})
    else
      spawnNewVehicle(data.model, {config = pathDefaultConfig, licenseText = data.licenseName})
    end
  else
    if vehicle then
      replaceVehicle(M.defaultVehicleModel, {})
    else
      spawnNewVehicle(M.defaultVehicleModel, {})
    end
  end
end

local function onVehicleSpawned(vehId)
  local veh = getObjectByID(vehId)
  if not veh then return end

  M.vehsCouplerCache[vehId], M.vehsCouplerTags[vehId], M.vehsCouplerOffset[vehId] = {}, {}, {}
  local couplerCache, couplerTags, couplerOffset = M.vehsCouplerCache[vehId], M.vehsCouplerTags[vehId], M.vehsCouplerOffset[vehId]

  local vehData = core_vehicle_manager.getVehicleData(vehId)
  local vdata = vehData.vdata

  if not vdata.nodes then return end
  local refPos = vdata.nodes[vdata.refNodes[0].ref].pos

  for _, n in pairs(vdata.nodes) do
    if n.couplerTag or n.tag then
      local cid = n.cid
      couplerTags[cid] = n.couplerTag
      local data = shallowcopy(n)
      couplerCache[cid] = data

      local pos = vdata.nodes[cid].pos
      couplerOffset[cid] = vec3(pos.x - refPos.x, pos.y - refPos.y, pos.z - refPos.z)
    end
  end
end

local function onVehicleDestroyed(vid)
  for i, coupler in ipairs(M.attachedCouplers) do
    if coupler[1] == vid then
      table.remove(M.attachedCouplers, i)
    end
  end

  M.vehsCouplerCache[vid], M.vehsCouplerTags[vid], M.vehsCouplerOffset[vid] = nil, nil, nil

  -- Remove vehicle and children from vehicle collection
  --removeVehicleFromCollection(vid)

  if SteamLicensePlateVehicleId == vid then
    SteamLicensePlateVehicleId = nil
  end
end

local function vehicleCollectionReset(vid)
  local function collectionResetRec(collection, currVid, parentTransform)
    -- Vehicle is part of a vehicle collection
    local veh = getObjectByID(currVid)
    if not veh then return end

    local transform = parentTransform or veh:getRefNodeMatrix()
    local vehData = collection.vehsData[currVid]

    -- Place the children vehicles relative to us
    for childVehId, childVehData in pairs(vehData.children or {}) do
      local childVeh = getObjectByID(childVehId)
      if childVeh then
        local offsetData = childVehData.offsetData
        if offsetData then
          if offsetData.type == "default" then
            local offset = offsetData.offset
            local offsetMat = MatrixF(true)
            offsetMat:setFromEuler(vec3(math.rad(offset.rx or 0), math.rad(offset.ry or 0), math.rad(offset.rz or 0)))
            offsetMat:setPosition(vec3(offset.x or 0, offset.y or 0, offset.z or 0))
            vehsPreventCollectionResetFlag[childVehId] = true
            local childTransform = positionChildVehicle(currVid, transform, childVehId, offsetMat)
            collectionResetRec(collection, childVehId, childTransform)
          elseif offsetData.type == "coupledNodes" then
            vehsPreventCollectionResetFlag[childVehId] = true
            local childTransform = placeChildVeh(currVid, transform, childVehId, offsetData)
            collectionResetRec(collection, childVehId, childTransform)
          end
        end
      end
    end
  end

  local collection = M.vehIdToVehCollection[vid]
  if collection then
    local mainVehData = collection.mainVehData
    if mainVehData.vehId == vid then
      -- main vehicle of collection
      -- TODO: uncomment setGhostEnabled() when its able to work well
      --be:queueObjectFastLua(vid, "obj:setGhostEnabled(true)")
      collectionResetRec(collection, vid)
      --be:queueObjectLua(vid, "obj:setGhostEnabled(false)")
      resetVehicleCollection(collection)
    else
      -- TODO: Implement this
      -- other vehicles in collection
      -- local mainVeh = getObjectByID(collection.mainVehData.vehId)
      -- if mainVeh then
      --   mainVeh:requestReset(RESET_PHYSICS)
      --   mainVeh:resetBrokenFlexMesh()
      -- end
    end
    if collection.resetState then
      collection.resetState.reset[vid] = true
      checkVehicleCollectionReset(collection)
    end
  end
end

-- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
-- local function onVehicleResetted(vid)
--   if M.resetVehCollectionEnabled then
--     if not vehsSpawningFlag[vid] then
--       if vehsPreventCollectionResetFlag[vid] then
--         vehsPreventCollectionResetFlag[vid] = nil
--       else
--         vehicleCollectionReset(vid)
--       end
--     end
--   end
-- end

-- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
-- local function onSetClusterPosRelRot(vehicleID, cNodeId)
--   if M.resetVehCollectionEnabled then
--     if vehsSpawningFlag[vehicleID] then
--       vehicleCollectionReset(vehicleID)
--       vehsSpawningFlag[vehicleID] = nil
--       return
--     end
--   end
-- end

local function onCouplerAttached(objId1, objId2, nodeId, obj2nodeId)
  table.insert(M.attachedCouplers, {objId1, objId2, nodeId, obj2nodeId})

  -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
  -- -- if the two vehicles coupled together are not part of the same vehicle collection, remove both collections
  -- local collection = M.vehIdToVehCollection[objId1]
  -- if collection then
  --   if collection.vehsData[objId2] then
  --     -- The two vehicles are part of the same vehicle collection
  --     if collection.resetState then
  --       -- On resets, record down coupled vehicles
  --       local coupledData = collection.resetState.coupled
  --       coupledData[objId1], coupledData[objId2] = coupledData[objId1] or {}, coupledData[objId2] or {}
  --       coupledData[objId1][objId2], coupledData[objId2][objId1] = true, true
  --       checkVehicleCollectionReset(collection)
  --     end
  --   else
  --     -- The two vehicles are part of different vehicle collections

  --     -- Determine who is the parent and who is the child
  --     local obj1CouplerTag = M.vehsCouplerTags[objId1][nodeId]
  --     local obj2CouplerTag = M.vehsCouplerTags[objId2][obj2nodeId]
  --     local couplerTag = obj1CouplerTag or obj2CouplerTag

  --     local parentVehicleId, childVehicleId, parentNode, vehNode

  --     if obj1CouplerTag then
  --       -- obj1 is the parent, obj2 is the child
  --       parentVehicleId = objId1
  --       childVehicleId = objId2
  --       parentNode = nodeId
  --       vehNode = obj2nodeId
  --     else
  --       -- obj2 is the parent, obj1 is the child
  --       parentVehicleId = objId2
  --       childVehicleId = objId1
  --       parentNode = obj2nodeId
  --       vehNode = nodeId
  --     end

  --     -- Remove the child vehicle collection and add the vehicles to the parent vehicle collection
  --     local parentCollection, childCollection = M.vehIdToVehCollection[parentVehicleId], M.vehIdToVehCollection[childVehicleId]
  --     addVehicleToCollectionViaCoupling(parentCollection, parentVehicleId, parentNode, childVehicleId, vehNode)

  --     -- Remove vehicles attached logically (rather than physically) to the parent vehicle collection with same coupler tag
  --     local parentVehData = parentCollection.vehsData[parentVehicleId]
  --     for vehId, vehData in pairs(parentVehData.children or {}) do
  --       if vehData.offsetData and vehData.offsetData.type == "coupledNodes" then
  --         local vdata = core_vehicle_manager.getVehicleData(vehId).vdata
  --         local childNode = getNodeFromName(vdata, vehData.offsetData.node)
  --         if childNode and childNode.tag == couplerTag then
  --           removeVehicleFromCollection(vehId)
  --         end
  --       end
  --     end
  --   end
  -- end
end

-- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
-- --Trigered when trailer coupler is detached by the user
-- local function onCouplerDetach(objId1, nodeId)
--   -- if the vehicle is part of a vehicle collection, remove the whole collection
--   local collection = M.vehIdToVehCollection[objId1]
--   --removeVehicleCollection(collection.mainVehData.vehId)

--   local vehData = collection.vehsData[objId1]
--   --local couplerTag = M.vehsCouplerTags[objId1][nodeId]

--   for childVehId, childVehData in pairs(vehData.children or {}) do
--     if childVehData.offsetData and childVehData.offsetData.type == "coupledNodes" then
--       -- TODO: We currently assume that the child vehicle is connected to one parent coupler
--       splitVehicleCollection(childVehId)
--     end
--   end
-- end

-- Trigered when trailer coupler is detached in any way
local function onCouplerDetached(obj1id, obj2id, nodeId, obj2nodeId)
  for i, coupler in ipairs(M.attachedCouplers) do
    if coupler[1] == obj1id and coupler[2] == obj2id and coupler[3] == nodeId and coupler[4] == obj2nodeId then
      table.remove(M.attachedCouplers, i)
      break
    end
  end

  -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
  -- local collection = M.vehIdToVehCollection[obj1id]
  -- if collection and collection.resetState then
  --   if collection.resetState.coupled[obj1id] then
  --     collection.resetState.coupled[obj1id][obj2id] = nil
  --   end
  --   if collection.resetState.coupled[obj2id] then
  --     collection.resetState.coupled[obj2id][obj1id] = nil
  --   end
  -- end

  --onCouplerDetach(obj1id, nodeId)
end

local function reloadVehicle(playerId)
  local veh = getPlayerVehicle(playerId)
  if veh and SteamLicensePlateVehicleId == veh:getId() then
    SteamLicensePlateVehicleId = nil
  end
end

-- Called on mesh visiblity change key bindings (don't use elsewhere!)
local function changeMeshVisibility(delta)
  extensions.core_vehicle_partmgmt.changeHighlightedPartsVisiblity(delta)
end

-- Called on vehicle config UI mesh visiblity buttons (don't use elsewhere!)
local function setMeshVisibility(alpha)
  extensions.core_vehicle_partmgmt.setHighlightedPartsVisiblity(alpha)
end

local function onSerialize()
  -- Don't serialize the vehsData list in the vehicle collections
  for mainVehId, initVehCollection in pairs(M.initVehCollections) do
    for vehId, vehData in pairs(initVehCollection.vehsData) do
      vehData.parent = nil
    end
    initVehCollection.vehsData = nil
  end
  for mainVehId, collection in pairs(M.vehCollections) do
    for vehId, vehData in pairs(collection.vehsData) do
      vehData.parent = nil
    end
    collection.vehsData = nil
  end

  local data = {
    attachedCouplers = M.attachedCouplers,
    vehsCouplerCache = M.vehsCouplerCache,
    vehsCouplerTags = M.vehsCouplerTags,
    vehsCouplerOffset = M.vehsCouplerOffset,

    resetVehCollectionEnabled = M.resetVehCollectionEnabled,
    initVehCollections = M.initVehCollections,
    vehCollections = M.vehCollections,
  }
  return data
end

local function onDeserialized(data)
  M.attachedCouplers = data.attachedCouplers
  M.vehsCouplerCache = data.vehsCouplerCache
  M.vehsCouplerTags = data.vehsCouplerTags
  M.vehsCouplerOffset = data.vehsCouplerOffset

  M.resetVehCollectionEnabled = data.resetVehCollectionEnabled
  M.initVehCollections = data.initVehCollections
  M.vehCollections = data.vehCollections

  for mainVehId, initVehCollection in pairs(M.initVehCollections) do
    buildVehCollectionCache(initVehCollection)
  end
  for mainVehId, collection in pairs(M.vehCollections) do
    buildVehCollectionCache(collection)
  end
end

-- Debug UI
-- M.onUpdate = function(dt)
--   local im = ui_imgui

--   local function drawVehicleCollectionRec(vehData)
--     im.Text('  -' .. vehData.vehId)
--     for childVehId, childVehData in pairs(vehData.children or {}) do
--       drawVehicleCollectionRec(childVehData)
--     end
--   end

--   if im.Begin("Vehicle Collections Debug") then
--     for mainVehId, collection in pairs(M.vehCollections) do
--       im.Text('Main Veh ID: ' .. mainVehId)
--       drawVehicleCollectionRec(collection.mainVehData)
--     end
--   end
--   im.End()
-- end

--public interface
M.getCurrentVehicleDetails = getCurrentVehicleDetails
M.getVehicleDetails = getVehicleDetails

M.getModel = getModel
M.openSelectorUI = openSelectorUI
M.requestList = notifyUI
M.requestListEnd = notifyUIEnd
M.getModelList = getModelList
M.getConfigList = getConfigList
M.getConfig = getConfig
M.generateAttachedVehiclesTree = generateAttachedVehiclesTree
M.getCouplerOffset = getCouplerOffset

M.replaceVehicle = replaceVehicle
M.spawnNewVehicle = spawnNewVehicle
M.removeCurrent = removeCurrent
M.cloneCurrent = cloneCurrent
M.removeAll = removeAll
M.removeAllExceptCurrent = removeAllExceptCurrent
M.removeAllWithProperty = removeAllWithProperty
M.clearCache = clearCache
--M.resetToInitState = resetToInitState -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
--M.resetAllToInitState = resetAllToInitState -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled

-- used to delete the cached data
M.onFileChanged = onFileChanged
M.onFileChangedEnd = onFileChangedEnd
M.onSettingsChanged = onSettingsChanged

-- License plate
M.setPlateText = setPlateText
M.getVehicleLicenseText = getVehicleLicenseText
M.makeVehicleLicenseText = makeVehicleLicenseText
M.regenerateVehicleLicenseText = regenerateVehicleLicenseText
M.isLicensePlateValid = isLicensePlateValid

M.reloadVehicle = reloadVehicle

-- Default Vehicle
M.loadDefaultVehicle  = loadDefaultVehicle
M.loadCustomVehicle   = loadCustomVehicle
M.spawnDefault        = spawnDefault
M.loadMaybeVehicle    = loadMaybeVehicle

M.onVehicleSpawned    = onVehicleSpawned
M.onVehicleDestroyed  = onVehicleDestroyed
--M.onVehicleResetted   = onVehicleResetted -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
--M.onSetClusterPosRelRot = onSetClusterPosRelRot -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached
--M.onCouplerDetach = onCouplerDetach -- TODO: Trailer respawn code is currently active, uncomment this when it's disabled

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

-- ui2
M.getVehicleList = getVehicleList

M.changeMeshVisibility = changeMeshVisibility
M.setMeshVisibility = setMeshVisibility

M.convertVehicleInfo = convertVehicleInfo


-- new vehicle selector prototype

--------------------------------
-- Missions Grid Screen --------


local function addPropertyFilters(value, card, groupsByKey)
  for propName, propVal in pairs(value.aggregates) do
    if tableContains(finalRanges, propName) then
      --[[
      --Ignore for now
      if filter[propName] then
        filter[propName].min = min(value.aggregates[propName].min, filter[propName].min)
        filter[propName].max = max(value.aggregates[propName].max, filter[propName].max)
      else
        filter[propName] = deepcopy(value.aggregates[propName])
      end
      ]]
    else
      for propKey,_ in pairs(propVal) do
        local groupKey = string.format("Prop-%s-%s",propName, propKey)
        groupsByKey[groupKey] = groupsByKey[groupKey] or {label = string.format("%s: %s", propName, propKey), propName = propName}
        card.filterData.groupTags[groupKey] = true
      end
    end
  end
end

local function getVehicleTiles()
  local tilesById = {}

  local groupsByKey = {}
  groupsByKey['allModels'] = {label="All Models"}

  for _, m in ipairs(getModelList(true).models) do
    local model = getModel(m.key)

    groupsByKey["model-"..m.key] = {
      label = m.Name,
      meta = {type = 'simple'}
    }


    local filterData = {
      groupTags = {},
      sortingValues = {},
    }
    filterData.groupTags['allModels'] = true
    local mCard = {
      id = "model-"..m.key,
      name = m.Name,
      image = m.preview,
      filterData = filterData,
      expandGroupKey = "model-"..m.key
    }
    addPropertyFilters(model.model, mCard, groupsByKey)

    tilesById[mCard.id] = mCard

    for _, c in pairs(model.configs) do

      local filterData = {
        groupTags = {},
        sortingValues = {},
      }
      filterData.groupTags["model-"..m.key] = true
      local cCard = {
        id = "config-"..c.key,
        name = c.Name,
        image = c.preview,
        filterData = filterData,
      }
      tilesById[cCard.id] = cCard
      addPropertyFilters(c, cCard, groupsByKey)
    end

    ::continue::
  end

  -- TODO: scenarios and other non-missions

  -- build group lists (new groups might have been added)
  for key, group in pairs(groupsByKey) do
    group.tileIdsUnsorted = {}
    group.meta = group.meta or {type=group.type}
  end

  -- add tiles to groups
  for id, tile in pairs(tilesById) do
    for groupKey, _ in pairs(tile.filterData.groupTags) do
      if groupsByKey[groupKey] then
        table.insert(groupsByKey[groupKey].tileIdsUnsorted, id)
      end
    end
  end

  -- groupSets
  local groupSetsByKey = {}
  for groupKey, group in pairs(groupsByKey) do
    table.insert(groupSetsByKey, groupKey)
  end
  table.sort(groupSetsByKey)
  return {
    tilesById = tilesById,
    groupsByKey = groupsByKey,
    groupKeys = groupSetsByKey
  }

end
M.getVehicleTiles = getVehicleTiles

local tileCount = 1500
local currentVehicleTiles = {}
M.resetVehicleTiles = function()
  currentVehicleTiles = {}
end

local function makeVehicleTiles()
  local tiles = getVehicleTiles()
  currentVehicleTiles = {}
  local cardIds = tableKeys(tiles.tilesById)
  for i = 1, tileCount do
    local tile = tiles.tilesById[cardIds[math.random(1, #cardIds)]]
    if tile then
      table.insert(currentVehicleTiles, tile)
    end
  end
end

M.getVehicleTileCount = function()
  makeVehicleTiles()
  return #currentVehicleTiles
end

M.getVehicleTilesFromTo = function(from, to)
  makeVehicleTiles()
  local tiles = {}
  for i = from, to do
    table.insert(tiles, currentVehicleTiles[i])
  end
  return tiles
end



return M
