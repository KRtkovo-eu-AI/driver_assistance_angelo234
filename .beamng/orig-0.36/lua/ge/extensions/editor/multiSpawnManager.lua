-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local logTag = "editor_multiSpawnManager"
local toolModeName = "vehicleGroupManager"
local toolName = "Vehicle Groups Manager"
local generatorWindowName = "generatorTester"
local customPaintWindowName = "customPaint"
local overwriteGroupWindowName = "overwriteGroup"

local currGroup = {}
local commonGroups, tempGroup
local prevFilePath = "vehicleGroups/"
local prevFileName = "traffic.vehGroup.json"
local spawnState = 0
local amountWarn = 12
local timedTexts = {}
local comboWidth, inputWidth
local dummy = im.ImVec2(5, 5)
local colorWarning, colorError = im.ImVec4(1, 1, 0, 1), im.ImVec4(1, 0, 0, 1)

local vehSelector, defaultPaint, defaultGenerator, paintKeysTable, options, imValues, selections -- these get initialized when this editor gets enabled

M.filePaths = {"vehicleGroups/", "settings/editor/vehicleGroups/"} -- default file paths to auto load from

local function convertBaseColor(val) -- converts the base color to array format
  local colorTbl
  if type(val) == "table" and val.baseColor then
    if val.baseColor[4] then
      colorTbl = deepcopy(val.baseColor)
    elseif val.baseColor.x and val.baseColor.y and val.baseColor.z and val.baseColor.w then
      colorTbl = {val.baseColor.x, val.baseColor.y, val.baseColor.z, val.baseColor.w}
    end
  else
    colorTbl = deepcopy(defaultPaint)
  end
  return colorTbl
end

local function paintToColor8(paint) -- converts a paint table to a color8 table
  paint = paint or {}
  paint.baseColor = paint.baseColor or {}
  local colorTbl = {clr = im.ArrayFloat(4), pbr = {}}

  colorTbl.clr[0] = im.Float(paint.baseColor[1] or 1)
  colorTbl.clr[1] = im.Float(paint.baseColor[2] or 1)
  colorTbl.clr[2] = im.Float(paint.baseColor[3] or 1)
  colorTbl.clr[3] = im.Float(paint.baseColor[4] or 1)

  colorTbl.pbr[1] = im.FloatPtr(paint.metallic or 0.2)
  colorTbl.pbr[2] = im.FloatPtr(paint.roughness or 0.5)
  colorTbl.pbr[3] = im.FloatPtr(paint.clearcoat or 0.8)
  colorTbl.pbr[4] = im.FloatPtr(paint.clearcoatRoughness or 0)

  return colorTbl
end

local function createSortedArray(t) -- returns a sorted list of values from a dict
  if type(t) ~= "table" then return {} end

  local sorted = {}
  for _, v in pairs(t) do
    table.insert(sorted, v)
  end
  table.sort(sorted)
  return sorted
end

local function validateName(name, prevName) -- checks for duplicate names
  local valid = true

  repeat -- recursive check
    valid = true
    for _, v in ipairs(commonGroups) do
      if name == v.name and prevName ~= v then
        name = name.." - Copy"
        valid = false
      end
    end
  until valid

  return name
end

local function getVehType(model) -- gets the vehicle type from the model
  if not model then return end

  local modelData = core_vehicles.getModel(model)
  if modelData and next(modelData) then
    return modelData.model.Type or "Unknown"
  end
end

local function setGenerator(generatorData) -- sets or creates generator data; also updates imgui values
  currGroup.generator = generatorData or deepcopy(defaultGenerator)
  for k, v in pairs(currGroup.generator) do
    if k == "amount" then
      imValues.collectionAmount[0] = v
    else
      imValues[k][0] = v
    end
  end
end

local function selectVehIndex(idx) -- selects the vehicle slot index and updates the vehicle selector widget
  if not currGroup.data or not currGroup.data[idx] then return end
  options.vehIdx = idx

  vehSelector:resetSelections()

  local currGroupData = currGroup.data[idx]
  vehSelector.vehType = currGroupData.type or getVehType(currGroupData.model)
  vehSelector.model = currGroupData.model
  vehSelector.config = currGroupData.config
  for _, v in ipairs(paintKeysTable) do
    vehSelector[v[1]] = currGroupData[v[1]]
    vehSelector[v[2]] = currGroupData[v[2]]
  end
  if vehSelector.config and string.endswith(vehSelector.config, ".pc") then
    vehSelector.customConfigActive = true
    vehSelector.customConfigPath = vehSelector.config
  end
end

local function cleanupGroup(groupData) -- cleans up data within the given group
  if not groupData.tags then
    groupData.tags = {}
  end

  if groupData.type == "custom" then
    groupData.generator = nil
    groupData.data = groupData.data or {}

    for i = #groupData.data, 1, -1 do
      if not groupData.data[i].model or groupData.data[i].model == "none" then
        table.remove(groupData.data, i)
      end
    end

    for i, data in ipairs(groupData.data) do
      for _, p in ipairs(paintKeysTable) do
        if data[p[1]] == "custom" then
          data[p[1]] = "(Custom)"
        elseif data[p[1]] == "random" then
          data[p[1]] = "(Random)"
        end
      end
    end
  else
    groupData.data = nil
  end
end

local function processGroup(groupData) -- returns group data for saving or spawning with
  cleanupGroup(groupData)

  local processedGroup = deepcopy(groupData)
  if processedGroup.data then -- this can be nil for generator groups
    for i, data in ipairs(processedGroup.data) do
      data.type = nil
      for _, p in ipairs(paintKeysTable) do
        if data[p[1]] ~= "(Custom)" then -- if paint is not custom, clear the paint table
          data[p[2]] = nil
        end
      end
    end
  end

  return processedGroup
end

local function autoLoadGroups() -- loads all groups
  commonGroups = {}
  local commonFiles = {}
  for _, v in ipairs(M.filePaths) do
    local files = FS:findFiles(v, "*.vehGroup.json", -1, true, true)
    table.sort(files)
    commonFiles = arrayConcat(commonFiles, files)
  end
  for _, f in ipairs(commonFiles) do
    local temp = jsonReadFile(f)
    if temp and temp.name then
      table.insert(commonGroups, {name = temp.name, file = f})
    end
  end
end

local function loadGroup(filePath) -- loads and prepares a group from a file
  local tempGroup = jsonReadFile(filePath)

  if tempGroup then
    local oldIdx
    for i, v in ipairs(commonGroups) do
      if v.file == filePath then
        oldIdx = i -- file path already exists in common groups
        break
      end
    end

    if commonGroups[options.groupListIdx] then
      commonGroups[options.groupListIdx].cache = deepcopy(currGroup)
    end
    options.groupListIdx = oldIdx or #commonGroups + 1

    currGroup = tempGroup
    cleanupGroup(currGroup)
    selectVehIndex(1)

    prevFilePath, prevFileName = path.split(filePath)
    if not oldIdx then
      table.insert(commonGroups, {name = currGroup.name, file = filePath})
    end
    log("I", logTag, "Loaded "..currGroup.name.." from "..filePath.." .")
  else
    log("W", logTag, "Failed to read data from "..filePath.." .")
  end
end

local function saveGroup(file, filePath) -- saves the current group to file
  jsonWriteFile(filePath, processGroup(currGroup), true)
  prevFilePath, prevFileName = path.split(filePath)
  commonGroups[options.groupListIdx].file = filePath
  log("I", logTag, "Saved "..currGroup.name.." to "..filePath.." .")
end

local function selectGroup(idx) -- selects a group from the list, by index; also caches previous data
  idx = idx or 0
  if idx ~= options.groupListIdx then
    if commonGroups[options.groupListIdx] then
      commonGroups[options.groupListIdx].cache = deepcopy(currGroup) -- keep the old group data, so that it can be returned later
    end
    options.groupListIdx = idx

    if commonGroups[idx] then
      if commonGroups[idx].cache then
        currGroup = deepcopy(commonGroups[idx].cache)
      else
        if commonGroups[idx].file then
          loadGroup(commonGroups[idx].file)
        else -- no file?
          log("W", logTag, "No file path found for "..commonGroups[idx].name.." .")
        end
      end

      if currGroup.type == "generator" then
        setGenerator(currGroup.generator)
      end
      ffi.copy(imValues.groupName, currGroup.name)
    else
      currGroup = {}
    end

    selectVehIndex(1)
  end
end

local function createGroup(data) -- creates a new vehicle group to edit
  data = data or {}
  local newName = "Custom Group "
  local newIdx = #commonGroups + 1

  currGroup = {
    name = data.name or newName..newIdx,
    type = data.type or "custom",
    tags = data.tags or {},
    data = data.data
  }

  table.insert(commonGroups, currGroup)
  selectGroup(newIdx)
end

local function editGroup() -- displays options and modifies the currently selected group
  im.TextUnformatted("Edit Group")

  if not currGroup.type then
    currGroup.type = "custom"
  elseif currGroup.type == "locked" then -- compatibility
    currGroup.type = "generator"
  end

  -- custom: user can select each vehicle in the available slots
  -- generator: user can set parameters and multispawn will provide the vehicle group

  local edited = im.BoolPtr(false)
  im.PushItemWidth(comboWidth)
  editor.uiInputText("Group Name##editGroup", imValues.groupName, nil, nil, nil, nil, edited)
  im.PopItemWidth()
  if edited[0] then
    currGroup.name = ffi.string(imValues.groupName)
    commonGroups[options.groupListIdx].name = currGroup.name
  end

  im.BeginChild1("groupType##editGroup", im.ImVec2(comboWidth, 32 * im.uiscale[0]), im.WindowFlags_None)
  local val = currGroup.type == "custom" and im.IntPtr(1) or im.IntPtr(2)

  if im.RadioButton2("Custom##editGroup", val, im.Int(1)) then
    currGroup.type = "custom"
  end
  im.tooltip("User defined vehicle selections.")
  im.SameLine()
  im.Dummy(dummy)
  im.SameLine()

  if im.RadioButton2("Generator##editGroup", val, im.Int(2)) then
    currGroup.type = "generator"
  end
  im.tooltip("Auto generated vehicle selections.")
  im.Dummy(dummy)
  im.SameLine()
  im.EndChild()

  im.SameLine()
  im.TextUnformatted("Group Type")

  im.BeginChild1("groupTags##editGroup", im.ImVec2(comboWidth, 64 * im.uiscale[0]), im.WindowFlags_None)

  if not currGroup.tags[1] then
    im.TextUnformatted("No tags applied")
  else
    for i, v in ipairs(currGroup.tags) do
      im.TextUnformatted(v)
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.close, im.ImVec2(16, 16)) then
        table.remove(currGroup.tags, i)
        break
      end
      im.tooltip("Remove Tag")
      im.SameLine()
      im.Dummy(dummy)
      if i % 4 ~= 0 then
        im.SameLine()
      end
    end
    im.Dummy(im.ImVec2(0, 0))
  end

  im.PushItemWidth(math.max(100, comboWidth - 100))
  editor.uiInputText("##editGroupTags", imValues.tagName, nil, nil, nil, nil, edited)
  im.PopItemWidth()
  im.SameLine()
  if im.Button("Add Tag##editGroup") then
    local tagName = ffi.string(imValues.tagName)
    if tagName ~= "" and not arrayFindValueIndex(currGroup.tags, tagName) then
      table.insert(currGroup.tags, tagName)
      ffi.copy(imValues.tagName, "")
    end
  end
  im.EndChild()

  im.SameLine()
  im.TextUnformatted("Group Tags")

  im.Dummy(dummy)

  ---- generator ----

  if currGroup.type ~= "custom" then
    if not currGroup.generator then setGenerator() end -- creates default generator data

    local generator = currGroup.generator

    im.PushItemWidth(inputWidth)
    if im.InputInt("Collection Amount##editGroup", imValues.collectionAmount, 1) then
      imValues.collectionAmount[0] = math.max(1, imValues.collectionAmount[0])
      generator.amount = imValues.collectionAmount[0]
    end
    im.PopItemWidth()

    if im.Checkbox("Use Mod Vehicles##editGroup", imValues.allMods) then
      generator.allMods = imValues.allMods[0]
    end

    if im.Checkbox("Use All Configs##editGroup", imValues.allConfigs) then
      generator.allConfigs = imValues.allConfigs[0]
    end

    im.Dummy(dummy)

    im.PushItemWidth(inputWidth)
    if im.InputInt("Max Model Year##editGroup", imValues.maxYear, 1) then
      if imValues.maxYear[0] == 1899 then imValues.maxYear[0] = 0 end -- if it is one less than the minimum valid year
      if imValues.maxYear[0] > 0 and imValues.maxYear[0] < 1900 then imValues.maxYear[0] = 1900 end
      imValues.maxYear[0] = math.max(0, imValues.maxYear[0])
      generator.maxYear = imValues.maxYear[0]
    end
    im.PopItemWidth()
    im.tooltip("Latest model year to include when generating the vehicle group.")

    im.PushItemWidth(inputWidth)
    if im.BeginCombo("Country", generator.country or "(Default)") then
      for _, v in ipairs(options.countries) do
        if im.Selectable1(v, v == generator.country) then
          generator.country = v
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()
    im.tooltip("Country name; allows domestic vehicles to be selected more often.")

    im.Dummy(dummy)

    im.PushItemWidth(inputWidth)
    if im.InputInt("Minimum Population##editGroup", imValues.minPop, 1) then
      imValues.minPop[0] = math.max(0, imValues.minPop[0])
      generator.minPop = imValues.minPop[0]
    end
    im.PopItemWidth()
    im.tooltip("Minimum population required for model / config to be usable (useful for filtering out super rare configs).")

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Model Population Power##editGroup", imValues.modelPopPower, 0.05, nil, "%.2f") then
      imValues.modelPopPower[0] = math.max(0, imValues.modelPopPower[0])
      generator.modelPopPower = imValues.modelPopPower[0]
    end
    im.PopItemWidth()
    im.tooltip("Exponent to apply to population; lower values mean that the model will be less biased to be selected by its base population value.")

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Config Population Power##editGroup", imValues.configPopPower, 0.05, nil, "%.2f") then
      imValues.configPopPower[0] = math.max(0, imValues.configPopPower[0])
      generator.configPopPower = imValues.configPopPower[0]
    end
    im.PopItemWidth()
    im.tooltip("Exponent to apply to population; lower values mean that the config will be less biased to be selected by its base population value.")

    im.PushItemWidth(inputWidth)
    if im.InputFloat("Population Decrease Factor##editGroup", imValues.popDecreaseFactor, 0.01, nil, "%.2f") then
      imValues.popDecreaseFactor[0] = math.max(0, imValues.popDecreaseFactor[0])
      generator.popDecreaseFactor = imValues.popDecreaseFactor[0]
    end
    im.PopItemWidth()
    im.tooltip("Population multiplier after vehicle insertion; use low values to prevent repeat models / configs.")

    im.Dummy(dummy)
    if im.Button("Test Generator...##editGroup") then
      editor.openModalWindow(generatorWindowName)
    end

    -- group generator modal window
    if editor.beginModalWindow(generatorWindowName, "Generator Tester") then
      if im.Button("Generate Group##testGenerator") then
        core_multiSpawn.useFullData = true
        options.generatedGroup = core_multiSpawn.createGroup(currGroup.generator.amount, currGroup.generator)
      end
      im.SameLine()

      if im.Button("Copy Group") then
        if options.generatedGroup[1] then
          local tempData = deepcopy(currGroup)
          tempData.name = tempData.name.." - Copy"
          tempData.type = "custom"
          tempData.data = deepcopy(options.generatedGroup)
          createGroup(tempData)

          core_multiSpawn.useFullData = false
          table.clear(options.generatedGroup)
          editor.closeModalWindow(generatorWindowName)
        end
      end
      im.SameLine()

      if im.Button("Close##testGenerator") then
        core_multiSpawn.useFullData = false
        table.clear(options.generatedGroup)
        editor.closeModalWindow(generatorWindowName)
      end

      im.Dummy(dummy)
      im.Separator()

      im.BeginChild1("Generated Group##editGroup", im.ImVec2(im.GetContentRegionAvailWidth(), 470 * im.uiscale[0]), im.WindowFlags_ChildWindow)
      local width1 = 40 * im.uiscale[0]
      local width2 = im.GetContentRegionAvailWidth() - 100 * im.uiscale[0]

      im.Columns(3, "list", false)
      im.SetColumnWidth(0, width1)
      im.SetColumnWidth(1, width2)

      im.TextUnformatted("Index")
      im.NextColumn()
      im.TextUnformatted("Config Name")
      im.NextColumn()
      im.TextUnformatted("Population")
      im.NextColumn()

      im.Columns(1)
      im.Separator()

      im.Columns(3, "list", false)
      im.SetColumnWidth(0, width1)
      im.SetColumnWidth(1, width2)

      for i, v in ipairs(options.generatedGroup) do
        im.TextUnformatted(tostring(i))
        im.NextColumn()

        local modelName, configName
        local model = core_vehicles.getModel(v.model).model
        if model.Name then
          modelName = model.Brand and model.Brand.." "..model.Name or model.Name
        else
          modelName = v.model
        end
        local config = v.config and core_vehicles.getModel(v.model).configs[v.config]
        configName = config and config.Configuration or ""
        im.TextUnformatted(modelName.." "..configName)
        im.NextColumn()
        im.TextUnformatted(tostring(v.configPop))
        im.NextColumn()
      end
      im.Columns(1)
      im.EndChild()
      editor.endModalWindow()
    end

  ---- custom vehicle group ----

  else
    if not currGroup.data then currGroup.data = {} end
    local width = im.GetContentRegionAvailWidth()
    local groupSize = #currGroup.data

    for i, data in ipairs(currGroup.data) do
      local isCurrent = i == options.vehIdx
      if isCurrent then
        im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonActive))
      end
      if im.Button(i.."##vehicleSlot", im.ImVec2(24 * im.uiscale[0], 20 * im.uiscale[0])) then
        selectVehIndex(i)
      end
      if isCurrent then
        im.PopStyleColor()
      end
      if i < groupSize and (i * 36 * im.uiscale[0]) % width >= 36 * im.uiscale[0] then -- ensures that buttons wrap downwards
        im.SameLine()
      end
    end

    im.Columns(2, "Vehicle Index", false)
    im.SetColumnWidth(0, 80 * im.uiscale[0])

    im.TextUnformatted("Vehicle #"..tostring(options.vehIdx))
    im.NextColumn()

    if im.Button("Add##vehicleSlot") then
      table.insert(currGroup.data, {})
      groupSize = #currGroup.data
      selectVehIndex(groupSize)
    end
    im.SameLine()
    if im.Button("Remove##vehicleSlot") then
      table.remove(currGroup.data, options.vehIdx)
      groupSize = #currGroup.data
      selectVehIndex(math.max(1, options.vehIdx - 1))
    end

    if not currGroup.data[1] then
      table.insert(currGroup.data, {})
      groupSize = #currGroup.data
      selectVehIndex(groupSize)
    end
    im.NextColumn()

    im.Columns(1)
    im.Dummy(dummy)

    local currGroupData = currGroup.data[options.vehIdx]

    local updateReason = vehSelector:widget()

    if vehSelector.paintKeys and vehSelector.paintKeys[1] ~= "(Custom)" then
      table.insert(vehSelector.paintKeys, 1, "(Random)")
      table.insert(vehSelector.paintKeys, 1, "(Custom)")
    end

    if updateReason then -- whenever vehicle selector updates, update the current data
      currGroupData.type = vehSelector.vehType
      currGroupData.model = vehSelector.model
      currGroupData.config = vehSelector.config
      currGroupData.paintName = vehSelector.paintName
      currGroupData.paintName2 = vehSelector.paintName2
      currGroupData.paintName3 = vehSelector.paintName3

      if string.startswith(updateReason, "paint") and currGroupData[updateReason] == "(Custom)" then
        options.colorPickerData = nil
        options.colorPickerPaintKey = updateReason:gsub("Name", "")
        editor.openModalWindow(customPaintWindowName)
      end
    end
  end

  -- custom paint modal window
  if editor.beginModalWindow(customPaintWindowName, "Custom Paint") then
    local editEnded = im.BoolPtr(false)
    local currPaint = currGroup.data[options.vehIdx][options.colorPickerPaintKey] or {}

    if not options.colorPickerData then
      options.colorPickerData = paintToColor8(currPaint)
    end

    editor.uiColorEdit8("##vehicleColorPicker", options.colorPickerData, nil, editEnded)
    if editEnded[0] then
      currPaint.baseColor = {options.colorPickerData.clr[0], options.colorPickerData.clr[1], options.colorPickerData.clr[2], options.colorPickerData.clr[3]}
      currPaint.metallic = options.colorPickerData.pbr[1][0]
      currPaint.roughness = options.colorPickerData.pbr[2][0]
      currPaint.clearcoat = options.colorPickerData.pbr[3][0]
      currPaint.clearcoatRoughness = options.colorPickerData.pbr[4][0]
      currGroup.data[options.vehIdx][options.colorPickerPaintKey] = currPaint
    end
    im.Dummy(dummy)
    if im.Button("Done##customPaint", im.ImVec2(im.GetContentRegionAvailWidth(), 20)) then
      editor.closeModalWindow(customPaintWindowName)
    end

    editor.endModalWindow()
  end

  -- confirm overwrite group modal window
  if editor.beginModalWindow(overwriteGroupWindowName, "Confirm") then
    im.TextUnformatted("Are you sure?")
    im.TextUnformatted("This will overwrite the current group.")
    if im.Button("Yes##overwriteGroup") then
      currGroup.type = "custom"
      currGroup.data = core_multiSpawn.spawnedVehsToGroup()
      selectVehIndex(1)
      editor.closeModalWindow(overwriteGroupWindowName)
    end
    im.SameLine()
    if im.Button("No##overwriteGroup") then
      editor.closeModalWindow(overwriteGroupWindowName)
    end

    editor.endModalWindow()
  end
end

local function spawnGroup() -- spawns the current vehicle group into the world
  im.TextUnformatted("Spawn Group")

  im.PushItemWidth(comboWidth)
  if im.BeginCombo("Spawn Mode##multiSpawn", options.spawnModeValue) then
    for _, v in ipairs(options.spawnModesSorted) do
      local selected = options.spawnModeValue == v
      if im.Selectable1(v, selected) then
        options.spawnMode, options.spawnModeValue = tableFindKey(options.spawnModesDict, v), v
      end
      if selected then
        im.SetItemDefaultFocus()
      end
    end
    im.EndCombo()
  end
  im.PopItemWidth()

  im.PushItemWidth(inputWidth)
  if im.InputInt("Spacing##multiSpawn", imValues.spawnGap, 1) then
    imValues.spawnGap[0] = math.max(0, imValues.spawnGap[0])
  end
  im.PopItemWidth()

  if currGroup.type == "generator" then
    imValues.shuffle[0] = true
    im.BeginDisabled()
  end
  im.Checkbox("Shuffle##multiSpawn", imValues.shuffle)
  if currGroup.type == "generator" then im.EndDisabled() end

  im.Dummy(dummy)

  im.PushItemWidth(inputWidth)
  if im.InputInt("Amount##multiSpawn", imValues.amount, 1) then
    imValues.amount[0] = math.max(0, imValues.amount[0])
  end
  im.PopItemWidth()

  if imValues.amount[0] > amountWarn then
    im.TextColored(colorWarning, "Warning, too many vehicles may result in poor performance!")
  end

  if spawnState == 1 then
    if not scenetree.MissionGroup then
      log("W", logTag, "No level loaded, unable to spawn vehicles!")
      spawnState = 0
    else
      if currGroup.generator then
        tempGroup = core_multiSpawn.createGroup(currGroup.generator.amount, currGroup.generator)
      elseif currGroup.data then
        tempGroup = processGroup(currGroup).data
      else
        log("W", logTag, "No vehicle group data exists!")
        spawnState = 0
      end

      if spawnState == 1 then
        core_multiSpawn.spawnGroup(tempGroup, imValues.amount[0], {name = options.groupKey, shuffle = imValues.shuffle[0], mode = options.spawnMode, gap = imValues.spawnGap[0]})
        tempGroup = nil
      end
    end
    spawnState = 2
  end

  if im.Button("Spawn##multiSpawn") then
    spawnState = 1
  end
  im.SameLine()

  if im.Button("Delete##multiSpawn") then
    core_multiSpawn.deleteVehicles(imValues.amount[0])
  end

  if spawnState > 0 then
    im.SameLine()
    im.TextColored(colorWarning, "Please wait...") -- while loading vehicles
  end
end

local function initTables() -- runs on first load
  vehSelector = require("/lua/ge/extensions/editor/util/vehicleSelectUtil")("Vehicle##multiSpawn")
  vehSelector.enablePaints = true
  vehSelector.enableCustomConfig = true
  vehSelector.paintLayers = 3
  vehSelector:resetSelections()

  defaultPaint = {1, 1, 1, 1}
  defaultGenerator = {amount = 10, allMods = false, allConfigs = false, minPop = 0, modelPopPower = 0, configPopPower = 0, popDecreaseFactor = 0.05, maxYear = 0}
  paintKeysTable = {{"paintName", "paint"}, {"paintName2", "paint2"}, {"paintName3", "paint3"}}

  options = {}
  options.spawnModesDict = {road = "Road", traffic = "Traffic", lineAhead = "Line (Ahead)", lineBehind = "Line (Behind)", lineLeft = "Line (Left)", lineRight = "Line (Right)", lineAbove = "Line (Above)", raceGrid = "Race Grid", raceGridAlt = "Race Grid (Alt)"}
  options.spawnMode = "road"
  options.spawnModeValue = options.spawnModesDict.road

  options.types = {Car = "Car", Truck = "Truck", Trailer = "Trailer", Prop = "Prop", Utility = "Utility", Automation = "Automation", Traffic = "Traffic"}
  options.typesSorted = {"Car", "Truck", "Trailer", "Prop", "Utility", "Automation", "Traffic"}

  options.countries = {"(Default)", "United States", "France", "Germany", "Italy", "Poland", "Japan"} -- temporary typical list of countries of origin

  options.models, options.configs, options.paints, options.generatedGroup = {}, {}, {}, {}
  options.vehIdx, options.groupListIdx = 0, 0

  options.spawnModesSorted = createSortedArray(options.spawnModesDict)

  imValues = {}
  imValues.amount, imValues.spawnGap, imValues.shuffle = im.IntPtr(1), im.IntPtr(15), im.BoolPtr(false)
  imValues.collectionAmount, imValues.allMods, imValues.allConfigs = im.IntPtr(10), im.BoolPtr(false), im.BoolPtr(false)
  imValues.minPop, imValues.modelPopPower, imValues.configPopPower, imValues.popDecreaseFactor = im.IntPtr(1), im.FloatPtr(0), im.FloatPtr(0), im.FloatPtr(0)
  imValues.maxYear = im.IntPtr(0)
  imValues.tagName, imValues.groupName = im.ArrayChar(256, ""), im.ArrayChar(256, "")

  selections = {
    {name = "Type", type = "types", key = "type", sortedRef = "typesSorted", default = "Car", active = true},
    {name = "Model", type = "models", key = "model", sortedRef = "modelsSorted", default = "(None)", active = true},
    {name = "Config", type = "configs", key = "config", sortedRef = "configsSorted", default = "(Default)", active = true},
    {name = "Paint 1", type = "paints", key = "paintName", paintKey = "paint", sortedRef = "paintsSorted", default = "(Default)", active = true},
    {name = "Paint 2", type = "paints", key = "paintName2", paintKey = "paint2", sortedRef = "paintsSorted", default = "(Default)", active = false},
    {name = "Paint 3", type = "paints", key = "paintName3", paintKey = "paint3", sortedRef = "paintsSorted", default = "(Default)", active = false}
  }

  autoLoadGroups()
end

local function onEditorGui(dt)
  if editor.beginWindow(toolModeName, toolName, im.WindowFlags_MenuBar) then
    if not commonGroups then initTables() end

    comboWidth = im.GetWindowWidth() * 0.6
    inputWidth = 100 * im.uiscale[0]

    im.BeginMenuBar()
    if im.BeginMenu("File") then
      if im.MenuItem1("New") then
        createGroup()
      end
      if im.MenuItem1("Load...") then
        editor_fileDialog.openFile(function(data) loadGroup(data.filepath) end, {{"Vehicle Group Files", ".vehGroup.json"}}, false, prevFilePath)
      end
      if im.MenuItem1("Save") then
        if commonGroups[options.groupListIdx].file then
          saveGroup(nil, commonGroups[options.groupListIdx].file)
        else
          editor_fileDialog.saveFile(function(data) saveGroup(nil, data.filepath) end, {{"Vehicle Group Files", ".vehGroup.json"}}, false, prevFilePath)
        end
      end
      if im.MenuItem1("Save as...") then
        editor_fileDialog.saveFile(function(data) saveGroup(nil, data.filepath) end, {{"Vehicle Group Files", ".vehGroup.json"}}, false, prevFilePath)
      end
      im.EndMenu()
    end

    if im.BeginMenu("Tools") then
      if im.MenuItem1("Shuffle Group") then
        if next(currGroup) and currGroup.data then
          currGroup.data = arrayShuffle(currGroup.data)
          selectVehIndex(1)
        end
      end
      if im.MenuItem1("Duplicate Group") then
        if next(currGroup) then
          local tempData = deepcopy(currGroup)
          tempData.name = tempData.name.." - Copy"
          createGroup(tempData)
        end
      end
      if im.MenuItem1("Set Scene Vehicles to Group") then
        if next(currGroup) then
          editor.openModalWindow(overwriteGroupWindowName)
        end
      end

      im.EndMenu()
    end

    if timedTexts.save then
      im.SameLine()
      im.TextColored(colorWarning, timedTexts.save[1])
    end
    im.EndMenuBar()

    im.TextUnformatted("Current Vehicle Group: ")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    local currName = commonGroups[options.groupListIdx] and commonGroups[options.groupListIdx].name or "(None)"
    if im.BeginCombo("##commonGroups", currName) then
      for i, v in ipairs(commonGroups) do
        if im.Selectable1(v.name.."##groupListIdx"..i, currName == v.name) then
          selectGroup(i)
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    im.Separator()

    if commonGroups[options.groupListIdx] then
      editGroup()
      im.Dummy(dummy)
      im.Separator()
      spawnGroup()
    else
      im.TextUnformatted("Create or select a vehicle group to continue.")
      if im.Button("New Group") then
        createGroup()
      end
    end
  end

  for k, v in pairs(timedTexts) do
    if v[2] then
      v[2] = v[2] - dt
      if v[2] <= 0 then timedTexts[k] = nil end
    end
  end

  editor.endWindow()
end

local function onVehicleGroupSpawned()
  spawnState = 0
end

local function onWindowMenuItem()
  if not commonGroups then initTables() end
  editor.clearObjectSelection()
  editor.showWindow(toolModeName)
end

local function onEditorInitialized()
  editor.registerWindow(toolModeName, im.ImVec2(500, 540))
  editor.registerModalWindow(generatorWindowName, im.ImVec2(440, 500), nil, true)
  editor.registerModalWindow(customPaintWindowName, im.ImVec2(320, 120), nil, true)
  editor.registerModalWindow(overwriteGroupWindowName, im.ImVec2(240, 100), nil, true)
  editor.addWindowMenuItem(toolName, onWindowMenuItem, {groupMenuName = "Gameplay"})
end

-- public interface
M.onVehicleGroupSpawned = onVehicleGroupSpawned
M.onWindowMenuItem = onWindowMenuItem
M.onEditorGui = onEditorGui
M.onEditorInitialized = onEditorInitialized
M.loadGroup = loadGroup
M.saveGroup = saveGroup

return M