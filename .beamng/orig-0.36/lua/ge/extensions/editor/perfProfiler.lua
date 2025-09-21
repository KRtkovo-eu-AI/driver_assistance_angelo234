-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local toolWindowName = "CameraProfiler"
local chartWindowPrefix = "Profiler Chart"
local im = ui_imgui
local plotParams = {
  autoScale = true,
  showCatmullRomCurve = true,
  catmullromCurveLines = 1,
}
local allFiles = {}
local columns, search, searchText, searchChanged, results
local dataKeys = {
  "fps_instant",
  "fps_average",
  "fps_min",
  "fps_max",
  "fps_waitForGPU",
  "gfx_polyCount",
  "gfx_drawCalls",
  "mem_osPhysAvailable",
  "mem_osPhysUsed",
  "mem_osVirtAvailable",
  "mem_osVirtUsed",
  "mem_processPhysUsed",
  "mem_processVirtUsed",
}
local currentDataKey = "fps_average"
local globalNames, globalColors = {}, {}

local settingNamesRaw = {
  {{"cpuInfo","name"},"CPU"},{{"gpuInfo","name"},"GPU"},{{"buildInfo","versiond"},"Version"},{{"settingsInfo","GraphicLightingQuality"},"Lighting"},{{"settingsInfo","GraphicOverallQuality"},"Overall"},{{"settingsInfo","GraphicPostfxQuality"},"Post"},{{"settingsInfo","GraphicMeshQuality"},"Meshes"},{{"settingsInfo","GraphicShaderQuality"},"Shader"},{{"settingsInfo","GraphicTextureQuality"},"Texture"},{{"renderAdapterType"},"Adapter"},{{"settingsInfo","GraphicDisplayResolutions"},"Resolution"}
}
local settingNames = {}
for _, e in ipairs(settingNamesRaw) do
  settingNames[table.concat(e[1],"/")] = e[2]
end

local maxCharts = 10
local charts = {}
local function newChartWindow()
  local chart = {
    plotHelperUtil = require('/lua/ge/extensions/editor/util/plotHelperUtil')(plotParams),
    currentDataKey = "fps_average",
    idx = -1
  }

  for i = 1, maxCharts do
    if charts[i] == nil then
      charts[i] = chart
      chart.idx = i
      local chartWindowName = chartWindowPrefix .. " " .. i
      chart.chartWindowName = chartWindowName
      editor.registerWindow(chartWindowName, im.ImVec2(500, 500))
      editor.showWindow(chartWindowName)
      chart.rebuildPlot = true
      chart.comparatorInfo = true
      return
    end
  end
end

local function drawCharts(dt)
  for i = 1, maxCharts do
    if charts[i] ~= nil then
      local chart = charts[i]
      if editor.beginWindow(chart.chartWindowName, chart.chartWindowName) then
        local size = im.GetContentRegionAvail()
        im.SetNextItemWidth(size.x)
        if im.BeginCombo("Data Key##"..chart.chartWindowName, chart.currentDataKey) then
          for _, key in ipairs(dataKeys) do
            if im.Selectable1(key, key == chart.currentDataKey) then
              chart.currentDataKey = key
              chart.rebuildPlot = true
            end
          end
          im.EndCombo()
        end


        if chart.rebuildPlot then
          chart.rebuildPlot = false
          local plot = {}
          local annotation = nil
          for f, file in ipairs(allFiles) do
            local startIdx = file.samples[1].idx
            if file.enabled then
              annotation = file.meta.annotation
              local row = {}
              for i, sample in ipairs(file.samples) do
                row[i] = {sample.idx - startIdx, sample[chart.currentDataKey]}
              end
              table.insert(plot, row)
            end
          end
          chart.plotHelperUtil:setDataMulti(plot)
          chart.plotHelperUtil:setSeriesNames(globalNames)
          chart.plotHelperUtil:setSeriesColors(globalColors)
          chart.plotHelperUtil:setAnnotationX(annotation)
        end
        chart.plotHelperUtil:draw(size.x-10, size.y-25, dt)

        local enabledFile = nil
        local enabledCount = 0
        for f, file in ipairs(allFiles) do
          local startIdx = file.samples[1].idx
          if file.enabled then
            enabledCount = enabledCount + 1
            enabledFile = file
          end
        end
        if enabledCount == 1 then
          local lines = {enabledFile.meta.name}
          for _, col in ipairs(columns or {}) do
            if col.enabled then
              local label = settingNames[table.concat(col.keys, "/")] or table.concat(col.keys, " / ")
              local val = enabledFile
              for _, key in ipairs(col.keys) do
                val = val[key]
              end
              table.insert(lines,label..": "..tostring(val))
            end
          end
          chart.plotHelperUtil:overlayTextLines(lines)
        end
        editor.endWindow()
      end
        --print("removing chart..")
        --charts[i] = nil
      --end
    end
  end
end

local function startRecording(file)
  extensions.load('test_util_fpsCamRecorder')
  extensions.load('util_stepHandler')
  extensions.load("test_util_camPosDataToBucket")
  editor.setEditorActive(false)
  ui_message("Starting Recording... Please wait.")
  local sequence = { util_stepHandler.makeStepWait(3) }
  test_util_camPosDataToBucket.processSamples({})
  local done = false
  local saveFile = nil
  local dir, fn, ext = path.split(file, true)
  fn = fn:sub(0,fn:len()-ext:len()-1)

  local initData = {}
  initData.useDtReal = true
  initData.reset = function (this) end
  --initData.getNextPath = function(this) return self.pathName end
  --initData.onNextControlPoint = nop--function(this,i,c) self:onNextControlPoint(i,c) end
  initData.finishedPath = function(this)
    done = true
  end
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      core_paths.playPath(core_paths.loadPath(file), 0, initData)
      return true
    end))
  table.insert(sequence, util_stepHandler.makeStepWait(1))
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function(step)
      step.timeout = math.huge
      test_util_fpsCamRecorder.start({sampleDuration = sampleDuration})
      core_paths.playPath(core_paths.loadPath(file), 0, initData)
      return true
    end))
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function(step)
      step.timeout = math.huge
      return done
    end))
  table.insert(sequence, util_stepHandler.makeStepReturnTrueFunction(
    function()
      test_util_fpsCamRecorder.stop()
      local annotationFileName = dir..fn..".perfAnnotation.json"
      local annotation = nil
      if FS:fileExists(annotationFileName) then
        annotation = jsonReadFile(annotationFileName)
      end
      test_util_fpsCamRecorder.save(fn, {name = fn, camPathFile = file, annotation = annotation})
      test_util_fpsCamRecorder.clear()
      ui_message("Done with the recording!")
      return true
    end))
  table.insert(sequence, util_stepHandler.makeStepWait(1))


  util_stepHandler.startStepSequence(sequence, function()
    editor.setEditorActive(true)
    M.loadFile({filepath = saveFile})

  end)
end

local function newRecordingPopup()
  if im.BeginPopup("NewRecording") then
    im.BeginChild1("rchild",im.ImVec2(500,500))
    im.TextWrapped("Start a new Recording")
    im.TextWrapped("For best Results:")
    im.BulletText("Start Recording directly after loading Level after starting the game")
    im.BulletText("Close unnecessary background programs")
    im.BulletText("Remove FPS Limiter")
    im.BulletText(string.format("There are %d vehicle spawned", #getAllVehicles()))
    if im.Button("Delete All Vehicles", im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
      local vehs = deepcopy(getAllVehicles())
      for _, obj in ipairs(vehs) do
        if obj then obj:delete() end
      end
    end
    local levelName = getCurrentLevelIdentifier()
    if not levelName then
      im.Text("Not in a level!")
    else
      local camPathFolder = "levels/"..levelName.."/perfRecordingCampaths/"
      local camPathFiles = FS:findFiles(camPathFolder, "*.camPath.json", 0, true, true)
      if not next(camPathFiles) then
        im.Text("No Campaths to test!")
      end
      for _, file in ipairs(camPathFiles) do
        local dir, fn, ext = path.split(file)
        if im.Button("Start " .. fn) then
          startRecording(file)
        end
      end
    end

    im.EndChild()
    im.EndPopup()
  end
end


local tableWindowName = "Recording Comparator"

local function sortColumn(a,b) return table.concat(a.keys) < table.concat(b.keys) end


local presets = {
  {name = "Default",cols = {{"cpuInfo","name"},{"gpuInfo","name"},{"buildInfo","versiond"},{"settingsInfo","GraphicLightingQuality"},{"settingsInfo","GraphicOverallQuality"},{"settingsInfo","GraphicPostfxQuality"},{"settingsInfo","GraphicMeshQuality"},{"settingsInfo","GraphicShaderQuality"},{"settingsInfo","GraphicTextureQuality"},{"renderAdapterType"},{"settingsInfo","GraphicDisplayResolutions"}}}
}
local function setPreset(preset)
  local keys = {}
  for _, col in ipairs(preset.cols) do
    keys[table.concat(col)] = true
  end
  for _, col in ipairs(columns) do
    col.enabled = keys[table.concat(col.keys)] or false
  end
end
local function drawSearch()
  if im.BeginCombo("##columns", "Add Column...", im.ComboFlags_HeightLarge) then
    if im.InputText("##searchInProject", searchText, nil, im.InputTextFlags_AutoSelectAll) then
      searchChanged = true
    end
    im.SameLine()
    if im.Button("X") then
      searchChanged = true
      searchText = im.ArrayChar(128)
    end
    if searchChanged then
      --self.search:setFrecencyData({})
      search:startSearch(ffi.string(searchText))
      --  self.search:setSameScoreResolvingFunction(sortFun)
      for idx, col in pairs(columns) do
        search:queryElement({
          id = idx,
          name = table.concat(col.keys," / "),
          col = col
        })
      end
      results = search:finishSearch()
      searchChanged = false
    end
    if search.matchString ~= "" and search.matchString then
      for _, result in ipairs(results) do
        im.BeginChild1(result.id.."child", im.ImVec2(im.GetContentRegionAvailWidth(), 22 * editor.getPreference("ui.general.scale") + 2))
        if im.Checkbox("##cba" .. result.id, im.BoolPtr(columns[result.id].enabled or false)) then
          columns[result.id].enabled = not columns[result.id].enabled
        end
        im.SameLine()
        im.HighlightText(result.name, search.matchString)
        im.EndChild()
        if im.IsItemClicked() then
          columns[result.id].enabled = not columns[result.id].enabled
        end
      end
    else
      for i, column in ipairs(columns) do
        im.BeginChild1(i.."child", im.ImVec2(im.GetContentRegionAvailWidth(), 22 * editor.getPreference("ui.general.scale") + 2))
        if im.Checkbox("##cbaa" .. i, im.BoolPtr(column.enabled or false)) then
          column.enabled = not column.enabled
        end
        im.SameLine()
        im.Text(table.concat(column.keys, " / "))
        im.EndChild()
        if im.IsItemClicked() then
          column.enabled = not column.enabled
        end
      end
    end
    im.EndCombo()
  end
  for _, preset in ipairs(presets) do
    im.SameLine()
    if im.Button(preset.name) then
      setPreset(preset)
    end
  end
end
local metadataFile = nil
local metaEdit = nil
local tableFlags = bit.bor(im.TableFlags_Hideable, im.TableFlags_ScrollY, im.TableFlags_Resizable, im.TableFlags_RowBg, im.TableFlags_Reorderable, im.TableFlags_Sortable, im.TableFlags_Borders)
local function drawTableComparator()
  editor.registerWindow(tableWindowName, im.ImVec2(500, 500))
  editor.showWindow(tableWindowName)

  editor.beginWindow(tableWindowName,tableWindowName)
  if next(allFiles) then
    if not columns then
      columns = {}
      table.insert(columns, {keys = {'level'}, enabled = false})
      table.insert(columns, {keys = {'renderAdapterType'}, enabled = false})
      for _, group in ipairs({"osInfo","cpuInfo","gpuInfo","buildInfo","settingsInfo"}) do
        for key, _ in pairs(allFiles[1][group]) do
          table.insert(columns, {keys = {group, key}, enabled = false})
        end
      end
      setPreset(presets[1])
      search = require('/lua/ge/extensions/editor/util/searchUtil')()
      searchText = im.ArrayChar(128)
    end
    drawSearch()
    local activeKeys = 0
    local cols = {}
    for _, col in ipairs(columns) do
      if col.enabled then activeKeys = activeKeys + 1 table.insert(cols, col) end

    end

    if im.BeginTable('', activeKeys+3, tableFlags) then
      im.TableSetupScrollFreeze(0,1)
      im.TableSetupColumn("Plot", nil, 10)
      im.TableSetupColumn("Clr", nil, 10)
      im.TableSetupColumn("File", nil, 60)
      for i, col in ipairs(cols) do
        local label = settingNames[table.concat(col.keys, "/")] or table.concat(col.keys, " / ")
        im.TableSetupColumn(label, nil, 30)
        im.tooltip(table.concat(col.keys, " > "))
      end
      im.TableHeadersRow()
      im.TableNextColumn()
      for i, file in ipairs(allFiles) do
        if im.Checkbox('##'..file.meta.name.."enable"..i, im.BoolPtr(file.enabled)) then
          file.enabled = not file.enabled
          filesChanged = true
        end
        im.TableNextColumn()
        if file.clr then
          if im.ColorEdit4('##'..file.meta.name.."Color"..i, file.clr, im.ColorEditFlags_NoInputs) then
            filesChanged = true
          end
        else
          im.Dummy(im.ImVec2(5,5))
        end
        im.TableNextColumn()
        if im.Selectable1(file.meta.name) then
          extensions.load('test_util_camPosDataToBucket')
          test_util_camPosDataToBucket.setConfig({sampleKey = currentDataKey})
          test_util_camPosDataToBucket.processSamples(file.samples)
          metadataFile = file
          metaEdit = nil
        end
        im.TableNextColumn()

        for _, col in ipairs(cols) do
          local val = file
          for _, key in ipairs(col.keys) do
            val = val[key]
          end
          im.Text(tostring(val))
          im.TableNextColumn()

        end
      end
      im.EndTable()
    end
  end
  editor.endWindow()

end
local metadataWindowName = "Recording Metadata"

local function drawMetadataWindow()
  editor.registerWindow(metadataWindowName, im.ImVec2(500, 500))
  editor.showWindow(metadataWindowName)

  editor.beginWindow(metadataWindowName, metadataWindowName)
  if metadataFile then
    if not metaEdit then
      metaEdit = {}
      metaEdit.name = im.ArrayChar(1024, metadataFile.meta.name)
    end
    if im.InputText("Name", metaEdit.name, 1024) then
      metadataFile.meta.name = ffi.string(metaEdit.name)
    end
    if im.Button("Save") then
      local file = deepcopy(metadataFile)
      file.name = nil
      file.enabled = nil
      file.filepath = nil
      jsonWriteFile(metadataFile.filepath, file )
    end
    im.Separator()
    im.Text(dumpsz(metadataFile.meta or {}, 1))
    im.Text(dumpsz(metadataFile, 1))
  end
  editor.endWindow()
end


local filesChanged, colorsChanged, openRecordPopup
M.loadFile = function(data)
  local filepath = data.filepath
  if filepath == nil or filepath == "" then return end
  local fileData = jsonReadFile(filepath)
  fileData.meta = fileData.meta or {}
  local dir, fn, ext = path.split(filepath, true)
  fileData.filepath = filepath
  fileData.meta.name = fileData.meta.name or fn
  --fileData.meta.annotation = {{0,"Start"},{10,"Section A"},{25,"Section C"}}
  if not fileData.meta.annotation then
    local lastFolder = dir:match(".-([^/]+)/$")
    if lastFolder then
      local lvl, annotationFilename = string.match(dir,".-([^/]+)/([^/]+)/$")
      dump(lvl, annotationFilename)
      if lvl and annotationFilename then
        local file = "/levels/"..lvl.."/perfRecordingCampaths/"..annotationFilename..".perfAnnotation.json"
        dump(file)
        if FS:fileExists(file) then
          fileData.meta.annotation = jsonReadFile(file)
          log("I","Loaded annotation file for recording. It might be out of date. ".. file)
        end
      end
    end
  end
  fileData.enabled = true
  for _, s in ipairs(fileData.samples) do
    s.pos = vec3(s.pos)
    s.rot = quat(s.rot)
  end

  table.insert(allFiles, fileData)
  filesChanged = true
  colorsChanged = true
  metadataFile = file
  metaEdit = nil
end

local function onEditorGui(dt)

  if editor.beginWindow(toolWindowName, toolWindowName, im.WindowFlags_MenuBar) then
    if im.BeginMenuBar() then
      if im.MenuItem1("Record...") then
        openRecordPopup = true
      end
      if im.MenuItem1("Load...") then

        editor_fileDialog.openFile(M.loadFile, {{"Performance Recordings", ".perfRecording.json"}}, false, "/perfRecordings/")
      end
      if im.MenuItem1("New Chart Window") then
        newChartWindow()
      end

      im.EndMenuBar()
    end

    if im.BeginTable("filesForFPS",3) then
      im.TableSetupColumn("Plot", nil, 10)
      im.TableSetupColumn("Clr", nil, 10)
      im.TableSetupColumn("File", nil, 60)
      im.TableNextColumn()
      im.Text("Plot")
      im.TableNextColumn()
      im.Text("Clr")
      im.TableNextColumn()
      im.Text("File")
      im.TableNextColumn()
      for i, file in ipairs(allFiles) do
        if im.Checkbox('##'..file.meta.name.."enable"..i, im.BoolPtr(file.enabled)) then
          file.enabled = not file.enabled
          filesChanged = true
        end
        im.TableNextColumn()
        if file.clr then
          if im.ColorEdit4('##'..file.meta.name.."Color"..i, file.clr, im.ColorEditFlags_NoInputs) then
            filesChanged = true
          end
        else
          im.Dummy(im.ImVec2(5,5))
        end
        im.TableNextColumn()
        if im.Selectable1(file.meta.name) then
          extensions.load('test_util_camPosDataToBucket')
          test_util_camPosDataToBucket.setConfig({sampleKey = currentDataKey})
          test_util_camPosDataToBucket.processSamples(file.samples)
          metadataFile = file
          metaEdit = nil
        end
        im.TableNextColumn()
      end
      im.EndTable()
    end

    if filesChanged then
      globalNames, globalColors = {}, {}
      for f, file in ipairs(allFiles) do
        if file.enabled then
          if not file.clr then
            local clr = rainbowColor(5.5, (f-1)%5.5, 1)
            file.clr = im.TableToArrayFloat({clr[1], clr[2], clr[3], clr[4]})
          end
          table.insert(globalNames, file.name)
          table.insert(globalColors, {file.clr[0],file.clr[1],file.clr[2],file.clr[3]})
        end
      end
      filesChanged = nil
      for i = 1, maxCharts do
        if charts[i] then charts[i].rebuildPlot = true end
      end
    end

    editor.endWindow()
    drawCharts(dt)
    drawTableComparator()
    drawMetadataWindow()
  end
  if openRecordPopup then
    im.OpenPopup("NewRecording")
    openRecordPopup = nil
  end
  newRecordingPopup()
end


local function show()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(500, 500), im.ImVec2(500, 500))
  editor.addWindowMenuItem("Performance Recorder", function() show() end,{groupMenuName="Debug"})
end

M.show = show
M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui
return M