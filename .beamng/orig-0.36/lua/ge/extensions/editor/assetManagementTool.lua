-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'asset_management_tool'
local imgui = ui_imgui
local ffi = require('ffi')
local imUtils = require('ui/imguiUtils')
local toolWindowName = "assetManagementTool"
local removeSelectedDlg = "assetManagement_removeSelectedDlg"
local migrationDlg = "assetManagement_migrationDlg"
local checkDlg = "assetManagement_migrationNewPathNotSetMsgDlg"
local searchDuplicatesDlg = "assetManagement_migrationSearchDuplicatesDlg"
local searchAllDuplicatesDlg = "assetManagement_migrationSearchAllDuplicatesDlg"
local savedListDlg = "assetManagement_migrationSavedListDlg"
local checkAssetsFolderDuplicatesDlg = "assetManagement_migrationCheckAssetsFolderDuplicatesDlg"
local checkForInvalidLinksDlg = "assetManagement_checkForInvalidLinksDlg"
local checkForInvalidFilenamesDlg = "assetManagement_checkForInvalidFilenamesDlg"
local deleteInvalidLinksDlg = "assetManagement_deleteInvalidLinksDlg"
local tryFixingInvalidLinksDlg = "assetManagement_tryFixingInvalidLinksDlg"
local clearDuplicateListDlg = "assetManagement_clearDuplicateListDlg"
local pathSize = 500
local newPathNotSetString = "/assets/UNDEFINED"
local gameDataFolders = { "/art", "/core", "/campaigns", "/levels", "/protected", "/vehicles" }
local listClip

local progress = 0
local message = ""
local stopped = false

local assets = {}
local assetsByHash = {}
local assetsByIndex = {}
local assetRecordRowOffsets = {}
local allAssetsWithHashes = {}
local selectedHashes = {}
local visibleHashes = {}
local invalidLinkFiles = {}
local invalidFilenames = {}

local inputTextOld = imgui.ArrayChar(pathSize)
local inputTextNew = imgui.ArrayChar(pathSize)
local searchFilter = imgui.ImGuiTextFilter()
local selectRecordBoolPtr = imgui.BoolPtr(false)
local recursiveDuplicateSearchPtr = imgui.BoolPtr(true)
local recursiveNamingSearchPtr = imgui.BoolPtr(true)
local skipUnnamedTargetPathsPtr = imgui.BoolPtr(true)
local visibleRecordCount = 0
local lastFileOpenPath = "/"
local forceFilterNow = false
local searchForDuplicatesPath = "/"
local searchNamingPath = "/assets"
local selectionTargetPath = ""
local changeTargetPathForHash = nil
local allowAddingFilesWithDifferentHashes = false
local totalDuplicatesFound = 0
local totalAssetRecordCount = 0
local totalRowCount = 0
local assetsOnlyStr = "Assets Only"
local searchFileTypesFilterStr = assetsOnlyStr
local namingFileTypesFilterStr = assetsOnlyStr
local devMode = worldEditorCppApi.isSelectMultipleFilesAvailable()

local assetFileTypesTbl = {
  assetsOnlyStr,
  "*.*",
  "*.dae,*.dts,*.prefab,*.cdae,*.ter",
  "*.png,*.jpg,*.dds,*.ico,*.bmp,*.tga,*.svg,*.gif,*.ttf,*.otf,*.eot,*.woff,*.exr,*.webp",
  "*.material.json,*.materials.json",
  "*.pc,*.jbeam",
  "*.lua,*.cs,*.js,*.cmd,*.bat,*.txt,*.csv,*.gui,*.html,*.css,*.blend,*.ini,*.log,*.md,*.nav,*.py",
  "*.json",
  "*.datablocks.json,*.datablock.json",
  "*.ogg,*.pogg,*.wav,*.flac,*.bank" }

local assetsOnlyFilter = table.concat(assetFileTypesTbl, "\t")
assetsOnlyFilter = assetsOnlyFilter:gsub(assetsOnlyStr .. "\t%*%.%*\t", "")
assetsOnlyFilter = assetsOnlyFilter:gsub(",", "\t")

local searchDuplicatesFileTypesComboItems = imgui.ArrayCharPtrByTbl(assetFileTypesTbl)
local searchDuplicatesFileTypesIndexPtr = imgui.IntPtr(0)
local namingFileTypesComboItems = imgui.ArrayCharPtrByTbl(assetFileTypesTbl)
local namingFileTypesIndexPtr = imgui.IntPtr(0)

local function isGamePathSameAsUserPath()
  -- TODO: use Torque3d::PathUtils::noUserPath, expose it to Lua
  return FS:getGamePath() == FS:getUserPath()
end

local function changeBasePath(path, newBasePath)
  return sanitizePath(path:gsub("(.*/)(.*)", newBasePath .. "/%2"), false)
end

local function isFilenameInvalid(filename)
  return filename:match("[^a-z0-9_./]") ~= nil
end

local function fixAssetFilename(filename)
  -- convert camelCase transitions (lowercase followed by uppercase) to lowercase with an underscore
  filename = filename:gsub("([a-z])([A-Z])", "%1_%2")
  filename = filename:lower()
  filename = filename:gsub("[^a-z0-9_./]+", "_")

  return filename
end

local function deleteAllInvalidLinkFiles()
  local validLinkFiles = {}

  for i, record in ipairs(invalidLinkFiles) do
    if not record.fixed then
      editor.log("Deleting invalid link file: " .. record.linkPath)
      FS:removeFile(record.linkPath)
    else
      table.insert(validLinkFiles, record)
    end
  end

  -- just replace the old list with what remains, which is valid links paths
  invalidLinkFiles = validLinkFiles
end

local function filterDuplicateList()
  local isFilterActive = imgui.ImGuiTextFilter_IsActive(searchFilter)
  local hasMatch

  visibleRecordCount = 0
  totalRowCount = 0
  visibleHashes = {}
  selectedHashes = {}
  assetRecordRowOffsets = {} -- first offset to first record

  for index, record in ipairs(assetsByIndex) do
    hasMatch = true

    if isFilterActive then
      hasMatch = false

      local str = imgui.TextFilter_GetInputBuf(searchFilter)

      for i = 1, #record.paths do
        if imgui.ImGuiTextFilter_PassFilter(searchFilter, record.paths[i]) then
          hasMatch = true
          break
        end
      end

      if imgui.ImGuiTextFilter_PassFilter(searchFilter, record.targetPath)
        or imgui.ImGuiTextFilter_PassFilter(searchFilter, record.hash) then
        hasMatch = true
      end
    end

    if hasMatch then
      visibleRecordCount = visibleRecordCount + 1

      -- compute offsets for each record, to use in the virtual list for imgui
      local m = #record.paths
      for j = 1, m do
          totalRowCount = totalRowCount + 1
          assetRecordRowOffsets[totalRowCount] = {recordIndex = index, pathIndex = j}
      end

      table.insert(visibleHashes, record.hash)
    end
  end
end

local function isTargetPathValid(path)
  local size = #newPathNotSetString

  if string.sub(path, 1, size) == newPathNotSetString then
    return false
  end

  return true
end

local function allTargetPathsAreValid()
  local size = #newPathNotSetString

  for hash, asset in pairs(assetsByHash) do
    if string.sub(asset.targetPath, 1, size) == newPathNotSetString then
      return false
    end
  end

  return true
end

local function clearEverything()
  totalDuplicatesFound = 0
  totalAssetRecordCount = 0
  allAssetsWithHashes = {}
  assets = {}
  assetsByHash = {}
  assetsByIndex = {}
  selectedHashes = {}
  filterDuplicateList()
end

local function rebuildAssetsByIndex()
  assetsByIndex = {}

  for key, val in pairs(assetsByHash) do
    table.insert(assetsByIndex, val)
  end

  totalAssetRecordCount = #assetsByIndex
end

local function migrateAssetsJob(job)
  stopped = false
  progress = 0

  local recordCount = tableSize(assetsByHash)
  local idx = 0
  local firstOldPath
  local linkPath
  local linkJsonData
  local updateIntervalCount = 50

  editor.logInfo("Started asset migration process...")

  for hash, asset in pairs(assetsByHash) do
    -- copy first asset in list to new location (ignore others in the list for this hash since they're the same)
    -- if we have asset paths to migrate, do it
    if #asset.paths ~= 0 then
      firstOldPath = asset.paths[1]

      if isTargetPathValid(asset.targetPath) then
        if FS:fileExists(asset.targetPath) then
          editor.logWarn("Asset file already exists, skipping: " .. asset.targetPath)
        else
          if not FS:isLinkFile(firstOldPath) then
            if FS:copyFile(firstOldPath, asset.targetPath) then
              message = "\tCopied '" .. firstOldPath .. "' to '" .. asset.targetPath .. "'"
              editor.logInfo(message)

              -- delete the original files and create links files
              for _, path in ipairs(asset.paths) do
                FS:removeFile(path)
                editor.logInfo("\tRemoved '" .. path .. "'")
                linkPath = path .. ".link"
                linkJsonData = {}
                linkJsonData["path"] = asset.targetPath
                linkJsonData["hash"] = FS:getHashFileAlgorithmId() .. ":" .. hash
                linkJsonData["time"] = getDateTimeUTCString()
                serializeJsonToFile(linkPath, linkJsonData, true)
                editor.logInfo("\tCreated link '" .. linkPath .. "'")
                assets[path] = nil
              end
              -- remove the asset from the list
              assetsByHash[hash] = nil
            end
          else
            editor.logWarn("\tLinks already present for '" .. asset.targetPath .. "', skipping...")
          end
        end
      else
        editor.logWarn("\tUnnamed target asset, skipping: " .. asset.targetPath)
      end
    end

    idx = idx + 1
    progress = idx / recordCount

    if idx % updateIntervalCount == 0 then
      coroutine.yield()
    end

    if stopped then break end
  end

  message = "Done."

  totalDuplicatesFound = 0
  selectedHashes = {}
  rebuildAssetsByIndex()
  filterDuplicateList()
  editor.log("Ended asset migration")
  stopped = true
  progress = 1
end

local function addFiles()
  if not devMode then return end

  local paths = worldEditorCppApi.selectMultipleFiles()

  if #paths then
    local vfsPath
    local hash

    editor.log("Adding " .. #paths .. " asset file(s)...")

    for _, filepath in ipairs(paths) do
      vfsPath = FS:native2Virtual(filepath)

      if not assets[vfsPath] then
        hash = FS:hashFile(vfsPath)
        editor.log("\tHashed `" .. vfsPath .. "` -> " .. hash)

        assets[vfsPath] = hash

        if not assetsByHash[hash] then
          local folder, fileNameOnly, ext = path.split(vfsPath)
          fileNameOnly = fixAssetFilename(fileNameOnly)
          assetsByHash[hash] = {hash = hash, targetPath = newPathNotSetString .. "/" .. fileNameOnly, paths = {}}
        end

        table.insert(assetsByHash[hash].paths, vfsPath)
      end
    end
  end

  rebuildAssetsByIndex()
  filterDuplicateList()
end

local function searchDuplicatesForList(job)
  progress = 0
  stopped = false

  local depth = -1

  if not recursiveDuplicateSearchPtr[0] then depth = 0 end

  local filenames = FS:findFiles(searchForDuplicatesPath, '*.*', depth, false, false)
  local hash
  local total = #filenames
  local updateInterval = 10

  editor.log("Searching for duplicates for the assets in the current migration list...")

  for i, filename in ipairs(filenames) do

    if i % updateInterval == 1 then
      message = "Hashing " .. filename
    end

    hash = FS:hashFile(filename)

    if assetsByHash[hash] and not tableContains(assetsByHash[hash].paths, filename) then
      table.insert(assetsByHash[hash].paths, filename)
      editor.log("\tAdded duplicate `" .. filename .. "` with hash: " .. hash)
    end

    progress = i / total
    coroutine.yield()

    if stopped then break end
  end

  message = "Done."
  editor.log("Ended search for duplicates.")
  stopped = true
  progress = 1
  rebuildAssetsByIndex()
  filterDuplicateList()
end

local function checkForDuplicates(searchFolders)
  stopped = false
  progress = 0

  clearEverything()

  local hash
  local filenames
  local totalFolders = #searchFolders
  local total = 0
  local updateIntervalCount = 20
  local updateCounter = 0
  local filter

  if assetsOnlyStr == searchFileTypesFilterStr:gsub("\t", "") then
    filter = assetsOnlyFilter
  else
    filter = searchFileTypesFilterStr
  end

  for folderIndex, folder in ipairs(searchFolders) do
    message = "Gathering files from: " .. folder
    coroutine.yield()

    filenames = FS:findFiles(folder, filter, -1, true, false)
    total = #filenames

    for i, filename in ipairs(filenames) do
      updateCounter = updateCounter + 1
      -- update less often since it will slow down the operation otherwise
      if updateCounter > updateIntervalCount then
        local folder, fileNameOnly, ext = path.split(filename)
        message = "Hashing files in: " .. folder
        progress = (folderIndex - 1) / totalFolders
        coroutine.yield()
        updateCounter = 0
      end

      hash = FS:hashFile(filename)

      if allAssetsWithHashes[hash] and not tableContains(allAssetsWithHashes[hash].paths, filename) then
        table.insert(allAssetsWithHashes[hash].paths, filename)
      else
        if not allAssetsWithHashes[hash] then
          allAssetsWithHashes[hash] = { hash = hash, paths = {} }
        end

        table.insert(allAssetsWithHashes[hash].paths, filename)
      end

      if stopped then break end
    end

    progress = (folderIndex - 1) / totalFolders
    coroutine.yield()

    if stopped then break end
  end

  editor.log("Discovered " .. tableSize(allAssetsWithHashes) .. " game files")

  -- now add the assets that have more than 1 file in the list
  local folderIndex = 0
  total = tableSize(allAssetsWithHashes)
  local idx = 0

  for hash, asset in pairs(allAssetsWithHashes) do
    -- if its more than one, then we have duplicates
    if #asset.paths > 1 then
      updateCounter = updateCounter + 1
      -- update less often since it will slow down the operation otherwise
      if updateCounter > updateIntervalCount then
        progress = idx / total
        message = "Adding duplicate files for: " .. asset.paths[1]
        coroutine.yield()
        updateCounter = 0
      end

      for _, filename in ipairs(asset.paths) do
        if assetsByHash[hash] and not tableContains(assetsByHash[hash].paths, filename) then
          table.insert(assetsByHash[hash].paths, filename)
        else
          if not assetsByHash[hash] then
            local folder, fileNameOnly, ext = path.split(filename)
            fileNameOnly = fixAssetFilename(fileNameOnly)
            assetsByHash[hash] = {hash = hash, targetPath = newPathNotSetString .. "/" .. fileNameOnly, paths = {}}
          end

          table.insert(assetsByHash[hash].paths, filename)
          totalDuplicatesFound = totalDuplicatesFound + 1
        end
      end
    end

    idx = idx + 1
  end

  message = "Found " .. totalDuplicatesFound .. " duplicate assets."
  stopped = true
  progress = 1
  rebuildAssetsByIndex()
  filterDuplicateList()
end

local function checkForInvalidLinks(searchPaths)
  stopped = false
  progress = 0

  local updateIntervalCount = 50
  local updateCounter = 0
  local filenames

  invalidLinkFiles = {}
  local totalFolders = #searchPaths

  for folderIndex, folder in ipairs(searchPaths) do
    message = "Gathering link files from: " .. folder
    coroutine.yield()
    filenames = FS:findLinkFiles(folder, -1, true, false)

    for i, filename in ipairs(filenames) do
      updateCounter = updateCounter + 1
      -- update less often since it will slow down the operation otherwise
      if updateCounter > updateIntervalCount then
        local folder, fileNameOnly, ext = path.split(filename)
        message = "Validating link files in: " .. folder
        progress = (folderIndex - 1) / totalFolders
        coroutine.yield()
        updateCounter = 0
      end

      local json = jsonReadFile(filename)

      if json["path"] then
        if not FS:fileExists(json["path"]) then
          editor.logError("Invalid link path for: " .. filename .. " --> " .. json["path"])
          json["linkPath"] = filename
          table.insert(invalidLinkFiles, json)
        end
      else
        editor.logError("Empty link path for: " .. filename)
        json["linkPath"] = filename
        table.insert(invalidLinkFiles, json)
      end

      if stopped then break end
    end

    progress = (folderIndex - 1) / totalFolders
    coroutine.yield()

    if stopped then break end
  end

  message = "Found " .. #invalidLinkFiles .. " invalid link files."
  stopped = true
  progress = 1
end

local function tryFixingInvalidLinks(job)
  stopped = false
  progress = 0

  local updateIntervalCount = 5
  local updateCounter = 0
  local filenames
  local totalInvalidLinkFiles = #invalidLinkFiles
  local fixedCount = 0
  local hash
  local assetsFolderFileHashes = {}

  message = "Gathering file hashes from common /assets folder..."
  coroutine.yield()
  filenames = FS:findFiles("/assets", "*.*", -1, true, false)
  local totalFileCount = #filenames

  for i, filename in ipairs(filenames) do
    updateCounter = updateCounter + 1
    -- update less often since it will slow down the operation otherwise
    if updateCounter > updateIntervalCount then
      local folder, fileNameOnly, ext = path.split(filename)
      message = "Hashing files in: " .. folder
      progress = (i - 1) / totalFileCount
      coroutine.yield()
      updateCounter = 0
    end

    hash = FS:getHashFileAlgorithmId() .. ":" .. FS:hashFile(filename)

    if nil == assetsFolderFileHashes[hash] then
      assetsFolderFileHashes[hash] = filename
    end

    if stopped then break end
    coroutine.yield()
  end

  for i, record in ipairs(invalidLinkFiles) do
    updateCounter = updateCounter + 1
    -- update less often since it will slow down the operation otherwise
    if updateCounter > updateIntervalCount then
      message = "Trying to fix link file: " .. record.linkPath
      progress = (i - 1) / totalInvalidLinkFiles
      coroutine.yield()
      updateCounter = 0
    end

    if assetsFolderFileHashes[record.hash] ~= nil then
      record.fixed = true
      record.path = assetsFolderFileHashes[record.hash]
      fixedCount = fixedCount + 1

      local json = deepcopy(record)
      json.fixed = nil
      json.linkPath = nil
      jsonWriteFile(record.linkPath, json, true)
    end

    if stopped then break end
  end

  message = "Fixed " .. fixedCount .. " invalid link files."
  stopped = true
  progress = 1
end

local function checkForInvalidFilenames(searchPath)
  stopped = false
  progress = 0

  local updateIntervalCount = 10
  local updateCounter = 0
  local filenames

  invalidFilenames = {}
  local depth = -1
  local filter

  if assetsOnlyStr == namingFileTypesFilterStr then
    filter = assetsOnlyFilter
  else
    filter = namingFileTypesFilterStr
  end

  if not recursiveNamingSearchPtr[0] then depth = 0 end

  message = "Gathering file names from: " .. searchPath
  coroutine.yield()
  filenames = FS:findFiles(searchPath, filter, depth, true, true)
  local totalFilenames = #filenames

  for i, filename in ipairs(filenames) do
    updateCounter = updateCounter + 1
    -- update less often since it will slow down the operation otherwise
    if updateCounter > updateIntervalCount then
      local folder, fileNameOnly, ext = path.split(filename)
      message = "Checking naming in: " .. folder
      progress = (i - 1) / totalFilenames
      coroutine.yield()
      updateCounter = 0
    end

    if isFilenameInvalid(filename) then
      local path, filenameOnly, ext = path.split(filename)
      table.insert(invalidFilenames, {filename = filename, suggestedFilename = fixAssetFilename(filenameOnly)})
    end

    coroutine.yield()

    if stopped then break end
  end

  message = "Found " .. #invalidFilenames .. " invalid asset filenames."
  stopped = true
  progress = 1
end

local function checkForInvalidLinksJob(job)
  checkForInvalidLinks(gameDataFolders)
end

local function searchGameForAllDuplicatesJob(job)
  checkForDuplicates(gameDataFolders)
end

local function checkForInvalidFilenamesJob(job)
  checkForInvalidFilenames(searchNamingPath)
end

local function setTargetPathForSelection(targetPath)
  for hash, _ in pairs(selectedHashes) do
    if assetsByHash[hash] then
      assetsByHash[hash].targetPath = changeBasePath(assetsByHash[hash].targetPath, targetPath)
    end
  end
end

local function saveDuplicatesList()
  jsonWriteFile("/temp/assetMigrationList.json", assetsByHash, true)
end

local function loadDuplicatesList()
  assetsByHash = jsonReadFile("/temp/assetMigrationList.json")

  if not assetsByHash then assetsByHash = {} end

  for key, val in pairs(assetsByHash) do
    for _, path in ipairs(val.paths) do
      assets[path] = val.hash
    end
  end

  rebuildAssetsByIndex()
  filterDuplicateList()
end

local function duplicateAssetListUi()
  imgui.Separator()
  imgui.Text(tostring(tableSize(assetsByHash)) .. " asset record(s)")

  local selectedCount = tableSize(selectedHashes)

  if selectedCount ~= 0 then
    imgui.SameLine()
    imgui.Text(", " .. tostring(selectedCount) .. " selected ")
  end

  if imgui.ImGuiTextFilter_IsActive(searchFilter) then
    imgui.SameLine()
    imgui.Text(", " .. tostring(visibleRecordCount) .. " found in filter")
  end

  local noSelection = tableIsEmpty(selectedHashes)

  imgui.Text("Selection operations:")
  imgui.SameLine()

  if imgui.Button("Select All##selectAllRecords") then
    selectedHashes = {}

    for _, hash in ipairs(visibleHashes) do
      selectedHashes[hash] = true
    end
  end

  imgui.SameLine()

  if imgui.Button("Deselect All##deselectAllRecords") then
    selectedHashes = {}
  end

  imgui.SameLine()

  if noSelection then imgui.BeginDisabled() end

  if imgui.Button("Remove Selected") then
    editor.openModalWindow(removeSelectedDlg)
  end

  imgui.SameLine()

  if imgui.Button("Set Target Folder Path:") then
    setTargetPathForSelection(selectionTargetPath)
  end

  imgui.PushItemWidth(400)
  ffi.copy(inputTextNew, selectionTargetPath)
  imgui.SameLine()

  if imgui.InputText("##selectionTartgetPath", inputTextNew, 2048) then
    selectionTargetPath = ffi.string(inputTextNew)
  end

  imgui.PopItemWidth()
  imgui.SameLine()

  if imgui.Button("...##chooseTargetPathForSelection") then
    editor_fileDialog.openFile(function(data)
      if data.path ~= "" then
        local path = data.path
        if path == nil or path == "" then return end
        selectionTargetPath = path
      end
    end, {{"All Folders","*"}}, true, "/assets")
  end
  imgui.tooltip("Pick a target path")

  if noSelection then imgui.EndDisabled() end

  if editor.uiInputSearchTextFilter("Search...", searchFilter, imgui.GetContentRegionAvailWidth()) or forceFilterNow then
    filterDuplicateList()
    forceFilterNow = false
  end

  local tableFlags = bit.bor(imgui.TableFlags_ScrollY,
    imgui.TableFlags_BordersV,
    imgui.TableFlags_BordersOuterH,
    imgui.TableFlags_Resizable,
    imgui.TableFlags_RowBg,
    imgui.TableFlags_NoBordersInBody)

  local colCount = 4

  if imgui.BeginTable('##assetRecordsTable', colCount, tableFlags) then
    imgui.TableSetupScrollFreeze(0, 1) -- Make top row always visible
    imgui.TableSetupColumn("#", imgui.TableColumnFlags_WidthFixed, 0)
    imgui.TableSetupColumn("Current Path(s)", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("New Path", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Hash (" .. FS:getHashFileAlgorithmId() .. ")", imgui.TableColumnFlags_NoHide)
    imgui.TableHeadersRow()

    local delPathIndex
    local delAssetHash

    listClip = imgui.ImGuiListClipper()
    imgui.ImGuiListClipper_Begin(listClip, totalRowCount)

    while imgui.ImGuiListClipper_Step(listClip) do
      for row = listClip.DisplayStart + 1, listClip.DisplayEnd, 1 do
        local mapping = assetRecordRowOffsets[row]
        local record = assetsByIndex[mapping.recordIndex]
        local path = record.paths[mapping.pathIndex] or "No Paths"
        local isFirstPath = mapping.pathIndex == 1
        local hash = record.hash

        imgui.TableNextColumn()

        if isFirstPath then
          imgui.TextColored(imgui.ImVec4(1, 1, 0, 1), tostring(mapping.recordIndex))
          imgui.SameLine()

          selectRecordBoolPtr[0] = selectedHashes[hash] or false

          if imgui.Checkbox("##select" .. tostring(mapping.recordIndex), selectRecordBoolPtr) then
            if selectRecordBoolPtr[0] then
              selectedHashes[hash] = true
            else
              selectedHashes[hash] = nil
            end
          end
        end

        imgui.TableNextColumn()
        imgui.PushID1(tostring(row))
        imgui.PushStyleColor2(imgui.Col_Button, imgui.ImVec4(0.3, 0, 0, 0.5))

        if editor.uiIconImageButton(editor.icons.close, imgui.ImVec2(24, 24)) then
          delAssetHash = hash
          delPathIndex = mapping.pathIndex
        end

        imgui.tooltip("Remove item")
        imgui.PopStyleColor()
        imgui.SameLine()
        imgui.PushItemWidth(-1)
        imgui.Text(path)
        imgui.PopItemWidth()
        imgui.PopID()
        imgui.TableNextColumn()

        if isFirstPath then
          local windowWidth = imgui.GetContentRegionAvail().x
          local inputWidth = windowWidth * 0.9
          local buttonWidth = 40.0

          if inputWidth + buttonWidth > windowWidth then
            inputWidth = windowWidth - buttonWidth
          end

          imgui.PushItemWidth(inputWidth)
          ffi.copy(inputTextNew, record.targetPath)

          if imgui.InputText("##newTxt" .. row, inputTextNew, 2048) then
            record.targetPath = ffi.string(inputTextNew)
          end

          imgui.PopItemWidth()
          imgui.SameLine()

          if imgui.Button("...##chooseTargetPath" .. row) then
            changeTargetPathForHash = hash
            editor_fileDialog.openFile(function(data)
              if data.path ~= "" then
                local path = data.path
                if path == nil or path == "" then return end
                if changeTargetPathForHash then
                  assetsByHash[changeTargetPathForHash].targetPath = changeBasePath(assetsByHash[changeTargetPathForHash].targetPath, path)
                end
              end
            end, {{"All Folders","*"}}, true, "/assets")
          end

          imgui.tooltip("Pick a target path")
        end

        imgui.TableNextColumn()

        if isFirstPath then
          imgui.Text(hash or "")
        end
      end
    end

    imgui.EndTable()

    -- delete the path at local record index
    if nil ~= delPathIndex then
      local record = assetsByHash[delAssetHash]
      assets[record.paths[delPathIndex]] = nil
      table.remove(record.paths, delPathIndex)

      -- if no paths, delete the asset record
      if #record.paths == 0 then
        assetsByHash[record.hash] = nil
        rebuildAssetsByIndex()
      end

      filterDuplicateList()
      delAssetHash = nil
      delPathIndex = nil
    end
  end
end

local function duplicatedAssetsUi()
  if not isGamePathSameAsUserPath() then
    imgui.TextWrapped("ATTENTION: your userpath is not the same as game path, it must be set so the files are moved and link files are created properly to be committed on SVN. You can set it in startup.ini as UserPath = .\\")
    return
  end

  if not devMode then return end

  if imgui.Button("Add Files...") then
    addFiles()
  end
  imgui.SameLine()
  if imgui.Button("Save List") then
    saveDuplicatesList()
    editor.openModalWindow(savedListDlg)
  end
  imgui.tooltip("Save the current list to a json file to process later")
  imgui.SameLine()
  if imgui.Button("Load List") then
    loadDuplicatesList()
  end
  imgui.tooltip("Load list from a json file. Will merge items with the current list")
  imgui.SameLine()
  if imgui.Button("Clear List") then
    editor.openModalWindow(clearDuplicateListDlg)
  end
  imgui.tooltip("Clear the current list")

  imgui.Separator()

  if imgui.Button("Search All Game Duplicates...") then
    editor.openModalWindow(searchAllDuplicatesDlg)
    core_jobsystem.create(searchGameForAllDuplicatesJob)
  end
  imgui.SameLine()
  imgui.Text("File Types:")
  imgui.SameLine()

  if imgui.Combo1("##searchDuplicatesFileTypeCombo", searchDuplicatesFileTypesIndexPtr, searchDuplicatesFileTypesComboItems) then
    local idx = searchDuplicatesFileTypesIndexPtr[0] + 1
    searchFileTypesFilterStr = string.gsub(assetFileTypesTbl[idx], ",", "\t")
  end

  imgui.Separator()

  if imgui.Button("Search duplicates of the assets from the list, in folder:") then
    editor.openModalWindow(searchDuplicatesDlg)
    core_jobsystem.create(searchDuplicatesForList)
  end

  imgui.SameLine()

  ffi.copy(inputTextNew, searchForDuplicatesPath)

  imgui.PushItemWidth(250)

  if imgui.InputText("##searchForDupesPath", inputTextNew, pathSize) then
    searchForDuplicatesPath = ffi.string(inputTextNew)
  end

  imgui.PopItemWidth()
  imgui.SameLine()
  if imgui.Button("...##chooseScanPathForSelection") then
    editor_fileDialog.openFile(function(data)
      if data.path ~= "" then
        local path = data.path
        if path == nil or path == "" then return end
        searchForDuplicatesPath = path
      end
    end, {{"All Folders","*"}}, true, "/")
  end
  imgui.tooltip("Pick a scan path")

  imgui.SameLine()

  if imgui.Checkbox("Recursive##recursiveDuplicateSearch", recursiveDuplicateSearchPtr) then
  end

  imgui.Separator()

  imgui.PushStyleColor2(imgui.Col_Button, imgui.ImVec4(0.5, 0, 0, 1))
  if imgui.Button("S T A R T   M I G R A T I O N") then
    if not skipUnnamedTargetPathsPtr[0] and not allTargetPathsAreValid() then
      editor.openModalWindow(checkDlg)
    else
      editor.openModalWindow(migrationDlg)
      core_jobsystem.create(migrateAssetsJob)
    end
  end
  imgui.PopStyleColor()

  if imgui.Checkbox("Skip " .. newPathNotSetString .. " target paths (otherwise show error if any present in the list)##skipUnnamedTargetPaths", skipUnnamedTargetPathsPtr) then
  end

  duplicateAssetListUi()
end

local function linkCheckerUi()
  if imgui.Button("Check For Invalid Links...") then
    editor.openModalWindow(checkForInvalidLinksDlg)
    core_jobsystem.create(checkForInvalidLinksJob)
  end

  if #invalidLinkFiles > 0 then
    imgui.SameLine()

    if imgui.Button("Delete All Invalid Link Files") then
      editor.openModalWindow(deleteInvalidLinksDlg)
    end

    imgui.SameLine()

    if imgui.Button("Try Fixing Invalid Links...") then
      editor.openModalWindow(tryFixingInvalidLinksDlg)
      core_jobsystem.create(tryFixingInvalidLinks)
    end
  end

  local tableFlags = bit.bor(imgui.TableFlags_ScrollY,
    imgui.TableFlags_BordersV,
    imgui.TableFlags_BordersOuterH,
    imgui.TableFlags_Resizable,
    imgui.TableFlags_RowBg,
    imgui.TableFlags_NoBordersInBody)

  local colCount = 6

  if imgui.BeginTable('##linksRecordsTable', colCount, tableFlags) then
    imgui.TableSetupScrollFreeze(0, 1) -- Make top row always visible
    imgui.TableSetupColumn("#", imgui.TableColumnFlags_WidthFixed, 0)
    imgui.TableSetupColumn("Path", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Target Path", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Status", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Hash", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Date", imgui.TableColumnFlags_NoHide)
    imgui.TableHeadersRow()
    listClip = imgui.ImGuiListClipper()
    imgui.ImGuiListClipper_Begin(listClip, #invalidLinkFiles)

    while imgui.ImGuiListClipper_Step(listClip) do
      for idx = listClip.DisplayStart + 1, listClip.DisplayEnd, 1 do
        local record = invalidLinkFiles[idx]

        imgui.TableNextColumn()
        imgui.TextColored(imgui.ImVec4(1, 1, 0, 1), tostring(idx))

        imgui.TableNextColumn()
        imgui.Text(record.linkPath or "ERROR")

        imgui.TableNextColumn()
        imgui.Text(record.path or "NONE")

        imgui.TableNextColumn()

        if record.fixed then
          imgui.TextColored(imgui.ImVec4(0, 1, 0, 1), "FIXED")
        else
          imgui.TextColored(imgui.ImVec4(1, 0, 0, 1), "INVALID")
        end

        imgui.TableNextColumn()
        imgui.Text(record.hash or "NONE")

        imgui.TableNextColumn()
        imgui.Text(record.time or "NOT SET")
      end
    end
    imgui.EndTable()
  end
end

local function assetNamingCheckerUi()
  imgui.Text("Naming check scan path:")
  imgui.SameLine()

  ffi.copy(inputTextNew, searchNamingPath)
  imgui.PushItemWidth(250)

  if imgui.InputText("##searchNamingPath", inputTextNew, pathSize) then
    searchNamingPath = ffi.string(inputTextNew)
  end

  imgui.PopItemWidth()
  imgui.SameLine()
  if imgui.Button("...##chooseScanPathForNaming") then
    editor_fileDialog.openFile(function(data)
      if data.path ~= "" then
        local path = data.path
        if path == nil or path == "" then return end
        searchNamingPath = path
      end
    end, {{"All Folders", "*"}}, true, "/")
  end
  imgui.tooltip("Pick a scan path")

  imgui.SameLine()

  if imgui.Checkbox("Recursive##recursiveNamingSearch", recursiveNamingSearchPtr) then
  end

  imgui.Text("File Types:")
  imgui.SameLine()

  if imgui.Combo1("##namingFileTypeCombo", namingFileTypesIndexPtr, namingFileTypesComboItems) then
    local idx = namingFileTypesIndexPtr[0] + 1
    namingFileTypesFilterStr = string.gsub(assetFileTypesTbl[idx], ",", "\t")
  end

  if imgui.Button("Check For Invalid Asset Filenames...") then
    editor.openModalWindow(checkForInvalidFilenamesDlg)
    core_jobsystem.create(checkForInvalidFilenamesJob)
  end

  local tableFlags = bit.bor(imgui.TableFlags_ScrollY,
    imgui.TableFlags_BordersV,
    imgui.TableFlags_BordersOuterH,
    imgui.TableFlags_Resizable,
    imgui.TableFlags_RowBg,
    imgui.TableFlags_NoBordersInBody)

  local colCount = 3

  if imgui.BeginTable('##linksRecordsTable', colCount, tableFlags) then
    imgui.TableSetupScrollFreeze(0, 1) -- Make top row always visible
    imgui.TableSetupColumn("#", imgui.TableColumnFlags_WidthFixed, 0)
    imgui.TableSetupColumn("Path", imgui.TableColumnFlags_NoHide)
    imgui.TableSetupColumn("Suggested Rename", imgui.TableColumnFlags_NoHide)
    imgui.TableHeadersRow()

    listClip = imgui.ImGuiListClipper()
    imgui.ImGuiListClipper_Begin(listClip, #invalidFilenames)

    while imgui.ImGuiListClipper_Step(listClip) do
      for idx = listClip.DisplayStart + 1, listClip.DisplayEnd, 1 do
        local record = invalidFilenames[idx]

        imgui.TableNextColumn()
        imgui.TextColored(imgui.ImVec4(1, 1, 0, 1), tostring(idx))

        imgui.TableNextColumn()
        imgui.Text(record.filename or "ERROR")

        imgui.TableNextColumn()
        imgui.Text(record.suggestedFilename or "NONE")

        imgui.SameLine()
        imgui.Spacing()
        imgui.SameLine()
        if imgui.Button("Copy##btnCopyFilename" .. idx) then
          local folder, fileNameOnly, ext = path.split(record.suggestedFilename)
          imgui.SetClipboardText(fileNameOnly)
        end
      end
    end
    imgui.EndTable()
  end
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName, "Asset Management Tool") then
    if imgui.BeginTabBar("Actions") then
      if devMode then
        if imgui.BeginTabItem("Duplicated Assets") then
          duplicatedAssetsUi()
          imgui.EndTabItem()
        end
      end

      if imgui.BeginTabItem("Link Checker") then
        linkCheckerUi()
        imgui.EndTabItem()
      end

      if imgui.BeginTabItem("Naming Checker") then
        assetNamingCheckerUi()
        imgui.EndTabItem()
      end

      imgui.EndTabBar()
    end
  end
  editor.endWindow()

  if editor.beginModalWindow(removeSelectedDlg, "Asset Management - Remove Selected") then
    imgui.Spacing()
    imgui.Text("Remove the selected assets from list ? (wont delete any file, cannot undo!)")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()
    imgui.Spacing()
    imgui.Spacing()

    if imgui.Button("Yes") then
      for hash, _ in pairs(selectedHashes) do
        local asset = assetsByHash[hash]
        -- remove the paths from the global list
        if asset and asset.paths then
          for _, path in ipairs(asset.paths) do
            assets[path] = nil
          end
        end
        -- remove the asset record
        assetsByHash[hash] = nil
      end

      rebuildAssetsByIndex()
      selectedHashes = {}
      filterDuplicateList()
      editor.closeModalWindow(removeSelectedDlg)
    end

    imgui.SameLine()

    if imgui.Button("Cancel") then
      editor.closeModalWindow(removeSelectedDlg)
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(clearDuplicateListDlg, "Asset Management - Clear Duplicates List") then
    imgui.Spacing()
    imgui.Text("Clear the asset duplicates list ?")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()
    imgui.Spacing()
    imgui.Spacing()

    if imgui.Button("Yes") then
      clearEverything()
      rebuildAssetsByIndex()
      selectedHashes = {}
      editor.closeModalWindow(clearDuplicateListDlg)
    end

    imgui.SameLine()

    if imgui.Button("Cancel") then
      editor.closeModalWindow(clearDuplicateListDlg)
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(deleteInvalidLinksDlg, "Asset Management - Delete Invalid Link Files") then
    imgui.Spacing()
    imgui.Text("Delete the invalid link files ? (Cannot undo)")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()
    imgui.Spacing()
    imgui.Spacing()

    if imgui.Button("Yes") then
      deleteAllInvalidLinkFiles()
      editor.closeModalWindow(deleteInvalidLinksDlg)
    end

    imgui.SameLine()

    if imgui.Button("Cancel") then
      editor.closeModalWindow(deleteInvalidLinksDlg)
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(migrationDlg, "Asset Management - Migration") then
    imgui.Spacing()
    imgui.Text("INFO: This will copy the first file in the lists to the new path\nONLY if the new file doesnt exists, it will not overwrite it.")
    imgui.Text("Migrating assets...")
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(migrationDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(migrationDlg)
      end
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(searchDuplicatesDlg, "Asset Management - Find Duplicates") then
    imgui.Spacing()
    imgui.Text("ATTENTION: Game must be in focus, otherwise the process is paused.")
    imgui.Text("Searching for duplicates of the assets in the list, inside:" .. searchForDuplicatesPath)
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(searchDuplicatesDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(searchDuplicatesDlg)
      end
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(searchAllDuplicatesDlg, "Asset Management - Find All Global Duplicates") then
    imgui.Spacing()
    imgui.Text("ATTENTION: Game must be in focus, otherwise the process is paused.")
    imgui.Text("Searching for all game duplicates...")
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(searchAllDuplicatesDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(searchAllDuplicatesDlg)
      end
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(checkForInvalidLinksDlg, "Asset Management - Check for invalid links") then
    imgui.Spacing()
    imgui.Text("ATTENTION: Game must be in focus, otherwise the process is paused.")
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(checkForInvalidLinksDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(checkForInvalidLinksDlg)
      end
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(tryFixingInvalidLinksDlg, "Asset Management - Try fixing invalid links") then
    imgui.Spacing()
    imgui.Text("ATTENTION: Game must be in focus, otherwise the process is paused.")
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(tryFixingInvalidLinksDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(tryFixingInvalidLinksDlg)
      end
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(checkForInvalidFilenamesDlg, "Asset Management - Check for invalid filenames") then
    imgui.Spacing()
    imgui.Text("ATTENTION: Game must be in focus, otherwise the process is paused.")
    imgui.Text(message)
    imgui.ProgressBar(progress, imgui.ImVec2(imgui.GetContentRegionAvailWidth(), 0))
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if not stopped then
      if imgui.Button("Abort") then
        stopped = true
        editor.closeModalWindow(checkForInvalidFilenamesDlg)
      end
    else
      if imgui.Button("Close") then
        editor.closeModalWindow(checkForInvalidFilenamesDlg)
      end
    end
  end
  editor.endModalWindow()


  if editor.beginModalWindow(checkDlg, "Asset Management - Migration") then
    imgui.Spacing()
    imgui.Text("ERROR: You still have new target paths with the default text: '" .. newPathNotSetString .. "', please set a proper path.")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if imgui.Button("OK") then
      editor.closeModalWindow(checkDlg)
    end
  end
  editor.endModalWindow()

  if editor.beginModalWindow(savedListDlg, "Asset Management - Saved List") then
    imgui.Spacing()
    imgui.Text("The asset list was saved.")
    imgui.Spacing()
    imgui.Separator()
    imgui.Spacing()

    if imgui.Button("OK") then
      editor.closeModalWindow(savedListDlg)
    end
  end
  editor.endModalWindow()
end

local function onEditorActivated()
end

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, imgui.ImVec2(1200, 900))
  editor.registerModalWindow(removeSelectedDlg, imgui.ImVec2(400, 100))
  editor.registerModalWindow(migrationDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(checkDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(searchDuplicatesDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(searchAllDuplicatesDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(savedListDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(checkForInvalidLinksDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(checkForInvalidFilenamesDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(deleteInvalidLinksDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(tryFixingInvalidLinksDlg, imgui.ImVec2(600, 200))
  editor.registerModalWindow(clearDuplicateListDlg, imgui.ImVec2(600, 200))
  editor.addWindowMenuItem("Asset Management Tool", onWindowMenuItem, {groupMenuName = 'Assets'})

  loadDuplicatesList()
end

local function onExtensionLoaded()
end

M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorGui = onEditorGui
M.onExtensionLoaded = onExtensionLoaded

return M