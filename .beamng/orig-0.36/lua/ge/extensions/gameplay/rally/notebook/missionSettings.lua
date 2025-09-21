-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')

local C = {}
local logTag = ''

local defaultSettings = {
  notebook = {
    filename = rallyUtil.defaultNotebookFname,
    -- filename = nil
    -- codriverName = rallyUtil.defaultCodriverName,
    codriverId = nil,
  },
  drivelineMode = RallyEnums.drivelineMode.route,
}

function C:init(missionDir)
  self.missionDir = missionDir
  if not self.missionDir then
    error('missionDir is nil')
    return
  end

  self.notebook = nil
  self.drivelineMode = nil
end

function C:fname()
  return rallyUtil.getMissionSettingsFile(self.missionDir)
end

local function listNotebooks(folder)
  local notebooksFullPath = folder..'/'..rallyUtil.notebooksPath
  local paths = {}
  -- log('I', logTag, 'loading all notebook names from '..notebooksFullPath)
  local files = FS:findFiles(notebooksFullPath, '*.notebook.json', -1, true, false)
  for _,fname in pairs(files) do
    table.insert(paths, fname)
  end
  table.sort(paths)

  log("D", logTag, dumps(paths))

  return paths
end

function C:load(notebook)
  if not self.missionDir and notebook then
    self.missionDir = notebook:getMissionDir()
  end

  -- create the mission settings file if it doesnt exist
  if not FS:fileExists(self:fname()) then
    -- log('D', logTag, "mission settings file not found: "..self:fname())
    -- log('D', logTag, "creating mission settings file")
    -- return false
    self.notebook = deepcopy(defaultSettings.notebook)
    self.drivelineMode = defaultSettings.drivelineMode
    self:write()
  end

  -- read the mission settings file
  local newMissionSettingsData = jsonReadFile(self:fname())
  if not newMissionSettingsData then
    log('E', logTag, 'unable to read mission settings: ' .. tostring(self:fname()))
    return false
  end

  -- check the notebook filename
  -- if the setting was missing, set it to the first existing notebook or the default one.

  local notebookFname = rallyUtil.getNotebookFullPath(self.missionDir, newMissionSettingsData.notebook.filename)
  if not newMissionSettingsData.notebook.filename or not FS:fileExists(notebookFname) then
    -- log('D', logTag, 'missionSetting for notebook.filename not found in mission settings file at '..tostring(self:fname()))
    local existingNotebooks = listNotebooks(self.missionDir)
    if #existingNotebooks > 0 then
      -- log('D', logTag, 'setting notebook.filename to first existing notebook: '..existingNotebooks[1])
      local _, basename, _ = path.split(existingNotebooks[1])
      self:setNotebookFilename(basename)
    else
      -- log('D', logTag, 'setting notebook.filename to default: '..rallyUtil.defaultNotebookFname)
      self:setNotebookFilename(rallyUtil.defaultNotebookFname)
    end
  end

  -- re-read the mission settings file
  local newMissionSettingsData = jsonReadFile(self:fname())
  if not newMissionSettingsData then
    log('E', logTag, 'unable to read mission settings: ' .. tostring(self:fname()))
    return false
  end

  -- check if the notebook file exists
  if not FS:fileExists(self:fname()) then
    -- log('D', logTag, 'notebook.filename setting not found: ' .. tostring(newMissionSettingsData.notebook.filename))
    -- log('D', logTag, 'setting notebook.filename to default: '..rallyUtil.defaultNotebookFname)
    self:setNotebookFilename(rallyUtil.defaultNotebookFname)
  end

  -- re-read the mission settings file
  local newMissionSettingsData = jsonReadFile(self:fname())
  if not newMissionSettingsData then
    log('E', logTag, 'unable to read mission settings: ' .. tostring(self:fname()))
    return false
  end

  -- probably can get rid of this next time you come across it.
  -- check if the codriver name is valid
  -- if notebook then
  --   local loadedCodriverName = newMissionSettingsData.notebook.codriverName
  --   log('D', logTag, 'notebook was passed during missionSettings load, checking codriver name against notebooks codrivers')
  --   local checkCodriverName = notebook:getCodriverByName(loadedCodriverName)
  --   log('D', logTag, 'missionSettings.codriver.name='..tostring(loadedCodriverName)..' notebook.codriver.name='..tostring(checkCodriverName and checkCodriverName.name or 'nil'))
  --   if not checkCodriverName then
  --     log('W', logTag, 'missionSettings: codriver name not found in notebook: ' .. tostring(loadedCodriverName))
  --     log('D', logTag, 'setting codriver name to first codriver: '..notebook:getFirstCodriver().name)
  --     self:setCodriverName(notebook:getFirstCodriver().name)
  --   end
  -- end

  if notebook then
    local loadedCodriverId = newMissionSettingsData.notebook.codriverId
    -- log('D', logTag, 'notebook was passed during missionSettings load, checking codriver id against notebooks codrivers')
    local checkCodriverId = notebook:getCodriverById(loadedCodriverId)
    -- log('D', logTag, 'missionSettings.codriver.id='..tostring(loadedCodriverId)..' notebook.codriver.id='..tostring(checkCodriverId and checkCodriverId.id or 'nil'))
    if not checkCodriverId then
      -- log('D', logTag, 'missionSettings: codriver id not found in notebook: ' .. tostring(loadedCodriverId))
      -- log('D', logTag, 'setting codriver id to first codriver: '..notebook:getFirstCodriver().id)
      self:setCodriverId(notebook:getFirstCodriver().pk)
    end
  end

  -- re-read the mission settings file
  local newMissionSettingsData = jsonReadFile(self:fname())
  if not newMissionSettingsData then
    log('E', logTag, 'unable to read mission settings: ' .. tostring(self:fname()))
    return false
  end

  -- clear legacy setting.
  newMissionSettingsData.notebook.codriver = nil

  self:onDeserialized(newMissionSettingsData)
  -- log('I', logTag, 'loaded rally mission settings from ' .. tostring(self:fname()))

  return true
end

function C:loadNotebook()
  -- log('I', logTag, 'loading notebook')
  local notebook = rallyUtil.loadNotebookForMissionDir(self.missionDir, self:getNotebookFilename())

  -- if not notebook then
    -- log('E', logTag, 'unable to load notebook: ' .. tostring(err))
    -- return nil
  -- end
  return notebook
end

function C:loadRace()
  -- log('I', logTag, 'loading race')
  local race, err = rallyUtil.loadRace(self.missionDir)
  if err then
    log('E', logTag, 'unable to load race: ' .. tostring(err))
    return nil
  end
  return race
end

function C:onSerialize()
  local ret = {
    notebook = self.notebook,
    drivelineMode = self.drivelineMode,
  }

  return ret
end

function C:onDeserialized(data)
  if not data then return end
  self.notebook = data.notebook or deepcopy(defaultSettings.notebook)
  self.drivelineMode = data.drivelineMode or defaultSettings.drivelineMode
end

function C:write()
  local json = self:onSerialize()
  jsonWriteFile(self:fname(), json, true)
end

function C:setCodriverId(codriverId)
  self.notebook = self.notebook or deepcopy(defaultSettings.notebook)
  self.notebook.codriverId = codriverId
  self:write()
end

function C:getCodriverId()
  return self.notebook.codriverId
end

function C:getCodriverPk()
  return self.notebook.codriverId
end

function C:setNotebookFilename(filename)
  self.notebook = self.notebook or deepcopy(defaultSettings.notebook)
  self.notebook.filename = filename
  self:write()
end

function C:getNotebookFilename()
  return self.notebook.filename
end

function C:getNotebookFullPath()
  return rallyUtil.getNotebookFullPath(self.missionDir, self:getNotebookFilename())
end

function C:setDrivelineMode(drivelineMode)
  self.drivelineMode = drivelineMode
  print('setDrivelineMode: '..tostring(drivelineMode))
  self:write()
end

function C:getDrivelineMode()
  return self.drivelineMode
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end