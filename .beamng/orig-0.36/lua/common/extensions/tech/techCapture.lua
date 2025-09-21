-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'TechCapture'

local tcom = require('tech/techCommunication')

local TCOM_CAPTURE_PREFIX = 'tcomCapture'

local LUA_CONTEXT = obj == nil and 'GE' or tostring(obj:getID())
local tcomCaptureName = nil
local tcomRequestFile = nil -- the file stores all the received commands to be debuggable/replayable
local tcomResponseFile = nil

local captureRequests = false
local captureResponses = false
-- limits the time between I/O flushes
--  <=0 = flush after every command
--  > 0 = flush the next request/response after n seconds since the last flush of any type
--  nil = never flush, count on I/O or clean close of the file
-- Default is set to 0 to be on the safe side during debugging. If you are recording a trace for
-- reproducibility reasons and you know that you will close the file, you can leave it set to nil.
local _flushInterval = nil

local lastFlush = os.clockhp()

-- Helper functions

-- The function takes a path of the form /path/to/file.<ext> or /path/to/file.<XXX>.<ext> and returns
-- all existing files of the form /path/to/file.<ext> (with the `completeMatch` argument)
-- and /path/to/file.<XXX>.<ext> (with the `intermediateMatch` argument).
local function getAllRelatedFiles(pathToFile, completeMatch, intermediateMatch)
  local dirname, baseFilename, extension = path.splitWithoutExt(pathToFile)
  baseFilename = baseFilename:gmatch("([^%.]+)")() -- change file.<XXX> into file
  if dirname == nil then dirname = '/' end
  local files = FS:findFiles(dirname, baseFilename .. '*.' .. extension, 0, true, false)
  local filteredFiles = {}
  for i, file in pairs(files) do -- this is a more fine-grained filter to preserve files with similar capture names
    local _, filename = path.split(file)
    local filenameLower = filename:lower()
    local interPattern1 = '^' .. baseFilename .. '%.[%d]+%.' .. extension .. '$'
    interPattern1 = interPattern1:lower()
    local interPattern2 = '^' .. baseFilename .. '%.GE%.' .. extension .. '$'
    interPattern2 = interPattern2:lower()
    local compPattern = '^' .. baseFilename .. '%.' .. extension .. '$'
    compPattern = compPattern:lower()

    if intermediateMatch and filenameLower:match(interPattern1) then
      table.insert(filteredFiles, dirname ..filename)
    elseif intermediateMatch and filenameLower:match(interPattern2) then
      table.insert(filteredFiles, dirname .. filename)
    elseif completeMatch and filenameLower:match(compPattern) then
      table.insert(filteredFiles, dirname .. filename)
    end
  end

  return filteredFiles
end

local function getCaptureTypeFromFile(inputFilename)
  local inputFile, err = io.open(inputFilename, 'r')

  if inputFile == nil then
    log('E', logTag, 'Couldn\'t open ' .. inputFilename .. ' for reading. Original error: ' .. err)
    return
  end
  local line = inputFile:read()
  inputFile:close()
  if line == 'TECH CAPTURE v1 COMPLETE' then
    return 'REQUEST', 'COMPLETE'
  end
  if line == 'TECH CAPTURE v1 INTERMEDIATE' then
    return 'REQUEST', 'INTERMEDIATE'
  end
  if line == 'TECH RESPONSE v1 COMPLETE' then
    return 'RESPONSE', 'COMPLETE'
  end
  if line == 'TECH RESPONSE v1 INTERMEDIATE' then
    return 'RESPONSE', 'INTERMEDIATE'
  end

  return nil
end

local function filterFilesByHeader(files, captureType, captureMerged)
  local filteredFiles = {}
  for i, file in ipairs(files) do
    local fileType, fileMerged = getCaptureTypeFromFile(file)
    if captureType ~= nil and captureMerged ~= nil then
      if fileType == captureType and fileMerged == captureMerged then
        table.insert(filteredFiles, file)
      end
    elseif fileType and fileType == captureType then
      table.insert(filteredFiles, file)
    elseif fileMerged and fileMerged == captureMerged then
      table.insert(filteredFiles, file)
    end
  end
  return filteredFiles
end

local function deletePreviousCaptureWithSameName(capturePath, captureType)
  local files = getAllRelatedFiles(capturePath, true, true)
  files = filterFilesByHeader(files, captureType)
  for _, file in ipairs(files) do
    log('D', logTag, 'Deleting ' .. file .. '.')
    FS:removeFile(file)
  end
end

local function buildCaptureFilename(name)
  return name .. '.' .. LUA_CONTEXT .. '.log'
end

local function convertBinaryDataToString(data)
  local t = type(data)
  if t == 'userdata' then
    return 'userdata'
  elseif t == 'table' then
    for k, v in pairs(data) do
      data[k] = convertBinaryDataToString(v)
    end
  end

  return data
end

local function prepareFileFromCaptureName(captureName, captureType)
  if captureName ~= nil then
    tcomCaptureName = captureName
  end
  if tcomCaptureName == nil then
    tcomCaptureName = TCOM_CAPTURE_PREFIX
  end

  if LUA_CONTEXT == 'GE' then
    deletePreviousCaptureWithSameName(tcomCaptureName .. '.log', captureType)
  end
  local filename = buildCaptureFilename(tcomCaptureName)
  return filename
end

-- The below functions write the BeamNG REQUESTS to the output log file provided

local function openRequestFile(captureName)
  if tcomRequestFile then
    M.closeRequestFile()
  end

  local filename = prepareFileFromCaptureName(captureName, 'REQUEST')
  local err
  tcomRequestFile, err = io.open(filename, 'w')
  if tcomRequestFile == nil then
    log('E', logTag, 'Couldn\'t open ' .. filename .. ' for writing. Original error: ' .. err)
  else
    tcomRequestFile:write('TECH CAPTURE v1 INTERMEDIATE\n')
  end

  return filename
end

local function isRecordingRequests()
  return captureRequests and not captureResponses and tcomRequestFile
end

local function isRecordingResponses()
  return captureResponses and tcomResponseFile
end

local function recordRequest(request)
  if not isRecordingRequests() then return false end

  local requestData = {}
  for key, value in pairs(request) do
    if key ~= 'skt' and key ~= 'handled' then
      requestData[key] = value
    end
  end

  -- log('D', logTag, 'Handling request: ', requestData)
  local clock = os.clockhp()
  local json = jsonEncode(requestData)
  tcomRequestFile:write(tostring(clock), '\n', LUA_CONTEXT, '\n', json, '\n')

  if _flushInterval == nil then return true end
  if _flushInterval <= 0 or (clock - lastFlush) > _flushInterval then
    tcomRequestFile:flush()
    lastFlush = clock
  end

  return true
end

local function closeRequestFile()
  if tcomRequestFile ~= nil then
    tcomRequestFile:flush()
    tcomRequestFile:close()
  end
  tcomRequestFile = nil
  tcomCaptureName = nil
end

local function openResponseFile(captureName)
  if tcomResponseFile then
    M.closeResponseFile()
  end

  local filename = prepareFileFromCaptureName(captureName, 'RESPONSE')
  local err
  tcomResponseFile, err = io.open(filename, 'w')
  if tcomResponseFile == nil then
    log('E', logTag, 'Couldn\'t open ' .. filename .. ' for writing. Original error: ' .. err)
  else
    tcomResponseFile:write('TECH RESPONSE v1 INTERMEDIATE\n')
  end
end

local function recordResponse(response)
  if not isRecordingResponses() then return end

  log('D', logTag, 'Handling response [' .. response.type .. '].')
  local clock = os.clockhp()
  response = convertBinaryDataToString(response)
  local json = jsonEncode(response)
  tcomResponseFile:write(tostring(clock), '\n', LUA_CONTEXT, '\n', json, '\n')
  if _flushInterval == nil then return end
  if _flushInterval <= 0 or (clock - lastFlush) > _flushInterval then
    tcomResponseFile:flush()
    lastFlush = clock
  end
end

local function closeResponseFile()
  if tcomResponseFile ~= nil then
    tcomResponseFile:flush()
    tcomResponseFile:close()
  end
  tcomResponseFile = nil
  tcomCaptureName = nil
end

-- TechCaptureRequest writes the BeamNG RESPONSES to the output log file provided
-- and it is a mock for the real network requests (see `techCommunication.lua`).
local TechCaptureRequest = {}

function TechCaptureRequest:new(o, callback)
  o = o or {}
  setmetatable(o, self)
  self.__index = self
  self.response = nil
  self._callback = callback
  return o
end

function TechCaptureRequest:markHandled() end

function TechCaptureRequest:sendResponse(message)
  message['_id'] = self['_id']
  local error = message.bngError or message.bngValueError
  if error then
    log('E', logTag, 'Error in response [' .. tostring(message._id) .. ']: ' .. error)
  elseif self._callback then
    self._callback(self, message) -- this can mutate the message
  end
  self.response = message
  M.recordResponse(message)
end

function TechCaptureRequest:sendACK(type)
  local message = {}
  if type ~= nil then
    message.type = type
  end
  self:sendResponse(message)
end

function TechCaptureRequest:sendBNGError(message)
  self:sendResponse({bngError = message})
end

function TechCaptureRequest:sendBNGValueError(message)
  self:sendResponse({bngValueError = message})
end

local function enableRequestCapture(captureName, flushInterval)
  if flushInterval ~= nil then
    _flushInterval = flushInterval
  end
  tcom.enableDebug()
  captureRequests = true
  captureName = openRequestFile(captureName)
  log('I', logTag, 'Recording requests to ' .. captureName .. '.')
end

local function disableRequestCapture()
  captureRequests = false
  closeRequestFile()
end

local function enableResponseCapture(captureName)
  captureResponses = true
  if captureName then
    openResponseFile(captureName)
    log('I', logTag, 'Recording responses to ' .. captureName .. '.log.')
  end
end

local function disableResponseCapture()
  captureResponses = false
  closeResponseFile()
end

local function injectMessage(payload, callback)
  local request = TechCaptureRequest:new(payload, callback)
  local processMore = tcom.callRequestHandler(tech_techCore, request)
  return request, processMore
end

local function export()
  return {
    captureName = tcomCaptureName,
    captureRequests = captureRequests,
    captureResponses = captureResponses,
    flushInterval = _flushInterval
  }
end

local function import(info)
  tcomCaptureName = info.captureName
  captureRequests = info.captureRequests
  captureResponses = info.captureResponses
  _flushInterval = info.flushInterval

  if captureRequests then
    openRequestFile(tcomCaptureName)
  end
  if captureResponses then
    openResponseFile(tcomCaptureName)
  end
end

local function onInit()
  setExtensionUnloadMode(M, 'manual')
end

M.onInit = onInit

M.getAllRelatedFiles = getAllRelatedFiles
M.getCaptureTypeFromFile = getCaptureTypeFromFile
M.filterFilesByHeader = filterFilesByHeader

M.openRequestFile = openRequestFile
M.recordRequest = recordRequest
M.closeRequestFile = closeRequestFile
M.isRecordingRequests = isRecordingRequests

M.openResponseFile = openResponseFile
M.recordResponse = recordResponse
M.closeResponseFile = closeResponseFile
M.isRecordingResponses = isRecordingResponses

M.enableRequestCapture = enableRequestCapture
M.disableRequestCapture = disableRequestCapture
M.enableResponseCapture = enableResponseCapture
M.disableResponseCapture = disableResponseCapture
M.injectMessage = injectMessage

M.export = export
M.import = import

return M