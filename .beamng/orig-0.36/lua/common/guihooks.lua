-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local currentVehicle -- table ptr

local cache = {}
local tmpTab = {}

local playerInfo = playerInfo or {firstPlayerSeated = true}
local vehicleLuaSpecific = nop

local queueHookJS --  queueHookJS(const char* hookName, const char* jsData, unsigned int targetDsmId) - the varargs are encoded as list in jsData
local queueStreamDataJS -- queueStreamDataJS(const char* streamName, const char* jsData)
if obj then
  queueHookJS = function(...) obj:queueHookJS(...) end
  queueStreamDataJS = function(...) obj:queueStreamDataJS(...) end
elseif be then
  queueHookJS = function(...) be:queueHookJS(...) end
  queueStreamDataJS = function(...) be:queueStreamDataJS(...) end
end

-- Garbage sensitive, do not change if you don't know what garbage is
local function trigger(hookName, ...)
  local i1 = 0
  for i = 1, select('#', ...) do
    local tv = select(i, ...)
    if tv ~= nil then
      i1 = i1 + 1
      tmpTab[i1] = tv
    end
  end

  queueHookJS(hookName, i1 == 0 and '[]' or jsonEncodeWorkBuffer(tmpTab), execCtxWebId or 0)
  table.clear(tmpTab)
end

-- Garbage sensitive, do not change if you don't know what garbage is
local function triggerClient(targetDsmId, hookName, ...)
  local i1 = 0
  for i = 1, select('#', ...) do
    local tv = select(i, ...)
    if tv ~= nil then
      i1 = i1 + 1
      tmpTab[i1] = tv
    end
  end
  queueHookJS(hookName, i1 == 0 and '[]' or jsonEncodeWorkBuffer(tmpTab), targetDsmId)
  table.clear(tmpTab)
end

local function triggerRawJS(hookName, rawJs)
  queueHookJS(hookName, concatWorkBuffer('[', tostring(rawJs), ']'))
end

local function triggerStream(streamName, streamData)
  queueStreamDataJS(streamName, jsonEncodeWorkBuffer(streamData))
end

local function checkStreamsAndVehicle()
  streams.update()

  --vehicleChange
  if currentVehicle ~= v.data then
    currentVehicle = v.data
    trigger("VehicleChange", v.data.vehicleDirectory)
    trigger("VehicleReset", 0)
  end
end

if obj then
  vehicleLuaSpecific = checkStreamsAndVehicle
end

-- WARNING: this can only be called from vehicle onGraphicsStep
local function sendStreams()
  vehicleLuaSpecific()
  for streamName, data in pairs(cache) do
    queueStreamDataJS(streamName, jsonEncodeWorkBuffer(data))
  end
  table.clear(cache)
end

local function reset()
  M.updateStreams = false
  table.clear(cache)

  if playerInfo.firstPlayerSeated then
    trigger('VehicleReset')
  end
end

-- cache data to be send later
local function queueStream(key, value)
  if M.updateStreams then
    cache[key] = value
  end
end

-- todo replace ui_message
-- instead message should directly call the hook
-- in light of different message types emerging it might be interesting to have a seperate message module
local function message(msg, ttl, category, icon)
  if not playerInfo.firstPlayerSeated then return end
  trigger('Message', {msg = msg, ttl = (ttl or 5), category = (category or ''), icon = icon})
end

-- UI app graph. accepts any amount of arguments, each argument can define in this order: { key, value, scale, unit, renderNegatives, color }
-- for example: graph( {"foo",foo},  {"bar",bar,100},  {"baz",baz,nil,"m/s",false,nil} )
local function graph(a, ...)
  local values = a and {a, ...} or {...} -- if first value is nil/false/etc, skip it
  local numOfSteps = #values
  for i,v in ipairs(values) do
    v[1] = v[1] or string.format("#%i", i) -- key
    v[2] = v[2] or 0 -- value
    v[3] = v[3] or 1 -- scale
    v[4] = v[4] or "" -- unit
    v[5] = v[5] or false -- renderNegatives
    v[6] = v[6] or { colorGetRGBA(jetColor(i / numOfSteps)) }
  end
  queueStream('genericGraphSimple', values)
  return values
end

local csvfile, csvfilename = nil, nil
local function graphWithCSV(filename, ...)
  local values = graph(...)
  if not csvfile then
    local keys = {}
    for _,v in ipairs(values) do
      table.insert(keys, string.format(v[4]=="" and "%s%s" or "%s (%s)", v[1], v[4]))
    end
    csvfile = require('csvlib').newCSV(unpack(keys))
    csvfilename = filename or string.format("graphcsv.%s.csv", os.date("%Y-%d-%mT%H_%M_%S"))
  end
  if csvfile then
    local row = {}
    for _,v in ipairs(values) do
      table.insert(row, v[2])
    end
    csvfile:add(unpack(row))
  end
end
local function graphWithCSVWrite()
  if csvfile then
    csvfile:write(csvfilename)
  else
    log("E", "", "No csvfile to write to: "..dumps(csvfile)..", "..dumps(csvfilename))
  end
end

-- public interface
M.reset = reset
M.sendStreams = sendStreams
-- WARNING: this can currently only be called from vehicle lua side. from ge this will break

M.trigger = trigger
M.triggerClient = triggerClient
M.triggerStream = triggerStream

M.triggerRawJS = triggerRawJS

M.queueStream = queueStream
M.send = queueStream

-- UI messaging shortcut
M.message = message
M.graph = graph -- 'Generic Graph' UI app
M.graphWithCSV = graphWithCSV -- 'Generic Graph' UI app
M.graphWithCSVWrite = graphWithCSVWrite -- 'Generic Graph' UI app

return M
