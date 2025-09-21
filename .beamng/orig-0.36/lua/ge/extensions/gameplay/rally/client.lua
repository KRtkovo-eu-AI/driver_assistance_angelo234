-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local httpClient = require("socket.http")
-- local socket = require("socket")
-- socket.TIMEOUT = 0
httpClient.TIMEOUT = 0.5

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local M = {}

local logTag = ''
local base_url = 'http://localhost:27872'

local function jsonRequestGet(uri)
  local respbody = {}
  local body, code, headers, status = httpClient.request {
    method = 'GET',
    url = uri,
    sink = ltn12.sink.table(respbody)
  }

  if code ~= 200 then
    log('E', logTag, 'rally client error: '..dumps(code))
    return {ok = false, error = tostring(code)}
  else
    --print('body:' .. tostring(body))
    --print('code:' .. tostring(code))
    --print('headers:' .. dumps(headers))
    --print('status:' .. tostring(status))
    local rv = jsonDecode(table.concat(respbody), 'json request response')
    return rv
  end
end

local function jsonRequestPost(uri, data)
  data = data or {}

  local respbody = {}
  local reqbody = jsonEncode(data)
  local body, code, headers, status = httpClient.request {
    method = 'POST',
    url = uri,
    source = ltn12.source.string(reqbody),
    headers = {
      ["Accept"] = "*/*",
      --["Accept-Encoding"] = "gzip, deflate",
      --["Accept-Language"] = "en-us",
      ["Content-Type"] = "application/json",
      ["content-length"] = string.len(reqbody)
    },
    sink = ltn12.sink.table(respbody)
  }

  if code ~= 200 then
    log('E', logTag, 'rally client error: '..dumps(code))
    return {ok = false, error = tostring(code)}
  else
    local rv = jsonDecode(table.concat(respbody), 'json request response')
    return rv
  end
end

-- local function transcribe_recording_start()
--   log('I', logTag, 'transcribe_recording_start')
--   local url = base_url..'/recordings/actions/start'
--   local resp = jsonRequestPost(url)
--   if not resp.ok then
--     if not network_issue_occurred then
--       network_issue_occurred = true
--       network_issue_msg = recording_disabled_msg
--     end
--     resp.client_msg = network_issue_msg or resp.error
--   end
--   return resp
-- end
--
-- local function transcribe_recording_stop()
--   log('I', logTag, 'client.transcribe_recording_stop')
--   local url = base_url..'/recordings/actions/stop'
--   local resp = jsonRequestPost(url)
--   -- guihooks.trigger('MenuHide') -- why did i keep this around?
--   if not resp.ok then
--     if not network_issue_occurred then
--       network_issue_occurred = true
--       network_issue_msg = recording_disabled_msg
--     end
--     resp.client_msg = network_issue_msg or resp.error
--   end
--   return resp
-- end

  -- if extensions.isExtensionLoaded(name) then
local function transcribe_recording_cut(requestBody)
  log('I', logTag, 'client.transcribe_recording_cut')

  local url = base_url..'/recordings/actions/cut'
  local resp = jsonRequestPost(url, requestBody)
  if not resp.ok then
    -- if not network_issue_occurred then
    --   network_issue_occurred = true
    --   network_issue_msg = recording_disabled_msg
    -- end
    -- resp.client_msg = network_issue_msg or resp.error
    resp.client_msg = resp.error
  end
  return resp
end

local function transcribe_transcripts_get(count)
  local url = base_url..'/transcripts/'..count
  local resp = jsonRequestGet(url)
  if not resp.ok then
    -- if not network_issue_occurred then
    --   network_issue_occurred = true
    --   network_issue_msg = recording_disabled_msg
    -- end
    -- resp.client_msg = network_issue_msg or resp.error
    resp.client_msg = resp.error
  end
  return resp
end

-- local function remote_audio_play_file(audioFname)
--   log('I', logTag, 'client.remote_audio_play_file')
--
--   local requestBody = {
--     audioFname = audioFname,
--   }
--
--   local url = base_url..'/remoteAudio/playFile'
--   local resp = jsonRequestPost(url, requestBody)
--   if not resp.ok then
--     resp.client_msg = resp.error
--   end
--   return resp
-- end
--
-- local function remote_audio_reset()
--   log('I', logTag, 'client.remote_audio_reset')
--
--   local url = base_url..'/remoteAudio/reset'
--   local resp = jsonRequestPost(url, {})
--   if not resp.ok then
--     resp.client_msg = resp.error
--   end
--   return resp
-- end

-- local function remote_audio_queue_size()
--   -- log('I', logTag, 'client.remote_audio_queue_size')
--
--   local url = base_url..'/remoteAudio/queueSize'
--   local resp = jsonRequestGet(url)
--   if not resp.ok then
--     resp.client_msg = resp.error
--   end
--   return resp
-- end

-- local function update_next_pacenotes(data)
--   -- log('D', 'wtf', dumps(data))
--   --
--   --
--   local req = {
--     created_at = tostring(rallyUtil.getTime()),
--     method= 'POST',
--     path= '/api/v1/update_next_pacenotes',
--     body= data,
--   }
--   local url = base_url..'/proxy'
--   local resp = jsonRequestPost(url, req)
--   if not resp.ok then
--     if not network_issue_occurred then
--       network_issue_occurred = true
--       network_issue_msg = recording_disabled_msg
--     end
--     resp.client_msg = network_issue_msg or resp.error
--   end
--   return resp
-- end

-- M.transcribe_recording_start = transcribe_recording_start
-- M.transcribe_recording_stop = transcribe_recording_stop
-- M.transcribe_recording_stop = transcribe_recording_stop
M.transcribe_recording_cut = transcribe_recording_cut
M.transcribe_transcripts_get = transcribe_transcripts_get
-- M.remote_audio_play_file = remote_audio_play_file
-- M.remote_audio_reset = remote_audio_reset
-- M.remote_audio_queue_size = remote_audio_queue_size
-- M.update_next_pacenotes = update_next_pacenotes

return M
