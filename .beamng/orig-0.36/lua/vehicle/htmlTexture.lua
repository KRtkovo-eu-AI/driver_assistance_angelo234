-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function callJS(self, jsMethod, data)
  --print('call(' .. tostring(self.webViewTag) .. ',' .. tostring(jsMethod) .. ',' .. dumps(data) .. ')')
  obj:queueWebViewJS(self.webViewTag, string.format("%s(%s)", jsMethod, jsonEncode(data)))
end

local function streamJS(self, streamName, jsMethod, data)
  --print('call(' .. tostring(self.webViewTag) .. ',' .. tostring(jsMethod) .. ',' .. dumps(data) .. ')')
  obj:queueWebViewStreamJS(self.webViewTag, streamName, string.format("%s(%s)", jsMethod, jsonEncode(data)))
end

local function new(webViewTag, uri, width, height, fps, usagemode)
  local r = {webViewTag = webViewTag, uri = uri, width = width, height = height}

  local usageModeID = 0
  if usagemode == nil or usagemode == "automatic" then
    usageModeID = 1 -- UI_TEXTURE_USAGE_AUTOMATIC
    fps = fps or 25
  elseif usagemode == "once" then
    usageModeID = 0 -- UI_TEXTURE_USAGE_ONCE
    fps = 0
  elseif usagemode == "manual" then
    usageModeID = 2 -- UI_TEXTURE_USAGE_MANUAL
    fps = 0
  end
  r.fps = fps
  r.usageModeID = usageModeID
  obj:createWebView(r.webViewTag, r.uri, r.width, r.height, r.usageModeID, r.fps)

  return setmetatable(r, {__index = {callJS = callJS, streamJS = streamJS}})
end

local function call(webViewTag, jsMethod, data)
  --print('call(' .. tostring(webViewTag) .. ',' .. tostring(jsMethod) .. ',' .. dumps(data) .. ')')
  local jsCmd = string.format("%s(%s)", jsMethod, jsonEncode(data))
  obj:queueWebViewJS(webViewTag, jsCmd)
end

-- only for the own vehicle
local function create(webViewTag, uri, width, height, fps, usagemode)
  --print('create(' .. tostring(webViewTag) .. ',' .. tostring(uri) .. ',' .. tostring(width) .. ',' .. tostring(height) .. ',' .. tostring(fps) .. ',' .. tostring(usagemode) .. ')')
  local usageModeID = 0
  if usagemode == nil or usagemode == "automatic" then
    usageModeID = 1 -- UI_TEXTURE_USAGE_AUTOMATIC
    fps = fps or 25
  elseif usagemode == "once" then
    usageModeID = 0 -- UI_TEXTURE_USAGE_ONCE
    fps = 0
  elseif usagemode == "manual" then
    usageModeID = 2 -- UI_TEXTURE_USAGE_MANUAL
    fps = 0
  end
  obj:createWebView(webViewTag, uri, width, height, usageModeID, fps)
end

M.new = new
M.create = create
M.call = call

return M
