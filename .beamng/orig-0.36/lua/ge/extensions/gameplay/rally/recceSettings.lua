-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}

local logTag = ''

local default_settings = {
  last_mission_ids = {},
  last_load_states = {},
  corner_call_style_name = nil,
}

function C:init()
  self.fname = rallyUtil.rallySettingsRoot..'/recce.json'
  self.corner_angles_data = nil
  self:_reset()
end

function C:_reset()
  self.last_mission_ids = default_settings.last_mission_ids
  self.last_load_states = default_settings.last_load_states
  self.corner_call_style_name = default_settings.corner_call_style_name
end

function C:loadCornerAngles()
  self.corner_angles_data = nil

  local json, err = rallyUtil.loadCornerAnglesFile()
  if json then
    self.corner_angles_data = json
  end
end

function C:load()
  self:_reset()
  self:loadCornerAngles()

  if not self.fname  then
    log('W', logTag, 'load: recce settings fname is nil: '..tostring(self.fname))
    return false
  end

  if not FS:fileExists(self.fname) then
    log('W', logTag, 'load: recce settings fname doesnt exist: '..tostring(self.fname))
    return false
  end

  local json = jsonReadFile(self.fname)
  if not json then
    log('E', logTag, 'load: couldnt find recce settings file: '..tostring(self.fname))
    return false
  end

  self:onDeserialized(json)
  -- log('I', logTag, 'loaded recce settings from '..tostring(self.fname))

  return true
end

function C:save()
  local json = self:onSerialize()
  jsonWriteFile(self.fname, json, true)
end

function C:onSerialize()
  local ret = {
    last_mission_ids = self.last_mission_ids,
    last_load_states = self.last_load_states,
    corner_call_style_name = self.corner_call_style_name,
  }

  return ret
end

function C:defaultCornerCallStyleName()
  if not self.corner_angles_data then return default_settings.corner_call_style_name end

  local firstStyle = self.corner_angles_data.pacenoteStyles[1]
  if firstStyle then
    return firstStyle.name
  else
    return default_settings.corner_call_style_name
  end
end

function C:cornerCallStyleNames()
  local styleNames = {}
  for _,style in ipairs(self.corner_angles_data.pacenoteStyles) do
    table.insert(styleNames, style.name)
  end
  return styleNames
end

function C:onDeserialized(data)
  if not data then return end
  self.last_mission_ids = data.last_mission_ids or default_settings.last_mission_ids
  self.last_load_states = data.last_load_states or default_settings.last_load_states
  self.corner_call_style_name = data.corner_call_style_name or self:defaultCornerCallStyleName()
end

function C:setLastMissionId(level, id)
  self.last_mission_ids[level] = id
  self:save()
end
function C:getLastMissionId(level)
  return self.last_mission_ids[level]
end

function C:setLastLoadState(level, state)
  self.last_load_states[level] = state
  self:save()
end
function C:getLastLoadState(level)
  return self.last_load_states[level]
end

function C:setCornerCallStyleName(style_name)
  self.corner_call_style_name = style_name
  self:save()
end
function C:getCornerCallStyleName()
  return self.corner_call_style_name
end
function C:getCornerCallStyle()
  for _,style in ipairs(self.corner_angles_data.pacenoteStyles) do
    if style.name == self:getCornerCallStyleName() then
      return style
    end
  end

  return nil
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
