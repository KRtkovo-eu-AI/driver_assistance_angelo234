-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local Schema = require('/lua/ge/extensions/gameplay/rally/notebook/structured/schema')

local C = {}

function C:init()
  self.fields = {}
  Schema.initDefaultFields(self.fields)
end

function C:onDeserialized(data)
  self.fields = {}

  -- init all fields to defaults
  -- for fieldName, _ in pairs(Schema.schema) do
  --   self.fields[fieldName] = Schema.schema[fieldName].default
  -- end

  Schema.initDefaultFields(self.fields)

  if not data then
    return
  end

  -- legacy migrations
  if data.cornerSeverity == 0 then
    data.cornerSeverity = -1
  end

  if data.cornerLength == 0 or data.cornerLength == 20 then
    data.cornerLength = 50
  end

  if data.cornerRadiusChange == 0 or data.cornerRadiusChange == 20 then
    data.cornerRadiusChange = 50
  end

  -- migrate caution to latest format
  if data.modCaution then
    data.caution = 1
  elseif data.modCaution1 then
    data.caution = 1
  elseif data.modCaution2 then
    data.caution = 2
  elseif data.modCaution3 then
    data.caution = 3
  end

  if data.modBumps then
    data.modBumpy = true
  end
  -- / legacy migrations

  -- overwrite fields with data
  for fieldName, _ in pairs(Schema.schema) do
    if data[fieldName] ~= nil then
      if Schema.schema[fieldName].type == 'number' or Schema.schema[fieldName].type == 'enum' then
        self.fields[fieldName] = tonumber(data[fieldName])
      else
        self.fields[fieldName] = data[fieldName]
      end
    end
  end
end

function C:onSerialize()
  return self.fields
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
