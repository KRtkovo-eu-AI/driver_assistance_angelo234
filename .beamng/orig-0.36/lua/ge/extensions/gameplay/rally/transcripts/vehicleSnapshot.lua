-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

function C:init(path, name, forceId)
  self.path = path

  -- sortedList fields
  self.id = forceId or self.path:getNextUniqueIdentifier()
  self.name = name or ('t_'..self.id)
  self.sortOrder = 999999

  self.pos = nil
  self.quat = nil-- or rot ?
  self.steering = nil-- or rot ?
  self.timestamp = nil
  self.cornerCall = nil
end

function C:onSerialize()
  local ret = {
    oldId = self.id,
    name = self.name,

    text = self.text,
  }

  return ret
end

function C:onDeserialized(data, oldIdMap)
  -- self.text = data.text or ''
  -- self.success = data.success or false
  -- self.src = data.src or ''
  -- self.file = data.file or ''
  -- self.beamng_file = data.beamng_file or ''
  -- self.timestamp = data.timestamp or 0.0
  -- self.vehicle_pos = data.vehicle_pos or {}
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
