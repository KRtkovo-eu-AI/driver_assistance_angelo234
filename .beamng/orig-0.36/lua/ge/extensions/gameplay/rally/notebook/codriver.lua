-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
local logTag = ''

function C:init(notebook, name, forceId)
  self.notebook = notebook

  if #self.notebook.codrivers.sorted == 0 then
    name = rallyUtil.defaultCodriverName
  end

  self.id = forceId or notebook:getNextUniqueIdentifier()
  self.name = name or ("Codriver " .. self.id)
  self.language = rallyUtil.default_codriver_language
  self.voice = rallyUtil.default_codriver_voice
  self.pk = rallyUtil.randomId()

  self.sortOrder = 999999
end

function C:onSerialize()
  local ret = {
    oldId = self.id,
    name = self.name,
    language = self.language,
    voice = self.voice,
    pk = self.pk,
  }

  return ret
end

function C:onDeserialized(data, oldIdMap)
  self.name = data.name
  self.language = data.language
  self.voice = data.voice
  self.pk = data.pk or rallyUtil.randomId()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
