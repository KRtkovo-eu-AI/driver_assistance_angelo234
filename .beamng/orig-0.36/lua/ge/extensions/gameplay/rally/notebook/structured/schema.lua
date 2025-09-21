-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- local defaults = {
--   -- corner
--   cornerSeverity = -1,     -- 0 to 100 inclusive. 0 is straight. 100 is the tighest. -1 is unknown or not set.
--   cornerDirection = 0,     -- left=-1, straight=0, right=1
--   cornerLength = 20,       -- enum: 10=short, 20=normal, 30=long, 40=extra_long
--   cornerRadiusChange = 20, -- enum: 10=opens, 20=normal, 30=tightens

--   cornerSquare = false,       -- a special type of corner

--   -- caution
--   caution = 0, -- 0=none, 1=caution, 2=double_caution, 3=triple_caution

--   -- modifiers
--   modDontCut = false,
--   modNarrows = false,
--   modWater = false,
--   modBumpy = false,
--   modBump = false,
--   modJump = false,
--   modCrest = false,
-- }

M.schema = {
  -- corner descriptors
  cornerSeverity = { type = 'number', default = -1, min = -1, max = 100 },
  cornerDirection = { type = 'enum', default = 0, values = {
    -1, -- left
     0, -- straight
     1, -- right
  }},
  cornerLength = { type = 'number', default = 50, min = 0, max = 100 },
  cornerRadiusChange = { type = 'number', default = 50, min = 0, max = 100 },
  cornerSquare = { type = 'boolean', default = false },

  -- caution descriptors
  caution = { type = 'enum', default = 0, values = {
    0, -- none
    1, -- caution
    2, -- double_caution
    3, -- triple_caution
  }},

  -- modifiers
  modDontCut = { type = 'boolean', default = false },
  modNarrows = { type = 'boolean', default = false },
  modWater   = { type = 'boolean', default = false },
  modBumpy   = { type = 'boolean', default = false },
  modBump    = { type = 'boolean', default = false },
  modJump    = { type = 'boolean', default = false },
  modCrest   = { type = 'boolean', default = false },

  -- finish line
  finishLine = { type = 'boolean', default = false },

  system = {
    'damage',
    'go',
    'countdown1',
    'countdown2',
    'countdown3',
    'countdown4',
    'countdown5',
    'firstnoteintro',
    'firstnoteoutro',
    'finish',
  }
}

M.default = function(keyName)
  return M.schema[keyName].default
end

M.initDefaultFields = function(fields)
  for fieldName, _ in pairs(M.schema) do
    fields[fieldName] = M.schema[fieldName].default
  end
end

return M
