-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function replaceWords(word_map, note)
  if not note then return note end

  local newnote, count
  newnote = note

  for _,mapping in ipairs(word_map) do
    local from, to = mapping[1], mapping[2]
    newnote, count = newnote:gsub(from, to)
    -- note = note:gsub(from, to)
  end
  return newnote
end

M.replaceWords = replaceWords

return M
