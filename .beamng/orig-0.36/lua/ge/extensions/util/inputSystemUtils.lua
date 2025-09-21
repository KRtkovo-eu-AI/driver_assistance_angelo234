-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.util_inputSystemUtils.resave()

local M = {}

-- sorting helper for printStats
local function sortTableByValue(tbl)
  local res = {}
  for k, v in pairs(tbl) do
    table.insert(res, {k, v})
  end
  table.sort(res, function(a, b) return a[2] > b[2] end)
  return res
end

-- dump some stats about input files
local function printStats()
  local bindingCount = 0
  local usedActions = {}
  for _, path in ipairs(FS:findFiles('/', '*.json', -1, false, false)) do
    if path:find('inputmaps/') then
      local m = jsonReadFile(path)
      if m then
        if type(m.bindings) == 'table' then
          bindingCount = bindingCount + #m.bindings
          for _, b in ipairs(m.bindings) do
            if b.action then
              if not usedActions[b.action] then usedActions[b.action] = 0 end
              usedActions[b.action] = usedActions[b.action] + 1
            end
          end
        end
      end
    end
  end

  print(tostring(bindingCount) .. ' total bindings')

  local sortedActions = sortTableByValue(usedActions)
  local tblLen = math.min(#sortedActions, 20)
  print('top ' .. tostring(tblLen) .. ' actions:')
  for n = 1, tblLen do
    print(' * ' .. sortedActions[n][1] .. ' - ' .. tostring(sortedActions[n][2]))
  end
end

local vendorNames = jsonReadFile("lua/ge/extensions/util/vendorNames.json")

-- helper for natural sorting, enables sorting 1, 11, 2 to 1, 2, 11 by fake-padding the number before comparing it
local function padnum(d) return ("%012d"):format(d) end
local function naturalSortHelper(a, b) return tostring(a):gsub("%d+",padnum) < tostring(b):gsub("%d+", padnum) end

-- the key weights for sorting. default level is 50. 1 = first, 99 = last
local tblWeights = {
  ["control"] = 10, -- control goes before the default
  -- default = 50
  ["bindings"] = 99, -- put bindings last
}

-- gets called when the encoder needs to decide if the table should be collapsed or not
local function foldingCallback(item, lvl, path)
  -- collapse anything below bindings that has less than 4 items
  return path:sub(1,10) == '/bindings/' and tableSize(item) < 4
end

local function isValidVIdPId(vidpid)
  if type(vidpid) ~= 'string' or vidpid:len() ~= 8 then
    return false
  end
  return string.match(vidpid, '^[A-Fa-f0-9]+$')
end

-- resaves all inputmaps with real nice, custom formatting
local function resave()
  for _, filepath in ipairs(FS:findFiles('/', '*.json', -1, false, false)) do
    if filepath:find('inputmaps/') then
      local content = readFile(filepath)
      if not content then
        log('E', 'input', 'unable to open file: ' .. tostring(filepath))
        goto continue
      end
      local info = jsonDecode(content, filepath)
      if not info then
        log('E', 'input', 'unable to read file: ' .. tostring(filepath))
        goto continue
      end
      local vendorId = 0
      if not info.vidpid then
        local dir, filename, ext = path.splitWithoutExt(filepath)
        info.vidpid = filename:upper()
        vendorId = filename:upper():sub(5, 8)
      else
        vendorId = info.vidpid:upper():sub(5, 8)
      end
      info.vendorName = info.vendorName or vendorNames[vendorId:lower()]

      if not isValidVIdPId(info.vidpid) then
        --print('invalid vid/pid: ' .. tostring(info.vidpid))
        info.vidpid = nil
      end

      if type(info.bindings) == 'table' then
        table.sort(info.bindings, function(a, b)
          if not a.control or not b.control then return tostring(a) < tostring(b) end -- fallback to table pointer in worst case the data is broken
          if a.control == b.control and a.action and b.action then
            return naturalSortHelper(a.action, b.action)
          end
          return naturalSortHelper(a.control, b.control)
        end)
      end

      local json = require('jsonPrettyEncoderCustom').encode(info, nil, nil, tblWeights, foldingCallback)
      local jsonFinal = json  -- .. '\n' -- what should be written to the file
      if content ~= jsonFinal then
        writeFile(filepath, jsonFinal)
        print('Resaved ' .. tostring(filepath))
      else
        print(tostring(filepath) .. ' - no changes needed')
      end
    end
    ::continue::
  end
  print('done')
end

M.printStats = printStats
M.resave = resave

return M