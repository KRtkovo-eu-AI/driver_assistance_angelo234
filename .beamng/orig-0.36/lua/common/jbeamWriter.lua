-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this file contains a special version of the json pretty writer that formats the object until a certain depth
-- this is not merged with the main function because of performance reasons

local M = {}

-- useful local shortcuts
local min, max, abs, fmod, floor, random = math.min, math.max, math.abs, math.fmod, math.floor, math.random
local stringformat, tableconcat = string.format, table.concat
local str_find, str_len, str_sub, byte = string.find, string.len, string.sub, string.byte

-- adds a maxLevel argument
local function escapeString(s)
  for i = 1, #s do
    local c = byte(s, i)
    if c < 32 or c == 34 or c == 92 then
      return ( s:gsub("\\", "\\\\"):gsub("\n", "\\n"):gsub("\t", "\\t"):gsub('"','\\"'):gsub("\r", "\\r") ) -- gsub returns 2 values, parens are needed
    end
  end
  return s
end

local function jsonEncodePrettyJbeam(v, lvl, numberPrecision, maxLevel)
  if lvl and maxLevel and lvl > maxLevel then
    return jsonEncode(v)
  end
  if v == nil then return "null" end
  local vtype = type(v)
  if vtype == 'string' then return stringformat('"%s"', escapeString(v)) end
  if vtype == 'number' then
    if v * 0 ~= 0 then -- inf,nan
      return v > 0 and '9e999' or '-9e999'
    else
      if numberPrecision == nil then
        return stringformat('%.10g', v)  -- .10g is needed for time
      else
        if v ~= floor(v) then
          return stringformat('%' .. numberPrecision .. '.' .. numberPrecision .. 'f', v)
        else
          return stringformat('%d', v)
        end
      end
    end
  end
  if vtype == 'boolean' then return tostring(v) end

  -- Handle tables
  if vtype == 'table' then
    lvl = lvl or 1
    local indent = string.rep('  ', lvl)
    local indentPrev = string.rep('  ', max(0, lvl - 1))
    local tmp = {}
    if next(v) == 1 and next(v, #v) == nil then
      for i = 1, #v do
        table.insert(tmp, jsonEncodePrettyJbeam(v[i], lvl + 1, numberPrecision, maxLevel))
      end
      return stringformat('[\n' .. indent .. '%s\n' .. indentPrev .. ']', table.concat(tmp, ',\n' .. indent))
    else
      if next(v) == nil then
        return '{}'
      else
        -- sort keys first
        local tableKeys = tableKeysSorted(v)
        for _, kk in pairs(tableKeys) do
          local vv = v[kk]
          local maxLevelTmp = maxLevel
          if kk == 'components' and lvl == 1 then
            -- components section is always pretty printed
            maxLevelTmp = 999
          end
          local cv = jsonEncodePrettyJbeam(vv, lvl + 1, numberPrecision, maxLevelTmp)
          if cv ~= nil then table.insert(tmp, stringformat('"%s":%s', escapeString(tostring(kk)), cv)) end
        end
        return stringformat('{\n'..indent .. '%s\n'.. indentPrev ..'}', table.concat(tmp, ',\n' .. indent))
      end
    end
  end

  if vtype == 'cdata' and ffi.offsetof(v, 'z') ~= nil then  -- vec3
    if ffi.offsetof(v, 'w') ~= nil then
      return stringformat('{"x":%.10g,"y":%.10g,"z":%.10g,"w":%.10g}', v.x, v.y, v.z, v.w)
    else
      return stringformat('{"x":%.10g,"y":%.10g,"z":%.10g}', v.x, v.y, v.z)
    end
  end

  return "null"
end

local function writeFile(filename, obj)
  local f = io.open(filename, "w")
  if f then
    local content = jsonEncodePrettyJbeam(obj, 1, numberPrecision, 2)
    f:write(content)
    f:close()
    return true
  else
    log("E", "jsonWriteFile", "Unable to open file for writing: "..dumps(filename))
    print(debug.tracesimple())
  end
  return false
end

M.writeFile = writeFile

return M