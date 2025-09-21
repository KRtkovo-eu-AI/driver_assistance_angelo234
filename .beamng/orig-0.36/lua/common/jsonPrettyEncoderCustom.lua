-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

require("utils")

--[[-- Example --

local exampleTable = {control = 5, bindings = {{1,2}, {4,5,6,7}}}

-- gets called when the encoder needs to decide if the table should be collapsed or not
local function foldingCallback(item, lvl, path)
  -- collapse anything below bindings that has less than 4 items
  return path:sub(1,10) == '/bindings/' and tableSize(item) < 4
end

-- the key weights for sorting. default level is 50. 1 = first, 99 = last
local tblWeights = {
  ["control"] = 10, -- control goes before the default
  -- default = 50
  ["bindings"] = 99, -- put bindings last
}

local json = require('jsonPrettyEncoderCustom').encode(exampleTable, nil, nil, tblWeights, foldingCallback)
dump(json)

--]]

local M = {}

local function jsonEscapeString(s)
  for i = 1, #s do
    local c = string.byte(s, i)
    if c < 32 or c == 34 or c == 92 then
      return ( s:gsub("\\", "\\\\"):gsub("\n", "\\n"):gsub("\t", "\\t"):gsub('"','\\"'):gsub("\r", "\\r") ) -- gsub returns 2 values, parens are needed
    end
  end
  return s
end

local function sortTableKeysByWeight(keys, weights)
  local weighted_vector = {}
  for i = 1, #keys do
    local s = keys[i]
    table.insert(weighted_vector, {s = s, w = weights[tostring(s):lower()] or 50})  -- 50 = middle if no weight is defined
  end

  -- sort the keys using the weight and the name
  table.sort(weighted_vector, function(a, b)
    if a.w == b.w then
      local typeA, typeB = type(a.s), type(b.s)
      if typeA ~= typeB then
        return typeA < typeB
      elseif typeA == "string" or typeA == "number" then
        return a.s < b.s
      elseif typeA == "boolean" then
        return a.s == true
      else
        return tostring(a.s) < tostring(b.s)
      end
    end
    return a.w < b.w
  end)

  local sorted_keys = {}
  for i = 1, #weighted_vector do
    table.insert(sorted_keys, weighted_vector[i].s)
  end
  return sorted_keys
end

local function tableKeysWeightSorted(tbl, tableWeights)
  return sortTableKeysByWeight(tableKeys(tbl), tableWeights)
end

local function encode(v, lvl, numberPrecision, tableWeights, foldingCallback, _levelPath)
  if v == nil then return "null" end
  local vtype = type(v)
  if vtype == 'string' then return string.format('"%s"', jsonEscapeString(v)) end
  if vtype == 'number' then
    if v * 0 ~= 0 then -- inf,nan
      return v > 0 and '9e999' or '-9e999'
    else
      if numberPrecision == nil then
        return string.format('%.10g', v)  -- .10g is needed for time
      else
        if v ~= math.floor(v) then
          return string.format('%' .. numberPrecision .. '.' .. numberPrecision .. 'f', v)
        else
          return string.format('%d', v)
        end
      end
    end
  end
  if vtype == 'boolean' then return tostring(v) end

  -- Handle tables
  if vtype == 'table' then
    lvl = lvl or 1
    _levelPath = _levelPath or ''
    local indent = string.rep('  ', lvl)
    local indentPrev = string.rep('  ', math.max(0, lvl - 1))
    local tmp = {}
    if next(v) == 1 and next(v, #v) == nil then
      for i = 1, #v do
        table.insert(tmp, encode(v[i], lvl + 1, numberPrecision, tableWeights, foldingCallback, _levelPath .. '/' .. tostring(i)))
      end
        local fold = true
        if foldingCallback then fold = foldingCallback(v, lvl ,_levelPath) end
        if not fold then
          return string.format('[\n'..indent .. '%s\n'.. indentPrev ..']', table.concat(tmp, ',\n' .. indent))
        end
        return string.format('[%s]', table.concat(tmp, ', '))
    else
      if next(v) == nil then
        return '{}'
      else
        -- sort keys first
        local tableKeys = tableKeysWeightSorted(v, tableWeights)
        for _, kk in pairs(tableKeys) do
          local vv = v[kk]
          local cv = encode(vv, lvl + 1, numberPrecision, tableWeights, foldingCallback, _levelPath .. '/' .. tostring(kk))
          if cv ~= nil then table.insert(tmp, string.format('"%s":%s', jsonEscapeString(tostring(kk)), cv)) end
        end
        local fold = true
        if foldingCallback then fold = foldingCallback(v, lvl, _levelPath) end
        if not fold then
          return string.format('{\n'..indent .. '%s\n'.. indentPrev ..'}', table.concat(tmp, ',\n' .. indent))
        end
        return string.format('{%s}', table.concat(tmp, ', '))
      end
    end
  end

  if vtype == 'cdata' and ffi.offsetof(v, 'z') ~= nil then  -- vec3
    if ffi.offsetof(v, 'w') ~= nil then
      return string.format('{"x":%.10g,"y":%.10g,"z":%.10g,"w":%.10g}', v.x, v.y, v.z, v.w)
    else
      return string.format('{"x":%.10g,"y":%.10g,"z":%.10g}', v.x, v.y, v.z)
    end
  end

  return "null"
end


M.encode = encode

return M