-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- utilities that are only neded in vehicle lua

-- useful local shortcuts
local abs, floor, min, max, stringformat, tableconcat = math.abs, math.floor, math.min, math.max, string.format, table.concat

--= HighPerfTimer
local _HighPerfTimer = {}
_HighPerfTimer.__index = _HighPerfTimer

function HighPerfTimer()
  return setmetatable({os.clockhp()}, _HighPerfTimer)
end

function _HighPerfTimer:reset()
  self[1] = os.clockhp()
end

function _HighPerfTimer:stop()
  return (os.clockhp() - self[1]) * 1000
end

function _HighPerfTimer:stopAndReset()
  local t = (os.clockhp() - self[1]) * 1000
  self[1] = os.clockhp()
  return t
end

function _HighPerfTimer:getTime()
  return os.clockhp() * 1000
end

function tableFromHeaderTable(entry)
  if #entry == 0 then
    return {}
  end
  --local function processTableWithSchema(vehicle, keyEntry, entry, newList)
  -- its a list, so a table for us. Verify that the first row is the header
  local header = entry[1]

  if type(header) ~= "table" then
    log("W", "tableFromHeaderTable", "*** Invalid table header: " .. dumpsz(entry, 2) .. " - " .. debug.tracesimple())

    return false
  end

  local headerSize = #header
  local localOptions = {}
  local output = {}
  local outputidx = 1

  -- walk the list entries
  for i = 2, #entry do
    local rowValue = entry[i]

    if type(rowValue) ~= "table" then
      log("W", "tableFromHeaderTable", "*** Invalid table row: " .. dumps(rowValue))
      return false
    end
    if tableIsDict(rowValue) then
      -- case where options is a dict on its own, filling a whole line
      localOptions = tableMerge(localOptions, deepcopy(rowValue))
    else
      -- allow last type to be the options always
      if #rowValue > headerSize + 1 then -- and type(rowValue[#rowValue]) ~= "table" then
        log("W", "tableFromHeaderTable", "*** Invalid table header, must be as long as all table cells (plus one additional options column):")
        log("W", "tableFromHeaderTable", "*** Table header: " .. dumps(header))
        log("W", "tableFromHeaderTable", "*** Mismatched row: " .. dumps(rowValue))
        return false
      end

      local newRow
      if next(localOptions) == nil then
        newRow = {}
      else
        newRow = deepcopy(localOptions)
      end

      local rvcc = 0
      for rk, rv in pairs(rowValue) do
        --log('D', "jbeam.processTableWithSchema", "### "..header[rk].."//"..tostring(newRow[header[rk]]))
        if header[rk] ~= nil then
          newRow[header[rk]] = rv
        end
        -- check if inline options are provided, merge them then
        if rvcc >= headerSize and type(rv) == "table" and tableIsDict(rv) and #rowValue > headerSize then
          tableMerge(newRow, rv)
          break
        end
        rvcc = rvcc + 1
      end

      if newRow.id ~= nil then
        newRow.name = newRow.id -- this keeps the name for debugging or alike
        newRow.id = nil
      end

      output[outputidx] = newRow
      outputidx = outputidx + 1
    end
  end
  return output
end

local function toJSONString(d)
  if type(d) == "string" then
    return '"' .. d .. '"'
  elseif type(d) == "number" then
    return string.format("%g", d)
  else
    return tostring(d)
  end
end

function saveCompiledJBeamRecursive(f, data, level)
  local indent = string.rep(" ", level * 2)
  local nl = true
  if level > 2 then
    nl = false
  end
  if level > 3 then
    indent = ""
  end
  --f:write(level..indent
  f:write(indent)

  if type(data) == "table" and type(data["partPath"]) == "string" and data["partPath"] ~= "" then
    f:write("\n" .. indent .. "/*" .. string.rep("*", 50) .. "\n")
    f:write(indent .. " * part " .. tostring(data["partPath"]) .. "\n")
    f:write(indent .. " *" .. string.rep("*", 49) .. "*/\n")
    f:write("\n" .. indent)
  end

  if level > 2 then
    indent = ""
  end
  if type(data) == "table" then
    if tableIsDict(data) then
      f:write("{")
      if nl then
        f:write("\n")
      end
      local localColumnCount = 0
      for _, _ in pairs(data) do
        localColumnCount = localColumnCount + 1
      end
      local i = 1
      for k, v in pairs(data) do
        if type(v) == "table" then
          --if nl then f:write("\n" end
          f:write(toJSONString(k) .. " : ")
          --if nl then f:write("\n" end
          saveCompiledJBeamRecursive(f, v, level + 1)
        else
          local txt = toJSONString(k) .. " : " .. toJSONString(v)
          f:write(txt)
        end
        if i < localColumnCount then
          f:write(", ")
        elseif i == localColumnCount then
          if nl then
            f:write("\n")
          end
        end
        i = i + 1
      end
      if nl then
        f:write("\n")
      end
      f:write("}")
      if level < 2 then
        f:write("\n")
      end
    else
      local nl = true
      if level > 2 then
        nl = false
      end
      f:write("[")
      if nl then
        f:write("\n")
      end
      for i = 1, #data, 1 do
        --k,v in pairs(data) do
        local v = data[i]
        if type(v) == "table" then
          saveCompiledJBeamRecursive(f, v, level + 1)
        else
          local txt = toJSONString(v)
          f:write(txt)
        end
        if i < #data then
          f:write(", ")
          if level == 2 then
            f:write("\n")
          end
        end
      end
      f:write("]")
      if level < 2 then
        f:write("\n")
      end
    end
  end
end

function saveCompiledJBeam(data, filename, lvl)
  local f = io.open(filename, "w")
  if f == nil then
    log("W", "saveCompiledJBeam", "unable to open file " .. filename .. " for writing")
    return false
  end
  saveCompiledJBeamRecursive(f, data, lvl or 0)
  f:close()
  return true
end

--== color things ==--
-- color conversion helpers
function getContrastColor(i, a)
  return jetColor((i % 17) / 16, a)
end

function getContrastColorStringRGB(i)
  local c1, c2, c3 = colorGetRGBA(jetColor((i % 17) / 16))
  return stringformat("#%02x%02x%02x", c1, c2, c3)
end

--== Math ==--
-- used in createCurve
local function CatMullRomSpline(points, returnArray)
  if #points < 3 then
    return nil
  end

  local res
  if returnArray and ffi then
    res = ffi.new("float[?]", points[#points][1] + 1)
  else
    res = table.new(points[#points][1] + 1, 0)
  end

  local pointSize = #points

  for i = 1, pointSize - 1 do
    local p0, p1, p2, p3 = points[max(i - 1, 1)], points[i], points[i + 1], points[min(i + 2, pointSize)]
    local step = 1 / (p2[1] - p1[1])
    local t = 0
    local y0, y1, y2, y3 = p0[2], p1[2], p2[2], p3[2]
    local c1 = 3 * (y1 - y2) + y3 - y0
    local c2 = 2 * y0 - 5 * y1 + 4 * y2 - y3
    local c3 = y2 - y0
    for x = floor(p1[1]), floor(p2[1]) do
      res[x] = y1 + 0.5 * t * (c3 + (c1 * t + c2) * t)
      t = t + step
    end
  end
  return res
end

function createCurve(points, returnArray)
  if #points < 2 then
    return nil
  end
  if #points == 2 then
    local res
    if returnArray and ffi then
      res = ffi.new("float[?]", points[#points][1] + 1)
    else
      res = table.new(points[#points][1] + 1, 0)
    end
    local p1, p2 = points[1], points[2]
    local p2p1 = p2[2] - p1[2]
    local step = 1 / (p2[1] - p1[1])
    local t = 0
    for x = floor(p1[1]), floor(p2[1]) do
      res[x] = p1[2] + t * p2p1
      t = t + step
    end
    return res
  end

  return CatMullRomSpline(points, returnArray)
end

function PSItoPascal(psi)
  return psi * 6894.757 + 101325
end
