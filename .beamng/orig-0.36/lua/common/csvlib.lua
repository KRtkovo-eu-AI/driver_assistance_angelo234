-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
--
-- Usage:
--
-- local csvfile = require('csvlib').newCSV("c1", "c2", "c3")  -- or .newTSV("c1", "c2", "c3") for Tab Separated Values
-- csvfile:add(1,2,3)
-- csvfile:add(5,6,7)
-- ...
--
-- csvfile:write()              -- writes csv_DATETIME.csv
-- csvfile:write("myname")      -- writes myname_DATETIME.csv
-- csvfile:write("myname.csv")  -- writes myname.csv
require 'utils'

local M = {}
local csvWriter = {}
csvWriter.__index = csvWriter

local buffer = require('string.buffer')
local byte, sub, sformat, tablenew, max = string.byte, string.sub, string.format, table.new, math.max

local function newXSV(delim, ...)
  local self = setmetatable({buf = buffer.new(), linedelim = "", delim = delim, delimnum = byte(delim, 1)}, csvWriter)
  local headercount = select('#', ...)
  if headercount ~= 0 then
    self:add(...)
    local header = {...}
    for i = 1, headercount do
      header[i] = #header[i] > 0 and tostring(header[i]):sub(1, 1) or "_"
    end
    self.headernym = table.concat(header)
  end
  return self
end

local function newCSV(...)
  return newXSV(",", ...)
end

local function newTSV(...)
  return newXSV("\t", ...)
end

function csvWriter:add(...)
  local delim, rundelim, buf = self.delim, self.linedelim, self.buf

  for i = 1, select('#', ...) do
    buf:put(rundelim)
    rundelim = delim

    local v = select(i, ...)
    local vtype = type(v)
    if vtype == 'number' then
      buf:putf('%.10g', v)
    elseif vtype == 'boolean' then
      buf:put(v and 1 or 0)
    elseif vtype == 'nil' then
    elseif vtype == 'string' then
      v = tostring(v)
      local delimnum = self.delimnum
      local c = byte(v, 1)
      if c == nil or c <= 32 or c == delimnum or c == 34 then -- space sep "
        buf:put( sformat('"%s"', v:gsub('"', '""')) )
      else
        local raw = true
        for i1 = 2, #v do
          local c = byte(v, i1)
          if c == delimnum or c == 34 or c == 13 or c == 10 then -- sep " CR LF
            buf:put( sformat('"%s"', v:gsub('"', '""')) ) -- gsub returns 2 values, parens are needed
            raw = false
            break
          end
        end
        if raw then buf:put( v ) end
      end
    end
  end
  self.linedelim = '\n'
  return self
end

function csvWriter:dump()
  return tostring(self.buf)
end

function csvWriter:write(filename)
  local format = self.delim == ',' and 'csv' or 'tsv'
  filename = filename or self.headernym or format
  if filename:sub(-4, -4) ~= '.' then
    filename = string.format("%s_%s.%s", filename, os.date("%Y-%m-%dT%H_%M_%S"), format)
  end
  writeFile(filename, self:dump())
  return filename
end

-- https://datatracker.ietf.org/doc/html/rfc4180
local function decode(s, sep)
  if s == nil then return nil end
  sep = sep and byte(sep) or 44 -- ,
  local res, resi, lineLen, li, c, i = {}, 1, 1, 1, 0, 1
  local line = tablenew(1, 0)
  while true do
    c = byte(s, i)
    while c == 32 or c == 9 do -- space tab
      i = i + 1; c = byte(s, i)
    end
    local si = i
    local val
    if c == 34 then -- "
      repeat
        i = i + 1; c = byte(s, i)
        if c == 34 and byte(s, i + 1) == 34 then -- "
          i = i + 2; c = byte(s, i)
        end
      until (c == 34 and byte(s, i+1) ~= 34) or c == nil -- "
      val = sub(s, si+1, i - 1):gsub('""', '"')
      repeat i=i+1; c = byte(s,i) until c~=32 and c~=9 -- space tab
    else
      local testnum = c ~= nil and ((c >= 48 and c <= 57) or c == 43 or c == 45) -- 0123456789+-
      while c ~= sep and c ~= 13 and c ~= 10 and c ~= nil do  -- sep CR LF
        i = i + 1; c = byte(s, i)
      end
      val = sub(s, si, i - 1)
      if testnum then val = tonumber(val) or val end
    end
    line[li] = val
    i = i + 1
    if c == sep then
      li = li + 1
    else
      c = byte(s, i)
      res[resi] = line
      resi = resi + 1
      for j = li, lineLen + 1, -1 do
        if line[j] == "" then
          li = li - 1; line[j] = nil
        else
          break
        end
      end
      lineLen = max(lineLen, li)
      line = tablenew(lineLen, 0)
      li = 1
      if c == 13 or c == 10 then
        i = i + 1
      end
      if c == nil then
        for i = resi-1, 1, -1 do
          if #res[i] == 1 and res[i][1] == "" then
            res[i] = nil
          else
            break
          end
        end
        return res
      end
    end
  end
end

local function readFileCSV(filename, sep)
  return decode(readFile(filename), sep)
end

M.newCSV = newCSV
M.newTSV = newTSV
M.readFileCSV = readFileCSV
M.decode = decode
return M