-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local bench = require("lua/console/bananabench")

-- Simple functions for XML output
XML = {}
XML.__index = XML

function XML.create()
  local xml = {}
  setmetatable(xml, XML)
  xml.name = ""
  xml.attribs = {}
  xml.children = {}
  return xml
end

function XML:addChild(xml)
  table.insert(self.children, xml)
end

function XML:encode()
  str = [[<?xml version="1.0" encoding="UTF-8"?>]]..'\n'
  return str..self:toString()
end

function XML:toString(indent)
  indent = indent or 0
  str = string.rep("  ", indent).."<"..self.name
  for k,v in pairs(self.attribs) do
    str = str.." "..k.."=\""..v.."\""
  end
  if #self.children > 0 or self.value then
    str = str..">\n"
  else
    str = str.." />\n"
  end
  if self.value then
    str = str..self.value
  end
  for k, v in pairs(self.children) do
    str = str..v:toString(indent + 1)
  end
  if #self.children > 0 or self.value then
    if not self.value then
      str = str..string.rep("  ", indent)
    end
    str = str.."</"..self.name..">\n"
  end
  return str
end

local function serializeXmlToFile(filename, obj)
  local f = io.open(filename, "w")
  f:write(obj:encode())
  f:close()
end

-- tiny xml helper function
local function xh(name, attribs)
    local t = XML.create()
    t.name = name
    t.attribs = attribs or {}
    return t
end

local function writeXML(res, filename)
    local xml = xh('report', {
        name = "perf1",
        categ = "performance",
        time  = string.format("%0.3f", res.time / 1000)
    })

    for vecname,v in pairs(res.tests) do
    for dynamicCol = 1, 2 do
      local firstTestResult = v.tests[1].res[dynamicCol]

      local test = xh('test', {
        name = (dynamicCol == 1) and (vecname .. "WithDynamicCollision") or vecname .. "WithoutDynamicCollision",
        executed = 'yes',
      })

      local result = xh('result')

      local passed = 'yes'
      local state = 100
      --if firstTest.logcache.max == 'warn' then
      --    passed = 'no'
      --    state = 70
      --elseif firstTest.logcache.max == 'error' then
      --    passed = 'no'
      --    state = 40
      --end

      local success = xh('success', {
        passed = passed,
        state = state,
        hasTimedOut = 'false',
      })
      result:addChild(success)

      local performance = xh('performance', {
        unit = 'MBeams/s',
        mesure = firstTestResult.beamspersec,
        isRelevant = 'true'
      })
      result:addChild(performance)

      local executiontime = xh('executiontime', {
        unit = 's',
        mesure = firstTestResult.time,
        isRelevant = 'true'
      })
      result:addChild(executiontime)

      local compiletime = xh('compiletime', {
        unit = 's',
        mesure = firstTestResult.spawntime,
        isRelevant = 'true'
      })
      result:addChild(compiletime)

      -- custom metrics
      local metrics = xh('metrics')
      local realtime = xh('realtime', {
        unit = '%',
        mesure = firstTestResult.percentRealtime,
        isRelevant = 'true'
      })
      metrics:addChild(realtime)

      local maxmbeams = xh('maxmbeams', {
        unit = 'MBeams/s',
        mesure = v.maxMbeams,
        isRelevant = 'true'
      })
      metrics:addChild(maxmbeams)

      local maxmbeamsvehicles = xh('maxmbeamsvehicles', {
        unit = 'vehicles',
        mesure = v.maxMbeamsNum,
        isRelevant = 'true'
      })
      metrics:addChild(maxmbeamsvehicles)

      result:addChild(metrics)

      -- Turned this of because logcache is not written to.
      --if not tableIsEmpty(firstTest.logcache) then
        --local errorlog = xh('errorlog')
        --local t = '<![CDATA[<pre>'
        --for _,l in pairs(firstTest.logcache) do
          --if l.level then
            --t = t .. tostring(l.level) .. '|' .. tostring(l.origin) .. '|' .. tostring(l.msg)
            --if l.newline then
              --t = t .. '\n'
            --end
          --end
        --end
        --t = t .. '</pre>]]>'
        --errorlog.value = t
        --result:addChild(errorlog)
      --end

      -- end
      test:addChild(result)
      xml:addChild(test)
    end
    end
  print("serializeXmlToFile")
    serializeXmlToFile(filename, xml)
end

--local res = bench.physics({'pigeon'}, 1,1)
--local vehicles = bench.getAllVehicles()
local vehicles = {
    "barstow",
    "burnside",
    "etki",
    "fullsize",
    "moonhawk",
    "pessima",
    "pickup",
    "pigeon",
    "pressure_ball",
}
dump(vehicles)

local filepath = "bananabench.xml"

local res = bench.physics(vehicles)

--dump(res)
writeXML(res, filepath)
