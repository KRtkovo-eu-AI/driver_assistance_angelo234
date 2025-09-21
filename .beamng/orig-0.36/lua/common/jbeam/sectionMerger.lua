--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

-- this deals with merging sections together for backward compatibility and all

local M = {}

local jbeamUtils = require("jbeam/utils")

-- not tested, be careful
local function mergeNumberedSections(vehicle, sectionNameTarget, sectionNameSource)
  -- safety guards
  vehicle[sectionNameTarget] = vehicle[sectionNameTarget] or {}
  vehicle[sectionNameSource] = vehicle[sectionNameSource] or {}

  -- first, count the source rows so we can add to the end
  -- this is so overly complicated because its 0 based
  local rowCounter = 0
  for i = 0, #vehicle[sectionNameTarget] do
    if not vehicle[sectionNameTarget][i] then
      break
    end
    rowCounter = rowCounter + 1
  end

  -- add the rows at the end of the target table
  for i = 0, #vehicle[sectionNameSource] do
    if vehicle[sectionNameSource][i] then
      vehicle.triggers[rowCounter + i] = vehicle[sectionNameSource][i]
    end
  end

  -- kill the source section to prevent it getting processed down the line
  vehicle[sectionNameSource] = nil
end

local function mergeNamedSections(vehicle, sectionNameTarget, sectionNameSource)
  -- safety guards
  vehicle.validTables = vehicle.validTables or {}
  vehicle.validTables[sectionNameTarget] = true
  vehicle[sectionNameTarget] = vehicle[sectionNameTarget] or {}
  vehicle[sectionNameSource] = vehicle[sectionNameSource] or {}

  -- add the source keys to target
  for k, v in pairs(vehicle[sectionNameSource]) do
    if vehicle[sectionNameTarget][k] then
      log('W', 'overwriting row: ' .. tostring(k) .. ' from section ' .. tostring(sectionNameTarget) .. ' into section ' .. tostring(sectionNameTarget))
    end
    v.originSection = sectionNameSource
    vehicle[sectionNameTarget][k] = v
  end

  -- kill the source section to prevent it getting processed down the line
  vehicle[sectionNameSource] = nil
  vehicle.validTables[sectionNameSource] = nil
end


local function process(vehicle, sectionRenames)
  profilerPushEvent('jbeam/sectionMerger.process')

  -- merge triggers2 into triggers
  mergeNamedSections(vehicle, 'triggers', 'triggers2')
  sectionRenames['triggers2'] = 'triggers'

  profilerPopEvent('jbeam/sectionMerger.process')
  return true
end

M.process = process

return M
