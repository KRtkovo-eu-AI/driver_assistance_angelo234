-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local testResults
local cancelTestRequested = false

local function getTestSequence(vehId)
  local sequence = {}

  table.insert(sequence, extensions.util_stepHandler.makeStepReturnTrueFunction(
    function()
      local vehicle = getObjectByID(vehId)
      local spawnPoint = scenetree.findObject("skidpadSpawn")
      spawn.safeTeleport(vehicle, spawnPoint:getPosition(), quat(0,0,1,0) * spawnPoint:getRotation(), nil, nil, nil, true, false)
      testResults = nil
      return true
    end
  ))

  table.insert(sequence, extensions.util_stepHandler.makeStepWait(0.2))

  table.insert(sequence, extensions.util_stepHandler.makeStepReturnTrueFunction(
    function()
      local vehicle = getObjectByID(vehId)

      local config = {
        wpTargetList = {"skidpadWP1", "skidpadWP2", "skidpadWP3", "skidpadWP4", "skidpadWP1"},
        noOfLaps = 2,
        aggression = 1
      }

      -- Make the AI drive the route
      vehicle:queueLuaCommand('ai.driveUsingPath(' .. serialize(config) .. ')')
      return true
    end
  ))

  return sequence
end

local function startTest(vehId)
  extensions.util_stepHandler.startStepSequence(getTestSequence(vehId), function()
  end)
end

local function getData()
  return testResults
end

local function cancelTest()
  cancelTestRequested = true
end

M.getData = getData
M.startTest = startTest
M.cancelTest = cancelTest

return M