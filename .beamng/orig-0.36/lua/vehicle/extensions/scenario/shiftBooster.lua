-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local boostTime = 0

local function applyBoost(strength)
  if strength == nil then
    strength = 300
  end

  local forceVec = obj:getDirectionVector()
  -- get a boost vector
  forceVec = forceVec * strength
  --[[
    for _, node in pairs(v.data.nodes) do
        print("node = " .. tostring(node.cid) .. ' > ' .. tostring(forceVec))
        obj:applyForceVector(node.cid, forceVec)
    end
    ]]
  print("applyBoost > " .. tostring(forceVec))
  obj:setWind(forceVec.x, forceVec.y, forceVec.z)
end

local function listenToShift()
  print("### SHIFTBOOSTER ###")
  local up_for_real = drivetrain.shiftUp
  drivetrain.shiftUp = function()
    print("### SHIFTBOOSTER ### > SHIFTUP")

    -- really shift up ...
    up_for_real()

    -- boost for 1 sec
    boostTime = 0.4

    -- show something on the UI
    ui_message("Boooooost!", 5, "boost")

    applyBoost()
  end
end

local function updateGFX(dt)
  if boostTime > 0.01 then
    boostTime = boostTime - dt
    applyBoost()
  end

  if boostTime < 0.01 then
    -- reset the wind
    obj:setWind(0, 0, 0)
  end
end

local function onDebugDraw(focusPos)
  if boostTime < 0.01 then
    return
  end
  obj.debugDrawProxy:drawSphere(boostTime, obj:getPosition() + vec3(0, 0, 2), color(math.sin(boostTime) * 255, math.cos(boostTime * 2) * 255, 0, 255))
end

local function onVehicleScenarioData(data)
end

local function manualBoost(duration, strength)
  boostTime = duration
  applyBoost(strength)
end

M.listenToShift = listenToShift
M.onDebugDraw = onDebugDraw
M.updateGFX = updateGFX
M.manualBoost = manualBoost
M.onVehicleScenarioData = onVehicleScenarioData

return M
