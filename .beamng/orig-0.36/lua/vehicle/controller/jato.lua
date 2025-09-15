-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxilliary"
M.defaultOrder = 1100

M.fuelCoef = 1

local max, min = math.max, math.min

local fuel = 0
local initialFuel = 0
local initialFuelCoef = 0
local lastJatoInput = 0
local lastFuelRatio = 0

local thrusterCount = 0
local thrusterNodes

local smokeTick = 0

local thrusterLoopName
local thrusterLoops1
local thrusterLoops2

local function inputStringCallback(string)
  M.fuelCoef = 1 - M.fuelCoef
  if M.fuelCoef <= 0 then
    print("When in doubt, JATO")
  end
end

local function updateSimpleControlButtons()
  extensions.ui_simplePowertrainControl.setButton("jato_" .. M.name, "JATO", "powertrain_jato", "0072bc", lastFuelRatio, "electrics.values.jatoInput = 1 - (electrics.values.jatoInput or 0)")
end

local function updateGFX(dt)
  local jatoInput = min(electrics.values.jatoInput or 0, 1)
  local throttleInput = input.throttle or 0
  local gearIndex = electrics.values.gearIndex or 0

  if throttleInput == 1 and gearIndex >= 0 and (input.state.throttle.filter == FILTER_PAD or input.state.throttle.filter == FILTER_DIRECT) then
    jatoInput = 1
  end

  fuel = max(fuel - jatoInput * dt * M.fuelCoef, 0)

  if fuel <= 0 then
    jatoInput = 0
  end

  smokeTick = smokeTick + dt * jatoInput * 100
  if smokeTick > 1 then
    for i = 1, thrusterCount, 1 do
      local thruster = thrusterNodes[i]
      obj:addParticleByNodes(thruster[1], thruster[2], 20, 81, 0.01, 1)
    end
    smokeTick = 0
  end

  if jatoInput > 0 and lastJatoInput <= 0 then
    thrusterLoops1 = thrusterLoops1 or {}
    for i = 1, thrusterCount do
      if thrusterLoops2 and thrusterLoops2[i] then
        obj:setVolume(thrusterLoops2[i], 0)
      end
      thrusterLoops1[i] = thrusterLoops1[i] or obj:createSFXSource2(thrusterLoopName, "AudioDefaultLoop3D", "jatoThrusterLoop", thrusterNodes[i][1], 0)
      obj:setVolume(thrusterLoops1[i], jatoInput)
      obj:playSFX(thrusterLoops1[i])
    end
  end

  if jatoInput <= 0 and lastJatoInput > 0 then
    for i = 1, thrusterCount do
      obj:stopSFX(thrusterLoops1 and thrusterLoops1[i] or -1)
    end
    thrusterLoops1, thrusterLoops2 = thrusterLoops2, thrusterLoops1
  end

  local fuelRatio = fuel * initialFuelCoef

  electrics.values.jato = jatoInput
  electrics.values.jatofuel = fuelRatio
  lastJatoInput = jatoInput

  if fuelRatio ~= lastFuelRatio then
    lastFuelRatio = fuelRatio
    updateSimpleControlButtons()
  end
end

local function reset()
  fuel = initialFuel
  lastFuelRatio = 0
  smokeTick = 0

  for i = 1, thrusterCount do
    obj:stopSFX(thrusterLoops1 and thrusterLoops1[i] or -1)
    obj:stopSFX(thrusterLoops2 and thrusterLoops2[i] or -1)
  end

  electrics.values.jatoInput = 0
  electrics.values.jato = 0
end

local function init(jbeamData)
  fuel = jbeamData.fuel or 20
  initialFuel = fuel
  if initialFuel > 0 then
    initialFuelCoef = 1 / initialFuel
  end

  smokeTick = 0

  electrics.values.jatoInput = 0
  electrics.values.jato = 0

  thrusterNodes = {}
  if jbeamData.thrusterNodes_nodes then
    local nodeCount = #jbeamData.thrusterNodes_nodes
    if nodeCount % 2 == 0 then
      for i = 1, nodeCount, 2 do
        table.insert(thrusterNodes, {jbeamData.thrusterNodes_nodes[i], jbeamData.thrusterNodes_nodes[i + 1]})
      end
    else
      log("E", "jato.init", "Even number of thruster nodes are required!")
    end
  end

  thrusterCount = #thrusterNodes
  thrusterLoopName = jbeamData.thrusterLoopName or "event:>Vehicle>Thrusters"

  extensions.inputAnalyzer.registerInputString("uuddlrlrba", inputStringCallback)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.inputStringCallback = inputStringCallback

M.updateSimpleControlButtons = updateSimpleControlButtons

return M
