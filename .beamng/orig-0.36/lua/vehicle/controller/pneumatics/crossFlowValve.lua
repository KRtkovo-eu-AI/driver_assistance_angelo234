-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local max = math.max
local sqrt = math.sqrt

local psiToPascal = 6894.757293178
local dischargeCoefficient = 0.97 -- closer to 1.0 for rounded orifices, decreases with sharpness of edges

local sourceTank
local targetTank
local flowPipeCrossSectionArea = 0
local sourceTankMinPressure = 0
local pressureDiffForFullFlow = 0

local function updateFixedStep(dt)
  local sourcePressure = sourceTank.currentPressure
  local sourceAirDensity = sourceTank.remainingMass * sourceTank.invCapacity
  local targetAirDensity = targetTank.remainingMass * targetTank.invCapacity
  local avgAirDensity = (sourceAirDensity + targetAirDensity) * 0.5
  local targetPressure = targetTank.currentPressure
  local flowRate = 0

  if sourcePressure >= sourceTankMinPressure and sourcePressure > targetPressure then
    local pressureDiff = max(0, sourcePressure - targetPressure)
    local flowCoef = min(1, pressureDiff / pressureDiffForFullFlow)

    flowRate = flowCoef * dischargeCoefficient * flowPipeCrossSectionArea * sqrt(2 * pressureDiff / avgAirDensity)
  end

  local airVolumeMoved = flowRate * dt
  local airEnergyMoved = min(sourceTank.storedEnergy, sourcePressure * airVolumeMoved)

  if airEnergyMoved > 0 then
    -- Remove energy from source tank and add it to target tank
    sourceTank.storedEnergy = sourceTank.storedEnergy - airEnergyMoved
    sourceTank.currentPressure = sourceTank.storedEnergy * sourceTank.invCapacity

    targetTank.storedEnergy = targetTank.storedEnergy + airEnergyMoved
    targetTank.currentPressure = targetTank.storedEnergy * targetTank.invCapacity
  end
end

local function reset()

end

local function init(jbeamData)
  local sourceTankName = jbeamData.sourceTankName or "mainAirTank"
  local targetTankName = jbeamData.targetTankName or "auxAirTank"

  sourceTank = energyStorage.getStorage(sourceTankName)
  targetTank = energyStorage.getStorage(targetTankName)

  if not sourceTank then
    log("D", "crossFlowValve.init", "Source tank not found: " .. sourceTankName)
    M.updateFixedStep = nop
  end
  if not targetTank then
    log("D", "crossFlowValve.init", "Target tank not found: " .. targetTankName)
    M.updateFixedStep = nop
  end

  local flowPipeRadius = jbeamData.flowPipeRadius or 0.0075 -- m

  flowPipeCrossSectionArea = math.pi * flowPipeRadius ^ 2

  sourceTankMinPressure = jbeamData.sourceTankMinPressure or 0
  if type(jbeamData.sourceTankMinPressurePSI) == "number" then
    sourceTankMinPressure = jbeamData.sourceTankMinPressurePSI * psiToPascal + 101325
  end

  pressureDiffForFullFlow = jbeamData.pressureDiffForFullFlow or 34473.8 -- Pascals; default = 5 PSI
  if type(jbeamData.pressureDiffForFullFlowPSI) == "number" then
    pressureDiffForFullFlow = jbeamData.pressureDiffForFullFlowPSI * psiToPascal -- This is a pressure delta; don't add atmospheric pressure!
  end
end

M.init = init
M.reset = reset
M.updateFixedStep = updateFixedStep

return M
