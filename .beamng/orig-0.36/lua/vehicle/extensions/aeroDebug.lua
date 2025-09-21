-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local abs = math.abs
local aeroData = {}

local frontRightWheelPos = nil
local frontLeftWheelPos = nil
local rearLeftWheelPos = nil
local rearRightWheelPos = nil

local isEnabled = false

local function onInit()
  isEnabled = false
end

local function updateGFX(dt)
  if not isEnabled then
    return
  end
  --obj:setWind(0,55.55,0)
  --obj:setWind(0,0,55.55)
  M.directionVector = -obj:getDirectionVector()
  M.directionVectorUp = obj:getDirectionVectorUp()
  --M.directionVectorLeft = M.directionVectorUp:cross(M.directionVector)
  M.directionVectorLeft = M.directionVector:cross(M.directionVectorUp)

  --local cog = obj:calcCenterOfGravityRel(true) + obj:getPosition()
  local cop = obj:calcCenterOfPressureRel()
  local copVec = obj:calcCenterOfPressureRel() + obj:getPosition()

  aeroData.vehID = obj:getID()
  aeroData.totalAeroForce = obj:calcTotalAeroForces()
  aeroData.totalAeroTorque = obj:calcTotalAeroTorque(cop)
  aeroData.test = 100

  --obj.debugDrawProxy:drawSphere(0.05, copVec, color(0,0,255,255))
  --obj.debugDrawProxy:drawSphere(0.05, cog, color(255,0,255,255))
  aeroData.totalAeroForceVehicle = vec3(aeroData.totalAeroForce:dot(M.directionVectorLeft), aeroData.totalAeroForce:dot(M.directionVector), aeroData.totalAeroForce:dot(M.directionVectorUp))
  aeroData.afX = aeroData.totalAeroForce:dot(M.directionVectorLeft)
  aeroData.afY = aeroData.totalAeroForce:dot(M.directionVector)
  aeroData.afZ = aeroData.totalAeroForce:dot(M.directionVectorUp)
  --print(M.afX..' '..M.afY..' '..M.afZ)

  aeroData.totalAeroTorqueVehicle = vec3(aeroData.totalAeroTorque:dot(M.directionVectorLeft), aeroData.totalAeroTorque:dot(M.directionVector), aeroData.totalAeroTorque:dot(M.directionVectorUp))
  aeroData.aTX = aeroData.totalAeroTorque:dot(M.directionVectorLeft)
  aeroData.aTY = aeroData.totalAeroTorque:dot(M.directionVector)
  aeroData.aTZ = aeroData.totalAeroTorque:dot(M.directionVectorUp)
  --print(M.aTX..' '..M.aTY..' '..M.aTZ)

  for _, wd in pairs(wheels.wheels) do
    local pos1 = obj:getNodePosition(wd.node1) + obj:getPosition()
    local pos2 = obj:getNodePosition(wd.node2) + obj:getPosition()
    local middlePos = (pos2 + pos1) * 0.5

    if wd.name == aeroData.wheelNameFR then
      frontRightWheelPos = middlePos
    end
    if wd.name == aeroData.wheelNameFL then
      frontLeftWheelPos = middlePos
    end
    if wd.name == aeroData.wheelNameRR then
      rearLeftWheelPos = middlePos
    end
    if wd.name == aeroData.wheelNameRL then
      rearRightWheelPos = middlePos
    end
  end

  aeroData.rearDownForce = 0
  aeroData.frontDownForce = 0
  aeroData.percentFront = 0
  aeroData.percentRear = 0

  if frontRightWheelPos and frontLeftWheelPos and rearLeftWheelPos and rearRightWheelPos then
    local frontAxlePos = (frontRightWheelPos + frontLeftWheelPos) * 0.5
    local rearAxlePos = (rearRightWheelPos + rearLeftWheelPos) * 0.5

    --obj.debugDrawProxy:drawSphere(0.02, rearAxlePos, color(255,0,0,255))
    --obj.debugDrawProxy:drawSphere(0.02, frontAxlePos, color(255,0,0,255))

    local frontAxleToCOP = copVec - frontAxlePos
    local rearAxleToCOP = copVec - rearAxlePos
    local wheelbase = (frontAxlePos - rearAxlePos):length()

    aeroData.rearDownForce = -(frontAxleToCOP:cross(aeroData.totalAeroForce) - aeroData.totalAeroTorque):dot(M.directionVectorLeft) / wheelbase
    aeroData.frontDownForce = (rearAxleToCOP:cross(aeroData.totalAeroForce) - aeroData.totalAeroTorque):dot(M.directionVectorLeft) / wheelbase

    aeroData.percentFront = aeroData.frontDownForce / (abs(aeroData.frontDownForce) + abs(aeroData.rearDownForce)) * 100
    aeroData.percentRear = aeroData.rearDownForce / (abs(aeroData.frontDownForce) + abs(aeroData.rearDownForce)) * 100

  --print(aeroData.frontDownForce..' '..aeroData.rearDownForce..' '..aeroData.afX..' '..aeroData.afY..' '..aeroData.afZ)
  --print("%F:"..' '..string.format("%.2f",aeroData.percentFront)..' '.."%R:"..' '..string.format("%.2f",aeroData.percentRear)..' '.."Net Force X:"..' '..string.format("%.2f",aeroData.afX)..' '.."Y:"..''..string.format("%.2f",aeroData.afY)..' '.."Z:"..''..string.format("%.2f",aeroData.afZ))
  end
end

local function getAeroData()
  return aeroData
end

local function setWheelNames(FL, FR, RL, RR)
  aeroData.wheelNameFL = FL
  aeroData.wheelNameFR = FR
  aeroData.wheelNameRL = RL
  aeroData.wheelNameRR = RR
end

local function enable()
  if not aeroData.wheelNameStrings then
    aeroData.wheelNameStrings = {}
    for _, wd in pairs(wheels.wheels) do
      table.insert(aeroData.wheelNameStrings, wd.name)
    end
  end
  isEnabled = true
end

local function disable()
  isEnabled = false
  aeroData.wheelNameStrings = nil
end

-- public interface
M.onInit = onInit
M.updateGFX = updateGFX
M.getAeroData = getAeroData
M.setWheelNames = setWheelNames

M.enable = enable
M.disable = disable

return M
