-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local min = math.min
local max = math.max

--== PID Parallel ==--
--parallel/ideal PID form
local PIDParallel = {}
PIDParallel.__index = PIDParallel

--Usage:
--local myPID = newPIDParallel(1, 0.5, 0.1, 0, 1)
--local control = myPID:get(processVariable, setPoint, dt)
--This PID uses derivative calculation based on the process variable rather than the error to avoid spikes when changing the setpoint
function newPIDParallel(kP, kI, kD, minOutput, maxOutput, integralInCoef, integralOutCoef, minIntegral, maxIntegral)
  local data = {
    kP = kP,
    kI = kI,
    kD = kD,
    error = 0,
    integral = 0,
    output = 0,
    integralInCoef = integralInCoef or 1,
    integralOutCoef = integralOutCoef or 1,
    lastProcessVariable = 0,
    minOutput = minOutput or -math.huge,
    maxOutput = maxOutput or math.huge,
    get = PIDParallel.getMethod
  }
  data.maxIntegral = maxIntegral or data.maxOutput / kI
  data.minIntegral = minIntegral or -data.maxIntegral
  setmetatable(data, PIDParallel)
  return data
end

function PIDParallel:setConfig(kP, kI, kD, minOutput, maxOutput, integralInCoef, integralOutCoef, minIntegral, maxIntegral)
  self.kP = kP or self.kP
  self.kI = kI or self.kI
  self.kD = kD or self.kD

  self.integralInCoef = integralInCoef or self.integralInCoef
  self.integralOutCoef = integralOutCoef or self.integralOutCoef

  self.minOutput = minOutput or self.minOutput
  self.maxOutput = maxOutput or self.maxOutput

  self.maxIntegral = maxIntegral or self.maxOutput / kI
  self.minIntegral = minIntegral or -self.maxIntegral
end

function PIDParallel:getMethod(processVariable, setPoint, dt)
  self.error = setPoint - processVariable
  local integral = self.integral
  integral = min(max(integral + self.error * (self.error > 0 and self.integralOutCoef or self.integralInCoef) * dt, self.minIntegral), self.maxIntegral)
  self.output = min(max(self.kP * self.error + self.kI * integral + self.kD * (self.lastProcessVariable - processVariable) / dt, self.minOutput), self.maxOutput)
  self.integral = integral

  self.lastProcessVariable = processVariable

  return self.output, self.error --return control value and error, error can be used to check if the PID reached a somewhat steady state
end

function PIDParallel:getWithDebugMethod(processVariable, setPoint, dt)
  local result = self:getMethod(processVariable, setPoint, dt)
  self:drawDebug()
  return result
end

function PIDParallel:reset()
  self.error = 0
  self.output = 0
  self.integral = 0
  self.lastProcessVariable = 0
end

function PIDParallel:drawDebug()
  guihooks.graph({"error", self.error},{"integral",self.integral},{"output", self.output})
end

function PIDParallel:setDebug(debugEnabled)
  self.get = debugEnabled and self.getWithDebugMethod or self.getMethod
end

function PIDParallel:dump()
  print(string.format("PID Parallel: kP: %.2f, kI: %.2f, kD: %.2f, Min: %.2f, Max: %.2f, Min Integral %.2f, Max Integral: %.2f", self.kP, self.kI, self.kD, self.minOutput, self.maxOutput, self.minIntegral, self.maxIntegral))
end

--== PID Standard ==--
--Standard PID form
local PIDStandard = {}
PIDStandard.__index = PIDStandard

--Usage:
--local myPID = newPIDStandard(1, 0.5, 0.1, 0, 1)
--local control = myPID:get(processVariable, setPoint, dt)
--This PID uses derivative calculation based on the process variable rather than the error to avoid spikes when changing the setpoint
function newPIDStandard(kP, tI, tD, minOutput, maxOutput, integralInCoef, integralOutCoef, minIntegral, maxIntegral)
  local data = {
    kP = kP,
    tICoef = tI > 0 and 1 / tI or 0, --integral time - try to eliminate past errors within this time, pre-calculate the 1 / tI for optimization purposes
    tD = tD, -- derivative time - try to predict error this time in the future
    error = 0,
    integral = 0,
    output = 0,
    integralInCoef = integralInCoef or 1,
    integralOutCoef = integralOutCoef or 1,
    lastProcessVariable = 0,
    minOutput = minOutput or -math.huge,
    maxOutput = maxOutput or math.huge,
    get = PIDStandard.getMethod
  }
  data.maxIntegral = maxIntegral or data.maxOutput
  data.minIntegral = minIntegral or -data.maxIntegral
  setmetatable(data, PIDStandard)
  return data
end

function PIDStandard:setConfig(kP, tI, tD, minOutput, maxOutput, integralInCoef, integralOutCoef, minIntegral, maxIntegral)
  self.kP = kP or self.kP
  self.tICoef = (tI and tI > 0) and 1 / tI or self.tICoef
  self.tD = tD or self.tD

  self.integralInCoef = integralInCoef or self.integralInCoef
  self.integralOutCoef = integralOutCoef or self.integralOutCoef

  self.minOutput = minOutput or self.minOutput
  self.maxOutput = maxOutput or self.maxOutput

  self.maxIntegral = maxIntegral or self.maxOutput
  self.minIntegral = minIntegral or -self.maxIntegral
end

function PIDStandard:getMethod(processVariable, setPoint, dt)
  self.error = setPoint - processVariable
  local integral = self.integral
  integral = min(max(integral + self.error * (self.error > 0 and self.integralOutCoef or self.integralInCoef) * dt, self.minIntegral), self.maxIntegral)
  self.output = min(max(self.kP * (self.error + self.tICoef * integral + self.tD * (self.lastProcessVariable - processVariable) / dt), self.minOutput), self.maxOutput)
  self.integral = integral

  self.lastProcessVariable = processVariable

  return self.output, self.error --return control value and error, error can be used to check if the PID reached a somewhat steady state
end

function PIDStandard:getWithDebugMethod(processVariable, setPoint, dt)
  local result = self:getMethod(processVariable, setPoint, dt)
  self:drawDebug()
  return result
end

function PIDStandard:reset()
  self.integral = 0
  self.lastProcessVariable = 0
end

function PIDStandard:drawDebug()
  guihooks.graph({"error", self.error},{"integral",self.integral},{"output", self.output})
end

function PIDStandard:setDebug(debugEnabled)
  self.get = debugEnabled and self.getWithDebugMethod or self.getMethod
end

function PIDStandard:dump()
  print(string.format("PID Standard: kP: %.2f, kI: %.2f, kD: %.2f, Min: %.2f, Max: %.2f, Min Integral %.2f, Max Integral: %.2f", self.kP, self.kI, self.kD, self.minOutput, self.maxOutput, self.minIntegral, self.maxIntegral))
end
