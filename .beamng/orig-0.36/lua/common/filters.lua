-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This file contains several filters. Please refer to the documentation created by BeamNG

local max, min, abs, cos, sqrt = math.max, math.min, math.abs, math.cos, math.sqrt
local pi2 = 2 * math.pi

--MARK: Freq

function freqGenC(period, ampl, t)
  return cos(t * pi2/(period + 1e-30)) * ampl
end

-- 1st order filters
local freqFilter1 = {}
freqFilter1.__index = freqFilter1

function newFreqFilter1()
  local data = {
    state = 0, state2 = 0, x = 0
  }
  return setmetatable(data, freqFilter1)
end

function freqFilter1:getLowPassFreq(sample, dt, cutOffFreq)
  local pfc, state = pi2 * dt * cutOffFreq, self.state
  state = state + pfc * (sample - state) / (pfc + 1)
  self.state = state
  return state
end

function freqFilter1:getLowPassPeriod(sample, dt, cutOffPeriod)
  local pdt, state = pi2 * dt, self.state
  state = state + pdt * (sample - state) / (pdt + cutOffPeriod)
  self.state = state
  return state
end

function freqFilter1:getHighPassFreq(sample, dt, cutOffFreq)
  local state = (self.state + sample - self.x) / (pi2 * dt * cutOffFreq + 1)
  self.state, self.x = state, sample
  return state
end

function freqFilter1:getHighPassPeriod(sample, dt, cutOffPeriod)
  local state = (self.state + sample - self.x) * cutOffPeriod / (pi2 * dt + cutOffPeriod)
  self.state, self.x = state, sample
  return state
end

function freqFilter1:getBandPassFreq(sample, dt, lowFreq, highFreq)
  local pdt = pi2 * dt
  local state = (self.state + sample - self.x) / (pdt * lowFreq + 1)
  local state2, pfc = self.state2, pdt * highFreq
  state2 = state2 + pfc * (state - state2) / (pfc + 1)
  self.state, self.state2, self.x = state, state2, sample
  return state2
end

function freqFilter1:getBandPassPeriod(sample, dt, lowPeriod, highPeriod)
  local pdt, state2 = pi2 * dt, self.state2
  local state = (self.state + sample - self.x) * lowPeriod / (pdt + lowPeriod)
  local state2 = state2 + pdt * (state - state2) / (pdt + highPeriod)
  self.state, self.state2, self.x = state, state2, sample
  return state2
end

function freqFilter1:getBandStopFreq(sample, dt, lowFreq, highFreq)
  local pdt = pi2 * dt
  local state = (self.state + sample - self.x) / (pdt * highFreq + 1)
  local state2, pfc = self.state2, pdt * lowFreq
  state2 = state2 + pfc * (sample - state2) / (pfc + 1)
  self.state, self.state2, self.x = state, state2, sample
  return state + state2
end

function freqFilter1:getBandStopPeriod(sample, dt, lowPeriod, highPeriod)
  local pdt, state2 = pi2 * dt, self.state2
  local state = (self.state + sample - self.x) * highPeriod/ (pdt + highPeriod)
  state2 = state2 + pdt * (sample - state2) / (pdt + lowPeriod)
  self.state, self.state2, self.x = state, state2, sample
  return state + state2
end

-- Frequency detector
local freqDetector = {}
freqDetector.__index = freqDetector

function newFreqDetector()
  local data = {
    peak = 0, bottom = 0, prevPeak = 0, prevBottom = 0,
    peakDt = 0, bottomDt = 0, prevPeakDt = 0, prevBottomDt = 0,
    peakL = 0, bottomL = 0, period = 0, ampl = 0, state = 0, bias = 0
  }
  return setmetatable(data, freqDetector)
end

-- returns period, peak amplitude, previous peakDt, DC bias: cos(peakDt*2*pi/(period+1e-30)) * ampl + bias
function freqDetector:get(sample, dt)
  self.peakDt, self.prevPeakDt = self.peakDt + dt, self.prevPeakDt + dt
  self.bottomDt, self.prevBottomDt = self.bottomDt + dt, self.prevBottomDt + dt
  local scoef = min(1, max(self.peakL - self.bottomL, 0) * 0.01)

  if sample > self.peakL then
    if self.state ~= -1 and self.prevBottomDt > self.peakDt and self.peakDt > self.bottomDt then
      self.period = self.prevBottomDt - self.bottomDt
      local midPeak = 0.5 * (self.peak + self.prevPeak)
      self.ampl = max(0, midPeak - self.prevBottom) * 0.5
      self.bias = (midPeak + self.prevBottom) * 0.5
      self.prevPeak = self.peak
      self.prevPeakDt = (self.peakDt + self.prevBottomDt + self.bottomDt) / 3
      self.state = -1
    end
    self.peakDt, self.peakL, self.peak = 0, sample, sample
  else
    self.peakL = self.peakL + scoef * (sample - self.peakL)
  end

  if sample < self.bottomL then
    if self.state ~= 1 and self.prevPeakDt > self.bottomDt and self.bottomDt > self.peakDt then
      self.period = self.prevPeakDt - self.peakDt
      local midBottom = 0.5 * (self.bottom + self.prevBottom)
      self.ampl = max(0, self.prevPeak - midBottom) * 0.5
      self.bias = (self.prevPeak + midBottom) * 0.5
      self.prevBottom = self.bottom
      self.prevBottomDt = (self.bottomDt + self.prevPeakDt + self.peakDt) / 3
      self.state = 1
    end
    self.bottomDt, self.bottomL, self.bottom = 0, sample, sample
  else
    self.bottomL = min(self.peakL, self.bottomL + scoef * (sample - self.bottomL))
  end

  local period = max(self.period, self.prevPeakDt - self.period)
  local pcoef = self.period / (period + 1e-30)
  return period, pcoef * self.ampl, self.prevPeakDt, pcoef * self.bias
end

function freqDetector:reset()
  self.peak, self.bottom, self.peakL, self.bottomL, self.state = 0, 0, 0, 0, 0
  self.period, self.ampl, self.bias = 0, 0, 0
end

function freqDetector:value(dt)
  dt = dt or 0
  self.peakDt, self.bottomDt = self.peakDt + dt, self.bottomDt + dt
  self.prevPeakDt, self.prevBottomDt = self.prevPeakDt + dt, self.prevBottomDt + dt
  return self.period, self.ampl, self.prevPeakDt, self.bias
end

-- Frequency existence: https://en.wikipedia.org/wiki/Goertzel_algorithm
local freqExists = {}
freqExists.__index = freqExists

function newFreqExists()
  local data = { s_prev = 0, s_prev2 = 0, cycleDt = 0, lastAmpl = 0, N = 0 }
  return setmetatable(data, freqExists)
end

-- returns amplitude of freq, larger cycleCount increases accuracy and latency
function freqExists:get(sample, dt, freq, cycleCount)
  local coeff, s_prev, cycleDt = 2*cos(pi2 * freq * dt), self.s_prev, self.cycleDt + dt
  local s = coeff * s_prev + (sample - self.s_prev2)
  self.N = self.N + 0.5
  if cycleDt * freq >= (cycleCount or 1) then
    self.lastAmpl = math.sqrt(max(0, s_prev*(s_prev - coeff*s) + s*s)) / self.N
    self.s_prev, self.s_prev2, self.cycleDt, self.N = 0, 0, 0, 0
  else
    self.s_prev2, self.s_prev, self.cycleDt = s_prev, s, cycleDt
  end
  return self.lastAmpl
end

-- returns per sample sliding amplitude of freq, larger winCoef increases accuracy and latency
function freqExists:getS(sample, dt, freq, winCoef)
  local fdt = freq * dt
  if fdt >= 0.5 then return 0 end
  winCoef = winCoef or 10
  local pfd = pi2 * fdt
  local coeff, s_prev = 2*cos(pfd), self.s_prev
  local s = (coeff * s_prev + (sample - self.s_prev2)) * (winCoef / (pfd + winCoef))
  self.s_prev2, self.s_prev = s_prev, s
  return sqrt(max(0, s_prev*(s_prev - coeff*s) + s*s)) * 0.48 / (winCoef*(0.5 - fdt))
end

function freqExists:bufferStart(dt, freq)
  self.s_prev, self.s_prev2, self.N, self.cycleDt = 0, 0, 0, 2*cos(pi2 * freq * dt)
end

function freqExists:bufferAdd(sample)
  local s_prev = self.s_prev
  local s = self.cycleDt * s_prev + (sample - self.s_prev2)
  self.s_prev2, self.s_prev, self.N = s_prev, s, self.N + 0.5
end

function freqExists:bufferEnd()
  local ampl = math.sqrt(max(0, self.s_prev2*(self.s_prev2 - self.cycleDt*self.s_prev) + self.s_prev*self.s_prev)) / self.N
  self.s_prev, self.s_prev2, self.N = 0, 0, 0
  return ampl
end

function freqExists:reset()
  self.s_prev, self.s_prev2, self.cycleDt, self.N, self.lastAmpl = 0, 0, 0, 0, 0
end

--MARK: Temporal

-- Spring based temporal
local temporalSpring = {}
temporalSpring.__index = temporalSpring

function newTemporalSpring(spring, damp, startingValue)
  local data = {spring = spring or 10, damp = damp or 2, state = startingValue or 0, vel = 0}
  return setmetatable(data, temporalSpring)
end

function temporalSpring:get(sample, dt)
  self.vel = self.vel * max(1 - self.damp * dt, 0) + (sample - self.state) * min(self.spring * dt, 1/dt)
  self.state = self.state + self.vel * dt
  return self.state
end

function temporalSpring:getWithSpringDamp(sample, dt, spring, damp)
  self.vel = self.vel * max(1 - damp * dt, 0) + (sample - self.state) * min(spring * dt, 1/dt)
  self.state = self.state + self.vel * dt
  return self.state
end

function temporalSpring:set(sample)
  self.state = sample
  self.vel = 0
end

function temporalSpring:value()
  return self.state
end

-- S-curve temporal
local temporalSigmoidSmoothing = {}
temporalSigmoidSmoothing.__index = temporalSigmoidSmoothing

function newTemporalSigmoidSmoothing(inRate, startAccel, stopAccel, outRate, startingValue)
  local rate = inRate or 1
  local startaccel = startAccel or math.huge
  local data = {[false] = rate, [true] = outRate or rate, startAccel = startaccel, stopAccel = stopAccel or startaccel, state = startingValue or 0, prevvel = 0}
  return setmetatable(data, temporalSigmoidSmoothing)
end

function temporalSigmoidSmoothing:get(sample, dt)
  local dif = sample - self.state

  local prevvel = self.prevvel * max(sign(self.prevvel * dif), 0)
  local vsq = prevvel * prevvel
  local absdif = abs(dif)
  local difsign = sign(dif)
  local acceldt

  local absdif2 = absdif * 2
  if vsq > absdif2 * self.stopAccel then
    acceldt = -difsign * min((vsq / absdif2) * dt, abs(prevvel))
  else
    acceldt = difsign * self.startAccel * dt
  end

  local ratelimit = self[dif * self.state >= 0]
  self.state = self.state + difsign * min(min(abs(prevvel + 0.5 * acceldt), ratelimit) * dt, absdif)
  self.prevvel = difsign * min(abs(prevvel + acceldt), ratelimit)
  return self.state
end

function temporalSigmoidSmoothing:getWithRateAccel(sample, dt, ratelimit, startAccel, stopAccel)
  local dif = sample - self.state
  local prevvel = self.prevvel * max(sign(self.prevvel * dif), 0)
  local vsq = prevvel * prevvel
  local absdif = abs(dif)
  local difsign = sign(dif)
  local acceldt

  local absdif2 = absdif * 2
  if vsq > absdif2 * (stopAccel or startAccel) then
    acceldt = -difsign * min((vsq / absdif2) * dt, abs(prevvel))
  else
    acceldt = difsign * startAccel * dt
  end

  self.state = self.state + difsign * min(min(abs(prevvel + 0.5 * acceldt), ratelimit) * dt, absdif)
  self.prevvel = difsign * min(abs(prevvel + acceldt), ratelimit)
  return self.state
end

function temporalSigmoidSmoothing:set(sample)
  self.state = sample
  self.prevvel = 0
end

function temporalSigmoidSmoothing:reset()
  self.state = 0
  self.prevvel = 0
end

function temporalSigmoidSmoothing:value()
  return self.state
end

-- Exponential/Non Linear temporal
local temporalSmoothingNonLinear = {}
temporalSmoothingNonLinear.__index = temporalSmoothingNonLinear

function newTemporalSmoothingNonLinear(inRate, outRate, startingValue)
  local rate = min(inRate or 1, 1e+30)
  local data = {[false] = rate, [true] = min(outRate or rate, 1e+30), state = startingValue or 0}
  return setmetatable(data, temporalSmoothingNonLinear)
end

function temporalSmoothingNonLinear:get(sample, dt)
  local st = self.state
  local dif = sample - st
  local ratedt = self[dif * st >= 0] * dt
  st = st + dif * ratedt / (1 + ratedt)
  self.state = st
  return st
end

function temporalSmoothingNonLinear:getWithRate(sample, dt, rate)
  local st = self.state
  local ratedt = rate * dt
  st = st + (sample - st) * ratedt / (1 + ratedt)
  self.state = st
  return st
end

function temporalSmoothingNonLinear:set(sample)
  self.state = sample
end

function temporalSmoothingNonLinear:value()
  return self.state
end

function temporalSmoothingNonLinear:reset()
  self.state = 0
end

-- Linear temporal
local temporalSmoothing = {}
temporalSmoothing.__index = temporalSmoothing

function newTemporalSmoothing(inRate, outRate, autoCenterRate, startingValue)
  inRate = max(inRate or 1, 1e-307)
  startingValue = startingValue or 0

  local data = {[false] = inRate, [true] = max(outRate or inRate, 1e-307), autoCenterRate = max(autoCenterRate or inRate, 1e-307),
                _startingValue = startingValue, state = startingValue}
  setmetatable(data, temporalSmoothing)

  if data.autoCenterRate ~= inRate then
    data.getUncapped = data.getUncappedAutoCenter
  end
  return data
end

function temporalSmoothing:getUncappedAutoCenter(sample, dt)
  local st = self.state
  local dif = (sample - st)
  local rate

  if sample == 0 then
    rate = self.autoCenterRate  -- autocentering
  else
    rate = self[dif * st >= 0]
  end
  st = st + dif * min(rate * dt / abs(dif), 1)
  self.state = st
  return st
end

function temporalSmoothing:get(sample, dt)
  local st = self.state
  local dif = sample - st
  st = st + dif * min(self[dif * st >= 0] * dt / abs(dif), 1)
  self.state = st
  return st
end

function temporalSmoothing:getCapped(sample, dt)
  return max(min(self:getUncapped(sample, dt), 1), -1)
end

function temporalSmoothing:getWithRate(sample, dt, rate)
  local st = self.state
  local dif = (sample - st)
  st = st + dif * min(rate * dt / (abs(dif) + 1e-307), 1)
  self.state = st
  return st
end

function temporalSmoothing:getWithRateCapped(sample, dt, rate)
  return max(min(self:getWithRate(sample, dt, rate), 1), -1)
end

temporalSmoothing.getUncapped = temporalSmoothing.get
temporalSmoothing.getWithRateUncapped = temporalSmoothing.getWithRate

function temporalSmoothing:reset()
  self.state = self._startingValue
end

function temporalSmoothing:value()
  return self.state
end

function temporalSmoothing:set(v)
  self.state = v
end

--MARK: Fixed dt

-- Linear
local linearSmoothing = {}
linearSmoothing.__index = linearSmoothing

function newLinearSmoothing(dt, inRate, outRate)
  inRate = max(inRate or 1, 1e-307)
  local data = {[false] = inRate * dt, [true] = max(outRate or inRate, 1e-307) * dt, state = 0}
  return setmetatable(data, linearSmoothing)
end

function linearSmoothing:get(sample) -- no autocenter
  local st = self.state
  local dif = (sample - st)
  st = st + dif * min(self[dif * st >= 0] / abs(dif), 1)
  self.state = st
  return st
end

function linearSmoothing:set(v)
  self.state = v
end

function linearSmoothing:reset()
  self.state = 0
end

local exponentialSmoothing = {}
exponentialSmoothing.__index = exponentialSmoothing

function newExponentialSmoothing(window, startingValue, fixedDt)
  local data = {a = 2 / max(window, 2), _startingValue = startingValue or 0, st = startingValue or 0}
  local adt = data.a * (fixedDt or 0.0005)
  data.a = (2000 + data.a) * adt / (1 + adt)
  return setmetatable(data, exponentialSmoothing)
end

function exponentialSmoothing:get(sample)
  local st = self.st
  st = st + self.a * (sample - st)
  self.st = st
  return st
end

function exponentialSmoothing:getWindow(sample, window)
  local st = self.st
  st = st + 2 * (sample - st) / max(window, 2)
  self.st = st
  return st
end

function exponentialSmoothing:value()
  return self.st
end

function exponentialSmoothing:set(value)
  self.st = value
end

function exponentialSmoothing:reset(value)
  self.st = value or self._startingValue
end

-- Exponential + Trend
local exponentialSmoothingT = {}
exponentialSmoothingT.__index = exponentialSmoothingT

function newExponentialSmoothingT(window, window2, startingValue)
  startingValue = startingValue or 0
  local data = {a = 2 / max(window, 2), a2 = 2 / max(window2 or math.huge, 2), startingValue = startingValue, st = startingValue, [true] = 0, [false] = 0}
  return setmetatable(data, exponentialSmoothingT)
end

function exponentialSmoothingT:get(sample)
  local a, a2, st, st1, st2 = self.a, self.a2, self.st, self[true], self[false]
  local samplst2 = sample - self[sample >= st]
  if (samplst2 - st) * (sample - st) < 0 then samplst2 = sample end
  st = st + a * (samplst2 - st)
  local dif = samplst2 - st
  local a2dif = a2 * dif
  if dif >= 0 then
    st1, st2 = min(dif, st1 + a2dif), min(0, st2 + a2dif)
    st = st + st1
  else
    st1, st2 = max(0, st1 + a2dif), max(dif, st2 + a2dif)
    st = st + st2
  end
  self.st, self[true], self[false] = st, st1, st2
  return st
end

function exponentialSmoothingT:getWindow(sample, window, window2)
  local st, st1, st2 = self.st, self[true], self[false]
  local samplst2 = sample - self[sample >= st]
  if (samplst2 - st) * (sample - st) < 0 then samplst2 = sample end
  st = st + (samplst2 - st) * 2 / max(window, 2)
  local dif = samplst2 - st
  local a2dif = dif * 2 / max(window2 or math.huge, 2)
  if dif >= 0 then
    st1, st2 = min(dif, st1 + a2dif), min(0, st2 + a2dif)
    st = st + st1
  else
    st1, st2 = max(0, st1 + a2dif), max(dif, st2 + a2dif)
    st = st + st2
  end
  self.st, self[true], self[false] = st, st1, st2
  return st
end

function exponentialSmoothingT:value()
  return self.st
end

function exponentialSmoothingT:set(value)
  self.st = value
end

function exponentialSmoothingT:reset(value)
  self.st, self[true], self[false] = value or self.startingValue, 0, 0
end