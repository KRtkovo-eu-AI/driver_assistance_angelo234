-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local understeerAssistedSurfaces = {METAL = 1, PLASTIC = 1, RUBBER = 1, GLASS = 1, WOOD = 1, ASPHALT = 1, ROCK = 1, RUMBLE_STRIP = 1, COBBLESTONE = 1} -- taken from game\lua\common\particles.json

local M = {}

M.keys = {} -- Backwards compatibility
local MT = {} -- metatable
local keysDeprecatedWarned
MT.__index = function(tbl, key)
  if not keysDeprecatedWarned then
    log("E", "", "Vehicle " .. dumps(vehiclePath) .. " tried to use input.keys[" .. dumps(key) .. "] which is outdated. The vehicle creator should instead use vehicle-specific bindings (see https://go.beamng.com/vehicleSpecificBindings for more info)")
    keysDeprecatedWarned = true
  end
  return rawget(M.keys, key)
end
setmetatable(M.keys, MT)
M.state = {}
M.lastFilterType = -1

M.lastInputs = {}
M.allowedInputSources = {}

--set kbd initial rates (derive these from the menu options eventually)
local kbdInRate = 2.2
local kbdOutRate = 1.6

--set kbd understeer limiting effect (A value of 1 will achieve min steering speed of 0*kbdOutRate)
local kbdUndersteerMult = 0.7
--set kbd oversteer help effect (A value of 1 will achieve max steering speed of 2*kbdOutRate)
local kbdOversteerMult = 0.7

local rateMult = nil
local kbdOutRateMult = 0
local kbdInRateMult = 0
local understeerInRateMult = 1 -- used by understeer assist to mitigate the slow return-to-center after long understeers
local understeerOutRateMult = 1 -- used by speed-sensitive assist to mitigate the fast turn-in at high speeds
local padSmoother = nil
local kbdSmoother = nil
local vehicleSteeringWheelLock = 450
local handbrakeSoundEngaging = nil
local handbrakeSoundDisengaging = nil
local handbrakeSoundDisengaged = nil
local inputNameCache = {}
local inputTSCache = {}

local gxSmoothMax = 0
local gx_Smoother = newTemporalSmoothing(4) -- it acts like a timer
local velVec = vec3()

-- oversteer reduction assistant
local stabilizationEnabled = false
local stabilizationEnabledDirect = false
local stabilizationEndSpeed = 13
local stabilizationMultiplier = 0
-- understeer reduction assistant
local understeerReductionEnabled = false
local understeerReductionEnabledDirect = false
local understeerReductionMultiplier = 0
local frontWheels = {}
local smUndersteer1 = newTemporalSmoothing()
local smUndersteer2 = newTemporalSmoothing()
local smOversteer = newTemporalSmoothing()
-- slower steering at high speed
local slowdownEnabled = false
local slowdownEnabledDirect = false
local slowdownMultiplier = nil
local slowdownM = nil
local slowdownB = nil
-- limit steering at high speed
local limitEnabled = false
local limitEnabledDirect = false
local limitMultiplier = nil
local limitM = nil
local limitB = nil
-- slower autocenter at low speed
local autocenterEnabled = false
local autocenterM = nil
local autocenterN = nil

local min, max, abs, sqrt = math.min, math.max, math.abs, math.sqrt

local function init()
  --inRate (towards the center), outRate (away from the center), autoCenterRate, startingValue
  M.state = {
    steering = {
      val = 0,
      filter = 0,
      smootherKBD = newTemporalSmoothing(),
      smootherPAD = newTemporalSmoothing(),
      minLimit = -1,
      maxLimit = 1
    },
    throttle = {
      val = 0,
      filter = 0,
      smootherKBD = newTemporalSmoothing(3, 3, 1000, 0),
      smootherPAD = newTemporalSmoothing(100, 100, nil, 0),
      minLimit = 0,
      maxLimit = 1
    },
    brake = {
      val = 0,
      filter = 0,
      smootherKBD = newTemporalSmoothing(3, 3, 1000, 0),
      smootherPAD = newTemporalSmoothing(100, 100, nil, 0),
      minLimit = 0,
      maxLimit = 1
    },
    parkingbrake = {
      val = 0,
      filter = 0,
      smootherKBD = newTemporalSmoothing(10, 10, nil, 0),
      smootherPAD = newTemporalSmoothing(10, 10, nil, 0),
      minLimit = 0,
      maxLimit = 1
    },
    clutch = {
      val = 0,
      filter = 0,
      smootherKBD = newTemporalSmoothing(10, 20, 20, 0),
      smootherPAD = newTemporalSmoothing(10, 10, nil, 0),
      minLimit = 0,
      maxLimit = 1
    }
  }
end

local function getLongitudinalLateralPrefix(wheelName)
  local long, side = string.match(wheelName, "^([FR][FR0-9_]*)([RL][RL0-9]*)")
  long = long or string.match(wheelName, "^([FR]+)")
  --print(string.format("/  Splitting '%10s' into long: '%5s', side: '%5s' .", wheelName, long, side))
  long = long and long:sub(1, 1) or long
  side = side and side:sub(1, 1) or side
  --print(string.format("\\_ Splitting '%10s' into long: '%5s', side: '%5s' .", wheelName, long, side))
  return long, side
end

local function initSecondStage()
  --scale rates based on steering wheel degrees
  local foundSteeringHydro = false

  if hydros then
    for _, h in pairs(hydros.hydros) do
      --check if it's a steering hydro
      if h.inputSource == "steering_input" then
        foundSteeringHydro = true
        --if the value is present, scale the values
        if h.steeringWheelLock then
          vehicleSteeringWheelLock = abs(h.steeringWheelLock)
          break
        end
      end
    end
  end

  if v.data.input and v.data.input.steeringWheelLock ~= nil then
    vehicleSteeringWheelLock = v.data.input.steeringWheelLock
  elseif foundSteeringHydro then
    if v.data.input == nil then
      v.data.input = {}
    end
    v.data.input.steeringWheelLock = vehicleSteeringWheelLock
  end

  for wi, wd in pairs(wheels.wheels) do
    if wd.parkingTorque and wd.parkingTorque > 0 then
      handbrakeSoundEngaging = handbrakeSoundEngaging or sounds.createSoundscapeSound("handbrakeEngaging")
      handbrakeSoundDisengaging = handbrakeSoundDisengaging or sounds.createSoundscapeSound("handbrakeDisengaging")
      handbrakeSoundDisengaged = handbrakeSoundDisengaged or sounds.createSoundscapeSound("handbrakeDisengaged")
      break
    end
  end

  -- identify and cache which are the front wheels
  table.clear(frontWheels)
  local debug = false
  for wi1, wd1 in pairs(wheels.wheels) do
    local long1, side1 = getLongitudinalLateralPrefix(wd1.name)
    if debug then
      print(string.format("----- '%s' = %s, %s", wd1.name, long1, side1))
    end
    if long1 == "F" then -- this is a front wheel
      local rearWheels = {}
      for wi2, wd2 in pairs(wheels.wheels) do
        local long2, side2 = getLongitudinalLateralPrefix(wd2.name)
        if long2 == "R" then -- this is a rear wheel
          if side1 == nil then -- we don't know the side of front wheel (maybe it's a 3-wheel pigeon)
            table.insert(rearWheels, wi2)
            if debug then
              print(string.format("    * '%s' = %s, %s", wd2.name, long2, side2))
            end
          elseif side1 == side2 then -- this rear wheel is from the same side
            table.insert(rearWheels, wi2)
            if debug then
              print(string.format("    - '%s' = %s, %s", wd2.name, long2, side2))
            end
          end
        end
      end
      if foundSteeringHydro and not next(rearWheels) then
        log("W", "", "Unable to identify any rear wheels for front wheel: "..dumps(wd1.name))
      end
      table.insert(frontWheels, {wi1, wd1, rearWheels, #rearWheels})
    end
  end
  if foundSteeringHydro and not next(frontWheels) then
    log("W", "", "Unable to identify any front wheels: driving assistants may not work")
  end
  if debug then
    for k, v in ipairs(frontWheels) do
      local namef = wheels.wheels[v[1]].name
      local namesr = {}
      for l, w in ipairs(v[3]) do
        table.insert(namesr, wheels.wheels[w].name)
      end
      print(string.format("front wheel '%s' has these rear wheels: %s", namef, dumps(namesr)))
    end
  end

  rateMult = 5 / 8
  if vehicleSteeringWheelLock ~= 1 then
    rateMult = 450 / vehicleSteeringWheelLock
  end

  kbdOutRateMult = min(kbdOutRate * rateMult, 2.68)
  kbdInRateMult = min(kbdInRate * rateMult, 3.68)
  padSmoother = newTemporalSmoothing()
  kbdSmoother = newTemporalSmoothing()

  M.reset()
end

local function dynamicInputRateKbd(v, dt, curx)
  local signv = sign(v)
  local signx = sign(curx)
  local gx = sensors.gx
  local signgx = sign(gx)
  local absgx = abs(gx)

  local gs = kbdSmoother:getWithRateUncapped(0, dt, 3)
  if absgx > gs then
    gs = absgx
    kbdSmoother:set(gs)
  end

  -- centering by lifting key:
  if v == 0 then
    local lowSpeedCoef = 1
    if autocenterEnabled then
      local wheelSpeed = electrics.values["wheelspeed"]
      velVec:set(obj:getSmoothRefVelocityXYZ())
      local spd = max(abs(wheelSpeed), velVec:length()) -- ensure the fallback case is a high speed (using max instead of the usual min), so that autocentering happens when in doubt (airplane carriers, tanks with reported wheelspeed of zero, etc)
      lowSpeedCoef = clamp(autocenterM * spd - autocenterN, 0, 1)
    end
    return lowSpeedCoef * understeerInRateMult * kbdInRateMult
  end

  local g = abs(obj:getGravity())
  --reduce steering speed only when steered into turn and pressing key into direction of turn (help limit the understeer)
  if signx == -signgx and signv == -signgx then
    kbdSmoother:set(0)
    local gLateral = min(absgx, g) / (g + 1e-30)
    return understeerOutRateMult * (kbdOutRateMult - (kbdOutRateMult * kbdUndersteerMult * gLateral))
  end

  --increase steering speed when pressing key out of direction of turn (help save the car from oversteer)
  if signv == signgx then
    local gLateralSmooth = min(gs, g) / (g + 1e-30)
    return understeerInRateMult * (kbdOutRateMult + kbdOutRateMult * kbdOversteerMult * gLateralSmooth)
  end
  return understeerOutRateMult * kbdOutRateMult
end

local function dynamicInputRateKbd2(v, curx)
  local signv = sign(v)
  local signx = sign(curx)
  local gx = sensors.gx
  local signgx = sign(gx)
  local mov = v - curx
  local signmov = sign(mov)

  -- centering by lifting key:
  if v == 0 then
    return understeerInRateMult * kbdInRateMult
  end

  -- centering by pressing opposite key:
  if signmov ~= signx then
    return understeerInRateMult * kbdInRateMult * 1.5
  end

  -- recovering from oversteer:
  if signv == signgx or signmov == signgx or signx == signgx then
    return understeerInRateMult * kbdInRateMult * 1.8
  end

  -- not enough data, fallback case
  local speed = electrics.values["wheelspeed"]
  if speed == nil then
    return understeerInRateMult * kbdInRateMult
  end

  -- regular steering:
  speed = abs(speed)
  local g = abs(obj:getGravity())
  return understeerOutRateMult * kbdOutRateMult * (1.4 - min(speed / 12, 1) * min(gxSmoothMax, g) / (g + 1e-30)) / 1.4
end

local function dynamicInputRatePad(v, dt, curx)
  local ps = padSmoother:getWithRateUncapped(0, dt, 0.2)
  local diff = v - curx
  local absdiff = abs(diff) * 0.9
  if absdiff > ps then
    ps = absdiff
    padSmoother:set(ps)
  end

  local baserate = (min(absdiff * 1.7, 3) + ps + 0.35)
  if diff * sign(curx) < 0 then
    return understeerInRateMult * min(baserate * 2, 5) * rateMult
  else
    return understeerOutRateMult * baserate * rateMult
  end
end

-- return vehicle mass at spawn time (will not change e.g. after losing a bumper)
local vehicleMassCache
local function vehicleMass()
  if not vehicleMassCache then
    vehicleMassCache = 0
    for _, n in pairs(v.data.nodes or {}) do
      vehicleMassCache = vehicleMassCache + n.nodeWeight
    end
  end
  return vehicleMassCache
end

local function getTotalDownforceFactor()
  local downforce = 0
  for _, wd in pairs(wheels.wheels or {}) do
    downforce = downforce + wd.downForce
  end
  return downforce / vehicleMass()
end

-- return what we estimate is the tightest possible vehicle turn radius, given the grip, downforce and speed of the vehicle
-- e.g. at 500 kmh, the best turn radius is likely huge (hundreds or thousands of meters), even if we're using slick tires.
-- while at 1 kmh, the best turn radius is tiny (maybe 0-5 meters), usually only achievable by installing a drift angle-kit (or anything that can make the front wheels turn much more than a typical street car)
local function getBestTurnRadius(vel)
  local accel = obj:getStaticFrictionCoef() * getTotalDownforceFactor()
  local radius = square(vel) / accel
  return clamp(radius, 0, 100000)
end

-- return surface materials where we allow understeer assistant to kick in
-- normally we allow this assistant on tarmac-like surfaces, and gravel-like surfaces benefit from understeer (so understeer should be allowed, in order to reach maximum grip)
local understeerAssistedSurfacesCache = nil
local function getUndersteerAssistedSurfacesById()
  if not understeerAssistedSurfacesCache then
    understeerAssistedSurfacesCache = {}
    for k, v in pairs(particles.getMaterialsParticlesTable()) do
      understeerAssistedSurfacesCache[k] = understeerAssistedSurfaces[v.name]
    end
  end
  return understeerAssistedSurfacesCache
end

-- 1 if all front wheels are on rigid surface, 0 if all front wheels are in loose surface, 0.5 if it's half and half, etc.
-- wheels with greater downforce have a greater contribution to the final value. wheels with no downforce have no contribution
local function getRatioFrontWheelsOnSolidSurface()
  local assistedSurfaces = getUndersteerAssistedSurfacesById()
  local assistedDownForce = 0
  local totalDownForce = 0
  for _, v in ipairs(frontWheels) do
    local wd = v[2]
    local mat, mat2 = wd.contactMaterialID1, wd.contactMaterialID2
    if mat == 4 then
      mat, mat2 = mat2, mat
    end
    totalDownForce = totalDownForce + wd.downForceRaw
    if assistedSurfaces[mat] then
      assistedDownForce = assistedDownForce + wd.downForceRaw
    end
  end
  return assistedDownForce / (totalDownForce + 1e-10)
end

-- compute the requested turn radius (what the user is asking by turning the steering wheel)
-- e.g. when in full lock, the requested turn radius is often 5-10 meters. vehicle speed does not matter in this calculation
local function getRequestedTurnRadius()
  local turnRadiusTotalWeighted = 0
  local totalDownForce = 0
  for _, v in ipairs(frontWheels) do
    local frontWheel = v[1]
    local wd = v[2]
    local rearWheels = v[3]
    local nRearWheels = v[4]

    -- there can be one or multiple rear wheels per front wheel (e.g. pigeon, duallies, 3-axle trucks, etc)
    -- compute the average angle of front angle against all possible rear wheels
    local wheelTurnRadiusTotal = 0
    for _, rearWheel in ipairs(rearWheels) do
      wheelTurnRadiusTotal = wheelTurnRadiusTotal + obj:wheelTurnRadius(frontWheel, rearWheel)
    end
    local wheelTurnRadiusAvg = wheelTurnRadiusTotal / (nRearWheels + 1e-10)

    -- assign importance proportional to the downforce
    -- e.g. in a extreme case, no downforce would mean that this front wheel is not contributing to the turning radius
    turnRadiusTotalWeighted = turnRadiusTotalWeighted + wd.downForceRaw * wheelTurnRadiusAvg
    totalDownForce = totalDownForce + wd.downForceRaw
  end
  return clamp(turnRadiusTotalWeighted / (totalDownForce + 1e-10), 0, 100000)
end

-- smart smoothing rate, to be passed onto a smoother. allows to pick how much to smooth based on how far away the smoother is from the desired value
-- a2/b2/c2 can be provided for assimetric decreases, otherwise a1/b1/c1 is used for both increases and decreases
local function getRate(currentValue, desiredValue, a1, b1, c1, a2, b2, c2)
  local diff = currentValue - desiredValue
  local diffabs = abs(diff)
  if a2 and diff > 0 then
    return a2 + b2 * diffabs + square(c2 * diffabs)
  else
    return a1 + b1 * diffabs + square(c1 * diffabs)
  end
end

-- compute which turn radius we want to aim for. this depends on user settings, driven surface, grip, empirical correction, etc
local function getTargetTurnRadius(vel, bestTurnRadius)
  local factorLowSpeed = clamp(0.072 * vel, 0, 1) -- fade out assistant at low speed
  local factorSurface = getRatioFrontWheelsOnSolidSurface() -- on loose surfaces, digging into the ground with massive understeer will increase grip
  local factorSetting = understeerReductionMultiplier -- user can choose 0% to 100% assistant strength
  return bestTurnRadius * factorLowSpeed * factorSurface * factorSetting
end

local function updateUIAppDebugging(st, velLen, requestedSteering, radiusRequested, radiusTarget, conversion, radiusRatio, mult, multSm, oversteerMult, overSm, assistance, understeerInRateMult, understeerOutRateMult)
  local currG = abs(sensors.gx2 / obj:getGravity())
  smExtra2 = smExtra2 or newTemporalSmoothing()
  local currGsm = smExtra2:getWithRateUncapped(currG, dt, getRate(smExtra2:value(), currG, 0, 1.5, 5.0))
  local gmax = 1.3
  local gfrac = 5
  guihooks.graph(
    false,
    --guihooks.graphWithCSV("understeer.csv"
    --,{"currG", currG, gmax, "G"}
    --,{"currGsm", currGsm, gmax, "G"}
    --,{"currGPlot", ((currG*gfrac)%1)/gfrac, 1/gfrac, "Gfake"}
    --,{"currGsmPlot", ((currGsm*gfrac)%1)/gfrac, 1/gfrac, "Gfake"}
    --,{"assistedSteering", abs(st), 1, "x"}
    --,{"static", obj:getStaticFrictionCoef(), 2, "x"}
    --,{"downforce", getTotalDownforceFactor(), 12, "x"}
    --,{"velLen", velLen, 100, "x"}
    {"requestedSteering", abs(requestedSteering), 1, "x"},
    --,{"radiusRequested", radiusRequested, 200, "m"}
    --,{"radiusTarget", radiusTarget, 200, "m"}
    --,{"conversion", conversion, 100, "x"}
    --,{"radiusRatio", radiusRatio, 5, "x"}
    --,{"mult", mult, 1.5, "x"}
    --,{"multSm", multSm, 1.5, "x"}
    --,{"oversteerMult", oversteerMult, 2, "x"}
    --,{"overSm", overSm, 1, "x"}
    {"assistance", assistance, 1.5, "x"},
    {"understeerInRateMult", understeerInRateMult, 5, "x"},
    {"understeerOutRateMult", understeerOutRateMult, 1, "x"}
  )
end

local rightVec, frontVec = vec3(), vec3()
local function inputStabilization(st, dt, filter)
  electrics.values.steeringUnassisted = st
  understeerInRateMult = 1
  understeerOutRateMult = 1
  local direct = filter == FILTER_DIRECT
  local useStabilization = stabilizationEnabled and (stabilizationEnabledDirect or not direct)
  local useUndersteerReduction = understeerReductionEnabled and (understeerReductionEnabledDirect or not direct)
  local useSlowdown = slowdownEnabled and (slowdownEnabledDirect or not direct)
  local useLimit = limitEnabled and (limitEnabledDirect or not direct)
  if not (useLimit or useSlowdown or useStabilization or useUndersteerReduction) then
    return st
  end

  -- oversteer detection
  local wheelSpeed = electrics.values["wheelspeed"]
  velVec:set(obj:getSmoothRefVelocityXYZ())
  rightVec:set(obj:getDirectionVectorRightXYZ())
  local velSqLen = velVec:squaredLength()
  local lowSpeedCoef = min(abs(wheelSpeed), velSqLen, 10) * 0.1
  local velLen = sqrt(velSqLen)
  local speedThreshold = 13
  local oversteer = lowSpeedCoef * min(velLen / speedThreshold, 1) * velVec:dot(rightVec) / (velLen + 1e-10)

  -- slower steering at high speed
  if useSlowdown then
    local speedMult = min(1, max(slowdownMultiplier, slowdownM * min(velLen, wheelSpeed) + slowdownB))
    local slowdownMult = clamp(5 * abs(oversteer), 0, 1) -- don't apply while oversteering
    understeerOutRateMult = max(slowdownMult, speedMult) -- slowdown the turn-in at high speed
    understeerInRateMult = understeerOutRateMult -- slowdown the turn-out at high speed
  end

  -- limit steering at high speed
  if useLimit then
    local speedMult = min(1, max(limitMultiplier, limitM * min(velLen, wheelSpeed) + limitB))
    local limitMult = clamp(5 * abs(oversteer), 0, 1) -- don't apply while oversteering
    st = st * max(limitMult, speedMult) -- limit steering range
  end

  -- oversteer reduction
  if useStabilization then
    local oversteerMult = lowSpeedCoef * min(velLen / (stabilizationEndSpeed + 1e-10), 1) * velVec:dot(rightVec) / (velLen + 1e-10)
    local countersteer = oversteerMult * stabilizationMultiplier
    st = st + sign(countersteer) * max(0, 1 - square(st)) * min(1, abs(countersteer))
  end

  -- understeer reduction -- if user is requesting too much steering, compared to what we believe the car can do, we reduce steering
  -- This assistant does not suit all vehicles equally:
  --   - some will get max lateral G-forces, some a bit less
  --   - some cars will get a bit of understeer if the user attempts a full lock (user can then pull back a tiny bit for optimum G-forces)
  --   - in a few rare cases (such as burnside_drag), it'll fall too short from understeering conditions, leaving some grip unreachable
  if useUndersteerReduction then
    -- compute the steering requested by user, vs the maximum steering we could aim for. expressed in terms of car turning radius (rather than steering wheel angle)
    local radiusTarget = getTargetTurnRadius(velLen, getBestTurnRadius(velLen))
    local radiusRequested = getRequestedTurnRadius()
    local radiusRatio = (radiusRequested == 0) and 0 or (radiusTarget / radiusRequested)

    -- convert from radius units, into a multiplier we can apply to the steering input value
    local a, b, c = 2600, -13, -3
    local conversion = clamp(c + a / (max(-b, 3.6 * velLen) + b), 5, 40) * 0.01 -- equation from hundreds of tests (car/speed/surface combinations)
    local steeringFactor = radiusRatio * conversion
    --local steeringFactor = radiusRatio * 0.3 -- this simplistic equation would be okay at 100kmh, but useless at 50kmh or 150kmh. hence the equation above^

    -- compute how much we'll correct the steering (smoothed, to avoid unrealistically sudden steering corrections)
    local mult = smUndersteer1:getWithRateUncapped(steeringFactor, dt, getRate(smUndersteer1:value(), steeringFactor, 1.5, 1.5, 0.75))
    local multSm = clamp(smUndersteer2:getWithRateUncapped(mult, dt, getRate(smUndersteer2:value(), mult, 0, 0.25, 2.5, 0.15, 0.5, 5)), 0, 1)

    -- determine if we're oversteering and therefore we should not be providing any understeer assistance
    frontVec:set(obj:getDirectionVectorXYZ())
    local margin = sign(st) == sign(oversteer) and -1 or 0.15
    local oversteerMult = 1 - clamp(5 * (abs(lowSpeedCoef * min(velLen / speedThreshold, 1) * sqrt(1 - max(0, velVec:dot(frontVec) / (velLen + 1e-10)))) - margin), 0, 1) -- zero while driving straight ahead (with a deadzone of 'margin' around 'straight ahead'); and 1 when sliding, spinning out, or driving in reverse
    local overSm = smOversteer:getWithRateUncapped(oversteerMult, dt, 3.0) -- avoid sudden inputs if e.g. spinning out fast

    -- apply assistant
    local assistance = multSm * overSm
    --local requestedSteering = st
    st = st * (1 - assistance)
    understeerInRateMult = understeerInRateMult * (1 + 3.5 * assistance) -- speedup the return to centered position (e.g. if user is full-lock but gets assisted into just 10deg of steering input, 10deg should return really fast, but the real 'full-lock' value takes longer to return - unless we speed it up to mimick the speed of a 10deg return)

  --updateUIAppDebugging(st, velLen, requestedSteering, radiusRequested, radiusTarget, conversion, radiusRatio, mult, multSm, oversteerMult, overSm, assistance, understeerInRateMult, understeerOutRateMult)
  end

  return st
end

local lockTypeWarned
local function updateGFX(dt)
  gxSmoothMax = gx_Smoother:getUncapped(0, dt)
  local absgx = abs(sensors.gx)
  if absgx > gxSmoothMax then
    gx_Smoother:set(absgx)
    gxSmoothMax = absgx
  end

  -- map the values
  for k, e in pairs(M.state) do
    local ival = e.val or 0
    local filter = e.filter
    local angle = e.angle or 0
    if angle > 0 and k == "steering" then
      filter = FILTER_DIRECT
    end -- enforce direct filter if user has chosen an angle for steering binding

    if filter == FILTER_DIRECT then
      if k == "steering" then
        -- use angle-matching for steering inputs
        local lockType = (angle <= 0) and 0 or e.lockType
        local vehicleAngle = vehicleSteeringWheelLock * 2 -- convert from jbeam scale (half range) to input scale (full range)
        local relation = angle / vehicleAngle
        -- 1:1 matching angle behaviour (in-game versus real life steering wheel angle):
        if (lockType == 0) or (lockType == 3 and relation < 1) then
          -- don't match
        elseif (lockType == 1) or (lockType == 3 and relation >= 1) then
          -- simple (may not reach full lock in some vehicles)
          ival = clamp(ival * relation, -1, 1)
        elseif lockType == 2 then
          -- progressive (move faster after half-lock to guarantee full lock)
          ival = ival * relation + sign(ival) * square(2 * max(0.5, abs(ival)) - 1) * max(0, 1 - relation) -- ival = linear + nonlinear
        elseif not lockTypeWarned then
          ival = 0
          lockTypeWarned = true
          log("E", "", "Unsupported steering lock type: " .. dumps(lockType))
        end
        ival = inputStabilization(ival, dt, filter)
      end
    else
      ival = min(max(ival, -1), 1)
      if filter == FILTER_PAD then -- joystick / game controller - smoothing without autocentering
        if k == "steering" then
          local prevVal = e.smootherPAD:value()
          local rate = dynamicInputRatePad(ival, dt, prevVal)
          local filteredVal = e.smootherPAD:getWithRateCapped(ival, dt, rate)
          ival = inputStabilization(filteredVal, dt, filter)
        else
          ival = e.smootherPAD:getCapped(ival, dt)
        end
      elseif filter == FILTER_KBD then
        if k == "steering" then
          local prevVal = e.smootherKBD:value()
          local rate = dynamicInputRateKbd(ival, dt, prevVal)
          local filteredVal = e.smootherKBD:getWithRateCapped(ival, dt, rate)
          ival = inputStabilization(filteredVal, dt, filter)
        else
          ival = e.smootherKBD:getCapped(ival, dt)
        end
      elseif filter == FILTER_KBD2 then
        if k == "steering" then
          local prevVal = e.smootherKBD:value()
          local rate = dynamicInputRateKbd2(ival, prevVal)
          local filteredVal = e.smootherKBD:getWithRateCapped(ival, dt, rate)
          ival = inputStabilization(filteredVal, dt, filter)
        else
          ival = e.smootherKBD:getCapped(ival, dt)
        end
      elseif filter == "FILTER_AI" then
        if k == "steering" then
          ival = e.smootherPAD:getWithRateCapped(ival, dt, 4 * rateMult)
          electrics.values.steeringUnassisted = ival
        else
          ival = e.val or 0
        end
      end
    end

    if k == "steering" then
      if playerInfo.anyPlayerSeated and not ai.isDriving() then
        if filter ~= M.lastFilterType then
          obj:queueGameEngineLua(string.format('extensions.hook("startTracking", {Name = "ControlsUsed", Method = "%s"})', FILTER_NAME[filter]))
          M.lastFilterType = filter
        end
      end
    end

    ival = clamp(ival, e.minLimit, e.maxLimit)

    if k == "parkingbrake" then
      local prev = M[k] or e.minLimit
      if handbrakeSoundEngaging and prev == e.minLimit and ival > prev then
        sounds.playSoundSkipAI(handbrakeSoundEngaging)
      end
      if handbrakeSoundDisengaging and prev == e.maxLimit and ival < prev then
        sounds.playSoundSkipAI(handbrakeSoundDisengaging)
      end
      if handbrakeSoundDisengaged and ival == e.minLimit and ival < prev then
        sounds.playSoundSkipAI(handbrakeSoundDisengaged)
      end
    end

    M[k] = ival

    inputNameCache[k] = inputNameCache[k] or k .. "_input"
    electrics.values[inputNameCache[k]] = ival
    if e.osClockHP then
      inputTSCache[k] = inputTSCache[k] or k .. "_timestamp"
      electrics.values[inputTSCache[k]] = e.osClockHP
    end
  end
end

local function reset()
  gxSmoothMax = 0
  gx_Smoother:reset()
  M.lastInputs = {}

  for k, e in pairs(M.state) do
    e.smootherKBD:reset()
    e.smootherPAD:reset()
  end
  M:settingsChanged()
end

local function getDefaultState(itype)
  return {
    val = 0,
    filter = 0,
    smootherKBD = newTemporalSmoothing(10, 10, nil, 0),
    smootherPAD = newTemporalSmoothing(10, 10, nil, 0),
    minLimit = -1,
    maxLimit = 1
  }
end

local function event(itype, ivalue, filter, angle, lockType, osClockHP, source)
  if M.state[itype] == nil then -- probably a vehicle-specific input
    log("W", "", "The vehicle-specific input event " .. dumps(itype) .. " was not defined, so gamepad smoothing, keyboard smoothing, and safe range of values is unknown. The vehicle creator should define this input event type, for example executing lua code such as 'input.state[" .. dumps(itype) .. "] = { minLimit=xxx, maxLimit=xxx, smootherKBD=..., smootherPAD=... }' during vehicle initialization (please search input.lua for more context). As safety fallback, a default definition will be used, which may or may not be suitable")
    M.state[itype] = getDefaultState(itype)
  end

  source = source or "local"

  M.lastInputs[source] = M.lastInputs[source] or {}
  M.lastInputs[source][itype] = ivalue

  if not M.allowedInputSources[itype] or M.allowedInputSources[itype][source] then
    M.state[itype].val = ivalue
    M.state[itype].filter = filter
    M.state[itype].angle = angle
    M.state[itype].lockType = lockType
    M.state[itype].osClockHP = osClockHP
    M.state[itype].source = source
  end
end

local function toggleEvent(itype)
  if M.state[itype] == nil then
    return
  end
  if M.state[itype].val > 0.5 then
    M.state[itype].val = 0
  else
    M.state[itype].val = 1
  end
  M.state[itype].filter = 0
end

-- keyboard (multi-key) compatibility
local kbdSteerLeft = 0
local kbdSteerRight = 0
local function kbdSteer(isRight, val, filter)
  if isRight then
    kbdSteerRight = val
  else
    kbdSteerLeft = val
  end
  event("steering", kbdSteerRight - kbdSteerLeft, filter)
end

-- gamepad( (mono-axis) compatibility
local function padAccelerateBrake(val, filter)
  if val > 0 then
    event("throttle", val, filter)
    event("brake", 0, filter)
  else
    event("throttle", 0, filter)
    event("brake", -val, filter)
  end
end

local function settingsChanged()
  -- countersteer reduction assistant
  stabilizationEnabled = settings.getValue("steeringStabilizationEnabled", false)
  stabilizationEnabledDirect = settings.getValue("steeringStabilizationEnabledDirect", false)
  stabilizationEndSpeed = clamp(settings.getValue("steeringStabilizationEndSpeed", 0), 0, 30) -- 0..30 m/s
  stabilizationMultiplier = clamp(settings.getValue("steeringStabilizationMultiplier", 0), 0, 2) -- 0..2 multiplier (up to 200%)

  -- understeer reduction assistant
  understeerReductionEnabled = settings.getValue("steeringUndersteerReductionEnabled", false)
  understeerReductionEnabledDirect = settings.getValue("steeringUndersteerReductionEnabledDirect", false)
  understeerReductionMultiplier = clamp(settings.getValue("steeringUndersteerReductionMultiplier", 0), 0, 1) -- 0..1 multiplier (up to 100%)

  -- slower steering at high speed
  slowdownEnabled = settings.getValue("steeringSlowdownEnabled", false)
  slowdownEnabledDirect = settings.getValue("steeringSlowdownEnabledDirect", false)
  local slowdownStartSpeed = clamp(settings.getValue("steeringSlowdownStartSpeed", 0), 0, 100) -- 0..100 m/s
  local slowdownEndSpeed = clamp(settings.getValue("steeringSlowdownEndSpeed", 0), 0, 100) -- 0..100 m/s
  slowdownMultiplier = clamp(settings.getValue("steeringSlowdownMultiplier", 0), 0, 1) -- 0..1 multiplier
  if slowdownEnabled and slowdownStartSpeed > slowdownEndSpeed then
    log("W", "", "Invalid configuration for slower steering at high speed. Sanitizing by swapping: [" .. dumps(slowdownStartSpeed) .. ".." .. dumps(slowdownEndSpeed) .. "]")
    slowdownStartSpeed, slowdownEndSpeed = slowdownEndSpeed, slowdownStartSpeed
  end
  slowdownM = (slowdownMultiplier - 1) / (1e-30 + slowdownEndSpeed - slowdownStartSpeed)
  slowdownB = 1 - slowdownM * slowdownStartSpeed

  -- limit steering at high speed
  limitEnabled = settings.getValue("steeringLimitEnabled", false)
  limitEnabledDirect = settings.getValue("steeringLimitEnabledDirect", false)
  local limitStartSpeed = clamp(settings.getValue("steeringLimitStartSpeed", 0), 0, 100) -- 0..100 m/s
  local limitEndSpeed = clamp(settings.getValue("steeringLimitEndSpeed", 0), 0, 100) -- 0..100 m/s
  limitMultiplier = clamp(settings.getValue("steeringLimitMultiplier", 0), 0, 1) -- 0..1 multiplier
  if limitEnabled and limitStartSpeed > limitEndSpeed then
    log("W", "", "Invalid configuration for limit steering at high speed. Sanitizing by swapping: [" .. dumps(limitStartSpeed) .. ".." .. dumps(limitEndSpeed) .. "]")
    limitStartSpeed, limitEndSpeed = limitEndSpeed, limitStartSpeed
  end
  limitM = (limitMultiplier - 1) / (1e-30 + limitEndSpeed - limitStartSpeed)
  limitB = 1 - limitM * limitStartSpeed

  -- slower autocenter at low speed
  autocenterEnabled = settings.getValue("steeringAutocenterEnabled", false)
  local autocenterStartSpeed = 0.1
  local autocenterEndSpeed = 1.0
  autocenterM = 1 / (1e-30 + autocenterEndSpeed - autocenterStartSpeed)
  autocenterN = autocenterStartSpeed * autocenterM
end

local function setAllowedInputSource(itype, source, enabled)
  M.allowedInputSources = M.allowedInputSources or {}
  if source == nil then
    M.allowedInputSources[itype] = nil
    return
  end

  M.allowedInputSources[itype] = M.allowedInputSources[itype] or {}
  M.allowedInputSources[itype][source] = enabled
end

-- public interface
M.updateGFX = updateGFX
M.init = init
M.initSecondStage = initSecondStage
M.reset = reset
M.event = event
M.toggleEvent = toggleEvent
M.kbdSteer = kbdSteer
M.padAccelerateBrake = padAccelerateBrake
M.settingsChanged = settingsChanged
M.setAllowedInputSource = setAllowedInputSource

return M
