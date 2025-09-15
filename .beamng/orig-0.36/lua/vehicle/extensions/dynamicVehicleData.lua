-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local abs = math.abs
local max = math.max
local floor = math.floor
local ceil = math.ceil

local timer = 0

local rpmRoundValue = 50
local torqueRoundValue = 5
local weightRoundValue = 5

local fiveKmh = 1.38889
local tenKmh = 2.7777777778
local hundredKmh = 27.77777778
local hundredTenKmh = 30.5556
local twoHundredKmh = 55.55555556
local threeHundredKmh = 83.3333333
local sixtyMph = 26.8224
local hundredMph = 44.704
local twoHundredMph = 89.408

local model_key = nil
local config_key = nil
local workerCoroutine = nil

local logTag = "dynamicVehicleData"

local function wait(seconds)
  local start = timer
  while timer <= start + seconds do
    coroutine.yield()
  end
end

local function compareData(oldData, newData, model_key, config_key)
  local threshold = 0.1

  for k, v in pairs(newData) do
    --print(k .. ": " .. v)
    if oldData then
      if type(oldData[k]) == "number" and type(v) == "number" then
        local relativeDifference = math.abs(1 - (v / oldData[k]))
        if relativeDifference > threshold then
          log("W", logTag, string.format("Old and new '%s' differ by %.2f%% for vehicle: '%s->%s'. Old/New: %f/%f", k, relativeDifference * 100, model_key, config_key, oldData[k], v))
        end
      end
    end
  end
end

local function clearData(data, whiteList)
  if data == nil then
    return {}
  end
  --print("data pre clearing:")
  --dump(data)
  for _, v in pairs(whiteList) do
    data[v] = nil
  end
  --print("data post clearing:")
  --dump(data)

  return data
end

-- saves changes to the info json file
local function saveInfo(newData, whiteList)
  log("I", logTag, (string.format("Got data (%s/%s):", model_key, config_key)) .. " = " .. dumps(newData))
  local filepath = "vehicles/" .. model_key .. "/info_" .. config_key .. ".json"
  local data = jsonReadFile(filepath)
  data = clearData(data, whiteList)
  compareData(data, newData, model_key, config_key)
  if data and newData then
    log("D", logTag, "Saving...")
    tableMergeRecursive(data, newData)
    jsonWriteFile(filepath, data, true)
  end
end

local function onInit()
end

-- switches to next vehicle
local function killswitch()
  --print(" === killswitch ===")
  obj:queueGameEngineLua("util_saveDynamicData.vehicleDone()")
end

local function watchdogHeartbeat()
  obj:queueGameEngineLua("util_saveDynamicData.heartbeat()")
end

local function resetVehicle(position)
  obj:queueGameEngineLua("be:resetVehicle(0)")
  wait(2)
  obj:queueGameEngineLua("getPlayerVehicle(0):setPositionRotation(" .. position .. ")")
  wait(2)
  obj:queueGameEngineLua("getPlayerVehicle(0):autoplace(false)")
  wait(2)
end

local function getVehiclePerformanceData()
  local stats = obj:calcBeamStats()
  local weight = ceil(stats.total_weight / weightRoundValue) * weightRoundValue

  local engines = powertrain.getDevicesByCategory("engine")
  if not engines or #engines <= 0 then
    log("I", logTag, "Can't find any engine, not getting static performance data")
    return {Weight = weight, whiteList = {"Weight"}}
  end

  local maxRPM = 0
  local maxTorque = -1
  local maxPower = -1
  local maxTorqueRPM = 0
  local maxPowerRPM = 0
  local curves
  if #engines > 1 then
    local torqueData = {}
    for _, v in pairs(engines) do
      local tData = v:getTorqueData()
      maxRPM = max(maxRPM, tData.maxRPM)
      table.insert(torqueData, tData)
    end

    local torqueCurve = {}
    local powerCurve = {}
    for _, td in ipairs(torqueData) do
      local engineCurves = td.curves[td.finalCurveName]
      for rpm, torque in pairs(engineCurves.torque) do
        torqueCurve[rpm] = (torqueCurve[rpm] or 0) + torque
      end
      for rpm, power in pairs(engineCurves.power) do
        powerCurve[rpm] = (powerCurve[rpm] or 0) + power
      end
    end
    for rpm, torque in pairs(torqueCurve) do
      if torque > maxTorque then
        maxTorque = torque
        maxTorqueRPM = rpm
      end
    end
    for rpm, power in pairs(powerCurve) do
      if power > maxPower then
        maxPower = power
        maxPowerRPM = rpm
      end
    end
    curves = {torque = torqueCurve, power = powerCurve}
  else
    local torqueData = engines[1]:getTorqueData()
    maxRPM = torqueData.maxRPM
    maxTorque = torqueData.maxTorque
    maxPower = torqueData.maxPower
    maxTorqueRPM = torqueData.maxTorqueRPM
    maxPowerRPM = torqueData.maxPowerRPM
    curves = torqueData.curves[torqueData.finalCurveName]
  end

  local minRPMTorque = -1
  local maxRPMTorque = -1
  local minRPMPower = -1
  local maxRPMPower = -1

  if curves then
    for i = maxTorqueRPM, 0, -1 do
      local torque = curves.torque[i] or 0
      local relDifference = abs(torque - maxTorque) / maxTorque
      if relDifference > 0.02 then
        minRPMTorque = i
        break
      end
    end
    for i = maxTorqueRPM, maxRPM, 1 do
      local torque = curves.torque[i] or 0
      local relDifference = abs(torque - maxTorque) / maxTorque
      if relDifference > 0.02 then
        maxRPMTorque = i
        break
      end
    end

    for i = maxPowerRPM, 0, -1 do
      local power = curves.power[i] or 0
      local relDifference = abs(power - maxPower) / maxPower
      if relDifference > 0.02 then
        minRPMPower = i
        break
      end
    end
    for i = maxPowerRPM, maxRPM, 1 do
      local power = curves.power[i] or 0
      local relDifference = abs(power - maxPower) / maxPower
      if relDifference > 0.02 or i == maxRPM then
        maxRPMPower = i
        break
      end
    end
  else
    print("Can't get torque curve for peak torque/power RPMs")
  end

  -- clean up the data
  local PowerPeakRPM = nil
  local TorquePeakRPM = nil
  local weightPower = nil
  if maxPower > 0 then
    maxPower = floor(maxPower)

    local powerMinRPM = ceil(minRPMPower / rpmRoundValue) * rpmRoundValue
    local powerMaxRPM = floor(maxRPMPower / rpmRoundValue) * rpmRoundValue
    local maxPowerRange = maxRPMPower - minRPMPower
    if maxPowerRange >= 500 then
      PowerPeakRPM = powerMinRPM .. " - " .. powerMaxRPM
    else
      PowerPeakRPM = powerMinRPM
    end
    weightPower = weight / maxPower
  else
    print("Max power <= 0...")
  end

  if maxTorque > 0 then
    maxTorque = ceil(maxTorque / torqueRoundValue) * torqueRoundValue

    local torqueMinRPM = ceil(minRPMTorque / rpmRoundValue) * rpmRoundValue
    local torqueMaxRPM = ceil(maxRPMTorque / rpmRoundValue) * rpmRoundValue
    local maxTorqueRange = maxRPMTorque - minRPMTorque
    if maxTorqueRange >= 500 then
      TorquePeakRPM = torqueMinRPM .. " - " .. torqueMaxRPM
    else
      TorquePeakRPM = torqueMinRPM
    end
  else
    print("Max torque <= 0...")
  end

  local perfData = {
    Weight = weight,
    PowerPeakRPM = PowerPeakRPM,
    TorquePeakRPM = TorquePeakRPM,
    Torque = maxTorque > 0 and maxTorque or nil,
    Power = maxPower > 0 and maxPower or nil,
    ["Weight/Power"] = weightPower
  }

  local whiteList = {"Weight", "PowerPeakRPM", "TorquePeakRPM", "Torque", "Power", "Weight/Power"}

  return {data = perfData, whiteList = whiteList}
end

local function writeBasicPerformanceData()
  local perfData = getVehiclePerformanceData()
  --log('E', logTag, dumps(perfData))
  saveInfo(perfData.data, perfData.whiteList)
end

local function getMotorType()
  local hasICE = false
  local hasElectricMotor = false
  local hasOther = false
  local motors = powertrain.getDevicesByCategory("engine")
  for _, v in pairs(motors) do
    if v.type == "combustionEngine" then
      hasICE = true
    elseif v.type == "electricMotor" or v.type == "seriesElectricMotor" or v.type == "electricMotorPMDC" then
      hasElectricMotor = true
    else
      hasOther = true
    end
  end

  local motorType
  if hasICE then
    if hasElectricMotor and not hasOther then
      motorType = "Hybrid"
    elseif hasOther then
      motorType = "Other"
    else
      motorType = "ICE"
    end
  elseif hasElectricMotor and not hasOther then
    motorType = "Electric"
  else
    motorType = "Other"
  end

  return motorType
end

local function getInductionType()
  local hasTurbo = false
  local hasSC = false
  local hasN2O = false
  local engines = powertrain.getDevicesByType("combustionEngine")
  for _, v in pairs(engines) do
    if v.turbocharger.isExisting then
      hasTurbo = true
    end
    if v.supercharger.isExisting then
      hasSC = true
    end
    if v.nitrousOxideInjection.isExisting then
      hasN2O = true
    end
  end

  local inductionType
  if hasTurbo and hasSC and hasN2O then
    inductionType = "Turbo + SC + N2O"
  elseif hasTurbo and hasSC then
    inductionType = "Turbo + SC"
  elseif hasTurbo and hasN2O then
    inductionType = "Turbo + N2O"
  elseif hasSC and hasN2O then
    inductionType = "SC + N2O"
  elseif hasTurbo then
    inductionType = "Turbo"
  elseif hasSC then
    inductionType = "SC"
  else
    inductionType = "NA"
  end

  return inductionType
end

local function getFuelType()
  local hasGasoline = false
  local hasDiesel = false
  local hasBattery = false
  local hasOther = false

  local energyStorages = energyStorage.getStorages()
  for _, v in pairs(energyStorages) do
    if v.type == "fuelTank" then
      if v.energyType == "gasoline" then
        hasGasoline = true
      elseif v.energyType == "diesel" then
        hasDiesel = true
      else
        hasOther = true
      end
    elseif v.type == "electricBattery" then
      hasBattery = true
    elseif v.type ~= "n2oTank" then
      hasOther = true
    end
  end

  local fuelType
  if hasGasoline or hasDiesel then
    if hasBattery and not hasOther then
      fuelType = "Hybrid"
    else
      fuelType = hasGasoline and "Gasoline" or "Diesel"
    end
  elseif hasBattery and not hasOther then
    fuelType = "Battery"
  else
    fuelType = "Other"
  end

  return fuelType
end

local function getTransmissionType()
  local hasManual = false
  local hasSequential = false
  local hasAuto = false
  local hasDCT = false
  local hasCVT = false
  local hasRangebox = false
  local hasOther = false

  local transmissions = powertrain.getDevicesByCategory("gearbox")
  for _, v in pairs(transmissions) do
    if v.type == "automaticGearbox" then
      hasAuto = true
    elseif v.type == "cvtGearbox" then
      hasCVT = true
    elseif v.type == "dctGearbox" then
      hasDCT = true
    elseif v.type == "manualGearbox" then
      hasManual = true
    elseif v.type == "rangeBox" then
      hasRangebox = true
    elseif v.type == "sequentialGearbox" then
      hasSequential = true
    else
      hasOther = true
    end
  end

  local transmissionType
  if hasOther then
    transmissionType = "Other"
  elseif hasAuto then
    if hasCVT or hasDCT or hasManual or hasSequential then
      transmissionType = "Other"
    else
      transmissionType = "Automatic"
    end
  elseif hasCVT then
    if hasAuto or hasDCT or hasManual or hasSequential then
      transmissionType = "Other"
    else
      transmissionType = "CVT"
    end
  elseif hasDCT then
    if hasAuto or hasCVT or hasManual or hasSequential then
      transmissionType = "Other"
    else
      transmissionType = "DCT"
    end
  elseif hasManual then
    if hasAuto or hasCVT or hasDCT or hasSequential then
      transmissionType = "Other"
    else
      transmissionType = "Manual"
    end
  elseif hasSequential then
    if hasAuto or hasCVT or hasDCT or hasManual then
      transmissionType = "Other"
    else
      transmissionType = "Sequential"
    end
  else
    transmissionType = "Other"
  end

  return transmissionType
end

local function writeBasicPowertrainData()
  local powertrainData = {}
  local whitelist = {}
  local motorType = getMotorType()
  powertrainData["Propulsion"] = motorType
  table.insert(whitelist, "Propulsion")
  if motorType == "ICE" or motorType == "Hybrid" then
    local inductionType = getInductionType()
    powertrainData["Induction Type"] = inductionType
    table.insert(whitelist, "Induction Type")
  end

  local fuelType = getFuelType()
  powertrainData["Fuel Type"] = fuelType
  table.insert(whitelist, "Fuel Type")

  local transmission = getTransmissionType()
  powertrainData.Transmission = transmission
  table.insert(whitelist, "Transmission")

  --log('E', logTag, dumps(perfData))
  saveInfo(powertrainData, whitelist)
end

local function getPowertrainLayout()
  local propulsedWheelsCount = 0
  local wheelCount = 0

  local adjustedWheels = deepcopy(wheels.wheels)

  local diffs = powertrain.getDevicesByCategory("differential")
  local actualDiffs = {} --diffs minus the duallies
  for _, diff in ipairs(diffs) do
    if diff.mode == "dually" then
      local childWheels1 = powertrain.getChildWheels(diff, 1)
      local childWheels2 = powertrain.getChildWheels(diff, 2)
      if #childWheels1 == 1 and #childWheels2 == 1 then
        adjustedWheels[childWheels1[1].wheelID] = nil
      end
    else
      table.insert(actualDiffs, diff)
    end
  end
  -- for _, wd in pairs(adjustedWheels) do
  --   print(wd.name)
  --   print(wd.rotatorType)
  -- end

  local avgWheelPos = vec3(0, 0, 0)
  for _, wd in pairs(adjustedWheels) do
    wheelCount = wheelCount + 1
    local wheelNodePos = v.data.nodes[wd.node1].pos --find the wheel position
    avgWheelPos = avgWheelPos + wheelNodePos --sum up all positions
    if wd.isPropulsed then
      propulsedWheelsCount = propulsedWheelsCount + 1
    end
  end

  avgWheelPos = avgWheelPos / wheelCount --make the average of all positions
  --print("Wheels: " .. tostring(wheelCount))
  --print("Propulsed: " .. tostring(propulsedWheelsCount))

  if wheelCount <= 4 and wheelCount > 1 then
    local vectorForward = vec3(v.data.nodes[v.data.refNodes[0].ref].pos) - vec3(v.data.nodes[v.data.refNodes[0].back].pos) --vector facing forward
    local vectorUp = vec3(v.data.nodes[v.data.refNodes[0].up].pos) - vec3(v.data.nodes[v.data.refNodes[0].ref].pos)
    local vectorRight = vectorForward:cross(vectorUp) --vector facing to the right

    local propulsedWheelLocations = {fr = 0, fl = 0, rr = 0, rl = 0}
    for _, wd in pairs(adjustedWheels) do
      if wd.isPropulsed then
        local wheelNodePos = vec3(v.data.nodes[wd.node1].pos) --find the wheel position
        local wheelVector = wheelNodePos - avgWheelPos --create a vector from our "center" to the wheel
        local dotForward = vectorForward:dot(wheelVector) --calculate dot product of said vector and forward vector
        local dotLeft = vectorRight:dot(wheelVector) --calculate dot product of said vector and left vector

        if dotForward >= 0 then
          if dotLeft >= 0 then
            propulsedWheelLocations.fr = propulsedWheelLocations.fr + 1
          else
            propulsedWheelLocations.fl = propulsedWheelLocations.fl + 1
          end
        else
          if dotLeft >= 0 then
            propulsedWheelLocations.rr = propulsedWheelLocations.rr + 1
          else
            propulsedWheelLocations.rl = propulsedWheelLocations.rl + 1
          end
        end
      end
    end

    local layout
    local diffCount = #actualDiffs
    if diffCount == 0 then
      if propulsedWheelLocations.fr > 0 or propulsedWheelLocations.fl > 0 then
        layout = "FWD"
      elseif propulsedWheelLocations.rr > 0 or propulsedWheelLocations.rl > 0 then
        layout = "RWD"
      else
        layout = string.format("%dx%d", wheelCount, propulsedWheelsCount)
      end
    elseif diffCount == 1 then
      if propulsedWheelLocations.fr > 0 and propulsedWheelLocations.fl > 0 then
        layout = "FWD"
      elseif propulsedWheelLocations.rr > 0 and propulsedWheelLocations.rl > 0 then
        layout = "RWD"
      else
        layout = string.format("%dx%d", wheelCount, propulsedWheelsCount)
      end
    elseif diffCount == 3 then
      local orderedDevices = powertrain.getOrderedDevices()
      local centerDiff
      for _, v in pairs(orderedDevices) do
        if centerDiff then
          break
        end
        for _, w in pairs(actualDiffs) do
          if w.name == v.name then
            centerDiff = w
            break
          end
        end
      end
      if not centerDiff then
        layout = string.format("%dx%d", wheelCount, propulsedWheelsCount)
      else
        if propulsedWheelLocations.fr > 0 and propulsedWheelLocations.fl > 0 and propulsedWheelLocations.rr > 0 and propulsedWheelLocations.rl > 0 then
          if centerDiff.mode == "locked" then
            layout = "4WD"
          else
            layout = "AWD"
          end
        else
          layout = string.format("%dx%d", wheelCount, propulsedWheelsCount)
        end
      end
    else
      layout = string.format("%dx%d", wheelCount, propulsedWheelsCount)
    end

    return layout
  elseif propulsedWheelsCount > 0 then
    return string.format("%dx%d", wheelCount, propulsedWheelsCount)
  end

  return "Other"
end

local function writePowertrainLayoutData()
  local powertrainLayoutData = {}
  local whitelist = {}

  local layout = getPowertrainLayout()
  powertrainLayoutData.Drivetrain = layout
  table.insert(whitelist, "Drivetrain")

  --log('E', logTag, dumps(perfData))
  saveInfo(powertrainLayoutData, whitelist)
end

local function constructAABB()
  local min = vec3(math.huge, math.huge, math.huge)
  local max = vec3(-math.huge, -math.huge, -math.huge)
  local nodes = v.data.nodes
  for i = 0, tableSizeC(nodes) - 1 do
    local node = nodes[i]
    local pos = node.pos
    if pos.x > max.x then
      max.x = pos.x
    end
    if pos.y > max.y then
      max.y = pos.y
    end
    if pos.z > max.z then
      max.z = pos.z
    end
    if pos.x < min.x then
      min.x = pos.x
    end
    if pos.y < min.y then
      min.y = pos.y
    end
    if pos.z < min.z then
      min.z = pos.z
    end
  end

  local refPos = vec3(nodes[v.data.refNodes[0].ref].pos)
  min = min - (refPos)
  max = max - (refPos)

  -- One corner point plus the neighboring points
  local points = {}
  table.insert(points, {min.x, min.y, min.z})
  table.insert(points, {max.x, max.y, max.z})

  -- table.insert(points, {min.x, min.y, min.z})
  -- table.insert(points, {min.x, min.y, max.z})
  -- table.insert(points, {min.x, max.y, max.z})
  -- table.insert(points, {max.x, max.y, max.z})

  -- table.insert(points, {max.x, max.y, min.z})
  -- table.insert(points, {max.x, min.y, min.z})
  -- table.insert(points, {max.x, min.y, max.z})
  -- table.insert(points, {min.x, max.y, min.z})

  return points
end

local function writeBBData()
  saveInfo({BoundingBox = constructAABB()}, {"BoundingBox"})
end

local function accelerationTests()
  if wheels.wheelCount <= 0 then
    return
  end

  resetVehicle("20,0,0.5,0,0,0,1")

  for _, diff in pairs(powertrain.getDevicesByType("differential")) do
    if diff.mode ~= "locked" then
      print("Setting diff '" .. diff.name .. "' to locked")
      diff:setMode("locked")
    end
  end

  for _, shaft in pairs(powertrain.getDevicesByType("shaft")) do
    if shaft.mode ~= "connected" then
      print("Setting shaft '" .. shaft.name .. "' to connected")
      shaft:setMode("connected")
    end
  end

  for _, rangebox in pairs(powertrain.getDevicesByType("rangeBox")) do
    if rangebox.mode ~= "high" then
      print("Setting rangebox '" .. rangebox.name .. "' to high")
      rangebox:setMode("high")
    end
  end

  extensions.load("perfectLaunch")
  perfectLaunch.onInit()
  perfectLaunch.prepare(vec3(20, -10, 0.5))

  wait(4)

  perfectLaunch.go()

  local time100kmh = nil
  local time200kmh = nil
  local time300kmh = nil
  local time100200kmh = nil
  local time60mph = nil
  local time100mph = nil
  local time200mph = nil
  local time60100mph = nil
  local maxSpeed = -1
  local maxSpeedTime = timer

  local speed = 0
  timer = 0
  repeat
    speed = electrics.values.airspeed
    coroutine.yield()

    if timer > 10 then
      print("Can't accelerate, aborting...")
      return
    end
  until speed > 0.15 --wait for the car to start moving before actually timing it

  timer = 0

  while timer <= 400 do
    speed = electrics.values.airspeed
    if not speed then
      -- no speed info, ship this
      print("Not getting any speed info, aborting...")
      return
    end

    if timer > 20 and speed <= fiveKmh then
      print("Can't accelerate, aborting...")
      return
    end

    if speed >= hundredKmh and not time100kmh then
      time100kmh = timer
      time100200kmh = timer
      print("0-100: " .. tostring(timer))
    end
    if speed >= twoHundredKmh and not time200kmh then
      time200kmh = timer
      time100200kmh = timer - time100200kmh
      print("0-200: " .. tostring(timer))
    end
    if speed >= threeHundredKmh and not time300kmh then
      time300kmh = timer
      print("0-300: " .. tostring(timer))
    end
    if speed >= sixtyMph and not time60mph then
      time60mph = timer
      time60100mph = timer
    end
    if speed >= hundredMph and not time100mph then
      time100mph = timer
      time60100mph = timer - time60100mph
    end
    if speed >= twoHundredMph and not time200mph then
      time200mph = timer
    end

    if speed - maxSpeed >= 0.1 then
      maxSpeed = speed
      maxSpeedTime = timer
    end
    -- reached no new max speed for at least 5 seconds?
    -- TODO: this needs some more improvements
    if perfectLaunch.launchFailed then
      print("launch failed...")
      break
    end

    if input.throttle < 0.95 then
      maxSpeedTime = timer
    end

    if timer - maxSpeedTime > 5 then
      print("reached max speed")
      break
    end
    coroutine.yield()
  end

  if timer >= 400 then
    print("high speed test timed out")
  end

  if not time200kmh then
    time100200kmh = nil
  end
  if not time100mph then
    time60100mph = nil
  end

  perfectLaunch.stop()

  local perfData = {
    ["Top Speed"] = maxSpeed,
    ["0-100 km/h"] = time100kmh and (floor(time100kmh * 10) / 10) or nil,
    ["0-200 km/h"] = time200kmh and (floor(time200kmh * 10) / 10) or nil,
    ["0-300 km/h"] = time300kmh and (floor(time300kmh * 10) / 10) or nil,
    ["100-200 km/h"] = time100200kmh and (floor(time100200kmh * 10) / 10) or nil,
    ["0-60 mph"] = time60mph and (floor(time60mph * 10) / 10) or nil,
    ["0-100 mph"] = time100mph and (floor(time100mph * 10) / 10) or nil,
    ["0-200 mph"] = time200mph and (floor(time200mph * 10) / 10) or nil,
    ["60-100 mph"] = time60100mph and (floor(time60100mph * 10) / 10) or nil
  }

  local whiteList = {"Top Speed", "0-100 km/h", "0-200 km/h", "0-300 km/h", "100-200 km/h", "0-60 mph", "0-100 mph", "0-200 mph", "60-100 mph"}

  saveInfo(perfData, whiteList)

  return maxSpeed >= 32
end

local function brakingTests()
  if wheels.wheelCount <= 0 then
    return
  end

  resetVehicle("20,0,0.5,0,0,0,1")

  for _, diff in pairs(powertrain.getDevicesByType("differential")) do
    if diff.mode ~= "locked" then
      print("Setting diff '" .. diff.name .. "' to locked")
      diff:setMode("locked")
    end
  end

  for _, shaft in pairs(powertrain.getDevicesByType("shaft")) do
    if shaft.mode ~= "connected" then
      print("Setting shaft '" .. shaft.name .. "' to connected")
      shaft:setMode("connected")
    end
  end

  for _, rangebox in pairs(powertrain.getDevicesByType("rangeBox")) do
    if rangebox.mode ~= "high" then
      print("Setting rangebox '" .. rangebox.name .. "' to high")
      rangebox:setMode("high")
    end
  end

  controller.mainController.setGearboxMode("arcade")
  wheels.setABSBehavior("arcade")

  extensions.load("cruiseControl")
  extensions.load("straightLine")
  straightLine.onInit()
  straightLine.setTargetDirection(vec3(20, -10, 0.5), "road")
  cruiseControl.setSpeed(hundredTenKmh)

  local maxSpeed = -1
  local maxSpeedTime = timer
  local speed = 0

  timer = 0
  while not cruiseControl.hasReachedTargetSpeed do
    coroutine.yield()

    speed = electrics.values.airspeed

    if timer > 20 and speed <= fiveKmh then
      print("Can't accelerate, aborting...")
      return
    end

    if speed - maxSpeed >= 0.1 then
      maxSpeed = speed
      maxSpeedTime = timer
    end

    if timer - maxSpeedTime > 15 then
      print("Stopped accelerating, can't go fast enough, aborting...")
      return
    end

    if timer > 120 then
      print("Can't accelerate fast enough for brake test, aborting...")
      return
    end
  end

  wait(2)

  repeat
    input.event("brake", 1, 1)
    input.event("clutch", 1, 1)
    input.event("throttle", 0, 1)
    coroutine.yield()
  until (electrics.values.airspeed <= hundredKmh) --wait for the car to start slowing down before actually timing it

  local startingPosition100 = obj:getPosition()
  local startingPosition60 = nil
  while electrics.values.airspeed > 0.5 do
    input.event("brake", 1, 1)
    input.event("clutch", 1, 1)
    if electrics.values.airspeed <= sixtyMph and not startingPosition60 then
      startingPosition60 = obj:getPosition()
    end
    coroutine.yield()
  end

  local endPosition = obj:getPosition()
  local distance100 = (endPosition - startingPosition100):length()
  local distance60 = (endPosition - startingPosition60):length() * 3.28084
  local avgDeceleration = -(square(electrics.values.airspeed) - square(hundredKmh)) / (2 * distance100)

  input.event("brake", 0, 1)
  input.event("clutch", 0, 1)

  wait(1)

  print("Brake distance: " .. tostring(distance100) .. " m")

  -- Prevents division by zero gravity
  local gravity = obj:getGravity()
  gravity = max(0.1, abs(gravity)) * sign2(gravity)

  saveInfo(
    {
      ["100-0 km/h"] = round(distance100 * 10) / 10,
      ["60-0 mph"] = round(distance60 * 10) / 10,
      ["Braking G"] = round(avgDeceleration / abs(gravity) * 1000) / 1000
    },
    {"100-0 km/h", "60-0 mph", "Braking G"}
  )

  obj:queueGameEngineLua("be:resetVehicle(0)")
  wait(1)
end

local function offroadTests()
  if wheels.wheelCount <= 0 then
    return
  end

  resetVehicle("0,0,0.5,0,0,0,1")

  extensions.load("straightLine")
  straightLine.onInit()
  straightLine.setTargetDirection(vec3(0, 600, 0), "offroad")

  controller.mainController.setGearboxMode("arcade")
  local esc = controller.getController("esc")
  if esc then
    esc.pauseESCAction = true
  end

  for _, diff in pairs(powertrain.getDevicesByType("differential")) do
    if diff.mode ~= "locked" then
      print("Setting diff '" .. diff.name .. "' to locked")
      diff:setMode("locked")
    end
  end

  for _, shaft in pairs(powertrain.getDevicesByType("shaft")) do
    if shaft.mode ~= "connected" then
      print("Setting shaft '" .. shaft.name .. "' to connected")
      shaft:setMode("connected")
    end
  end

  for _, rangebox in pairs(powertrain.getDevicesByType("rangeBox")) do
    if rangebox.mode ~= "low" then
      print("Setting rangebox '" .. rangebox.name .. "' to low")
      rangebox:setMode("low")
    end
  end

  extensions.load("cruiseControl")
  cruiseControl.minimumSpeed = tenKmh
  cruiseControl.setSpeed(tenKmh)

  local startingPosition = obj:getPosition()
  local firstBeamBreakPosition = nil
  wait(5)
  local stats = obj:calcBeamStats()
  local startingBrokenBeams = stats.beams_broken

  local distance = 0
  local beamBrokenDistance = -1

  local maxDistance = -1
  local maxDistanceTime = timer

  timer = 0

  while distance < 500 do
    local endPosition = obj:getPosition()
    distance = (endPosition - startingPosition):length()
    stats = obj:calcBeamStats()
    if not firstBeamBreakPosition and stats.beams_broken > startingBrokenBeams then
      firstBeamBreakPosition = obj:getPosition()
      beamBrokenDistance = (firstBeamBreakPosition - startingPosition):length()
      print("beam broken")
    end

    if distance - maxDistance >= 0.1 then
      maxDistance = distance
      maxDistanceTime = timer
    end

    if timer - maxDistanceTime > 10 then
      print("Stopped advancing, can't go further, aborting...")
      break
    end
    coroutine.yield()
  end

  straightLine.stop()
  cruiseControl.setEnabled(false)

  if beamBrokenDistance < 0 then
    beamBrokenDistance = distance
  end

  print("Off-Road distance: " .. tostring(distance) .. " m")
  print("Off-Road beam break distance: " .. tostring(beamBrokenDistance) .. " m")
  saveInfo({["Off-Road Score"] = math.ceil(((3 * distance + beamBrokenDistance) / 500 / 4) * 100)}, {"Off-Road Score"})

  if esc then
    esc.pauseESCAction = true
  end
end

local function updateGFX(dt)
  timer = timer + dt

  if workerCoroutine ~= nil then
    local errorfree, value = coroutine.resume(workerCoroutine)
    if not errorfree then
      log("E", logTag, debug.traceback(workerCoroutine, "workerCoroutine: " .. value))
    end
    watchdogHeartbeat()
    if coroutine.status(workerCoroutine) == "dead" then
      log("I", logTag, "coroutine dead, hitting killswitch")
      killswitch()
      workerCoroutine = nil
      return
    end
  end
end

local function performTests(_model_key, _config_key)
  workerCoroutine =
    coroutine.create(
    function()
      -- save for later usage
      model_key = _model_key
      config_key = _config_key
      log("I", logTag, string.format(" *** testing car: %s->%s ***", model_key, config_key))

      log("I", logTag, " *** getting static performance data ***")
      writeBasicPerformanceData()

      log("I", logTag, " *** getting static powertrain data ***")
      writeBasicPowertrainData()

      log("I", logTag, " *** getting static powertrain layout data ***")
      writePowertrainLayoutData()

      log("I", logTag, " *** getting bounding box data ***")
      writeBBData()

      log("I", logTag, " *** getting offroad data ***")
      offroadTests()

      log("I", logTag, " *** getting acceleration data ***")
      local canReach115 = accelerationTests()

      if canReach115 then
        log("I", logTag, " *** getting braking data ***")
        brakingTests()
      else
        log("I", logTag, " *** vehicle not fast enough for braking tests, skipping ***")
      end

      --local touchedFilePath = "vehicles/" .. model_key .. "/info_" .. config_key .. ".touched"
      --jsonWriteFile(touchedFilePath, {}, true)

      log("I", logTag, " *** finished ***")
    end
  )
end

-- public interface
M.onInit = onInit
M.onReset = onInit
M.performTests = performTests
M.updateGFX = updateGFX
M.getPowertrainLayout = getPowertrainLayout

return M
