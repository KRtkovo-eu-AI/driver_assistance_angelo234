-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Other control parameters.
local maxVehicleRangeSq = 5000.0        -- The maximum squared distance from the player vehicle, at which other vehicles can be detected.
local NaN = 0/0

local max, min, abs, sqrt, acos, deg = math.max, math.min, math.abs, math.sqrt, math.acos, math.deg  -- for objects data
-- Local road data/other vehicle data.
local lastVelPlayer = vec3(0, 0)     -- An initial (default) starting value for the player vehicle velocity.
local lastPos, lastVel = {}, {}         -- Initialise tables to store the last-known position and velocity data, for all other vehicles in the simulator.
local lastDt = 0.0                      -- The previous time step (used for velocity and acceleration computations).
local vehiclesOldData = {}

-- Player vehicle state.
local pos = vec3(0, 0)                                                    -- The player vehicle position.
local fwd, right = vec3(0,0), vec3(0, 0)                                 -- The player vehicle orthogonal frame.
local vehFront = vec3(0, 0)                                               -- The player vehicle front/rear bumper midpoint positions.
local vel, acc = vec3(0, 0), vec3(0, 0)                                   -- The player vehicle velocity and acceleration vectors.
local vehLength = 0
local nullReading = {
  vehicleID = 0, width = 0, length = 0,
  positionB = { x = 0, y = 0, z = 0 },
  distToPlayerVehicleSq = 0, relDistX = 0, relDistY = 0,
  velBB = { x = 0, y = 0, z = 0 }, acc = { x = 0, y = 0, z = 0 },
  relVelX = 0, relVelY = 0, relAccX = 0, relAccY = 0 }

-- the other vehicles
local simData = {
  closestVehicles1 = nullReading,
  closestVehicles2 = nullReading,
  closestVehicles3 = nullReading,
  closestVehicles4 = nullReading }

local csvWriter = require('csvlib')
-- local platoonArch = require('ge\\extensions\\tech\\PlatoonArch')
-- local platoonFunctions = require('ge\\extensions\\tech\\platoonFunctions')

local timer
local csvData

local standStillDistance = 12   -- target distance at v = 0 in m
local minimumDistance = 12
local vehicleSize
local deltaTime = 3               -- delta time in s
local prevDistance = 0
local prevDistanceToCars = {}
local prevSpeed = 0
local distanceToLeadCar = 100
local leadingSpeed = 100
local lostTrack = false
local lostTrackCounter = 0

local pastU = 0
local mass = 0
local prevLeaderSpeed = 0
local WINDOW_SIZE = 20
local leaderSpeedBuffer = {}

--local vid
local WIDTH = 100
local HEIGHT = 100
local resolution = {WIDTH, HEIGHT}
local targetSpeed
local data
local debug
local vehicleID
local noCars
local mode
local lastUpdateTime = os.clock()  -- Initialize the last update time
local lastUpdate = os.clock()
local timeToPrint = false
local maintainSpeedFlag = false
local dataToSave = "Time,Longitudinal_Position_Vehicle_One,Velocity_Vehicle_One,Acceleration_Vehicle_One,Longitudinal_Position_Vehicle_Two,Velocity_Vehicle_Two,Acceleration_Vehicle_Two\n"
local vehicleIndexInPlatoon = 0
local vehicleId
local platoonLeaderID
local loaded = false
local noData = false
local lastMailboxVersionVehicleIndex = nil
local currentMailboxVersionVehicleIndex = nil
local lastMailboxVersionLeaderID = nil
local currentMailboxVersionLeaderID = nil
local lastMailboxVersionUltrasonic = nil
local currentMailboxVersionUltrasonic = nil
local ultrasonicHazardDetection = false
local mailboxSensorName = nil
local mailboxNameLeader = nil
local vehicleInfront = nil
local updatedSpeed = nil
local platoonsSpeed = nil
local simTime = 0
local relayVehicles = {}
local relayVehiclesVel = {}
local velocities = 0

-- Projects a vector onto another vector.
local function project(a, b) return (a:dot(b) / b:dot(b)) * b end

-- Computes the relative quantity (difference) between two vectors in the direction of a given axis.
local function getRelativeQuantity(v1, v2, axis) return project(v2, axis):length() - project(v1, axis):length() end

-- Sorts a table by instances' squared distance value.
local function getKeysSortedByDistanceSq(tbl, sortFunction)
  local keys = {}
  for key in pairs(tbl) do
    table.insert(keys, key)
  end
  table.sort(keys, function(a, b)
    return sortFunction(tbl[a], tbl[b])
  end)
  return keys
end

-- A sort function, used to sort vehicles by squared distance to player vehicle.
local function sortAscending(a, b) return a.distToPlayerVehicleSq < b.distToPlayerVehicleSq end

local function createCSV()
  csvData = csvWriter.newCSV("time", "speed", "targetSpeed", "throttle")
  timer = 0
end

local function saveCSV()
  -- save in AppData/Local/BeamNG.drive/<current Version>
  csvData:write('testLog')
end

local function mMultiplication(A, B)
  local rows = #A
  local columns = #B[1]
  local length = #B
  local C = {}

  for i = 1, rows do
    table.insert(C, i, {})
    for j = 1, columns do
      table.insert(C[i], j, 0)
      for l = 1, length do
        C[i][j] = C[i][j] + A[i][l] * B[l][j]
      end
    end
  end
  return C
end

local function mPower(A, n)
  local B = A
  for i = 1, (n - 1) do
    B = mMultiplication(B, A)
  end
  return B
end

local function mSum(A, B)
  local C = {}
  for i = 1, #A do
    table.insert(C, i, {})
    for j = 1, #A[1] do
      table.insert(C[i], j, A[i][j] + B[i][j])
    end
  end

  return C
end

local function mMultiplicationScalar(A, b)
  local C = {}
  for i = 1, #A do
    table.insert(C, i, {})
    for j = 1, #A[1] do
      table.insert(C[i], j, A[i][j] * b)
    end
  end

  return C
end

local function mTranspose(A)
  local AT = {}
  for i = 1, #A[1] do
    table.insert(AT, i, {})
    for j = 1, #A do
      table.insert(AT[i], j, 0)
    end
  end

  for i = 1, #A[1] do
    for j = 1, #A do
      AT[i][j] = A[j][i]
    end
  end
  return AT
end

local function mDeterminant(A)
  local n = #A
  local toggle = 1
  local lum = {}
  for i = 1, #A do
    table.insert(lum, i, {})
    for j = 1, #A do
      table.insert(lum[i], j, A[i][j])
    end
  end

  local perm = {}
  for i = 1, n do
    table.insert(perm, i, i)
  end

  for j = 1, n do
    local max = math.abs(lum[j][j])
    local piv = j

    for i = j+1, n do
      local xij = math.abs(lum[i][j])
      if xij > max then
        max = xij
        piv = i
      end
    end

    if piv ~= j then
      lum[j] = A[piv]
      lum[piv] = A[j]

      local t = perm[piv]
      perm[piv] = perm[j]
      perm[j] = t

      toggle = - toggle
    end

    local xjj = lum[j][j]

    if xjj ~= 0 then
      for i = j+1, n do
        local xij = lum[i][j] / xjj
        lum[i][j] = xij
        for k = j+1, n do
          lum[i][k] = lum[i][k] - xij * lum[j][k]
        end
      end
    end
  end

  local det = toggle
  for i = 1, n do
    det = det * lum[i][i]
  end

  return det
end

local function mInverse(A)
  local adj = {}
  for i = 1, #A do
    table.insert(adj, i, {})
    for j = 1, #A do
      local M = {}
      for i = 1, #A do
        table.insert(M, i, {})
        for j = 1, #A do
          table.insert(M[i], j, A[i][j])
        end
      end
      table.remove(M, i)
      for k = 1, #M do
        table.remove(M[k], j)
      end
      table.insert(adj[i], j, (-1)^(i + j)*mDeterminant(M))
    end
  end
  return mMultiplicationScalar(adj, 1/mDeterminant(A))
end

local function printMatrix(A)
  for i = 1, #A do
    dump(A[i])
  end
end

local function getMass()  -- !!!!!!!!!!!!!!!!!!
  for _, n in pairs(v.data.nodes) do
    mass = mass + n.nodeWeight
  end
end

local function average(t)
  local sum = 0
  for _,v in pairs(t) do
    sum = sum + v
  end
  return sum / #t
end

local function getAccFactor()
  local totalForce = 0
  local devices = powertrain.getDevicesByCategory("engine")
  for i = 1, #devices do
    local torqueData = devices[i].torqueCurve
    local gearRatio = devices[i].cumulativeGearRatio
    local torque = average(torqueData) * gearRatio
    local connectedWheels = powertrain.getChildWheels(devices[i], 1)
    local radius = connectedWheels[1].dynamicRadius
    local force = torque / radius
    totalForce = totalForce + force
  end
  local accFactor = totalForce /mass
  return accFactor
end

local function computeKmpc()
  local resFactor = - 0.0306
  local accFactor = getAccFactor()
  local Ts = 0.1
  local N = 5
  local Q = {{1}}
  local R = 10

  local A = {{1 + resFactor * Ts, Ts * (1 + 1/2*resFactor*Ts) * accFactor}, {0, 1}}
  local B = {{Ts * (1 + 1/2*resFactor*Ts) * accFactor}, {1}}
  local C = {{1, 0}}

  local Phi = {}
  for i = 1, N do
    local A_i = mPower(A, i)
    table.insert(Phi, i*2 - 1, A_i[1])
    table.insert(Phi, i*2, A_i[2])
  end

  local Gamma = {}
  for i = 1, N*2 do
    table.insert(Gamma, i, {})
    for j = 1, N do
      table.insert(Gamma[i], j, 0)
    end
  end

  for i = 2, N do
    for j = 1, (i - 1) do
      local A_i = mPower(A, i - j)
      local Gamma_i = mMultiplication(A_i, B)
      Gamma[i*2 - 1][j] = Gamma_i[1][1]
      Gamma[i*2][j] = Gamma_i[2][1]
    end
  end
  for i = 1, N do
    Gamma[i*2 - 1][i] = B[1][1]
    Gamma[i*2][i] = B[2][1]
  end

  local Omega = {}
  for i = 1, N*2 do
    table.insert(Omega, i, {})
    for j = 1, N*2 do
      table.insert(Omega[i], j, 0)
    end
  end
  local Omega_i = mMultiplication(mMultiplication(mTranspose(C), Q), C)
  for i = 1, N do
    Omega[i*2 - 1][i*2 - 1] = Omega_i[1][1]
    Omega[i*2][i*2 - 1] = Omega_i[2][1]
    Omega[i*2 - 1][i*2] = Omega_i[1][2]
    Omega[i*2][i*2] = Omega_i[2][2]
  end

  local Sigma = {}
  for i = 1, N*2 do
    table.insert(Sigma, i, {})
    for j = 1, N do
      table.insert(Sigma[i], j, 0)
    end
  end
  local Sigma_i = mMultiplication(mTranspose(C), Q)
  for i = 1, N do
    Sigma[i*2 - 1][i] = Sigma_i[1][1]
    Sigma[i*2][i] = Sigma_i[2][1]
  end

  local Psi = {}
  for i = 1, N do
    table.insert(Psi, i, {})
    for j = 1, N do
      table.insert(Psi[i], j, 0)
      if i == j then
        Psi[i][j] = R
      end
    end
  end

  local G = mMultiplicationScalar(mSum(mMultiplication(mMultiplication(mTranspose(Gamma), Omega), Gamma), Psi), 2)
  local F = mMultiplicationScalar(mMultiplication(mMultiplication(mTranspose(Gamma), Omega), Phi), 2)
  local F_2 = mMultiplicationScalar(mMultiplication(mTranspose(Gamma), Sigma), -2)
  for i = 1, #F do
    for j = 1, #F_2[1] do
      table.insert(F[i], 2 + j, F_2[i][j])
    end
  end

  local I = {{-1}}
  for i = 2, N do
    table.insert(I[1], i, 0)
  end

  local Kmpc = mMultiplication(mMultiplication(I, mInverse(G)), F)
  return Kmpc[1]
end
-------------------------------------------------

local function calcAvgSpeed (speeds)
  local total = 0
  for _, speed in ipairs(speeds) do
    total = total + speed
  end
  return total/#speeds
end

local function detectSpeedTrend(currentSpeed, targetSpeedIn) --function for calculating the speed trends
  table.insert(leaderSpeedBuffer, currentSpeed)--adding new speeds to the buffer and removing the oldest ones

  if #leaderSpeedBuffer > WINDOW_SIZE then
    table.remove(leaderSpeedBuffer, 1)
  end

  local averageSpeed = calcAvgSpeed(leaderSpeedBuffer)
  local threshold = 0.1 --threshold for detecting the acceleration of deceleration


  if targetSpeedIn == 0 then
    return "Car Stopped"
  elseif currentSpeed > averageSpeed + threshold then
    return "Accelerating"
  elseif currentSpeed < averageSpeed - threshold then
    return "Decelerating"
  else
    return "Maintaining Speed"
  end
end

local function adjustThrottle(velocityDifference)
  local throttlePower = 0
  local maxVelocityDifference = 5

  if velocityDifference > 0 then
    throttlePower = math.min(1, velocityDifference / maxVelocityDifference)
  elseif velocityDifference < 0 then
    throttlePower = math.max(0, -velocityDifference / maxVelocityDifference)
  end

  return throttlePower
end




local function MPCOverride(targetSpeedVal)
  ui_message("Cruise Override", 0.01, "Tech", "forward")

  -- Calculate the time elapsed since the last update
  local currentTime = os.clock()
  local deltaTime = currentTime - (lastUpdateTime or currentTime)
  lastUpdateTime = currentTime

  deltaTime = math.min(deltaTime, 0.1)

  local velocityyx = obj:getVelocity().x
  local velocityyy = obj:getVelocity().y
  local velocityyz = obj:getVelocity().z

  local currentSpeed = electrics.values.wheelspeed -- Speed of the vehicle
  local velocityy = math.sqrt(velocityyx ^ 2 + velocityyy ^ 2 + velocityyz ^ 2)

  -- Compute throttle adjustment
  local targetThrottle = adjustThrottle(targetSpeedVal)

  -- Throttle smoothing logic
  local throttleTransitionRate = 0.05 -- Determines how quickly throttle changes
  if throttleValue < targetThrottle then
      throttleValue = math.min(throttleValue + throttleTransitionRate * deltaTime, targetThrottle)
  elseif throttleValue > targetThrottle then
      throttleValue = math.max(throttleValue - throttleTransitionRate * deltaTime, targetThrottle)
  end

  if (math.abs(targetSpeedVal - currentSpeed)) <= 1 then
      electrics.values.throttleOverride = nil
  elseif electrics.values.brake > 0.05 then
      electrics.values.throttleOverride = nil
  elseif electrics.values.throttle > 0.05 then
      electrics.values.brakeOverride = nil
  elseif electrics.values.isShifting then
      targetThrottle = adjustThrottle(targetSpeedVal)
  elseif throttleValue > 0 then
      electrics.values.throttleOverride = throttleValue
  end

  -- Log output for debugging
  if printDebugLogs then
      log("I", "", "targetSpeed: " .. tostring(targetSpeedVal)
          .. ", currentSpeed: " .. tostring(currentSpeed)
          .. ", targetThrottle: " .. tostring(targetThrottle)
          .. ", smoothedThrottle: " .. tostring(throttleValue))
  end
end



local function MPC(mode, encodedDistances, targetSpeedIn, inputSpeed, vehicleID, dtSim, debug)
  local currentTime = os.clock()

  local timediff = currentTime - lastUpdate

  if timediff > 1 then
    lastUpdate = currentTime
    dataToSave = dataToSave..currentTime
    timeToPrint = true
  end


  -- Calculate the time elapsed since the last update
  deltaTime = currentTime - lastUpdateTime

  -- Update the last update time for the next iteration
  lastUpdateTime = currentTime

  local velocityyx = obj:getVelocity().x
  local velocityyy = obj:getVelocity().y
  local velocityyz = obj:getVelocity().z

  local velocityy = math.sqrt(velocityyx^2 + velocityyy^2 + velocityyz^2) --following vehcile's velocity
  
  local currentSpeed = electrics.values.wheelspeed    --speed of the vehicle with the acc
  local speedDifference = math.sqrt((velocityy-targetSpeedIn)^2)
  local distanceToCars = encodedDistances
  local targetDistance = standStillDistance + deltaTime * currentSpeed --for testing
  
  local targetDistanceOld = standStillDistance + deltaTime * currentSpeed  --for testing
  


  if mode == "platoon" then
    --3 Cases:
    --1. ego vehicle moving faster/same speed/slower than leading vehicle but distance is more than what it should be(accelerate) ~ distance +ve
    --2. ego vehicle moving faster/slower than the leading vehicle but distance is less than than  it should be(decelerate) ~ speed difference +ve & distance -ve
    --3. ego vehicle moving ~same speed as leading vehicle and distance is almost equal to standstill distance (maintain speed) ~ speed diff and distance are equal

    local timeGap = 1.0
    local targetDistance = standStillDistance + deltaTime * speedDifference + velocityy*0.1-- currentSpeed was used(only when there are two vehicles on the street and not in a pltoon this would work), replace bu speed difference between both vehicles--deltatime???? if deltatime is the time needed for the vehicle to stop then it needs to be editted to match the breking power
   


  end

  if vehiclesOldData[vehicleID] then
    local observedSpeed = (distanceToCars - vehiclesOldData[vehicleID]) / dtSim + prevSpeed --changed to variable the form the function call insetad of from the for loop on the table
    distanceToLeadCar = distanceToCars
    leadingSpeed = observedSpeed
  end




  local speed2 = leadingSpeed + (distanceToLeadCar - targetDistance) / dtSim --is negative, maintain same speed, add it with the conditions for acc and dec

  local speed2 = velocityy + (distanceToLeadCar - targetDistance) / dtSim
  targetSpeed = math.min(math.max(speed2, velocityy), targetSpeedIn) --add .max for -ve speed and the velocity is fot the leading vehicle's speed --brakes because of zero
  local distanceError = distanceToLeadCar - targetDistance
  local Kp = 0.1
  if distanceToLeadCar > targetDistance*1.1 then
    maintainSpeedFlag = false
    targetSpeed = targetSpeedIn + distanceError/targetDistance*targetSpeedIn
    
  elseif distanceToLeadCar < targetDistance*0.9 then
    targetSpeed = 0.95*targetSpeedIn
    maintainSpeedFlag = false
  else
    
    maintainSpeedFlag = true
    targetSpeed = targetSpeedIn
  end

  local Kmpc = computeKmpc()
  local deltaU = Kmpc[1]*velocityy + Kmpc[2]*pastU + Kmpc[3]*targetSpeed + Kmpc[4]*targetSpeed + Kmpc[5]*targetSpeed + Kmpc[6]*targetSpeed + Kmpc[7]*targetSpeed
  local u = pastU + deltaU
  local u2 = u


  if math.floor(distanceToLeadCar) > math.floor(targetDistance) and targetSpeedIn > 0 then --add a condition for targetspeed==0?
    targetSpeed = targetSpeedIn + distanceError/targetDistance*targetSpeedIn
    u2 = adjustThrottle(targetSpeedIn-targetSpeed)
  
  elseif math.floor(distanceToLeadCar) > math.floor(targetDistance) and targetSpeedIn == 0 then --brakes onn and off issue still on --edittied check if problems occur, removed targetSpeedIn == 0 from conition, was anded
   
    targetSpeed = inputSpeed
    u2 = 1
    
  end
  local velDiff = velocityy - targetSpeedIn

  if targetSpeedIn == 0 then
    targetDistance = standStillDistance
    
  end


  

  if distanceToLeadCar > targetDistance  and targetSpeedIn == 0 then
    
    targetSpeed = targetSpeedIn + Kp * distanceError
    
    u2 = adjustThrottle(targetSpeedIn-targetSpeed)
  end


  u = u2
  
  local leaderSpeedingState = detectSpeedTrend(targetSpeed, targetSpeedIn) --using the leading speed value in this function
  

  if u >= 1  then
   
    u = 1
  elseif u <= -1.0 then

    u = -1
  elseif electrics.values.isShifting and velDiff < 0 and distanceToLeadCar-targetDistance > 0 then
    
    u = adjustThrottle(targetSpeed)

  end


  


  if u > 0  then
    electrics.values.throttleOverride = u electrics.values.brakeOverride = 0
   
  elseif (u == 0 and targetSpeed > 0)  then --added for ego vehicle to move once leader vehicle moves
    
    electrics.values.throttleOverride = u electrics.values.brakeOverride = nil
  else -- added the if and then parts
    electrics.values.throttleOverride = nil electrics.values.brakeOverride = -u --stopping and pressing breaks
    
  end
  if debug then
    local time = math.floor(timer * 1000) / 1000 -- Make sure time doesn't have dozen of digits
    csvData:add(time, velocityy, targetSpeed, u) timer = timer + dtSim
  end

  pastU = u

  prevSpeed = velocityy

  vehiclesOldData[vehicleID] = distanceToCars
end

-----------------------------------------------------------------

local function changeSpeed(speed)
  targetSpeed = speed
end


local function vehiclesData(dtSim)
  -- compute the objects data
  -- Update the player vehicle properties.
  mailboxNameLeader = "currentLeader"..vehicleId
  lastMailboxVersionLeaderID = obj:getLastMailboxVersion(mailboxNameLeader) --NFSA
  --checking if vehicle index has changed.
  if currentMailboxVersionLeaderID ~= lastMailboxVersionLeaderID then -- it changed when any vehicle leaves the platoon
    
    currentMailboxVersionLeaderID = lastMailboxVersionLeaderID
    platoonLeaderID = lpack.decode(obj:getLastMailbox(mailboxNameLeader))
    
  end

  local mailBoxNameVehicleInfront = "vehicleInfront"..vehicleId
  local vehicleInfront = lpack.decode(obj:getLastMailbox(mailBoxNameVehicleInfront))

    

  pos = obj:getPosition()
  vel = obj:getVelocity()
  fwd = obj:getForwardVector():normalized()
  right = obj:getDirectionVectorRight():normalized()
  vehFront = obj:getFrontPosition()

  local vehID = obj:getID()
  local playerPosToFront = vehFront - pos
  local lastDtInv = 1.0 / max(1e-12, lastDt)
  acc = (vel - lastVelPlayer) * lastDtInv                                                                         -- Use FD once to get acceleration.
  lastVelPlayer = vel

  local vx = tonumber(vel.x)
  local vy = tonumber(vel.y)
  local velocity = math.sqrt( vx^2 + vy^2)*3.6

  local ax = tonumber(acc.x)
  local ay = tonumber(acc.y)
  local accel = math.sqrt( ax^2 + ay^2)
  local posPos = pos.y*-1

  if timeToPrint == true then
    dataToSave = dataToSave..","..posPos..","..velocity..","..accel
  end

  -- Compute the relevant properties for the other vehicles.
  local vehicles, ctr = {}, 1
  for k, v in pairs(mapmgr.getObjects()) do
    if k == tonumber(vehicleInfront) then -- for dmpc key
      
      -- Compute the position, velocity and acceleration of this other vehicle.
      
      local posB, velB, accB = obj:getObjectCenterPosition(k), vec3(0, 0, 0), vec3(0, 0, 0)
      velB = v.vel
      
      if lastPos[k] ~= nil then
        local velR = (posB - lastPos[k]) * lastDtInv                                                                    -- Use FD once to get velocity.
        
        local vxR = tonumber(velR.x)
        local vyR = tonumber(velR.y)
        local velocityR = math.sqrt( vxR^2 + vyR^2)*3.6 
        
      end
      if lastVel[k] ~= nil then
        accB = (velB - lastVel[k]) * lastDtInv                                                                    -- Use FD twice to get acceleration.
      end
      lastPos[k], lastVel[k] = posB, velB
      local vx = tonumber(velB.x)
      local vy = tonumber(velB.y)
      local velocityB = math.sqrt( vx^2 + vy^2)*3.6
     
      local ax = tonumber(accB.x)
      local ay = tonumber(accB.y)
      local accelB = math.sqrt( ax^2 + ay^2)
      local posBPos = posB.y*-1


      -- Store the data of all other vehicles which satisfy the following conditions:
      -- i) within a certain range of the player vehicle.
      -- ii) facing the same direction as the player vehicle.
      -- iii) in front of the player vehicle.
      local fwdB = obj:getObjectDirectionVector(k)
      fwdB:normalize()
      local distantPoint = pos + (1e12 * fwd)                                                                     -- A point on the player vehicle forward vector, far in the distance.
      -- if fwd:dot(fwdB) > 0.0 and (distantPoint - pos):lenSquared() > (distantPoint - posB):lenSquared() then      -- Test that other vehicle is in front hemisphere and has same dir. --edit for 360 comment it
      local distToPlayerVehicleSq = (pos - posB):lenSquared()
                                               -- The squared distance between the player and other vehicle.
      if distToPlayerVehicleSq < maxVehicleRangeSq then     --max distance for radar                                                    -- Only consider vehicles which are within the set range.

        local upB = obj:getObjectDirectionVectorUp(k)                                                           -- The other vehicle's frame.
        upB:normalize()
        local widthB, lengthB = obj:getObjectInitialWidth(k), obj:getObjectInitialLength(k)                     -- The other vehicle's dimensions.
        local frontB = obj:getObjectFrontPosition(k)                                                            -- The other vehicle's front/rear bumper midpoint positions.
        local rearB = frontB - (fwdB * lengthB)
        local playerPosToBRear = rearB - pos                                                                    -- The vector from the player position to the other vehicle rear.
        local relDistX = getRelativeQuantity(playerPosToFront, playerPosToBRear, fwd)                           -- The relative distance to the player vehicle front position.
        local relDistY = getRelativeQuantity(playerPosToFront, playerPosToBRear, right)
        local relVelX, relVelY = getRelativeQuantity(vel, velB, fwd), getRelativeQuantity(vel, velB, right)     -- The relative velocity wrt the player vehicle frame.
        local relAccX, relAccY = getRelativeQuantity(acc, accB, fwd), getRelativeQuantity(acc, accB, right)     -- The relative acceleration wrt the player vehicle frame.
        vehicles[ctr] = {
          vehicleID = k,
          width = widthB, length = lengthB,
          distToPlayerVehicleSq = distToPlayerVehicleSq or 0.0,
          relDistX = relDistX, relDistY = relDistY,
          velBB = velB,
          relVelX = relVelX, relVelY = relVelY,
          acc = accB,
          relAccX = relAccX, relAccY = relAccY }
        ctr = ctr + 1
      end
  
    end
  end


  -- Sort the candidate vehicles by their squared distance to the player vehicle, ascending.
  local vClosest1, vClosest2, vClosest3, vClosest4 = nullReading, nullReading, nullReading, nullReading
  local sortMap = getKeysSortedByDistanceSq(vehicles, sortAscending)
  if sortMap[1] ~= nil then
    vClosest1 = vehicles[sortMap[1]]
  end
  if sortMap[2] ~= nil then
    vClosest2 = vehicles[sortMap[2]]
  end
  if sortMap[3] ~= nil then
    vClosest3 = vehicles[sortMap[3]]
  end
  if sortMap[4] ~= nil then
    vClosest4 = vehicles[sortMap[4]]
  end

  simData = {
    -- time = obj:getSimTime(),
    closestVehicles1 = vClosest1,
    closestVehicles2 = vClosest2,
    closestVehicles3 = vClosest3,
    closestVehicles4 = vClosest4 }
   -- Cycle the dt values, so we have a recent memory (used in the velocity and acceleration computations).
  lastDt = dtSim
end

local function leavePlatoon(leaderID,vid)
  
  loaded = false
  electrics.values.throttleOverride = nil
  electrics.values.brakeOverride = nil
  local vehiclesData = {leaderID = leaderID, vehicleID = vid}
  obj:queueGameEngineLua(string.format("tech_platoonFunctions.removeVehicleFromPlatoon(%q)", lpack.encode(vehiclesData)))
  ai.setMode('manual')
  
  ui_message("Vehicle Exit Platoon", 5, "Tech", "forward")
end

local function updateGFX(dtSim)

  if launched then
    vel = obj:getVelocity()
    local vx = tonumber(vel.x)
    local vy = tonumber(vel.y)
    local velocity = math.sqrt( vx^2 + vy^2)
   
    simTime = simTime + dtSim
   

    for i, vehicles in ipairs(relayVehicles) do 
      for k, v in pairs(mapmgr.getObjects()) do
        if k == tonumber(vehicles) then
          velocityRelay = math.sqrt(v.vel.x^2 + v.vel.y^2) 
         
          if velocityRelay >= math.floor(velocity*0.90) then
           
            relayVehiclesVel[i] = 1
          else
          
            relayVehiclesVel[i] = 0
          end
        end
      end
      
    end

    velocities = 0
    for i, k in ipairs(relayVehiclesVel) do
      velocities = velocities + k
    end
   

    if (velocities == #relayVehiclesVel and updatedSpeed < platoonsSpeed and velocity>= math.floor(updatedSpeed*0.90) ) or (simTime > 5 and updatedSpeed < platoonsSpeed) then
      
      simTime = 0
      updatedSpeed = updatedSpeed + 2
      ai.setSpeed(updatedSpeed)

      
    end 


    if simTime > 5.2 then
      updatedSpeed = velocity
      simTime = 0
    end
    
  end

  if not loaded then
    return
  end

 
  lastMailboxVersionVehicleIndex = obj:getLastMailboxVersion(tostring(vehicleId)) --NFSA
 
  if currentMailboxVersionVehicleIndex ~= lastMailboxVersionVehicleIndex then -- it changes when any vehicle leaves the platoon
   
    currentMailboxVersionVehicleIndex = lastMailboxVersionVehicleIndex
    vehicleIndexInPlatoon = (lpack.decode(obj:getLastMailbox(tostring(vehicleId))))-1
   
  end
  --for NFSA, need to check the mailbox if updated here for the vehicleIndex -------NFSA
  mailboxNameLeader = "currentLeader"..vehicleId
  lastMailboxVersionLeaderID = obj:getLastMailboxVersion(mailboxNameLeader) --NFSA
  --checking if vehicle index has changed.
  if currentMailboxVersionLeaderID ~= lastMailboxVersionLeaderID then -- it changed when any vehicle leaves the platoon
   
    currentMailboxVersionLeaderID = lastMailboxVersionLeaderID
    platoonLeaderID = lpack.decode(obj:getLastMailbox(mailboxNameLeader))
   
    vehicleSize = obj:getObjectInitialLength(tonumber(platoonLeaderID))
   
  end
  
  standStillDistance = minimumDistance

  local distance
  if lostTrackCounter > 5 then
    
    leavePlatoon(platoonLeaderID,vehicleId) -- check, function editted for NFSA
    loaded = false
  end
  

  

  vehiclesData(dtSim)
  if next(simData) ~= nil then  -- simData contains the data of the closest vehicles
    for k, v in pairs(simData) do
      if type(v) == "table" then
        if k =="closestVehicles1" then  -- closest detected vehicle
          for k2,v2 in pairs(v) do
            if k2 == "distToPlayerVehicleSq" then
              if v2 ~= nil and v2 ~= 0 then
                distance = math.sqrt(v2)  -- distance between the vehicles
                
                lostTrackCounter = 0
              elseif v2 == nil or v2 == 0 then
                lostTrackCounter = lostTrackCounter + 1
               
              end
            end
            if k2 == "vehicleID" then
              vehicleID = v2
              
            end

            if k2 == "velBB" then
              if type(v2) == "cdata" then
                local x = tonumber(v2.x)
                local y = tonumber(v2.y)
                local z = tonumber(v2.z)
                local sq2 = math.sqrt( x^2 + y^2)

                if sq2 <= 0.01 then
                  sq2 = 0
                end
                targetSpeed = sq2
              end
            end
          end
         
        end
      end

    end

  
  end

  lastMailboxVersionUltrasonic = obj:getLastMailboxVersion(mailboxSensorName)
  if currentMailboxVersionUltrasonic ~= lastMailboxVersionUltrasonic then -- it changes when any vehicle leaves the platoon
    
    currentMailboxVersionUltrasonic = lastMailboxVersionUltrasonic
    distance = (lpack.decode(obj:getLastMailbox(mailboxSensorName)))
    
    ultrasonicHazardDetection = true
  
  else 
    ultrasonicHazardDetection = false
  end
  
  if ultrasonicHazardDetection == true then
    
    ui_message("Ultrasonic Detected Hazardous Distance", 5, "Tech", "forward")

  
    targetSpeed = targetSpeed*0.9
    
  end
  
  if distance then
    local inputSpeed = 3
    MPC(mode, distance, targetSpeed, inputSpeed, vehicleID, dtSim, debug)  -- call MPC with input : driving mode, distance, target speed, input speed and closest vehicle id
  end
end

local function getVehicleLength(vid)  -- length of the vehicle (id input)
  local length = obj:getObjectInitialLength(vid)
  return length
end

local function formPlatoon(leaderID, vid, speed, debugFlag)
  local leaderLength = getVehicleLength(leaderID)
  local vehicleLength = getVehicleLength(vid)
  local vehiclesLengthRatio = (math.abs(leaderLength-vehicleLength)/leaderLength)*100
  -- check if the vehicle's proportions fit
  if vehiclesLengthRatio <= 20 then
    loaded = true
    mode = "platoon"
    
    vehicleId = vid
    
    if not vid or vid == -1 then
      return
    end
    
    currentMailboxVersionVehicleIndex = obj:getLastMailboxVersion(tostring(vehicleId))
    vehicleIndexInPlatoon = (lpack.decode(obj:getLastMailbox(tostring(vehicleId))))-1 --NFSA
    mailboxSensorName = vehicleId.."UltrasonicReading"
    currentMailboxVersionUltrasonic = obj:getLastMailboxVersion(mailboxSensorName)
   
    leaderLength = getVehicleLength(leaderID)
   

    if leaderLength >= 4.8 and leaderLength <= 6.5 then

      standStillDistance = 12
      minimumDistance = 12
    elseif leaderLength < 4.8 then
      standStillDistance = 8
      minimumDistance = 8
    elseif leaderLength > 6.5 then
      standStillDistance = 15
      minimumDistance = 15
    else
      standStillDistance = minimum
    end
    ai.setTargetObjectID(leaderID)
    ai.setMode('follow')
    ai.driveInLane('on')

    assert(vid >= 0, "platooning.lua - Failed to get a valid vehicle ID")
    local radarArgs = {}
    targetSpeed = speed
    getMass()

    debug = debugFlag
    if debug then
      createCSV()
    end
    ui_message("Platoon leader assigned", 5, "Tech", "forward")
  else
    ui_message("vehicles sizes are not compatible for a platoon", 5, "Tech", "forward")
    -- loaded = false
  end
end

local function joinPlatoon(leaderID, vid, speed, debugFlag)  -- a new vehicle (id) is checked to join a platoon or not wrt its size proportions

  vehicleId = vid
  local leaderLength = getVehicleLength(leaderID)
  local vehicleLength = getVehicleLength(vid)
  local vehiclesLengthRatio = (math.abs(leaderLength-vehicleLength)/leaderLength)*100
  if vehiclesLengthRatio <= 20 then
    loaded = true
    mode = "platoon"
    
    currentMailboxVersionVehicleIndex = obj:getLastMailboxVersion(tostring(vehicleId))
    vehicleIndexInPlatoon = (lpack.decode(obj:getLastMailbox(tostring(vehicleId))))-1 --NFSA
    mailboxSensorName = vehicleId.."UltrasonicReading"
    currentMailboxVersionUltrasonic = obj:getLastMailboxVersion(mailboxSensorName)
   

    leaderLength = getVehicleLength(leaderID)
    

    if leaderLength >= 4.8 and leaderLength <= 6.5 then

      standStillDistance = 12
      minimumDistance = 12

    elseif leaderLength < 4.8 then
      standStillDistance = 13
      minimumDistance = 13
    
    elseif leaderLength > 6.5 then
      standStillDistance = 15
      minimumDistance = 15
    
    else
      standStillDistance = minimumDistance
    end

    if not vid or vid == -1 then
      return
    end

    ai.setTargetObjectID(leaderID)
    ai.setMode('follow')
    ai.driveInLane('on')

    assert(vid >= 0, "platooning.lua - Failed to get a valid vehicle ID")
    local radarArgs = {}
    targetSpeed = speed
    getMass()  --!!!!!!!!!!!!!

    debug = debugFlag
    if debug then
      createCSV()
    end
    ui_message("Vehicle Joined The Platoon", 5, "Tech", "forward")
  else
    ui_message("Relay vehicle size is not compatible to join this platoon", 5, "Tech", "forward")
  end
end

local function updateLeaderToFollow(leaderID)
  

  loaded = true
  mode = "platoon"
  currentMailboxVersionVehicleIndex = obj:getLastMailboxVersion(tostring(vehicleId))
  vehicleIndexInPlatoon = (lpack.decode(obj:getLastMailbox(tostring(vehicleId))))-1 --NFSA
  
  standStillDistance = minimumDistance

  ai.setTargetObjectID(leaderID)
  ai.setMode('follow')
  ai.driveInLane('on')


  assert(leaderID >= 0, "platooning.lua - Failed to get a valid vehicle ID")
  local radarArgs = {}
  
  getMass()  --!!!!!!!!!!!!!

  debug = debugFlag
  if debug then
    createCSV()
  end

  ui_message("Leader of vehicle "..vehicleId.." updated", 5, "Tech", "forward")
end

local function launchPlatoon(leaderID, leaderMode, speed)  -- it launches a platoon wrt a driving mode and a speed input
  
  if leaderMode == 2 then
    mode = 'traffic'
  elseif leaderMode == 1 then
    mode = 'span'
  elseif leaderMode == 0 then
    mode = 'manual'
  end
  launched = true
  vehicleId = leaderID
  updatedSpeed = 3
  platoonsSpeed = speed
  ai.setSpeedMode('limit')
  ai.setSpeed(updatedSpeed)
  ai.setMode(mode)
  ai.driveInLane('on')
  ui_message("Platoon Launched successfully", 5, "Tech", "forward")
end

local function vehicleJoined(vid)
  print("vid: "..vid)
  table.insert(relayVehicles,vid)
end

local function loadWithIDPlatoon(vid, speed, debugFlag)
  loaded = true
  mode = "platoon"
  vehicleId = vid
  if not vid or vid == -1 then
    return
  end
  assert(vid >= 0, "Failed to get a valid vehicle ID")
  local radarArgs = {}
  targetSpeed = speed
  getMass()  -- to check

  debug = debugFlag
  if debug then
    createCSV()
  end
  ui_message("Platoon extension loaded", 5, "Tech", "forward")
end

local function leaderExitPlatoon(leaderID)  -- to set the leader out of the platoon
  electrics.values.throttleOverride = nil
  electrics.values.brakeOverride = nil
  launched = false
  ai.setMode('manual')
  ui_message("Leader Exit Platoon", 5, "Tech", "forward")
end

local function reassignLeader(platoonID, vid)  -- it assigns new leader to the platoon
  loaded = false
  launched = true
  electrics.values.throttleOverride = nil 
  electrics.values.brakeOverride = nil   
  vehicleId = vid
  ai.setSpeedMode('limit')
  ai.setSpeed(10)
  ai.setMode('span')
  ai.driveInLane('on')
  ui_message("Platoon Leader assigned successfully", 5, "Tech", "forward")
end

local function reassignLeaderNFSA(vid)
  loaded = false
  launched = true
  electrics.values.throttleOverride = nil 
  electrics.values.brakeOverride = nil   
  vehicleId = vid
  ai.setSpeedMode('limit')
  ai.setSpeed(10)
  ai.setMode('span')
  ai.driveInLane('on')
  ui_message("Platoon Leader assigned successfully", 5, "Tech", "forward")

end

local function changeLeader(vid) 
  loaded = false
  launched = true
  electrics.values.throttleOverride = nil 
  electrics.values.brakeOverride = nil    
  vehicleId = vid
  ai.setSpeedMode('limit')
  ai.setSpeed(10)
  ai.setMode('span')
  ai.driveInLane('on')

  ui_message("Platoon Leader assigned successfully", 5, "Tech", "forward")
 
end

local function changeLeaderSpeed(leaderID, speed)
  
  ai.setSpeed(speed)
end

local function endPlatoon(vid)
  loaded = false
  electrics.values.throttleOverride = nil
  electrics.values.brakeOverride = nil
  ai.setMode('manual')
  if debug then
    saveCSV()
  end
  ui_message("platooning extension unloaded", 5, "Tech", "forward")
end

-- Public interface.
M.updateGFX = updateGFX
M.loadWithIDPlatoon    = loadWithIDPlatoon
M.changeSpeed          = changeSpeed
M.formPlatoon          = formPlatoon
M.joinPlatoon          = joinPlatoon
M.launchPlatoon        = launchPlatoon
M.endPlatoon           = endPlatoon
M.leavePlatoon         = leavePlatoon
M.reassignLeader       = reassignLeader
M.reassignLeaderNFSA   = reassignLeaderNFSA
M.leaderExitPlatoon    = leaderExitPlatoon
M.updateLeaderToFollow = updateLeaderToFollow
M.getVehicleLength     = getVehicleLength
M.changeLeader         = changeLeader
M.vehicleJoined        = vehicleJoined

return M
