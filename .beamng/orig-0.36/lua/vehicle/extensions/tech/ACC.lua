-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
local M = {}

local printDebugLogs = false

-- Other control parameters.
local maxVehicleRangeSq = 2500.0 -- The maximum squared distance from the player vehicle, at which other vehicles can be detected.
local NaN = 0 / 0

local max, min, abs, sqrt, acos, deg = math.max, math.min, math.abs, math.sqrt, math.acos, math.deg -- for objects data
-- Local road data/other vehicle data.
local lastVelPlayer = vec3(0, 0) -- An initial (default) starting value for the player vehicle velocity.
local lastPos, lastVel = {}, {} -- Initialise tables to store the last-known position and velocity data, for all other vehicles in the simulator.
local lastDt = 0.0 -- The previous time step (used for velocity and acceleration computations).
local vehiclesOldData = {}

-- Player vehicle state.
local pos = vec3(0, 0) -- The player vehicle position.
local fwd, right = vec3(0, 0), vec3(0, 0) -- The player vehicle orthogonal frame.
local vehFront = vec3(0, 0) -- The player vehicle front/rear bumper midpoint positions.
local vel, acc = vec3(0, 0), vec3(0, 0) -- The player vehicle velocity and acceleration vectors.

local nullReading = {
    vehicleID = 0,
    width = 0,
    length = 0,
    positionB = {
        x = 0,
        y = 0,
        z = 0
    },
    distToPlayerVehicleSq = 0,
    relDistX = 0,
    relDistY = 0,
    velBB = {
        x = 0,
        y = 0,
        z = 0
    },
    acc = {
        x = 0,
        y = 0,
        z = 0
    },
    relVelX = 0,
    relVelY = 0,
    relAccX = 0,
    relAccY = 0
}

-- the other vehicles
local simData = {
    closestVehicles1 = nullReading,
    closestVehicles2 = nullReading,
    closestVehicles3 = nullReading,
    closestVehicles4 = nullReading
}

local csvWriter = require('csvlib')

local timer
local csvData

local standStillDistance = 15 -- target distance at v = 0 in m
local deltaTime = 2 -- delta time in s
local prevDistance = 0
local prevDistanceToCars = {}
local prevSpeed = 0
local distanceToLeadCar = 100
local leadingSpeed = 100
local leadingVehicle

local pastU = 0
local mass = 0
local prevLeaderSpeed = 0
local WINDOW_SIZE = 20
local leaderSpeedBuffer = {}

-- local vid
local WIDTH = 100
local HEIGHT = 100
local resolution = {WIDTH, HEIGHT}
local targetSpeed
local targetSpeedVal -- Actual entered speed value for normal cruise
local data
local debug
local vehicleID
local noCars
local mode
local lastUpdateTime = os.clock() -- Initialize the last update time
local maintainSpeedFlag = false
local loaded = false

-- Projects a vector onto another vector.
local function project(a, b)
    return (a:dot(b) / b:dot(b)) * b
end

-- Computes the relative quantity (difference) between two vectors in the direction of a given axis.
local function getRelativeQuantity(v1, v2, axis)
    return project(v2, axis):length() - project(v1, axis):length()
end

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
local function sortAscending(a, b)
    return a.distToPlayerVehicleSq < b.distToPlayerVehicleSq
end

local function createCSV()
    csvData = csvWriter.newCSV("time", "speed", "targetSpeed", "throttle")
    timer = 0
end

local function saveCSV()
    -- save in AppData/Local/BeamNG/<current Version>
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

        for i = j + 1, n do
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

            toggle = -toggle
        end

        local xjj = lum[j][j]

        if xjj ~= 0 then
            for i = j + 1, n do
                local xij = lum[i][j] / xjj
                lum[i][j] = xij
                for k = j + 1, n do
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
            table.insert(adj[i], j, (-1) ^ (i + j) * mDeterminant(M))
        end
    end
    return mMultiplicationScalar(adj, 1 / mDeterminant(A))
end

local function printMatrix(A)
    for i = 1, #A do
        dump(A[i])
    end
end

local function getMass()
    for _, n in pairs(v.data.nodes) do
        mass = mass + n.nodeWeight
    end
end

local function average(t)
    local sum = 0
    for _, v in pairs(t) do
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
    local accFactor = totalForce / mass
    return accFactor
end

-- Koopman MPC
local function computeKmpc()
    local resFactor = -0.0306
    local accFactor = getAccFactor()
    local Ts = 0.5
    local N = 10
    local Q = {{1}}
    local R = 100

    local A = {{1 + resFactor * Ts, Ts * (1 + 1 / 2 * resFactor * Ts) * accFactor}, {0, 1}}
    local B = {{Ts * (1 + 1 / 2 * resFactor * Ts) * accFactor}, {1}}
    local C = {{1, 0}}

    local Phi = {}
    for i = 1, N do
        local A_i = mPower(A, i)
        table.insert(Phi, i * 2 - 1, A_i[1])
        table.insert(Phi, i * 2, A_i[2])
    end

    local Gamma = {}
    for i = 1, N * 2 do
        table.insert(Gamma, i, {})
        for j = 1, N do
            table.insert(Gamma[i], j, 0)
        end
    end

    for i = 2, N do
        for j = 1, (i - 1) do
            local A_i = mPower(A, i - j)
            local Gamma_i = mMultiplication(A_i, B)
            Gamma[i * 2 - 1][j] = Gamma_i[1][1]
            Gamma[i * 2][j] = Gamma_i[2][1]
        end
    end
    for i = 1, N do
        Gamma[i * 2 - 1][i] = B[1][1]
        Gamma[i * 2][i] = B[2][1]
    end

    local Omega = {}
    for i = 1, N * 2 do
        table.insert(Omega, i, {})
        for j = 1, N * 2 do
            table.insert(Omega[i], j, 0)
        end
    end
    local Omega_i = mMultiplication(mMultiplication(mTranspose(C), Q), C)
    for i = 1, N do
        Omega[i * 2 - 1][i * 2 - 1] = Omega_i[1][1]
        Omega[i * 2][i * 2 - 1] = Omega_i[2][1]
        Omega[i * 2 - 1][i * 2] = Omega_i[1][2]
        Omega[i * 2][i * 2] = Omega_i[2][2]
    end

    local Sigma = {}
    for i = 1, N * 2 do
        table.insert(Sigma, i, {})
        for j = 1, N do
            table.insert(Sigma[i], j, 0)
        end
    end
    local Sigma_i = mMultiplication(mTranspose(C), Q)
    for i = 1, N do
        Sigma[i * 2 - 1][i] = Sigma_i[1][1]
        Sigma[i * 2][i] = Sigma_i[2][1]
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

local function calcAvgSpeed(speeds)
    local total = 0
    for _, speed in ipairs(speeds) do
        total = total + speed
    end
    return total / #speeds
end

local function detectSpeedTrend(currentSpeed, targetSpeedIn) -- function for calculating the speed trends
    table.insert(leaderSpeedBuffer, currentSpeed) -- adding new speeds to the buffer and removing the oldest ones

    if #leaderSpeedBuffer > WINDOW_SIZE then
        table.remove(leaderSpeedBuffer, 1)
    end

    local averageSpeed = calcAvgSpeed(leaderSpeedBuffer)
    local threshold = 0.1 -- threshold for detecting the acceleration of decelration
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

-----------------------------------------------------------------


local throttleValue = 0 -- Initialize a persistent throttle value

-- MPCOverride to remove edge case when target vehicle leaves range
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

-----------------------------------------------------------------

-- Main MPC
local function MPC(mode, encodedDistances, targetSpeedIn, inputSpeed, vehicleID, dtSim, debug)
    local targetSpeedIn = math.floor(targetSpeedIn * 10) / 10
    local currentTime = os.clock()

    -- Calculate the time elapsed since the last update
    deltaTime = currentTime - lastUpdateTime

    -- Update the last update time for the next iteration
    lastUpdateTime = currentTime

    local velocityyx = obj:getVelocity().x
    local velocityyy = obj:getVelocity().y
    local velocityyz = obj:getVelocity().z

    -- Velocity in every direction
    local velocityy = math.sqrt(velocityyx ^ 2 + velocityyy ^ 2 + velocityyz ^ 2)

    local currentSpeed = electrics.values.wheelspeed -- speed of the vehicle with the acc
    local speedDifference = math.sqrt((velocityy - targetSpeedIn) ^ 2)
    local distanceToCars = encodedDistances
    -- Ensure targetSpeed is always non-negative
    targetSpeedIn = math.max(0, targetSpeedIn)
    local targetDistance = standStillDistance + deltaTime * currentSpeed -- for testing

    if mode == "acc" then
        local targetDistance = standStillDistance + deltaTime * currentSpeed
    end

    if vehiclesOldData[vehicleID] then
        local observedSpeed = (distanceToCars - vehiclesOldData[vehicleID]) / dtSim + prevSpeed -- changed to variable the form the function call instead of from the for loop on the table
        distanceToLeadCar = distanceToCars
        leadingSpeed = observedSpeed
    end

    local speed2 = leadingSpeed + (distanceToLeadCar - targetDistance) / dtSim -- is negative, maintain same speed, add it with the conditions for acc and dec

    local speed2 = velocityy + (distanceToLeadCar - targetDistance) / dtSim

    targetSpeed = math.min(math.max(speed2, velocityy), targetSpeedIn) -- add .max for -ve speed and the velocity is for the leading vehicle's speed --brakes because of zero
    local distanceError = distanceToLeadCar - targetDistance
    local Kp = 0.1

    -- Setting acceptable error range for ego vehicle
    if distanceToLeadCar > targetDistance * 1.1 then
        maintainSpeedFlag = false
        targetSpeed = targetSpeedIn + distanceError / targetDistance * targetSpeedIn
    elseif distanceToLeadCar < targetDistance * 0.98 then
        targetSpeed = 0
        maintainSpeedFlag = false
    else
        maintainSpeedFlag = true
        targetSpeed = targetSpeedIn
    end

    local Kmpc = computeKmpc()
    -- Change in u
    local deltaU = Kmpc[1] * velocityy + Kmpc[2] * pastU + Kmpc[3] * targetSpeed + Kmpc[4] * targetSpeed + Kmpc[5] *
                       targetSpeed + Kmpc[6] * targetSpeed + Kmpc[7] * targetSpeed
    local u = pastU + deltaU

    -- log("I", "", "Distance to Lead Car: " .. tostring(distanceToLeadCar))
    -- log("I", "", "velocityTest: " .. tostring(velocityy))
    -- log("I", "", "targetSpeed: " .. tostring(targetSpeedIn))

    local velDiff = velocityy - targetSpeedIn

    if targetSpeedIn == 0 then
        targetDistance = standStillDistance
    end

    local leaderSpeedingState = detectSpeedTrend(targetSpeed, targetSpeedIn) -- using the leading speed value in this function

    -- MPC override to prevent reverse behaivior when stopping
    -- if velocityy < 12.0 then
    --     -- log("I", "", "Distance to leadCar: " .. tostring(distanceToLeadCar))
    --     u = (-1 / (distanceToLeadCar / 5))
    --     if velocityy < 0.1 then
    --         electrics.values.parkingbrake = 1
    --         u = 0
    --         -- log("I", "", "Velocity is below threshold: " .. tostring(velocityy))
    --     end
    --     -- log("I", "", "Velocity under limit: " .. tostring(u))
    -- end

    if velocityy > 9.0 then

        if electrics.values.isShifting and velDiff < 0 and distanceToLeadCar - targetDistance > 0 then
            u = adjustThrottle(targetSpeed)

        end

        if maintainSpeedFlag == true then
            u = pastU
        end

        if u > 0 then
            electrics.values.throttleOverride = u
            electrics.values.brakeOverride = nil
        elseif (u == 0 and targetSpeed > 0) then -- added for ego vehicle to move once leader vehicle moves
            electrics.values.throttleOverride = u
            electrics.values.brakeOverride = nil
        elseif (currentSpeed < 0) then
            u = 0
        else
            u = u / 3
            electrics.values.throttleOverride = nil
            electrics.values.brakeOverride = -u -- stopping and pressing breaks
        end

        if debug then
            local time = math.floor(timer * 1000) / 1000 -- Make sure time doesn't have dozen of digits
            csvData:add(time, velocityy, targetSpeed, u)
            timer = timer + dtSim
        end

        pastU = u

        prevSpeed = velocityy

        vehiclesOldData[vehicleID] = distanceToCars
    else
        MPCOverride(currentSpeed)
    end
end

-----------------------------------------------------------------

local function changeSpeed(speed)
    targetSpeed = speed
end

local function updateGFX(dtSim)

    if not loaded then
        return
    end
    -- compute the objects data
    -- Update the player vehicle properties.
    pos = obj:getPosition()
    vel = obj:getVelocity()
    fwd = obj:getForwardVector():normalized()
    right = obj:getDirectionVectorRight():normalized()
    vehFront = obj:getFrontPosition()
    local playerPosToFront = vehFront - pos
    local lastDtInv = 1.0 / max(1e-12, lastDt)
    acc = (vel - lastVelPlayer) * lastDtInv -- Use FD once to get acceleration.
    lastVelPlayer = vel

    -- Compute the relevant properties for the other vehicles.
    local vehicles, ctr = {}, 1
    for k, _ in pairs(mapmgr.getObjects()) do
        if k ~= objectId then
            -- Compute the position, velocity and acceleration of this other vehicle.
            local posB, velB, accB = obj:getObjectCenterPosition(k), vec3(0, 0, 0), vec3(0, 0, 0)
            if lastPos[k] ~= nil then
                velB = (posB - lastPos[k]) * lastDtInv -- Use FD once to get velocity.
            end
            if lastVel[k] ~= nil then
                accB = (velB - lastVel[k]) * lastDtInv -- Use FD twice to get acceleration.
            end
            lastPos[k], lastVel[k] = posB, velB

            -- Store the data of all other vehicles which satisfy the following conditions:
            -- i) within a certain range of the player vehicle.
            -- ii) facing the same direction as the player vehicle.
            -- iii) in front of the player vehicle.
            local fwdB = obj:getObjectDirectionVector(k)
            fwdB:normalize()
            local distantPoint = pos + (1e12 * fwd) -- A point on the player vehicle forward vector, far in the distance.
            if fwd:dot(fwdB) > 0.0 and (distantPoint - pos):lenSquared() > (distantPoint - posB):lenSquared() then -- Test that other vehicle is in front hemisphere and has same dir.
                local distToPlayerVehicleSq = (pos - posB):lenSquared() -- The squared distance between the player and other vehicle.
                if distToPlayerVehicleSq < maxVehicleRangeSq then -- Only consider vehicles which are within the set range.
                    local upB = obj:getObjectDirectionVectorUp(k) -- The other vehicle's frame.
                    upB:normalize()
                    local widthB, lengthB = obj:getObjectInitialWidth(k), obj:getObjectInitialLength(k) -- The other vehicle's dimensions.
                    local frontB = obj:getObjectFrontPosition(k) -- The other vehicle's front/rear bumper midpoint positions.
                    local rearB = frontB - (fwdB * lengthB)
                    local playerPosToBRear = rearB - pos -- The vector from the player position to the other vehicle rear.
                    local relDistX = getRelativeQuantity(playerPosToFront, playerPosToBRear, fwd) -- The relative distance to the player vehicle front position.
                    local relDistY = getRelativeQuantity(playerPosToFront, playerPosToBRear, right)
                    local relVelX, relVelY = getRelativeQuantity(vel, velB, fwd), getRelativeQuantity(vel, velB, right) -- The relative velocity wrt the player vehicle frame.
                    local relAccX, relAccY = getRelativeQuantity(acc, accB, fwd), getRelativeQuantity(acc, accB, right) -- The relative acceleration wrt the player vehicle frame.

                    vehicles[ctr] = {
                        vehicleID = k,
                        width = widthB,
                        length = lengthB,
                        distToPlayerVehicleSq = distToPlayerVehicleSq or 0.0,
                        relDistX = relDistX,
                        relDistY = relDistY,
                        velBB = velB,
                        relVelX = relVelX,
                        relVelY = relVelY,
                        acc = accB,
                        relAccX = relAccX,
                        relAccY = relAccY
                    }
                    ctr = ctr + 1
                end
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
        time = obj:getSimTime(),
        currentSp = currentSpeed,
        targetSp = targetSpeed,
        closestVehicles1 = vClosest1,
        closestVehicles2 = vClosest2,
        closestVehicles3 = vClosest3,
        closestVehicles4 = vClosest4
    }
    -- Cycle the dt values, so we have a recent memory (used in the velocity and acceleration computations).
    lastDt = dtSim

    local distance

    if next(simData) ~= nil then
        for k, v in pairs(simData) do
            if type(v) == "table" then
                if k == "closestVehicles1" then
                    for k2, v2 in pairs(v) do
                        if k2 == "distToPlayerVehicleSq" then
                            if v2 ~= nil and v2 ~= 0 then
                                distance = math.sqrt(v2)
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
                                local sq2 = math.sqrt(x ^ 2 + y ^ 2)

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
    if distance then
        local inputSpeed = 3
        MPC(mode, distance, targetSpeed, inputSpeed, vehicleID, dtSim, debug)
    else
        local currentSpeed = electrics.values.wheelspeed
        if currentSpeed >= 0 then
            currentSpeed = math.floor(currentSpeed + 0.5)
        else
            currentSpeed = math.ceil(currentSpeed - 0.5)
        end
        if printDebugLogs then
            log('I', '', tostring(currentSpeed))
        end
        MPCOverride(currentSpeed + 1)
    end
end

local function loadAccWithID(vid, speed, debugFlag)
    -- local velocity to prevent loading when ego vehicle is too slow
    local velocityyx = obj:getVelocity().x
    local velocityyy = obj:getVelocity().y
    local velocityyz = obj:getVelocity().z

    local velocityy = math.sqrt(velocityyx ^ 2 + velocityyy ^ 2 + velocityyz ^ 2)

    if velocityy > 12.0 then
        loaded = true
    else
        ui_message("Must be atleast 45 kmh", 5, "Tech", "forward")
        return
    end
    mode = "acc"

    if not vid or vid == -1 then
        return
    end
    assert(vid >= 0, "Failed to get a valid vehicle ID")
    targetSpeed = speed
    targetSpeedVal = speed -- Actual entered speed value for normal cruise
    getMass()

    debug = debugFlag
    if debug then
        createCSV()
    end
    ui_message("ACC extension loaded", 5, "Tech", "forward")
end

local function unloadACC()
    loaded = false
    electrics.values.throttleOverride = nil
    electrics.values.brakeOverride = nil
    log('I', 'ACC', 'ACC extension unloaded')
    if debug then
        saveCSV()
    end
    ui_message("ACC extension unloaded", 5, "Tech", "forward")
end

-- Public interface.
M.updateGFX = updateGFX
M.unloadACC = unloadACC
M.changeSpeed = changeSpeed
M.loadAccWithID = loadAccWithID

return M
