-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local logTag = 'cosimulationSignalEditor'

local sensorMgr = require('extensions/tech/sensors')
local vSensors = require('editor/sensorConfigurationEditor')                                        -- The sensor configuration (vehicles) module.
local dat = require('tech/cosimulationNames')
local csvlib = require('csvlib')

-- Module constants.
local im = ui_imgui
local abs, min, max, floor, ceil = math.abs, math.min, math.max, math.floor, math.ceil

-- Module constants (UI).
local names, groups = dat.names, dat.groups                                                         -- The common string values used with cosimulation coupling.
local toolWinName, toolWinSize = 'cosimulationSignalEditor', im.ImVec2(580, 225)                    -- The main tool window of the editor. The main UI entry point.
local signalsWinName, signalsWinSize = 'SignalsWindow', im.ImVec2(750, 600)                         -- The vehicle signals window.
local isSignalsWinOpen = false                                                                      -- A flag which indicates if the vehicle signals window is open or closed.
local dullWhite = im.ImVec4(1, 1, 1, 0.5)                                                           -- Some commonly-used Imgui colour vectors.
local redB, redD = im.ImVec4(0.7, 0.5, 0.5, 1), im.ImVec4(0.7, 0.5, 0.5, 0.5)
local greenB, greenD = im.ImVec4(0.5, 0.7, 0.5, 1), im.ImVec4(0.5, 0.7, 0.5, 0.5)
local blueB, blueD = im.ImVec4(0.5, 0.5, 0.7, 1), im.ImVec4(0.5, 0.5, 0.7, 0.5)
local orangeB = im.ImVec4(184/255, 127/255, 75/255, 1)

-- Module state (back-end).
local vehicles = {}                                                                                 -- An ordered list of all vehicles currently in the scene.
local signals = {}                                                                                  -- An ordered list of available vehicle signals.
local cData = {}                                                                                    -- A table containing the collected data from vlua.
local selectedVehicleIdx = 1                                                                        -- The index of the selected vehicle, in the vehicles list.
local isCosimulationSignalEditor = false                                                            -- A flag which indicates if this editor is currently active.
local isExecuting = false                                                                           -- A flag which indicates if the coupling is currently being executed.
local isVluaDataReturned = false                                                                    -- A flag which indicates if requested vlua data has returned to ge lua.
local isRequestSent = false                                                                         -- A flag which indicates if a request has been sent to vlua.

-- Module state (front-end).
local compTime3rdParty = im.FloatPtr(0.0005)                                                        -- The expected 3rd party computation time (per cycle).
local pingTime = im.FloatPtr(0.00001)                                                               -- The expected udp socket ping time.
local sIP, rIP = im.ArrayChar(16, "127.0.0.1"), im.ArrayChar(16, "127.0.0.1")                       -- The IP addresses for the udp communication (3rd party computer).
local sPort, rPort = im.IntPtr(64890), im.IntPtr(64891)                                             -- The port numbers for the udp communication.



-- ImGui Pointer Variables
local isKinematics, isDriver, isWheels, isVSL, isCosim = im.BoolPtr(true), im.BoolPtr(true), im.BoolPtr(true), im.BoolPtr(false), im.BoolPtr(false)
-- These are pointers (`im.BoolPtr`) that ImGui uses to modify the values by reference, useful for checkbox or UI elements.


-- TODO: Add frequency option
-- Logging Frequency
-- local loggingFrequency = im.IntPtr(5)  -- Frequency for logging, set as an integer pointer

-- Separate Flags for Tracking State
local isVSLEnabled = false  -- Boolean flag to manage the state of VSL (Logging)
local isCosimEnabled = false  -- Boolean flag to manage the state of Co-Simulation

-- Enable/Disable Flags
local enableVSL = false  -- Boolean flag to determine whether logging is enabled or disabled
local enableCosim = true  -- Boolean flag to determine whether co-simulation is enabled or disabled

-- Previous State Tracking
local prevVSLState = false  -- Boolean flag to store the previous state of VSL for comparison


local isElectrics, isPowertrain, isSensors = im.BoolPtr(true), im.BoolPtr(true), im.BoolPtr(true)
local isPose = im.BoolPtr(false)                                                                    -- A flag which indicates whether to store the vehicle pose, or not.


-- Compute the vehicle space position of a sensor, given the local reference frame coefficients.
local function coeffs2PosVS(c, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return c.x * fwd + c.y * right + c.z * up
end

-- Compute the vehicle space/world space frame of a sensor, given the local reference frame.
local function sensor2VS(dirLoc, upLoc, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return vec3(fwd:dot(dirLoc), right:dot(dirLoc), up:dot(dirLoc)), vec3(fwd:dot(upLoc), right:dot(upLoc), up:dot(upLoc))
end

-- The callback function for use when collecting vehicle data from vlua.
local function updateCollectedVehicleData(collectedData)
  cData, isVluaDataReturned = lpack.decode(collectedData), true
end

-- Populate the current vehicles list.
local function getCurrentVehicleList()
  table.clear(vehicles)
  local ctr = 1
  for vid, veh in activeVehiclesIterator() do
    vehicles[ctr] = {
      vid = vid, veh = veh, name = veh:getName(),
      jBeam = veh.JBeam, config = veh:getField('partConfig', '0')}
    ctr = ctr + 1
  end
end

-- Populates the current available signals list.
local function updateSignalsList()

  -- Dispatch a request to vlua, to collect all the relevant possible signals.
  if not isRequestSent then
    table.clear(cData)
    isRequestSent, isVluaDataReturned = true, false
    local vid = vehicles[selectedVehicleIdx].vid
    be:queueObjectLua(vid, "extensions.tech_vehicleSearcher.collectVehicleData()")
  end

  -- Do not go any further until the requested data has been returned from vlua.
  if not isVluaDataReturned then
    return false
  end
  isRequestSent, isVluaDataReturned = false, false
  local vid = vehicles[selectedVehicleIdx].vid

  -----------------------
  -- Kinematics Group:
  -----------------------
  if isKinematics[0] then

    -- Vehicle position.
    signals[#signals + 1] = {
      name = names.vehiclePositionX, groupName = groups.kinematics, description = 'Vehicle position - Lateral - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }
    signals[#signals + 1] = {
      name = names.vehiclePositionY, groupName = groups.kinematics, description = 'Vehicle position - Longitudinal - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }
    signals[#signals + 1] = {
      name = names.vehiclePositionZ, groupName = groups.kinematics, description = 'Vehicle position - Vertical - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }

    -- Vehicle velocity.
    signals[#signals + 1] = {
      name = names.vehicleVelocityX, groupName = groups.kinematics, description = 'Vehicle velocity - Lateral - m/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }
    signals[#signals + 1] = {
      name = names.vehicleVelocityY, groupName = groups.kinematics, description = 'Vehicle velocity - Longitudinal - m/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }
    signals[#signals + 1] = {
      name = names.vehicleVelocityZ, groupName = groups.kinematics, description = 'Vehicle velocity - Vertical - m/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = true }

    -- Vehicle acceleration.
    signals[#signals + 1] = {
      name = names.vehicleAccelerationX, groupName = groups.kinematics, description = 'Vehicle acceleration - Lateral - ms^-2',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleAccelerationY, groupName = groups.kinematics, description = 'Vehicle acceleration - Longitudinal - ms^-2',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleAccelerationZ, groupName = groups.kinematics, description = 'Vehicle acceleration - Vertical - ms^-2',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle roll/pitch/yaw.
    signals[#signals + 1] = {
      name = names.vehicleRoll, groupName = groups.kinematics, description = 'Roll angle - rad',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehiclePitch, groupName = groups.kinematics, description = 'Pitch angle - rad',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleYaw, groupName = groups.kinematics, description = 'Yaw angle - rad',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle roll/pitch/yaw rate.
    signals[#signals + 1] = {
      name = names.vehicleRollRate, groupName = groups.kinematics, description = 'Roll rate - rad/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehiclePitchRate, groupName = groups.kinematics, description = 'Pitch rate - rad/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleYawRate, groupName = groups.kinematics, description = 'Yaw rate - rad/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Ground speed.
    signals[#signals + 1] = {
      name = names.vehicleGroundSpeed, groupName = groups.kinematics, description = 'Vehicle ground speed - m/s',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Altitude.
    signals[#signals + 1] = {
      name = names.vehicleAltitude, groupName = groups.kinematics, description = 'Vehicle altitude - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle local orthonormal frame.
    signals[#signals + 1] = {
      name = names.vehicleForwardX, groupName = groups.kinematics, description = 'Unit forward vector - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleForwardY, groupName = groups.kinematics, description = 'Unit forward vector - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleForwardZ, groupName = groups.kinematics, description = 'Unit forward vector - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleUpX, groupName = groups.kinematics, description = 'Unit up vector - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleUpY, groupName = groups.kinematics, description = 'Unit up vector - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleUpZ, groupName = groups.kinematics, description = 'Unit up vector - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRightX, groupName = groups.kinematics, description = 'Unit right vector - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRightY, groupName = groups.kinematics, description = 'Unit right vector - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRightZ, groupName = groups.kinematics, description = 'Unit right vector - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle length/width/height (initial values).
    signals[#signals + 1] = {
      name = names.vehicleInitialLength, groupName = groups.kinematics, description = 'Initial vehicle length - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleInitialWidth, groupName = groups.kinematics, description = 'Initial vehicle width - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleInitialHeight, groupName = groups.kinematics, description = 'Initial vehicle height - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle Center-of-Gravity (COG) [with and without wheels included].
    signals[#signals + 1] = {
      name = names.vehicleCOGWithWheelsX, groupName = groups.kinematics, description = 'COG (inc. wheels) - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleCOGWithWheelsY, groupName = groups.kinematics, description = 'COG (inc. wheels) - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleCOGWithWheelsZ, groupName = groups.kinematics, description = 'COG (inc. wheels) - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleCOGWithoutWheelsX, groupName = groups.kinematics, description = 'COG (not inc. wheels) - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleCOGWithoutWheelsY, groupName = groups.kinematics, description = 'COG (not inc. wheels) - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleCOGWithoutWheelsZ, groupName = groups.kinematics, description = 'COG (not inc. wheels) - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle mid-front-bumper and mid-rear-bumper positions.
    signals[#signals + 1] = {
      name = names.vehicleMidFrontBumperX, groupName = groups.kinematics, description = 'Front bumper midpoint - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleMidFrontBumperY, groupName = groups.kinematics, description = 'Front bumper midpoint - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleMidFrontBumperZ, groupName = groups.kinematics, description = 'Front bumper midpoint - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleMidRearBumperX, groupName = groups.kinematics, description = 'Rear bumper midpoint - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleMidRearBumperY, groupName = groups.kinematics, description = 'Rear bumper midpoint - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleMidRearBumperZ, groupName = groups.kinematics, description = 'Rear bumper midpoint - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Vehicle front-axle-midpoint and rear-axle-midpoint positions.
    signals[#signals + 1] = {
      name = names.vehicleFrontAxleMidpointX, groupName = groups.kinematics, description = 'Front axle midpoint - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleFrontAxleMidpointY, groupName = groups.kinematics, description = 'Front axle midpoint - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleFrontAxleMidpointZ, groupName = groups.kinematics, description = 'Front axle midpoint - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRearAxleMidpointX, groupName = groups.kinematics, description = 'Rear axle midpoint - Lat - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRearAxleMidpointY, groupName = groups.kinematics, description = 'Rear axle midpoint - Long - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
    signals[#signals + 1] = {
      name = names.vehicleRearAxleMidpointZ, groupName = groups.kinematics, description = 'Rear axle midpoint - Vert - meters',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
  end

  -----------------------
  -- Driver Control Group:
  -----------------------

  if isDriver[0] then

    -- Throttle pedal.
    signals[#signals + 1] = {
      name = names.throttle, groupName = groups.driver, description = 'Throttle pedal - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = true, readOnly = false, isIncluded = true }
    signals[#signals + 1] = {
      name = names.throttleInput, groupName = groups.driver, description = 'Throttle pedal input value - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Brake pedal.
    signals[#signals + 1] = {
      name = names.brake, groupName = groups.driver, description = 'Brake pedal - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = true, readOnly = false, isIncluded = true }
    signals[#signals + 1] = {
      name = names.brakeInput, groupName = groups.driver, description = 'Brake pedal input value - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Clutch pedal.
    signals[#signals + 1] = {
      name = names.clutch, groupName = groups.driver, description = 'Clutch pedal - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = true, readOnly = false, isIncluded = true }
    signals[#signals + 1] = {
      name = names.clutchInput, groupName = groups.driver, description = 'Clutch pedal input value - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Parking brake.
    signals[#signals + 1] = {
      name = names.parkingBrake, groupName = groups.driver, description = 'Parking brake - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = true, readOnly = false, isIncluded = true }
    signals[#signals + 1] = {
      name = names.parkingBrakeInput, groupName = groups.driver, description = 'Parking brake input value - range [0..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }

    -- Steering wheel.
    signals[#signals + 1] = {
      name = names.steeringWheelPosition, groupName = groups.driver, description = 'Steering wheel position - range [-1..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = true, readOnly = false, isIncluded = true }
    signals[#signals + 1] = {
      name = names.steeringWheelPositionInput, groupName = groups.driver, description = 'Steering wheel input value - range [-1..1]',
      type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
      isFrom = false, readOnly = true, isIncluded = false }
  end

  -----------------------
  -- Wheels Group:
  -----------------------

  if isWheels[0] then
    local wheelData = cData.wheels
    local numWheels = #wheelData
    for i = 1, numWheels do
      local wId = tostring(wheelData[i])
      signals[#signals + 1] = {
        name = names.wheelSpeed .. wId, groupName = groups.wheels, description = wId .. ' - Wheel speed - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.angularVelocity .. wId, groupName = groups.wheels, description = wId .. ' - Angular velocity - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.downforce .. wId, groupName = groups.wheels, description = wId .. ' - Downforce - N-m',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.brakingTorque .. wId, groupName = groups.wheels, description = wId .. ' - Braking torque - N-m',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = false, isIncluded = true }
      signals[#signals + 1] = {
        name = names.propulsionTorque .. wId, groupName = groups.wheels, description = wId .. ' - Propulsion torque - N-m',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = false, isIncluded = true }
      signals[#signals + 1] = {
        name = names.frictionTorque .. wId, groupName = groups.wheels, description = wId .. ' - Friction torque - N-m',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = false, isIncluded = false }
      signals[#signals + 1] = {
        name = names.wheelAngle .. wId, groupName = groups.wheels, description = wId .. ' - Wheel angle - rad',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end
  end

  -----------------------
  -- Electrics Group:
  -----------------------
  if isElectrics[0] then
    local elecData = cData.electrics
    local numElec = #elecData
    for i = 1, numElec do
      signals[#signals + 1] = {
        name = elecData[i].name, groupName = groups.electrics, description = elecData[i].name,
        type = elecData[i].type, isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end
  end

  -----------------------
  -- Powertrain Group:
  -----------------------
  if isPowertrain[0] then
    local pTData = cData.powertrain
    local numPT = #pTData
    for i = 1, numPT do
      signals[#signals + 1] = {
        name = pTData[i].name, groupName = groups.powertrain, description = pTData[i].name,
        type = pTData[i].type, isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end
  end

  -----------------------
  -- IMU Sensors Group(s):
  -----------------------
  if isSensors[0] then
    local sensors = vSensors.sensorConfigs[vid]
    local numIMU, IMUids = vSensors.numberOfSensorType(sensors, 'IMU')
    for i = 1, numIMU do
      local sensor = sensors[IMUids[i]]
      local name = sensor.name
      signals[#signals + 1] = {
        name = names.imuPositionX, groupName = name, description = 'Position - Lateral - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuPositionY, groupName = name, description = 'Position - Longitudinal - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuPositionZ, groupName = name, description = 'Position - Vertical - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis1DirectionX, groupName = name, description = 'Axis 1 direction - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis1DirectionY, groupName = name, description = 'Axis 1 direction - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis1DirectionZ, groupName = name, description = 'Axis 1 direction - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis2DirectionX, groupName = name, description = 'Axis 2 direction - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis2DirectionY, groupName = name, description = 'Axis 2 direction - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis2DirectionZ, groupName = name, description = 'Axis 2 direction - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis3DirectionX, groupName = name, description = 'Axis 3 direction - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis3DirectionY, groupName = name, description = 'Axis 3 direction - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAxis3DirectionZ, groupName = name, description = 'Axis 3 direction - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuMass, groupName = name, description = 'Mass at sensor position - kg',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocityRawAxis1, groupName = name, description = 'Angular velocity raw - Lat - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocityRawAxis2, groupName = name, description = 'Angular velocity raw - Long - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocityRawAxis3, groupName = name, description = 'Angular velocity raw - Vert - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocitySmoothedAxis1, groupName = name, description = 'Angular velocity smoothed - Lat - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocitySmoothedAxis2, groupName = name, description = 'Angular velocity smoothed - Long - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularVelocitySmoothedAxis3, groupName = name, description = 'Angular velocity smoothed - Vert - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationRawAxis1, groupName = name, description = 'Acceleration raw - Lat - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationRawAxis2, groupName = name, description = 'Acceleration raw - Long - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationRawAxis3, groupName = name, description = 'Acceleration raw - Vert - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationSmoothedAxis1, groupName = name, description = 'Acceleration smoothed - Lat - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationSmoothedAxis2, groupName = name, description = 'Acceleration smoothed - Long - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAccelerationSmoothedAxis3, groupName = name, description = 'Acceleration smoothed - Vert - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularAccelerationAxis1, groupName = name, description = 'Angular acceleration - Lat - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularAccelerationAxis2, groupName = name, description = 'Angular acceleration - Long - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuAngularAccelerationAxis3, groupName = name, description = 'Angular acceleration - Vert - rad/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.imuReadingTimestamp, groupName = name, description = 'IMU Reading timestamp - seconds',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end

    -----------------------
    -- GPS Sensors Group(s):
    -----------------------
    local numGPS, GPSids = vSensors.numberOfSensorType(sensors, 'GPS')
    for i = 1, numGPS do
      local sensor = sensors[GPSids[i]]
      local name = sensor.name
      signals[#signals + 1] = {
        name = names.gpsXCoordinate, groupName = name, description = 'Lateral Pos - world-space - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.gpsYCoordinate, groupName = name, description = 'Longitudinal Pos - world-space - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.gpsLongitude, groupName = name, description = 'Longitude - degrees',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.gpsLatitude, groupName = name, description = 'Latitude - degrees',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.gpsReadingTimestamp, groupName = name, description = 'GPS Reading timestamp - seconds',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end

    -----------------------
    -- Ideal Radar Group:
    -----------------------
    if vSensors.doesContainSensorType(sensors, 'idealRADAR') then
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1Distance, groupName = groups.idealRADAR, description = 'Vehicle #1 - distance to - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1Length, groupName = groups.idealRADAR, description = 'Vehicle #1 - length - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1Width, groupName = groups.idealRADAR, description = 'Vehicle #1 - width - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1VelocityX, groupName = groups.idealRADAR, description = 'Vehicle #1 - velocity - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1VelocityY, groupName = groups.idealRADAR, description = 'Vehicle #1 - velocity - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1VelocityZ, groupName = groups.idealRADAR, description = 'Vehicle #1 - velocity - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1AccelerationX, groupName = groups.idealRADAR, description = 'Vehicle #1 - acceleration - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1AccelerationY, groupName = groups.idealRADAR, description = 'Vehicle #1 - acceleration - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1AccelerationZ, groupName = groups.idealRADAR, description = 'Vehicle #1 - acceleration - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeDistanceX, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative dist - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeDistanceY, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative dist - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeVelocityX, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative vel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeVelocityY, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative vel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeAccelerationX, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative accel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle1RelativeAccelerationY, groupName = groups.idealRADAR, description = 'Vehicle #1 - relative accel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }

      signals[#signals + 1] = {
        name = names.idealRADARVehicle2Distance, groupName = groups.idealRADAR, description = 'Vehicle #2 - distance to - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2Length, groupName = groups.idealRADAR, description = 'Vehicle #2 - length - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2Width, groupName = groups.idealRADAR, description = 'Vehicle #2 - width - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2VelocityX, groupName = groups.idealRADAR, description = 'Vehicle #2 - velocity - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2VelocityY, groupName = groups.idealRADAR, description = 'Vehicle #2 - velocity - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2VelocityZ, groupName = groups.idealRADAR, description = 'Vehicle #2 - velocity - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2AccelerationX, groupName = groups.idealRADAR, description = 'Vehicle #2 - acceleration - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2AccelerationY, groupName = groups.idealRADAR, description = 'Vehicle #2 - acceleration - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2AccelerationZ, groupName = groups.idealRADAR, description = 'Vehicle #2 - acceleration - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeDistanceX, groupName = groups.idealRADAR, description = 'Vehicle #2 - relative dist - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeDistanceY, groupName = groups.idealRADAR, description = 'Vehicle #2 - relative dist - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeVelocityX, groupName = groups.idealRADAR, description = 'Vehicle #2 - relative vel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeVelocityY, groupName = groups.idealRADAR, description = 'Vehicle #2 - relative vel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeAccelerationX, groupName = groups.idealRADAR,  description = 'Vehicle #2 - relative accel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle2RelativeAccelerationY, groupName = groups.idealRADAR,  description = 'Vehicle #2 - relative accel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }

      signals[#signals + 1] = {
        name = names.idealRADARVehicle3Distance, groupName = groups.idealRADAR, description = 'Vehicle #3 - distance to - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3Length, groupName = groups.idealRADAR, description = 'Vehicle #3 - length - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3Width, groupName = groups.idealRADAR, description = 'Vehicle #3 - width - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3VelocityX, groupName = groups.idealRADAR, description = 'Vehicle #3 - velocity - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3VelocityY, groupName = groups.idealRADAR, description = 'Vehicle #3 - velocity - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3VelocityZ, groupName = groups.idealRADAR, description = 'Vehicle #3 - velocity - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3AccelerationX, groupName = groups.idealRADAR, description = 'Vehicle #3 - acceleration - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3AccelerationY, groupName = groups.idealRADAR, description = 'Vehicle #3 - acceleration - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3AccelerationZ, groupName = groups.idealRADAR, description = 'Vehicle #3 - acceleration - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeDistanceX, groupName = groups.idealRADAR, description = 'Vehicle #3 - relative dist - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeDistanceY, groupName = groups.idealRADAR, description = 'Vehicle #3 - relative dist - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeVelocityX, groupName = groups.idealRADAR, description = 'Vehicle #3 - relative vel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeVelocityY, groupName = groups.idealRADAR, description = 'Vehicle #3 - relative vel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeAccelerationX, groupName = groups.idealRADAR,  description = 'Vehicle #3 - relative accel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle3RelativeAccelerationY, groupName = groups.idealRADAR,  description = 'Vehicle #3 - relative accel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }

      signals[#signals + 1] = {
        name = names.idealRADARVehicle4Distance, groupName = groups.idealRADAR, description = 'Vehicle #4 - distance to - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4Length, groupName = groups.idealRADAR, description = 'Vehicle #4 - length - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4Width, groupName = groups.idealRADAR, description = 'Vehicle #4 - width - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4VelocityX, groupName = groups.idealRADAR, description = 'Vehicle #4 - velocity - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4VelocityY, groupName = groups.idealRADAR, description = 'Vehicle #4 - velocity - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4VelocityZ, groupName = groups.idealRADAR, description = 'Vehicle #4 - velocity - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4AccelerationX, groupName = groups.idealRADAR, description = 'Vehicle #4 - acceleration - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4AccelerationY, groupName = groups.idealRADAR, description = 'Vehicle #4 - acceleration - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4AccelerationZ, groupName = groups.idealRADAR, description = 'Vehicle #4 - acceleration - Vert - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeDistanceX, groupName = groups.idealRADAR,
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false, description = 'Vehicle #4 - relative dist - Lat - m/s',
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeDistanceY, groupName = groups.idealRADAR,
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false, description = 'Vehicle #4 - relative dist - Long - m/s',
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeVelocityX, groupName = groups.idealRADAR, description = 'Vehicle #4 - relative vel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeVelocityY, groupName = groups.idealRADAR, description = 'Vehicle #4 - relative vel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeAccelerationX, groupName = groups.idealRADAR,  description = 'Vehicle #4 - relative accel - Lat - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.idealRADARVehicle4RelativeAccelerationY, groupName = groups.idealRADAR,  description = 'Vehicle #4 - relative accel - Long - m/s',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }

      signals[#signals + 1] = {
        name = names.idealRADARReadingTimestamp, groupName = groups.idealRADAR, description = 'Reading timestamp',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end

    -------------------------
    -- Roads Sensor Group:
    -------------------------
    if vSensors.doesContainSensorType(sensors, 'roads') then
      signals[#signals + 1] = {
        name = names.roadsRoadHalfWidth, groupName = groups.roadsSensor, description = 'Local road half-width - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsRoadRadius, groupName = groups.roadsSensor, description = 'Local road radius - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsRoadHeading, groupName = groups.roadsSensor, description = 'Local road heading - rad',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsDistanceToCenterline, groupName = groups.roadsSensor, description = 'Distance to road centerline - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsDistanceToRoadLeftEdge, groupName = groups.roadsSensor, description = 'Distance to road left edge - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsDistanceToRoadRightEdge, groupName = groups.roadsSensor, description = 'Distance to road right edge - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsDrivability, groupName = groups.roadsSensor, description = 'Road drivability score',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsSpeedLimit, groupName = groups.roadsSensor, description = 'Road speed limit',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsIsOneWay, groupName = groups.roadsSensor, description = 'Is one-way road',
        type = 'boolean', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsClosestPointX, groupName = groups.roadsSensor, description = 'Closest road point - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsClosestPointY, groupName = groups.roadsSensor, description = 'Closest road point - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsClosestPointZ, groupName = groups.roadsSensor, description = 'Closest road point - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads2ndClosestPointX, groupName = groups.roadsSensor, description = '2nd closest road point - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads2ndClosestPointY, groupName = groups.roadsSensor, description = '2nd closest road point - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads2ndClosestPointZ, groupName = groups.roadsSensor, description = '2nd closest road point - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads3rdClosestPointX, groupName = groups.roadsSensor, description = '3rd closest road point - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads3rdClosestPointY, groupName = groups.roadsSensor, description = '3rd closest road point - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads3rdClosestPointZ, groupName = groups.roadsSensor, description = '3rd closest road point - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads4thClosestPointX, groupName = groups.roadsSensor, description = '4th closest road point - Lat - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads4thClosestPointY, groupName = groups.roadsSensor, description = '4th closest road point - Long - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roads4thClosestPointZ, groupName = groups.roadsSensor, description = '4th closest road point - Vert - meters',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
      signals[#signals + 1] = {
        name = names.roadsReadingTimestamp, groupName = groups.roadsSensor, description = 'Reading timestamp - seconds',
        type = 'number', isMultiply = false, isAdd = false, isFreeze = false,
        isFrom = false, readOnly = true, isIncluded = false }
    end
  end
  return true
end

-- Unlink all signals (reset configuration).
local function unlinkAllSignals()
  local numSignals = #signals
  for i = 1, numSignals do
    signals[i].isIncluded = false
  end
end

-- Fetches two ordered arrays containing the included 'to' and 'from' signals, respectively.
local function getToFromSignals()
  local to, from, tCtr, fCtr, numSignals = {}, {}, 1, 1, #signals
  for i = 1, numSignals do
    local sig = signals[i]
    if sig.isIncluded then
      if sig.isFrom then
        from[fCtr] = sig
        fCtr = fCtr + 1
      else
        to[tCtr] = sig
        tCtr = tCtr + 1
      end
    end
  end
  return to, from
end

-- Saves the current signals configuration state to file.
local function saveConfiguration(vehicle)
  extensions.editor_fileDialog.saveFile(
    function(data)
      local h = dat.headers
      local csv = csvlib.newCSV(h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8])

      -- Write the 'to' signal list.
      -- [The mode is not applicable to outgoing signals - the 3rd party can handle them any way it chooses there].
      local numSig = #signals
      for i = 1, numSig do
        local s = signals[i]
        if s.isIncluded then
          if not s.isFrom then
            csv:add('signalTo', s.groupName, s.name, nil, s.type, s.description, nil, nil)
          end
        end
      end

       -- Write the 'from' signal list, including all requested channels for every included signal.
       for i = 1, numSig do
        local s = signals[i]
        if s.isIncluded then
          if s.isFrom then
            csv:add('signalFrom', s.groupName, s.name, 'value', s.type, s.description, nil, nil)
            if s.isMultiply then csv:add('signalFrom', s.groupName, s.name, 'multiply', s.type, s.description, nil, nil) end
            if s.isAdd then csv:add('signalFrom', s.groupName, s.name, 'add', s.type, s.description, nil, nil) end
            if s.isFreeze then csv:add('signalFrom', s.groupName, s.name, 'freeze', 'boolean', s.description, nil, nil) end
          end
        end
      end

      -- Write the vehicle identifier and pose info.
      local vehicle = vehicles[selectedVehicleIdx]
      csv:add('vehicle', nil, 'Vehicle Model', vehicle.jBeam, "string", nil, nil, nil)
      csv:add('vehicle', nil, 'Vehicle Config', vehicle.config, "string", nil, nil, nil)
      if isPose[0] then
        local veh = vehicle.veh
        local pos, rot = veh:getPosition(), quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp())
        csv:add('vehicle', nil, 'posX', tostring(pos.x), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'posY', tostring(pos.y), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'posZ', tostring(pos.z), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'rotX', tostring(rot.x), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'rotY', tostring(rot.y), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'rotZ', tostring(rot.z), "number", nil, nil, nil)
        csv:add('vehicle', nil, 'rotW', tostring(rot.w), "number", nil, nil, nil)
      end

      -- Write the connection/socket info.
      csv:add('connection', nil, '3rd Party Computation Time', tostring(compTime3rdParty[0]), 'number', nil, nil, nil)
      csv:add('connection', nil, 'Ping Time', tostring(pingTime[0]), 'number', nil, nil, nil)
      csv:add('connection', 'otherUDP', 'IP', ffi.string(sIP), 'string', nil, nil, nil)
      csv:add('connection', 'otherUDP', 'port', tostring(sPort[0]), 'number', nil, nil, nil)
      csv:add('connection', 'beamngUDP', 'IP', ffi.string(rIP), 'string', nil, nil, nil)
      csv:add('connection', 'beamngUDP', 'port', tostring(rPort[0]), 'number', nil, nil, nil)

      -- Write the sensor info.
      local vid = vehicle.vid
      local sensors = vSensors.sensorConfigs[vid]
      local numIMU, IMUids = vSensors.numberOfSensorType(sensors, 'IMU')
      for i = 1, numIMU do
        local sensor = sensors[IMUids[i]]
        local name = sensor.name
        csv:add('sensors', name, 'posX', tostring(sensor.pos.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'posY', tostring(sensor.pos.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'posZ', tostring(sensor.pos.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirX', tostring(sensor.dir.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirY', tostring(sensor.dir.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirZ', tostring(sensor.dir.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upX', tostring(sensor.up.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upY', tostring(sensor.up.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upZ', tostring(sensor.up.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'physicsUpdateTime', tostring(sensor.physicsUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'GFXUpdateTime', tostring(sensor.GFXUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'isUsingGravity', tostring(sensor.isUsingGravity), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isAllowWheelNodes', tostring(sensor.isAllowWheelNodes), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'accelWindowWidth', tostring(sensor.accelWindowWidth), 'number', nil, nil, nil)
        csv:add('sensors', name, 'gyroWindowWidth', tostring(sensor.gyroWindowWidth), 'number', nil, nil, nil)
        csv:add('sensors', name, 'isVisualised', tostring(sensor.isVisualised), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isStatic', tostring(sensor.isStatic), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isSnappingDesired', tostring(sensor.isSnappingDesired), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isForceInsideTriangle', tostring(sensor.isForceInsideTriangle), 'boolean', nil, nil, nil)
      end
      local numGPS, GPSids = vSensors.numberOfSensorType(sensors, 'GPS')
      for i = 1, numGPS do
        local sensor = sensors[GPSids[i]]
        local name = sensor.name
        csv:add('sensors', name, 'posX', tostring(sensor.pos.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'posY', tostring(sensor.pos.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'posZ', tostring(sensor.pos.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirX', tostring(sensor.dir.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirY', tostring(sensor.dir.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'dirZ', tostring(sensor.dir.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upX', tostring(sensor.up.x), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upY', tostring(sensor.up.y), 'number', nil, nil, nil)
        csv:add('sensors', name, 'upZ', tostring(sensor.up.z), 'number', nil, nil, nil)
        csv:add('sensors', name, 'physicsUpdateTime', tostring(sensor.physicsUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'GFXUpdateTime', tostring(sensor.GFXUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'isAllowWheelNodes', tostring(sensor.isAllowWheelNodes), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'refLon', tostring(sensor.refLon), 'number', nil, nil, nil)
        csv:add('sensors', name, 'refLat', tostring(sensor.refLat), 'number', nil, nil, nil)
        csv:add('sensors', name, 'isVisualised', tostring(sensor.isVisualised), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isStatic', tostring(sensor.isStatic), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isSnappingDesired', tostring(sensor.isSnappingDesired), 'boolean', nil, nil, nil)
        csv:add('sensors', name, 'isForceInsideTriangle', tostring(sensor.isForceInsideTriangle), 'boolean', nil, nil, nil)
      end
      local numIR, IRids = vSensors.numberOfSensorType(sensors, 'idealRADAR')
      for i = 1, numIR do
        local sensor = sensors[IRids[i]]
        local name = sensor.name
        csv:add('sensors', name, 'physicsUpdateTime', tostring(sensor.physicsUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'GFXUpdateTime', tostring(sensor.GFXUpdateTime), 'number', nil, nil, nil)
      end
      local numRS, RSids = vSensors.numberOfSensorType(sensors, 'roads')
      for i = 1, numRS do
        local sensor = sensors[RSids[i]]
        local name = sensor.name
        csv:add('sensors', name, 'physicsUpdateTime', tostring(sensor.physicsUpdateTime), 'number', nil, nil, nil)
        csv:add('sensors', name, 'GFXUpdateTime', tostring(sensor.GFXUpdateTime), 'number', nil, nil, nil)
      end

      csv:write(data.filepath)
    end,
    {{"csv",".csv"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a signals configuration state from file, if appropriate.
local function loadConfiguration(vehicle)
  extensions.editor_fileDialog.openFile(
    function(data)

      -- Get the original signals list and remove all linking.
      isRequestSent = false
      updateSignalsList()
      unlinkAllSignals()

      -- Read the .csv file into a lines structure.
      local csv = csvlib.readFileCSV(data.filepath)

      -- Collect all the 'To' signals, and set them in the vehicle signals array.
      local numLines, numSignals = #csv, #signals
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'signalTo' then
          for j = 1, numSignals do
            local s = signals[j]
            if d[2] == s.groupName and d[3] == s.name then
              s.isIncluded, s.isFrom = true, false
              break
            end
          end
        end
      end

      -- Collect all the 'From' signals, and set them in the vehicle signals array.
      local fromGroup = {}
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'signalFrom' then
          local name = d[3]
          if not fromGroup[name] then
            fromGroup[name] = { type = d[5], groupName = d[2], name = name, isMultiply = false, isAdd = false, isFreeze = false }
          end
          local mode = d[4]
          if mode == 'multiply' then fromGroup[name].isMultiply = true end
          if mode == 'add' then fromGroup[name].isAdd = true end
          if mode == 'freeze' then fromGroup[name].isFreeze = true end
        end
      end
      for name, c in pairs(fromGroup) do
        for j = 1, numSignals do
          local s = signals[j]
          if c.groupName == s.groupName and name == s.name then
            s.isIncluded, s.isFrom = true, true
            s.isMultiply, s.isAdd, s.isFreeze = c.isMultiply, c.isAdd, c.isFreeze
            break
          end
        end
      end

      -- Check the vehicle against the vehicle in the .csv file, and issue a warning if it is different.
      local jBeam, config = vehicle.jBeam, vehicle.config
      local posX, posY, posZ, rotX, rotY, rotZ, rotW = nil, nil, nil, nil, nil, nil, nil
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'vehicle' then
          if d[3] == 'Vehicle Model' and d[4] ~= jBeam then
            log('W', logTag, 'Vehicle (jBeam) in .csv different to currently-selected vehicle!')
            break
          end
          if d[3] == 'Vehicle Config' and d[4] ~= config then
            log('W', logTag, 'Vehicle (config) in .csv different to currently-selected vehicle!')
            break
          end
          if d[3] == 'posX' then posX = tonumber(d[4]) end
          if d[3] == 'posY' then posY = tonumber(d[4]) end
          if d[3] == 'posZ' then posZ = tonumber(d[4]) end
          if d[3] == 'rotX' then rotX = tonumber(d[4]) end
          if d[3] == 'rotY' then rotY = tonumber(d[4]) end
          if d[3] == 'rotZ' then rotZ = tonumber(d[4]) end
          if d[3] == 'rotW' then rotW = tonumber(d[4]) end
        end
      end

      -- If the vehicle pose was provided in the .csv, set the flag and teleport the vehicle to the given pose.
      isPose = im.BoolPtr(false)
      if posX then
        isPose = im.BoolPtr(true)
        spawn.safeTeleport(vehicle.veh, vec3(posX, posY, posZ), quat(rotX, rotY, rotZ, rotW))
      end

      -- Collect the connection data, and set the appropriate module state variables.
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'connection' then
          if d[3] == '3rd Party Computation Time' then compTime3rdParty = im.FloatPtr(tonumber(d[4])) end
          if d[3] == 'Ping Time' then pingTime = im.FloatPtr(tonumber(d[4])) end
          if d[2] == 'otherUDP' then
            if d[3] == 'IP' then sIP = im.ArrayChar(128, d[4]) end
            if d[3] == 'port' then sPort = im.IntPtr(tonumber(d[4])) end
          end
          if d[2] == 'beamngUDP' then
            if d[3] == 'IP' then rIP = im.ArrayChar(128, d[4]) end
            if d[3] == 'port' then rPort = im.IntPtr(tonumber(d[4])) end
          end
        end
      end

      -- Identify all the sensors listed in the .csv file, and store their names in a hashtable.
      local IMUsTable, GPSsTable, idealRADARsTable, roadsTable = {}, {}, {}, {}
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'sensors' then
          local sensorName = d[2]
          if string.find(sensorName, 'IMU') then IMUsTable[sensorName] = true end
          if string.find(sensorName, 'GPS') then GPSsTable[sensorName] = true end
          if sensorName == 'Ideal RADAR' then idealRADARsTable[sensorName] = true end
          if sensorName == 'Local Roads [Info]' then roadsTable[sensorName] = true end
        end
      end

    end,
    {{"csv",".csv"}},
    false,
    "/")
end

-- Executes the coupling.
local function execute()
  extensions.editor_fileDialog.openFile(
    function(data)
      isExecuting = true

      -- Read the .csv file into a lines structure.
      local csv = csvlib.readFileCSV(data.filepath)

      -- Collect all the 'To' signals.
      local signalsTo, ctr, numLines = {}, 1, #csv
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'signalTo' then
          signalsTo[ctr] = { type = d[5], groupName = d[2], name = d[3] }
          ctr = ctr + 1
        end
      end

      -- Collect all the 'From' signals.
      local signalsFrom, ctr = {}, 1
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'signalFrom' then
          signalsFrom[ctr] = {
            type = d[5], groupName = d[2], name = d[3],
            isValue = d[4] == 'value', isMultiply = d[4] == 'multiply', isAdd = d[4] == 'add', isFreeze = d[4] == 'freeze' }
          ctr = ctr + 1
        end
      end

      -- Check the vehicle against the vehicle in the .csv file, and issue a warning if it is different.
      local vehicle = vehicles[selectedVehicleIdx]
      local jBeam, config = vehicle.jBeam, vehicle.config
      local posX, posY, posZ, rotX, rotY, rotZ, rotW = nil, nil, nil, nil, nil, nil, nil
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'vehicle' then
          if d[3] == 'Vehicle Model' and d[4] ~= jBeam then
            log('W', logTag, 'Vehicle (jBeam) in .csv different to currently-selected vehicle!')
            break
          end
          if d[3] == 'Vehicle Config' and d[4] ~= config then
            log('W', logTag, 'Vehicle (config) in .csv different to currently-selected vehicle!')
            break
          end
          if d[3] == 'posX' then posX = tonumber(d[4]) end
          if d[3] == 'posY' then posY = tonumber(d[4]) end
          if d[3] == 'posZ' then posZ = tonumber(d[4]) end
          if d[3] == 'rotX' then rotX = tonumber(d[4]) end
          if d[3] == 'rotY' then rotY = tonumber(d[4]) end
          if d[3] == 'rotZ' then rotZ = tonumber(d[4]) end
          if d[3] == 'rotW' then rotW = tonumber(d[4]) end
        end
      end

      -- If the vehicle pose was provided in the .csv, teleport the vehicle to the given pose.
      -- Also move the camera to the vehicle.
      if posX then
        spawn.safeTeleport(vehicle.veh, vec3(posX, posY, posZ), quat(rotX, rotY, rotZ, rotW))
      end
      core_camera.setByName(0, "orbit", false)
      be:enterVehicle(0, scenetree.findObject(vehicle.vid))

      -- Collect the connection data, and set the appropriate module state variables.
      local time3rdParty, roundTripTime, udpSendIP, udpReceiveIP, udpSendPort, udpReceivePort = nil, nil, nil, nil, nil, nil
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'connection' then
          if d[3] == '3rd Party Computation Time' then time3rdParty = tonumber(d[4]) end
          if d[3] == 'Ping Time' then roundTripTime = tonumber(d[4]) end
          if d[2] == 'otherUDP' then
            if d[3] == 'IP' then udpSendIP = ffi.string(im.ArrayChar(128, d[4])) or '127.0.0.1' end
            if d[3] == 'port' then udpSendPort = tonumber(d[4]) end
          end
          if d[2] == 'beamngUDP' then
            if d[3] == 'IP' then udpReceiveIP = ffi.string(im.ArrayChar(128, d[4])) or '127.0.0.1' end
            if d[3] == 'port' then udpReceivePort = tonumber(d[4]) end
          end
        end
      end

      -- Identify all the sensors listed in the .csv file, and store their names in a hashtable.
      local IMUsTable, GPSsTable, idealRADARsTable, roadsTable = {}, {}, {}, {}
      local IMUsArray, GPSsArray, idealRADARsArray, roadsArray = {}, {}, {}, {}
      local iCtr, gCtr, irCtr, rCtr = 1, 1, 1, 1
      for i = 2, numLines do
        local d = csv[i]
        if d[1] == 'sensors' then
          local sensorName = d[2]
          if string.find(sensorName, 'IMU') and not IMUsTable[sensorName] then
            IMUsTable[sensorName] = true
            IMUsArray[iCtr] = sensorName
            iCtr = iCtr + 1
          end
          if string.find(sensorName, 'GPS') and not GPSsTable[sensorName] then
            GPSsTable[sensorName] = true
            GPSsArray[gCtr] = sensorName
            gCtr = gCtr + 1
          end
          if sensorName == 'Ideal RADAR' and not idealRADARsTable[sensorName] then
            idealRADARsTable[sensorName] = true
            idealRADARsArray[irCtr] = sensorName
            irCtr = irCtr + 1
          end
          if sensorName == 'Local Roads [Info]' and not roadsTable[sensorName] then
            roadsTable[sensorName] = true
            roadsArray[rCtr] = sensorName
            rCtr = rCtr + 1
          end
        end
      end

      -- Collect all the data for each identified sensor.
      local IMUs, GPSs, idealRADARs, roadsSensors = {}, {}, {}, {}
      for j = 1, #IMUsArray do
        local sensorName = IMUsArray[j]
        IMUs[j] = { name = sensorName, pos = vec3(0, 0), dir = vec3(0, 0), up = vec3(0, 0)}
        for i = 2, numLines do
          local d = csv[i]
          if d[2] == sensorName then
            local val = d[4]
            if val == 'true' then val = true elseif val == 'false' then val = false elseif val == 'nil' then val = false else val = tonumber(val) end
            if d[3] == 'posX' then IMUs[j].pos.x = val
            elseif d[3] == 'posY' then IMUs[j].pos.y = val
            elseif d[3] == 'posZ' then IMUs[j].pos.z = val
            elseif d[3] == 'dirX' then IMUs[j].dir.x = val
            elseif d[3] == 'dirY' then IMUs[j].dir.y = val
            elseif d[3] == 'dirZ' then IMUs[j].dir.z = val
            elseif d[3] == 'upX' then IMUs[j].up.x = val
            elseif d[3] == 'upY' then IMUs[j].up.y = val
            elseif d[3] == 'upZ' then IMUs[j].up.z = val
            else IMUs[j][d[3]] = val end
          end
        end
      end
      for j = 1, #GPSsArray do
        local sensorName = GPSsArray[j]
        GPSs[j] = { name = sensorName, pos = vec3(0, 0), dir = vec3(0, 0), up = vec3(0, 0)}
        for i = 2, numLines do
          local d = csv[i]
          if d[2] == sensorName then
            local val = d[4]
            if val == 'true' then val = true elseif val == 'false' then val = false elseif val == 'nil' then val = false else val = tonumber(val) end
            if d[3] == 'posX' then GPSs[j].pos.x = val
            elseif d[3] == 'posY' then GPSs[j].pos.y = val
            elseif d[3] == 'posZ' then GPSs[j].pos.z = val
            elseif d[3] == 'dirX' then GPSs[j].dir.x = val
            elseif d[3] == 'dirY' then GPSs[j].dir.y = val
            elseif d[3] == 'dirZ' then GPSs[j].dir.z = val
            elseif d[3] == 'upX' then GPSs[j].up.x = val
            elseif d[3] == 'upY' then GPSs[j].up.y = val
            elseif d[3] == 'upZ' then GPSs[j].up.z = val
            else GPSs[j][d[3]] = val end
          end
        end
      end
      for j = 1, #idealRADARsArray do
        local sensorName = idealRADARsArray[j]
        idealRADARs[j] = { name = sensorName }
        for i = 2, numLines do
          local d = csv[i]
          if d[2] == sensorName then
            local val = d[4]
            if val == 'true' then val = true elseif val == 'false' then val = false elseif val == 'nil' then val = false else val = tonumber(val) end
            idealRADARs[j][d[3]] = val
          end
        end
      end
      for j = 1, #roadsArray do
        local sensorName = roadsArray[j]
        roadsSensors[j] = { name = sensorName }
        for i = 2, numLines do
          local d = csv[i]
          if d[2] == sensorName then
            local val = d[4]
            if val == 'true' then val = true elseif val == 'false' then val = false elseif val == 'nil' then val = false else val = tonumber(val) end
            roadsSensors[j][d[3]] = val
          end
        end
      end

      -- Create the sensor map by sending a message to gelua to do so.
      -- [This is also where we create the sensor instances, and ensure the 'sensors' extension is loaded].
      extensions.load('tech_sensors')
      local mapIMUs, mapGPSs, mapIdealRADARs, mapRoads = {}, {}, {}, {}
      local vid, veh, sensorMap = vehicle.vid, vehicle.veh, {}
      for i = 1, #IMUs do
        IMUs[i].pos= coeffs2PosVS(IMUs[i].pos, veh)
        IMUs[i].dir, IMUs[i].up = sensor2VS(IMUs[i].dir, IMUs[i].up, veh)
        mapIMUs[i] = { name = IMUs[i].name, id = sensorMgr.createAdvancedIMU(vid, IMUs[i]) }
      end
      for i = 1, #GPSs do
        GPSs[i].pos= coeffs2PosVS(GPSs[i].pos, veh)
        GPSs[i].dir, GPSs[i].up = sensor2VS(GPSs[i].dir, GPSs[i].up, veh)
        mapGPSs[i] = { name = GPSs[i].name, id = sensorMgr.createGPS(vid, GPSs[i]) }
      end
      for i = 1, #idealRADARs do
        mapIdealRADARs[i] = { name = idealRADARs[i].name, id = sensorMgr.createIdealRADARSensor(vid, idealRADARs[i]) }
      end
      for i = 1, #roadsSensors do
        mapRoads[i] = { name = roadsSensors[i].name, id = sensorMgr.createRoadsSensor(vid, roadsSensors[i]) }
      end
      sensorMap = { IMUs = mapIMUs, GPSs = mapGPSs, idealRADARs = mapIdealRADARs, roads = mapRoads }

      local cData = {
        signalsTo = signalsTo, signalsFrom = signalsFrom,
        sensorMap = sensorMap,
        time3rdParty = time3rdParty, pingTime = roundTripTime,
        udpSendPort = udpSendPort, udpReceivePort = udpReceivePort,
        udpSendIP = udpSendIP, udpReceiveIP = udpReceiveIP,
        enableVSL=false,  -- Disable VSL
        enableCosim=true  -- Force CoSim mode
        -- enableVSL=enableVSL,
        -- enableCosim=enableCosim
      }
      be:queueObjectLua(vid, string.format("controller.loadControllerExternal('tech/cosimulationCoupling', 'cosimulationCoupling', %s)", serialize(lpack.encode({cData}))))

    end,
    {{"csv",".csv"}},
    false,
    "/")
end

-- Stops executing the coupling.
local function triggerStartLogging()
  be:queueObjectLua(vehicles[selectedVehicleIdx].vid, "controller.getController('cosimulationCoupling').StartLogging()")
-- Stops executing the coupling.
end
local function stopExecute()
  be:queueObjectLua(vehicles[selectedVehicleIdx].vid, "controller.getController('cosimulationCoupling').stop()")
  be:queueObjectLua(vehicles[selectedVehicleIdx].vid, "controller.unloadControllerExternal('cosimulationCoupling')")
  isExecuting = false
end




-- Manage the main tool window.
local function manageMainToolWindow()
  if editor.beginWindow(toolWinName, "Scene Vehicles###1", im.WindowFlags_NoTitleBar) then
    im.Separator()
    local listWidth = toolWinSize.x - 125  -- account for scrollbar width
    local listHeight = toolWinSize.y - 30
    if im.BeginListBox("", im.ImVec2(listWidth, listHeight), im.WindowFlags_ChildWindow) then
      local numVehicles = #vehicles
      selectedVehicleIdx = math.max(1, math.min(numVehicles, selectedVehicleIdx))

      for i = 1, numVehicles do
        local veh = vehicles[i]
        -- im.Columns(8, "sceneVehiclesListBoxColumns", false) -- Adjusted number of columns to 8 to include dropdown.
        im.Columns(7, "sceneVehiclesListBoxColumns", false) -- Adjusted number of columns to 8 to include dropdown.
        im.SetColumnWidth(0, 230)
        im.SetColumnWidth(1, 32)
        im.SetColumnWidth(2, 32)
        im.SetColumnWidth(3, 32)
        im.SetColumnWidth(4, 32)
        im.SetColumnWidth(5, 32)
        im.SetColumnWidth(6, 32)
        -- im.SetColumnWidth(7, 110) -- Adjusted for the dropdown.

        -- Handle the individual row selection.
        local vName = tostring(veh.vid .. ": " .. veh.name .. " - " .. veh.jBeam)
        if im.Selectable1(vName, i == selectedVehicleIdx, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
          if i ~= selectedVehicleIdx and not isExecuting then
            -- when executing it should not be possible to select another vehicle
            -- because clicking "stop coupling" on a different vehicle would not work correctly
            -- this is also why the "remove vehicle" button is disabled when executing
            selectedVehicleIdx = i
            table.clear(signals)
            return
          end
        end
        im.NextColumn()

        -- 'Remove Vehicle' button.
        if #vehicles > 1 then
          local text = 'Remove this vehicle from scene.'
          if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(22, 22), redB, nil, nil, 'removeVehicleButton') then
            if not isExecuting then
              local vehToDelete = vehicles[i]
              vehToDelete.veh:delete()
              if not vehicles[selectedVehicleIdx] then
                table.clear(signals)
                return
              end
              selectedVehicleIdx = math.min(numVehicles, selectedVehicleIdx)
              return
            end
          end
          if isExecuting then
            text = 'Cannot remove vehicle while coupling is active.'
          end
          im.tooltip(text)
        end
        im.NextColumn()

        -- 'Go To Vehicle' button.
        if editor.uiIconImageButton(editor.icons.cameraFocusOnVehicle2, im.ImVec2(21, 21), greenB, nil, nil, 'goToVehicleButton') then
          core_camera.setByName(0, "orbit", false)
          be:enterVehicle(0, scenetree.findObject(veh.vid))
          if i ~= selectedVehicleIdx and not isExecuting then
            selectedVehicleIdx = i
            table.clear(signals)
            return
          end
        end
        im.tooltip('Go to the selected vehicle.')
        im.NextColumn()

        -- 'Open Signals Window' button.
        local btnCol = blueB
        if isSignalsWinOpen and i == selectedVehicleIdx then btnCol = blueD end
        if editor.uiIconImageButton(editor.icons.code, im.ImVec2(19, 19), btnCol, nil, nil, 'openSignalsWinButton') then
          if i == selectedVehicleIdx or not isSignalsWinOpen then
            isSignalsWinOpen = not isSignalsWinOpen
          end
          if isSignalsWinOpen then
            editor.showWindow(signalsWinName)
          else
            editor.hideWindow(signalsWinName)
          end
          if i ~= selectedVehicleIdx then
            table.clear(signals)
            return
          end
          selectedVehicleIdx = i
        end
        im.tooltip('Open the signals window for this vehicle.')
        im.NextColumn()

        -- 'Start/Stop Coupling' toggle button.
        if selectedVehicleIdx == i then
          local btnCol = dullWhite
          local btnIcon = editor.icons.jointUnlocked
          local btnText = 'Start coupling with 3rd party.'
          local btnTextStop = 'Stop coupling with 3rd party.'
          if isExecuting then
            btnCol, btnIcon, btnText = orangeB, editor.icons.jointLocked, btnTextStop
          end
          if editor.uiIconImageButton(btnIcon, im.ImVec2(19, 19), btnCol, nil, nil, 'executeToggleButton') then
            if not isExecuting then
              execute()
            else
              stopExecute()
            end
          end
          im.tooltip(btnText)
        end
        im.NextColumn()

        -- 'Save Signals Configuration' button.
        if selectedVehicleIdx == i then
          if editor.uiIconImageButton(editor.icons.floppyDisk, im.ImVec2(19, 19), nil, nil, nil, 'saveSignalsConfig') then
            saveConfiguration(vehicles[i])
          end
          im.tooltip('Save the current signals configuration, for this vehicle, to disk.')
        end
        im.NextColumn()

        -- 'Load Signals Configuration' button.
        if selectedVehicleIdx == i then
          if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(19, 19), dullWhite, nil, nil, 'loadSignalsConfig') then
            loadConfiguration(vehicles[i])
          end
          im.tooltip('Load a signals configuration, for this vehicle, from disk.')
        end
        im.NextColumn()


        -- -- Custom Mode Dropdown Implementation
        -- if selectedVehicleIdx == i then
        --   local modes = {"CoSim", "VSL", "VSL+CoSim" }
        --   local currentMode = im.IntPtr(0) -- Default to VSL (index 0)

        --   -- Set currentMode based on current state
        --   if enableCosim and enableVSL then
        --       currentMode[0] = 1 -- VSL+CoSim
        --   elseif enableCosim and not enableVSL then
        --       currentMode[0] = 2 -- CoSim
        --   else
        --       currentMode[0] = 0 -- VSL
        --   end

        --   -- Apply the current mode settings (whether changed or default)
        --   local function applyModeSettings(mode)
        --       if mode == 0 then -- VSL
        --           enableCosim = false
        --           enableVSL = true
        --       elseif mode == 1 then -- VSL+CoSim
        --           enableCosim = true
        --           enableVSL = true
        --       elseif mode == 2 then -- CoSim
        --           enableCosim = true
        --           enableVSL = false
        --       end
        --   end

        --   -- Ensure default mode is applied
        --   applyModeSettings(currentMode[0])

        --   im.SetNextItemWidth(100) -- Set width for dropdown
        --   if im.BeginCombo("##ModeDropdown", modes[currentMode[0] + 1]) then
        --       for j, mode in ipairs(modes) do
        --           if im.Button(mode) then
        --               if currentMode[0] ~= (j - 1) then -- Only log if mode actually changes
        --                   currentMode[0] = j - 1
        --                   applyModeSettings(currentMode[0])
        --                   log('I', logTag, string.format("Mode switched to %s", modes[j]))
        --               end
        --               im.CloseCurrentPopup()
        --               break
        --           end
        --       end
        --       im.EndCombo()
        --   end

        --   im.tooltip("Select the mode: VSL (Logging only), VSL+CoSim (Logging and Co-Simulation), or CoSim (Co-Simulation only).")
        -- end

        -- im.NextColumn()

        -- im.Separator()
      end
      im.EndListBox()
    end
    im.Separator()
  end
  editor.endWindow()
end





local function manageVehicleSignalsWindow()
  if isSignalsWinOpen and vehicles[selectedVehicleIdx] then
    if editor.beginWindow(signalsWinName, vehicles[selectedVehicleIdx].name .. " [available signals]###2") then
      local toCtr, fromCtr = 1, 1

      -- Top row of checkboxes for each available signals group.
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Kinematics", isKinematics) then
        if isDriver[0] or isWheels[0] or isElectrics[0] or isPowertrain[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isKinematics = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Kinematics signals group.')
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Driver", isDriver) then
        if isKinematics[0] or isWheels[0] or isElectrics[0] or isPowertrain[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isDriver = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Driver signals group.')
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Wheels", isWheels) then
        if isKinematics[0] or isDriver[0] or isElectrics[0] or isPowertrain[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isWheels = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Wheels signals group.')
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Electrics", isElectrics) then
        if isKinematics[0] or isDriver[0] or isWheels[0] or isPowertrain[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isElectrics = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Electrics signals group.')
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Powertrain", isPowertrain) then
        if isKinematics[0] or isDriver[0] or isWheels[0] or isElectrics[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isPowertrain = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Powertrain signals group.')
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      if im.Checkbox("Sensors", isSensors) then
        if isKinematics[0] or isDriver[0] or isWheels[0] or isElectrics[0] or isPowertrain[0] then
          isRequestSent, isVluaDataReturned = false, false
          table.clear(signals)
        else
          isSensors = im.BoolPtr(true)
        end
      end
      im.tooltip('Include the Attached Sensors signals group.')

      im.Separator()

      -- Signals listbox.
      if im.BeginListBox("", im.ImVec2(665, 370), im.WindowFlags_ChildWindow) then
        local numSignals = #signals
        for i = 1, numSignals do
          local signal = signals[i]
          im.Columns(6, "vehSignalsListBoxColumns", true)
          im.SetColumnWidth(0, 40)
          im.SetColumnWidth(1, 55)
          im.SetColumnWidth(2, 110)
          im.SetColumnWidth(3, 260)
          im.SetColumnWidth(4, 60)
          im.SetColumnWidth(5, 32)

          -- Handle the individual row selection.
          if im.Selectable1("", false, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then end
          im.SameLine()

          -- 'Include Signal' checkbox.
          if signals[i].isIncluded then
            if editor.uiIconImageButton(editor.icons.check_box, im.ImVec2(20, 20), redB, nil, nil, 'includeSignalButton') then
              signals[i].isIncluded = false
            end
            im.tooltip('Do not include this signal in the vehicle signals configuration.')
          else
            if editor.uiIconImageButton(editor.icons.check_box_outline_blank, im.ImVec2(20, 20), redD, nil, nil, 'discludeSignalButton') then
              signals[i].isIncluded = true
            end
            im.tooltip('Include this signal in the vehicle signals configuration.')
          end
          im.SameLine()
          im.NextColumn()

          -- Currently-assigned signal position (index in configuration file).
          local posStr, ctrCol = ' ', greenB
          if signal.isIncluded then
            if signal.isFrom then
              posStr, ctrCol = tostring(fromCtr), redB
              fromCtr = fromCtr + 1
              if signal.isMultiply then
                posStr = posStr .. 'M'
                fromCtr = fromCtr + 1
              end
              if signal.isAdd then
                posStr = posStr .. 'A'
                fromCtr = fromCtr + 1
              end
              if signal.isFreeze then
                posStr = posStr .. 'F'
                fromCtr = fromCtr + 1
              end
            else
              posStr = tostring(toCtr)
              toCtr = toCtr + 1
            end
          end
          im.TextColored(ctrCol, posStr)
          im.SameLine()
          im.NextColumn()

          -- Signal group name, name, and data type.
          im.TextColored(blueB, signal.groupName)
          im.SameLine()
          im.NextColumn()
          im.TextColored(greenB, signal.description)
          im.SameLine()
          im.NextColumn()
          local type, typeCol = signal.type, redB
          if type == 'boolean' then
            typeCol = greenB
          elseif type == 'string' then
            typeCol = blueB
          end
          im.TextColored(typeCol, type)
          im.SameLine()
          im.NextColumn()

          -- 'To / From Direction' button.
          -- [Choice is only for available for signals which are not read only].
          if signal.readOnly then
            if editor.uiIconImageButton(editor.icons.fast_forward, im.ImVec2(20, 20), greenD, nil, nil, 'signalReadOnlyButton') then
            end
            im.tooltip('This signal is read only, and goes from BeamNG -> 3rd Party.')
          else
            if signal.isFrom then
              if editor.uiIconImageButton(editor.icons.fast_rewind, im.ImVec2(20, 20), greenB, nil, nil, 'signalFromButton') then
                  signal.isFrom = false
              end
              im.tooltip('Current Direction: 3rd Party -> BeamNG.')
            else
              if editor.uiIconImageButton(editor.icons.fast_forward, im.ImVec2(20, 20), greenB, nil, nil, 'signalToButton') then
                signal.isFrom = true
              end
              im.tooltip('Current Direction: BeamNG -> 3rd Party.')
            end
          end

          im.NextColumn()

          -- Add an extra separator between signal groups.
          if i < numSignals and signal.groupName ~= signals[i + 1].groupName then
            im.Separator()
          end
        end
        im.EndListBox()
      end
      im.Separator()

      -- 'Reload Signals' button.
      if editor.uiIconImageButton(editor.icons.autorenew, im.ImVec2(28, 28), nil, nil, nil, 'reloadSignals') then
        isRequestSent, isVluaDataReturned = false, false
        table.clear(signals)
      end
      im.tooltip("Reload available signals.")
      im.SameLine()

      -- 'Unlink All Signals' button.
      if editor.uiIconImageButton(editor.icons.unlink, im.ImVec2(28, 28), nil, nil, nil, 'unlinkAllSignals') then
        unlinkAllSignals()
      end
      im.tooltip("Unlink all selected signals (reset configuration).")
      im.SameLine()

      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()

      -- Display the proposed message sizes.
      im.Dummy(im.ImVec2(5, 0))
      im.SameLine()
      local fromKb, toKb, fromWarn, toWarn = fromCtr * 0.008, toCtr * 0.008, '', ''
      local fromCol, toCol = greenB, greenB
      if fromKb > 1.5 then fromCol, fromWarn = redB, '  [WARNING: > 1 MPC]' end
      if toKb > 1.5 then toCol, toWarn = redB, '  [WARNING: > 1 MPC]' end
      im.TextColored(fromCol, '                            From: ' .. fromKb .. '/1.5kb' .. fromWarn)
      im.SameLine()
      im.TextColored(toCol, '                       To: ' .. toKb .. '/1.5kb' .. toWarn)

      im.Separator()

      -- 'Store Vehicle Pose' checkbox.
      im.Checkbox("Store Pos/Rot", isPose)
      im.tooltip('Toggle whether to store the vehicle pose (position and rotation) with configuration file.')
      im.SameLine()

      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()

      -- 3rd party computation time input box.
      im.PushItemWidth(110)
      im.InputFloat("3rd Party Computation Time", compTime3rdParty, 1e-4, 0.0, "%.5f s")
      compTime3rdParty = im.FloatPtr(max(1e-4, min(1e4, compTime3rdParty[0])))
      im.tooltip('The expected computation time for each 3rd party cycle in the coupling.')
      im.PopItemWidth()
      im.SameLine()

      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()
      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()

      -- UDP ping time input box.
      im.PushItemWidth(110)
      im.InputFloat("UDP Ping Time", pingTime, 1e-5, 0.0, "%.5f s")
      pingTime = im.FloatPtr(max(1e-5, min(1e4, pingTime[0])))
      im.tooltip('The expected udp socket ping time.')
      im.PopItemWidth()

      im.Separator()

      -- Third party IP and socket.
      im.Text('3rd Party IP/port:')
      im.SameLine()
      im.PushItemWidth(80)
      im.InputText("###300", sIP)
      im.ArrayChar(128, "Server description")
      im.tooltip('Set the IP address on the 3rd party computer.')
      im.PopItemWidth()
      im.SameLine()
      im.PushItemWidth(120)
      im.InputInt("###301", sPort, 10, nil)
      im.tooltip('Set the port number on the 3rd party computer.')
      im.PopItemWidth()
      sPort = im.IntPtr(max(1025, min(65536, sPort[0])))
      im.SameLine()

      im.Dummy(im.ImVec2(15, 0))
      im.SameLine()

      -- BeamNG IP and socket.
      im.Text('BeamNG IP/port:')
      im.SameLine()
      im.PushItemWidth(80)
      im.InputText("###302", rIP)
      im.tooltip('Set the IP address on the BeamNG computer.')
      im.PopItemWidth()
      im.SameLine()
      im.PushItemWidth(120)
      im.InputInt("###303", rPort, 10, nil)
      im.tooltip('Set the port number on the 3rd party computer.')
      im.PopItemWidth()
      rPort = im.IntPtr(max(1025, min(65536, rPort[0])))

      -- im.Separator()
      -- TODO: Add frequency option
      -- Logging frequency input box with text label before it.
      -- im.PushItemWidth(85)  -- Set input box width to fit 2-3 digits
      -- im.Text("Logging frequency (sec):")
      -- im.SameLine()  -- Place the input box right after the label
      -- im.InputInt("", loggingFrequency, 1, 10)  -- Leave the label in InputInt blank to avoid duplicate text
      -- loggingFrequency[0] = math.max(1, loggingFrequency[0])  -- Ensure it's an integer with a minimum value of 1
      -- im.tooltip('The signal logging frequency in seconds.')

      -- im.PopItemWidth()

    else
      editor.hideWindow(signalsWinName) -- Handle window close.
      isSignalsWinOpen = false
    end
    editor.endWindow()
  end
end

-- World editor main callback for rendering the UI.
local function onEditorGui()
  if not isCosimulationSignalEditor then
    return
  end

  -- Update the vehicles list to show what is currently available in the scene.
  getCurrentVehicleList()

  -- Compute the signals list, if required.
  -- [This is only done if it does not currently exist, such as after a vehicle change].
  if #signals < 1 then
    if not updateSignalsList() then
      return
    end
  end

  -- Manage the front end.
  manageMainToolWindow()
  manageVehicleSignalsWindow()
end

-- Called when the 'Cosimulation Signal Editor' icon is pressed.
local function onActivate()
  editor.clearObjectSelection()
  editor.showWindow(toolWinName)
  isCosimulationSignalEditor = true
end

-- Called when the 'Cosimulation Signal Editor' is exited.
local function onDeactivate()
  editor.hideWindow(toolWinName)
  editor.hideWindow(signalsWinName)
  isCosimulationSignalEditor = false
  isSignalsWinOpen = false
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  if tech_license.isValid() then
    editor.editModes.cosimulationSignalEditMode = {
      displayName = "Edit Co-Simulation Signals",
      onUpdate = nop,
      onActivate = onActivate,
      onDeactivate = onDeactivate,
      icon = editor.icons.jointLocked,
      iconTooltip = "Co-Simulation Editor",
      auxShortcuts = {},
      hideObjectIcons = true }
    editor.registerWindow(toolWinName, toolWinSize)
    editor.registerWindow(signalsWinName, signalsWinSize)
  end
end

-- Callback for when the vehicle has been changed.
local function onVehicleReplaced(vid)
  table.clear(signals)
end

-- Serialization function.
local function onVehicleSpawned(vid)
  isExecuting = false
  log('I', logTag, 'On vehicle spawn - called on CTRL + R to reset coupling.')
end


-- Public interface.
M.updateCollectedVehicleData =                            updateCollectedVehicleData
M.onEditorGui =                                           onEditorGui
M.onEditorInitialized =                                   onEditorInitialized
M.onVehicleReplaced =                                     onVehicleReplaced
M.onVehicleSpawned =                                      onVehicleSpawned

return M