-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"tech_CANBus_CANBusPeak"}

local abs = math.abs
local max = math.max
local deg = math.deg

local engines = {}

local messageIds = {
  sendVehicleData1 = 0x10,
  sendVehicleData2 = 0x11,
  sendInput1 = 0x20,
  sendEnvData1 = 0x30,
  sendECU1 = 0x100,
  sendECU2 = 0x101,
  sendECU3 = 0x102,
  sendTCU1 = 0x110,
  sendWheelData1 = 0x150,
  sendWheelData2 = 0x151,
  sendWheelData3 = 0x152,
  sendLights1 = 0x200
}

local vehicleData1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local vehicleData2Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local input1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local envData1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local ecu1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local ecu2Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local ecu3Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local tcu1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local lights1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local wheelData1Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local wheelData2Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
local wheelData3Message = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}

local canBus

local updateTimer = 0
local updateTime = 1 / 100

--helper function to easily access wheel data without checking for a certain wheel's existance
local function getWheelDataOrDefault(wheelName, wheelProperty, default)
  if wheels.wheelIDs[wheelName] and wheels.wheels[wheels.wheelIDs[wheelName]] then
    return wheels.wheels[wheels.wheelIDs[wheelName]][wheelProperty] or default
  end
  return default
end

local function updateHardwareState(dt)
  local accX = sensors.gx2
  local accY = sensors.gy2
  local accZ = sensors.gz2
  local rollAV, pitchAV, yawAV = obj:getRollPitchYawAngularVelocity()
  local rollAVByte1, rollAVByte2 = canBus.twoBytes(deg(rollAV) * 10)
  local pitchAVByte1, pitchAVByte2 = canBus.twoBytes(deg(pitchAV) * 10)
  local yawAVByte1, yawAVByte2 = canBus.twoBytes(deg(yawAV) * 10)
  vehicleData1Message[1] = accX
  vehicleData1Message[2] = accY
  vehicleData1Message[3] = accZ
  vehicleData1Message[4] = rollAVByte1
  vehicleData1Message[5] = rollAVByte2
  vehicleData1Message[6] = pitchAVByte1
  vehicleData1Message[7] = pitchAVByte2
  canBus.sendCANMessage(messageIds.sendVehicleData1, vehicleData1Message, "vehicleData1")

  local vehicleVelocity = obj:getVelocity():length()
  local vehicleVelocityByte1, vehicleVelocityByte2 = canBus.twoBytes(vehicleVelocity * 100 * 3.6)
  local rollAngle, pitchAngle = obj:getRollPitchYaw()
  local rollAngleByte1, rollAngleByte2 = canBus.twoBytes(deg(rollAngle) * 10)
  local pitchAngleByte1, pitchAngleByte2 = canBus.twoBytes(deg(pitchAngle) * 10)
  vehicleData2Message[1] = vehicleVelocityByte1
  vehicleData2Message[2] = vehicleVelocityByte2
  vehicleData2Message[3] = rollAngleByte1
  vehicleData2Message[4] = rollAngleByte2
  vehicleData2Message[5] = pitchAngleByte1
  vehicleData2Message[6] = pitchAngleByte2
  vehicleData2Message[7] = yawAVByte1
  vehicleData2Message[8] = yawAVByte2
  canBus.sendCANMessage(messageIds.sendVehicleData2, vehicleData2Message, "vehicleData2")

  local throttleInput = electrics.values.throttle_input * 100
  local brakeInput = electrics.values.brake_input * 100
  local clutchInput = electrics.values.clutch_input * 100
  input1Message[1] = throttleInput
  input1Message[2] = brakeInput
  input1Message[3] = clutchInput
  canBus.sendCANMessage(messageIds.sendInput1, input1Message, "input1")

  local tEnv = obj:getEnvTemperature() - 273.15
  local tEnvByte1, tEnvByte2 = canBus.twoBytes(tEnv * 10)
  local pressureEnv = obj:getEnvPressure()
  local pressureEnvByte1, pressureEnvByte2 = canBus.twoBytes(pressureEnv * 0.01) --in kPa
  local altitude = electrics.values.altitude
  local altitudeByte1, altitudeByte2, altitudeByte3, altitudeByte4 = canBus.fourBytes(altitude * 100)
  envData1Message[1] = tEnvByte1
  envData1Message[2] = tEnvByte2
  envData1Message[3] = pressureEnvByte1
  envData1Message[4] = pressureEnvByte2
  envData1Message[5] = altitudeByte1
  envData1Message[6] = altitudeByte2
  envData1Message[7] = altitudeByte3
  envData1Message[8] = altitudeByte4
  canBus.sendCANMessage(messageIds.sendEnvData1, envData1Message, "envData")

  local rpm = electrics.values.rpm or 0
  local load = (electrics.values.engineLoad or 0) * 100
  local rpmByte1, rpmByte2 = canBus.twoBytes(rpm * 4)
  local torque = 0
  local power = 0
  for _, engine in ipairs(engines) do
    torque = torque + engine.outputTorque1 or 0
    power = power + torque * engine.outputAV1 / 1000
  end
  local torqueByte1, torqueByte2 = canBus.twoBytes(torque * 10)
  local powerByte1, powerByte2 = canBus.twoBytes(power * 10)
  local ignitionLevel = electrics.values.ignitionLevel
  ecu1Message[1] = rpmByte1
  ecu1Message[2] = rpmByte2
  ecu1Message[3] = load
  ecu1Message[4] = torqueByte1
  ecu1Message[5] = torqueByte2
  ecu1Message[6] = powerByte1
  ecu1Message[7] = powerByte2
  ecu1Message[8] = ignitionLevel
  canBus.sendCANMessage(messageIds.sendECU1, ecu1Message, "ecu1")

  local coolantTemp = electrics.values.watertemp or 0
  local oilTemp = electrics.values.oiltemp or 0
  local coolantByte1, coolantByte2 = canBus.twoBytes(coolantTemp * 10)
  local oilByte1, oilByte2 = canBus.twoBytes(oilTemp * 10)
  ecu2Message[1] = coolantByte1
  ecu2Message[2] = coolantByte2
  ecu2Message[3] = oilByte1
  ecu2Message[4] = oilByte2
  canBus.sendCANMessage(messageIds.sendECU2, ecu2Message, "ecu2")

  local intakePressure = (electrics.values.turboBoost or 0) * 6.89476 + pressureEnv * 0.001 --PSI to kPa
  local turboRPM = electrics.values.turboRPM or 0
  local intakePressureByte1, intakePressureByte2 = canBus.twoBytes(intakePressure * 10)
  local turboRPMByte1, turboRPMByte2 = canBus.twoBytes(turboRPM * 0.1)
  ecu3Message[1] = intakePressureByte1
  ecu3Message[2] = intakePressureByte2
  ecu3Message[3] = turboRPMByte1
  ecu3Message[4] = turboRPMByte2
  canBus.sendCANMessage(messageIds.sendECU3, ecu3Message, "ecu3")

  local velocity = electrics.values.wheelspeed or 0
  local velocityByte1, velocityByte2 = canBus.twoBytes(velocity * 100 * 3.6)
  local gear = electrics.values.gearIndex or 0
  tcu1Message[1] = gear
  tcu1Message[2] = velocityByte1
  tcu1Message[3] = velocityByte2
  canBus.sendCANMessage(messageIds.sendTCU1, tcu1Message, "tcu1")

  local lights1Byte1, lights1Byte2 = 0, 0
  lights1Byte1 = lights1Byte1 + (electrics.values.signal_L > 0 and 0x1 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.signal_R > 0 and 0x2 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.fog > 0 and 0x4 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.lowbeam > 0 and 0x8 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.highbeam > 0 and 0x10 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.reverse > 0 and 0x20 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.parkingbrake > 0 and 0x40 or 0x0)
  lights1Byte1 = lights1Byte1 + (electrics.values.checkengine and 0x80 or 0x0) --bool, not a number...
  lights1Byte2 = lights1Byte2 + (electrics.values.lowfuel and 0x1 or 0x0) --bool, not a number...
  lights1Byte2 = lights1Byte2 + (electrics.values.lowpressure > 0 and 0x2 or 0x0)
  lights1Byte2 = lights1Byte2 + ((electrics.values.esc or 0) > 0 and 0x4 or 0x0)
  lights1Byte2 = lights1Byte2 + ((electrics.values.tcs or 0) > 0 and 0x8 or 0x0)
  lights1Message[1] = lights1Byte1
  lights1Message[2] = lights1Byte2
  canBus.sendCANMessage(messageIds.sendLights1, lights1Message, "lights1")

  local wheelSpeedFL = getWheelDataOrDefault("FL", "wheelSpeed", 0)
  local wheelSpeedFR = getWheelDataOrDefault("FR", "wheelSpeed", 0)
  local wheelSpeedRL = getWheelDataOrDefault("RL", "wheelSpeed", 0)
  local wheelSpeedRR = getWheelDataOrDefault("RR", "wheelSpeed", 0)
  local FLSpeedByte1, FLSpeedByte2 = canBus.twoBytes(abs(wheelSpeedFL) * 3.6 * 100)
  local FRSpeedByte1, FRSpeedByte2 = canBus.twoBytes(abs(wheelSpeedFR) * 3.6 * 100)
  local RLSpeedByte1, RLSpeedByte2 = canBus.twoBytes(abs(wheelSpeedRL) * 3.6 * 100)
  local RRSpeedByte1, RRSpeedByte2 = canBus.twoBytes(abs(wheelSpeedRR) * 3.6 * 100)
  wheelData1Message[1] = FLSpeedByte1
  wheelData1Message[2] = FLSpeedByte2
  wheelData1Message[3] = FRSpeedByte1
  wheelData1Message[4] = FRSpeedByte2
  wheelData1Message[5] = RLSpeedByte1
  wheelData1Message[6] = RLSpeedByte2
  wheelData1Message[7] = RRSpeedByte1
  wheelData1Message[8] = RRSpeedByte2
  canBus.sendCANMessage(messageIds.sendWheelData1, wheelData1Message, "wheeldata1")

  local airPressureFL = obj:getGroupPressure(getWheelDataOrDefault("FL", "pressureGroupId", -1))
  local airPressureFR = obj:getGroupPressure(getWheelDataOrDefault("FR", "pressureGroupId", -1))
  local airPressureRL = obj:getGroupPressure(getWheelDataOrDefault("RL", "pressureGroupId", -1))
  local airPressureRR = obj:getGroupPressure(getWheelDataOrDefault("RR", "pressureGroupId", -1))
  local airPressureFLByte1, airPressureFLByte2 = canBus.twoBytes(max(airPressureFL - pressureEnv, 0) * 0.01)
  local airPressureFRByte1, airPressureFRByte2 = canBus.twoBytes(max(airPressureFR - pressureEnv, 0) * 0.01)
  local airPressureRLByte1, airPressureRLByte2 = canBus.twoBytes(max(airPressureRL - pressureEnv, 0) * 0.01)
  local airPressureRRByte1, airPressureRRByte2 = canBus.twoBytes(max(airPressureRR - pressureEnv, 0) * 0.01)
  wheelData2Message[1] = airPressureFLByte1
  wheelData2Message[2] = airPressureFLByte2
  wheelData2Message[3] = airPressureFRByte1
  wheelData2Message[4] = airPressureFRByte2
  wheelData2Message[5] = airPressureRLByte1
  wheelData2Message[6] = airPressureRLByte2
  wheelData2Message[7] = airPressureRRByte1
  wheelData2Message[8] = airPressureRRByte2
  canBus.sendCANMessage(messageIds.sendWheelData2, wheelData2Message, "wheeldata2")

  local brakeTempFL = getWheelDataOrDefault("FL", "brakeSurfaceTemperature", 0)
  local brakeTempFR = getWheelDataOrDefault("FR", "brakeSurfaceTemperature", 0)
  local brakeTempRL = getWheelDataOrDefault("RL", "brakeSurfaceTemperature", 0)
  local brakeTempRR = getWheelDataOrDefault("RR", "brakeSurfaceTemperature", 0)
  local brakeTempFLByte1, brakeTempFLByte2 = canBus.twoBytes(brakeTempFL * 10)
  local brakeTempFRByte1, brakeTempFRByte2 = canBus.twoBytes(brakeTempFR * 10)
  local brakeTempRLByte1, brakeTempRLByte2 = canBus.twoBytes(brakeTempRL * 10)
  local brakeTempRRByte1, brakeTempRRByte2 = canBus.twoBytes(brakeTempRR * 10)
  wheelData3Message[1] = brakeTempFLByte1
  wheelData3Message[2] = brakeTempFLByte2
  wheelData3Message[3] = brakeTempFRByte1
  wheelData3Message[4] = brakeTempFRByte2
  wheelData3Message[5] = brakeTempRLByte1
  wheelData3Message[6] = brakeTempRLByte2
  wheelData3Message[7] = brakeTempRRByte1
  wheelData3Message[8] = brakeTempRRByte2
  canBus.sendCANMessage(messageIds.sendWheelData3, wheelData3Message, "wheeldata3")
  --dumpz(wheels.wheels[0], 1)
  --dump(electrics.values.turboBoost)
end

local function updateGFX(dt)
  if playerInfo.firstPlayerSeated then
    if canBus then
      updateTimer = updateTimer - dt
      if updateTimer <= 0 then
        updateHardwareState(dt)

        updateTimer = updateTimer + updateTime
      end
    end
  end
end

local function onExtensionLoaded()
  log("I", "RacingDisplay.onExtensionLoaded", "CANBus Racing Display extension loaded")
  canBus = extensions.tech_CANBus_CANBusPeak
  if canBus then
    log("D", "RacingDisplay.onExtensionLoaded", "CANBus extension found")
    if not canBus.isConnected then
      log("D", "RacingDisplay.onExtensionLoaded", "CANBus extension is not connected, connecting now...")
      local connectionResult = canBus.initCANBus()
      if connectionResult ~= canBus.errorCodes.OK then
        log("E", "RacingDisplay.onExtensionLoaded", "Non-OK init result for CAN Bus, shutting down... Result: " .. canBus.errorCodeLookup[connectionResult])
        canBus = nil
      end
    end
  else
    log("E", "RacingDisplay.onExtensionLoaded", "CANBus extension NOT found CANBus Controller won't work...")
  end

  engines = {}
  for _, engine in ipairs(powertrain.getDevicesByCategory("engine")) do
    table.insert(engines, engine)
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

return M
