-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[

-- extensions.load('mqttGrafanaDemo')

]]
local mqttBrokerURI = '127.0.0.1'



local M = {}
local mqtt = require("libs/luamqtt/mqtt/init")
local client = nil
local data = {position = {}}

local acos = math.acos
local deg = math.deg

local function updateWheelData()
  local vectorForward = obj:getDirectionVector()
  local vectorUp = obj:getDirectionVectorUp()
  local vectorRight = vectorForward:cross(vectorUp)

  for _, wd in pairs(wheels.wheels) do
    local name = wd.name
    local wheelData = {}
    if wd.steerAxisUp and wd.steerAxisDown then
      wheelData.caster = deg(acos(obj:nodeVecPlanarCos(wd.steerAxisUp, wd.steerAxisDown, vectorUp, vectorForward)))
      wheelData.sai = deg(acos(obj:nodeVecPlanarCos(wd.steerAxisUp, wd.steerAxisDown, vectorUp, vectorRight)))
    end
    --local camberSign = obj:nodeVecCos(wd.node2, wd.node2, vectorForward) --unused
    wheelData.camber = (90 - deg(acos(obj:nodeVecPlanarCos(wd.node2, wd.node1, vectorUp, vectorRight))))
    local toeSign = obj:nodeVecCos(wd.node1, wd.node2, vectorForward)
    wheelData.toe = deg(acos(obj:nodeVecPlanarCos(wd.node1, wd.node2, vectorRight, vectorForward)))
    if wheelData.toe > 90 then
      wheelData.toe = (180 - wheelData.toe) * sign(toeSign)
    else
      wheelData.toe = wheelData.toe * sign(toeSign)
    end
    -- failsafes for NaN below, broke UI before ...
    if isnan(wheelData.toe) or isinf(wheelData.toe) then
      wheelData.toe = 0
    end
    if isnan(wheelData.camber) or isinf(wheelData.camber) then
      wheelData.camber = 0
    end
    local hasPressure = wd.pressureGroup and v.data.pressureGroups and v.data.pressureGroups[wd.pressureGroup]
    wheelData.pressure = hasPressure and obj:getGroupPressure(v.data.pressureGroups[wd.pressureGroup]) * 0.000145038 or 0
    wheelData.angularVelocity = wd.angularVelocity

    wheelData.brakeSurfaceTemperature = wd.brakeSurfaceTemperature
    wheelData.brakeCoreTemperature = wd.brakeCoreTemperature
    wheelData.isBrakeMolten = wd.isBrakeMolten
    wheelData.radius = wd.radius
    wheelData.wheelDir = wd.wheelDir
    wheelData.propulsionTorque = wd.propulsionTorque
    wheelData.lastSlip = wd.lastSlip
    wheelData.downForce = wd.downForce
    wheelData.brakingTorque = wd.brakingTorque
    wheelData.brakeTorque = wd.brakeTorque

    data['wheel_' .. name] = wheelData
  end
end

local function updateElectrics()
  data.temp = obj:getEnvTemperature() - 273.15
  data.signal_L = electrics.values.signal_L
  data.signal_R = electrics.values.signal_R
  data.lights = electrics.values.lights
  data.highbeam = electrics.values.highbeam
  data.fog = 0 --no fog lights on vivace
  data.lowpressure = electrics.values.lowpressure
  data.lowfuel = electrics.values.lowfuel
  data.parkingbrake = electrics.values.parkingbrake
  data.checkengine = electrics.values.checkengine
  data.hazard = electrics.values.hazard
  data.oil = electrics.values.oil
  data.cruiseControlActive = electrics.values.cruiseControlActive
  data.gear = electrics.values.gear
  data.rpmTacho = electrics.values.rpmTacho
  data.fuel = electrics.values.fuel
  data.watertemp = electrics.values.watertemp
  data.engineRunning = electrics.values.engineRunning
  data.wheelspeed = electrics.values.wheelspeed
  data.esc = electrics.values.esc
  data.escActive = electrics.values.escActive
  data.tcs = electrics.values.tcs
  data.tcsActive = electrics.values.tcsActive
  data.pwr = powerDisplay
  data.clutch = electrics.values.clutch
  data.brake = electrics.values.brake
  data.throttle = electrics.values.throttle
  data.engineInfo = controller.mainController.engineInfo
end

local function updateSensors()
  data.gx = sensors.gx
  data.gy = sensors.gy
  data.gz = sensors.gz
  data.gx2 = sensors.gx2
  data.gy2 = sensors.gy2
  data.gz2 = sensors.gz2
  data.forceAtWheelNorm = hydros.forceAtWheelNorm
  data.forceAtDriverNorm = hydros.forceAtDriverNorm
  data.curForceLimitNorm = hydros.curForceLimitNorm
  local lp = data.position
  lp.x, lp.y, lp.z = obj:getPositionXYZ()
  data.roll, data.pitch, data.yaw = obj:getRollPitchYaw()
  data.gravity = obj:getGravity()
end

local function updateGFX(dt)
  if not client then return end
  updateWheelData()
  updateElectrics()
  updateSensors()

  --dump{'MQTT DATA: car_data', data}
  client:publish({topic = 'car_data', payload = jsonEncode(data)})
end

--[[
-- TODO: receive messages
local function handleMQTTMessage(mid, topic, payload)
  log("I", '', "Received MQTT message: " .. tostring(topic) .. " - " .. tostring(payload))
  if topic == "button/press" then
    if payload == "button1" then
      -- Simulate button 1 press
      electrics.toggle_left_signal() -- Example action
    elseif payload == "button2" then
      -- Simulate button 2 press
      electrics.toggle_right_signal() -- Example action
    end
  end
end
--]]

local function onExtensionLoaded()
  client = mqtt.client({uri = mqttBrokerURI, clean = true})
  if client and client:start_connecting() then
    log("I", '', "MQTT connected: " .. mqttBrokerURI)
    -- TODO: receive messages
    --client:subscribe({topic = 'car_input', qos = 0})
    --client:on_message(handleMQTTMessage)
  else
    client = nil
    log("E", '', "Failed to connect to MQTT client: " .. mqttBrokerURI)
  end
end

local function onExtensionUnloaded()
  if client then
    client:disconnect()
    client = nil
    log("I", '', "MQTT client disconnected and cleaned up")
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.reset = reset

M.updateGFX = updateGFX

return M
