-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min = math.min
local max = math.max

local activeThrusters = {}
local thrusterState = {}
local autoThrusters = {}
local impulseState = {}
local clusterThrust = {}
local thrusting = false
local zeroVec = vec3(0, 0, 0)

local function update()
  -- node1 is source, node2 is destination
  -- we apply and measure forces/velocity for node2

  local t
  for _, thruster in ipairs(autoThrusters) do
    local vel = -obj:getNodeVelocity(thruster.id2, thruster.id1)
    if vel > 0.3 then
      t = (vel * vel) * thruster.factor
      if thrusting then
        t = min(max(obj:getNodeForce(thruster.id1, thruster.id2), 0) + t, thruster.thrustLimit)
      else
        t = min(t, thruster.thrustLimit)
      end
      obj:applyForce(thruster.id2, thruster.id1, t)
    end
  end

  for _, thruster in ipairs(thrusterState) do
    -- applyForce(node1, node2, forceMagnitude)
    obj:applyForce(thruster[2], thruster[1], thruster[3])
  end

  local impulseCount = #impulseState
  for i = impulseCount, 1, -1 do
    -- applyForce(node1, node2, forceMagnitude)
    local thruster = impulseState[i]
    obj:applyForce(thruster[1], thruster[2], thruster[3])
    local ttl = thruster[4]
    ttl = ttl - physicsDt
    if ttl <= 0 then
      table.remove(impulseState, i)
    else
      thruster[4] = ttl
    end
  end

  -- clusters
  for nodeId, thruster in pairs(clusterThrust) do
    local ttl = thruster[2]
    ttl = ttl - physicsDt
    if ttl > 0 then
      if ttl < physicsDt then
        thruster[1]:setScaled(ttl / physicsDt)
      end
      obj:applyClusterLinearAngularAccel(nodeId, thruster[1], thruster[3] or zeroVec)
      thruster[2] = ttl
    else
      clusterThrust[nodeId] = nil
    end
  end

  if #activeThrusters + impulseCount + #autoThrusters == 0 and next(clusterThrust) == nil then
    M.update = nop
    updateCorePhysicsStepEnabled()
  end
end

local invalidThrusterControlWarned
local function updateGFX()
  table.clear(thrusterState)
  for _, thruster in ipairs(activeThrusters) do
    if thruster.control then
      if electrics.values[thruster.control] then
        table.insert(thrusterState, {thruster.id1, thruster.id2, min(electrics.values[thruster.control] * thruster.factor, thruster.thrustLimit)})
      else
        if not invalidThrusterControlWarned then
          log("E", "", "Thruster with id1="..dumps(thruster.id1).." and id2="..dumps(thruster.id2).." in vehicle "..dumps(vehiclePath).." tried to use an invalid control="..dumps(thruster.control)..". The vehicle creator should instead use vehicle-specific bindings (or an electrics channel) to control this thruster")
          invalidThrusterControlWarned = true
        end
      end
    end
  end

  thrusting = #thrusterState > 0
end

local function applyImpulse(n1, n2, force, dt)
  for _, thruster in ipairs(impulseState) do
    if thruster[1] == n1 and thruster[2] == n2 then
      thruster[3] = force
      thruster[4] = dt or lastDt
      return
    end
  end

  table.insert(impulseState, {n1, n2, force, dt})
  M.update = update
  updateCorePhysicsStepEnabled()
end

local function applyImpulseBody(force, dt)
  local n1, n2 = v.data.refNodes[0].ref, v.data.refNodes[0].back
  for _, thruster in ipairs(impulseState) do
    if thruster[1] == n1 and thruster[2] == n2 then
      thruster[3] = force
      thruster[4] = dt or lastDt
      return
    end
  end

  table.insert(impulseState, {n1, n2, force, dt})
  M.update = update
  updateCorePhysicsStepEnabled()
end

local function applyAccel(accel, dt, nodeId, angularAccel)
  dt = dt or lastDt
  nodeId = nodeId or (v.data.refNodes and v.data.refNodes[0].cid) or 0
  clusterThrust[nodeId] = {accel, dt, angularAccel}
  M.update = update
  updateCorePhysicsStepEnabled()
end

local function getAccelDt(nodeId)
  local ct = clusterThrust[nodeId]
  return ct and ct[2] or 0
end

local function applyVelocity(velocity, dt, nodeId)
  dt = dt or lastDt
  if dt == 0 then return end
  nodeId = nodeId or (v.data.refNodes and v.data.refNodes[0].cid) or 0
  applyAccel((velocity - obj:getClusterVelocityWithoutWheels(nodeId)) / dt, dt, nodeId)
end

local function isPhysicsStepUsed()
  return M.update == update
end

local function init()
  -- update public interface
  if v.data.thrusters == nil or next(v.data.thrusters) == nil then
    M.update = nop
    M.updateGFX = nop
    return
  else
    M.update = update
    M.updateGFX = updateGFX
  end

  thrusterState = {}
  autoThrusters = {}
  impulseState = {}
  activeThrusters = {}
  for _, thruster in pairs(v.data.thrusters) do
    if thruster.control == "auto" then
      table.insert(autoThrusters, thruster)
    else
      table.insert(activeThrusters, thruster)
    end
  end

  for _, thruster in pairs(activeThrusters) do
    thruster.factor = thruster.factor or 1
    thruster.thrustLimit = thruster.thrustLimit or math.huge
  end
end

-- public interface
M.reset = init
M.init = init
M.update = nop
M.updateGFX = nop
M.applyImpulse = applyImpulse
M.applyImpulseBody = applyImpulseBody
M.applyAccel = applyAccel
M.applyVelocity = applyVelocity
M.getAccelDt = getAccelDt
M.isPhysicsStepUsed = isPhysicsStepUsed

return M