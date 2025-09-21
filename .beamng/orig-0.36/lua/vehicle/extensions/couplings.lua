-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

--called from beamstate when auto coupling is activated from the toggle functionality
local function onBeamstateActivateAutoCoupling()
  local fiftwheels = controller.getControllersByType("couplings/fifthwheel")
  for _, fifthwheel in ipairs(fiftwheels) do
    fifthwheel.setFifthwheelIndicatorVisibility(true)
  end
end

--called from beamstate when auto coupling is disabled from the toggle functionality
local function onBeamstateDisableAutoLatching()
  local fiftwheels = controller.getControllersByType("couplings/fifthwheel")
  for _, fifthwheel in ipairs(fiftwheels) do
    fifthwheel.setFifthwheelIndicatorVisibility(false)
  end
end

--called from beamstate when couplers are detached from the toggle functionality
local function onBeamstateDetachCouplers()
  local fiftwheels = controller.getControllersByType("couplings/fifthwheel")
  for _, fifthwheel in ipairs(fiftwheels) do
    fifthwheel.detachFifthwheel()
  end
end

local function couplingAttached(nodeId, obj2id, obj2nodeId)
  obj:stopLatching()
  beamstate.disableAutoCoupling()
end

local function couplingDetached(nodeId, obj2id, obj2nodeId)
end

--called from beamstate when checking what to do in the toggle functionality
local function isAutoCouplingActive()
  return false
end

--called from beamstate when checking what to do in the toggle functionality
local function isCouplerAttached()
  local fiftwheels = controller.getControllersByType("couplings/fifthwheel")
  for _, fifthwheel in ipairs(fiftwheels) do
    if fifthwheel.isAttached() then
      return true
    end
  end
end

local function onReset()
end

M.onReset = onReset

M.couplingDetached = couplingDetached
M.couplingAttached = couplingAttached

M.onBeamstateActivateAutoCoupling = onBeamstateActivateAutoCoupling
M.onBeamstateDisableAutoLatching = onBeamstateDisableAutoLatching
M.onBeamstateDetachCouplers = onBeamstateDetachCouplers

M.isAutoCouplingActive = isAutoCouplingActive
M.isCouplerAttached = isCouplerAttached

return M
