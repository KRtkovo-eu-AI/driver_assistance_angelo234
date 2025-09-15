-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

-- local function foo()
-- end

-- local function update(dt)
-- end

-- local function updateWheelsIntermediate(dt)
-- end

-- local function updateFixedStep(dt)
-- end

local function updateGFX(dt)
end

-- local function debugDraw(focusPos)
-- end

-- local function beamBroken(id, energy)
-- end

-- local function beamDeformed(id, ratio)
-- end

-- local function nodeCollision(p)
-- end

-- local function onCouplerFound(nodeId, obj2id, obj2nodeId)
-- end

-- local function onCouplerAttached(nodeId, obj2id, obj2nodeId)
-- end

-- local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
-- end

-- local function onGameplayEvent(eventName, ...)
-- end

-- local function settingsChanged()
-- end

-- local function resetSounds(jbeamData)
-- end

local function reset(jbeamData)
end

-- local function resetLastStage(jbeamData)
-- end

-- local function initSounds(jbeamData)
-- end

local function init(jbeamData)
end

-- local function initSecondStage(jbeamData)
-- end

-- local function initLastStage(jbeamData)
-- end

-- local function serialize()
--   return {
--     bar = 0
--   }
-- end

-- local function deserialize(data)
--   if data then
--     if data.bar then
--     --do something with data.bar
--     end
--   end
-- end

M.init = init
--M.initSecondStage = initSecondStage
--M.initLastStage = initLastStage
--M.initSounds = initSounds

M.reset = reset
--M.resetLastStage = resetLastStage
--M.resetSounds = resetSounds

--M.update = update
--M.updateWheelsIntermediate = updateWheelsIntermediate
--M.updateFixedStep = updateFixedStep
M.updateGFX = updateGFX

--M.debugDraw = debugDraw

--M.beamBroken = beamBroken
--M.beamDeformed = beamDeformed
--M.nodeCollision = nodeCollision

--M.onCouplerFound = onCouplerFound
--M.onCouplerAttached = onCouplerAttached
--M.onCouplerDetached = onCouplerDetached

--M.onGameplayEvent = onGameplayEvent
--M.settingsChanged = settingsChanged

--M.serialize = serialize
--M.deserialize = deserialize

--M.foo = foo

return M
