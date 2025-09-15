-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local basePersonality = {aggression = 0.5, patience = 0.5, bravery = 0.5}

function C:init(veh, name, data)
  data = data or {}
  self.veh = veh
  self.name = name or 'none'
  self.class = 'none'
  self.actionName = 'none' -- specific action name
  self.actionTimer = 0
  self.state = 'none' -- generic state name
  self.ignorePersonality = data.ignorePersonality or false -- if true, personality values stay neutral
  self.keepPersonalityOnRefresh = data.keepPersonalityOnRefresh or false
  self.keepActionOnRefresh = data.keepActionOnRefresh or false
  self.lockAction = false -- if true, the action stays the same after the vehicle resets
  self.targetVisible = false
  self.targetNear = false

  self.personalityModifiers = {}
  self.driver = {
    personality = deepcopy(basePersonality),
    aggression = veh.vars.baseAggression
  }
  self.flags = {}
  self.actions = {}
  self.baseActions = {
    pullOver = function (args) -- vehicle pulls over to the side of the road
      -- NOTE: It seems that this is not working properly; vehicle may not pull over to the side
      -- this should really be done in ai.lua instead
      args = args or {}
      if self.veh.isAi then
        local legalSide = map.getRoadRules().rightHandDrive and -1 or 1
        local changeLaneDist = args.dist or self.veh:getBrakingDistance(nil, 0.5) -- uses expected braking distance
        local sideDist = legalSide * 5 -- should be distance to side
        if args.useWarnSignal then
          getObjectByID(self.veh.id):queueLuaCommand('electrics.set_warn_signal(1)')
        end

        self.veh:setAiMode('traffic')
        -- the following calls are delayed while the AI plan rebuilds itself (needs improvement)
        -- this should really be done in ai.lua instead
        self.veh.queuedFuncs.laneChange = {timer = 0.25, vLua = 'ai.laneChange(nil, '..changeLaneDist..', '..sideDist..')'}
        self.veh.queuedFuncs.setStopPoint = {timer = 0.25, vLua = 'ai.setStopPoint(nil, '..(changeLaneDist + 20)..')'}
      end

      self.flags.pullOver = 1
      self.state = 'pullOver'
    end,
    disabled = function () -- vehicle permanently stops
      if self.veh.isAi then
        self.veh:setAiMode('stop')
        getObjectByID(self.veh.id):queueLuaCommand('electrics.set_warn_signal(1)')
        getObjectByID(self.veh.id):queueLuaCommand('electrics.set_lightbar_signal(0)')
      end
      self.state = 'disabled'
    end
  }
end

function C:postInit()
  if not self.keepActionOnRefresh then
    self:resetAction()
  end
  if not self.keepPersonalityOnRefresh then
    self:applyPersonality(self:generatePersonality())
  end
  self:onRefresh()
end

function C:setupFlowgraph(fgFile, varData) -- sets up custom flowgraph logic for this vehicle to use
  local path = FS:fileExists(fgFile or '')
  if not fgFile then
    log('E', 'traffic', 'Flowgraph file not found: '..dumps(fgFile))
    return
  end

  -- load the flowgraph and set its variables
  self.flowgraph = core_flowgraphManager.loadManager(fgFile)
  self.flowgraph.transient = true -- prevent flowgraph from restarting flowgraphs after ctrl+L
  for key, value in pairs(varData or {}) do
    if self.flowgraph.variables:variableExists(key) then
      self.flowgraph.variables:changeBase(key, value)
    else
      log('W', 'traffic', 'Flowgraph missing required variable when setting up baserole: '..dumps(key) .. " -> " .. dumps(value))
    end
  end
  self.flowgraph.vehicle = self.veh
  self.flowgraph.vehId = self.veh.id
  self.flowgraph:setRunning(true)
  self.flowgraph.modules.traffic.keepTrafficState = true
  self.veh:setAiMode('stop')
  self.lockAction = true
end

function C:clearFlowgraph() -- clears the flowgraph and resets the vehicle to default
  if self.flowgraph then
    self.flowgraph:setRunning(false, true)
    self.flowgraph = nil
    self.veh:setAiMode()
    self.lockAction = false
  end
end

function C:setTarget(id) -- sets the target vehicle id
  local obj = getObjectByID(self.veh.id)
  if id and getObjectByID(id) then
    self.targetId = id
    if self.veh.isAi then
      obj:queueLuaCommand('ai.setTargetObjectID('..self.targetId..')')
    end
  end
end

function C:setAction(name, args) -- sets the action to perform
  if name and self.actions[name] then
    if not self.lockAction then
      self.actions[name](args or {})
      self.actionName = name
      extensions.hook('onTrafficAction', self.veh.id, name, {targetId = self.targetId or 0, data = args or {}})
    end
  else
    log('E', 'traffic', 'Traffic role action not found: '..tostring(name))
  end
end

function C:setAggression(baseAggression, ignorePersonality) -- helper method that sets and sends the ai aggression value
  baseAggression = baseAggression or self.veh.vars.baseAggression
  local modifier = ignorePersonality and 0 or (self.driver.personality.aggression - 0.5) * 0.2 -- from -0.1 to 0.1
  self.driver.aggression = clamp(baseAggression + modifier, 0.2, 1)
  getObjectByID(self.veh.id):queueLuaCommand('ai.setAggression('..self.driver.aggression..')') -- slightly randomized aggression (mean 0.3)
end

function C:resetAction() -- resets the action to the default
  -- this always gets called when the vehicle respawns, unless self.keepActionOnRefresh is true
  if self.lockAction then return end
  if self.veh.isAi then
    self.veh:setAiMode() -- reset AI mode to whatever the main mode was
    self.veh:setAiParameters()
    self.veh:resetElectrics()
  end
  table.clear(self.flags)
  self.state = 'none'
  self.actionName = 'none'
  self.targetId = nil
  self.mapTarget = nil
end

function C:generatePersonality() -- returns a randomly generated personality
  if not self.veh.isAi then return end
  if self.ignorePersonality then
    return deepcopy(basePersonality)
  end

  local personality = {}
  for _, v in pairs({'aggression', 'patience', 'bravery'}) do
    local mod = {}
    if self.driver.personalityModifiers and self.driver.personalityModifiers[v] then
      mod = self.driver.personalityModifiers[v]
    end
    local value = mod.isLinear and math.random() or randomGauss3() / 3 -- linear or gaussian randomness
    personality[v] = clamp(value + (mod.offset or 0), mod.min or 0, mod.max or 1)
  end

  return personality
end

function C:applyPersonality(data) -- sets parameters based on personality data
  -- this always gets called when the vehicle respawns, unless self.keepPersonalityOnRefresh is true
  if type(data) ~= 'table' then
    self.driver.personality = deepcopy(basePersonality)
    return
  end
  local obj = getObjectByID(self.veh.id)

  self.driver.personality = tableMerge(self.driver.personality, data)
  self:setAggression()

  --[[ ai.lua parameters
  -- it would be nice to have more parameters or do this differently
  local params = {
    trafficWaitTime = data.patience * 5 -- intersection max wait time
  }

  obj:queueLuaCommand('ai.setParameters('..serialize(params)..')') ]]--
end

function C:checkTargetVisible(id) -- checks if the other vehicle is visible (static raycast)
  local visible = false
  local targetId = id or self.targetId
  local targetVeh = targetId and gameplay_traffic.getTrafficData()[targetId]
  if targetVeh then
    visible = self.veh:checkRayCast(targetVeh.pos + vec3(0, 0, 1))
  end

  return visible
end

function C:freezeTrafficSignals(state) -- overrides traffic lights; intended for emergency vehicles
  if not core_trafficSignals.getData().active then return end
  if state then
    self.flags.freezeSignals = 1
    for _, sequence in ipairs(core_trafficSignals.getSequences()) do
      local freeze = true
      local valid = false
      for _, link in pairs(sequence.linkedControllers) do
        local ctrlState = link.controller.states[link.stateIdx]
        if ctrlState then
          -- this could be better...
          if string.startswith(ctrlState.state, 'green') then
            sequence:advance()
            freeze = false
            break
          elseif string.startswith(ctrlState.state, 'yellow') then
            freeze = false
          elseif string.startswith(ctrlState.state, 'red') then
            valid = true
          end
        end
      end
      if freeze and valid then -- stops traffic signal sequence if all lights are red
        sequence._trafficFreeze = true
        sequence:enableTimer(false)
      end
    end
  else
    self.flags.freezeSignals = nil
    for _, sequence in ipairs(core_trafficSignals.getSequences()) do
      if sequence._trafficFreeze then
        sequence._trafficFreeze = nil
        sequence:enableTimer(true)
      end
    end
  end
end

function C:tryRandomEvent()
end

function C:onRefresh()
end

function C:onRoleStarted()
end

function C:onRoleEnded()
end

function C:onCrashDamage(data)
end

function C:onOtherCrashDamage(otherId, data)
end

function C:onCollision(otherId, data)
end

function C:onOtherCollision(id1, id2, data)
end

function C:onTrafficTick(tickTime)
end

function C:onUpdate(dt, dtSim)
end

function C:onSerialize()
  local data = {
    id = self.veh.id,
    name = self.name,
    state = self.state,
    actionName = self.actionName,
    targetId = self.targetId
  }
  return data
end

function C:onDeserialized(data)
  self.veh = gameplay_traffic.getTrafficData()[data.id]
  self.name = data.name
  self.state = data.state
  self.actionName = data.actionName
  self.targetId = data.targetId
end

return function(derivedClass, ...)
  local o = ... or {}
  setmetatable(o, C)
  C.__index = C
  o:init(o.veh, o.name)

  for k, v in pairs(derivedClass) do
    o[k] = v
  end

  o:init()
  o:postInit()
  return o
end
