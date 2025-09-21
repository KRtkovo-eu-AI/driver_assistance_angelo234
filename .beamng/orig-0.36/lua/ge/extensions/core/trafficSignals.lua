-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Traffic Signals V2

local M = {}

local logTag = 'trafficSignals'
local instances, controllers, sequences, mapNodeSignals, signalObjectsDict, controllerDefinitions = {}, {}, {}, {}, {}, {}
local instancesByName, controllersByName, sequencesByName, elementsById = {}, {}, {}, {}
local instanceColorKeys = {'instanceColor', 'instanceColor1', 'instanceColor2', 'instanceColor3'}
local lightOff, lightOn = '0 0 0 1', '1 1 1 1'
local timer = -1
local loaded = false
local active = false
local viewDistSq = 90000
local vecUp = vec3(0, 0, 1)

local queue = require('graphpath').newMinheap() -- optimally processes signal updates
local signalUpdates = {}

local debugColors = {
  main = ColorF(1, 1, 1, 0.4),
  selected = ColorF(1, 1, 0.25, 0.4),
  guide = ColorF(0.25, 1, 0.25, 0.4),
  error = ColorF(1, 0.25, 0.25, 0.4),
  textMain = ColorF(0, 0, 0, 1),
  textError = ColorF(1, 0, 0, 1),
  textLabel = ColorF(1, 1, 1, 1),
  textBox1 = ColorI(0, 0, 0, 128),
  textBox2 = ColorI(0, 0, 0, 255)
}

local debugPos, debugTarget = vec3(), vec3()

M.useDtReal = false
M.debugLevel = 0

local _uid = 0
local function getNextId() -- increments and generates a new id for the next element
  _uid = _uid + 1
  return _uid
end

local function getElementById(id) -- returns the element (instance, controller, or sequence) via its unique id
  return elementsById[id or 0]
end

local function setupSignalObjects() -- sets and caches the table of TSStatics that are used as signals
  table.clear(signalObjectsDict)

  local statics = getObjectsByClass('TSStatic')
  if statics then
    for _, obj in ipairs(statics) do -- search for static objects with dynamic data
      local instanceName = obj.signalInstance -- signal instance dynamic field
      if instanceName then
        signalObjectsDict[instanceName] = signalObjectsDict[instanceName] or {}
        table.insert(signalObjectsDict[instanceName], obj:getId())
      end
    end
  end
end

local function resetControllerDefinitions() -- resets vital controller definitions back to default
  controllerDefinitions = jsonReadFile('settings/trafficSignals.json')
  if not controllerDefinitions then
    controllerDefinitions = {states = {}, types = {}}
    log('E', logTag, 'Failed to load default signal controller definitions!') -- this should never happen
  end

  controllerDefinitions.signalActions = { -- signal actions enumeration (send index num to vLua) (incomplete)
    alert = 1,
    stop = 2,
    briefStop = 3,
    slow = 4
  }
  controllerDefinitions.signalColors = { -- mimic colors based on real world, used for debug draws (not actual signal objects, which use their own palettes)
    red = ColorF(1, 0.2, 0, 1),
    amber = ColorF(1, 0.6, 0, 1),
    yellow = ColorF(1, 0.8, 0, 1),
    green = ColorF(0, 1, 0.5, 1),
    cyan = ColorF(0, 1, 0.9, 1),
    blue = ColorF(0.3, 0.8, 1, 1),
    white = ColorF(1, 1, 1, 1),
    black = ColorF(0, 0, 0, 1)
  }
end
resetControllerDefinitions()

---- Traffic signal classes ----

-- Signal Instance
-- Contains a single signal object with a position, direction, controller id, and sequence id
local SignalInstance = {}
SignalInstance.__index = SignalInstance

-- Signal Controller
-- Contains signal type and state data (types include traffic lights, stop signs, etc.)
-- This can be used within multiple signal sequences
local SignalController = {}
SignalController.__index = SignalController

-- Signal Sequence
-- Assigns controllers to phases, which run in a sequence and can simulate an intersection
-- This can be applied to multiple signal instances (for example, four signal instances at an intersection share the same sequence)
local SignalSequence = {}
SignalSequence.__index = SignalSequence

-- Note: Controllers and sequences are reusable; signal instances can share the same controller or sequence

function SignalInstance:new(data)
  local o = {}
  data = data or {}
  setmetatable(o, self)

  o.id = data.id or getNextId()
  o.name = data.name or 'signal'
  o.pos = data.pos or vec3()
  o.dir = data.dir or vec3()
  o.radius = data.radius or 5 -- radius around the signal point
  o.group = data.group -- optional intersection group
  o.controllerId = data.controllerId or 0 -- controller is required
  o.sequenceId = data.sequenceId or 0 -- sequence is optional, usually used for dynamic signals (traffic lights)
  o.useCurrentLane = data.useCurrentLane -- unused at the moment (lane data is unsupported)
  o.startDisabled = data.startDisabled and true or false -- if disabled, signal will not run when system gets activated

  if o.pos:squaredLength() == 0 and core_camera then -- if pos is (0, 0, 0), try to set it to camera position
    o.pos = vec3(core_camera.getPosition())
  end

  if o.dir:squaredLength() == 0 then -- if dir is (0, 0, 0), try to automatically calculate it based on the closest road
    o.road = o:getBestRoad()
    if o.road then
      o.dir = vec3(o.road.dir)
    end
  end

  return o
end

function SignalInstance:setLights(lightsArray) -- directly sets the lights of all linked signal objects (signal state does not change)
  -- generally, this is only used internally
  if not self.linkedObjects then return end
  lightsArray = lightsArray or {'black', 'black', 'black'}

  for _, id in ipairs(self.linkedObjects) do -- actual traffic signal objects
    local obj = scenetree.findObjectById(id)
    if obj then
      for i, light in ipairs(lightsArray) do
        obj:setField(instanceColorKeys[i], '0', light == 'black' and lightOff or lightOn)
      end
    end
  end
end

function SignalInstance:setStrictState(stateIdx) -- manually sets the signal state from the given controller index (overrides auto state)
  -- recommended to use this after disabling the sequence timer; otherwise, the timer will interfere
  -- alternatively, use mySignal:getSequence():setStrictState(ctrlName, stateIdx)
  -- or mySignal:getSequence():setPhase(phaseIndex)
  -- or mySignal:getSequence():setStep(stepIndex)
  local ctrl = self:getController()
  if ctrl then
    self.priorityStateIndex = stateIdx
    if stateIdx and stateIdx > 0 then
      signalUpdates[self.name] = ctrl.states[stateIdx] and ctrl.states[stateIdx].state or 'none'
    else
      self:setActive(true) -- if stateIdx is nil, reset the signal to its expected state
    end
  end
end

function SignalInstance:getSignalObjects(refresh) -- returns table of objects that are used as signals, as ids
  if refresh or not next(signalObjectsDict) then setupSignalObjects() end
  return signalObjectsDict[self.name] or {}
end

function SignalInstance:clearSignalObjects(alsoDelete) -- unlinks world objects from this signal instance (optionally deletes them)
  local objData = self:getSignalObjects(true)
  for _, objId in pairs(objData) do
    if scenetree.objectExists(objId) then
      if alsoDelete then
        scenetree.findObjectById(objId):delete()
      else
        scenetree.findObjectById(objId):setDynDataFieldbyName('signalInstance', 0, '')
      end
    end
  end
end

function SignalInstance:linkSignalObject(objId) -- links world objects that will be synced to this signal instance (typically traffic lights)
  if scenetree.objectExists(objId) then
    scenetree.findObjectById(objId):setDynDataFieldbyName('signalInstance', 0, self.name)
  end
end

function SignalInstance:unlinkSignalObject(objId) -- unlinks world objects from this signal instance
  if scenetree.objectExists(objId) then
    scenetree.findObjectById(objId):setDynDataFieldbyName('signalInstance', 0, '')
  end
end

function SignalInstance:createSignalObject(shapeFile, pos, rot) -- creates and processes a signal object for this signal instance
  shapeFile = shapeFile or 'art/shapes/objects/trafficlight_overhead1.dae'
  if not FS:fileExists(shapeFile) then
    local artDir = path.split(getMissionFilename()) or '' -- first, search the current level assets
    artDir = artDir..'art/shapes/'
    for _, f in ipairs(FS:findFiles(artDir, '*', -1, true, false)) do
      if string.find(f, shapeFile) then
        shapeFile = f
        break
      end
    end

    if not FS:fileExists(shapeFile) then
      artDir = 'art/shapes/' -- if not found, then search the common assets
      for _, f in ipairs(FS:findFiles(artDir, '*', -1, true, false)) do
        if string.find(f, shapeFile) then
          shapeFile = f
          break
        end
      end
    end

    if not FS:fileExists(shapeFile) then
      log('E', logTag, 'Unable to find given shape file!')
      return
    end
  end
  pos = pos or core_camera.getPosition()
  rot = rot or core_camera.getQuat()

  local obj = createObject('TSStatic')
  obj.useInstanceRenderData = false
  obj:setField('shapeName', 0, shapeFile)
  obj:setField('dynamic', 0, 'true')
  obj:setField('collisionType', 0, 'None')
  obj:setField('decalType', 0, 'None')
  obj:setField('annotation', 0, 'TRAFFIC_SIGNALS')
  obj:registerObject()

  local transform = QuatF(rot.x, rot.y, rot.z, rot.w):getMatrix()
  transform:setPosition(pos)
  obj:setTransform(transform)
  self:linkSignalObject(obj:getID())

  if scenetree.MissionGroup then
    local group = scenetree.AutoTrafficSignals
    if not group then
      group = createObject('SimGroup')
      group:registerObject('AutoTrafficSignals')
      group.canSave = false -- unsure if this should be true or false
      scenetree.MissionGroup:addObject(group.obj)
    end

    scenetree.AutoTrafficSignals:addObject(obj.obj)
  end

  return obj
end

function SignalInstance:setup(pos, dir, controllerId, sequenceId) -- alternative way to set up a signal; creates controller and sequence if needed
  self.pos = pos or self.pos
  self.dir = dir or self.dir
  self.road = self:getBestRoad()

  self.controllerId = controllerId or 0
  if not self.controllerId or not elementsById[self.controllerId] or not elementsById[self.controllerId].states then
  -- if creating a new controller, it will use the default signal type, "lightsBasic"
    self.controller = SignalController:new()
    self.controller:applyDefinition('lightsBasic')
    self.controllerId = self.controller.id
  end

  self.sequenceId = sequenceId or 0
  if not self.sequenceId or not elementsById[self.sequenceId] or not elementsById[self.sequenceId].phases then
    self.sequence = SignalSequence:new()
    self.sequenceId = self.sequence.id
  end
end

function SignalInstance:getBestRoad(pos, dir) -- gets data from the best road near the given position
  pos = pos or self.pos
  local n1, n2, dist = map.findClosestRoad(pos)
  if not n1 then
    return
  else
    local mapNodes = map.getMap().nodes

    local p1, p2 = mapNodes[n1].pos, mapNodes[n2].pos
    local nDir = (p2 - p1):normalized()
    local link = mapNodes[n1].links[n2] or mapNodes[n2].links[n1]
    if link and link.oneWay then -- if the road is one way, match the direction of the road
      if n1 ~= link.inNode then
        n1, n2 = n2, n1
        p1, p2 = p2, p1
        nDir:setScaled(-1)
      end
    else -- otherwise, match the direction according to the side of the road
      dir = dir or vec3(self.dir)
      if dir:squaredLength() == 0 then
        dir = vec3(nDir)
        dir:setScaled(sign2(pos:xnormOnLine(p1, p1 + nDir:cross(vecUp))))
      end
      if nDir:dot(dir) < 0 then
        n1, n2 = n2, n1
        p1, p2 = p2, p1
        nDir:setScaled(-1)
      end
    end

    local xnorm = pos:xnormOnLine(p1, p2)
    local nPos = linePointFromXnorm(p1, p2, xnorm)
    local radius = lerp(mapNodes[n1].radius, mapNodes[n2].radius, xnorm)

    return {n1 = n1, n2 = n2, dist = dist, pos = nPos, dir = nDir, radius = radius}
  end
end

function SignalInstance:getVehPlacement(vehId) -- returns vehicle placement data in relation to the signal
  -- (TW) NOTE: I feel like this could be improved; the detection fails in a few cases
  local veh = getObjectByID(vehId)
  if not veh then return end

  local valid = false
  local vehPos, vehDir = veh:getPosition(), veh:getDirectionVector()
  -- direction vector check will fail if the vehicle is driving in reverse
  local dist = vehPos:distance(self.pos)
  local vehDot = vehDir:dot(self.dir)
  if dist <= math.max(5, self.radius) and vehDot > 0.707 then -- should also check for current road
    valid = true
  end
  local relDist = vehPos:xnormOnLine(self.pos, self.pos + self.dir) -- negative while before signal point

  return {valid = valid, dot = vehDot, dist = dist, relDist = relDist}
end

function SignalInstance:isVehAfterSignal(vehId) -- returns true if the vehicle has passed the signal point in a valid way
  local data = self:getVehPlacement(vehId)
  if not data then return false end

  local valid = false
  if data.valid and data.relDist >= 0 then
    valid = true
  end
  return valid, data.relDist
end

function SignalInstance:calcTargetPos() -- calculates the position at the end of the signal vector (and also the navgraph node)
  if not self.road then
    self.road = self:getBestRoad()
    if not self.road then return end
  end

  self.targetPos = self.targetPos or (self.pos + self.dir * self.radius)
  local road = self:getBestRoad(self.targetPos)
  self.road.n3 = road and road.n2 -- n3 = the target node at the end point
end

function SignalInstance:calcIntersectionPos() -- calculates the intersection midpoint for the signal (signal system must be active)
  -- this only resolves if the signal system is active, so that other signal instances can be queried
  local vectors = {}
  local count = 0
  local refPos = self.pos + self.dir * math.max(3, self.radius) -- ideally, the radius should reach to somewhere near the intersection center
  for _, instance in ipairs(instances) do
    -- intersection requirements here
    if self.sequenceId == instance.sequenceId then
      if instance.pos:squaredDistance(refPos) <= 1600 then -- within 40 m of the reference position (safe enough?)
        if instance.dir:dot(refPos - instance.pos) > 0 then -- facing the same direction
          vectors[instance.name] = instance.pos + instance.dir * instance.radius
          count = count + 1
        end
      end
    end
  end

  if count >= 2 then
    self.targetPos = vec3()
    for _, v in pairs(vectors) do
      self.targetPos:setAdd(v)
    end
    self.targetPos:setScaled(1 / count)

    local road = self:getBestRoad(self.targetPos) -- gets the ideal node of the intersection
    if road then
      local mapNodes = map.getMap().nodes
      if mapNodes[road.n1].pos:squaredDistance(self.targetPos) > mapNodes[road.n2].pos:squaredDistance(self.targetPos) then
        road.n1, road.n2 = road.n2, road.n1
      end
      if self.road then
        self.road.n3 = road.n1 -- n3 = the target node at the intersection point
      end
    end

    for k, _ in pairs(vectors) do -- sets the same intersection data for all matching instances
      instancesByName[k].targetPos = self.targetPos
      instancesByName[k].intersectionId = self.id -- uses the current instance id as the intersection id
      if instancesByName[k].road and self.road and self.road.n3 then
        instancesByName[k].road.n3 = self.road.n3
      end
    end
  end
end

function SignalInstance:drawDebug(isEditMode, isSelected, showText, capColor, cylinderScl, arrowScl) -- debug draws shapes for this instance
  capColor = capColor or debugColors.error
  cylinderScl = cylinderScl or 5
  arrowScl = arrowScl or self.radius

  debugPos:set(self.pos)
  debugPos.z = debugPos.z + cylinderScl
  if cylinderScl > 0 then
    debugDrawer:drawCylinder(self.pos, debugPos, 0.06, isSelected and debugColors.selected or debugColors.main)
  end

  if arrowScl > 0 then
    debugTarget:setScaled2(self.dir, arrowScl)
    debugTarget:setAdd2(self.pos, debugTarget)
    debugDrawer:drawSquarePrism(self.pos, debugTarget, Point2F(0.5, arrowScl * 0.3), Point2F(0.5, 0), debugColors.guide)
  end

  if isEditMode then
    local str = self.name
    if isSelected then
      local ctrl = self:getController()
      local ctrlName = ctrl and ctrl.name or "(Null controller)"

      local seq = self:getSequence()
      local seqName = seq and seq.name or "(Null Sequence)"

      str = str.." / "..ctrlName.." / "..seqName
    end

    if showText then
      debugDrawer:drawTextAdvanced(self.pos, String(str), debugColors.textLabel, true, false, isSelected and debugColors.textBox2 or debugColors.textBox1)
    end

    debugDrawer:drawSphere(debugPos, 0.3, capColor)

    if isSelected then
      debugDrawer:drawSphere(self.pos, 0.6, debugColors.main)

      local signalObjects = self.linkedObjects
      if not signalObjects or not signalObjects[1] then
        signalObjects = self.tempSignalObjects or {} -- tempSignalObjects property may exist while in the editor
      end

      for _, oid in ipairs(signalObjects) do
        local obj = scenetree.findObjectById(oid)
        if obj then
          debugPos:set(obj:getWorldBox():getCenter())
          debugPos.z = debugPos.z + obj:getWorldBox():getExtents().z * 0.5 + 0.25
          debugTarget:set(debugPos)
          debugTarget.z = debugTarget.z + 1
          debugDrawer:drawSquarePrism(debugPos, debugTarget, Point2F(0, 0), Point2F(0.5, 0.5), debugColors.selected)
        end
      end
    end
  else
    local stateName, stateData = self:getState()

    local lightsTbl = stateData.lights or {}
    debugPos.z = debugPos.z + #lightsTbl * 0.5

    if showText then
      if self._invalid then
        debugDrawer:drawText(self.pos, String(self.name..' (ERROR)'), debugColors.textError)
      else
        debugDrawer:drawText(self.pos, String(self.name), debugColors.textMain)
      end
    end

    for _, light in ipairs(lightsTbl) do
      debugDrawer:drawSphere(debugPos, 0.25, controllerDefinitions.signalColors[light] or controllerDefinitions.signalColors.white)
      debugPos.z = debugPos.z - 0.5
    end

    if showText then
      debugDrawer:drawText(debugPos, String('state: '..(stateName or 'none')), debugColors.textMain)
    end
  end
end

function SignalInstance:getController()
  return elementsById[self.controllerId]
end

function SignalInstance:getSequence()
  return elementsById[self.sequenceId]
end

function SignalInstance:getStateAfterTime(seconds, isAbsolute) -- returns the state name and data of the signal after the given time (signal system must be active)
  -- isAbsolute uses the start of the sequence, ignoring the current time
  local controller = self:getController()
  local sequence = self:getSequence()
  local state = 'none'
  local stateData = controllerDefinitions.states.none
  seconds = seconds or 0 -- should negative time be supported?

  if controller and self.active then
    local stateIdx = 1
    if self.priorityStateIndex then -- priority state index overrides sequence
      stateIdx = self.priorityStateIndex
    elseif sequence and sequence.sequenceTimings and sequence.sequenceTimings[1] then -- sequence is optional, otherwise the default state will be used
      local step = sequence.currStep
      local duration = sequence.stateTime - timer -- delta time until next step
      if isAbsolute then
        duration = sequence.sequenceTimings[1] - sequence.startTime -- delta time from the start of the sequence
      end

      local tempControllers = seconds <= duration and sequence.linkedControllers or deepcopy(sequence.linkedControllers) -- deepcopy if changes are expected

      while seconds > duration do -- loop until the given time is less than the duration
        seconds = seconds - duration

        step = step + 1
        if not sequence.sequenceTimings[step] then
          step = 1
        end

        local t1 = sequence.sequenceTimings[step]
        local t2 = sequence.sequenceTimings[step + 1] or sequence.sequenceDuration
        duration = t2 - t1

        local stepData = sequence.sequence[sequence.sequenceTimings[step]] or {}

        for _, inner in ipairs(stepData) do -- inner table: {controllerName, phaseIndex, stateIndex}
          local seqCtrl = tempControllers[inner[1]]
          if seqCtrl then
            seqCtrl.stateIdx = inner[3] or 1
          end
        end
      end

      local seqCtrl = tempControllers[controller.name]
      if seqCtrl then
        stateIdx = seqCtrl.stateIdx
      end
    end
    if controller.states[stateIdx] then
      state = controller.states[stateIdx].state
      stateData = controller:getStateData(state)
    end
  end

  return state, stateData
end

function SignalInstance:getState() -- returns the state name and data of the signal (signal system must be active)
  return self:getStateAfterTime()
end

function SignalInstance:refresh() -- updates running controllers and sequences linked to this instance
  if not loaded then return end

  self:setActive(not self.startDisabled) -- this resets and updates the expected signal state
  local sequence = self:getSequence()
  if sequence then
    local currStep = sequence.currStep
    local ignoreTimer = sequence.ignoreTimer
    sequence:setActive(not sequence.startDisabled)
    sequence:enableTimer(not ignoreTimer)
    if currStep > 0 then sequence:setStep(currStep) end -- jumps to the saved step
  end
end

function SignalInstance:setController(id) -- sets the controller to use, and refreshes the system, if applicable
  self.controllerId = id or 0
  if active then self:refresh() end
end

function SignalInstance:setSequence(id) -- sets the sequence to use, and refreshes the system, if applicable
  self.sequenceId = id or 0
  if active then self:refresh() end
end

function SignalInstance:setActive(val) -- sets the active state of the signal (boolean)
  self.active = val and true or false
  if self.active then
    if self:getController() then
      local valid = true
      if not self:getSequence() then
        valid = false
      else
        local sequence = self:getSequence()
        local controller = self:getController()
        if sequence.linkedControllers and sequence.linkedControllers[controller.name] then
          local lc = sequence.linkedControllers[controller.name]
          signalUpdates[self.name] = lc.controller.states[lc.stateIdx].state
        else
          valid = false
        end
      end

      if valid then
        self.priorityStateIndex = nil
      else
        self.priorityStateIndex = self:getController().defaultIndex or 1 -- priority state index is permanent, unless a sequence update resolves
      end

      if not self.intersectionId then
        self:calcIntersectionPos() -- also tries to calculate target pos
      end
      if not self.targetPos then
        self:calcTargetPos()
      end

      local stateName, stateData = self:getState()
      signalUpdates[self.name] = stateName
      if stateData and stateData.flashingLights then
        stateData.flashingActive = true
        stateData.lightIdx = 0
        stateData.lightTime = -math.huge
      end
    end
  else
    signalUpdates[self.name] = 'none'
  end
end

function SignalInstance:setAuxiliaryData() -- sets and validates additional data
  local valid = true
  self._invalid = nil

  self.road = self:getBestRoad()
  if not self.road then
    if not self._invalid then
      log('E', logTag, 'Map node could not be set for signal instance: '..self.name)
    end
    valid = false
  end

  if not self:getController() then
    if not self._invalid then
      log('E', logTag, 'Controller is missing from signal instance: '..self.name)
    end
    valid = false
  end

  if not valid then
    self._invalid = true
    self.linkedObjects = {}
  else
    self.linkedObjects = self:getSignalObjects()
  end
end

function SignalInstance:include() -- adds the signal instance to the main data
  if not instancesByName[self.name] then
    table.insert(instances, self)
    instancesByName[self.name] = self
    elementsById[self.id] = self

    -- if controller or sequence objects directly exist within this instance, include them as well
    if self.controller and not controllersByName[self.controller.name] then
      self.controller:include()
    end
    if self.sequence and not sequencesByName[self.sequence.name] then
      self.sequence:include()
    end
  end
end

function SignalInstance:exclude() -- removes the signal instance from the main data
  if instancesByName[self.name] then
    for i, instance in ipairs(instances) do
      if instance.name == self.name then
        table.remove(instances, i)
        break
      end
    end
    instancesByName[self.name] = nil
    elementsById[self.id] = nil
  end
end

function SignalInstance:onSerialize()
  local data = {
    id = self.id,
    name = self.name,
    pos = self.pos:toTable(),
    dir = self.dir:toTable(),
    radius = self.radius,
    group = self.group,
    controllerId = self.controllerId,
    sequenceId = self.sequenceId,
    useCurrentLane = self.useCurrentLane,
    startDisabled = self.startDisabled
  }

  return data
end

function SignalInstance:onDeserialized(data)
  data = data or {}

  for k, v in pairs(data) do
    self[k] = v
  end
  self.pos = vec3(self.pos)
  self.dir = vec3(self.dir)
end

function SignalController:new(data)
  local o = {}
  data = data or {}
  setmetatable(o, self)

  o.id = data.id or getNextId()
  o.name = data.name or 'controller'
  o.type = data.type or 'none'
  o.isSimple = data.isSimple and true or false
  o.defaultIndex = data.defaultIndex
  o.states = data.states or {}

  return o
end

function SignalController:applyDefinition(key) -- copies and sets the controller definition data (signal type) to the controller
  key = key or 'none'
  self.type = key
  table.clear(self.states)

  local data = controllerDefinitions.types[key]
  if data then
    self.isSimple = data.isSimple and true or false
    self.defaultIndex = data.defaultIndex
    if data.states then
      for _, state in ipairs(data.states) do
        local stateData = self:getStateData(state)
        if stateData then
          table.insert(self.states, {state = state, duration = stateData.duration})
        else
          log('E', logTag, 'Controller definition state not found: '..state)
        end
      end
    end
  else
    log('E', logTag, 'Controller definition type not found: '..key)
  end
end

function SignalController:getStateData(state) -- returns the states table
  return controllerDefinitions.states[state]
end

function SignalController:autoSetTimings(speedMetresPerSecond, intersectionLength, isPermissive) -- automatically calculate and set basic timings of the signals
  -- currently, these timing values assume the "permissive yellow" rule
  -- http://onlinepubs.trb.org/Onlinepubs/trr/1985/1027/1027-005.pdf

  for _, state in ipairs(self.data.states) do
    if state.type == 'greenTrafficLight' or state.type == 'greenFlashingTrafficLight' then
      state.duration = clamp(speedMetresPerSecond * 0.7 + intersectionLength * 0.6, 10, 40) -- approximate values based on speed and intersection size
    elseif state.name == 'yellowTrafficLight' then
      state.duration = clamp(1 + (speedMetresPerSecond - 4.167) / 3.27, 3, 7) -- extended kinematic equation (3.27 = 9.81 * 0.333)
    elseif state.name == 'redTrafficLight' or state.type == 'redYellowTrafficLight' then
      state.duration = 1
    end
  end
end

function SignalController:include() -- adds the signal controller to the main data
  if not controllersByName[self.name] then
    table.insert(controllers, self)
    controllersByName[self.name] = self
    elementsById[self.id] = self
  end
end

function SignalController:exclude() -- removes the signal controller from the main data
  if controllersByName[self.name] then
    for i, ctrl in ipairs(controllers) do
      if ctrl.name == self.name then
        table.remove(controllers, i)
        break
      end
    end
    controllersByName[self.name] = nil
    elementsById[self.id] = nil
  end
end

function SignalController:onSerialize()
  local data = {
    id = self.id,
    name = self.name,
    type = self.type,
    isSimple = self.isSimple,
    defaultIndex = self.defaultIndex,
    states = self.states
  }

  return data
end

function SignalController:onDeserialized(data)
  data = data or {}

  for k, v in pairs(data) do
    self[k] = v
  end
end

function SignalSequence:new(data)
  local o = {}
  data = data or {}
  setmetatable(o, self)

  o.id = data.id or getNextId()
  o.name = data.name or 'sequence'
  o.startTime = data.startTime or 0 -- can be negative
  o.phases = data.phases or {}
  o.customSequence = data.customSequence
  o.startDisabled = data.startDisabled and true or false
  o.ignoreTimer = data.ignoreTimer and true or false

  return o
end

function SignalSequence:createPhase(phase, idx) -- creates a new phase
  phase = phase or {}
  phase.controllerData = phase.controllerData or {}
  if idx then
    table.insert(self.phases, idx, phase)
  else
    table.insert(self.phases, phase)
  end
end

function SignalSequence:deletePhase(idx) -- removes a phase
  idx = idx or #self.phases
  if self.phases[idx] then
    table.remove(self.phases, idx)
  end
end

function SignalSequence:resolveUpdates(forceUpdate) -- updates all linked signal states and lights
  for _, lc in pairs(self.linkedControllers) do
    if lc._updated or forceUpdate then
      local stateBase = lc.controller.states[lc.stateIdx]
      for _, instance in pairs(lc.linkedSignals) do
        if instance.active then
          signalUpdates[instance.name] = stateBase.state
          instance.priorityStateIndex = nil
        end
      end
      lc._updated = nil
    end
  end
end

function SignalSequence:onSequenceUpdate() -- used internally whenever the sequence step updates
  -- processes next sequence timing
  local stepData = self.sequence[self.sequenceTimings[self.currStep]] or {}

  for _, inner in ipairs(stepData) do -- inner table: {controllerName, phaseIndex, stateIndex}
    self.currPhase = inner[2]
    local lc = self.linkedControllers[inner[1]]
    if lc then
      lc.stateIdx = inner[3]
      lc._updated = true

      if not self.ignoreTimer then
        local t1 = self.sequenceTimings[self.currStep]
        local t2 = self.sequenceTimings[self.currStep + 1] or self.sequenceDuration
        local duration = t2 - t1

        if self._deltaTime then
          duration = duration - self._deltaTime -- delta time adjustment, for accuracy
        end

        local time = timer + duration
        queue:insert(time, self.name)
        self.stateTime = time

        if self.enableTestTimer then
          self.testTimer = t1
        end
      end
    end
  end

  self:resolveUpdates()
end

function SignalSequence:resetSequence() -- resets all sequence data
  self.currStep = 0
  self.currPhase = 1

  for k, lc in pairs(self.linkedControllers) do
    lc.stateIdx = lc.controller.defaultIndex or 1
    lc._updated = true
  end
  self:resolveUpdates()

  local time = self.startTime + timer
  queue:insert(time, self.name)
  self.stateTime = time
end

function SignalSequence:setStep(index) -- manual setting of sequence step
  -- loops through the sequence until the sequence timing index is reached
  index = index or 1
  if not self.sequenceTimings[index] then return end

  local currStep = self.currStep

  repeat
    self.currStep = self.currStep + 1
    if not self.sequenceTimings[self.currStep] then
      self.currStep = 1
    end

    local stepData = self.sequence[self.sequenceTimings[self.currStep]]
    for _, inner in ipairs(stepData) do -- inner table: {controllerName, phaseIndex, stateIndex}
      local lc = self.linkedControllers[inner[1]]
      if lc then
        lc.stateIdx = inner[3]
        lc._updated = true
      end
    end
  until (self.currStep == index or self.currStep == currStep)

  self:onSequenceUpdate()
end

function SignalSequence:setPhase(index) -- manual setting of sequence phase
  -- loops through the phases until the phase index is reached
  index = index or 1
  if not self.phases[index] then return end

  local currStep = self.currStep
  local currPhase = self.currPhase
  if currPhase == index then
    self.currPhase = 0
  end

  repeat
    self.currStep = self.currStep + 1
    if not self.sequenceTimings[self.currStep] then
      self.currStep = 1
    end

    local stepData = self.sequence[self.sequenceTimings[self.currStep]]
    for _, inner in ipairs(stepData) do
      local lc = self.linkedControllers[inner[1]]
      if lc then
        lc.stateIdx = inner[3]
        lc._updated = true
        if currPhase ~= inner[2] then
          currPhase = inner[2]
          self.currPhase = inner[2]
        end
      end
    end
  until (self.currPhase == index or self.currStep == currStep)

  self:onSequenceUpdate()
end

function SignalSequence:advance() -- advances to next step
  self.currStep = self.currStep + 1
  if not self.sequenceTimings[self.currStep] then
    self.currStep = 1
  end

  self:onSequenceUpdate()
  self._deltaTime = nil
end

function SignalSequence:setStrictState(ctrlName, stateIdx) -- manually sets a controller state and updates all related signals (overrides auto state)
  -- for best results, use this after disabling the sequence timer; otherwise, the timer will interfere
  -- self:enableTimer(false)
  local lc = self.linkedControllers[ctrlName]
  if lc then
    stateIdx = stateIdx or lc.stateIdx
    local stateBase = lc.controller.states[stateIdx or 1]
    if stateBase then
      local stateRef = lc.controller:getStateData(stateBase.state)
      for _, instance in pairs(lc.linkedSignals) do
        instance:setLights(stateRef and stateRef.lights)
        signalUpdates[instance.name] = stateBase.state or 'none'
      end
    end
  end
end

function SignalSequence:setActive(val) -- (boolean) sets the active state of the signal sequence, and also runs setup
  self.active = val and true or false

  self.currStep = 1
  self.currPhase = 1
  self.stateTime = 0

  if self.active then
    table.clear(self.sequence)
    table.clear(self.linkedControllers)
    local totalTime = 0

    if self.customSequence then -- if custom sequence exists, use it instead of auto sequence
      self.sequence = self.customSequence
      self.sequenceTimings = tableKeysSorted(self.sequence)
      self:resetSequence()
      return
    end

    -- the following code builds internal sequence and linkedControllers tables, which are read and processed during the simulation
    for phaseIdx, phase in ipairs(self.phases) do
      local totalTimeUpdated = false
      local phaseTime = totalTime
      for _, cd in ipairs(phase.controllerData) do
        if elementsById[cd.id] then
          local controller = elementsById[cd.id]
          -- build reference table for controllers and signal instances
          self.linkedControllers[controller.name] = {controller = controller, stateIdx = 1, linkedSignals = {}}

          for _, instance in ipairs(instances) do
            if instance.sequenceId == self.id and instance.controllerId == cd.id then
              self.linkedControllers[controller.name].linkedSignals[instance.name] = instance
            end
          end

          local stateTime = phaseTime
          for stateIdx, state in ipairs(controller.states) do
            self.sequence[stateTime] = self.sequence[stateTime] or {}
            -- build sequence step, using the state change time as the key
            table.insert(self.sequence[stateTime], {controller.name, phaseIdx, stateIdx})

            local duration = state.duration
            if not duration or duration < 0 then
              duration = 1e6 -- adds a non-infinite duration so that the sequence can still be sorted
            end
            stateTime = stateTime + duration
            if cd.required then
              totalTime = math.max(totalTime, stateTime) -- totalTime should be less if requirements are passed
              totalTimeUpdated = true
            end

            local stateData = controller:getStateData(state.state)
            if stateData and stateData.flashingLights then
              stateData.flashingActive = true
              stateData.lightIdx = 0
              stateData.lightTime = -math.huge
            end
          end
        end
      end

      if not totalTimeUpdated then
        totalTime = totalTime + 1e6 -- adds a non-infinite duration so that the sequence can still be sorted
      end
    end

    self.sequenceTimings = tableKeysSorted(self.sequence) -- sequence timings table is used to order the sequence (by timestamp)
    self.sequenceDuration = totalTime
    --dump(self.sequence)
    --dump(self.sequenceTimings)

    self:resetSequence()

    if self.enableTestTimer then
      self.testTimer = 0
    end
  else
    for _, lc in pairs(self.linkedControllers) do
      for _, instance in pairs(lc.linkedSignals) do
        signalUpdates[instance.name] = 'none'
      end
    end

    table.clear(self.sequence)
    table.clear(self.sequenceTimings)
    table.clear(self.linkedControllers)
    self.sequenceDuration = 0
  end
end

function SignalSequence:enableTimer(val) -- (boolean) sets if this sequence responds to the main timer
  val = val and true or false
  self.ignoreTimer = not val
  if self.ignoreTimer and not val then -- invalidate queue
    self.stateTime = math.huge
  elseif not self.ignoreTimer and val then -- restart queue
    self:onSequenceUpdate()
  end
end

function SignalSequence:include() -- adds the signal sequence to the main data
  if not sequencesByName[self.name] then
    table.insert(sequences, self)
    sequencesByName[self.name] = self
    elementsById[self.id] = self
  end
end

function SignalSequence:exclude() -- removes the signal sequence from the main data
  if sequencesByName[self.name] then
    for i, sequence in ipairs(sequences) do
      if sequence.name == self.name then
        table.remove(sequences, i)
        break
      end
    end
    sequencesByName[self.name] = nil
    elementsById[self.id] = nil
  end
end

function SignalSequence:onSerialize()
  local data = {
    id = self.id,
    name = self.name,
    phases = self.phases,
    customSequence = self.customSequence,
    startTime = self.startTime,
    startDisabled = self.startDisabled,
    ignoreTimer = self.ignoreTimer
  }

  return data
end

function SignalSequence:onDeserialized(data)
  data = data or {}

  for k, v in pairs(data) do
    self[k] = v
  end
end

local function newInstance(data)
  return SignalInstance:new(data)
end

local function newController(data)
  return SignalController:new(data)
end

local function newSequence(data)
  return SignalSequence:new(data)
end

local function setControllerDefinitions(data) -- sets custom controller states and types (e.g. custom traffic light phase)
  if type(data) == 'table' then
    tableMerge(controllerDefinitions.states, data.states or {})
    tableMerge(controllerDefinitions.types, data.types or {})
  end
end

local function getControllerDefinitions() -- returns all controller definitions
  return controllerDefinitions
end

local function getInstances() -- returns the array of signal instances
  return instances
end

local function getMapNodeSignals() -- returns the processed dict of map nodes relating to signals
  return mapNodeSignals
end

local function getInstanceByName(name) -- returns the signal instance via the given name
  return instancesByName[name]
end

local function getControllers() -- returns the array of signal controllers
  return controllers
end

local function getControllerByName(name) -- returns the signal controller via the given name
  return controllersByName[name]
end

local function getSequences() -- returns the array of signal sequences
  return sequences
end

local function getSequenceByName(name) -- returns the signal sequence via the given name
  return sequencesByName[name]
end

local function getTimer() -- returns the elapsed time value
  return timer
end

local function getData(full) -- returns relevant data from this module
  local data = {
    instances = instances,
    controllers = controllers,
    sequences = sequences,
    loaded = loaded,
    active = active,
    timer = timer,
    nextTime = not queue:empty() and queue:peekKey() or 0
  }
  if full then
    data.instancesByName = instancesByName
    data.controllersByName = controllersByName
    data.sequencesByName = sequencesByName
    data.elementsById = elementsById
  end

  return data
end

local function resetTimer() -- resets the timer & queue, and activates the sequences
  timer = 0
  queue:clear()

  for _, sequence in ipairs(sequences) do
    sequence:setActive(not sequence.startDisabled)
  end
  for _, instance in pairs(instances) do
    if not instance._invalid then
      instance:setActive(not instance.startDisabled)
    end
  end
end

local function setTimer(val) -- directly sets the timer, which can instantly update the signal states
  if not val then resetTimer() end
  timer = val
end

local function buildMapNodeSignals() -- creates a reference dict, linking map nodes to signal instances and states
  -- this gets sent to VLua
  table.clear(mapNodeSignals)

  for _, instance in ipairs(instances) do
    if not instance._invalid then
      local stateName, stateData = instance:getState()
      local n1, n2, n3 = instance.road.n1, instance.road.n2, instance.road.n3
      mapNodeSignals[n1] = mapNodeSignals[n1] or {}
      mapNodeSignals[n1][n2] = mapNodeSignals[n1][n2] or {}
      -- array containing one or more signal data points
      -- just in case two or more signals exist on the same road segment
      table.insert(mapNodeSignals[n1][n2], {instance = instance.name, pos = instance.pos, target = n3, useLane = instance.useCurrentLane, state = stateName, action = controllerDefinitions.signalActions[stateData.action or 'none'] or 0})
      instance._innerIdx = #mapNodeSignals[n1][n2]
    end
  end
end

local function runSignals() -- finishes signal setup and activates main logic
  if instances[1] then
    for _, instance in pairs(instances) do
      instance.active = false
      instance:setAuxiliaryData()
    end

    for _, sequence in ipairs(sequences) do
      sequence.sequence = {}
      sequence.sequenceTimings = {}
      sequence.linkedControllers = {}
    end

    resetTimer()
    buildMapNodeSignals()
    table.clear(signalObjectsDict) -- optimizes memory usage by clearing this table, as it is not used during runtime
    loaded = true
    active = true
  end
end

local function setActive(val, autoRun) -- sets the timer active state
  -- autoRun is false when the level is loaded or the navgraph is reloaded; in this case, runSignals gets called when the navgraph is ready
  -- this is a problem when loading signals from a file, as runSignals will not get called unless the navgraph is reloaded
  -- a clean solution here would be appreciated
  if autoRun and not loaded then
    runSignals()
    if not loaded then
      log('W', logTag, 'Unable to change active state of signals, due to no signals data')
      return
    end
  end

  active = val and true or false
end

local function setLightsManual(id, stateArray) -- directly sets the visible lights of a traffic signal object (will not affect actual state)
  -- generally unused; see SignalInstance:setStrictState
  local obj = scenetree.findObjectById(id)
  stateArray = stateArray or {false, false, false}
  if obj then
    for i, state in ipairs(stateArray) do
      obj:setField(instanceColorKeys[i], '0', state and lightOn or lightOff)
    end
  end
end

local function setupSignals(data, merge) -- processes and enables the signals system; can merge with existing ones
  if not be then return end
  loaded = false
  active = false

  table.clear(instancesByName)
  table.clear(controllersByName)
  table.clear(sequencesByName)
  table.clear(elementsById)

  if data then
    setupSignalObjects()

    if merge then -- merges existing and new data
      arrayConcat(instances, data.instances or {})
      arrayConcat(controllers, data.controllers or {})
      arrayConcat(sequences, data.sequences or {})
    else
      instances = data.instances or {}
      controllers = data.controllers or {}
      sequences = data.sequences or {}
    end

    local delInstances, delControllers, delSequences = {}, {}, {}
    for i, v in ipairs(instances) do
      if not elementsById[v.id] then
        instancesByName[v.name] = v
        elementsById[v.id] = v
      else
        table.insert(delInstances, i)
      end
    end
    for i, v in ipairs(controllers) do
      if not elementsById[v.id] then
        controllersByName[v.name] = v
        elementsById[v.id] = v
      else
        table.insert(delControllers, i)
      end
    end
    for i, v in ipairs(sequences) do
      if not elementsById[v.id] then
        sequencesByName[v.name] = v
        elementsById[v.id] = v
      else
        table.insert(delSequences, i)
      end
    end

    -- remove invalid elements
    for i = #delInstances, 1, -1 do
      table.remove(instances, delInstances[i])
    end
    for i = #delControllers, 1, -1 do
      table.remove(controllers, delControllers[i])
    end
    for i = #delSequences, 1, -1 do
      table.remove(sequences, delSequences[i])
    end

    setActive(true)
  else
    table.clear(instances)
    table.clear(controllers)
    table.clear(sequences)
  end
end

local function resetSignals() -- clears signals
  setupSignals()
end

local function loadControllerDefinitions(filePath) -- loads default and custom controller definitions
  resetControllerDefinitions()

  if not filePath then
    local levelDir = path.split(getMissionFilename()) or ''
    filePath = levelDir..'signalControllerDefinitions.json'
  end

  if FS:fileExists(filePath) then
    local defs = jsonReadFile(filePath)
    if defs then
      setControllerDefinitions(defs)
      log('I', logTag, 'Custom signal controller definitions applied')
    end
  end
end

local function loadSignals(filePath) -- loads signals json file from given file path or default file path
  if not filePath then -- auto load signals, if they exist
    local levelDir = path.split(getMissionFilename()) or ''
    filePath = levelDir..'signals.json'
  end

  loadControllerDefinitions() -- auto load controller definitions, if they exist

  if FS:fileExists(filePath) then
    local data = jsonReadFile(filePath)
    if data then
      data.instances = data.instances or {}
      for i, v in ipairs(data.instances) do
        local new = SignalInstance:new()
        new:onDeserialized(v)
        data.instances[i] = new
      end
      data.controllers = data.controllers or {}
      for i, v in ipairs(data.controllers) do
        local new = SignalController:new()
        new:onDeserialized(v)
        data.controllers[i] = new
      end
      data.sequences = data.sequences or {}
      for i, v in ipairs(data.sequences) do
        local new = SignalSequence:new()
        new:onDeserialized(v)
        data.sequences[i] = new
      end

      setupSignals(data)
      return true
    end
  end

  return false
end

local function onUpdate(dt, dtSim)
  if not loaded then return end

  local dtUsed = M.useDtReal and dt or dtSim

  if active then
    for stateName, stateDef in pairs(controllerDefinitions.states) do
      if stateDef.flashingActive and stateDef.flashingLights then -- process flashing lights
        if timer >= stateDef.lightTime then
          stateDef.lightTime = timer + stateDef.flashingInterval
          stateDef.lightIdx = stateDef.flashingLights[stateDef.lightIdx + 1] and stateDef.lightIdx + 1 or 1
          stateDef.lights = stateDef.flashingLights[stateDef.lightIdx]

          for _, instance in ipairs(instances) do
            if instance:getState() == stateName then
              instance:setLights(stateDef.lights)
            end
          end
        end
      end
    end

    for _, sequence in ipairs(sequences) do
      if sequence.active and sequence.enableTestTimer and not sequence.ignoreTimer then -- optional self timer for sequences (for debug or inspection purposes)
        sequence.testTimer = sequence.testTimer or 0
        if sequence.currStep > 0 then
          sequence.testTimer = sequence.testTimer + dtUsed
        end
      end
    end

    -- whenever the timer is greater than the queue key, pops the queue and runs the sequence update (if valid)
    while not queue:empty() and queue:peekKey() <= timer do -- while loop is needed due to some timings happening at the same time
      local key, seqName = queue:pop()
      local sequence = sequencesByName[seqName]

      if sequence then
        if sequence.stateTime == key then -- key must match, otherwise ignore the signal update (this can happen if a signal was changed manually)
          sequence._deltaTime = timer - key -- ensures accuracy
          sequence:advance()
        end
      end
    end

    timer = timer + dtUsed
  end

  if next(signalUpdates) then -- runs whenever signal updates are queued (can be multiple per frame)
    local signalsData = {}
    local signalUpdateResults = {}

    for k, state in pairs(signalUpdates) do
      signalUpdateResults[k] = {state = state}
      local instance = instancesByName[k]
      if instance and instance._innerIdx then
        local stateData = instance:getController():getStateData(state)
        local stateAction = stateData and stateData.action or 'none'
        signalUpdateResults[k].linkedObjects = instance.linkedObjects
        signalUpdateResults[k].stateAction = stateAction

        if state ~= 'none' then
          instance:setLights(stateData and stateData.lights)
          if stateData then
            signalUpdateResults[k].lightColors = stateData.lights
            signalUpdateResults[k].lightStates = {}
            for _, light in ipairs(stateData.lights) do
              table.insert(signalUpdateResults[k].lightStates, light ~= 'black')
            end
          end
        else
          instance:setLights()
        end
        local n1, n2 = instance.road.n1, instance.road.n2
        local actionNum = controllerDefinitions.signalActions[stateAction] or 0

        mapNodeSignals[n1][n2][instance._innerIdx].state = state -- this might be redundant
        mapNodeSignals[n1][n2][instance._innerIdx].action = actionNum
        -- changeSet format {inNode, outNode, innerIndex, actionValue}
        table.insert(signalsData, n1)
        table.insert(signalsData, n2)
        table.insert(signalsData, instance._innerIdx)
        table.insert(signalsData, actionNum)
      end
    end

    be:sendToMailbox('trafficSignalUpdates', lpack.encodeBin(signalsData))
    extensions.hook('onTrafficSignalUpdate', signalUpdateResults)
    table.clear(signalUpdates)
  end

  if not M.debugLevel or M.debugLevel <= 0 then return end

  for _, instance in ipairs(instances) do
    if core_camera.getPosition():squaredDistance(instance.pos) <= viewDistSq then -- checks if camera is close enough to signal
      instance:drawDebug(false, false, M.debugLevel == 2, nil, nil, M.debugLevel == 2 and 2 or 0)
    end
  end
end

local function onNavgraphReloaded() -- reloads all signals (if system is active)
  -- important: always runs after onClientStartMission and onDeserialized
  if active then
    runSignals()
  end
end

local function onClientStartMission()
  loadSignals()
end

local function onClientEndMission()
  resetSignals()
end

local function onSerialize()
  local data = {}

  for k, e in pairs({instances = instances, controllers = controllers, sequences = sequences}) do
    data[k] = {}
    for _, v in ipairs(e) do
      table.insert(data[k], v:onSerialize())
    end
  end

  data.active = active
  data.loaded = loaded
  data.useDtReal = M.useDtReal
  data.debugLevel = M.debugLevel

  return data
end

local function onDeserialized(data)
  M.useDtReal = data.useDtReal
  M.debugLevel = data.debugLevel or 0
  loadControllerDefinitions()

  for _, v in ipairs(data.instances) do
    local res = SignalInstance:new()
    res:onDeserialized(v)
    res:include()
  end
  for _, v in ipairs(data.controllers) do
    local res = SignalController:new()
    res:onDeserialized(v)
    res:include()
  end
  for _, v in ipairs(data.sequences) do
    local res = SignalSequence:new()
    res:onDeserialized(v)
    res:include()
  end

  setActive(data.active)

  -- resolves the highest unique id
  _uid = 0
  for k, e in pairs({instances = instances, controllers = controllers, sequences = sequences}) do
    for i, v in ipairs(e) do
      if type(v.id) == 'number' then
        _uid = math.max(_uid, v.id)
      end
    end
  end
end

-- public interface
M.newSignal = newInstance
M.newController = newController
M.newSequence = newSequence

M.getSignals = getInstances
M.getSignalByName = getInstanceByName
M.getControllers = getControllers
M.getControllerByName = getControllerByName
M.getSequences = getSequences
M.getSequenceByName = getSequenceByName
M.getElementById = getElementById
M.getControllerDefinitions = getControllerDefinitions
M.getMapNodeSignals = getMapNodeSignals
M.getTimer = getTimer
M.getData = getData
M.getSignalsDict = nop

M.setupSignalObjects = setupSignalObjects
M.loadSignals = loadSignals
M.setupSignals = setupSignals
M.resetSignals = resetSignals
M.loadControllerDefinitions = loadControllerDefinitions
M.setControllerDefinitions = setControllerDefinitions
M.resetControllerDefinitions = resetControllerDefinitions
M.resetTimer = resetTimer
M.setTimer = setTimer
M.setActive = setActive
M.setLightsManual = setLightsManual

M.onNavgraphReloaded = onNavgraphReloaded
M.onUpdate = onUpdate
M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M