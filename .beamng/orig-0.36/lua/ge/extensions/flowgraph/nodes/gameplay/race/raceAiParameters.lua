-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Race AI Parameters'
C.description = 'Sets additional parameters for all AI vehicles.'
C.category = 'once_instant'
C.color = im.ImVec4(1, 1, 0, 0.75)
C.author = 'BeamNG'

C.pinSchema = {
  {dir = 'in', type = 'table', name = 'raceData', tableType = 'raceData',  description = 'Race data; this needs be active and have all vehicle states.'},
  {dir = 'in', type = 'number', name = 'aggression', description = 'Aggression value.'},
  {dir = 'in', type = 'number', name = 'racerSkill', description = 'General road driving skill (from 0 to 1).'},
  {dir = 'in', type = 'number', name = 'randomizationScale', description = 'Strength of randomization to apply to all values (from 0 to 1).'},
  {dir = 'in', type = 'bool', name = 'avoidCollisions', description = 'Enables AI awareness.'},
  {dir = 'in', type = 'bool', name = 'rubberband', description = 'Enables AI rubberbanding.'},
  {dir = 'in', type = 'bool', name = 'offroad', description = 'Enhances offroad driving (experimental).'}
}

C.tags = {'ai', 'race'}

function C:getRandomNumber(scale)
  scale = scale or 1
  return (math.random() * 2 - 1) * scale
end

function C:workOnce()
  if not self.pinIn.raceData.value then return end
  local scale = clamp(self.pinIn.randomizationScale.value or 0, 0, 2)
  local skill = clamp(self.pinIn.racerSkill.value or 1, 0, 1)

  for id, state in pairs(self.pinIn.raceData.value.states) do
    local veh = getObjectByID(id)
    if veh and not veh:isPlayerControlled() then
      local aggression = self.pinIn.aggression.value or 0.9
      aggression = clamp(aggression + (self:getRandomNumber(scale) * 0.05), 0.25, 2)
      veh:queueLuaCommand('ai.setAggression('..aggression..')')
      state.baseAggression = aggression -- saves the aggression value to use later

      -- TODO: test these values; the underlying AI code has changed since this was implemented
      local baseEdgeDist = 1.5 - skill * 1.5 -- distance from margin of road, in metres
      local baseTurnForce = 4 * math.pow(skill, 1.5) -- race line curvature
      local baseAwarenessForce = 0.25 * math.pow(skill, 1.5) -- racer avoidance strength

      local aiParams = {
        edgeDist = clamp(baseEdgeDist + (self:getRandomNumber(scale) * 0.25), 0, 1.5),
        turnForceCoef = clamp(baseTurnForce + (self:getRandomNumber(scale) * 2), 0.01, 10),
        awarenessForceCoef = clamp(baseAwarenessForce + (self:getRandomNumber(scale) * 0.15), 0.01, 0.5)
      }

      veh:queueLuaCommand('ai.setParameters('..serialize(aiParams)..')')
      veh:queueLuaCommand('ai.setRacing(true)') -- new mode, improves racing AI

      if self.pinIn.avoidCollisions.value ~= nil then
        local mode = self.pinIn.avoidCollisions.value and 'on' or 'off'
        veh:queueLuaCommand('ai.setAvoidCars("'..mode..'")')
      end
      if self.pinIn.rubberband.value ~= nil then
        local mode = self.pinIn.rubberband.value and 'rubberBand' or 'off'
        veh:queueLuaCommand('ai.setAggressionMode("'..mode..'")')
      end
      if self.pinIn.offroad.value ~= nil then
        local mode = self.pinIn.offroad.value and 'offRoad' or 'default'
        veh:queueLuaCommand('ai.setParameters({driveStyle = "'..mode..'"})')
      end
    end
  end
end

return _flowgraph_createNode(C)
