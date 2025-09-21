-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Groundmarkers'
C.icon = "navigation"
C.description = 'Creates markers that show the way to the chosen waypoint.'
C.category = 'repeat_instant'

C.color = im.ImVec4(1, 1, 0, 0.75)
C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'clear', description = "Reset the markers", impulse = true },
  { dir = 'in', type = { 'string', 'table', 'vec3' }, tableType = 'navgraphPath', name = 'target', description = "The target to navigate to. Can be a waypoint name, position or list of these things." },
  { dir = 'in', type = 'color', name = 'color', hidden = true, default = { 0.2, 0.53, 1, 1 }, hardcoded = true, description = "Color of the markings (DEPRECATED!)" },
  { dir = 'in', type = 'number', name = 'cutOffDrivability', hidden = true, description = "The drivability value, above which the road is penalized in planning (Optional)" },
  { dir = 'in', type = 'number', name = 'penaltyAboveCutoff', hidden = true, description = "The penalty above the cutoff drivability (Optional)" },
  { dir = 'in', type = 'number', name = 'penaltyBelowCutoff', hidden = true, description = "The penalty below the cutoff drivability (Optional)" },
  { dir = 'in', type = 'number', name = 'threshold', hidden = true, default = 0.1, description = "Threshold for checking if the target has changed", hardcoded = true },
}
C.legacyPins = {
  _in = {
    pos = 'target',
    waypoint = 'target',
    reset = 'clear'
  }
}
C.tags = { 'arrow', 'path', 'route', 'destination', 'navigation' }
C.dependencies = { 'core_groundMarkers' }

function C:init(mgr, ...)
  --self.data.step = 8
  self.data.fadeStart = 100
  self.data.fadeEnd = 150
  --self.data.color = { 0.2, 0.53, 1, 1 }
end

local options = {
  cutOffDrivability = nil,
  penaltyAboveCutoff = nil,
  penaltyBelowCutoff = nil
}

function C:hasNewTarget()
  if self.lastTarget == nil then
    return true
  end
  -- different types?
  if type(self.lastTarget) ~= type(self.pinIn.target.value) then
    --print("groundmarkers: different types: " .. type(self.lastTarget) .. " " .. type(self.pinIn.target.value))
    return true
  end
  -- same type, table?
  if type(self.lastTarget) == 'table' and type(self.lastTarget) == 'table' then
    -- different size?
    if #self.lastTarget ~= #self.pinIn.target.value then
      --print("groundmarkers: different size: " .. #self.lastTarget .. " " .. #self.pinIn.target.value)
      return true
    end

    -- check if all elements are the same
    local allSame = true
    for i = 1, #self.pinIn.target.value do
      local threshold = self.pinIn.threshold.value or 0.1
      if type(self.lastTarget[i]) == 'number' and type(self.pinIn.target.value[i]) == 'number' then
        if math.abs(self.lastTarget[i] - self.pinIn.target.value[i]) > threshold then
          --print("groundmarkers: different element: " .. self.lastTarget[i] .. " " .. self.pinIn.target.value[i])
          allSame = false
          break
        end
      else
        if self.lastTarget[i] ~= self.pinIn.target.value[i] then
          --print("groundmarkers: different element: " .. self.lastTarget[i] .. " " .. self.pinIn.target.value[i])
          allSame = false
          break
        end
      end
    end
    if not allSame then
      --print("groundmarkers: different elements")
    end
    return not allSame

  end
  if type(self.lastTarget) == 'string' and type(self.pinIn.target.value) == 'string' then
    if self.lastTarget ~= self.pinIn.target.value then
      --print("groundmarkers: different strings: " .. self.lastTarget .. " " .. self.pinIn.target.value)
    end
    return self.lastTarget ~= self.pinIn.target.value
  end

  return false
end

function C:work(args)
  if self.pinIn.clear.value then
    core_groundMarkers.resetAll()
    self.lastTarget = nil
  end

  if self.pinIn.flow.value and self.pinIn.target.value then
    local newTarget = self:hasNewTarget()
    --print("groundmarkers: " .. tostring(newTarget))
    if newTarget then
      local target = self.pinIn.target.value
      self.lastTarget = target
      if type(target) == 'table' and type(target[1]) == 'number' then
        target = vec3(target)
      end
      table.clear(options)
      if self.pinIn.cutOffDrivability.value then
        options.cutOffDrivability = self.pinIn.cutOffDrivability.value
      end
      if self.pinIn.penaltyAboveCutoff.value then
        options.penaltyAboveCutoff = self.pinIn.penaltyAboveCutoff.value
      end
      if self.pinIn.penaltyBelowCutoff.value then
        options.penaltyBelowCutoff = self.pinIn.penaltyBelowCutoff.value
      end
      core_groundMarkers.setPath(target, options)
    end
  end
end

function C:_executionStopped()
  if core_groundMarkers then
    core_groundMarkers.resetAll()
  end
end

function C:_executionStarted()
  self.lastTarget = nil
end

return _flowgraph_createNode(C)
