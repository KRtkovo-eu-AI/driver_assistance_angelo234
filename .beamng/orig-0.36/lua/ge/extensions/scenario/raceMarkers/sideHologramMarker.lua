-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
-- local basePrefix = "base_marker_"
local sidesPrefix = "cylinder_marker_"
local topColumnLeft = "art/shapes/interface/s_single_faded_column_rect.dae"
local topColumnRight = "art/shapes/interface/s_single_faded_column_rect.dae"
local pillarShapeLeft = "art/shapes/interface/s_mm_gate_pillar_left.dae"
local pillarShapeRight = "art/shapes/interface/s_mm_gate_pillar_right.dae"
local bottomConeShape = "art/shapes/interface/s_mm_gatecone.dae"
local arrowShape = "art/shapes/interface/s_mm_arrow_floating.dae"

local cylinderShape = "art/shapes/interface/track_editor_marker.dae" -- this is a 1x1x1 cylinder

local leftPillar, rightPillar, leftColumn, rightColumn, leftCone, rightCone, leftCylinder, rightCylinder, arrow

local modeInfos = {
  default = {
    pillarColor = {0, 0.4, 1},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
    -- baseColor = {1, 1, 1},
  },
  next = {
    pillarColor = {0.0, 0.0, 0.0},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 0.75,
    -- baseColor = {0, 0, 0},
  },
  start = {
    pillarColor = {0.4, 1, 0.2},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
    -- baseColor = {1, 1, 1},
  },
  lap = {
    pillarColor = {0.4, 1, 0.2},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
    -- baseColor = {1, 1, 1},
  },
  recovery = {
    pillarColor = {1, 0.85, 0},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
      -- baseColor = {1, 1, 1},
  },
  final = {
    pillarColor = {0.1, 0.3, 1},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
    -- baseColor = {1, 1, 1},
  },
  branch = {
    pillarColor = {1, 0.6, 0},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 1,
    -- baseColor = {1, 1, 1},
  },
  hidden = {
    pillarColor = {1, 1, 1},
    pillarColor1 = {1, 1, 1},
    alpha = 1,
    scalarThin = 0,
    -- baseColor = {0, 0, 0},
  }
}

local zVec = vec3(0,0,1)

local fadeNear = 50
local fadeFar = 80

local bobAmplitude = 0.05
local bobFrequency = 0.333
local bobOffset = vec3(0,0,0)
local CONE_HEIGHT = 1

local playerPosition = vec3(0,0,0)
local scale = vec3()
local columnOffset = vec3(0,0,7)

local function inverseLerp(min, max, value)
 if math.abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end

-- todo: replace this by a HSV-lerp if blending with non-gray colors
local lerpedColor = vec3()
local function lerpColor(a,b,t)
  lerpedColor:set(lerp(a[1],b[1],t), lerp(a[2],b[2],t), lerp(a[3],b[3],t))
  return lerpedColor
end

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init(id)
  self.id = id
  self.visible = false

  self.pos = nil
  self.radius = nil
  self.color = nil
  self.pillarColor = ColorF(1,1,1,1):asLinear4F()
  self.pillarColor1 = ColorF(1,1,1,1):asLinear4F()
  self.columnColor = ColorF(1,1,1,1):asLinear4F()
  self.coneColor = ColorF(1,1,1,1):asLinear4F()
  self.cylinderColor = ColorF(1,1,1,1):asLinear4F()
  self.arrowColor = ColorF(1,1,1,1):asLinear4F()
  -- self.colorBase = ColorF(1,1,1,1):asLinear4F()

  self.colorTimer = 0
  self.colorLerpDuration = 0.8
  self.minAlpha = 0.25

  self.fadeNear = fadeNear
  self.fadeFar = fadeFar

  self.modeInfos = deepcopy(modeInfos)

  self:clearMarkers()
  self.distant = nil
  self.normal = nil

  self.mode = 'hidden'
  self.oldMode = 'hidden'

  -- Store ground positions
  self.leftConePos = nil
  self.rightConePos = nil
  self.leftCylinderPos = nil
  self.rightCylinderPos = nil
  self.nextPos = nil
end

-- called every frame to update the visuals.
function C:update(dt, dtSim)
  self.colorTimer = self.colorTimer + dt
  if self.colorTimer >= self.colorLerpDuration then
    if self.mode == 'hidden' then
      self:hide()
    end
  end
  if not self.visible then return end

  -- Calculate bobbing offset
  local bobTime = self.colorTimer * bobFrequency * math.pi * 2
  bobOffset:set(0, 0, math.sin(bobTime) * bobAmplitude)

  playerPosition:set(core_camera.getPosition())
  local distanceFromMarker = self.pos:distance(playerPosition)

  -- old w color function to fade slightly on distance:
  -- clamp(inverseLerp(self.fadeNear,self.fadeFar,distanceFromMarker),0,0.75) + clamp(self.minAlpha, 0, 0.25)

  local t = clamp(self.colorTimer / self.colorLerpDuration,0,1)
  local alpha = lerp(self.modeInfos[self.oldMode or 'default'].alpha, self.modeInfos[self.mode or 'default'].alpha, t)
  -- instance color 1
  local color = lerpColor(self.modeInfos[self.oldMode or 'default'].pillarColor, self.modeInfos[self.mode or 'default'].pillarColor, t)
  self.pillarColor.x = color.x
  self.pillarColor.y = color.y
  self.pillarColor.z = color.z
  self.pillarColor.w = alpha

  -- Column color with same color as pillar, but fading out on distance
  self.columnColor.x = color.x
  self.columnColor.y = color.y
  self.columnColor.z = color.z
  self.columnColor.w = clamp(inverseLerp(60,120,distanceFromMarker),0.15,1) * alpha

  -- cone color same as pillar
  self.coneColor.x = color.x
  self.coneColor.y = color.y
  self.coneColor.z = color.z
  self.coneColor.w = self.pillarColor.w

  -- cylinder color same as pillar
  self.cylinderColor.x = color.x
  self.cylinderColor.y = color.y
  self.cylinderColor.z = color.z
  self.cylinderColor.w = self.pillarColor.w

  -- Arrow color same as pillar
  self.arrowColor.x = color.x
  self.arrowColor.y = color.y
  self.arrowColor.z = color.z
  self.arrowColor.w = alpha

  -- instance color 2
  color = lerpColor(self.modeInfos[self.oldMode or 'default'].pillarColor1, self.modeInfos[self.mode or 'default'].pillarColor1, t)
  self.pillarColor1.x = color.x
  self.pillarColor1.y = color.y
  self.pillarColor1.z = color.z
  self.pillarColor1.w = alpha

  local scalarThin = lerp(self.modeInfos[self.oldMode or 'default'].scalarThin, self.modeInfos[self.mode or 'default'].scalarThin, t)

  local sideRadius = math.max(0.125, distanceFromMarker*0.03)
  local sideHeight = clamp(inverseLerp(60,180,distanceFromMarker),0,20)+1 +clamp(inverseLerp(1800,2040,distanceFromMarker),0,20)

  leftPillar = scenetree.findObjectById(self.leftId)
  if leftPillar then
    leftPillar.instanceColor = self.pillarColor
    leftPillar.instanceColor1 = self.pillarColor1
    leftPillar:setPosition(self.leftPillarPos + bobOffset)
    leftPillar:setScale(vec3(scalarThin,scalarThin,1))
    leftPillar:updateInstanceRenderData()
  end
  rightPillar = scenetree.findObjectById(self.rightId)
  if rightPillar then
    rightPillar.instanceColor = self.pillarColor
    rightPillar.instanceColor1 = self.pillarColor1
    rightPillar:setPosition(self.rightPillarPos + bobOffset)
    rightPillar:setScale(vec3(scalarThin,scalarThin,1))
    rightPillar:updateInstanceRenderData()
  end

  leftCone = scenetree.findObjectById(self.leftConeId)
  if leftCone then
    leftCone.instanceColor = self.coneColor
    leftCone:setPosition(self.leftConePos)
    -- Scale cone based on bobbing offset
    local coneScale = 0 + (self.leftPillarOffset+bobOffset.z) / CONE_HEIGHT
    leftCone:setScale(vec3(scalarThin, scalarThin, coneScale))
    leftCone:updateInstanceRenderData()
  end
  rightCone = scenetree.findObjectById(self.rightConeId)
  if rightCone then
    rightCone.instanceColor = self.coneColor
    rightCone:setPosition(self.rightConePos)
    -- Scale cone based on bobbing offset
    local coneScale = 0 + (self.rightPillarOffset+bobOffset.z) / CONE_HEIGHT
    rightCone:setScale(vec3(scalarThin, scalarThin, coneScale))
    rightCone:updateInstanceRenderData()
  end

  leftColumn = scenetree.findObjectById(self.leftColumnId)
  if leftColumn then
    leftColumn.instanceColor = self.columnColor
    leftColumn:setPosition(self.leftConePos + columnOffset)
    scale:set(scalarThin, scalarThin, sideHeight)
    leftColumn:setScale(scale)
    leftColumn:updateInstanceRenderData()
  end
  rightColumn = scenetree.findObjectById(self.rightColumnId)
  if rightColumn then
    rightColumn.instanceColor = self.columnColor
    rightColumn:setPosition(self.rightConePos + columnOffset)
    scale:set(scalarThin, scalarThin, sideHeight)
    rightColumn:setScale(scale)
    rightColumn:updateInstanceRenderData()
  end

  leftCylinder = scenetree.findObjectById(self.leftCylinderId)
  if leftCylinder then
    leftCylinder.instanceColor = self.cylinderColor
    --leftCylinder:setPosition(self.leftCylinderPos)
    leftCylinder:setScale(vec3(0.1*scalarThin,0.1*scalarThin, self.leftConePos.z-self.leftCylinderPos.z))
    leftCylinder:updateInstanceRenderData()
  end
  rightCylinder = scenetree.findObjectById(self.rightCylinderId)
  if rightCylinder then
    rightCylinder.instanceColor = self.cylinderColor
    --rightCylinder:setPosition(self.rightCylinderPos)
    rightCylinder:setScale(vec3(0.1*scalarThin,0.1*scalarThin, self.rightConePos.z-self.rightCylinderPos.z))
    rightCylinder:updateInstanceRenderData()
  end

  arrow = scenetree.findObjectById(self.arrowId)
  if arrow and self.nextPos then
    arrow.instanceColor = self.arrowColor
    -- Position arrow 4m above waypoint
    local arrowPos = vec3(self.pos.x, self.pos.y, self.pos.z + 4)
    arrow:setPosition(arrowPos)

    -- Calculate direction to next position
    local dir = (self.nextPos - self.pos):normalized()
    local rot = quatFromDir(dir)
    local rotQuat = rot:toTorqueQuat()
    arrow:setField('rotation', 0, rotQuat.x .. ' ' .. rotQuat.y .. ' ' .. rotQuat.z .. ' ' .. rotQuat.w)
    arrow:updateInstanceRenderData()
  end
end

-- setting it to represent checkpoints. mode can be:
-- default (red, "normal" checkpoint)
-- branch (yellow, for branching paths)
-- next (black, the one after the current checkpoint)
-- lap (green, last cp in non-last lap)
-- finish (blue, last cp in last lap)
-- start (green, first cp when using rolling start)
function C:setToCheckpoint(wp)
  self.pos = vec3(wp.pos)
  self.radius = wp.radius
  self.normal = wp.normal and vec3(wp.normal) or vec3(1,1,0):normalized()
  self.side = (self.normal or vec3(1,0,0)):cross(vec3(0,0,1))
  self.nextPos = wp.nextPos and vec3(wp.nextPos) or nil

  self.fadeNear = wp.fadeNear or self.fadeNear
  self.fadeFar = wp.fadeFar or self.fadeFar
  self.minAlpha = wp.minAlpha or self.minAlpha

  -- Calculate side positions with ground raycasts
  local rot = quatFromDir(self.normal:z0())
  local up = vec3(0,0,1)
  local rayLengthUp, rayLengthDown = 6, -25

  -- Find positions on side
  local pLeft = self.pos + rot*(vec3(-self.radius,0,0))
  local pRight = self.pos + rot*(vec3(self.radius,0,0))

  -- Calculate original positions for comparison
  local originalLeftPos = self.pos - self.side * self.radius
  local originalRightPos = self.pos + self.side * self.radius

  -- Disable forest for raycasts
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end

  local hitLeft = Engine.castRay(pLeft+up*rayLengthUp, pLeft+up*rayLengthDown, true, false)
  local hitRight = Engine.castRay(pRight+up*rayLengthUp, pRight+up*rayLengthDown, true, false)

  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  -- Get ground positions
  self.leftConePos = hitLeft and vec3(hitLeft.pt) or originalLeftPos
  self.rightConePos = hitRight and vec3(hitRight.pt) or originalRightPos
  if originalLeftPos.z -  self.leftConePos.z  > 1 then self.leftConePos.z  = originalLeftPos.z end
  if originalRightPos.z - self.rightConePos.z > 1 then self.rightConePos.z = originalRightPos.z end
  self.leftCylinderPos = hitLeft and vec3(hitLeft.pt) or originalLeftPos
  self.rightCylinderPos = hitRight and vec3(hitRight.pt) or originalRightPos

  self.leftPillarPos = vec3(self.leftConePos)
  self.rightPillarPos = vec3(self.rightConePos)

  if self.leftConePos.z < originalLeftPos.z then
    self.leftPillarPos.z = self.leftConePos.z + (originalLeftPos.z - self.leftConePos.z)/2
  end
  if self.rightConePos.z < originalRightPos.z then
    self.rightPillarPos.z = self.rightConePos.z + (originalRightPos.z - self.rightConePos.z)/2
  end
  self.leftPillarPos.z = self.leftPillarPos.z + CONE_HEIGHT
  self.rightPillarPos.z = self.rightPillarPos.z + CONE_HEIGHT

  self.leftPillarOffset = self.leftPillarPos.z - self.leftConePos.z
  self.rightPillarOffset = self.rightPillarPos.z - self.rightConePos.z

  -- Set rotations for all objects
  local rotQuat = rot:toTorqueQuat()
  local rotStr = rotQuat.x .. ' ' .. rotQuat.y .. ' ' .. rotQuat.z .. ' ' .. rotQuat.w

  leftPillar = scenetree.findObjectById(self.leftId)
  if leftPillar then
    leftPillar:setPosition(self.leftPillarPos)
    leftPillar:setField('rotation', 0, rotStr)
    leftPillar:setScale(vec3(1,1,1))
  end
  rightPillar = scenetree.findObjectById(self.rightId)
  if rightPillar then
    rightPillar:setPosition(self.rightPillarPos)
    rightPillar:setField('rotation', 0, rotStr)
    rightPillar:setScale(vec3(1,1,1))
  end

  leftColumn = scenetree.findObjectById(self.leftColumnId)
  if leftColumn then
    leftColumn:setPosition(self.leftConePos)
    leftColumn:setField('rotation', 0, rotStr)
    leftColumn:setScale(vec3(1,1,1))
  end
  rightColumn = scenetree.findObjectById(self.rightColumnId)
  if rightColumn then
    rightColumn:setPosition(self.rightConePos)
    rightColumn:setField('rotation', 0, rotStr)
    rightColumn:setScale(vec3(1,1,1))
  end

  leftCone = scenetree.findObjectById(self.leftConeId)
  if leftCone then
    leftCone:setPosition(self.leftConePos)
    leftCone:setField('rotation', 0, rotStr)
    leftCone:setScale(vec3(1,1,1))
  end
  rightCone = scenetree.findObjectById(self.rightConeId)
  if rightCone then
    rightCone:setPosition(self.rightConePos)
    rightCone:setField('rotation', 0, rotStr)
    rightCone:setScale(vec3(1,1,1))
  end

  leftCylinder = scenetree.findObjectById(self.leftCylinderId)
  if leftCylinder then
    leftCylinder:setPosition(self.leftCylinderPos)
    leftCylinder:setField('rotation', 0, rotStr)
    leftCylinder:setScale(vec3(0.1,0.1, self.leftConePos.z-self.leftCylinderPos.z))
  end
  rightCylinder = scenetree.findObjectById(self.rightCylinderId)
  if rightCylinder then
    rightCylinder:setPosition(self.rightCylinderPos)
    rightCylinder:setField('rotation', 0, rotStr)
    rightCylinder:setScale(vec3(0.1,0.1, self.rightConePos.z-self.rightCylinderPos.z))
  end

end

function C:setMode(mode)
  if mode ~= 'hidden' then
    self:show()
  end
  self.oldMode = self.mode
  self.mode = mode
  self.colorTimer = 0

  self:update(0,0)
end

-- visibility management
function C:setVisibility(v)
  self.visible = v
  -- base = scenetree.findObjectById(self.baseId)
  -- if base then
  --   base.hidden = not v
  -- end
  leftPillar = scenetree.findObjectById(self.leftId)
  if leftPillar then
    leftPillar.hidden = not v
  end
  rightPillar = scenetree.findObjectById(self.rightId)
  if rightPillar then
    rightPillar.hidden = not v
  end
  leftColumn = scenetree.findObjectById(self.leftColumnId)
  if leftColumn then
    leftColumn.hidden = not v
  end
  rightColumn = scenetree.findObjectById(self.rightColumnId)
  if rightColumn then
    rightColumn.hidden = not v
  end
  leftCone = scenetree.findObjectById(self.leftConeId)
  if leftCone then
    leftCone.hidden = not v
  end
  rightCone = scenetree.findObjectById(self.rightConeId)
  if rightCone then
    rightCone.hidden = not v
  end
  leftCylinder = scenetree.findObjectById(self.leftCylinderId)
  if leftCylinder then
    leftCylinder.hidden = not v
  end
  rightCylinder = scenetree.findObjectById(self.rightCylinderId)
  if rightCylinder then
    rightCylinder.hidden = not v
  end
  arrow = scenetree.findObjectById(self.arrowId)
  if arrow then
    arrow.hidden = not v
  end
end

function C:hide() self:setVisibility(false) end
function C:show() self:setVisibility(true)  end

-- marker management
function C:createObject(shapeName, objectName)
  local marker =  createObject('TSStatic')
  marker:setField('shapeName', 0, shapeName)
  marker:setPosition(vec3(0, 0, 0))
  marker.scale = vec3(1, 1, 1)
  marker:setField('rotation', 0, '1 0 0 0')
  marker.useInstanceRenderData = true
  marker:setField('instanceColor', 0, '1 1 1 1')
  marker:setInternalName('marker')
  marker.canSave = false
  marker.hidden = true
  marker:registerObject(objectName)

  local scenarioObjectsGroup = scenetree.ScenarioObjectsGroup
  if scenarioObjectsGroup then
    scenarioObjectsGroup:addObject(marker)
  end

  return marker
end

-- creates neccesary objects
function C:createMarkers()
  self:clearMarkers()
  -- self.baseId  = self:createObject(baseShape,basePrefix..self.id):getId()
  self.leftId  = self:createObject(pillarShapeLeft,sidesPrefix.."left"..self.id):getId()
  self.rightId = self:createObject(pillarShapeRight,sidesPrefix.."right"..self.id):getId()
  self.leftColumnId = self:createObject(topColumnLeft,sidesPrefix.."leftColumn"..self.id):getId()
  self.rightColumnId = self:createObject(topColumnRight,sidesPrefix.."rightColumn"..self.id):getId()
  self.leftConeId = self:createObject(bottomConeShape,sidesPrefix.."leftCone"..self.id):getId()
  self.rightConeId = self:createObject(bottomConeShape,sidesPrefix.."rightCone"..self.id):getId()
  self.leftCylinderId = self:createObject(cylinderShape,sidesPrefix.."leftCylinder"..self.id):getId()
  self.rightCylinderId = self:createObject(cylinderShape,sidesPrefix.."rightCylinder"..self.id):getId()
  self.arrowId = self:createObject(arrowShape,sidesPrefix.."arrow"..self.id):getId()
end

function C:clearMarker(id)
  if id then
    local obj = scenetree.findObjectById(id)
    if obj then obj:delete() end
  end
end

-- destorys/cleans up all objects created by this
function C:clearMarkers()
  -- self:clearMarker(self.baseId)
  self:clearMarker(self.leftId)
  self:clearMarker(self.rightId)
  self:clearMarker(self.leftColumnId)
  self:clearMarker(self.rightColumnId)
  self:clearMarker(self.leftConeId)
  self:clearMarker(self.rightConeId)
  self:clearMarker(self.leftCylinderId)
  self:clearMarker(self.rightCylinderId)
  self:clearMarker(self.arrowId)
  -- self.baseId = nil
  self.leftId = nil
  self.rightId = nil
  self.leftColumnId = nil
  self.rightColumnId = nil
  self.leftConeId = nil
  self.rightConeId = nil
  self.leftCylinderId = nil
  self.rightCylinderId = nil
  self.arrowId = nil
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end