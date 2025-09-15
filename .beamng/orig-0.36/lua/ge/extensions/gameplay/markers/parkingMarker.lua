-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}

local vecZero = vec3(0,0,0)
local vecOne = vec3(1,1,1)
local quatZero = quat(0,0,0,0)
local vecX = vec3(1,0,0)
local vecY = vec3(0,1,0)
local vecZ = vec3(0,0,1)
local outlinePrefix = "ParkingMarkerOutline_"
local outlineMarker = "art/shapes/interface/park_outline_marker.dae"

local decalFadeStart, decalFadeEnd = 3.5, 6
local decalFadeStartFocus, decalFadeEndFocus = 3.5, 6
-- icon renderer
local iconRendererName = "markerIconRenderer"
local lineColorF = ColorF(1,1,1,1)
local colorAsLinear4F = ColorF(1,1,1,1):asLinear4F()
local lineColorFFullAlpha = ColorF(1,1,1,1)
local playModeColorI = ColorI(255,255,255,255)
local iconHeightBottom = 1.0
local iconHeightTopOffset = 0.25
local iconRendererObj
local tmpVec = vec3()
local idCounter = 0
function C:init()

  self.stopTimer = 1
  idCounter = idCounter + 1
  self.numId = idCounter
  self.cornersPos = {}

  self.visible = true
  self.iconPositionSmoother = newTemporalSmoothingNonLinear(10,10)
  self.cruisingSmoother = newTemporalSmoothingNonLinear(10,10)
  self.iconAlphaSmoother = newTemporalSmoothingNonLinear(20,20)
  self.iconDistanceSmoother = newTemporalSmoothingNonLinear(20,20)
  self.overlapSmoother = newTemporalSmoothingNonLinear(10,10)
end
local outlineTmp
function C:setup(cluster)
  iconRendererObj = scenetree.findObjectById(self.iconRendererId)

  self.pos = cluster.pos or vecZero
  self.rot = cluster.rot or quatZero
  self.scl = cluster.scl or vecZero
  self.mode = cluster.mode or "overlap" -- or "contained"
  if self.mode == "contained" then
    self.boxScl = vec3(math.max(self.scl.x + 1, 1), math.max(self.scl.y +1, 1), self.scl.z+20)
  elseif self.mode == "overlap" then
    self.boxScl = vec3(1,1, self.scl.z)
  end

  self.zVec, self.yVec, self.xVec = self.rot*vecZ*self.boxScl.z/2, self.rot*vecY*self.boxScl.y/2, self.rot*vecX*self.boxScl.x/2

  self.onlyForward = false --data.onlyForward or false
  self.stopTimer = parkDelay
  self.staticMarkers = false--data.staticMarkers or false
  self.cluster = cluster
  self.iconPositionSmoother:set(iconHeightBottom)
  self.cruisingSmoother:set(0)

  if not self.outlineId then
    outlineTmp = self:createObject(outlineMarker, outlinePrefix..self.numId)
    self.outlineId  = outlineTmp:getId()
    --dump("Outline Id" ,self.outlineId)
  end
  outlineTmp = scenetree.findObjectById(self.outlineId)
  if outlineTmp then
    outlineTmp:setPositionXYZ(self.pos.x, self.pos.y, self.pos.z)
    local rot = self.rot:toTorqueQuat()
    outlineTmp:setField('rotation', 0, rot.x .. ' ' .. rot.y .. ' ' .. rot.z .. ' ' .. rot.w)
    outlineTmp:setScaleXYZ(self.scl.x / 1, self.scl.y / 2.05, 0.5)
  end

  if iconRendererObj then
    local playModeIconName = self.cluster.icon or "poi_parking_rect"
    local iconId = iconRendererObj:addIcon(string.format("%s-psIcon",self.numId), playModeIconName, self.pos)
    self.iconInfo = iconRendererObj:getIconById(iconId)
    self.iconInfo.color = ColorI(255,255,255,0)
    self.iconInfo.customSize = vec3(1,1,1)
    self.iconInfo.drawIconShadow = false
    self.iconId = iconId
  end

  self.focus = cluster.focus

  -- setting up the ground decal
  self.groundDecalData = {
    texture = 'art/shapes/interface/parkDecalStripes.png',
    position = self.pos,
    forwardVec = self.rot * vec3(0, 1, 0),
    color = ColorF(1,1,1,0.35 ),
    scale = vec3(self.scl.x, self.scl.y, 1),
    fadeStart = self.focus and decalFadeStartFocus or decalFadeStart,
    fadeEnd = self.focus and decalFadeEndFocus or decalFadeEnd
  }
  self.iconOff = vec3(0,0,0)
  self.iconPos = vec3(0,0,0)
  self.lastAlpha = -1
end


local bounce = function (x)
  local n1 = 7.5625
  local d1 = 2.75

  if x < 1 / d1 then
    return 0
  elseif x < 2 / d1 then
    x = x - 1.5 / d1
    return 1-(n1 * x * x + 0.75)
  elseif x < 2.5 / d1 then
    x = x - 2.25 / d1
    return 1-(n1 * x * x + 0.9375)
  else
    x = x - 2.625 / d1
    return 1-(n1 * x * x + 0.984375)
  end
end


local distance = 0
local nearDist, farDist = 15,60
local bigScaleDist, smallScaleDist = 100, 1000
local function distanceToHeight(dist)
  return linearScale(distance, nearDist, 700, iconHeightBottom, 500)
end
local xVector = vec3(1,0,0)
local yVector = vec3(0,1,0)
local zVector = vec3(0,0,1)
local garbages = {}
local tmpCorner, tmpX, tmpY, tmpZ = vec3(), vec3(), vec3(), vec3()
local tmpNormal = vec3()
function C:update(data)
  if not self.visible then return end
  self:checkParking(data)

  local cruisingFactor = self.cruisingSmoother:get(((distance > nearDist or data.bigMapActive) and 0 or (1-data.cruisingSpeedFactor)), data.dt)
  if cruisingFactor < 0.1 then
    self.overlap = false
  end

  local iconHeight
  if not self.focus or data.bigMapActive then
    iconHeight = self.iconPositionSmoother:get(
      (self.overlap
        and (data.highestBBPointZ-self.pos.z+iconHeightTopOffset)
        or  ((distance > farDist) and 0 or iconHeightBottom))
      * (0.5 + 0.5*cruisingFactor), data.dt)
    self.iconPos = self.pos + vec3(0,0,iconHeight)
  else

    local iconHeightFar = distanceToHeight(distance)
    iconHeight = self.iconPositionSmoother:get(
      (self.overlap
        and ((data.highestBBPointZ-self.pos.z+iconHeightTopOffset) * (0.5 + 0.5*cruisingFactor))
        or  ((distance > nearDist) and iconHeightFar or iconHeightBottom))
      , data.dt)

    -- camera-oriented vectors
    tmpX:set(1,0,0)
    tmpX:setRotate(data.camRot)
    tmpY:set(0,1,0)
    tmpY:setRotate(data.camRot)
    tmpZ:set(0,0,1)
    tmpZ:setRotate(data.camRot)
    -- plane normal vector that points up from the plane constructed from the camera point and the upper edge of the screen
    local fovRadians = core_camera.getFovRad()
    tmpNormal:set(tmpZ)
    tmpNormal:setScaled(math.tan(fovRadians/2))
    tmpNormal:setAdd(tmpY)
    tmpNormal:setScaled(-1)
    tmpNormal:setCross(tmpNormal, tmpX)
    tmpNormal:normalize()

    -- icon position with added height
    self.iconPos:set(self.pos)
    self.iconPos:setAddXYZ(0,0,iconHeight)

    tmpVec:set(self.iconPos)
    tmpVec:setSub(data.camPos)

    tmpZ:setScaled(tmpVec:dot(tmpY) / 25)
    self.iconPos:setAdd(tmpZ)

    -- iconPos with inverse screenspace offset
    self.iconOff:set(self.iconPos)
    tmpVec:set(self.pos)
    tmpVec:setAddXYZ(0,0,iconHeight)
    self.iconOff:setSub(tmpVec)

    -- project icon position on plane
    local dist = intersectsRay_Plane(self.iconPos, zVector, data.camPos, tmpNormal)

    dist = clamp(dist, -iconHeight, 0)
    self.iconPos:setAddXYZ(0,0,dist)
    self.iconPos:setSub(self.iconOff)
  end
  if self.iconInfo then
    local customSize = 1
    if self.focus then
      self.iconOff:normalize()
      self.iconOff:setScaled(bounce((os.clockhp() * 0.9)%1) * 0.4 * linearScale(distance, nearDist, nearDist +3, 1, 0))
      self.iconPos:setAdd(self.iconOff)
      customSize = linearScale(distance, bigScaleDist, smallScaleDist, 1, 0.65)
    end
    self.iconInfo.customSizeFactor = customSize
    self.iconInfo.worldPosition = self.iconPos

    tmpVec:set(data.camPos)
    tmpVec:setSub(self.iconPos)

    local rayLength =  tmpVec:length()
    local iconVisible = castRayStatic(self.iconPos, tmpVec, rayLength, nil) >= rayLength

    if not self.focus then
      lineColorF.alpha = self.iconAlphaSmoother:get((not self.visible or not iconVisible or (distance > farDist) or data.bigMapActive) and 0 or 1, data.dt) * (0.5 + 0.5*cruisingFactor)
    else
      lineColorF.alpha = self.iconAlphaSmoother:get((not self.visible or not iconVisible or data.bigMapActive) and ((not data.bigMapActive and distance < 750) and 0.35 or 0) or 1, data.dt)
    end
    playModeColorI.alpha = lineColorF.alpha * 255
    self.iconInfo.color = playModeColorI

    --simpleDebugText3d(string.format("%0.1f %0.1f %0.1f %0.1f",lineColorF.r, lineColorF.g, lineColorF.b, lineColorF.alpha), self.pos + vec3(0,0,iconHeight), 0.25)

    if lineColorF.alpha < 0.8 and not self.focus then
      debugDrawer:drawLine(self.pos, self.iconPos, lineColorF)
    else
      debugDrawer:drawLineInstance(self.pos, self.iconPos,1, lineColorF)
    end

  end
  if self.lastAlpha ~= cruisingFactor then
    outlineTmp = scenetree.findObjectById(self.outlineId)
    if outlineTmp  then
      lineColorF.alpha = cruisingFactor
      colorAsLinear4F.w = lineColorF.alpha
      outlineTmp.instanceColor = colorAsLinear4F
      outlineTmp:updateInstanceRenderData()
      outlineTmp:setScaleXYZ(self.scl.x / 1, self.scl.y / 2.05, cruisingFactor * 0.2)
    end
  end
  self.lastAlpha = cruisingFactor

  if self.overlap ~= self.isInAreaLastFrame then
    if self.overlap then self.isInAreaChanged = "in" end
    if not self.overlap then self.isInAreaChanged = "out" end
  else
    self.isInAreaChanged = nil
  end
  self.isInAreaLastFrame = self.overlap

  self.groundDecalData.fadeEnd = decalFadeEnd + self.overlapSmoother:get(self.overlap and 1 or 0, data.dt) * 20
end

function C:checkParking(data)
  if not data.veh then return end

  if data.isFreeCam then
    tmpCorner:set(data.camPos)
    tmpCorner:setSub(self.pos)
    distance = tmpCorner:length()
  else
    tmpCorner:set(data.playerPosition)
    tmpCorner:setSub(self.pos)
    distance = tmpCorner:length()
  end
  self.overlap = false

  if distance < (nearDist) then
    if self.mode == "contained" then
      self.overlap = (data.cruisingSpeedFactor < 1) and containsOBB_OBB(self.pos, self.xVec, self.yVec, self.zVec, data.bbCenter, data.bbHalfAxis0, data.bbHalfAxis1, data.bbHalfAxis2)
    elseif self.mode == "overlap" then
      self.overlap = (data.cruisingSpeedFactor < 1) and overlapsOBB_OBB(self.pos, self.xVec, self.yVec, self.zVec, data.bbCenter, data.bbHalfAxis0, data.bbHalfAxis1, data.bbHalfAxis2)
    end

    if self.overlap then
      --self:drawAxisBox(corner - vehX - vehY - vehZ, vehX*2, vehY*2, vehZ*2, color(64,64,128,64))
      --self:drawAxisBox(self.pos - self.xVec - self.yVec - self.zVec, self.xVec*2, self.yVec*2, self.zVec*2, color(self.overlap and 0 or 255, self.overlap and 255 or 0, 0,64))
      distance = 0
    end
  end
  self.parked = self.overlap
end


function C:createObject(shapeName, objectName)
  local obj = createObject('TSStatic')
  obj:setField('shapeName', 0, shapeName)
  obj:setPosition(vec3(0, 0, 0))
  obj.scale = vec3(1, 1, 1)
  obj:setField('rotation', 0, '1 0 0 0')
  obj.useInstanceRenderData = true
  obj:setField('instanceColor', 0, '1 1 1 1')
  obj.canSave = false
  obj:registerObject(objectName)

  return obj
end

function C:createObjects()
  self:clearObjects()
  iconRendererObj = scenetree.findObject(iconRendererName)
  if not iconRendererObj then
    iconRendererObj = createObject("BeamNGWorldIconsRenderer")
    iconRendererObj:registerObject(iconRendererName);
    iconRendererObj.maxIconScale = 2
    iconRendererObj.mConstantSizeIcons = true
    iconRendererObj.canSave = false
    iconRendererObj:loadIconAtlas("core/art/gui/images/iconAtlas.png", "core/art/gui/images/iconAtlas.json");
  end
  self.iconRendererId = iconRendererObj:getId()
end

function C:hide()
  if not self.visible then return end
  self.visible = false

  outlineTmp = scenetree.findObjectById(self.outlineId)
  if outlineTmp then
    outlineTmp.instanceColor = ColorF(1,1,1,0):asLinear4F()
    outlineTmp:updateInstanceRenderData()
  end

  if self.iconRendererId then
    iconRendererObj = scenetree.findObject(self.iconRendererId)
    if iconRendererObj and self.iconInfo then
      playModeColorI.alpha = 0
      self.iconInfo.color = playModeColorI
    end
  end
end

function C:show()
  if self.visible then return end
  self.visible = true
  self.lastAlpha = -1
end

function C:clearObjects()
  self.stopTimer = 1

  if self.outlineId and editor and editor.onRemoveSceneTreeObjects then
    editor.onRemoveSceneTreeObjects({self.outlineId})
  end
  outlineTmp = scenetree.findObjectById(self.outlineId)
  if outlineTmp then
    outlineTmp:delete()
  end

  outlineTmp = nil
  self.outlineId = nil

  -- floating icon
  if self.iconRendererId then
    iconRendererObj = scenetree.findObject(self.iconRendererId)
    if iconRendererObj and self.iconId then
      iconRendererObj:removeIconById(self.iconId)
      self.iconInfo = nil
      self.iconId = nil
    end
  end
end

function C:interactInPlayMode(interactData, interactableElements)
  if interactData.canInteract and self.parked then
    for _, elem in ipairs(self.cluster.elemData) do
      table.insert(interactableElements, elem)
    end
  end
end

function C:instantFade(visible)
end

-----------------------------------






function C:drawAxisBox(corner, x, y, z, clr)
  clr = clr or color(128,64,64,32)
  -- draw all faces in a loop
  for _, face in ipairs({{x,y,z},{x,z,y},{y,z,x}}) do
    local a,b,c = face[1],face[2],face[3]
    -- spokes
    debugDrawer:drawLine((corner    ), (corner+c    ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a  ), (corner+c+a  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+b  ), (corner+c+b  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a+b), (corner+c+a+b), ColorF(0,0,0,0.75))
    -- first side
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+a  ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner+b  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    -- back of first side
    debugDrawer:drawTriSolid(
      vec3(corner+a  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+b  ),
      vec3(corner+a+b),
      clr)
    -- other side
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+a  ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner+b  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    -- back of other side
    debugDrawer:drawTriSolid(
      vec3(c+corner+a  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+b  ),
      vec3(c+corner+a+b),
      clr)
  end
end

local function create(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

-- parking markers are clustered if the path to the parking spot is identical.
-- if no path is given, that marker will never cluster...
local function cluster(pois, allClusters)
  local poisByPath = {}
  local iconExistence = {}
  for _, poi in ipairs(pois) do
    local path = poi.markerInfo.parkingMarker.path or "noPath"
    poisByPath[path] = poisByPath[path] or {}
    table.insert(poisByPath[path], poi)
    iconExistence[poi.markerInfo.parkingMarker.icon or "noIcon"] = true
  end
  for key, poisInCluster in pairs(poisByPath) do
    local pm = poisInCluster[1].markerInfo.parkingMarker
    local icon = pm.icon
    local count = #poisByPath

    if count > 1 then
      if #tableKeys(iconExistence) > 1 then
        icon = ("poi_no-0") .. math.min(count, 9)
      end
    end
    local cluster = {
      id = 'parkingMarker#'..key,
      type = "parkingMarker",
      containedIdsLookup = {},
      pos = pm.pos,
      rot = pm.rot,
      scl = pm.scl,
      path = key,
      mode = pm.mode,
      icon = icon,
      focus = false,
      visibilityPos = pm.pos,
      visibilityRadius = 5,
      elemData = {},
      create = create,
    }
    for _, poi in ipairs(poisInCluster) do
      table.insert(cluster.elemData, poi.data)
      cluster.containedIdsLookup[poi.id] = true
      cluster.focus = cluster.focus or poi.markerInfo.parkingMarker.focus
    end
    table.insert(allClusters, cluster)
  end
end

return {
  create = create,
  cluster = cluster
}