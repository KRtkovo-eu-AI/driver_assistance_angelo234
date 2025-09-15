-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}

local markerIndexCorrection = { { 3, 4, 2, 1 }, { 1, 2, 4, 3 } }
local vecZero = vec3(0,0,0)
local vecOne = vec3(1,1,1)
local quatZero = quat(0,0,0,0)
local vecX = vec3(1,0,0)
local vecY = vec3(0,1,0)
local vecZ = vec3(0,0,1)
local playModeColorI = ColorI(255,255,255,255)
local iconHeightBottom = 1.25
local iconHeightTop = 2.5
local iconRendererObj
local iconWorldSize = 20
local tmpVec = vec3()
function C:init()
  self.visible = true
end

function C:setup(cluster)
  self.pos = cluster.pos or vecZero
  self.rot = cluster.rot or quatZero
  self.scl = cluster.scl or vecZero

  self.cluster = cluster

  self.pumps = {}
  iconRendererObj = scenetree.findObjectById(self.iconRendererId)
  for idx, pair in ipairs(cluster.pumps or {}) do
    local area = scenetree.findObject(pair[1])
    local icon = scenetree.findObject(pair[2])
    if area and icon then
      local pos, rot, scl = area:getPosition(), quat(area:getRotation()), area:getScale()
      local zVec, yVec, xVec = rot*vecZ*scl.z, rot*vecY*scl.y, rot*vecX*scl.x
      local iconPos = icon:getPosition()

      local pumpData = {
        areaPos = pos,
        xVec = xVec, yVec = yVec, zVec = zVec,
        iconPos = iconPos,
        iconPosSmoother = newTemporalSmoothingNonLinear(10,10),
        iconAlphaSmoother = newTemporalSmoothingNonLinear(30,30),
        overlap = false,
      }

      pumpData.iconAlphaSmoother:set(0)

      if iconRendererObj then
        local playModeIconName = "poi_fuel_round"
        if cluster.electric then
          playModeIconName = "poi_charge_round"
        end
        local iconId = iconRendererObj:addIcon(string.format("%s-gsIcon-%d",cluster.clusterId, idx), playModeIconName, iconPos)
        local iconInfo = iconRendererObj:getIconById(iconId)
        iconInfo.color = ColorI(255,255,255,0)
        iconInfo.customSize = iconWorldSize
        iconInfo.drawIconShadow = false

        pumpData.iconId = iconId
        pumpData.iconInfo = iconInfo
      end

      table.insert(self.pumps, pumpData)
    end
  end
end

local function alphaNeedsUpdate(overlap, area)
  if overlap then
    return true
  else
    return area.iconAlphaSmoother:value() >= 1e-30
  end
end

function C:update(data)
  if not self.visible or not data.veh then return end
  local anyOverlap = false
  for idx, area in ipairs(self.pumps or {}) do
    --simpleDebugText3d(idx, area.iconPos, 0.25)
    local overlap = (data.cruisingSpeedFactor < 1) and overlapsOBB_OBB(data.bbCenter, data.bbHalfAxis0, data.bbHalfAxis1, data.bbHalfAxis2, area.areaPos, area.xVec, area.yVec, area.zVec)
    anyOverlap = anyOverlap or overlap
    area.overlap = overlap
    local iconInfo = area.iconInfo
    if iconInfo and alphaNeedsUpdate(overlap, area) then
      tmpVec:set(iconInfo.worldPosition)
      tmpVec:setSub(data.camPos)
      local rayLength = tmpVec:length()
      local hitDist = castRayStatic(data.camPos, tmpVec, rayLength, nil)
      local visible = hitDist >= rayLength
      local smootherVal = area.iconPosSmoother:get(overlap and 1 or 0, data.dt)
      tmpVec:set(0,0,smootherVal*0.25)
      tmpVec:setAdd(area.iconPos)
      iconInfo.worldPosition = tmpVec
      playModeColorI.alpha = area.iconAlphaSmoother:get((overlap and visible) and 1 or 0, data.dt) * 255
      iconInfo.color = playModeColorI
    end
  end

  if anyOverlap ~= self.anyOverlap then
    if anyOverlap then self.isInAreaChanged = "in" end
    if not anyOverlap then self.isInAreaChanged = "out" end
  else
    self.isInAreaChanged = nil
  end

  self.anyOverlap = anyOverlap
end

local iconRendererName = "markerIconRenderer"
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
  if self.iconRendererId then
    iconRendererObj = scenetree.findObject(self.iconRendererId)
    if iconRendererObj then
      for idx, area in ipairs(self.pumps or {}) do
        playModeColorI.alpha = 0

        area.iconInfo.color = playModeColorI
      end
    end
  end
end

function C:show()
  if self.visible then return end
  self.visible = true
end

function C:clearObjects()
  if self.iconRendererId then
    iconRendererObj = scenetree.findObject(self.iconRendererId)
    if iconRendererObj then
      for idx, area in ipairs(self.pumps or {}) do
        iconRendererObj:removeIconById(area.iconId)
      end
    end
  end
  self.pumps = nil
end

function C:interactInPlayMode(interactData, interactableElements)
  if interactData.canInteract and self.anyOverlap then
    for _, elem in ipairs(self.cluster.elemData) do
      table.insert(interactableElements, elem)
    end
  end
end



function C:instantFade(visible)
end


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

-- gasstationMarkers are not grouped/merged - each poi will be one cluster.
local function cluster(pois, allClusters)
  for _, poi in ipairs(pois) do
    local cluster = {
      id = 'gasStationMarker#'..poi.id,
      --containedIds = {poi.id},
      pumps = poi.markerInfo.gasStationMarker.pumps,
      electric = poi.markerInfo.gasStationMarker.electric,
      visibilityPos = poi.markerInfo.gasStationMarker.pos,
      visibilityRadius = poi.markerInfo.gasStationMarker.radius,
      elemData = {poi.data},
      create = create,
    }
    table.insert(allClusters, cluster)
  end
end

return {
  create = create,
  cluster = cluster
}
