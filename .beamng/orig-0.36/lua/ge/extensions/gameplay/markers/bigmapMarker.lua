-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local upVector = vec3(0,0,1)

local idCounter = 0

-- icon renderer
local iconRendererName = "markerIconRenderer"
local iconWorldSize = 20

-- default height for columns
local columnHeight = 3.5 --m
-- factor because the columnObject is not 1m high
local columnScl = 1/30
-- how quickly and where the marker should fade
local markerAlphaRate = 1/0.2
local markerShowDistance = 25
-- how quickly and where the icon should fade
local iconAlphaRate = 1/0.4
local iconShowDistance = 70
-- how quickly the cruising smoother should transition
local cruisingSmootherRate = 1/0.4
local cruisingRadius = 0.25
local markerFullRadiusDistance = 10

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init()
  self.id = idCounter
  idCounter = idCounter + 1

  -- ids of spawned objects
  self.iconRendererId = nil

  self.bigMapMarkerAlphaSmoother = newTemporalSmoothing()

  self.visible = true
end

local function inverseLerp(min, max, value)
 if math.abs(max - min) < 1e-30 then return min end
 return (value - min) / (max - min)
end



local camPos2d, markerPos2d = vec3(), vec3()
local tmpVec = vec3()
local vecZero = vec3(0,0,0)
local camQuat = quat()
local camUp = vec3()
local camToCluster = vec3()
local camToClusterLeft = vec3()

local bigMapModeColorI = ColorI(255,255,255,255)
local iconRendererObj


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

-- called every frame to update the visuals.
function C:update(data)
  --if not self.visible then return end
  profilerPushEvent("BigMap Marker")
  --debugDrawer:drawTextAdvanced(self.pos, String(self.bigMapIconId), ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
  -- desired height is the actual height of the icon
  local bigMapActive = self.visible and not data.bigmapTransitionActive
  local smootherVal = self.bigMapMarkerAlphaSmoother:getWithRateUncapped(bigMapActive and 1 or 0, data.dt, markerAlphaRate)
  --print(string.format("%0.2f - %s", smootherVal, self.id))
  local bigMapMarkerAlpha = clamp(smootherVal,0,1)
  --simpleDebugText3d(string.format("%0.2f %s %s %0.2f %d",bigMapMarkerAlpha, self.visible and "V" or "I", data.bigmapTransitionActive and "T" or "N", data.dt, self.id), self.pos)
  bigMapMarkerAlpha = 1-((1-bigMapMarkerAlpha)*(1-bigMapMarkerAlpha))

  if bigMapMarkerAlpha > 0 or self.visibleLastFrame then
    self.visibleLastFrame = true
    profilerPopEvent("BigMap Marker PreCalculation")
    local resolutionFactor = 800 / freeroam_bigMapMode.getVerticalResolution()
    camQuat:set(core_camera.getQuatXYZW())
    camUp:setRotate(camQuat, upVector)
    camToCluster:setSub2(self.pos, data.camPos)
    camToClusterLeft:setCross(camUp, camToCluster)
    camToClusterLeft:normalize()
    local camToUpperPoint = quatFromAxisAngle(camToClusterLeft, (resolutionFactor * 0.05 * core_camera.getFovRad())):__mul(camToCluster)

    local extraHeight = career_modules_linearTutorial and career_modules_linearTutorial.bounceBigmapIcons and bounce((os.clockhp() * 0.9)%1) * 0.05 or 0
    self.selected = self.cluster.containedIdsLookup[freeroam_bigMapMode.selectedPoiId]
    self.hovered = self.cluster.containedIdsLookup[freeroam_bigMapMode.hoveredPoiId]
    self.hoveredListItem = self.cluster.containedIdsLookup[freeroam_bigMapMode.hoveredListItem]
    profilerPushEvent("BigMap Marker Icons")
    -- updating the icons
    if self.bigMapIconId then
      local iconInfo = self.iconDataById[self.bigMapIconId]
      if iconInfo then
        local iconPos = quatFromAxisAngle(camToClusterLeft, (resolutionFactor * (0.02 + extraHeight) * core_camera.getFovRad())):__mul(camToUpperPoint)
        bigMapModeColorI.alpha = bigMapMarkerAlpha *255
        iconInfo.color = bigMapModeColorI
        tmpVec:set(data.camPos)
        tmpVec:setAdd(iconPos or vecZero)
        iconInfo.worldPosition = tmpVec
        if self.hovered or self.selected or self.hoveredListItem then
          iconInfo.customSizeFactor = 1.5
        else
          iconInfo.customSizeFactor = 1
        end
      end
    end

    if self.bigMapColumnIconId then
      local iconInfo = self.iconDataById[self.bigMapColumnIconId]
      if iconInfo then
        local iconPosColumn = quatFromAxisAngle(camToClusterLeft, (resolutionFactor * -0.03 * core_camera.getFovRad())):__mul(camToUpperPoint * 1.1)
        tmpVec:set(data.camPos)
        tmpVec:setAdd(iconPosColumn or vecZero)
        iconInfo.worldPosition = tmpVec
        bigMapModeColorI.alpha = bigMapMarkerAlpha *255
        iconInfo.color = bigMapModeColorI
      end
    end
  else
    self.visibleLastFrame = false
  end
  self.hovered = false
  profilerPopEvent("BigMap Marker")
end


function C:setup(cluster)
  self.pos = cluster.pos
  self.cluster = cluster

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

  -- setting up the icon
  if cluster.id then
    iconRendererObj = scenetree.findObjectById(self.iconRendererId)
    if iconRendererObj then
      local bigMapIconName = cluster.icon
      self.iconDataById = {}

      self.bigMapIconId = iconRendererObj:addIcon(cluster.id .. "bigMap", bigMapIconName or "mission_primary_triangle", self.pos + vec3(0,0,columnHeight))
      local iconInfo = iconRendererObj:getIconById(self.bigMapIconId)
      iconInfo.color = ColorI(255,255,255,0)
      iconInfo.customSize = iconWorldSize
      iconInfo.drawIconShadow = false
      self.iconDataById[self.bigMapIconId] = iconInfo

      self.bigMapColumnIconId = iconRendererObj:addIcon(cluster.id .. "bigMapColumn", "marker_column", self.pos + vec3(0,0,columnHeight))
      local iconInfo = iconRendererObj:getIconById(self.bigMapColumnIconId)
      iconInfo.color = ColorI(255,255,255,0)
      iconInfo.customSize = iconWorldSize
      iconInfo.drawIconShadow = false
      self.iconDataById[self.bigMapColumnIconId] = iconInfo
    end
  end
  -- setting up the smoothers
  self.bigMapMarkerAlphaSmoother:set(0)
end

function C:setFullAlphaInstant()
  self.bigMapMarkerAlphaSmoother:set(1)
end


function C:setHidden(value)
end

function C:hide()
  self.visible = false
end

function C:show()
  self.visible = true

end

function C:instantFade(visible)

end


-- destorys/cleans up all objects created by this
function C:clearObjects()
  if self.iconRendererId then
    iconRendererObj = scenetree.findObjectById(self.iconRendererId)
    if iconRendererObj then
      for id, _ in pairs(self.iconDataById or {}) do
        iconRendererObj:removeIconById(id)
      end
    end
  end
  self.bigMapIconId = nil
  self.playModeIconId = nil
  self._ids = nil
  self.borderObj = nil
  self.decalObj = nil
  self.bigMapColumnObj = nil
  self.iconRendererObj = nil
  self.iconDataById = {}
end


return {
  createMarker = function(...)
    local o = {}
    setmetatable(o, C)
    C.__index = C
    o:init(...)
    return o
  end,
  merge = function(pois, idPrefix)
    local cluster = {
      id = idPrefix.."#",
      containedIds = {},
      pos = vec3(),
      icon = "",
      containedIdsLookup = {}
    }
    local containsOnlyMissions, containsAnyMission = true, false
    local containsOnlyDelivery, containsAnyDelivery = true, false
    local containsAnyDropoff = false
    local count = 0
    for i, poi in ipairs(pois) do
      cluster.pos = cluster.pos + poi.markerInfo.bigmapMarker.pos
      cluster.icon = poi.markerInfo.bigmapMarker.icon
      cluster.containedIds[i] = poi.id
      cluster.id = cluster.id..poi.id
      cluster.containedIdsLookup[poi.id] = true
      containsOnlyMissions = containsOnlyMissions and poi.data.type == "mission"
      containsAnyMission = containsAnyMission or poi.data.type == "mission"
      containsOnlyDelivery = containsOnlyDelivery and poi.data.type == "logisticsParking"
      containsAnyDelivery = containsAnyDelivery or poi.data.type == "logisticsParking"
      containsAnyDropoff = containsAnyDropoff or poi.data.hasPlayerCargo
      count = count + 1
    end
    cluster.pos = cluster.pos / count

    if count > 1 then
      local prefix = "poi_no-%02d_round_orange_green"
      if   containsAnyMission and containsOnlyMissions
        or containsAnyDelivery and containsOnlyDelivery then
        prefix = "poi_no-%02d_round_orange_blue"
      end
      if containsAnyDropoff then
        prefix = "poi_no-%02d_round_orange"
      end
      cluster.icon = string.format(prefix,math.min(count, 9))
    end
    return cluster
  end
}
