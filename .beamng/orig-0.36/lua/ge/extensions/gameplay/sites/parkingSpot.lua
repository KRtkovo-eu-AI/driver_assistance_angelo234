-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local xVector = vec3(1,0,0)
local yVector = vec3(0,1,0)
local zVector = vec3(0,0,1)

local zeroVector = vec3(0,0,0)
local zeroQuat = quat(0,0,0,1)

function C:init(sites, name, forceId)
  self.sites = sites
  self.id = forceId or sites:getNextUniqueIdentifier()
  self.name = name or "Parking " .. self.id
  self.color = vec3(1, 1, 1)
  self.vertices = {}
  self.pos = vec3()
  self.rot = quat(0, 0, 0, 1)
  self.scl = vec3()
  self.isMultiSpot = false
  self.multiSpotData = { spotAmount = 1, spotOffset = 0, spotDirection = "Left", spotRotation = 0 }
  self.wasGenerated = false

  self._drawMode = 'faded'
  self.sortOrder = 999999
  self.customFields = require('/lua/ge/extensions/gameplay/sites/customFields')()
end

function C:onSerialize()
  local rot = self.rotOrig or self.rot

  local ret = {
    name = self.name,
    pos = { self.pos.x, self.pos.y, self.pos.z },
    rot = { rot.x, rot.y, rot.z, rot.w },
    scl = { self.scl.x, self.scl.y, self.scl.z },
    isMultiSpot = self.isMultiSpot,
    spotAmount = self.multiSpotData.spotAmount,
    spotOffset = self.multiSpotData.spotOffset,
    spotDirection = self.multiSpotData.spotDirection,
    spotRotation = self.multiSpotData.spotRotation,
    color = self.color:toTable(),
    oldId = self.id,
    customFields = self.customFields:onSerialize()
  }
  return ret
end

function C:onDeserialized(data)
  self.name = data.name
  self.color = vec3(data.color)
  self.pos = vec3(data.pos)
  self.rot = quat(data.rot)
  self.scl = vec3(data.scl)
  self.isMultiSpot = data.isMultiSpot
  self.multiSpotData.spotAmount = data.spotAmount or 1
  self.multiSpotData.spotOffset = data.spotOffset or 0
  self.multiSpotData.spotDirection = data.spotDirection or "Left"
  self.multiSpotData.spotRotation = data.spotRotation or 0
  self.customFields:onDeserialized(data.customFields)
  self:calcVerts()
end

function C:set(pos, rot, scl)
  if pos then
    self.pos = vec3(pos)
  end
  if rot then
    self.rot = quat(rot)
  end
  if scl then
    self.scl = vec3(scl)
  end
  self:calcVerts()
end

function C:getPath()
  return self.sites.dir .. self.sites.filename .. '#'..self.name
end

function C:drawDebug(drawMode, clr)
  if self.isProcedural then
    return
  end

  drawMode = drawMode or self._drawMode
  if drawMode == 'none' then return end

  local camSqDist = self.pos:squaredDistance(core_camera.getPosition())
  local drawDist = editor.isEditorActive() and editor.getPreference("gizmos.visualization.visualizationDrawDistance") or 300

  local alpha = drawMode == 'normal' and 0.4 or 1
  local textDrawDist = drawMode == 'highlight' and math.huge or square(drawDist)
  if drawMode ~= 'faded' and camSqDist <= textDrawDist then
    debugDrawer:drawTextAdvanced(self.pos, String(self.name), ColorF(1, 1, 1, alpha), true, false, ColorI(0, 0, 0, alpha * 255))
  end

  if camSqDist > square(drawDist) then return end

  clr = clr or self.color:toTable()
  if drawMode == 'highlight' then
    clr = { 1, 1, 1, 1 }
  end
  local shapeAlpha = (drawMode == 'highlight') and 0.5 or 0.25

  local tmpSpotAmount = 1
  if self.isMultiSpot then
    tmpSpotAmount = self.multiSpotData.spotAmount
  end
  for i = 0, tmpSpotAmount - 1 do
    local rot = self.rot
    local dirVec
    if self.multiSpotData.spotDirection == "Left" then
      dirVec = rot * vec3(-i * (self.scl.x + self.multiSpotData.spotOffset), 0, 0)
    elseif self.multiSpotData.spotDirection == "Right" then
      dirVec = rot * vec3(i * (self.scl.x + self.multiSpotData.spotOffset), 0, 0)
    elseif self.multiSpotData.spotDirection == "Front" then
      dirVec = rot * vec3(0, i * (self.scl.y + self.multiSpotData.spotOffset), 0)
    elseif self.multiSpotData.spotDirection == "Back" then
      dirVec = rot * vec3(0, -i * (self.scl.y + self.multiSpotData.spotOffset), 0)
    end
    rot = quatFromEuler(0,0,self.multiSpotData.spotRotation) * rot
    local pos = self.pos + dirVec
    local x, y, z = rot * vec3(self.scl.x / 2, 0, 0), rot * vec3(0, self.scl.y / 2, 0), rot * vec3(0, 0, self.scl.z / 2)
    local clrI

    if self.isMultiSpot and i == 0 then
      clrI = color(0.91 * 255, 0.49 * 255, 0.24 * 255, shapeAlpha * 255)
    else
      clrI = color(clr[1] * 255, clr[2] * 255, clr[3] * 255, shapeAlpha * 255)
    end
    -- one side
    self:drawSide(pos + x + y, pos + x - y, z, clrI)
    self:drawSide(pos + x - y, pos - x - y, z, clrI)
    self:drawSide(pos - x - y, pos - x + y, z, clrI)
    self:drawSide(pos - x + y, pos + x + y, z, clrI)
    if self.isMultiSpot and i == 0 then
      clrI = color(0.91 * 255, 0.49 * 255, 0.24 * 255, shapeAlpha * 128)
    else
      clrI = color(clr[1] * 255, clr[2] * 255, clr[3] * 255, shapeAlpha * 128)
    end

    if camSqDist <= square(drawDist * 0.333) then -- for performance reasons
      debugDrawer:drawTriSolid(
              vec3(pos + x + y),
              vec3(pos + x - y),
              vec3(pos - x - y),
              clrI)
      debugDrawer:drawTriSolid(
              vec3(pos - x - y),
              vec3(pos - x + y),
              vec3(pos + x + y),
              clrI)

      debugDrawer:drawTriSolid(
              vec3(pos - x - y + z / 2),
              vec3(pos + y + z / 2),
              vec3(pos + x - y + z / 2),
              clrI)
    end
  end
end

function C:drawSide(a, b, z, clrI)
  debugDrawer:drawTriSolid(
          vec3(a),
          vec3(a + z),
          vec3(b + z),
          clrI)
  debugDrawer:drawTriSolid(
          vec3(b + z),
          vec3(b),
          vec3(a),
          clrI)
  debugDrawer:drawTriSolid(
          vec3(a),
          vec3(b + z),
          vec3(a + z),
          clrI)
  debugDrawer:drawTriSolid(
          vec3(b + z),
          vec3(a),
          vec3(b),
          clrI)
end

function C:generateMultiSpots(parkingSpotList)
  if self.wasGenerated then
    return
  end

  self.wasGenerated = true
  self.rotOrig = quat(self.rot)
  self.rot = quatFromEuler(0, 0, self.multiSpotData.spotRotation) * self.rotOrig
  self:calcVerts()

  for i = 1, self.multiSpotData.spotAmount do
    if i > 1 then -- generates a new parking spot
      local dirVec
      local j = i - 1

      if self.multiSpotData.spotDirection == "Left" then
        dirVec = self.rotOrig * vec3(-j * (self.scl.x + self.multiSpotData.spotOffset), 0, 0)
      elseif self.multiSpotData.spotDirection == "Right" then
        dirVec = self.rotOrig * vec3(j * (self.scl.x + self.multiSpotData.spotOffset), 0, 0)
      elseif self.multiSpotData.spotDirection == "Front" then
        dirVec = self.rotOrig * vec3(0, j * (self.scl.y + self.multiSpotData.spotOffset), 0)
      elseif self.multiSpotData.spotDirection == "Back" then
        dirVec = self.rotOrig * vec3(0, -j * (self.scl.y + self.multiSpotData.spotOffset), 0)
      end

      local pos = self.pos + dirVec
      local rot = quatFromEuler(0, 0, self.multiSpotData.spotRotation) * self.rotOrig
      local newSpot = parkingSpotList:create(self.name .. "." .. (i + 1))
      newSpot:set(pos, rot, self.scl)

      for _, tag in ipairs(self.customFields.sortedTags) do
        newSpot.customFields:addTag(tag)
      end

      for _,fieldName in ipairs(self.customFields.names) do
        local fieldValue, fieldType = self.customFields:get(fieldName)
        newSpot.customFields:add(fieldName, fieldType, fieldValue)
      end

      newSpot.isProcedural = true
      newSpot:calcVerts()
    end
  end
end

function C:calcVerts()
  table.clear(self.vertices)

  for i = 1, 4 do -- 2D vertices
    local a = (i == 2 or i == 3) and 1 or -1
    local b = i <= 2 and 1 or -1
    local x = self.rot * vec3(self.scl.x / 2, 0, 0)
    local y = self.rot * vec3(0, self.scl.y / 2, 0)
    table.insert(self.vertices, self.pos + x * a + y * b)
  end
end

function C:containsPoint(pos)
  if not self.vertices[1] then
    self:calcVerts()
  end

  if pos:squaredDistance(self.pos) <= self.pos:squaredDistance(self.vertices[1]) then -- quick radius check, as an optimization
    local offset = pos - self.pos
    local dx = xVector:rotated(self.rot)
    local dy = yVector:rotated(self.rot)
    local dz = zVector:rotated(self.rot)

    return math.abs(offset:dot(dx)) <= self.scl.x / 2 and math.abs(offset:dot(dx)) <= self.scl.y / 2 and math.abs(offset:dot(dx)) <= self.scl.z / 2
  end

  return false
end

local p1, p2, p3, p4, p5, p6, p7, p8 = vec3(), vec3(), vec3(), vec3(), vec3(), vec3(), vec3(), vec3()
local bbPoints = {}
local function getBBPoints(bbCenter, bbAxis0, bbAxis1, bbAxis2)
  p1:set(bbCenter) p1:setAdd(bbAxis0) p1:setSub(bbAxis1) p1:setSub(bbAxis2)
  p2:set(bbCenter) p2:setAdd(bbAxis0) p2:setSub(bbAxis1) p2:setAdd(bbAxis2)
  p3:set(bbCenter) p3:setSub(bbAxis0) p3:setSub(bbAxis1) p3:setAdd(bbAxis2)
  p4:set(bbCenter) p4:setSub(bbAxis0) p4:setSub(bbAxis1) p4:setSub(bbAxis2)
  p5:set(bbCenter) p5:setAdd(bbAxis0) p5:setAdd(bbAxis1) p5:setSub(bbAxis2)
  p6:set(bbCenter) p6:setAdd(bbAxis0) p6:setAdd(bbAxis1) p6:setAdd(bbAxis2)
  p7:set(bbCenter) p7:setSub(bbAxis0) p7:setAdd(bbAxis1) p7:setAdd(bbAxis2)
  p8:set(bbCenter) p8:setSub(bbAxis0) p8:setAdd(bbAxis1) p8:setSub(bbAxis2)
  bbPoints[0] = p1; bbPoints[1] = p2; bbPoints[2] = p3; bbPoints[3] = p4;
  bbPoints[4] = p5; bbPoints[5] = p6; bbPoints[6] = p7; bbPoints[7] = p8;
  return bbPoints
end

local bbPointIds = { 0, 3, 7, 4 }
local bbPointIdsAlt = { 7, 4, 0, 3 }
local velocity, bbCenter, bbAxis0, bbAxis1, bbAxis2 = vec3(), vec3(), vec3(), vec3(), vec3()
local parkingSpotDir, vehicleDir, tempPos1, tempPos2 = vec3(), vec3(), vec3(), vec3()
function C:checkParking(vehId, precision, drivingSpeed, parkingSpeed)
  local valid = false
  local res = { false, false, false, false } -- for each vertice of the parking spot

  local veh = getObjectByID(vehId)
  if not veh then
    return valid, res
  end

  velocity:set(veh:getVelocityXYZ())
  local speed = velocity:squaredLength()
  drivingSpeed = drivingSpeed or 5
  parkingSpeed = parkingSpeed or 1
  precision = precision or 0.8

  if speed <= square(drivingSpeed) then
    bbCenter:set(be:getObjectOOBBCenterXYZ(vehId))

    if precision <= 0 then
      if self:containsPoint(bbCenter) then -- vehicle center only
        res = { true, true, true, true }
        valid = true
      end
    else
      vehicleDir:set(veh:getDirectionVectorXYZ())
      parkingSpotDir:set(yVector)
      parkingSpotDir:setRotate(self.rot)
      local parkingDir = vehicleDir:dot(parkingSpotDir)

      if math.abs(parkingDir) >= 0.707 then
        local pointIds = parkingDir >= 0 and bbPointIds or bbPointIdsAlt -- different BB corner order for forwards versus reverse
        valid = true

        bbAxis0:set(be:getObjectOOBBHalfAxisXYZ(vehId, 0))
        bbAxis1:set(be:getObjectOOBBHalfAxisXYZ(vehId, 1))
        bbAxis2:set(be:getObjectOOBBHalfAxisXYZ(vehId, 2))
        local bbPoints = getBBPoints(bbCenter, bbAxis0, bbAxis1, bbAxis2)
        for i, pointId in ipairs(pointIds) do
          if self:containsPoint(linePointFromXnorm(bbCenter, bbPoints[pointId], precision)) then
            res[i] = true
          else
            valid = false
          end
        end
      end
    end
  end

  if speed > square(parkingSpeed) then valid = false end

  return valid, res
end

function C:boxFits(extentsX, extentsY, extentsZ)
  return extentsX <= self.scl.x and extentsY <= self.scl.y and extentsZ <= self.scl.z
end

function C:vehicleFits(vehId)
  local veh = getObjectByID(vehId)
  if not veh then return false end

  local vehicleExtents = veh.initialNodePosBB:getExtents()
  return self:boxFits(vehicleExtents.x, vehicleExtents.y, vehicleExtents.z)
end

function C:hasAnyVehicles(playerId)
  -- if playerId is given, ignores this vehicle for this spot
  local hasVehicles = false
  local spotVehIds = {}

  if not self.vertices[1] then
    self:calcVerts()
  end

  for _, veh in ipairs(getAllVehicles()) do
    local vehId = veh:getID()
    if veh:getActive() and (not playerId or playerId ~= vehId) then
      bbCenter:set(be:getObjectOOBBCenterXYZ(vehId))

      if self:containsPoint(bbCenter) then -- simple check for all vehicles, including props
        hasVehicles = true
        table.insert(spotVehIds, vehId)
      else
        if map.objects[vehId] then -- complex check for tracked vehicles
          bbAxis0:set(be:getObjectOOBBHalfAxisXYZ(vehId, 0))
          bbAxis1:set(be:getObjectOOBBHalfAxisXYZ(vehId, 1))
          bbAxis2:set(be:getObjectOOBBHalfAxisXYZ(vehId, 2))
          local bbPoints = getBBPoints(bbCenter, bbAxis0, bbAxis1, bbAxis2)
          for i, pointId in ipairs(bbPointIds) do
            local nextId = bbPointIds[i + 1] or bbPointIds[1]
            tempPos1:set(bbPoints[pointId])
            tempPos2:set(bbPoints[nextId])
            if self:containsPoint(linePointFromXnorm(tempPos1, tempPos2, clamp(self.pos:xnormOnLine(tempPos1, tempPos2), 0, 1))) then
              hasVehicles = true
              table.insert(spotVehIds, vehId)
              break
            end
          end
        end
      end
    end
  end

  return hasVehicles, spotVehIds
end

local backwardsQuat = quat(0, 0, 1, 0)
function C:moveResetVehicleTo(vehId, lowPrecision, backwards, addedOffsetPos, addedOffsetRot, useSafeTeleport, removeTraffic, resetVehicleInSafeTeleport, options)
  local veh = getObjectByID(vehId)
  if not veh then return end

  options = options or {}
  addedOffsetPos = addedOffsetPos or zeroVector
  addedOffsetRot = addedOffsetRot or zeroQuat

  local selfRot = quat(self.rot)
  if backwards then
    selfRot = selfRot * backwardsQuat
  end

  -- the following code applies a positional and rotational offset to the spawn transform, and ensures that the vehicle corners are still contained in the spot
  local vehRot = addedOffsetRot * selfRot
  local newPos = self.pos
  if useSafeTeleport then
    spawn.safeTeleport(veh, self.pos, vehRot, options.skipVehicleIntersectionCheck, nil, removeTraffic, true, resetVehicleInSafeTeleport)
  else
    -- NOTE: this needs optimization
    local oobb = veh:getSpawnWorldOOBB()
    local fl  = oobb:getPoint(0)
    local fr  = oobb:getPoint(3)
    local bl  = oobb:getPoint(4)
    local flU = oobb:getPoint(1)

    local xVeh = (fr -fl) xVeh:normalize()
    local yVeh = (fl -bl) yVeh:normalize()
    local zVeh = (flU-fl) zVeh:normalize()

    local pos = veh:getPosition()
    local posOffset = (pos - fl)
    local localOffset = vec3(xVeh:dot(posOffset), yVeh:dot(posOffset), zVeh:dot(posOffset))

    local xLine, yLine, zLine = selfRot * xVector, selfRot * yVector, selfRot * zVector
    local newFLPos = self.pos - xLine * fl:distance(fr) * 0.5 + yLine * fl:distance(bl) * 0.5
    local newAddedOffset = xLine * addedOffsetPos.x + yLine * addedOffsetPos.y

    local newOffset = xLine * localOffset.x + yLine * localOffset.y + zLine * localOffset.z
    newPos = newFLPos + newOffset + newAddedOffset

    if lowPrecision then -- this must be used if the vehicle is loaded in the first frame, because the OOBB does not work correctly then
      newPos = self.pos + yLine * 2 + zLine * 0.5 -- spawn the vehicle 2m behind and 0.5m above self
    end

    vehRot = vehRot * backwardsQuat
    veh:setPositionRotation(newPos.x, newPos.y, newPos.z, vehRot.x, vehRot.y, vehRot.z, vehRot.w)
    veh:resetBrokenFlexMesh()
    veh:autoplace(false)
  end

  return newPos, vehRot
end

--[[
function C:setToVehicle(vehId)
  local veh = scenetree.findObjectById(vehId)
  if not veh then return end
  local fl  = vec3(veh:getSpawnWorldOOBB():getPoint(0))
  local fr  = vec3(veh:getSpawnWorldOOBB():getPoint(3))
  local bl  = vec3(veh:getSpawnWorldOOBB():getPoint(4))
  local flU = vec3(veh:getSpawnWorldOOBB():getPoint(1))

  local center = fl/2 + fr/2
  if scenetree.findClassObjects("TerrainBlock") then
    center.z = core_terrain.getTerrainHeight(center)
    local normalTip = center + (bl-fl)
    normalTip = vec3(normalTip.x, normalTip.y, core_terrain.getTerrainHeight(normalTip))
    self.rot = quatFromDir((center - normalTip):normalized(), (flU-bl):normalized())
  else
    self.rot = quatFromDir((fl - bl):normalized(), (flU-bl):normalized())
  end
  self.pos = center

end
]]

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end