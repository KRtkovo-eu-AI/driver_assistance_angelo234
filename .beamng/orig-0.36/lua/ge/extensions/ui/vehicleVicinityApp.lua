-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this is the lua part of the vehicle vicinity app

local M = {}

local couplerCache = {}

local function getCouplerPoints(veh, vehId)
  if not couplerCache[vehId] then
    local res = {}
    local vData = extensions.core_vehicle_manager.getVehicleData(vehId)
    if vData and vData.vdata and vData.vdata.nodes then
      for nodeId, node in pairs(vData.vdata.nodes) do
        if (node.couplerTag and node.couplerTag:find('fifthwheel')) or (node.tag and node.tag:find('fifthwheel')) then
          table.insert(res, node)
        end
      end
      couplerCache[vehId] = res
    end
  end
  -- update node positions
  local vehPosition = veh:getPosition()
  for nodeId, node in ipairs(couplerCache[vehId]) do
    node.livePos = veh:getNodePosition(node.cid) + vehPosition
  end
  return couplerCache[vehId]
end

local function onGuiUpdate(dtReal, dtSim, dtRaw)
  local data = {objects = {}}
  data.playerVehicleId = be:getPlayerVehicleID(0)

  for i = 0, be:getObjectCount()-1 do
    local veh = be:getObject(i)
    local vehId = veh:getId()
    local vData = extensions.core_vehicle_manager.getVehicleData(vehId)
    if veh then
      local bb = veh:getSpawnWorldOOBB()
      local dir = bb:getAxis(0)
      local type = 'vehicle'
      if veh.isTraffic == 'true' then
        type = 'traffic'
      end
      if veh.isParked == 'true' then
        type = 'parked'
      end
      if string.lower(veh.jbeam):find('trailer') then
        type = 'trailer'
      end
      data.objects[vehId] = {
        centerX = bb:getCenter().x,
        centerY = bb:getCenter().y,
        centerZ = bb:getCenter().z,
        sizeX = bb:getHalfExtents().x * 2,
        sizeY = bb:getHalfExtents().y * 2,
        sizeZ = bb:getHalfExtents().z * 2,
        rotX = math.atan2(dir.y, dir.x),
        type = type,
        couplers = getCouplerPoints(veh, vehId),
      }
    end
  end
  guihooks.trigger('onVehicleVicinityData', data)
end

-- invalidate cache

local function onVehicleSwitched()
  couplerCache = {}
end

local function onVehicleSpawned()
  couplerCache = {}
end


M.onVehicleSpawned = onVehicleSpawned
M.onVehicleSwitched = onVehicleSwitched
M.onGuiUpdate = onGuiUpdate


return M