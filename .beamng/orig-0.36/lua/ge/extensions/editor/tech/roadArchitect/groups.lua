-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local auditionHeight = 1000.0                                                                       -- The height above zero, at which the prefab groups are auditioned, in metres.
local camRotInc = math.pi / 500                                                                     -- The step size of the angle when rotating the camera around the audition center.
local auditionPlanarDistFac = 1.1                                                                   -- A factor used for determining the audition camera planar distance.
local auditionElevationFac = 0.4                                                                    -- A factor used for determining the audition camera elevation.
local prefabGroupFilepaths =                                                                        -- A table of paths to the prefab group preset data files.
  {
    { path = 'roadArchitect/groups/Urban Block Single.json', name = 'Urban Block - Single' },
    { path = 'roadArchitect/groups/Block_Urban.json', name = 'Urban Block' },

    { path = 'roadArchitect/groups/Staggered_Jct.json', name = 'Staggered Junction' },

    { path = 'roadArchitect/groups/Double Crossroads + Island.json', name = 'Double Crossroads + Island' },

    { path = 'roadArchitect/groups/Double_Y.json', name = 'Double Y-Junction' },

    { path = 'roadArchitect/groups/Double_Roundabout.json', name = 'Double Roundabout' },
    { path = 'roadArchitect/groups/Roundabout - Separator.json', name = 'Roundabout With Separators' },

    { path = 'roadArchitect/groups/Urban To Highway Split.json', name = 'Urban <-> Highway Split' },
    { path = 'roadArchitect/groups/Urban-Rural 1Way Link.json', name = 'Urban <-> Rural 1-Way Link' },
    { path = 'roadArchitect/groups/Urban-Rural Link Series.json', name = 'Urban <-> Rural Link Series' },
    { path = 'roadArchitect/groups/Urban-Rural Split+Fork.json', name = 'Urban <-> Rural Split + Fork' },

    { path = 'roadArchitect/groups/RIRO_Intersection.json', name = 'RIRO Intersection' }
  }

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- External modules used.
local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing the road structure/handling road calculations.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Private constants.
local im = ui_imgui
local min, max, abs, sin, cos, tan = math.min, math.max, math.abs, math.sin, math.cos, math.tan
local twoPi = math.pi * 2.0
local gView, auditionVec, auditionCamPos = vec3(0, 0), vec3(0, 0, auditionHeight), vec3(0, 0)
local tmp0, tmp1 = vec3(0, 0), vec3(0, 0)
local camRotAngle = 0.0

-- Module state.
local groups = {}                                                                                   -- The array of prefab group templates currently available in the editor.
local placedGroups = {}                                                                             -- The collection of placed groups in the editor session.
local oldPos, oldRot = nil, nil                                                                     -- The previous camera pose, before going to profile view.
local isInGroupView = false                                                                         -- A flag which indicates if the camera is in group view, or not.


-- Copies a group road and elevates it to the audition pose.
local function elevateToAuditionPose(road)
  local nodes = road.nodes
  local numNodes = #nodes
  for i = 1, numNodes do
    local p = nodes[i].p
    nodes[i].p:set(p.x, p.y, p.z + auditionHeight)
  end
end

-- Computes the center and radius of the given group.
local function getCenterAndRadiusOfGroup(g)
  local m = g.roads
  local xMin, xMax, yMin, yMax, zMin, zMax, numMembers = 1e99, -1e99, 1e99, -1e99, 1e99, -1e99, #m
  for i = 1, numMembers do
    local nodes = m[i].nodes
    local numNodes = #nodes
    for j = 1, numNodes do
      local p = nodes[j].p
      local x, y, z = p.x, p.y, p.z
      xMin, xMax, yMin, yMax, zMin, zMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y), min(zMin, z), max(zMax, z)
    end
  end
  tmp0:set(xMin, yMin, zMin)
  tmp1:set(xMax, yMax, zMax)
  local c = tmp0 + 0.5 * (tmp1 - tmp0)
  return c, tmp1:distance(c)
end

-- Updates the camera position upon changing of audition group.
local function updateCameraPose(gIdx)
  local _, r = getCenterAndRadiusOfGroup(groups[gIdx])
  gView:set(0.0, -r * auditionPlanarDistFac, auditionHeight + r * auditionElevationFac)
  local gRot = quatFromDir(auditionVec - gView)
  commands.setFreeCamera()
  core_camera.setPosRot(0, gView.x, gView.y, gView.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Rotate camera around the audition centroid.
local function rotateCamera(ang)
  local x, y, s, c = gView.x, gView.y, sin(ang), cos(ang)
  auditionCamPos:set(x * c - y * s, x * s + y * c, gView.z)
  local gRot = quatFromDir(auditionVec - auditionCamPos)
  core_camera.setPosRot(0, auditionCamPos.x, auditionCamPos.y, auditionCamPos.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Moves the camera to the prefab group preview pose.
-- [Also adjusts the timing parameters respectively].
local function goToGroupView(gIdx, timer, time)
  if not isInGroupView then
    time, isInGroupView = 0.0, true
    timer:stopAndReset()
    oldPos, oldRot = core_camera.getPosition(), core_camera.getQuat()                               -- Store the current camera position so we can return to it later.
    updateCameraPose(gIdx)                                                                          -- Move the camera to the audition pose for the selected group.
  end
  return time
end

-- Adds a prefab group to the roads container, so it will be rendered for the purpose of auditioning.
-- [Roads will be elevated to the audition pose and remain hidden in the roads array].
local function addGroupToRoadsAudition(gIdx)

  -- Compute the center of mass of the group.
  local group = groups[gIdx]
  local cen, _ = getCenterAndRadiusOfGroup(group)

  local groupRoads, roads = group.roads, roadMgr.roads
  local numRoads, ctr = #groupRoads, #roads + 1
  for i = 1, numRoads do
    local r = groupRoads[i]
    local rCopy = roadMgr.copyRoad(r)
    rCopy.isDisplayNodeSpheres = im.BoolPtr(false)
    rCopy.isDisplayRoadOutline = im.BoolPtr(true)
    rCopy.isDisplayRefLine = im.BoolPtr(true)
    rCopy.isDisplayLaneInfo = im.BoolPtr(true)
    rCopy.isConformRoadToTerrain = im.BoolPtr(false)
    rCopy.isHidden = true
    local nodes = rCopy.nodes
    local numNodes = #nodes
    for j = 1, numNodes do
      nodes[j].p = nodes[j].p - cen
    end
    elevateToAuditionPose(rCopy)
    roadMgr.setDirty(rCopy)
    roads[ctr] = rCopy
    ctr = ctr + 1
  end

  -- Re-compute the road map.
  roadMgr.recomputeMap()

  -- Update the camera to fit the newly-selected group.
  -- [This is only done if we are already in the group view, since the old pos/rot have not been recorded yet].
  if isInGroupView then
    updateCameraPose(gIdx)
  end
end

-- Adds a group template to the session (adds all the group roads to roads list).
-- [The user has chosen this group, so the roads will not be elevated to the audition pose and will not be hidden].
-- [Unique name ids will be given to each profile and road in the group].
local function addGroupToRoadsPlace(gIdx, isConformGroupToTerrain)
  placedGroups = placedGroups or {}
  local placed = {}
  local groupRoads, roads = groups[gIdx].roads, roadMgr.roads
  local numGroupRoads = #groupRoads
  local firstRIdx = #roads + 1
  local ctr = firstRIdx
  for i = 1, numGroupRoads do
    local r = groupRoads[i]
    local rCopy = roadMgr.copyRoad(r)
    local idNew = Engine.generateUUID()
    rCopy.name, rCopy.isHidden = idNew, false
    rCopy.isConformRoadToTerrain = im.BoolPtr(isConformGroupToTerrain)
    roads[ctr] = rCopy
    for j = 1, #rCopy.nodes do
      placed[#placed + 1] = { r = rCopy.name, n = j }
    end
    rCopy.groupIdx = { #placedGroups + 1 }
    ctr = ctr + 1
  end

  -- Re-compute the road map.
  roadMgr.recomputeMap()

  -- Update the road connectivity id data with the newly-generated id numbers.
  local pGroup, pCtr, lastRIdx = {}, 1, ctr - 1
  for i = firstRIdx, lastRIdx do
    pGroup[pCtr] = roads[i].name
    pCtr = pCtr + 1
  end

  -- Add the new group to the placed groups container.
  placedGroups[#placedGroups + 1] = { name = im.ArrayChar(32, groups[gIdx].name), list = placed }

  roadMgr.removeHiddenRoads()

  return pGroup
end

-- Manages the rotation of the audition camera.
local function manageRotateCam()
  rotateCamera(camRotAngle)
  camRotAngle = camRotAngle + camRotInc
  if camRotAngle > twoPi then
    camRotAngle = camRotAngle - twoPi
  end
end

-- Identifies group (from given polygon) and creates a new prefab group.
local function createGroup(gPolygon, roads)

  -- Convert the polygon to 2D.
  local polyLen = #gPolygon
  for i = 1, polyLen do
    local p = gPolygon[i]
    gPolygon[i]:set(p.x, p.y, 0.0)
  end

  -- Find all the road nodes inside the polygon.
  placedGroups = placedGroups or {}
  local placedGroupIdx = #placedGroups + 1
  local placed, ctr = {}, 1
  for i = 1, #roads do
    local r = roads[i]
    for j = 1, #r.nodes do
      if not r.isJctRoad and not r.isBridge and r.nodes[j].p:inPolygon(gPolygon) then
        placed[ctr] = { r = r.name, n = j }
        ctr = ctr + 1
        util.tryAddGroupIdxToRoad(r, placedGroupIdx)
      end
    end
  end

  -- If there are no roads in the group, leave without creating any group.
  if #placed < 1 then
    return
  end

  placedGroups[placedGroupIdx] = { name = im.ArrayChar(32, 'New Group'), list = placed }
end

-- Deep copies the given placed group.
local function copyPlacedGroup(pg)
  return {
    name = ffi.string(pg.name),
    list = deepcopy(pg.list) }
end

-- Turns the given placed group to a saved prefab group.
local function createPrefabGroup(g)

  -- Collect all the roads into a hash table.
  local roads = {}
  for i = 1, #g.list do
    local gR = g.list[i]
    local r = roadMgr.roads[roadMgr.map[gR.r]]
    if not roads[r.name] then
      roads[r.name] = roadMgr.copyRoad(r)
      local newName = Engine.generateUUID()
      roads[r.name].name = newName
    end
  end

  -- Convert to an array.
  local roadArray, ctr = {}, 1
  for _, v in pairs(roads) do
    roadArray[ctr] = v
    ctr = ctr + 1
  end

  -- Create the group.
  if #roadArray > 0 then
    local gIdx = #groups + 1
    groups[gIdx] = { name = 'User ' .. tostring(gIdx), roads = roadArray }
  end
end

-- Creates a unique group from a data file.
local function importGroup(roads, name)

  -- Copy the road and profile, and add it to the new group.
  local gRoads, numRoads = {}, #roads
  for i = 1, numRoads do
    local r = roads[i]
    gRoads[i] = roadMgr.copyRoad(r)
    local newRId = Engine.generateUUID()
    gRoads[i].name, gRoads[i].isHidden = newRId, true
    roadMgr.setDirty(gRoads[i])
  end

  -- Create the group, as long as there is at least one road present.
  if #gRoads > 0 then
    local gIdx = #groups + 1
    local gName = (name or 'User ') .. tostring(gIdx)
    groups[gIdx] = { name = gName, roads = gRoads }
  end
end

-- Returns the camera to the stored old view.
local function goToOldView()
  if oldPos and oldRot then
    core_camera.setPosRot(0, oldPos.x, oldPos.y, oldPos.z, oldRot.x, oldRot.y, oldRot.z, oldRot.w)
  end
  isInGroupView = false
end

-- Serialises a prefab group.
local function serialiseGroup(g)
  local gSer, r = { name = g.name, roads = {} }, g.roads
  local numRoads = #r
  for i = 1, numRoads do
    gSer.roads[i] = roadMgr.serialiseRoad(r[i])
  end
  return gSer
end

-- De-serialises a prefab group.
local function deserialiseGroup(gSer)
  local g, rSer = { name = gSer.name, roads = {} }, gSer.roads
  local numRoads = #rSer
  for i = 1, numRoads do
    g.roads[i] = roadMgr.deserialiseRoad(rSer[i])
  end
  return g
end

-- Saves the collection of current prefab groups to disk.
local function save(idx)
  extensions.editor_fileDialog.saveFile(
    function(data)
      local encodedData = { data = { group = serialiseGroup(groups[idx]) }}
      jsonWriteFile(data.filepath, encodedData, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a collection of current prefab groups from disk.
local function load()
  extensions.editor_fileDialog.openFile(
    function(data)
      local loadedJson = jsonReadFile(data.filepath).data
      local gSer = deserialiseGroup(loadedJson.group)
      importGroup(gSer.roads)
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Populates the collection of default prefab groups, from the corresponding data files.
local function getDefaultGroups()
  if #groups > 0 then                                                                               -- If there are groups already there, do nothing.
    return
  end
  for i = 1, #prefabGroupFilepaths do
    local g = prefabGroupFilepaths[i]
    local loadedJson = jsonReadFile(g.path).data
    local gSer = deserialiseGroup(loadedJson.group)
    importGroup(gSer.roads, g.name)
  end
end

-- Updates the placed groups list (called after a road has been removed).
local function updateGroupsAfterRoadRemove()
  if placedGroups then
    for i = #placedGroups, 1, -1 do
      local gPlaced = placedGroups[i]
      for j = #gPlaced.list, 1, -1 do
        local road = roadMgr.roads[roadMgr.map[gPlaced.list[j].r]]
        if not road then
          table.remove(placedGroups, i)
          break
        end
      end
    end
  end
end

-- Gets the collection of placed groups.
local function getPlacedGroups() return placedGroups end

-- Sets the collection of placed groups.
local function setPlacedGroups(pg) placedGroups = pg end

-- Removes a placed group from the placed group collection, but not the roads inside it.
local function removePlacedGroupSoft(groupIdx) table.remove(placedGroups, groupIdx) end

-- Removes a placed group from the placed group collection, including the roads inside it.
local function removePlacedGroupHard(groupIdx)
  local list = placedGroups[groupIdx].list
  for j = 1, #list do
    local r = roadMgr.roads[roadMgr.map[list[j].r]]
    if r then
      roadMgr.removeGroupFromRoad(r, groupIdx)
      if #r.groupIdx < 1 then
        roadMgr.removeRoad(list[j].r)
      end
    end
  end
  table.remove(placedGroups, groupIdx)
  updateGroupsAfterRoadRemove()
end

-- Moves the camera above the placed group with the given index.
local function goToPlacedGroup(idx)
  -- Compute the 2D axis-aligned bounding box of the given group.
  -- [We also compute the largest height value].
  local xMin, xMax, yMin, yMax, zMax = 1e24, -1e24, 1e24, -1e24, -1e24
  local roads = roadMgr.roads
  local gList = placedGroups[idx].list
  for j = 1, #gList do
    local road = roads[roadMgr.map[gList[j].r]]
    if road then
      local nodes = road.nodes
      local numNodes = #nodes
      for i = 1, numNodes do
        local p = nodes[i].p
        local px, py = p.x, p.y
        xMin, xMax = min(xMin, px), max(xMax, px)
        yMin, yMax = min(yMin, py), max(yMax, py)
        zMax = max(zMax, p.z)
      end
    end
  end

  -- If box is too small (eg one node), do nothing.
  if abs(xMax - xMin) < 0.1 and abs(yMax - yMin) < 0.1 then
    return
  end

  -- Determine the required distance which the camera should be at.
  local midX, midY = (xMin + xMax) * 0.5, (yMin + yMax) * 0.5                                       -- Midpoint of axis-aligned bounding box.
  local tmp0 = vec3(midX, midY, 0.0)
  local tmp1 = vec3(xMax, yMax, 0.0)
  local groundDist = tmp0:distance(tmp1)                                                            -- The largest distance from the center of the box to the outside.
  local halfFov = core_camera.getFovRad() * 0.5                                                     -- Half the camera field of view (in radians).
  local height = groundDist / tan(halfFov) + zMax + 5.0                                             -- The height that the camera should be to fit all the trajectory in view.
  local rot = quatFromDir(vec3(0, 0, -1))

  -- Move the camera to the appropriate pose.
  commands.setFreeCamera()
  core_camera.setPosRot(0, midX, midY, height, rot.x, rot.y, rot.z, rot.w)
end


-- Public interface.
M.groups =                                                groups

M.addGroupToRoadsAudition =                               addGroupToRoadsAudition
M.addGroupToRoadsPlace =                                  addGroupToRoadsPlace
M.goToGroupView =                                         goToGroupView
M.manageRotateCam =                                       manageRotateCam

M.createGroup =                                           createGroup
M.copyPlacedGroup =                                       copyPlacedGroup
M.createPrefabGroup =                                     createPrefabGroup

M.goToOldView =                                           goToOldView
M.getDefaultGroups =                                      getDefaultGroups
M.getPlacedGroups =                                       getPlacedGroups
M.setPlacedGroups =                                       setPlacedGroups
M.removePlacedGroupSoft =                                 removePlacedGroupSoft
M.removePlacedGroupHard =                                 removePlacedGroupHard
M.goToPlacedGroup =                                       goToPlacedGroup
M.updateGroupsAfterRoadRemove =                           updateGroupsAfterRoadRemove

M.save =                                                  save
M.load =                                                  load

M.serialiseGroup =                                        serialiseGroup
M.deserialiseGroup =                                      deserialiseGroup

return M