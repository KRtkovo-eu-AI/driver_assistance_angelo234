-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local spinTime = 0.05                                                                               -- The amount of time between each profile auditioner rotation, in seconds.
local placeRotAngFac = 0.002                                                                        -- A angular factor used when rotating prefab groups, during placement.
local doubleClickTime = 200                                                                         -- The temporal tolerance used when determining a mouse double click, in ms.
local waitTime = 300                                                                                -- The amount of time to wait after selection of nodes, in ms.
local mouseMoveTol = 1e-2                                                                           -- A tolerance used for determining if the mouse has moved since the last frame.
local tempFilepath = 'temp/roadArchitect.json'                                                      -- The path of the temporary file used when serialising/deserialising.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'RoadArchitect'

-- Check for a Tech license.
local isTechLicense = tech_license.isValid()

-- External modules used.
local roadMgr = require('editor/tech/roadArchitect/roads')                                          -- A module for managing roads.
local profileMgr = require('editor/tech/roadArchitect/profiles')                                    -- A module for managing road profiles.
local groupMgr = require('editor/tech/roadArchitect/groups')                                        -- A module for managing groups.
local jctMgr = require('editor/tech/roadArchitect/junctions')                                       -- A module for managing junctions.
local linkUtil = require('editor/tech/roadArchitect/link')                                          -- A utility class for linking roads and junctions.
local render = require('editor/tech/roadArchitect/render')                                          -- A module for managing the rendering the road architect edit visualisations.
local terra = require('editor/tech/roadArchitect/terraform')                                        -- A module containing functions for performing terraforming operations.
local staticMgr = require('editor/tech/roadArchitect/staticMeshMgr')                                -- A module for managing static mesh selection and audition.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.
local import, export = nil, nil
if isTechLicense then
  import = require('editor/tech/roadArchitect/import')                                              -- A module containing functions for importing road networks.
  export = require('editor/tech/roadArchitect/export')                                              -- A module containing functions for exporting road networks.
end

-- Module constants (core).
local im = ui_imgui
local abs, min, max = math.abs, math.min, math.max
local sin, cos, atan2 = math.sin, math.cos, math.atan2

-- Module constants (UI).
local simSetNameFilter = im.ImGuiTextFilter()
local materialSet = {}
local texObjs = {}
local loadedTextures = 0

-- Module state (back-end).
local isRoadArchitectActive = false                                                                 -- A flag which indicates if this editor is currently active.

local hasBeenSaved = false                                                                          -- A flag which indicates if the current session has been saved previously.
local savePath = nil                                                                                -- The path of the previously saved session file.
local isLoadTerrain = im.BoolPtr(true)                                                              -- A flag which indicates the terrains should be loaded when loading sessions.

local isGroupPlaceMode = false                                                                      -- A flag which indicates if the editor is in 'group placement' mode, or not.
local isCreateGroup = false                                                                         -- A flag which indicates if the editor is in 'create group' mode, or not.
local isJctPlaceMode = false                                                                        -- A flag which indicates if the editor is in 'junction placement' mode, or not.
local stateGroupPre = nil                                                                           -- A table which stores the state of the roads before placing a group.

local isAutoJctMode = false                                                                         -- A flag which indicates if the editor is in 'auto junction' mode, or not.
local isAutoJctArmed = false                                                                        -- A flag which indicates if the auto-junction tool is 'armed', or not.
local autoJctCen = vec3(0, 0)                                                                       -- The center of the auto-junction, which is being created.
local autoJctRad = 1.0                                                                              -- The radius of the auto-junction circle, which is being created.

local isFinalise = false                                                                            -- A flag which indicates if decals have been laid/collision mesh built.

local isConformGroupToTerrain = false                                                               -- A flag which indicates if the group will be conformed to the terrain, or not.
local isEditProfileDirty = false                                                                    -- A flag which indicates if the profile-under-edit, has been changed significantly.
local hasProfilesListBeenComputed = false                                                           -- Flags which indicate if materials/mesh/profiles/groups lists have been computed.
local hasGroupsListBeenComputed = false
local hasMaterialsListBeenComputed = false
local hasMeshListBeenComputed = false
local terrain = nil                                                                                 -- The terrain block, if if exists (used here only for testing existence).
local timer, mouseTimer = hptimer(), hptimer()
local time = 0.0                                                                                    -- The time state, in seconds.
local heldTime = 0.0                                                                                -- The time which the mouse left button has been held for.
local isOpDelay = false
local opDelayTime = 0.0
local pGroup = nil                                                                                  -- A group of roads which are to be placed by the user.
local gPolygon = {}                                                                                 -- A polygon used when creating new prefab groups.
local mouseLast = vec3(0, 0)                                                                        -- The last position of the mouse.
local multiCentroidOnLeftHold = nil                                                                 -- The multi-select region centroid, used when rotating around Z.
local timeSinceLastClick = 1e99                                                                     -- The time since the last left mouse click, in seconds.
local masterWidth = im.FloatPtr(3.5)                                                                -- The master lane width (for UI control in individual road edit section).
local importCO, importTT2I, importO2T = im.BoolPtr(false), im.BoolPtr(false), im.BoolPtr(false)     -- Checkbox flags used for various importing options.
local importCustomOffset = im.FloatPtr(0.0)                                                         -- Custom offset used for importing.

local isDisplayGuidelines = false                                                                   -- Indicates if the road guidelines/measurements are being used, or not.

local isGimbalActive = false                                                                        -- A flag which indicates if the (translational) gimbal is being used.
local lastFramePosn = vec3(0, 0)                                                                    -- The last gimbal/mouse position (from last frame). Used when dragging.
local gimbalDragPre, mouseDragPre = {}, {}                                                          -- A table for storing the road state at gimbal/mouse drag start.
local isNodeBeingDragged = false                                                                    -- A flag which indicates if a node is being dragged with the mouse.
local selectedLink = nil                                                                            -- The selected possible link (table containing its data).
local selectedCandidateJct = nil                                                                    -- The selected possible junction.
local ctrlCProfile = {}                                                                             -- The profile which is being copied/pasted.
local lastAltDown = false                                                                           -- A flag which indicates if ALT was pressed during the last cycle.
local dragOffset = vec3(0, 0)
local downVec = vec3(0, 0, -1)

local tmp0, tmp1 = vec3(0, 0), vec3(0, 0)

local win = {
  toolWinName = 'roadArchitect', toolWinSize = im.ImVec2(300, 300),                                 -- The main tool window of the editor. The main UI entry point.
  materialSelectWinName = 'materialSelect', materialSelectWinSize =  im.ImVec2(300, 300),           -- The material selection window.
  meshSelectWinName = 'meshSelect', meshSelectWinSize =  im.ImVec2(300, 300),                       -- The static mesh selection window.
  nodeEditWinName = 'NodeEditWindow', nodeEditWinSize = im.ImVec2(712, 310),                        -- The individual road node editor window (secondary window).
  profilesListWinName = 'ProfilesListWindow', profilesListWinSize = im.ImVec2(350, 370),            -- The lateral road profiles list window (primary window).
  profileEditWinName = 'ProfileEditWindow', profileEditWinSize = im.ImVec2(845, 490),               -- The lateral road profile editor window (secondary window).
  groupsListWinName = 'GroupsListWindow', groupsListWinSize = im.ImVec2(410, 400),                  -- The groups list window (secondary window).
  importWinName = 'ImportOptionsWindow', importWinSize = im.ImVec2(230, 200) }                      -- The import options window (secondary window).

local vec24, vec28 = im.ImVec2(24, 24), im.ImVec2(28, 28)                                           -- Some commonly-used Imgui vectors.
local vec36, vec40 = im.ImVec2(36, 36), im.ImVec2(40, 40)

local cols = {
  fullWhite = im.ImVec4(1, 1, 1, 1),
  dullWhite = im.ImVec4(1, 1, 1, 0.25),                                                              -- Some commonly-used Imgui colour vectors.
  darkLockCol = im.ImVec4(0.05, 0.05, 0.05, 1.0),
  unlinkCol = im.ImVec4(0.75, 0.75, 0.75, 1.0),
  greenB = im.ImVec4(0.28627450980392155, 0.7137254901960784, 0.4470588235294118, 1.0),
  greenD = im.ImVec4(0.16862745098039217, 0.39215686274509803, 0.24705882352941178, 1.0),
  blueB = im.ImVec4(0.13725490196078433, 0.5764705882352941, 0.7215686274509804, 1.0),
  blueD = im.ImVec4(0.00784313725490196, 0.37254901960784315, 0.4980392156862745, 1.0) }

local keyIdx = {                                                                                    -- Relevant cached key indices.
  ctrl = im.GetKeyIndex(im.Key_ModCtrl),
  shift = im.GetKeyIndex(im.Key_ModShift),
  del = im.GetKeyIndex(im.Key_Delete),
  alt = im.GetKeyIndex(im.Key_ModAlt),
  a = im.GetKeyIndex(im.Key_A),
  c = im.GetKeyIndex(im.Key_C),
  v = im.GetKeyIndex(im.Key_V) }

-- Module state (front-end).
local mfe = {
  isProfilesListWinOpen = false,                                                                    -- Flags which indicates if the sub-windows are open or closed.
  isProfileEditWinOpen = false,
  isGroupsListWinOpen = false,
  isNodeEditWinOpen = false,
  isImportWinOpen = false,
  isMaterialSelectWinOpen = false,
  isMeshSelectWinOpen = false,
  selectedRoadIdx = 1,                                                                              -- Listbox selection index values.
  selectedNodeIdx = 1,
  selectedLayerIdx = 1,
  selectedProfileIdx = 1,
  selectedLaneIdx = 1,
  selectedGroupIdx = 1,
  selectedPlacedGroupIdx = 1,
  selectedJctIdx = 1,
  selectedMeshIdx = 1,
  selectedMeshLaneIdx = 1,
  selectedCustom = 1,
  selProfileMaterial = 1,
  selectedSingleIdx = 1,
  selectedSidewalkIdx = 1,
  isSingleMeshSelect = false,
  isRoadIdxChanged = false,
  isNodeIdxChanged = false,
  isProfileIdxChanged = false,
  isNewNodeFresh = false,
  isMaterialForRoad = false,
  isMaterialForEdgeBlendLeft = false,
  isMaterialForEdgeBlendRight = false,
  isMaterialForJctArrows = false,
  isMaterialForOverlay = false,
  materialForRoadTarget = nil,
  jctIdxWhenApplyMat = 1,
  hasDelFired = false,
  hasCtrlFired = false,
  isShowRoads = im.BoolPtr(true),
  isShowBridges = im.BoolPtr(true),
  isShowOverlays = im.BoolPtr(true) }

local terraParams = {
  domainOfInfluence = im.IntPtr(30),                                                                -- Terraforming: The domain of influence, used for terraforming, in meters.
  terraMargin = im.FloatPtr(1.0),                                                                   -- Terraforming: the margin around roads, in meters.
  isShowSingleRoad = im.BoolPtr(false),                                                             -- Terraforming: a flag which indicates if the group range will be visualised, or not.
  isShowGroup = im.BoolPtr(false) }                                                                 -- Terraforming: a flag which indicates if the selected road will be visualised, or not.


-- Sets the material list.
local function setMaterialList()
  if not hasMaterialsListBeenComputed then
    table.clear(materialSet)                                                                        -- Fetch the available materials list.
    local materials = Sim.getMaterialSet()
    for iii = 0, materials:size() - 1 do
      table.insert(materialSet, materials:at(iii))
    end
    hasMaterialsListBeenComputed = true
  end
end

-- Undo callback for road edits.
local function editRoadUndo(data)
  if isFinalise then
    return
  end
  roadMgr.removeTree()
  roadMgr.removeAll()
  local d = data.old.roads
  for i = 1, #d do
    roadMgr.roads[i] = d[i]
    roadMgr.setDirty(d[i])
  end
  roadMgr.recomputeMap()
  table.clear(jctMgr.junctions)
  local d = data.old.junctions
  for i = 1, #d do
    jctMgr.junctions[i] = d[i]
  end
  groupMgr.setPlacedGroups(data.old.placedGroups)
  roadMgr.computeAllRoadRenderData()
end

-- Redo callback for road edits.
local function editRoadRedo(data)
  if isFinalise then
    return
  end
  roadMgr.removeTree()
  roadMgr.removeAll()
  local d = data.new.roads
  for i = 1, #d do
    roadMgr.roads[i] = d[i]
    roadMgr.setDirty(d[i])
  end
  roadMgr.recomputeMap()
  table.clear(jctMgr.junctions)
  local d = data.new.junctions
  for i = 1, #d do
    jctMgr.junctions[i] = d[i]
  end
  groupMgr.setPlacedGroups(data.new.placedGroups)
  roadMgr.computeAllRoadRenderData()
end

-- Deep copies the state of all roads, junctions and groups (used for undo/redo support).
local function copyDataState()
  local cRoads = {}
  for i = 1, #roadMgr.roads do
    cRoads[i] = roadMgr.copyRoad(roadMgr.roads[i])
  end
  local cJunctions = {}
  for i = 1, #jctMgr.junctions do
    cJunctions[i] = jctMgr.copyJunction(jctMgr.junctions[i])
  end
  local cGroups = {}
  local placedGroups = groupMgr.getPlacedGroups() or {}
  for i = 1, #placedGroups do
    cGroups[i] = groupMgr.copyPlacedGroup(placedGroups[i])
  end
  return { roads = cRoads, junctions = cJunctions, placedGroups = cGroups }
end

-- Handles the dragging of nodes with the mouse.
-- [This is used when the gimbal is inactive and the user is moving nodes with the mouse].
local function handleNodeMouseDragging(mousePos)
  if mfe.selectedRoadIdx and mfe.selectedNodeIdx then
    local roads = roadMgr.roads
    local road = roads[mfe.selectedRoadIdx]
    if road then
      local nodes = road.nodes
      local node = nodes[mfe.selectedNodeIdx]
      if node then
        local gPoint = mousePos + dragOffset
        if node.p:squaredDistance(gPoint) > 1e-3 then
          if not node.isLocked then
            local numNodes = #nodes
            local v = gPoint - node.p
            if road.isRigidTranslation[0] then
              for i = 1, numNodes do
                if not nodes[i].isLocked then
                  nodes[i].p = nodes[i].p + v
                end
              end
            else
              if road.isArc and mfe.selectedNodeIdx == 2 then                                       -- Arc middle points cannot move vertically.
                node.p:set(gPoint.x, gPoint.y, node.p.z)
              else

                -- First, move the central node to the mouse position.
                local fieldInv = 1.0 / max(1e-7, road.forceField[0])                                -- [Force-Field Translation].
                node.p = vec3(gPoint.x, gPoint.y, gPoint.z)
                lastFramePosn = vec3(gPoint.x, gPoint.y, gPoint.z)

                if road.forceField[0] > 1.01 then
                  -- Iterate from just below the central node, to the start of the polyline.
                  for i = mfe.selectedNodeIdx - 1, 1, -1 do
                    local n = nodes[i]
                    if n.isLocked then                                                              -- Do not go beyond any locked node in the -ve direction.
                      break
                    end
                    n.p = n.p + v * min(1.0, max(0.0, 1.0 - node.p:distance(n.p) * fieldInv))       -- Move this node by a distance-based ratio (based on the force field).
                  end

                  -- Iterate from just above the central node, to the end of the polyline.
                  for i = mfe.selectedNodeIdx + 1, numNodes do
                    local n = nodes[i]
                    if n.isLocked then                                                              -- Do not go beyond and locked node in the +ve direction.
                      break
                    end
                    n.p = n.p + v * min(1.0, max(0.0, 1.0 - node.p:distance(n.p) * fieldInv))       -- Move this node by a distance-based ratio (based on the force field).
                  end
                end

              end
            end
            if road.isBridge or road.isOverlay then                                                 -- If this is a bridge, then force the nodes to the terrain.
              if nodes[1] then
                nodes[1].p.z = core_terrain.getTerrainHeight(nodes[1].p)
              end
              if nodes[2] then
                nodes[2].p.z = core_terrain.getTerrainHeight(nodes[2].p)
              end
            end
          end
          roadMgr.setDirty(road)
        end

      end
    end
  end
end

-- Handles the case when the user drags on a node with CTRL down, to increase/decrease the local road width.
local function handleNodeWidthChangeOnDrag()
  if mfe.selectedRoadIdx and roadMgr.roads[mfe.selectedRoadIdx] then
    local roadPre = copyDataState()
    roadMgr.setLocalWidth(mfe.selectedRoadIdx, mfe.selectedNodeIdx, im.GetIO().MouseWheel * 0.1)
    local roadPost = copyDataState()
    editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
    roadMgr.setDirty(roadMgr.roads[mfe.selectedRoadIdx])
  end
end

-- The callback function for begin axis gizmo dragging.
local function gizmoBeginDrag()
  gimbalDragPre = copyDataState()
end

-- The callback function for end axis gizmo dragging.
local function gizmoEndDrag()
  local gimbalDragPost = copyDataState()
  editor.history:commitAction("EditRoad", { old = gimbalDragPre, new = gimbalDragPost }, editRoadUndo, editRoadRedo)
end

-- The callback function for continuing axis gizmo dragging.
local function gizmoDragging()
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then                               -- Handle dragging on the translation gizmo.
    if mfe.selectedRoadIdx and mfe.selectedNodeIdx then
      local roads = roadMgr.roads
      local road = roads[mfe.selectedRoadIdx]
      if road then
        local nodes = road.nodes
        local node = nodes[mfe.selectedNodeIdx]
        if node then
          local gPoint = editor.getAxisGizmoTransform():getColumn(3)
          local multi = roadMgr.multi
          local multiSize = #multi
          if multiSize > 1 then                                                                     -- CASE A: [Multi Select].
            local mCen = roadMgr.getMultiSelectionCentroid()
            local centroidOffset = mCen - node.p
            local trVec = gPoint - node.p - centroidOffset
            for i = 1, multiSize do
              local m = multi[i]
              roads[m.r].nodes[m.n].p = roads[m.r].nodes[m.n].p + trVec
              roadMgr.setDirty(roads[m.r])
            end
          elseif node.p:squaredDistance(gPoint) > 1e-3 then                                         -- CASE B: [Single Select].
            if not node.isLocked then
              local numNodes = #nodes
              if road.isRigidTranslation[0] then
                local v = gPoint - node.p
                for i = 1, numNodes do
                  if not nodes[i].isLocked then
                    nodes[i].p = nodes[i].p + v
                  end
                end
              else
                if road.isArc and mfe.selectedNodeIdx == 2 then                                     -- Arc middle points cannot move vertically.
                  node.p:set(gPoint.x, gPoint.y, node.p.z)
                else

                  -- First, move the central node to the mouse position.
                  local fieldInv = 1.0 / max(1e-7, road.forceField[0])                              -- [Force-Field Translation].
                  node.p = editor.getAxisGizmoTransform():getColumn(3)
                  local v = node.p - lastFramePosn
                  lastFramePosn = editor.getAxisGizmoTransform():getColumn(3)

                  if road.forceField[0] > 1.01 then
                    -- Iterate from just below the central node, to the start of the polyline.
                    for i = mfe.selectedNodeIdx - 1, 1, -1 do
                      local n = nodes[i]
                      if n.isLocked then                                                            -- Do not go beyond any locked node in the -ve direction.
                        break
                      end
                      n.p = n.p + v * min(1.0, max(0.0, 1.0 - node.p:distance(n.p) * fieldInv))     -- Move this node by a distance-based ratio (based on the force field).
                    end

                    -- Iterate from just above the central node, to the end of the polyline.
                    for i = mfe.selectedNodeIdx + 1, numNodes do
                      local n = nodes[i]
                      if n.isLocked then                                                            -- Do not go beyond and locked node in the +ve direction.
                        break
                      end
                      n.p = n.p + v * min(1.0, max(0.0, 1.0 - node.p:distance(n.p) * fieldInv))     -- Move this node by a distance-based ratio (based on the force field).
                    end
                  end

                end
              end
              if road.isBridge or road.isOverlay then                                               -- If this is a bridge, then force the nodes to the terrain.
                if nodes[1] then
                  nodes[1].p.z = core_terrain.getTerrainHeight(nodes[1].p)
                end
                if nodes[2] then
                  nodes[2].p.z = core_terrain.getTerrainHeight(nodes[2].p)
                end
              end
            end
            roadMgr.setDirty(road)
          end

        end
      end
    end
  end
end

-- Handles the gimbals for translation.
local function handleGimbals(pos)
  lastFramePosn = editor.getAxisGizmoTransform():getColumn(3)
  if not isGroupPlaceMode and not isAutoJctMode and not mfe.isGroupsListWinOpen and not isJctPlaceMode then
    local rotation = QuatF(0, 0, 0, 1)
    local transform = rotation:getMatrix()
    transform:setPosition(pos)
    editor.setAxisGizmoTransform(transform)
    editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoDragging)
    editor.drawAxisGizmo()
  end
end

-- Gets the first (non-hidden) road index.
local function getFirstRoadIdx()
  local roads = roadMgr.roads
  local numRoads = #roads
  for i = 1, numRoads do
    if not roads[i].isHidden then
      return i
    end
  end
  return nil
end

-- Gets the last (non-hidden) road index.
local function getLastRoadIdx()
  local roads = roadMgr.roads
  for i = #roads, 1, -1 do
    if not roads[i].isHidden then
      return i
    end
  end
  return nil
end

-- Gets the selected road index, from a given UI listbox index.
local function getSelRoadIdx(iLB)
  local roads = roadMgr.roads
  local numRoads, ctr = #roads, 0
  for i = 1, numRoads do
    if not roads[i].isHidden then
      ctr = ctr + 1
      if ctr == iLB then
        return i
      end
    end
  end
  return getFirstRoadIdx()
end

-- Closes all open windows related to the editor.
local function closeAllWindows()
  if mfe.isProfilesListWinOpen then
    editor.hideWindow(win.profilesListWinName)
    mfe.isProfilesListWinOpen = false
    profileMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isGroupsListWinOpen then
    editor.hideWindow(win.groupsListWinName)
    mfe.isGroupsListWinOpen = false
    groupMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isMeshSelectWinOpen then
    editor.hideWindow(win.meshSelectWinName)
    mfe.isMeshSelectWinOpen = false
    staticMgr.removeAuditionMesh()
    staticMgr.goToOldView()
  end
  if mfe.isProfileEditWinOpen then
    editor.hideWindow(win.profileEditWinName)
    mfe.isProfileEditWinOpen = false
  end
  if mfe.isNodeEditWinOpen then
    editor.hideWindow(win.nodeEditWinName)
    mfe.isNodeEditWinOpen = false
  end
end

-- Adds all the nodes in the given road, to the multi-selection.
local function addAllRoadNodesToMulti(rIdx, multi)
  local road = roadMgr.roads[rIdx]
  local nodes = road.nodes
  table.clear(multi)
  for i = 1, #nodes do
    multi[i] = { r = rIdx, n = i }
  end
end

-- Adds all the nodes from all roads in the given group, to the multi-selection.
local function addGroupNodesToMulti(gIdx, multi)
  table.clear(multi)
  local list, ctr = groupMgr.getPlacedGroups()[gIdx].list, 1
  for j = 1, #list do
    multi[ctr] = { r = roadMgr.map[list[j].r], n = list[j].n }
    ctr = ctr + 1
  end
end

-- Adds all the nodes from all roads in the given junction, to the multi-selection.
local function addJctNodesToMulti(jIdx, multi)
  table.clear(multi)
  local rNames = jctMgr.junctions[jIdx].roads
  local ctr = 1
  for j = 1, #rNames do
    local rIdx = roadMgr.map[rNames[j]]
    local road = roadMgr.roads[rIdx]
    if road then
      local nodes = road.nodes
      for i = 1, #nodes do
        multi[ctr] = { r = rIdx, n = i }
        ctr = ctr + 1
      end
    end
  end
end

-- Attempts to add the given node to the multi-selection, or remove it if its already there.
local function tryAddOrRemoveToMulti(rIdx, nIdx, multi)
  for i = 1, #multi do
    local m = multi[i]
    if m.r == rIdx and m.n == nIdx then
      table.remove(multi, i)
      return
    end
  end
  multi[#multi + 1] = { r = rIdx, n = nIdx }
end

-- Determines if the multi-selection contains the given node.
local function doesMultiContain(rIdx, nIdx, multi)
  for i = 1, #multi do
    local m = multi[i]
    if m.r == rIdx and m.n == nIdx then
      return true
    end
  end
  return false
end

-- Handles the creation of roads (moving and selecting nodes, adding nodes, etc).
local function handleCreateRoads(
  roads, mousePos, isMouseClickedL, isMouseDownL, isMouseClickedR, isDoubleClick,
  isCtrlDown, isADown, isShiftDown, isDelDown)

  if not isMouseDownL then
    multiCentroidOnLeftHold = nil
  end

  -- Right mouse click will remove multi (if no gimbal).
  local multi = roadMgr.multi
  if isMouseClickedR then
    table.clear(multi)
  end

  -- If we have a multi-selection, an active gimbal, and the left mouse button is held, allow rotation of the multi-selection.
  if isGimbalActive and #multi > 1 and isMouseDownL and not editor.isAxisGizmoHovered() then
    local dy = mousePos.y - mouseLast.y
    if abs(dy) > 1e-3 then
      local theta = sign2(dy) * max(10.0, abs(dy)) * placeRotAngFac
      if isShiftDown then
        theta = 0.05 * theta
      end
      local s, c = sin(theta), cos(theta)
      local cen = multiCentroidOnLeftHold
      if not multiCentroidOnLeftHold then
        cen = roadMgr.getMultiSelectionCentroid()
        multiCentroidOnLeftHold = cen
      end
      for i = 1, #multi do
        local m = multi[i]
        local mRoad = roads[m.r]
        local n = mRoad.nodes[m.n]
        local p = n.p - cen
        local x, y = p.x, p.y
        n.p:set(x * c - y * s + cen.x, x * s + y * c + cen.y, n.p.z)
        roadMgr.setDirty(mRoad)
      end
    end
    return
  end

  -- Handle the node delete key pressing.
  if isDelDown and not mfe.hasDelFired and mfe.selectedNodeIdx and mfe.selectedRoadIdx and roads[mfe.selectedRoadIdx] and roads[mfe.selectedRoadIdx].nodes[mfe.selectedNodeIdx] then
    local roadPre = copyDataState()
    roadMgr.removeNode(mfe.selectedRoadIdx, mfe.selectedNodeIdx)
    mfe.hasDelFired = true
    if not mfe.selectedNodeIdx then
      mfe.selectedNodeIdx = 1
      mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
    end
    local road = roadMgr.roads[mfe.selectedRoadIdx]
    mfe.selectedNodeIdx = max(1, min(#road.nodes, mfe.selectedNodeIdx - 1))
    mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
    roadMgr.setDirty(road)
    local roadPost = copyDataState()
    editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
    table.clear(roadMgr.multi)
    return
  end

  local isOverNode, rIdx, nIdx = util.isMouseOverNode(roads)
  local isOverTerrain = util.isMouseHoveringOverTerrain()

  -- Manage creation of new nodes.
  if not isOverNode and not isGimbalActive and not isShiftDown and isOverTerrain then
    local road = roads[mfe.selectedRoadIdx]
    if mfe.selectedRoadIdx and road and not road.isJctRoad then
      if not (road.isArc and #road.nodes > 2) and not (road.isBridge and #road.nodes > 1) then      -- User cannot add more than 3 on arcs, and 2 on bridges.
        util.drawSphere(mousePos)                                                                   -- Draw a sphere around the mouse position on the terrain.
        if isMouseClickedL then
          local roadPre = copyDataState()
          local isInter, interPos, interIdx = roadMgr.isMouseAtIntermediatePos(mfe.selectedRoadIdx, mousePos)
          if isInter then
            roadMgr.addNodeToRoadAtIdx(mfe.selectedRoadIdx, interPos, interIdx)
            mfe.selectedNodeIdx = interIdx
          else
            roadMgr.addNodeToRoad(mfe.selectedRoadIdx, mousePos)
            mfe.selectedNodeIdx = #roads[mfe.selectedRoadIdx].nodes
          end
          mfe.isNewNodeFresh = true
          mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          table.clear(multi)
          tryAddOrRemoveToMulti(mfe.selectedRoadIdx, mfe.selectedNodeIdx, multi)
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
      end
    end
  end

  -- Manage CTRL + A node selection.
  if isCtrlDown and isADown then
    addAllRoadNodesToMulti(mfe.selectedRoadIdx, multi)
  end

  -- Manage node selection (with mouse).
  local road = roads[rIdx]
  if not isNodeBeingDragged and isOverNode and not isOpDelay then                                   -- [If mouse is over a (non-hidden) road node, while left is not held long].
    if not road.isHidden and not road.isJctRoad then
      local hNode = road.nodes[nIdx]
      util.drawSphereHighlight(hNode.p)
      if isMouseClickedL and not isShiftDown then                                                   -- Selection without SHIFT will start a new multi-selection.
        table.clear(multi)
        tryAddOrRemoveToMulti(rIdx, nIdx, multi)
        isOpDelay = true
        mfe.selectedRoadIdx = multi[1].r
        mfe.selectedNodeIdx = multi[1].n
        mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
        return
      end
      if isMouseClickedL then
        if isShiftDown then                                                                         -- If SHIFT down, then add new selections to the multi-selection.
          tryAddOrRemoveToMulti(rIdx, nIdx, multi)
          isOpDelay = true
          if #multi > 0 then
            mfe.selectedRoadIdx = multi[1].r
            mfe.selectedNodeIdx = multi[1].n
          else
            mfe.selectedNodeIdx = nil
          end
          mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          return
        end
      end
    end
  end

  -- Check for double-clicking over node (casts it to the surface below).
  if isDoubleClick and isOverNode and not isNodeBeingDragged then
    local p = roadMgr.roads[rIdx].nodes[nIdx].p
    p.z = p.z + 4.5
    p.z = p.z - castRayStatic(p, downVec, 1000)
  end

  -- Check for mouse-node dragging start.
  local selRoad = roads[mfe.selectedRoadIdx]
  if not isNodeBeingDragged and isOverTerrain and isMouseDownL and isOverNode and selRoad then
    mouseDragPre = copyDataState()
    dragOffset = roads[rIdx].nodes[nIdx].p - mousePos
    lastFramePosn = mousePos
    isNodeBeingDragged = true
  end

  -- Check if dragging start/end node is over another road end, for possible linkage.
  if isNodeBeingDragged and isOverTerrain and selRoad and (mfe.selectedNodeIdx == 1 or mfe.selectedNodeIdx == #selRoad.nodes) and not mfe.isNewNodeFresh then
    local isClose, rIdxs, nIdxs, rMids, nMids = util.isMouseCloseToNode(roads, mfe.selectedRoadIdx, mfe.selectedNodeIdx)
    if isClose then
      if #selRoad.nodes > 1 then
        selectedLink = linkUtil.computePossibleLinks(mfe.selectedRoadIdx, mfe.selectedNodeIdx, rIdxs, nIdxs)
        if not selectedLink and #rMids > 0 then
          selectedCandidateJct = linkUtil.computePossibleJct(mfe.selectedRoadIdx, mfe.selectedNodeIdx, rMids, nMids)
        end
      end
    end
  end
end

-- Computes the centroid of the collection of roads stored in the placing group array.
-- [The centroid is only on the XY-plane.  In Z, we use the minimum to ensure whole group appears above mouse height].
local function getPlacingGroupCentroid(roads)
  local pGroupLen, map = #pGroup, roadMgr.map
  local xMin, xMax, yMin, yMax, zMin, zMax = 1e99, -1e99, 1e99, -1e99, 1e99, -1e99
  for i = 1, pGroupLen do
    local r = roads[map[pGroup[i]]]
    local nodes = r.nodes
    local numNodes = #nodes
    for j = 1, numNodes do
      local p = nodes[j].p
      local x, y, z = p.x, p.y, p.z
      xMin, xMax, yMin, yMax, zMin, zMax = min(xMin, x), max(xMax, x), min(yMin, y), max(yMax, y), min(zMin, z), max(zMax, z)
    end
  end
  tmp0:set(xMin, yMin, zMin)
  tmp1:set(xMax, yMax, zMin)
  return tmp0 + 0.5 * (tmp1 - tmp0)
end

-- Handles the placing of prefab groups.
local function handlePlaceGroup(roads, mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown)

  -- Prefab group placing functionality:
  -- i)   If the user moves the mouse freely (no button held), the candidate group will move with the mouse.
  -- ii)  If the user holds down the left mouse button, moving the mouse (in Y) will rotate the group. Holding SHIFT will slow the rotation down.
  -- iii) If the user double-clicks the left mouse button, the group will be placed and we leave the group placing edit mode.
  -- iv)  If the user right-clicks the mouse, the group will be removed and the editor will revert to its normal state.
  if isDoubleClick then
    isGroupPlaceMode, pGroup = false, nil
    if stateGroupPre then
      editor.history:commitAction("EditRoad", { old = stateGroupPre, new = copyDataState() }, editRoadUndo, editRoadRedo)
      stateGroupPre = nil
    end
    return
  end

  if isMouseClickedR then
    local pGroupLen = #pGroup
    for i = 1, pGroupLen do
      roadMgr.removeRoad(pGroup[i])
    end
    table.clear(pGroup)
    isGroupPlaceMode = false
    groupMgr.updateGroupsAfterRoadRemove()
    jctMgr.updateJunctionsAfterRoadRemove()
  end

  if isMouseDownL then
    local dy = mousePos.y - mouseLast.y
    if abs(dy) > 1e-3 then
      local theta = sign2(dy) * max(10.0, abs(dy)) * placeRotAngFac
      if isShiftDown then
        theta = 0.05 * theta
      end
      local cen = getPlacingGroupCentroid(roads)
      local map, pGroupLen = roadMgr.map, #pGroup
      for i = 1, pGroupLen do
        local r = roads[map[pGroup[i]]]
        local nodes = r.nodes
        local numNodes = #nodes
        for j = 1, numNodes do
          local n = nodes[j]
          local zOld = n.p.z
          local p = n.p - cen
          local x, y, s, c = p.x, p.y, sin(theta), cos(theta)
          n.p:set(x * c - y * s + cen.x, x * s + y * c + cen.y, zOld)
        end
        roadMgr.setDirty(r)
      end
    end
  elseif util.isMouseHoveringOverTerrain() and mousePos:squaredDistance(mouseLast) > mouseMoveTol then
    local offset = mousePos - getPlacingGroupCentroid(roads)
    local map, pGroupLen = roadMgr.map, #pGroup
    for i = 1, pGroupLen do
      local r = roads[map[pGroup[i]]]
      local nodes = r.nodes
      local numNodes = #nodes
      for j = 1, numNodes do
        local n = nodes[j]
        n.p = n.p + offset
        n.height = im.FloatPtr(n.p.z)
      end
      roadMgr.setDirty(r)
    end
  end
end

-- Handles the creation of auto-junctions.
local function handleAutoJct(roads, mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown)

  -- Auto-junction placing functionality:
  -- i)   The mouse is first highlighted with a specific colour.
  -- ii)  The user left-clicks the mouse button to set the center of the auto-junction region and then drags to set the size.
  local radSq = autoJctRad * autoJctRad
  local jNodes, markedRoads = {}, {}
  if isAutoJctArmed and autoJctRad > 0.1 then
    for i = 1, #roads do
      local r = roads[i]
      local p1, p2 = r.nodes[1].p, r.nodes[#r.nodes].p
      local dSq1, dSq2 = p1:squaredDistance(autoJctCen), p2:squaredDistance(autoJctCen)
      if dSq1 < radSq and dSq1 < dSq2 then
        jNodes[#jNodes + 1] = { rName = r.name, lie = 'start' }
        markedRoads[i] = true
        util.drawSphereHighlightRed(p1)
        util.drawAutoJctMarkup(p1, ffi.string(r.displayName), 'start')
      end
      if dSq2 < radSq and dSq2 < dSq1 and not markedRoads[i] then
        jNodes[#jNodes + 1] = { rName = r.name, lie = 'end' }
        markedRoads[i] = true
        util.drawSphereHighlightRed(p2)
        util.drawAutoJctMarkup(p2, ffi.string(r.displayName), 'end')
      end
    end    
  end
  
  -- Render the spheres.
  if isAutoJctArmed then
    util.drawHighlightSphereRad(autoJctCen, autoJctRad)
    util.drawSphereHighlightPurple(autoJctCen)
  else
    util.drawSphereHighlightPurple(mousePos)
  end

  if isMouseDownL then
    if not isAutoJctArmed then
      isAutoJctArmed = true
      autoJctCen:set(mousePos.x, mousePos.y, mousePos.z)
    end
    autoJctRad = mousePos:distance(autoJctCen)
  else
    isAutoJctArmed = false
    if #jNodes > 0 then
      isAutoJctMode = false
      jctMgr.chopRoadsToSphere(jNodes, autoJctCen, autoJctRad)
      jctMgr.createAutoJctOverlays(jNodes, 3.5)
      local allRoadsGroup = { list = {} }
      local ctr = 1
      for i = 1, #roadMgr.roads do
        local tR = roadMgr.roads[i]
        for j = 1, #tR.nodes do
          allRoadsGroup.list[ctr] = { r = tR.name, n = j }
          ctr = ctr + 1
        end
      end
      terra.terraformMultiRoads(terraParams.domainOfInfluence[0], terraParams.terraMargin[0], allRoadsGroup)
    end
  end
  if isMouseClickedR then
    isAutoJctArmed = false
    isAutoJctMode = false
  end
end

-- Handles the placing of junctions.
local function handlePlaceJct(mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown)

  -- Junction placing functionality:
  -- i)   If the user moves the mouse freely (no button held), the junction will move with the mouse.
  -- ii)  If the user holds down the left mouse button, moving the mouse (in Y) will rotate the junction. Holding SHIFT will slow the rotation down.
  -- iii) If the user double-clicks the left mouse button, the junction will be placed and we leave the junction placing edit mode.
  -- iv)  If the user right-clicks the mouse, the junction will be removed and the editor will revert to its normal state.
  local selJctIdx = mfe.selectedJctIdx
  if not selJctIdx then
    isJctPlaceMode = false
    return
  end

  if isDoubleClick then
    isJctPlaceMode = false
    return
  end

  if isMouseClickedR then
    jctMgr.removeJunction(selJctIdx)
    isJctPlaceMode = false
    return
  end

  if isMouseDownL then
    local dy = mousePos.y - mouseLast.y
    if abs(dy) > 1e-3 then
      local theta = sign2(dy) * max(10.0, abs(dy)) * placeRotAngFac
      if isShiftDown then
        theta = 0.05 * theta
      end
      local cen = jctMgr.getJunctionCentroid(selJctIdx)
      jctMgr.rotateJunction(selJctIdx, cen, theta)
    end
  elseif util.isMouseHoveringOverTerrain() and mousePos:squaredDistance(mouseLast) > mouseMoveTol then
    local offset = mousePos - jctMgr.getJunctionCentroid(selJctIdx)
    jctMgr.translateJunction(selJctIdx, offset)
  end
end

-- Handles the creation of new prefab groups.
-- [Allows user to draw a polygon around roads, using mouse clicks, then double-click to complete and create group].
local function handleCreateGroup(roads, mousePos, isMouseClickedL, isDoubleClick, isMouseClickedR)
  util.drawGroupSphere(mousePos)                                                                    -- Draw a sphere at the mouse position.
  local gPolygonLen = #gPolygon
  if isMouseClickedR then                                                                           -- Clicking the right mouse button clears the polygon.
    table.clear(gPolygon)
    isCreateGroup = false
  elseif isDoubleClick and gPolygonLen > 2 then                                                     -- Double-clicking the left mouse button closes the polygon/creates the group.
    gPolygon[#gPolygon + 1] = gPolygon[1]
    groupMgr.createGroup(gPolygon, roads)
    table.clear(gPolygon)
    isCreateGroup = false
  elseif isMouseClickedL then                                                                       -- Single-clicking the left mouse button adds another vertex to the polygon.
    gPolygon[#gPolygon + 1] = mousePos
  end
end

-- Handles the laying/unlaying of decals.
local function handleisFinalise()
  if isFinalise then
    if mfe.isNodeEditWinOpen then
      editor.hideWindow(win.nodeEditWinName)
      mfe.isNodeEditWinOpen = false
    end
    local wasProfileOpen = mfe.isProfilesListWinOpen or mfe.isProfileEditWinOpen
    if mfe.isProfilesListWinOpen then
      editor.hideWindow(win.profilesListWinName)
      mfe.isProfilesListWinOpen = false
    end
    if mfe.isProfileEditWinOpen then
      editor.hideWindow(win.profileEditWinName)
      mfe.isProfileEditWinOpen = false
    end
    if wasProfileOpen then
      profileMgr.goToOldView()
      roadMgr.removeHiddenRoads()
    end
    if mfe.isGroupsListWinOpen then
      editor.hideWindow(win.groupsListWinName)
      mfe.isGroupsListWinOpen = false
      groupMgr.goToOldView()
      roadMgr.removeHiddenRoads()
    end
    if mfe.isMeshSelectWinOpen then
      editor.hideWindow(win.meshSelectWinName)
      mfe.isMeshSelectWinOpen = false
      staticMgr.removeAuditionMesh()
      staticMgr.goToOldView()
    end
    roadMgr.finalise()                                                                              -- Switch to the 'finalise' state.
  else
    roadMgr.unfinalise()                                                                            -- Revert to the 'edit' state.
  end
end

-- Updates a road to take on a new profile template, from the given profile index.
local function updateRoadToNewProfileFromIdx(pIdx)
  local road = roadMgr.roads[mfe.selectedRoadIdx]
  local profile = profileMgr.profiles[pIdx]
  local profileName = profile.name
  profileMgr.updateToNewTemplate(road, profileName)
  road.laneKeys, road.leftKeys, road.rightKeys = profileMgr.computeLaneKeys(profile)
  if isEditProfileDirty then
    roadMgr.updateWAndHToNewProfile(road)
  end
  roadMgr.setDirty(road)
end

-- Saves the session at the given file path.
local function saveSessionAlreadySaved(filepath)
  local serRoads, serProfiles, serGroups, serJunctions = {}, {}, {}, {}
  local roads, profiles, groups, junctions = roadMgr.roads, profileMgr.profiles, groupMgr.groups, jctMgr.junctions
  local numRoads, numProfiles, numGroups, numJcts = #roads, #profiles, #groups, #junctions

  -- Serialise all the roads data.
  for i = 1, numRoads do
    serRoads[i] = roadMgr.serialiseRoad(roads[i])
  end

  -- Serialise all the lateral profile data.
  for i = 1, numProfiles do
    serProfiles[i] = profileMgr.serialiseProfile(profiles[i])
  end

  -- Serialise all the prefab group data.
  for i = 1, numGroups do
    serGroups[i] = groupMgr.serialiseGroup(groups[i])
  end

  -- Serialise the placed groups list.
  local serPlacedGroups = {}
  local placedGroups = groupMgr.getPlacedGroups()
  for i = 1, #placedGroups do
    serPlacedGroups[i] = {
      name = ffi.string(placedGroups[i].name),
      list = {} }
    for j = 1, #placedGroups[i].list do
      serPlacedGroups[i].list[j] = placedGroups[i].list[j]
    end
  end

  -- Serialise all the junctions.
  for i = 1, numJcts do
    serJunctions[i] = jctMgr.serialiseJct(junctions[i])
  end

  -- Write the .json file (contains the roads, junction, groups data etc).
  local encodedData =
  {
    data =
      {
        roads = serRoads,
        profiles = serProfiles,
        groups = serGroups,
        placedGroups = serPlacedGroups,
        junctions = serJunctions,
        mapName = core_levels.getLevelName(getMissionFilename())
      }
    }
  jsonWriteFile(filepath, encodedData, true)

  -- Write the .png file (contains the heightmap data).
  local pathOnly = util.removeFileNameFromPath(filepath)
  local filenameAndExt = util.getFilenameFromPath(filepath)
  local filenameWithoutExt = util.removeExtension(filenameAndExt)
  terra.writeHeightmapToPng(pathOnly .. filenameWithoutExt .. '.png')
end

-- Saves the current editor session to disk.
local function saveSession()
  extensions.editor_fileDialog.saveFile(
    function(data)
      saveSessionAlreadySaved(data.filepath)
      hasBeenSaved = true
      savePath = data.filepath
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a previously-saved editor session from disk.
local function loadSession()
  extensions.editor_fileDialog.openFile(
    function(data)

      -- Collect the loaded data.
      local loadedJson = jsonReadFile(data.filepath).data
      local serRoads, serProfiles, serJunctions = loadedJson.roads, loadedJson.profiles, loadedJson.junctions
      local serGroups, serPlacedGroups = loadedJson.groups, loadedJson.placedGroups

      -- Ensure the map used in the saved session matches the currently-loaded map.
      if loadedJson.mapName ~= core_levels.getLevelName(getMissionFilename()) then
        log('E', logTag, 'The currently-loaded map does not match the map in the saved session file. Please switch to the map: ' .. loadedJson.mapName)
        return
      end

      -- Remove all meshes and decals from scene.
      roadMgr.removeAll()

      -- Load the terrain file, if requested, and set the terrain.
      if isLoadTerrain[0] then
        local pathOnly = util.removeFileNameFromPath(data.filepath)
        local filenameAndExt = util.getFilenameFromPath(data.filepath)
        local filenameWithoutExt = util.removeExtension(filenameAndExt)
        terra.setHeightmapFromPng(pathOnly .. filenameWithoutExt .. '.png')
      end

      -- De-serialise all the stored profiles, into the profiles container.
      table.clear(profileMgr.profiles)
      for i = 1, #serProfiles do
        profileMgr.profiles[i] = profileMgr.deserialiseProfile(serProfiles[i])
      end

      -- De-serialise all the stored roads, into the roads container.
      table.clear(roadMgr.roads)
      for i = 1, #serRoads do
        roadMgr.roads[i] = roadMgr.deserialiseRoad(serRoads[i])
        roadMgr.setDirty(roadMgr.roads[i])
      end
      roadMgr.recomputeMap()

      -- De-serialise all the stored prefab groups, into the prefab groups container.
      table.clear(groupMgr.groups)
      for i = 1, #serGroups do
        groupMgr.groups[i] = groupMgr.deserialiseGroup(serGroups[i])
      end

      -- De-serialise the placed groups list.
      local placedGroups = {}
      if serPlacedGroups then
        for i = 1, #serPlacedGroups do
          placedGroups[i] = { name = im.ArrayChar(32, serPlacedGroups[i].name), list = {} }
          for j = 1, #serPlacedGroups[i].list do
            placedGroups[i].list[j] = serPlacedGroups[i].list[j]
          end
        end
        groupMgr.setPlacedGroups(placedGroups)
      end

      -- De-serialise all the junctions.
      table.clear(jctMgr.junctions)
      for i = 1, #serJunctions do
        jctMgr.junctions[i] = jctMgr.deserialiseJct(serJunctions[i])
      end

      -- Compute the render data for all roads.
      roadMgr.computeAllRoadRenderData()

      mfe.selectedRoadIdx = 1
      profileMgr.updateLaneFlags(roadMgr.roads[mfe.selectedRoadIdx].profile)

      -- Now that the meshes exist in the scene, re-compute the collision mesh to take them into account.
      be:reloadCollision()

      hasBeenSaved = true
      savePath = data.filepath
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Handles the main tool window.
local function handleMainToolWindow(roads)
  if editor.beginWindow(win.toolWinName, "Road Architect###8", im.WindowFlags_NoCollapse) then

    if mfe.isProfilesListWinOpen or mfe.isGroupsListWinOpen or mfe.isMeshSelectWinOpen then
      return
    end

    im.BeginChild1("topBtnRow1", im.ImVec2(-1, 60), im.WindowFlags_ChildWindow)

    -- Buttons row 1.
    im.Columns(8, "toolWdwTopRowBtns", false)

    -- 'Is Finalise' button.
    if not isCreateGroup and not isGroupPlaceMode and not isAutoJctMode then
      local finIcon = editor.icons.roadEditOutline
      if isFinalise then finIcon = editor.icons.roadEditSolid end
      if editor.uiIconImageButton(finIcon, vec40, cols.fullWhite, nil, nil, 'LayDecalsButton') then
        if #roadMgr.roads > 0 then
          isFinalise = not isFinalise
          handleisFinalise()
        end
      end
      if isFinalise then
        im.tooltip("Road Architect is in Render Mode. Click to switch back to Edit Mode.")
      else
        im.tooltip("Road Architect is in Edit Mode. Click to switch to Render Mode.")
      end
    end
    im.SameLine()
    im.NextColumn()

    if isFinalise then
      return
    end

    -- The 'Guidelines On/Off' toggle button.
    local btnCol, btnIcon = cols.dullWhite, editor.icons.roadGuideArrowOutline
    if isDisplayGuidelines then btnCol, btnIcon = cols.fullWhite, editor.icons.roadGuideArrowSolid end
    if editor.uiIconImageButton(btnIcon, vec40, btnCol, nil, nil, 'guidelinesToggleBtn') then
      isDisplayGuidelines = not isDisplayGuidelines
    end
    im.tooltip('Toggles the road guidelines/measurements (extends from road start and end).')
    im.SameLine()
    im.NextColumn()

    -- Toggle gimbal on/off button.
    local btnCol, gimbalBtn = cols.dullWhite, editor.icons.gizmosOutline
    if isGimbalActive then btnCol, gimbalBtn = cols.fullWhite, editor.icons.gizmosSolid end
    if editor.uiIconImageButton(gimbalBtn, vec40, btnCol, nil, nil, 'toggleGimbalOnOffBtn') then
      isGimbalActive = not isGimbalActive
    end
    im.tooltip('Toggles the translational gimbal on/off.')
    im.SameLine()
    im.NextColumn()

    -- Save session button.
    if hasBeenSaved and savePath then
      if editor.uiIconImageButton(editor.icons.floppyDisk, vec40, nil, nil, nil, 'saveSessionButton') then
        saveSessionAlreadySaved(savePath)
      end
      im.tooltip('Saves the Road Architect session (to the current save file).')
    else
      im.Dummy(vec40)
    end
    im.SameLine()
    im.NextColumn()

    -- 'Save As' button.
    if editor.uiIconImageButton(editor.icons.floppyDiskPlus, vec40, nil, nil, nil, 'saveAsButton') then
      saveSession()
    end
    im.tooltip('Save As (saves a new Road Architect session).')
    im.SameLine()
    im.NextColumn()

    -- Load session button.
    if editor.uiIconImageButton(editor.icons.roadFolder, vec40, cols.dullWhite, nil, nil, 'loadSession') then
      loadSession()
    end
    im.tooltip('Loads a Road Architect session.')
    im.SameLine()
    im.NextColumn()

    -- 'Import Road Network' button.
    if isTechLicense and not isFinalise then
      if editor.uiIconImageButton(editor.icons.terrain_import, vec40, cols.fullWhite, nil, nil, 'importOpenDRIVEBtn') then
        mfe.isImportWinOpen = not mfe.isImportWinOpen
        if mfe.isImportWinOpen then
          editor.showWindow(win.importWinName)
        else
          editor.hideWindow(win.importWinName)
        end
      end
      im.tooltip('Import road network from disk (OpenDRIVE .xodr).')
    else
      im.Dummy(vec40)
    end
    im.SameLine()
    im.NextColumn()

    -- 'Export Road Network' button.
    if isTechLicense and not isFinalise then
      if editor.uiIconImageButton(editor.icons.terrain_export, vec40, cols.fullWhite, nil, nil, 'exportOpenDRIVEBtn') then
        export.export()
      end
      im.tooltip('Export road network to disk, (OpenDRIVE .xodr).')
    else
      im.Dummy(vec40)
    end
    im.NextColumn()

    im.EndChild()

    im.Columns(1)

    im.Separator()

    -- The tab bar [fixed at top].
    local selectedTab = nil
    if im.BeginTabBar("Tooltabs1") then
      if im.BeginTabItem("Roads") then
        selectedTab = 1
        im.EndTabItem()
      end
      if im.BeginTabItem("Junctions") then
        selectedTab = 2
        im.EndTabItem()
      end
      if im.BeginTabItem("Groups") then
        selectedTab = 3
        im.EndTabItem()
      end
      if im.BeginTabItem("Options") then
        selectedTab = 4
        im.EndTabItem()
      end
      im.EndTabBar()
    end

    -- Create a child region for the scrollable content.
    im.SetCursorPosY(im.GetCursorPosY() + 5)
    im.BeginChild1("ScrollingRegion1", im.ImVec2(-1, 800), im.WindowFlags_ChildWindow)

    -- Roads List tab.
    if selectedTab == 1 then

      -- Road visibility checkboxes.
      im.Columns(3, "roadsVisibilityCheckboxesRowTop1", true)
      if im.Checkbox("Roads###11211", mfe.isShowRoads) then
        roadMgr.setRoadsVisibilityMaster(mfe.isShowRoads[0])
      end
      im.tooltip('Show all roads (master switch).')
      im.SameLine()
      im.NextColumn()
      if im.Checkbox("Bridges###11212", mfe.isShowBridges) then
        roadMgr.setBridgesVisibilityMaster(mfe.isShowBridges[0])
      end
      im.tooltip('Show all bridges (master switch).')
      im.SameLine()
      im.NextColumn()
      if im.Checkbox("Overlays###11213", mfe.isShowOverlays) then
        roadMgr.setOverlaysVisibilityMaster(mfe.isShowOverlays[0])
      end
      im.tooltip('Show all overlays (master switch).')
      im.NextColumn()

      im.Columns(1)
      im.Separator()

      -- Roads list.
      im.PushItemWidth(-1)
      if im.BeginListBox('', im.ImVec2(-1, 200)) then

        im.Columns(6, "roadsListBoxColumns", true)
        im.SetColumnWidth(0, 30)
        im.SetColumnWidth(1, 180)
        im.SetColumnWidth(2, 35)
        im.SetColumnWidth(3, 35)
        im.SetColumnWidth(4, 70)
        im.SetColumnWidth(5, 60)

        local numRoads, rCtr = #roads, 1
        local wCtr = 700
        for i = 1, numRoads do
          local road = roads[i]
          if not road.isHidden and not road.isJctRoad then
            local flag = i == mfe.selectedRoadIdx
            if im.Selectable1("###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
              table.clear(roadMgr.multi)
              mfe.selectedRoadIdx = getSelRoadIdx(i)
              profileMgr.updateLaneFlags(road.profile)
            end
            if mfe.isRoadIdxChanged and flag then
              im.SetScrollHereY()
              mfe.isRoadIdxChanged = false
            end

            wCtr = wCtr + 1
            im.SameLine()
            im.NextColumn()

            im.PushItemWidth(180)
            im.InputText("###" .. tostring(wCtr), road.displayName, 32)
            im.PopItemWidth()
            im.tooltip('Edit the road name.')
            wCtr = wCtr + 1
            im.SameLine()
            im.NextColumn()

            -- 'Remove Selected Road' button.
            if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeRoad') then
              local roadPre = copyDataState()
              roadMgr.removeRoad(road.name)
              groupMgr.updateGroupsAfterRoadRemove()
              jctMgr.updateJunctionsAfterRoadRemove()
              roadMgr.updateMultiAfterRemove()
              local roadPost = copyDataState()
              editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
              mfe.selectedRoadIdx, mfe.selectedNodeIdx = getFirstRoadIdx(), 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
              if mfe.isNodeEditWinOpen then
                editor.hideWindow(win.nodeEditWinName)
                mfe.isNodeEditWinOpen = false
              end
              table.clear(roadMgr.multi)
              return
            end
            im.tooltip('Remove this road from the session.')
            im.SameLine()
            im.NextColumn()

            -- 'Go To Selected Road' button.
            if road.nodes and #road.nodes > 1 then
              if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, vec24, cols.unlinkCol, nil, nil, 'goToSelectedRoad') then
                roadMgr.goToRoad(i)
              end
              im.tooltip('Go to this road.')
            else
              im.Text("")
            end
            im.SameLine()
            im.NextColumn()

            im.Checkbox("Show###" .. tostring(wCtr), road.isVis)
            im.tooltip('Show this road in edit visualisation (checked), or not (unchecked).')
            wCtr = wCtr + 1
            im.SameLine()
            im.NextColumn()

            if #road.groupIdx > 0 then
              im.TextColored(
                cols.greenB,
                tostring(road.groupIdx[1]) .. " " ..
                tostring(road.groupIdx[2] or '') .. " " ..
                tostring(road.groupIdx[3] or '') .. " " ..
                tostring(road.groupIdx[4] or '') .. " " ..
                tostring(road.groupIdx[5] or ''))
            else
              im.Text('')
            end
            im.tooltip('The group(s) which this road belongs to, if any.')
            wCtr = wCtr + 1
            im.NextColumn()

            im.Separator()
            rCtr = rCtr + 1
          end
        end
        im.EndListBox()
      end
      im.PopItemWidth()

      im.Separator()

      im.Columns(6, "roadsListButtonColumns1", false)

      -- 'Add New Spline Road' button.
      if editor.uiIconImageButton(editor.icons.bSpline, vec36, cols.blueB, nil, nil, 'addNewSplineRoadBtn') then
        if not hasProfilesListBeenComputed then
          profileMgr.populateProfileTemplates()                                                   -- Populate the default lateral road profile templates.
          hasProfilesListBeenComputed = true
        end
        local roadPre = copyDataState()
        local rIdx = #roadMgr.roads + 1
        local newRoad = roadMgr.createRoadFromTemplate(profileMgr.profiles[mfe.selectedProfileIdx].name)
        roadMgr.roads[rIdx] = newRoad
        roadMgr.map[newRoad.name] = rIdx
        roadMgr.recomputeMap()
        local roadPost = copyDataState()
        editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        mfe.selectedLayerIdx = 1
        mfe.selectedRoadIdx = getLastRoadIdx()
        if mfe.isNodeEditWinOpen then
          editor.hideWindow(win.nodeEditWinName)
          mfe.isNodeEditWinOpen = false
        end
        isGimbalActive = false
      end
      im.tooltip('Add a new spline road.')
      im.SameLine()
      im.NextColumn()

      -- 'Add New Arc Road' button.
      if editor.uiIconImageButton(editor.icons.pathArc, vec36, cols.blueB, nil, nil, 'addNewArcRoadBtn') then
        if not hasProfilesListBeenComputed then
          profileMgr.populateProfileTemplates()                                                   -- Populate the default lateral road profile templates.
          hasProfilesListBeenComputed = true
        end
        local roadPre = copyDataState()
        local rIdx = #roadMgr.roads + 1
        local newRoad = roadMgr.createRoadFromTemplate(profileMgr.profiles[mfe.selectedProfileIdx].name)
        newRoad.isArc = true
        roadMgr.roads[rIdx] = newRoad
        roadMgr.map[newRoad.name] = rIdx
        roadMgr.recomputeMap()
        local roadPost = copyDataState()
        editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        mfe.selectedLayerIdx = 1
        mfe.selectedRoadIdx = getLastRoadIdx()
        if mfe.isNodeEditWinOpen then
          editor.hideWindow(win.nodeEditWinName)
          mfe.isNodeEditWinOpen = false
        end
        isGimbalActive = false
      end
      im.tooltip('Add a new arc road.')
      im.SameLine()
      im.NextColumn()

      -- 'Add New Overlay' button.
      if editor.uiIconImageButton(editor.icons.tb_bank_right, vec36, cols.blueB, nil, nil, 'addNewOverlayBtn') then
        local roadPre = copyDataState()
        local rIdx = #roadMgr.roads + 1
        local cProf = profileMgr.createOverlayProfile(3.5)
        local newRoad = roadMgr.createRoadFromProfile(cProf)
        newRoad.isOverlay = true
        newRoad.isDrivable = false
        newRoad.displayName = im.ArrayChar(32, 'New Overlay')
        roadMgr.roads[rIdx] = newRoad
        roadMgr.map[newRoad.name] = rIdx
        roadMgr.recomputeMap()
        local roadPost = copyDataState()
        editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        mfe.selectedLayerIdx = 1
        mfe.selectedRoadIdx = getLastRoadIdx()
        if mfe.isNodeEditWinOpen then
          editor.hideWindow(win.nodeEditWinName)
          mfe.isNodeEditWinOpen = false
        end
        isGimbalActive = false
      end
      im.tooltip('Add a new overlay (for tire tread markings only).')
      im.SameLine()
      im.NextColumn()

      -- 'Add New Bridge' button.
      if editor.uiIconImageButton(editor.icons.bridgeWithRiver, vec36, cols.blueB, nil, nil, 'addNewBridgeBtn') then
        local roadPre = copyDataState()
        local rIdx = #roadMgr.roads + 1
        local bridgeProf = profileMgr.createBridgeProfile(5.5, 4.0)
        local newRoad = roadMgr.createRoadFromProfile(bridgeProf)
        newRoad.isBridge = true
        newRoad.isDrivable = false
        newRoad.displayName = im.ArrayChar(32, 'New Bridge')
        newRoad.granFactor = im.IntPtr(3)
        roadMgr.roads[rIdx] = newRoad
        roadMgr.map[newRoad.name] = rIdx
        roadMgr.recomputeMap()
        local roadPost = copyDataState()
        editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        mfe.selectedLayerIdx = 1
        mfe.selectedRoadIdx = getLastRoadIdx()
        if mfe.isNodeEditWinOpen then
          editor.hideWindow(win.nodeEditWinName)
          mfe.isNodeEditWinOpen = false
        end
        isGimbalActive = false
      end
      im.tooltip('Add a new bridge.')
      im.SameLine()
      im.NextColumn()

      -- 'Reload Collision Mesh' button.
      if editor.uiIconImageButton(editor.icons.polygonalCube, vec36, cols.greenB, nil, nil, 'reloadCollisioMeshBtn') then
        be:reloadCollision()
      end
      im.tooltip('Reloads the collision mesh (allows nodes to be placed on bridges after creating/moving bridge).')
      im.SameLine()
      im.NextColumn()

      -- 'Clear/Reset Road Network' button.
      if editor.uiIconImageButton(editor.icons.trashBin2, vec36, cols.fullWhite, nil, nil, 'clearRoadNetwork') then
        roadMgr.clearAllRoads()
        groupMgr.updateGroupsAfterRoadRemove()
        jctMgr.updateJunctionsAfterRoadRemove()
        roadMgr.updateMultiAfterRemove()
        if mfe.isNodeEditWinOpen then
          editor.hideWindow(win.nodeEditWinName)
          mfe.isNodeEditWinOpen = false
        end
        mfe.selectedRoadIdx, mfe.selectedNodeIdx = nil, nil
        return
      end
      im.tooltip('Reset/clear the road network.')
      im.NextColumn()

      im.Columns(1)
      im.Separator()

      -- Road Edit section.
      local road = roads[mfe.selectedRoadIdx]
      if road and mfe.selectedRoadIdx and not road.isJctRoad then
        local isArcRoad = road.isArc
        local roadNodes = road.nodes

        im.PushItemWidth(-1)
        if im.BeginListBox('###103', im.ImVec2(-1, 200)) then

          if roadNodes then
            im.Columns(6, "nodeEditBoxColumns", true)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 40)
            im.SetColumnWidth(2, 40)
            im.SetColumnWidth(3, 40)
            im.SetColumnWidth(4, 40)
            im.SetColumnWidth(5, 40)
            local numNodes = #roadNodes
            for i = 1, numNodes do
              local flag = i == mfe.selectedNodeIdx or doesMultiContain(mfe.selectedRoadIdx, i, roadMgr.multi)
              if im.Selectable1('Node [' .. tostring(i) .. ']', flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                table.clear(roadMgr.multi)
                mfe.selectedNodeIdx = i
                mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
              end
              if mfe.isNodeIdxChanged and flag then
                im.SetScrollHereY()
                mfe.isNodeIdxChanged = false
              end
              im.tooltip('The node Id (ordered from start of road to end of road).')
              im.SameLine()
              im.NextColumn()

              -- 'Remove Selected Node' button.
              -- [This is only displayed if there are more than two nodes].
              if numNodes > 2 then
                if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeNode') then
                  local roadPre = copyDataState()
                  roadMgr.removeNode(mfe.selectedRoadIdx, i)
                  if not mfe.selectedNodeIdx then
                    mfe.selectedNodeIdx = 1
                  end
                  mfe.selectedNodeIdx = max(1, min(#roadMgr.roads[mfe.selectedRoadIdx].nodes, mfe.selectedNodeIdx - 1))
                  roadMgr.setDirty(road)
                  local roadPost = copyDataState()
                  editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
                  table.clear(roadMgr.multi)
                  return
                end
                im.tooltip('Remove this node from the road.')
              else
                im.Text('')
              end
              im.SameLine()
              im.NextColumn()

              -- 'Add New Node' buttons.
              -- [This is not displayed for the very last node].
              if not road.isBridge then
                if editor.uiIconImageButton(editor.icons.vertical_align_top, vec24, cols.greenB, nil, nil, 'addNewNodeAboveBtn') then
                  local roadPre = copyDataState()
                  roadMgr.addIntermediateNode(mfe.selectedRoadIdx, i, 'above')
                  roadMgr.setDirty(road)
                  local roadPost = copyDataState()
                  editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
                end
                im.tooltip('Add a new node before this node.')
              end
              im.SameLine()
              im.NextColumn()
              if not road.isBridge then
                if editor.uiIconImageButton(editor.icons.vertical_align_bottom, vec24, cols.greenB, nil, nil, 'addNewNodeBelowBtn') then
                  local roadPre = copyDataState()
                  roadMgr.addIntermediateNode(mfe.selectedRoadIdx, i, 'below')
                  roadMgr.setDirty(road)
                  local roadPost = copyDataState()
                  editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
                end
                im.tooltip('Add a new node after this node.')
              end
              im.SameLine()
              im.NextColumn()

              -- 'Edit Selected Node' button.
              if not road.isOverlay and not road.isBridge then
                local editNodeCol = cols.unlinkCol
                if mfe.isNodeEditWinOpen and i == mfe.selectedNodeIdx then editNodeCol = cols.dullWhite end
                if editor.uiIconImageButton(editor.icons.edit, vec24, editNodeCol, nil, nil, 'editNode') then
                  if i == mfe.selectedNodeIdx then
                    if mfe.isNodeEditWinOpen then                                                   -- If this node is already selected, toggle window open/closed.
                      editor.hideWindow(win.nodeEditWinName)
                    else
                      editor.showWindow(win.nodeEditWinName)
                    end
                    mfe.isNodeEditWinOpen = not mfe.isNodeEditWinOpen
                  else                                                                              -- If node not currently selected, open/keep window open, but with this node.
                    editor.showWindow(win.nodeEditWinName)
                    mfe.isNodeEditWinOpen = true
                  end
                  mfe.selectedNodeIdx = i
                  mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
                end
                im.tooltip('Edit this node (opens edit window).')
              end
              im.SameLine()
              im.NextColumn()

              -- Lock column button.
              if roadNodes[i].isLocked then
                if editor.uiIconImageButton(editor.icons.lock, vec24, cols.darkLockCol, nil, nil, 'unlockNode') then
                  local roadPre = copyDataState()
                  roadNodes[i].isLocked = false
                  local roadPost = copyDataState()
                  editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
                end
                im.tooltip('Unlock the highlighted node, so it can be moved.')
              else
                im.Text('')
              end
              im.NextColumn()

              im.Separator()
            end
          end
          im.EndListBox()
        end
        im.PopItemWidth()
        im.Separator()

        -- First row of buttons:
        im.Columns(7, "underRoadsListBtns", false)

        -- 'Profile Select' button
        local road = roads[mfe.selectedRoadIdx]
        if not road.isOverlay and not road.isBridge then
          local profileButtonCol = cols.blueB
          if mfe.isProfilesListWinOpen then profileButtonCol = cols.blueD end
          if editor.uiIconImageButton(editor.icons.roadProfile, vec36, profileButtonCol, nil, nil, 'selectProfile') then

            local pCopy = profileMgr.copyProfile(road.profile)
            profileMgr.removeAllTempCurrentProfiles()
            pCopy.name = im.ArrayChar(32, 'Current Profile')
            profileMgr.profiles[#profileMgr.profiles + 1] = pCopy
            mfe.selectedProfileIdx = #profileMgr.profiles
            roadMgr.setAuditionProfileDirty()
            isEditProfileDirty = true
            mfe.isProfileIdxChanged = true

            if mfe.isProfilesListWinOpen then
              profileMgr.updateToNewTemplate(road, profileMgr.profiles[mfe.selectedProfileIdx].name)
              roadMgr.updateWAndHToNewProfile(road)
              roadMgr.setDirty(road)
              editor.hideWindow(win.profilesListWinName)
            else
              if not hasProfilesListBeenComputed then
                profileMgr.populateProfileTemplates()                                                 -- Populate the default lateral road profile templates.
                hasProfilesListBeenComputed = true
              end
              editor.showWindow(win.profilesListWinName)
              if mfe.isNodeEditWinOpen then
                editor.hideWindow(win.nodeEditWinName)
                mfe.isNodeEditWinOpen = false
              end
            end
            mfe.isProfilesListWinOpen = not mfe.isProfilesListWinOpen
            if mfe.isProfileEditWinOpen then
              editor.hideWindow(win.profileEditWinName)
              mfe.isProfileEditWinOpen = false
            end
            if not mfe.isProfilesListWinOpen and not mfe.isProfileEditWinOpen then
              profileMgr.goToOldView()
              roadMgr.removeHiddenRoads()
              mfe.selectedRoadIdx = getSelRoadIdx(mfe.selectedRoadIdx)
              roadMgr.setDirty(roads[mfe.selectedRoadIdx])
            end
            if mfe.isGroupsListWinOpen then
              editor.hideWindow(win.groupsListWinName)
              mfe.isGroupsListWinOpen = false
              groupMgr.goToOldView()
              roadMgr.removeHiddenRoads()
              mfe.selectedRoadIdx = getSelRoadIdx(mfe.selectedRoadIdx)
              roadMgr.setDirty(roads[mfe.selectedRoadIdx])
            end
            if mfe.isMeshSelectWinOpen then
              editor.hideWindow(win.meshSelectWinName)
              mfe.isMeshSelectWinOpen = false
              staticMgr.removeAuditionMesh()
              staticMgr.goToOldView()
            end
          end
          im.tooltip('Select a new profile template for this road (warning: will undo many existing road properties).')
        else
          im.Dummy(vec36)
        end
        im.SameLine()
        im.NextColumn()

        -- 'Create New Template From Profile' button.
        if not road.isOverlay and not road.isBridge then
          if editor.uiIconImageButton(editor.icons.fg_type_square_2, vec36, cols.dullWhite, nil, nil, 'CreateNewTemplateFromProfileBtn') then
            profileMgr.createTemplateOnRequest(road.profile)
          end
          im.tooltip('Create a new template profile from the current profile of this road (will appear in templates list, and can be saved from there).')
        else
          im.Dummy(vec36)
        end
        im.NextColumn()

        -- 'Use Civil Engineered Roads' button. Line-spiral-arc-spiral-line at bends, instead of fitted CR-splines.
        if not road.isOverlay and not road.isBridge and not isArcRoad then
          local useCivilEngButtonCol = cols.greenB
          local rBtn = editor.icons.bezierPath1
          if road.isCivilEngRoads[0] then useCivilEngButtonCol, rBtn = cols.greenD, editor.icons.bezierPath2 end
          if editor.uiIconImageButton(rBtn, vec36, useCivilEngButtonCol, nil, nil, 'UseCivilEngButton') then
            road.isCivilEngRoads = im.BoolPtr(not road.isCivilEngRoads[0])
            roadMgr.setDirty(road)
          end
          im.tooltip('Uses line-spiral-arc-spiral-line sections (instead of splines).')
        else
          im.Dummy(vec36)
        end
        im.SameLine()
        im.NextColumn()

        -- 'Split Road At Node' button (splits the road into two, with a gap).
        -- [Only show this button if the node is ii) not an arc road, iii) unlocked, and iv) the selected node is not an start/end point].
        if not isArcRoad and not road.isBridge and roadNodes[mfe.selectedNodeIdx] and not roadNodes[mfe.selectedNodeIdx].isLocked and mfe.selectedNodeIdx and mfe.selectedNodeIdx > 1 and mfe.selectedNodeIdx < #roadNodes then
          if editor.uiIconImageButton(editor.icons.content_cut, vec36, cols.greenB, nil, nil, 'splitRoadAtNode') then
            if mfe.isNodeEditWinOpen then
              editor.hideWindow(win.nodeEditWinName)
              mfe.isNodeEditWinOpen = false
            end
            roadMgr.splitRoad(mfe.selectedRoadIdx, mfe.selectedNodeIdx)
            mfe.selectedRoadIdx = getLastRoadIdx()
            return
          end
          im.tooltip('Split road at node, and create a gap (for junctioning).')
        else
          im.Dummy(vec36)
        end
        im.SameLine()
        im.NextColumn()

        -- 'Flip Road Direction' button.
        -- [There must be at least two nodes].
        if #roadNodes > 1 and not road.isBridge then
          if editor.uiIconImageButton(editor.icons.autorenew, vec36, cols.greenB, nil, nil, 'flipRoadDirection') then
            local roadPre = copyDataState()
            roadMgr.flipRoad(mfe.selectedRoadIdx)
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
            return
          end
          im.tooltip('Flip the road direction. Changes reference line position on profile')
        else
          im.Dummy(vec36)
        end
        im.SameLine()
        im.NextColumn()

        -- 'Lock/Unlock Node' button.
        if roadNodes and roadNodes[mfe.selectedNodeIdx] and roadNodes[mfe.selectedNodeIdx].isLocked then
          if editor.uiIconImageButton(editor.icons.lock_open, vec36, cols.blueB, nil, nil, 'unlockNode') then
            local roadPre = copyDataState()
            roadNodes[mfe.selectedNodeIdx].isLocked = false
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Unlock the highlighted node, so it can be moved.')
        else
          if editor.uiIconImageButton(editor.icons.lock, vec36, cols.blueD, nil, nil, 'lockNode') then
            local roadPre = copyDataState()
            roadNodes[mfe.selectedNodeIdx].isLocked = true
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Lock the highlighted node, so that it becomes fixed in space.')
        end
        im.SameLine()
        im.NextColumn()

        -- 'Conform Road To Terrain' button.
        if terrain and not road.isOverlay and not road.isBridge then
          local isConfTToRButtonCol = cols.blueD
          if road.isConformRoadToTerrain[0] then isConfTToRButtonCol = cols.blueB end
          if editor.uiIconImageButton(editor.icons.lineToTerrain, vec36, isConfTToRButtonCol, nil, nil, 'conformRoadToTerrainButton') then
            local roadPre = copyDataState()
            road.isConformRoadToTerrain = im.BoolPtr(not road.isConformRoadToTerrain[0])
            roadMgr.setDirty(road)
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Conform the road to the terrain.')
        else
          im.Dummy(vec36)
        end
        im.NextColumn()

        im.Columns(1)

        -- Bridge Options.
        if road.isBridge then
          im.Separator()
          im.TextColored(cols.greenB, 'Bridge Controls:')
          im.PushItemWidth(-150)
          if im.InputFloat("Bridge Width (m) ###10100", road.bridgeWidth, 0.1, 0.0) then
            road.bridgeWidth = im.FloatPtr(max(1.0, min(20.0, road.bridgeWidth[0])))
            roadMgr.updateBridgeParameters(road)
            roadMgr.setDirty(road)
          end
          im.tooltip('Set the half-width of the bridge, in meters (lateral distance from center to edge).')
          if im.InputFloat("Bridge Depth (m) ###10101", road.bridgeDepth, 0.1, 0.0) then
            road.bridgeDepth = im.FloatPtr(max(0.4, min(4.0, road.bridgeDepth[0])))
            roadMgr.updateBridgeParameters(road)
            roadMgr.setDirty(road)
          end
          im.tooltip('Set the depth of the bridge, in meters (vertical distance from bottom to top).')
          if im.InputFloat("Bridge Arch Amount (m) ###10102", road.bridgeArch, 0.1, 0.0) then
            road.bridgeArch = im.FloatPtr(max(-20.0, min(20.0, road.bridgeArch[0])))
            roadMgr.setDirty(road)
          end
          im.tooltip('Set the depth of the bridge, in meters (vertical distance from bottom to top).')
          im.PopItemWidth()
        end

        -- Overlay Options.
        if road.isOverlay then
          im.Separator()
          im.TextColored(cols.greenB, 'Overlay Material:')
          im.Columns(2, "overlayRowCBoxes2", false)
          im.SetColumnWidth(0, 30)
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatOverlayBtn') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForOverlay = true
          end
          im.tooltip('Select a new material for this overlay (choose an edge material)')
          im.SameLine()
          im.NextColumn()
          im.Text(roads[mfe.selectedRoadIdx].overlayMat or 'None')
          im.tooltip('The currently-selected material for this overlay.')
          im.NextColumn()

          im.Columns(1)
        end

        -- Terraforming options.
        local selRoad = roads[mfe.selectedRoadIdx]
        if terrain and #roads > 0 and selRoad and #selRoad.nodes > 1 and not selRoad.isBridge and not selRoad.isOverlay then
          im.Separator()
          if im.TreeNode1("Terraform Control") then
            im.Columns(3, 'TerraRoadRow1', false)
            if editor.uiIconImageButton(editor.icons.terrainToLine, vec36, cols.greenB, nil, nil, 'terraformSingle') then
              terra.conformTerrainToRoad(mfe.selectedRoadIdx, terraParams.domainOfInfluence[0], terraParams.terraMargin[0])
            end
            im.tooltip('Terraform the terrain to this single road (warning: ignores other roads).')
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.terrainToTwoLines, vec36, cols.greenB, nil, nil, 'terraformFullRoadNetwork') then
              local allRoadsGroup = { list = {} }
              local ctr = 1
              for i = 1, #roads do
                local tR = roads[i]
                for j = 1, #tR.nodes do
                  allRoadsGroup.list[ctr] = { r = tR.name, n = j }
                  ctr = ctr + 1
                end
              end
              terra.terraformMultiRoads(terraParams.domainOfInfluence[0], terraParams.terraMargin[0], allRoadsGroup)
            end
            im.tooltip('Terraform the terrain to all roads together.')
            im.SameLine()
            im.NextColumn()
            if im.Checkbox("Show", terraParams.isShowSingleRoad) then
              if terraParams.isShowSingleRoad[0] then
                terraParams.isShowGroup = im.BoolPtr(false)
              end
            end
            im.tooltip('Show the proposed terraforming range on the map, for the selected road.')
            im.SameLine()
            im.Dummy(vec36)
            im.NextColumn()
            im.Columns(1)

            -- 'Domain Of Influence' and 'Margin' sliders (for terraforming).
            im.PushItemWidth(-1)
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            im.SliderInt("###49", terraParams.domainOfInfluence, 1, 500, "Domain Of Influence (m) %d")
            im.tooltip('Set the domain of influence of the terraforming, in meters.')
            im.SliderFloat("###48", terraParams.terraMargin, 0.0, 20.0, "Margin (m) = %.3f")
            im.tooltip('Set the terraforming margin (around road), in meters.')
            im.PopStyleVar()
            im.PopItemWidth()
            im.TreePop()
          end
        end

        -- Further Road Edit Options.
        im.PushItemWidth(-1)
        if not road.isOverlay and not road.isBridge then
          im.Separator()
          if im.TreeNode1("Granularity Control") then
            if im.SliderInt("###859", road.granFactor, 1, 3, "Granularity Level (m) %d") then
              roadMgr.setDirty(road)
            end
            im.tooltip('Set the granularity level for this road (less = coarse, more = dense).')
            im.TreePop()
          end

          -- 'Master Width' control.
          im.Separator()
          if im.TreeNode1("Master Width Control") then
            if im.SliderFloat("###103", masterWidth, 0.5, 10.0, "Master Lane Width (m) = %.3f") then
              profileMgr.applyMasterWidth(road.profile, masterWidth[0])
              roadMgr.updateWAndHToNewProfile(road)
              roadMgr.setDirty(road)
            end
            im.tooltip('Set all road lanes to a master width (over-writes profile).')
            im.TreePop()
          end

          -- 'Translation Parameters' button.
          im.Separator()
          if im.TreeNode1("Translation Control") then
            local useRigidTranButtonCol = cols.greenD
            if road.isRigidTranslation[0] then useRigidTranButtonCol = cols.greenB end
            if editor.uiIconImageButton(editor.icons.transform, vec36, useRigidTranButtonCol, nil, nil, 'UseRigidTranslationButton') then
              local roadPre = copyDataState()
              road.isRigidTranslation = im.BoolPtr(not road.isRigidTranslation[0])
              local roadPost = copyDataState()
              editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
            end
            im.tooltip('Switch on/off rigid translation mode for this road.')

            -- Force Field slider (for non-rigid translations).
            if not road.isRigidTranslation[0] then
              im.SliderFloat("###3", road.forceField, 1.0, 205.0, "Movement Field = %.3f")
              im.tooltip('Amount of nearby elastic effect when dragging single nodes.')
            end
            im.TreePop()
          end

          -- 'Use Auto Banking' checkbox.
          im.Separator()
          if im.TreeNode1("Auto Banking Control") then
            local profile = road.profile
            im.Columns(2, 'autoBankingBtnCols1', false)
            if im.Checkbox("Use Auto Banking", profile.isAutoBanking) then
              if profile.isAutoBanking[0] then
                local nodes = road.nodes
                for i = 1, #nodes do
                  nodes[i].rot = im.FloatPtr(0.0)
                  nodes[i].isAutoBanked = true
                end
              else
                local nodes = road.nodes
                for i = 1, #nodes do
                  nodes[i].isAutoBanked = false
                end
              end
              roadMgr.setDirty(road)
            end
            im.tooltip('Use auto banking on this road (applied on corners only).')
            im.SameLine()
            im.NextColumn()

            -- 'Use Extra Corner Width' checkbox.
            if im.Checkbox("Extra Hairpin Width", profile.isExtraWidth) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Apply extra width to hairpin corners (> 90 degrees).')
            im.NextColumn()

            im.Columns(1)
            if profile.isAutoBanking[0] then
              if im.SliderFloat("###2948", profile.autoBankingFactor, 0.0, 2.0, "Depth (m) = %.2f") then
                roadMgr.setDirty(road)
              end
              im.tooltip('The amount of auto-banking to apply [0.5 = half, 1 = standard, 2 = double, etc].')
            end
            im.TreePop()
          end
          im.PopItemWidth()
        end

        -- Layer Options.
        im.Columns(1)
        if not road.isOverlay and not road.isBridge then
          im.Separator()
          if im.TreeNode1("Layers") then
            local profile = road.profile
            local layers = profile.layers
            local lMin, lMax = profileMgr.getMinMaxLaneKeys(profile)

            im.PushItemWidth(-1)
            if im.BeginListBox('', im.ImVec2(-1, 200)) then

              im.Columns(8, "layersListBoxColumns", true)
              im.SetColumnWidth(0, 30)
              im.SetColumnWidth(1, 30)
              im.SetColumnWidth(2, 30)
              im.SetColumnWidth(3, 150)
              im.SetColumnWidth(4, 30)
              im.SetColumnWidth(5, 30)
              im.SetColumnWidth(6, 30)
              im.SetColumnWidth(7, 65)

              local numLayers = #layers
              local wCtr = 4320
              for i = 1, numLayers do
                local layer = layers[i]
                if not layer.isHidden then
                  local flag = i == mfe.selectedLayerIdx
                  if im.Selectable1(tostring(i) .. "###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                    mfe.selectedLayerIdx = max(1, min(numLayers, i))
                  end
                  im.tooltip('The layer render priority (1 = top).')
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()

                  -- 'Increase Render Priority' button.
                  if i ~= 1 then
                    if editor.uiIconImageButton(editor.icons.arrow_upward, vec24, cols.blueB, nil, nil, 'layerPriorityUpButton') then
                      profileMgr.layerChangePriority(profile, i, 'raise')
                      mfe.selectedLayerIdx = max(1, min(numLayers, i - 1))
                    end
                    im.tooltip('Increase the render priority of this layer.')
                  end
                  im.SameLine()
                  im.NextColumn()

                  -- 'Decrease Render Priority' button.
                  if i ~= numLayers then
                    if editor.uiIconImageButton(editor.icons.arrow_downward, vec24, cols.blueB, nil, nil, 'layerPriorityDownButton') then
                      profileMgr.layerChangePriority(profile, i, 'lower')
                      mfe.selectedLayerIdx = max(1, min(numLayers, i + 1))
                    end
                    im.tooltip('Decrease the render priority of this layer.')
                  end
                  im.SameLine()
                  im.NextColumn()

                  im.PushItemWidth(150)
                  if im.InputText("###" .. tostring(wCtr), layer.name, 32) then
                    mfe.selectedLayerIdx = max(1, min(numLayers, i))
                  end
                  im.PopItemWidth()
                  im.tooltip('Edit the layer name.')
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()

                  -- 'Remove Selected Layer' button.
                  if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeLayer') then
                    profileMgr.removeLayer(profile, i)
                    mfe.selectedLayerIdx = max(1, min(numLayers, i))
                    return
                  end
                  im.tooltip('Remove this layer from the session.')
                  im.SameLine()
                  im.NextColumn()

                  -- 'Add New Layer Above' button.
                  if editor.uiIconImageButton(editor.icons.vertical_align_top, vec24, cols.greenB, nil, nil, 'addLayerAboveBtn') then
                    profileMgr.addLayer(profile, i, 'above')
                    mfe.selectedLayerIdx = max(1, min(numLayers, i))
                    return
                  end
                  im.tooltip('Add a new layer above this layer.')
                  im.SameLine()
                  im.NextColumn()

                  -- 'Add New Layer Below' button.
                  if editor.uiIconImageButton(editor.icons.vertical_align_bottom, vec24, cols.greenB, nil, nil, 'addLayerBelowBtn') then
                    profileMgr.addLayer(profile, i, 'below')
                    mfe.selectedLayerIdx = max(1, min(numLayers, i + 1))
                    return
                  end
                  im.tooltip('Add a new layer below this layer.')
                  im.SameLine()
                  im.NextColumn()

                  -- 'Show On Visualisation' checkbox.
                  if im.Checkbox("Show###" .. tostring(wCtr), layer.isDisplay) then
                    mfe.selectedLayerIdx = max(1, min(numLayers, i))
                  end
                  wCtr = wCtr + 1
                  im.tooltip('Highlights the layer on the road visualization.')
                  im.NextColumn()

                  im.Separator()
                end
              end
              im.EndListBox()
            end
            im.PopItemWidth()

            im.Columns(1)

            if editor.uiIconImageButton(editor.icons.add_box, vec36, cols.blueB, nil, nil, 'addNewLayerBtn') then
              profileMgr.addLayer(profile, #layers + 1, 'above')
              mfe.selectedLayerIdx = #layers
            end
            im.tooltip('Adds a new layer.')

            -- Selected Layer Edit section.
            local selLayerIdx = mfe.selectedLayerIdx
            local layer = layers[selLayerIdx]
            if #layers > 0 and selLayerIdx and layer then
              local layerType = layer.type[0]
              local wCtr = 300

              im.Columns(1)
              im.TextColored(cols.greenB, "Layer Type:")
              im.Columns(4)
              im.RadioButton2("L-Span###" .. tostring(wCtr), layer.type, 0)
              im.tooltip('A longitudinal layer which spans a lane/multiple adjacent lanes.')
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("Offset###" .. tostring(wCtr), layer.type, 1)
              im.tooltip('A longitudinal layer which has a fixed width, and is offset from one side of a lane.')
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("Patch###" .. tostring(wCtr), layer.type, 2)
              im.tooltip('A single-instance layer, at some chosen position along the road. Spans a lane/multiple lanes.')
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("Decal###" .. tostring(wCtr), layer.type, 3)
              im.tooltip('A single-instance layer, at some chosen position along the road, and indexes a larger tiled-material. Has a fixed size.')
              wCtr = wCtr + 1
              im.NextColumn()
              im.Columns(1)
              im.Columns(2)
              im.RadioButton2("Road-Span Mesh###" .. tostring(wCtr), layer.type, 4)
              im.tooltip('A road-spanning layer comprised of adjacent static mesh units.')
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("Single Mesh###" .. tostring(wCtr), layer.type, 5)
              im.tooltip('A single-instance static mesh, at some chosen position along the road.')
              wCtr = wCtr + 1
              im.NextColumn()

              im.Columns(1)

              -- DecalRoad and Decal type layers.
              if layerType < 4 then

                if layerType == 0 or layerType == 1 or layerType == 2 then
                  im.PushItemWidth(-150)
                  if im.InputFloat("Texture Length###" .. tostring(wCtr), layer.texLen, 0.1, 0.0) then
                    layer.texLen = im.FloatPtr(max(1.0, min(200.0, layer.texLen[0])))
                  end
                  wCtr = wCtr + 1
                  im.tooltip('Set the texture length of the material.')
                  im.PopItemWidth()
                end

                -- 'Select Material' row.
                im.TextColored(cols.greenB, "Layer Material:")
                im.Columns(2, "layerSecondRowCBoxes2", false)
                im.SetColumnWidth(0, 30)
                if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtn') then
                  setMaterialList()
                  editor.showWindow(win.materialSelectWinName)
                  mfe.isMaterialSelectWinOpen = true
                  mfe.selProfileMaterial = profile
                  mfe.selectedLayerIdx = selLayerIdx
                  mfe.isMaterialForRoad = false
                  mfe.isMaterialForJctArrows = false
                  mfe.isMaterialForEdgeBlendLeft = false
                  mfe.isMaterialForEdgeBlendRight = false
                  mfe.isMaterialForOverlay = false
                end
                im.tooltip('Select a new material for layer ' .. tostring(selLayerIdx))
                im.SameLine()
                im.NextColumn()
                im.Text(layer.mat or 'None')
                im.tooltip('The currently-selected material for layer ' .. tostring(selLayerIdx))
                im.NextColumn()

                im.Columns(1)

                im.TextColored(cols.greenB, "Attach Properties:")

                if layerType == 0 or layerType == 2 then
                  -- 'Min Lane' input box.
                  im.PushItemWidth(-150)
                  local oldVal = layer.laneMin[0]
                  im.InputInt("Min Lane Index###" .. tostring(wCtr), layer.laneMin, 1)
                  im.tooltip('The index of the left-most lane, from which this layer will span.')
                  wCtr = wCtr + 1
                  if layer.laneMin[0] == 0 then layer.laneMin = im.IntPtr(-oldVal) end
                  layer.laneMin = im.IntPtr(max(lMin, min(min(lMax, layer.laneMax[0]), layer.laneMin[0])))

                  -- 'Max Lane' input box.
                  local oldVal = layer.laneMax[0]
                  im.InputInt("Max Lane Index###" .. tostring(wCtr), layer.laneMax, 1)
                  im.tooltip('The index of the right-most lane, to which this layer will span.')
                  wCtr = wCtr + 1
                  if layer.laneMax[0] == 0 then layer.laneMax = im.IntPtr(-oldVal) end
                  layer.laneMax = im.IntPtr(max(max(layer.laneMin[0], lMin), min(lMax, layer.laneMax[0])))
                  im.PopItemWidth()
                end

                if layerType == 1 or layerType == 3 then
                  -- 'Layer Lane Index' input box.
                  im.PushItemWidth(-150)
                  local oldVal = layer.lane[0]
                  im.InputInt("Lane Index###" .. tostring(wCtr), layer.lane, 1)
                  im.tooltip('The index of the lane to which this layer is attached.')
                  wCtr = wCtr + 1
                  if layer.lane[0] == 0 then layer.lane = im.IntPtr(-oldVal) end
                  layer.lane = im.IntPtr(max(lMin, min(lMax, layer.lane[0])))
                  im.PopItemWidth()
                end

                -- 'Layer Lateral/Longitudinal Offset' input box.
                im.PushItemWidth(-150)
                local limitMin, limitMax, fScale = -10.0, 10.0, 0.1
                if layerType == 2 or layerType == 3 then
                  limitMin, limitMax, fScale = 0.0, 1.0, 0.001
                end
                im.InputFloat("Layer Position###" .. tostring(wCtr), layer.off, fScale, 0.0)
                wCtr = wCtr + 1
                im.tooltip('Set the layer position. For non-patch types: laterally from the attach lane. For patch/decal types: longitudinally along the length of the road.')
                layer.off = im.FloatPtr(max(limitMin, min(limitMax, layer.off[0])))

                -- 'Layer Width' input box.
                if layerType == 1 or layerType == 2 then
                  im.InputFloat("Layer Width###" .. tostring(wCtr), layer.width, 0.1, 0.0)
                  layer.width = im.FloatPtr(max(0.01, min(30.0, layer.width[0])))
                  wCtr = wCtr + 1
                  im.tooltip('Set the width of layer ' .. tostring(selLayerIdx))
                end

                if layerType == 3 then
                  im.InputFloat("Lateral Offset###" .. tostring(wCtr), layer.pos, 0.1, 0.0)
                  wCtr = wCtr + 1
                  im.tooltip('Set the lateral offset (from the chosen lane and lane side).')
                  layer.pos = im.FloatPtr(max(-50.0, min(50.0, layer.pos[0])))

                  im.InputInt("Material: Num Rows###" .. tostring(wCtr), layer.numRows, 1)
                  layer.numRows = im.IntPtr(max(1, min(99, layer.numRows[0])))
                  wCtr = wCtr + 1
                  im.tooltip('The number of rows in the selected (tiled) material.')

                  im.InputInt("Material: Num Columns###" .. tostring(wCtr), layer.numCols, 1)
                  layer.numCols = im.IntPtr(max(1, min(99, layer.numCols[0])))
                  wCtr = wCtr + 1
                  im.tooltip('The number of columns in the selected (tiled) material.')

                  im.InputInt("Frame Number###" .. tostring(wCtr), layer.frame, 1)
                  layer.frame = im.IntPtr(max(0, min(layer.numRows[0] * layer.numCols[0], layer.frame[0])))
                  wCtr = wCtr + 1
                  im.tooltip('The frame (tile) to use from the selected material.')

                  im.InputFloat("Decal Size###" .. tostring(wCtr), layer.size, 0.1, 0.0)
                  wCtr = wCtr + 1
                  im.tooltip('Set the size of the square decal (length = width).')
                  layer.size = im.FloatPtr(max(0.0, min(50.0, layer.size[0])))
                end

                -- 'Attach Left' checkbox.
                if layerType == 1 or layerType == 3 then
                  im.Checkbox("Attach Left/Right###" .. tostring(wCtr), layer.isLeft)
                  wCtr = wCtr + 1
                  im.tooltip('Attach to left side of lane (checked), or right (unchecked).')
                end

                -- Pre-rotation row.
                if layerType == 3 then
                  im.PushItemWidth(-1)
                  im.TextColored(cols.greenB, "Rotation around Z-axis (degrees):")
                  im.Columns(4)
                  im.RadioButton2("0###" .. tostring(wCtr), layer.rot, 0)
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()
                  im.RadioButton2("90###" .. tostring(wCtr), layer.rot, 1)
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()
                  im.RadioButton2("180###" .. tostring(wCtr), layer.rot, 2)
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()
                  im.RadioButton2("270###" .. tostring(wCtr), layer.rot, 3)
                  wCtr = wCtr + 1
                  im.NextColumn()
                  im.PopItemWidth()
                  im.tooltip('Set the pre-rotation around the Z-axis for this mesh, to better align it on the lane.')

                  im.Columns(1)
                end

                if layerType == 0 or layerType == 1 then
                  im.Columns(2, 'type1or2ExtraColsB', false)
                  im.Checkbox("Span Road Length###" .. tostring(wCtr), layer.isSpanLong)
                  wCtr = wCtr + 1
                  im.tooltip('Layer will span the entire longitudinal road length (checked), or be limited to a node-to-node interval (unchecked).')
                  im.SameLine()
                  im.NextColumn()
                  im.Checkbox("Finish Before Ends###" .. tostring(wCtr), layer.isPaint)
                  wCtr = wCtr + 1
                  im.tooltip('Layer will leave extra space from the start/end of the road (useful for eg paint markings).')
                  im.NextColumn()

                  im.Columns(1)
                  if not layer.isSpanLong[0] then
                    im.InputInt("Min Node Index###" .. tostring(wCtr), layer.nMin, 1)
                    layer.nMin = im.IntPtr(max(1, min(layer.nMax[0], min(#road.nodes or 1, layer.nMin[0]))))
                    wCtr = wCtr + 1
                    im.tooltip('The start node index, for this layer.')
                    im.InputInt("Max Node Index###" .. tostring(wCtr), layer.nMax, 1)
                    layer.nMax = im.IntPtr(max(layer.nMin[0] , max(1, min(#road.nodes or 1, layer.nMax[0]))))
                    wCtr = wCtr + 1
                    im.tooltip('The end node index, for this layer.')
                  end

                  im.InputFloat("Fade-In [Start]###" .. tostring(wCtr), layer.fadeS, 0.01, 0.0)
                  layer.fadeS = im.FloatPtr(max(0.0, min(100.0, layer.fadeS[0])))
                  wCtr = wCtr + 1
                  im.tooltip('Set the fade-in value of layer ' .. tostring(selLayerIdx))
                  im.InputFloat("Fade-Out [End]###" .. tostring(wCtr), layer.fadeE, 0.01, 0.0)
                  layer.fadeE = im.FloatPtr(max(0.0, min(100.0, layer.fadeE[0])))
                  wCtr = wCtr + 1
                  im.tooltip('Set the fade-out value of layer ' .. tostring(selLayerIdx))
                end

                if layer.type[0] == 0 or layer.type[0] == 1 then
                  im.TextColored(cols.greenB, 'General Properties:')
                  im.Checkbox("Reverse###" .. tostring(wCtr), layer.isReverse)
                  wCtr = wCtr + 1
                  im.tooltip('Flip the direction of this layer (checked), or not (unchecked). Useful for edge blending and gutter layers, for example.')
                end

              elseif layerType == 4 then
                -- Road-spanning mesh type layers.

                -- 'Open Material Selection Window' button.
                im.TextColored(cols.greenB, 'Static Mesh:')
                im.Columns(2, "laneMeshListBoxColumns", true)
                im.SetColumnWidth(0, 30)
                if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectStaticMeshBtn1') then
                  if not hasMeshListBeenComputed then
                    staticMgr.fetchAvailableStaticMeshes()                                              -- Fetch the available static meshes list.
                    hasMeshListBeenComputed = true
                  end
                  mfe.selectedMeshLaneIdx = mfe.selectedLayerIdx
                  mfe.selectedCustom = mfe.selectedLayerIdx
                  mfe.isSingleMeshSelect = false
                  mfe.isMaterialForRoad = false
                  mfe.isMaterialForEdgeBlendLeft = false
                  mfe.isMaterialForEdgeBlendRight = false
                  mfe.isMaterialForJctArrows = false
                  mfe.isMaterialForOverlay = false
                  staticMgr.addMeshToAudition(mfe.selectedMeshIdx, road, mfe.selectedLayerIdx)
                  editor.showWindow(win.meshSelectWinName)
                  mfe.isMeshSelectWinOpen = true
                end
                im.tooltip('Select a static mesh unit for this layer.')
                im.SameLine()
                im.NextColumn()

                -- Material name display column.
                local locMatDisplay = layer.matDisplay
                if locMatDisplay == '' or locMatDisplay == '[None]' or locMatDisplay == '[none]' then
                  im.Text('[select mesh for this lane]')
                else
                  im.Text(locMatDisplay)
                end
                im.tooltip('The selected static mesh unit.')
                im.NextColumn()

                im.Columns(1)
                im.TextColored(cols.greenB, 'Pose Parameters:')

                -- Top row.
                im.Columns(1)

                im.Columns(2, 'singleMeshUnitTopRowCols', false)
                im.Checkbox("Span Road Length###" .. tostring(wCtr), layer.isSpanLong)
                wCtr = wCtr + 1
                im.tooltip('Layer will span the entire longitudinal road length (checked), or be limited to a node-to-node interval (unchecked).')
                im.SameLine()
                im.NextColumn()

                im.Checkbox("Attach To Left/Right###" .. tostring(wCtr), layer.isLeft)
                wCtr = wCtr + 1
                im.tooltip('Attach to left side of lane (checked), or right (unchecked).')
                im.NextColumn()
                im.Columns(1)

                if not layer.isSpanLong[0] then
                  im.PushItemWidth(-150)
                  im.InputInt("Node Index [Min]###" .. tostring(wCtr), layer.nMin, 1)
                  layer.nMin = im.IntPtr(max(1, min(layer.nMax[0], min(#road.nodes or 1, layer.nMin[0]))))
                  wCtr = wCtr + 1
                  im.tooltip('The start node index, for this layer.')
                  im.InputInt("Node Index [Max]###" .. tostring(wCtr), layer.nMax, 1)
                  layer.nMax = im.IntPtr(max(layer.nMin[0] , max(1, min(#road.nodes or 1, layer.nMax[0]))))
                  wCtr = wCtr + 1
                  im.tooltip('The end node index, for this layer.')
                  im.PopItemWidth()
                end

                im.Checkbox("Use World Z-Value###" .. tostring(wCtr), layer.useWorldZ)
                wCtr = wCtr + 1
                im.tooltip('Sets whether to align the mesh Z-axis with the world Z-axis.')

                -- Property input boxes.
                im.PushItemWidth(-150)
                im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
                im.InputFloat("Spacing Between Units###" .. tostring(wCtr), layer.spacing, 0.1, 0.0)
                layer.spacing = im.FloatPtr(max(-0.2, min(200.0, layer.spacing[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the longitudinal spacing between each unit of the custom mesh lane.')
                local oldVal = layer.lane[0]
                im.InputInt("Lane Index###" .. tostring(wCtr), layer.lane, 1)
                im.tooltip('The index of the lane to which this mesh unit is attached.')
                wCtr = wCtr + 1
                if layer.lane[0] == 0 then layer.lane = im.IntPtr(-oldVal) end
                local lMin, lMax = profileMgr.getMinMaxLaneKeys(road.profile)
                layer.lane = im.IntPtr(max(lMin, min(lMax, layer.lane[0])))
                im.tooltip('Set the vertical offset of the custom mesh.')
                im.InputFloat("Lateral Offset###" .. tostring(wCtr), layer.latOffset, 0.1, 0.0)
                layer.latOffset = im.FloatPtr(max(-100.0, min(100.0, layer.latOffset[0])))
                wCtr = wCtr + 1
                im.InputFloat("Vertical Offset###" .. tostring(wCtr), layer.vertOffset, 0.1, 0.0)
                layer.vertOffset = im.FloatPtr(max(-100.0, min(100.0, layer.vertOffset[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the lateral offset of the custom mesh.')
                im.InputFloat("Amount Of Jitter###" .. tostring(wCtr), layer.jitter, 0.001, 0.0)
                layer.jitter = im.FloatPtr(max(0.0, min(0.2, layer.jitter[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the amount of random jitter of the custom mesh.')
                im.PopStyleVar()
                im.PopItemWidth()

                im.Columns(1)

                -- Pre-rotation row.
                im.PushItemWidth(-1)
                im.TextColored(cols.greenB, "Pre-rotation around Z-axis (degrees):")
                im.Columns(4)
                im.RadioButton2("0###" .. tostring(wCtr), layer.rot, 0)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("90###" .. tostring(wCtr), layer.rot, 1)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("180###" .. tostring(wCtr), layer.rot, 2)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("270###" .. tostring(wCtr), layer.rot, 3)
                wCtr = wCtr + 1
                im.NextColumn()
                im.PopItemWidth()
                im.tooltip('Set the pre-rotation around the Z-axis for this mesh, to better align it on the lane.')

                im.Columns(1)

              elseif layerType == 5 then
                -- Single mesh type layers.

                im.TextColored(cols.greenB, 'Static Mesh:')
                im.Columns(2, "singleMeshListBoxColumns", true)
                im.SetColumnWidth(0, 30)
                if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectStaticMeshBtn1') then
                  if not hasMeshListBeenComputed then
                    staticMgr.fetchAvailableStaticMeshes()                                              -- Fetch the available static meshes list.
                    hasMeshListBeenComputed = true
                  end
                  mfe.selectedSingleIdx = mfe.selectedLayerIdx
                  mfe.selectedCustom = mfe.selectedLayerIdx
                  mfe.isSingleMeshSelect = true
                  mfe.isMaterialForRoad = false
                  mfe.isMaterialForEdgeBlendLeft = false
                  mfe.isMaterialForEdgeBlendRight = false
                  mfe.isMaterialForJctArrows = false
                  mfe.isMaterialForOverlay = false
                  staticMgr.addMeshToAudition(mfe.selectedMeshIdx, road, mfe.selectedLayerIdx)
                  editor.showWindow(win.meshSelectWinName)
                  mfe.isMeshSelectWinOpen = true
                end
                im.tooltip('Select a static mesh unit.')
                im.SameLine()
                im.NextColumn()

                -- Material name display column.
                local locMatDisplay = layer.matDisplay
                if locMatDisplay == '' or locMatDisplay == '[None]' or locMatDisplay == '[none]' then
                  im.Text('[select a mesh unit]')
                else
                  im.Text(locMatDisplay)
                end
                im.tooltip('The selected static mesh unit.')
                im.NextColumn()

                im.Columns(1)
                im.TextColored(cols.greenB, 'Pose Parameters:')

                im.PushItemWidth(-150)
                local oldVal = layer.lane[0]
                im.InputInt("Lane Index###" .. tostring(wCtr), layer.lane, 1)
                im.tooltip('The index of the lane to which this mesh unit is attached.')
                wCtr = wCtr + 1
                if layer.lane[0] == 0 then layer.lane = im.IntPtr(-oldVal) end
                local lMin, lMax = profileMgr.getMinMaxLaneKeys(road.profile)
                layer.lane = im.IntPtr(max(lMin, min(lMax, layer.lane[0])))

                im.Checkbox("Attach To Left/Right###" .. tostring(wCtr), layer.isLeft)
                wCtr = wCtr + 1
                im.tooltip('Attach to left side of lane (checked), or right (unchecked).')

                im.InputFloat("Position###" .. tostring(wCtr), layer.pos, 0.001, 0.0)
                layer.pos = im.FloatPtr(max(0.0, min(1.0, layer.pos[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the longitudinal position (along the road) for this mesh unit.')

                im.InputFloat("Lateral Offset###" .. tostring(wCtr), layer.latOffset, 0.1, 0.0)
                layer.latOffset = im.FloatPtr(max(-20.0, min(20.0, layer.latOffset[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the lateral offset of the mesh unit.')

                im.InputFloat("Vertical Offset###" .. tostring(wCtr), layer.vertOffset, 0.1, 0.0)
                layer.vertOffset = im.FloatPtr(max(-20.0, min(20.0, layer.vertOffset[0])))
                wCtr = wCtr + 1
                im.tooltip('Set the vertical offset of the mesh unit.')
                im.PopItemWidth()

                -- Pre-rotation row.
                im.PushItemWidth(-1)
                im.TextColored(cols.greenB, "Pre-rotation around Z-axis (degrees):")
                im.Columns(4)
                im.RadioButton2("0###" .. tostring(wCtr), layer.rot, 0)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("90###" .. tostring(wCtr), layer.rot, 1)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("180###" .. tostring(wCtr), layer.rot, 2)
                wCtr = wCtr + 1
                im.SameLine()
                im.NextColumn()
                im.RadioButton2("270###" .. tostring(wCtr), layer.rot, 3)
                wCtr = wCtr + 1
                im.NextColumn()
                im.PopItemWidth()
                im.tooltip('Set the pre-rotation around the Z-axis for this mesh, to better align it on the lane.')

                im.Columns(1)
              end
              im.Columns(1)
            end
            im.TreePop()
          end
        end

        -- Sidewalk Options.
        im.Columns(1)
        local laneFlags = profileMgr.getLaneFlags()
        if not road.isOverlay and not road.isBridge and laneFlags.isSidewalk then
          im.Separator()
          if im.TreeNode1("Sidewalks") then
            local profile = road.profile
            local wCtr = 400
            local sWKeys = {}
            im.PushItemWidth(-1)
            if im.BeginListBox('') then

              im.Columns(2, "sidewalkListBoxColumns", true)
              im.SetColumnWidth(0, 30)
              im.SetColumnWidth(1, 150)

              for iii = -20, 20 do
                if profile[iii] and profile[iii].type == 'sidewalk' then
                  sWKeys[iii] = true
                  local flag = iii == mfe.selectedSidewalkIdx
                  if im.Selectable1(tostring(iii) .. "###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                    mfe.selectedSidewalkIdx = iii
                  end
                  im.tooltip('The lane Id.')
                  wCtr = wCtr + 1
                  im.SameLine()
                  im.NextColumn()

                  local sideFacingText = 'Right-Facing'
                  if profile[iii].isLeftSide[0] then
                    sideFacingText = 'Left-Facing'
                  end
                  im.Text(sideFacingText)
                  im.tooltip('Which way the sidewalk faces (Left-Facing = curb on left, Right-Facing = curb on right).')
                  im.NextColumn()

                  im.Separator()
                end
              end
              im.EndListBox()
            end
            im.PopItemWidth()

            im.Columns(1)

            -- Individual sidewalk edit options.
            local selSW = mfe.selectedSidewalkIdx
            if selSW and sWKeys[selSW] then

              -- 'Is Left Side' checkbox.
              im.TextColored(cols.greenB, "Orientation Parameters:")
              im.PushItemWidth(-150)
              im.Checkbox("Left Side###" .. tostring(wCtr), profile[selSW].isLeftSide)
              wCtr = wCtr + 1
              im.tooltip('Left side sidewalk (checked), or right side sidewalk (unchecked).')

              -- 'Curb Width' input box.
              im.TextColored(cols.greenB, "Curb Shaping Parameters:")
              im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
              if im.InputFloat("Curb Width###" .. tostring(wCtr), profile[selSW].kerbWidth, 0.1, 0.0) then
                profile[selSW].kerbWidth = im.FloatPtr(max(0.05, min(10.0, profile[selSW].kerbWidth[0])))
              end
              wCtr = wCtr + 1
              im.tooltip('The curb width.')

              -- 'Curb Corner Lateral Offset' input box.
              if im.InputFloat("Curb Lateral Offset###" .. tostring(wCtr), profile[selSW].cornerLatOff, 0.01, 0.0) then
                profile[selSW].cornerLatOff = im.FloatPtr(max(-0.3, min(0.3, profile[selSW].cornerLatOff[0])))
              end
              wCtr = wCtr + 1
              im.tooltip('The lateral offset of the curb corner.')

              -- 'Curb Corner Height Offset' input box.
              if im.InputFloat("Vertical Offset###" .. tostring(wCtr), profile[selSW].cornerDrop, 0.01, 0.0) then
                profile[selSW].cornerDrop = im.FloatPtr(max(-0.1, min(0.1, profile[selSW].cornerDrop[0])))
              end
              wCtr = wCtr + 1
              im.tooltip('The vertical offset of the curb corner.')
              im.PopStyleVar()

              im.TextColored(cols.greenB, "Curb Texture Mapping Position:")
              im.Columns(4)
              im.RadioButton2("[A]###" .. tostring(wCtr), profile[selSW].vStart, 0)
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("[B]###" .. tostring(wCtr), profile[selSW].vStart, 1)
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("[C]###" .. tostring(wCtr), profile[selSW].vStart, 2)
              wCtr = wCtr + 1
              im.SameLine()
              im.NextColumn()
              im.RadioButton2("[D]###" .. tostring(wCtr), profile[selSW].vStart, 3)
              wCtr = wCtr + 1
              im.NextColumn()
              im.tooltip("Set the starting 'V' position of the curb UV-mapping (each provides a different curb texture).")

              im.PopItemWidth()

              im.Columns(1)
            end
            im.TreePop()
          end
        end

        -- Tunnels - further options.
        if not road.isOverlay and not road.isBridge and #road.tunnels > 0 then
          im.Separator()
          if im.TreeNode1("Tunnel Control") then

            -- Wall Thickness.
            im.PushItemWidth(-80)
            if im.InputFloat("Wall Depth", road.thickness, 0.1, 0.0) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Sets the thickness of the tunnel walls.')
            road.thickness = im.FloatPtr(max(0.1, min(20.0, road.thickness[0])))
            im.PopItemWidth()

            -- Radius offset.
            im.PushItemWidth(-80)
            if im.InputFloat("R Offset", road.radOffset, 0.1, 0.0) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Sets the tunnel radius offset.')
            road.radOffset = im.FloatPtr(max(-10.0, min(10.0, road.radOffset[0])))
            im.PopItemWidth()

            -- Wall Thickness.
            im.PushItemWidth(-80)
            if im.InputFloat("Z Offset", road.zOffsetFromRoad, 0.1, 0.0) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Sets the vertical offset of the road inside the tunnel.')
            road.zOffsetFromRoad = im.FloatPtr(max(-30.0, min(30.0, road.zOffsetFromRoad[0])))
            im.PopItemWidth()

            -- Start protrusion amount.
            im.PushItemWidth(-80)
            if im.InputFloat("Extend S", road.protrudeS, 0.1, 0.0) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Sets the amount of extension at the start of the tunnel.')
            road.protrudeS = im.FloatPtr(max(0.0, min(30.0, road.protrudeS[0])))
            im.PopItemWidth()

            -- End protrusion amount.
            im.PushItemWidth(-80)
            if im.InputFloat("Extend E", road.protrudeE, 0.1, 0.0) then
              roadMgr.setDirty(road)
            end
            im.tooltip('Sets the amount of extension at the end of the tunnel.')
            road.protrudeE = im.FloatPtr(max(0.0, min(30.0, road.protrudeE[0])))
            im.PopItemWidth()

            -- Start position on road (longitudinal div point).
            im.PushItemWidth(-1)
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            if im.SliderInt("###42", road.extraS, 0, 15, "Start Pos %d") then
              roadMgr.setDirty(road)
            end
            im.PopStyleVar()
            im.PopItemWidth()
            im.tooltip('Sets the general tunnel start position, on the road.')

            -- End position on road (longitudinal div point).
            im.PushItemWidth(-1)
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            if im.SliderInt("###43", road.extraE, 0, 15, "End Pos %d") then
              roadMgr.setDirty(road)
            end
            im.PopStyleVar()
            im.PopItemWidth()
            im.tooltip('Sets the general tunnel end position, on the road.')

            -- Tunnel mesh granularity.
            im.PushItemWidth(-1)
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            if im.SliderInt("###41", road.radGran, 6, 50, "Granularity %d") then
              roadMgr.setDirty(road)
            end
            im.PopStyleVar()
            im.PopItemWidth()
            im.tooltip('Tunnel mesh granularity.')

            im.Separator()
            im.TreePop()
          end
        end
      end

      -- 'Road Condition' parameters.
      if #roads > 0 and road and not road.isOverlay and not road.isBridge and mfe.selectedRoadIdx and roads[mfe.selectedRoadIdx] and not roads[mfe.selectedRoadIdx].isJctRoad then
        im.Separator()
        if im.TreeNode1("Road Condition Control") then
          local profile = road.profile
          im.PushItemWidth(-150)
          im.TextColored(cols.greenB, "Road Class:")
          im.Columns(3)
          if im.RadioButton2("Urban###2222", profile.styleType, 0) then
            profile.class = 'urban'
            profileMgr.updateCondition(road)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Urban class (for street-based roads, eg with sidewalks, lamp posts, etc).')
          im.SameLine()
          im.NextColumn()
          if im.RadioButton2("Highway###2223", profile.styleType, 1) then
            profile.class = 'highway'
            profileMgr.updateCondition(road)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Highway class (for highways and intersections, eg with hard shoulders, crash barriers, etc).')
          im.SameLine()
          im.NextColumn()
          if im.RadioButton2("Dirt###2224", profile.styleType, 2) then
            profile.class = 'dirt'
            profileMgr.updateCondition(road)
            mfe.selectedLayerIdx = 1
          end
          im.NextColumn()
          im.tooltip('Dirt Road class (for tracks going across dirt, sand, etc).')

          im.Columns(1)

          -- 'Aphalt Style' options.
          if profile.class == 'urban' or profile.class == 'highway' then
            im.TextColored(cols.greenB, 'Road Condition:')
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            im.PushItemWidth(-1)
            if im.SliderFloat("###3160", profile.condition, 0.0, 1.0, "Road Condition = %.3f") then
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')
            if im.SliderInt("###3161",profile.numPatches, 0, 50, "Repair Patches = %d") then
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('The number of damage patches to include on the road.')

            if im.SliderInt("###3438", profile.numPotholes, 0, 50, "Pot Holes = %d") then
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('The number of pothole patches to include on the road.')
            im.PopItemWidth()
            if im.InputInt("Random Seed Value###3939", profile.conditionSeed, 1) then
              profile.conditionSeed = im.IntPtr(max(0, min(4294967295, profile.conditionSeed[0])))
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('Set the random seed for the road condition (change to get different wear patterns).')
            im.PopStyleVar()

            im.Columns(1)

            if im.InputFloat("Fade In [Start]###4188", profile.fadeS, 0.01, 0.0) then
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('Set the tire tread marks fade-in amount (from road start), in meters.')
            if im.InputFloat("Fade Out [End]###4189", profile.fadeE, 0.01, 0.0) then
              profileMgr.updateCondition(road)
              mfe.selectedLayerIdx = 1
            end
            im.tooltip('Set the tire tread marks fade-out amount (from road end), in meters.')

          elseif profile.class == 'dirt' then

            im.TextColored(cols.greenB, 'Material Parameters:')
            im.Columns(3, "dirtStyleTypeCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Dirt Material")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesDirt') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'dirt'
            end
            im.tooltip('Select a new material for the dirt track.')
            im.SameLine()
            im.NextColumn()
            im.Text(profile.dirtMat)
            im.tooltip('The currently-selected material for the dirt tracks.')
            im.NextColumn()

            im.Columns(1)
          end

          im.Columns(1)
          im.PopItemWidth()
          im.TreePop()
        end
      end

      -- Road Edit Display Parameters.
      if #roads > 0 and road and not road.isOverlay and not road.isBridge and mfe.selectedRoadIdx and roads[mfe.selectedRoadIdx] and not roads[mfe.selectedRoadIdx].isJctRoad then
        im.Separator()
        if im.TreeNode1("Visualization Control") then
          im.Columns(5, "visControlsColBtns1", false)

          -- The 'Road Surface Mesh' toggle button.
          local roadSurfaceColor = cols.blueD
          if road.isDisplayRoadSurface[0] then roadSurfaceColor = cols.blueB end
          if editor.uiIconImageButton(editor.icons.roadFace, vec36, roadSurfaceColor, nil, nil, 'roadSurfaceMeshToggle') then
            local roadPre = copyDataState()
            road.isDisplayRoadSurface = im.BoolPtr(not road.isDisplayRoadSurface[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the road surface mesh.')
          im.SameLine()
          im.NextColumn()

          -- The 'Road Outline Mesh' toggle button.
          local roadOutlineColor = cols.blueD
          if road.isDisplayRoadOutline[0] then roadOutlineColor = cols.blueB end
          if editor.uiIconImageButton(editor.icons.roadOutlineMesh, vec36, roadOutlineColor, nil, nil, 'roadOutlineMeshToggle') then
            local roadPre = copyDataState()
            road.isDisplayRoadOutline = im.BoolPtr(not road.isDisplayRoadOutline[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the road outline mesh (shows render granularity/quadrilaterals).')
          im.SameLine()
          im.NextColumn()

          -- The 'Road Reference Line' toggle button.
          local roadRefLineColor = cols.blueD
          if road.isDisplayRefLine[0] then roadRefLineColor = cols.blueB end
          if editor.uiIconImageButton(editor.icons.roadRefPath, vec36, roadRefLineColor, nil, nil, 'roadRefLineToggle') then
            local roadPre = copyDataState()
            road.isDisplayRefLine = im.BoolPtr(not road.isDisplayRefLine[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the road reference line (center line of road).')
          im.SameLine()
          im.NextColumn()

          im.Dummy(vec36)
          im.SameLine()
          im.NextColumn()

          -- Toggle the 'Lane Info' button.
          local laneInfoColor = cols.blueD
          if road.isDisplayLaneInfo[0] then laneInfoColor = cols.blueB end
          if editor.uiIconImageButton(editor.icons.roadInfo, vec36, laneInfoColor, nil, nil, 'laneInfoToggle') then
            local roadPre = copyDataState()
            road.isDisplayLaneInfo = im.BoolPtr(not road.isDisplayLaneInfo[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the lane info.')
          im.NextColumn()

          im.Columns(5, "firstRowBtns4", false)

          -- The 'Node Spheres' and 'Node Numbers toggle buttons.
          local nodeSpheresColor = cols.greenD
          if road.isDisplayNodeSpheres[0] then nodeSpheresColor = cols.greenB end
          if editor.uiIconImageButton(editor.icons.sphereOnMesh, vec36, nodeSpheresColor, nil, nil, 'nodeSpheresToggle') then
            local roadPre = copyDataState()
            road.isDisplayNodeSpheres = im.BoolPtr(not road.isDisplayNodeSpheres[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the node spheres.')
          im.SameLine()
          im.NextColumn()

          local nodeNumbersColor = cols.greenD
          if road.isDisplayNodeNumbers[0] then nodeNumbersColor = cols.greenB end
          if editor.uiIconImageButton(editor.icons.sphereOnPathNumber, vec36, nodeNumbersColor, nil, nil, 'nodeNumbersToggle') then
            local roadPre = copyDataState()
            road.isDisplayNodeNumbers = im.BoolPtr(not road.isDisplayNodeNumbers[0])
            local roadPost = copyDataState()
            editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
          end
          im.tooltip('Toggles the node numbering.')
          im.SameLine()
          im.NextColumn()

          im.Dummy(vec36)
          im.SameLine()
          im.NextColumn()
          im.Dummy(vec36)
          im.SameLine()
          im.NextColumn()
          im.Dummy(vec36)
          im.NextColumn()

          im.Columns(1)
          im.TreePop()
        end
      end

      -- Rendering options for this road.
      if #roads > 0 and road and mfe.selectedRoadIdx and roads[mfe.selectedRoadIdx] then
        im.Separator()
        if im.TreeNode1("Render Options") then
          im.Checkbox('isOverObject Flag', road.isOverObject)
          im.tooltip("All decals created by this road will have the 'isOverObjects' flag set true (checked), or set false (unchecked).")
          im.TreePop()
        end
      end

      -- Rendering options for this road.
      if #roads > 0 then
        im.Separator()
        if im.TreeNode1("Road Mask Export Options") then
          if editor.uiIconImageButton(editor.icons.terrain_export, vec36, cols.greenB, nil, nil, 'exportRoadMasks') then
            extensions.editor_fileDialog.saveFile(
              function(data)

                local allRoadsGroup = { list = {} }
                local ctr = 1
                for i = 1, #roadMgr.roads do
                  local tR = roadMgr.roads[i]
                  for j = 1, #tR.nodes do
                    allRoadsGroup.list[ctr] = { r = tR.name, n = j }
                    ctr = ctr + 1
                  end
                end
                local roadMask, marginMask = terra.getRoadMasks(150.0, 0.0, allRoadsGroup)

                local width, height = #roadMask, #roadMask[1]
                local bmp = GBitmap()
                bmp:init(width, height)
                bmp:allocateBitmap(width, height, false, "GFXFormatR16")
                for x = 0, width - 1 do
                  local xPlusOne = x + 1
                  for y = 0, height - 1 do
                    local val = roadMask[xPlusOne][y + 1]
                    bmp:setTexel(x, y, val, val, val, 255)
                  end
                end
                bmp:saveFile(data.filepath)
              end,
              {{"PNG",".png"}},
              false,
              "/",
              "File already exists.\nDo you want to overwrite the file?")
          end
          im.tooltip('Export the road masks to a file.')
          im.TreePop()
        end
      end

      im.EndTabItem()
    end

    -- 'Junctions' tab.
    if selectedTab == 2 then
      local junctions = jctMgr.junctions
      im.PushItemWidth(-1)
      if im.BeginListBox('') then

        -- Junctions list.
        im.Columns(6, "jctListBoxColumns", true)
        im.SetColumnWidth(0, 30)
        im.SetColumnWidth(1, 150)
        im.SetColumnWidth(2, 35)
        im.SetColumnWidth(3, 35)
        im.SetColumnWidth(4, 35)
        im.SetColumnWidth(5, 35)

        local wCtr = 3700
        for i = 1, #junctions do
          local jct = junctions[i]
          local flag = i == mfe.selectedJctIdx
          if im.Selectable1(tostring(i) .. "###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            mfe.selectedJctIdx = max(1, min(#junctions, i))
            addJctNodesToMulti(i, roadMgr.multi)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[i].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          im.tooltip('The Id of this junction.')
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          im.PushItemWidth(150)
          im.InputText("###" .. tostring(wCtr), jct.name, 32)
          im.PopItemWidth()
          im.tooltip('Edit the name of this junction.')
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          -- 'Remove Selected Junction' button.
          if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeJctBtn') then
            jctMgr.removeJunction(i)
            mfe.selectedJctIdx = max(1, min(#junctions, i - 1))
            groupMgr.updateGroupsAfterRoadRemove()
            roadMgr.updateMultiAfterRemove()
            return
          end
          im.tooltip('Remove this junction from the session (removes all roads).')
          im.SameLine()
          im.NextColumn()

          -- 'Finalise Junction' button.
          if editor.uiIconImageButton(editor.icons.lock, vec24, cols.darkLockCol, nil, nil, 'finaliseJunctionBtn') then
            jctMgr.finaliseJunction(i)
            mfe.selectedJctIdx = max(1, min(#junctions, i - 1))
            return
          end
          im.tooltip('Finalize this junction (will export the junction roads to the Roads List, and will become connectable. However, the junction will no longer be editable in the junction designer).')
          im.SameLine()
          im.NextColumn()

          -- 'Save Junction To Disk' button.
          if editor.uiIconImageButton(editor.icons.floppyDisk, vec24, nil, nil, nil, 'saveJunctionBtn') then
            jctMgr.saveJunction(i)
          end
          im.tooltip('Save this junction to disk.')
          im.SameLine()
          im.NextColumn()

          -- 'Go To Selected Junction' button.
          if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, vec24, cols.unlinkCol, nil, nil, 'goToSelectedJctBtn') then
            jctMgr.goToJunction(i)
          end
          im.tooltip('Go to this junction.')
          im.NextColumn()

          im.Separator()
        end
        im.EndListBox()
        im.PopItemWidth()
      end

      -- URBAN JUNCTIONS:
      if not isJctPlaceMode then
        im.Separator()
        im.TextColored(cols.greenB, 'Urban/Rural Junctions:')
        im.Columns(8, "urbanJctsRow1", false)

        -- 'Add Simple Crossing Junction' button.
        if editor.uiIconImageButton(editor.icons.roadPedestrianCrossing02, vec36, cols.blueB, nil, nil, 'addPedCrossingOnlyJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addPedXJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a 2-way simple crossing junction.')
        im.SameLine()
        im.NextColumn()

        -- 'Add 4-Way Crossroads Junction' button.
        if editor.uiIconImageButton(editor.icons.roadXJunction, vec36, cols.blueB, nil, nil, 'add4WCrossroadsJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addCrossroads(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a 4-way crossroads style junction.')
        im.SameLine()
        im.NextColumn()

        -- 'Add 3-Way T-Junction' button.
        if editor.uiIconImageButton(editor.icons.roadTJunction, vec36, cols.blueB, nil, nil, 'add3WTJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addTJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a 3-way T-style junction.')
        im.SameLine()
        im.NextColumn()

        -- 'Add 3-Way Angled Y-Junction' button.
        if editor.uiIconImageButton(editor.icons.roadYJunction, vec36, cols.blueB, nil, nil, 'add3WYJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addYJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a 3-way Y-style junction (with angular control).')
        im.SameLine()
        im.NextColumn()

        -- 'Add 4-Way Roundabout' button.
        if editor.uiIconImageButton(editor.icons.roadRoundaboutJunction, vec36, cols.blueB, nil, nil, 'addRoundaboutJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addRoundaboutJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a 4-way roundabout junction.')
        im.SameLine()
        im.NextColumn()

        -- 'Add Rural/Urban Transition' button.
        if editor.uiIconImageButton(editor.icons.roadSidewalkTransition, vec36, cols.blueB, nil, nil, 'addRuralUrbanTransitionJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addRuralUrbanTransJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a sidewalk transition junction (joins an urban road with sidewalk to an urban/rural road without sidewalk).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Urban Merge/Taper' button.
        if editor.uiIconImageButton(editor.icons.urbanRoad3To2Merge02, vec36, cols.blueB, nil, nil, 'addUrbanMergeJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addUrbanMergeJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add an urban merge junction (eg 2 lanes -> 1 lane, 3 lanes -> 2 lanes).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Urban Separator' button.
        if editor.uiIconImageButton(editor.icons.urbanRoadSeparate, vec36, cols.blueB, nil, nil, 'addUrbanSeparateJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addUrbanSeparatorJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a urban separator junction (splits a two-way road into two one-way sections, to make linking separable).')
        im.NextColumn()

        -- HIGHWAY JUNCTIONS:

        im.Columns(1)
        im.Separator()
        im.TextColored(cols.greenB, 'Highway Junctions:')
        im.Columns(8, "highwayJctsRow1", false)

        -- 'Add Highway Merge' button.
        if editor.uiIconImageButton(editor.icons.highwayMerge, vec36, cols.greenB, nil, nil, 'addHwyMergeJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addHighwayMergeJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a highway merge junction (eg 2 lanes -> 1 lane, 3 lanes -> 2 lanes).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Highway/Urban Transition' button.
        if editor.uiIconImageButton(editor.icons.highwayToUrbanRoadTransition01, vec36, cols.greenB, nil, nil, 'addHwyUrbanTransitionJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addHighwayUrbanTransJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a highway <-> urban transition junction (tapers the central reservation down to a centerline).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Highway Separator' button.
        if editor.uiIconImageButton(editor.icons.highwaySeparate, vec36, cols.greenB, nil, nil, 'addHwySeparateJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addHighwaySeparatorJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a highway separator junction (splits a two-way highway into two one-way sections, to make linking separable).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Shoulder Merge' button.
        if editor.uiIconImageButton(editor.icons.highwayShoulderMerge, vec36, cols.greenB, nil, nil, 'addShoulderMergeJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addShoulderFadeJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a shoulder fade junction (tapers out the hard shoulder lane, to connect Highway to Urban types).')
        im.SameLine()
        im.NextColumn()

        -- 'Add Highway Slip' button.
        if editor.uiIconImageButton(editor.icons.roadMarkingOutline, vec36, cols.greenB, nil, nil, 'addHwySlipJctBtn') then
          local roadPre = copyDataState()
          jctMgr.addHighwaySlipJunction(true)
          mfe.selectedJctIdx = #junctions
          addJctNodesToMulti(mfe.selectedJctIdx, roadMgr.multi)
          isJctPlaceMode = true
          local roadPost = copyDataState()
          editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
        end
        im.tooltip('Add a highway slip junction.')
        im.SameLine()
        im.NextColumn()

        im.Dummy(vec36)
        im.SameLine()
        im.NextColumn()

        -- 'Create Auto Junction' button.
        local btnIcon = editor.icons.jointLocked
        if isAutoJctMode then
          btnIcon = editor.icons.jointUnlocked
        end
        if editor.uiIconImageButton(btnIcon, vec36, nil, nil, nil, 'createAutoJunction') then
          isAutoJctMode = not isAutoJctMode
        end
        im.tooltip('Create an auto-junction by grouping road ends with a polygon. Toggles mode on/off.')
        im.SameLine()
        im.NextColumn()

        -- 'Load Junction' button.
        if editor.uiIconImageButton(editor.icons.roadFolderPlus, vec36, nil, nil, nil, 'loadJunction') then
          jctMgr.loadJunction()
        end
        im.tooltip('Load a previously-saved junction from disk.')
        im.NextColumn()
      end

      im.Columns(1)
      im.Separator()

      -- Junction Edit Options.
      local wCtr = 2034
      local selJctIdx = mfe.selectedJctIdx
      local selJct = junctions[selJctIdx]
      if #junctions > 0 and selJct and selJctIdx then

        -- Junction properties (type-specific).
        local type = selJct.type
        if type == 'crossing' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputFloat("L Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Len###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          im.Columns(2, 'jctSidewalkCols_row2', false)
          if im.Checkbox("Sidewalk###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')
          im.SameLine()
          im.NextColumn()
          if selJct.isSidewalk[0] then
            if im.Checkbox("Low Corners###" .. tostring(wCtr), selJct.isLowerSWAtPedX) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Lower the sidewalks at pedestrian crossings.')
          else
            im.Text('')
          end
          im.NextColumn()

          im.Columns(1)
          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')
          end

          if im.InputFloat("Crossing Length###" .. tostring(wCtr), selJct.pedXDist, 0.1, 0.0) then
            selJct.pedXDist = im.FloatPtr(max(2.0, min(30.0, selJct.pedXDist[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the crossing road.')

          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Light Parameters:')
          if im.Checkbox("Include Traffic Lights###" .. tostring(wCtr), selJct.isTLights) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic lights.')

          if selJct.isTLights[0] then
            if im.InputFloat("Pole Lateral Offset###" .. tostring(wCtr), selJct.trafficLatOff, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.trafficLatOff = im.FloatPtr(max(-20.0, min(20.0, selJct.trafficLatOff[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the lateral offset of the traffic lights.')
          end

          -- Pedestrian Crossing Parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Pedestrian Crossing Parameters:')
          if im.Checkbox("Ped X###" .. tostring(wCtr), selJct.isPedX1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 1.')

          if selJct.isPedX1[0] then
            if im.InputFloat("Crossing Width###" .. tostring(wCtr), selJct.pedXWidth, 0.1, 0.0) then
              selJct.pedXWidth = im.FloatPtr(max(0.5, min(5.0, selJct.pedXWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the pedestrian crossings.')
          end

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Seed###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

          im.Columns(1)

          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

        elseif type == 'crossroads' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Num Lanes - X Dir###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputInt("Num Lanes - Y Dir###" .. tostring(wCtr), selJct.numLanesY, 1) then
            selJct.numLanesY = im.IntPtr(max(1, min(6, selJct.numLanesY[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the Y direction.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          if im.Checkbox("Is Y One-Way###" .. tostring(wCtr), selJct.isYOneWay) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Use a one-way road for the Y direction roads.')

          if selJct.isYOneWay[0] then
            im.Columns(2, 'crossroadsYDirCols1', false)
            if im.Checkbox("Y1 Out/In###" .. tostring(wCtr), selJct.isY1Outwards) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('The Y1-Exit will be outwards-pointing (checked), or inwards-pointing (unchecked).')
            im.SameLine()
            im.NextColumn()
            if im.Checkbox("Y2 Out/In###" .. tostring(wCtr), selJct.isY2Outwards) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('The Y2-Exit will be outwards-pointing (checked), or inwards-pointing (unchecked).')
            im.NextColumn()
          end

          im.Columns(1)

          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          im.Columns(2, 'jctSidewalkCols_row2', false)
          if im.Checkbox("Sidewalk###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')
          im.SameLine()
          im.NextColumn()
          if selJct.isSidewalk[0] then
            if im.Checkbox("Low Corners###" .. tostring(wCtr), selJct.isLowerSWAtPedX) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Lower the sidewalks at pedestrian crossings.')
          else
            im.Text('')
          end
          im.NextColumn()

          im.Columns(1)
          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')

            if im.InputFloat("Sidewalk Corner Radius###" .. tostring(wCtr), selJct.bevel, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.bevel = im.FloatPtr(max(2.5, min(20.0, selJct.bevel[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the corner radius of the sidewalks.')
          end

          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Light Parameters:')
          if im.Checkbox("Traffic Lights###" .. tostring(wCtr), selJct.isTLights) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic lights.')

          if selJct.isTLights[0] then
            if im.InputFloat("Pole Lateral Offset###" .. tostring(wCtr), selJct.trafficLatOff, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.trafficLatOff = im.FloatPtr(max(-20.0, min(20.0, selJct.trafficLatOff[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the lateral offset of the traffic lights.')
          end

          -- Pedestrian Crossing Parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Pedestrian Crossing Parameters:')
          im.Columns(4, 'jctPedXCheckboxesCols1', false)
          if im.Checkbox("PX 1###" .. tostring(wCtr), selJct.isPedX1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 1.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 2###" .. tostring(wCtr), selJct.isPedX2) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 2.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 3###" .. tostring(wCtr), selJct.isPedX3) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 3.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 4###" .. tostring(wCtr), selJct.isPedX4) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 4.')
          im.NextColumn()

          im.Columns(1)

          local isPedXBeingUsed = selJct.isPedX1[0] or selJct.isPedX2[0] or selJct.isPedX3[0] or selJct.isPedX4[0]
          if isPedXBeingUsed then
            if im.InputFloat("Crossing Width###" .. tostring(wCtr), selJct.pedXWidth, 0.1, 0.0) then
              selJct.pedXWidth = im.FloatPtr(max(0.5, min(5.0, selJct.pedXWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the pedestrian crossings.')
          end

          -- Lane Arrow Decal parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Lane Arrow Parameters:')
          im.Columns(2, 'arrowCols1', false)
          if im.Checkbox("Front Arrows###" .. tostring(wCtr), selJct.isArrow) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include arrow decals on all roads approaching the junction.')
          im.SameLine()
          im.NextColumn()
          if selJct.isArrow[0] then
            if im.Checkbox("Rear Arrows###" .. tostring(wCtr), selJct.isDoubleArrows) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Include a second row of arrows some distance behind the first row (if space permits).')
          else
            im.Text('')
          end
          im.NextColumn()
          im.Columns(1)
          if im.InputFloat("Arrow Size###" .. tostring(wCtr), selJct.arrowSize, 0.01, 0.0) then
            selJct.arrowSize = im.FloatPtr(max(0.5, min(4.5, selJct.arrowSize[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The size of the arrow decals.')
          if im.InputFloat("Front Arrow Distance###" .. tostring(wCtr), selJct.arrowFrontDistFromEnd, 0.01, 0.0) then
            selJct.arrowFrontDistFromEnd = im.FloatPtr(max(0.0, min(selJct.arrowBackDistFromEnd[0] - selJct.arrowSize[0], selJct.arrowFrontDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the front arrows shall appear.')
          if im.InputFloat("Rear Arrow Distance###" .. tostring(wCtr), selJct.arrowBackDistFromEnd, 0.01, 0.0) then
            selJct.arrowBackDistFromEnd = im.FloatPtr(max(selJct.arrowFrontDistFromEnd[0] + selJct.arrowSize[0], min(50.0, selJct.arrowBackDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the back arrows shall appear.')
          im.Columns(3, "jctArrowsMatSelA", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text('Material')
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMat4JctArrowsBtn') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = false
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = true
            mfe.isMaterialForOverlay = false
          end
          im.tooltip('Select a new material for the arrows.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.arrowMat)
          im.tooltip('The currently-selected material for the arrows.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.Separator()
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

        elseif type == 't-junction' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Num Lanes - X Dir###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputInt("Num Lanes - Y Dir###" .. tostring(wCtr), selJct.numLanesY, 1) then
            selJct.numLanesY = im.IntPtr(max(1, min(6, selJct.numLanesY[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the Y direction.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          if im.Checkbox("Is Y One-Way###" .. tostring(wCtr), selJct.isYOneWay) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Use a one-way road for the Y direction roads.')

          if selJct.isYOneWay[0] then
            if im.Checkbox("Outwards/Inwards###" .. tostring(wCtr), selJct.isY2Outwards) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('The Y-Exit will be outwards-pointing (checked), or inwards-pointing (unchecked).')
          end

          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          im.Columns(2, 'jctSidewalkCols_row2', false)
          if im.Checkbox("Sidewalk###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')
          im.SameLine()
          im.NextColumn()
          if selJct.isSidewalk[0] then
            if im.Checkbox("Low Corners###" .. tostring(wCtr), selJct.isLowerSWAtPedX) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Lower the sidewalks at pedestrian crossings.')
          else
            im.Text('')
          end
          im.NextColumn()

          im.Columns(1)
          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')

            if im.InputFloat("Curb Corner Radius###" .. tostring(wCtr), selJct.bevel, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.bevel = im.FloatPtr(max(2.5, min(20.0, selJct.bevel[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the corner radius of the sidewalks.')
          end

          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Light Parameters:')
          if im.Checkbox("Include Traffic Lights###" .. tostring(wCtr), selJct.isTLights) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic lights.')

          if selJct.isTLights[0] then
            if im.InputFloat("Pole Lateral Offset###" .. tostring(wCtr), selJct.trafficLatOff, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.trafficLatOff = im.FloatPtr(max(-20.0, min(20.0, selJct.trafficLatOff[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the lateral offset of the traffic lights.')
          end

          -- Pedestrian Crossing Parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Pedestrian Crossing Parameters:')
          im.Columns(3, 'jctPedXCheckboxesCols1', false)
          if im.Checkbox("PX 1###" .. tostring(wCtr), selJct.isPedX1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 1.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 2###" .. tostring(wCtr), selJct.isPedX2) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 2.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 3###" .. tostring(wCtr), selJct.isPedX3) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 3.')
          im.NextColumn()

          im.Columns(1)

          local isPedXBeingUsed = selJct.isPedX1[0] or selJct.isPedX2[0] or selJct.isPedX3[0]
          if isPedXBeingUsed then
            if im.InputFloat("Crossing Width###" .. tostring(wCtr), selJct.pedXWidth, 0.1, 0.0) then
              selJct.pedXWidth = im.FloatPtr(max(0.5, min(5.0, selJct.pedXWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the pedestrian crossings.')
          end

          -- Lane Arrow Decal parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Lane Arrow Parameters:')
          im.Columns(2, 'arrowCols1', false)
          if im.Checkbox("Front Arrows###" .. tostring(wCtr), selJct.isArrow) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include arrow decals on all roads approaching the junction.')
          im.SameLine()
          im.NextColumn()
          if selJct.isArrow[0] then
            if im.Checkbox("Back Arrows###" .. tostring(wCtr), selJct.isDoubleArrows) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Include a second row of arrows some distance behind the first row (if space permits).')
          else
            im.Text('')
          end
          im.NextColumn()
          im.Columns(1)
          if im.InputFloat("Arrow Size###" .. tostring(wCtr), selJct.arrowSize, 0.01, 0.0) then
            selJct.arrowSize = im.FloatPtr(max(0.5, min(4.5, selJct.arrowSize[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The size of the arrow decals.')
          if im.InputFloat("Front Arrow Distance###" .. tostring(wCtr), selJct.arrowFrontDistFromEnd, 0.01, 0.0) then
            selJct.arrowFrontDistFromEnd = im.FloatPtr(max(0.0, min(selJct.arrowBackDistFromEnd[0] - selJct.arrowSize[0], selJct.arrowFrontDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the front arrows shall appear.')
          if im.InputFloat("Rear Arrow Distance###" .. tostring(wCtr), selJct.arrowBackDistFromEnd, 0.01, 0.0) then
            selJct.arrowBackDistFromEnd = im.FloatPtr(max(selJct.arrowFrontDistFromEnd[0] + selJct.arrowSize[0], min(50.0, selJct.arrowBackDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the back arrows shall appear.')
          im.Columns(3, "jctArrowsMatSelA", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text('Material')
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMat4JctArrowsBtn') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = false
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = true
            mfe.isMaterialForOverlay = false
          end
          im.tooltip('Select a new material for the arrows.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.arrowMat)
          im.tooltip('The currently-selected material for the arrows.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          im.PushItemWidth(-1)
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()
          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.Separator()
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

        elseif type == 'y-junction' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Exit Road Parameters:')
          if im.InputFloat("Y-Exit Road Angle (deg)###" .. tostring(wCtr), selJct.theta, 1, 0.0) then
            selJct.theta = im.FloatPtr(max(-30.0, min(30.0, selJct.theta[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Road Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          if im.Checkbox("Include Sidewalks###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')

          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')
          end

          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Light Parameters:')
          if im.Checkbox("Include Traffic Lights###" .. tostring(wCtr), selJct.isTLights) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic lights.')

          -- Pedestrian Crossing Parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Pedestrian Crossing Parameters:')
          im.Columns(3, 'jctPedXCheckboxesCols1', false)
          if im.Checkbox("PX 1###" .. tostring(wCtr), selJct.isPedX1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 1.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 2###" .. tostring(wCtr), selJct.isPedX2) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 2.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 3###" .. tostring(wCtr), selJct.isPedX3) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 3.')
          im.NextColumn()

          im.Columns(1)

          local isPedXBeingUsed = selJct.isPedX1[0] or selJct.isPedX2[0] or selJct.isPedX3[0]
          if isPedXBeingUsed then
            if im.InputFloat("Crossing Width###" .. tostring(wCtr), selJct.pedXWidth, 0.1, 0.0) then
              selJct.pedXWidth = im.FloatPtr(max(0.5, min(5.0, selJct.pedXWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the pedestrian crossings.')
          end

          -- Lane Arrow Decal parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Lane Arrow Parameters:')
          im.Columns(2, 'arrowCols1', false)
          if im.Checkbox("Front Arrows###" .. tostring(wCtr), selJct.isArrow) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include arrow decals on all roads approaching the junction.')
          im.SameLine()
          im.NextColumn()
          if selJct.isArrow[0] then
            if im.Checkbox("Rear Arrows###" .. tostring(wCtr), selJct.isDoubleArrows) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Include a second row of arrows some distance behind the first row (if space permits).')
          else
            im.Text('')
          end
          im.NextColumn()
          im.Columns(1)
          if im.InputFloat("Arrow Size###" .. tostring(wCtr), selJct.arrowSize, 0.01, 0.0) then
            selJct.arrowSize = im.FloatPtr(max(0.5, min(4.5, selJct.arrowSize[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The size of the arrow decals.')
          if im.InputFloat("Front Arrow Distance###" .. tostring(wCtr), selJct.arrowFrontDistFromEnd, 0.01, 0.0) then
            selJct.arrowFrontDistFromEnd = im.FloatPtr(max(0.0, min(selJct.arrowBackDistFromEnd[0] - selJct.arrowSize[0], selJct.arrowFrontDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the front arrows shall appear.')
          if im.InputFloat("Rear Arrow Distance###" .. tostring(wCtr), selJct.arrowBackDistFromEnd, 0.01, 0.0) then
            selJct.arrowBackDistFromEnd = im.FloatPtr(max(selJct.arrowFrontDistFromEnd[0] + selJct.arrowSize[0], min(50.0, selJct.arrowBackDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the back arrows shall appear.')
          im.Columns(3, "jctArrowsMatSelA", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text('Material')
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMat4JctArrowsBtn') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = false
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = true
            mfe.isMaterialForOverlay = false
          end
          im.tooltip('Select a new material for the arrows.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.arrowMat)
          im.tooltip('The currently-selected material for the arrows.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          im.PushItemWidth(-1)
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()
          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.Separator()
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

        elseif type == 'roundabout' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Center Circle Parameters:')
          if im.InputFloat("Center Circle Radius (deg)###" .. tostring(wCtr), selJct.extraRadRB, 0.1, 0.0) then
            selJct.extraRadRB = im.FloatPtr(max(-5.0, min(20.0, selJct.extraRadRB[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the extra radius amount (base radius determined by exit road size).')

          im.Separator()
          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            selJct.numRBLanes = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          im.Columns(2, 'jctSidewalkCols_row2', false)
          if im.Checkbox("Sidewalk###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')
          im.SameLine()
          im.NextColumn()
          if selJct.isSidewalk[0] then
            if im.Checkbox("Low Corners###" .. tostring(wCtr), selJct.isLowerSWAtPedX) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Lower the sidewalks at pedestrian crossings.')
          else
            im.Text('')
          end
          im.NextColumn()

          im.Columns(1)
          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')

            if im.InputFloat("Sidewalk Corner Radius###" .. tostring(wCtr), selJct.bevel, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.bevel = im.FloatPtr(max(2.5, min(20.0, selJct.bevel[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the corner radius of the sidewalks.')
          end

          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Light Parameters:')
          if im.Checkbox("Include Traffic Lights###" .. tostring(wCtr), selJct.isTLights) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic lights.')

          if selJct.isTLights[0] then
            if im.InputFloat("Poles Lateral Offset###" .. tostring(wCtr), selJct.trafficLatOff, 0.1, 2.0) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.trafficLatOff = im.FloatPtr(max(-20.0, min(20.0, selJct.trafficLatOff[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the lateral offset of the traffic lights.')
          end

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Pedestrian Crossing Parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Pedestrian Crossing Parameters:')
          im.Columns(4, 'jctPedXCheckboxesCols1', false)
          if im.Checkbox("PX 1###" .. tostring(wCtr), selJct.isPedX1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 1.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 2###" .. tostring(wCtr), selJct.isPedX2) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 2.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 3###" .. tostring(wCtr), selJct.isPedX3) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 3.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("PX 4###" .. tostring(wCtr), selJct.isPedX4) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include a pedestrian cross on road 4.')
          im.NextColumn()

          im.Columns(1)

          local isPedXBeingUsed = selJct.isPedX1[0] or selJct.isPedX2[0] or selJct.isPedX3[0] or selJct.isPedX4[0]
          if isPedXBeingUsed then
            if im.InputFloat("Crossing Width###" .. tostring(wCtr), selJct.pedXWidth, 0.1, 0.0) then
              selJct.pedXWidth = im.FloatPtr(max(0.5, min(5.0, selJct.pedXWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the pedestrian crossings.')
          end

          -- Lane Arrow Decal parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Lane Arrow Parameters:')
          im.Columns(2, 'arrowCols1', false)
          if im.Checkbox("Front Arrows###" .. tostring(wCtr), selJct.isArrow) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include arrow decals on all roads approaching the junction.')
          im.SameLine()
          im.NextColumn()
          if selJct.isArrow[0] then
            if im.Checkbox("Rear Arrows###" .. tostring(wCtr), selJct.isDoubleArrows) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Include a second row of arrows some distance behind the first row (if space permits).')
          else
            im.Text('')
          end
          im.NextColumn()
          im.Columns(1)
          if im.InputFloat("Arrow Size###" .. tostring(wCtr), selJct.arrowSize, 0.01, 0.0) then
            selJct.arrowSize = im.FloatPtr(max(0.5, min(4.5, selJct.arrowSize[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The size of the arrow decals.')
          if im.InputFloat("Front Arrow Distance###" .. tostring(wCtr), selJct.arrowFrontDistFromEnd, 0.01, 0.0) then
            selJct.arrowFrontDistFromEnd = im.FloatPtr(max(0.0, min(selJct.arrowBackDistFromEnd[0] - selJct.arrowSize[0], selJct.arrowFrontDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the front arrows shall appear.')
          if im.InputFloat("Rear Arrow Distance###" .. tostring(wCtr), selJct.arrowBackDistFromEnd, 0.01, 0.0) then
            selJct.arrowBackDistFromEnd = im.FloatPtr(max(selJct.arrowFrontDistFromEnd[0] + selJct.arrowSize[0], min(50.0, selJct.arrowBackDistFromEnd[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('The distance from the junction, at which the back arrows shall appear.')
          im.Columns(3, "jctArrowsMatSelA", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text('Material')
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMat4JctArrowsBtn') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = false
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = true
            mfe.isMaterialForOverlay = false
          end
          im.tooltip('Select a new material for the arrows.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.arrowMat)
          im.tooltip('The currently-selected material for the arrows.')
          im.NextColumn()
          im.Columns(1)

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.Separator()
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

        elseif type == 'rural_urban_transition' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          im.Columns(2, 'setYDirCols1', false)
          if im.Checkbox("Is Y One-Way###" .. tostring(wCtr), selJct.isYOneWay) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Use a one-way road for the Y direction roads.')
          im.SameLine()
          im.NextColumn()

          if selJct.isYOneWay[0] then
            if im.Checkbox("Direction###" .. tostring(wCtr), selJct.isY1Outwards) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip("Set the direction of this one-way junction. 'From' sidewalk (checked) or 'To' sidewalk (unchecked).")
          else
            im.Text('')
          end
          im.NextColumn()

          im.Columns(1)
          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
            selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the sidewalks.')

          if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
          wCtr = wCtr + 1
          im.tooltip('Set the height of the sidewalks.')

          -- Junction condition parameters.
          im.PushItemWidth(-1)
          im.Separator()
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        elseif type == 'urban_merge' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of trunk lanes (on the thin end).')

          if im.InputFloat("Taper Section Length###" .. tostring(wCtr), selJct.s2Length, 0.1, 0.0) then
            selJct.s2Length = im.FloatPtr(max(1.0, min(80.0, selJct.s2Length[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the taper section (middle).')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(1.0, min(50.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the end roads.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the trunk and exit roads.')

          if im.InputFloat("Inner Taper Width###" .. tostring(wCtr), selJct.sepWidthI, 0.1, 0.0) then
            selJct.sepWidthI = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthI[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the inner width of the taper (one third along).')

          if im.InputFloat("Outer Taper Width###" .. tostring(wCtr), selJct.sepWidthO, 0.1, 0.0) then
            selJct.sepWidthO = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthO[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the outer width of the taper (two thirds along).')

          -- Sidewalk parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          if im.Checkbox("Include Sidewalks###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')

          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          if not selJct.isSidewalk[0]then
            im.Separator()
            im.TextColored(cols.greenB, 'Edge Blending Material:')
            im.Columns(3, "edgeBlendingJctCols1", false)
            im.SetColumnWidth(0, 150)
            im.SetColumnWidth(1, 30)
            im.Text("Edge Blending")
            im.SameLine()
            im.NextColumn()
            if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
              setMaterialList()
              editor.showWindow(win.materialSelectWinName)
              mfe.isMaterialSelectWinOpen = true
              mfe.isMaterialForRoad = true
              mfe.isMaterialForEdgeBlendLeft = false
              mfe.isMaterialForEdgeBlendRight = false
              mfe.isMaterialForJctArrows = false
              mfe.isMaterialForOverlay = false
              mfe.materialForRoadTarget = 'jctEdgeBlend'
            end
            im.tooltip('Select a new material for the edge blending.')
            im.SameLine()
            im.NextColumn()
            im.Text(selJct.edgeBlendMat)
            im.tooltip('The currently-selected material for the junction edge blending')
            im.NextColumn()
          end

        elseif type == 'urban_separator' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

        elseif type == 'highway_merge' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of trunk lanes (on the thin end).')

          if im.InputFloat("Taper Section Length###" .. tostring(wCtr), selJct.s2Length, 0.1, 0.0) then
            selJct.s2Length = im.FloatPtr(max(1.0, min(80.0, selJct.s2Length[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the taper section (middle).')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(1.0, min(50.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the end roads.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the trunk and exit roads.')

          if im.InputFloat("Hard Shoulder Width###" .. tostring(wCtr), selJct.hardWidth, 0.1, 0.0) then
            selJct.hardWidth = im.FloatPtr(max(0.0, min(10.0, selJct.hardWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the hard shoulder lanes.')

          if im.InputFloat("Central Res Width###" .. tostring(wCtr), selJct.cResWidth, 0.1, 0.0) then
            selJct.cResWidth = im.FloatPtr(max(0.0, min(20.0, selJct.cResWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the central reservation.')

          if im.InputFloat("Inner Taper Width###" .. tostring(wCtr), selJct.sepWidthI, 0.1, 0.0) then
            selJct.sepWidthI = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthI[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the inner width of the taper (one third along).')

          if im.InputFloat("Outer Taper Width###" .. tostring(wCtr), selJct.sepWidthO, 0.1, 0.0) then
            selJct.sepWidthO = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthO[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the outer width of the taper (two thirds along).')

          -- Crash barrier parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Crash Barrier Parameters:')
          im.Columns(2, 'crashBarrierCols1', false)
          if im.Checkbox("Inner Barriers###" .. tostring(wCtr), selJct.isBarriersI) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include inner crash barriers.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("Outer Barriers###" .. tostring(wCtr), selJct.isBarriersO) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include outer crash barriers.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        elseif type == 'highway_urban_transition' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of trunk lanes (on the thin end).')

          if im.InputFloat("Taper Section Length###" .. tostring(wCtr), selJct.s2Length, 0.1, 0.0) then
            selJct.s2Length = im.FloatPtr(max(1.0, min(80.0, selJct.s2Length[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the taper section (middle).')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(1.0, min(50.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the end roads.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the trunk and exit roads.')

          if im.InputFloat("Hard Shoulder Width###" .. tostring(wCtr), selJct.hardWidth, 0.1, 0.0) then
            selJct.hardWidth = im.FloatPtr(max(0.0, min(10.0, selJct.hardWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the hard shoulder lanes.')

          if im.InputFloat("Central Res Width###" .. tostring(wCtr), selJct.cResWidth, 0.1, 0.0) then
            selJct.cResWidth = im.FloatPtr(max(0.0, min(20.0, selJct.cResWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the central reservation.')

          if im.InputFloat("Inner Taper Width###" .. tostring(wCtr), selJct.sepWidthI, 0.1, 0.0) then
            selJct.sepWidthI = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthI[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the inner width of the taper (one third along).')

          if im.InputFloat("Outer Taper Width###" .. tostring(wCtr), selJct.sepWidthO, 0.1, 0.0) then
            selJct.sepWidthO = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthO[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the outer width of the taper (two thirds along).')

          im.Separator()
          im.TextColored(cols.greenB, 'Sidewalk Parameters:')
          if im.Checkbox("Include Sidewalks###" .. tostring(wCtr), selJct.isSidewalk) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include sidewalks.')
          if selJct.isSidewalk[0] then
            if im.InputFloat("Sidewalk Width###" .. tostring(wCtr), selJct.sidewalkWidth, 0.1, 2.0) then
              selJct.sidewalkWidth = im.FloatPtr(max(0.5, min(10.0, selJct.sidewalkWidth[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            wCtr = wCtr + 1
            im.tooltip('Set the width of the sidewalks.')

            if im.InputFloat("Sidewalk Height###" .. tostring(wCtr), selJct.sidewalkHeight, 0.01, 0.1) then
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            selJct.sidewalkHeight = im.FloatPtr(max(0.0, min(0.5, selJct.sidewalkHeight[0])))
            wCtr = wCtr + 1
            im.tooltip('Set the height of the sidewalks.')
          end

          -- Crash barrier parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Crash Barrier Parameters:')
          im.Columns(2, 'crashBarrierCols1', false)
          if im.Checkbox("Inner Barriers###" .. tostring(wCtr), selJct.isBarriersI) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include inner crash barriers.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("Outer Barriers###" .. tostring(wCtr), selJct.isBarriersO) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include outer crash barriers.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        elseif type == 'highway_separator' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of lanes for roads heading in the X direction.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the exit roads.')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(0.5, min(20.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the exit roads.')

          -- Crash barrier parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Crash Barrier Parameters:')
          im.Columns(2, 'crashBarrierCols1', false)
          if im.Checkbox("Inner Barriers###" .. tostring(wCtr), selJct.isBarriersI) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include inner crash barriers.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("Outer Barriers###" .. tostring(wCtr), selJct.isBarriersO) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include outer crash barriers.')
          im.NextColumn()
          im.Columns(1)

          -- Signs parameters.
          im.Separator()
          im.TextColored(cols.greenB, 'Traffic Sign Parameters:')
          if im.Checkbox("Include Traffic Signs###" .. tostring(wCtr), selJct.isSigns) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include traffic signs (poles).')

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        elseif type == 'shoulder_fade' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of trunk lanes (on the thin end).')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(1.0, min(50.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the end roads.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the trunk and exit roads.')

          if im.InputFloat("Hard Shoulder Width###" .. tostring(wCtr), selJct.hardWidth, 0.1, 0.0) then
            selJct.hardWidth = im.FloatPtr(max(0.0, min(10.0, selJct.hardWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the hard shoulder lanes.')

          if im.Checkbox("Direction###" .. tostring(wCtr), selJct.isY1Outwards) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip("Set the direction of this one-way junction. 'To' highway (checked) or 'From' highway (unchecked).")

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        elseif type == 'highway_slip' then

          im.PushItemWidth(-150)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)

          im.Columns(1)

          im.TextColored(cols.greenB, 'Road Sizing Parameters:')
          if im.InputInt("Number Of Lanes###" .. tostring(wCtr), selJct.numLanesX, 1) then
            selJct.numLanesX = im.IntPtr(max(1, min(6, selJct.numLanesX[0])))
            selJct.numLanesY = im.IntPtr(selJct.numLanesX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the number of trunk lanes.')

          if im.InputFloat("Taper Section Length###" .. tostring(wCtr), selJct.s2Length, 0.1, 0.0) then
            selJct.s2Length = im.FloatPtr(max(1.0, min(80.0, selJct.s2Length[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the taper section (section 2).')

          if im.InputFloat("Split Section Length###" .. tostring(wCtr), selJct.s3Length, 0.1, 0.0) then
            selJct.s3Length = im.FloatPtr(max(1.0, min(80.0, selJct.s3Length[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the split section (section 3).')

          if im.InputFloat("Exit Roads Length###" .. tostring(wCtr), selJct.capLength, 0.1, 0.0) then
            selJct.capLength = im.FloatPtr(max(1.0, min(50.0, selJct.capLength[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the length of the end roads.')

          if im.InputFloat("Lane Width###" .. tostring(wCtr), selJct.laneWidthX, 0.1, 0.0) then
            selJct.laneWidthX = im.FloatPtr(max(1.0, min(20.0, selJct.laneWidthX[0])))
            selJct.laneWidthY = im.FloatPtr(selJct.laneWidthX[0])
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the lane width of the trunk and exit roads.')

          if im.InputFloat("Hard Shoulder Width###" .. tostring(wCtr), selJct.hardWidth, 0.1, 0.0) then
            selJct.hardWidth = im.FloatPtr(max(0.0, min(10.0, selJct.hardWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the hard shoulder lanes.')

          if im.InputFloat("Central Res Width###" .. tostring(wCtr), selJct.cResWidth, 0.1, 0.0) then
            selJct.cResWidth = im.FloatPtr(max(0.0, min(20.0, selJct.cResWidth[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the width of the central reservation.')

          if im.InputFloat("Inner Taper Width###" .. tostring(wCtr), selJct.sepWidthI, 0.1, 0.0) then
            selJct.sepWidthI = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthI[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the inner width of the split section.')

          if im.InputFloat("Outer Taper Width###" .. tostring(wCtr), selJct.sepWidthO, 0.1, 0.0) then
            selJct.sepWidthO = im.FloatPtr(max(0.0, min(20.0, selJct.sepWidthO[0])))
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Set the outer width of the split section.')

          -- Overlay parameters:
          im.Separator()
          im.TextColored(cols.greenB, 'Tread Overlay Parameters:')
          if im.Checkbox("Include Tread Overlays###" .. tostring(wCtr), selJct.isCrossings) then
            jctMgr.updateJunctionAfterChange(selJctIdx)
            mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          wCtr = wCtr + 1
          im.tooltip('Include tread overlays (checked), or not (unchecked).')
          if selJct.isCrossings[0] then
            if im.InputInt("Number Of Overlays###4432", selJct.numCrossings, 1) then
              selJct.numCrossings = im.IntPtr(max(0, min(50, selJct.numCrossings[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the number of overlays to be included.')
            if im.InputInt("Random Seed Value###4433", selJct.seed, 1) then
              selJct.seed = im.IntPtr(max(0, min(4294967295, selJct.seed[0])))
              jctMgr.updateJunctionAfterChange(selJctIdx)
              mfe.selectedRoadIdx = roadMgr.map[jctMgr.junctions[selJctIdx].roads[1]]
              mfe.selectedNodeIdx = 1
              mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
            end
            im.tooltip('Set the random seed for the overlays.')
          end

          -- Junction condition parameters.
          im.Separator()
          im.PushItemWidth(-1)
          im.TextColored(cols.greenB, 'Junction Condition Parameters:')
          if im.SliderFloat("###7160", selJct.condition, 0.0, 1.0, "Road Condition = %.3f") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The road condition [0 = clean, 1 = damaged/worn].')

          if im.SliderInt("###7161", selJct.numPatches, 0, 50, "Repair Patches = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of damage patches to include on the road.')

          if im.SliderInt("###7438", selJct.numPotholes, 0, 50, "Pot Holes = %d") then
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('The number of pothole patches to include on the road.')
          im.PopItemWidth()

          if im.InputInt("Random Seed Value###7232", selJct.conditionSeed, 1) then
            selJct.conditionSeed = im.IntPtr(max(0, min(4294967295, selJct.conditionSeed[0])))
            jctMgr.updateJunctionCondition(selJct)
            mfe.selectedLayerIdx = 1
          end
          im.tooltip('Set the random seed for the junction condition (change to get different wear patterns).')
          im.PopStyleVar()
          im.PopItemWidth()
          im.Columns(1)

          -- Edge blending material.
          im.Separator()
          im.TextColored(cols.greenB, 'Edge Blending Material:')
          im.Columns(3, "edgeBlendingJctCols1", false)
          im.SetColumnWidth(0, 150)
          im.SetColumnWidth(1, 30)
          im.Text("Edge Blending")
          im.SameLine()
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.youtube_searched_for, vec24, cols.fullWhite, nil, nil, 'selectMatBtnEdgesJctLeft') then
            setMaterialList()
            editor.showWindow(win.materialSelectWinName)
            mfe.isMaterialSelectWinOpen = true
            mfe.isMaterialForRoad = true
            mfe.isMaterialForEdgeBlendLeft = false
            mfe.isMaterialForEdgeBlendRight = false
            mfe.isMaterialForJctArrows = false
            mfe.isMaterialForOverlay = false
            mfe.materialForRoadTarget = 'jctEdgeBlend'
          end
          im.tooltip('Select a new material for the edge blending.')
          im.SameLine()
          im.NextColumn()
          im.Text(selJct.edgeBlendMat)
          im.tooltip('The currently-selected material for the junction edge blending')
          im.NextColumn()

        end

        im.Columns(1)
      end
      im.EndTabItem()
    end

    -- 'Groups' sub-menu.
    if selectedTab == 3 then

      if not hasGroupsListBeenComputed then
        groupMgr.getDefaultGroups()                                                         -- Populate the default groups, if it has not already been computed.
        hasGroupsListBeenComputed = true
      end

      im.PushItemWidth(-1)
      if im.BeginListBox('') then

          -- 'Placed Groups' list.
        im.Columns(6, "groupsListBoxColumns", true)
        im.SetColumnWidth(0, 30)
        im.SetColumnWidth(1, 150)
        im.SetColumnWidth(2, 35)
        im.SetColumnWidth(3, 35)
        im.SetColumnWidth(4, 35)
        im.SetColumnWidth(5, 35)

        local placedGroups = groupMgr.getPlacedGroups() or {}
        local wCtr = 8100
        for i = 1, #placedGroups do
          local placedGroup = placedGroups[i]
          local flag = i == mfe.selectedPlacedGroupIdx
          if im.Selectable1(tostring(i) .. "###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            mfe.selectedPlacedGroupIdx = max(1, min(#placedGroups, i))
            addGroupNodesToMulti(i, roadMgr.multi)
            mfe.selectedRoadIdx = roadMgr.map[placedGroups[i].list[1].r]
            mfe.selectedNodeIdx = 1
            mfe.isNodeIdxChanged, mfe.isRoadIdxChanged = true, true
          end
          im.tooltip('The Id of this group.')
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          im.PushItemWidth(150)
          im.InputText("###" .. tostring(wCtr), placedGroup.name, 32)
          im.PopItemWidth()
          im.tooltip('Edit the name of this group.')
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          -- 'Remove Selected Placed Group - Soft' button.
          if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removePlacedGroupSoft') then
            groupMgr.removePlacedGroupSoft(i)
            roadMgr.updateRoadsAfterRemovingGroup(placedGroups)
            mfe.selectedPlacedGroupIdx = max(1, min(#placedGroups, i))
            return
          end
          im.tooltip('Soft delete - removes this group from the session, but NOT the roads inside the group.')
          im.SameLine()
          im.NextColumn()

          -- 'Remove Selected Placed Group - Hard' button.
          if editor.uiIconImageButton(editor.icons.delete_forever, vec24, cols.unlinkCol, nil, nil, 'removePlacedGroupHard') then
            groupMgr.removePlacedGroupHard(i)
            mfe.selectedPlacedGroupIdx = max(1, min(#placedGroups, i))
            roadMgr.updateRoadsAfterRemovingGroup(placedGroups)
            jctMgr.updateJunctionsAfterRoadRemove()
            roadMgr.updateMultiAfterRemove()
            return
          end
          im.tooltip('Hard delete - removes this group from the session, AND the roads inside it.')
          im.SameLine()
          im.NextColumn()

          -- 'Create New Prefab Group From Profile' button.
          if editor.uiIconImageButton(editor.icons.fg_type_square_2, vec24, cols.dullWhite, nil, nil, 'CreateNewPrefabGroupFromProfileBtn') then
            groupMgr.createPrefabGroup(placedGroup)
          end
          im.tooltip('Create a new template profile from the current profile of this road (will appear in templates list, and can be saved from there).')
          im.SameLine()
          im.NextColumn()

          -- 'Go To Selected Group' button.
          if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, vec24, cols.unlinkCol, nil, nil, 'goToSelectedPlacedGroup') then
            groupMgr.goToPlacedGroup(i)
          end
          im.tooltip('Go to this group.')
          im.NextColumn()

          im.Separator()
        end
        im.EndListBox()
        im.PopItemWidth()
      end

      im.Separator()

      im.Columns(5, "toolWindowCols2bv", false)
      if not isCreateGroup and not isGroupPlaceMode and not isAutoJctMode and not isFinalise then
        local groupsButtonCol = cols.blueB
        if mfe.isGroupsListWinOpen then groupsButtonCol = cols.blueD end
        if editor.uiIconImageButton(editor.icons.roadStack, vec36, groupsButtonCol, nil, nil, 'GroupsButton') then
          mfe.isGroupsListWinOpen = not mfe.isGroupsListWinOpen
          if mfe.isNodeEditWinOpen then
            editor.hideWindow(win.nodeEditWinName)
            mfe.isNodeEditWinOpen = false
          end
          if mfe.isProfileEditWinOpen then
            editor.hideWindow(win.profileEditWinName)
            mfe.isProfileEditWinOpen = false
          end
          if mfe.isProfilesListWinOpen then
            editor.hideWindow(win.profilesListWinName)
            mfe.isProfilesListWinOpen = false
            profileMgr.goToOldView()
            roadMgr.removeHiddenRoads()
          end
          if mfe.isMeshSelectWinOpen then
            editor.hideWindow(win.meshSelectWinName)
            mfe.isMeshSelectWinOpen = false
            staticMgr.removeAuditionMesh()
            staticMgr.goToOldView()
          end
          if mfe.isGroupsListWinOpen then
            if not hasGroupsListBeenComputed then
              groupMgr.getDefaultGroups()                                                         -- Populate the default prefab groups.
              hasGroupsListBeenComputed = true
            end
            editor.showWindow(win.groupsListWinName)
            roadMgr.removeHiddenRoads()
            groupMgr.addGroupToRoadsAudition(mfe.selectedGroupIdx)
          else
            editor.hideWindow(win.groupsListWinName)
            groupMgr.goToOldView()
            roadMgr.removeHiddenRoads()
          end
        end
        im.tooltip('Open/close the group templates selection window.')
      end
      im.SameLine()
      im.NextColumn()

      -- 'Create Group' button.
      if not isGroupPlaceMode and not isAutoJctMode and not isFinalise then
        local createGroupButtonCol = cols.blueB
        if isCreateGroup then createGroupButtonCol = cols.blueD end
        if editor.uiIconImageButton(editor.icons.roadStackPlus, vec36, createGroupButtonCol, nil, nil, 'CreateGroupButton') then
          isCreateGroup = not isCreateGroup
          if isCreateGroup then
            closeAllWindows()
            table.clear(gPolygon)
          end
        end
        im.tooltip('Create a new group template, by drawing a polygon around roads in the session.')
      end
      im.SameLine()
      im.NextColumn()

      im.Dummy(vec36)
      im.SameLine()
      im.NextColumn()
      im.Dummy(vec36)
      im.SameLine()
      im.NextColumn()
      im.Dummy(vec36)
      im.NextColumn()

      im.Columns(1)

      -- Terraforming options.
      local placedGroups = groupMgr.getPlacedGroups()
      if #placedGroups > 0 and terrain and mfe.selectedPlacedGroupIdx and placedGroups[mfe.selectedPlacedGroupIdx] then
        im.Separator()
        if im.TreeNode1("Terraform Control [Group]") then
          im.Columns(2, 'TerraGroupRow1', false)
          if editor.uiIconImageButton(editor.icons.terrainToTwoLines, vec36, cols.greenB, nil, nil, 'terraformGroupBtn') then
            terra.terraformMultiRoads(terraParams.domainOfInfluence[0], terraParams.terraMargin[0], placedGroups[mfe.selectedPlacedGroupIdx])
          end
          im.tooltip('Terraform the terrain to the selected group.')
          im.SameLine()
          im.NextColumn()
          if im.Checkbox("Show", terraParams.isShowGroup) then
            if terraParams.isShowGroup[0] then
              terraParams.isShowSingleRoad = im.BoolPtr(false)
            end
          end
          im.tooltip('Show the proposed terraforming range on the map, for the selected group.')
          im.SameLine()
          im.Dummy(vec36)
          im.NextColumn()
          im.Columns(1)

          -- 'Domain Of Influence' and 'Margin' sliders (for terraforming).
          im.PushItemWidth(-1)
          im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
          im.SliderInt("###99949", terraParams.domainOfInfluence, 1, 500, "Domain Of Influence (m) %d")
          im.tooltip('Set the domain of influence of the terraforming, in meters.')
          im.SliderFloat("###99948", terraParams.terraMargin, 0.0, 20.0, "Margin (m) = %.3f")
          im.tooltip('Set the terraforming margin (around road), in meters.')
          im.PopStyleVar()
          im.PopItemWidth()
          im.TreePop()
        end
      end

      im.EndTabItem()
    end

    if selectedTab == 4 then
      im.TextColored(cols.greenB, 'Disk Options:')
      im.Checkbox('Load Terrains With Sessions', isLoadTerrain)
      im.tooltip('On loading a session, the saved terrain (.png) file will also be loaded to the map (checked), otherwise only the road network data will be loaded (unchecked).')
      im.EndTabItem()
    end

    im.EndChild()
    im.Separator()
  end
end

-- Handles the node edit sub-window.
local function handleNodeEditSubWindow(roads)
  if mfe.isNodeEditWinOpen then
    if editor.beginWindow(win.nodeEditWinName, "Node: [" .. tostring(mfe.selectedNodeIdx) .. "]###6611", im.WindowFlags_NoCollapse) then
      local road = roadMgr.roads[mfe.selectedRoadIdx]
      if road then
        local nodes = road.nodes
        local node = nodes[mfe.selectedNodeIdx]
        if node then

          local widths = node.widths

          -- Left lanes.
          im.Separator()
          im.Text('Left Lanes:')
          im.PushItemWidth(-1)
          local wCtr = 7172
          if im.BeginListBox('###87') then
            im.Columns(5, "roadsListBoxColumnsLeft", true)
            for i = -20, -1 do
              if widths[i] then
                local flag = i == mfe.selectedLaneIdx
                if im.Selectable1('[' .. tostring(i) .. ']', flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                  mfe.selectedLaneIdx = i
                end
                im.SameLine()
                im.NextColumn()

                local laneType = road.profile[i].type
                im.Text(laneType)
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Width' input box.
                if im.InputFloat("W###" .. tostring(wCtr), node.widths[i], 0.1, 0.0) then
                  node.widths[i] = im.FloatPtr(max(0.0, min(50.0, node.widths[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the width of this lane.')
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Left Relative Height' input box.
                if im.InputFloat("H(L)###" .. tostring(wCtr), node.heightsL[i], 0.1, 0.0) then
                  node.heightsL[i] = im.FloatPtr(max(0.0, min(5.0, node.heightsL[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the relative height of the left edge of this lane (height above base).')
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Right Relative Height' input box.
                if im.InputFloat("H(R)###" .. tostring(wCtr), node.heightsR[i], 0.1, 0.0) then
                  node.heightsR[i] = im.FloatPtr(max(0.0, min(5.0, node.heightsR[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the relative height of the right edge of this lane (height above base).')
                im.SameLine()
                im.Separator()
                im.NextColumn()
              end
            end
            im.EndListBox()
          end
          im.PopItemWidth()

          -- Right lane widths.
          im.Separator()
          im.Text('Right Lanes:')
          im.PushItemWidth(-1)
          if im.BeginListBox('###172') then
              im.Columns(5, "roadsListBoxColumnsRight", true)
              for i = 1, 20 do
              if widths[i] then
                local flag = i == mfe.selectedLaneIdx
                if im.Selectable1('[' .. tostring(i) .. ']', flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                  mfe.selectedLaneIdx = i
                end
                im.SameLine()
                im.NextColumn()

                local laneType = road.profile[i].type
                im.Text(laneType)
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Width' input box.
                if im.InputFloat("W###" .. tostring(wCtr), node.widths[i], 0.1, 0.0) then
                  node.widths[i] = im.FloatPtr(max(0.0, min(50.0, node.widths[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the width of this lane.')
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Left Relative Height' input box.
                if im.InputFloat("H(L)###" .. tostring(wCtr), node.heightsL[i], 0.1, 0.0) then
                  node.heightsL[i] = im.FloatPtr(max(0.0, min(5.0, node.heightsL[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the relative height of the left edge of this lane (height above base).')
                im.SameLine()
                im.NextColumn()

                -- 'Set Node Right Relative Height' input box.
                if im.InputFloat("H(R)###" .. tostring(wCtr), node.heightsR[i], 0.1, 0.0) then
                  node.heightsR[i] = im.FloatPtr(max(0.0, min(5.0, node.heightsR[i][0])))
                  roadMgr.setDirty(roads[mfe.selectedRoadIdx])
                end
                wCtr = wCtr + 1
                im.tooltip('Sets the relative height of the right edge of this lane (height above base).')
                im.SameLine()
                im.Separator()
                im.NextColumn()
              end
            end
            im.EndListBox()
          end
          im.PopItemWidth()

          im.Columns(3, "roadsListSliderColumns3", true)

          -- The relative height and lateral rotation controls only appear if the road is not set to conform to the terrain.
          -- [This is also not available for arc middle nodes].
          if not road.isConformRoadToTerrain[0] and not (road.isArc and mfe.selectedNodeIdx == 2) then

            -- The 'Relative Height' input box.
            local oldHeight = node.p.z
            local nodeHeight = im.FloatPtr(oldHeight)
            if im.InputFloat("Height", nodeHeight, 0.1, 0.0) then
              roadMgr.adjustHeight(nodeHeight[0], oldHeight, mfe.selectedNodeIdx, mfe.selectedRoadIdx)
              nodeHeight = im.FloatPtr(max(0, min(2000, nodeHeight[0])))
            end
            im.tooltip('The elevation at the node, in meters.')
            im.SameLine()
            im.NextColumn()

            -- The 'Lateral Rotation' input box.
            local oldRot = node.rot[0]
            im.InputFloat("Rotation", node.rot, 0.25, 0.0)
            im.tooltip('The lateral rotation at this node, in degrees.')
            if oldRot ~= node.rot[0] then
              node.isAutoBanked = false                                                             -- If user changes lateral rotation, then auto banking switches off for this node.
              roadMgr.adjustLateralRotation(node.rot[0], oldRot, mfe.selectedNodeIdx, mfe.selectedRoadIdx)
            end
            im.SameLine()
            im.NextColumn()
          else
            im.SameLine()
            im.NextColumn()
            im.SameLine()
            im.NextColumn()
          end

          -- The 'Civil Engineering Node Incircle Radius' slider.
          if road.isCivilEngRoads[0] then
            local oldICR = node.incircleRad[0]
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            im.SliderFloat("Arc Rad", node.incircleRad, 1.0, 2.0, "Radius = %.3f")
            im.tooltip('The radius of the arc at this node.')
            im.PopStyleVar()
            node.incircleRad = im.FloatPtr(min(2.0, max(1.0, node.incircleRad[0])))
            if oldICR ~= node.incircleRad[0] then
              roadMgr.setDirty(road)
            end
          end
          im.SameLine()
          im.NextColumn()
        else
          mfe.isNodeEditWinOpen = false                                                             -- Node no longer exists, so close the node edit window.
          editor.hideWindow(win.nodeEditWinName)
        end
      end
    else
      mfe.isNodeEditWinOpen = false -- handle close sub-window.
    end
  end
end

-- Handles the profiles list sub-window.
local function handleProfilesListSubWindow(roads)
  if mfe.isProfilesListWinOpen then
    if editor.beginWindow(win.profilesListWinName, "Profile Templates###12", im.WindowFlags_NoCollapse) then
      im.PushItemWidth(-1)

      if im.BeginListBox('###7777', im.ImVec2(-1, 300)) then

        im.Columns(6, "profilesListBoxColumns", true)
        im.SetColumnWidth(0, 0)
        im.SetColumnWidth(1, 200)
        im.SetColumnWidth(2, 40)
        im.SetColumnWidth(3, 40)
        im.SetColumnWidth(4, 40)
        im.SetColumnWidth(5, 40)

        local numProfiles = #profileMgr.profiles
        local wCtr = 500
        for i = 1, numProfiles do
          local profile = profileMgr.profiles[i]
          if not profile.isHidden then
            local flag = i == mfe.selectedProfileIdx
            if im.Selectable1("###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
              mfe.selectedProfileIdx = i
              roadMgr.setAuditionProfileDirty()
              isEditProfileDirty = true
            end
            wCtr = wCtr + 1
            im.SameLine()
            im.NextColumn()
            if mfe.isProfileIdxChanged and flag then
              im.SetScrollHereY()
              mfe.isProfileIdxChanged = false
            end

            im.PushItemWidth(200)
            if im.InputText("###" .. tostring(wCtr), profile.name, 32) then
              mfe.selectedProfileIdx = i
            end
            im.tooltip('Change the name of this template profile.')
            im.PopItemWidth()
            wCtr = wCtr + 1

            im.SameLine()
            im.NextColumn()

            -- 'Select Profile' button.
            if editor.uiIconImageButton(editor.icons.forest_select, vec24, cols.fullWhite, nil, nil, 'selectProfileBtn') then
              local roadPre = copyDataState()
              updateRoadToNewProfileFromIdx(i)
              roadMgr.computeRoadRenderDataSingle(mfe.selectedRoadIdx)
              profileMgr.updateCondition(roads[mfe.selectedRoadIdx])
              local roadPost = copyDataState()
              editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
              mfe.selectedLayerIdx = 1
              isEditProfileDirty = false
              mfe.isProfilesListWinOpen = false -- handle close sub-window.
              if mfe.isProfileEditWinOpen then
                editor.hideWindow(win.profileEditWinName)
                mfe.isProfileEditWinOpen = false
                editor.hideWindow(win.materialSelectWinName)
                mfe.isMaterialSelectWinOpen = false
              end
              profileMgr.goToOldView()
              roadMgr.removeHiddenRoads()
              mfe.selectedRoadIdx = getSelRoadIdx(mfe.selectedRoadIdx)
              roadMgr.setDirty(roads[mfe.selectedRoadIdx])
            end
            im.tooltip('Select/apply this profile to the selected road.')
            im.SameLine()
            im.NextColumn()

            -- 'Edit Selected Profile' button.
            local editProfileCol = cols.blueB
            if mfe.isProfileEditWinOpen and i == mfe.selectedProfileIdx then editProfileCol = cols.blueD end
            if editor.uiIconImageButton(editor.icons.edit, vec24, editProfileCol, nil, nil, 'editProfile') then
              if i == mfe.selectedProfileIdx then
                if mfe.isProfileEditWinOpen then                                                    -- If this profile is already selected, toggle window open/closed.
                  editor.hideWindow(win.profileEditWinName)
                else
                  editor.showWindow(win.profileEditWinName)
                end
                mfe.isProfileEditWinOpen = not mfe.isProfileEditWinOpen
              else                                                                                  -- If profile not currently selected, open/keep window open, but with this profile.
                editor.showWindow(win.profileEditWinName)
                mfe.isProfileEditWinOpen = true
              end
              if not mfe.isProfilesListWinOpen and not mfe.isProfileEditWinOpen then
                profileMgr.goToOldView()
                roadMgr.removeHiddenRoads()
              end
              if mfe.isGroupsListWinOpen then
                editor.hideWindow(win.groupsListWinName)
                mfe.isGroupsListWinOpen = false
                groupMgr.goToOldView()
                roadMgr.removeHiddenRoads()
              end
              if mfe.isMeshSelectWinOpen then
                editor.hideWindow(win.meshSelectWinName)
                mfe.isMeshSelectWinOpen = false
                staticMgr.removeAuditionMesh()
                staticMgr.goToOldView()
              end
              local pCopy = profileMgr.copyProfile(profileMgr.profiles[i])
              pCopy.name = im.ArrayChar(32, ffi.string(pCopy.name) .. ' [copy]')
              profileMgr.profiles[#profileMgr.profiles + 1] = pCopy
              mfe.selectedProfileIdx = #profileMgr.profiles
              roadMgr.setAuditionProfileDirty()
              isEditProfileDirty = true
            end
            im.tooltip('Edit this template profile (opens edit window), before applying it to a road.')
            im.SameLine()
            im.NextColumn()

            -- 'Save Road Profile' button.
            if editor.uiIconImageButton(editor.icons.floppyDisk, vec24, cols.fullWhite, nil, nil, 'SaveRoadProfileBtn') then
              profileMgr.save(profile)
            end
            im.tooltip('Save this profile template to disk.')
            im.SameLine()
            im.NextColumn()

            -- 'Delete Profile' button.
            if profile.isDeletable then
              if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'DeleteProfileBtn') then
                table.remove(profileMgr.profiles, i)
                mfe.selectedProfileIdx = 1
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
                return
              end
              im.tooltip('Delete this profile template.')
            else
              im.Dummy(vec24)
            end
            im.NextColumn()

            im.Separator()
          end
        end
        im.EndListBox()
      end
      im.PopItemWidth()

      im.Columns(1)

      im.Separator()

      im.Columns(2, 'profTemplateBottomCols1', false)

      -- 'Load Lateral Road Profile' button.
      if editor.uiIconImageButton(editor.icons.roadFolderPlus, vec36, nil, nil, nil, 'loadProfileBtn') then
        profileMgr.load()
      end
      im.tooltip('Load a profile from disk.')
      im.SameLine()
      im.NextColumn()

      -- 'Delete All User Templates' button.
      if editor.uiIconImageButton(editor.icons.refresh, vec36, nil, nil, nil, 'resetTemplatesBtn') then
        profileMgr.resetTemplates()
        mfe.selectedProfileIdx = 1
        roadMgr.setAuditionProfileDirty()
        isEditProfileDirty = true
        return
      end
      im.tooltip('Reset all templates (also removes any loaded templates).')
      im.NextColumn()

    else
      mfe.selectedLayerIdx = 1
      isEditProfileDirty = false
      mfe.isProfilesListWinOpen = false -- handle close sub-window.
      if mfe.isProfileEditWinOpen then
        editor.hideWindow(win.profileEditWinName)
        mfe.isProfileEditWinOpen = false
        editor.hideWindow(win.materialSelectWinName)
        mfe.isMaterialSelectWinOpen = false
      end
      profileMgr.goToOldView()
      roadMgr.removeHiddenRoads()
    end
  end
end

-- Handles the profile template edit sub-window.
local function handleProfileEditSubWindow(roads)
  if mfe.isProfileEditWinOpen then
    local profile = profileMgr.profiles[mfe.selectedProfileIdx]
    local numLeft, numRight = profileMgr.getNumLanesLR(profile)
    local numLanes = numLeft + numRight
    if editor.beginWindow(win.profileEditWinName, "Template Editor - [" .. tostring(ffi.string(profile.name)) .. "]###1113", im.WindowFlags_NoCollapse) then
      local wCtr = 1

      -- Left lanes.
      im.PushItemWidth(-1)
      if im.CollapsingHeader1("Left Lanes:", im.TreeNodeFlags_DefaultOpen) then
        if im.BeginListBox('###107') then

          im.Columns(10, "profileEditBoxColumnsLeft", true)
          im.SetColumnWidth(0, 40)
          im.SetColumnWidth(1, 40)
          im.SetColumnWidth(2, 40)
          im.SetColumnWidth(3, 40)
          im.SetColumnWidth(4, 40)
          im.SetColumnWidth(5, 150)
          im.SetColumnWidth(6, 40)
          im.SetColumnWidth(7, 150)
          im.SetColumnWidth(8, 150)
          im.SetColumnWidth(9, 150)

          -- Iterate over all possible left lanes, and display them in order from outermost to innermost.
          for i = -20, -1 do
            local lane = profile[i]
            if lane then
              local flag = i == mfe.selectedLaneIdx
              if im.Selectable1(tostring(i), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                mfe.selectedLaneIdx = i
              end
              im.tooltip('The lane index (-ve for left lanes, +ve for right lanes).')
              im.SameLine()
              im.NextColumn()

              -- 'Remove Selected Lane' button.
              if numLanes > 1 then
                if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeSelectedLaneLeft') then
                  profileMgr.removeLane(mfe.selectedProfileIdx, i, 'left')
                  roadMgr.setAuditionProfileDirty()
                  isEditProfileDirty = true
                end
                im.tooltip('Remove this lane from profile.')
              else
                im.Dummy(vec24)
              end
              im.SameLine()
              im.NextColumn()

              -- 'Add New Lane Above' button.
              if editor.uiIconImageButton(editor.icons.vertical_align_top, vec24, cols.greenB, nil, nil, 'addLaneAboveLeft') then
                profileMgr.addLane(mfe.selectedProfileIdx, i, 'left', 'above')
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
              im.tooltip('Add a new lane above this lane.')
              im.SameLine()
              im.NextColumn()

              -- 'Add New Lane Below' button.
              if editor.uiIconImageButton(editor.icons.vertical_align_bottom, vec24, cols.greenB, nil, nil, 'addLaneBelowLeft') then
                profileMgr.addLane(mfe.selectedProfileIdx, i, 'left', 'below')
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
              im.tooltip('Add a new lane below this lane.')
              im.SameLine()
              im.NextColumn()

              -- 'Select New Lane Type' button.
              if editor.uiIconImageButton(editor.icons.fg_lt, vec24, cols.blueB, nil, nil, 'selectLaneTypeLeft1Button') then
                lane.type = profileMgr.cycleLaneTypeBack(lane.type)
                roadMgr.setAuditionProfileDirty()
              end
              im.tooltip('Cycle back through available lane types.')
              im.SameLine()
              im.NextColumn()

              -- Display the current type for this lane.
              im.Text(lane.type)
              im.tooltip('The lane type.')
              im.SameLine()
              im.NextColumn()

              -- 'Select New Lane Type' button.
              if editor.uiIconImageButton(editor.icons.fg_gt, vec24, cols.blueB, nil, nil, 'selectLaneTypeLeft2Button') then
                lane.type = profileMgr.cycleLaneType(lane.type)
                roadMgr.setAuditionProfileDirty()
              end
              im.tooltip('Cycle forward through available lane types.')
              im.SameLine()
              im.NextColumn()

              -- Width input box.
              local oldWidth = lane.width[0]
              im.PushItemWidth(-30)
              im.InputFloat("W###" .. tostring(wCtr), lane.width, 0.1, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The lane width.')
              im.PopItemWidth()
              im.SameLine()
              im.NextColumn()
              lane.width = im.FloatPtr(max(0.0, min(10.0, lane.width[0])))
              if lane.width[0] ~= oldWidth then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end

              -- Relative height (left) input box.
              local oldHeightL = lane.heightL[0]
              im.PushItemWidth(-30)
              im.InputFloat("H(L)###" .. tostring(wCtr), lane.heightL, 0.01, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The relative height of the lane left edge.')
              im.PopItemWidth()
              im.SameLine()
              im.NextColumn()
              lane.heightL = im.FloatPtr(max(0.0, min(5.0, lane.heightL[0])))
              if lane.heightL[0] ~= oldHeightL then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end

              -- Relative height (right) input box.
              local oldHeightR = lane.heightR[0]
              im.PushItemWidth(-30)
              im.InputFloat("H(R)###" .. tostring(wCtr), lane.heightR, 0.01, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The relative height of the lane right edge.')
              im.PopItemWidth()
              im.SameLine()
              im.Separator()
              im.NextColumn()
              lane.heightR = im.FloatPtr(max(0.0, min(5.0, lane.heightR[0])))
              if lane.heightR[0] ~= oldHeightR then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
            end
          end
          im.EndListBox()
        end
      end
      im.PopItemWidth()

      -- Right lanes.
      im.Separator()
      im.PushItemWidth(-1)
      if im.CollapsingHeader1("Right Lanes:", im.TreeNodeFlags_DefaultOpen) then
        if im.BeginListBox('###108') then

          im.Columns(10, "profileEditBoxColumnsRight", true)
          im.SetColumnWidth(0, 40)
          im.SetColumnWidth(1, 40)
          im.SetColumnWidth(2, 40)
          im.SetColumnWidth(3, 40)
          im.SetColumnWidth(4, 40)
          im.SetColumnWidth(5, 150)
          im.SetColumnWidth(6, 40)
          im.SetColumnWidth(7, 150)
          im.SetColumnWidth(8, 150)
          im.SetColumnWidth(9, 150)

          -- Iterate over all possible right lanes, and display them in order from innermost to outermost.
          for i = 1, 20 do
            local lane = profile[i]
            if lane then
              local flag = i == mfe.selectedLaneIdx
              if im.Selectable1(tostring(i), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                mfe.selectedLaneIdx = i
              end
              im.tooltip('The lane index.')
              im.SameLine()
              im.NextColumn()

              -- 'Remove Selected Lane' button.
              if numLanes > 1 then
                if editor.uiIconImageButton(editor.icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeSelectedLaneRight') then
                  profileMgr.removeLane(mfe.selectedProfileIdx, i, 'right')
                  roadMgr.setAuditionProfileDirty()
                  isEditProfileDirty = true
                end
                im.tooltip('Remove this lane from profile.')
              else
                im.Dummy(vec24)
              end
              im.SameLine()
              im.NextColumn()

              -- 'Add New Lane Above' button.
              if editor.uiIconImageButton(editor.icons.vertical_align_top, vec24, cols.greenB, nil, nil, 'addLaneAboveRight') then
                profileMgr.addLane(mfe.selectedProfileIdx, i, 'right', 'above')
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
              im.tooltip('Add a new lane above this lane.')
              im.SameLine()
              im.NextColumn()

              -- 'Add New Lane Below' button.
              if editor.uiIconImageButton(editor.icons.vertical_align_bottom, vec24, cols.greenB, nil, nil, 'addLaneBelowRight') then
                profileMgr.addLane(mfe.selectedProfileIdx, i, 'right', 'below')
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
              im.tooltip('Add a new lane below this lane.')
              im.SameLine()
              im.NextColumn()

              -- 'Select New Lane Type' button.
              if editor.uiIconImageButton(editor.icons.fg_lt, vec24, cols.blueB, nil, nil, 'selectLaneTypeRight1Button') then
                lane.type = profileMgr.cycleLaneTypeBack(lane.type)
                roadMgr.setAuditionProfileDirty()
              end
              im.tooltip('Cycle back through available lane types.')
              im.SameLine()
              im.NextColumn()

              -- Display the current type for this lane.
              im.Text(lane.type)
              im.tooltip('The lane type.')
              im.SameLine()
              im.NextColumn()

              -- 'Select New Lane Type' button.
              if editor.uiIconImageButton(editor.icons.fg_gt, vec24, cols.blueB, nil, nil, 'selectLaneTypeRight2Button') then
                lane.type = profileMgr.cycleLaneType(lane.type)
                roadMgr.setAuditionProfileDirty()
              end
              im.tooltip('Cycle forward through available lane types.')
              im.SameLine()
              im.NextColumn()

              -- Width input box.
              local oldWidth = lane.width[0]
              im.PushItemWidth(-30)
              im.InputFloat("W###" .. tostring(wCtr), lane.width, 0.1, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The lane width.')
              im.PopItemWidth()
              im.SameLine()
              im.NextColumn()
              lane.width = im.FloatPtr(max(0.0, min(10.0, lane.width[0])))
              if lane.width[0] ~= oldWidth then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end

              -- Relative height (left) input box.
              local oldHeightL = lane.heightL[0]
              im.PushItemWidth(-30)
              im.InputFloat("H(L)###" .. tostring(wCtr), lane.heightL, 0.01, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The relative height of the lane left edge.')
              im.PopItemWidth()
              im.SameLine()
              im.NextColumn()
              lane.heightL = im.FloatPtr(max(0.0, min(5.0, lane.heightL[0])))
              if lane.heightL[0] ~= oldHeightL then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end

              -- Relative height (right) input box.
              local oldHeightR = lane.heightR[0]
              im.PushItemWidth(-30)
              im.InputFloat("H(R)###" .. tostring(wCtr), lane.heightR, 0.01, 0.0)
              wCtr = wCtr + 1
              im.tooltip('The relative height of the right outer edge.')
              im.PopItemWidth()
              im.SameLine()
              im.Separator()
              im.NextColumn()
              lane.heightR = im.FloatPtr(max(0.0, min(5.0, lane.heightR[0])))
              if lane.heightR[0] ~= oldHeightR then
                roadMgr.setAuditionProfileDirty()
                isEditProfileDirty = true
              end
            end
          end
          im.EndListBox()
        end
      end
      im.PopItemWidth()
      im.Separator()
    else
      mfe.isProfileEditWinOpen = false
      editor.hideWindow(win.materialSelectWinName)
      mfe.isMaterialSelectWinOpen = false
      if not mfe.isProfilesListWinOpen then
        profileMgr.goToOldView()
        roadMgr.removeHiddenRoads()
      end
    end
  end
end

local function getTexObj(absPath)
  if texObjs[absPath] == nil and loadedTextures < 5 then
    local texture = editor.texObj(absPath)
    loadedTextures = loadedTextures + 1
    if texture and not tableIsEmpty(texture) then
      texObjs[absPath] = texture
    else
      texObjs[absPath] = false
    end
  end

  return texObjs[absPath]
end

-- Handles the material selection window.
local function handleMaterialSelectionSubWindow()
  if mfe.isMaterialSelectWinOpen then
    loadedTextures = 0
    im.PushID1("SimSetNameFilter")
    im.ImGuiTextFilter_Draw(simSetNameFilter, "", 200)
    im.PopID()
    im.SameLine()
    if editor.beginWindow(win.materialSelectWinName, "Material Selector###82", im.WindowFlags_NoCollapse) then
      for i = 1, tableSize(materialSet) do
        local obj = materialSet[i]
        local tag0 = obj:getField("materialTag", 0)
        local tag1 = obj:getField("materialTag", 1)
        local tag2 = obj:getField("materialTag", 2)
        local isInFilter =
          tag0 == 'RoadAndPath' or tag1 == 'RoadAndPath' or tag2 == 'RoadAndPath' or
          tag0 == 'decal' or tag1 == 'decal' or tag2 == 'decal'
        local objName = obj:getName()
        if objName ~= "" and isInFilter then
          local skipMaterial = false
          local isSelected = false
          local clickedImage = false
          local mat = scenetree.findObject(objName)
          if mat then
            local imgPath = mat:getField("diffuseMap", 0)
            local absPath = imgPath
            if absPath ~= "" then                                                                   -- Check if path is absolute or relative.
              absPath = (string.find(absPath, "/") ~= nil and absPath or (mat:getPath() .. absPath))
            end
            local texture = getTexObj(absPath)
            if texture then
              if not (texture.texId == nil) then
                if im.ImageButton(
                  "##DisplayMaterialButton",
                  texture.texId,
                  im.ImVec2(32, 32),
                  im.ImVec2Zero,
                  im.ImVec2One,
                  im.ImColorByRGB(255, 255, 255, 255).Value,
                  im.ImColorByRGB(255, 255, 255, 255).Value) then
                    clickedImage = true
                end
                im.SameLine()
              end
            else
              skipMaterial = true
            end
            if not skipMaterial then
              if im.Selectable1(objName, isSelected) or clickedImage then

                -- Set the material at the chosen target.
                if mfe.isMaterialForRoad then                                                       -- Material is for a road property (paint lines, gutters, etc).
                  if mfe.materialForRoadTarget == 'centerline' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.centerlineMat = objName
                  elseif mfe.materialForRoadTarget == 'edge_L' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.edgeMatL = objName
                  elseif mfe.materialForRoadTarget == 'edge_R' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.edgeMatR = objName
                  elseif mfe.materialForRoadTarget == 'laneDivisions' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.laneMarkingsMat = objName
                  elseif mfe.materialForRoadTarget == 'endStopLine_S' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.endStopMatS = objName
                  elseif mfe.materialForRoadTarget == 'endStopLine_E' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.endStopMatE = objName
                  elseif mfe.materialForRoadTarget == 'gutter' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.gutterMat = objName
                  elseif mfe.materialForRoadTarget == 'dirt' then
                    roadMgr.roads[mfe.selectedRoadIdx].profile.dirtMat = objName
                  elseif mfe.materialForRoadTarget == 'jctEdgeBlend' then
                    jctMgr.junctions[mfe.selectedJctIdx].edgeBlendMat = objName
                    jctMgr.updateJunctionAfterChange(mfe.selectedJctIdx)
                  end
                  profileMgr.updateCondition(roadMgr.roads[mfe.selectedRoadIdx])
                  mfe.selectedLayerIdx = 1
                elseif mfe.isMaterialForEdgeBlendLeft then                                          -- Material is for the left-edge blending of a road.
                  roadMgr.roads[mfe.selectedRoadIdx].profile.blendLeftMat = objName
                  profileMgr.updateCondition(roadMgr.roads[mfe.selectedRoadIdx])
                  mfe.selectedLayerIdx = 1
                elseif mfe.isMaterialForEdgeBlendRight then                                         -- Material is for the right-edge blending of a road.
                  roadMgr.roads[mfe.selectedRoadIdx].profile.blendRightMat = objName
                  profileMgr.updateCondition(roadMgr.roads[mfe.selectedRoadIdx])
                  mfe.selectedLayerIdx = 1
                elseif mfe.isMaterialForJctArrows then                                              -- Material is for the arrows of a junction.
                  jctMgr.junctions[mfe.selectedJctIdx].profile.arrowMat = objName
                  jctMgr.updateJunctionAfterChange(mfe.selectedJctIdx)
                elseif mfe.isMaterialForOverlay then                                                -- Material is for an overlay.
                  roadMgr.roads[mfe.selectedRoadIdx].overlayMat = objName
                else                                                                                -- Material is for a layer.
                  mfe.selProfileMaterial.layers[mfe.selectedLayerIdx].mat = objName
                end
              end
              if isSelected then
                im.SetItemDefaultFocus()                                                            -- Set the initial focus when opening the combo.
              end
            end
          end
        end
      end
    else
      mfe.isMaterialSelectWinOpen = false           -- Handle close sub-window.
      editor.hideWindow(win.materialSelectWinName)
    end
  end
end

-- Handles the static mesh selection window.
local function handleMeshSelectionSubWindow()
  if mfe.isMeshSelectWinOpen then
    if editor.beginWindow(win.meshSelectWinName, "Static Mesh Selector###114", im.WindowFlags_NoCollapse) then
      im.Separator()
      im.PushItemWidth(-1)
      if im.BeginListBox('##123', im.ImVec2(300, 500)) then
        im.Columns(1, "meshSelectListboxColumns")
        local availStaticMeshes = staticMgr.availStaticMeshes
        for i = 1, #availStaticMeshes do
          local flag = i == mfe.selectedMeshIdx
          if im.Selectable1(availStaticMeshes[i].filename, flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            mfe.selectedMeshIdx = i
            staticMgr.addMeshToAudition(mfe.selectedMeshIdx, roadMgr.roads[mfe.selectedRoadIdx], mfe.selectedCustom)

            local prof = roadMgr.roads[mfe.selectedRoadIdx].profile
            if mfe.isSingleMeshSelect then
              prof.layers[mfe.selectedCustom].mat = availStaticMeshes[i].path                       -- For single static mesh units.
              prof.layers[mfe.selectedCustom].matDisplay = availStaticMeshes[i].filename
            else
              prof.layers[mfe.selectedCustom].mat = availStaticMeshes[i].path                       -- For mesh lanes.
              prof.layers[mfe.selectedCustom].matDisplay = availStaticMeshes[i].filename
            end
          end
          im.tooltip('Select this static mesh.')
          im.NextColumn()
        end

        im.EndListBox()
      end
      if editor.uiIconImageButton(editor.icons.check_box, vec24, cols.blueB, nil, nil, 'meshOkButton') then
        if mfe.isMeshSelectWinOpen then
          editor.hideWindow(win.meshSelectWinName)
          mfe.isMeshSelectWinOpen = false
          staticMgr.removeAuditionMesh()
          staticMgr.goToOldView()
        end
        mfe.isMeshSelectWinOpen = false           -- Handle close sub-window.
        editor.hideWindow(win.meshSelectWinName)
      end
      im.tooltip('Use the selected mesh and return to the main view.')
      im.PopItemWidth()
    else
      if mfe.isMeshSelectWinOpen then
        editor.hideWindow(win.meshSelectWinName)
        mfe.isMeshSelectWinOpen = false
        staticMgr.removeAuditionMesh()
        staticMgr.goToOldView()
      end
      mfe.isMeshSelectWinOpen = false           -- Handle close sub-window.
      editor.hideWindow(win.meshSelectWinName)
    end
  end
end

-- Handles the groups list sub window.
local function handleGroupsListSubWindow()
  if mfe.isGroupsListWinOpen then
    if editor.beginWindow(win.groupsListWinName, "Group Templates###3314", im.WindowFlags_NoCollapse) then

      im.Separator()
      im.PushItemWidth(-1)
      if im.BeginListBox('##123', im.ImVec2(-1, 300)) then

        im.Columns(3, "groupsListBoxColumns", true)
        im.SetColumnWidth(0, 200)
        im.SetColumnWidth(1, 40)
        im.SetColumnWidth(2, 40)

        local numGroups = #groupMgr.groups
        for i = 1, numGroups do
          local group = groupMgr.groups[i]
          local flag = i == mfe.selectedGroupIdx
          if im.Selectable1(group.name, flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            if mfe.selectedGroupIdx ~= i then
              mfe.selectedGroupIdx = i
              roadMgr.removeHiddenRoads()
              groupMgr.addGroupToRoadsAudition(i)
            end
          end
          im.SameLine()
          im.NextColumn()

          -- 'Pick Group' button.
          if editor.uiIconImageButton(editor.icons.add_box, vec24, cols.blueB, nil, nil, 'pickGroupButton') then
            isGroupPlaceMode = true
            mfe.isGroupsListWinOpen = false
            editor.hideWindow(win.groupsListWinName)
            groupMgr.goToOldView()
            roadMgr.removeHiddenRoads()
            pGroup = groupMgr.addGroupToRoadsPlace(i, isConformGroupToTerrain)
            mfe.selectedGroupIdx = i
            stateGroupPre = copyDataState()
          end
          im.tooltip('Place this group template in the session.')
          im.SameLine()
          im.NextColumn()

          -- 'Save Group' button.
          if editor.uiIconImageButton(editor.icons.floppyDisk, vec24, cols.blueB, nil, nil, 'pickGroupButton') then
            groupMgr.save(i)
          end
          im.tooltip('Save this group template to disk.')
          im.Separator()
          im.NextColumn()
        end
        im.EndListBox()
      end
      im.PopItemWidth()

      im.Columns(2, "groupsListColumns", false)

      -- 'Load Group' button.
      if editor.uiIconImageButton(editor.icons.roadFolderPlus, vec36, cols.dullWhite, nil, nil, 'loadGroup') then
        groupMgr.load()
      end
      im.tooltip('Load a group template from disk.')
      im.SameLine()
      im.NextColumn()

      -- 'Conform Group To Terrain' button.
      -- [Only available if a terrain block is present].
      local isConfGToRButtonCol = cols.greenD
      if isConformGroupToTerrain then isConfGToRButtonCol = cols.greenB end
      if terrain then
        if editor.uiIconImageButton(editor.icons.terrain_height_raise, vec36, isConfGToRButtonCol, nil, nil, 'conformGroupToTerrainButton2') then
          isConformGroupToTerrain = not isConformGroupToTerrain
        end
        im.tooltip('Conform the group template to the terrain, upon placing.')
      end
      im.NextColumn()

    else
      mfe.isGroupsListWinOpen = false -- handle close sub-window.
      editor.hideWindow(win.groupsListWinName)
      groupMgr.goToOldView()
      roadMgr.removeHiddenRoads()
    end
  end
end

-- Handles the import options sub-window.
local function handleImportSubWindow()
  if mfe.isImportWinOpen then
    if editor.beginWindow(win.importWinName, "Import Options###6215", im.WindowFlags_NoCollapse) then

      if im.Checkbox("Offset To Terrain", importO2T) then
        if importO2T[0] then
          importCO = im.BoolPtr(false)
        end
      end
      im.tooltip('Apply vertical offset to imported road network, to sit on existing terrain.')

      im.Separator()

      if im.Checkbox("Use Custom Offset", importCO) then
        if importCO[0] then
          importO2T = im.BoolPtr(false)
        end
      end
      im.tooltip('Apply a custom vertical offset to imported road network.')

      if importCO[0] then
        im.PushItemWidth(-1)
        im.InputFloat("Offset", importCustomOffset, 0.1, 0.0)
        im.tooltip('Sets the custom vertical offset amount.')
        im.PopItemWidth()
      end

      im.Separator()

      if terrain then
        im.Checkbox("Terraform Terrain To Import", importTT2I)
        im.tooltip('Terraform the terrain to fit the imported road network.')
      end

      if importTT2I[0] then
        im.PushItemWidth(-1)
        im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
        im.SliderInt("###49", terraParams.domainOfInfluence, 1, 500, "Domain Of Influence (m) %d")
        im.tooltip('Set the domain of influence of the terraforming, in meters.')
        im.SliderFloat("###48", terraParams.terraMargin, 0.0, 20.0, "Margin (m) = %.3f")
        im.tooltip('Set the terraforming margin (around road), in meters.')
        im.PopStyleVar()
        im.PopItemWidth()
      end

      if editor.uiIconImageButton(editor.icons.ab_asset_jbeam, vec28, nil, nil, nil, 'importFromImportWdw') then
        mfe.isImportWinOpen = false
        editor.hideWindow(win.importWinName)
        jctMgr.clearAllJunctions()
        roadMgr.clearAllRoads()
        table.clear(profileMgr.profiles)
        profileMgr.populateProfileTemplates()
        import.import(importO2T[0], importCO[0], importTT2I[0], importCustomOffset[0], terraParams.domainOfInfluence[0], terraParams.terraMargin[0])
      end
      im.tooltip('Import from file.')
    else
      mfe.isImportWinOpen = false -- handle close sub-window.
    end
  end
end

-- World editor main callback for rendering the UI.
local function onEditorGui()

  if not isRoadArchitectActive then
    return
  end

  local roads, map = roadMgr.roads, roadMgr.map

  -- Cache the (relevant) mouse state.
  local mousePos = util.mouseOnMapPos()
  local isMouseClickedL, isMouseClickedR, isMouseDownL = im.IsMouseClicked(0), im.IsMouseClicked(1), im.IsMouseDown(0)
  local dt = mouseTimer:stopAndReset()
  timeSinceLastClick = timeSinceLastClick + dt
  local isDoubleClick = isMouseClickedL and timeSinceLastClick < doubleClickTime
  if isMouseClickedL then
    timeSinceLastClick = 0.0
  end
  if isMouseDownL then
    heldTime = heldTime + dt                                                                        -- We store the amount of time which the mouse has been held down for.
  else
    heldTime = 0.0
    mfe.isNewNodeFresh = false                                                                      -- When new nodes are added, we set this flag true until the mouse is released.
  end
  if isOpDelay then
    opDelayTime = opDelayTime + dt
    if opDelayTime > waitTime then
      isOpDelay, opDelayTime = false, 0.0
    end
  end

  -- Cache the (relevant) keyboard state.
  local isCtrlDown = im.IsKeyDown(keyIdx.ctrl)
  local isShiftDown = im.IsKeyDown(keyIdx.shift)
  local isAltDown = im.IsKeyDown(keyIdx.alt)
  local isADown = im.IsKeyDown(keyIdx.a)
  local isCDown = im.IsKeyDown(keyIdx.c)
  local isVDown = im.IsKeyDown(keyIdx.v)
  local isDelDown = im.IsKeyDown(keyIdx.del)
  if not isCtrlDown then                                                                            -- We only want CTRL to act once.
    mfe.hasCtrlFired = false
  end
  if not isDelDown then                                                                             -- We only want DEL to act once.
    mfe.hasDelFired = false
  end

  -- Check if the user is toggling the gimbal on/off.
  if isAltDown and isAltDown ~= lastAltDown then
    isGimbalActive = not isGimbalActive
  end
  lastAltDown = isAltDown

  -- Check if the user is attempting to copy/paste a profile.
  if isCtrlDown and isCDown and mfe.selectedRoadIdx then
    ctrlCProfile = profileMgr.copyProfile(roads[mfe.selectedRoadIdx].profile)
  end
  if isCtrlDown and isVDown and mfe.selectedRoadIdx then
    local roadPre = copyDataState()
    local road = roads[mfe.selectedRoadIdx]
    road.profile = ctrlCProfile
    road.laneKeys, road.leftKeys, road.rightKeys = profileMgr.computeLaneKeys(road.profile)
    roadMgr.updateWAndHToNewProfile(road)
    roadMgr.setDirty(road)
    local roadPost = copyDataState()
    profileMgr.updateCondition(roads[mfe.selectedRoadIdx])
    mfe.selectedLayerIdx = 1
    editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
  end

  -- If the user has finished dragging nodes, switch state.
  if isNodeBeingDragged and not isMouseDownL then
    if selectedLink then                                                                            -- If there is a proposed link when mouse is released, create the link.
      local roadPre = copyDataState()
      linkUtil.joinRoads(selectedLink)
      groupMgr.updateGroupsAfterRoadRemove()
      jctMgr.updateJunctionsAfterRoadRemove()
      local roadPost = copyDataState()
      editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
    elseif selectedCandidateJct then
      local roadPre = copyDataState()
      linkUtil.createSplitJunction(selectedCandidateJct)
      groupMgr.updateGroupsAfterRoadRemove()
      jctMgr.updateJunctionsAfterRoadRemove()
      local roadPost = copyDataState()
      editor.history:commitAction("EditRoad", { old = roadPre, new = roadPost }, editRoadUndo, editRoadRedo)
    end
    selectedLink = nil
    selectedCandidateJct = nil
    isNodeBeingDragged = false
    local mouseDragPost = copyDataState()
	  editor.history:commitAction("EditRoad", { old = mouseDragPre, new = mouseDragPost }, editRoadUndo, editRoadRedo)
  end

  -- Handle the gimbals.
  if isGimbalActive then                                                                            -- If the gimbal is being used to move nodes/roads.
    local multi = roadMgr.multi
    if #multi > 0 then
      handleGimbals(multiCentroidOnLeftHold or roadMgr.getMultiSelectionCentroid())
    else
      if mfe.selectedRoadIdx and mfe.selectedNodeIdx then
        local road = roads[mfe.selectedRoadIdx]
        if road then
          local node = road.nodes[mfe.selectedNodeIdx]
          if node then
            handleGimbals(node.p)
          end
        end
      end
    end
  elseif isNodeBeingDragged and not isShiftDown then
    handleNodeMouseDragging(mousePos)                                                             -- If the user is manually moving nodes with the mouse (gimbal inactive).
  end

  -- If the user wants to increase the node width by dragging the mouse.
  if isCtrlDown and not isMouseDownL and abs(im.GetIO().MouseWheel) > 0.01 then
    handleNodeWidthChangeOnDrag()
  end

  -- Handle the camera rotations for the audition views.
  if mfe.isGroupsListWinOpen then
    time = groupMgr.goToGroupView(mfe.selectedGroupIdx, timer, time)
    time = time + timer:stopAndReset() * 0.001
    if time > spinTime then
      groupMgr.manageRotateCam()
      time = time - spinTime
    end
  elseif mfe.isProfilesListWinOpen or mfe.isProfileEditWinOpen then
    time = profileMgr.goToProfileView(timer, time)
    time = time + timer:stopAndReset() * 0.001
    if time > spinTime then
      roadMgr.manageTempRoadSection(mfe.selectedProfileIdx)
      time = time - spinTime
    end
  elseif mfe.isMeshSelectWinOpen then
    time = staticMgr.goToMeshView(timer, time)
    time = time + timer:stopAndReset() * 0.001
    if time > spinTime then
      staticMgr.manageRotateCam()
      time = time - spinTime
    end
  end

  -- Handle the back-end (mode-specific functionality).
  selectedLink = nil                                                                                -- Keep the selected link/junction as nil, until one is selected.
  selectedCandidateJct = nil
  if not isFinalise and util.isMouseHoveringOverTerrain() then
    if isJctPlaceMode then                                                                          -- MODE: [junction placement]: The user is placing a selected junction.
      handlePlaceJct(mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown)
    elseif isCreateGroup then                                                                       -- MODE: [create group]: The user is creating a new group.
      handleCreateGroup(roads, mousePos, isMouseClickedL, isDoubleClick, isMouseClickedR)
    elseif isGroupPlaceMode then                                                                    -- MODE: [group placement]: The user is placing a selected group.
      handlePlaceGroup(roads, mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown)
    elseif isAutoJctMode then                                                                       -- MODE: [auto junction mode]: The user is creating an auto-junction.
      handleAutoJct(roads, mousePos, isDoubleClick, isMouseDownL, isMouseClickedR, isShiftDown) 
    else                                                                                            -- MODE: [road]: The user is creating/editing roads.
      handleCreateRoads(
        roads, mousePos,
        isMouseClickedL, isMouseDownL, isMouseClickedR, isDoubleClick,
        isCtrlDown, isADown, isShiftDown, isDelDown)
    end
  end

  -- Manage the updating of roads (geometry, meshes, decals, rendering).
  roadMgr.updateRoads()
  if not isFinalise then
    render.drawRoadMarkups(
      roads, map, roadMgr.getTree(),
      mfe.selectedRoadIdx, mfe.selectedNodeIdx, mfe.selectedLayerIdx,
      mfe.isGroupsListWinOpen, mfe.isProfilesListWinOpen,
      isCreateGroup, gPolygon,
      roadMgr.multi,
      selectedLink, selectedCandidateJct,
      isDisplayGuidelines, terraParams.isShowSingleRoad[0],
      terraParams.isShowGroup[0], groupMgr.getPlacedGroups()[mfe.selectedPlacedGroupIdx], terraParams)
  end

  -- Handle the front-end (UI).
  -- [Manage the display of each window of the editor].
  handleMainToolWindow(roads)
  handleNodeEditSubWindow(roads)
  handleProfilesListSubWindow(roads)
  handleProfileEditSubWindow(roads)
  handleGroupsListSubWindow()
  handleMaterialSelectionSubWindow()
  handleMeshSelectionSubWindow()
  handleImportSubWindow()

  -- Some preparation for the next iteration of the main loop callback.
  mouseLast = mousePos
end

-- Called when the 'Road Architect' icon is pressed.
local function onActivate()
  editor.clearObjectSelection()
  editor.showWindow(win.toolWinName)
  isRoadArchitectActive = true
end

-- Called when the 'Road Architect' is exited.
local function onDeactivate()

  -- First close down the profile list/groups list windows, if open.
  if mfe.isProfilesListWinOpen then
    mfe.isProfilesListWinOpen = false
    profileMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isGroupsListWinOpen then
    mfe.isGroupsListWinOpen = false
    groupMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isMeshSelectWinOpen then
    editor.hideWindow(win.meshSelectWinName)
    mfe.isMeshSelectWinOpen = false
    staticMgr.removeAuditionMesh()
    staticMgr.goToOldView()
  end

  editor.hideWindow(win.toolWinName)
  editor.hideWindow(win.materialSelectWinName)
  editor.hideWindow(win.meshSelectWinName)
  editor.hideWindow(win.nodeEditWinName)
  editor.hideWindow(win.profilesListWinName)
  editor.hideWindow(win.profileEditWinName)
  editor.hideWindow(win.groupsListWinName)
  editor.hideWindow(win.importWinName)

  isRoadArchitectActive = false
  mfe.isNodeEditWinOpen, mfe.isMaterialSelectWinOpen, mfe.isMeshSelectWinOpen, mfe.isImportWinOpen = false, false, false, false
  mfe.isProfilesListWinOpen, mfe.isProfileEditWinOpen, mfe.isGroupsListWinOpen = false, false, false
end

-- Validates the selection for the scenetree right click menu.
local function validateSceneTreeRightClickMenuSelection(node)
  if node then
    local object = scenetree.findObjectById(node.id)
    if object then
      return object:getClassName() == "DecalRoad"
    end
  end
end

-- Called on scenetree right click menu, if validated.
local function processSceneTreeRightClickMenuSelection(node)
  if node then
    roadMgr.convertDecalRoads2RoadArchitect()
  end
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  editor.editModes.roadArchitectEditMode = {
    displayName = "Road Architect",
    onUpdate = nop,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    icon = editor.icons.autobahn,
    iconTooltip = "Road Architect",
    auxShortcuts = {},
    hideObjectIcons = true }

  -- Set up the scenetree right click menu for importing.
  editor.addExtendedSceneTreeObjectMenuItem({
    title = "Convert to Road Architect",
    extendedSceneTreeObjectMenuItems = processSceneTreeRightClickMenuSelection,
    validator = validateSceneTreeRightClickMenuSelection })

  editor.registerWindow(win.toolWinName, win.toolWinSize)
  editor.registerWindow(win.materialSelectWinName, win.materialSelectWinSize)
  editor.registerWindow(win.meshSelectWinName, win.meshSelectWinSize)
  editor.registerWindow(win.nodeEditWinName, win.nodeEditWinSize)
  editor.registerWindow(win.profilesListWinName, win.profilesListWinSize)
  editor.registerWindow(win.profileEditWinName, win.profileEditWinSize)
  editor.registerWindow(win.groupsListWinName, win.groupsListWinSize)
  editor.registerWindow(win.importWinName, win.importWinSize)

  terrain = extensions.editor_terrainEditor.getTerrainBlock()                                       -- Get a reference to the terrain block, if it exists.

  -- If the user changes map, remove all the road and junction data from the structure.
  local latestMap = core_levels.getLevelName(getMissionFilename())
  local currentMap = roadMgr.getCurrentMap()
  if currentMap and currentMap ~= latestMap and #roadMgr.roads > 0 then
    roadMgr.clearBridges()
    roadMgr.removeAll()
    local tree = roadMgr.getTree()
    tree = nil
    table.clear(jctMgr.junctions)
    groupMgr.setPlacedGroups({})
  end
  roadMgr.clearBridges()
  roadMgr.setCurrentMap(latestMap)                                                                  -- Cache the currently-selected map (for later comparison on map change events).
  roadMgr.setAllDirty()
  roadMgr.updateRoads()
end

-- Serialization function.
local function onSerialize()

  -- First close down the profile list/groups list windows, if open.
  if mfe.isProfilesListWinOpen then
    mfe.isProfilesListWinOpen = false
    profileMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isGroupsListWinOpen then
    mfe.isGroupsListWinOpen = false
    groupMgr.goToOldView()
    roadMgr.removeHiddenRoads()
  end
  if mfe.isMeshSelectWinOpen then
    editor.hideWindow(win.meshSelectWinName)
    mfe.isMeshSelectWinOpen = false
    staticMgr.removeAuditionMesh()
    staticMgr.goToOldView()
  end

  -- Gather the data which requires serialised.
  local serRoads, serProfiles, serGroups, serJunctions = {}, {}, {}, {}
  local roads, profiles, groups, junctions = roadMgr.roads, profileMgr.profiles, groupMgr.groups, jctMgr.junctions
  local numRoads, numProfiles, numGroups, numJcts = #roads, #profiles, #groups, #junctions

  -- Serialise all the roads.
  for i = 1, numRoads do
    serRoads[i] = roadMgr.serialiseRoad(roads[i])
  end

  -- Serialise all the profiles.
  for i = 1, numProfiles do
    serProfiles[i] = profileMgr.serialiseProfile(profiles[i])
  end

  -- Serialise all the groups.
  for i = 1, numGroups do
    serGroups[i] = groupMgr.serialiseGroup(groups[i])
  end

  -- Serialise all the junctions.
  for i = 1, numJcts do
    serJunctions[i] = jctMgr.serialiseJct(junctions[i])
  end

  -- Remove all meshes and decals from scene.
  if isFinalise then
    roadMgr.unfinalise()
  end
  roadMgr.removeAll()

  -- Serialise the placed groups list.
  local serPlacedGroups = {}
  local placedGroups = groupMgr.getPlacedGroups() or {}
  for i = 1, #placedGroups do
    serPlacedGroups[i] = {
      name = ffi.string(placedGroups[i].name),
      list = {} }
    for j = 1, #placedGroups[i].list do
      serPlacedGroups[i].list[j] = placedGroups[i].list[j]
    end
  end

  -- Compress the data, ready for the serialisation process to commence.
  local encodedData =
    {
      data =
        {
          roads = serRoads,
          profiles = serProfiles,
          groups = serGroups,
          junctions = serJunctions,
          placedGroups = serPlacedGroups
        }
    }
  jsonWriteFile(tempFilepath, encodedData, true)
  return { d = { name = 'roadArchitectSerializationData'} }
end

-- Deserialization function.
local function onDeserialized()

  -- Collect the data which requires de-serialised.
  local loadedJson = jsonReadFile(tempFilepath).data
  local serRoads, serProfiles, serGroups, serPlacedGroups, serJunctions = loadedJson.roads, loadedJson.profiles, loadedJson.groups, loadedJson.placedGroups, loadedJson.junctions
  local numRoads, numProfiles, numGroups, numJcts = #serRoads, #serProfiles, #serGroups, #serJunctions

  -- De-serialise all the profiles.
  -- [Note: this must be done before the roads are deserialised].
  table.clear(profileMgr.profiles)
  for i = 1, numProfiles do
    profileMgr.profiles[i] = profileMgr.deserialiseProfile(serProfiles[i])
  end

  -- De-serialise all the roads.
  table.clear(roadMgr.roads)
  for i = 1, numRoads do
    roadMgr.roads[i] = roadMgr.deserialiseRoad(serRoads[i])
    roadMgr.setDirty(roadMgr.roads[i])
  end
  roadMgr.recomputeMap()

  -- De-serialise all the groups.
  table.clear(groupMgr.groups)
  for i = 1, numGroups do
    groupMgr.groups[i] = groupMgr.deserialiseGroup(serGroups[i])
  end

  -- De-serialise all the junctions.
  table.clear(jctMgr.junctions)
  for i = 1, numJcts do
    jctMgr.junctions[i] = jctMgr.deserialiseJct(serJunctions[i])
  end

  -- De-serialise the placed groups list.
  local placedGroups = {}
  for i = 1, #serPlacedGroups do
    placedGroups[i] = {
      name = im.ArrayChar(64, serPlacedGroups[i].name),
      list = {} }
    for j = 1, #serPlacedGroups[i].list do
      placedGroups[i].list[j] = serPlacedGroups[i].list[j]
    end
  end
  groupMgr.setPlacedGroups(placedGroups)

  -- Update the condition data for all roads.
  for i = 1, numRoads do
    profileMgr.updateCondition(roadMgr.roads[i])
    mfe.selectedLayerIdx = 1
  end

  -- Compute the render data for all roads.
  roadMgr.computeAllRoadRenderData()

  if not roadMgr.roads or #roadMgr.roads < 1 or not mfe.selectedRoadIdx then
    return
  end

  profileMgr.updateLaneFlags(roadMgr.roads[mfe.selectedRoadIdx].profile)
end


-- Public interface.
M.onEditorGui =                                           onEditorGui
M.onEditorInitialized =                                   onEditorInitialized

M.onSerialize =                                           onSerialize
M.onDeserialized =                                        onDeserialized

return M