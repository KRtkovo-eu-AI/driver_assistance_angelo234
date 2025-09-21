-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local toolPrefixStr = 'Stitch Spline' -- The global prefix for the stitch spline tool.

local zExtra = 3.0 -- Extra z-offset to add to the ribbon points when vertical raycasting.
local zRayOffset = 0.05 -- The z-offset to add to the ribbon points when raycasting to the surface below.
local maxRaycastDist = 10000 -- The maximum distance to raycast for ribbon points.
local minSplineDivisions = 100 -- The minimum number of subdivisions to use for a stitch spline.
local splitPartingDistance = 1.0 -- The distance by which to part a spline which is being split, at the split point.
local minImportSize = 10.0 -- The minimum size of a stitch spline to import.

local defaultPoleMeshPath = 'art/shapes/objects/electrical_pole_test.dae' -- TODO: this section needs updated when defaults are formalised.
local defaultPoleMeshName = 'electrical_pole_test'
local defaultWireMeshPath = 'art/shapes/objects/s_wire_test.dae'
local defaultWireMeshName = 's_wire_test'

-- Default slider parameters for UI.
local defaultParams = {
  spacing = 10.0,
  sag = 1.0,
  jitterForward = 0.0,
  jitterRight = 0.0,
  jitterUp = 0.0,
}

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'stitchSpline'

-- External modules.
local pop = require('editor/stitchSpline/populate')
local geom = require('editor/toolUtilities/geom')
local util = require('editor/toolUtilities/util')

-- Module constants.
local abs, min = math.abs, math.min
local down = vec3(0, 0, -1)

-- Module state.
local stitchSplines = {}
local splineMap = {}
local tmpPoint2I = Point2I(0, 0)
local tmp1 = vec3()


-- Fetches the tool prefix string.
local function getToolPrefixStr() return toolPrefixStr end

-- Fetches the stitch splines.
local function getStitchSplines() return stitchSplines end

-- Fetches the stitch spline map.
local function getSplineMap() return splineMap end

-- Sets the stitch spline with the given index.
local function setStitchSpline(spline, idx)
  stitchSplines[idx] = spline

  -- Ensure we have a unique stitch spline name.
  local baseName = spline.name
  local uniqueName = util.generateUniqueName(baseName, toolPrefixStr)

  -- Create a new scene tree folder for the stitch spline.
  if not spline.sceneTreeFolderId or not scenetree.findObjectById(spline.sceneTreeFolderId) then
    local newFolder = createObject("SimGroup")
    newFolder:registerObject(string.format("%s - %s", uniqueName, spline.id))
    newFolder.cansave = false
    scenetree.MissionGroup:addObject(newFolder)
    spline.sceneTreeFolderId = newFolder:getId()
  end
  spline.isDirty = true
end

-- Sets the stitch splines (full table).
local function setStitchSplines(splines)
  for i = 1, #splines do
    setStitchSpline(splines[i], i)
  end
end

-- Returns the default slider parameters.
local function getDefaultSliderParams() return defaultParams end

-- Adds a new stitch spline.
local function addNewStitchSpline()
  -- Ensure we have a unique stitch spline name.
  local baseName = string.format(toolPrefixStr .. " %d", #stitchSplines + 1)
  local uniqueName = util.generateUniqueName(baseName, toolPrefixStr)
  local id = Engine.generateUUID()

  -- Create a new scene tree folder for the stitch spline.
  local newFolder = createObject("SimGroup")
  newFolder:registerObject(string.format("%s - %s", uniqueName, id))
  newFolder.cansave = false
  scenetree.MissionGroup:addObject(newFolder)

  -- Get the extents data of the pole mesh.
  local tmpMesh = createObject('TSStatic')
  tmpMesh:setField('shapeName', 0, defaultPoleMeshPath)
  tmpMesh:registerObject('temp_pole')
  tmpMesh.canSave = false
  local box = tmpMesh:getObjBox()
  local worldBox = tmpMesh:getWorldBox()
  local center = tmpMesh:getPosition()
  local minExtents = worldBox.minExtents
  local maxExtents = worldBox.maxExtents
  local extents = box:getExtents()
  local pole_boxXLeft_Center, pole_boxXRight_Center = center.x - minExtents.x, maxExtents.x - center.x
  local pole_boxYLeft_Center, pole_boxYRight_Center = center.y - minExtents.y, maxExtents.y - center.y
  local pole_boxZLeft_Center, pole_boxZRight_Center = center.z - minExtents.z, maxExtents.z - center.z
  local pole_extentsL_Center, pole_extentsW_Center, pole_extentsZ_Center = extents.x, extents.y, extents.z
  if tmpMesh and simObjectExists(tmpMesh) then
    tmpMesh:delete()
  end

  -- Get the extents data of the wire mesh.
  local tmpMesh = createObject('TSStatic')
  tmpMesh:setField('shapeName', 0, defaultWireMeshPath)
  tmpMesh:registerObject('temp_wire')
  tmpMesh.canSave = false
  local box = tmpMesh:getObjBox()
  local worldBox = tmpMesh:getWorldBox()
  local center = tmpMesh:getPosition()
  local minExtents, maxExtents = worldBox.minExtents, worldBox.maxExtents
  local extents = box:getExtents()
  local wire_boxXLeft_Center, wire_boxXRight_Center = center.x - minExtents.x, maxExtents.x - center.x
  local wire_boxYLeft_Center, wire_boxYRight_Center = center.y - minExtents.y, maxExtents.y - center.y
  local wire_boxZLeft_Center, wire_boxZRight_Center = center.z - minExtents.z, maxExtents.z - center.z
  local wire_extentsL_Center, wire_extentsW_Center, wire_extentsZ_Center = extents.x, extents.y, extents.z
  if tmpMesh and simObjectExists(tmpMesh) then
    tmpMesh:delete()
  end

  -- Create a static mesh for the stitch spline.
  table.insert(stitchSplines, {

    -- The core properties of the stitch spline (for use with the center mesh components).
    name = uniqueName,
    id = id,
    sceneTreeFolderId = newFolder:getId(),
    isDirty = false,
    isEnabled = true,
    isLink = false,
    linkId = nil,
    nodes = {},
    widths = {},
    nmls = {},
    divPoints = {},
    divWidths = {},
    tangents = {},
    binormals = {},
    normals = {},
    discMap = {},
    spacing = defaultParams.spacing,
    isConformToTerrain = true,
    sag = defaultParams.sag,
    jitterForward = defaultParams.jitterForward,
    jitterRight = defaultParams.jitterRight,
    jitterUp = defaultParams.jitterUp,

    -- The properties of the pole mesh component.
    poleMeshPath = defaultPoleMeshPath,
    poleMeshName = defaultPoleMeshName,
    pole_extentsW_Center = pole_extentsW_Center,
    pole_extentsL_Center = pole_extentsL_Center,
    pole_extentsZ_Center = pole_extentsZ_Center,
    pole_boxXLeft_Center = pole_boxXLeft_Center,
    pole_boxXRight_Center = pole_boxXRight_Center,
    pole_boxYLeft_Center = pole_boxYLeft_Center,
    pole_boxYRight_Center = pole_boxYRight_Center,
    pole_boxZLeft_Center = pole_boxZLeft_Center,
    pole_boxZRight_Center = pole_boxZRight_Center,

    -- The properties of the wire mesh component.
    wireMeshPath = defaultWireMeshPath,
    wireMeshName = defaultWireMeshName,
    wire_extentsW_Center = wire_extentsW_Center,
    wire_extentsL_Center = wire_extentsL_Center,
    wire_extentsZ_Center = wire_extentsZ_Center,
    wire_boxXLeft_Center = wire_boxXLeft_Center,
    wire_boxXRight_Center = wire_boxXRight_Center,
    wire_boxYLeft_Center = wire_boxYLeft_Center,
    wire_boxYRight_Center = wire_boxYRight_Center,
    wire_boxZLeft_Center = wire_boxZLeft_Center,
    wire_boxZRight_Center = wire_boxZRight_Center,
  })

  -- Update the spline map.
  util.computeIdToIdxMap(stitchSplines, splineMap)
end

-- Removes the stitch spline with the given index.
local function removeStitchSpline(idx)
  local spline = stitchSplines[idx]
  pop.tryRemove(spline) -- Remove the static meshes related to this stitch spline from the scene.
  if spline.sceneTreeFolderId then
    local folder = scenetree.findObjectById(spline.sceneTreeFolderId)
    if folder and simObjectExists(folder) then
      folder:delete() -- Remove the scene tree folder for the stitch spline.
    end
  end
  table.remove(stitchSplines, idx) -- Finally, remove the stitch spline from the list.

  -- Update the spline map.
  util.computeIdToIdxMap(stitchSplines, splineMap)
end

-- Removes all stitch splines.
-- [If isIncludeDisabled is true, then all stitch splines will be removed, including disabled ones.]
-- [If isIncludeDisabled is false, then only enabled stitch splines will be removed.]
local function removeAllStitchSplines(isIncludeDisabled)
  if isIncludeDisabled then
    for i = #stitchSplines, 1, -1 do
      local spline = stitchSplines[i]
      if spline then
        removeStitchSpline(i)
      end
    end
  else
    for i = #stitchSplines, 1, -1 do
      local spline = stitchSplines[i]
      if spline and spline.isEnabled and not spline.isLink then -- Only remove enabled and unlinked stitch splines.
        removeStitchSpline(i)
      end
    end
  end
end

-- Deep copies the given stitch spline.
local function deepCopyStitchSpline(spline)
  local copy = {}

  -- Deep copy the properties.
  copy.name = spline.name -- Keep the same name.
  copy.id = spline.id -- Keep the same id.
  copy.sceneTreeFolderId = spline.sceneTreeFolderId -- This will be set on update, if it no longer exists.
  copy.isDirty = true -- Set dirty so geometry is updated.
  copy.isLink = spline.isLink -- Keep the same link status.
  copy.linkId = spline.linkId
  copy.isEnabled = spline.isEnabled -- Keep the same enabled status.
  copy.spacing = spline.spacing
  copy.isConformToTerrain = spline.isConformToTerrain
  copy.sag = spline.sag
  copy.jitterForward = spline.jitterForward
  copy.jitterRight = spline.jitterRight
  copy.jitterUp = spline.jitterUp
  copy.poleMeshPath = spline.poleMeshPath
  copy.poleMeshName = spline.poleMeshName
  copy.pole_extentsW_Center = spline.pole_extentsW_Center
  copy.pole_extentsL_Center = spline.pole_extentsL_Center
  copy.pole_extentsZ_Center = spline.pole_extentsZ_Center
  copy.pole_boxXLeft_Center = spline.pole_boxXLeft_Center
  copy.pole_boxXRight_Center = spline.pole_boxXRight_Center
  copy.pole_boxYLeft_Center = spline.pole_boxYLeft_Center
  copy.pole_boxYRight_Center = spline.pole_boxYRight_Center
  copy.pole_boxZLeft_Center = spline.pole_boxZLeft_Center
  copy.pole_boxZRight_Center = spline.pole_boxZRight_Center
  copy.wireMeshPath = spline.wireMeshPath
  copy.wireMeshName = spline.wireMeshName
  copy.wire_extentsW_Center = spline.wire_extentsW_Center
  copy.wire_extentsL_Center = spline.wire_extentsL_Center
  copy.wire_extentsZ_Center = spline.wire_extentsZ_Center
  copy.wire_boxXLeft_Center = spline.wire_boxXLeft_Center
  copy.wire_boxXRight_Center = spline.wire_boxXRight_Center
  copy.wire_boxYLeft_Center = spline.wire_boxYLeft_Center
  copy.wire_boxYRight_Center = spline.wire_boxYRight_Center
  copy.wire_boxZLeft_Center = spline.wire_boxZLeft_Center
  copy.wire_boxZRight_Center = spline.wire_boxZRight_Center

  -- Deep copy the nodes.
  local numNodes = #(spline.nodes or {})
  local copyNodes = table.new(numNodes, 0)
  local splineNodes = spline.nodes
  for i = 1, numNodes do
    copyNodes[i] = vec3(splineNodes[i])
  end
  copy.nodes = copyNodes

  -- Deep copy the widths.
  copy.widths = deepcopy(spline.widths)

  -- Deep copy the nmls.
  local numNmls = #(spline.nmls or {})
  local copyNmls = table.new(numNmls, 0)
  local splineNmls = spline.nmls
  for i = 1, numNmls do
    copyNmls[i] = vec3(splineNmls[i])
  end
  copy.nmls = copyNmls

  -- Deep copy the div points.
  local numDivPoints = #(spline.divPoints or {})
  local copyDivPoints = table.new(numDivPoints, 0)
  local splineDivPoints = spline.divPoints
  for i = 1, numDivPoints do
    copyDivPoints[i] = vec3(splineDivPoints[i])
  end
  copy.divPoints = copyDivPoints
  copy.divWidths = deepcopy(spline.divWidths)

  -- Deep copy the tangents.
  local copyTangents = table.new(numDivPoints, 0)
  local splineTangents = spline.tangents
  for i = 1, numDivPoints do
    copyTangents[i] = vec3(splineTangents[i])
  end
  copy.tangents = copyTangents
  local copyBinormals = table.new(numDivPoints, 0)
  local splineBinormals = spline.binormals
  for i = 1, numDivPoints do
    copyBinormals[i] = vec3(splineBinormals[i])
  end
  copy.binormals = copyBinormals

  -- Deep copy the normals.
  local copyNormals = table.new(numDivPoints, 0)
  local splineNormals = spline.normals
  for i = 1, numDivPoints do
    copyNormals[i] = vec3(splineNormals[i])
  end
  copy.normals = copyNormals

  -- Deep copy the map from node to div point.
  copy.discMap = deepcopy(spline.discMap)

  return copy
end

-- Deep copies the full stitch spline state.
local function deepCopyStitchSplineState()
  local numSplines = #stitchSplines
  local copy = table.new(numSplines, 0)
  for i = 1, numSplines do
    copy[i] = deepCopyStitchSpline(stitchSplines[i])
  end
  return copy
end

-- Splits the selected stitch spline into two, at the selected node.
local function splitStitchSpline(selectedSplineIdx, selectedNodeIdx)
  local spline = stitchSplines[selectedSplineIdx]
  if spline then
    local copy = deepCopyStitchSpline(spline) -- First, deep copy the nodes of the stitch spline to be split.
    removeStitchSpline(selectedSplineIdx) -- Remove the stitch spline from the session.

    -- Add the two new stitch splines to the session.
    addNewStitchSpline()
    stitchSplines[#stitchSplines].name = copy.name .. '_split_A'
    stitchSplines[#stitchSplines].isDirty = true
    stitchSplines[#stitchSplines].isLink = false
    stitchSplines[#stitchSplines].linkId = nil
    stitchSplines[#stitchSplines].isEnabled = true
    stitchSplines[#stitchSplines].spacing = copy.spacing
    stitchSplines[#stitchSplines].sag = copy.sag
    stitchSplines[#stitchSplines].isConformToTerrain = copy.isConformToTerrain
    stitchSplines[#stitchSplines].jitterForward = copy.jitterForward
    stitchSplines[#stitchSplines].jitterRight = copy.jitterRight
    stitchSplines[#stitchSplines].jitterUp = copy.jitterUp
    stitchSplines[#stitchSplines].poleMeshPath = copy.poleMeshPath
    stitchSplines[#stitchSplines].poleMeshName = copy.poleMeshName
    stitchSplines[#stitchSplines].pole_extentsW_Center = copy.pole_extentsW_Center
    stitchSplines[#stitchSplines].pole_extentsL_Center = copy.pole_extentsL_Center
    stitchSplines[#stitchSplines].pole_extentsZ_Center = copy.pole_extentsZ_Center
    stitchSplines[#stitchSplines].pole_boxXLeft_Center = copy.pole_boxXLeft_Center
    stitchSplines[#stitchSplines].pole_boxXRight_Center = copy.pole_boxXRight_Center
    stitchSplines[#stitchSplines].pole_boxYLeft_Center = copy.pole_boxYLeft_Center
    stitchSplines[#stitchSplines].pole_boxYRight_Center = copy.pole_boxYRight_Center
    stitchSplines[#stitchSplines].pole_boxZLeft_Center = copy.pole_boxZLeft_Center
    stitchSplines[#stitchSplines].pole_boxZRight_Center = copy.pole_boxZRight_Center
    stitchSplines[#stitchSplines].wireMeshPath = copy.wireMeshPath
    stitchSplines[#stitchSplines].wireMeshName = copy.wireMeshName
    stitchSplines[#stitchSplines].wire_extentsW_Center = copy.wire_extentsW_Center
    stitchSplines[#stitchSplines].wire_extentsL_Center = copy.wire_extentsL_Center
    stitchSplines[#stitchSplines].wire_extentsZ_Center = copy.wire_extentsZ_Center
    stitchSplines[#stitchSplines].wire_boxXLeft_Center = copy.wire_boxXLeft_Center
    stitchSplines[#stitchSplines].wire_boxXRight_Center = copy.wire_boxXRight_Center
    stitchSplines[#stitchSplines].wire_boxYLeft_Center = copy.wire_boxYLeft_Center
    stitchSplines[#stitchSplines].wire_boxYRight_Center = copy.wire_boxYRight_Center
    stitchSplines[#stitchSplines].wire_boxZLeft_Center = copy.wire_boxZLeft_Center
    stitchSplines[#stitchSplines].wire_boxZRight_Center = copy.wire_boxZRight_Center

    -- Include the nodes and widths from the original stitch spline, up to the selected node.
    for i = 1, selectedNodeIdx do
      table.insert(stitchSplines[#stitchSplines].nodes, copy.nodes[i])
      table.insert(stitchSplines[#stitchSplines].widths, copy.widths[i])
      table.insert(stitchSplines[#stitchSplines].nmls, copy.nmls[i])
    end

    -- Push apart the split point, for visual clarity.
    local nds = stitchSplines[#stitchSplines].nodes
    local tangent = nds[#nds] - nds[#nds - 1]
    tangent:normalize()
    nds[#nds] = nds[#nds] - (tangent * splitPartingDistance)

    -- Add the second new stitch spline to the session.
    addNewStitchSpline()
    stitchSplines[#stitchSplines].name = copy.name .. '_split_B'
    stitchSplines[#stitchSplines].isDirty = true
    stitchSplines[#stitchSplines].isLink = false
    stitchSplines[#stitchSplines].linkId = nil
    stitchSplines[#stitchSplines].isEnabled = true
    stitchSplines[#stitchSplines].spacing = copy.spacing
    stitchSplines[#stitchSplines].sag = copy.sag
    stitchSplines[#stitchSplines].isConformToTerrain = copy.isConformToTerrain
    stitchSplines[#stitchSplines].jitterForward = copy.jitterForward
    stitchSplines[#stitchSplines].jitterRight = copy.jitterRight
    stitchSplines[#stitchSplines].jitterUp = copy.jitterUp
    stitchSplines[#stitchSplines].poleMeshPath = copy.poleMeshPath
    stitchSplines[#stitchSplines].poleMeshName = copy.poleMeshName
    stitchSplines[#stitchSplines].pole_extentsW_Center = copy.pole_extentsW_Center
    stitchSplines[#stitchSplines].pole_extentsL_Center = copy.pole_extentsL_Center
    stitchSplines[#stitchSplines].pole_extentsZ_Center = copy.pole_extentsZ_Center
    stitchSplines[#stitchSplines].pole_boxXLeft_Center = copy.pole_boxXLeft_Center
    stitchSplines[#stitchSplines].pole_boxXRight_Center = copy.pole_boxXRight_Center
    stitchSplines[#stitchSplines].pole_boxYLeft_Center = copy.pole_boxYLeft_Center
    stitchSplines[#stitchSplines].pole_boxYRight_Center = copy.pole_boxYRight_Center
    stitchSplines[#stitchSplines].pole_boxZLeft_Center = copy.pole_boxZLeft_Center
    stitchSplines[#stitchSplines].pole_boxZRight_Center = copy.pole_boxZRight_Center
    stitchSplines[#stitchSplines].wireMeshPath = copy.wireMeshPath
    stitchSplines[#stitchSplines].wireMeshName = copy.wireMeshName
    stitchSplines[#stitchSplines].wire_extentsW_Center = copy.wire_extentsW_Center
    stitchSplines[#stitchSplines].wire_extentsL_Center = copy.wire_extentsL_Center
    stitchSplines[#stitchSplines].wire_extentsZ_Center = copy.wire_extentsZ_Center
    stitchSplines[#stitchSplines].wire_boxXLeft_Center = copy.wire_boxXLeft_Center
    stitchSplines[#stitchSplines].wire_boxXRight_Center = copy.wire_boxXRight_Center
    stitchSplines[#stitchSplines].wire_boxYLeft_Center = copy.wire_boxYLeft_Center
    stitchSplines[#stitchSplines].wire_boxYRight_Center = copy.wire_boxYRight_Center
    stitchSplines[#stitchSplines].wire_boxZLeft_Center = copy.wire_boxZLeft_Center
    stitchSplines[#stitchSplines].wire_boxZRight_Center = copy.wire_boxZRight_Center

    -- Include the nodes and widths from the original stitch spline, from the selected node onwards.
    for i = selectedNodeIdx, #copy.nodes do
      table.insert(stitchSplines[#stitchSplines].nodes, copy.nodes[i])
      table.insert(stitchSplines[#stitchSplines].widths, copy.widths[i])
      table.insert(stitchSplines[#stitchSplines].nmls, copy.nmls[i])
    end

    -- Push apart the split point, for visual clarity.
    local nds = stitchSplines[#stitchSplines].nodes
    local tangent = nds[1] - nds[2]
    tangent:normalize()
    nds[1] = nds[1] - (tangent * splitPartingDistance)

    -- Update the spline map.
    util.computeIdToIdxMap(stitchSplines, splineMap)
  end
end

-- Undo/redo core function.
local function singleSplineEditUndoRedoCore(data)
  local idx = splineMap[data.id]
  if idx then
    local spline = stitchSplines[idx]
    spline.name = data.name
    spline.isDirty = true -- Set the dirty flag to true.
    spline.isLink = data.isLink
    spline.linkId = data.linkId
    spline.nodes = data.nodes
    spline.widths = data.widths
    spline.nmls = data.nmls
    spline.divPoints = data.divPoints
    spline.divWidths = data.divWidths
    spline.tangents = data.tangents
    spline.binormals = data.binormals
    spline.normals = data.normals
    spline.isEnabled = data.isEnabled
    spline.spacing = data.spacing
    spline.sag = data.sag
    spline.isConformToTerrain = data.isConformToTerrain
    spline.jitterForward = data.jitterForward
    spline.jitterRight = data.jitterRight
    spline.jitterUp = data.jitterUp
    spline.poleMeshPath = data.poleMeshPath
    spline.poleMeshName = data.poleMeshName
    spline.pole_extentsW_Center = data.pole_extentsW_Center
    spline.pole_extentsL_Center = data.pole_extentsL_Center
    spline.pole_extentsZ_Center = data.pole_extentsZ_Center
    spline.pole_boxXLeft_Center = data.pole_boxXLeft_Center
    spline.pole_boxXRight_Center = data.pole_boxXRight_Center
    spline.pole_boxYLeft_Center = data.pole_boxYLeft_Center
    spline.pole_boxYRight_Center = data.pole_boxYRight_Center
    spline.pole_boxZLeft_Center = data.pole_boxZLeft_Center
    spline.pole_boxZRight_Center = data.pole_boxZRight_Center
    spline.wireMeshPath = data.wireMeshPath
    spline.wireMeshName = data.wireMeshName
    spline.wire_extentsW_Center = data.wire_extentsW_Center
    spline.wire_extentsL_Center = data.wire_extentsL_Center
    spline.wire_extentsZ_Center = data.wire_extentsZ_Center
    spline.wire_boxXLeft_Center = data.wire_boxXLeft_Center
    spline.wire_boxXRight_Center = data.wire_boxXRight_Center
    spline.wire_boxYLeft_Center = data.wire_boxYLeft_Center
    spline.wire_boxYRight_Center = data.wire_boxYRight_Center
    spline.wire_boxZLeft_Center = data.wire_boxZLeft_Center
    spline.wire_boxZRight_Center = data.wire_boxZRight_Center

    -- Special case for when the stitch spline has been renamed.
    if data.isUpdateSceneTree then
      local folder = scenetree.findObjectById(spline.sceneTreeFolderId)
      if folder then
        folder:setName(spline.name)
      end
      editor.refreshSceneTreeWindow()
    end

    -- Remove all the static meshes for the stitch spline.
    pop.tryRemove(spline)
  end
end

-- Handles the undo for a single stitch spline.
local function singleSplineEditUndo(stitchSplineData)
  local data = stitchSplineData.old
  if data then
    singleSplineEditUndoRedoCore(data)
  end
end

-- Handles the redo for a single stitch spline.
local function singleSplineEditRedo(stitchSplineData)
  local data = stitchSplineData.new
  if data then
    singleSplineEditUndoRedoCore(data)
  end
end

-- Handles the undo for trans stitch spline edits.
local function transSplineEditUndo(data)
  removeAllStitchSplines(true)
  setStitchSplines(data.old)
  for i = 1, #stitchSplines do
    stitchSplines[i].isDirty = true
  end

  -- Update the spline map.
  util.computeIdToIdxMap(stitchSplines, splineMap)
end

-- Handles the redo for trans stitch spline edits.
local function transSplineEditRedo(data)
  removeAllStitchSplines(true)
  setStitchSplines(data.new)
  for i = 1, #stitchSplines do
    stitchSplines[i].isDirty = true
  end

  -- Update the spline map.
  util.computeIdToIdxMap(stitchSplines, splineMap)
end

-- Copies the profile of the given stitch spline.
local function copyStitchSplineProfile(spline)
  local copy = deepCopyStitchSpline(spline)
  copy.nodes = nil
  copy.widths = nil
  copy.nmls = nil
  copy.divPoints = nil
  copy.divWidths = nil
  copy.tangents = nil
  copy.binormals = nil
  copy.normals = nil
  copy.discMap = nil
  return copy
end

-- Pastes the profile to the given stitch spline.
local function pasteStitchSplineProfile(spline, profile)
  spline.isDirty = true
  spline.spacing = profile.spacing
  spline.sag = profile.sag
  spline.isConformToTerrain = profile.isConformToTerrain
  spline.jitterForward = profile.jitterForward
  spline.jitterRight = profile.jitterRight
  spline.jitterUp = profile.jitterUp
  spline.poleMeshPath = profile.poleMeshPath
  spline.poleMeshName = profile.poleMeshName
  spline.pole_extentsW_Center = profile.pole_extentsW_Center
  spline.pole_extentsL_Center = profile.pole_extentsL_Center
  spline.pole_extentsZ_Center = profile.pole_extentsZ_Center
  spline.pole_boxXLeft_Center = profile.pole_boxXLeft_Center
  spline.pole_boxXRight_Center = profile.pole_boxXRight_Center
  spline.pole_boxYLeft_Center = profile.pole_boxYLeft_Center
  spline.pole_boxYRight_Center = profile.pole_boxYRight_Center
  spline.pole_boxZLeft_Center = profile.pole_boxZLeft_Center
  spline.pole_boxZRight_Center = profile.pole_boxZRight_Center
  spline.wireMeshPath = profile.wireMeshPath
  spline.wireMeshName = profile.wireMeshName
  spline.wire_extentsW_Center = profile.wire_extentsW_Center
  spline.wire_extentsL_Center = profile.wire_extentsL_Center
  spline.wire_extentsZ_Center = profile.wire_extentsZ_Center
  spline.wire_boxXLeft_Center = profile.wire_boxXLeft_Center
  spline.wire_boxXRight_Center = profile.wire_boxXRight_Center
  spline.wire_boxYLeft_Center = profile.wire_boxYLeft_Center
  spline.wire_boxYRight_Center = profile.wire_boxYRight_Center
  spline.wire_boxZLeft_Center = profile.wire_boxZLeft_Center
  spline.wire_boxZRight_Center = profile.wire_boxZRight_Center
end

-- Serializes a stitch spline to a table.
local function serializeStitchSpline(spline)
  local data = {}
  data.name = spline.name
  data.id = spline.id
  data.sceneTreeFolderId = spline.sceneTreeFolderId -- This will be set on update, if it no longer exists.
  data.isLink = spline.isLink -- Keep the same link status.
  data.linkId = spline.linkId
  data.isEnabled = spline.isEnabled -- Keep the same enabled status.
  data.spacing = spline.spacing
  data.sag = spline.sag
  data.isConformToTerrain = spline.isConformToTerrain
  data.jitterForward = spline.jitterForward
  data.jitterRight = spline.jitterRight
  data.jitterUp = spline.jitterUp
  data.poleMeshPath = spline.poleMeshPath
  data.poleMeshName = spline.poleMeshName
  data.pole_extentsW_Center = spline.pole_extentsW_Center
  data.pole_extentsL_Center = spline.pole_extentsL_Center
  data.pole_extentsZ_Center = spline.pole_extentsZ_Center
  data.pole_boxXLeft_Center = spline.pole_boxXLeft_Center
  data.pole_boxXRight_Center = spline.pole_boxXRight_Center
  data.pole_boxYLeft_Center = spline.pole_boxYLeft_Center
  data.pole_boxYRight_Center = spline.pole_boxYRight_Center
  data.pole_boxZLeft_Center = spline.pole_boxZLeft_Center
  data.pole_boxZRight_Center = spline.pole_boxZRight_Center
  data.wireMeshPath = spline.wireMeshPath
  data.wireMeshName = spline.wireMeshName
  data.wire_extentsW_Center = spline.wire_extentsW_Center
  data.wire_extentsL_Center = spline.wire_extentsL_Center
  data.wire_extentsZ_Center = spline.wire_extentsZ_Center
  data.wire_boxXLeft_Center = spline.wire_boxXLeft_Center
  data.wire_boxXRight_Center = spline.wire_boxXRight_Center
  data.wire_boxYLeft_Center = spline.wire_boxYLeft_Center
  data.wire_boxYRight_Center = spline.wire_boxYRight_Center
  data.wire_boxZLeft_Center = spline.wire_boxZLeft_Center
  data.wire_boxZRight_Center = spline.wire_boxZRight_Center

  -- Serialise the nodes data.
  local numNodes = #spline.nodes
  local copyNodes = table.new(numNodes, 0)
  local splineNodes = spline.nodes
  for i = 1, numNodes do
    local n = splineNodes[i]
    copyNodes[i] = { x = n.x, y = n.y, z = n.z }
  end
  data.nodes = copyNodes

  -- Serialise the widths data.
  data.widths = spline.widths

  -- Serialise the nmls data.
  local numNmls = #(spline.nmls or {})
  local copyNmls = table.new(numNmls, 0)
  local splineNmls = spline.nmls
  for i = 1, numNmls do
    local n = splineNmls[i]
    copyNmls[i] = { x = n.x, y = n.y, z = n.z }
  end
  data.nmls = copyNmls

  return data
end

-- Deserializes a stitch spline from a table.
local function deserializeStitchSpline(data, isCreateObject)
  local spline = {}
  spline.name = data.name
  spline.id = data.id
  spline.sceneTreeFolderId = data.sceneTreeFolderId -- This will be set on update, if it no longer exists.
  spline.isDirty = true -- Set dirty so geometry is updated.
  spline.isLink = (data.isLink == true or data.isLink == 1) and true or false
  spline.linkId = data.linkId
  spline.isEnabled = (data.isEnabled == true or data.isEnabled == 1) and true or false
  spline.spacing = data.spacing
  spline.sag = data.sag
  spline.isConformToTerrain = (data.isConformToTerrain == true or data.isConformToTerrain == 1) and true or false
  spline.jitterForward = data.jitterForward
  spline.jitterRight = data.jitterRight
  spline.jitterUp = data.jitterUp
  spline.poleMeshPath = data.poleMeshPath
  spline.poleMeshName = data.poleMeshName
  spline.pole_extentsW_Center = data.pole_extentsW_Center
  spline.pole_extentsL_Center = data.pole_extentsL_Center
  spline.pole_extentsZ_Center = data.pole_extentsZ_Center
  spline.pole_boxXLeft_Center = data.pole_boxXLeft_Center
  spline.pole_boxXRight_Center = data.pole_boxXRight_Center
  spline.pole_boxYLeft_Center = data.pole_boxYLeft_Center
  spline.pole_boxYRight_Center = data.pole_boxYRight_Center
  spline.pole_boxZLeft_Center = data.pole_boxZLeft_Center
  spline.pole_boxZRight_Center = data.pole_boxZRight_Center
  spline.wireMeshPath = data.wireMeshPath
  spline.wireMeshName = data.wireMeshName
  spline.wire_extentsW_Center = data.wire_extentsW_Center
  spline.wire_extentsL_Center = data.wire_extentsL_Center
  spline.wire_extentsZ_Center = data.wire_extentsZ_Center
  spline.wire_boxXLeft_Center = data.wire_boxXLeft_Center
  spline.wire_boxXRight_Center = data.wire_boxXRight_Center
  spline.wire_boxYLeft_Center = data.wire_boxYLeft_Center
  spline.wire_boxYRight_Center = data.wire_boxYRight_Center
  spline.wire_boxZLeft_Center = data.wire_boxZLeft_Center
  spline.wire_boxZRight_Center = data.wire_boxZRight_Center

  -- Deserialise the nodes data.
  local splineNodes = data.nodes or {}
  local numNodes = #splineNodes
  local copyNodes = table.new(numNodes, 0)
  for i = 1, numNodes do
    local n = splineNodes[i]
    copyNodes[i] = vec3(n.x, n.y, n.z)
  end
  spline.nodes = copyNodes

  -- Deserialise the widths data.
  spline.widths = data.widths or {}

  -- Deserialise the nmls data.
  local nmls = data.nmls or {}
  local numNmls = #nmls
  local copyNmls = table.new(numNmls, 0)
  for i = 1, numNmls do
    local n = nmls[i]
    copyNmls[i] = vec3(n.x, n.y, n.z)
  end
  spline.nmls = copyNmls

  -- Allocate secondary geometry data.
  spline.divPoints = {}
  spline.divWidths = {}
  spline.tangents = {}
  spline.binormals = {}
  spline.normals = {}
  spline.discMap = {}

  -- Create a new scene tree folder for the stitch spline, if requested.
  if isCreateObject then
    -- Ensure we have a unique stitch spline name.
    local baseName = string.format(toolPrefixStr .. " %d", #stitchSplines + 1)
    local uniqueName = util.generateUniqueName(baseName, toolPrefixStr)

    -- Create a new scene tree folder for the stitch spline.
    local newFolder = createObject("SimGroup")
    newFolder:registerObject(string.format("%s - %s", uniqueName, spline.id))
    newFolder.cansave = false
    scenetree.MissionGroup:addObject(newFolder)
    spline.sceneTreeFolderId = newFolder:getId()
  end
  return spline
end

-- Updates the geometry and meshes of any dirty stitch splines.
local function updateDirtyStitchSplines()
  for i = 1, #stitchSplines do
    local spline = stitchSplines[i]
    if spline.isDirty then
      if not spline.isLink then -- Linked splines are updated via the Road Group Editor - not here.
        if spline.isConformToTerrain then
          local nodes = spline.nodes
          for j = 1, #nodes do
            local node = nodes[j]
            tmp1:set(node.x, node.y, node.z + zExtra)
            local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
            node.z = tmp1.z - d + zRayOffset
          end
          geom.catmullRomRaycast(spline, minSplineDivisions) -- Ensure the secondary geometry is updated and conforms to the terrain.
        else
          geom.catmullRomFree(spline, minSplineDivisions) -- Ensure the secondary geometry is updated, but remains free in 3D (not conformed to the terrain).
        end
      end
      pop.populateStitchSpline(spline, i)
      spline.isDirty = false
    end
  end
end

-- Converts the given paths (traced from a bitmap) to stitch splines.
local function convertPathsToStitchSplines(paths)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  for i = 1, #paths do
    local path = paths[i]

    -- Create a new stitch spline.
    addNewStitchSpline()
    local spline = stitchSplines[#stitchSplines]

    -- Convert the grid points to world space.
    local points = path.points
    local numPoints = #points
    local pointsWS = table.new(numPoints, 0)
    for j = 1, numPoints do
      local pointGrid = points[j]
      tmpPoint2I.x, tmpPoint2I.y = pointGrid.x, pointGrid.y
      pointsWS[j] = te:gridToWorldByPoint2I(tmpPoint2I, tb)
    end

    -- If the path is sufficiently large, then import it.
    local aabb = geom.getAABB(pointsWS)
    if min(abs(aabb.xMax - aabb.xMin), abs(aabb.yMax - aabb.yMin)) > minImportSize then
      spline.nodes, spline.widths = pointsWS, path.widths -- Set the nodes and widths directly.
      for j = 1, numPoints do
        spline.nmls[j] = geom.getTerrainNormal(pointsWS[j]) -- Compute the normals for each node.
      end
    end
  end
  log('I', logTag, string.format("Converted %d traced paths to stitch splines. %d paths were too small to import.", #paths, #paths - #stitchSplines))
end

-- Returns the current list of stitch splines.
local function getCurrentStitchSplineList()
  local numSplines = #stitchSplines
  local list, ctr = table.new(numSplines, 0), 1
  for i = 1, numSplines do
    local spline = stitchSplines[i]
    if spline.isEnabled then
      list[ctr] = { name = spline.name, id = spline.id, type = toolPrefixStr }
      ctr = ctr + 1
    end
  end
  return list
end

-- Returns true if the given stitch spline is linked, false otherwise.
local function isLinked(id)
  local spline = stitchSplines[splineMap[id]]
  return spline and spline.isLink or false
end

-- Sets the given linked stitch spline to be linked or not.
local function setLink(id, groupId, isLink)
  local spline = stitchSplines[splineMap[id]]
  if spline then
    spline.isLink = isLink
    if isLink then
      spline.linkId = groupId
    else
      spline.linkId = nil
    end
    spline.isDirty = true
  end
end

-- Updates the given linked stitch spline with the given geometry.
local function updateLinkedStitchSpline(id, points, widths, tangents, binormals, normals)
  local spline = stitchSplines[splineMap[id]]
  if spline then
    spline.divPoints, spline.divWidths, spline.tangents, spline.binormals, spline.normals = points, widths, tangents, binormals, normals
    spline.isDirty = true
  end
end

-- Unlinks all stitch splines.
local function unlinkAll()
  for i = 1, #stitchSplines do
    local spline = stitchSplines[i]
    spline.isLink = false
    spline.linkId = nil
    spline.isDirty = true
  end
end

-- Burns the given linked stitch spline to the scene.
-- [This will remove the stitch spline from the tool, but will leave its folder and static meshes in the scene, ready for level save events.]
local function burnToScene(id)
  local spline = stitchSplines[splineMap[id]]
  if not spline then
    return
  end

  -- Set the cansave flag for the stitch spline folder.
  if spline.sceneTreeFolderId then
    local folder = scenetree.findObjectById(spline.sceneTreeFolderId)
    if folder then
      folder.cansave = true
    end
  end

  -- Set the cansave flag for all the static meshes of the stitch spline.
  pop.setCansave(spline)

  -- Remove the stitch spline from the tool, without removing its static meshes.
  table.remove(stitchSplines, splineMap[id])
end


-- Public interface.
M.getToolPrefixStr =                                    getToolPrefixStr
M.getStitchSplines =                                    getStitchSplines
M.getSplineMap =                                        getSplineMap
M.setStitchSplines =                                    setStitchSplines
M.setStitchSpline =                                     setStitchSpline
M.getDefaultSliderParams =                              getDefaultSliderParams

M.addNewStitchSpline =                                  addNewStitchSpline
M.splitStitchSpline =                                   splitStitchSpline
M.removeStitchSpline =                                  removeStitchSpline
M.removeAllStitchSplines =                              removeAllStitchSplines

M.deepCopyStitchSpline =                                deepCopyStitchSpline
M.deepCopyStitchSplineState =                           deepCopyStitchSplineState
M.copyStitchSplineProfile =                             copyStitchSplineProfile
M.pasteStitchSplineProfile =                            pasteStitchSplineProfile

M.singleSplineEditUndo =                                singleSplineEditUndo
M.singleSplineEditRedo =                                singleSplineEditRedo
M.transSplineEditUndo =                                 transSplineEditUndo
M.transSplineEditRedo =                                 transSplineEditRedo

M.updateDirtyStitchSplines =                            updateDirtyStitchSplines

M.convertPathsToStitchSplines =                         convertPathsToStitchSplines

M.serializeStitchSpline =                               serializeStitchSpline
M.deserializeStitchSpline =                             deserializeStitchSpline
M.deepCopyStitchSplineState =                           deepCopyStitchSplineState

M.getCurrentStitchSplineList =                          getCurrentStitchSplineList
M.isLinked =                                            isLinked
M.setLink =                                             setLink
M.updateLinkedStitchSpline =                            updateLinkedStitchSpline
M.unlinkAll =                                           unlinkAll

M.burnToScene =                                         burnToScene

return M