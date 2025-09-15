-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for using the gizmo with various spline-editing tools.

local M = {}

-- Module constants.
local min, max = math.min, math.max
local zeroQuatF = QuatF(0, 0, 0, 1)

-- Module state.
local splines = nil -- The array of splines to control.
local deepCopyFunct = nil -- The cached function which deep copies the state of a spline.
local undoFunct, redoFunct = nil, nil -- The cached undo and redo callback functions.
local gizmoDragPre = nil -- State to store the spline before the gizmo drag begins.
local splineIdx, nodeIdx = nil, nil -- The cached index of the selected spline and node.
local isRotEnabled = false -- A flag which indicates if the rotation gizmo is enabled.
local isConformToTerrain = false -- A flag which indicates if the controlled spline should conform to the terrain, or not.
local isRigid = false -- A flag which indicates if the controlled spline should have rigid translation, or not
local beginDragRot = nil
local delta, tmpTan = vec3(), vec3()
local tmpQuatF = QuatF(0, 0, 0, 1)


-- The callback function for begin axis gizmo dragging.
local function gizmoBeginDrag()
  local spline = splines[splineIdx]
  gizmoDragPre = deepCopyFunct(spline) -- Store the spline state upon beginning the drag.

  if isRotEnabled then
    local nodes = spline.nodes
    local p1, p2 = nodes[max(1, nodeIdx - 1)], nodes[min(#nodes, nodeIdx + 1)]
    tmpTan:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
    tmpTan:normalize()
    beginDragRot = quatFromDir(tmpTan, spline.nmls[nodeIdx])
  end
end

-- The callback function for end axis gizmo dragging.
local function gizmoEndDrag()
  editor.history:commitAction("Gizmo Drag", { old = gizmoDragPre, new = deepCopyFunct(splines[splineIdx]) }, undoFunct, redoFunct)
  beginDragRot = nil
end

-- The callback function for handling dragging of the gizmo.
local function gizmoDragging()
  -- Handle the gizmo for translation.
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate and splineIdx > 0 and splineIdx <= #splines then
    local spline = splines[splineIdx]
    local nodes = spline.nodes
    local selNode = nodes[nodeIdx]
    local gizmoPos = editor.getAxisGizmoTransform():getColumn(3) -- Update the node position to where the gizmo is.
    if isRigid then
      delta:set(gizmoPos.x - selNode.x, gizmoPos.y - selNode.y, gizmoPos.z - selNode.z)
      for i = 1, #nodes do
        nodes[i] = nodes[i] + delta
      end
    else
      selNode:set(gizmoPos)
    end
    spline.isDirty = true -- Mark the spline as dirty to trigger a re-render.
  end

  -- Handle the gizmo for rotation.
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
    local spline = splines[splineIdx]
    local rotMat = editor.getAxisGizmoTransform()
    tmpQuatF:setFromMatrix(rotMat)
    local q = nil
    if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
      q = quat(tmpQuatF)
    else
      q = beginDragRot * quat(tmpQuatF)
    end
    local _, up = q:toDirUp()
    if isRigid then
      for i = 1, #spline.nodes do
        spline.nmls[i] = vec3(up)
      end
    else
      spline.nmls[nodeIdx] = vec3(up)
    end
    spline.isDirty = true
  end
end

-- Handles the gizmo for translation.
local function handleGizmo(isRotEnabledIn, selectedSplineIdx, selectedNodeIdx, splinesIn, isConformToTerrainIn, isRigidIn, deepCopyFunctIn, undoFunctIn, redoFunctIn)
  splines, splineIdx, nodeIdx = splinesIn, selectedSplineIdx, selectedNodeIdx -- Keep the spline/node target in state.
  isRotEnabled, isConformToTerrain, isRigid = isRotEnabledIn, isConformToTerrainIn, isRigidIn
  deepCopyFunct, undoFunct, redoFunct =deepCopyFunctIn, undoFunctIn, redoFunctIn -- Keep the relevant callback functions in state.
  if #splines > 0 then
    local spline = splines[selectedSplineIdx]
    local nodes = spline.nodes
    local selNode = nodes[nodeIdx]
    if selNode then
      local rotation = zeroQuatF
      if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
        local p1, p2 = nodes[max(1, nodeIdx - 1)], nodes[min(#nodes, nodeIdx + 1)]
        tmpTan:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        tmpTan:normalize()
        local q = quatFromDir(tmpTan, spline.nmls[nodeIdx])
        rotation.x, rotation.y, rotation.z, rotation.w = q.x, q.y, q.z, q.w
      end
      local transform = rotation:getMatrix()
      transform:setPosition(selNode)
      editor.setAxisGizmoTransform(transform)
      editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoDragging)
      editor.drawAxisGizmo()
      editor.setAxisGizmoRotateLock(false, isRotEnabled, false) -- We only allow rotation around the local tangent.
      editor.setAxisGizmoTranslateLock(true, true, not isConformToTerrain)
      if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
        editor.setAxisGizmoAlignment(editor.AxisGizmoAlignment_Local) -- Force-set the gizmo to local alignment (only when rotating).
      end
    end
  end
end


-- Public interface.
M.handleGizmo =                                         handleGizmo

return M