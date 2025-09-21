-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_gizmo"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local api = nil

M.data = {}
--[[
M.data.uid = `layer.uid`
M.data.type = "drag", "rotate", ...
M.data.objectType = "decal", "group", "path", "path_dataPoint"
[optional] M.data.dataPointIndex = `index of point in array`
M.data.objectDataCopy = -- data of the object at the begin of the drag
]]
M.transform = MatrixF(true)
M.translateFn = nil
M.rotateFn = nil
M.scaleFn = nil

M.transformModes = {
  none = 0,
  translate = 1,
  rotate = 2,
  scale = 3
}
local transformMode = M.transformModes.none
local dragCount = 0

local cubePoints = {
  vec3(-0.5, -0.5, -0.5), vec3(-0.5, -0.5, 0.5), vec3(-0.5, 0.5, -0.5), vec3(-0.5, 0.5, 0.5),
  vec3(0.5, -0.5, -0.5), vec3(0.5, -0.5, 0.5), vec3(0.5, 0.5, -0.5), vec3(0.5, 0.5, 0.5)
}

local function getTransformMode()
  return transformMode
end

local function setTransformMode(value)
  transformMode = value
end

local function resetTransformFn()
  M.translateFn = nil
  M.rotateFn = nil
  M.scaleFn = nil
end

local function getDragCount()
  return dragCount
end

-- called when the axis gizmo started to be dragged
local function gizmoBeginDrag()
  -- M.data.objectDataCopy = deepcopy(M.getLayerByUid(M.data.))
  -- you can initialize some dragging variables here
  -- or save the previous state of your edited object
  dragCount = 0
end

local function gizmoEndDrag()
  -- you can add some undo action here after all dragging ended
  -- comprised of previous and current state
  dragCount = 0
end

local function gizmoDragging()
  dragCount = dragCount + 1
  if dragCount == 1 then
    return
  end

  -- update/save our gizmo matrix
  if transformMode == M.transformModes.translate then
    if M.translateFn then
      M.translateFn(editor.getAxisGizmoTransform())
    else
      editor.log("M.translateFn not yet implemented")
    end
  elseif transformMode == M.transformModes.rotate then
    if M.rotateFn then
      M.rotateFn(editor.getAxisGizmoTransform())
    else
      editor.log("M.rotateFn not yet implemented")
    end
  elseif transformMode == M.transformModes.scale then
    if M.scaleFn then
      M.scaleFn(editor.getAxisGizmoTransform())
    else
      editor.log("M.scaleFn not yet implemented")
    end
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "gizmo", nil, {
    {displayBoundingBox = {"bool", true, "displayBoundingBox"}},
    {boundingBoxColor = {"table", {1,0,0,0.75}, "boundingBoxColor", nil, nil, nil, nil, nil, function(cat, subCat, item)
      if im.ColorEdit4("##prefsboundingBoxColor", editor.getTempFloatArray4_TableTable(editor.getPreference("dynamicDecalsTool.gizmo.boundingBoxColor")), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        editor.setPreference("dynamicDecalsTool.gizmo.boundingBoxColor", editor.getTempFloatArray4_TableTable())
      end
    end}},
    {displayDebugObjWhileMovingControlPoints = {"bool", true, "displayDebugObjWhileMovingControl"}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  if transformMode ~= M.transformModes.none and editor.active then
    editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoDragging)
    editor.drawAxisGizmo()

    if editor.active and editor.getPreference("dynamicDecalsTool.gizmo.displayBoundingBox") then
      -- TODO: Cache selection transform
      local selection = editor.selection["dynamicDecalLayer"]
      if selection then
        local layer = api.getLayerByUid(next(selection))
        if not layer then return end
        if layer.type == api.layerTypes.decal then
          local transform = api.getDecalWorldTransform(layer)
          local color = editor.getPreference("dynamicDecalsTool.gizmo.boundingBoxColor")
          editor.drawSelectionBBox(transform, ColorF(color[1], color[2], color[3], color[4]))
        end
      end
    end
  end

  if editor.getPreference("dynamicDecalsTool.gizmo.displayDebugObjWhileMovingControlPoints") and data and data.debugObjPos then
    local col = editor.getPreference("dynamicDecalsTool.general.dataPointSphereColor")
    debugDrawer:drawSphere(data.debugObjPos, editor.getPreference("dynamicDecalsTool.general.dataPointSphereSize"), ColorF(col[1], col[2], col[3], col[4]), col[4] < 0.99 and true or false)
  end
end

local function setup(tool_in)
  tool = tool_in

  api = extensions.editor_api_dynamicDecals
end

M.getTransformMode = getTransformMode
M.setTransformMode = setTransformMode
M.resetTransformFn = resetTransformFn
M.getDragCount = getDragCount
M.gizmoBeginDrag = gizmoBeginDrag
M.gizmoEndDrag = gizmoEndDrag
M.gizmoDragging = gizmoDragging
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.editModeUpdate = editModeUpdate
M.setup = setup

return M