-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_selection"
local im = ui_imgui
local gizmo = nil

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local function getDraggableChildLayer(layer)
  if layer.children then
    for _, child in ipairs(layer.children) do
      if child.type == api.layerTypes.decal or child.type == api.layerTypes.brushStroke or child.type == api.layerTypes.path then
        return child
      end
      return getDraggableChildLayer(child)
    end
  end

  return nil
end

local function selectLayer(uid, addToSelection)
  if not uid then editor.logWarn(string.format("%s.selectLayer(): 'uid' argument must not be empty.", logTag)) return end
  if not addToSelection then
    editor.selection = {}
  end
  if not editor.selection["dynamicDecalLayer"] then
    editor.selection["dynamicDecalLayer"] = {}
  end
  local layerData = deepcopy(api.getLayerByUid(uid))
  editor.selection["dynamicDecalLayer"][uid] = layerData

  -- Reset gizmo transform functions
  gizmo.resetTransformFn()

  -- enabling gizmo
  if layerData.type == api.layerTypes.decal or layerData.type == api.layerTypes.brushStroke or layerData.type == api.layerTypes.path or layerData.type == api.layerTypes.group then
    gizmo.data.type = "drag"
    gizmo.data.uid = layerData.uid

    local vehicleObj = getPlayerVehicle(0)

    if layerData.type == api.layerTypes.decal then
      gizmo.data.objectType = "decal"
      gizmo.transform = api.getDecalWorldTransform(layerData)

      gizmo.translateFn = function(newGizmoTransform)
        api.moveLayerLocalPos(layerData.uid, newGizmoTransform:getPosition(), true)
      end
      gizmo.rotateFn = function(newGizmoTransform)
        local delta = vec3(worldEditorCppApi.getAxisGizmoRotateOffset())
        api.rotateLayer(layerData, delta)
        --layerData.decalRotation = layerData.decalRotation + delta.y
        --api.setLayer(layerData, true)
      end
      gizmo.scaleFn = function(newGizmoTransform)
        if gizmo.getDragCount() > 0 then
          local delta = vec3(worldEditorCppApi.getAxisGizmoScaleOffset())
          api.scaleLayer(layerData, delta)
        end
      end
    elseif layerData.type == api.layerTypes.brushStroke then
      local layerDataCopy = shallowcopy(layerData)
      layerDataCopy.cursorPosScreenUv = {x = layerData.dataPoints[1].x, y = layerData.dataPoints[1].y}
      gizmo.transform:setPosition(api.getDecalLocalPos(layerDataCopy))
      gizmo.data.objectType = "brushStroke"

      gizmo.translateFn = function(newGizmoTransform)
        api.moveLayerLocalPos(layerData.uid, newGizmoTransform:getPosition() - vehicleObj:getTransform():getPosition(), true)
      end
    elseif layerData.type == api.layerTypes.path then
      local layerDataCopy = shallowcopy(layerData)
      layerDataCopy.cursorPosScreenUv = {x = layerData.dataPoints[1].x, y = layerData.dataPoints[1].y}
      gizmo.transform:setPosition(api.getDecalLocalPos(layerDataCopy))
      gizmo.data.objectType = "path"

      gizmo.translateFn = function(newGizmoTransform)
        api.moveLayerLocalPos(layerData.uid, newGizmoTransform:getPosition() - vehicleObj:getTransform():getPosition(), true)
      end
    elseif layerData.type == api.layerTypes.group then
      gizmo.data.objectType = "group"
      -- we gotta check whether the group layer has children that can be moved since the group itself has no cursorPosScreenUv property
      local draggableChildLayer = getDraggableChildLayer(layerData)
      if draggableChildLayer then
        if draggableChildLayer.type == api.layerTypes.decal then
          gizmo.transform:setPosition(api.getDecalLocalPos(draggableChildLayer))
        elseif draggableChildLayer.type == api.layerTypes.path then
          draggableChildLayer.cursorPosScreenUv = {x = draggableChildLayer.dataPoints[1].x, y = draggableChildLayer.dataPoints[1].y}
          gizmo.transform:setPosition(api.getDecalLocalPos(draggableChildLayer))
        elseif draggableChildLayer.type == api.layerTypes.brushStroke then
          draggableChildLayer.cursorPosScreenUv = {x = draggableChildLayer.dataPoints[1].x, y = draggableChildLayer.dataPoints[1].y}
          gizmo.transform:setPosition(api.getDecalLocalPos(draggableChildLayer))
        end
        gizmo.translateFn = function(newGizmoTransform)
          api.moveLayerLocalPos(layerData.uid, newGizmoTransform:getPosition() - vehicleObj:getTransform():getPosition(), true, draggableChildLayer.uid)
        end
      else
        editor.log("no draggableChildLayer found")
      end
    else
      gizmo.data.objectType = "other layer type"
      gizmo.translateFn = function(newGizmoTransform)
        editor.log("M.translateFn not yet implemented")
      end
    end

    editor.setAxisGizmoTransform(gizmo.transform)
  else
    gizmo.translateFn = nil
  end
end

local function deselectLayer(uid)
  if uid then
    if editor.selection["dynamicDecalLayer"] and editor.selection["dynamicDecalLayer"][uid] then
      editor.selection["dynamicDecalLayer"][uid] = nil
      if tableSize(editor.selection["dynamicDecalLayer"]) == 0 then
        editor.selection["dynamicDecalLayer"] = nil
      end
    end
  else
    editor.selection["dynamicDecalLayer"] = nil
  end
  gizmo.setTransformMode(gizmo.transformModes.none)
  gizmo.translateFn = nil
  gizmo.data = {}
  tool.toolMode = tool.toolModes.decal
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  gizmo = extensions.editor_dynamicDecals_gizmo
end

M.selectLayer = selectLayer
M.deselectLayer = deselectLayer

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M