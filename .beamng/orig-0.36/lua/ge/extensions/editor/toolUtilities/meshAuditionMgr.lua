-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for auditioning static meshes and handling the static mesh selection window, used across various tools.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local auditionMeshName = 'Temp Audition Mesh' -- The name of the audition mesh, as it will be registered in the scene.
local auditionHeight = 1000.0 -- The height above zero, at which the static meshes are auditioned, in metres.
local camStepSizeRad = math.pi / 500 -- The step size of the angle when rotating the camera around the audition center.
local auditionPlanarDistFac = 1.6 -- A factor used for determining the audition camera planar distance.
local auditionElevationFac = 0.8 -- A factor used for determining the audition camera elevation.
local camRotAngle = 0.0 -- The current angle of the camera around the audition center.
local meshRad = 10.0 -- The assumed radius of the audition mesh.
local spinTime = 0.05 -- The amount of time between each mesh audition rotation, in seconds.

local staticMeshPaths = { -- The paths of the common static meshes, used for searching on init. Can be added to later for larger searches.
  'art/shapes/objects',
  'art/shapes/garage_and_dealership/Clutter',
}

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module constants.
local im = ui_imgui
local meshSelectWinName, meshSelectWinSize = 'meshSelect', im.ImVec2(300, 300)
local sin, cos = math.sin, math.cos
local twoPi = math.pi * 2.0
local gView, auditionVec, auditionCamPos = vec3(0, 0), vec3(0, 0, auditionHeight), vec3(0, 0)
local scaleVec = vec3(1, 1, 1)
local greenB = im.ImVec4(0.28627450980392155, 0.7137254901960784, 0.4470588235294118, 1.0)

-- Module state.
local isMeshSelectWinOpen = false -- A flag which indicates if the mesh selection window is open, or not.
local auditionMesh = nil -- The mesh under audition.
local isAuditionMeshLive = false -- A flag which indicates if the audition mesh exists, or not.
local availStaticMeshes = {} -- The collection of available static meshes.
local selectedMeshIdx = 1 -- The index of the selected mesh.
local hasMeshListBeenComputed = false -- A flag which indicates if the mesh list has been computed, or not.
local oldPos, oldRot = nil, nil -- The previous camera pose, before going to profile view.
local isInMeshView = false -- A flag which indicates if the camera is in static mesh audition view, or not.
local meshFilterBuf = im.ArrayChar(64, "") -- Search bar input buffer.
local timer, time = hptimer(), 0.0
local tmp1 = vec3(0, 0, 0)


-- Function to get the list of available static meshes.
local function getAvailableStaticMeshes() return availStaticMeshes end

-- Populates the list of available static meshes.
local function populateAvailableStaticMeshes()
  table.clear(availStaticMeshes)
  local ctr = 1
  for j = 1, #staticMeshPaths do
    local meshPaths = FS:findFiles(staticMeshPaths[j], "*.dae", -1, true, false)
    for i = 1, #meshPaths do
      availStaticMeshes[ctr] = { path = meshPaths[i], filename = meshPaths[i]:match("([^/]+)$") }
      ctr = ctr + 1
    end
  end
  table.sort(availStaticMeshes, function(a, b) return a.filename < b.filename end)
end

-- Ensures the mesh list is populated [done once, then locked with corresponding flag].
local function ensureMeshListIsPopulated()
  if not hasMeshListBeenComputed then
    populateAvailableStaticMeshes()
    hasMeshListBeenComputed = true
  end
end

-- Removes the static mesh under audition.
local function removeAuditionMesh()
  if isAuditionMeshLive then
    if auditionMesh then
      auditionMesh:delete()
    end
    isAuditionMeshLive = false
  end
  auditionMesh = nil
end

-- Updates the camera position upon changing of selected mesh.
local function updateCameraPose()
  gView:set(0.0, -meshRad * auditionPlanarDistFac, auditionHeight + meshRad * auditionElevationFac)
  local gRot = quatFromDir(auditionVec - gView)
  commands.setFreeCamera()
  core_camera.setPosRot(0, gView.x, gView.y, gView.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Add/replace the mesh under audition with the mesh with the given index.
local function addMeshToAudition(selectedMeshIdx, callback)
  -- Make sure the mesh list is populated.
  ensureMeshListIsPopulated()

  -- Remove the existing audition mesh, if it exists.
  removeAuditionMesh()

  -- Open the mesh selection window, if it is not already open.
  if not isMeshSelectWinOpen then
    isMeshSelectWinOpen = true
    editor.showWindow(meshSelectWinName)
  end

  -- Add the new audition mesh to the scene.
  local path = availStaticMeshes[selectedMeshIdx].path
  isAuditionMeshLive = true
  auditionMesh = createObject('TSStatic')
  auditionMesh:setField('shapeName', 0, path)
  auditionMesh:registerObject(auditionMeshName)
  auditionMesh.canSave = false
  scenetree.MissionGroup:addObject(auditionMesh)
  auditionMesh:setPosRot(0.0, 0.0, auditionHeight, 0, 0, 0, 1)
  auditionMesh.scale = scaleVec

  -- Update the camera distance, based on the size of the newly-selected mesh.
  meshRad = 1.2 * auditionMesh:getObjBox():getLength()
  if isInMeshView then
    updateCameraPose()
  end

  -- Call the given callback function, if it was provided.
  if callback then
    callback(auditionMesh, path)
  end
end

-- Rotate camera around the audition centroid.
local function rotateCamera(ang)
  local x, y, s, c = gView.x, gView.y, sin(ang), cos(ang)
  auditionCamPos:set(x * c - y * s, x * s + y * c, gView.z)
  tmp1:set(auditionVec.x - auditionCamPos.x, auditionVec.y - auditionCamPos.y, auditionVec.z - auditionCamPos.z)
  local gRot = quatFromDir(tmp1)
  core_camera.setPosRot(0, auditionCamPos.x, auditionCamPos.y, auditionCamPos.z, gRot.x, gRot.y, gRot.z, gRot.w)
end

-- Moves the camera to the mesh audition preview pose. [Also adjusts the timing parameters respectively].
local function goToMeshView(timer, time)
  if not isInMeshView then
    time, isInMeshView = 0.0, true
    timer:stopAndReset()
    oldPos, oldRot = core_camera.getPosition(), core_camera.getQuat()                               -- Store the current camera position so we can return to it later.
    updateCameraPose()
  end
  return time
end

-- Manages the rotation of the audition camera.
local function manageRotateCam()
  rotateCamera(camRotAngle)
  camRotAngle = camRotAngle + camStepSizeRad
  if camRotAngle > twoPi then
    camRotAngle = camRotAngle - twoPi
  end
end

-- Returns the camera to the stored old view.
local function goToOldView()
  if oldPos and oldRot then
    core_camera.setPosRot(0, oldPos.x, oldPos.y, oldPos.z, oldRot.x, oldRot.y, oldRot.z, oldRot.w)
  end
  isInMeshView = false
end

-- Registers the mesh selection window.
local function registerWindow() editor.registerWindow(meshSelectWinName, meshSelectWinSize) end

-- Handles the leaving of the audition view (moves camera back and removes audition mesh) then handles the closing of the mesh selection window.
local function leaveAuditionView()
  if isMeshSelectWinOpen then
    removeAuditionMesh()
    goToOldView()
    isMeshSelectWinOpen = false
    editor.hideWindow(meshSelectWinName)
  end
end

-- Handles the static mesh audition selection window.
local function handleMeshAuditionAndSelection(meshTarget, onMeshSelectedFunct)
  if not isMeshSelectWinOpen then
    return -- If there is no currently no mesh auditioning/selection, leave immediately.
  end

  -- Manage the rotation of the camera.
  time = goToMeshView(timer, time)
  time = time + timer:stopAndReset() * 0.001
  if time > spinTime then
    manageRotateCam()
    time = time - spinTime
  end

  -- Render the UI window.
  if editor.beginWindow(meshSelectWinName, "Static Mesh Selector###1184", im.WindowFlags_NoCollapse) then
    -- Search bar panel.
    im.Separator()
    im.TextColored(greenB, "Choose a static mesh for the [" .. meshTarget .. "] component")
    im.Separator()

    -- Begin a 2-column table: left expands, right is fixed width.
    if im.BeginTable("meshSearchBar", 2) then
      im.TableSetupColumn("SearchField", im.TableColumnFlags_WidthStretch)
      im.TableSetupColumn("SearchLabel", im.TableColumnFlags_WidthFixed, 70)
      im.TableNextRow()

      -- Search input (stretchy).
      im.TableSetColumnIndex(0)
      im.PushItemWidth(-1)
      im.InputText("###7377", meshFilterBuf, 64)
      im.tooltip("Type to filter static meshes by name.")
      im.PopItemWidth()

      -- Search label (fixed width).
      im.TableSetColumnIndex(1)
      im.Text("Search...")

      im.EndTable()
    end

    -- Mesh list scroll panel.
    im.Separator()
    im.BeginChild1("meshListScrollArea", im.ImVec2(-1, -1))
    if im.BeginListBox('##meshListBox', im.ImVec2(-1, -1)) then
      im.Columns(1, "meshSelectListboxColumns")
      local filterStr = ffi.string(meshFilterBuf):lower()
      local wCtr = 6345
      for i = 1, #availStaticMeshes do
        local name = availStaticMeshes[i].filename
        if filterStr == "" or string.find(name:lower(), filterStr, 1, true) then
          local flag = i == selectedMeshIdx
          if im.Selectable1(name .. "###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            selectedMeshIdx = i
            addMeshToAudition(selectedMeshIdx, onMeshSelectedFunct)
          end
          wCtr = wCtr + 1
          im.tooltip('Select this static mesh for: [' .. meshTarget .. ']')
          im.Separator()
          im.NextColumn()
        end
      end
      im.EndListBox()
    end
    im.EndChild()
    im.PopItemWidth()
  else
    leaveAuditionView()
  end
end


-- Public interface.
M.getAvailableStaticMeshes =                            getAvailableStaticMeshes

M.populateAvailableStaticMeshes =                       populateAvailableStaticMeshes

M.addMeshToAudition =                                   addMeshToAudition
M.goToMeshView =                                        goToMeshView
M.manageRotateCam =                                     manageRotateCam
M.goToOldView =                                         goToOldView
M.leaveAuditionView =                                   leaveAuditionView
M.removeAuditionMesh =                                  removeAuditionMesh
M.updateCameraPose =                                    updateCameraPose

M.registerWindow =                                      registerWindow
M.handleMeshAuditionAndSelection =                      handleMeshAuditionAndSelection

return M