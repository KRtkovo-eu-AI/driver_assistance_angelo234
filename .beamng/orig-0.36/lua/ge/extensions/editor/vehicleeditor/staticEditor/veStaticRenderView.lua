-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui
local imUtils = require('ui/imguiUtils')

-- Constants
local settingsPath = "settings/vehicleEditorStatic/staticRenderView.json"
local wndNamePrefix = "Static Render View##"

local VIEW_MODE_3D = 1
local VIEW_MODE_LEFT = 2
local VIEW_MODE_RIGHT = 3
local VIEW_MODE_BACK = 4
local VIEW_MODE_FRONT = 5
local VIEW_MODE_BOTTOM = 6
local VIEW_MODE_TOP = 7

local viewModesText = {"3D", "Left (-X)", "Right (+X)", "Back (-Y)", "Front (+Y)", "Bottom (-Z)", "Top (+Z)"}

local camPerspBaseSpeed = 5
local camOrthoBaseSpeed = 2
local camFastSpeedMult = 2.5

local axisArrowColRadius = 0.075

local zeroVec = vec3(0,0,0)
local unitVecs = {vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)}

local axisArrowCols = {
  ColorF(0.5,0,0,1), ColorF(1,0,0,1),
  ColorF(0,0.5,0,1), ColorF(0,1,0,1),
  ColorF(0,0,0.5,1), ColorF(0,0,1,1)
}

local axisArrowTexts = {
  "-X", "X", "-Y", "Y", "-Z", "Z"
}

local rgbCols = {ColorF(1,0,0,1), ColorF(0,1,0,1), ColorF(0,0,1,1)}

-- Variables
local wndsData = {}
--local targetAllMainRVs = DebugDrawerTargetRenderViews(false)
local hoveredWndID = -1
local focusedWndID = -1
local tmpRect = RectI(0,0,0,0)
local camMove = {left = 0, right = 0, forward = 0, backward = 0}
local camSpeedMult = 1

local function getClosestObjectToCamera(cameraPos, hitObjects)
  if next(hitObjects) == nil then return nil end

  local chosenObjData = hitObjects[1]
  if #hitObjects > 1 then
    -- If multiple hit objects, use closest one to camera

    local minDist = (chosenObjData.pos - cameraPos):length()

    for k, objData in ipairs(hitObjects) do
      if k >= 2 then
        local dist = (objData.pos - cameraPos):length()

        if dist < minDist then
          minDist = dist
          chosenObjData = objData
        end
      end
    end
  end

  return chosenObjData
end

local function unprojectRayIntoRVImg(rvData)
  local mousePos = im.GetMousePos()

  local rv = rvData.renderView
  local imagePos = rvData.imagePos
  local imageSize = rvData.imageSize

  --print(imagePos.x .. ", " .. imagePos.y)
  --print(imageSize.x .. ", " .. imageSize.y)

  -- Determine if mouse cursor is in renderview image
  if mousePos.x >= imagePos.x and mousePos.x <= imagePos.x + imageSize.x
  and mousePos.y >= imagePos.y and mousePos.y <= imagePos.y + imageSize.y then

    -- Map image coords to viewport coords
    local x = (mousePos.x - imagePos.x) / imageSize.x * rv.viewPort.extent.x
    local y = (mousePos.y - imagePos.y) / imageSize.y * rv.viewPort.extent.y

    return rv:getUnprojectedRayPosDir(x, y)
  end

  return nil
end

-- Save renderviews data to disk
local function saveSettings()
  local viewsSerialized = {}
  for wndID, wndData in ipairs(wndsData) do
    if wndData then
      local mainRVData = wndData.mainRVData
      local axisGizmoRVData = wndData.axisGizmoRVData

      viewsSerialized[wndID] = {
        wndName = wndData.wndName,
        mainRVData = {
          name = mainRVData.name,
          textureName = mainRVData.textureName,
          mode = mainRVData.mode,
          nearClip = mainRVData.nearClip[0],
          farClip = mainRVData.farClip[0],
          fov = mainRVData.fov[0],
          pos = mainRVData.pos,
          rot = mainRVData.rot,
          gridRot = mainRVData.gridRot,
          ortho = mainRVData.ortho[0],
        },
        axisGizmoRVData = {
          name = axisGizmoRVData.name,
          textureName = axisGizmoRVData.textureName,
        }
      }
    end
  end

  jsonWriteFile(settingsPath, viewsSerialized, true)
end

-- wndData can be nil
local function createRenderViewUIData(wndID, wndData)
  local wndName = wndNamePrefix .. tostring(wndID)

  wndData = wndData or {}
  local mainRVData = wndData.mainRVData or {}
  local axisGizmoRVData = wndData.axisGizmoRVData or {}

  wndsData[wndID] = {
    wndName = wndName,
    -- Main render view
    mainRVData = {
      name = wndName .. 'main',
      textureName = '#' .. wndName .. 'main',
      mode = mainRVData.mode or VIEW_MODE_3D,
      nearClip = im.FloatPtr(mainRVData.nearClip or 0.01),
      farClip = im.FloatPtr(mainRVData.farClip or 100),
      fov = im.FloatPtr(mainRVData.fov or 70),
      pos = mainRVData.pos or vec3(0,0,0),
      rot = mainRVData.rot and quat(mainRVData.rot) or quatFromDir(vec3(0,1,0), unitVecs[3]),
      gridRot = mainRVData.gridRot and quat(mainRVData.gridRot) or quatFromAxisAngle(vec3(1,0,0), math.rad(90)),
      ortho = im.BoolPtr(mainRVData.ortho or false),
      imagePos = im.ImVec2(0,0),
      imageSize = im.ImVec2(0,0),
    },
    -- Axis gizmo view
    axisGizmoRVData = {
      name = wndName .. 'axisGizmo',
      textureName = '#' .. wndName .. 'axisGizmo',
      imagePos = im.ImVec2(0,0),
      imageSize = im.ImVec2(0,0),
    }
  }

  return wndName
end

-- Load renderviews data in memory (not actually creating renderviews)
local function init()
  local viewsSerialized = jsonReadFile(settingsPath)

  if not viewsSerialized then
    createRenderViewUIData(1)
  else
    for id, wndData in ipairs(viewsSerialized) do
      if wndData then
        createRenderViewUIData(id, wndData)
      end
    end
  end
end

-- Creates a new renderview UI
-- Go through list of renderview datas
-- and if data exists but window for that data is hidden, show it
-- else if data doesn't exist, create new data and register new window
local function createRenderViewUI()
  -- Get ID not in use
  local idNotInUse = -1

  for id, wndData in ipairs(wndsData) do
    if not wndData or (wndData and not editor.isWindowVisible(wndData.wndName)) then
      idNotInUse = id
      break
    end
  end

  if idNotInUse == -1 then idNotInUse = #wndsData + 1 end

  local wndName = nil
  if wndsData[idNotInUse] then
    wndName = wndsData[idNotInUse].wndName
  else
    wndName = createRenderViewUIData(idNotInUse)
  end

  if not editor.isWindowRegistered(wndName) then
    editor.registerWindow(wndName, im.ImVec2(400,600))
  end
  editor.showWindow(wndName)

  saveSettings()
end

-- Doesn't close imgui window, only destroys renderview object
-- Used when the editor closes
local function destroyRenderView(id)
  local wndData = wndsData[id]

  if wndData then
    if wndData.mainRVData.renderView then
      RenderViewManagerInstance:destroyView(wndData.mainRVData.renderView)
      wndData.mainRVData.renderView = nil

      --local rvNames = {}
      --for k,v in ipairs(wndsData) do
      --  table.insert(rvNames, v.mainRVData.name)
      --end
      --targetAllMainRVs:setTargets(false, rvNames)
    end
    if wndData.axisGizmoRVData.renderView then
      RenderViewManagerInstance:destroyView(wndData.axisGizmoRVData.renderView)
      wndData.axisGizmoRVData.renderView = nil
    end
  end
end

-- Unregisters imgui window and destroys associated renderview objects
local function removeRenderViewUI(id)
  if wndsData[id] then
    local wndName = wndsData[id].wndName

    editor.unregisterWindow(wndName)
    destroyRenderView(id)
    wndsData[id] = false
    saveSettings()
  end
end

local function _enterViewMode(wndID, viewMode)
  local wndData = wndsData[wndID]
  if not wndData then return end

  local mainRVData = wndData.mainRVData

  if viewMode == VIEW_MODE_3D then
    -- Perspective
    mainRVData.ortho[0] = false
    mainRVData.pos = vec3(0,0,0)
    mainRVData.gridRot = quatFromDir(unitVecs[3], unitVecs[2])

    mainRVData.nearClip[0] = 0.01
    mainRVData.farClip[0] = 2000
    mainRVData.fov[0] = 70

  else
    -- Orthographic
    local qDir
    if viewMode == VIEW_MODE_LEFT then
      qDir = quatFromDir(vec3(1,0,0), unitVecs[3])
    elseif viewMode == VIEW_MODE_RIGHT then
      qDir = quatFromDir(vec3(-1,0,0), unitVecs[3])
    elseif viewMode == VIEW_MODE_BACK then
      qDir = quatFromDir(vec3(0,1,0), unitVecs[3])
    elseif viewMode == VIEW_MODE_FRONT then
      qDir = quatFromDir(vec3(0,-1,0), unitVecs[3])
    elseif viewMode == VIEW_MODE_BOTTOM then
      qDir = quatFromDir(vec3(0,0,1), unitVecs[3])
    elseif viewMode == VIEW_MODE_TOP then
      qDir = quatFromDir(vec3(0,0,-1), unitVecs[1])
    else
      log('E','',"Invalid viewMode: " .. tostring(viewMode))
      return
    end

    mainRVData.ortho[0] = true
    mainRVData.nearClip[0] = 0.6
    mainRVData.farClip[0] = 100
    mainRVData.fov[0] = 150

    mainRVData.pos = qDir * vec3(0,-10,0)
    mainRVData.rot = qDir
    mainRVData.gridRot = qDir
  end

  mainRVData.mode = viewMode
end

--[[
local function _togglePerspectiveOrthoView(wndID)
  local wndData = wndsData[wndID]
  if not wndData then return end

  _enterViewMode()
  if wndsData[wndID].mainRVData.ortho[0] then
    _enterPerspectiveView(wndID)
  else
    _enterOrthographicView(wndID)
  end
end
]]--

local function _drawgrid(gridSize, blockSize, rot, gridOrigin, width)
  local halfSize = gridSize / 2
  local p1 = vec3(-gridSize,0,0)
  local p2 = vec3(gridSize,0,0)
  local col = ColorF(0.75,0.75,0.75,1)
  local axis1Col = ColorF(0.9,0,0,1)
  local axis2Col = ColorF(0,0.9,0,1)
  local bgColor = ColorF(1,1,1,1)

  local numLines = math.floor(gridSize / blockSize)

  for i = 0, numLines do
    local pos = i * blockSize - halfSize

    local lineCol1, lineCol2 = nil, nil
    if i == numLines / 2 then
      lineCol1, lineCol2 = axis1Col, axis2Col
    else
      lineCol1, lineCol2 = col, col
    end

    p1.x = -halfSize
    p2.x = halfSize
    p1.z = pos
    p2.z = pos

    debugDrawer:drawLine(rot * p1 + gridOrigin, rot * p2 + gridOrigin, lineCol1)

    p1.x = pos
    p2.x = pos
    p1.z = -halfSize
    p2.z = halfSize

    debugDrawer:drawLine(rot * p1 + gridOrigin, rot * p2 + gridOrigin, lineCol2)
  end
end

local tempLinePoint1, tempLinePoint2 = vec3(), vec3()

local function _selectAndDrawAxisArrows(wndID)
  local wndData = wndsData[wndID]
  local mainRVData, axisGizmoRVData = wndData.mainRVData, wndData.axisGizmoRVData

  local mainRVMat = mainRVData.renderView.cameraMatrix:inverse()
  local axisGizmoRenderView = axisGizmoRVData.renderView

  local rayDist = 100
  local vecMult = 0.33

  local arrowVecs = {
    -mainRVMat:getColumn(0) * vecMult, mainRVMat:getColumn(0) * vecMult,
    -mainRVMat:getColumn(1) * vecMult, mainRVMat:getColumn(1) * vecMult,
    -mainRVMat:getColumn(2) * vecMult, mainRVMat:getColumn(2) * vecMult
  }

  local hitArrows = {}

  local res, pos, dir = unprojectRayIntoRVImg(axisGizmoRVData)

  -- First get axis arrows hovered by mouse cursor
  if res then
    local rayStartPos = pos
    local rayEndPos = pos + dir * rayDist

    for k, arrowVec in ipairs(arrowVecs) do
      -- Don't allow picking arrow if its pretty much inline with vector projected out of user's screen
      if arrowVec:dot(vec3(0,-1,0)) < 0.99 then
        local xnorm1, xnorm2 = closestLinePoints(rayStartPos, rayEndPos, zeroVec, arrowVec)

        if xnorm2 >= 0 and xnorm2 <= 1 then
          tempLinePoint1:setLerp(rayStartPos, rayEndPos, xnorm1)
          tempLinePoint2:setLerp(zeroVec, arrowVec, clamp(xnorm2, 0, 1))

          local minSqPointDis = tempLinePoint1:squaredDistance(tempLinePoint2)

          if minSqPointDis < axisArrowColRadius * axisArrowColRadius then
            -- Collision occurred!
            table.insert(hitArrows, {id = k, pos = (rayStartPos + rayEndPos) * 0.5})

            --debugDrawer:drawLine(zeroVec, arrowVec, ColorF(1,0,1,1))
          end
        end
      end
    end
  end

  -- Choose closet one of hovered ones
  local chosenArrow = getClosestObjectToCamera(pos, hitArrows)

  -- Render and do stuff based on chosen one
  for k, arrowVec in ipairs(arrowVecs) do
    local arrowCol = nil

    if chosenArrow and chosenArrow.id == k then
      arrowCol = ColorF(1,1,0,1)

      if im.IsMouseClicked(0) then
        _enterViewMode(wndID, 1 + k)
      end
    else
      arrowCol = axisArrowCols[k]
    end

    -- If arrow vec is becoming inline with the vector pointing out of user's screen
    -- fade out the arrow to be able to see the other arrows
    if math.abs(arrowVec:dot(vec3(0,-1,0))) < 0.99 * vecMult then
      --arrowCol.a = (dotProd - 0.75) * 1 / -0.25 + 1
      debugDrawer:drawCylinder(zeroVec, arrowVec, 0.01, arrowCol)

      if k % 2 == 0 then
        debugDrawer:drawText(arrowVec, axisArrowTexts[k], ColorF(0.1,0.1,0.1,1))
      end
    end
  end
end

-- Do debugDrawer stuff here
local function onPreRender()
  for wndID, wndData in ipairs(wndsData) do
    if wndData and wndData.mainRVData.renderView and wndData.axisGizmoRVData.renderView then
      -- Main Render View
      local mainRVData = wndData.mainRVData
      --debugDrawer:setTargetRenderViews(mainRVData.targetRVs)

      local gridSize = 10 -- meters
      local gridBlockSize = 0.25
      local gridLineWidth = 0.5

      --debugDrawer:drawLine(vec3(0,0,0), vec3(1,0,0), ColorF(1,0,0,1))
      --debugDrawer:drawLine(vec3(0,0,0), vec3(0,1,0), ColorF(0,1,0,1))
      --debugDrawer:drawLine(vec3(0,0,0), vec3(0,0,1), ColorF(0,0,1,1))

      _drawgrid(gridSize, gridBlockSize, mainRVData.gridRot, vec3(0,0,0), gridLineWidth)

      debugDrawer:drawTextAdvanced((vec3(5,15,0)), "Mode: " .. viewModesText[mainRVData.mode], ColorF(0,0,0,1), false, true, ColorI(0, 0, 0, 255))

      --local txt = dumps{'type: ', viewModesText[mainRVData.mode], 'view.pos: ', mainRVData.pos, 'view.rot: ', mainRVData.rot}
      --debugDrawer:drawTextAdvanced((vec3(0,15,0)), mainRVData.name, ColorF(0,0,0,1), false, true, ColorI(0, 0, 0, 255))
      --debugDrawer:drawTextAdvanced((vec3(0,35,0)), "PLEASE DISABLE AMBIENT OCCLUSION", ColorF(0,0,0,1), false, true, ColorI(0, 0, 0, 255))
      --debugDrawer:drawTextAdvanced((vec3(0,55,0)), txt, ColorF(0,0,0,1), false, true, ColorI(0, 0, 0, 255))
      --debugDrawer:drawTextAdvanced((vec3(0,75,0)), 'viewport: ' .. mainRVData.renderView.viewPort:__tostring(), ColorF(0,0,0,1), false, true, ColorI(0, 0, 0, 255))

      -- Axis Gizmo Render View
      local axisGizmoRVData = wndData.axisGizmoRVData
      --debugDrawer:setTargetRenderViews(axisGizmoRVData.targetRVs)
      _selectAndDrawAxisArrows(wndID)

      --debugDrawer:clearTargetRenderViews()
    end
  end
end

local function _drawContextMenu(wndID)
  local wndData = wndsData[wndID]
  local mainRVData = wndData.mainRVData

  im.PushStyleVar2(im.StyleVar_WindowPadding, im.ImVec2(6, 6))
  if im.BeginPopup('viewcontrol' .. tostring(wndData.wndName)) then
    if im.BeginMenu('Mode') then
      if im.MenuItem1('3D') then
        _enterViewMode(wndID, VIEW_MODE_3D)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Left') then
        _enterViewMode(wndID, VIEW_MODE_LEFT)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Right') then
        _enterViewMode(wndID, VIEW_MODE_RIGHT)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Front') then
        _enterViewMode(wndID, VIEW_MODE_FRONT)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Back') then
        _enterViewMode(wndID, VIEW_MODE_BACK)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Top') then
        _enterViewMode(wndID, VIEW_MODE_TOP)
        im.CloseCurrentPopup()
        saveSettings()
      end
      if im.MenuItem1('Bottom') then
        _enterViewMode(wndID, VIEW_MODE_BOTTOM)
        im.CloseCurrentPopup()
        saveSettings()
      end
      im.EndMenu()
    end
    im.PushItemWidth(100)
    if im.SliderFloat('Near Clip', mainRVData.nearClip, 0.001, math.min(55, mainRVData.farClip[0] - 0.001), "%.3f") then saveSettings() end
    im.PushItemWidth(100)
    if im.SliderFloat('Far Clip', mainRVData.farClip, mainRVData.nearClip[0] + 0.001, 100, "%.3f") then saveSettings() end
    im.PushItemWidth(100)
    if im.SliderFloat('FOV', mainRVData.fov, 0.001, 179, "%.3f") then saveSettings() end
    im.Separator()
    if im.MenuItem1('Add New View') then
      createRenderViewUI()
      im.CloseCurrentPopup()
    end
    if im.MenuItem1('Delete This View') then
      removeRenderViewUI(wndID)
      im.CloseCurrentPopup()
    end
    im.PopItemWidth()
    im.EndPopup()
  end
  im.PopStyleVar()
end

local function _windowContent(wndID)
  local wndData = wndsData[wndID]
  local mainRVData = wndData.mainRVData
  local axisGizmoRVData = wndData.axisGizmoRVData

  local mainRVScreenPos = im.GetCursorScreenPos()
  local mainRVPos = im.GetCursorPos()
  local mainRVSize = im.GetContentRegionAvail()

  local axisGizmoRVPadding = 15
  local axisGizmoRVSize = im.ImVec2(100,100)
  local axisGizmoRVPos = im.ImVec2(mainRVPos.x + mainRVSize.x - axisGizmoRVSize.x - axisGizmoRVPadding, mainRVPos.y + axisGizmoRVPadding)

 -- dump{axisGizmoRVPos.x, axisGizmoRVPos.y}

  -- Main RenderView
  local mainRVMat = QuatF(mainRVData.rot.x, mainRVData.rot.y, mainRVData.rot.z, mainRVData.rot.w):getMatrix()
  mainRVMat:setPosition(mainRVData.pos)

  mainRVData.renderView.renderCubemap = false
  mainRVData.renderView.cameraMatrix = mainRVMat
  mainRVData.renderView.resolution = Point2I(mainRVSize.x, mainRVSize.y)
  tmpRect:set(0, 0, mainRVSize.x, mainRVSize.y)
  mainRVData.renderView.viewPort = tmpRect

  local aspectRatio = mainRVSize.x / mainRVSize.y
  mainRVData.renderView.frustum = Frustum.construct(mainRVData.ortho[0], math.rad(mainRVData.fov[0]), aspectRatio, mainRVData.nearClip[0], mainRVData.farClip[0])
  mainRVData.renderView.fov = mainRVData.fov[0]
  mainRVData.renderView.renderEditorIcons = false

  -- Axis Gizmo RenderView (same rotation as main renderview but at position (0,0,0))
  --local axisGizmoRVMat = QuatF(mainRVData.rot.x, mainRVData.rot.y, mainRVData.rot.z, mainRVData.rot.w):getMatrix()
  local axisGizmoRVMat = QuatF(0,0,0,1):getMatrix()
  axisGizmoRVMat:setPosition(vec3(0,-1,0))

  axisGizmoRVData.renderView.renderCubemap = false
  axisGizmoRVData.renderView.cameraMatrix = axisGizmoRVMat
  axisGizmoRVData.renderView.resolution = Point2I(axisGizmoRVSize.x, axisGizmoRVSize.y)
  tmpRect:set(0, 0, axisGizmoRVSize.x, axisGizmoRVSize.y)
  axisGizmoRVData.renderView.viewPort = tmpRect

  local aspectRatio = axisGizmoRVSize.x / axisGizmoRVSize.y
  axisGizmoRVData.renderView.frustum = Frustum.construct(false, math.rad(45), aspectRatio, 0.01, 5)
  axisGizmoRVData.renderView.fov = math.rad(45)
  axisGizmoRVData.renderView.renderEditorIcons = false

  local mainRVTexObj = imUtils.texObj(mainRVData.textureName)
  local axisGizmoRVTexObj = imUtils.texObj(axisGizmoRVData.textureName)

  local startCursorPos = im.GetCursorPos()

  --[[
  -- Button to switch between ortho and perspective
  local btnWidth = im.CalcTextSize(">>>").x
  im.SetCursorPos(im.ImVec2(mainRVSize.x - btnWidth - 30, 200))
  if editor.uiIconImageButton(editor.icons.switch_camera, nil, im.ImColorByRGB(255,255,255,255).Value, nil, nil) then
    _togglePerspectiveOrthoView(wndID)
  end
  ]]--

  -- Main Render View Image
  im.SetCursorPos(startCursorPos)
  im.Image(mainRVTexObj.texId, mainRVSize)
  _drawContextMenu(wndID)
  if im.IsItemClicked(1) then
    im.OpenPopup('viewcontrol' .. tostring(wndData.wndName))
  end

  -- Axis Gizmo Render View Image
  im.SetCursorPos(axisGizmoRVPos)
  local axisGizmoRVScreenPos = im.GetCursorScreenPos()

  im.Image(axisGizmoRVTexObj.texId, axisGizmoRVSize)

  wndData.mainRVData.imagePos = mainRVScreenPos
  wndData.mainRVData.imageSize = mainRVSize
  wndData.axisGizmoRVData.imagePos = axisGizmoRVScreenPos
  wndData.axisGizmoRVData.imageSize = axisGizmoRVSize
end

local function pollInput(dt)
  local focusedWndData = wndsData[focusedWndID]
  if not focusedWndData then return end

  local hoveredWndData = wndsData[hoveredWndID]

  local mainRVData = focusedWndData.mainRVData
  local axisGizmoRVData = focusedWndData.axisGizmoRVData

  local mousePos = im.GetMousePos()

  local dx,dy,dz = 0,0,0

  -- middle click dragging?
  if focusedWndData.mouseDragging2 then
    dx = (mousePos.x - focusedWndData.lastMouseDragPos.x) * 0.005 --* (mainRVData.fov[0] / 50)
    dy = (mousePos.y - focusedWndData.lastMouseDragPos.y) * 0.005 --* (mainRVData.fov[0] / 50)
  end

  if hoveredWndData and hoveredWndID == focusedWndID then
    -- zooming
    local w = im.GetIO().MouseWheel
    if mainRVData.ortho[0] then
      mainRVData.fov[0] = mainRVData.fov[0] - w * 10
      if mainRVData.fov[0] < 0.1 then mainRVData.fov[0] = 0.1 end
      if mainRVData.fov[0] > 170 then mainRVData.fov[0] = 170 end
    else
      dz = -w
    end

    if dx ~= 0 or dy ~= 0 or dz ~= 0 then saveSettings() end
  end

  -- Move camera based on projection
  if mainRVData.ortho[0] then
    mainRVData.pos = mainRVData.pos + mainRVData.rot * vec3(camMove.right - camMove.left, 0, camMove.forward - camMove.backward)
    * dt * camOrthoBaseSpeed * camSpeedMult

    -- Enter perspective if mouse middle button dragged
    if focusedWndData.mouseDragging2 and focusedWndData.lastMouseDragPos then
      _enterViewMode(focusedWndID, VIEW_MODE_3D)
    end
  else
    mainRVData.pos = mainRVData.pos + mainRVData.rot * vec3(camMove.right - camMove.left, camMove.forward - camMove.backward, 0)
    * dt * camPerspBaseSpeed * camSpeedMult

    if focusedWndData.mouseDragging2 and focusedWndData.lastMouseDragPos then
      mainRVData.rot = quatFromAxisAngle(vec3(1,0,0), dy) * mainRVData.rot * quatFromAxisAngle(vec3(0,0,1), dx)
    end
  end

  if im.IsMouseClicked(0) then
    if hoveredWndData then
      hoveredWndData.lastMouseDragPos = mousePos
    end
  end

  local isMouseDragging2 = im.IsMouseDragging(2)

  if not focusedWndData.mouseDragging2 then
    if isMouseDragging2 and hoveredWndID == focusedWndID then
      focusedWndData.mouseDragging2 = isMouseDragging2
      focusedWndData.lastMouseDragPos = mousePos
    end
  else
    focusedWndData.mouseDragging2 = isMouseDragging2
    if focusedWndData.mouseDragging2 then
      focusedWndData.lastMouseDragPos = mousePos
    end
  end
end

local function onEditorGui(dt)
  im.PushStyleVar1(im.StyleVar_WindowBorderSize, 0)
  im.PushStyleVar2(im.StyleVar_WindowPadding, im.ImVec2(0, 0))

  -- Move camera on window focused
  pollInput(dt)

  hoveredWndID = -1
  focusedWndID = -1

  for wndID, wndData in ipairs(wndsData) do
    if wndData and editor.isWindowRegistered(wndData.wndName) then
      if editor.beginWindow(wndData.wndName, wndData.wndName) then -- Window open
        if wndData.mainRVData.renderView and wndData.axisGizmoRVData.renderView then
          wndData.wndOpen = true

          if im.IsWindowHovered() then hoveredWndID = wndID end
          if im.IsWindowFocused() then focusedWndID = wndID end

          -- Render window contents
          _windowContent(wndID)
        else
          -- Create renderview if it doesn't exist for window
          im.TextUnformatted('...')

          local mainRVData = wndData.mainRVData
          local mainRVName = mainRVData.name
          mainRVData.renderView = RenderViewManagerInstance:getOrCreateView(mainRVName)
          mainRVData.renderView.namedTexTargetColor = mainRVName
          --mainRVData.targetRVs = DebugDrawerTargetRenderViews({mainRVName})
          mainRVData.renderView:enterFocusObjectsMode()

          local axisGizmoRVData = wndData.axisGizmoRVData
          local axisGizmoRVName = axisGizmoRVData.name
          axisGizmoRVData.renderView = RenderViewManagerInstance:getOrCreateView(axisGizmoRVName)
          axisGizmoRVData.renderView.namedTexTargetColor = axisGizmoRVName
          --axisGizmoRVData.targetRVs = DebugDrawerTargetRenderViews({axisGizmoRVName})
          axisGizmoRVData.renderView:enterFocusObjectsMode()

          -- Set targetAllMainRVs to just main renderviews for the debugdrawing into all main renderviews
          --local rvNames = {}
          --for k,v in ipairs(wndsData) do
          --  table.insert(rvNames, v.mainRVData.name)
          --end
          --targetAllMainRVs:setTargets(false, rvNames)
        end
      else -- Window closed
        -- Hacky way to remove renderview :(
        if wndData.wndOpen == false then
          destroyRenderView(wndID)
        end

        wndData.wndOpen = false
      end
      editor.endWindow()
    end
  end

  im.PopStyleVar()
  im.PopStyleVar()
end

local function getMainRenderViewMouseRay()
  local wndData = wndsData[hoveredWndID]
  if wndData then
    return unprojectRayIntoRVImg(wndData.mainRVData)
  end

  return nil
end

local function destroyAllRenderViews()
  for id, data in ipairs(wndsData) do
    if data then
      destroyRenderView(id)
    end
  end
end

local function onEditorHeadlessChange(enabled, toolName)
  if toolName == vEditor.EDITOR_NAMES[vEditor.EDITOR_MODE_STATIC] then
    if enabled then
      -- Entering editor
      setRenderWorldMain(false)
    else
      -- Exiting editor
      saveSettings()
      setRenderWorldMain(true)
      destroyAllRenderViews()
    end
  end
end

local function onEditorInitialized()
  init()

  if editor.isHeadlessToolActive(vEditor.EDITOR_NAMES[vEditor.EDITOR_MODE_STATIC]) then
    setRenderWorldMain(false)
  end
end

local function onSerialize()
  saveSettings()
  --editor_veMain.saveCurrentWindowLayout()
  return nil
end

local function onDeserialize(data)
end

-- Functions called on keybindings

local function moveLeft    (val) camMove.left     = val end
local function moveRight   (val) camMove.right    = val end
local function moveForward (val) camMove.forward  = val end
local function moveBackward(val) camMove.backward = val end
local function setCameraSpeed(s) camSpeedMult = s * (camFastSpeedMult - 1) + 1  end

M.onPreRender = onPreRender
M.onEditorGui = onEditorGui

M.createRenderViewUI = createRenderViewUI
M.getMainRenderViewMouseRay = getMainRenderViewMouseRay
M.destroyAllRenderViews = destroyAllRenderViews

M.onEditorHeadlessChange = onEditorHeadlessChange
M.onEditorInitialized = onEditorInitialized
M.onSerialize = onSerialize
M.onDeserialize = onDeserialize

M.moveLeft       = moveLeft
M.moveRight      = moveRight
M.moveForward    = moveForward
M.moveBackward   = moveBackward
M.setCameraSpeed = setCameraSpeed

return M