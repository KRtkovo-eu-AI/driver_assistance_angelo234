-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = ui_imgui
local imguiUtils = require('ui/imguiUtils')
local vehicleActionMaps = {"VehicleCommonActionMap", "VehicleSpecificActionMap"}

M.dependencies = {"ui_imgui", "render_renderViews"}
M.state = {show= false}

local cameraConfigFile

-- Of the currently spawned vehicle
local currConfigName
local currModelName

-- helper variables
local columnFlags = bit.bor(im.TableColumnFlags_NoResize, im.TableColumnFlags_WidthStretch)
local tableFlags = bit.bor(im.TableFlags_NoBordersInBody)
local imVec24x24 = im.ImVec2(24,24)
local imVec32x32 = im.ImVec2(32, 32)
local imVec4Grey = im.ImVec4(0.65,0.65,0.65,1)
local imVec4Yellow = im.ImVec4(1,1,0,1)
local imVec4Red = im.ImVec4(1,0,0,1)
local imVec4Green = im.ImVec4(0,1,0,1)

local workerCoroutine = nil
local forceQuit = false

local windowOpen = im.BoolPtr(false)
local initialWindowSize = im.ImVec2(300, 500)
local runDone = false
local currentlyPreviewingCamera = false
local plRes

local vehList -- table array  {vehFolder, boolptr, nameStr, config number?}

local userDefinedDynamicReflections -- used to revert "GraphicDynReflectionEnabled" back to what the user had

local thumbnailConfig = {
  fileEnding = ".jpg",
  fov = 20,
  nearPlane = 0.1,
}

local presetOutputDestinations = {
  "Vehicle thumbnails",
  "Screenshot/showroom folder"
}

local presetResolutions = { -- name, width, height
  {'thumbnail', 500, 281},
  {'thumbnail temp', 2000, 1124},
  {'720p'   ,  1280, 720},
  {'1080p'  , 1920, 1080},
  {'Square' , 1920, 1920},
  {'WQHD'   , 2560, 1440},
  {'UWQHD'  , 3440, 1440},
  {'4k'     , 3840, 2160},
  {'8k'     , 8192, 4320},
}

local thumbnailDefaultValues = { -- default values for imgui controls
  ouputDestination = 1, -- "Vehicle thumbnails"
  resolution = 2 -- "500 * 281"
}

local reviewData

local ctrls = { -- imgui controls
  generateMissingThumbnailsOnly = im.BoolPtr(false),
  reloadUIOnJobFinished = im.BoolPtr(true),
  camSpeedPtr = im.FloatPtr(core_camera.getSpeed()),
  drivingEnabled = true,
  uiToggled = true,
  resolutionToggled = false,
  keepAspectRatio = im.BoolPtr(false),
  multiplyRes = im.FloatPtr(2)
}

local workConfig = {} -- current screenshot config

-- check if current settings are the same as the defaults ones
local function isDefaultConfig()
  return ctrls.currOuputDestinationPtr[0] == thumbnailDefaultValues.ouputDestination -1 and ctrls.currResolutionsPtr[0] == thumbnailDefaultValues.resolution - 1
end

-- check if the user wants to update the vehicles thumbnails, if the current settings match the default thumbnail settings
local function isThumbnailConfigGood()
  return not(presetOutputDestinations[ctrls.currOuputDestinationPtr[0]+1] == "Vehicle thumbnails"
  and ctrls.currResolutionsPtr[0] + 1 ~= thumbnailDefaultValues.resolution)
end

local function isInShowroom()
  return getCurrentLevelIdentifier() == "showroom_v2"
end

local function sanitizeVehConfigName(dirtyName)
  return string.match(dirtyName, "([^/]+)%.pc$")
end

local function trySetDynamicReflections(value)
  if isInShowroom() then
    settings.setValue('GraphicDynReflectionEnabled', value)
  end
end

local function getSelectedVehCount()
  local count = 0
  for k, v in pairs(vehList) do
    if v[2][0] then
      count = count + 1
    end
  end
  return count
end


local function populateVehGui()
  vehList = {}
  local models = core_vehicles.getModelList().models --because
  local modelKeys={}
  for k,_ in pairs(models) do table.insert(modelKeys,k) end
  table.sort(modelKeys)
  for _,k in ipairs(modelKeys) do
    local v = models[k]
    vehList[_] = {k,im.BoolPtr(false), (v.Brand and (v.Brand.." ") or "") ..v.Name }
  end
  table.sort(vehList, function(a,b) return a[3]<b[3] end )
end

local function isBatch()
  local cmdArgs = Engine.getStartingArgs()
  local probability = 0
  for i = 1, #cmdArgs do
    local arg = cmdArgs[i]
    arg = arg:stripchars('"')
    if arg == "-onLevelLoad_ext" or arg == "'util/createThumbnails'" then
      probability = probability +1
    end
  end
  return probability > 1
end

local function yieldSec(yieldfn,sec)
  local start  = os.clock()
  while (start+sec)>os.clock() do
    yieldfn()
  end
end

local function onPreRender(dt)
  if workerCoroutine ~= nil then
    local errorfree, value = coroutine.resume(workerCoroutine)
    if not errorfree then
      log('E', '', "workerCoroutine: "..value)
      log("E", '', debug.traceback(workerCoroutine))
    end
    if coroutine.status(workerCoroutine) == "dead" then
      workerCoroutine = nil
      if isBatch() then
        log('E', '', 'coroutine BROKE')
        shutdown(0)
      end
    end
  end
end


local function frameVehicle(veh, fov, nearPlane, aspectRatio)
  nearPlane = nearPlane or 0.1

  local bb = veh:getSpawnWorldOOBB()
  local bbCenter = bb:getCenter()
  local axis0, axis1, axis2 = bb:getAxis(0), bb:getAxis(1), bb:getAxis(2)
  local halfExtentsX, halfExtentsY, halfExtentsZ = bb:getHalfExtents().x, bb:getHalfExtents().y, bb:getHalfExtents().z

  local camOffsetAxisLocal = vec3(-0.75, -0.66, 0.1):normalized()
  local camLeftLocal = camOffsetAxisLocal:cross(vec3(0,0,1))
  local camUpLocal = camOffsetAxisLocal:cross(vec3(1,0,0))

  local camOffsetAxis = axis0 * (camOffsetAxisLocal.x) + axis1 * (camOffsetAxisLocal.y) + axis2 * (camOffsetAxisLocal.z)
  local camLeft = (axis0 * (camLeftLocal.x) + axis1 * (camLeftLocal.y) + axis2 * (camLeftLocal.z)):normalized()
  local camUp = (axis0 * (camUpLocal.x) + axis1 * (camUpLocal.y) + axis2 * (camUpLocal.z)):normalized()

  local bbUpperPoint = bbCenter - axis0 * 0 - axis1 * 0 + axis2 * (halfExtentsZ + 0.35)
  local bbForwardPoint = bbCenter - axis0 * 0 - axis1 * (halfExtentsY + 0.35) + axis2 * 0

  -- Calculate cam pos based on vertical fov
  local upperCamFovAngle = fov/2
  local upperCamFovDir = quatFromAxisAngle(camLeft, (upperCamFovAngle) / 180 * math.pi):__mul(-camOffsetAxis)

  local camPosBasedOnVertical = bbUpperPoint - upperCamFovDir * intersectsRay_Plane(bbUpperPoint, -upperCamFovDir, bbCenter + camOffsetAxis, camUp)

  -- Calculate cam pos based on horizontal fov
  local viewportHeight = nearPlane * math.tan((fov / 180 * math.pi) / 2)
  local viewportWidth = aspectRatio * viewportHeight
  local horizontalFOV = math.atan(viewportWidth / nearPlane) * 2
  horizontalFOV = horizontalFOV * 180 / math.pi

  local rightCamFovAngle = horizontalFOV/2
  local rightCamFovDir = quatFromAxisAngle(camUp, (rightCamFovAngle) / 180 * math.pi):__mul(-camOffsetAxis)

  local camPosBasedOnHorizontal = bbForwardPoint - rightCamFovDir * intersectsRay_Plane(bbForwardPoint, -rightCamFovDir, bbCenter + camOffsetAxis, camLeft)

  local finalCamPos
  -- Check which cam pos is further away and choose that one
  if camPosBasedOnVertical:distance(bbCenter) > camPosBasedOnHorizontal:distance(bbCenter) then
    finalCamPos = camPosBasedOnVertical
  else
    finalCamPos = camPosBasedOnHorizontal
  end

  local camRot = quatFromDir(bbCenter - (axis1 * halfExtentsY / 8) - finalCamPos)

  return finalCamPos, camRot
end

local function getListOfSelectedModels()
  local models = {}
  -- Add the selection
  for k, v in pairs(vehList) do
    if v[2][0] then
      table.insert(models, v[1])
    end
  end

  -- If no selection, then add the player's model by default
  if not models[1] then
    models = {getPlayerVehicle(0).JBeam}
  end

  return models
end

local function loadCameraConfig()
  cameraConfigFile = jsonReadFile("/settings/thumbnailCameraConfig.json")
  if not cameraConfigFile or not next(cameraConfigFile) then
    cameraConfigFile = {
      vehModels = {},
      vehConfigs = {}
    }
  end
end

local function saveCameraConfig()
  jsonWriteFile("/settings/thumbnailCameraConfig.json", cameraConfigFile, true)
end

local function getThumbnailResolution()
  local thumbnailPreset = presetResolutions[thumbnailDefaultValues.resolution - 1]
  return {thumbnailPreset[2], thumbnailPreset[3]}
end

local function getCurrentResolution()
  if not windowOpen[0] then
    return getThumbnailResolution()
  else
    local presetRes = presetResolutions[ctrls.currResolutionsPtr[0]]
    if not presetRes then -- if we are not using a preset, ie, if we are a custom res
      return {ctrls.imageResolution[0], ctrls.imageResolution[1]}
    else
      return {presetRes[2], presetRes[3]}
    end
  end
end

local function getCameraConfig(modelName, configName)
  if not cameraConfigFile then loadCameraConfig() end

  local cameraConfigs = {}
  if not cameraConfigFile.vehConfigs[modelName.."/"..configName] then
    cameraConfigFile.vehConfigs[modelName.."/"..configName] = {
      configCamEnabled = false,
      modelCamEnabled = false
    }
  end
  cameraConfigs.vehCamConfig = cameraConfigFile.vehConfigs[modelName.."/"..configName]

  if cameraConfigFile.vehModels[modelName] then
    cameraConfigs.modelCamConfig = cameraConfigFile.vehModels[modelName]
  end
  return cameraConfigs
end

local function getCameraOffsets()
  -- get the unit vectors for vehicle coordinate system
  local veh = getPlayerVehicle(0)
  local bb = veh:getSpawnWorldOOBB()
  local bbCenter = bb:getCenter()
  local x, y, z  = bb:getAxis(0), bb:getAxis(1), bb:getAxis(2)

  -- get vector from vehicle to camera
  local vehToCam = core_camera.getPosition() - veh:getPosition()

  -- un-rotate the camera rotation (remove vehicle rotation)
  local vehicleRotation = quatFromDir(bb:getAxis(1),  bb:getAxis(2))
  local vehicleRotationInverse = vehicleRotation:inversed()

  local unRotatedCamRotation = core_camera.getQuat() * vehicleRotationInverse

  -- split that offset into components using the vehicle unit vectors
  local xOff = x:dot(vehToCam)
  local yOff = y:dot(vehToCam)
  local zOff = z:dot(vehToCam)
  local offset = vec3(xOff, yOff, zOff)

  return offset, unRotatedCamRotation
end

local function rewindCameraOffsets(posOffset, rotOffset)
  local veh = getPlayerVehicle(0)
  local bb = veh:getSpawnWorldOOBB()
  local bbCenter = bb:getCenter()
  local x, y, z  = bb:getAxis(0), bb:getAxis(1), bb:getAxis(2)

  local offset = x * posOffset.x + y*posOffset.y + z*posOffset.z
  local finalPos = offset+veh:getPosition()

  -- rotate the camera rotation by the car rotation
  local vehicleRotation = quatFromDir(bb:getAxis(1),  bb:getAxis(2))
  local finalRot = rotOffset * vehicleRotation

  return finalPos, finalRot
end


local function getFinalCameraPosAndRotForVehicle(vehicle, configName)
  local cameraConfigs = getCameraConfig(vehicle.jbeam, configName)
  local p, r

  if cameraConfigs.vehCamConfig then
    if cameraConfigs.vehCamConfig.configCamEnabled then
      p, r = rewindCameraOffsets(cameraConfigs.vehCamConfig.cameraConfig.posOffset, cameraConfigs.vehCamConfig.cameraConfig.rotOffset)
      return p, r, "config camera"
    elseif cameraConfigs.vehCamConfig.modelCamEnabled then
      p, r = rewindCameraOffsets(cameraConfigs.modelCamConfig.cameraConfig.posOffset, cameraConfigs.modelCamConfig.cameraConfig.rotOffset)
      return p, r, "model camera"
    end
  end

  local currRes = getCurrentResolution()
  p, r = frameVehicle(vehicle, thumbnailConfig.fov, thumbnailConfig.nearPlane, currRes[1] / currRes[2])
  return p, r, "procedural camera"
end

local function takeThumbnail(options)
  local currRes = getCurrentResolution()
  local camPos, camRot, camName = getFinalCameraPosAndRotForVehicle(options.vehicle, options.configName)

  local renderViewOptions = {
    renderViewName = "thumbnail",
    screenshotDelay = 0,
    resolution = vec3(currRes[1], currRes[2], 0),
    rot = camRot,
    pos = camPos + vec3(0,0,-0.2),
    fov = thumbnailConfig.fov,
    nearPlane = thumbnailConfig.nearPlane,
    filename = options.filepath
  }

  -- Take screenshot
  log('I', '', "Saved screenshot:" .. renderViewOptions.filename)

  render_renderViews.takeScreenshot(renderViewOptions)
  yieldSec(coroutine.yield, 0.75)


  if options.isDefaultConfig then
    FS:copyFile(options.filepath .. thumbnailConfig.fileEnding, 'vehicles/' .. options.modelKey .. '/default' .. thumbnailConfig.fileEnding)
    log('I', logTag, "saved default:" .. options.modelKey .. thumbnailConfig.fileEnding)
  end
  return camName
end

local function startWork(workOptions)
  -- Sanitizing workOptions
  if not workOptions or type(workOptions) ~= 'table' then
    workOptions = {
      selection = "selectedModels", -- "selectedModels" or currConfigName
      onlyMissingThumbnails = false,
    }
  end
  if not workOptions.selection then workOptions.selection = "selectedModels" end

  -- reset reviewData for each run
  reviewData = {
    onlyMissingThumbnails = workOptions.onlyMissingThumbnails,
    selection = workOptions.selection,
    configs = {}
  }

  if workerCoroutine then
    log('E', "startWork", "coroutine already exist")
    return
  end


  -- main thing
  workerCoroutine = coroutine.create(function()
    log('I', '', "Starting thumbnail work coroutine")
    trySetDynamicReflections(false)

    local listOfSelectedModels = getListOfSelectedModels()

    forceQuit = false

    be:setPhysicsSpeedFactor(2)

    if workOptions.selection == "selectedModels" then -- take thumbnails of the selected models
      for _, modelName in pairs(listOfSelectedModels) do
        for _, configData in pairs(core_vehicles.getModel(modelName).configs) do
          if forceQuit then
            be:setPhysicsSpeedFactor(0)
            return
          end

          -- generate the thumbnail filename
          local folder = "vehicles/"
          local vehName = configData.model_key .. "/" .. configData.key

          if windowOpen[0] and ctrls.currOuputDestinationPtr[0] == 1 then
            folder = "screenshots/showroom/"
          end
          local filepath = folder .. vehName ..thumbnailConfig.fileEnding

          local skip = workOptions.onlyMissingThumbnails and windowOpen[0] and ctrls.currOuputDestinationPtr[0] == 0 and FS:fileExists(filepath)

          local camName
          local status
          if not skip then -- replace vehicle
            core_vehicles.replaceVehicle(configData.model_key, { config = configData.key, licenseText = "BeamNG"})
            yieldSec(coroutine.yield, 0.7)

            local newVehicle = getPlayerVehicle(0)

            newVehicle:queueLuaCommand("input.event('parkingbrake', 1, 1)")
            newVehicle:queueLuaCommand("input.event('throttle', 0, 2)")
            newVehicle:queueLuaCommand("controller.mainController.setEngineIgnition(false)")

            camName = takeThumbnail(
              {
                vehicle = newVehicle,
                configName = currConfigName,
                filepath = filepath,
                modelKey = configData.model_key,
                isDefaultConfig = configData.is_default_config
              }
            )
            status = "done"
          else
            status = "skipped"
          end
          table.insert(reviewData.configs, {vehName = vehName, camName = camName, status = status, thumbnailPath = filepath})
        end
      end
    else -- take thumbnail of the current vehicle only
      local playerVehicle = getPlayerVehicle(0)
      local vehManager = extensions.core_vehicle_manager
      local playerVehicleData = vehManager.getPlayerVehicleData()
      local filepath = playerVehicleData.vehicleDirectory .. workOptions.selection..thumbnailConfig.fileEnding

      local camName = takeThumbnail({vehicle = playerVehicle, filepath = filepath, configName = workOptions.selection})

      table.insert(reviewData.configs, {camName = camName, vehName = playerVehicleData.vehicleDirectory, status = "done", thumbnailPath = filepath})
    end

    be:setPhysicsSpeedFactor(0)

    -- when the job is finished
    runDone = true
    if windowOpen[0] and ctrls.reloadUIOnJobFinished[0] then
      reloadUI()
    end
  end)
end

local function selectPlayerVehicle()
  local playerVehicle = getPlayerVehicle(0)
  if playerVehicle then
    for _,v in ipairs(vehList) do
      v[2][0] = v[1] == playerVehicle.JBeam
    end
  else
    log("E", "selectCurVeh", "Failed to get current vehicle")
  end
end

local function resetToDefaultValues()
  ctrls.currOuputDestinationPtr = im.IntPtr(thumbnailDefaultValues.ouputDestination - 1)
  ctrls.currResolutionsPtr = im.IntPtr(thumbnailDefaultValues.resolution - 1)
  local currRes = getCurrentResolution()
  ctrls.imageResolution = im.ArrayInt(3)
  ctrls.imageResolution[0] = currRes[1]
  ctrls.imageResolution[1] = currRes[2]
end

local previousFov
local isCameraSet = true
local function setCamera(p, r)
  previousFov = core_camera.getFovDeg()
  core_camera.setByName(0, "free")
  core_camera.setPosRot(0, p.x, p.y, p.z, r.x, r.y, r.z, r.w)
  core_camera.setFOV(0, thumbnailConfig.fov)
  isCameraSet = true
end

local function previewCamera(p, r)
  if currentlyPreviewingCamera then return end
  setCamera(p, r)

  currentlyPreviewingCamera = true
end

local function resetCamera()
  core_camera.setByName(0, "orbit")
  core_camera.setFOV(0, previousFov or 60)
  isCameraSet = false
end

local function stopPreviewCamera()
  if not currentlyPreviewingCamera then return end
  resetCamera()
  currentlyPreviewingCamera = false
end

local function setDimHelper(w, h)
  local vm = GFXDevice.getVideoMode()
  if not plRes then
    plRes = {vm.width, vm.height}
  end
  if vm.width == w and vm.height == h and vm.displayMode == "Borderless" then
    -- nothing to change
    return
  end
  log('I', '', "requesting new video mode")
  vm.width = w
  vm.height = h
  vm.displayMode = "Borderless"
  GFXDevice.setVideoMode(vm)
end

local lastFrameImageResolution = {}
local function onUpdate(dtReal, dtSim, dtRaw)
  if windowOpen[0] ~= true then return end

  local isRunning = workerCoroutine ~= nil

  im.SetNextWindowSize(initialWindowSize, im.Cond_FirstUseEver)

  if im.Begin("Vehicle Screenshot Creator (WIP)", windowOpen) then
    local partConfig = getPlayerVehicle(0).partConfig

    if not string.endswith(partConfig, ".pc") then
      im.Text("ERROR: Player vehicle does not originate from a part config file (.pc). Please choose a vehicle that does.")
      goto continue
    end

    currConfigName = sanitizeVehConfigName(partConfig)
    currModelName = getPlayerVehicle(0).jbeam

    -- we set the default values
    if not ctrls.currOuputDestinationPtr then
      resetToDefaultValues()
    end


    if isRunning then
      if editor.uiIconImageButton(editor.icons.stop, imVec32x32, im.ImColorByRGB(0,255,0,255).Value, nil, nil) then
        forceQuit = true
      end
      if im.IsItemHovered() then im.BeginTooltip() im.Text("Stop") im.EndTooltip() end
    else
      if editor.uiIconImageButton(editor.icons.play_arrow, imVec32x32, im.ImColorByRGB(0,255,0,127).Value, nil, nil) then
        startWork({selection = "selectedModels", onlyMissingThumbnails = ctrls.generateMissingThumbnailsOnly[0]})
      end
      if im.IsItemHovered() then im.BeginTooltip() im.Text("Run (".. getSelectedVehCount() .. ") selected models (See 'Selection'tab). If none are selected, player's vehicle model will run by default") im.EndTooltip() end

      im.SameLine()

      if editor.uiIconImageButton(editor.icons.play_arrow, imVec32x32, imVec4Yellow, nil, nil) then
        startWork({selection = currConfigName})
      end
      if im.IsItemHovered() then im.BeginTooltip() im.Text("Only update thumbnail of current config : '"..currConfigName .."' (Will not update the config itself!)") im.EndTooltip() end

      im.Checkbox("Generate missing thumbnails only", ctrls.generateMissingThumbnailsOnly)
      if im.IsItemHovered() then im.BeginTooltip() im.Text("Checks if the thumbnail file is missing. A blank/white thumbnail is not a missing thumbnail") im.EndTooltip() end
      im.SameLine()
      im.Checkbox("Reload UI when run is finished", ctrls.reloadUIOnJobFinished)
      if im.IsItemHovered() then im.BeginTooltip() im.Text("If not, opening the vehicle menu after updating the thumbnails, won't show the new thumbnails") im.EndTooltip() end

      im.Dummy(im.ImVec2(1, 5))
    end

    im.PopStyleColor()


    if isRunning then im.BeginDisabled() end
    if im.BeginTabBar("main Menu##") then
      if im.BeginTabItem('Output') then
        if isDefaultConfig() then
          im.BeginDisabled()
        end
        if editor.uiIconImageButton(editor.icons.undo, imVec32x32, nil, nil, nil) then
          resetToDefaultValues()
        end
        im.tooltip("Restore settings to thumbnail's")
        if isDefaultConfig() then
          im.EndDisabled()
        end

        -- this build the string from the presets to feed the Combo2
        local s = ""
        for _, r in ipairs(presetOutputDestinations) do
          s = s .. r .. '\0'
        end
        ctrls.outputDestinationsStr = s
        im.Combo2("Output destination", ctrls.currOuputDestinationPtr, ctrls.outputDestinationsStr)

        im.Dummy(im.ImVec2(1, 20))

        -- this build the string from the presets to feed the Combo2
        local s = 'custom\0'
        for _, r in ipairs(presetResolutions) do
          s = s .. r[1] .. ' - ' .. r[2] .. ' x ' .. r[3] .. '\0'
        end
        ctrls.presetResolutionsComboStr = s .. '\0'
        im.Combo2("Common Resolutions", ctrls.currResolutionsPtr, ctrls.presetResolutionsComboStr)

        -- make sure the final image resolution is updated
        if ctrls.currResolutionsPtr[0] > 0 then
          local preset = presetResolutions[ctrls.currResolutionsPtr[0]]
          ctrls.imageResolution[0] = preset[2]
          ctrls.imageResolution[1] = preset[3]
        end

        im.Checkbox("Keep aspect ratio on manual resolution change", ctrls.keepAspectRatio)

        local recalculate = false

        im.InputFloat("##mul", ctrls.multiplyRes)
        im.SameLine()
        if im.Button("Multiply current res") then
          if ctrls.multiplyRes[0] > 0 then
            ctrls.imageResolution[1] = ctrls.imageResolution[1] * ctrls.multiplyRes[0]
            ctrls.imageResolution[0] = ctrls.imageResolution[0] * ctrls.multiplyRes[0]

            recalculate = true
          end
        end

        if im.InputInt2("Final image resolution", ctrls.imageResolution, im.InputTextFlags_EnterReturnsTrue) then

          -- check if we want to keep the aspect ratio
          if ctrls.keepAspectRatio[0] then
            if lastFrameImageResolution[0] == ctrls.imageResolution[0] then
              local ratio = ctrls.imageResolution[1] / lastFrameImageResolution[1]
              ctrls.imageResolution[0] = lastFrameImageResolution[0] * ratio
            elseif lastFrameImageResolution[1] == ctrls.imageResolution[1] then
              local ratio = ctrls.imageResolution[0] / lastFrameImageResolution[0]
              ctrls.imageResolution[1] = lastFrameImageResolution[1] * ratio
            end
          end

          recalculate = true
        end


        if recalculate then
          local found = false
          -- check if custom resolution matches one of the presets, otherwise use custom
          for i, r in ipairs(presetResolutions) do
            if ctrls.currResolutionsPtr and ctrls.imageResolution[0] == r[2] and ctrls.imageResolution[1] == r[3] then
              ctrls.currResolutionsPtr[0] = i
              found = true
              break
            end
          end
          if not found then
            ctrls.currResolutionsPtr[0] = 0 -- custom
          end
        end

        if im.Button(ctrls.resolutionToggled and "Reset resolution" or "Preview resolution") then
          ctrls.resolutionToggled = not ctrls.resolutionToggled
          if ctrls.resolutionToggled then
            setDimHelper(ctrls.imageResolution[0], ctrls.imageResolution[1])
          else
            setDimHelper(plRes[1], plRes[2])
          end
        end
        im.SameLine()
        if im.Button("Toggle UI apps") then
          ctrls.uiToggled = not ctrls.uiToggled
          if ctrls.uiToggled then
            core_gamestate.setGameState('freeroam', 'freeroam', 'freeroam')
          else
            core_gamestate.setGameState('freeroam', {}, 'freeroam')
          end
        end
        if not isThumbnailConfigGood() then
          editor.uiIconImage(editor.icons.warning, imVec24x24, imVec4Yellow)
          im.SameLine()
          im.PushStyleColor2(im.Col_Text, imVec4Yellow)
          im.TextWrapped(string.format("Correct size for vehicles thumbnails are 500 * 281 and you have chosen %i * %i", getCurrentResolution()[1], getCurrentResolution()[2]))
          im.PopStyleColor()
        else
          im.Dummy(im.ImVec2(1, 20))
        end

        if not ctrls.superSamplingPtr then
          ctrls.superSamplingPtr = im.IntPtr(1)
          workConfig.superSampling = 0
        end
        -- if im.SliderInt("Supersampling", ctrls.superSamplingPtr, 0, 64) then
        --   workConfig.superSampling = ctrls.superSamplingPtr[0]
        -- end

        ----------------------------------------------------------------------------
        local s = math.sqrt(ctrls.superSamplingPtr[0])
        local x = ctrls.imageResolution[0]
        local y = ctrls.imageResolution[1]

        -- this is the same math as in the c++ side
        local maxSize = math.max(x, y) * math.sqrt(s)
        if x > y then
          y = y * (maxSize / x)
          x = maxSize
        else
          x = x * (maxSize / y)
          y = maxSize
        end
        x = math.floor(x)
        y = math.floor(y)

        im.Dummy(im.ImVec2(1, 20))

        im.TextUnformatted('Final resolution: ' .. tostring(x) .. ' x ' .. tostring(y))
        im.TextUnformatted('Megapixel = ' .. string.format('%0.2f', x * y / 1000000))
        local rawSize = x * y * 3 -- RGB = 3 byte
        im.TextUnformatted('Raw image size = ' .. (bytes_to_string(rawSize)))
        im.TextUnformatted('Estimated JPG file size = ' .. (bytes_to_string(rawSize * 0.14)))

        im.EndTabItem()
      end
      if im.BeginTabItem("Models selection") then
        if im.SmallButton("Player Vehicle Model") then
          selectPlayerVehicle()
        end
        im.SameLine()
        if im.SmallButton("Select All") then
          for _,v in ipairs(vehList) do
            v[2][0] = true
          end
        end
        im.SameLine()
        if im.SmallButton("Unselect All") then
          for _,v in ipairs(vehList) do
            v[2][0] = false
          end
        end
        im.SameLine()
        if im.SmallButton("Invert Selection") then
          for _,v in ipairs(vehList) do
            v[2][0] = not v[2][0]
          end
        end

        local halfWidth = im.GetContentRegionAvailWidth() / 2
        if im.BeginChild1("unselectedSection", im.ImVec2(halfWidth, 0), true) then
          im.Text("Unselected Models")
          if im.BeginChild1("unselectedVehs", im.ImVec2(0,0), true) then
            if vehList then
              for _,v in ipairs(vehList) do
                if not v[2][0] then
                  im.Selectable2(v[3] .. " (" ..v[1] .. ")",v[2])
                end
              end
            end
          end
        end
        im.EndChild()
        im.EndChild()

        im.SameLine()
        if im.BeginChild1("selected", im.ImVec2(halfWidth, 0), true) then
          im.Text("Selected Models")
          if im.BeginChild1("selectedVehs", im.ImVec2(0,0), true) then
            if vehList then
              for _,v in ipairs(vehList) do
                if v[2][0] then
                  im.Selectable2(v[3] .. " (" ..v[1] .. ")",v[2])
                end
              end
            end
          end
        end

        im.EndChild()
        im.EndChild()

        im.EndTabItem()
      end
      if im.BeginTabItem("Manual controls") then

        im.TextWrapped("This tab is used to override the procedural camera placement per model and/or config, during the generation of thumbnails")
        im.TextWrapped("A vehicle config's camera will override its model camera")

        im.Dummy(im.ImVec2(1, 10))

        im.TextWrapped("The ")
        local _, _, camName = getFinalCameraPosAndRotForVehicle(getPlayerVehicle(0), currConfigName)

        im.PushStyleColor2(im.Col_Text, imVec4Green)
        im.SameLine()
        im.TextWrapped(camName)
        im.PopStyleColor()
        im.tooltip("To disable procedural camera placement during thumbnail generation, enable either one of the manual camera placement below")
        im.SameLine()
        im.TextWrapped(" will be used for the spawned config.")
        im.Dummy(im.ImVec2(1, 10))


        if currConfigName == "default" then
          im.PushStyleColor2(im.Col_Text, imVec4Red)
          editor.uiIconImage(editor.icons.error, imVec24x24, imVec4Red)
          im.SameLine()
          im.TextWrapped("Current vehicle is your own 'default' vehicle. Spawn a vehicle from the vehicle selector (Will be fixed)")
          im.PopStyleColor()
        else
          if im.Button("Preview final thumbnail") then end
          if im.IsItemHovered() then
            local p, r = getFinalCameraPosAndRotForVehicle(getPlayerVehicle(0), currConfigName)
            previewCamera(p, r)
          else
            stopPreviewCamera()
          end
          im.Dummy(im.ImVec2(1, 10))

          local imguiHeight = 230
          if im.BeginChild1("Parent parent", im.ImVec2(0, imguiHeight), true) then
            if im.BeginChild1("Config list", im.ImVec2(im.GetContentRegionAvailWidth() / 2, 0), nil) then
              if im.BeginTable('Model configs', 4, nil) then
                im.TableSetupColumn("Config name",nil, 16)
                im.TableSetupColumn("Config cam",nil, 7)
                im.TableSetupColumn("Model cam",nil, 7)
                im.TableSetupColumn("Spawn",nil, 7)
                im.TableNextColumn()
                im.Text("Config name")
                im.TableNextColumn()
                im.Text("Config cam")
                im.TableNextColumn()
                im.Text("Model cam")
                im.TableNextColumn()
                im.TableNextColumn()

                local isCurrent
                for _, configData in pairs(core_vehicles.getModel(currModelName).configs) do
                  isCurrent = configData.key == currConfigName
                  local cameraConfigs = getCameraConfig(configData.model_key, configData.key)
                  im.Text(configData.key)
                  if isCurrent then
                    im.SameLine()
                    im.Text("(Current)")
                  end
                  im.TableNextColumn()
                  if cameraConfigs.vehCamConfig.cameraConfig then
                    local enabledPtr = im.BoolPtr(cameraConfigs.vehCamConfig.configCamEnabled)
                    if im.Checkbox("##"..configData.key, enabledPtr) then
                      cameraConfigs.vehCamConfig.configCamEnabled = enabledPtr[0]
                      saveCameraConfig()
                    end
                    im.SameLine(42)
                    if editor.uiIconImageButton(editor.icons.switch_camera, imVec24x24, im.ImColorByRGB(0,255,0,127).Value, nil, nil) then
                      local p,r = rewindCameraOffsets(cameraConfigs.vehCamConfig.cameraConfig.posOffset, cameraConfigs.vehCamConfig.cameraConfig.rotOffset)
                      setCamera(p, r)
                    end
                    im.tooltip(string.format("Click to set camera to '%s' config's camera", configData.key))
                  else
                    im.Text("None")
                  end
                  im.TableNextColumn()
                  if cameraConfigs.modelCamConfig then
                    local enabledPtr = im.BoolPtr(cameraConfigs.vehCamConfig.modelCamEnabled)
                    if im.Checkbox("##"..configData.key..".", enabledPtr) then
                      cameraConfigs.vehCamConfig.modelCamEnabled = enabledPtr[0]
                      saveCameraConfig()
                    end
                  else
                    im.Text("None")
                  end
                  im.TableNextColumn()
                  if not isCurrent then
                    if im.Button("Spawn##"..configData.key) then
                      core_vehicles.replaceVehicle(configData.model_key, { config = configData.key, licenseText = "BeamNG"})
                    end
                  end
                  im.TableNextColumn()
                end
                im.EndTable()
              end
            end
            im.EndChild()
            im.SameLine()

            local cameraConfigs = getCameraConfig(currModelName, currConfigName)
            if im.BeginChild1("Config custom camera", im.ImVec2(im.GetContentRegionAvailWidth() / 2, 0), true) then
              if cameraConfigs.vehCamConfig and cameraConfigs.vehCamConfig.cameraConfig then
                editor.uiIconImage(editor.icons.check, imVec24x24, imVec4Green)
                im.SameLine()
                im.TextWrapped("Current vehicle config has a custom camera")
                if im.Button("Set camera to config camera") then
                  local p,r = rewindCameraOffsets(cameraConfigs.vehCamConfig.cameraConfig.posOffset, cameraConfigs.vehCamConfig.cameraConfig.rotOffset)
                  setCamera(p, r)
                end

                local enabledPtr = im.BoolPtr(cameraConfigs.vehCamConfig.configCamEnabled)
                if im.Checkbox("Enabled", enabledPtr) then
                  cameraConfigs.vehCamConfig.configCamEnabled = enabledPtr[0]
                  saveCameraConfig()
                end
                im.tooltip("If enabled, will use the current vehicle config's custom camera during thumbnail generation.")
              else
                editor.uiIconImage(editor.icons.error, imVec24x24, imVec4Red)
                im.SameLine()
                im.TextWrapped("Current vehicle config doesn't have a custom camera")
              end
              if im.Button("Overwrite config camera") then
                local s = currModelName.."/"..currConfigName
                local posOffset, rotOffset = getCameraOffsets()
                cameraConfigFile.vehConfigs[s] = {
                  cameraConfig = {posOffset=posOffset, rotOffset=rotOffset}
                }
                if not cameraConfigFile.vehConfigs[s] then
                  cameraConfigFile.vehConfigs[s] = {}
                end
                cameraConfigFile.vehConfigs[s].modelCamEnabled = false
                cameraConfigFile.vehConfigs[s].configCamEnabled = true
                saveCameraConfig()
              end
              im.tooltip("Will save the current camera's position and rotation for current vehicle config only")
            end
            im.EndChild()
            im.SameLine()
            if im.BeginChild1("Model custom camera", nil, true) then
              if cameraConfigs.vehCamConfig and cameraConfigs.vehCamConfig.configCamEnabled then
                im.TextWrapped("Vehicle config's manual camera overrides the vehicle model camera. Since the current vehicle config camera is enabled, this window is deactivated")
              else
                if cameraConfigs.modelCamConfig then
                  editor.uiIconImage(editor.icons.check, imVec24x24, imVec4Green)
                  im.SameLine()
                  im.TextWrapped("Current model has a custom camera")
                  if im.Button("Set camera to model camera") then
                    local p,r = rewindCameraOffsets(cameraConfigs.modelCamConfig.cameraConfig.posOffset, cameraConfigs.modelCamConfig.cameraConfig.rotOffset)
                    setCamera(p, r)
                  end

                  local enabledPtr = im.BoolPtr(cameraConfigs.vehCamConfig.modelCamEnabled)
                  if im.Checkbox("Enabled", enabledPtr) then
                    cameraConfigs.vehCamConfig.modelCamEnabled = enabledPtr[0]
                    saveCameraConfig()
                  end
                  im.tooltip("If enabled, will use the current vehicle model's custom camera during thumbnail generation.")

                else
                  editor.uiIconImage(editor.icons.error, imVec24x24, imVec4Red)
                  im.SameLine()
                  im.TextWrapped("Current model doesn't have a custom camera")
                end
                if im.Button("Overwrite model camera") then
                  local s = currModelName.."/"..currConfigName
                  local posOffset, rotOffset = getCameraOffsets()
                  cameraConfigFile.vehModels[currModelName] = {
                    cameraConfig = {posOffset=posOffset, rotOffset=rotOffset}
                  }
                  if not cameraConfigFile.vehConfigs[s] then
                    cameraConfigFile.vehConfigs[s] = {}
                  end
                  cameraConfigFile.vehConfigs[s].modelCamEnabled = true
                  cameraConfigFile.vehConfigs[s].configCamEnabled = false
                  saveCameraConfig()
                end
                im.tooltip("Will save the current camera's position and rotation for current model")
              end
            end
            im.EndChild()
          end
          im.EndChild()

            im.Text("Manual controls")
            if im.Button(ctrls.drivingEnabled and "Disable driving" or "Enable driving") then
              ctrls.drivingEnabled = not ctrls.drivingEnabled

              for _, mapName in ipairs(vehicleActionMaps) do
                local map = scenetree.findObject(mapName)
                if map then
                  map:setEnabled(ctrls.drivingEnabled)
                end
              end

            end
            if im.IsItemHovered() then im.BeginTooltip() im.Text("Enable or disable vehicle's controls") im.EndTooltip() end
            if im.Button("Set camera to procedural") then
              local currRes = getCurrentResolution()
              local p, r = frameVehicle(getPlayerVehicle(0), thumbnailConfig.fov, thumbnailConfig.nearPlane, currRes[1] / currRes[2])
              setCamera(p, r)
            end
            if isCameraSet then
              if im.Button("Revert camera") then
                resetCamera()
              end
              if editor.uiSliderFloat("Camera Speed", ctrls.camSpeedPtr, 2, 100, "%.1f") then
                core_camera.setSpeed(ctrls.camSpeedPtr[0])
              end
            end
        end
        im.EndTabItem()
      end

      if runDone then im.PushStyleColor2(im.Col_Text, imVec4Red) end
      if im.BeginTabItem("Last run review") then
        if im.Button("Open user's vehicle folder") then
          if not fileExistsOrNil('/vehicles/') then  -- create dir if it doesnt exist
            FS:directoryCreate('/vehicles/', true)
          end
          Engine.Platform.exploreFolder('/vehicles/')
        end
        im.SameLine()
        if im.Button("Open user's screenshot/showroom folder") then
          if not fileExistsOrNil('/screenshots/showroom/') then  -- create dir if it doesnt exist
            FS:directoryCreate('/screenshots/showroom/', true)
          end
          Engine.Platform.exploreFolder('/screenshots/showroom/')
        end

        if reviewData and next(reviewData) then
          local totalSkipped = 0
          local totalDone = 0

          local halfWidth = im.GetContentRegionAvailWidth() / 2
          if im.BeginChild1("Vehicle list info", im.ImVec2(halfWidth, 0), true) then
            im.Text("Thumbnail status list : ")
            im.Dummy(im.ImVec2(1, 10))

            if im.BeginTable('Status list', 4, nil) then
              im.TableSetupColumn("Veh name",nil, 11)
              im.TableSetupColumn("Status",nil, 6)
              im.TableSetupColumn("Camera",nil, 12)
              im.TableSetupColumn("Preview",nil, 7)
              im.TableNextColumn()
              im.Text("Veh name")
              im.TableNextColumn()
              im.Text("Status")
              im.TableNextColumn()
              im.Text("Camera")
              im.TableNextColumn()
              im.TableNextColumn()
              for _, data in ipairs(reviewData.configs) do
                im.Text(data.vehName)
                im.TableNextColumn()

                im.Text(data.status)
                im.TableNextColumn()

                im.Text(data.camName or "")
                im.TableNextColumn()

                if data.status == "skipped" then
                  totalSkipped = totalSkipped + 1
                elseif data.status == "done" then
                  totalDone = totalDone + 1
                end

                im.BeginDisabled()
                if im.Button("Preview") then
                end
                if im.IsItemHovered() then
                  local thumb = imguiUtils.texObj(data.thumbnailPath)
                  im.BeginTooltip()
                  im.Image(thumb.texId, thumb.size, im.ImVec2(0, 0), im.ImVec2(1, 1))
                  im.EndTooltip()
                end
                im.TableNextColumn()
                im.EndDisabled()

              end
              im.EndTable()
            end
          end
          im.EndChild()

          im.SameLine()

          if im.BeginChild1("Run info", im.ImVec2(halfWidth, 0), true) then

            im.Text("General info : ")
            im.Dummy(im.ImVec2(1, 10))
            im.Text("Generate missing thumbnails only : " .. (reviewData.onlyMissingThumbnails and "Yes" or "No"))
            im.Text("Total done : " .. totalDone)
            im.Text("Total skipped : " .. totalSkipped)
          end
          im.EndChild()
        else
          im.Text("There is no last run to review")
        end

        im.EndTabItem()

        runDone = false
      end
      if runDone then im.PopStyleColor() end
    end
    im.EndTabBar()

    if isRunning then im.EndDisabled() end
  end

  ::continue::
  im.End()

  lastFrameImageResolution[0] = ctrls.imageResolution[0]
  lastFrameImageResolution[1] = ctrls.imageResolution[1]
end

local function onExtensionLoaded()
  populateVehGui()

  -- save the user setting to revert it back when leaving the showroom
  if isInShowroom() then
    userDefinedDynamicReflections = settings.getValue("GraphicDynReflectionEnabled")
  end
  trySetDynamicReflections(false)
end

local function onExtensionUnloaded()
  trySetDynamicReflections(userDefinedDynamicReflections)

  log('I', '', "Module unloaded")
end

local function openWindow()
  extensions.editor_main.initializeModules()
  windowOpen[0] = true
end

local function onSerialize()
  return {
    windowOpen = windowOpen[0],
    userDefinedDynamicReflections = userDefinedDynamicReflections,
    generateMissingThumbnailsOnly = ctrls.generateMissingThumbnailsOnly[0],
    reloadUIOnJobFinished = ctrls.reloadUIOnJobFinished[0],
    resolutionPtr = (ctrls.currResolutionsPtr and ctrls.currResolutionsPtr[0]) or thumbnailDefaultValues.resolution,
    reviewData = reviewData,
    runDone = runDone,
    currentlyPreviewingCamera = currentlyPreviewingCamera,
    previousFov = previousFov,
    isCameraSet = isCameraSet,
    plRes = plRes,
    keepAspectRatio = ctrls.keepAspectRatio[0]
  }
end

local function onDeserialized(data)
  if data.windowOpen ~= nil then
    resetToDefaultValues()

    windowOpen[0] = data.windowOpen
    userDefinedDynamicReflections = data.userDefinedDynamicReflections
    reviewData = data.reviewData or {}
    ctrls.generateMissingThumbnailsOnly[0] = data.generateMissingThumbnailsOnly
    ctrls.currResolutionsPtr[0] = data.resolutionPtr
    ctrls.reloadUIOnJobFinished[0] = data.reloadUIOnJobFinished
    runDone = data.runDone
    currentlyPreviewingCamera = data.currentlyPreviewingCamera
    previousFov = data.previousFov
    isCameraSet = data.isCameraSet
    plRes = data.plRes
    ctrls.keepAspectRatio[0] = data.keepAspectRatio
  end
end

M.onPreRender = onPreRender
M.onUpdate = onUpdate
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.openWindow = openWindow
M.startWork = startWork
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.frameVehicle = frameVehicle
return M
