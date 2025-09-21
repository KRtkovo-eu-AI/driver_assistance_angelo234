-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local screenshotPath = 'screenshots/'

local function getMetadataJson()
  local res = {
    versionb = beamng_versionb,
    versiond = beamng_versiond,
    windowtitle = beamng_windowtitle,
    buildtype = beamng_buildtype,
    buildinfo = beamng_buildinfo,
    arch = beamng_arch,
    buildnumber = beamng_buildnumber,
    shipping_build = shipping_build,
  }
  res.level = getMissionFilename()
  if extensions.core_gamestate.state.state then
    res.gameState = extensions.core_gamestate.state.state
  end

  if Steam and Steam.isWorking and Steam.accountID ~= "" then
    res.steamIDHash = tostring(hashStringSHA1(Steam.accountID))
    res.steamPlayerName = Steam.playerName
  end

  local pos = core_camera.getPosition()
  local rot = core_camera.getQuat()
  if pos.x ~= 0 or pos.y ~=0 or pos.z ~= 0 then
    res.cameraPos = {pos.x, pos.y, pos.z}
    res.cameraRot = {rot.x, rot.y, rot.z, rot.w}
  end

  res.os = Engine.Platform.getOSInfo()
  res.cpu = Engine.Platform.getCPUInfo()
  res.gpu = Engine.Platform.getGPUInfo()
  if res.gpu then
    res.gpu.vulkanEnabled = Engine.Render.getAdapterType() == "Vulkan"
  end
  if core_environment then
    res.tod = core_environment.getTimeOfDay()
  end
  extensions.hook('onCollectScreenshotMetadata', res)
  return jsonEncode(res)
end

local function doScreenshot(batchTag, upload, path, ext)
  -- find the next available screenshot filename
  local counter = 0

  local finalPath, format, filepath, filename
  if path and ext then
    finalPath = path
    format = ext
    upload = nil
    batchTag = nil
  else
    format = settings.getValue("screenshotFormat")

    filename = ''
    local filename_without_ext = ''
    filepath = ''
    local screenPath = screenshotPath .. tostring(getScreenShotFolderString())
    if not FS:directoryExists(screenPath) then
      FS:directoryCreate(screenPath)
    end
    repeat
      filename_without_ext = 'screenshot_' .. tostring(getScreenShotDateTimeString())
      if counter > 0 then
        filename_without_ext = filename_without_ext .. '_' .. tostring(counter)
      end
      filename = filename_without_ext .. '.' ..format
      filepath = screenPath .. '/' .. filename
      counter = counter + 1
    until not FS:fileExists(filepath)
    finalPath = screenPath .. '/' .. filename_without_ext
  end

   -- TODO: fix metadata to include level, vehicles, etc


  createScreenshot(finalPath, format, 1, 1, 0, false, upload, getMetadataJson())
end

local function publish(batchTag)
  if settings.getValue('onlineFeatures') ~= 'enable' then
    log('E', 'screenshot.publish', 'screenshot publishing disabled because online features are disabled')
    guihooks.trigger("toastrMsg", {type="warning", title="Error uploading screenshot", msg="Online features are disabled. This setting must be enbled to upload screenshots to BeamNG's media server"})
    return
  end
  doScreenshot(batchTag, true)
end

local function doSteamScreenshot()
  if settings.getValue('onlineFeatures') ~= 'enable' then
    log('E', 'screenshot.publish', 'screenshot publishing disabled because online features are disabled')
    return
  end
  Steam.triggerScreenshot()
end

local function openScreenshotsFolderInExplorer()
  if not fileExistsOrNil('/screenshots/') then  -- create dir if it doesnt exist
    FS:directoryCreate('/screenshots/', true)
  end
   Engine.Platform.exploreFolder('/screenshots/')
end

local function _screenshot(superSampling, tiles, overlap, highest, downsample )
  M.screenshotHighest = highest

  -- set the new values
  if M.screenshotHighest then
    -- log('I','screenshot', "Setting new render parameters ...")
    -- save current values
    M.sc_detailAdjustSaved = TorqueScriptLua.getVar("$pref::TS::detailAdjust")
    M.sc_lodScaleSaved = TorqueScriptLua.getVar("$pref::Terrain::lodScale")
    M.sc_GroundCoverScaleSaved =  getGroundCoverScale()

    local sunsky = scenetree.findObject("sunsky")
    if sunsky then
      M.sc_sunskyTexSizeSaved = sunsky.texSize
      M.sc_sunskyShadowDistanceSaved = sunsky.shadowDistance
      sunsky.texSize = 8192         -- 1024 -- default value on our levels, high is better
      sunsky.shadowDistance = 8000  -- 1600; -- default for gridmap, high is better
    end

    TorqueScriptLua.setVar("$pref::TS::detailAdjust", 20) -- 1.5; -- high is better
    TorqueScriptLua.setVar("$pref::Terrain::lodScale", 0.001) -- 0.75; -- lower is better
    setGroundCoverScale(8) -- 1 -- bigger is better
    flushGroundCoverGrids()
  end


  local screenshotFolderString = getScreenShotFolderString()
  local path = string.format("screenshots/%s", screenshotFolderString)
  if not FS:directoryExists(path) then FS:directoryCreate(path) end
  local screenshotDateTimeString = getScreenShotDateTimeString()
  local subFilename = string.format("%s/screenshot_%s", path, screenshotDateTimeString)
  local screenshotFormat = settings.getValue("screenshotFormat")

  local fullFilename
  local screenshotNumber = 0
  repeat
    if screenshotNumber > 0 then
      fullFilename = FS:expandFilename(string.format("%s_%s", subFilename, screenshotNumber))
    else
      fullFilename = FS:expandFilename(subFilename)
    end
    screenshotNumber = screenshotNumber + 1
  until not FS:fileExists(fullFilename)
  log('I','screenshot', "writing screenshot: " .. fullFilename)

  -- log('I','screenshot', "Taking screenshot "..fullFilename.." Format = "..screenshotFormat.." superSampling = "..tostring(superSampling).." tiles = "..tostring(tiles).." overlap = "..tostring(overlap).." downsample = "..tostring(downsample))
  createScreenshot(fullFilename, screenshotFormat, superSampling, tiles, overlap, downsample)
end

-- executed by c++ when the screenshot is done
-- res == 0 means all good
local function screenshotSaved(res, filename)
  --dump{'screenshot saved to disk', filename}
end

-- this is called when the screenshot is taken on the GPU, but not written to disc yet. see screenshotSaved
-- we revet some graphic settings in here only to return to a normal render state
local function screenshotTaken()
  if M.screenshotHighest then
      log('I','screenshot', "Screenshot done, resetting render parameters")
      TorqueScriptLua.setVar("$pref::TS::detailAdjust", M.sc_detailAdjustSaved)
      TorqueScriptLua.setVar("$pref::Terrain::lodScale", M.sc_lodScaleSaved)
      setGroundCoverScale(M.sc_GroundCoverScaleSaved)

    local sunsky = scenetree.findObject("sunsky")
    if sunsky then
      sunsky.texSize = M.sc_sunskyTexSizeSaved
      sunsky.shadowDistance = M.sc_sunskyShadowDistanceSaved
    end
  end
end

-- public interface
M.publish = publish
M.doScreenshot = doScreenshot
M.doSteamScreenshot = doSteamScreenshot
M.openScreenshotsFolderInExplorer = openScreenshotsFolderInExplorer
M.takeScreenShot = function() _screenshot(4, 1, 0, false, true) end
M.takeBigScreenShot = function() _screenshot(9, 1, 0, false) end
M.takeHugeScreenShot = function() _screenshot(36, 1, 0, true) end

M.screenshotTaken = screenshotTaken -- GPU snapshot done
M.screenshotSaved = screenshotSaved -- saved to disk

return M