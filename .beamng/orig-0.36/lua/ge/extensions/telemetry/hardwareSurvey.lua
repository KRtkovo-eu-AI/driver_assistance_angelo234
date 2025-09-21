-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- the part that is responsible for hardware surveys

-- extensions.telemetry_hardwareSurvey.checkUpdateSurvey()
-- extensions.telemetry_hardwareSurvey.createNewSurvey()


local M = {}

local filename = 'telemetry/anonymousHardwareSurvey.json'

local function createNewSurvey()
  if not FS:directoryExists("telemetry") then
    FS:directoryCreate("telemetry")
  end

  local res = {}

  res.timestamp = getDateTimeUTCString()

  res.mem = Engine.Platform.getMemoryInfo()
  res.cpu = Engine.Platform.getCPUInfo()
  res.gpu = Engine.Platform.getGPUInfo()
  res.os  = Engine.Platform.getOSInfo()
  res.pwr = Engine.Platform.getPowerInfo()
  res.gpu.adaptertype = Engine.Render.getAdapterType()
  res.adapters = GFXInit.getAdapters()
  res.monitors = Engine.Platform.getMonitorInfo()

  res.buildInfo = {
    versionb = beamng_versionb,
    versiond = beamng_versiond,
    version = beamng_version,
    windowtitle = beamng_windowtitle,
    buildtype = beamng_buildtype,
    buildinfo = beamng_buildinfo,
    arch = beamng_arch,
    buildnumber = beamng_buildnumber,
    appname = beamng_appname,
  }

  res.inputDevices = WinInput.getControllersInfo()

  res.bindings = {
    loadedFiles = core_input_bindings.getUsedBindingsFiles()
  }

  res.audio = Engine.Audio.getInfo()

  res.settings = settings.impl.getValues()

  res.locales = {
    selectedLanguage = Lua:getSelectedLanguage(),
    steamLanguage = Lua:getSteamLanguage(),
    osLanguage = Lua:getOSLanguage(),
  }

  res.userSaltHashed = extensions.telemetry_telemetryManager.getSaltHashed()

  res.format = 2

  jsonWriteFile(filename, res, true)
end

local function daysAgoISO8601(isoDateString)
  local parsedTime = parseISO8601Date(isoDateString)
  local currentTime = os.time(os.date("!*t"))
  local differenceInSeconds = os.difftime(currentTime, parsedTime)
  return differenceInSeconds / (60 * 60 * 24) -- Convert seconds to days
end

local function checkUpdateSurvey()
  if FS:fileExists(filename) then
    local data = jsonReadFile(filename)
    local daysOld = daysAgoISO8601(data.timestamp)
    if data.format == 2 and daysOld < 60 then
      -- format still ok, and newer than 60 days
      print('hardwareSurvey still ok: ' .. tostring(daysOld) .. ' days old')
      return
    end
  end
  createNewSurvey()
end




M.checkUpdateSurvey = checkUpdateSurvey
M.createNewSurvey = createNewSurvey

return M
