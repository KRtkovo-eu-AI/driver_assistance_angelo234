-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this extension will generate json and other resoruces that can then be integrated into our documentation / hugo system

-- extensions.util_docCreator.run()

local jsonEncodeFull = require('libs/lunajson/lunajson').encode -- slow but conform encoder

local M = {}

local outputFolderData = 'doc-out/data/'
local outputFolderResources = 'doc-out/resources/'
local quitOnDone = false

local function changeLanguage(lang)
  Lua.userLanguage = lang
  Lua:reloadLanguages()
  local langNow = Lua:getSelectedLanguage()
  if langNow ~= lang then
    log('E', '', 'Unable to switch to language ' .. tostring(lang) .. ' - game is chose ' .. tostring(langNow))
  end
end

local function jsonOut(filename, data)
  filename = outputFolderData .. filename
  local f = io.open(filename, "w")
  if not f then return end
  f:write(jsonEncodePretty(data))
  --f:write(jsonEncodeFull(data))
  f:close()
end

local function getLanguagesAvailable()
  local locales = FS:findFiles('/locales/', '*.json', -1, true, false)
  local res = {}
  for _, l in pairs(locales) do
    local key = string.match(l, 'locales/(.*).json')
    if key ~= "not-shipping.internal" then
      table.insert(res, key)
    end
  end
  return res
end

local function cleanupTable(job, tbl)
  if type(tbl) == 'table' then
    for k, v in pairs(tbl) do
      if type(v) == 'table' then
        cleanupTable(job, v)
      elseif type(v) == 'string' then
        tbl[k] = translateLanguage(v, v, true) -- true = silent logs
        v = tbl[k]

        if string.len(v) > 0 and FS:fileExists(v) then
          local orgFilename = v
          local dir, filename, ext = path.split(v)
          ext = ext:lower()
          local newFilename = FS:hashFileSHA1(v) .. '.' .. ext
          local outFilename = outputFolderResources .. newFilename
          if not FS:fileExists(outFilename) then
            FS:copyFile(v, outFilename)
            job.yield()
          end
          tbl[k] = '/game/resources/' .. newFilename
        end
      end
    end
  end
  job.yield()
end


local function exportDataLangSpecific(job, lang)
  log('I', '', 'Processing language: ' .. tostring(lang))
  local langFilename = lang:gsub('-', '_')
  changeLanguage(lang)

  local levels = deepcopy(extensions.core_levels.getList())
  cleanupTable(job, levels)

  for _, level in ipairs(levels) do
    if type(level.size) == 'table' and #level.size > 1 then
      if level.size[1] == -1 and level.size[2] == -1 then
        level.size = nil
      end
    end
    level.openLink = 'beamng:v1/openMap/{"level":"' .. level.fullfilename .. '"}'
  end

  local vehicles = {
    models = extensions.core_vehicles.getModelList(true),
    configs = extensions.core_vehicles.getConfigList(true)
  }
  vehicles = deepcopy(vehicles)
  cleanupTable(job, vehicles)

  jsonOut('levels_' .. langFilename .. '.json', levels)
  jsonOut('vehicles_' .. langFilename .. '.json', vehicles)
end

local function getTestedControllers()
  local vendorNames = jsonReadFile("lua/ge/extensions/util/vendorNames.json")
  local mergedInfo = {} -- use a map to merge information that is spread across multiple json files

  -- gather data from each known inputmap file (json files only, which correspond to the default configs shipped with the game)
  local inputmapPaths = FS:findFiles('/settings/inputmaps/', '*.json', -1, true, false)
  for _,inputmapPath in ipairs(inputmapPaths) do
    local out = jsonReadFile(inputmapPath)

    -- try to use VIDPID from file contents
    if out.vidpid then
      out.vidpid = string.lower(out.vidpid)
    end

    -- try to use PIDVID from file name
    out.filename = string.lower(select(2, path.splitWithoutExt(inputmapPath)))
    out.identifier = string.match(out.filename, "^[^_]+") -- remove '_whatever' suffixes, used for additional secondary inputmaps
    out.pidvid = #out.identifier == 8 and string.match(out.identifier, "^[a-f0-9]+$")

    -- decide whether to use file contents or the file name pidvid or the file name as-is
    if out.vidpid and out.pidvid then
      if out.pidvid ~= out.vidpid then
        log("E", "", "Mismatching file vidpid "..dumps(out.vidpid)".. vs filename "..dumps(out.pidvid)..". Fix the mismatch.")
      end
    end
    out.vidpid = nil -- remove incorrect field name, after we've converted it to pidvid already

    -- split PIDVID into PID and VID
    if out.pidvid then
      out.pid = out.pidvid:sub(1, 4)
      out.vid = out.pidvid:sub(5, 8)
    end

    -- try to figure out the vendor
    out.vendorId = out.vid or out.identifier
    out.vendorName = out.vendorName or vendorNames[out.vendorId]
    if not out.vendorName then
      log("W", "", "Unrecognized vendor "..dumps(out.vendorId).." for device named: "..dumps(out.name, out.displayName))
    end

    -- detect if ffb is enabled on this configuration
    for _,binding in ipairs(out.bindings or {}) do
      if binding.action == "steering" and binding.isForceEnabled then
        out.ffbEnabled = true
        break
      end
    end

    -- detect if ffb is supported on this device
    local x = {}
    table.insert(x, out.ffbEnabled           and "<div class='bng-hw ffb'></div>")
    table.insert(x, out.ffbSupported == true and "<div class='bng-hw ffb'></div>" or out.ffbSupported)
    table.insert(x, out.trueforceSupported   and "<div class='bng-hw trueforce'></div>")
    out.ffb = table.concat(x, " ")

    -- write all the information into an intermediate temporary table
    if mergedInfo[out.identifier] then
      log("W", "", "Found additional inputmap files for identifier "..dumps(out.identifier)..". Extra filename: "..dumps(out.filename))
    else
      mergedInfo[out.identifier] = {}
    end
    tableMerge(mergedInfo[out.identifier], out)
  end

  -- gather all information into an unsorted list
  local entries = {}
  local header = {}
  table.insert(header, "In-game controls")
  table.insert(header, "Force-feedback")
  table.insert(header, "Controller [(vendor ID, product ID)](http://www.the-sz.com/products/usbid)")

  for k,info in pairs(mergedInfo) do
    local entry = {}

    -- configured controls
    local assigned = not tableIsEmpty(info.bindings or {})
    if info.assigned ~= nil then
      assigned = info.assigned
    end
    table.insert(entry, assigned and "<div class='bng-hw assigned'></div>" or "<div class='bng-hw not-assigned'></div>") -- TODO also grab info from other stuff, see current docs to see what's missing

    -- force feedback
    local ffb = info.ffb or "" -- TODO also grab info from other stuff, see current docs to see what's missing
    table.insert(entry, ffb)

    -- support for multiple entries for a single json file via displayNames field
    local names = {}
    if info.displayNames then
      if info.displayName then
        log("E", "", "Conflict: both displayNames and displayName fields are provided: "..dumps(info.displayName, info.displayNames))
      end
      for _,displayName in ipairs(info.displayNames) do
        table.insert(names, displayName)
      end
    else
      local name = info.displayName or info.name or ""
      table.insert(names, name)
    end

    for _,name in ipairs(names) do
      local e = deepcopy(entry)
      if info.vendorName then
        if string.lower(string.sub(name, 1, string.len(info.vendorName))) == string.lower(info.vendorName) then
          name = string.sub(name, string.len(info.vendorName) + 1)
          name = name:gsub("^%s+", "") -- strip whitespaces
          name = name:gsub("^-+", "") -- remove '-' prefix, for exampe for "PXN-V12" name
        end
      end

      -- device name + notes
      local controller = {}
      local vendor = info.vendorName or ""
      table.insert(controller, vendor)
      table.insert(controller, "<b>"..name.."</b>")
      if info.vid or info.pid then
        local x = {}
        table.insert(x, info.vid and ("0x"..info.vid..""))
        table.insert(x, info.pid and ("0x"..info.pid..""))
        table.insert(controller, "<span class='bng-hw vendor'>("..table.concat(x, ", ")..")</span>")
      end
      local content = table.concat(controller, " ")
      content = content .. (info.notes and ("<div class='bng-hw hw-note'>Note: "..info.notes.."</div>") or "")
      table.insert(e, content)

      table.insert(entries, e)
    end
  end

  --TODO add data that's not available in inputmap files

  -- sort all entries
  table.sort(entries,
    function(a,b)
      return a[3] < b[3] -- compare name/notes
    end
  )

  -- add header and return it all
  table.insert(entries, 1, header)
  return entries
end

local function exportDataCommon()
  -- write some version file so the doc knows where this came from
  local versionInfo = {}
  versionInfo['beamng_versionb'] = beamng_versionb
  versionInfo['beamng_versiond'] = beamng_versiond
  versionInfo['beamng_windowtitle'] = beamng_windowtitle
  versionInfo['beamng_buildtype'] = beamng_buildtype
  versionInfo['beamng_buildinfo'] = beamng_buildinfo
  versionInfo['beamng_arch'] = beamng_arch
  versionInfo['beamng_buildnumber'] = beamng_buildnumber
  versionInfo['beamng_appname'] = beamng_appname
  versionInfo['shipping_build'] = tostring(shipping_build)
  jsonOut('game_version.json', versionInfo)

  -- Materials
  local materials, _ = require("particles").getMaterialsParticlesTable()
  local materialsClean = {}
  for i, m in pairs(materials) do
    table.insert(materialsClean, m.name) -- {m.colorR, m.colorG, m.colorB}
  end
  jsonOut('physics_materials.json', materialsClean)

  -- jbeam defaults
  local loader = require("jbeam/loader")
  local jbeamDefaults = {
    defaultBeamSpring = loader.defaultBeamSpring,
    defaultBeamDeform = loader.defaultBeamDeform,
    defaultBeamDamp = loader.defaultBeamDamp,
    --defaultBeamStrength = loader.defaultBeamStrength,
    defaultNodeWeight = loader.defaultNodeWeight,
  }
  jsonOut('physics_jbeam_defaults.json', jbeamDefaults)

  -- jbeam stats
  jsonOut('jbeam_stats.json', extensions.util_jbeamStats.getStats(), true)

  -- tested input controllers
  local testedControllers = getTestedControllers()
  jsonOut("tested_controllers.json", testedControllers)
end

local function run(job)
  FS:directoryCreate(outputFolderData)
  FS:directoryCreate(outputFolderResources)
  --exportData('en-US')

  for _, lang in ipairs(getLanguagesAvailable()) do
    exportDataLangSpecific(job, lang)
  end
  exportDataCommon(job)
  print("DONE")
  if quitOnDone then
    shutdown(0)
  end
end

local function runAsync()
  extensions.core_jobsystem.create(run, 1)
end

local function runAsyncAndQuit()
  quitOnDone = true
  extensions.core_jobsystem.create(run, 1)
end

M.run = runAsync
M.runAndQuit = runAsyncAndQuit

return M
