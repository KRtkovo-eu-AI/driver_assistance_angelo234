-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = extensions.ui_imgui
local jbeamTableSchema = require('jbeam/tableSchema')

local wndName = "JBeam Variables Checker"
M.menuEntry = "JBeam Variables Checker"

local windowOpen = im.BoolPtr(false)

-- From common/jbeam/io.lua that parses a JBeam file from its filename
local function parseFile(filename)
  local content = readFile(filename)
  if content then
    local ok, data = pcall(json.decode, content)
    if ok == false then
      log('E', "jbeam.parseFile","unable to decode JSON: "..tostring(filename))
      log('E', "jbeam.parseFile","JSON decoding error: "..tostring(data))
      return nil
    end
    return data
  else
    log('E', "jbeam.parseFile","unable to read file: "..tostring(filename))
  end
end

local function checkValInRange(vv, val, varDefFilename, valDefFilename, partName)
  vv.val = val
  vv.min, vv.max = math.min(vv.min, vv.max), math.max(vv.min, vv.max)

  local valBeforeClamp = vv.val
  vv.val = clamp(vv.val, vv.min, vv.max)

  if valBeforeClamp ~= vv.val then
    log('W', '', 'variable ' .. tostring(vv.name) .. ' value out of range! value ' .. tostring(valBeforeClamp) .. ' clamped to range [' .. tostring(vv.min) .. ',' .. tostring(vv.max) .. '] as ' .. tostring(vv.val))
    log('W', '', '  var defined: ' .. varDefFilename .. ':' .. partName)
    log('W', '', '  val defined: ' .. valDefFilename)
    log('W', '', '')
  end
end

local function analyzeJBeamFile(filePath, fileName, jbeamFileData, varToConfigsAndVals)
  for partName, partData in pairs(jbeamFileData) do
    jbeamTableSchema.process(partData, false, true)
    if partData.variables then
      for kv,vv in pairs(partData.variables) do
        checkValInRange(vv, vv.default, filePath, filePath, partName)

        local configsAndVals = varToConfigsAndVals[vv.name]
        if configsAndVals then
          for k, configAndVal in ipairs(configsAndVals) do
            local pcFilePath = configAndVal.pcFilePath
            local val = configAndVal.val

            checkValInRange(vv, val, filePath, pcFilePath, partName)
          end
        end
      end
    end
  end
end

local function analyze()
  local dirPaths = FS:findFiles('vehicles', "*", 0, false, true)

  for _, vehDir in ipairs(dirPaths) do
    local pcFilePaths = FS:findFiles(vehDir, "*.pc", -1, false, false)
    local jbeamFilePaths = FS:findFiles(vehDir, "*.jbeam", -1, false, false)

    local varToConfigsAndVals = {}

    for _, pcFilePath in ipairs(pcFilePaths) do
      local vehConfig = extensions.core_vehicle_partmgmt.buildConfigFromString(vehDir, pcFilePath)

      if vehConfig.vars then
        for var, val in pairs(vehConfig.vars) do
          if not varToConfigsAndVals[var] then
            varToConfigsAndVals[var] = {}
          end
          table.insert(varToConfigsAndVals[var], {pcFilePath = pcFilePath, val = val})
        end
      end
    end

    for _, jbeamFilePath in ipairs(jbeamFilePaths) do
      local dir, fileName, _ = path.splitWithoutExt(jbeamFilePath)

      local data = parseFile(jbeamFilePath)
      analyzeJBeamFile(jbeamFilePath, fileName, data, varToConfigsAndVals)
    end
  end
  print('Done!')
end

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if im.Begin(wndName, windowOpen) then
    if im.Button("Start Analysis") then
      analyze()
    end
  end

  ::continue::
  im.End()
end

local function open()
  windowOpen[0] = true
end

local function onSerialize()
  return {
    windowOpen = windowOpen[0],
  }
end

local function onDeserialized(data)
  windowOpen[0] = data.windowOpen
end

M.open = open

M.onUpdate = onUpdate

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M