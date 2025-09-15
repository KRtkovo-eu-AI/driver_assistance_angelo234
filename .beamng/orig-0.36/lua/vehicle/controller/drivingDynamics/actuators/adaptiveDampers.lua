-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

M.isActive = false
M.isActing = false

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local configPacket = {sourceType = "adaptiveDampers", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "adaptiveDampers"}

local beamModes = {}
local dampBeams = {}

local function setDamperMode(modeName)
  local mode = beamModes[modeName]
  if not mode then
    log("E", "adaptiveDampers.setDamperMode", "Can't find mode: " .. modeName)
    return
  end

  for _, cid in ipairs(dampBeams) do
    local beam = v.data.beams[cid]
    local beamDamp = beam.beamDamp * mode.beamDampCoef
    local beamDampRebound = beam.beamDampRebound * mode.beamDampReboundCoef
    local beamDampFast = beam.beamDampFast * mode.beamDampFastCoef
    local beamDampReboundFast = beam.beamDampReboundFast * mode.beamDampReboundFastCoef
    local beamDampVelocitySplit = beam.beamDampVelocitySplit * mode.beamDampVelocitySplitCoef
    obj:setBoundedBeamDamp(cid, beamDamp, beamDampRebound, beamDampFast, beamDampReboundFast, beamDampVelocitySplit, beamDampVelocitySplit)
  end
end

local function reset()
end

local function init(jbeamData)
  local dampBeamNames = jbeamData.dampBeamNames or {}
  dampBeams = {}
  for _, b in pairs(v.data.beams) do
    if b.name then
      for _, name in pairs(dampBeamNames) do
        if b.name == name then
          table.insert(dampBeams, b.cid)
        end
      end
    end
  end

  local modeData = tableFromHeaderTable(jbeamData.modes or {})

  beamModes = {}
  for _, mode in pairs(modeData) do
    beamModes[mode.name] = {
      beamDampCoef = mode.beamDampCoef or 1,
      beamDampFastCoef = mode.beamDampFastCoef or 1,
      beamDampReboundCoef = mode.beamDampReboundCoef or 1,
      beamDampReboundFastCoef = mode.beamDampReboundFastCoef or 1,
      beamDampVelocitySplitCoef = mode.beamDampVelocitySplitCoef or 1
    }
  end

  local nameString = jbeamData.name
  local slashPos = nameString:find("/", -nameString:len())
  if slashPos then
    nameString = nameString:sub(slashPos + 1)
  end
  debugPacket.sourceName = nameString

  M.isActive = true
end

local function initLastStage()
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled
end

local function registerCMU(cmu)
  CMU = cmu
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
end

local function setParameters(parameters)
  if parameters.damperMode then
    setDamperMode(parameters.damperMode)
  end
end

local function setConfig(configTable)
  controlParameters = configTable
end

local function getConfig()
  return deepcopy(controlParameters)
end

local function sendConfigData()
  configPacket.config = controlParameters
  CMU.sendDebugPacket(configPacket)
end

M.init = init
M.reset = reset
M.initLastStage = initLastStage

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

return M
