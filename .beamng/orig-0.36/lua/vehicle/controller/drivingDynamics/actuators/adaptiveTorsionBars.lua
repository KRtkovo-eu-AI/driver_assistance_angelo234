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

local configPacket = {sourceType = "adaptiveTorsionBars", packetType = "config", config = controlParameters}
local debugPacket = {sourceType = "adaptiveTorsionBars"}

local torsionBarModes = {}
local torsionBars = {}

local function setTorsionBarMode(modeName)
  local mode = torsionBarModes[modeName]
  if not mode then
    log("E", "adaptiveTorsionBars.setTorsionBarMode", "Can't find mode: " .. modeName)
    return
  end

  for _, cid in ipairs(torsionBars) do
    local spring = v.data.torsionbars[cid].spring * mode.springCoef
    local damp = v.data.torsionbars[cid].damp * mode.dampCoef
    obj:setTorsionbarSpringDamp(cid, spring, damp)
  end
end

local function reset()
end

local function init(jbeamData)
  local torsionBarNames = jbeamData.torsionBarNames or {}
  torsionBars = {}
  for _, b in pairs(v.data.torsionbars) do
    if b.name then
      for _, name in pairs(torsionBarNames) do
        if b.name == name then
          table.insert(torsionBars, b.cid)
        end
      end
    end
  end

  local modeData = tableFromHeaderTable(jbeamData.modes or {})

  torsionBarModes = {}
  for _, mode in pairs(modeData) do
    torsionBarModes[mode.name] = {
      springCoef = mode.springCoef or 1,
      dampCoef = mode.dampCoef or 1
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
  if parameters.torsionBarMode then
    setTorsionBarMode(parameters.torsionBarMode)
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
