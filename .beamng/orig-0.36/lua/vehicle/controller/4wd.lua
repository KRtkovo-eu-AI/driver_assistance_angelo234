-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.relevantDevice = "transfercase"

local driveModesRange = nil
local driveModesTransfercase = nil
local driveModesDifferentials = nil

local shaft = nil
local rangeBox = nil
local hasBuiltPie = false

local function updateGFX(dt)
  electrics.values.modeRangeBox = rangeBox and (rangeBox.mode == "low" and 1 or 0) or 0
  electrics.values.mode4WD = shaft and (shaft.mode == "connected" and 1 or 0) or 0
end

local function toggleDiffs()
  if driveModesDifferentials then
    for _, c in ipairs(driveModesDifferentials) do
      c.nextDriveMode()
    end
  else
    --backwards compat code for older vehicles that only use the 4wd controller and no drivemodes
    --input action calls this method if a 4wd controller is found, if not it directly call the powertrain API
    powertrain.toggleDefaultDiffs()
  end
end

local function toggleRange()
  if driveModesRange then
    for _, c in ipairs(driveModesRange) do
      c.nextDriveMode()
    end
  else
    --backwards compat code for older vehicles that only use the 4wd controller and no drivemodes
    if rangeBox then
      powertrain.toggleDeviceMode(rangeBox.name)
    end
  end
end

local function setRangeMode(mode)
  if rangeBox then
    powertrain.setDeviceMode(rangeBox.name, mode)
  end
end

local function toggle4WD()
  if driveModesTransfercase then
    for _, c in ipairs(driveModesTransfercase) do
      c.nextDriveMode()
    end
  else
    --backwards compat code for older vehicles that only use the 4wd controller and no drivemodes
    if shaft and not shaft.isPhysicallyDisconnected then
      powertrain.toggleDeviceMode(shaft.name)
    end
  end
end

local function set4WDMode(mode)
  if shaft and not shaft.isPhysicallyDisconnected then
    powertrain.setDeviceMode(shaft.name, mode)
  end
end

local function serialize()
  return {
    mode4WD = shaft and shaft.mode or nil,
    modeRange = rangeBox and rangeBox.mode or nil
  }
end

local function deserialize(data)
  if data then
    if shaft and data.mode4WD then
      set4WDMode(data.mode4WD)
    end
    if rangeBox and data.modeRange then
      setRangeMode(data.modeRange)
    end
  end
end

local function init(jbeamData)
  shaft = powertrain.getDevice(jbeamData.shaftName)
  rangeBox = powertrain.getDevice(jbeamData.rangeBoxName)
  if shaft then
    electrics.values.mode4WD = shaft.mode == "connected" and 1 or 0
  end
  if rangeBox then
    electrics.values.modeRangeBox = rangeBox.mode == "low" and 1 or 0
  end

  if not hasBuiltPie then
    if shaft then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          generator = function(entries)
            local wdIcon
            if shaft.mode == "disconnected" then
              wdIcon = "jointUnlocked"
            else
              wdIcon = "jointLocked"
            end

            local wdEntry = {
              title = "ui.radialmenu2.powertrain.4WD_Mode",
              icon = wdIcon,
              uniqueID = "4wdMode",
              onSelect = function()
                if shaft.mode == "disconnected" then
                  controller.getController(M.name).set4WDMode("connected")
                else
                  controller.getController(M.name).set4WDMode("disconnected")
                end
                return {"reload"}
              end
            }
            if shaft.mode == "connected" then
              wdEntry.color = "#ff6600"
            end
            table.insert(entries, wdEntry)
          end
        }
      )
    end
    if rangeBox then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          uniqueID = "rangeboxMode",
          generator = function(entries)
            local rmIcon
            if rangeBox.mode == "low" then
              rmIcon = "rangeboxLo"
            else
              rmIcon = "rangeboxHi"
            end

            local rmEntry = {
              title = "ui.radialmenu2.powertrain.rangebox_mode",
              icon = rmIcon,
              onSelect = function()
                if rangeBox.mode == "low" then
                  controller.getController(M.name).setRangeMode("high")
                else
                  controller.getController(M.name).setRangeMode("low")
                end
                return {"reload"}
              end
            }
            rmEntry.color = "#ff6600"
            table.insert(entries, rmEntry)
          end
        }
      )
    end
    hasBuiltPie = true
  end
end

local function initLastStage(jbeamData)
  local driveModesRangeNames = jbeamData.driveModesRangeNames or {}
  local driveModesTransfercaseNames = jbeamData.driveModesTransfercaseNames or {}
  local driveModesDifferentialNames = jbeamData.driveModesDifferentialNames or {}

  driveModesDifferentials = nil
  for _, name in ipairs(driveModesDifferentialNames) do
    local c = controller.getController(name)
    if c then
      driveModesDifferentials = driveModesDifferentials or {}
      table.insert(driveModesDifferentials, c)
    end
  end

  driveModesTransfercase = nil
  for _, name in ipairs(driveModesTransfercaseNames) do
    local c = controller.getController(name)
    if c then
      driveModesTransfercase = driveModesTransfercase or {}
      table.insert(driveModesTransfercase, c)
    end
  end

  driveModesRange = nil
  for _, name in ipairs(driveModesRangeNames) do
    local c = controller.getController(name)
    if c then
      driveModesRange = driveModesRange or {}
      table.insert(driveModesRange, c)
    end
  end
end

M.init = init
M.initLastStage = initLastStage
M.updateGFX = updateGFX
M.toggleDiffs = toggleDiffs
M.toggle4WD = toggle4WD
M.set4WDMode = set4WDMode
M.toggleRange = toggleRange
M.setRangeMode = setRangeMode
M.serialize = serialize
M.deserialize = deserialize

return M
