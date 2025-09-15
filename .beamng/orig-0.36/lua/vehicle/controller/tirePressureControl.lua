-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local max = math.max

local hasRegisteredQuickAccess = false

local wheelGroups = {}
local activeGroups = {}
local inflateMaxFlow
local deflateMaxFlow
local pEnv = 101325
local activeGroupPressureChange = 0
local activeGroupPressureElectricsName
local activeGroupPressureElectricsSmoother = newTemporalSigmoidSmoothing(300000, 500000, 200000, 300000)

local function updateGFX(dt)
  pEnv = obj:getEnvPressure()
  local activeGroupAvgPressure = 0
  local activeGroupCount = 0

  for groupName, wheelGroup in pairs(wheelGroups) do
    local groupPressure = 0
    for _, wheelData in pairs(wheelGroup.wheelData) do
      wheelData.isWheelBrokenOff = wheels.wheels[wheelData.wheelId].isBroken
      wheelData.currentPressure = obj:getGroupPressure(wheelData.pressureGroupId)
      groupPressure = groupPressure + (wheelData.isWheelBrokenOff and 0 or wheelData.currentPressure)
    end
    groupPressure = groupPressure / wheelGroup.wheelCount
    local preSyncGroupPressure = groupPressure --save the current group pressure for purposes of pressure equalization between wheels of a single axle
    groupPressure = 0 --reset the avg pressure so that we can re-calculate it with the changed pressures after equalization

    for _, wheelData in pairs(wheelGroup.wheelData) do
      local pressureDiff = preSyncGroupPressure - wheelData.currentPressure
      local pressureChange = clamp(pressureDiff * dt, -wheelGroup.groupEqualizeFlowLimit, wheelGroup.groupEqualizeFlowLimit)
      wheelData.currentPressure = wheelData.isWheelBrokenOff and wheelData.currentPressure or max(wheelData.currentPressure + pressureChange, pEnv) --limit lowest pressure to environment pressure
      obj:setGroupPressure(wheelData.pressureGroupId, wheelData.currentPressure)
      groupPressure = groupPressure + wheelData.currentPressure
      --debug
    end
    groupPressure = groupPressure / wheelGroup.wheelCount --calculate adjusted avg again for equalization between active circuits

    if activeGroups[groupName] then
      activeGroupAvgPressure = activeGroupAvgPressure + groupPressure
      activeGroupCount = activeGroupCount + 1
    end
  end

  activeGroupAvgPressure = activeGroupCount > 0 and (activeGroupAvgPressure / activeGroupCount) or pEnv -- calculate avg between all active circuits
  electrics.values[activeGroupPressureElectricsName] = activeGroupPressureElectricsSmoother:get(activeGroupAvgPressure, dt)

  local pressureChangeInflateDeflate = activeGroupPressureChange * dt
  for groupName, isActive in pairs(activeGroups) do
    local wheelGroup = wheelGroups[groupName]
    if isActive then
      for _, wheelData in pairs(wheelGroup.wheelData) do
        local pressureDiff = activeGroupAvgPressure - wheelData.currentPressure
        local pressureChangeEqualize = clamp(pressureDiff * dt, -wheelGroup.groupEqualizeFlowLimit * dt, wheelGroup.groupEqualizeFlowLimit * dt)
        local limitAdjustedPressureChangeInflateDeflate = pressureChangeInflateDeflate
        if (pressureChangeInflateDeflate > 0 and wheelData.currentPressure >= wheelGroup.maxPressure) or (pressureChangeInflateDeflate < 0 and wheelData.currentPressure <= wheelGroup.minPressure) then
          limitAdjustedPressureChangeInflateDeflate = 0
        end
        wheelData.currentPressure = max(wheelData.currentPressure + pressureChangeEqualize + limitAdjustedPressureChangeInflateDeflate, pEnv) --limit lowest pressure to environment pressure
        obj:setGroupPressure(wheelData.pressureGroupId, wheelData.currentPressure)
      end
    end
    electrics.values[wheelGroup.isActiveElectricsName] = isActive and 1 or 0
  end

  --print(electrics.values[activeGroupPressureElectricsName])
end

local function setGroupState(groupName, state)
  if activeGroups[groupName] == nil then
    log("E", "tirePressureControl.setGroupState", string.format("Can't find group with name %q", (groupName or "nil")))
    return
  end
  activeGroups[groupName] = state
  guihooks.message(
    {
      txt = "vehicle.tirePressureControl.message.setGroupState",
      context = {
        groupName = wheelGroups[groupName].uiName,
        groupState = state and "vehicle.tirePressureControl.message.groupActive" or "vehicle.tirePressureControl.message.groupInactive"
      }
    },
    2,
    "vehicle.tirePressureControl.groupState." .. groupName
  )
end

local function toggleGroupState(groupName)
  if activeGroups[groupName] == nil then
    log("E", "tirePressureControl.toggleGroupState", string.format("Can't find group with name %q", (groupName or "nil")))
    return
  end
  setGroupState(groupName, not activeGroups[groupName])
end

local function startInflateActiveGroups()
  activeGroupPressureChange = inflateMaxFlow
  guihooks.message("vehicle.tirePressureControl.message.inflatingActiveGroups", 2, "vehicle.tirePressureControl.inflateDeflate")
end

local function startDeflateActiveGroups()
  activeGroupPressureChange = -deflateMaxFlow
  guihooks.message("vehicle.tirePressureControl.message.deflatingActiveGroups", 2, "vehicle.tirePressureControl.inflateDeflate")
end

local function stopActiveGroups()
  activeGroupPressureChange = 0
  guihooks.message("vehicle.tirePressureControl.message.stoppingActiveGroups", 2, "vehicle.tirePressureControl.inflateDeflate")
end

local function registerQuickAccess()
  if not hasRegisteredQuickAccess then
    if not tableIsEmpty(wheelGroups) then
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/",
          generator = function(entries)
            table.insert(
              entries,
              {
                title = "ui.radialmenu2.tirePressureControl.title",
                priority = 40,
                icon = "tirePressureGaugeOutlined01",
                uniqueID = "tirePressureControl",
                ["goto"] = "/root/playerVehicle/vehicleFeatures/tirePressureControl/"
              }
            )
          end
        }
      )
      core_quickAccess.addEntry(
        {
          level = "/root/playerVehicle/vehicleFeatures/tirePressureControl/",
          generator = function(entries)
            table.insert(
              entries,
              {
                title = "ui.radialmenu2.tirePressureControl.startInflating",
                priority = 10,
                icon = "tirePressureIncrease01",
                originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "tirePressureControl"},
                onSelect = function()
                  controller.getControllerSafe(M.name).startInflateActiveGroups()
                  return {"reload"}
                end
              }
            )
            table.insert(
              entries,
              {
                title = "ui.radialmenu2.tirePressureControl.startDeflating",
                priority = 20,
                icon = "tirePressureDecrease01",
                originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "tirePressureControl"},
                onSelect = function()
                  controller.getControllerSafe(M.name).startDeflateActiveGroups()
                  return {"reload"}
                end
              }
            )
            table.insert(
              entries,
              {
                title = "ui.radialmenu2.tirePressureControl.stopChanges",
                priority = 30,
                icon = "tirePressureStopLine",
                originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "tirePressureControl"},
                onSelect = function()
                  controller.getControllerSafe(M.name).stopActiveGroups()
                  return {"reload"}
                end
              }
            )

            for groupName, wheelGroup in pairs(wheelGroups) do
              table.insert(
                entries,
                {
                  title = "ui.radialmenu2.tirePressureControl.toggleGroup",
                  context = {groupName = wheelGroup.uiName},
                  priority = 40,
                  icon = wheelGroup.uiIcon,
                  originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "tirePressureControl"},
                  onSelect = function()
                    controller.getControllerSafe(M.name).toggleGroupState(groupName)
                    return {"reload"}
                  end
                }
              )
            end
          end
        }
      )
      hasRegisteredQuickAccess = true
    end
  end
end

local function reset(jbeamData)
  for groupName, wheelGroup in pairs(wheelGroups) do
    activeGroups[groupName] = false
    for _, wheelData in pairs(wheelGroup.wheelData) do
      wheelData.currentPressure = 0
      wheelData.isWheelBrokenOff = false
    end
  end
  activeGroupPressureChange = 0
  activeGroupPressureElectricsSmoother:reset()
end

-- local function initSounds(jbeamData)
-- end

local function init(jbeamData)
end

local function initSecondStage(jbeamData)
  local mode = jbeamData.mode or "manualControl"
  if mode == "manualControl" then
    wheelGroups = {}
    activeGroups = {}
    inflateMaxFlow = jbeamData.inflateMaxFlow or 10
    deflateMaxFlow = jbeamData.deflateMaxFlow or 10
    activeGroupPressureElectricsName = jbeamData.activeGroupPressureElectricsName or (M.name .. "_activeGroupPressure")
    local wheelGroupData = tableFromHeaderTable(jbeamData.wheelGroups or {})
    for _, wheelGroup in pairs(wheelGroupData) do
      local groupName = wheelGroup.groupName
      local uiName = wheelGroup.uiName
      local uiIcon = wheelGroup.uiIcon
      local wheelNames = wheelGroup.wheelNames
      local maxPressure = wheelGroup.maxPressure
      local minPressure = wheelGroup.minPressure
      local groupEqualizeFlowLimit = wheelGroup.groupEqualizeFlowLimit
      if groupName and uiName and wheelNames and #wheelNames > 0 and maxPressure and minPressure and groupEqualizeFlowLimit then
        local wheelData = {}
        for _, wheelName in pairs(wheelNames) do
          local wheelId = wheels.wheelIDs[wheelName]
          if wheelId then
            local pressureGroupId = wheels.wheels[wheelId].pressureGroupId
            if pressureGroupId then
              wheelData[wheelName] = {
                pressureGroupId = pressureGroupId,
                wheelId = wheelId,
                isWheelBrokenOff = false,
                currentPressure = obj:getGroupPressure(pressureGroupId)
              }
            else
              log("E", "tirePressureControl.initSecondStage", string.format("Can't find pressure group id for wheel %q", wheelName))
            end
          else
            log("E", "tirePressureControl.initSecondStage", string.format("Can't find wheel id for wheel %q", wheelName))
          end
        end

        if not tableIsEmpty(wheelData) then
          wheelGroups[groupName] = {
            uiName = uiName,
            uiIcon = uiIcon,
            wheelNames = wheelNames,
            wheelData = wheelData,
            wheelCount = tableSize(wheelData),
            maxPressure = maxPressure,
            minPressure = minPressure,
            groupEqualizeFlowLimit = groupEqualizeFlowLimit,
            isActiveElectricsName = M.name .. "_" .. groupName .. "_isActive"
          }
          activeGroups[groupName] = false
        else
          log("D", "tirePressureControl.initSecondStage", "No valid wheel data provided for group: " .. (groupName or "nil"))
        end
      else
        log("E", "tirePressureControl.initSecondStage", "Invalid jbeam data provided")
        log("E", "tirePressureControl.initSecondStage", dumps(jbeamData))
      end
    end
  end
  registerQuickAccess()
  --dump(wheelGroups)
end

M.init = init
M.initSecondStage = initSecondStage
--M.initSounds = initSounds
M.reset = reset
--M.resetSounds = resetSounds

M.updateGFX = updateGFX
M.setGroupState = setGroupState
M.toggleGroupState = toggleGroupState
M.startInflateActiveGroups = startInflateActiveGroups
M.startDeflateActiveGroups = startDeflateActiveGroups
M.stopActiveGroups = stopActiveGroups

return M
