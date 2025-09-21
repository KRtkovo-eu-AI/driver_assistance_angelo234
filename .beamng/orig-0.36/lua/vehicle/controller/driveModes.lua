-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local defaultIcon = "ESC"

local hasRegisteredQuickAccess = false
local quickAccessTitle
local quickAccessIcon
local radialIcon
local quickAccessLevel

local defaultSettings = {}

local driveModes = {}
local enabledDriveModes = {}

local activeDriveMode = nil
local activeDriveModeIndex = nil
local uiName

local simpleControlButtons = {}

local function setExhaustGainMufflingOffset(engineName, mufflingOffset, gainOffset)
  local engine = powertrain.getDevice(engineName)
  if not engine or not engine.setExhaustGainMufflingOffset then
    return
  end

  engine:setExhaustGainMufflingOffset(mufflingOffset, gainOffset)
end

local function setTurbochargerWastegateOffset(engineName, wastegateOffset)
  local engine = powertrain.getDevice(engineName)
  if not engine or not engine.turbocharger or not engine.turbocharger.setWastegateOffset then
    return
  end

  engine.turbocharger.setWastegateOffset(wastegateOffset)
end

local function setSuperchargerBypassPressure(engineName, bypassPressure)
  local engine = powertrain.getDevice(engineName)
  if not engine or not engine.supercharger or not engine.supercharger.setBypassPressure then
    return
  end

  engine.supercharger.setBypassPressure(bypassPressure)
end

local function setGearboxMode(mode)
  if controller.mainController.setDefaultForwardMode then
    controller.mainController.setDefaultForwardMode(mode)
  end
end

local function updateSimpleControlButton(buttonData)
  extensions.ui_simplePowertrainControl.setButton(buttonData.id, buttonData.uiName, buttonData.icon, buttonData.currentColor, nil, buttonData.onClick)
end

local function setSimpleControlButton(id, buttonUIName, icon, color, offColor, offColorElectric, onClick, remove)
  if remove then
    simpleControlButtons[id] = nil
    extensions.ui_simplePowertrainControl.setButton(id, nil, nil, nil, nil, nil, true)
  else
    simpleControlButtons[id] = simpleControlButtons[id] or {}
    simpleControlButtons[id].id = id
    simpleControlButtons[id].uiName = buttonUIName
    simpleControlButtons[id].icon = icon
    simpleControlButtons[id].lastColor = nil
    simpleControlButtons[id].currentColor = color
    simpleControlButtons[id].color = color
    simpleControlButtons[id].offColor = offColor or "343434"
    simpleControlButtons[id].offColorElectric = offColorElectric
    simpleControlButtons[id].onClick = onClick or string.format("controller.getController(%q).nextDriveMode()", M.name)
    updateSimpleControlButton(simpleControlButtons[id])
  end
end

local function updateSimpleControlButtons()
  for _, buttonData in pairs(simpleControlButtons) do
    updateSimpleControlButton(buttonData)
  end
end

local function applyDriveMode(mode)
  --dump(mode)
  for _, settingsKey in ipairs(mode.settingsOrder) do
    local setting = mode.settings[settingsKey]

    if setting.type == "exhaust" then
      setExhaustGainMufflingOffset(setting.name, setting.mufflingOffset, setting.gainOffset)
    elseif setting.type == "turbocharger" then
      setTurbochargerWastegateOffset(setting.name, setting.wastegateOffset)
    elseif setting.type == "supercharger" then
      setSuperchargerBypassPressure(setting.name, setting.bypassPressure)
    elseif setting.type == "transmission" then
      setGearboxMode(setting.defaultForwardMode)
    elseif setting.type == "quickAccess" then
      quickAccessIcon = setting.icon or quickAccessIcon
      quickAccessTitle = setting.title or quickAccessTitle
    elseif setting.type == "simpleControlButton" then
      setSimpleControlButton(setting.buttonId, setting.uiName, setting.icon, setting.color, setting.offColor, setting.offColorElectric, setting.onClick, setting.remove)
    elseif setting.type == "controller" then
      local controller = controller.getController(setting.controllerName)
      if controller and controller.setParameters then
        local controllerSettings = deepcopy(setting)
        controllerSettings.type = nil
        controllerSettings.key = nil
        controllerSettings.controllerName = nil
        controller.setParameters(controllerSettings)
      end
    elseif setting.type == "powertrainDeviceMode" then
      local deviceName = setting.deviceName
      local deviceMode = setting.mode
      if deviceName and deviceMode then
        local device = powertrain.getDevice(deviceName)
        if device and device.setMode then
          device:setMode(deviceMode)
        end
      end
    elseif setting.type == "electricsValue" then
      local electricsName = setting.electricsName
      local electricsValue = setting.value
      if electricsName then
        electrics.values[electricsName] = electricsValue
      end
    end
  end
end

local function updateGFX(dt)
  for _, buttonData in pairs(simpleControlButtons) do
    local desiredColor = (electrics.values[buttonData.offColorElectric] and electrics.values[buttonData.offColorElectric] >= 1) and buttonData.offColor or buttonData.color
    if desiredColor ~= buttonData.currentColor then
      buttonData.currentColor = desiredColor
      updateSimpleControlButton(buttonData)
    end
  end
end

local function setDriveMode(modeKey)
  if not modeKey then
    return
  end
  local mode = driveModes[modeKey]
  if mode then
    activeDriveMode = mode
    applyDriveMode(mode)

    if uiName then
      guihooks.message(uiName .. ": " .. mode.name, 5, "vehicle.driveModes." .. uiName)
    else
      guihooks.message("Drivemode: " .. mode.name .. " ([action=toggleESCMode] to change)", 5, "vehicle.driveModes")
    end
  end
end

local function nextDriveMode()
  local key
  activeDriveModeIndex, key = next(enabledDriveModes, activeDriveModeIndex)
  if not key then
    activeDriveModeIndex, key = next(enabledDriveModes)
  end
  setDriveMode(key)
end

local function previousDriveMode()
  local newIndex = activeDriveModeIndex - 1
  if newIndex < 1 then
    newIndex = #enabledDriveModes
  end
  activeDriveModeIndex = newIndex
  setDriveMode(enabledDriveModes[activeDriveModeIndex])
end

local function getCurrentDriveModeKey()
  if activeDriveMode then
    return activeDriveMode.key
  end
end

local function getDriveModeData(modeKey)
  if modeKey then
    return driveModes[modeKey]
  end
end

local function reset(jbeamData)
  quickAccessTitle = jbeamData.quickAccessTitle or "ui.radialmenu2.ESC"
  quickAccessIcon = jbeamData.quickAccessIcon
end

local function resetLastStage()
  if activeDriveMode then
    setDriveMode(activeDriveMode.key)
  end
end

local function serialize()
  if activeDriveMode then
    return {activeDriveModeKey = activeDriveMode.key, activeDriveModeIndex = activeDriveModeIndex}
  end
end

local function deserialize(data)
  if data then
    if data.activeDriveModeKey and data.activeDriveModeIndex then
      activeDriveModeIndex = data.activeDriveModeIndex
      setDriveMode(data.activeDriveModeKey)
    end
  end
end

local function registerQuickAccess()
  if not hasRegisteredQuickAccess and #enabledDriveModes > 0 then
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle" .. quickAccessLevel,
        generator = function(entries)
          local currentDriveModeName = activeDriveMode.name
          local entry = {
            title = quickAccessTitle,
            desc = {txt = "ui.radialmenu2.currentValueColon", context = {value = currentDriveModeName}},
            priority = 40,
            uniqueID = "driveModeNext" .. quickAccessTitle,
            onSelect = function()
              controller.getControllerSafe(M.name).nextDriveMode()
              return {"reload"}
            end
          }
          if radialIcon then
            entry.icon = radialIcon
          elseif quickAccessIcon then
            entry.icon = quickAccessIcon
          else
            entry.icon = defaultIcon
          end
          table.insert(entries, entry)
        end
      }
    )
    hasRegisteredQuickAccess = true
  end
end

local function init(jbeamData)
  uiName = jbeamData.uiName
  quickAccessTitle = jbeamData.quickAccessTitle or "ui.radialmenu2.ESC"
  quickAccessIcon = jbeamData.quickAccessIcon
  quickAccessLevel = jbeamData.quickAccessLevel or "/vehicleFeatures/"
  radialIcon = jbeamData.radialIcon

  defaultSettings = {}
  if jbeamData.defaultSettings then
    local defaults = jbeamData.defaultSettings
    local defaultSettingData = tableFromHeaderTable(defaults)
    for _, v in pairs(defaultSettingData) do
      local key = v.key or v.controllerName or v.type
      defaultSettings[key] = v
    end
  end

  -- local modeData = jbeamData.modes
  -- if not modeData then
  --   return
  -- end

  local enabledModeKeys = {}
  local modeData = {}
  for k, v in pairs(jbeamData) do
    if k:sub(1, #"modes") == "modes" then
      tableMergeRecursive(modeData, v)
    end

    if k:sub(1, #"enabledModes") == "enabledModes" then
      for _, modeKey in pairs(v) do
        enabledModeKeys[modeKey] = true
      end
    end
  end

  local highPrioritySettings = {yawControl = 0, tractionControl = 0}

  --dump(modeData)

  --dump(enabledModeKeys)

  local modeSorting = {}

  for k, mode in pairs(modeData) do
    if driveModes[mode.order] then
      log("E", "driveModes.init", string.format("Duplicate mode order (%d) with mode: %s", mode.order, mode.name))
    end

    driveModes[k] = {
      key = k,
      name = mode.name,
      order = mode.order,
      settings = deepcopy(defaultSettings)
    }

    local modeSettings = tableFromHeaderTable(mode.settings or {})
    if modeSettings then
      driveModes[k].settingsOrder = {}
      for _, newSetting in pairs(modeSettings) do
        local key = newSetting.key or newSetting.deviceName or newSetting.electricsName or newSetting.controllerName or newSetting.type
        driveModes[k].settings[key] = newSetting
      end

      for settingsKey, _ in pairs(driveModes[k].settings) do
        table.insert(driveModes[k].settingsOrder, settingsKey)
      end

      table.sort(
        driveModes[k].settingsOrder,
        function(a, b)
          local ra, rb = highPrioritySettings[a] or 1, highPrioritySettings[b] or 1
          return ra < rb
        end
      )
    end

    if enabledModeKeys[k] then
      table.insert(modeSorting, {key = k, order = mode.order})
    end
  end

  table.sort(
    modeSorting,
    function(a, b)
      local ra, rb = a.order or 0, b.order or 0
      return ra < rb
    end
  )
  --dump(modeSorting)

  --dump(driveModes)

  for _, mode in pairs(modeSorting) do
    table.insert(enabledDriveModes, mode.key)
  end

  --dump(enabledDriveModes)

  -- for name, mode in pairs(modeSorting) do
  --   if driveModeOrder[mode.order] then
  --     log("E", "driveModes.init", string.format("Duplicate mode order (%d) with mode: %s", mode.order, mode.name))
  --   end
  --   driveModeOrder[mode.order] = name
  -- end

  --dump(driveModeOrder)

  registerQuickAccess()
end

local function initLastStage(jbeamData)
  local key
  local requestedDefaultMode = jbeamData.defaultMode or ""

  if driveModes[requestedDefaultMode] then
    for k, modeName in ipairs(enabledDriveModes) do
      if modeName == requestedDefaultMode then
        key = requestedDefaultMode
        activeDriveModeIndex = k
      end
    end
  end

  if not key then
    activeDriveModeIndex, key = next(enabledDriveModes)
  end
  setDriveMode(key)
end

M.init = init
M.reset = reset
M.resetLastStage = resetLastStage
M.initLastStage = initLastStage
M.updateGFX = updateGFX
M.serialize = serialize
M.deserialize = deserialize

M.nextDriveMode = nextDriveMode
M.previousDriveMode = previousDriveMode
M.setDriveMode = setDriveMode
M.getCurrentDriveModeKey = getCurrentDriveModeKey
M.getDriveModeData = getDriveModeData

M.updateSimpleControlButtons = updateSimpleControlButtons
M.setSimpleControlButton = setSimpleControlButton

return M
