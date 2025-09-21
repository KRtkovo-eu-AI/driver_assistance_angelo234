-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local unknownSystemName = "?" -- if you modify this, you need to modify openxrHelper.cpp 'unknownSystemName' too

local im = ui_imgui
local debugWindowOpen = im.BoolPtr(false)
local debugWindowSize = im.ImVec2(0, 0)

local welcomeTitle     = "ui.openXR.welcomeTitle"
local welcomeBody      = "ui.openXR.welcomeBody"
local buttonOkText     = "ui.openXR.buttonOkText"
local buttonNoVulkanText="ui.openXR.buttonNoVulkanText"
local buttonCancelText = "ui.openXR.buttonCancelText"
local buttonOkLua     = "extensions.render_openxr.closeWelcome(true)"
local buttonNoVulkanLua="extensions.render_openxr.closeWelcome(false)"
local buttonCancelLua = "extensions.render_openxr.closeWelcome(false)"

local framesUntilCenter = nil

local M = {}
M.stateString = "disabled" -- can also be "enabled" and "welcome"
M.cefDialogOpen = nil
M.state = {}
M.state.systemName = unknownSystemName

local variableOnUpdate = nop
local function constOnUpdate(...)
  if framesUntilCenter then
    framesUntilCenter = framesUntilCenter - 1
    if framesUntilCenter < 0 then
      framesUntilCenter = nil
      M.center(0) -- end centering
    end
  end
  variableOnUpdate(...)
end

local function logState()
    log("D", "", "Current OpenXR state: "..dumps(M.state))
end

local fieldsTriggeringGuiHook = { "enabled", "sessionRunning", "headsetActive", "controller0Active", "controller1Active", "controller0poseValid", "controller1poseValid", "systemName", "targetRefreshRate", "renderedWidth", "renderedHeight", "recommendedWidth", "recommendedHeight" }
local lastKnownUIState = {}
local function updateUI(forced)
  local changed = forced
  for _,triggerField in ipairs(fieldsTriggeringGuiHook) do
    if M.state[triggerField] ~= lastKnownUIState[triggerField] then
      changed = true
      lastKnownUIState[triggerField] = M.state[triggerField]
    end
  end
  if not changed then return end

  if forced then
    logState()
  end
  guihooks.trigger('OpenXRStateChanged', M.state)
end

local function stateChanged(enabled, sessionRunning, headsetActive, controller0Active, controller1Active, controller0poseValid, controller1poseValid, systemName, currentRefreshRate, renderedWidth, renderedHeight, recommendedWidth, recommendedHeight, supportedWidth, supportedHeight, supportedLayers, sessionState, ipd, handSeparation, fov0hz, fov0vt, fov1hz, fov1vt)
  currentRefreshRate = currentRefreshRate > 1e10 and 0 or currentRefreshRate -- sanitize value: skip infinite refresh rates, due to (1/period) when period is unknown (zero)
  M.state.enabled         = enabled
  M.state.sessionRunning  = sessionRunning
  M.state.systemName      = (systemName == unknownSystemName) and "ui.options.graphics.openXRsystemName.unknown" or systemName
  M.state.currentRefreshRate = currentRefreshRate
  M.state.targetRefreshRate = math.max(M.state.targetRefreshRate or 0, M.state.currentRefreshRate) -- grab the highest number we've seen (in case reprojection had temporarily downgraded to half refresh rate, and we're now back to true refresh rate)
  M.state.renderedWidth   = renderedWidth
  M.state.renderedHeight  = renderedHeight
  M.state.recommendedWidth= recommendedWidth
  M.state.recommendedHeight=recommendedHeight
  M.state.supportedWidth  = supportedWidth
  M.state.supportedHeight = supportedHeight
  M.state.supportedLayers = supportedLayers
  M.state.sessionState    = sessionState
  M.state.ipd             = ipd
  M.state.handSeparation  = handSeparation
  M.state.headsetActive   = headsetActive
  M.state.controller0Active=controller0Active
  M.state.controller1Active=controller1Active
  M.state.controller0poseValid = controller0poseValid
  M.state.controller1poseValid = controller1poseValid
  M.state.fov0hz          = fov0hz
  M.state.fov0vt          = fov0vt
  M.state.fov1hz          = fov1hz
  M.state.fov1vt          = fov1vt

  if M.state.enabled and M.stateString ~= "enabled" then M.setStateUI("enabled") end
  if not M.state.enabled and M.stateString == "enabled" then M.setStateUI("disabled") end
  debugWindowOpen[0] = enabled
  updateUI()
end

local function saveSettings()
  settings.setValue("openXRuiEnabled"     , M.openXRuiEnabled[0])
  settings.setValue("openXRuiMode"        , M.openXRuiMode[0])
  settings.setValue("openXRwindowViewMode", M.openXRwindowViewMode[0])
  settings.setValue("openXRdebugEnabled"  , M.openXRdebugEnabled[0])
end

local function onSettingsChanged()
  local prev = {}
  prev.openXRdebugEnabled  = M.openXRdebugEnabled and M.openXRdebugEnabled[0]

  M.openXRimguiEnabled   = settings.getValue("openXRimguiEnabled")
  M.openXRuiEnabled      = im.BoolPtr(settings.getValue("openXRuiEnabled"))
  M.openXRuiMode         = im.IntPtr(settings.getValue("openXRuiMode"))
  M.openXRwindowViewMode = im.IntPtr(settings.getValue("openXRwindowViewMode"))
  M.openXRdebugEnabled   = im.BoolPtr(settings.getValue("openXRdebugEnabled"))

  local curr = {}
  curr.openXRdebugEnabled  = M.openXRdebugEnabled[0]

  local restartNeeded = false
  restartNeeded = restartNeeded or (curr.openXRdebugEnabled ~= prev.openXRdebugEnabled)
  if restartNeeded and M.isSessionRunning() then
    M.restart()
  end
  if OpenXR.getEnable() then
    OpenXR.generateUiCurvature() -- will use those settings we just got
  end
end

local function onInit()
  setExtensionUnloadMode(M, "manual")
  onSettingsChanged()
end

local function closeWelcome(enable)
  M.cefDialogOpen = nil
  M.setStateUI(enable and "enabled" or "disabled")
end

local logStatePending = false
local function onUpdate(dtReal, dtSim, dtRaw)
  if logStatePending then
    logStatePending = false
    logState()
  end

  if not M.openXRimguiEnabled then return end
  if not M.state.sessionRunning then return end

  im.SetNextWindowSize(debugWindowSize --[[, im.Cond_FirstUseEver--]] )
  im.Begin("OpenXR debug tools##openXRwindow", debugWindowOpen)
  if debugWindowOpen[0] == false then
    -- user closed the OpenXR window: we interpret this as wanting to shut down OpenXR for now
    M.setStateUI("disabled")
  else
    -- OpenXR is active: draw the dev UI
    local changed = false

    if im.Button("Turn on/off (ctrl+numpad0)##openXRclose") then
      M.setStateUI("disabled")
    end
    im.SameLine()
    if im.Button("Center (ctrl+numpad5)##openXRcenter") then
      M.center()
    end

    changed = im.Combo2("2D screen view##openXRwindowViewMode", M.openXRwindowViewMode, "Empty (fastest)\0Between eyes (slowest)\0Reuse left eye\0Reuse right eye\0") or changed
    changed = im.Checkbox("Display user interface (CEF)", M.openXRuiEnabled) or changed
    im.Text("   ") im.SameLine()
    --changed = im.Combo2("Anchor##openXRuiMode", M.openXRuiMode, "Room\0Head\0Vehicle (NI)\0Level (NI)\0") or changed -- hide not implemented modes for now
    changed = im.Combo2("Anchor##openXRuiMode", M.openXRuiMode, "Room\0Head") or changed

    local restartNeeded = false
    restartNeeded = im.Checkbox("Enable debug mode (reduced framerate, will restart OpenXR)", M.openXRdebugEnabled) or restartNeeded
    changed = changed or restartNeeded

    im.Separator()
    im.Text("System name: \"%s\"", M.state.systemName)
    im.Text("   Rendered resolution: %.0fx%.0f", M.state.renderedWidth, M.state.renderedHeight)
    im.Text("   Recommended resolution: %.0fx%.0f", M.state.recommendedWidth, M.state.recommendedHeight)
    im.Text("   Supported resolution: %.0fx%.0f", M.state.supportedWidth, M.state.supportedHeight)
    im.Text("   Supported layers: %.0f", M.state.supportedLayers)
    im.Text("   XrSessionState = \"%s\"", M.state.sessionState)
    im.Text("FOV: left %.3fx%.2f, right %.3fx%.2f (rendered)", M.state.fov0hz, M.state.fov0vt, M.state.fov1hz, M.state.fov1vt)
    im.Text("IPD: %.3f mm", M.state.ipd * 1000)
    im.Text("Hand separation: %.3f m", M.state.handSeparation)

    if im.Button("Reference setIdentity") then
      OpenXR.setLocalReference(true)
    end

    if changed then
      saveSettings()
    end
    if restartNeeded then
      M.restart()
    end

  end
  im.End()
end

local function restart()
  M.setStateUI("disabled")
  M.setStateUI("enabled")
end

M.setStateUI = function(stateString)
  M.stateString = stateString
  variableOnUpdate = nop
  if stateString == "welcome" then
    M.cefDialogOpen = true
    if Engine.Render.getAdapterType() == "Vulkan" then
      guihooks.trigger('ConfirmationDialogOpen', welcomeTitle, welcomeBody, buttonOkText, buttonOkLua, buttonCancelText, buttonCancelLua)
    else
      guihooks.trigger('ConfirmationDialogOpen', welcomeTitle, welcomeBody, nil, nil, buttonNoVulkanText, buttonNoVulkanLua)
    end
  elseif stateString == "enabled" then
    if M.cefDialogOpen then
      M.cefDialogOpen = nil
      guihooks.trigger('ConfirmationDialogClose', welcomeTitle)
    end
    if not OpenXR.getEnable() then
      OpenXR.toggle()
      logStatePending = true
    end
    if OpenXR.getEnable() then
      variableOnUpdate = onUpdate
    else
      log("D", "", "Unable to enable OpenXR") -- all error details should have been logged already by C++ side, no need to throw more Error level logs here, leave as Debug
      M.setStateUI("disabled")
    end
  elseif stateString == "disabled" then
    if M.cefDialogOpen then
      M.cefDialogOpen = nil
      guihooks.trigger('ConfirmationDialogClose', welcomeTitle)
    end
    if OpenXR.getEnable() then
      OpenXR.toggle()
    end
    extensions.unload(M)
  else
    log("E", "", "Unknown requested stateString: "..dumps(stateString)". Disabling...")
    M.setStateUI("disabled")
  end
end

local function toggle()
  if     M.stateString == "disabled" then M.setStateUI("welcome")
  elseif M.stateString == "welcome"  then M.setStateUI("enabled")
  else                                    M.setStateUI("disabled")
  end
end

local lastCenterRequest = false
local function center(value)
  if not value then
    -- this is a one-time center request (rather than a long continuous hold-and-release center request)
    center(1) -- begin centering
    framesUntilCenter = 1 -- stop centering after one frame (otherwise C++ simply cancels the centering request)
    return
  end
  local request = value > 0.2 and true or false
  OpenXR.center(request)
  if request == false and request ~= lastCenterRequest then
    log("D", "", "Headset centered, current OpenXR state:\n"..dumps(M.state))
  end
  lastCenterRequest = request
end

local function isSessionRunning()
  return M.state and M.state.sessionRunning or false
end

local function errorDetected(err)
  log("E", "", "An OpenXR error was detected with error ID: "..dumps(err))
  local translationId
  if not err then
    translationId = "unkown"
    log("E", "", "An OpenXR error was detected, but no error ID was provided")
  end
  if type(err) ~= "string" then
    translationId = "unkownType"
    log("E", "", string.format("An OpenXR error was detected, but a wrong error type was passed: %s ('%s')", type(err), dumps(err)))
  end
  translationId = "ui.openXR.errors."..(translationId or err)
  log("E", "", "openXR error detected: "..translateLanguage(translationId, "", false))
  guihooks.trigger("toastrMsg", {type = "error", title = "ui.openXR.errorsTitle", msg = translationId, config = { closeButton = true, timeOut = 0, extendedTimeOut = 0 } })
end

M.onUpdate = constOnUpdate
M.onInit = onInit
M.onSettingsChanged = onSettingsChanged
M.stateChanged = stateChanged
M.toggle = toggle
M.center = center
M.isSessionRunning = isSessionRunning
M.restart = restart
M.closeWelcome = closeWelcome
M.updateUI = updateUI
M.errorDetected = errorDetected

return M

