-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "serverConnection"

local function onCameraHandlerSetInitial()
  if worldReadyState == 0 then
    local canvas = scenetree.findObject("Canvas")
    if canvas then
      canvas:enableCursorHideIfMouseInactive(true)
    else
      log('E', logTag, 'canvas not found')
    end
    -- The first control object has been set by the server
    -- and we are now ready to go.

    log('D', logTag, 'Everything should be loaded setting worldReadyState to 1')
    worldReadyState = 1 -- should be ready, wait for the vehicle to be done, then switch. stage 2 is in the frame update
    extensions.hook('onWorldReadyState', worldReadyState)
  end
end

--trigger by starting game and then starting a new level
local function disconnectActual(callback, loadingScreen, p)
  -- We need to stop the client side simulation
  -- else physics resources will not cleanup properly.
  be:physicsStopSimulation()
  if p then p:add("disconnectActual.physicsStopSimulation") end
  local canvas = scenetree.findObject("Canvas")
  if p then p:add("disconnectActual.findCanvas") end
  if not canvas then
    log('E', logTag, 'canvas not found')
    if p then p:add("disconnectActual.canvasError") end
  else
    canvas:enableCursorHideIfMouseInactive(false);
    if p then p:add("disconnectActual.hideMouse") end
  end

  -- Disable mission lighting if it's going, this is here
  -- in case we're disconnected while the mission is loading.

  TorqueScriptLua.setVar("$lightingMission", "false")
  TorqueScriptLua.setVar("$sceneLighting::terminateLighting", "true")
  if p then p:add("disconnectActual.setVars") end

  -- Call destroyServer in case we're hosting
  server.destroy(p)
  if p then p:add("disconnectActual.server") end
  setMissionFilename("")
  if p then p:add("disconnectActual.filename") end
  if loadingScreen then core_gamestate.requestExitLoadingScreen(logTag) end
  if p then p:add("disconnectActual.requestExitLoadingScreen") end
  local ret
  if callback then
    ret = callback()
  end
  if p then p:add("disconnectActual.callback") end
  return ret
end

local function disconnectWrapper (callback, loadingScreen, p)
  if loadingScreen == nil then loadingScreen = true end
  if p then p:add("disconnectWrapper.bool") end
  local function help (p)
    disconnectActual(callback, loadingScreen, p)
  end
  if loadingScreen then
    core_gamestate.requestEnterLoadingScreen(logTag, help)
    if p then p:add("disconnectWrapper.requestEnterLoadingScreen") end
  else
    help(p)
    if p then p:add("disconnectWrapper.help") end
  end
end

-- TODO: clean this up, but not call disconnectActual directly it just will result in the gamestate getting mixed messages
local function noLoadingScreenDisconnect (p)
  disconnectWrapper(nop, false, p)
end

M.onCameraHandlerSetInitial = onCameraHandlerSetInitial
M.disconnect = disconnectWrapper
M.noLoadingScreenDisconnect = noLoadingScreenDisconnect -- used in .cs
return M