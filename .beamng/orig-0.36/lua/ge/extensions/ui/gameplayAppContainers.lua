-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local debug = false

local appContainersById = {
  ['gameplayApps'] = {
    contexts = {
      rally = true,
      drift = true,
      drag = true,
      pointsBar = true,
    },
    currentContext = nil,
    trigger = 'setGameplayAppContext',
  }
}

local function setContainerContext(containerId, context)
  if not appContainersById[containerId] then
    log('E', 'gameplayAppContainers', 'container not found: ' .. containerId)
    return
  end
  local container = appContainersById[containerId]
  if not container.contexts[context] then
    log('E', 'gameplayAppContainers', 'context not found: ' .. context .. ' for container: ' .. containerId)
    return
  end
  if container.currentContext == context then
    log('I', 'gameplayAppContainers', 'context already set to: ' .. context .. ' for container: ' .. containerId)
    return
  end
  if container.currentContext and context then
    log("E", "gameplayAppContainers", "context already set to: " .. container.currentContext .. " for container: " .. containerId .. " - cannot set to: " .. context .. ". Please reset the context first.")
    return
  end

  container.currentContext = context
  log("I", "gameplayAppContainers", "setting context to: " .. context .. " for container: " .. containerId)
  guihooks.trigger(container.trigger, {
    context = context,
  })
end

local function resetContainerContext(containerId)
  if not appContainersById[containerId] then
    log('E', 'gameplayAppContainers', 'container not found: ' .. containerId)
    return
  end
  local container = appContainersById[containerId]
  container.currentContext = nil
  log("I", "gameplayAppContainers", "resetting context for container: " .. containerId)
  guihooks.trigger(container.trigger, {
    context = nil,
  })
  extensions.hook("onUIContainerContextReset")
end

local function getContainerContext(containerId)
  if not appContainersById[containerId] then
    log('E', 'gameplayAppContainers', 'container not found: ' .. containerId)
    return nil
  end
  return appContainersById[containerId].currentContext
end

local function onSerialize()
  local data = {}
  for containerId, container in pairs(appContainersById) do
    data[containerId] = container.currentContext
    resetContainerContext(containerId)
  end
  return data
end

local function onDeserialize(data)
  for containerId, context in pairs(data) do
    setContainerContext(containerId, context)
  end
end

local function getAvailableContexts(containerId)
  if not appContainersById[containerId] then
    log('E', 'gameplayAppContainers', 'container not found: ' .. containerId)
    return nil
  end
  return appContainersById[containerId].contexts
end

M.setContainerContext = setContainerContext
M.getContainerContext = getContainerContext
M.resetContainerContext = resetContainerContext
M.getAvailableContexts = getAvailableContexts

M.onSerialize = onSerialize
M.onDeserialize = onDeserialize



-- debug code
local im
if debug then

im = ui_imgui

local function onUpdate()
  if not im then return end

  im.Begin("Gameplay App Containers Debug")

  for containerId, container in pairs(appContainersById) do
    im.Text("Container: " .. containerId)
    -- Show current context
    im.Text("Current Context: " .. (container.currentContext or "none"))

    -- Allow setting context through combo
    if im.BeginCombo("Set Context", container.currentContext or "none") then
      -- Add "none" option to reset
      if im.Selectable1("none", container.currentContext == nil) then
        resetContainerContext(containerId)
      end

      -- Add all available contexts
      for _, context in ipairs(tableKeysSorted(container.contexts)) do
        if im.Selectable1(context, context == container.currentContext) then
          setContainerContext(containerId, context)
        end
      end
      im.EndCombo()
    end

    -- Show available contexts
    im.Text("Available Contexts:")
    for _, context in ipairs(tableKeysSorted(container.contexts)) do
      im.BulletText(context)
    end

  end

  im.End()
end

M.onUpdate = onUpdate
end

return M
