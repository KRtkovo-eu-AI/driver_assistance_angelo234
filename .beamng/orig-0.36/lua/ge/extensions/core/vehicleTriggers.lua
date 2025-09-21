-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.core_vehicleTriggers.enableDebugUI()

local im = ui_imgui
local toolWindowName = "Vehicle event debug"
local maxTriggerDistance = 1000

local M = { state = {} }
M.state.cefVisible = true
M.state.cursorVisibility = true
M.state.mouseLocked = false
M.state.cursorVisible = M.state.cursorVisibility and not M.state.mouseLocked

local currentlyUsedTrigger = nil
local fpsLimiter = newFPSLimiter(20)

local debugUIEnabled = false
local highLightedTriggerData = nil
local debugTimer = 0

local function isAnyControllerConnected()
  local inputDevices = WinInput.getRegisteredDevices()
  for _, d in ipairs(inputDevices) do
    if d ~= 'mouse0' and d ~= 'keyboard0' then
      return true
    end
  end
  return false
end

local function queueCmd(vehId, cmd)
  local vehObj = scenetree.findObject(vehId)
  if vehObj then
    vehObj:queueLuaCommand(cmd)
    if debugUIEnabled then
      log('I', 'triggers', 'Executing trigger code: ' .. tostring(cmd))
    end
  end
end

local function _replaceCmd(cmd, actionValue, vehicleId)
  cmd = cmd:gsub("VALUE", tostring(actionValue))
  cmd = cmd:gsub("VEHICLEID", tostring(vehicleId))
  -- TODO: improve hardcoded filter, etc
  cmd = cmd:gsub("FILTERTYPE", '-1')
  cmd = cmd:gsub("PLAYER", '0')
  cmd = cmd:gsub("ANGLE", '900')
  cmd = cmd:gsub("LOCKTYPE", '0')
  return cmd
end

-- returns the executed action count
local function executeLinkCommand(evt, actionValue, vehicleId)
  if evt.ctx == nil or evt.ctx == 'vlua' then
    -- send to vehicle
    if evt.onDown and actionValue == 1 then
      local cmdStr = _replaceCmd(evt.onDown, actionValue, vehicleId)
      return queueCmd(vehicleId, cmdStr)
    elseif evt.onUp and actionValue == 0 then
      local cmdStr = _replaceCmd(evt.onUp, actionValue, vehicleId)
      return queueCmd(vehicleId, cmdStr)
    elseif evt.onChange then
      local cmdStr = _replaceCmd(evt.onChange, actionValue, vehicleId)
      return queueCmd(vehicleId, cmdStr)
    end
  elseif evt.ctx == 'elua' or evt.ctx == 'tlua' then
    -- GE
    if evt.onDown and actionValue == 1 then
      local cmdStr = _replaceCmd(evt.onDown, actionValue, vehicleId)
      Lua:queueLuaCommand(cmdStr)
      return 1
    elseif evt.onUp and actionValue == 0 then
      local cmdStr = _replaceCmd(evt.onUp, actionValue, vehicleId)
      Lua:queueLuaCommand(cmdStr)
      return 1
    elseif evt.onChange then
      local cmdStr = _replaceCmd(evt.onChange, actionValue, vehicleId)
      Lua:queueLuaCommand(cmdStr)
      return 1
    end

  elseif evt.ctx == 'bvlua' then
    -- to all objects
    if evt.onDown and actionValue == 1 then
      local cmdStr = _replaceCmd(evt.onDown, actionValue, vehicleId)
      be:queueAllObjectLua(cmdStr)
      return 1
    elseif evt.onUp and actionValue == 0 then
      local cmdStr = _replaceCmd(evt.onUp, actionValue, vehicleId)
      be:queueAllObjectLua(cmdStr)
      return 1
    elseif evt.onChange then
      local cmdStr = _replaceCmd(evt.onChange, actionValue, vehicleId)
      be:queueAllObjectLua(cmdStr)
      return 1
    end
  end
  return 0
end

local function executeLink(vdata, lnk, actionValue, vehicleId)
  if lnk.version and lnk.version == 2 then
    --dump({'>>>>> executeLink', lnk.inputAction, actionValue, vehicleId})

    if lnk.namespace == 'vehicle' then
      if not vdata.inputActions[lnk.inputAction] then
        log('E', 'triggers', 'input action not found: ' .. tostring(lnk.inputAction))
        return 0
      end
      return executeLinkCommand(vdata.inputActions[lnk.inputAction], actionValue, vehicleId)
    elseif lnk.namespace == 'common' then
      if lnk.commonLua then
        if not vdata.inputActions[lnk.inputAction] then
          log('E', 'triggers', 'input action not found: ' .. tostring(lnk.inputAction))
          return 0
        end
        return executeLinkCommand(vdata.inputActions[lnk.inputAction], actionValue, vehicleId)
      end
      -- invoke c++ actionmap code
      if debugUIEnabled then
        ActionMap.debugEnabled = true
      end
      local triggerdBindingCount = ActionMap.triggerBindingByNameDigital(lnk.inputAction, actionValue > 0.9, os.clockhp(), vehicleId)
      if debugUIEnabled then
        ActionMap.debugEnabled = false
      end
      if debugUIEnabled and triggerdBindingCount == 0 then
        log('W', 'triggers', 'No binding triggered: ' .. tostring(lnk.inputAction) .. ' for value ' .. tostring(actionValue) )
      end
    end

  else
  -- old: backward compatibility, using event section
    return executeLinkCommand(lnk.targetEvent, actionValue, vehicleId)
  end
  return 0
end

local function drawDebugUI(dt)
  debugTimer = debugTimer + dt
  if debugTimer > 1000 then debugTimer = debugTimer - 1000 end -- prevent overflow or inprecision

  im.SetNextWindowSize(im.ImVec2(500, 500), im.Cond_FirstUseEver)
  if im.Begin(toolWindowName, openPtr) then
    local tableFlags = bit.bor(im.TableFlags_BordersV,
    im.TableFlags_BordersOuterH,
    im.TableFlags_Resizable,
    im.TableFlags_RowBg)

    for i = 0, be:getObjectCount() - 1 do
      local veh = be:getObject(i)
      local vehId = veh:getId()
      local vData = extensions.core_vehicle_manager.getVehicleData(vehId)


      local open = im.TreeNodeEx1("Vehicle " .. tostring(vehId) .. '##vehicle' .. tostring(vehId))
      im.SameLine()
      im.PushStyleColor2(im.Col_Text, im.ImVec4(0, 1, 0, 1))
      local title = ''
      if vData.vdata then
        title = tostring(vData.vdata.model)
      end
      im.TextUnformatted(title)
      im.PopStyleColor()
      im.SameLine()
      im.PushStyleColor2(im.Col_Text, im.ImVec4(0, 1, 1, 1))
      if vData.config then
        local dir, filename, ext = path.splitWithoutExt(tostring(vData.config.partConfigFilename))
        im.TextUnformatted(filename)
      end
      im.PopStyleColor()
      im.SameLine()
      im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 0, 1, 1))
      im.TextUnformatted(vehId == be:getPlayerVehicleID(0) and ' [ACTIVE]' or '')
      im.PopStyleColor()


      if open then
        local triggerCount = vData.vdata.maxIDs and vData.vdata.maxIDs.triggers or 0
        local open2 = im.TreeNodeEx1("Triggers##triggers"..tostring(vehId))
        im.SameLine()
        im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 1, 0, 1))
        im.TextUnformatted(tostring(triggerCount))
        im.PopStyleColor()

        if open2 then
          if im.BeginTable('Triggers##vehicleTriggers'..tostring(vehId), 5, tableFlags) then
            im.TableSetupScrollFreeze(0, 1) -- Make top row always visible
            im.TableSetupColumn("Id")
            im.TableSetupColumn("Name")
            im.TableSetupColumn("Action")
            im.TableSetupColumn("Namespace")
            im.TableSetupColumn("Controls")
            im.TableHeadersRow()
            im.TableNextRow()
            if vData and vData.vdata and type(vData.vdata.triggers) == 'table' then
              for _, trg in pairs(vData.vdata.triggers or {}) do

                for actionStr, lnkTable in pairs(vData.vdata.triggerEventLinksDict[trg.cid] or {}) do
                  if lnkTable and #lnkTable > 0 then
                    for lnkIdx, lnk in pairs(lnkTable) do

                      local isSelected = highLightedTriggerData and (highLightedTriggerData[1] == vehId and  highLightedTriggerData[2] == trg.cid)
                      if isSelected then
                        -- Push dark green color for selected row
                        im.PushStyleColor2(im.Col_TableRowBg, im.ImVec4(0.0, 0.5, 0.0, 1.0)) -- RGBA for dark green
                        im.PushStyleColor2(im.Col_TableRowBgAlt, im.ImVec4(0.0, 0.5, 0.0, 1.0)) -- Same color for alternating rows
                      end
                      im.TableNextColumn()
                      im.TextUnformatted(tostring(trg.cid))
                      im.TableNextColumn()
                      im.Text(translateLanguage(trg.name, trg.name, true))
                      im.TableNextColumn()

                      if lnk.triggerInput then
                        -- triggers2
                        im.TextUnformatted(tostring(lnk.inputAction))
                        im.TableNextColumn()
                        im.TextUnformatted(tostring(lnk.namespace))
                        if lnk.namespace == 'common' then
                          im.SameLine()
                          im.TextUnformatted(tostring(lnk.commonLua and "[LUA]" or "[C++]"))
                        end
                        if trg.originSection ~= 'triggers2' then
                          im.SameLine()
                          im.TextUnformatted(' (' .. tostring(trg.originSection) .. ')')
                        end
                        im.TableNextColumn()
                        im.SmallButton((tostring(lnk.triggerInput) or 'trigger') .. '##lnk2_'..tostring(lnk.cid)..'_'..tostring(vehId))

                        if im.IsItemHovered() and im.IsMouseClicked(0) then
                          local actionsExecuted = executeLink(vData.vdata, lnk, 1, vehId)
                          if debugUIEnabled and actionsExecuted == 0 then
                            log('E', 'triggers', 'Nothing executed on action [1]: '.. dumps({lnk, 1}))
                          end
                        end
                        if im.IsItemHovered() and im.IsMouseReleased(0) then
                          local actionsExecuted = executeLink(vData.vdata, lnk, 0, vehId)
                          if debugUIEnabled and actionsExecuted == 0 then
                            log('E', 'triggers', 'Nothing executed on action [2]: '.. dumps({lnk, 0}))
                          end
                        end
                      elseif lnk.targetEvent then
                        -- triggers (1)
                        im.TextUnformatted(tostring(lnk.action) .. ' - ' .. tostring(lnk.targetEvent.name))
                        im.SameLine()
                        im.SmallButton('trigger##lnk_'..tostring(lnk.cid)..'_'..tostring(vehId))
                        if im.IsItemHovered() and im.IsMouseClicked(0) then
                          local actionsExecuted = executeLink(vData.vdata, lnk, 1, vehId)
                          if debugUIEnabled and actionsExecuted == 0 then
                            log('E', 'triggers', 'Nothing executed on action [3]: '.. dumps({lnk, 1}))
                          end
                        end
                        if im.IsItemHovered() and im.IsMouseReleased(0) then
                          local actionsExecuted = executeLink(vData.vdata, lnk, 0, vehId)
                          if debugUIEnabled and actionsExecuted == 0 then
                            log('E', 'triggers', 'Nothing executed on action [4]: '.. dumps({lnk, 0}))
                          end
                        end
                        im.TableNextRow()
                      end

                      im.SameLine()
                      if not isSelected and im.SmallButton('highlight##highlight_'..tostring(trg.cid) .. '_' .. tostring(actionStr)) then
                        highLightedTriggerData = {vehId, trg.cid}
                      end

                      im.TableNextRow()
                      if isSelected then
                        im.PopStyleColor(2)
                      end

                    end
                  end
                end
              end
            end
            im.EndTable()
          end
          im.TreePop()
        end


        local eventsCount = vData.vdata.maxIDs and vData.vdata.maxIDs.events or 0
        local open3 = im.TreeNodeEx1("Events##Events"..tostring(vehId))
        im.SameLine()
        im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 1, 0, 1))
        im.TextUnformatted(tostring(eventsCount))
        im.PopStyleColor()

        if open3 then
          if im.BeginTable('Events##vehicleEventNames'..tostring(vehId), 4, tableFlags) then
            im.TableSetupScrollFreeze(0, 1) -- Make top row always visible
            im.TableSetupColumn("Id")
            im.TableSetupColumn("Name")
            im.TableSetupColumn("Description")
            im.TableSetupColumn("Controls")
            im.TableHeadersRow()
            if vData and vData.vdata and type(vData.vdata.events) == 'table' then
              for _, evt in pairs(vData.vdata.events or {}) do
                im.TableNextRow()
                im.TableNextColumn()
                im.TextUnformatted(tostring(evt.cid))
                im.TableNextColumn()
                im.Text(translateLanguage(evt.name, evt.name, true))
                im.TableNextColumn()
                im.Text(translateLanguage(evt.desc, evt.desc, true))
                im.TableNextColumn()

                im.SmallButton('trigger##u'..tostring(evt.cid)..'_'..tostring(vehId))
                if im.IsItemHovered() and im.IsMouseClicked(0) and evt.onDown then
                  queueCmd(vehId, evt.onDown)
                end
                if im.IsItemHovered() and im.IsMouseReleased(0) and evt.onUp then
                  queueCmd(vehId, evt.onUp)
                end
                im.SameLine()

                local sameLineNeeded = false
                if evt.onUp ~= nil then
                  if im.SmallButton('up##u'..tostring(evt.cid)..'_'..tostring(vehId)) then
                    queueCmd(vehId, evt.onUp)
                  end
                  sameLineNeeded = true
                end
                if evt.onDown ~= nil then
                  if sameLineNeeded then im.SameLine() end
                  if im.SmallButton('down##d'..tostring(evt.cid)..'_'..tostring(vehId)) then
                    queueCmd(vehId, evt.onDown)
                  end
                  sameLineNeeded = true
                end
                if evt.onChange then
                  if sameLineNeeded then im.SameLine() end
                  if im.SmallButton('-1##z'..tostring(evt.cid)..'_'..tostring(vehId)) then
                    local cmdStr = evt.onChange:gsub("VALUE", tostring(-1))
                    print('-1 - '..cmdStr)
                    queueCmd(vehId, cmdStr)
                  end
                  im.SameLine()
                  if im.SmallButton('0##z'..tostring(evt.cid)..'_'..tostring(vehId)) then
                    local cmdStr = evt.onChange:gsub("VALUE", tostring(0))
                    print('0 - '..cmdStr)
                    queueCmd(vehId, cmdStr)
                  end
                  im.SameLine()
                  if im.SmallButton('1##o'..tostring(evt.cid)..'_'..tostring(vehId)) then
                    local cmdStr = evt.onChange:gsub("VALUE", tostring(1))
                    print('1 - '..cmdStr)
                    queueCmd(vehId, cmdStr)
                  end
                end
              end
            end
            im.EndTable()
          end
          im.TreePop()
        end

        if vData.vdata and vData.vdata.maxIDs and not vData.vdata.maxIDs.triggerEventLinksDict then
          vData.vdata.maxIDs.triggerEventLinksDict = tableSize(vData.vdata.triggerEventLinksDict or {})
        end

        local eventsCount = vData.vdata.maxIDs and vData.vdata.maxIDs.triggerEventLinksDict or 0
        local open4 = im.TreeNodeEx1("TriggerEventLinks##TriggerEventLinks"..'_'..tostring(vehId))
        im.SameLine()
        im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 1, 0, 1))
        im.TextUnformatted(tostring(eventsCount))
        im.PopStyleColor()

        if open4 then
          if im.BeginTable('TriggerEventLinks##TriggerEventLinks'..tostring(vehId), 2, tableFlags) then
            im.TableSetupScrollFreeze(0, 1) -- Make top row always visible
            im.TableSetupColumn("TriggerId")
            im.TableSetupColumn("Controls")
            im.TableHeadersRow()

            if vData and vData.vdata and type(vData.vdata.triggerEventLinksDict) == 'table' then
              for triggerId, lnkDict in pairs(vData.vdata.triggerEventLinksDict or {}) do
                im.TableNextRow()
                im.TableNextColumn()
                im.TextUnformatted(tostring(triggerId))
                im.TableNextColumn()

                for actionStr, lnkTable in pairs(lnkDict) do
                  for _, lnk in pairs(lnkTable) do
                    if lnk.triggerInput then
                      -- triggers2
                      im.TextUnformatted(tostring(lnk.triggerInput) .. ' - ' .. tostring(lnk.inputAction))
                      im.SameLine()
                      im.SmallButton('trigger##u'..tostring(lnk.cid)..'_'..tostring(vehId))
                      if im.IsItemHovered() and im.IsMouseClicked(0) then
                        local actionsExecuted = executeLink(vData.vdata, lnk, 1, vehId)
                        if debugUIEnabled and actionsExecuted == 0 then
                          log('E', 'triggers', 'Nothing executed on action [5]: '.. dumps({lnk, 1}))
                        end
                      end
                      if im.IsItemHovered() and im.IsMouseReleased(0) then
                        local actionsExecuted = executeLink(vData.vdata, lnk, 0, vehId)
                        if debugUIEnabled and actionsExecuted == 0 then
                          log('E', 'triggers', 'Nothing executed on action [6]: '.. dumps({lnk, 0}))
                        end
                      end
                    elseif lnk.targetEvent then
                      -- triggers (1)
                      im.TextUnformatted(tostring(lnk.action) .. ' - ' .. tostring(lnk.targetEvent.name))
                      im.SameLine()
                      im.SmallButton('trigger##u'..tostring(lnk.cid)..'_'..tostring(vehId))
                      if im.IsItemHovered() and im.IsMouseClicked(0) then
                        local actionsExecuted = executeLink(vData.vdata, lnk, 1, vehId)
                        if debugUIEnabled and actionsExecuted == 0 then
                          log('E', 'triggers', 'Nothing executed on action [7]: '.. dumps({lnk, 1}))
                        end

                      end
                      if im.IsItemHovered() and im.IsMouseReleased(0) then
                        local actionsExecuted = executeLink(vData.vdata, lnk, 0, vehId)
                        if debugUIEnabled and actionsExecuted == 0 then
                          log('E', 'triggers', 'Nothing executed on action [8]: '.. dumps({lnk, 0}))
                        end
                      end
                    end
                  end
                end
              end
            end
            im.EndTable()
          end
          im.TreePop()
        end
        im.TreePop()
      end
    end
    im.End()
  end

  if highLightedTriggerData then
    local highLightedTriggerVehId = highLightedTriggerData[1]
    local highLightedTriggerId = highLightedTriggerData[2]
    local vData = extensions.core_vehicle_manager.getVehicleData(highLightedTriggerVehId)
    local veh = getObjectByID(highLightedTriggerVehId)
    if veh and vData and vData.vdata.triggers then
      local trg = vData.vdata.triggers[highLightedTriggerId]
      local to = veh:getTrigger(highLightedTriggerId)
      if trg and to then
        local pos = to:getCenter()
        local r = 0.1
        local col = ColorF(1,0,1,1)
        if trg.size then
          r = math.sqrt(trg.size.x ^ 2 + trg.size.y ^ 2 + trg.size.z ^ 2) / 2
        end
        --if trg.color then
        --  col.r = trg.color[1]
        --  col.g = trg.color[2]
        --  col.b = trg.color[3]
        --end

        col.alpha = 0.3 * math.sin(debugTimer * math.pi * 2) + 0.5

        debugDrawer:drawSphere(pos, r, col)

        local text = tostring(highLightedTriggerId) .. ' - ' .. tostring(trg.name) .. ' [' .. tostring(trg.originSection or 'triggers') .. ']'
        debugDrawer:drawTextAdvanced(pos, String(text), ColorF(1,1,1,1), true, false, ColorI(0,0,0,192))
      end
    end
  end
end

local function onCursorVisibilityChanged(visible)
  M.state.cursorVisibility = visible or VehicleTrigger.debug -- always visible in debug mode
  M.state.cursorVisible = M.state.cursorVisibility and not M.state.mouseLocked
end

local function onMouseLocked(locked)
  M.state.mouseLocked = locked
  M.state.cursorVisible = M.state.cursorVisibility and not M.state.mouseLocked
end

local function isEnabled()
  return (
    not photoModeOpen      -- always disallow in photomode
    and M.state.cefVisible -- cef must be visible
    and (
      M.state.cursorVisible   -- either cursor is visible...
      or currentlyUsedTrigger -- ...or a trigger is in use right now...
      or (                    -- ...or a non-kbdmouse device exists + camera was moved in driver/walking camera
        isAnyControllerConnected()
        and core_camera.timeSinceLastRotation() < 1000
        and isUnicycle
      )
    )
  )
end
local function onUpdate(dtReal, dtSim, dtRaw)

  if debugUIEnabled then
    drawDebugUI(dtReal)
  end

  if not M.state.cefVisible then return end
  local vehicleData = core_vehicle_manager and core_vehicle_manager.getPlayerVehicleData()
  local isUnicycle = vehicleData and vehicleData.mainPartName == "unicycle"

  local enabled = isEnabled()
  local renderFilterObjectId = isUnicycle and 0 or be:getPlayerVehicleID(0) -- restrict the triggers to your own vehicle unless you're in 1st person (you should use all vehicles triggers)
  renderFilterObjectId = 0 -- temporary change to allow interaction with all vehicles (such as attached trailers), to be reconsidered after some testing
  VehicleTrigger.renderFilterObjectId = renderFilterObjectId
  VehicleTrigger.renderingEnabled = enabled
  VehicleTrigger.enabled = enabled
  if not enabled then return end

  if currentlyUsedTrigger then
    -- highlight currently used trigger
    local vehicleObj = getObjectByID(currentlyUsedTrigger.v)
    if not vehicleObj then
      log("E", "", "Invalid vehicle id "..dumps(currentlyUsedTrigger.v).." for vehicle trigger "..dumps(currentlyUsedTrigger.t))
      return
    end
    --getPlayerVehicle(0):selectProp("rollback_lever_raise_R", 0) -- this will disable selection
    --getPlayerVehicle(0):selectProp("rollback_lever_raise_L", 1) -- first state... yellow
    --vehicleObj:selectProp("rollback_lever_raise_L", 2) -- second state... red
    local to = vehicleObj:getTrigger(currentlyUsedTrigger.t)
    if not to then
      log("E", "", "Invalid vehicle trigger "..dumps(currentlyUsedTrigger.t).." for vehicle id "..dumps(currentlyUsedTrigger.v))
      return
    end
    to:setUsedThisFrame() -- this will highlight the trigger visually
  else
    -- do not select another trigger if we are still using one
    if fpsLimiter:update(dtReal) then
      -- allow the c++ classes to draw the alpha according to the distance to this ray
      local useCursorCoordinates = M.state.cursorVisible
      be:triggerRaycastClosest(maxTriggerDistance, useCursorCoordinates)
    end
  end
end

local function triggerEvent(actionStr, actionValue, triggerId, vehicleId, vdata)
  if not vdata.triggerEventLinksDict then return end
  if type(vdata.triggerEventLinksDict[triggerId]) ~= 'table' then return end
  if type(vdata.triggerEventLinksDict[triggerId][actionStr]) ~= 'table' then return end

  -- TODO: this is overly simplistic and serves as a prototype :)
  for _, lnk in pairs(vdata.triggerEventLinksDict[triggerId][actionStr]) do
    local actionsExecuted = executeLink(vdata, lnk, actionValue, vehicleId)
    if debugUIEnabled and actionsExecuted == 0 then
      local valuetext = tostring(actionValue)
      if actionValue == 1 then
        valuetext = valuetext .. ' [DOWN]'
      else
        valuetext = valuetext .. ' [UP]'
      end
      log('W', 'triggers', 'Nothing executed on value '.. valuetext .. ' for triggerEvent: ' .. dumps({lnk, actionValue}))
    end
  end
end

-- executed by C++ side (typically VR controllers)
local function triggerEventWithoutVdata(actionNum, actionValue, triggerId, vehicleId)
  local vData = extensions.core_vehicle_manager.getVehicleData(vehicleId)
  local actionStr = 'action'..tostring(actionNum)
  return triggerEvent(actionStr, actionValue, triggerId, vehicleId, vData.vdata)
end

local currentTriggerHit
-- typically executed by the input actions "triggerAction0" 1 and 2
local function onActionEvent(actionNumber, inputValue)
  -- dump{'triggers.onActionEvent', actionNumber, inputValue}
  if not isEnabled() then return end
  if inputValue == 0 and currentlyUsedTrigger then
    currentTriggerHit = currentlyUsedTrigger
    currentlyUsedTrigger = nil
  else
    currentTriggerHit = be:triggerRaycastClosest(maxTriggerDistance, M.state.cursorVisible)
  end
  if not currentTriggerHit then return end

  local vData = extensions.core_vehicle_manager.getVehicleData(currentTriggerHit.v)
  if vData and vData.vdata and type(vData.vdata.triggers) == 'table' then
    local trigger = vData.vdata.triggers[currentTriggerHit.t]
    if trigger then
      triggerEvent('action' .. tostring(actionNumber), inputValue, currentTriggerHit.t, currentTriggerHit.v, vData.vdata)
      if inputValue ~= 0 then
        currentlyUsedTrigger = currentTriggerHit
      end
    end
  end
end

local function onCefVisibilityChanged(cefVisible)
  M.state.cefVisible = cefVisible
end

local function enableDebugUI()
  debugUIEnabled = true
end

local function onSerialize()
  return {
    debugUIEnabled = debugUIEnabled,
    highLightedTriggerData = highLightedTriggerData,
  }
end

local function onDeserialized(data)
  if data then
    debugUIEnabled = data.debugUIEnabled
    highLightedTriggerData = data.highLightedTriggerData
  end
end

M.onCefVisibilityChanged = onCefVisibilityChanged
M.onUpdate = onUpdate
M.onActionEvent = onActionEvent
M.triggerEvent = triggerEvent
M.triggerEventWithoutVdata = triggerEventWithoutVdata
M.onCursorVisibilityChanged = onCursorVisibilityChanged
M.onMouseLocked = onMouseLocked
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.enableDebugUI = enableDebugUI

return M
