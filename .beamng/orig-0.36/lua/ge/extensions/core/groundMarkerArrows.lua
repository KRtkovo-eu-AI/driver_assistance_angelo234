-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local arrowSmall, arrowLarge = 3, 4.5
local upVec = vec3(0,0,1)
local arrowHeight = vec3(0,0,3.5)
local wrongDirectionCounter = -20

-- Proxy system for arrow management
local arrowProxies = {} -- id -> {pos, dir, state, wp, nudgeForwardSmoother, nearFarScaleSmoother}
local wpToArrowId = {} -- wp -> id
local arrowToWp = {} -- id -> wp

local function getUnusedArrow()
  -- Look through proxies to find an unused arrow
  for id, proxy in pairs(arrowProxies) do
    if proxy.state == "unused" then
      --log('I', 'arrow', 'Found unused arrow in proxies: ' .. tostring(id))
      return scenetree.findObjectById(id)
    end
  end
  --log('W', 'arrow', 'No unused arrows found in proxies')
  return nil
end

local function clearArrows()
  --log('I', 'arrow', 'Clearing arrows')
  -- Delete all arrow objects through their proxies
  for id, proxy in pairs(arrowProxies) do
    local arrow = scenetree.findObjectById(id)
    if arrow then arrow:delete() end
  end

  -- Clear all tracking tables
  arrowProxies = {}
  wpToArrowId = {}
  arrowToWp = {}
  wrongDirectionCounter = -20

  -- Clean up the pool if it exists
  local arrowPool = scenetree.findObject("arrowPool")
  if arrowPool then
    arrowPool:delete()
  end
end

local function createArrowPool(floatingArrowColor)
  local group = scenetree.findObject("arrowPool")
  if not group then
    --log('I', 'arrow', 'Creating new arrow pool')
    -- Create the group if there is none yet
    group = createObject("SimGroup")
    group:registerObject("arrowPool")
    group.canSave = false
    for i = 0, 10 do
      local arrow = createObject('TSStatic')
      arrow:setField('shapeName', 0, "art/shapes/interface/s_mm_arrow_floating.dae")
      arrow.scale = vec3(arrowScale, arrowScale, arrowScale)
      arrow.useInstanceRenderData = true
      --arrow:setField('instanceColor1', 0, "1 1 1 1")
      arrow:setField('collisionType', 0, "None")
      arrow:setField('decalType', 0, "None")

      arrow.canSave = false
      arrow:registerObject(Sim.getUniqueName("arrow"))
      group:addObject(arrow)

      -- Create proxy for this arrow
      local id = arrow:getId()
      arrowProxies[id] = {
        pos = vec3(0,0,0),
        rot = quat(0,0,0,1),
        state = "unused",
        wp = nil,
        nudgeForwardSmoother = newTemporalSmoothingNonLinear(3,3),
        nudgeForwardSmootherTarget = 0,
        nearFarScaleSmoother = newTemporalSmoothingNonLinear(15,15, arrowSmall),
        nearFarScaleSmootherTarget = 0,
        alphaSmoother = newTemporalSmoothingNonLinear(5,5, 0),
        alphaSmootherTarget = 0,
      }
      --log('I', 'arrow', 'Created arrow and proxy: ' .. tostring(id))
    end
  else
    --log('I', 'arrow', 'Using existing arrow pool: ' .. tostring(group:getId()))
    -- Reset all proxies to unused state
    for id, proxy in pairs(arrowProxies) do
      proxy.state = "unused"
      proxy.wp = nil
      proxy.nudgeForwardSmoother:set(0)
      proxy.nudgeForwardSmootherTarget = 0
      proxy.nearFarScaleSmoother:set(0)
      proxy.nearFarScaleSmootherTarget = 0
      proxy.alphaSmoother:set(0)
      proxy.alphaSmootherTarget = 0
      local arrow = scenetree.findObjectById(id)
      if arrow then
        arrow.hidden = true
      end
    end
    wpToArrowId = {}
    arrowToWp = {}
  end
end

local function setSmootherTargets(id, distToVehicle)
  arrowProxies[id].nudgeForwardSmootherTarget = distToVehicle > 45 and 0 or 1
  arrowProxies[id].nearFarScaleSmootherTarget = distToVehicle > 65 and arrowLarge or arrowSmall
  arrowProxies[id].alphaSmootherTarget = 1
end

--local lastWpLog = {}

local function updateArrows(path, pathLength)
  --log('I', 'arrow', 'Updating arrows')
  local usedWpIds = {}
  local maxArrowDistance = 180 -- Only show arrows up to 180m in front

  --table.clear(lastWpLog)

  for i = 1, #path - 1 do
    --local wpLog = {pos = path[i].pos}
    --wpLog.distToTarget = path[i].distToTarget
    local dirNextPoint = (path[i+1].pos - path[i].pos)
    dirNextPoint:normalize()
    if i > 1 then
      local dirPrevPoint = (path[i].pos - path[i-1].pos)
      dirPrevPoint:normalize()
      --wpLog.dirPrevPoint = dirPrevPoint

      -- Calculate distance from vehicle to this point
      local distToVehicle = pathLength - path[i].distToTarget
      --wpLog.distToVehicle = distToVehicle
      if distToVehicle > maxArrowDistance then
        break -- Stop processing points beyond max distance
      end

      --wpLog.nodeToNodeAngle = math.acos(dirPrevPoint:cosAngle(dirNextPoint)) * 180/math.pi
      --wpLog.wp = path[i].wp
      --wpLog.linkCount = path[i].linkCount
      if (path[i].linkCount and path[i].linkCount > 2) and path[i].wp then
        local nodeToNodeAngle = math.acos(dirPrevPoint:cosAngle(dirNextPoint)) * 180/math.pi

        -- Check if the route has the smallest angle of any possible path of the intersection
        local routeHasSmallestAngle = true
        if nodeToNodeAngle <= 25 then
          for wpId, edgeInfo in pairs(map.getGraphpath().graph[path[i].wp]) do
            if path[i-1].wp ~= wpId then
              local connectedNodePos = map.getMap().nodes[wpId].pos
              local dirConnectedNode = (connectedNodePos - path[i].pos); dirConnectedNode:normalize()
              local connectedNodeAngle = math.acos(dirPrevPoint:cosAngle(dirConnectedNode)) * 180/math.pi
              if nodeToNodeAngle > connectedNodeAngle then
                routeHasSmallestAngle = false
                break
              end
            end
          end
        end
        --wpLog.routeHasSmallestAngle = routeHasSmallestAngle
        if nodeToNodeAngle > 25 or not routeHasSmallestAngle then
          usedWpIds[path[i].wp] = true
          local existingArrowId = wpToArrowId[path[i].wp]

          if existingArrowId then
            setSmootherTargets(existingArrowId, distToVehicle)
          else
            -- Create new arrow
            local arrow = getUnusedArrow()
            if arrow then
              local id = arrow:getId()
              --log('I', 'arrow', 'Creating new arrow ' .. tostring(id) .. ' at wp ' .. tostring(path[i].wp))

              wpToArrowId[path[i].wp] = id
              arrowToWp[id] = path[i].wp

              local arrowColor = core_groundMarkers.floatingArrowColor
              arrow:setField('instanceColor', 0, ""..arrowColor[1].." "..arrowColor[2].." "..arrowColor[3].." 1")
              arrow:setField('instanceColor1', 0, ""..arrowColor[1].." "..arrowColor[2].." "..arrowColor[3].." 1")

              local pos = path[i].pos + arrowHeight
              local rot = quatFromDir(dirNextPoint, upVec)
              -- Update proxy
              arrowProxies[id].pos = pos
              arrowProxies[id].rot = rot
              arrowProxies[id].state = "visible"
              arrowProxies[id].wp = path[i].wp
              setSmootherTargets(id, distToVehicle)
              arrowProxies[id].nudgeForwardSmoother:set(arrowProxies[id].nudgeForwardSmootherTarget)
              arrowProxies[id].nearFarScaleSmoother:set(arrowProxies[id].nearFarScaleSmootherTarget)
              arrowProxies[id].alphaSmoother:set(0)
            else
              log('W', 'arrow', 'Failed to get unused arrow for wp ' .. tostring(path[i].wp))
            end
          end
        end
      end
    end
    --table.insert(lastWpLog, wpLog)
  end

  -- TODO commented out for after the update
  --[[ if path[2] then
    local wrongDirection = getPlayerVehicle(0):getVelocity():dot(path[2].pos - getPlayerVehicle(0):getPosition()) < 0
    if wrongDirection then
      wrongDirectionCounter = wrongDirectionCounter + 1
    else
      wrongDirectionCounter = wrongDirectionCounter - 1
    end

    wrongDirectionCounter = clamp(wrongDirectionCounter, -20, 20)
  end
  if wrongDirectionCounter > 0 then
    ui_message("Please make a U-turn when possible", 5, 'wrongDirection', 'directions_run')
  else
    ui_message("", 5, 'wrongDirection', 'directions_run')
  end ]]

  for id, proxy in pairs(arrowProxies) do
    if not usedWpIds[proxy.wp] then
      if proxy.state == "visible" then
        proxy.state = "fadeout"
        proxy.alphaSmootherTarget = -0.2
        arrowToWp[id] = nil
        wpToArrowId[proxy.wp] = nil
      end
    end
  end
end

local yVec = vec3(0,1.5,0)
local actualPos = vec3()
local fwd = vec3()
local scale = vec3()
local function onPreRender(dt)
  -- Update all arrow objects based on their proxies
  for id, proxy in pairs(arrowProxies) do
    if proxy.state == "unused" then
      goto continue
    end
    local arrow = scenetree.findObjectById(id)
    if arrow then
      actualPos:set(proxy.pos.x, proxy.pos.y, proxy.pos.z)
      fwd:setRotate(proxy.rot, yVec)
      local nudgeForwardValue = proxy.nudgeForwardSmoother:get(proxy.nudgeForwardSmootherTarget, dt) -- Fade in over 0.5 seconds
      actualPos:setAdd(push3(fwd) * (nudgeForwardValue - 0.75))

      local alphaValue = proxy.alphaSmoother:get(proxy.alphaSmootherTarget, dt)

      if proxy.state == "visible" then
        -- Update nudgeForwardSmoother to fade in
        arrow.hidden = false
      elseif proxy.state == "fadeout" then
        if alphaValue <= 0 then
          -- Transition complete, set to unused
          proxy.state = "unused"
          arrow.hidden = true
        end
      end

      -- Update scale based on distance
      local scaleVal = proxy.nearFarScaleSmoother:get(proxy.nearFarScaleSmootherTarget, dt) * alphaValue
      scale:set(scaleVal, scaleVal, scaleVal)
      arrow:setScale(scale)

      arrow:setPosRot(actualPos.x, actualPos.y, actualPos.z, proxy.rot.x, proxy.rot.y, proxy.rot.z, proxy.rot.w)
      arrow:updateInstanceRenderData()

      --simpleDebugText3d(string.format("Arrow %s: state=%s", id, proxy.state), proxy.pos)
    else
      log('W', 'arrow', 'Arrow object not found for id: ' .. tostring(id))
    end
    ::continue::
  end

  --[[
  for wpId, wpLog in pairs(lastWpLog) do
    simpleDebugText3d(string.format("Wp %s: nodeToNodeAngle=%0.3f, %s, %s, links: %d, %0.1f", wpId, wpLog.nodeToNodeAngle or -1, wpLog.routeHasSmallestAngle and "smallest Angle" or "", wpLog.wp, wpLog.linkCount or -1, wpLog.distToVehicle or -1), wpLog.pos)
  end
  local im = ui_imgui
  -- Draw ImGui debug window
  if im.Begin("Arrow Debug") then
    im.Text("Arrow Pool Status")
    im.Separator()

    -- Show active arrows
    im.Text("Active Arrows:")
    if im.BeginTable("activeArrows", 6, tableFlags) then
      im.TableNextColumn()
      im.Text("ID")
      im.TableNextColumn()
      im.Text("State")
      im.TableNextColumn()
      im.Text("WP")
      im.TableNextColumn()
      im.Text("Position")
      im.TableNextColumn()
      im.Text("Scale")
      im.TableNextColumn()
      im.Text("Actions")

      for id, proxy in pairs(arrowProxies) do
        if proxy.state ~= "unused" then
          im.TableNextColumn()
          im.Text(tostring(id))
          im.TableNextColumn()
          im.Text(proxy.state)
          im.TableNextColumn()
          im.Text(tostring(proxy.wp or "none"))
          im.TableNextColumn()
          im.Text(string.format("%.1f, %.1f, %.1f", proxy.pos.x, proxy.pos.y, proxy.pos.z))
          im.TableNextColumn()
          local scale = proxy.nearFarScaleSmoother:get(proxy.nearFarScaleSmootherTarget, 0)
          im.Text(string.format("%.2f", scale))
          im.TableNextColumn()
          if im.Button("Hide##"..id) then
            proxy.state = "fadeout"
            proxy.alphaSmootherTarget = -0.2
            arrowToWp[id] = nil
            wpToArrowId[proxy.wp] = nil
          end
        end
      end
      im.EndTable()
    end

    -- Show unused arrows
    im.Separator()
    im.Text("Unused Arrows:")
    if im.BeginTable("unusedArrows", 2, tableFlags) then
      im.TableNextColumn()
      im.Text("ID")
      im.TableNextColumn()
      im.Text("Actions")

      for id, proxy in pairs(arrowProxies) do
        if proxy.state == "unused" then
          im.TableNextColumn()
          im.Text(tostring(id))
          im.TableNextColumn()
          if im.Button("Delete##"..id) then
            local arrow = scenetree.findObjectById(id)
            if arrow then arrow:delete() end
            arrowProxies[id] = nil
          end
        end
      end
      im.EndTable()
    end

    -- Show waypoint mappings
    im.Separator()
    im.Text("Waypoint Mappings:")
    if im.BeginTable("wpMappings", 3, tableFlags) then
      im.TableNextColumn()
      im.Text("Waypoint ID")
      im.TableNextColumn()
      im.Text("Arrow ID")
      im.TableNextColumn()
      im.Text("Status")

      -- Show wpToArrowId mappings
      for wp, arrowId in pairs(wpToArrowId) do
        im.TableNextColumn()
        im.Text(tostring(wp))
        im.TableNextColumn()
        im.Text(tostring(arrowId))
        im.TableNextColumn()
        local proxy = arrowProxies[arrowId]
        if proxy then
          im.Text(string.format("%s (Scale: %.2f)", proxy.state, proxy.nearFarScaleSmoother:get(proxy.nearFarScaleSmootherTarget, 0)))
        else
          im.TextColored(im.ImVec4(1,0,0,1), "Missing Proxy!")
        end
      end
      im.EndTable()
    end

    -- Show path information
    im.Separator()
    im.Text("Path Information:")
    if im.BeginTable("pathInfo", 4, tableFlags) then
      im.TableNextColumn()
      im.Text("Index")
      im.TableNextColumn()
      im.Text("WP")
      im.TableNextColumn()
      im.Text("Distance")
      im.TableNextColumn()
      im.Text("Angle")

      for i, wpLog in ipairs(lastWpLog) do
        im.TableNextColumn()
        im.Text(tostring(i))
        im.TableNextColumn()
        im.Text(tostring(wpLog.wp or "none"))
        im.TableNextColumn()
        im.Text(string.format("%.1f", wpLog.distToVehicle or -1))
        im.TableNextColumn()
        im.Text(string.format("%.1f°", wpLog.nodeToNodeAngle or -1))
      end
      im.EndTable()
    end

    im.End()
  end
  ]]
end

M.createArrowPool = createArrowPool
M.updateArrows = updateArrows
M.clearArrows = clearArrows
M.onPreRender = onPreRender

return M
