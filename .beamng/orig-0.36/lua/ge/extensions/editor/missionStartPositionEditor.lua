-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'editor_missionStartPositionEditor'
local actionMapName = "MissionStartPositionEditor"
local editModeName = "Edit Mission Start Position"
local im = ui_imgui

-- State variables
local selectedMission = nil
local isDragging = false
local dragStartPosition = nil
local oldPosition = nil
local nearbyMissions = {} -- Track nearby missions for highlighting

-- Helper functions
local function isMissionInList(mission, list)
  for _, m in ipairs(list) do
    if m == mission then
      return true
    end
  end
  return false
end

local function getMissionStartPosition(mission)
  if not mission or not mission.startTrigger then return nil end
  return vec3(mission.startTrigger.pos)
end

local function setMissionStartPosition(mission, position)
  if not mission or not mission.startTrigger then return end
  mission.startTrigger.pos = position:toTable()
  mission._dirty = true
end

local function findNearbyMissions(mission, maxDistance)
  if not mission then return {} end
  local currentLevel = getCurrentLevelIdentifier()
  local allMissions = gameplay_missions_missions.getFilesData() or {}
  local nearby = {}
  local missionPos = getMissionStartPosition(mission)

  if not missionPos then return {} end

  for _, otherMission in ipairs(allMissions) do
    if otherMission ~= mission and otherMission.startTrigger and otherMission.startTrigger.level == currentLevel then
      local otherPos = getMissionStartPosition(otherMission)
      if otherPos then
        local distance = (otherPos - missionPos):length()
        if distance <= maxDistance then
          table.insert(nearby, otherMission)
        end
      end
    end
  end
  return nearby
end

local function moveNearbyMissions(mission, maxDistance)
  if not mission then return end
  local missionPos = getMissionStartPosition(mission)
  if not missionPos then return end

  local nearby = findNearbyMissions(mission, maxDistance)
  local oldPositions = {}

  -- Store old positions and move missions
  for _, nearbyMission in ipairs(nearby) do
    oldPositions[nearbyMission] = getMissionStartPosition(nearbyMission)
    setMissionStartPosition(nearbyMission, missionPos)
  end

  -- Create undo/redo action
  editor.history:commitAction("MoveNearbyMissions",
    {missions = nearby, oldPositions = oldPositions, newPos = missionPos},
    function(data)
      for _, mission in ipairs(data.missions) do
        setMissionStartPosition(mission, data.oldPositions[mission])
      end
    end,
    function(data)
      for _, mission in ipairs(data.missions) do
        setMissionStartPosition(mission, data.newPos)
      end
    end
  )
end

-- Edit mode functions
local function onActivate()
  log('I', logTag, "onActivate")
  selectedMission = nil
  isDragging = false
  dragStartPosition = nil
  oldPosition = nil
end

local function onDeactivate()
  log('I', logTag, "onDeactivate")
  selectedMission = nil
  isDragging = false
  dragStartPosition = nil
  oldPosition = nil
end

local function onUpdate()
  -- Get all missions in the current level
  local currentLevel = getCurrentLevelIdentifier()
  local allMissions = gameplay_missions_missions.getFilesData() or {}

  -- Update nearby missions list for highlighting
  if selectedMission then
    nearbyMissions = findNearbyMissions(selectedMission, 10)
  else
    nearbyMissions = {}
  end

  -- Get mouse ray info
  local ray = getCameraMouseRay()
  local rayCast = cameraMouseRayCast()
  local mousePos3D = rayCast and vec3(rayCast.pos) or nil

  -- Get camera position for distance calculations
  local camPos = core_camera.getPosition()

  -- Handle Ctrl+click for direct positioning of selected mission
  if selectedMission and mousePos3D and im.IsMouseClicked(0) and editor.keyModifiers.ctrl then
    local oldPosition = getMissionStartPosition(selectedMission)
    setMissionStartPosition(selectedMission, mousePos3D)

    -- Create undo/redo action
    editor.history:commitAction("MoveMissionStart",
      {mission = selectedMission, oldPos = oldPosition, newPos = mousePos3D},
      function(data)
        setMissionStartPosition(data.mission, data.oldPos)
      end,
      function(data)
        setMissionStartPosition(data.mission, data.newPos)
      end
    )
    return -- Skip regular update after direct positioning
  end

  -- Draw all mission start positions
  for _, mission in ipairs(allMissions) do
    if mission.startTrigger and mission.startTrigger.level == currentLevel then
      local startPos = getMissionStartPosition(mission)
      if startPos then
        -- Calculate distance-based size with specific requirements
        local distToCam = (startPos - camPos):length()
        local isSelected = (mission == selectedMission)
        local isNearby = isMissionInList(mission, nearbyMissions)

        -- Linear interpolation between 3 at 500m and 10 at 1500m
        local size = 1.5 + (10 - 1.5) * (clamp(distToCam - 500, 0, 1000) / 1000)
        if isSelected then size = size * 1.3 end -- Make selected missions 30% larger
        if isNearby then size = size * 1.2 end -- Make nearby missions 20% larger

        -- Use different colors for selected vs other missions with 80% opacity
        local color
        if isSelected then
          color = ColorF(0, 1, 0, 0.8) -- Green for selected
        elseif isNearby then
          color = ColorF(1, 0.5, 0, 0.8) -- Orange for nearby
        else
          color = ColorF(0, 0, 1, 0.5) -- Blue for others
        end

        -- Draw the sphere
        debugDrawer:drawSphere(startPos, size, color)

        -- Draw the mission name with higher opacity
        local textColor = isSelected and ColorF(1, 1, 1, 1) or ColorF(1, 1, 1, 0.9)
        debugDrawer:drawText(startPos, mission.id, textColor)

        -- Handle mouse interaction for all missions
        if mousePos3D and not editor.keyModifiers.ctrl then -- Don't handle regular clicks when Ctrl is pressed
          local distance = (mousePos3D - startPos):length()

          -- If mouse is near the start position (use dynamic size for interaction)
          if distance < size * 2 then
            -- Show tooltip with mission info
            if im.IsItemHovered() then
              im.BeginTooltip()
              im.Text("Mission: " .. mission.id)
              im.Text("Type: " .. mission.missionType)
              if mission.name then
                im.Text("Name: " .. mission.name)
              end
              im.EndTooltip()
            end

            -- Handle selection
            if im.IsMouseClicked(0) then
              selectedMission = mission
              isDragging = true
              dragStartPosition = mousePos3D
              oldPosition = startPos
            end
          end
        end

        -- Handle dragging only for selected mission
        if isSelected and isDragging and im.IsMouseDown(0) then
          if mousePos3D then
            setMissionStartPosition(mission, mousePos3D)
          end
        end

        -- End dragging
        if isSelected and isDragging and im.IsMouseReleased(0) then
          isDragging = false
          if oldPosition then
            -- Create undo/redo action
            local newPosition = getMissionStartPosition(mission)
            editor.history:commitAction("MoveMissionStart",
              {mission = mission, oldPos = oldPosition, newPos = newPosition},
              function(data)
                setMissionStartPosition(data.mission, data.oldPos)
              end,
              function(data)
                setMissionStartPosition(data.mission, data.newPos)
              end
            )
          end
          dragStartPosition = nil
          oldPosition = nil
        end
      end
    end
  end
end

local function onToolbar()
  if not selectedMission then
    im.Text("No mission selected")
    return
  end

  im.Text("Mission: " .. selectedMission.id)
  im.Separator()

  local startPos = getMissionStartPosition(selectedMission)
  if startPos then
    im.Text(string.format("Start Position: %.2f, %.2f, %.2f", startPos.x, startPos.y, startPos.z))
  end

  im.Separator()

  -- Add button to move nearby missions
  if im.Button("Move Nearby Missions (10m)") then
    moveNearbyMissions(selectedMission, 10)
  end

  -- Show count of nearby missions in tooltip
  if im.IsItemHovered() then
    im.BeginTooltip()
    im.Text(string.format("%d missions within 10m", #nearbyMissions))
    im.EndTooltip()
  end
end

-- Initialize edit mode
local function onEditorInitialized()
  editor.editModes.missionStartPositionEditMode = {
    displayName = editModeName,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    onUpdate = onUpdate,
    onToolbar = onToolbar,
    actionMap = actionMapName,
    icon = editor.icons.flag,
    iconTooltip = "Mission Start Position Editor",
    auxShortcuts = {},
    hideObjectIcons = true
  }
end

-- Public interface
M.onEditorInitialized = onEditorInitialized
M.setSelectedMission = function(mission) selectedMission = mission end

return M