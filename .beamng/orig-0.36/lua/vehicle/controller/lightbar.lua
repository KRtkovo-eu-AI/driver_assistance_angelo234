-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min

local hasRegisteredQuickAccess = false
local lightbarElectric  -- Will store the electric value name

local modes = nil
local currentMode = nil
local currentModeIndex = 1
local lastLightbarElectric = -1

-- Updates electrical values that are static (non-animated) for the current mode
-- These are values that stay constant and don't change with pattern timing
local function updateOnceElectrics()
  -- Skip if no mode is selected
  if not currentMode then
    return
  end

  -- Apply each static electrical value
  for k, value in pairs(currentMode.electricsOnce) do
    -- Set the electrical output, scaled by the electric's value (capped at 1.0)
    -- This ensures the output changes properly when electric is reduced
    electrics.values[k] = value * min(electrics.values[lightbarElectric], 1)
  end
end

-- Main update function for animated lighting patterns
-- @param dt: Delta time since last frame in seconds
local function updateGFX(dt)
  -- Skip if lightbar system isn't active
  if not electrics.values[lightbarElectric] then
    return
  end

  -- If lightbar electrics value has changed, reset all patterns
  if lastLightbarElectric ~= electrics.values[lightbarElectric] then
    -- Reset all pattern timers and states
    for k, v in pairs(currentMode.electrics) do
      v.timer = 0 -- Reset pattern timing
      v.stateIndex = 1 -- Reset to first state
      electrics.values[k] = 0 -- Turn off output
    end
    lastLightbarElectric = electrics.values[lightbarElectric] -- Store new electrics value
    updateOnceElectrics(currentMode) -- Update static electrics values
  end

  -- Skip if no mode selected or lightbar is off
  if not currentMode or electrics.values[lightbarElectric] <= 0 then
    return
  end

  -- Update each electrics pattern
  for k, v in pairs(currentMode.electrics) do
    if electrics.values[lightbarElectric] > 0 then
      -- Advance pattern timer
      v.timer = v.timer + dt
      -- Check if it's time to move to next state
      if v.timer >= v.states[v.stateIndex].duration then
        -- Reset timer and advance to next state
        v.timer = v.timer - v.states[v.stateIndex].duration
        v.stateIndex = v.stateIndex + 1
        -- Loop back to start if pattern completed
        if v.stateIndex > #v.states then
          v.stateIndex = 1
        end
        -- Apply new state value, scaled by electrics value
        electrics.values[k] = v.states[v.stateIndex].value * clamp(electrics.values[lightbarElectric], 0, 1)
      end
    else
      -- Reset pattern if electric value is off
      v.timer = 0
      v.stateIndex = 1
      electrics.values[k] = 0
    end
  end
end

-- Sets the lightbar mode to a specific index and resets all patterns
-- @param index: The index of the desired mode in the modes table
local function setModeIndex(index)
  -- Update mode tracking variables
  if not modes[index] then
    return
  end
  currentModeIndex = index
  currentMode = modes[currentModeIndex]

  -- Reset all pattern states and timers for the new mode
  for k, v in pairs(currentMode.electrics) do
    v.timer = 0 -- Reset pattern timing
    v.stateIndex = 1 -- Start at first state
    electrics.values[k] = 0 -- Turn off output
  end

  -- Apply any static (non-animated) electrical values
  updateOnceElectrics(currentMode)

  -- Show mode change notification to user
  guihooks.message("Lightbar Mode: " .. currentMode.name, 5, "vehicle.lightbar.mode")
end

-- Cycles to the next available lightbar mode
local function toggleMode()
  currentModeIndex = currentModeIndex + 1
  -- Loop back to first mode if at end
  if currentModeIndex > #modes then
    currentModeIndex = 1
  end
  setModeIndex(currentModeIndex)
end

local function init(jbeamData)
  -- Load the electric value name from jbeam data, default to "lightbar" for backwards compatibility
  lightbarElectric = jbeamData.lightbarElectricsName or "lightbar"

  -- Process mode configurations from jbeam data
  modes = tableFromHeaderTable(jbeamData.modes)
  for _, vm in pairs(modes) do
    -- Convert mode config into usable format
    local configEntries = tableFromHeaderTable(deepcopy(vm.config))
    vm.config = nil
    vm.electrics = {} -- stores animated patterns
    vm.electricsOnce = {} -- stores static values

    -- Process each config entry into pattern states
    for _, j in pairs(configEntries) do
      -- Create new pattern if it doesn't exist
      if not vm.electrics[j.electric] then
        vm.electrics[j.electric] = {states = {}, timer = 0, stateIndex = 1}
      end
      -- Add this state to the pattern
      table.insert(vm.electrics[j.electric].states, {duration = j.duration, value = j.value})
    end

    -- Optimize: Move single-state patterns to electricsOnce for better performance
    for electricName, data in pairs(vm.electrics) do
      if #data.states == 1 then
        vm.electricsOnce[electricName] = data.states[1].value
        vm.electrics[electricName] = nil
      end
    end
  end

  -- Set initial mode and state
  currentModeIndex = jbeamData.defaultModeIndex or 1
  currentMode = modes[currentModeIndex]
  lastLightbarElectric = electrics.values[lightbarElectric]

  -- Register quick access menu if multiple modes exist
  if not hasRegisteredQuickAccess and #modes > 1 then
    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/",
        uniqueID = "lightbarmodes",
        title = "Lightbar Modes",
        icon = "wigwags",
        ["goto"] = "/root/playerVehicle/lights/lightbarmodes/"
      }
    )

    core_quickAccess.addEntry(
      {
        level = "/root/playerVehicle/lights/lightbarmodes/",
        generator = function(entries)
          -- Create menu entry for each available mode
          for k, v in pairs(modes) do
            local entry = {
              uniqueID = v.name .. jbeamData.cid, -- Unique identifier for this mode
              title = v.name, -- Display name
              icon = "wigwags", -- Menu icon
              originalActionInfo = {level = "/root/playerVehicle/lights/", uniqueID = "lightbarmodes"},
              onSelect = function()
                -- Handler when mode is selected
                setModeIndex(k)
                return {"reload"}
              end
            }
            -- Highlight currently active mode
            if currentModeIndex == k then
              entry.color = "#ff6600"
            end
            table.insert(entries, entry)
          end
        end
      }
    )
    hasRegisteredQuickAccess = true
  end
end

M.init = init
M.updateGFX = updateGFX
M.toggleMode = toggleMode
M.setModeIndex = setModeIndex

return M
