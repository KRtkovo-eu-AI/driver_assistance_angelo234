-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_levels'}

local logTag = 'precompileShaders'

local levels = {}
local levelsToLoad = {
    ['automation_test_track'] = true,
    ['autotest'] = true,
    ['cliff'] = true,
    ['derby'] = true,
    ['driver_training'] = true,
    ['east_coast_usa'] = true,
    ['garage_v2'] = true,
    ['glow_city'] = true,
    ['gridmap_v2'] = true,
    ['hirochi_raceway'] = true,
    ['industrial'] = true,
    ['italy'] = true,
    ['johnson_valley'] = true,
    ['jungle_rock_island'] = true,
    ['showroom_v2'] = true,
    ['small_island'] = true,
    ['smallgrid'] = true,
    ['tech_ground'] = true,
    ['template'] = true,
    ['utah'] = true,
    ['west_coast_usa'] = true,
}

local loaded = nil
local finished = false
local levelToLoad = 0

local function loadNextLevel()
    loaded = false
    local v = levels[levelToLoad]
    log('I', logTag, string.format('Loading level %s (%d/%d).', v.levelName, levelToLoad, #levels))
    if string.find(v.fullfilename, '.mis') then
        core_levels.startLevel(v.fullfilename)
    else
        core_levels.startLevel(path.getPathLevelMain(v.levelName))
    end
end

local function onInit()
    local allLevels = core_levels.getList()
    for _, value in pairs(allLevels) do
        if levelsToLoad[string.lower(value.levelName)] == true then
            table.insert(levels, value)
        end
    end
    Engine.Render.setAsyncShaderCompilation(false)

    log('I', logTag, string.format('Will precompile data for %d levels.', #levels))
    levelToLoad = 1
    loadNextLevel()
end

local function onClientStartMission()
    loaded = true
end

local function onPreRender(dt)
    if finished then return end

    if loaded then
        levelToLoad = levelToLoad + 1

        if levelToLoad > #levels then
            log('I', logTag, 'Precompiled data for all levels.')
            finished = true
            quit()
        else
            loadNextLevel()
        end
    end
end

M.onInit = onInit
M.onClientStartMission = onClientStartMission
M.onPreRender = onPreRender

return M