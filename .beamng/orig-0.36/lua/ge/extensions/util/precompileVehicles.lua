-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'freeroam_freeroam', 'core_vehicles'}

local logTag = 'precompileVehicles'

local vehicles = {}
local vehiclesToLoad = {
    ['atv'] = true,
    ['autobello'] = true,
    ['ball'] = true,
    ['barrels'] = true,
    ['barrier'] = true,
    ['barrier_plastic'] = true,
    ['barstow'] = true,
    ['bastion'] = true,
    ['blockwall'] = true,
    ['bluebuck'] = true,
    ['bolide'] = true,
    ['bollard'] = true,
    ['boxutility'] = true,
    ['boxutility_large'] = true,
    ['burnside'] = true,
    ['bx'] = true,
    ['cannon'] = true,
    ['caravan'] = true,
    ['cardboard_box'] = true,
    ['cargotrailer'] = true,
    ['chair'] = true,
    ['christmas_tree'] = true,
    ['citybus'] = true,
    ['common'] = true,
    ['cones'] = true,
    ['containerTrailer'] = true,
    ['couch'] = true,
    ['covet'] = true,
    ['delineator'] = true,
    ['dolly'] = true,
    ['dryvan'] = true,
    ['engine_props'] = true,
    ['etk800'] = true,
    ['etkc'] = true,
    ['etki'] = true,
    ['flail'] = true,
    ['flatbed'] = true,
    ['flipramp'] = true,
    ['frameless_dump'] = true,
    ['fridge'] = true,
    ['fullsize'] = true,
    ['gate'] = true,
    ['haybale'] = true,
    ['hopper'] = true,
    ['inflated_mat'] = true,
    ['kickplate'] = true,
    ['lansdale'] = true,
    ['large_angletester'] = true,
    ['large_bridge'] = true,
    ['large_cannon'] = true,
    ['large_crusher'] = true,
    ['large_hamster_wheel'] = true,
    ['large_roller'] = true,
    ['large_spinner'] = true,
    ['large_tilt'] = true,
    ['large_tire'] = true,
    ['legran'] = true,
    ['log_trailer'] = true,
    ['logs'] = true,
    ['mattress'] = true,
    ['metal_box'] = true,
    ['metal_ramp'] = true,
    ['midsize'] = true,
    ['midtruck'] = true,
    ['miramar'] = true,
    ['moonhawk'] = true,
    ['pessima'] = true,
    ['piano'] = true,
    ['pickup'] = true,
    ['pigeon'] = true,
    ['porta_potty'] = true,
    ['racetruck'] = true,
    ['roadsigns'] = true,
    ['roamer'] = true,
    ['rockbouncer'] = true,
    ['rocks'] = true,
    ['rollover'] = true,
    ['sawhorse'] = true,
    ['sbr'] = true,
    ['scintilla'] = true,
    ['shipping_container'] = true,
    ['steel_coil'] = true,
    ['streetlight'] = true,
    ['sunburst'] = true,
    ['suspensionbridge'] = true,
    ['tanker'] = true,
    ['testroller'] = true,
    ['tiltdeck'] = true,
    ['tirestacks'] = true,
    ['tirewall'] = true,
    ['trafficbarrel'] = true,
    ['tsfb'] = true,
    ['tub'] = true,
    ['tube'] = true,
    ['tv'] = true,
    ['unicycle'] = true,
    ['us_semi'] = true,
    ['utv'] = true,
    ['van'] = true,
    ['vivace'] = true,
    ['wall'] = true,
    ['weightpad'] = true,
    ['wendover'] = true,
    ['wigeon'] = true,
    ['woodcrate'] = true,
    ['woodplanks'] = true
}

local loaded = nil
local finished = false
local vehicleToLoad = 0
local frames = 0

local function loadNextVehicle()
    loaded = false
    local v = vehicles[vehicleToLoad]
    log('I', logTag, string.format('Loading vehicle %s (%d/%d).', v, vehicleToLoad, #vehicles))
    core_vehicles.replaceVehicle(v, {})
end

local function onInit()
    Engine.Render.setAsyncShaderCompilation(false)

    local allModels = core_vehicles.getModelList().models
    for model, _ in pairs(allModels) do
        if vehiclesToLoad[string.lower(model)] == true then
            table.insert(vehicles, model)
        end
    end

    log('I', logTag, string.format('Will precompile data for %d vehicles.', #vehicles))
    freeroam_freeroam.startFreeroam(path.getPathLevelMain('smallgrid'))
end

local function onVehicleSpawned(vehicleId)
    loaded = true
    frames = 200
end

local function onClientStartMission()
    vehicleToLoad = 1
    loadNextVehicle()
end

local function onPreRender(dt)
    if finished then return end

    if loaded and frames > 0 then
        frames = frames - 1
    end
    if loaded and frames <= 0 then
        vehicleToLoad = vehicleToLoad + 1

        if vehicleToLoad > #vehicles then
            log('I', logTag, 'Precompiled data for all vehicles.')
            finished = true
            quit()
        else
            loadNextVehicle()
        end
    end
end

M.onInit = onInit
M.onVehicleSpawned = onVehicleSpawned
M.onClientStartMission = onClientStartMission
M.onPreRender = onPreRender

return M