-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- simple example on how to use scenario vehicle extensions
-- how to use in scenario .json:

--[[
    "vehicles": {
        "scenario_player0": {
            "playerUsable": true,
            "startFocus": true,
            "extensions": {
                "annotate": {
                    "text": "This is your car, treat it well.",
                },
            },
        },
        "*": {
            "playerUsable": false
        }
    }
--]]

local data = {}

local function onDebugDraw(focusPos)
    local p1 = obj:getPosition() + vec3(0, 0, 2)
    obj.debugDrawProxy:drawText(p1, color(0,0,0,255), data.text)
    obj.debugDrawProxy:drawLine(obj:getPosition(), p1, color(0, 0, 0, 255))
end

local function onVehicleScenarioData(_data)
    print("### onVehicleScenarioData ###")
    data = deepcopy(_data)
    if type(data.text) ~= 'string' then
        log('E', 'annotate', 'invalid text for annotation extension')
    end
    dump(data)
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleScenarioData = onVehicleScenarioData
M.onDebugDraw = onDebugDraw

return M
