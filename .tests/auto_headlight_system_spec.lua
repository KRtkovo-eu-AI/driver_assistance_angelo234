local busted = require('busted')

package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = {}

local logs = {}
_G.log = function(level, tag, msg)
  table.insert(logs, {level, tag, msg})
end

local auto_headlight_system = require('scripts/driver_assistance_angelo234/autoHeadlightSystem')

describe('Auto Headlight Dimming', function()
  it('dims and restores headlights based on vehicles ahead', function()
    logs = {}
    -- global variable used by module to read light state
    _G.electrics_values_angelo234 = { lights_state = 2 }

    local veh = { commands = {} }
    function veh:queueLuaCommand(cmd)
      table.insert(self.commands, cmd)
    end

    -- system switched on with high beams
    auto_headlight_system.systemSwitchedOn()

    -- first update to arm the system (no other vehicles)
    auto_headlight_system.update(0.25, veh, {})

    -- ensure no error when sensor data is missing
    assert.has_no.errors(function()
      auto_headlight_system.update(0.25, veh, nil)
    end)

    -- simulate vehicle ahead at 100 units
    auto_headlight_system.update(0.25, veh, { { other_veh = {}, shortest_dist = 100 } })
    assert.are.equal('electrics.highbeam = false; electrics.setLightsState(1)', veh.commands[#veh.commands])

    -- update electrics state to reflect low beams being set
    _G.electrics_values_angelo234.lights_state = 1

    -- no vehicle ahead anymore
    auto_headlight_system.update(0.25, veh, {})
    assert.are.equal('electrics.highbeam = true; electrics.setLightsState(2)', veh.commands[#veh.commands])

    assert.equal(3, #logs)
    assert.matches('Detected vehicle', logs[1][3])
    assert.matches('Attempting to switch to low beams', logs[2][3])
    assert.matches('Attempting to restore high beams', logs[3][3])
  end)
end)

