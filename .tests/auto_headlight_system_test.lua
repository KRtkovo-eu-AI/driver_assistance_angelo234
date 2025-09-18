local laura = require('laura')
local describe = laura.describe
local it = laura.it
local expect = laura.expect

package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = {}

local logger = require('scripts/driver_assistance_angelo234/logger')
logger.setEnabled(true)

local auto_headlight_system = require('scripts/driver_assistance_angelo234/autoHeadlightSystem')

describe('Auto Headlight Dimming', function()
  it('dims and restores headlights based on vehicles ahead', function()
    local logs = {}
    local previous_log = _G.log
    _G.log = function(level, tag, msg)
      table.insert(logs, {level, tag, msg})
    end

    local function run()
      _G.electrics_values_angelo234 = { lights_state = 2 }

      local veh = { commands = {} }
      function veh:queueLuaCommand(cmd)
        table.insert(self.commands, cmd)
      end

      auto_headlight_system.systemSwitchedOn()
      auto_headlight_system.update(0.25, veh, {})

      expect(function()
        auto_headlight_system.update(0.25, veh, nil)
      end).notToFail()

      auto_headlight_system.update(0.25, veh, { { other_veh = {}, shortest_dist = 100 } })
      expect(veh.commands[#veh.commands]).toEqual('electrics.highbeam = false; electrics.setLightsState(1)')

      _G.electrics_values_angelo234.lights_state = 1

      auto_headlight_system.update(0.25, veh, {})
      expect(veh.commands[#veh.commands]).toEqual('electrics.highbeam = true; electrics.setLightsState(2)')

      expect(#logs).toEqual(3)
      expect(logs[1][3]).toMatch('Detected vehicle')
      expect(logs[2][3]).toMatch('Attempting to switch to low beams')
      expect(logs[3][3]).toMatch('Attempting to restore high beams')
    end

    local ok, err = pcall(run)
    _G.log = previous_log
    if not ok then
      error(err)
    end
  end)
end)
