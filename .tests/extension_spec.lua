local busted = require('busted')

describe('extension', function()
  before_each(function()
    -- clear loaded module to allow fresh environment per test
    package.loaded['scripts/driver_assistance_angelo234/extension'] = nil
    package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = nil
    package.loaded['scripts/driver_assistance_angelo234/sensorSystem'] = nil
    package.loaded['scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem'] = nil
    package.loaded['scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem'] = nil
    package.loaded['scripts/driver_assistance_angelo234/accSystem'] = nil
    package.loaded['scripts/driver_assistance_angelo234/hillStartAssistSystem'] = nil
    package.loaded['scripts/driver_assistance_angelo234/autoHeadlightSystem'] = nil
  end)

  it('does not call missing init functions on first update', function()
    -- stub dependencies with minimal placeholders
    package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = {}
    package.loaded['scripts/driver_assistance_angelo234/sensorSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/accSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/hillStartAssistSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/autoHeadlightSystem'] = {
      onHeadlightsOff = function() end,
      onHeadlightsOn = function() end,
      systemSwitchedOn = function() end,
      update = function() end,
    }

    _G.be = { getPlayerVehicle = function() return nil end }

    _G.newExponentialSmoothing = function()
      return { get = function(self, v) return v end }
    end

    local extension = require('scripts/driver_assistance_angelo234/extension')
    assert.has_no.errors(function() extension.onUpdate(0.1) end)
  end)

  it('polls front sensors when only auto headlight system is active', function()
    -- track calls to front sensors and auto headlight update
    local poll_called = 0
    local passed_vehs

    local sample_front_data = {0, { {other_veh = 'dummy', shortest_dist = 1} }, {}}

    package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = {
      getPart = function(part)
        if part == 'auto_headlight_angelo234' then return true end
        return nil
      end,
      getVehicleProperties = function() return {id = 1} end
    }
    package.loaded['scripts/driver_assistance_angelo234/sensorSystem'] = {
      pollFrontSensors = function()
        poll_called = poll_called + 1
        return sample_front_data
      end,
      pollRearSensors = function() return {} end,
    }
    package.loaded['scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/accSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/hillStartAssistSystem'] = {}
    package.loaded['scripts/driver_assistance_angelo234/autoHeadlightSystem'] = {
      onHeadlightsOff = function() end,
      onHeadlightsOn = function() end,
      systemSwitchedOn = function() end,
      update = function(_, _, vehs) passed_vehs = vehs end,
    }

    _G.be = {
      getPlayerVehicle = function()
        return { getJBeamFilename = function() return 'common' end, queueLuaCommand = function() end }
      end,
      getObjectCount = function() return 0 end,
      getObject = function() return nil end,
      getEnabled = function() return true end
    }
    _G.FS = { fileExists = function() return false end }
    _G.ui_message = function() end

    _G.newExponentialSmoothing = function() return { get = function(self, v) return v end } end
    _G.jsonDecode = function(x) return x end
    _G.jsonEncode = function(x) return x end

    -- globals required for sensor data gathering
    _G.veh_accs_angelo234 = { [1] = {0,0,0} }
    _G.electrics_values_angelo234 = { [1] = 0, lights_state = 0 }
    _G.angular_speed_angelo234 = 0
    _G.input_throttle_angelo234 = 0
    _G.input_brake_angelo234 = 0
    _G.input_clutch_angelo234 = 0
    _G.input_parkingbrake_angelo234 = 0
    _G.gearbox_mode_angelo234 = 'dummy'

    local extension = require('scripts/driver_assistance_angelo234/extension')
    extension.onExtensionLoaded()
    extension.toggleAutoHeadlightSystem()

    extension.onUpdate(0.3)
    extension.onUpdate(0.3)

    assert.is_true(poll_called > 0)
    assert.equal(sample_front_data[2], passed_vehs)
  end)
end)
