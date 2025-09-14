local busted = require('busted')

-- Stub dependencies required by the extension
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

-- Stub the BeamNG API used by the extension
_G.be = {
  getPlayerVehicle = function() return nil end
}

-- Stub smoothing function used at module load
_G.newExponentialSmoothing = function()
  return { get = function(self, v) return v end }
end

local extension = require('scripts/driver_assistance_angelo234/extension')

describe('extension', function()
  it('does not call missing init functions on first update', function()
    assert.has_no.errors(function()
      extension.onUpdate(0.1)
    end)
  end)
end)
