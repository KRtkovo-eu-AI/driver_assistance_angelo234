local busted = require('busted')

-- minimal vector implementation for tests
local function vec(x, y, z)
  local v = {x = x or 0, y = y or 0, z = z or 0}
  local mt = {}
  function mt.__add(a, b) return vec(a.x + b.x, a.y + b.y, a.z + b.z) end
  function mt.__sub(a, b) return vec(a.x - b.x, a.y - b.y, a.z - b.z) end
  function mt.__mul(a, b)
    if type(a) == 'number' then return vec(a * b.x, a * b.y, a * b.z)
    elseif type(b) == 'number' then return vec(a.x * b, a.y * b, a.z * b) end
  end
  mt.__index = {
    length = function(self) return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z) end,
    normalized = function(self)
      local l = self:length()
      if l == 0 then return vec(0,0,0) end
      return vec(self.x / l, self.y / l, self.z / l)
    end,
    dot = function(self, other) return self.x * other.x + self.y * other.y + self.z * other.z end
  }
  return setmetatable(v, mt)
end

-- stubbed utilities used by sensor system
local extra_utils = {}
function extra_utils.getVehicleProperties(veh) return veh.props end
function extra_utils.getCircularDistance(a, b) return (b.center_pos - a.center_pos):length() end
function extra_utils.getWaypointStartEndAdvanced() return { start_wp_pos = vec(0,0,0), end_wp_pos = vec(1,0,0), wp_radius = 100 } end
function extra_utils.checkIfOtherCarOnSameRoad() return true end

package.loaded['scripts/driver_assistance_angelo234/extraUtils'] = extra_utils

-- stubbed BeamNG API
_G.castRayStatic = function() return 9999 end
_G.be = {
  vehicles = {},
  getObjectCount = function(self) return #self.vehicles end,
  getObject = function(self, i) return self.vehicles[i + 1] end
}

local sensor_system = require('scripts/driver_assistance_angelo234/sensorSystem')

describe('sensor system', function()
  it('detects vehicles ahead regardless of orientation', function()
    local my_veh = {
      props = {
        id = 1,
        center_pos = vec(0,0,0),
        front_pos = vec(-1,0,0), -- swapped to mimic incorrect data
        rear_pos = vec(1,0,0),
        velocity = vec(0,0,0),
        dir = vec(1,0,0),
        speed = 0
      },
      getJBeamFilename = function() return 'car' end,
      getID = function() return 1 end
    }

    local other = {
      props = {
        id = 2,
        center_pos = vec(50,0,0),
        front_pos = vec(51,0,0),
        rear_pos = vec(49,0,0),
        velocity = vec(0,0,0),
        speed = 0
      },
      getJBeamFilename = function() return 'car' end,
      getID = function() return 2 end
    }

    be.vehicles = { my_veh, other }

    local aeb_params = {
      sensors_polled_per_iteration = 0,
      num_of_sensors = 1,
      vehicle_search_radius = 100,
      sensor_offset_forward = 0,
      parking_sensor_rel_height = 0
    }
    local data = sensor_system.pollFrontSensors(0.1, my_veh.props, {}, aeb_params)
    assert.is_true(#data[2] >= 1)
  end)
end)

