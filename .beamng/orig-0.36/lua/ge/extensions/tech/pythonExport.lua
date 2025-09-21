-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local logTag = 'pythonExport'

local LineLengthDefault = 80

local LuaToPython = {}
local PythonDefaults = {}
local SensorTypeMapping = {
  camera='Camera',
  LiDAR='Lidar',
  ultrasonic='Ultrasonic',
  RADAR='Radar',
  IMU='AdvancedIMU',
  GPS='GPS',
  idealRADAR='IdealRadar',
  roads='RoadsSensor',
  powertrain='PowertrainSensor',
  mesh='Mesh'
}

local sbuffer = require('string.buffer')

local function coeffs2Python(c, veh)
  local offset = veh:getInitialNodePosition(veh:getRefNodeId())
  return vec3(-c.y + offset.x, -c.x + offset.y, c.z + offset.z)
end

local function dir2Python(c, veh)
  return vec3(-c.y, -c.x, c.z)
end

local function formatValueAsPython(value)
  if type(value) == 'boolean' then
    return value and 'True' or 'False'
  end
  if type(value) == 'cdata' and ffi.offsetof(value, 'z') ~= nil then  -- vec3
    if ffi.offsetof(value, 'w') ~= nil then -- quat
      return string.format('(%.4g, %.4g, %.4g, %.4g)', value.x, value.y, value.z, value.w)
    end
    return string.format('(%.4g, %.4g, %.4g)', value.x, value.y, value.z)
  end
  if type(value) == 'number' then
    if math.floor(value) ~= value then
      return string.format("%.4g", value)
    end
    return tostring(value)
  end
  if type(value) == 'table' then
    local first = true
    local buf = sbuffer.new()
    buf:put('(')
    for _, v in ipairs(value) do
      if not first then
        buf:put(', ')
      end
      first = false
      buf:put(formatValueAsPython(v))
    end
    buf:put(')')
    return buf:get()
  end
  return tostring(value)
end

local function getScenarioConfig(buffer, vehicle, vehData)
  local line = string.format("scenario = Scenario('%s', 'we_export')\n", getCurrentLevelIdentifier())
  buffer:put(line)

  local vehName = vehicle:getName()
  local vehModel = vehicle:getJBeamFilename()
  line = string.format("%s = Vehicle('%s', '%s')\n", vehName, vehName, vehModel)
  buffer:put(line)

  local vehPos = vehicle:getPosition()
  line = string.format("scenario.add_vehicle(%s, pos=%s", vehName, formatValueAsPython(vehPos))
  buffer:put(line)
  if vehData ~= nil and vehData.rot then
    buffer:put(", rot_quat=")
    buffer:put(formatValueAsPython(vehData.rot))
  end
  buffer:put(")\n")

  buffer:put("scenario.make(beamng)\n")
  buffer:put("beamng.scenario.load(scenario)\n")
  buffer:put("beamng.scenario.start()\n")
end

local function getSensorConfig(buffer, vehicle, sensor, lineLength)
  if lineLength == nil then
    lineLength = LineLengthDefault
  end

  local sensorType = SensorTypeMapping[sensor.type]
  local sensorName = sensor.name

  local vid = vehicle == nil and 'None' or vehicle:getName()
  local nonDefaultData = {
    {vehicle=vid}
  }
  local pythonData = LuaToPython[sensorType](sensor, vehicle)
  for _, kv in pairs(PythonDefaults[sensorType]) do
    local property, defaultValue = next(kv, nil)
    local value = pythonData[property]
    if value ~= nil and value ~= defaultValue then
      table.insert(nonDefaultData, {[property] = value})
    end
  end

  local lineStart = #buffer
  buffer:put(sensorName:gsub('[%[%] ]+', '_'):rstripchars('_'):lower())
  buffer:put(" = ")
  buffer:put(sensorType)
  buffer:put("('")
  buffer:put(sensorName)
  buffer:put("', beamng")
  for _, kv in pairs(nonDefaultData) do
    local length = #buffer - lineStart
    if length >= lineLength then
      buffer:put(",\n")
      lineStart = #buffer
      buffer:put("  ")
    else
      buffer:put(", ")
    end
    local property, value = next(kv, nil)
    buffer:put(property)
    buffer:put("=")
    buffer:put(formatValueAsPython(value))
  end
  buffer:put(")\n")
  return buffer
end

local bufferScenario = nil
local bufferSensors = nil
local currentPtr = nil

local function getFullConfigFinish()
  local code = sbuffer.new()
  code:put(bufferScenario)
  code:put(bufferSensors)
  bufferScenario = nil
  bufferSensors = nil
  ffi.copy(currentPtr, code, #code)
  currentPtr = nil
end

local function getFullConfigVehicleCallback(vid, data)
  bufferScenario = sbuffer.new()
  bufferScenario:put("# Scenario configuration\n")
  local unpacked = nil
  if data then
    unpacked = lpack.decode(data)
  end
  getScenarioConfig(bufferScenario, getObjectByID(vid), unpacked)
  getFullConfigFinish()
end

local function getFullConfig(vehicle, sensors, codePtr)
  bufferSensors = sbuffer.new()
  bufferSensors:put("\n# Sensor configuration\n")
  for i = 1, #sensors do
    getSensorConfig(bufferSensors, vehicle, sensors[i])
  end

  if not vehicle then
    vehicle = be:getPlayerVehicle(0)
  end

  currentPtr = codePtr
  local cmd = [[
    local rot = quat(obj:getRotation())
    local data = { rot = rot }
    obj:queueGameEngineLua(
      string.format('extensions.tech_pythonExport.getFullConfigVehicleCallback(%d, %q)', obj:getID(), lpack.encode(data))
    )
  ]]
  vehicle:queueLuaCommand(cmd)
end

local function luaToPythonPosDir(data, luaSensor, vehicle)
  data.is_static = vehicle == nil
  if data.is_static then
    data.pos = luaSensor.pos
    data.dir = luaSensor.dir
  else
    data.pos = coeffs2Python(luaSensor.pos, vehicle)
    data.dir = dir2Python(luaSensor.dir, vehicle)
  end
end

LuaToPython.Camera = function(luaSensor, vehicle)
  local data = {}
  luaToPythonPosDir(data, luaSensor, vehicle)

  data.name = luaSensor.name
  data.requested_update_time = luaSensor.updateTime
  data.update_priority = luaSensor.updatePriority
  data.up = luaSensor.up
  data.resolution = luaSensor.size
  data.field_of_view_y = luaSensor.fovY
  data.near_far_planes = luaSensor.nearFarPlanes
  data.is_using_shared_memory = false
  data.is_streaming = false
  data.is_render_colours = luaSensor.isRenderColours
  data.is_render_annotations = luaSensor.isRenderAnnotations
  data.is_render_instance = luaSensor.isRenderInstance
  data.is_render_depth = luaSensor.isRenderDepth
  data.is_depth_inverted = false
  data.is_visualised = luaSensor.isVisualised
  data.is_snapping_desired = luaSensor.isSnappingDesired
  data.is_force_inside_triangle = luaSensor.isSnappingDesired

  return data
end

PythonDefaults.Camera = {
  {requested_update_time=0.1},
  {update_priority=0.0},
  {pos=vec3(0, 0, 3)},
  {dir=vec3(0, -1, 0)},
  {up=vec3(0, 0, 1)},
  {resolution={512, 512}},
  {field_of_view_y=70},
  {near_far_planes={0.05, 100.0}},
  {is_using_shared_memory=false},
  {is_render_colours=true},
  {is_render_annotations=true},
  {is_render_instance=false},
  {is_render_depth=true},
  {is_depth_inverted=false},
  {is_visualised=false},
  {is_streaming=false},
  {is_static=false},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {postprocess_depth=false},
  {is_dir_world_space=false},
  {integer_depth=true},
}

LuaToPython.Lidar = function(luaSensor, vehicle)
  local data = {}
  luaToPythonPosDir(data, luaSensor, vehicle)

  data.requested_update_time=luaSensor.updateTime
  data.update_priority=luaSensor.updatePriority
  data.up=luaSensor.up
  data.vertical_resolution=luaSensor.verticalResolution
  data.vertical_angle=luaSensor.verticalAngle
  data.frequency=luaSensor.frequency
  data.horizontal_angle=luaSensor.horizontalAngle
  data.max_distance=luaSensor.maxDistance
  data.is_360_mode=luaSensor.is360
  data.is_rotate_mode=luaSensor.isRotate
  data.is_using_shared_memory=false
  data.is_streaming=false
  data.is_annotated=luaSensor.isAnnotated
  data.is_visualised=luaSensor.isVisualised
  data.is_snapping_desired=luaSensor.isSnappingDesired
  data.is_force_inside_triangle=luaSensor.isSnappingDesired

  return data
end

PythonDefaults.Lidar = {
  {requested_update_time=0.1},
  {update_priority=0.0},
  {pos=vec3(0, 0, 1.7)},
  {dir=vec3(0, -1, 0)},
  {up=vec3(0, 0, 1)},
  {vertical_resolution=64},
  {vertical_angle=26.9},
  {frequency=20},
  {horizontal_angle=360},
  {max_distance=120},
  {density=100},
  {is_rotate_mode=false},
  {is_360_mode=true},
  {is_using_shared_memory=true},
  {is_visualised=true},
  {is_streaming=false},
  {is_annotated=false},
  {is_static=false},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {is_dir_world_space=false},
}

LuaToPython.Ultrasonic = function(luaSensor, vehicle)
  local data = {}
  luaToPythonPosDir(data, luaSensor, vehicle)

  data.requested_update_time=luaSensor.updateTime
  data.update_priority=luaSensor.updatePriority
  data.up=luaSensor.up
  data.resolution=luaSensor.size
  data.field_of_view_y=luaSensor.fovY
  data.near_far_planes=luaSensor.nearFarPlanes
  data.range_roundness=luaSensor.rangeRoundness
  data.range_cutoff_sensitivity=luaSensor.rangeCutoffSensitivity
  data.range_shape=luaSensor.rangeShape
  data.range_focus=luaSensor.rangeFocus
  data.range_min_cutoff=luaSensor.rangeMinCutoff
  data.range_direct_max_cutoff=luaSensor.rangeDirectMaxCutoff
  data.sensitivity=luaSensor.sensitivity
  data.fixed_window_size=luaSensor.fixedWindowSize
  data.is_streaming=false
  data.is_visualised=luaSensor.isVisualised
  data.is_snapping_desired=luaSensor.isSnappingDesired
  data.is_force_inside_triangle=luaSensor.isSnappingDesired

  return data
end

PythonDefaults.Ultrasonic = {
  {requested_update_time=0.1},
  {update_priority=0.0},
  {pos=vec3(0, 0, 1.7)},
  {dir=vec3(0, -1, 0)},
  {up=vec3(0, 0, 1)},
  {resolution={200, 200}},
  {field_of_view_y=5.7},
  {near_far_planes={0.1, 5.1}},
  {range_roundness=-1.15},
  {range_cutoff_sensitivity=0.0},
  {range_shape=0.3},
  {range_focus=0.376},
  {range_min_cutoff=0.1},
  {range_direct_max_cutoff=5.0},
  {sensitivity=3.0},
  {fixed_window_size=10},
  {is_visualised=true},
  {is_streaming=false},
  {is_static=false},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {is_dir_world_space=false},
}

LuaToPython.Radar = function(luaSensor, vehicle)
  local data = {}
  luaToPythonPosDir(data, luaSensor, vehicle)

  data.requested_update_time=luaSensor.updateTime
  data.update_priority=luaSensor.updatePriority
  data.up=luaSensor.up
  data.range_bins=luaSensor.rangeBins
  data.azimuth_bins=luaSensor.azimuthBins
  data.vel_bins=luaSensor.velBins
  data.range_min=luaSensor.rangeMin
  data.range_max=luaSensor.rangeMax
  data.vel_min=luaSensor.velMin
  data.vel_max=luaSensor.velMax
  data.half_angle_deg=luaSensor.halfAngleDeg
  data.resolution=luaSensor.size
  data.field_of_view_y=luaSensor.fovY
  data.near_far_planes=luaSensor.nearFarPlanes
  data.range_roundness=luaSensor.rangeRoundness
  data.range_cutoff_sensitivity=luaSensor.rangeCutoffSensitivity
  data.range_shape=luaSensor.rangeShape
  data.range_focus=luaSensor.rangeFocus
  data.range_min_cutoff=luaSensor.rangeMinCutoff
  data.range_direct_max_cutoff=luaSensor.rangeDirectMaxCutoff
  data.is_streaming=false
  data.is_visualised=luaSensor.isVisualised
  data.is_snapping_desired=luaSensor.isSnappingDesired
  data.is_force_inside_triangle=luaSensor.isSnappingDesired

  return data
end

PythonDefaults.Radar = {
  {requested_update_time=0.1},
  {update_priority=0.0},
  {pos=vec3(0, 0, 1.7)},
  {dir=vec3(0, -1, 0)},
  {up=vec3(0, 0, 1)},
  {range_bins=200},
  {azimuth_bins=200},
  {vel_bins=200},
  {range_min=0.1},
  {range_max=100.0},
  {vel_min=-50.0},
  {vel_max=50.0},
  {half_angle_deg=30.0},
  {resolution={200, 200}},
  {field_of_view_y=70},
  {near_far_planes={0.1, 150.0}},
  {range_roundness=-2.0},
  {range_cutoff_sensitivity=0.0},
  {range_shape=0.23},
  {range_focus=0.12},
  {range_min_cutoff=0.5},
  {range_direct_max_cutoff=150.0},
  {is_visualised=true},
  {is_streaming=false},
  {is_static=false},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {is_dir_world_space=false},
}

LuaToPython.AdvancedIMU = function(luaSensor, vehicle)
  local data = {}
  luaToPythonPosDir(data, luaSensor, vehicle)

  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime
  data.up=luaSensor.up
  data.accel_window_width=luaSensor.accelWindowWidth
  data.gyro_window_width=luaSensor.gyroWindowWidth
  data.accel_frequency_cutoff=nil
  data.gyro_frequency_cutoff=nil
  data.is_send_immediately=false
  data.is_using_gravity=luaSensor.isUsingGravity
  data.is_allow_wheel_nodes=luaSensor.isAllowWheelNodes
  data.is_visualised=luaSensor.isVisualised
  data.is_snapping_desired=luaSensor.isSnappingDesired
  data.is_force_inside_triangle=luaSensor.isSnappingDesired

  return data
end

PythonDefaults.AdvancedIMU = {
  {gfx_update_time=0.0},
  {physics_update_time=0.01},
  {pos=vec3(0, 0, 1.7)},
  {dir=vec3(0, -1, 0)},
  {up=vec3(-0, 0, 1)},
  {accel_window_width=nil},
  {gyro_window_width=nil},
  {accel_frequency_cutoff=nil},
  {gyro_frequency_cutoff=nil},
  {is_send_immediately=false},
  {is_using_gravity=false},
  {is_allow_wheel_nodes=true},
  {is_visualised=true},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {is_dir_world_space=false},
}

LuaToPython.GPS = function(luaSensor, vehicle)
  local data = {}

  data.pos = coeffs2Python(luaSensor.pos, vehicle)

  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime
  data.ref_lon=luaSensor.refLon
  data.ref_lat=luaSensor.refLat
  data.is_send_immediately=false
  data.is_visualised=luaSensor.isVisualised
  data.is_snapping_desired=luaSensor.isSnappingDesired
  data.is_force_inside_triangle=luaSensor.isSnappingDesired

  return data
end

PythonDefaults.GPS = {
  {gfx_update_time=0.0},
  {physics_update_time=0.01},
  {pos=vec3(0, 0, 1.7)},
  {ref_lon=0.0},
  {ref_lat=0.0},
  {is_send_immediately=false},
  {is_visualised=true},
  {is_snapping_desired=false},
  {is_force_inside_triangle=false},
  {is_dir_world_space=false},
}

LuaToPython.IdealRadar = function(luaSensor, vehicle)
  local data = {}

  data.is_send_immediately=false
  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime

  return data
end

PythonDefaults.IdealRadar = {
  {gfx_update_time=0.0},
  {physics_update_time=0.01},
  {is_send_immediately=false},
}

LuaToPython.RoadsSensor = function(luaSensor, vehicle)
  local data = {}

  data.is_send_immediately=false
  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime

  return data
end

PythonDefaults.RoadsSensor = {
  {gfx_update_time=0.0},
  {physics_update_time=0.01},
  {is_send_immediately=false},
  {is_visualised=false},
}

LuaToPython.PowertrainSensor = function(luaSensor, vehicle)
  local data = {}

  data.is_send_immediately=false
  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime

  return data
end

PythonDefaults.PowertrainSensor = {
  {gfx_update_time=0.0},
  {physics_update_time=0.01},
  {is_send_immediately=false},
}

LuaToPython.Mesh = function(luaSensor, vehicle)
  local data = {}

  data.gfx_update_time=luaSensor.GFXUpdateTime
  data.physics_update_time=luaSensor.physicsUpdateTime

  return data
end

PythonDefaults.Mesh = {
  {gfx_update_time=0.0},
  {physics_update_time=0.015},
  {groups_list={}},
  {is_track_beams=true},
}

M.getFullConfigVehicleCallback = getFullConfigVehicleCallback
M.getFullConfig = getFullConfig
M.getScenarioConfig = getScenarioConfig
M.getSensorConfig = getSensorConfig

return M
