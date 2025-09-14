local M = {
  enabled = false,
  sensors = {
    front_sensor = false,
    rear_sensor = false,
    lidar = false
  }
}

function M.setEnabled(value)
  M.enabled = value and true or false
end

function M.toggle()
  M.enabled = not M.enabled
  return M.enabled
end

function M.setSensorEnabled(sensor, value)
  if M.sensors[sensor] ~= nil then
    M.sensors[sensor] = value and true or false
  end
end

function M.toggleSensor(sensor)
  if M.sensors[sensor] ~= nil then
    M.sensors[sensor] = not M.sensors[sensor]
    return M.sensors[sensor]
  end
end

function M.isEnabled()
  return M.enabled
end

function M.isSensorEnabled(sensor)
  return M.sensors[sensor]
end

function M.log(level, tag, msg)
  if M.enabled and log then
    local sensor_flag = M.sensors[tag]
    if sensor_flag == nil or sensor_flag then
      log(level, tag, msg)
    end
  end
end

return M
