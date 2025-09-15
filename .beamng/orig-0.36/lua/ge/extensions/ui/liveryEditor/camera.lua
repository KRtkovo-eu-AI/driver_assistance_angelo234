-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals

local ORTHOGRAPHIC_VIEWS = {
  front = vec3(180, 0, 0),
  back = vec3(0, 0, 0),
  left = vec3(-90, 0, 0),
  right = vec3(90, 0, 0),
  topfront = vec3(180, -90, 0),
  topleft = vec3(-90, -90, 0),
  topright = vec3(90, -90, 0),
  topback = vec3(0, -90, 0),

  default = vec3(145, -5, 0)
}

local orientationCoordinates = {
  -- z inverted, y
  left = {
    x = {
      index = 2,
      inverted = false
    },
    y = {
      index = 3,
      inverted = false
    }
  },
  -- z, y face
  right = {
    x = {
      index = 2,
      inverted = true
    },
    y = {
      index = 3,
      inverted = false
    }
  },
  -- x inverted, y face
  front = {
    x = {
      index = 1,
      inverted = false
    },
    y = {
      index = 3,
      inverted = false
    }
  },
  -- x, y face
  back = {
    x = {
      index = 1,
      inverted = true
    },
    y = {
      index = 3,
      inverted = false
    }
  },
  -- x, z face
  topright = {
    x = {
      index = 2,
      inverted = true
    },
    y = {
      index = 1,
      inverted = false
    }
  },
  -- x, z face
  topleft = {
    x = {
      index = 2,
      inverted = false
    },
    y = {
      index = 1,
      inverted = true
    }
  },
  -- x, z face
  topfront = {
    x = {
      index = 1,
      inverted = false
    },
    y = {
      index = 2,
      inverted = false
    }
  },
  -- x, z face
  topback = {
    x = {
      index = 1,
      inverted = true
    },
    y = {
      index = 2,
      inverted = true
    }
  }
}

local orthographicView = nil

local resetCursorPosition = function()
  local cursorPosition = api.getCursorPosition()
  cursorPosition.x = 0.5
  cursorPosition.y = 0.5
  api.setCursorPosition(cursorPosition)
end

local function setCameraRotationInJob(job)
  commands.setGameCamera()
  core_camera.setByName(0, "orbit", false)
  job.sleep(0.00001) -- sleep for one frame so the orbit cam can update correctly
  core_camera.setDefaultRotation(be:getPlayerVehicleID(0), job.args[1])
  core_camera.resetCamera(0)
end

local function setToOrbitCameraJob(job)
  commands.setGameCamera()
  core_camera.setByName(0, "orbit", false)

  local resetCam = job.args and job.args[1] or false
  if resetCam then
    core_camera.resetCamera(0)
  end
  job.sleep(0.00001) -- sleep for one frame so the orbit cam can update correctly
end

local function setCameraPosRotInJob(job)
  local layer = job.args[1]
  local pos = layer.camPosition
  local camDir = layer.camDirection

  local vehId = be:getPlayerVehicleID(0)
  local veh = getPlayerVehicle(0)

  core_camera.setByName(0, 'free')
  local idealDistance = veh:getViewportFillingCameraDistance() * 1.05
  core_camera.setDistance(0, idealDistance)
  core_camera.setFOV(0, 45)
  core_camera.setOffset(0, vec3(0, 0, 2 / idealDistance))

  core_camera.setPosition(0, layer.camPosition + veh:getPosition())
  core_camera.setRotation(0, quatFromDir(layer.camDirection))
end

local notifyUiListeners = function(data)
  guihooks.trigger("LiveryEditor_OnCameraChanged", data)
end

M.setCameraByLayer = function(layer)
  core_jobsystem.create(setCameraPosRotInJob, nil, layer)
end

M.setOrthographicView = function(view)
  -- orthographicView = view
  core_jobsystem.create(setCameraRotationInJob, nil, M.orthographicViews[view])
  -- core_jobsystem.create(setCameraRotationInJob, nil, ORTHOGRAPHIC_VIEWS[orthographicView])
  -- resetCursorPosition()
  -- notifyUiListeners(view)
end

M.switchToOrbit = function(resetCam)
  core_jobsystem.create(setToOrbitCameraJob, nil, resetCam)
end

M.setOrthographicViewByPosition = function(pos)
  local view = M.getOrthographicViewByPosition(pos)
  M.setOrthographicView(view)
end

M.setOrthographicViewLayer = function(layer)
  local view = M.getOrthographicViewByPosition(layer.camPosition)
  M.setOrthographicView(view)
end

M.getViewPointByLayer = function(layer)
  return M.getOrthographicViewByPosition(layer.camPosition)
end

M.switchOrthographicViewByDirection = function(x, y)
  if not orthographicView then
    M.setOrthographicView("right")
  else
    local switchOrder = {"right", "front", "left", "back"}
    local startsWithTop = string.sub(orthographicView, 1, 3) == "top"

    if x and (x == 1 or x == -1) and not startsWithTop then
      local index

      for k, v in ipairs(switchOrder) do
        if v == orthographicView then
          index = k
        end
      end

      if index == 1 and x == -1 then
        M.setOrthographicView(switchOrder[#switchOrder])
      else
        if index == #switchOrder and x == 1 then
          M.setOrthographicView(switchOrder[1])
        else
          M.setOrthographicView(switchOrder[index + x])
        end
      end
    end

    if y and (y == 1 or y == -1) then
      if startsWithTop and y == -1 then
        local view = string.sub(orthographicView, 4)
        M.setOrthographicView(view)
      elseif not startsWithTop and y == 1 then
        M.setOrthographicView("top" .. orthographicView)
      end
    end
  end
end

M.getOrthographicViewByPosition = function(pos)
  local view = pos.z > 1 and "top" or ""

  if pos.x < 0 then
    return view .. "right"
  elseif pos.y < 0 then
    return view .. "front"
  elseif pos.x > 1 then
    return view .. "left"
  elseif pos.y > 1 then
    return view .. "back"
  end
end

M.getCoordinatesByView = function(view)
  return orientationCoordinates[view]
end

M.getOrthographicView = function()
  return orthographicView
end

M.getCoordinates = function()
  return orientationCoordinates[orthographicView]
end

M.orthographicViews = ORTHOGRAPHIC_VIEWS

return M
