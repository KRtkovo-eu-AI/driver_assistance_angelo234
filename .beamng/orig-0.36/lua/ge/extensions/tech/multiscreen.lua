-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.views = {}

local function addView(view)
  table.insert(M.views, view)
end

local function addVehicleView(name, positionX, positionY, positionZ, rotationX, rotationY, rotationZ, resolutionX, resolutionY, renderDetail, fov, nearClip, farClip, enableShadows, windowX, windowY, borderless)
  local view = {
    name = name,
    position = vec3(positionX or 0, positionY or 0, positionZ or 0),
    rotation = quatFromEuler(rotationX or 0, rotationY or 0, rotationZ or 0),
    resolutionX = resolutionX or 1080, resolutionY = resolutionY or 720,
    renderDetail = renderDetail or 0.3,
    fov = math.rad(fov or 80),
    nearClip = nearClip or 1, farClip = farClip or 1000,
    enableShadows = enableShadows or 0,
    windowX = windowX or 0,
    windowY = windowY or 0,
    borderless = borderless or false
  }
  addView(view)
end

local function removeAllViews()
  if destroyCameraToWindow then
    for _, view in ipairs(M.views) do
      destroyCameraToWindow(view.name)
    end
  end
  M.views = {}
end

local resolution = Point2I(0, 0)
local clipPlane = Point2F(0, 0)
local rotation = QuatF(0, 0, 0, 1)
local function onPreRender()
  if not getOrCreateCameraToWindow or not requestCameraToWindowRender then return end
  if tableIsEmpty(M.views) then return end

  local veh = getPlayerVehicle(0)
  local p0 = Point2F(0, 0)
  if veh then
    local vehicleRotation = quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp())
    local vehiclePosition = veh:getPosition()
    for _, view in ipairs(M.views) do
      local pos = vehiclePosition + (vehicleRotation * view.position)
      if view.offsetNode then
        pos:setAdd(push3(veh:getNodePositionXYZ(view.offsetNode)))
      end
      local rot = view.rotation * vehicleRotation
      rotation.x, rotation.y, rotation.z, rotation.w = rot.x, rot.y, rot.z, rot.w
      local window = getOrCreateCameraToWindow(view.name, view.windowX, view.windowY, view.borderless)
      resolution.x, resolution.y = view.resolutionX, view.resolutionY
      clipPlane.x, clipPlane.y = view.nearClip, view.farClip
      requestCameraToWindowRender(window, pos, rotation, resolution, view.renderDetail, view.fov, clipPlane, view.enableShadows, p0)
    end
  end
end

local function onUnload()
  M.removeAllViews()
end

M.onDeserialize = true
M.onPreRender = onPreRender
M.onUnload = onUnload
M.addView = addView
M.addVehicleView = addVehicleView
M.removeAllViews = removeAllViews

return M
