-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"ui_imgui"}
local im = ui_imgui
local mrad = math.rad

local offsetData = {}

local mouseInteraction = false
local mouseData = {}
local screenRatio = 0.01
local MAXDEGREE = 20
local yawBeforeFocus = nil
local pitchBeforeFocus

local windowOpen = im.BoolPtr(false)
local initialWindowSize = im.ImVec2(300, 100)
local imguiSliderData = {}
local imguiMirrordata

local function settingsSave()
  jsonWriteFile("/settings/mirrorOffsets.json", offsetData, true)
end

local function settingsLoad()
  local s = jsonReadFile("/settings/mirrorOffsets.json")
  if s then
    offsetData = s
  end
end

local function onSerialize()
  local data = {}
  data.windowOpen = windowOpen[0]
  return data
end

local function onDeserialized(data)
  if data.windowOpen ~= nil then windowOpen[0] = data.windowOpen end
  M.onSettingsChanged()
end

local function onSettingsChanged()
  local vm = GFXDevice.getVideoMode()
  screenRatio = MAXDEGREE / ((vm.width+vm.height)*0.5)
end

local function onVehicleDestroyed(vid)
end

local function onVehicleSpawned(vid, veh)
  local vdata = extensions.core_vehicle_manager.getVehicleData(vid)
  local offset = M.getAnglesOffset(vid, veh)
  if #offset then
    for k,v in pairs(offset) do
      M.setAngleOffset(k,v.angleOffset.x,v.angleOffset.z, veh)
    end
  end
end


local function getAnglesOffset(vid, v)
  local veh = v or getPlayerVehicle(0)
  if vid == nil then vid= veh:getId() end
  local vdata = extensions.core_vehicle_manager.getVehicleData(vid)
  if not vdata or not vdata.vdata or not vdata.vdata.mirrors then return {} end

  local configName = veh.partConfig or "default"
  if configName:startswith("{") then --custom
    configName = "custom" -- +"_"+stringHash(configName)
  end

  local mytable = {}
  for k,v in pairs(vdata.vdata.mirrors ) do
    if v.mesh then
      mytable[v.mesh] = {}
      mytable[v.mesh].name = v.mesh
      mytable[v.mesh].id = v.id
      mytable[v.mesh].angleOffset = {x=0,z=0}
      if not v.clampX then
        mytable[v.mesh].clampX = {-MAXDEGREE,MAXDEGREE}
      else
        mytable[v.mesh].clampX = v.clampX
      end
      if not v.clampZ then
        mytable[v.mesh].clampZ = {-MAXDEGREE,MAXDEGREE}
      else
        mytable[v.mesh].clampZ = v.clampZ
      end
      if offsetData[veh.JBeam] then
        if offsetData[veh.JBeam][configName] then
          if offsetData[veh.JBeam][configName][ v.mesh ] then
            mytable[v.mesh].angleOffset = offsetData[veh.JBeam][configName][ v.mesh ]
          end
        end
      end
      mytable[v.mesh].position = v.UiColumn
      mytable[v.mesh].icon = v.icon
      mytable[v.mesh].row = v.UiRow
      mytable[v.mesh].label = v.label
    end
  end

  return mytable
end

local function setAngleOffset(mirrorName, x, z, v, save)
  local veh = v or getPlayerVehicle(0)
  local vid = veh:getId()
  local vdata = extensions.core_vehicle_manager.getVehicleData(vid)
  if not vdata or not vdata.vdata or not vdata.vdata.mirrors then log("E","setO","no veh data"); return end

  local mid = -1
  for i in pairs(vdata.vdata.mirrors) do
    if vdata.vdata.mirrors[i].mesh == mirrorName then
      mid = vdata.vdata.mirrors[i].id
      break
    end
  end
  local mirror = veh:getMirror(mid)
  if not mirror then log("E","setO","getMirror failed!"); return end

  local q = quatFromEuler(mrad(x),mrad(0),mrad(z))
  mirror.offsetNormal = vec3(0,1,0):rotated(q)

  local configName = veh.partConfig or "default"
  if configName:startswith("{") then --custom
    configName = "custom" -- +"_"+stringHash(configName)
  end

  if not offsetData[veh.JBeam] then
    offsetData[veh.JBeam] = {}
  end
  if not offsetData[veh.JBeam][configName] then
    offsetData[veh.JBeam][configName] = {}
  end
  offsetData[veh.JBeam][configName][mirrorName] = {x=x,z=z}
  if save then
    settingsSave()
  end

end

local function _mouseUpdate(save)
  local mousePos = vec3(im.GetMousePos().x, im.GetMousePos().y, 1)
  local offset = (mouseData.startPos - mousePos) * screenRatio
  mouseData.newAngle = mouseData.originalOffset - offset
  mouseData.newAngle.x = clamp(mouseData.newAngle.x, -MAXDEGREE, MAXDEGREE)
  mouseData.newAngle.y = clamp(mouseData.newAngle.y, -MAXDEGREE, MAXDEGREE)
  setAngleOffset(mouseData.name , mouseData.newAngle.y, -mouseData.newAngle.x, nil, save)
end

local function onUpdate(dtReal, dtSim, dtRaw)

  if mouseInteraction then
    _mouseUpdate()
  end

  if windowOpen[0] ~= true then return end
  im.SetNextWindowSize(initialWindowSize, im.Cond_FirstUseEver)
  im.SetNextWindowPos(initialWindowSize, im.Cond_FirstUseEver)
  if( im.Begin("core_vehicle_mirror Debugger", windowOpen) ) then
    if im.Button("get") then
      imguiMirrordata = getAnglesOffset()
      imguiSliderData = {}
      for k,v in pairs(imguiMirrordata) do
        imguiSliderData[k] = {im.FloatPtr(v.angleOffset.x or 0), im.FloatPtr(v.angleOffset.z or 0)}
      end
    end

    if imguiMirrordata then
      if im.Button("save") then
        for k,v in pairs(imguiMirrordata) do
          setAngleOffset(k,imguiSliderData[k][1][0], imguiSliderData[k][2][0], nil, true)
        end
      end
      local focusedAny = false
      for k,v in pairs(imguiMirrordata) do
        im.TextUnformatted(dumps(k))
        local hover = im.IsItemHovered()
        local mod = im.SliderFloat("X##"..k, imguiSliderData[k][1],v.clampX[1],v.clampX[2], "%.1f")
        hover = hover or im.IsItemHovered()
        mod = im.SliderFloat("Z##"..k, imguiSliderData[k][2],v.clampZ[1],v.clampZ[2], "%.1f") or mod
        hover = hover or im.IsItemHovered()
        if mod then
          setAngleOffset(k,imguiSliderData[k][1][0], imguiSliderData[k][2][0])
        end
        if hover then
          M.focusOnMirror(k)
          focusedAny = true
        end
      end
      if not focusedAny and yawBeforeFocus then
        M.focusOnMirror()
      end
    end

  end
  im.End()
end

local function setDebug(newValue)
  if newValue then
    windowOpen[0] = true
  else
    windowOpen[0] = false
  end
end

local function vehicleEvent( evtType, vid, mirror_name )
  if evtType == "onDown" then
    mouseInteraction = true
    local mousePos = im.GetMousePos()
    mouseData.startPos = vec3(mousePos.x,mousePos.y,1)
    mouseData.name = mirror_name
    mouseData.vid = vid
    local mdata = getAnglesOffset(vid, getObjectByID(vid))
    if mdata[mirror_name] then
      mouseData.clampX = mdata[mirror_name].clampX
      mouseData.clampZ = mdata[mirror_name].clampZ
      if mdata[mirror_name].angleOffset then
        mouseData.originalOffset = vec3(-mdata[mirror_name].angleOffset.z, mdata[mirror_name].angleOffset.x, 0 ) --warn in screen format
      else
        mouseData.originalOffset = vec3(0,0,0)
      end
    end
  elseif evtType == "onUp" then
    mouseInteraction = false
    _mouseUpdate(true)
    mouseData = {}
  else
    log("E","vehEvt","event type unknown")
  end

end

local function focusOnMirror(mirror_name)
  local veh = v or getPlayerVehicle(0)
  local vid = veh:getId()
  local vdata = extensions.core_vehicle_manager.getVehicleData(vid)
  local camData = core_camera.getCameraDataById(vid)
  if not camData or not camData.driver then
    log("E","focus", "invalid camera data")
    return
  end

  if core_camera.getActiveCamName() ~= "driver" then
    core_camera.setByName(0, "driver", false)
  end
  if not mirror_name then
    camData.driver.relativeYaw = yawBeforeFocus or 0
    camData.driver.relativePitch = pitchBeforeFocus or 0
    yawBeforeFocus = nil
  end
  if not vdata or not vdata.vdata or not vdata.vdata.mirrors then log("E","setO","no veh data"); return end

  local viewFrustum = Engine.sceneGetCameraFrustum()
  for i in pairs(vdata.vdata.mirrors) do
    if vdata.vdata.mirrors[i].mesh == mirror_name then
      local camData = core_camera.getCameraDataById(vid)
      if not camData or not camData.driver then
        log("E","focus", "invalid camera data")
        return
      end
      if not yawBeforeFocus then
        yawBeforeFocus = camData.driver.relativeYaw
        pitchBeforeFocus = camData.driver.relativePitch
      end

      local mpos = veh:getInitialNodePosition(vdata.vdata.mirrors[i].idRef)
      local absoluteMPos = veh:getPosition() + veh:getNodePosition(vdata.vdata.mirrors[i].idRef)
      if not viewFrustum:isPointContained(absoluteMPos) then
        local camPos = veh:getInitialNodePosition(core_camera.getDriverDataById(vid))
        local rot = quatFromDir( (camPos-mpos) , vec3(0,0,1) )
        local eu = rot:toEulerYXZ()

        camData.driver.relativeYaw = eu.x/math.pi
        camData.driver.relativePitch = -eu.y
      end
      break
    end
  end
end

local function onExtensionLoaded()
  settingsLoad()
end

M.onExtensionUnloaded = onExtensionUnloaded
M.onExtensionLoaded = onExtensionLoaded
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.onSettingsChanged = onSettingsChanged
M.onClientEndMission = onClientEndMission
M.onUpdate = onUpdate

M.onVehicleSpawned = onVehicleSpawned
M.onVehicleDestroyed = onVehicleDestroyed

M.focusOnMirror = focusOnMirror
M.setAngleOffset = setAngleOffset
M.getAnglesOffset = getAnglesOffset

M.vehicleEvent = vehicleEvent

M.setDebug = setDebug

return M
