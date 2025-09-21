-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local logTag = ''
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

function C:init(path, name, forceId)
  self.path = path

  -- sortedList fields
  self.id = forceId or self.path:getNextUniqueIdentifier()
  self.name = name or ('t_'..self.id)
  self.sortOrder = 999999
  self.show = true

  self.text = nil
  self.success = false
  self.src = nil
  self.file = nil
  self.beamng_file = nil
  self.timestamp = nil
  self.vehicle_data = nil

  self.grouped_captures = nil
end

function C:toggleShow()
  self.show = not self.show
end

function C:setShow(val)
  self.show = val
end

function C:isUsable()
  return self.vehicle_data and self.vehicle_data.vehicle_data and self.show
end

function C:debugDrawText(hovered)
  local txt = self.text
  if hovered then
    txt = txt .. ' (click to copy)'
  end
  return txt
end

function C:vehiclePos()
  if not (self.vehicle_data.vehicle_data and self.vehicle_data.vehicle_data.pos) then return nil end
  return vec3(self.vehicle_data.vehicle_data.pos)
end

function C:vehicleQuat()
  if not (self.vehicle_data.vehicle_data and self.vehicle_data.vehicle_data.quat) then return nil end
  return quat(self.vehicle_data.vehicle_data.quat)
end

function C:capture_data()
  if not self.vehicle_data.capture_data then return nil end
  return self.vehicle_data.capture_data
end

local function mylerp(a, b, t)
  return a + (b - a) * t
end

local function createGradient(steps)
  local gradient = {}

  for i = 1, steps do
    local t = (i - 1) / (steps - 1)  -- Normalize t to 0-1
    local r, g, b

    if t <= 0.5 then
      -- Interpolate between red and yellow
      r = 1
      g = mylerp(0, 1, t * 2)  -- Double t because it's only half the gradient
      b = 0
    else
      -- Interpolate between yellow and green
      r = mylerp(1, 0, (t - 0.5) * 2)  -- Adjust t and double for second half
      g = 1
      b = 0
    end

    table.insert(gradient, {r, g, b})
  end

  return gradient
end

function C:get_grouped_captures()
  local force_reload = false

  local capture_data = self:capture_data()
  if not capture_data then return end

  if self.grouped_captures then return self.grouped_captures end

  local sortedAngles = nil

  -- local function scale_l(i, n)
  --   return i / n
  -- end

  if editor_rallyEditor then
    local cornerAnglesStyle = capture_data.cornerAnglesStyle
    local corner_angles_data = editor_rallyEditor.getTranscriptsWindow():getCornerAngles(force_reload)
    local style_data = nil
    for _,style in ipairs(corner_angles_data.pacenoteStyles) do
      if style.name == cornerAnglesStyle then
        style_data = style
      end
    end

    -- Example usage
    -- local base_clr = cc.clr_teal
    -- local h, s, l = rgbToHsl(base_clr[1], base_clr[2], base_clr[3])
    -- print("HSL:", h, s, l)

    -- for i, color in ipairs(gradientColors) do
      -- print("Step " .. i .. ": RGB(" .. color.r .. ", " .. color.g .. ", " .. color.b .. ")")
    -- end

    sortedAngles = {}
    for i,angle in ipairs(style_data.angles) do
      table.insert(sortedAngles, angle)
    end
    local function sortByAngleRev(a, b)
      return a.fromAngleDegrees > b.fromAngleDegrees
    end
    table.sort(sortedAngles, sortByAngleRev)

    local steps = #sortedAngles - 1  -- Number of color steps in the gradient
    -- subtract 1 for Center
    local gradientColors = createGradient(steps)
    for i,angle in ipairs(sortedAngles) do
      angle.color = gradientColors[i]
    end
    sortedAngles[#sortedAngles].color = cc.clr_white
  end

  local subgroups = {{captures = {}, label_point=-1, calc=nil}}

  for _,cap in ipairs(capture_data.captures) do
    local captures = subgroups[#subgroups].captures

    local angle_data, cornerCallStr, pct = rallyUtil.determineCornerCall(sortedAngles, cap.steering)
    cap.calc = {
      -- angle_i = angle_i,
      angle_pct = pct,
      angle_data = angle_data,
      cornerCallStr = cornerCallStr,
    }

    -- if cornerCallStr == 'C' then
      -- cap.calc.color_within_angle = cap.calc.angle_data.color
    -- else
    --   -- local h, s, l = 0.33, 0.5, 0.5 -- Example HSL values
    --   local base_clr = cap.calc.angle_data.color
    --   local h,s,l = rgbToHsl(base_clr[1], base_clr[2], base_clr[3])
    --   local scaled_l = pct -- scale_l(pct)
    --   scaled_l = scaled_l * 0.5
    --   scaled_l = scaled_l + 0.25
    --   log("D", "wtf", scaled_l)
    --   local r, g, b = hslToRgb(h, s, scaled_l)
    --   cap.calc.color_within_angle = {r,g,b}
    -- end

    if #captures == 0 then
      table.insert(captures, cap)
      subgroups[#subgroups].calc = cap.calc
    elseif captures[#captures].calc.cornerCallStr ~= cap.calc.cornerCallStr then
      table.insert(subgroups, {captures={cap}, label_point=-1, calc=cap.calc})
    else
      table.insert(captures, cap)
      subgroups[#subgroups].calc = cap.calc
    end
  end

  for _,grp in ipairs(subgroups) do
    local label_i = round(#grp.captures / 2)
    grp.label_point = grp.captures[label_i]
  end

  -- calc the corner call for each grp>cap
  -- for _,grp in ipairs(subgroups) do
  --   for _,cap in ipairs(grp.captures) do
  --     -- for _,angle in ipairs(self.sortedAngles) do
  --     cap.cornerCall_calc = rallyUtil.determineCornerCall(self.sortedAngles, cap.steering)
  --     -- end
  --   end
  -- end

  -- log('D', 'wtf', '===')
  -- for _,grp in ipairs(subgroups) do
  --   for i,cap in ipairs(grp.captures) do
  --     if i == grp.label_point then
  --       log('D', 'wtf', '> '..cap.cornerCall)
  --     else
  --       log('D', 'wtf', cap.cornerCall)
  --     end
  --   end
  --   log('D', 'wtf', '---')
  -- end

  self.grouped_captures = subgroups

  return self.grouped_captures
end

function C:onSerialize()
  local ret = {
    oldId = self.id,
    name = self.name,
    show = self.show,

    text = self.text,
    success = self.success,
    src = self.src,
    file = self.file,
    beamng_file = self.beamng_file,
    timestamp = self.timestamp,
    vehicle_data = self.vehicle_data,
  }

  return ret
end

function C:onDeserialized(data, oldIdMap)
  self.text = data.text or ''
  self.success = data.success or false
  self.src = data.src or ''
  self.file = data.file or ''
  self.beamng_file = data.beamng_file or ''
  self.timestamp = data.timestamp or 0.0
  self.vehicle_data = data.vehicle_data or {}

  if not self:vehiclePos() then
    self.show = false
  else
    self.show = (data.show == nil and true) or data.show
  end
end

function C:drawDebug(is_hovered, is_selected)
  self:drawDebugVehicleData(is_hovered, is_selected)
  self:drawDebugCaptureData(is_hovered, is_selected)
end

function C:drawDebugVehicleData(is_hovered, is_selected)
  local pos = self:vehiclePos()
  local rot = self:vehicleQuat()

  if self.show and rot and pos then
    local h = 1.6
    local w = 1.8
    local l = 4.4

    local upVector = vec3(0,0,1)  -- 'up' in a Z-up system
    local rotatedUpVector = rot * upVector * h  -- Rotate and scale the up vector
    -- log('D', logTag, dumps(rotatedUpVector:normalized()))

    local forwardVector = vec3(0,1,0)
    local rotatedForwardVector = rot * forwardVector * (l/2) -- assume pos is the center of car so divide length by 2
    local frontOfCar = pos + rotatedForwardVector
    local backOfCar = pos - rotatedForwardVector

    local raise = vec3(0,0,h/2)
    frontOfCar = frontOfCar + raise
    backOfCar = backOfCar + raise

    local wheelPositions = {
      {0.5, vec3(-(w/2*0.9),   l/2 * 0.6,  0.4)}, -- Front left
      {0.5, vec3( (w/2*0.9),   l/2 * 0.6,  0.4)}, -- Front right
      {0.6, vec3(-(w/2*1.1), -(l/2 * 0.6), 0.4)}, -- Rear left
      {0.6, vec3( (w/2*1.1), -(l/2 * 0.6), 0.4)}, -- Rear right
    }

    -- Function to rotate and translate a local position to a world position
    local function toWorldPosition(localPos)
      local rotatedPos = rot * localPos  -- Rotate by car's orientation
      return pos + rotatedPos             -- Translate to car's world position
    end

    -- Draw the wheels
    for _, wheelPos in ipairs(wheelPositions) do
      local worldWheelPos = toWorldPosition(wheelPos[2])
      debugDrawer:drawSphere(worldWheelPos, wheelPos[1], ColorF(0,0,0,1))
    end

    local clr_base = cc.clr_teal
    local clr = clr_base
    local shapeAlpha = 1.0
    local textAlpha = 0.7
    local clr_text_fg = cc.clr_black
    local clr_text_bg = cc.clr_teal

    if is_hovered then
      clr = cc.clr_white
      clr_text_bg = cc.clr_teal_2
      textAlpha = 1.0
    elseif is_selected then
      clr = cc.clr_teal_2
      clr_text_bg = cc.clr_teal_2
      textAlpha = 1.0
    end

    debugDrawer:drawSquarePrism(
      frontOfCar,
      backOfCar,
      Point2F(h*0.7, w*0.7), -- make the car look more aero
      Point2F(h, w),
      ColorF(clr[1], clr[2], clr[3], shapeAlpha)
    )

    debugDrawer:drawTextAdvanced(
      backOfCar + vec3(0,0,h/2),
      String(self:debugDrawText(is_hovered)),
      ColorF(clr_text_fg[1], clr_text_fg[2], clr_text_fg[3], textAlpha),
      true,
      false,
      ColorI(clr_text_bg[1]*255, clr_text_bg[2]*255, clr_text_bg[3]*255, textAlpha*255)
    )
  end
end

function C:drawDebugCaptureData(is_hovered, is_selected)
  if not is_hovered and not is_selected then
    return
  end

  local capture_data = self:capture_data()
  if not capture_data then return end

  -- local captures = capture_data.captures
  local cornerAnglesStyle = capture_data.cornerAnglesStyle

  local radius = 0.5
  local shapeAlpha = 0.9
  local clr = cc.clr_teal

  local groups = self:get_grouped_captures()
  if not groups then return end

  for i_grp,grp in ipairs(groups) do
    for _,cap in ipairs(grp.captures) do
      clr = cap.calc.angle_data.color
      -- clr = cap.calc.color_within_angle
      -- log('D', 'wtf', dumps(clr))

      local pos = vec3(cap.pos)
      debugDrawer:drawSphere(pos, radius, ColorF(clr[1],clr[2],clr[3],shapeAlpha))
    end
    -- log('D', 'wtf', '---')
    local label_point = grp.label_point
    local calc = grp.calc
    local clr_text_fg = cc.clr_black
    local clr_text_bg = calc.angle_data.color
    local textAlpha = 1.0

    debugDrawer:drawTextAdvanced(
      vec3(label_point.pos),
      String(calc.cornerCallStr..' '),
      ColorF(clr_text_fg[1], clr_text_fg[2], clr_text_fg[3], textAlpha),
      true,
      false,
      ColorI(clr_text_bg[1]*255, clr_text_bg[2]*255, clr_text_bg[3]*255, textAlpha*255)
    )
  end

  -- for _,capture in ipairs(captures) do
    -- {
    --   "cornerCall":"1R",
    --   "pos":{
    --     "x":-394.0718994,
    --     "y":67.10164642,
    --     "z":51.19839096
    --   },
    --   "quat":{
    --     "w":0.163846025,
    --     "x":-0.01032584195,
    --     "y":-0.01322749495,
    --     "z":0.986343191
    --   },
    --   "steering":-190.2713122,
    --   "ts":45.6030861
    -- },

    -- local pos = vec3(capture.pos)
    -- debugDrawer:drawSphere(pos, radius, ColorF(clr[1],clr[2],clr[3],shapeAlpha))
  -- end
end

-- {
--   "looped":false,
--   "manualFov":false,
--   "markers":[
--     {
--       "fov":20,
--       "movingEnd":true,
--       "movingStart":true,
--       "pos":{
--         "x":547.2601318,
--         "y":576.2945557,
--         "z":118.7703247
--       },
--       "rot":{
--         "z":-0.7501421743,
--         "w":0.6541676806,
--         "x":0.06355749063,
--         "y":0.072882161
--       },
--       "time":0,
--       "trackPosition":true
--     },
--     {
--       "fov":20,
--       "movingEnd":true,
--       "movingStart":true,
--       "pos":{
--         "x":549.3669434,
--         "y":573.342041,
--         "z":118.89608
--       },
--       "rot":{
--         "z":-0.7501421743,
--         "w":0.6541676806,
--         "x":0.06355749063,
--         "y":0.072882161
--       },
--       "time":3,
--       "trackPosition":true
--     }
--   ],
--   "name":"camPath1",
--   "rotFixId":2,
--   "version":"6"
-- }

-- copied from core_paths.loadPath
local function loadPath(captures)
  local capture_markers = {}

  local t_zero = captures[1].ts

  for i,cap in ipairs(captures) do
    local marker = {
      fov = 60,
      movingEnd = true,
      movingStart = true,
      pos = cap.pos + vec3(0,0,1),
      rot = cap.quat,
      time = cap.ts - t_zero,
      trackPosition = false,
      -- positionSmooth = 0.0,
    }
    table.insert(capture_markers, marker)
  end

  local pathJsonObj = {
    looped = false,
    loopTime = nil,
    name = 'rally_campath',
    rotFixId = 2,
    version = "6",
    markers = capture_markers,
  }

  local defaultSplineSmoothing = 0.5

  -- unchanged from original below this line --

  local res = { markers = {}}

  res.looped = pathJsonObj.looped or false
  res.loopTime = pathJsonObj.loopTime


  -- extract all its markers
  for i, markerData in ipairs(pathJsonObj.markers) do
    local marker = {
      pos = vec3(markerData.pos.x, markerData.pos.y, markerData.pos.z),
      rot = quat(markerData.rot.x, markerData.rot.y, markerData.rot.z, markerData.rot.w),
      time = markerData.time,
      fov = markerData.fov,
      trackPosition = markerData.trackPosition,
      positionSmooth = markerData.positionSmooth or defaultSplineSmoothing,
      bullettime = markerData.bullettime or 1,
      cut = markerData.cut,
      movingStart = markerData.movingStart or true,
      movingEnd = markerData.movingEnd or true
    }
    table.insert(res.markers, marker)
  end
  -- fix up the rotations
  local markerCount = tableSize(pathJsonObj.markers)
  for i = 2, markerCount do
    if res.markers[i] then
      if res.markers[i].rot:dot(res.markers[i - 1].rot) < 0 then
        res.markers[i].rot = -res.markers[i].rot
      end
    end
  end
  res.rotFixId = markerCount

  -- res.filename = pathFileName
  -- res.replay = pathJsonObj.replay
  res.name = pathJsonObj.name
  -- addPath(res)
  return res
end

function C:playCameraPath()
  local cam_path = loadPath(self:capture_data().captures)
  log('D', logTag, 'playing camPath')
  core_paths.playPath(cam_path)
end

function C:lookAtMe()
  rallyUtil.setCameraTarget(self:vehiclePos())
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
