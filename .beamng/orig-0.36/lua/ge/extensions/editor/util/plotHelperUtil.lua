-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local min = math.min
local max = math.max
local abs = math.abs
local ceil = math.ceil
local floor = math.floor
local huge = math.huge
local log10 = math.log10

local C = {}

local borderSize = 50

local borderColor = im.GetColorU322(im.ImVec4(1, 1, 1, 0.4), 1)
local gridColor = im.GetColorU322(im.ImVec4(1, 1, 1, 0.15), 1)
local annotationColor = im.GetColorU322(im.ImVec4(1, 1, 1, 0.22), 1)
local crossHairColor = im.GetColorU322(im.ImVec4(1, 1, 1, 0.5), 1)

local textColor = im.ImVec4(1, 1, 1, 0.5)

local p --= LuaProfiler("JBeam_Table_Visualizer")

-- USAGE
-- params.scale = {xMin, xMax, yMin, yMax} or nil
-- params.initData = data or nil
-- params.autoScale = true or false
-- params.showCatmullRomCurve = true or false
-- params.catmullromCurveLines = x or nil (x > 200 preferably)
function C:init(params)
  params = params or {
    scale = {xMin = -5, xMax = 5, yMin = -5, yMax = 5},
    autoScale = false,
    showCatmullRomCurve = true,
    catmullDataPoints = 10,
    allowZoom = true,
  }
  self._cmT = {}
  self._cmY0, self._cmY1, self._cmY2, self._cmY3 = {}, {}, {}, {}
  self._cmC1, self._cmC2, self._cmC3 = {}, {}, {}

  self.catmullDataPoints = nil
  self.formattingPerRow = {}

  self.screenPos = {x = 0, y = 0}
  self.innerSize = {x = 0, y = 0}
  self.scale = {}


  if params.scale then
    self:setScale(params.scale.xMin, params.scale.xMax, params.scale.yMin, params.scale.yMax)
  else
    self:setScale(-5, 5, -5, 5)
  end

  self.allowZoom = params.allowZoom or true
  if params.allowZoom == false then
    self.allowZoom = false
  end
  self.autoScale = params.autoScale or false
  self.showCatmullRomCurve = params.showCatmullRomCurve or false
  self.catmullromCurveLines = params.catmullromCurveLines or 10
  self.pointSize = params.pointSize or 4
  self.pointThickness = params.pointThickness or 2
  self.formatX = params.formatX or "%0.3f"
  self.formatY = params.formatY or "%0.3f"

  if params.initData then
    self:setData(params.initData)
  end
end

-- Method to set graph data
-- Must be in this format (multiple y values for one x allowed):
-- {{x0, y0, ...}, {x1, y1, ...}, ...}
function C:setData(data)
  local multiData = {}
  for _, d in ipairs(data) do
    local row = {}
    for n = 2, #d do
      table.insert(row, {d[1],d[n]})
    end
    table.insert(multiData, row)
  end
  self:setDataMulti(multiData)
end
-- Method to set graph data
-- Must be in this format (multiple y values for one x allowed):
-- {{x0, y0, ...}, {x1, y1, ...}, ...}
function C:setDataMulti(data)
  self.data = data

  if self.autoScale then
    self:scaleToFitData()
  end

  if self.showCatmullRomCurve then
    self:generateCatmullSplinePoints()
  end
end
-- sets the names of the series. is a list of names
function C:setSeriesNames(names)
  local format = self.seriesFormat or {}
  for i, name in ipairs(names) do
    format[i] = format[i] or {}
    format[i].name = name
  end
  self:setSeriesFormat(format)
end

-- sets the names of the series. is a list of names. Color should be a {r,g,b,a} format, 0-1 values
function C:setSeriesColors(colors)
  local format = self.seriesFormat or {}
  for i, color in ipairs(colors) do
    format[i] = format[i] or {}
    format[i].color = color
    format[i].uColor = im.GetColorU322(im.ImVec4(color[1], color[2], color[3], color[4]))
  end
  self:setSeriesFormat(format)
end

-- sets the names of the series. is a list of names
function C:setSeriesFormat(format)
  self.seriesFormat = format
end

function C:setAnnotationX(annotation)
  self.annotationX = annotation
end

-- Sets the domain/range of the graph to fit all data points in view
function C:scaleToFitData()
  if not self.data then return end

  local xMin = huge
  local xMax = -huge
  local yMin = huge
  local yMax = -huge

  for _, row in ipairs(self.data) do
    for _, d in ipairs(row) do
      local ptX = d[1]
      local ptY = d[2]

      xMin = min(xMin, ptX)
      xMax = max(xMax, ptX)
      yMin = min(yMin, ptY)
      yMax = max(yMax, ptY)
    end
  end

  local marginX = (xMax - xMin) / 10
  local marginY = (yMax - yMin) / 10

  self:setScale(xMin - marginX, xMax + marginX, yMin - marginY, yMax + marginY)
end

-- Sets the domain/range of the graph.
-- All parameters are optional so you can just set one of them and leave rest nil for example
function C:setScale(xMin, xMax, yMin, yMax)
  if xMin then
    self.scale.xMin = xMin
  end
  if xMax then
    self.scale.xMax = xMax
  end
  if yMin then
    self.scale.yMin = yMin
  end
  if yMax then
    self.scale.yMax = yMax
  end
  self.centerPos = {(self.scale.xMax - self.scale.xMin) / 2.0, (self.scale.yMax - self.scale.yMin) / 2.0}
end

-- Moves graph when dragging mouse with left mouse button
function C:moveGraph(dt)
  if im.IsMouseDragging(0) then
    if not self.isMouseDragging then
      -- First time
      -- Mouse must be inside graph first time
      if not self.mouseDownInGraph then return end
      self.originalScale = deepcopy(self.scale)

      self.isMouseDragging = true
    end

    local mouseDragDelta = im.GetMouseDragDelta(0)

    local xDelta = (self.originalScale.xMax - self.originalScale.xMin) * mouseDragDelta.x / self.innerSize.x
    local yDelta = (self.originalScale.yMax - self.originalScale.yMin) * mouseDragDelta.y / self.innerSize.y

    self:setScale(
      self.originalScale.xMin - xDelta,
      self.originalScale.xMax - xDelta,
      self.originalScale.yMin + yDelta,
      self.originalScale.yMax + yDelta
    )
  else
    self.isMouseDragging = false
  end
  if im.IsMouseClicked(0) then
    self.mouseDownInGraph = self.mouseInGraph
  end
end

-- Zooms graph with mouse scroll wheel and based on mouse position
function C:zoomGraph(dt)
  if not self.allowZoom then return end
  if not self.mouseInGraph then return end

  local mZoom = -self.mouseWheel * 20

  local xZoom = (self.scale.xMax - self.scale.xMin) * mZoom
  local yZoom = (self.scale.yMax - self.scale.yMin) * mZoom

  self:setScale(
    self.scale.xMin - xZoom * self.mousePosInWindowX / self.windowSize.x * dt,
    self.scale.xMax + xZoom * (self.windowSize.x - self.mousePosInWindowX) / self.windowSize.x * dt,
    self.scale.yMin - yZoom * (self.windowSize.y - self.mousePosInWindowY) / self.windowSize.y * dt,
    self.scale.yMax + yZoom * self.mousePosInWindowY / self.windowSize.y * dt
  )
end

-- Main method to call on every graphics update
function C:draw(width, height, dt)
  self.io = im.GetIO()
  self.mousePos = self.io.MousePos
  self.mouseWheel = self.io.MouseWheel

  self.windowPos = im.GetWindowPos()
  self.windowSize = im.GetWindowSize()

  self.mousePosInWindowX = self.mousePos.x - self.windowPos.x
  self.mousePosInWindowY = self.mousePos.y - self.windowPos.y

  if self.mousePosInWindowX >= 0 and self.mousePosInWindowX <= self.windowSize.x
  and self.mousePosInWindowY >= 0 and self.mousePosInWindowY <= self.windowSize.y then
    -- Cursor inside window
    self.mouseInWindow = true

    -- Cursor inside graph
    self.mouseInGraph =
      self.mousePosInWindowX >= borderSize and self.mousePosInWindowX <= self.windowSize.x - borderSize
      and self.mousePosInWindowY >= borderSize and self.mousePosInWindowY <= self.windowSize.y - borderSize
  else
    self.mouseInWindow = false
    self.mouseInGraph = false
  end

  self:moveGraph(dt)
  self:zoomGraph(dt)

  if p then p:start() end

  self.size = im.ImVec2(width, height)
  self.innerSize = im.ImVec2(width-2*borderSize, height-2*borderSize)
  im.BeginChild1('curve_'..self.id, self.size)
  self.dl = im.GetWindowDrawList()
  self.screenPos = im.GetCursorScreenPos()
  self.startCursorPos = im.GetCursorPos()
  if p then p:add("imgui init") end

  -- drawList
  if not self.data then return end

  self:drawGridAndAxesLabels()
  if p then p:add("drawGridAndAxesLabels()") end

  self:drawData()
  if p then p:add("drawData()") end

  if self.showCatmullRomCurve then
    self:drawCatmullSpline()
    if p then p:add("drawCatmullSpline()") end
  end

  if self.mouseInWindow then
    local min, max = self:normToImSS(0,0), self:normToImSS(1,1)
    im.ImDrawList_AddLine(self.dl, im.ImVec2(min.x,self.mousePos.y), im.ImVec2(max.x,self.mousePos.y), crossHairColor, 1)
    im.ImDrawList_AddLine(self.dl, im.ImVec2(self.mousePos.x,min.y), im.ImVec2(self.mousePos.x,max.y), crossHairColor, 1)
  end

  if self.annotationX then
    local endPos = im.GetCursorPos()
    local botPos, topPos = self:normToImWS(0,0), self:normToImWS(1,1)
    local min, max = self:normToImSS(0,0), self:normToImSS(1,1)
    for _, pair in ipairs(self.annotationX) do
      local x, label = pair[1],pair[2]
      local xPos = self:graphPtToImWS(x,self.scale.yMin + (self.scale.yMax - self.scale.yMin)/2)
      if xPos then
        im.SetCursorPos(im.ImVec2(xPos.x, botPos.y - im.GetTextLineHeightWithSpacing()))
        im.Text(" "..label)
        local xSs = self:graphPtToImSS(x,self.scale.yMin + (self.scale.yMax - self.scale.yMin)/2)
        im.ImDrawList_AddLine(self.dl, im.ImVec2(xSs.x,min.y), im.ImVec2(xSs.x,max.y), annotationColor, 1)
      end
    end
    im.SetCursorPos(endPos)
  end

  if p then p:finish(true) end

  im.EndChild()
end

function C:overlayTextLines(lines)
  if not lines then return end
  local endPos = im.GetCursorPos()
  local pos = self:normToImWS(0,1)
  for i, line in ipairs(lines) do
    im.SetCursorPos(im.ImVec2(pos.x + borderSize/3, pos.y +borderSize - im.GetTextLineHeight()/4 + im.GetTextLineHeight()*i ))
    im.Text(line)
  end
  im.SetCursorPos(endPos)
end

-- From lua/vehicle/ve_utils.lua and modified to allow multiple y values
function C:generateCatmullSplinePoints()
  local linesToDraw = self.catmullromCurveLines
  local res = {}

  for r, row in ipairs(self.data) do
    local cRow = {}
    local rowLength = #row
    if rowLength < 2 then
      goto continue
    elseif rowLength == 2 or linesToDraw == 1 then
      cRow = deepcopy(row)
      goto continue
    else
      for i = 1, rowLength - 1 do
        local p0, p1, p2, p3 = row[max(i - 1, 1)], row[i], row[i + 1], row[min(i + 2, rowLength)]
        local step = 0
        local numSteps = 0

        step = (p2[1] - p1[1]) / linesToDraw
        numSteps = linesToDraw

        self._cmT = 0
        self._cmY0, self._cmY1, self._cmY2, self._cmY3 = p0[2], p1[2], p2[2], p3[2]
        self._cmC1 = 3*(self._cmY1 - self._cmY2) + self._cmY3 - self._cmY0
        self._cmC2 = 2*self._cmY0 - 5*self._cmY1 + 4*self._cmY2 - self._cmY3
        self._cmC3 = self._cmY2 - self._cmY0

        for x = 0, numSteps do
          local val = {
            p1[1] + x * step,
            self._cmY1 + 0.5*self._cmT*(self._cmC3 + (self._cmC1*self._cmT + self._cmC2)*self._cmT)
          }
          self._cmT = self._cmT + (1 / numSteps)
          cRow[#cRow+1] = val
        end
      end
    end
    ::continue::
    res[r] = cRow
  end

  self.catmullDataPoints = res
end

function C:drawCatmullSpline()
  if not self.catmullDataPoints then return end
  local rowCount = #self.catmullDataPoints
  for r, row in ipairs(self.catmullDataPoints) do
    local rowLength = #row
    local splineColor = nil
    if self.seriesFormat and self.seriesFormat[r] and self.seriesFormat[r].uColor then
      splineColor = self.seriesFormat[r].uColor
    else
      local color = rainbowColor(rowCount, r, 1)
      splineColor = im.GetColorU322(im.ImVec4(color[1], color[2], color[3], color[4]))
    end
    for i = 1, rowLength -1 do
      local p0 = row[i]
      local p1 = row[i + 1]

      local x0 = p0[1]
      local y0 = p0[2]
      local x1 = p1[1]
      local y1 = p1[2]

      if x0 and y0 and x1 and y1 then
        local pt0 = self:graphPtToImSS(x0, y0)
        local pt1 = self:graphPtToImSS(x1, y1)

        if pt0 and pt1 then
          im.ImDrawList_AddLine(self.dl, pt0, pt1, splineColor, 1)
        end
      end
    end
  end
end

-- Draws the gridlines
function C:drawGridAndAxesLabels()
  -- Try to have roughly this many ticks
  local majorTicksX = 10
  local majorTicksY = 10

  local rangeX = self.scale.xMax - self.scale.xMin
  local rangeY = self.scale.yMax - self.scale.yMin

  -- Figure out the optimal increment to get closet to majorTicks
  -- increments of 1, 2, or 5 (in relation to magnitude of range)
  local incX = self:getOptimalTickIncrement(majorTicksX, rangeX)
  local incY = self:getOptimalTickIncrement(majorTicksY, rangeY)

  majorTicksX = rangeX / incX
  majorTicksY = rangeY / incY

  -- Draw Graph Border Outline
  im.ImDrawList_AddLine(self.dl, self:normToImSS(0,0), self:normToImSS(0,1), borderColor, 1)
  im.ImDrawList_AddLine(self.dl, self:normToImSS(0,1), self:normToImSS(1,1), borderColor, 1)
  im.ImDrawList_AddLine(self.dl, self:normToImSS(1,1), self:normToImSS(1,0), borderColor, 1)
  im.ImDrawList_AddLine(self.dl, self:normToImSS(1,0), self:normToImSS(0,0), borderColor, 1)

  local txt, txtSize

  -- Draw Vertical lines and labels
  for i = 0, majorTicksX do
    local graphX = i * incX + ceil(self.scale.xMin / incX) * incX

    local topPos = self:graphPtToImSS(graphX, self.scale.yMax, true)
    local botPos = self:graphPtToImSS(graphX, self.scale.yMin, true)

    im.ImDrawList_AddLine(self.dl, topPos, botPos, gridColor, 1)

    local txtPos = self:graphPtToImWS(graphX, self.scale.yMin)
    if txtPos then
      txt = tostring(abs(graphX))

      if #txt <= 5 then
        txt = tostring(graphX)
      else
        txt = string.format("%.3e", graphX)
      end
      txtSize = im.CalcTextSize(txt)

      txtPos.x = txtPos.x - txtSize.x / 2

      im.SetCursorPos(txtPos)
      im.TextColored(textColor, txt)
    end
  end

  -- Draw Horizontal Lines and labels
  for i = 0, majorTicksY do
    local graphY = i * incY + ceil(self.scale.yMin / incY) * incY

    local leftPos = self:graphPtToImSS(self.scale.xMax, graphY, true)
    local rightPos = self:graphPtToImSS(self.scale.xMin, graphY, true)

    im.ImDrawList_AddLine(self.dl, leftPos, rightPos, gridColor, 1)

    local txtPos = self:graphPtToImWS(self.scale.xMin, graphY)
    if txtPos then
      txt = tostring(abs(graphY))

      if #txt <= 5 then
        txt = tostring(graphY)
      else
        txt = string.format("%.3e", graphY)
      end

      txtSize = im.CalcTextSize(txt)

      txtPos.x = txtPos.x - txtSize.x - #txt / txtSize.x
      txtPos.y = txtPos.y - txtSize.y / 2

      im.SetCursorPos(txtPos)
      im.TextColored(textColor, txt)
    end
  end
end

-- Figure out the optimal increment to get closet to ticks
-- increments of 1, 2, or 5 (in relation to magnitude of range using incOneMagLess)
function C:getOptimalTickIncrement(ticks, range)
  local inc = range / ticks
  local rndInc = 10^self:getOrderOfMagnitude(inc)

  if abs(ticks - range / (rndInc * 10)) < abs(ticks - range / (rndInc * 5)) then
    return rndInc * 10
  elseif abs(ticks - range / (rndInc * 5)) < abs(ticks - range / (rndInc * 2)) then
    return rndInc * 5
  elseif abs(ticks - range / (rndInc * 2)) < abs(ticks - range / (rndInc * 1)) then
    return rndInc * 2
  else
    return rndInc * 1
  end
end

function C:drawData()
  local btnWidth = 10
  local btnSize = im.ImVec2(btnWidth,btnWidth)


  local count = tableSize(self.data)
  for r, row in ipairs(self.data) do
    local circleColor = nil
    local format = self.seriesFormat and self.seriesFormat[r] or {}
    if format and format.uColor then
      circleColor = format.uColor
    else
      local color = rainbowColor(count, r, 1)
      circleColor = im.GetColorU322(im.ImVec4(color[1], color[2], color[3], color[4]))
    end
    for i, d in ipairs(row) do

      local ptX = d[1]
      local ptY = d[2]

      local ptOnGraph = self:graphPtToImSS(ptX, ptY)

      -- nil means pt not visible
      if ptOnGraph then
        -- Draw point
        --ImDrawList_ctx, ImVec2_center, float_radius, ImU32_col, int_num_segments, float_thickness

        im.ImDrawList_AddCircle(self.dl, ptOnGraph, self.pointSize, circleColor, 8, self.pointThickness)

        -- Draw Tooltip
        local ptOnGraphWS = self:graphPtToImWS(ptX, ptY)

        ptOnGraphWS.x = ptOnGraphWS.x - btnWidth / 2
        ptOnGraphWS.y = ptOnGraphWS.y - btnWidth / 2

        local strX = string.format(self.formatX, ptX)
        local strY = string.format(self.formatY, ptY)

        im.SetCursorPos(ptOnGraphWS)
        im.InvisibleButton('##pointTooltipButton'..r.."/"..i, btnSize)
        local name = format.name or ("Series " .. r)
        im.tooltip(name .. " (" .. strX .. ", " .. strY .. ")")
      end
    end
  end
end

-- Returns IMGUI coordinates (screen space) on graph from normalized coordinates
-- x,y = 0 : bottom left
-- x,y = 1 : top right
function C:normToImSS(x,y)
  return im.ImVec2(borderSize + self.screenPos.x + x * self.innerSize.x,  borderSize + self.screenPos.y + (1-y) * self.innerSize.y)
end

-- Returns IMGUI coordinates (screen space) on graph from graph coordinates
function C:graphPtToImSS(x,y, clampVal)
  local newX = (x - self.scale.xMin) / (self.scale.xMax - self.scale.xMin)
  local newY = (y - self.scale.yMin) / (self.scale.yMax - self.scale.yMin)

  if clampVal then
    newX = clamp(newX, 0, 1)
    newY = clamp(newY, 0, 1)
  end

  -- If point goes out of bounds of graph, return nil
  if newX > 1 or newY > 1 or newX < 0 or newY < 0 then
    return nil
  end

  return self:normToImSS(newX, newY)
end

-- Returns IMGUI coordinates (window space) on graph from normalized coordinates
-- x,y = 0 : bottom left
-- x,y = 1 : top right
function C:normToImWS(x,y)
  return im.ImVec2(borderSize + x * self.innerSize.x,  borderSize + (1-y) * self.innerSize.y)
end

-- Returns IMGUI coordinates (window space) on graph from graph coordinates
function C:graphPtToImWS(x,y)
  local newX = (x - self.scale.xMin) / (self.scale.xMax - self.scale.xMin)
  local newY = (y - self.scale.yMin) / (self.scale.yMax - self.scale.yMin)

  -- If point goes out of bounds of graph, return nil
  if newX > 1 or newY > 1 or newX < 0 or newY < 0 then
    return nil
  end

  return self:normToImWS(newX, newY)
end

function C:roundToDigits(num, dec)
  local multiple = 10^dec

  return floor(num * multiple + 0.5) / multiple
end

-- Returns exponent of num represented in scientific notation (a * 10^x, where x is returned)
function C:getOrderOfMagnitude(num)
  return floor(log10(abs(num)))
end

function C:scaleX(x) return self.scale.xMin + (self.scale.xMax - self.scale.xMin) * x end
function C:scaleY(y) return self.scale.yMin + (self.scale.yMax - self.scale.yMin) * y end

local idCounter = 0
return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o.id = idCounter
  idCounter = idCounter + 1
  o:init(...)
  return o
end