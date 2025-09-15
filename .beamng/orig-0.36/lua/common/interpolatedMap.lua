-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--https://x-engineer.org/bilinear-interpolation/

local clamp = clamp

-- Performs bilinear interpolation to find a value at point (x,y) in a 2D data map
-- Uses the formula from: https://x-engineer.org/bilinear-interpolation/
-- @param self: The interpolation map object containing the data and indices
-- @param x: The x-coordinate to interpolate at
-- @param y: The y-coordinate to interpolate at
-- @return: The interpolated value, or nil if coordinates are out of range and not clamping
local function getBiLinear(self, x, y)
  -- Check if x is within bounds, clamp if enabled
  if x < self.xMin or x > self.xMax then
    if self.clampToDataRange then
      --print("clamping x to range")
      x = clamp(x,self.xMin, self.xMax)
    else
      --print("x outside of range")
      return nil
    end
  end

  -- Check if y is within bounds, clamp if enabled
  if y < self.yMin or y > self.yMax then
    if self.clampToDataRange then
      --print("clamping y to range")
      y = clamp(y,self.yMin, self.yMax)
    else
      --print("y outside of range")
      return nil
    end
  end

  -- Find the x indices for interpolation
  local x1Index
  local x2Index

  if x == self.xMax then
    --print("x is xMax")
    x1Index = self.xMaxIndex - 1
    x2Index = self.xMaxIndex
  else
    for xIndex, xValue in ipairs(self.xIndices) do
      if x >= xValue and x < self.xIndices[xIndex + 1] then
        x1Index = xIndex
        x2Index = xIndex + 1
        break
      end
    end
  end

  -- Return nil if we couldn't find valid x indices
  if not x1Index or not x2Index then
    --print("Could not find valid x indices")
    return nil
  end

  -- print("x1: "..x1Index)
  -- print("x2: "..x2Index)

  local y1Index
  local y2Index

-- Find the y indices for interpolation
  if y == self.yMax then
    --print("y is yMax")
    y1Index = self.yMaxIndex - 1
    y2Index = self.yMaxIndex
  else
    for yIndex, yValue in ipairs(self.yIndices) do
      if y >= yValue and y < self.yIndices[yIndex + 1] then
        y1Index = yIndex
        y2Index = yIndex + 1
        break
      end
    end
  end

  -- Return nil if we couldn't find valid y indices
  if not y1Index or not y2Index then
    -- print("Could not find valid y indices")
    return nil
  end

  -- print("y1: "..y1Index)
  -- print("y2: "..y2Index)

  -- Get the corner values for interpolation
  local q11 = self.data[y1Index][x1Index]  -- Value at (x1,y1)
  local q12 = self.data[y1Index][x2Index]  -- Value at (x2,y1)
  local q21 = self.data[y2Index][x1Index]  -- Value at (x1,y2)
  local q22 = self.data[y2Index][x2Index]  -- Value at (x2,y2)

  -- print("q11: "..q11)
  -- print("q12: "..q12)
  -- print("q21: "..q21)
  -- print("q22: "..q22)

  -- Get the actual coordinate values for interpolation
  local x1Value = self.xIndices[x1Index]
  local x2Value = self.xIndices[x2Index]
  local y1Value = self.yIndices[y1Index]
  local y2Value = self.yIndices[y2Index]

  -- print("x1Value: "..x1Value)
  -- print("x2Value: "..x2Value)
  -- print("y1Value: "..y1Value)
  -- print("y2Value: "..y2Value)

  -- Perform bilinear interpolation
  -- First interpolate along x direction for both y values
  local r1 = q11 * (y2Value - y) / (y2Value - y1Value) + q21 * (y - y1Value) / (y2Value - y1Value)
  local r2 = q12 * (y2Value - y) / (y2Value - y1Value) + q22 * (y - y1Value) / (y2Value - y1Value)
  -- Then interpolate between those results along y direction
  local p = r1 * (x2Value - x) / (x2Value - x1Value) + r2 * (x - x1Value) / (x2Value - x1Value)

  -- print("r1: "..r1)
  -- print("r2: "..r2)
  -- print("p: "..p)

  return p
end

-- Gets an interpolated value from the 2D map at coordinates (x,y)
-- Currently only supports bilinear interpolation
-- @param x: The x-coordinate to look up
-- @param y: The y-coordinate to look up
-- @return: The interpolated value at (x,y), or nil if outside data range and not clamping
local get = function(self, x, y)
  return getBiLinear(self, x, y)
end

local loadData = function(self, inputData)
  --[[
  example data from jbeam:
  first line is _just_ the x axis header
  every following row then has the y axis header value as the first entry
  and then the x axis values
        "gearRatioMap":[
            [0, 0.5, 1]
            [0, 0.5, 1, 1],
            [7, 0.4, 0.9, 1],
            [14, 0.3, 0.8, 1],
            [21, 0.2, 0.7, 1],
            [28, 0.1, 0.6, 1],
            [35, 0, 0.5, 1],
            [42, 0, 0.4, 0.9],
            [49, 0, 0.3, 0.8],
            [56, 0, 0.2, 0.7],
            [63, 0, 0.1, 0.6],
            [70, 0, 0, 0.5]
        ]
  ]]
  local data = deepcopy(inputData)
  --we expectd the first row to be the header and that it has one less entry than the second row,
  --if that is not the case (for example because it contains a label in the very first cell), fix it
  if #data[1] == #data[2] then
    table.remove(data[1], 1)
  end
  self.xIndices = data[1]

  self.yIndices = {}
  self.data = {}
  for yIndex, rowData in ipairs(data) do
    if yIndex > 1 then
      table.insert(self.yIndices, rowData[1])
      local row = {}
      for xIndex, value in ipairs(rowData) do
        if xIndex > 1 then
          table.insert(row, value)
        end
      end
      table.insert(self.data, row)
    end
  end

  self.xMin = self.xIndices[1]
  self.xMax = self.xIndices[#self.xIndices]
  self.xMaxIndex = #self.xIndices
  self.yMin = self.yIndices[1]
  self.yMax = self.yIndices[#self.yIndices]
  self.yMaxIndex = #self.yIndices
end

local function reset(self)
  --TODO
end

local methods = {
  get = get,
  loadData = loadData,
  reset = reset
}

local new = function(interpolationMethod) --TODO useCache, cachePrecisionX, cachePrecisionY, preFillCache)
  local r = {
    data = {},
    xIndices = {},
    yIndices = {},
    xMin = 0,
    xMax = 0,
    xMaxIndex = 0,
    yMin = 0,
    yMax = 0,
    yMaxIndex = 0,
    clampToDataRange = false,
    --useCache = useCache, --TODO
    --cachePrecisionX = cachePrecisionX, --TODO
    --cachePrecisionY = cachePrecisionY, --TODO
    --preFillCache = preFillCache, --TODO
    --cacheData = {} --TODO
  }

  return setmetatable(r, {__index = methods})
end

return {
  new = new
}
