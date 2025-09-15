-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local sensorColor = color(0, 0, 0, 255)                                                             -- The line colour of the sensor box visualisations.
local sensorTriColor = color(127, 127, 127, 255)                                                    -- The face colour of the sensor box visualisations.
local sphereColor = color(0, 0, 255, 255)                                                           -- The colour of the mouse spheres, when placing sensors.
local dirLineColor = color(0, 255, 0, 255)                                                          -- The colour of the 'dir' axes, when drawing sensor poses.
local upLineColor = color(0, 0, 255, 255)                                                           -- The colour of the 'up' axes, when drawing sensor poses.
local rightLineColor = color(255, 0, 0, 255)                                                        -- The colour of the 'right' axes, when drawing sensor poses.
local textA = color(25, 25, 25, 255)                                                                -- The markup text foreground colour.
local textB = color(255, 255, 255, 192)                                                             -- The markup text background colour.
local beamColour = ColorF(0.5, 0.5, 0.5, 0.1)                                                       -- The colour of the sensor beam (ultrasonic, RADAR).
local sensorLineThickness = 7                                                                       -- The thickness of the sensor box lines (visualisation).
local frameLineThickness = 7                                                                        -- The thickness of the frame lines, when drawing sensor poses.
local numBeamDivs = 300                                                                             -- The number of longitudinal divisions when rendering sensor beams.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

local dbgDraw = require('utils/debugDraw')

local min, max, sqrt, pow = math.min, math.max, math.sqrt, math.pow


-- Compute the local reference frame coefficients of a sensor, given the vehicle space position.
local function posVS2Coeffs(posVS, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return vec3(posVS:dot(fwd), posVS:dot(right), posVS:dot(up))
end

-- Compute the vehicle space position of a sensor, given the local reference frame coefficients.
local function coeffs2PosVS(c, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return c.x * fwd + c.y * right + c.z * up
end

-- Compute the vehicle space/world space frame of a sensor, given the local reference frame.
local function sensor2VS(dirLoc, upLoc, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return vec3(fwd:dot(dirLoc), right:dot(dirLoc), up:dot(dirLoc)), vec3(fwd:dot(upLoc), right:dot(upLoc), up:dot(upLoc))
end

-- Compute the local reference frame of a sensor, given the vehicle space/world space frame.
local function vS2Sensor(dirVS, upVS, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return vec3(fwd:dot(dirVS), right:dot(dirVS), up:dot(dirVS)), vec3(fwd:dot(upVS), right:dot(upVS), up:dot(upVS))
end

-- Computes the radius at a given y-value, for a sensor beam (ultrasonic and RADAR).
local function computeBeamShapeRadius(y, rangeRoundness, exponent, limitCoef)

  -- Set the initial interval on the spatial axis, in which we want to search for the root.
  local left, right = 0.0, 100.0

  -- Evaluate a simplified range function at the left and right positions.
  local distanceFromSensorSq = y * y;
  local f1 = distanceFromSensorSq + left * left;
  local fLeft = (y / sqrt(f1)) * (1.0 - rangeRoundness) + rangeRoundness - (limitCoef * pow(f1, exponent));
  local f2 = distanceFromSensorSq + right * right;
  local fRight = (y / sqrt(f2)) * (1.0 - rangeRoundness) + rangeRoundness - (limitCoef * pow(f2, exponent));

  -- Assuming monotonicity, if the left and right evaluations are of different sign, we have a root in the interval.
  -- If so, we proceeed to perform some iterations of the binary search algorithm to approach this root, since this formula is difficult to solve analytically.
  local mid
  if fLeft * fRight < 0.0 then
    for i = 0, 10 do
      mid = (left + right) * 0.5
      local f3 = distanceFromSensorSq + mid * mid;
      local fMid = (y / (sqrt(f3))) * (1.0 - rangeRoundness) + rangeRoundness - (limitCoef * pow(f3, exponent));
      if fMid < 0.0 then
        right = mid
      else
        left = mid
      end
    end
    return mid;
  end
  return 0.0
end

-- Renders a sensor beam shape (relevant for ultrasonic and RADAR sensors).
local function renderBeamShape(s, pos, fwd, up, right)
  local rangeRoundness = s.rangeRoundness
  local rangeCutoffSensitivity = s.rangeCutoffSensitivity
  local rangeShape = s.rangeShape
  local rangeFocus = s.rangeFocus
  local rangeMinCutoff = s.rangeMinCutoff
  local rangeDirectMaxCutoff = s.rangeDirectMaxCutoff
  local dy = rangeDirectMaxCutoff - rangeMinCutoff
  local divs = dy / numBeamDivs
  for i = 0, numBeamDivs do
    local y = rangeMinCutoff + i * divs
    local r = min(
      computeBeamShapeRadius(y, rangeRoundness, rangeShape, rangeFocus),
      computeBeamShapeRadius(y, rangeRoundness, 2, rangeCutoffSensitivity),
      sqrt(max(0, rangeDirectMaxCutoff * rangeDirectMaxCutoff - y*y)))
    local p1, p2 = pos + fwd * y, pos + fwd * (rangeMinCutoff + (i + 1) * divs)
    debugDrawer:drawCylinder(p1, p2, r, beamColour)
  end
end

-- Draws a sphere to represent the mouse position.
local function drawMouseSphere(p)
  dbgDraw.drawSphere(p, 0.05 * sqrt(p:distance(core_camera.getPosition())), sphereColor)
end

-- Renders the local frame of a sensor.
local function renderLocalFrame(posWS, dir, up)
  local dirEndPos, upEndPos, rightEndPos = posWS + dir, posWS + up, posWS + dir:cross(up)
  dbgDraw.drawLineInstance_MinArg(posWS, dirEndPos, frameLineThickness, dirLineColor)
  dbgDraw.drawLineInstance_MinArg(posWS, upEndPos, frameLineThickness, upLineColor)
  dbgDraw.drawLineInstance_MinArg(posWS, rightEndPos, frameLineThickness, rightLineColor)
  dbgDraw.drawTextAdvanced(dirEndPos, 'Dir', textA, true, false, textB)
  dbgDraw.drawTextAdvanced(upEndPos, 'Up', textA, true, false, textB)
  dbgDraw.drawTextAdvanced(rightEndPos, 'Right', textA, true, false, textB)
end

-- Renders a sensor box and local frame.
local function renderSensorBoxAndFrame(pos, fwd, up, right)
  local wHalf, lHalf, hHalf = 0.15, 0.15, 0.15
  local whr, whf, whu = wHalf * right, lHalf * fwd, hHalf * up
  local c1, c2, c3, c4 = pos - whr + whf - 0.5 * whu, pos + whr + whf - 0.5 * whu, pos - whr - whf - 0.5 * whu, pos + whr - whf - 0.5 * whu
  local c5, c6, c7, c8 = c1 + whu, c2 + whu, c3 + whu, c4 + whu
  local e1, e2 = pos + 1.2 * whf - 1.2 * whr - 0.6 * whu, pos + 1.2 * whf + 1.2 * whr - 0.6 * whu
  local e3, e4 = e1 + 1.2 * whu, e2 + 1.2 * whu

  dbgDraw.drawLineInstance_MinArg(c1, c2, sensorLineThickness, sensorColor)                         -- The bottom four lines.
  dbgDraw.drawLineInstance_MinArg(c3, c4, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c1, c3, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c2, c4, sensorLineThickness, sensorColor)

  dbgDraw.drawLineInstance_MinArg(c5, c6, sensorLineThickness, sensorColor)                         -- The top four lines.
  dbgDraw.drawLineInstance_MinArg(c7, c8, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c5, c7, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c6, c8, sensorLineThickness, sensorColor)

  dbgDraw.drawLineInstance_MinArg(c1, c5, sensorLineThickness, sensorColor)                         -- The four vertical lines.
  dbgDraw.drawLineInstance_MinArg(c2, c6, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c3, c7, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c4, c8, sensorLineThickness, sensorColor)

  dbgDraw.drawLineInstance_MinArg(e1, e2, sensorLineThickness, sensorColor)                         -- The aperture end lines
  dbgDraw.drawLineInstance_MinArg(e3, e4, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(e1, e3, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(e2, e4, sensorLineThickness, sensorColor)

  dbgDraw.drawLineInstance_MinArg(c1, e1, sensorLineThickness, sensorColor)                         -- The box-to-aperture lines.
  dbgDraw.drawLineInstance_MinArg(c2, e2, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c5, e3, sensorLineThickness, sensorColor)
  dbgDraw.drawLineInstance_MinArg(c6, e4, sensorLineThickness, sensorColor)

  dbgDraw.drawTriSolid(c1, c2, c3, sensorTriColor, true)                                            -- Bottom.
  dbgDraw.drawTriSolid(c2, c4, c3, sensorTriColor, true)
  dbgDraw.drawTriSolid(c1, c3, c2, sensorTriColor, true)
  dbgDraw.drawTriSolid(c2, c3, c4, sensorTriColor, true)

  dbgDraw.drawTriSolid(c5, c6, c7, sensorTriColor, true)                                            -- Top.
  dbgDraw.drawTriSolid(c6, c8, c7, sensorTriColor, true)
  dbgDraw.drawTriSolid(c5, c7, c6, sensorTriColor, true)
  dbgDraw.drawTriSolid(c6, c7, c8, sensorTriColor, true)

  dbgDraw.drawTriSolid(c1, c3, c5, sensorTriColor, true)                                            -- Left.
  dbgDraw.drawTriSolid(c3, c5, c7, sensorTriColor, true)
  dbgDraw.drawTriSolid(c1, c5, c3, sensorTriColor, true)
  dbgDraw.drawTriSolid(c3, c7, c5, sensorTriColor, true)

  dbgDraw.drawTriSolid(c2, c4, c6, sensorTriColor, true)                                            -- Right.
  dbgDraw.drawTriSolid(c4, c6, c8, sensorTriColor, true)
  dbgDraw.drawTriSolid(c2, c6, c4, sensorTriColor, true)
  dbgDraw.drawTriSolid(c4, c8, c6, sensorTriColor, true)

  dbgDraw.drawTriSolid(c3, c4, c7, sensorTriColor, true)                                            -- Back.
  dbgDraw.drawTriSolid(c3, c7, c8, sensorTriColor, true)
  dbgDraw.drawTriSolid(c3, c7, c4, sensorTriColor, true)
  dbgDraw.drawTriSolid(c3, c8, c7, sensorTriColor, true)

  dbgDraw.drawTriSolid(e1, e2, e3, sensorTriColor, true)                                            -- Front.
  dbgDraw.drawTriSolid(e2, e3, e4, sensorTriColor, true)
  dbgDraw.drawTriSolid(e1, e3, e2, sensorTriColor, true)
  dbgDraw.drawTriSolid(e2, e4, e3, sensorTriColor, true)

  dbgDraw.drawTriSolid(e3, e4, c5, sensorTriColor, true)                                            -- Top aperture.
  dbgDraw.drawTriSolid(e4, c5, c6, sensorTriColor, true)
  dbgDraw.drawTriSolid(e3, c5, e4, sensorTriColor, true)
  dbgDraw.drawTriSolid(e4, c6, c5, sensorTriColor, true)

  dbgDraw.drawTriSolid(c1, c2, e1, sensorTriColor, true)                                            -- Bottom aperture.
  dbgDraw.drawTriSolid(c2, e1, e2, sensorTriColor, true)
  dbgDraw.drawTriSolid(c1, e1, c2, sensorTriColor, true)
  dbgDraw.drawTriSolid(c2, e2, e1, sensorTriColor, true)

  dbgDraw.drawTriSolid(c1, e1, e3, sensorTriColor, true)                                            -- Left aperture.
  dbgDraw.drawTriSolid(c1, e3, c5, sensorTriColor, true)
  dbgDraw.drawTriSolid(c1, e3, e1, sensorTriColor, true)
  dbgDraw.drawTriSolid(c1, c5, e3, sensorTriColor, true)

  dbgDraw.drawTriSolid(c2, c6, e4, sensorTriColor, true)                                            -- Right aperture.
  dbgDraw.drawTriSolid(c2, e2, e4, sensorTriColor, true)
  dbgDraw.drawTriSolid(c2, e4, c6, sensorTriColor, true)
  dbgDraw.drawTriSolid(c2, e4, e2, sensorTriColor, true)
end


-- Public interface.
M.posVS2Coeffs =                                          posVS2Coeffs
M.coeffs2PosVS =                                          coeffs2PosVS
M.sensor2VS =                                             sensor2VS
M.vS2Sensor =                                             vS2Sensor
M.computeBeamShapeRadius =                                computeBeamShapeRadius
M.renderBeamShape =                                       renderBeamShape
M.drawMouseSphere =                                       drawMouseSphere
M.renderLocalFrame =                                      renderLocalFrame
M.renderSensorBoxAndFrame =                               renderSensorBoxAndFrame

return M