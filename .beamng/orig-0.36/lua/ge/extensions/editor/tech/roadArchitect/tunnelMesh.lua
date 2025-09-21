-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Control parameters.
local material = 'm_asphalt_new_01'                                                                 -- The material to use for the mesh.

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Module state.
local meshes = {}
local tunnelHoles = {}
local meshIdx = 1

-- Module constants.
local floor, ceil = math.floor, math.ceil
local min, max, sqrt = math.min, math.max, math.sqrt
local twoPi = 2.0 * math.pi
local uvs = {                                                                                       -- Fixed UV-map corner points (used in all road meshes).
  { u = 0.0, v = 0.0 },
  { u = 0.0, v = 1.0 },
  { u = 1.0, v = 0.0 },
  { u = 1.0, v = 1.0 } }
local origin = vec3(0, 0, 0)                                                                        -- A vec3 used for representing the scene origin.
local scaleVec = vec3(1, 1, 1)                                                                      -- A vec3 used for representing uniform scale.
local up = vec3(0, 0, 1)
local tmp0, tmp1 = vec3(0, 0), vec3(0, 0)


-- Pierces (edits) the holemap with respect to the given tunnel.
-- [Any height plateau inside the tunnel will be converted to a hole].
local function editHolemap(roadIdx, name, tunnel, rData)

  -- Determine the pipe center and lateral edge indices.
  local rD1, cIdx1, cIdx2, lIdx, rIdx = rData[1], -1, 3, nil, nil
  if not rD1[-1] then
    cIdx1, cIdx2 = 1, 4
  end
  for i = -20, 20 do
    if rD1[i] then
      lIdx = i
      break
    end
  end
  for i = 20, -20, -1 do
    if rD1[i] then
      rIdx = i
      break
    end
  end

  -- Compute the ring at each longitudinal point in the given section.
  local iStart, iEnd, radOffset, zOffsetFromRoad = tunnel.s, tunnel.e, tunnel.radOffset, tunnel.zOffsetFromRoad
  local zVec, roadLen = up * zOffsetFromRoad, #rData
  local numRings = iEnd - iStart + 1
  local pipe = {}
  for i = 1, numRings do
    local idx = iStart - 1 + i
    local rD = rData[idx]
    local rTan = rData[min(roadLen, idx + 1)][cIdx1][cIdx2] - rData[max(1, idx - 1)][cIdx1][cIdx2]
    rTan:normalize()                                                                                -- The road unit tangent vector.
    local rDLeft = rD[lIdx]
    local rLeft, rRight = rDLeft[4], rD[rIdx][3]                                                    -- The left-most and right-most lateral road points.
    local rCen = (rLeft + rRight) * 0.5                                                             -- The road lateral center.
    local rWidth = rLeft:distance(rRight)                                                           -- The (lateral) road width at this longitudinal point.
    pipe[i] = {
      r = rWidth + radOffset,                                                                       -- The radius of the pipe at this longitudinal point.
      cen = rCen + zVec,                                                                            -- The pipe center.
      tan = rTan }                                                                                  -- The pipe unit tangent vector.
  end

  -- Iterate across each section of the pipe, and determine if the heightmap intersects it.
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local granFac = 2
  local holes, hCtr = {}, 1
  local xMinG, xMaxG, yMinG, yMaxG = 1e99, -1e99, 1e99, -1e99
  local cenS, cenE, tanS, tanE = pipe[1].cen, pipe[numRings].cen, pipe[1].tan, pipe[numRings].tan
  for i = 2, numRings do
    local pipe1, pipe2 = pipe[i - 1], pipe[i]
    local r1, c1, r2, c2 = pipe1.r, pipe1.cen, pipe2.r, pipe2.cen
    local xMin, xMax = floor(min(c1.x - r1, c2.x - r2)), ceil(max(c1.x + r1, c2.x + r2))            -- The 2D AABB of this section, with a radial margin included.
    local yMin, yMax = floor(min(c1.y - r1, c2.y - r2)), ceil(max(c1.y + r1, c2.y + r2))
    xMinG, xMaxG, yMinG, yMaxG = min(xMinG, xMin), max(xMaxG, xMax), min(yMinG, yMin), max(yMaxG, yMax)
    local rAvg = (r1 + r2) * 0.5
    local rAvgSq = rAvg * rAvg
    local dx, dy = xMax - xMin, yMax - yMin
    local xGran, yGran = dx * granFac, dy * granFac
    local xFac, yFac = dx / xGran, dy / yGran
    for xxx = 0, xGran do                                                                           -- Iterate over the 2D AABB, and find any intersections.
      local xx = xMin + xxx * xFac
      local sx1, sx2 = floor(xx), ceil(xx)
      for yyy = 0, yGran do
        local yy = yMin + yyy * yFac
        local sy1, sy2 = floor(yy), ceil(yy)
        tmp0:set(sx1, sy1, 0)
        tmp1:set(sx1, sy1, core_terrain.getTerrainHeight(tmp0))
        local dSq = tmp1:squaredDistanceToLineSegment(c1, c2)
        if (i < 5 and (tmp1 - cenS):dot(tanS) > 0.0) or (i > numRings - 5 and (tmp1 - cenE):dot(tanE) < 0.0) or (i > 4 and i < numRings - 4) then   -- At ends, use plane clipping.
          if dSq < rAvgSq then
            holes[hCtr] = { p = vec3(tmp1.x, tmp1.y, 0), i = tb:getMaterialIdxWs(tmp1) }
            hCtr = hCtr + 1
            tb:setMaterialIdxWs(tmp1, 255)
          end
        end

        tmp0:set(sx1, sy2, 0)
        tmp1:set(sx1, sy2, core_terrain.getTerrainHeight(tmp0))
        local dSq = tmp1:squaredDistanceToLineSegment(c1, c2)
        if (i < 5 and (tmp1 - cenS):dot(tanS) > 0.0) or (i > numRings - 5 and (tmp1 - cenE):dot(tanE) < 0.0) or (i > 4 and i < numRings - 4) then   -- At ends, use plane clipping.
          if dSq < rAvgSq then
            holes[hCtr] = { p = vec3(tmp1.x, tmp1.y, 0), i = tb:getMaterialIdxWs(tmp1) }
            hCtr = hCtr + 1
            tb:setMaterialIdxWs(tmp1, 255)
          end
        end

        tmp0:set(sx2, sy1, 0)
        tmp1:set(sx2, sy1, core_terrain.getTerrainHeight(tmp0))
        local dSq = tmp1:squaredDistanceToLineSegment(c1, c2)
        if (i < 5 and (tmp1 - cenS):dot(tanS) > 0.0) or (i > numRings - 5 and (tmp1 - cenE):dot(tanE) < 0.0) or (i > 4 and i < numRings - 4) then   -- At ends, use plane clipping.
          if dSq < rAvgSq then
            holes[hCtr] = { p = vec3(tmp1.x, tmp1.y, 0), i = tb:getMaterialIdxWs(tmp1) }
            hCtr = hCtr + 1
            tb:setMaterialIdxWs(tmp1, 255)
          end
        end

        tmp0:set(sx2, sy2, 0)
        tmp1:set(sx2, sy2, core_terrain.getTerrainHeight(tmp0))
        local dSq = tmp1:squaredDistanceToLineSegment(c1, c2)
        if (i < 5 and (tmp1 - cenS):dot(tanS) > 0.0) or (i > numRings - 5 and (tmp1 - cenE):dot(tanE) < 0.0) or (i > 4 and i < numRings - 4) then   -- At ends, use plane clipping.
          if dSq < rAvgSq then
            holes[hCtr] = { p = vec3(tmp1.x, tmp1.y, 0), i = tb:getMaterialIdxWs(tmp1) }
            hCtr = hCtr + 1
            tb:setMaterialIdxWs(tmp1, 255)
          end
        end
      end
    end
  end

  -- Update the terrain block.
  local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
  local te = extensions.editor_terrainEditor.getTerrainEditor()
  te:worldToGridByPoint2I(vec3(xMinG, yMinG), gMin, tb)
  te:worldToGridByPoint2I(vec3(xMaxG, yMaxG), gMax, tb)
  local w2gMin, w2gMax = vec3(gMin.x, gMin.y), vec3(gMax.x, gMax.y)
  tb:updateGridMaterials(w2gMin, w2gMax)
  tb:updateGrid(w2gMin, w2gMax)

  tunnelHoles[roadIdx] = tunnelHoles[roadIdx] or {}
  tunnelHoles[roadIdx][name] = holes
end

-- Recover the holemap after the removal of a tunnel.
local function recoverHolemap(roadIdx, name)
  local tb = extensions.editor_terrainEditor.getTerrainBlock()
  local xMinG, xMaxG, yMinG, yMaxG = 1e99, -1e99, 1e99, -1e99
  local done = {}
  local holesRoad = tunnelHoles[roadIdx]
  if holesRoad then
    local tunHoles = holesRoad[name]
    if tunHoles then
      for i = 1, #tunHoles do
        local hS = tunHoles[i]
        local p = hS.p
        local x, y = p.x, p.y
        if not done[x] or not done[x][y] then
          xMinG, xMaxG, yMinG, yMaxG = min(xMinG, x), max(xMaxG, x), min(yMinG, y), max(yMaxG, y)
          tb:setMaterialIdxWs(p, hS.i)
          if not done[x] then
            done[x] = {}
          end
          done[x][y] = true
        end
      end

      -- Update the terrain block.
      local gMin, gMax = Point2I(0, 0), Point2I(0, 0)
      local te = extensions.editor_terrainEditor.getTerrainEditor()
      te:worldToGridByPoint2I(vec3(xMinG, yMinG), gMin, tb)
      te:worldToGridByPoint2I(vec3(xMaxG, yMaxG), gMax, tb)
      local w2gMin, w2gMax = vec3(gMin.x, gMin.y), vec3(gMax.x, gMax.y)
      tb:updateGridMaterials(w2gMin, w2gMax)
      tb:updateGrid(w2gMin, w2gMax)

      tunnelHoles[roadIdx][name] = nil
    end
  end
end

-- Create a procedural auto tunnel mesh for a road.
local function createTunnel(roadIdx, name, rData, tunnel, folder)

  -- Determine the pipe center and lateral edge indices.
  local rD1, cIdx1, cIdx2, lIdx, rIdx = rData[1], -1, 3, nil, nil
  if not rD1[-1] then
    cIdx1, cIdx2 = 1, 4
  end
  for i = -20, 20 do
    if rD1[i] then
      lIdx = i
      break
    end
  end
  for i = 20, -20, -1 do
    if rD1[i] then
      rIdx = i
      break
    end
  end

  -- Compute the ring at each longitudinal point in the given section.
  local iStart, iEnd, radGran, radOffset = tunnel.s, tunnel.e, tunnel.radGran, tunnel.radOffset
  local thickness, zOffsetFromRoad = tunnel.thickness, tunnel.zOffsetFromRoad
  local protrudeS, protrudeE = tunnel.protrudeS, tunnel.protrudeE
  local theta, zVec, roadLen, rGranP1 = twoPi / radGran, up * zOffsetFromRoad, #rData, radGran + 1
  local ringsI, ringsO, normals = {}, {}, {}
  local numRings = iEnd - iStart + 1
  for i = 1, numRings do
    local idx = iStart - 1 + i
    local rD = rData[idx]
    local rTan = rData[min(roadLen, idx + 1)][cIdx1][cIdx2] - rData[max(1, idx - 1)][cIdx1][cIdx2]
    rTan:normalize()                                                                                -- The road unit tangent vector.
    local rDLeft = rD[lIdx]
    local rNorm = rDLeft[5]                                                                         -- The road unit normal vector.
    local rLeft, rRight = rDLeft[4], rD[rIdx][3]                                                    -- The left-most and right-most lateral road points.
    local rCen = (rLeft + rRight) * 0.5                                                             -- The road lateral center.
    if i == 1 then
      rCen = rCen - rTan * protrudeS                                                                -- Protrude the start point, if required.
    elseif i == numRings then
      rCen = rCen + rTan * protrudeE                                                                -- Protrude the end point, if required.
    end
    local rWidth = rLeft:distance(rRight)                                                           -- The (lateral) road width at this longitudinal point.
    local nOff = rWidth + radOffset                                                                 -- The normal offset, based on the radii.
    local normWithRInner, normWithROuter = rNorm * (nOff - thickness), rNorm * (nOff + thickness)   -- The full normal offset vectors.
    local pCen = rCen + zVec                                                                        -- The pipe center.
    local ringI, ringO = {}, {}
    local zI = min(rLeft.z, rRight.z)
    local zO = zI - thickness
    for j = 1, rGranP1 do                                                                           -- Compute the cross-sectional rings of points aroung this lon. point.
      local angle = (j - 1) * theta
      ringI[j] = pCen + util.rotateVecAroundAxis(normWithRInner, rTan, angle)
      ringO[j] = pCen + util.rotateVecAroundAxis(normWithROuter, rTan, angle)
      ringI[j].z = max(ringI[j].z, zI)
      ringO[j].z = max(ringO[j].z, zO)
    end
    ringsI[i], ringsO[i], normals[i] = ringI, ringO, rNorm
  end

  -- Create the vertices and their corresponding normals and uv map values.
  local numRings = #ringsI
  local vertsO, vertsI, normsO, normsI, ctr = {}, {}, {}, {}, 1
  for i = 1, numRings do
    local ringI, ringO, norm = ringsI[i], ringsO[i], normals[i]
    for j = 1, rGranP1 do
      local inner, outer = ringI[j], ringO[j]
      vertsO[ctr] = { x = outer.x, y = outer.y, z = outer.z }
      vertsI[ctr] = { x = inner.x, y = inner.y, z = inner.z }
      normsO[ctr] = { x = norm.x, y = norm.y, z = norm.z }
      normsI[ctr] = { x = -norm.x, y = -norm.y, z = -norm.z }
      ctr = ctr + 1
    end
  end

  -- Create the faces.
  local facesO, facesI, ctr, numRingsMinus1, rGranP1 = {}, {}, 1, numRings - 1, radGran + 1
  local faceO_UVs, faceI_UVs, uCtr = {}, {}, 1
  for i = 1, numRingsMinus1 do
    local i1 = (i - 1) * rGranP1
    local i2 = i1 + 1
    local i3 = i1 + rGranP1
    local i4 = i2 + rGranP1
    for _ = 1, rGranP1 do
      local ctr1, ctr2, ctr3, ctr4, ctr5 = ctr + 1, ctr + 2, ctr + 3, ctr + 4, ctr + 5

      local q1, q2, q3 = vertsO[i1 + 1], vertsO[i2 + 1], vertsO[i3 + 1]
      local dx = sqrt((q1.x - q2.x) * (q1.x - q2.x) + (q1.y - q2.y) * (q1.y - q2.y) + (q1.z - q2.z) * (q1.z - q2.z))
      local dy = sqrt((q1.x - q3.x) * (q1.x - q3.x) + (q1.y - q3.y) * (q1.y - q3.y) + (q1.z - q3.z) * (q1.z - q3.z))
      faceO_UVs[uCtr] = { u = 0, v = 0 }
      faceO_UVs[uCtr + 1] = { u = dx, v = 0 }
      faceO_UVs[uCtr + 2] = { u = 0, v = dy }
      faceO_UVs[uCtr + 3] = { u = dx, v = dy }

      facesO[ctr] = { v = i1, n = i1, u = uCtr - 1 }                                                       -- General: Outer triangle 1-3-2.
      facesO[ctr1] = { v = i3, n = i3, u = uCtr + 1 }
      facesO[ctr2] = { v = i2, n = i2, u = uCtr }
      facesO[ctr3] = { v = i3, n = i3, u = uCtr + 1 }                                                      -- General: Outer triangle 3-4-2.
      facesO[ctr4] = { v = i4, n = i4, u = uCtr + 2 }
      facesO[ctr5] = { v = i2, n = i2, u = uCtr }

      local q1, q2, q3 = vertsI[i1 + 1], vertsI[i2 + 1], vertsI[i3 + 1]
      local dx = sqrt((q1.x - q2.x) * (q1.x - q2.x) + (q1.y - q2.y) * (q1.y - q2.y) + (q1.z - q2.z) * (q1.z - q2.z))
      local dy = sqrt((q1.x - q3.x) * (q1.x - q3.x) + (q1.y - q3.y) * (q1.y - q3.y) + (q1.z - q3.z) * (q1.z - q3.z))
      faceI_UVs[uCtr] = { u = 0, v = 0 }
      faceI_UVs[uCtr + 1] = { u = dx, v = 0 }
      faceI_UVs[uCtr + 2] = { u = 0, v = dy }
      faceI_UVs[uCtr + 3] = { u = dx, v = dy }

      facesI[ctr] = { v = i2, n = i2, u = uCtr }                                                       -- General: Inner triangle 2-3-1.
      facesI[ctr1] = { v = i3, n = i3, u = uCtr + 1 }
      facesI[ctr2] = { v = i1, n = i1, u = uCtr - 1 }
      facesI[ctr3] = { v = i2, n = i2, u = uCtr }                                                      -- General: Inner triangle 2-4-3.
      facesI[ctr4] = { v = i4, n = i4, u = uCtr + 2 }
      facesI[ctr5] = { v = i3, n = i3, u = uCtr + 1 }

      ctr = ctr + 6
      uCtr = uCtr + 4
      i1, i2, i3, i4 = i1 + 1, i2 + 1, i3 + 1, i4 + 1
    end
  end

  -- Start cap vertices.
  local capVertsS, ctr = {}, 1
  for i = 1, rGranP1 do
    capVertsS[ctr] = { x = vertsO[i].x, y = vertsO[i].y, z = vertsO[i].z }
    capVertsS[ctr + 1] = { x = vertsI[i].x, y = vertsI[i].y, z = vertsI[i].z }
    ctr = ctr + 2
  end

  -- Start cap faces.
  local normS = normals[1]
  local capSnormal = { { x = normS.x, y = normS.y, z = normS.z } }                                  -- The start normal is the same for all start cap triangles.
  local capFacesS, ctr, vIdx = {}, 1, 1
  for _ = 1, radGran do
    local vIdxM1, vIdxP1, vIdxP2 = vIdx - 1, vIdx + 1, vIdx + 2
    capFacesS[ctr] = { v = vIdxP1, n = 0, u = 1 }                                                   -- Start cap: quad triangle A.
    capFacesS[ctr + 1] = { v = vIdx, n = 0, u = 2 }
    capFacesS[ctr + 2] = { v = vIdxM1, n = 0, u = 0 }
    capFacesS[ctr + 3] = { v = vIdxP2, n = 0, u = 1 }                                               -- Start cap: quad triangle B.
    capFacesS[ctr + 4] = { v = vIdx, n = 0, u = 0 }
    capFacesS[ctr + 5] = { v = vIdxP1, n = 0, u = 3 }
    ctr, vIdx = ctr + 6, vIdx + 2
  end

  -- End cap vertices
  local capVertsE, ctr, numVertsO = {}, 1, #vertsO
  for i = 1, rGranP1 do
    local idx = numVertsO - i + 1
    capVertsE[ctr] = { x = vertsO[idx].x, y = vertsO[idx].y, z = vertsO[idx].z }
    capVertsE[ctr + 1] = { x = vertsI[idx].x, y = vertsI[idx].y, z = vertsI[idx].z }
    ctr = ctr + 2
  end

  -- End cap faces.
  local normE = normals[#normals]
  local capEnormal = { { x = normE.x, y = normE.y, z = normE.z } }
  local capFacesE, ctr = {}, 1
  local vIdx = 1
  for _ = 1, radGran do
    local vIdxM1, vIdxP1, vIdxP2 = vIdx - 1, vIdx + 1, vIdx + 2
    capFacesE[ctr] = { v = vIdxP1, n = 0, u = 1 }                                                   -- End cap: quad triangle A.
    capFacesE[ctr + 1] = { v = vIdx, n = 0, u = 2 }
    capFacesE[ctr + 2] = { v = vIdxM1, n = 0, u = 0 }
    capFacesE[ctr + 3] = { v = vIdxP2, n = 0, u = 1 }                                               -- End cap: quad triangle B.
    capFacesE[ctr + 4] = { v = vIdx, n = 0, u = 0 }
    capFacesE[ctr + 5] = { v = vIdxP1, n = 0, u = 3 }
    ctr, vIdx = ctr + 6, vIdx + 2
  end

  -- Create the primitives structures.
  local pOuter = { verts = vertsO, faces = facesO, normals = normsO, uvs = faceO_UVs, material = material }
  local pInner = { verts = vertsI, faces = facesI, normals = normsI, uvs = faceI_UVs, material = material }
  local pSCap = { verts = capVertsS, faces = capFacesS, normals = capSnormal, uvs = uvs, material = material }
  local pECap = { verts = capVertsE, faces = capFacesE, normals = capEnormal, uvs = uvs, material = material }

  local mesh = { pOuter, pInner, pSCap, pECap }

  -- Generate the procedural mesh.
  local proc = createObject('ProceduralMesh')
  proc:registerObject('Auto_Tunnel_' .. tonumber(meshIdx))
  meshIdx = meshIdx + 1
  proc.canSave = true
  folder:addObject(proc.obj)
  proc:createMesh({ mesh })
  proc:setPosition(origin)
  proc.scale = scaleVec

  meshes[roadIdx] = meshes[roadIdx] or {}
  meshes[roadIdx][name] = proc

  editHolemap(roadIdx, name, tunnel, rData)
end

-- Attempts to removes the given mesh from the scene (if it exists).
local function tryRemove(roadIdx, name)
  if meshes[roadIdx] and name and meshes[roadIdx][name] then
    recoverHolemap(roadIdx, name)
    local mesh = meshes[roadIdx][name]
    if mesh then
      mesh:delete()
    end
    meshes[roadIdx][name] = nil
  end
end


-- Public interface.
M.createTunnel =                                          createTunnel

M.tryRemove =                                             tryRemove

return M