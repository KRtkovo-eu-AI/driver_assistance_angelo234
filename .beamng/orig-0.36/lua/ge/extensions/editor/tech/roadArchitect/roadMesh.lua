-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Module constants.
local kerbVGap = 0.00332                                                                            -- The small gap at the end of each sub-section of the curb UV-material, in v.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- External modules used.
local util = require('editor/tech/roadArchitect/utilities')                                         -- A module containing miscellaneous utility functions.

-- Module state.
local meshes = {}                                                                                   -- The collection of road (procedural) meshes.
local bridges = {}

-- Module constants.
local uvs = {                                                                                       -- Fixed uv-map corner points (used in all road meshes).
  { u = 0.0, v = 0.0 },
  { u = 0.0, v = 1.0 },
  { u = 1.0, v = 0.0 },
  { u = 1.0, v = 1.0 } }
local downVec = vec3(0, 0, -1)
local raised = vec3(0, 0, 4.5)
local origin = vec3(0, 0, 0)                                                                        -- A vec3 used for representing the scene origin.
local scaleVec = vec3(1, 1, 1)                                                                      -- A vec3 used for representing uniform scale.
local tmp1, tmp2, tmp3 = vec3(0, 0), vec3(0, 0), vec3(0, 0)                                         -- Some temporary vectors.
local materials =  {                                                                                -- The mesh materials, per type.
  road_lane = 'm_asphalt_new_01',
  sidewalk = 'slabs_large',
  curbMaterial = 'm_sidewalk_curb_trim_01',
  defaultMaterial = 'm_asphalt_new_01' }


-- Identifies the different lateral sections of the road, by lane indexing.
local function computeSections(r)
  local sections, sCtr = {}, 1
  local lMin, lMax = r.laneKeys[1], r.laneKeys[#r.laneKeys]
  local rData = r.renderData
  local tCur = rData[1][lMin][8]
  local tBuild = { min = lMin, type = tCur }
  for i = lMin + 1, lMax do
    if i ~= 0 then
      local tHere = rData[1][i][8]
      if tHere ~= tBuild.type or tHere == 'sidewalk' then                                           -- Sidewalks should all be separate sections (one lane per section).
        tBuild.max = i - 1
        sections[sCtr] = tBuild
        sCtr = sCtr + 1
        tBuild = { min = i, type = tHere }
      end
    end
  end

  -- Add the final lateral section.
  tBuild.max = lMax
  sections[sCtr] = tBuild

  return sections
end

-- Creates procedural meshes for a left (right-facing) sidewalk + curb lane.
local function createLeftSidewalkKerb(r, sec, roadMeshIdx, s, folder)
  local lIdx = sec.min
  local rData = r.renderData
  local kVStart = r.profile[lIdx].vStart[0] * 0.2
  local vertsSW, vertsK, ctr = {}, {}, 1
  for i = 1, #rData do
    local rD = rData[i][lIdx]
    local p1 = vec3(rD[1].x, rD[1].y, rD[1].z) + raised
    local p2 = vec3(rD[2].x, rD[2].y, rD[2].z) + raised
    local p3 = vec3(rD[3].x, rD[3].y, rD[3].z) + raised
    local p4 = vec3(rD[4].x, rD[4].y, rD[4].z) + raised
    p1.z = p1.z - castRayStatic(p1, downVec, 1000) + rD[1].z - rD[4].z
    p2.z = p2.z - castRayStatic(p2, downVec, 1000) + rD[2].z - rD[3].z
    p3.z = p3.z - castRayStatic(p3, downVec, 1000)
    p4.z = p4.z - castRayStatic(p4, downVec, 1000)

    local sTop = p2 - (p2 - p1):normalized() * r.profile[lIdx].kerbWidth[0]
    local sBot = p3 - (p3 - p4):normalized() * r.profile[lIdx].kerbWidth[0]
    p2 = p2 - (p3 - p2):normalized() * r.profile[lIdx].cornerDrop[0] - rD[6] * r.profile[lIdx].cornerLatOff[0]    -- Move the curb corner as requested.
    vertsSW[ctr] = { x = p1.x, y = p1.y, z = p1.z }
    vertsSW[ctr + 1] = { x = sTop.x, y = sTop.y, z = sTop.z }
    vertsSW[ctr + 2] = { x = sBot.x, y = sBot.y, z = sBot.z }
    vertsSW[ctr + 3] = { x = p4.x, y = p4.y, z = p4.z }
    vertsSW[ctr + 4] = { x = p1.x, y = p1.y, z = p1.z }

    vertsK[ctr] = { x = sTop.x, y = sTop.y, z = sTop.z }
    vertsK[ctr + 1] = { x = p2.x, y = p2.y, z = p2.z }
    vertsK[ctr + 2] = { x = p3.x, y = p3.y, z = p3.z }
    vertsK[ctr + 3] = { x = sBot.x, y = sBot.y, z = sBot.z }
    vertsK[ctr + 4] = { x = sTop.x, y = sTop.y, z = sTop.z }
    ctr = ctr + 5
  end

  local facesSW, facesK, ctr = {}, {}, 1
  local normalsSW, normalsK, nCtr, nCtr2 = {}, {}, 1, 1
  local uvSW, uvK, uvCtr = {}, {}, 1
  for i = 1, #vertsSW - 5, 5 do

    -- Top quad.
    local i1 = (i - 1) + 1

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    -- The curb corner normal is interpolated.
    local j1 = (i - 1) + 2
    tmp1:set(vertsSW[j1].x, vertsSW[j1].y, vertsSW[j1].z)
    tmp2:set(vertsSW[j1 + 1].x, vertsSW[j1 + 1].y, vertsSW[j1 + 1].z)
    tmp3:set(vertsSW[j1 + 5].x, vertsSW[j1 + 5].y, vertsSW[j1 + 5].z)
    local n2 = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n2:normalize()
    local nCorner = util.slerp(n, n2, 0.5)
    normalsK[nCtr2 + 1] = { x = nCorner.x, y = nCorner.y, z = nCorner.z }

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dxT = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart }
    uvK[uvCtr + 1] = { u = dxT, v = kVStart }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 3] = { u = dxT, v = kVStart + 0.1 }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 2
    uvCtr = uvCtr + 4

    -- Right quad.
    local i1 = (i - 1) + 2

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dy = tmp1:distance(tmp2)
    local dx = tmp1:distance(tmp3)
    uvSW[uvCtr] = { u = 0.0, v = dy }
    uvSW[uvCtr + 1] = { u = dx, v = dy }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy + dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy + dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dx2 = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 1] = { u = dx2, v = kVStart + 0.1 }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.2 - kerbVGap }
    uvK[uvCtr + 3] = { u = dx2, v = kVStart + 0.2 - kerbVGap }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 2, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 2, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 2, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4

    --[[
    -- Bottom quad.
    local i1 = (i - 1) + 3

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dxT = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart }
    uvK[uvCtr + 1] = { u = dxT, v = kVStart }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 3] = { u = dxT, v = kVStart + 0.1 }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4
    --]]

    -- Left quad.
    local i1 = (i - 1) + 4

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dx2 = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 1] = { u = dx2, v = kVStart + 0.1 }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.2 - kerbVGap }
    uvK[uvCtr + 3] = { u = dx2, v = kVStart + 0.2 - kerbVGap }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4
  end

  local surfaceMeshSW = { verts = vertsSW, faces = facesSW, normals = normalsSW, uvs = uvSW, material = materials.sidewalk }
  local surfaceMeshK = { verts = vertsK, faces = facesK, normals = normalsK, uvs = uvK, material = materials.curbMaterial }

  -- End caps.
  local last = #vertsSW
  local vertsSW2 = { vertsSW[1], vertsSW[2], vertsSW[3], vertsSW[4], vertsSW[last - 3], vertsSW[last - 2], vertsSW[last - 1], vertsSW[last] }
  local vertsK2 = { vertsK[1], vertsK[2], vertsK[3], vertsK[4], vertsK[last - 3], vertsK[last - 2], vertsK[last - 1], vertsK[last] }

  local nStart = rData[1][lIdx][5]:cross(rData[1][lIdx][6])
  local nEnd = rData[#rData][lIdx][5]:cross(rData[#rData - 1][lIdx][6])
  local normals = {
    { x = nStart.x, y = nStart.y, z = nStart.z },
    { x = nEnd.x, y = nEnd.y, z = nEnd.z } }

  local faces = {
    { v = 0, n = 0, u = 0 },
    { v = 3, n = 0, u = 3 },
    { v = 1, n = 0, u = 1 },

    { v = 3, n = 0, u = 3 },
    { v = 2, n = 0, u = 2 },
    { v = 1, n = 0, u = 1 },

    { v = 4, n = 1, u = 0 },
    { v = 5, n = 1, u = 1 },
    { v = 7, n = 1, u = 3 },

    { v = 7, n = 1, u = 3 },
    { v = 5, n = 1, u = 1 },
    { v = 6, n = 1, u = 2 } }

  local endCapsSW = { verts = vertsSW2, faces = faces, normals = normals, uvs = uvs, material = materials.sidewalk }
  local endCapsK = { verts = vertsK2, faces = faces, normals = normals, uvs = uvs, material = materials.curbMaterial }

  -- Generate the procedural meshes for the sidewalk.
  local mesh = createObject('ProceduralMesh')
  mesh:registerObject('Sidewalk_' .. tostring(roadMeshIdx) .. '_' .. tostring(s))
  mesh.canSave = true
  folder:addObject(mesh.obj)
  mesh:createMesh({ { surfaceMeshSW, endCapsSW } })
  mesh:setPosition(origin)
  mesh.scale = scaleVec
  meshes[r.name][#meshes[r.name] + 1] = mesh

  local mesh = createObject('ProceduralMesh')
  mesh:registerObject('Curb_' .. tostring(roadMeshIdx) .. '_' .. tostring(s))
  mesh.canSave = true
  folder:addObject(mesh.obj)
  mesh:createMesh({ { surfaceMeshK, endCapsK } })
  mesh:setPosition(origin)
  mesh.scale = scaleVec
  meshes[r.name][#meshes[r.name] + 1] = mesh
end

-- Creates procedural meshes for a right (left-facing) sidewalk + curb lane.
local function createRightSidewalkKerb(r, sec, roadMeshIdx, s, folder)
  local lIdx = sec.min
  local rData = r.renderData
  local kVStart = r.profile[lIdx].vStart[0] * 0.2
  local vertsSW, vertsK, ctr = {}, {}, 1
  for i = #rData, 1, -1 do
    local rD = rData[i][lIdx]
    local p2 = vec3(rD[1].x, rD[1].y, rD[1].z) + raised
    local p1 = vec3(rD[2].x, rD[2].y, rD[2].z) + raised
    local p4 = vec3(rD[3].x, rD[3].y, rD[3].z) + raised
    local p3 = vec3(rD[4].x, rD[4].y, rD[4].z) + raised
    p1.z = p1.z - castRayStatic(p1, downVec, 1000) + rD[2].z - rD[3].z
    p2.z = p2.z - castRayStatic(p2, downVec, 1000) + rD[1].z - rD[4].z
    p3.z = p3.z - castRayStatic(p3, downVec, 1000)
    p4.z = p4.z - castRayStatic(p4, downVec, 1000)
    local sTop = p2 - (p2 - p1):normalized() * r.profile[lIdx].kerbWidth[0]
    local sBot = p3 - (p3 - p4):normalized() * r.profile[lIdx].kerbWidth[0]
    p2 = p2 - (p3 - p2):normalized() * r.profile[lIdx].cornerDrop[0] - rD[6] * r.profile[lIdx].cornerLatOff[0]    -- Move the curb corner as requested.
    vertsSW[ctr] = { x = p1.x, y = p1.y, z = p1.z }
    vertsSW[ctr + 1] = { x = sTop.x, y = sTop.y, z = sTop.z }
    vertsSW[ctr + 2] = { x = sBot.x, y = sBot.y, z = sBot.z }
    vertsSW[ctr + 3] = { x = p4.x, y = p4.y, z = p4.z }
    vertsSW[ctr + 4] = { x = p1.x, y = p1.y, z = p1.z }

    vertsK[ctr] = { x = sTop.x, y = sTop.y, z = sTop.z }
    vertsK[ctr + 1] = { x = p2.x, y = p2.y, z = p2.z }
    vertsK[ctr + 2] = { x = p3.x, y = p3.y, z = p3.z }
    vertsK[ctr + 3] = { x = sBot.x, y = sBot.y, z = sBot.z }
    vertsK[ctr + 4] = { x = sTop.x, y = sTop.y, z = sTop.z }
    ctr = ctr + 5
  end

  local facesSW, facesK, ctr = {}, {}, 1
  local normalsSW, normalsK, nCtr, nCtr2 = {}, {}, 1, 1
  local uvSW, uvK, uvCtr = {}, {}, 1
  for i = 1, #vertsSW - 5, 5 do

    -- Top quad.
    local i1 = (i - 1) + 1

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    -- The curb corner normal is interpolated.
    local j1 = (i - 1) + 2
    tmp1:set(vertsSW[j1].x, vertsSW[j1].y, vertsSW[j1].z)
    tmp2:set(vertsSW[j1 + 1].x, vertsSW[j1 + 1].y, vertsSW[j1 + 1].z)
    tmp3:set(vertsSW[j1 + 5].x, vertsSW[j1 + 5].y, vertsSW[j1 + 5].z)
    local n2 = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n2:normalize()
    local nCorner = util.slerp(n, n2, 0.5)
    normalsK[nCtr2 + 1] = { x = nCorner.x, y = nCorner.y, z = nCorner.z }

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dxT = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart }
    uvK[uvCtr + 1] = { u = dxT, v = kVStart }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 3] = { u = dxT, v = kVStart + 0.1 }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 2
    uvCtr = uvCtr + 4

    -- Right quad.
    local i1 = (i - 1) + 2

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dy = tmp1:distance(tmp2)
    local dx = tmp1:distance(tmp3)
    uvSW[uvCtr] = { u = 0.0, v = dy }
    uvSW[uvCtr + 1] = { u = dx, v = dy }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy + dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy + dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dx2 = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 1] = { u = dx2, v = kVStart + 0.1 }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.2 - kerbVGap }
    uvK[uvCtr + 3] = { u = dx2, v = kVStart + 0.2 - kerbVGap }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 2, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 2, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 2, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4

    --[[
    -- Bottom quad.
    local i1 = (i - 1) + 3

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dxT = tmp1:distance(tmp3)
    local dyT = tmp1:distance(tmp2)
    uvK[uvCtr] = { u = 0.0, v = kVStart }
    uvK[uvCtr + 1] = { u = dxT, v = kVStart }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + dyT }
    uvK[uvCtr + 3] = { u = dxT, v = kVStart + dyT }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4
    --]]

    -- Left quad.
    local i1 = (i - 1) + 4

    tmp1:set(vertsSW[i1].x, vertsSW[i1].y, vertsSW[i1].z)
    tmp2:set(vertsSW[i1 + 1].x, vertsSW[i1 + 1].y, vertsSW[i1 + 1].z)
    tmp3:set(vertsSW[i1 + 5].x, vertsSW[i1 + 5].y, vertsSW[i1 + 5].z)
    local n = -(tmp2 - tmp1):cross(tmp3 - tmp1)
    n:normalize()

    normalsSW[nCtr] = { x = n.x, y = n.y, z = n.z }
    normalsK[nCtr2] = { x = n.x, y = n.y, z = n.z }

    local dx = tmp1:distance(tmp3)
    local dy = tmp1:distance(tmp2)
    uvSW[uvCtr] = { u = 0.0, v = 0.0 }
    uvSW[uvCtr + 1] = { u = dx, v = 0.0 }
    uvSW[uvCtr + 2] = { u = 0.0, v = dy }
    uvSW[uvCtr + 3] = { u = dx, v = dy }

    tmp1:set(vertsK[i1].x, vertsK[i1].y, vertsK[i1].z)
    tmp2:set(vertsK[i1 + 1].x, vertsK[i1 + 1].y, vertsK[i1 + 1].z)
    tmp3:set(vertsK[i1 + 5].x, vertsK[i1 + 5].y, vertsK[i1 + 5].z)
    local dx2 = tmp1:distance(tmp3)
    uvK[uvCtr] = { u = 0.0, v = kVStart + 0.1 }
    uvK[uvCtr + 1] = { u = dx2, v = kVStart + 0.1 }
    uvK[uvCtr + 2] = { u = 0.0, v = kVStart + 0.2 - kerbVGap }
    uvK[uvCtr + 3] = { u = dx2, v = kVStart + 0.2 - kerbVGap }

    local i1 = i1 - 1
    facesSW[ctr] = { v = i1, n = nCtr - 1, u = uvCtr - 1 }
    facesSW[ctr + 1] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 2] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }
    facesSW[ctr + 3] = { v = i1 + 1, n = nCtr - 1, u = uvCtr + 1 }
    facesSW[ctr + 4] = { v = i1 + 6, n = nCtr - 1, u = uvCtr + 2 }
    facesSW[ctr + 5] = { v = i1 + 5, n = nCtr - 1, u = uvCtr }

    facesK[ctr] = { v = i1, n = nCtr2 - 1, u = uvCtr - 1 }
    facesK[ctr + 1] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 2] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }
    facesK[ctr + 3] = { v = i1 + 1, n = nCtr2 - 1, u = uvCtr + 1 }
    facesK[ctr + 4] = { v = i1 + 6, n = nCtr2 - 1, u = uvCtr + 2 }
    facesK[ctr + 5] = { v = i1 + 5, n = nCtr2 - 1, u = uvCtr }

    ctr = ctr + 6
    nCtr = nCtr + 1
    nCtr2 = nCtr2 + 1
    uvCtr = uvCtr + 4
  end

  local surfaceMeshSW = { verts = vertsSW, faces = facesSW, normals = normalsSW, uvs = uvSW, material = materials.sidewalk }
  local surfaceMeshK = { verts = vertsK, faces = facesK, normals = normalsK, uvs = uvK, material = materials.curbMaterial }

  -- End caps.
  local last = #vertsSW
  local vertsSW2 = { vertsSW[1], vertsSW[2], vertsSW[3], vertsSW[4], vertsSW[last - 3], vertsSW[last - 2], vertsSW[last - 1], vertsSW[last] }
  local vertsK2 = { vertsK[1], vertsK[2], vertsK[3], vertsK[4], vertsK[last - 3], vertsK[last - 2], vertsK[last - 1], vertsK[last] }

  local nStart = rData[1][lIdx][5]:cross(rData[1][lIdx][6])
  local nEnd = rData[#rData][lIdx][5]:cross(rData[#rData - 1][lIdx][6])
  local normals = {
    { x = nStart.x, y = nStart.y, z = nStart.z },
    { x = nEnd.x, y = nEnd.y, z = nEnd.z } }

  local faces = {
    { v = 0, n = 0, u = 0 },
    { v = 3, n = 0, u = 3 },
    { v = 1, n = 0, u = 1 },

    { v = 3, n = 0, u = 3 },
    { v = 2, n = 0, u = 2 },
    { v = 1, n = 0, u = 1 },

    { v = 4, n = 1, u = 0 },
    { v = 5, n = 1, u = 1 },
    { v = 7, n = 1, u = 3 },

    { v = 7, n = 1, u = 3 },
    { v = 5, n = 1, u = 1 },
    { v = 6, n = 1, u = 2 } }

  local endCapsSW = { verts = vertsSW2, faces = faces, normals = normals, uvs = uvs, material = materials.sidewalk }
  local endCapsK = { verts = vertsK2, faces = faces, normals = normals, uvs = uvs, material = materials.curbMaterial }

  -- Generate the procedural meshes for the sidewalk.
  local mesh = createObject('ProceduralMesh')
  mesh:registerObject('Sidewalk_' .. tostring(roadMeshIdx) .. '_' .. tostring(s))
  mesh.canSave = true
  folder:addObject(mesh.obj)
  mesh:createMesh({ { surfaceMeshSW, endCapsSW } })
  mesh:setPosition(origin)
  mesh.scale = scaleVec
  meshes[r.name][#meshes[r.name] + 1] = mesh

  local mesh = createObject('ProceduralMesh')
  mesh:registerObject('Curb_' .. tostring(roadMeshIdx) .. '_' .. tostring(s))
  mesh.canSave = true
  folder:addObject(mesh.obj)
  mesh:createMesh({ { surfaceMeshK, endCapsK } })
  mesh:setPosition(origin)
  mesh.scale = scaleVec
  meshes[r.name][#meshes[r.name] + 1] = mesh
end

-- Creates a road mesh from the given rendering data.
-- [The index of the road is used outside this module to reference the mesh later].
local function createRoad(r, roadMeshIdx, folder)
  local sections = computeSections(r)
  meshes[r.name] = {}
  for s = 1, #sections do
    local sec = sections[s]
    local ty = sec.type
    if ty == 'sidewalk' then
      if r.profile[sec.min].isLeftSide[0] then
        createLeftSidewalkKerb(r, sec, roadMeshIdx, s, folder)
      else
        createRightSidewalkKerb(r, sec, roadMeshIdx, s, folder)
      end
    end
  end
end

-- Attempts to removes the meshes of the road with the given name, from the scene (if it exists).
local function tryRemove(roadName)
  local mesh = meshes[roadName]
  if mesh then
    for s = 1, #mesh do
      mesh[s]:delete()
    end
  end
  meshes[roadName] = nil
end

-- Updates the given bridge.
local function updateBridge(r, folder)

  -- First, remove the old bridge (if it exists).
  local mesh = bridges[r.name]
  if mesh then
    mesh:delete()
    mesh = nil
  end

  -- Now create the new bridge.
  local material = materials.defaultMaterial
  local lMin, lMax = -1, 1

  -- Compute the ring indices.
  local rData = r.renderData
  local last = #rData
  local ringRaw, ctr = {}, 1
  for i = lMin, lMax do                                                                             -- Add top face vertices, clockwise around lanes.
    if rData[last][i] then
      ringRaw[ctr] = { idx1 = i, idx2 = 1 }
      ringRaw[ctr + 1] = { idx1 = i, idx2 = 2 }
      ctr = ctr + 2
    end
  end
  for i = lMax, lMin, -1 do                                                                         -- Add bottom face vertices, clockwise around lanes.
    if rData[last][i] then
      ringRaw[ctr] = { idx1 = i, idx2 = 3 }
      ringRaw[ctr + 1] = { idx1 = i, idx2 = 4 }
      ctr = ctr + 2
    end
  end

  local ring, ctr = { ringRaw[1] }, 2                                                               -- Remove duplicate points from ring.
  for i = 2, #ringRaw do
    local r1, r2 = ringRaw[i - 1], ringRaw[i]
    local p1, p2 = rData[last][r1.idx1][r1.idx2], rData[last][r2.idx1][r2.idx2]
    if p1:squaredDistance(p2) > 0.01 then
      ring[ctr] = ringRaw[i]
      ctr = ctr + 1
    end
  end

  -- Compute the vertices.
  local depth = r.bridgeDepth[0]
  local n1, n2 = r.nodes[1].p, r.nodes[2].p
  local h = depth + r.bridgeArch[0]
  local start = rData[1][1][1]
  local rLenInv = 1.0 / (n1:distance(n2))
  local verts, ctr, ringLen = {}, 1, #ring
  for i = 1, #rData do
    local div = rData[i]
    local q = div[1][1]:distance(start) * rLenInv
    local dZ = -4 * (h + depth) * q * q + 4 * (h + depth) * q - depth
    for k = 1, ringLen do
      local dRing = ring[k]
      local p = div[dRing.idx1][dRing.idx2]
      verts[ctr] = { x = p.x, y = p.y, z = p.z + dZ }
      ctr = ctr + 1
    end
    local dRing = ring[1]                                                                     -- Repeat the first ring point at the end of the ring.
    local p = div[dRing.idx1][dRing.idx2]
    verts[ctr] = { x = p.x, y = p.y, z = p.z + dZ }
    ctr = ctr + 1
  end

  -- Compute the faces and normals.
  local faces, normals, uvSurf, fCtr, nCtr, uCtr = {}, {}, {}, 1, 1, 1
  local ringLenPlus1 = ringLen + 1
  for i = 2, #rData do
    local rowB = (i - 2) * ringLenPlus1
    local rowF = (i - 1) * ringLenPlus1
    for j = 2, ringLenPlus1 do
      local jMinus1 = j - 1
      local i1, i2, i3, i4 = rowB + jMinus1, rowB + j, rowF + jMinus1, rowF + j
      local vv1, vv2, vv3 = verts[i1], verts[i2], verts[i3]
      tmp1:set(vv1.x, vv1.y, vv1.z)
      tmp2:set(vv2.x, vv2.y, vv2.z)
      tmp3:set(vv3.x, vv3.y, vv3.z)
      local edge1, edge2 = tmp2 - tmp1, tmp3 - tmp1
      local dx, dy = edge1:length(), edge2:length()
      local n = -edge1:cross(edge2)
      n:normalize()
      normals[nCtr] = { x = n.x, y = n.y, z = n.z }

      uvSurf[uCtr] = { u = 0, v = 0 }
      uvSurf[uCtr + 1] = { u = dx, v = 0 }
      uvSurf[uCtr + 2] = { u = 0, v = dy }
      uvSurf[uCtr + 3] = { u = dx, v = dy }

      faces[fCtr] = { v = i1 - 1, n = nCtr - 1, u = uCtr - 1 }
      faces[fCtr + 1] = { v = i2 - 1, n = nCtr - 1, u = uCtr }
      faces[fCtr + 2] = { v = i3 - 1, n = nCtr - 1, u = uCtr + 1 }

      faces[fCtr + 3] = { v = i2 - 1, n = nCtr - 1, u = uCtr }
      faces[fCtr + 4] = { v = i4 - 1, n = nCtr - 1, u = uCtr + 2 }
      faces[fCtr + 5] = { v = i3 - 1, n = nCtr - 1, u = uCtr + 1 }
      fCtr = fCtr + 6
      nCtr = nCtr + 1
      uCtr = uCtr + 4
    end
  end
  local surfaceMesh = { verts = verts, faces = faces, normals = normals, uvs = uvSurf, material = material }

  -- Add end caps.
  local vertsLane, facesLane, normalsLane, uvLane, vCtr, fCtr, nCtr, uCtr = {}, {}, {}, {}, 1, 1, 1, 1
  local numDivs = #rData
  for k = lMin, lMax do
    if k ~= 0 then
      local b, f = rData[1][k], rData[numDivs][k]

      -- Append the vertices.
      local b1, b2, b3, b4, f1, f2, f3, f4 = b[1], b[2], b[3], b[4], f[1], f[2], f[3], f[4]
      local i1, i2, i3, i4, i5, i6, i7, i8 = vCtr, vCtr + 1, vCtr + 2, vCtr + 3, vCtr + 4, vCtr + 5, vCtr + 6, vCtr + 7
      vertsLane[i1] = { x = b1.x, y = b1.y, z = b1.z - depth }
      vertsLane[i2] = { x = b2.x, y = b2.y, z = b2.z - depth }
      vertsLane[i3] = { x = b3.x, y = b3.y, z = b3.z - depth }
      vertsLane[i4] = { x = b4.x, y = b4.y, z = b4.z - depth }
      vertsLane[i5] = { x = f1.x, y = f1.y, z = f1.z - depth }
      vertsLane[i6] = { x = f2.x, y = f2.y, z = f2.z - depth }
      vertsLane[i7] = { x = f3.x, y = f3.y, z = f3.z - depth }
      vertsLane[i8] = { x = f4.x, y = f4.y, z = f4.z - depth }

      -- Append the normals.
      local n, l = b[5], b[6]
      local t = n:cross(l)
      local n1, n2 = nCtr, nCtr + 1
      normalsLane[n1] = { x = t.x, y = t.y, z = t.z }
      normalsLane[n2] = { x = -t.x, y = -t.y, z = -t.z }

      local dxB, dyB = (b2 - b1):length(), (b4 - b1):length()
      local dxF, dyF = (f2 - f1):length(), (f4 - f1):length()
      uvLane[uCtr] = { u = 0, v = 0 }
      uvLane[uCtr + 1] = { u = dxB, v = 0 }
      uvLane[uCtr + 2] = { u = 0, v = dyB }
      uvLane[uCtr + 3] = { u = dxB, v = dyB }

      uvLane[uCtr + 4] = { u = 0, v = 0 }
      uvLane[uCtr + 5] = { u = dxF, v = 0 }
      uvLane[uCtr + 6] = { u = 0, v = dyF }
      uvLane[uCtr + 7] = { u = dxF, v = dyF }

      -- Reduce the vertex and normal indices by one, since face indexing starts at zero.
      i1, i2, i3, i4, i5, i6, i7, i8 = i1 - 1, i2 - 1, i3 - 1, i4 - 1, i5 - 1, i6 - 1, i7 - 1, i8 - 1
      n1, n2 = n1 - 1, n2 - 1

      -- Append the front cap faces [f1 - f3 - f2, f3 - f1 - f4].
      facesLane[fCtr] = { v = i1, n = n1, u = uCtr + 3 }
      facesLane[fCtr + 1] = { v = i3 , n = n1, u = uCtr + 4 }
      facesLane[fCtr + 2] = { v = i2, n = n1, u = uCtr + 6 }
      facesLane[fCtr + 3] = { v = i3, n = n1, u = uCtr + 4 }
      facesLane[fCtr + 4] = { v = i1, n = n1, u = uCtr + 3 }
      facesLane[fCtr + 5] = { v = i4, n = n1, u = uCtr + 5 }

      -- Append the back cap faces [b1 - b2 - b3, b3 - b4 - b1].
      facesLane[fCtr + 6] = { v = i5, n = n2, u = uCtr + 1 }
      facesLane[fCtr + 7] = { v = i6 , n = n2, u = uCtr }
      facesLane[fCtr + 8] = { v = i7, n = n2, u = uCtr + 2 }
      facesLane[fCtr + 9] = { v = i7, n = n2, u = uCtr }
      facesLane[fCtr + 10] = { v = i8, n = n2, u = uCtr + 1 }
      facesLane[fCtr + 11] = { v = i5, n = n2, u = uCtr - 1 }

      vCtr = vCtr + 8
      fCtr = fCtr + 12
      nCtr = nCtr + 2
      uCtr = uCtr + 8
    end
  end
  local endCaps = { verts = vertsLane, faces = facesLane, normals = normalsLane, uvs = uvLane, material = material }

  -- Generate the procedural meshes for the road.
  local mesh = createObject('ProceduralMesh')
  mesh:registerObject('Bridge_' .. tostring(r.name))
  mesh.canSave = true
  folder:addObject(mesh.obj)
  mesh:createMesh({ { surfaceMesh, endCaps } })
  mesh:setPosition(origin)
  mesh.scale = scaleVec

  bridges[r.name] = mesh
end

-- Attempts to remove the bridge with the given index.
local function tryRemoveBridge(name)
  local mesh = bridges[name]
  if mesh then
    bridges[name]:delete()
    bridges[name] = nil
  end
end

-- Clears the bridges structure.
local function clearBridges() bridges = {} end


-- Public interface.
M.createRoad =                                            createRoad
M.tryRemove =                                             tryRemove

M.updateBridge =                                          updateBridge
M.tryRemoveBridge =                                       tryRemoveBridge
M.clearBridges =                                          clearBridges

return M