-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local depthCoef = 0.5 / physicsDt

local materials, materialsMap = particles.getMaterialsParticlesTable()

M.particleData = {
  id1 = 0,
  pos = 0,
  normal = 0,
  nodeVel = 0,
  perpendicularVel = 0,
  slipVec = 0,
  slipVel = 0,
  slipForce = 0,
  normalForce = 0,
  depth = 0,
  materialID1 = 0,
  materialID2 = 0
}

local function nodeCollision(p)
  --log('D', "particlefilter.particleEmitted", "particleEmitted()")

  --log('D', "particlefilter.particleEmitted", p.materialID1..", "..p.materialID2)
  if p.perpendicularVel > p.depth * depthCoef then
    local pKey = p.materialID1 * 10000 + p.materialID2
    local mmap = materialsMap[pKey]
    if mmap ~= nil then
      for _, r in pairs(mmap) do
        if r.compareFunc(p) then
          --log('D', "particlefilter.particleEmitted", "spawned particle type " .. tostring(p.particleType))
          obj:addParticleVelWidthTypeCount(p.id1, p.normal, p.nodeVel, r.veloMult, r.width, r.particleType, r.count)
        end
      end
    end
  end
end

-- public interface
M.nodeCollision = nodeCollision

return M
