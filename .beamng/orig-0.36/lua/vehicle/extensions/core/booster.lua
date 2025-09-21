-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local boosts = {}

local function updateGFX(dtSim)
  local planets = {}

  for _, boost in ipairs(boosts) do
    local boostVec = boost[1] * 1000

    local planetPos = obj:getPosition() - boostVec

    -- x,y,z, radius, mass
    table.insert(planets, {planetPos.x, planetPos.y, planetPos.z, 10, -1e18})

    local ttl = boost[2]
    ttl = ttl - dtSim
    if ttl <= 0 then
      table.remove(boosts, i)
    else
      boost[2] = ttl
    end
  end

  if #planets > 0 then
    print("obj:setPlanets(" .. dumps(planets) .. ")")
  end
  obj:setPlanets(planets)
end

local function boost(force, dt)
  table.insert(boosts, {vec3(force), dt})
end

-- public interface
M.boost = boost
M.updateGFX = updateGFX

return M
