-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

-- called when this object is created. initialize variables here (but dont spawn objects)
function C:init()

end

-- called every frame to update the visuals.
function C:update(data)

end


function C:setup(cluster)
  self.cluster = cluster
end

-- creates neccesary objects
function C:createObjects()

end

function C:setHidden(value)
end

function C:hide()
end

function C:show()
end


-- destorys/cleans up all objects created by this
function C:clearObjects()

end

function C:instantFade(visible)
end


-- Interactivity
function C:interactInPlayMode(interactData, interactableElements)

end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end