-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local blurDepthShader = scenetree.findObject("BlurDepthShader")
if not blurDepthShader then
  blurDepthShader = createObject("ShaderData")
  blurDepthShader.DXVertexShaderFile = "shaders/common/lighting/shadowMap/boxFilterV.hlsl"
  blurDepthShader.DXPixelShaderFile  = "shaders/common/lighting/shadowMap/boxFilterP.hlsl"
  blurDepthShader.pixVersion = 5.0;
  blurDepthShader:registerObject("BlurDepthShader")
end