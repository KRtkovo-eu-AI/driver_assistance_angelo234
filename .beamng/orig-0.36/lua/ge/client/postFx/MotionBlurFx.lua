-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local pfxMotionBlurShader = scenetree.findObject("PFX_MotionBlurShader")
if not pfxMotionBlurShader then
  pfxMotionBlurShader = createObject("ShaderData")
  pfxMotionBlurShader.DXVertexShaderFile    = "shaders/common/postFx/motionBlurP.hlsl";
  pfxMotionBlurShader.DXPixelShaderFile     = "shaders/common/postFx/motionBlurP.hlsl";
  pfxMotionBlurShader.pixVersion = 5.0
  pfxMotionBlurShader:registerObject("PFX_MotionBlurShader")
end

local motionBlurFXCallbacks = {}
motionBlurFXCallbacks.setShaderConsts = function()
  local motionBlurFX = scenetree.findObject("MotionBlurFX")
  if motionBlurFX then
    motionBlurFX:setShaderConst( "$velocityMultiplier", 3000 )
  end
end
rawset(_G, "MotionBlurFXCallbacks", motionBlurFXCallbacks)

local motionBlurFX = scenetree.findObject("MotionBlurFX")
if not motionBlurFX then
  motionBlurFX = createObject("PostEffect")
  motionBlurFX.isEnabled = false
  motionBlurFX:setField("renderTime", 0, "PFXAfterDiffuse")
  motionBlurFX:setField("shader", 0, "PFX_MotionBlurShader")
  motionBlurFX:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  motionBlurFX:setField("texture", 0, "$backbuffer")
  motionBlurFX:setField("texture", 1, "#prepass[RT0]")
  motionBlurFX:setField("texture", 2, "#prepass[Depth]")
  motionBlurFX:setField("target", 0, "$backBuffer")
  motionBlurFX:registerObject("MotionBlurFX")
end