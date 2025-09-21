-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local distCoeffecient =  0.0
local cubeDistortionFactor = -0.25
local colorDistortionFactor = "-0.0025 0.0 0.0025"

local pfxDefaultChromaticLensStateBlock = scenetree.findObject("PFX_DefaultChromaticLensStateBlock")
if not pfxDefaultChromaticLensStateBlock then
  pfxDefaultChromaticLensStateBlock = createObject("GFXStateBlockData")
  pfxDefaultChromaticLensStateBlock.zDefined = true;
  pfxDefaultChromaticLensStateBlock.zEnable = false;
  pfxDefaultChromaticLensStateBlock.zWriteEnable = false;
  pfxDefaultChromaticLensStateBlock.samplersDefined = false;
  pfxDefaultChromaticLensStateBlock:setField("samplerStates", 0, "SamplerClampPoint")
  pfxDefaultChromaticLensStateBlock:registerObject("PFX_DefaultChromaticLensStateBlock")
end

local pfxChromaticLensShader = scenetree.findObject("PFX_ChromaticLensShader")
if not pfxChromaticLensShader then
  pfxChromaticLensShader = createObject("ShaderData")
  pfxChromaticLensShader.DXVertexShaderFile = "shaders/common/postFx/chromaticLens.hlsl"
  pfxChromaticLensShader.DXPixelShaderFile  = "shaders/common/postFx/chromaticLens.hlsl"
  pfxChromaticLensShader.pixVersion = 5.0;
  pfxChromaticLensShader:registerObject("PFX_ChromaticLensShader")
end

local chromaticLensPostFXCallbacks = {}
chromaticLensPostFXCallbacks.setShaderConsts = function ()
  log('I','postfx','Calling setShaderConsts from chromaticLensPostFX')
  local chromaticLensPostFX = scenetree.findObject("ChromaticLensPostFX")
  if chromaticLensPostFX then
    chromaticLensPostFX:setShaderConst("$distCoeff", distCoeffecient );
    chromaticLensPostFX:setShaderConst("$cubeDistort", cubeDistortionFactor );
    chromaticLensPostFX:setShaderConst("$colorDistort", colorDistortionFactor );
  end
end
rawset(_G, "ChromaticLensPostFXCallbacks", chromaticLensPostFXCallbacks)

local chromaticLensPostFX = scenetree.findObject("ChromaticLensPostFX")
if not chromaticLensPostFX then
  chromaticLensPostFX = createObject("PostEffect")
  chromaticLensPostFX:setField("renderTime", 0, "PFXAfterDiffuse")
  chromaticLensPostFX.renderPriority = 100
  chromaticLensPostFX.isEnabled = false
  chromaticLensPostFX.allowReflectPass = false
  chromaticLensPostFX:setField("shader", 0, "PFX_ChromaticLensShader")
  chromaticLensPostFX:setField("stateBlock", 0, "PFX_DefaultChromaticLensStateBlock")
  chromaticLensPostFX:setField("texture", 0, "$backBuffer")
  chromaticLensPostFX:setField("target", 0, "$backBuffer")
  chromaticLensPostFX:registerObject("ChromaticLensPostFX")
end

-- function reloadChromaticLens()
-- {
--   exec( "./chromaticLens.cs" );
--   ChromaticLensPostFX.reload();
--   ChromaticLensPostFX.disable();
--   ChromaticLensPostFX.enable();
-- }