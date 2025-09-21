-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Vector Light State
local al_VectorLightState = scenetree.findObject("AL_VectorLightState")
if not al_VectorLightState then
  al_VectorLightState = createObject("GFXStateBlockData")
  al_VectorLightState.blendDefined = true
  al_VectorLightState.blendEnable = true
  al_VectorLightState:setField("blendSrc", 0, "GFXBlendOne")
  al_VectorLightState:setField("blendDest", 0, "GFXBlendOne")
  al_VectorLightState:setField("blendOp", 0, "GFXBlendOpAdd")
  al_VectorLightState.zDefined = true
  al_VectorLightState.zEnable = false
  al_VectorLightState.zWriteEnable = false
  al_VectorLightState.samplersDefined = true
  al_VectorLightState:setField("samplerStates", 0, "SamplerClampPoint") -- G-buffer
  al_VectorLightState:setField("samplerStates", 1, "SamplerClampPoint") -- G-buffer
  al_VectorLightState:setField("samplerStates", 2, "SamplerClampPoint") -- Shadow Map (Do not change this to linear, as all cards can not filter equally.)
  al_VectorLightState:setField("samplerStates", 3, "SamplerClampLinear") -- SSAO Mask
  al_VectorLightState:setField("samplerStates", 4, "SamplerWrapPoint") -- Random Direction Map
  al_VectorLightState:setField("samplerStates", 5, "SamplerClampPoint") -- G-buffer
  al_VectorLightState:setField("samplerStates", 6, "SamplerClampPoint") -- G-buffer
  al_VectorLightState.cullDefined = true
  al_VectorLightState:setField("cullMode", 0, "GFXCullNone")
  al_VectorLightState.stencilDefined = true
  al_VectorLightState.stencilEnable = true
  al_VectorLightState:setField("stencilFailOp", 0, "GFXStencilOpKeep")
  al_VectorLightState:setField("stencilZFailOp", 0, "GFXStencilOpKeep")
  al_VectorLightState:setField("stencilPassOp", 0, "GFXStencilOpKeep")
  al_VectorLightState:setField("stencilFunc", 0, "GFXCmpLess")
  al_VectorLightState.stencilRef = 4
  al_VectorLightState:registerObject("AL_VectorLightState")
end

-- Vector Light Material
local al_VectorLightShader = scenetree.findObject("AL_VectorLightShader")
if not al_VectorLightShader then
  al_VectorLightShader = createObject("ShaderData")
  al_VectorLightShader.DXVertexShaderFile    = "shaders/common/lighting/advanced/farFrustumQuadV.hlsl"
  al_VectorLightShader.DXPixelShaderFile     = "shaders/common/lighting/advanced/vectorLightP.hlsl"
  al_VectorLightShader.pixVersion = 5.0
  al_VectorLightShader:registerObject("AL_VectorLightShader")
end

local al_VectorLightMaterial = scenetree.findObject("AL_VectorLightMaterial")
if not al_VectorLightMaterial then
  al_VectorLightMaterial = createObject("CustomMaterial")
  al_VectorLightMaterial:setField("shader", 0, "AL_VectorLightShader")
  al_VectorLightMaterial:setField("stateBlock", 0, "AL_VectorLightState")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer", "#prepass[RT0]")
  al_VectorLightMaterial:setField("sampler", "prePassDepthBuffer", "#prepass[Depth]")
  al_VectorLightMaterial:setField("sampler", "ShadowMap", "$dynamiclight")
  al_VectorLightMaterial:setField("sampler", "ssaoMask", "#ssaoMask")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer1", "#prepass[RT1]")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer2", "#prepass[RT2]")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer3", "#prepass[RT3]")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer4", "#prepass[RT4]")
  al_VectorLightMaterial:setField("sampler", "prePassBuffer5", "#prepass[RT5]")
  al_VectorLightMaterial:setField("target", 0, "lightinfo")
  al_VectorLightMaterial.pixVersion = 5.0
  al_VectorLightMaterial:registerObject("AL_VectorLightMaterial")
end
------------------------------------------------------------------------------

-- Convex-geometry light states
local al_ConvexLightState = scenetree.findObject("AL_ConvexLightState")
if not al_ConvexLightState then
  al_ConvexLightState = createObject("GFXStateBlockData")
  al_ConvexLightState.blendDefined = true;
  al_ConvexLightState.blendEnable = true;
  al_ConvexLightState:setField("blendSrc", 0, "GFXBlendOne")
  al_ConvexLightState:setField("blendDest", 0, "GFXBlendOne")
  al_ConvexLightState:setField("blendOp", 0, "GFXBlendOpAdd")
  al_ConvexLightState.zDefined = true;
  al_ConvexLightState.zEnable = true;
  al_ConvexLightState.zWriteEnable = false;
  local useReversedDepthBuffer = TorqueScriptLua.getBoolVar("$Scene::useReversedDepthBuffer")
  if useReversedDepthBuffer then
    al_ConvexLightState:setField("zFunc", 0, "GFXCmpLessEqual")
  else
    al_ConvexLightState:setField("zFunc", 0, "GFXCmpGreaterEqual")
  end
  al_ConvexLightState.samplersDefined = true;
  al_ConvexLightState:setField("samplerStates", 0, "SamplerClampPoint") -- G-buffer
  al_ConvexLightState:setField("samplerStates", 1, "SamplerClampPoint") -- G-buffer
  al_ConvexLightState:setField("samplerStates", 2, "SamplerClampPoint") -- Shadow Map (Do not use linear, these are perspective projections)
  al_ConvexLightState:setField("samplerStates", 3, "SamplerClampLinear") -- Cookie Map
  al_ConvexLightState:setField("samplerStates", 4, "SamplerWrapPoint") -- Random Direction Map
  al_ConvexLightState:setField("samplerStates", 5, "SamplerClampPoint") -- G-buffer
  al_ConvexLightState:setField("samplerStates", 6, "SamplerClampPoint") -- G-buffer
  al_ConvexLightState.cullDefined = true
  al_ConvexLightState:setField("cullMode", 0, "GFXCullCW")
  al_ConvexLightState.stencilDefined = true
  al_ConvexLightState.stencilEnable = true
  al_ConvexLightState:setField("stencilFailOp", 0, "GFXStencilOpKeep")
  al_ConvexLightState:setField("stencilZFailOp", 0, "GFXStencilOpKeep")
  al_ConvexLightState:setField("stencilPassOp", 0, "GFXStencilOpKeep")
  al_ConvexLightState:setField("stencilFunc", 0, "GFXCmpLess")
  al_ConvexLightState.stencilRef = 4
  al_ConvexLightState:registerObject("AL_ConvexLightState")
end

-- Point Light Material
local al_PointLightShader = scenetree.findObject("AL_PointLightShader")
if not al_PointLightShader then
  al_PointLightShader = createObject("ShaderData")
  al_PointLightShader.DXVertexShaderFile = "shaders/common/lighting/advanced/convexGeometryV.hlsl"
  al_PointLightShader.DXPixelShaderFile  = "shaders/common/lighting/advanced/pointLightP.hlsl"
  al_PointLightShader.pixVersion = 5.0;
  al_PointLightShader:registerObject("AL_PointLightShader")
end

local al_PointLightMaterial = scenetree.findObject("AL_PointLightMaterial")
if not al_PointLightMaterial then
  al_PointLightMaterial = createObject("CustomMaterial")
  al_PointLightMaterial:setField("shader", 0, "AL_PointLightShader")
  al_PointLightMaterial:setField("stateBlock", 0, "AL_ConvexLightState")
  al_PointLightMaterial:setField("sampler", "prePassBuffer", "#prepass[RT0]")
  al_PointLightMaterial:setField("sampler", "prePassDepthBuffer", "#prepass[Depth]")
  al_PointLightMaterial:setField("sampler", "shadowMap", "$dynamiclight")
  al_PointLightMaterial:setField("sampler", "cookieTex", "$dynamiclightmask")
  al_PointLightMaterial:setField("sampler", "prePassBuffer1", "#prepass[RT1]")
  al_PointLightMaterial:setField("sampler", "prePassBuffer2", "#prepass[RT2]")
  al_PointLightMaterial:setField("target", 0, "lightinfo")
  al_PointLightMaterial.pixVersion = 5.0
  al_PointLightMaterial:registerObject("AL_PointLightMaterial")
end

-- Spot Light Material
local al_SpotLightShader = scenetree.findObject("AL_SpotLightShader")
  if not al_SpotLightShader then
  al_SpotLightShader = createObject("ShaderData")
  al_SpotLightShader.DXVertexShaderFile = "shaders/common/lighting/advanced/convexGeometryV.hlsl"
  al_SpotLightShader.DXPixelShaderFile  = "shaders/common/lighting/advanced/spotLightP.hlsl"
  al_SpotLightShader.pixVersion = 5.0;
  al_SpotLightShader:registerObject("AL_SpotLightShader")
end

local al_SpotLightMaterial = scenetree.findObject("AL_SpotLightMaterial")
if not al_SpotLightMaterial then
  al_SpotLightMaterial = createObject("CustomMaterial")
  al_SpotLightMaterial:setField("shader", 0, "AL_SpotLightShader")
  al_SpotLightMaterial:setField("stateBlock", 0, "AL_ConvexLightState")
  al_SpotLightMaterial:setField("sampler", "prePassBuffer", "#prepass[RT0]")
  al_SpotLightMaterial:setField("sampler", "prePassDepthBuffer", "#prepass[Depth]")
  al_SpotLightMaterial:setField("sampler", "shadowMap", "$dynamiclight")
  al_SpotLightMaterial:setField("sampler", "cookieTex", "$dynamiclightmask")
  al_SpotLightMaterial:setField("sampler", "prePassBuffer1", "#prepass[RT1]")
  al_SpotLightMaterial:setField("sampler", "prePassBuffer2", "#prepass[RT2]")
  al_SpotLightMaterial:setField("target", 0, "lightinfo")
  al_SpotLightMaterial.pixVersion = 5.0
  al_SpotLightMaterial:registerObject("AL_SpotLightMaterial")
end

-- This material is used for generating prepass
-- materials for objects that do not have materials.
local al_DefaultPrePassMaterial = scenetree.findObject("AL_DefaultPrePassMaterial")
if not al_DefaultPrePassMaterial then
  al_DefaultPrePassMaterial = createObject("Material")
  -- We need something in the first pass else it
  -- won't create a proper material instance.
  --
  -- We use color here because some objects may not
  -- have texture coords in their vertex format...
  -- for example like terrain.
  al_DefaultPrePassMaterial:setField("diffuseColor", 0, "1 1 1 1")
  al_DefaultPrePassMaterial:registerObject("AL_DefaultPrePassMaterial")
end

-- This material is used for generating shadow
-- materials for objects that do not have materials.
local al_DefaultShadowMaterial = scenetree.findObject("AL_DefaultShadowMaterial")
if not al_DefaultShadowMaterial then
  al_DefaultShadowMaterial = createObject("Material")
  -- We need something in the first pass else it
  -- won't create a proper material instance.
  --
  -- We use color here because some objects may not
  -- have texture coords in their vertex format...
  -- for example like terrain.
  --
  al_DefaultShadowMaterial:setField("diffuseColor", 0, "1 1 1 1")
   -- This is here mostly for terrain which uses
   -- this material to create its shadow material.
   --
   -- At sunset/sunrise the sun is looking thru
   -- backsides of the terrain which often are not
   -- closed.  By changing the material to be double
   -- sided we avoid holes in the shadowed geometry.
   --
  al_DefaultShadowMaterial.doubleSided = true
  al_DefaultShadowMaterial:registerObject("AL_DefaultShadowMaterial")
end

-- Particle System Point Light Material
local al_ParticlePointLightShader = scenetree.findObject("AL_ParticlePointLightShader")
if not al_ParticlePointLightShader then
  al_ParticlePointLightShader = createObject("ShaderData")
  al_ParticlePointLightShader.DXVertexShaderFile = "shaders/common/lighting/advanced/particlePointLightV.hlsl"
  al_ParticlePointLightShader.DXPixelShaderFile  = "shaders/common/lighting/advanced/particlePointLightP.hlsl"
  al_ParticlePointLightShader.pixVersion = 5.0;
  al_ParticlePointLightShader:registerObject("AL_ParticlePointLightShader")
end

local al_ParticlePointLightMaterial = scenetree.findObject("AL_ParticlePointLightMaterial")
if not al_ParticlePointLightMaterial then
  al_ParticlePointLightMaterial = createObject("CustomMaterial")
  al_ParticlePointLightMaterial:setField("shader", 0, "AL_ParticlePointLightShader")
  al_ParticlePointLightMaterial:setField("stateBlock", 0, "AL_ConvexLightState")
  al_ParticlePointLightMaterial:setField("sampler", "prePassBuffer", "#prepass[RT0]")
  al_ParticlePointLightMaterial:setField("sampler", "prePassDepthBuffer", "#prepass[Depth]")
  al_ParticlePointLightMaterial:setField("target", 0, "lightinfo")
  al_ParticlePointLightMaterial:setField("sampler", "prePassBuffer1", "#prepass[RT1]")
  al_ParticlePointLightMaterial:setField("sampler", "prePassBuffer2", "#prepass[RT2]")
  al_ParticlePointLightMaterial.pixVersion = 5.0
  al_ParticlePointLightMaterial:registerObject("AL_ParticlePointLightMaterial")
end