Shader "Unlit/BoxCloudsLite"
{
    Properties
    {
        _ShapeNoise("_ShapeNoise",2d)=""{}
        _DetailNoiseTex("_DetailNoiseTex",3d)=""{}

        _DensityMultiplier("_DensityMultiplier",float) = 1
        _DensityOffset("_DensityOffset",float) = -.5
        _CloudScale("_CloudScale",float) = .6

        _DetailNoiseScale("_DetailNoiseScale",float) = 3
        _DetailNoiseWeight("_DetailNoiseWeight",float) = 1
        _DetailWeights("_DetailWeights",vector) = (1,1,1,1)

        _ShapeNoiseWeights("_ShapeNoiseWeights",vector)=(1,.5,.2,0)
        _PhaseParams("_PhaseParams",vector) = (.8,.3,1,.5)

        _NumSteps("_NumSteps",int) = 11
        _NumStepsLight("_NumStepsLight",int) = 8
        _RayOffsetStrength("_RayOffsetStrength",float) = 10

        _DetailSpeed("_DetailSpeed",float) = 1

        _BoundsMin("_BoundsMin",vector) = (-1000,-1000,-1000,0)
        _BoundsMax("_BoundsMax",vector) = (1000,1000,1000,0)

        _LightAbsorptionTowardSun("_LightAbsorptionTowardSun",float) = 1.2
        _LightAbsorptionThroughCloud("_LightAbsorptionThroughCloud",float) = 0.75
        _DarknessThreshold("_DarknessThreshold",float) = 0.15

        _WindDir("_WindDir",vector) = (0.1,0,0,0)
        _DetailWindDir("_DetailWindDir",vector) = (.1,0,0,0)
        _ContainerEdgeFadeDst("_ContainerEdgeFadeDst",float) = 50
        _CloudSmooth("_CloudSmooth",float) = 5
        _DensityThreshold("_DensityThreshold",float) = 0.25
        _Color("_Color",color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100
        // No culling or depth
        Cull Off ZWrite Off ZTest Always
        // blend srcAlpha oneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"
#include "CloudLib.hlsl"
            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv:TEXCOORD;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float3 viewDir:TEXCOORD;
            };


            sampler2D _CameraDepthTexture;
            float3 _BoundsMin, _BoundsMax;
            float _CloudScale, _DetailNoiseScale;
            float3 _Wind, _detailNoiseWind;
            Texture2D<float4> _ShapeNoise;
            Texture3D<float4> _DetailNoise;
            Texture2D<float4> _BlueNoise;
            SamplerState sampler_ShapeNoise;
            SamplerState sampler_DetailNoise;
            SamplerState sampler_BlueNoise;

            float _ContainerEdgeFadeDst;
            float _DetailNoiseWeight;
            float _DensityThreshold;
            float _DensityMultiplier;
            float _LightAbsorptionThroughCloud;
            float4 _PhaseParams;
            int _NumSteps, _NumStepsLight;
            float _LightAbsorptionTowardSun;
            float _DarknessThreshold;
            float _CloudSmooth;
            half4 _Color;
            float _Alpha;
            float _RayOffsetStrength;
            float _RenderDistance;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                float3 worldPos = mul(unity_ObjectToWorld,v.vertex);
                o.viewDir = worldPos - _WorldSpaceCameraPos;

// float3 viewVector = mul(unity_CameraInvProjection, float4(v.uv * 2 - 1, 0, -1));
//                 o.viewDir = mul(unity_CameraToWorld, float4(viewVector,0));
                return o;
            }

            float sampleDensity(float3 pos)
            {
                float3 uvw = pos * _CloudScale * 0.001 + _Wind.xyz * 0.1 * _Time.y * _CloudScale;
                float3 size = _BoundsMax - _BoundsMin;
                float3 boundsCentre = (_BoundsMin+_BoundsMax) * 0.5f;

                float3 duvw = pos * _DetailNoiseScale * 0.001 + _detailNoiseWind.xyz * 0.1 * _Time.y * _DetailNoiseScale;

                float dstFromEdgeX = min(_ContainerEdgeFadeDst, min(pos.x - _BoundsMin.x, _BoundsMax.x - pos.x));
                float dstFromEdgeY = min(_CloudSmooth, min(pos.y - _BoundsMin.y, _BoundsMax.y - pos.y));
                float dstFromEdgeZ = min(_ContainerEdgeFadeDst, min(pos.z - _BoundsMin.z, _BoundsMax.z - pos.z));
                float edgeWeight = min(dstFromEdgeZ,dstFromEdgeX)/_ContainerEdgeFadeDst;

                float4 shape = _ShapeNoise.SampleLevel(sampler_ShapeNoise, uvw.xz, 0);
                float4 detail = _DetailNoise.SampleLevel(sampler_DetailNoise, duvw, 0);
                float density = max(0, lerp(shape.x, detail.x, _DetailNoiseWeight) - _DensityThreshold) * _DensityMultiplier;
                return density * edgeWeight * (dstFromEdgeY/_CloudSmooth);
            }

            // Calculate proportion of light that reaches the given point from the lightsource
            float lightmarch(float3 position) {
                float3 dirToLight = _WorldSpaceLightPos0.xyz;
                float dstInsideBox = rayBoxDst(_BoundsMin, _BoundsMax, position, 1/dirToLight).y;
                
                float stepSize = dstInsideBox/_NumStepsLight;
                float totalDensity = 0;

                for (int step = 0; step < _NumStepsLight; step ++) {
                    position += dirToLight * stepSize;
                    totalDensity += max(0, sampleDensity(position) * stepSize);
                }

                float transmittance = exp(-totalDensity * _LightAbsorptionTowardSun);
                return _DarknessThreshold + transmittance * (1-_DarknessThreshold);
            }

fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                float2 screenUV = i.vertex.xy/_ScreenParams.xy;
                float viewLength = length(i.viewDir);
                float3 rayOrigin = _WorldSpaceCameraPos;
                float3 rayDir = i.viewDir / viewLength;

                //Depth
                float nonlin_depth = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, screenUV);
                float depth = LinearEyeDepth(nonlin_depth) * viewLength;
                float2 rayToContainerInfo = rayBoxDst(_BoundsMin, _BoundsMax, rayOrigin, 1/rayDir);
                float dstToBox = rayToContainerInfo.x;
                float dstInsideBox = rayToContainerInfo.y;
                // if(dstToBox + dstInsideBox > _RenderDistance) return 0;

                // random starting offset (makes low-res results noisy rather than jagged/glitchy, which is nicer)
                float randomOffset = _BlueNoise.SampleLevel(sampler_BlueNoise, squareUV(screenUV *3), 0);
                randomOffset *= _RayOffsetStrength;

                float dstTravelled = randomOffset;
                float stepSize = dstInsideBox / _NumSteps;
                float dstLimit = min(depth - dstToBox, dstInsideBox);

                float3 entryPoint = rayOrigin + rayDir * dstToBox;
                float transmittance = 1;
                float3 lightEnergy = 0;

                // Phase function makes clouds brighter around sun
                float cosAngle = dot(rayDir, _WorldSpaceLightPos0.xyz);
                float phaseVal = phase(cosAngle,_PhaseParams);

                while (dstTravelled < dstLimit) {
                    rayOrigin = entryPoint + rayDir * dstTravelled;
                    float density = sampleDensity(rayOrigin);
                    
                    if (density > 0) {
                        float lightTransmittance = lightmarch(rayOrigin);
                        lightEnergy += density * stepSize * transmittance * lightTransmittance * phaseVal;
                        transmittance *= exp(-density * stepSize * _LightAbsorptionThroughCloud);
                    
                        // Exit early if T is close to zero as further samples won't affect the result much
                        if (transmittance < 0.1) {
                            break;
                        }
                    }
                    dstTravelled += stepSize;
                }
                float3 cloudCol = lightEnergy * _Color;
                return float4(cloudCol,1 - transmittance);
                // float3 col0 = col * transmittance + cloudCol;
                // return float4(lerp(col, col0, smoothstep(_RenderDistance, _RenderDistance * 0.25f, dstToBox + dstInsideBox) * _Alpha), 0);
            }
            ENDCG
        }
    }
}
