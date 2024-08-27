Shader "Unlit/BoxCloudsLite"
{
    Properties
    {
        _NoiseTex("_NoiseTex",2d)=""{}
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

            // Textures
            sampler2D _NoiseTex;
            sampler3D _DetailNoiseTex;
            sampler2D _WeatherMap;
            sampler2D _BlueNoise;
            
            sampler2D _MainTex;
            sampler2D _CameraDepthTexture;

            // Shape settings
            float _DensityMultiplier;
            float _DensityOffset;
            float _CloudScale;
            float _DetailNoiseScale;
            float _DetailNoiseWeight;
            float3 _DetailWeights;
            float4 _ShapeNoiseWeights;
            float4 _PhaseParams;

            // March settings
            int _NumSteps;
            int _NumStepsLight;
            float _RayOffsetStrength;

            float3 _BoundsMin;
            float3 _BoundsMax;

            // Light settings
            float _LightAbsorptionTowardSun;
            float _LightAbsorptionThroughCloud;
            float _DarknessThreshold;
            float4 _LightColor0;

            // Animation settings
            float _DetailSpeed;

            //-----
            float3 _WindDir;
            float3 _DetailWindDir;

            float _ContainerEdgeFadeDst;
            float _CloudSmooth;
            float _DensityThreshold;
            float4 _Color;

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

            float sampleDensity(float3 pos){
                float3 uvw = pos * 0.001 + _WindDir.xyz * _Time.y;
                uvw *= _CloudScale;

                float3 size = _BoundsMax - _BoundsMin;
                float3 boundsCenter = (_BoundsMin + _BoundsMax) * 0.5;

                float3 duvw = pos * 0.001 + _DetailWindDir.xyz * _Time.y;
                duvw *= _DetailNoiseScale;

                float3 dstFromEdge = min(float3(_ContainerEdgeFadeDst,_CloudSmooth,_ContainerEdgeFadeDst),min(pos - _BoundsMin,_BoundsMax - pos));
                float edgeWeight = min(dstFromEdge.x,dstFromEdge.z)/_ContainerEdgeFadeDst;

                float4 shape = tex2Dlod(_NoiseTex,float4(uvw.xz,0,0));
                float4 detailShape = tex3Dlod(_DetailNoiseTex,float4(duvw,0));

                float density = max(0, lerp(shape.x,detailShape.x,_DetailNoiseWeight) - _DensityThreshold) * _DensityMultiplier;
                return density * edgeWeight;// * (dstFromEdge.y/_CloudSmooth);
            }

            float lightmarch(float3 pos){
                float3 lightDir = _WorldSpaceLightPos0.xyz;

                float dstInsideBox = rayBoxDst(_BoundsMin,_BoundsMax,pos,1/lightDir);

                float stepSize = dstInsideBox/_NumStepsLight;
                float d = 0;
                for(int i=0;i< _NumStepsLight;i++){
                    pos += lightDir * stepSize;
                    d += max(0, sampleDensity(pos) * stepSize);
                }
                float transmittance = exp(-d * _LightAbsorptionTowardSun);
                return lerp(1,transmittance,_DarknessThreshold);
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float3 rayPos = _WorldSpaceCameraPos;
                float viewLen = length(i.viewDir);
                float3 rayDir = i.viewDir/viewLen;

                float2 rayBoxDstInfo = rayBoxDst(_BoundsMin,_BoundsMax,rayPos,1/rayDir);
                float dstToBox = rayBoxDstInfo.x;
                float dstInsideBox = rayBoxDstInfo.y;

                float2 screenUV = i.vertex.xy/_ScreenParams.xy;
                float rawDepth = tex2D(_CameraDepthTexture,screenUV);
                float depth = LinearEyeDepth(rawDepth) * viewLen;

                float cosAngle = dot(rayDir,_WorldSpaceLightPos0);
                float phaseVal = phase(cosAngle,_PhaseParams);

                //float stepSize = dstInsideBox/_NumSteps;
                float stepSize = 11;
                float maxDist = min(depth - dstToBox,dstInsideBox);

                float3 entryPos = rayPos + rayDir * dstToBox;

                float transmittance = 1;
                float lightEnergy = 0;

                float curDist = 0;
                while(curDist < maxDist){
                    rayPos = entryPos + rayDir * curDist;
                    float density = sampleDensity(rayPos);
                    return density;
                    if(density > 0){
                        float lightTransmittance = lightmarch(rayPos);
                        lightEnergy += density * transmittance * lightTransmittance * phaseVal * stepSize;
                        transmittance *= exp(-density * _LightAbsorptionThroughCloud * stepSize);
                        if(transmittance < 0.1)
                            break;
                    }
                    curDist += stepSize;
                }
                float3 cloudCol = lightEnergy * _Color;
                return lightEnergy;
                float4 col = float4(cloudCol,1 - transmittance);
                return col;
            }
            ENDCG
        }
    }
}
