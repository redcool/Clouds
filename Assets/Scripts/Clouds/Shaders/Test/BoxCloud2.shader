Shader "Unlit/BoxCloud2"
{
    Properties
    {
        _NoiseTex("_NoiseTex",3d)=""{}
        _BoundsMin("_BoundsMin",vector) = (-100,-100,-100,0)
        _BoundsMax("_BoundsMax",vector) = (100,100,100,0)

        _WindDir("_WindDir",vector) = (1,0,0,0)

        _CloudScale("_CloudScale",float) = 1

        [Header(NoiseTex)]
        _DensityOffset("_DensityOffset",float) = 0.1
        _ShapeNoiseWeights("_ShapeNoiseWeights",Vector) = (0.2,0.1,0.3,0.4)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100
        Cull Off ZWrite Off ZTest Always
        // blend srcAlpha oneMinusSrcAlpha


        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"
#include "CloudLib.hlsl"

#define MAX_STEPS 100
#define MAX_DIST 100
#define SURF_DIST 1e-3            

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float3 viewDir:TEXCOORD2;
            };

            sampler3D _NoiseTex;
            sampler2D _CameraDepthTexture;

            float3 _BoundsMin,_BoundsMax;
            float3 _WindDir;
            float _DensityOffset;
            float4 _ShapeNoiseWeights;
            float _CloudScale;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                // o.vertex = float4(v.vertex.xy * 2,0,1);
                o.uv = v.uv;

                float3 worldPos = mul(unity_ObjectToWorld,v.vertex);
                o.viewDir = worldPos - _WorldSpaceCameraPos;

                return o;
            }

            float SampleDensity(float3 rayPos){
                float3 size = _BoundsMax -_BoundsMin;
                float3 boundsCentre = (_BoundsMax+_BoundsMin) * .5;
                float3 shapeSamplePos = (rayPos * 0.001 + _WindDir *_Time.y) * _CloudScale;

                                // Calculate base shape density
                float4 shapeNoise = tex3Dlod(_NoiseTex, float4(shapeSamplePos, 0));
                float4 normalizedShapeWeights = _ShapeNoiseWeights / dot(_ShapeNoiseWeights, 1);
                float shapeFBM = dot(shapeNoise, normalizedShapeWeights) * 1;
                float baseShapeDensity = shapeFBM + _DensityOffset;
                return baseShapeDensity;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float2 screenUV = i.vertex.xy/_ScreenParams.xy;


                float3 rayPos = _WorldSpaceCameraPos;
                float rayLength = length(i.viewDir);
                float3 rayDir = i.viewDir/rayLength;

                float rawDepth = tex2D(_CameraDepthTexture,screenUV);
                float depth = LinearEyeDepth(rawDepth);
                
                float2 rayToContainerInfo = rayBoxDst(_BoundsMin, _BoundsMax, rayPos, 1/rayDir);
                float dstToBox = rayToContainerInfo.x;
                float dstInsideBox = rayToContainerInfo.y;

                float3 ro = rayPos + rayDir * dstToBox;

                float maxDist = min(depth - dstToBox,dstInsideBox);
                float curDist = 0;
                
                const float stepSize = 11;

                while(curDist < maxDist){
                    rayPos = ro + rayDir * curDist;
                    float density = SampleDensity(rayPos);
return density;
                    curDist += stepSize;
                }

                float4 col = 0;
                return col;
            }
            ENDCG
        }
    }
}
