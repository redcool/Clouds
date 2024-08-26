
Shader "Nature/BoxClouds"
{
    
    Properties
    {
    }
    SubShader
    {
        
        // No culling or depth
        Cull Off ZWrite Off ZTest Always
        blend srcAlpha oneMinusSrcAlpha

        Pass
        {

            CGPROGRAM

            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"
#include "CloudLib.hlsl"
            // vertex input: position, UV
            struct appdata {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
                float3 viewVector : TEXCOORD1;
            };
            
            v2f vert (appdata v) {
                v2f output;
                output.uv = v.uv;

                #if !defined(FULL_SCREEN)
                output.pos = UnityObjectToClipPos(v.vertex);
                float3 hitPos = mul(unity_ObjectToWorld,float4(v.vertex) );
                output.viewVector = hitPos - _WorldSpaceCameraPos;
                #endif

                #if defined(FULL_SCREEN)
                output.pos = float4(v.vertex.xy*2,0,1);
                // Camera space matches OpenGL convention where cam forward is -z. In unity forward is positive z.
                // (https://docs.unity3d.com/ScriptReference/Camera-cameraToWorldMatrix.html)
                float2 dirScale = float2(1,1);
                #if defined(UNITY_UV_STARTS_AT_TOP)
                    dirScale.y =-1;
                #endif
                float3 viewVector = mul(unity_CameraInvProjection, float4(output.pos.xy*dirScale, 0, -1));
                output.viewVector = mul(unity_CameraToWorld, float4(viewVector,0));
                #endif

                return output;
            }

            // Textures
            sampler3D NoiseTex;
            sampler3D DetailNoiseTex;
            sampler2D WeatherMap;
            sampler2D BlueNoise;
            
            sampler2D _MainTex;
            sampler2D _CameraDepthTexture;

            // Shape settings
            float4 params;
            int3 mapSize;
            float densityMultiplier;
            float densityOffset;
            float scale;
            float detailNoiseScale;
            float detailNoiseWeight;
            float3 detailWeights;
            float4 shapeNoiseWeights;
            float4 phaseParams;

            // March settings
            int numStepsLight;
            float rayOffsetStrength;

            float3 boundsMin;
            float3 boundsMax;

            float3 shapeOffset;
            float3 detailOffset;

            // Light settings
            float lightAbsorptionTowardSun;
            float lightAbsorptionThroughCloud;
            float darknessThreshold;
            float4 _LightColor0;
            float4 colA;
            float4 colB;

            // Animation settings
            float timeScale;
            float baseSpeed;
            float detailSpeed;

            // Debug settings:
            int debugViewMode; // 0 = off; 1 = shape tex; 2 = detail tex; 3 = weathermap
            int debugGreyscale;
            int debugShowAllChannels;
            float debugNoiseSliceDepth;
            float4 debugChannelWeight;
            float debugTileAmount;
            float viewerSize;

            //-----
            float3 _WindDir;
            


            float sampleDensity1(float3 rayPos) {
                // Constants:
                const int mipLevel = 0;
                const float baseScale = 1/1000.0;
                const float offsetSpeed = 1/100.0;

                // Calculate texture sample positions
                float time = _Time.x * timeScale;
                float3 size = boundsMax - boundsMin;
                float3 boundsCentre = (boundsMin+boundsMax) * .5;
                float3 uvw = (size * .5 + rayPos) * baseScale * scale;
                float3 shapeSamplePos = uvw + shapeOffset * offsetSpeed + float3(time,time*0.1,time*0.2) * baseSpeed;

                // Calculate falloff at along x/z edges of the cloud container
                const float containerEdgeFadeDst = 50;
                float dstFromEdgeX = min(containerEdgeFadeDst, min(rayPos.x - boundsMin.x, boundsMax.x - rayPos.x));
                float dstFromEdgeZ = min(containerEdgeFadeDst, min(rayPos.z - boundsMin.z, boundsMax.z - rayPos.z));
                float edgeWeight = min(dstFromEdgeZ,dstFromEdgeX)/containerEdgeFadeDst;
                
                // Calculate height gradient from weather map
                //float2 weatherUV = (size.xz * .5 + (rayPos.xz-boundsCentre.xz)) / max(size.x,size.z);
                //float weatherMap = WeatherMap.SampleLevel(samplerWeatherMap, weatherUV, mipLevel).x;
                float gMin = .2;
                float gMax = .7;
                float heightPercent = (rayPos.y - boundsMin.y) / size.y;
                float heightGradient = saturate(remap(heightPercent, 0.0, gMin, 0, 1)) * saturate(remap(heightPercent, 1, gMax, 0, 1));
                heightGradient *= edgeWeight;

                // Calculate base shape density
                float4 shapeNoise = tex3Dlod(NoiseTex, float4(shapeSamplePos, mipLevel));
                float4 normalizedShapeWeights = shapeNoiseWeights / dot(shapeNoiseWeights, 1);
                float shapeFBM = dot(shapeNoise, normalizedShapeWeights) * heightGradient;
                float baseShapeDensity = shapeFBM + densityOffset;

                // Save sampling from detail tex if shape density <= 0
                if (baseShapeDensity > 0) {
                    // Sample detail noise
                    float3 detailSamplePos = uvw*detailNoiseScale + detailOffset * offsetSpeed + float3(time*.4,-time,time*0.1)*detailSpeed;
                    float4 detailNoise = tex3Dlod(DetailNoiseTex,float4(detailSamplePos, 0));
                    float3 normalizedDetailWeights = detailWeights / dot(detailWeights, 1);
                    float detailFBM = dot(detailNoise, normalizedDetailWeights);

                    // Subtract detail noise from base shape (weighted by inverse density so that edges get eroded more than centre)
                    float oneMinusShape = 1 - shapeFBM;
                    float detailErodeWeight = oneMinusShape * oneMinusShape * oneMinusShape;
                    float cloudDensity = baseShapeDensity - (1-detailFBM) * detailErodeWeight * detailNoiseWeight;
    
                    return cloudDensity * densityMultiplier;
                }
                return baseShapeDensity;
            }

            float sampleDensity(float3 rayPos) {

                // Calculate texture sample positions
                // float time = _Time.x * timeScale;
                float3 size = boundsMax - boundsMin;
                float3 boundsCentre = (boundsMin+boundsMax) * .5;
                float3 shapeSamplePos = (rayPos * 0.001 + _WindDir *_Time.y) * scale;

                // Calculate falloff at along x/z edges of the cloud container
                const float containerEdgeFadeDst = 50;
                float3 dstEdge = min(containerEdgeFadeDst,min(rayPos - boundsMin,boundsMax - rayPos));
                float edgeWeight = min(dstEdge.x,min(dstEdge.y,dstEdge.z))/containerEdgeFadeDst;
                
                // Calculate height gradient from weather map
                //float2 weatherUV = (size.xz * .5 + (rayPos.xz-boundsCentre.xz)) / max(size.x,size.z);
                //float weatherMap = WeatherMap.SampleLevel(samplerWeatherMap, weatherUV, mipLevel).x;

                float heightPercent = (rayPos.y - boundsMin.y) / size.y;
                float gMin = .2;
                float gMax = .7;
                float heightGradient = saturate(remap(heightPercent, 0.0, gMin, 0, 1)) * saturate(remap(heightPercent, 1, gMax, 0, 1));
                heightGradient *= edgeWeight;

                // float heightGradient = edgeWeight * ((1-abs(heightPercent-0.5)));

                // Calculate base shape density
                float4 shapeNoise = tex3Dlod(NoiseTex, float4(shapeSamplePos, 0));
                float4 normalizedShapeWeights = shapeNoiseWeights / dot(shapeNoiseWeights, 1);
                float shapeFBM = dot(shapeNoise, normalizedShapeWeights) * heightGradient;
                float baseShapeDensity = shapeFBM + densityOffset;

                // Save sampling from detail tex if shape density <= 0
                if (baseShapeDensity > 0) {
                    // Sample detail noise
                    float3 detailSamplePos = rayPos*0.001 * detailNoiseScale + _WindDir * _Time.y * detailSpeed;

                    float4 detailNoise = tex3Dlod(DetailNoiseTex,float4(detailSamplePos, 0));
                    float3 normalizedDetailWeights = detailWeights / dot(detailWeights, 1);
                    float detailFBM = dot(detailNoise, normalizedDetailWeights);

                    // Subtract detail noise from base shape (weighted by inverse density so that edges get eroded more than centre)
                    float oneMinusShape = 1 - shapeFBM;
                    float detailErodeWeight = oneMinusShape * oneMinusShape * oneMinusShape;
                    float cloudDensity = baseShapeDensity - (1-detailFBM) * detailErodeWeight * detailNoiseWeight;
    
                    return cloudDensity * densityMultiplier;
                }
                return baseShapeDensity;
            }
            // Calculate proportion of light that reaches the given point from the lightsource
            float lightmarch(float3 position) {
                float3 dirToLight = _WorldSpaceLightPos0.xyz;
                float dstInsideBox = rayBoxDst(boundsMin, boundsMax, position, 1/dirToLight).y;
                
                float stepSize = dstInsideBox/numStepsLight;
                float totalDensity = 0;

                for (int step = 0; step < numStepsLight; step ++) {
                    position += dirToLight * stepSize;
                    totalDensity += max(0, sampleDensity(position) * stepSize);
                }

                float transmittance = exp(-totalDensity * lightAbsorptionTowardSun);
                return darknessThreshold + transmittance * (1-darknessThreshold);
            }

          
            float4 frag (v2f i) : SV_Target
            {
                // Create ray
                float3 rayPos = _WorldSpaceCameraPos;

                float viewLength = length(i.viewVector);
                float3 rayDir = i.viewVector / viewLength;

                // Depth and cloud container intersection info:
                float nonlin_depth = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);
                float depth = LinearEyeDepth(nonlin_depth) * viewLength;
                
                float2 rayToContainerInfo = rayBoxDst(boundsMin, boundsMax, rayPos, 1/rayDir);
                float dstToBox = rayToContainerInfo.x;
                float dstInsideBox = rayToContainerInfo.y;

                // point of intersection with the cloud container
                float3 entryPoint = rayPos + rayDir * dstToBox;

                // random starting offset (makes low-res results noisy rather than jagged/glitchy, which is nicer)
                float randomOffset = tex2Dlod(BlueNoise, float4(squareUV(i.uv*3),0, 0));
                randomOffset *= rayOffsetStrength;
                
                // Phase function makes clouds brighter around sun
                float cosAngle = dot(rayDir, _WorldSpaceLightPos0.xyz);
                float phaseVal = phase(cosAngle,phaseParams);

                float dstTravelled = randomOffset;
                float dstLimit = min(depth-dstToBox, dstInsideBox);
                const float stepSize = 11;

                // March through volume:
                float transmittance = 1;
                float3 lightEnergy = 0;

                while (dstTravelled < dstLimit) {
                    rayPos = entryPoint + rayDir * dstTravelled;
                    float density = sampleDensity(rayPos);
                    
                    if (density > 0) {
                        float lightTransmittance = lightmarch(rayPos);
                        lightEnergy += density * stepSize * transmittance * lightTransmittance * phaseVal;
                        transmittance *= exp(-density * stepSize * lightAbsorptionThroughCloud);
                    
                        // Exit early if T is close to zero as further samples won't affect the result much
                        if (transmittance < 0.01) {
                            break;
                        }
                    }
                    dstTravelled += stepSize;
                }

                // Add clouds to background
                float3 backgroundCol = tex2D(_MainTex,i.uv);
                float3 cloudCol = lightEnergy * _LightColor0;
                // use alpha blend
                return float4(cloudCol,1-transmittance);

                float3 col = backgroundCol * transmittance + cloudCol;
                return float4(col,0);

            }

            ENDCG
        }
    }
}