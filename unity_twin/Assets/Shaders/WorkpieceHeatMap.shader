Shader "MIRACLE/WorkpieceHeatMap"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.8, 0.8, 0.85, 1)
        _Metallic ("Metallic", Range(0,1)) = 0.95
        _Smoothness ("Smoothness", Range(0,1)) = 0.6
        _Temperature ("Temperature", Float) = 20
        _MinTemp ("Min Temperature", Float) = 20
        _MaxTemp ("Max Temperature", Float) = 200
        _HeatPosition ("Heat Position", Vector) = (0, 0, 0, 0)
        _HeatRadius ("Heat Radius", Float) = 0.01
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" }
        LOD 200

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float2 uv : TEXCOORD2;
            };

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float _Metallic;
                float _Smoothness;
                float _Temperature;
                float _MinTemp;
                float _MaxTemp;
                float4 _HeatPosition;
                float _HeatRadius;
            CBUFFER_END

            // Temperature to color ramp
            float3 TemperatureToColor(float t)
            {
                // 0.0 = silver (ambient), 0.3 = yellow, 0.65 = orange, 1.0 = red
                float3 silver = float3(0.8, 0.8, 0.85);
                float3 yellow = float3(1.0, 1.0, 0.3);
                float3 orange = float3(1.0, 0.5, 0.1);
                float3 red = float3(1.0, 0.1, 0.1);

                if (t < 0.3)
                    return lerp(silver, yellow, t / 0.3);
                else if (t < 0.65)
                    return lerp(yellow, orange, (t - 0.3) / 0.35);
                else
                    return lerp(orange, red, (t - 0.65) / 0.35);
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
                output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                output.uv = input.uv;
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                // Distance from heat source
                float dist = length(input.positionWS - _HeatPosition.xyz);
                float heatInfluence = 1.0 - saturate(dist / max(_HeatRadius * 5.0, 0.001));
                heatInfluence *= heatInfluence; // Quadratic falloff

                // Temperature normalization
                float tempNorm = saturate((_Temperature * heatInfluence - _MinTemp) / max(_MaxTemp - _MinTemp, 1.0));

                // Color from temperature
                float3 heatColor = TemperatureToColor(tempNorm);
                float3 baseColor = _BaseColor.rgb;

                // Blend base with heat
                float3 finalColor = lerp(baseColor, heatColor, tempNorm * 0.7);

                // Simple PBR-like lighting
                float3 lightDir = normalize(float3(0.5, 1, 0.3));
                float ndotl = saturate(dot(input.normalWS, lightDir));
                finalColor *= (0.3 + 0.7 * ndotl);

                // Metallic reflection approximation
                float3 viewDir = normalize(_WorldSpaceCameraPos - input.positionWS);
                float3 halfDir = normalize(lightDir + viewDir);
                float spec = pow(saturate(dot(input.normalWS, halfDir)), 32) * _Smoothness;
                finalColor += spec * _Metallic;

                return half4(finalColor, 1);
            }
            ENDHLSL
        }
    }
}
