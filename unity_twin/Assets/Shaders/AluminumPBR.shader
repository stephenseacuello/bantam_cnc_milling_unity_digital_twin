Shader "MIRACLE/AluminumPBR"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.85, 0.87, 0.89, 1)
        _Metallic ("Metallic", Range(0,1)) = 0.95
        _Smoothness ("Smoothness", Range(0,1)) = 0.7
        [Normal] _BrushedNormalMap ("Brushed Normal Map", 2D) = "bump" {}
        _NormalStrength ("Normal Strength", Range(0,2)) = 1.0
        _AnisotropyDirection ("Anisotropy Direction (0=U, 1=V)", Range(0,1)) = 0.0
        _AnisotropyIntensity ("Anisotropy Intensity", Range(0,1)) = 0.5
        _BrushedTiling ("Brushed Tiling", Vector) = (4, 4, 0, 0)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" "Queue"="Geometry" }
        LOD 300

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode"="UniversalForward" }

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _ADDITIONAL_LIGHTS
            #pragma multi_compile _ _SHADOWS_SOFT
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float4 tangentOS : TANGENT;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float3 tangentWS : TEXCOORD2;
                float3 bitangentWS : TEXCOORD3;
                float2 uv : TEXCOORD4;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            TEXTURE2D(_BrushedNormalMap);
            SAMPLER(sampler_BrushedNormalMap);

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float _Metallic;
                float _Smoothness;
                float _NormalStrength;
                float _AnisotropyDirection;
                float _AnisotropyIntensity;
                float4 _BrushedTiling;
            CBUFFER_END

            // Ward anisotropic specular approximation for brushed metal
            float WardAnisotropic(float3 normal, float3 halfVec, float3 tangent, float3 bitangent, float roughnessT, float roughnessB)
            {
                float hdotT = dot(halfVec, tangent);
                float hdotB = dot(halfVec, bitangent);
                float hdotN = dot(halfVec, normal);

                float exponent = -2.0 * ((hdotT * hdotT) / (roughnessT * roughnessT) +
                                          (hdotB * hdotB) / (roughnessB * roughnessB)) /
                                          max(1.0 + hdotN, 0.001);

                float denom = 12.566371 * roughnessT * roughnessB * sqrt(max(dot(normal, _MainLightPosition.xyz) * hdotN, 0.001));
                return exp(exponent) / max(denom, 0.001);
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_TRANSFER_INSTANCE_ID(input, output);

                VertexPositionInputs posInputs = GetVertexPositionInputs(input.positionOS.xyz);
                VertexNormalInputs normInputs = GetVertexNormalInputs(input.normalOS, input.tangentOS);

                output.positionCS = posInputs.positionCS;
                output.positionWS = posInputs.positionWS;
                output.normalWS = normInputs.normalWS;
                output.tangentWS = normInputs.tangentWS;
                output.bitangentWS = normInputs.bitangentWS;
                output.uv = input.uv * _BrushedTiling.xy;
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(input);

                // Sample normal map for brushed texture
                float3 normalTS = UnpackNormalScale(SAMPLE_TEXTURE2D(_BrushedNormalMap, sampler_BrushedNormalMap, input.uv), _NormalStrength);

                // Transform normal from tangent to world space
                float3x3 TBN = float3x3(normalize(input.tangentWS), normalize(input.bitangentWS), normalize(input.normalWS));
                float3 normalWS = normalize(mul(normalTS, TBN));

                // Use original tangent/bitangent for anisotropy direction
                float3 tangent = normalize(input.tangentWS);
                float3 bitangent = normalize(input.bitangentWS);

                // Swap tangent/bitangent based on anisotropy direction parameter
                float3 anisoTangent = lerp(tangent, bitangent, _AnisotropyDirection);
                float3 anisoBitangent = lerp(bitangent, tangent, _AnisotropyDirection);

                // View and light vectors
                float3 viewDir = normalize(_WorldSpaceCameraPos - input.positionWS);
                Light mainLight = GetMainLight();
                float3 lightDir = normalize(mainLight.direction);
                float3 halfDir = normalize(lightDir + viewDir);

                // Base diffuse
                float NdotL = saturate(dot(normalWS, lightDir));
                float3 diffuse = _BaseColor.rgb * NdotL * mainLight.color;

                // Ambient / indirect
                float3 ambient = _BaseColor.rgb * 0.15;

                // Anisotropic specular for brushed aluminum look
                // Roughness is elongated along the brush direction
                float roughness = 1.0 - _Smoothness;
                float roughnessT = roughness * (1.0 + _AnisotropyIntensity * 3.0); // Stretched along brush direction
                float roughnessB = roughness * (1.0 - _AnisotropyIntensity * 0.5);  // Tighter perpendicular to brush

                float anisoSpec = WardAnisotropic(normalWS, halfDir, anisoTangent, anisoBitangent, roughnessT, roughnessB);
                anisoSpec = saturate(anisoSpec);

                // Standard isotropic specular (GGX-like approximation)
                float NdotH = saturate(dot(normalWS, halfDir));
                float specPower = exp2(10.0 * _Smoothness + 1.0);
                float isoSpec = pow(NdotH, specPower) * (specPower + 1.0) / 6.283185;

                // Blend between isotropic and anisotropic specular
                float spec = lerp(isoSpec, anisoSpec, _AnisotropyIntensity);

                // Fresnel - Schlick approximation
                // Aluminum F0 is approximately 0.91-0.95
                float3 F0 = lerp(float3(0.04, 0.04, 0.04), _BaseColor.rgb, _Metallic);
                float VdotH = saturate(dot(viewDir, halfDir));
                float3 fresnel = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);

                // Metallic workflow: metals tint their specular with base color, reduce diffuse
                float3 specularColor = spec * fresnel * mainLight.color;
                float3 diffuseContrib = diffuse * (1.0 - _Metallic);

                // Combine
                float3 finalColor = ambient + diffuseContrib + specularColor;

                // Subtle reflection probe sampling approximation
                float3 reflectDir = reflect(-viewDir, normalWS);
                float3 envReflect = _BaseColor.rgb * _Metallic * 0.15 * (1.0 + saturate(dot(reflectDir, float3(0, 1, 0))));
                finalColor += envReflect;

                return half4(finalColor, 1);
            }
            ENDHLSL
        }

        // Shadow caster pass
        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode"="ShadowCaster" }

            ZWrite On
            ZTest LEqual
            ColorMask 0

            HLSLPROGRAM
            #pragma vertex ShadowPassVertex
            #pragma fragment ShadowPassFragment
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/ShadowCasterPass.hlsl"
            ENDHLSL
        }

        // Depth only pass
        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode"="DepthOnly" }

            ZWrite On
            ColorMask R

            HLSLPROGRAM
            #pragma vertex DepthOnlyVertex
            #pragma fragment DepthOnlyFragment
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/DepthOnlyPass.hlsl"
            ENDHLSL
        }
    }
    FallBack "Universal Render Pipeline/Lit"
}
