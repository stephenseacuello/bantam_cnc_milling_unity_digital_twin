Shader "MIRACLE/HSSToolSteel"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.25, 0.25, 0.28, 1)
        _TintColor ("Tint Color (Heat Treatment)", Color) = (0.35, 0.30, 0.45, 1)
        _Metallic ("Metallic", Range(0,1)) = 0.8
        _Smoothness ("Smoothness", Range(0,1)) = 0.6
        _ColorVariation ("Color Variation", Range(0,1)) = 0.15
        _WearAmount ("Wear Amount", Range(0,1)) = 0.0
        _WearEdgeSharpness ("Wear Edge Sharpness", Range(0.1, 10)) = 2.0
        _GrainScale ("Surface Grain Scale", Float) = 50.0
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
                float2 uv : TEXCOORD3;
                float3 positionOS : TEXCOORD4;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float4 _TintColor;
                float _Metallic;
                float _Smoothness;
                float _ColorVariation;
                float _WearAmount;
                float _WearEdgeSharpness;
                float _GrainScale;
            CBUFFER_END

            // Simple hash for procedural noise (used for surface grain and color variation)
            float Hash(float2 p)
            {
                float3 p3 = frac(float3(p.xyx) * 0.1031);
                p3 += dot(p3, p3.yzx + 33.33);
                return frac((p3.x + p3.y) * p3.z);
            }

            // Value noise for smooth procedural variation
            float ValueNoise(float2 p)
            {
                float2 i = floor(p);
                float2 f = frac(p);
                f = f * f * (3.0 - 2.0 * f); // Smoothstep interpolation

                float a = Hash(i);
                float b = Hash(i + float2(1.0, 0.0));
                float c = Hash(i + float2(0.0, 1.0));
                float d = Hash(i + float2(1.0, 1.0));

                return lerp(lerp(a, b, f.x), lerp(c, d, f.x), f.y);
            }

            // Fractal brownian motion for multi-octave noise
            float FBM(float2 p, int octaves)
            {
                float value = 0.0;
                float amplitude = 0.5;
                for (int i = 0; i < octaves; i++)
                {
                    value += amplitude * ValueNoise(p);
                    p *= 2.0;
                    amplitude *= 0.5;
                }
                return value;
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
                output.uv = input.uv;
                output.positionOS = input.positionOS.xyz;
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(input);

                float3 normalWS = normalize(input.normalWS);

                // --- Surface color variation from heat treatment ---
                // Use object-space position for consistent noise across UVs
                float2 noiseUV = input.positionOS.xz * 3.0;
                float variation = FBM(noiseUV, 3);

                // Blend base steel color with tint color based on variation
                float3 baseColor = lerp(_BaseColor.rgb, _TintColor.rgb, variation * _ColorVariation);

                // Subtle dark steel grain from microscopic surface texture
                float grain = ValueNoise(input.uv * _GrainScale);
                baseColor *= lerp(0.92, 1.08, grain);

                // --- Tool wear simulation ---
                // Wear tends to occur at edges/tips - approximate using curvature (normal divergence)
                // For a simple approximation, darken and roughen based on wear parameter
                float wearMask = saturate(1.0 - pow(abs(dot(normalWS, float3(0, 0, 1))), _WearEdgeSharpness));
                float wearFactor = _WearAmount * wearMask;

                // Worn areas are darker and rougher
                baseColor = lerp(baseColor, baseColor * 0.4, wearFactor);
                float smoothness = lerp(_Smoothness, _Smoothness * 0.3, wearFactor);
                float metallic = lerp(_Metallic, _Metallic * 0.6, wearFactor);

                // --- Lighting ---
                float3 viewDir = normalize(_WorldSpaceCameraPos - input.positionWS);
                Light mainLight = GetMainLight();
                float3 lightDir = normalize(mainLight.direction);
                float3 halfDir = normalize(lightDir + viewDir);

                // Diffuse
                float NdotL = saturate(dot(normalWS, lightDir));
                float3 diffuse = baseColor * NdotL * mainLight.color;

                // Ambient / hemispheric approximation
                float upFactor = dot(normalWS, float3(0, 1, 0)) * 0.5 + 0.5;
                float3 ambient = baseColor * lerp(0.08, 0.18, upFactor);

                // Specular - GGX-like approximation
                float NdotH = saturate(dot(normalWS, halfDir));
                float specPower = exp2(10.0 * smoothness + 1.0);
                float spec = pow(NdotH, specPower) * (specPower + 1.0) / 6.283185;

                // Fresnel - HSS steel has F0 around 0.56-0.58
                float3 F0 = lerp(float3(0.04, 0.04, 0.04), baseColor, metallic);
                float VdotH = saturate(dot(viewDir, halfDir));
                float3 fresnel = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);

                float3 specularColor = spec * fresnel * mainLight.color;

                // Metallic workflow
                float3 diffuseContrib = diffuse * (1.0 - metallic);

                // Combine
                float3 finalColor = ambient + diffuseContrib + specularColor;

                // Subtle dark edge rim for tool steel look
                float rimFactor = 1.0 - saturate(dot(viewDir, normalWS));
                rimFactor = pow(rimFactor, 4.0);
                finalColor += baseColor * rimFactor * 0.05 * metallic;

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
