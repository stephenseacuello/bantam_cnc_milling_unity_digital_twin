Shader "MIRACLE/ProceduralWorkpiece"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.85, 0.87, 0.89, 1)
        _Metallic ("Metallic", Range(0,1)) = 0.85
        _Smoothness ("Smoothness", Range(0,1)) = 0.6
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" "Queue"="Geometry" }
        LOD 200

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode"="UniversalForward" }

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _ADDITIONAL_LIGHTS
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            // Vertex data from marching cubes compute shader (position + normal, 6 floats)
            struct MCVertex
            {
                float3 position;
                float3 normal;
            };

            StructuredBuffer<MCVertex> _Vertices;

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float _Metallic;
                float _Smoothness;
            CBUFFER_END

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
            };

            Varyings vert(uint vertexID : SV_VertexID)
            {
                Varyings output;
                MCVertex v = _Vertices[vertexID];

                float3 posWS = TransformObjectToWorld(v.position);
                output.positionCS = TransformWorldToHClip(posWS);
                output.positionWS = posWS;
                output.normalWS = TransformObjectToWorldNormal(v.normal);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                float3 normalWS = normalize(input.normalWS);
                float3 viewDir = normalize(_WorldSpaceCameraPos - input.positionWS);

                // Main light
                Light mainLight = GetMainLight();
                float3 lightDir = normalize(mainLight.direction);
                float3 halfDir = normalize(lightDir + viewDir);

                // Diffuse (Lambert)
                float NdotL = saturate(dot(normalWS, lightDir));
                float3 diffuse = _BaseColor.rgb * NdotL * mainLight.color;

                // Ambient
                float3 ambient = _BaseColor.rgb * 0.18;

                // Specular (Blinn-Phong)
                float NdotH = saturate(dot(normalWS, halfDir));
                float specPower = exp2(10.0 * _Smoothness + 1.0);
                float spec = pow(NdotH, specPower) * (specPower + 1.0) / 6.283185;

                // Fresnel (Schlick)
                float3 F0 = lerp(float3(0.04, 0.04, 0.04), _BaseColor.rgb, _Metallic);
                float VdotH = saturate(dot(viewDir, halfDir));
                float3 fresnel = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);

                float3 specularColor = spec * fresnel * mainLight.color;
                float3 diffuseContrib = diffuse * (1.0 - _Metallic);

                // Rim light for better edge definition
                float rim = 1.0 - saturate(dot(normalWS, viewDir));
                float3 rimColor = _BaseColor.rgb * pow(rim, 3.0) * 0.15;

                return half4(ambient + diffuseContrib + specularColor + rimColor, 1);
            }
            ENDHLSL
        }
    }
    FallBack "Universal Render Pipeline/Unlit"
}
