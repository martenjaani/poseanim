Shader "PersonOnly"
{
    Properties {
        _MainTex ("Webcam Texture", 2D) = "white" {}
        _MaskTex ("Mask Texture", 2D) = "black" {}
        _Threshold ("Mask Threshold", Range(0,1)) = 0.5
        _BlendEdge ("Blend Edge Softness", Range(0.001, 0.1)) = 0.02
        _DilationAmount ("Mask Dilation Amount", Range(0.0, 0.05)) = 0.01
    }
    SubShader {
        Tags { "RenderType"="Opaque" }
        Pass {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            sampler2D _MainTex;     // Webcam texture
            sampler2D _MaskTex;      // Segmentation mask
            uniform float4x4 _InvTransform;  // Matrix to transform webcam UVs to mask UVs
            float _Threshold;        // Segmentation threshold
            float _BlendEdge;        // Edge softness for blending
            float _DilationAmount;   // How much to expand the mask
            
            fixed4 frag(v2f_img i) : SV_Target {
                // Get the original webcam color
                fixed4 camColor = tex2D(_MainTex, i.uv);
                
                // Transform webcam UV to segmentation UV
                float4 uvFull = float4(i.uv, 0, 1);
                float4 maskUV4 = mul(_InvTransform, uvFull);
                float2 maskUV = maskUV4.xy;
                
                // Check if UVs are out of bounds
                bool validUV = maskUV.x >= 0.0 && maskUV.x <= 1.0 && 
                               maskUV.y >= 0.0 && maskUV.y <= 1.0;
                
                // If UV is invalid, just return black
                if (!validUV) {
                    return fixed4(0, 0, 0, 1);
                }
                
                // Clamp UVs to valid range for sampling
                float2 clampedUV = clamp(maskUV, 0.001, 0.999);
                
                // Sample the mask (assuming grayscale mask)
                float maskVal = tex2D(_MaskTex, clampedUV).r;
                
                // Dilate the mask by sampling neighboring pixels and taking maximum
                float dilatedMask = maskVal;
                
                // Sample in 8 directions and take maximum for better dilation
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(_DilationAmount, 0)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV - float2(_DilationAmount, 0)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(0, _DilationAmount)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV - float2(0, _DilationAmount)).r);
                
                // Sample diagonal directions too for more uniform expansion
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(_DilationAmount, _DilationAmount)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(_DilationAmount, -_DilationAmount)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(-_DilationAmount, _DilationAmount)).r);
                dilatedMask = max(dilatedMask, tex2D(_MaskTex, clampedUV + float2(-_DilationAmount, -_DilationAmount)).r);
                
                // Apply threshold with smooth edge blending
                float blend = smoothstep(_Threshold - _BlendEdge, _Threshold + _BlendEdge, dilatedMask);
                
                // For person-only mode, we want the person (where mask is) to show through,
                // and everything else to be black
                // This is the reverse of the original shader logic
                fixed4 replacementColor = fixed4(0, 0, 0, 1); // Black
                
                // Composite the final color - blend between webcam and black
                // Note that we're using (1-blend) to invert the mask effect:
                // Where mask is strong (person), we want webcam to show through
                fixed4 finalColor = lerp(replacementColor, camColor, blend);
                return finalColor;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}