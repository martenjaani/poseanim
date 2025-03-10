Shader "SegPaint"
{
    Properties {
        _MainTex ("Webcam Texture", 2D) = "white" {}
        _MaskTex ("Mask Texture", 2D) = "black" {}
        _BgTex ("Background Model Texture", 2D) = "black" {}
        _Threshold ("Mask Threshold", Range(0,1)) = 0.5
        _BackgroundColor ("Fallback Background Color", Color) = (0,0,0,1)
        _DebugMode ("Debug Mode", Int) = 0
        _BlendEdge ("Blend Edge Softness", Range(0.001, 0.1)) = 0.02
        _UseDynamicBg ("Use Dynamic Background", Int) = 1
        _DilationAmount ("Mask Dilation Amount", Range(0.0, 0.05)) = 0.01
    }
    SubShader {
        Tags { "RenderType"="Opaque" }
        Pass {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            sampler2D _MainTex;    // Webcam texture
            sampler2D _MaskTex;     // Segmentation mask
            sampler2D _BgTex;       // Dynamic background model texture
            uniform float4x4 _InvTransform;  // Matrix to transform webcam UVs to mask UVs
            float _Threshold;        // Segmentation threshold
            fixed4 _BackgroundColor; // Fallback background color
            int _DebugMode;          // Debug visualization mode
            float _BlendEdge;        // Edge softness for blending
            int _UseDynamicBg;       // Whether to use the dynamic background (1) or solid color (0)
            float _DilationAmount;   // How much to expand the mask
            
            fixed4 frag(v2f_img i) : SV_Target {
                // Get the original webcam color
                fixed4 camColor = tex2D(_MainTex, i.uv);
                
                // Get the background model color at the same position
                fixed4 bgColor = tex2D(_BgTex, i.uv);
                
                // Transform webcam UV to segmentation UV
                float4 uvFull = float4(i.uv, 0, 1);
                float4 maskUV4 = mul(_InvTransform, uvFull);
                float2 maskUV = maskUV4.xy;
                
                // Check if UVs are out of bounds
                bool validUV = maskUV.x >= 0.0 && maskUV.x <= 1.0 && 
                               maskUV.y >= 0.0 && maskUV.y <= 1.0;
                
                // If UV is invalid, just return the webcam texture
                if (!validUV && _DebugMode == 0) {
                    return camColor;
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
                
                // Debug visualization modes
                if (_DebugMode == 1) {
                    // Debug: Show raw UV coordinates (red = x, green = y)
                    return fixed4(maskUV.x, maskUV.y, 0, 1);
                }
                else if (_DebugMode == 2) {
                    // Debug: Show mask
                    return fixed4(maskVal, maskVal, maskVal, 1);
                }
                else if (_DebugMode == 3) {
                    // Debug: Show blend value
                    return fixed4(blend, blend, blend, 1);
                }
                else if (_DebugMode == 4) {
                    // Debug: Show UV validity - Red for out of bounds
                    return validUV ? fixed4(0, 1, 0, 1) : fixed4(1, 0, 0, 1);
                }
                else if (_DebugMode == 5) {
                    // Debug: Show background model texture
                    return bgColor;
                }
                
                // Determine the replacement color (either dynamic background or solid color)
                fixed4 replacementColor = _UseDynamicBg ? bgColor : _BackgroundColor;
                
                // Check if background pixel is too dark/empty, use fallback color if needed
                float bgLuminance = dot(bgColor.rgb, float3(0.2126, 0.7152, 0.0722));
                if (_UseDynamicBg && bgLuminance < 0.02) {
                    replacementColor = _BackgroundColor;
                }
                
                // Composite the final color - blend between webcam and replacement color
                fixed4 finalColor = lerp(camColor, replacementColor, blend);
                return finalColor;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}