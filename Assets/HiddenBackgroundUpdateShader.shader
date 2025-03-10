Shader "Hidden/BackgroundUpdate"
{
    Properties
    {
        _MainTex ("Background Texture", 2D) = "black" {}
        _CurrentFrame ("Current Frame", 2D) = "white" {}
        _SegMask ("Segmentation Mask", 2D) = "black" {}
        _InvTransform ("Inverse Transform Matrix", Matrix) = (1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
        _UpdateRate ("Background Update Rate", Range(0, 1)) = 0.05
        _Threshold ("Mask Threshold", Range(0, 1)) = 0.5
        _Width ("Segmentation Width", Int) = 256
        _Height ("Segmentation Height", Int) = 256
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }
            
            sampler2D _MainTex;         // Current background
            sampler2D _CurrentFrame;    // New webcam frame
            sampler2D _SegMask;         // Segmentation mask
            float4x4 _InvTransform;     // Transform from webcam to seg space
            float _UpdateRate;
            float _Threshold;
            int _Width;
            int _Height;
            
            fixed4 frag (v2f i) : SV_Target
            {
                // Current background color
                fixed4 bgColor = tex2D(_MainTex, i.uv);
                
                // Current frame color
                fixed4 frameColor = tex2D(_CurrentFrame, i.uv);
                
                // Transform current UV to mask UV space
                float4 uvFull = float4(i.uv, 0, 1);
                float4 maskUV4 = mul(_InvTransform, uvFull);
                float2 maskUV = maskUV4.xy;
                
                // Default - assume person IS detected (don't update)
                float maskVal = 1.0;
                
                // Check if mask UVs are valid
                if (maskUV.x >= 0.0 && maskUV.x <= 1.0 && 
                    maskUV.y >= 0.0 && maskUV.y <= 1.0)
                {
                    // Sample mask
                    float2 clampedUV = clamp(maskUV, 0.001, 0.999);
                    maskVal = tex2D(_SegMask, clampedUV).r;
                }
                
                // CRITICAL FIX: Only update background where NO person is detected
                // The mask value is high (close to 1) where person IS detected
                float updateFactor = (maskVal < _Threshold) ? _UpdateRate : 0.0;
                
                // Blend current frame into background model (gradually)
                return lerp(bgColor, frameColor, updateFactor);
            }
            ENDCG
        }
    }
}