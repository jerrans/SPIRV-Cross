///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Intel Corporation
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of 
// the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

#define THREAD_COUNT 64
static float rand_array[THREAD_COUNT] = 
{
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
    1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f,
};

float4 rand4(uint ID) { return float4(rand_array[ID], rand_array[ID], rand_array[ID], rand_array[ID]); }
float3 rand3(uint ID) { return float3(rand_array[ID], rand_array[ID], rand_array[ID]); }
float2 rand2(uint ID) { return float2(rand_array[ID], rand_array[ID]); }
float  rand1(uint ID) { return rand_array[ID]; }

struct AtomicBuffer
{
    uint AtomicIndex;
};

struct OutputBuffer
{
    float4 f4;
    float3 f3;
    float2 f2;
    float f1;
};

RWStructuredBuffer< AtomicBuffer > g_AtomicBuffer : register(u1);
RWStructuredBuffer< OutputBuffer > g_OutputBuffer : register(u2);


[numthreads(THREAD_COUNT, 1, 1)]
void main(uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID)
{
    float4 a4 = rand4(DTid.x), b4 = rand4(DTid.x), c4 = rand4(DTid.x), r4;
    float3 a3 = rand3(DTid.x % THREAD_COUNT), b3 = rand3(DTid.x % THREAD_COUNT), c3 = rand3(DTid.x % THREAD_COUNT), r3;
    float2 a2 = rand2(DTid.x % THREAD_COUNT), b2 = rand2(DTid.x % THREAD_COUNT), c2 = rand2(DTid.x % THREAD_COUNT), r2;
    float a1 = rand1(DTid.x % THREAD_COUNT), b1 = rand1(DTid.x % THREAD_COUNT), c1 = rand1(DTid.x % THREAD_COUNT), r1;
    bool rb1;
    bool2 rb2;
    bool3 rb3;
    bool4 rb4;
    int ri1;
    int2 ri2;
    int3 ri3;
    int4 ri4;

    r4 = lerp(a4, b4, c1);
    r3 = lerp(a3, b3, c1);
    r2 = lerp(a2, b2, c1);
    r1 = lerp(a1, b1, c1);

    r1 = dot(a4, b4);
    r1 = dot(a3, b3);
    r1 = dot(a2, b2);

    r4 = reflect(a4, b4);
    r3 = reflect(a3, b3);
    r2 = reflect(a2, b2);

    r1 = length(a4);
    r1 = length(a3);
    r1 = length(a2);

    g_AtomicBuffer.IncrementCounter();
    g_AtomicBuffer.DecrementCounter();

    r4 = abs(a4);
    r3 = abs(a3);
    r2 = abs(a2);
    r1 = abs(a1);

    r4 = cos(a4);
    r3 = cos(a3);
    r2 = cos(a2);
    r1 = cos(a1);

    r4 = acos(a4);
    r3 = acos(a3);
    r2 = acos(a2);
    r1 = acos(a1);

    r4 = cosh(a4);
    r3 = cosh(a3);
    r2 = cosh(a2);
    r1 = cosh(a1);

    r4 = sin(a4);
    r3 = sin(a3);
    r2 = sin(a2);
    r1 = sin(a1);

    r4 = asin(a4);
    r3 = asin(a3);
    r2 = asin(a2);
    r1 = asin(a1);

    r4 = sinh(a4);
    r3 = sinh(a3);
    r2 = sinh(a2);
    r1 = sinh(a1);

    r4 = tan(a4);
    r3 = tan(a3);
    r2 = tan(a2);
    r1 = tan(a1);

    r4 = atan(a4);
    r3 = atan(a3);
    r2 = atan(a2);
    r1 = atan(a1);

    r4 = tanh(a4);
    r3 = tanh(a3);
    r2 = tanh(a2);
    r1 = tanh(a1);

//    r4 = atan2(a4); // glslang doesn't appear to support this...
//    r3 = atan2(a3);
//    r2 = atan2(a2);
//    r1 = atan2(a1);

    r4 = ceil(a4);
    r3 = ceil(a3);
    r2 = ceil(a2);

    r4 = floor(a4);
    r3 = floor(a3);
    r2 = floor(a2);

    r4 = round(a4);
    r3 = round(a3);
    r2 = round(a2);
    r1 = round(a1);

    r4 = log(a4);
    r3 = log(a3);
    r2 = log(a2);
    r1 = log(a1);

    r4 = log2(a4);
    r3 = log2(a3);
    r2 = log2(a2);
    r1 = log2(a1);

    r4 = log10(a4);
    r3 = log10(a3);
    r2 = log10(a2);
    r1 = log10(a1);

    r4 = exp(a4);
    r3 = exp(a3);
    r2 = exp(a2);
    r1 = exp(a1);

    r4 = exp2(a4);
    r3 = exp2(a3);
    r2 = exp2(a2);
    r1 = exp2(a1);

    r4 = rcp(a4);
    r3 = rcp(a3);
    r2 = rcp(a2);
    r1 = rcp(a1);

    r4 = sqrt(a4);
    r3 = sqrt(a3);
    r2 = sqrt(a2);
    r1 = sqrt(a1);

    r4 = rsqrt(a4);
    r3 = rsqrt(a3);
    r2 = rsqrt(a2);
    r1 = rsqrt(a1);

    r4 = saturate(a4);
    r3 = saturate(a3);
    r2 = saturate(a2);
    r1 = saturate(a1);

    rb1 = all(a4);
    rb1 = all(a3);
    rb1 = all(a2);
    rb1 = all(a1);

    rb1 = any(a4);
    rb1 = any(a3);
    rb1 = any(a2);
    rb1 = any(a1);

    r3 = cross(a3, b3);

    r4 = degrees(a4);
    r3 = degrees(a3);
    r2 = degrees(a2);
    r1 = degrees(a1);

    r4 = radians(a4);
    r3 = radians(a3);
    r2 = radians(a2);
    r1 = radians(a1);

    r1 = distance(a4, b4);
    r1 = distance(a3, b3);
    r1 = distance(a2, b2);
//    r1 = distance(a1, b1);  //glslvalidator doesn't like this

    r4 = frac(a4);
    r3 = frac(a3);
    r2 = frac(a2);
    r1 = frac(a1);

    ri4 = trunc(a4);
    ri3 = trunc(a3);
    ri2 = trunc(a2);
    ri1 = trunc(a1);

    r4 = lerp(a4, b4, c1); // transformed to 'mix'
    r3 = lerp(a3, b3, c1);
    r2 = lerp(a2, b2, c1);
    r1 = lerp(a1, b1, c1);

    r4 = mad(a4, b4, c4); 
    r3 = mad(a3, b3, c3);
    r2 = mad(a2, b2, c2);
    r1 = mad(a1, b1, c1);

    r4 = min(a4, b4);
    r3 = min(a3, b3);
    r2 = min(a2, b2);
    r1 = min(a1, b1);

    r4 = max(a4, b4);
    r3 = max(a3, b3);
    r2 = max(a2, b2);
    r1 = max(a1, b1);

    r4 = normalize(a4);
    r3 = normalize(a3);
    r2 = normalize(a2);

    r4 = pow(a4, b4);
    r3 = pow(a3, b3);
    r2 = pow(a2, b2);

    r4 = pow(a4, b1);
    r3 = pow(a3, b1);
    r2 = pow(a2, b1);
    r1 = pow(a1, b1);

    r4 = sign(a4);
    r3 = sign(a3);
    r2 = sign(a2);
    r1 = sign(a1);

    sincos(a4, b4, c4);
    sincos(a3, b3, c3);
    sincos(a2, b2, c2);
    sincos(a1, b1, c1);

    r4 = smoothstep(a4, b4, c1);
    r3 = smoothstep(a3, b3, c1);
    r2 = smoothstep(a2, b2, c1);
    r1 = smoothstep(a1, b1, c1);

    // not yet supported by glslvalidator
    //r4 = step(a4, b4, c1);
    //r3 = step(a3, b3, c1);
    //r2 = step(a2, b2, c1);
    //r1 = step(a1, b1, c1);

    r4 = fmod(a4, b4);
    r3 = fmod(a3, b3);
    r2 = fmod(a2, b2);
    r1 = fmod(a1, b1);

    r4 = fmod(a4, b1);
    r3 = fmod(a3, b1);
    r2 = fmod(a2, b1);
    r1 = fmod(a1, b1);

//    r4 = refract(a4, b4, c1); // needs further work
//    r3 = refract(a3, b3, c1);
//    r2 = refract(a2, b2, c1);


    OutputBuffer buff;

    buff.f4 = r4;
    buff.f3 = r3;
    buff.f2 = r2;
    buff.f1 = r1;

    g_OutputBuffer[DTid.x] = buff;
}
