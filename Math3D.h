/*
    3D Math Library for the Arduino Platform
    Quaternion, Vector and Rotation Matrix datatypes and functions
    by Phillip Schmidt
    July 2014, December 2014, June2015
    v3.0.0
    
    
// =========================
//        STRUCTURES
// =========================
        Vec3    x, y, z
        Quat    x, y, z, w            (i, j, k, real)
        M3x3    | a11, a12, a13 |
                | a21, a22, a23 |
                | a31, a32, a33 |
        
        
// =========================
//        BASIC OPERATIONS 
// =========================
        Quat Mul( Quat, Quat )      // multiply: Quat * Quat --16 mult, 6 add, 6 sub  (266us)
        Quat Mul( Quat, Vec3 )      // multiply: Quat * [0,vec3] --12mult, 3 add, 6 sub
        M3x3 Mul( M3x3, M3x3 )      // multiply: M3x3 * M3x3
        Quat Mul( Quat, float )     // multiply: Quat * float
        Quat Mul( float, Quat )     // multiply: float * Quat
        Vec3 Mul( Vec3, float )     // multiply: Vec3 * float
        Vec3 Mul( float, Vec3 )     // multiply: float * Vec3
        Quat Sum( Quat, Quat )      // add two quaternions    
        Vec3 Sum( Vec3, Vec3 )      // add two vectors
        Vec3 Sub( Vec3, Vec3 )      // subtract two vectors
        Vec3 CrossProd( Vec3, Vec3 )    // cross product of 3D vectors
        float DotProd( Quat, Quat )     // dot product of quaternions
        float DotProd( Vec3, Vec3 )     // dot product of 3D vectors
        Vec3 Vector( float, float, float) // 3x float to vector (x, y, z)
        Quat Conj( Quat )           // conjugate quaternion (-x, -y, -z, w)
        Quat Vector2Quat( Vec3 )    // [0,Vec3] to Quat
        Vec3 Quat2Vector( Quat )    // Quat to Vec3

    
// =========================
//        COMPOUND OPERATIONS
// =========================
        float InvSqrt( float )      // returns inverse square root of a float (borrowed from MultiWii v2.4)
        float Magnitude( Quat )     // return magnitude of a quaternion
        float Magnitude( Vec3 )     // return magnitude of a vector
        Vec3 Normalize( Vec3 )      // return normalized vector
        Quat Normalize( Quat )      // return normalized quaternion

        
// =========================
//        SPECIAL FUNCTIONS 
// =========================
        Vec3 Rotate( M3x3, Vec3 )       // vector rotated by matrix (V^ = Matrix * V -- 148us)
        Vec3 Rotate( Vec3, M3x3 )       // vector rotated by matrix (V^ = V * Matrix -- 148us)
        Vec3 Rotate( Vec3, Quat )       // Vector rotated by a Quaternion (matches V^ = V * Matrix)
        Vec3 Rotate( Quat, Vec3 )       // Vector rotated by a Quaternion (matches V^ = Matrix * V)
        Quat Quaternion( Vec3, unsigned long )  // (angular vel vector[rad/s], time interval[us]) -- Small angle approximation
        Quat Quaternion( Vec3 )         // (angle vector[rad])    --Large Rotation Quaternion
        void Quat2Matrix( Quat, M3x3 )  // Quaternion ==> Matrix

        
// =========================
//        DISPLAY FUNCTIONS 
// =========================
        void display(Vec3)      // "X: 0.0000    Y: 0.0000    Z: 0.0000"
        void display(Quat)      // "X: 0.0000    Y: 0.0000    Z: 0.0000    W: 1.0000"
        void display(M3x3)      // "    1.000    0.000    0.000"
                                // "    0.000    1.000    0.000"
                                // "    0.000    0.000    1.000"

*/
 
#ifndef MATH3D_h
#define MATH3D_h
 
#include "Arduino.h"
 
 
 
// =========================
//      STRUCTURES
// =========================

struct Quat
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;    
};

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct M3x3    // Matrix 3x3
{
    float a11 = 1.0f; float a12 = 0.0f; float a13 = 0.0f;    
    float a21 = 0.0f; float a22 = 1.0f; float a23 = 0.0f;
    float a31 = 0.0f; float a32 = 0.0f; float a33 = 1.0f;
};




// =========================
//      BASIC OPERATIONS 
// =========================

inline Quat Mul(const Quat& lhs, const Quat& rhs)  // multiply: Quat * Quat --16 mult, 6 add, 6 sub  (266us)
{
    Quat a;
    a.x = lhs.w * rhs.x + lhs.z * rhs.y - lhs.y * rhs.z + lhs.x * rhs.w;  
    a.y = lhs.w * rhs.y + lhs.x * rhs.z + lhs.y * rhs.w - lhs.z * rhs.x;
    a.z = lhs.y * rhs.x - lhs.x * rhs.y + lhs.w * rhs.z + lhs.z * rhs.w;
    a.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
    return a;
}

inline Quat Mul(const Quat& lhs, const Vec3& rhs)  // multiply: Quat * [0,vec3] --12mult, 3 add, 6 sub
{
    Quat a;
    a.x =  lhs.w * rhs.x + lhs.z * rhs.y - lhs.y * rhs.z;  
    a.y =  lhs.w * rhs.y + lhs.x * rhs.z - lhs.z * rhs.x;
    a.z =  lhs.y * rhs.x - lhs.x * rhs.y + lhs.w * rhs.z;
    a.w = -lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
    return a;
}

inline M3x3 Mul(const M3x3& l, const M3x3& r)  // multiply: Q3x3 * Q3x3 --27mult, 18 add
{
    M3x3 a;
    
    // *** ROW 1 ***
    a.a11 =  l.a11 * r.a11 + l.a12 * r.a21 + l.a13 * r.a31;  
    a.a12 =  l.a11 * r.a12 + l.a12 * r.a22 + l.a13 * r.a32;
    a.a13 =  l.a11 * r.a13 + l.a12 * r.a23 + l.a13 * r.a33;
    
    // *** ROW 2 ***
    a.a21 =  l.a21 * r.a11 + l.a22 * r.a21 + l.a23 * r.a31;  
    a.a22 =  l.a21 * r.a12 + l.a22 * r.a22 + l.a23 * r.a32;
    a.a23 =  l.a21 * r.a13 + l.a22 * r.a23 + l.a23 * r.a33;    
    
    // *** ROW 3 ***
    a.a31 =  l.a31 * r.a11 + l.a32 * r.a21 + l.a33 * r.a31;  
    a.a32 =  l.a31 * r.a12 + l.a32 * r.a22 + l.a33 * r.a32;
    a.a33 =  l.a31 * r.a13 + l.a32 * r.a23 + l.a33 * r.a33;    

    return a;
}

inline Quat Mul(Quat a, const float& b) // multiply: quat * float
{
    a.w *= b;
    a.x *= b;
    a.y *= b;
    a.z *= b;
    return a;
}

inline Quat Mul(const float& b, const Quat& a) // multiply: float * quat
{
    return Mul(a, b);
}

inline Vec3 Mul(Vec3 a, const float& b) // multiply: vec3 * float
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    return a;
}

inline Vec3 Mul(const float& b, const Vec3& a) // multiply: float * vec3
{
    return Mul(a, b);
}

inline Quat Sum(Quat a, const Quat& b)
{
    a.w += b.w;
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}


inline Vec3 Sum(Vec3 a, const Vec3& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vec3 Sub(Vec3 a, const Vec3& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Vec3 CrossProd(const Vec3& L, const Vec3& R) // cross product of 3D vectors 
{ 
    Vec3 a;
    a.x = L.y * R.z - L.z * R.y;
    a.y = L.z * R.x - L.x * R.z;
    a.z = L.x * R.y - L.y * R.x;
    return a;    // 76us
}

inline float DotProd(const Quat& L, const Quat& R)
{
    return L.w * R.w + L.x * R.x + L.y * R.y + L.z * R.z;
}

inline float DotProd(const Vec3& L, const Vec3& R)
{
    return L.x * R.x + L.y * R.y + L.z * R.z;
}

inline Vec3 Vector(const float& x, const float& y, const float& z) // 3x float to vector
{ 
    Vec3 a;
    a.x = x;
    a.y = y;
    a.z = z;
    return a;
}

inline Quat Conj(Quat a)
{
    //a.w = a.w;
    a.x = -a.x;
    a.y = -a.y;
    a.z = -a.z;
    return a;
}

inline Quat Vector2Quat(const Vec3& a)    // [0,Vec3] to Quat
{
    Quat b;
    b.w = 0.0f;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return b;
}

inline Vec3 Quat2Vector(const Quat& a)    // Quat to Vec3
{
    Vec3 b;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return b;
}


// =========================
//  COMPOUND OPERATIONS
// =========================

inline float InvSqrt(const float& x)    // borrowed from MultiWii v2.4
{ 
    union{  
        int32_t i;  
        float    f; 
    } conv; 
    conv.f = x; 
    conv.i = 0x5f1ffff9 - (conv.i >> 1); 
    return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}

inline float InvSqrtFast(const float& x)    // use when already very near 1.0
{ 
    return (3.0f - x) * 0.5f;
}

inline float Magnitude(const Quat& a)
{
    return sqrt(a.w*a.w + a.x*a.x + a.y*a.y + a.z*a.z);
}

inline float Magnitude(const Vec3& a)
{
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

inline Vec3 Normalize(const Vec3& a)
{
    return Mul(a, InvSqrt(a.x*a.x + a.y*a.y + a.z*a.z));
}

inline Vec3 NormalizeFast(const Vec3& a)
{
    return Mul(a, InvSqrtFast(a.x*a.x + a.y*a.y + a.z*a.z));
}

inline Quat Normalize(const Quat& a)  
{
    return Mul(a, InvSqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));    // 148us
    //return Mul(a, 1.0f / sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));  // 164us
}

inline Quat NormalizeFast(const Quat& a)  // 120us
{
    return Mul(a, InvSqrtFast(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));
}


// =========================
//  SPECIAL FUNCTIONS 
// =========================

inline Vec3 Rotate(const M3x3& L, const Vec3& R) // vector rotated by matrix (V^ = Matrix * V -- 148us)
{ 
    Vec3 a;
    a.x = L.a11 * R.x + L.a12 * R.y + L.a13 * R.z;
    a.y = L.a21 * R.x + L.a22 * R.y + L.a23 * R.z;
    a.z = L.a31 * R.x + L.a32 * R.y + L.a33 * R.z;
    return a;
}

inline Vec3 Rotate(const Vec3& L, const M3x3& R) // vector rotated by matrix (V^ = V * Matrix -- 148us)
{ 
    Vec3 a;
    a.x = R.a11 * L.x + R.a21 * L.y + R.a31 * L.z;
    a.y = R.a12 * L.x + R.a22 * L.y + R.a32 * L.z;
    a.z = R.a13 * L.x + R.a23 * L.y + R.a33 * L.z;
    return a;
}


inline Vec3 Rotate(const Vec3& v, const Quat& q)    // Vector rotated by a Quaternion (matches V^ = V * Matrix)
{    
    // v + 2*r X (r X v + q.w*v) -- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
    Vec3 r;         // vector r is the three imaginary coefficients of quaternion q
    r.x = -q.x;    // reverse signs to change direction of rotation
    r.y = -q.y;
    r.z = -q.z;
    return Sum(v, CrossProd(Sum(r, r), Sum(CrossProd(r, v), Mul(q.w, v)))); // 296us 
    
    //return Quat2Vector(Mul(Mul(q, v), Conj(q)));        // 460us 
}

inline Vec3 Rotate(const Quat& q, const Vec3& v)    // Vector rotated by a Quaternion (matches V^ = Matrix * V)
{
    // v + 2*r X (r X v + q.w*v) -- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
    Vec3 r; 
    r.x = q.x;
    r.y = q.y;
    r.z = q.z;
    return Sum(v, CrossProd(Sum(r, r), Sum(CrossProd(r, v), Mul(q.w, v))));    // 296us    

    //return Quat2Vector(Mul(Mul(Conj(q), v), q));        // 460us
}


inline Quat Quaternion(const Vec3& w, const unsigned long& t)    // (angular vel vector[rad/s], time interval[us]) -- Small angle approximation
{  
    float dT_2;
    Quat  a;    
    
    dT_2 = float(t) * 0.0000005f; // time in seconds & divided in half for theta/2 computations
    a.x  = w.x * dT_2;
    a.y  = w.y * dT_2;
    a.z  = w.z * dT_2;
    a.w  = 1.0f - 0.5f * (a.x * a.x + a.y * a.y + a.z * a.z);
    return a;    // time = 116us + mult = 362us        (REF: RotMatrix = 588us)
}


inline Quat Quaternion(const Vec3& w)    // (angle vector[rad])    --Large Rotation Quaternion
{  
    Quat a;
    
    float vMag = Magnitude(w);
    
    if(vMag < 1.0e-6) return a;  // terminate early if magnitude is negligible (a is zero rotation quat 0,0,0,1)
    
    float theta_2 = vMag * 0.5f;            // rotation angle divided by 2
    float Sin_Mag = sin(theta_2) / vMag;    // computation minimization
    
    a.x = w.x * Sin_Mag;
    a.y = w.y * Sin_Mag;
    a.z = w.z * Sin_Mag;
    a.w = cos(theta_2);
    
    return a;        // time = 390us + mult  = 636
}


inline void Quat2Matrix(const Quat& q, M3x3& m) // Quaternion ==> Matrix
{
    // pre-compute to reduce multiplies (10xMult, 18xAdd/Sub -- 248us total)
    float ww  = q.w * q.w;
    float xx  = q.x * q.x;
    float yy  = q.y * q.y;
    float zz  = q.z * q.z;

    float w2  = q.w + q.w;
    float wx2 =  w2 * q.x;
    float wy2 =  w2 * q.y;
    float wz2 =  w2 * q.z;

    float x2  = q.x + q.x;
    float xy2 =  x2 * q.y;
    float xz2 =  x2 * q.z;

    float yz2 = (q.y + q.y) * q.z;

    // Diagonal
    m.a11 = ww + xx - yy - zz;
    m.a22 = ww - xx + yy - zz;
    m.a33 = ww - xx - yy + zz;

    // Lower Left
    m.a21 = xy2 + wz2;
    m.a31 = xz2 - wy2;
    m.a32 = yz2 + wx2;

    // Upper Right
    m.a12 = xy2 - wz2;
    m.a13 = xz2 + wy2;
    m.a23 = yz2 - wx2;
}


inline Vec3 VerticalInBody(const Quat& q) // Quaternion ==> vertical unit vector rotated from earth to body frame
{
    // Based on Quat ==> M3x3 method
    Vec3 a;

    float w2  = q.w + q.w;
    float wx2 =  w2 * q.x; //
    float wy2 =  w2 * q.y; //

    float xz2 = (q.x + q.x) * q.z; //

    float yz2 = (q.y + q.y) * q.z; //

    a.x = xz2 - wy2;
    a.y = yz2 + wx2;

    return a;

}


// FAST ATAN2 Approximation from: https://gist.github.com/volkansalma/2972237
// |error| < 0.005
float atan2fast( float y, float x )
{
    if ( x == 0.0f )
    {
        if ( y > 0.0f ) return 1.5707963f;
        if ( y == 0.0f ) return 0.0f;
        return -1.5707963f;
    }
    float atan;
    float z = y/x;
    if ( fabs( z ) < 1.0f )
    {
        atan = z/(1.0f + 0.28f*z*z);
        if ( x < 0.0f )
        {
            if ( y < 0.0f ) return atan - 3.14159265f;
            return atan + 3.14159265f;
        }
    }
    else
    {
        atan = 1.5707963f - z/(z*z + 0.28f);
        if ( y < 0.0f ) return atan - 3.14159265f;
    }
    return atan;
}

////// YAW -> PITCH -> ROLL = YPR -- Rotation Order
inline float YPR_Yaw(const Quat& q)
{
    return atan2fast(q.y * q.z + q.w * q.x, 0.5f - (q.x * q.x + q.y * q.y));    // Yaw
}

inline float YPR_Pitch(const Quat& q)
{
    return asin(2.0f * (q.x * q.z - q.w * q.y));;    // Pitch
}

inline float YPR_Roll(const Quat& q)
{
    return atan2fast(q.x * q.y + q.w * q.z, 0.5f - (q.y * q.y + q.z * q.z));    // Roll
}

Vec3 YawPitchRoll(const Quat& q) // Tait-Bryan Angles - 440us
{
    Vec3 ypr;
    
    ypr.z = YPR_Yaw(q);        // YAW
    ypr.y = YPR_Pitch(q);    // PITCH
    ypr.x = YPR_Roll(q);    // ROLL

    return ypr;
}

////// ROLL -> PITCH -> YAW = RPY -- Rotation Order
inline float RPY_Roll(const Quat& q)
{
    return atan2fast(q.w * q.x + q.y * q.z,  0.5f - (q.x * q.x + q.y * q.y));    // Roll
}

inline float RPY_Pitch(const Quat& q)
{
    return asin(2.0f * (q.w * q.y - q.z * q.x));    // Pitch
}

inline float RPY_Yaw(const Quat& q)
{
    return atan2fast(q.w * q.z + q.x * q.y,  0.5f - (q.y * q.y  + q.z * q.z));    // Yaw
}

Vec3 RollPitchYaw(const Quat& q)
{
    Vec3 ypr;

    ypr.x = RPY_Roll(q);    // Roll
    ypr.y = RPY_Pitch(q);    // Pitch
    ypr.z = RPY_Yaw(q);        // Yaw
    
    return ypr;
}


inline float veryFastSin(float x) // from: http://lab.polygonal.de/?p=205  (60us faster, ~0.05 error)
{
    float s;

    //always wrap input angle to -PI..PI
    if (x < -3.14159265) x += 6.28318531;
    else
    if (x >  3.14159265) x -= 6.28318531;

    //compute sine
    if (x < 0)
        s = 1.27323954 * x + .405284735 * x * x;
    else
        s = 1.27323954 * x - 0.405284735 * x * x;

    return s;
}

inline float veryFastCos(float x)
{  
  //compute cosine: sin(x + PI/2) = cos(x)
  return veryFastSin(x + 1.57079632f);

}

inline float fastSin(float x) // from: http://lab.polygonal.de/?p=205  (24us faster, ~0.0004 error)
{
    float s;

    //always wrap input angle to -PI..PI
    if (x < -3.14159265f) x += 6.28318531f;
    else
    if (x >  3.14159265f) x -= 6.28318531f;

    //compute sine
    if (x < 0.0f)
    {
         s = 1.27323954 * x + .405284735 * x * x;
        
        if (s < 0)
            s = .225 * (s * -s - s) + s;
        else
            s = .225 * (s *  s - s) + s;
    }
    else
    {
        s = 1.27323954 * x - 0.405284735 * x * x;
     
        if (s < 0)
            s = .225 * (s * -s - s) + s;
        else
            s = .225 * (s *  s - s) + s;
    }
    return s;

}

inline float fastCos(float x)
{  
  //compute cosine: sin(x + PI/2) = cos(x)
  return fastSin(x + 1.57079632f);

}



// =========================
//      DISPLAY FUNCTIONS 
// =========================

void display(const Vec3& v)
{
    #ifdef SERIAL_PORT
        String outputBuffer;
        
        outputBuffer  = "X: ";
        outputBuffer += String(v.x, 4);
        outputBuffer += "    Y: ";
        outputBuffer += String(v.y, 4);
        outputBuffer += "    Z: ";
        outputBuffer += String(v.z, 4);
        outputBuffer += '\n';
    
        SERIAL_PORT.print(outputBuffer);
    #endif         
}

void display(const Quat& q)
{
    #ifdef SERIAL_PORT
        String outputBuffer;
        
        outputBuffer  = "X: ";
        outputBuffer += String(q.x, 4);
        outputBuffer += "  Y: ";
        outputBuffer += String(q.y, 4);
        outputBuffer += "  Z: ";
        outputBuffer += String(q.z, 4);
        outputBuffer += "  W: ";
        outputBuffer += String(q.w, 4);
        outputBuffer += '\n';
    
        SERIAL_PORT.print(outputBuffer);
    #endif         
}

void display(const M3x3& m)
{
    #ifdef SERIAL_PORT
        String outputBuffer;
        
        outputBuffer  = String(m.a11, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a12, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a13, 3);
        outputBuffer += '\n';
        
        outputBuffer += String(m.a21, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a22, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a23, 3);
        outputBuffer += '\n';

        outputBuffer += String(m.a31, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a32, 3);
        outputBuffer += " ";
        outputBuffer += String(m.a33, 3);
        outputBuffer += '\n';
    
        SERIAL_PORT.print(outputBuffer);
    #endif     
}

#endif 
