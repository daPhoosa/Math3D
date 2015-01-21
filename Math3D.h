/*
    3D Math Library for the Arduino Platform
	Quaternion and vector datatypes and functions
    by Phillip Schmidt
    July 2014, December 2014
    v2.0.0
	
	
	STRUCTURES
		Vec3 - 	x, y, z
		Quat - 	x, y, z, w			(i, j, k, real)
		M3x3 -	a11, a12, a13
				a21, a22, a23
				a31, a32, a33
		
		
	BASIC OPERATIONS
		Quat QuatMultiply(Quat, Quat)			// combine rotations encoded in quaternion form
		Quat QuatMultiply(Quat, Vec3) 		//	used during vector translation
		Quat QuatConj(Quat)						// represents a rotation opposite of the original
		Quat Vector2Quat(Vec3)					// convert a 3D vector to a quaternion (add component w=0)
		Quat QuatScale(Quat, float)			// multiply a quaternion be a scaler
		
		Vec3 Quat2Vector(Quat)					// convert a quaternion to a 3D vector (truncate w component)
		Vec3 VecScale(Vec3, float)				// multiply a 3D vector by a scaler
		Vec3 VecAdd(Vec3, Vec3) 				// sum the components of two 3D vectors
		Vec3 VecDiff(Vec3, Vec3) 				// subtract the components of two vectors
		Vec3 VecCross(Vec3, Vec3) 			// cross product of two vectors

		
	COMPOUND OPERATIONS
		float InvSqrt (float)						// fast inverse square root using
		float Magnitude(Quat)						// quaternion magnitude
		float Magnitude(Vec3)						// 3D vector magnitude
		Vec3 Normalize(Vec3)					// 3D vector normalize
		Quat Normalize(Quat)						// quaternion normalize
		
		
	SPECIAL FUNCTIONS
		Vec3 Vec2Vehicle(Quat, Vec3)		// translate a 3D vector from the inertial frame to the vehicle frame
		Vec3 Vec2Inertial(Quat, Vec3)		// translate a 3D vector from the vehicle frame to the inertial frame
		Quat QuatIntegrate(Quat, Vec3, unsigned long)	// use the current orientation quaternion, 3D rotation rate vector, and time interval to compute the current orientation
		Vec3 AccelComp(Quat, Vec3)			// compute a 3D vector that describes the rotation required to correct the discrepancy between estimated vertical and the gravitational vertical
		Vec3 MagComp(Quat, Vec3)			// compute a 3D vector that describes the rotation required to correct the discrepancy between estimated north and the magnetic north
	
*/
 
#ifndef MATH3D_h
#define MATH3D_h
 
#include "Arduino.h"
 
 
 
// =========================
// 			CLASSES
// =========================

class Quat
{
	public:
    float w, x, y, z;	
	
	Quat();
	Quat(float, float, float, float);
	
	//~Quat();
	
	Quat& operator+=(const Quat&);
	Quat& operator+(const Quat&) const;
	Quat operator*(const Quat&) const;
	
	friend Quat operator*(Quat);

};

class M3x3	// Matrix 3x3
{
	public:
    float a11, a12, a13;	
	float a21, a22, a23;
	float a31, a32, a33;
	
	M3x3();
	
	//~Vec3();
	
	M3x3& operator+=(const M3x3&);
	M3x3& operator+(const M3x3&) const;
	M3x3  operator*(const M3x3&) const;
	
	friend M3x3 operator*(M3x3);		// for 
	
};

class Vec3
{
	public:
    float x, y, z;	
	
	Vec3();
	Vec3(float, float, float);
	
	
	//~Vec3();
	
	Vec3& operator+=(const Vec3&);
	Vec3& operator+(const Vec3&) const;
	Vec3& operator-=(const Vec3&);
	Vec3& operator-(const Vec3&) const;
	Vec3 operator*(const Vec3&) const;
	
	Vec3 operator=(const Quat&);
	friend Vec3 operator*(Vec3);
	
};







// =========================
// 			CONSTRUCTORS 
// =========================

Quat::Quat(float a, float b, float c, float d)
{
	w = a;
	x = b;
	y = c;
	z = d;
}

Quat::Quat()
{
	w = 1.0f;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

M3x3::M3x3(	const float& a, const float& b, const float& c, 
			const float& d, const float& e, const float& f, 
			const float& g, const float& h, const float& i)
{
	a11 = a; a12 = b; a13 = c;
	a21 = d; a22 = e; a23 =	f;
	a31 = g; a32 = h; a33 = i;
}

M3x3::M3x3()
{
	a11 = a22 = a33 = 1.0f;
	a12 = a13 = a21 = a23 =	a31 = a32 = 0.0f;
}

Vec3::Vec3(float a, float b, float c)
{
	x = a;
	y = b;
	z = c;
}

Vec3::Vec3()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}


// =========================
// 		BASIC OPERATIONS 
// =========================

inline Quat Quat::operator*(const Quat& rhs) const // multiply: Quat * Quat --16 mult, 6 add, 6 sub  (266us)
{
	Quat lhs = *this;	// 
	Quat a;
	a.x = lhs.w * rhs.x + lhs.z * rhs.y - lhs.y * rhs.z + lhs.x * rhs.w;  
	a.y = lhs.w * rhs.y + lhs.x * rhs.z + lhs.y * rhs.w - lhs.z * rhs.x;
	a.z = lhs.y * rhs.x - lhs.x * rhs.y + lhs.w * rhs.z + lhs.z * rhs.w;
	a.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	return a;
}

inline Quat operator*(const Quat& lhs, const Vec3& rhs)
{
	Quat a;
	a.x =  lhs.w * rhs.x + lhs.z * rhs.y - lhs.y * rhs.z;  
	a.y =  lhs.w * rhs.y + lhs.x * rhs.z - lhs.z * rhs.x;
	a.z =  lhs.y * rhs.x - lhs.x * rhs.y + lhs.w * rhs.z;
	a.w = -lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	return a;
}

inline Quat operator*=(Quat& a, float b) // multiply: quat * float
{
	a.w *= b;
	a.x *= b;
	a.y *= b;
	a.z *= b;
	return a;
}

inline Quat operator*(Quat a, float b) // multiply: quat * float
{
	return a *= b;
}

inline Quat operator*(float b, Quat a) // multiply: float * quat
{
	return a *= b;
}

inline Quat & Quat::operator+=(const Quat& b)
{
	Quat& a = *this;
	a.w += b.w;
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return *this;
}

inline Quat & Quat::operator+(const Quat& b) const
{
	Quat a = *this;
	a += b;
	return a;
}


inline Vec3 & Vec3::operator+=(const Vec3& b)
{
	Vec3& a = *this;
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return *this;
}

inline Vec3 & Vec3::operator+(const Vec3& b) const
{
	Vec3 a = *this;
	return a += b;
}

inline Vec3 & Vec3::operator-=(const Vec3& b)
{
	Vec3& a = *this;
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return *this;
}

inline Vec3 & Vec3::operator-(const Vec3& b) const
{
	Vec3 a = *this;
	return a -= b;
}

inline Vec3 operator*=(Vec3& a, const float& b) // multiply: vec3 * float
{
	a.x *= b;
	a.y *= b;
	a.z *= b;
	return a;
}

inline Vec3 operator*(Vec3 a, const float& b) // multiply: vec3 * float
{
	return a *= b;
}

inline Vec3 operator*(const float& b, Vec3 a) // multiply: float * vec3
{
	return a *= b;
}


inline Vec3 Vec3::operator*(const Vec3& R) const // cross product of 3d vectors
{ 
    Vec3 L = *this;
	Vec3 a(	L.y * R.z - L.z * R.y,
			L.z * R.x - L.x * R.z,
			L.x * R.y - L.y * R.x);
    return a;
}

inline Vec3 Vector(const float& x, const float& y, const float& z)
{
	Vec3 v(x, y, z);
	return v;
}

inline Quat conj(Quat a)
{
	//a.w = a.w;
	a.x = -a.x;
	a.y = -a.y;
	a.z = -a.z;
	return a;
}

inline Quat Vector2Quat(const Vec3& a)	// Vec3 to Quat
{
    Quat b;
    b.w = 0.0f;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return(b);
}

inline Vec3 Quat2Vector(const Quat& a)	// Quat to Vec3
{
	Vec3 b;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return(b);
}


// =========================
//		COMPOUND OPERATIONS
// =========================

inline float InvSqrt (float x)	// the "Quake" inverse square root -- 
{ 
	union{  
		int32_t i;  
		float    f; 
	} conv; 
	conv.f = x; 
	conv.i = 0x5f3759df - (conv.i >> 1); 
	return 0.5f * conv.f * (3.0f - x * conv.f * conv.f); //
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
	return a * InvSqrt (a.x*a.x + a.y*a.y + a.z*a.z);
}

inline Quat Normalize(const Quat& a)
{
	return a * InvSqrt (a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
}



// =========================
//		SPECIAL FUNCTIONS 
// =========================

inline Vec3 Rotate(const M3x3& L, const Vec3& R) // vector rotated by matrix (V^ = Matrix * V -- 148us)
{ 
	Vec3 a(	L.a11 * R.x + L.a12 * R.y + L.a13 * R.z,
			L.a21 * R.x + L.a22 * R.y + L.a23 * R.z,
			L.a31 * R.x + L.a32 * R.y + L.a33 * R.z);
    return a;
}

inline Vec3 Rotate(const Vec3& L, const M3x3& R) // vector rotated by matrix (V^ = V * Matrix -- 148us)
{ 
	Vec3 a(	R.a11 * L.x + R.a21 * L.y + R.a31 * L.z,
			R.a12 * L.x + R.a22 * L.y + R.a32 * L.z,
			R.a13 * L.x + R.a23 * L.y + R.a33 * L.z);
    return a;
}

inline Vec3 Rotate(const Vec3& v, const Quat& q)	// Vector rotated by a Quaternion (matches V^ = V * Matrix)
{	
	Vec3 r(-q.x, -q.y, -q.z);								// v + 2*r X (r X V + q.w*v)
	return v + (r + r) * (r * v + v * q.w);					// 15mult + 12add -- 268us
	
	//v = Quat2Vector((q * v) * conj(q));					// 460us
	//return v;
}

inline Vec3 Rotate(const Quat& q, const Vec3& v)	// Vector rotated by a Quaternion (matches V^ = Matrix * V)
{	
	Vec3 r(q.x, q.y, q.z);								// v + 2*r X (r X V + q.w*v)
	return v + ((r + r) * ((r * v) + (v * q.w)));				// 15mult + 12add -- 268us
	
	//v = Quat2Vector((conj(q) * v) * q);					// 460us
	//return v;
}


inline Quat Quaternion(const Vec3& w, const unsigned long& t)	// (angular vel vector[rad/s], time interval[us])
{  
	float dT_2 = float(t) * 0.0000005f; // time in seconds & divided in half for theta/2 computations
	Quat a;
	a.x = w.x * dT_2;
 	a.y = w.y * dT_2;
	a.z = w.z * dT_2;
	a.w = 1.0 - 0.5 * (a.x * a.x + a.y * a.y + a.z * a.z);
	return a;	// time = 116us + mult = 362us		(REF: RotMatrix = 588us)
}


inline Quat Quaternion(const Vec3& w)	// (angle vector[rad])	--Large Rotation Quaternion
{  
	
	float vMag = Magnitude(w);
	float theta_2 = vMag * 0.5f;	// rotation angle divided by 2
	float Sin_Mag = sin(theta_2) / vMag;			// computation minimization
	
	Quat a;
	a.x = w.x * Sin_Mag;
 	a.y = w.y * Sin_Mag;
	a.z = w.z * Sin_Mag;
	a.w = cos(theta_2);
	return a;		// time = 390us + mult  = 636
}

inline void Quat2Matrix(const Quat& q, M3x3& m)
{
	float static ww, xx,  yy,  zz;
	float static w2, wx2, wy2, wz2;
	float static x2, xy2, xz2, yz2;
	
	// pre-compute to reduce multiplies (10xMult, 18xAdd/Sub -- 248us)
	ww  = q.w * q.w;
	xx  = q.x * q.x;
	yy  = q.y * q.y;
	zz  = q.z * q.z;

	w2  = q.w + q.w;
	wx2 =  w2 * q.x;
	wy2 =  w2 * q.y;
	wz2 =  w2 * q.z;

	x2  = q.x + q.x;
	xy2 =  x2 * q.y;
	xz2 =  x2 * q.z;

	yz2 = (q.y + q.y) * q.z;

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


// =========================
// 		DISPLAY FUNCTIONS 
// =========================

void display(const Vec3& v)
{
	Serial.print("X: ");
	Serial.print(v.x, 3);
	Serial.print("   ");
	
	Serial.print("Y: ");
	Serial.print(v.y, 3);
	Serial.print("   ");
	
	Serial.print("Z: ");
	Serial.println(v.z, 3);

}

void display(const Quat& q)
{
	Serial.print("W: ");
	Serial.print(q.w, 3);
	Serial.print("   ");
	
	Serial.print("X: ");
	Serial.print(q.x, 3);
	Serial.print("   ");
	
	Serial.print("Y: ");
	Serial.print(q.y, 3);
	Serial.print("   ");
	
	Serial.print("Z: ");
	Serial.println(q.z, 3);

}

void display(const M3x3& m)
{
	Serial.print("   ");
	Serial.print(m.a11, 3);
	Serial.print("   ");
	
	Serial.print(m.a12, 3);
	Serial.print("   ");
	
	Serial.println(m.a13, 3);
	
	Serial.print("   ");
	Serial.print(m.a21, 3);
	Serial.print("   ");
	
	Serial.print(m.a22, 3);
	Serial.print("   ");
	
	Serial.println(m.a23, 3);

	Serial.print("   ");
	Serial.print(m.a31, 3);
	Serial.print("   ");
	
	Serial.print(m.a32, 3);
	Serial.print("   ");
	
	Serial.println(m.a33, 3);	
}

#endif 
