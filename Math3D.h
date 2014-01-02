/*
    3D Math Library for the Arduino Platform
	Quaternion and vector datatypes and functions
    by Phillip Schmidt
    Dec 2013
    v0.1.0 - beta
	
	quaternion = w, x, y, z (double)
	vector3 = x, y, z (double)
	
	
	(matrix3 = 3x3 matrix - implement later... maybe...)
	(matrix4 = 4x4 matrix - implement later... maybe...)
	
	
*/
 
#ifndef MATH3D_h
#define MATH3D_h
 
#include "Arduino.h"
 
 
class quaternion
{
	public:
    double w, x, y, z;	
	
	quaternion();
	quaternion(double, double, double, double);
	
	//~quaternion();
	
	quaternion& operator+=(const quaternion&);
	quaternion& operator+(const quaternion&) const;
	quaternion operator*(const quaternion&) const;
	
	friend quaternion operator*(quaternion);

	
};

quaternion::quaternion()
{
	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

quaternion::quaternion(double a, double b, double c, double d)
{
	w = a;
	x = b;
	y = c;
	z = d;
}

class vector3
{
	public:
    double x, y, z;	
	
	vector3();
	vector3(double, double, double);
	
	//~vector3();
	
	vector3& operator+=(const vector3&);
	vector3& operator+(const vector3&) const;
	vector3& operator-=(const vector3&);
	vector3& operator-(const vector3&) const;
	vector3 operator*(const vector3&) const;
	
	vector3 operator=(const quaternion&);
	friend vector3 operator*(vector3);
	
};

vector3::vector3(double a, double b, double c)
{
	x = a;
	y = b;
	z = c;
}

vector3::vector3()
{
	x = 0;
	y = 0;
	z = 0;
}

inline quaternion & quaternion::operator+=(const quaternion& b)
{
	quaternion& a = *this;
	a.w += b.w;
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return *this;
}

inline quaternion & quaternion::operator+(const quaternion& b) const
{
	quaternion a = *this;
	a += b;
	return a;
}

inline quaternion quaternion::operator*(const quaternion& b) const // multiply: quat * quat
{
	quaternion a = *this;
	quaternion c;
	c.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;  // 16 mult, 6 add, 6 sub
	c.x = a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y;
	c.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	c.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return c;
}

inline quaternion qConj(quaternion a)
{
	a.w = a.w;
	a.x = -a.x;
	a.y = -a.y;
	a.z = -a.z;
	return a;
}

inline quaternion v2q(const vector3& a)
{
    quaternion b;
    b.w = 0.0;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return(b);
}

inline vector3 q2v(const quaternion& a)
{
	vector3 b;
    b.x = a.x;
    b.y = a.y;
    b.z = a.z;
    return(b);
}

inline vector3 vector3::operator=(const quaternion& b)
{
	vector3& a = *this;
	a.x = b.x;
	a.y = b.y;
	a.z = b.z;
	return *this;
}

inline quaternion operator*(const quaternion& a, const vector3& b) // multiply: quat * vec3
{
	return a * v2q(b);
}

inline quaternion operator*(const vector3& a, const quaternion& b) // multiply: vec3 * quat
{
	return v2q(a) * b;
}

inline quaternion operator*=(quaternion& a, double b) // multiply: quat * double
{
	a.w *= b;
	a.x *= b;
	a.y *= b;
	a.z *= b;
	return a;
}

inline quaternion operator*(quaternion a, double b) // multiply: quat * double
{
	return a *= b;
}

inline quaternion operator*(double b, quaternion a) // multiply: double * quat
{
	return a *= b;
}

inline vector3 & vector3::operator+=(const vector3& b)
{
	vector3& a = *this;
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return *this;
}

inline vector3 & vector3::operator+(const vector3& b) const
{
	vector3 a = *this;
	return a += b;
}

inline vector3 & vector3::operator-=(const vector3& b)
{
	vector3& a = *this;
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return *this;
}

inline vector3 & vector3::operator-(const vector3& b) const
{
	vector3 a = *this;
	return a -= b;
}

inline vector3 operator*=(vector3& a, double b) // multiply: vec3 * double
{
	a.x *= b;
	a.y *= b;
	a.z *= b;
	return a;
}

inline vector3 operator*(vector3 a, double b) // multiply: vec3 * double
{
	return a *= b;
}

inline vector3 operator*(double b, vector3 a) // multiply: double * vec3
{
	return a *= b;
}

inline vector3 vector3::operator*(const vector3& c) const // cross product of 3d vectors
{ 
    vector3 b = *this;
	vector3 a(b.y * c.z - b.z * c.y,
			  b.z * c.x - b.x * c.z,
			  b.x * c.y - b.y * c.x);
    return a;
}

quaternion incQuat(const vector3& w, const unsigned long& t){  // (angular vel vec[rad/s], time interval[us])
	double _t = double(t) * 0.0000005; // time in seconds & divided in half for theta/2 computations

	quaternion a;
	a.w = 1.0 - 0.5*(w.x * w.x + w.y * w.y + w.z * w.z)*_t*_t;
	a.x = w.x * _t;
 	a.y = w.y * _t;
	a.z = w.z * _t;
	
	return a;
}

vector3 grav2sc(const quaternion& a){ // translate 0,0,1 into SC frame
	vector3 c;
	//c.w = 0;  // reduced form of Q Vg Q*
	c.x = 2.0 * a.y * a.w + 2.0 * a.z * a.x;
	c.y = 2.0 * a.z * a.y - 2.0 * a.x * a.w;
	c.z = a.z * a.z - a.y * a.y - a.x * a.x + a.w * a.w;
	return(c);
}

double mag(const quaternion& a){
	return sqrt(a.w*a.w + a.x*a.x + a.y*a.y + a.z*a.z);
}

double mag(const vector3& a){
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

#endif 
