//
//  Quarternion.hpp
//  Quarternion
//
//  Created by KUAN LU on 16/5/1.
//  Copyright © 2016年 KUAN LU. All rights reserved.
//

#ifndef Quaternion_hpp
#define Quaternion_hpp
#define eps 0.00001f
#define PI 3.14159265f

#include <cmath>
#include <iostream>
#include <fstream>
#include <GL\glut.h>

using namespace std;

class Vec3
{
public:
	GLfloat x = 0;
	GLfloat y = 0;
	GLfloat z = 0;
	Vec3(GLfloat x, GLfloat y, GLfloat z)
	{
		this->x=x;
		this->y=y;
		this->z=z;
	}
	void normalize()
	{
		GLfloat mag2 = x*x + y*y + z*z;
		if (mag2 != 0 && fabs(mag2 - 1.0) > FLT_EPSILON)
		{
			GLfloat mag = sqrtf(mag2);
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
	GLfloat operator[](const int &index)
	{
		switch (index) {
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;
		case 2:
			return z;
			break;
		default:
			throw exception();
			break;
		}
	}
	void print()
	{
		cout << "[" << x << " " << y << " " << z << "]"<< endl;
	}
};

class Quaternion
{
public:
	GLfloat x, y, z, w;
    Quaternion(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
    {
        this->x=x;
        this->y=y;
        this->z=z;
        this->w=w;
    }
	Quaternion(const Vec3 v3, GLfloat angle)
	{
		GLfloat sinAngle=0;
		angle *= 0.5f;
		Vec3 vn(v3);
		vn.normalize();

		sinAngle = sinf(angle);

		x=vn[0]*sinAngle;
		y=vn[1]*sinAngle;
		z=vn[2]*sinAngle;
		w = cosf(angle);
	}
	Quaternion(GLfloat pitch, GLfloat yaw, GLfloat roll)
	{
		GLfloat p = pitch * PI / 2.0f;
		GLfloat y = yaw  * PI / 2.0f;
		GLfloat r = roll*PI / 2.0f;

		float sinp= sin(p);
		float siny= sin(y);
		float sinr= sin(r);
		float cosp= cos(p);
		float cosy= cos(y);
		float cosr= cos(r);
		
		this->x = sinr*cosp*cosy - cosr*sinp*siny;
		this->y = cosr*sinp*cosy + sinr*cosp*siny;
		this->z = cosr*cosp*siny - sinr*sinp*cosy;
		this->w = cosr*cosp*cosy + sinr*sinp*siny;

		normalize();
	}
    void normalize()
    {
        GLfloat mag2 = x*x+y*y+z*z+w*w;
        if(mag2!=0.f&&fabs(mag2-1.0)>eps)
        {
            GLfloat mag=sqrt(mag2);
            x/=mag;
            y/=mag;
            z/=mag;
            w/=mag;
        }
    }
    Quaternion conjugate()
    {
        return Quaternion(-x,-y,-z,w);
    }

	GLfloat *getMatrix()
	{
		normalize();
		GLfloat *Matrix=new GLfloat[16];
		
		float x2 = x*x;
		float y2 = y*y;
		float z2 = z*z;
		float xy = x*y;
		float xz = x*z;
		float yz = y*z;
		float wx = w*x;
		float wy = w*y;
		float wz = w*z;

		Matrix[0] = 1.f - 2.f * (y2 + z2);
		Matrix[1] = 2.f*(xy - wz);
		Matrix[2] = 2.f*(xz + wy);
		Matrix[3] = 0.f;
		Matrix[4] = 2.f*(xy + wz);
		Matrix[5] = 1.f - 2.f*(x2 + z2);
		Matrix[6] = 2.f*(yz - wx);
		Matrix[7] = 0.f;
		Matrix[8] = 2.f*(xz - wy);
		Matrix[9] = 2.f*(yz + wx);
		Matrix[10] = 1.f - 2.f*(x2 + y2);
		Matrix[11] = 0.f;
		Matrix[12] = 0.f;
		Matrix[13] = 0.f;
		Matrix[14] = 0.f;
		Matrix[15] = 1.f;

		return Matrix;
	}

	void getAxisAngle(Vec3 *axis, GLfloat *angle)
	{
		GLfloat scale = sqrtf(x*x + y*y + z*z);
		axis->x = x / scale;
		axis->y = y / scale;
		axis->z = z / scale;
		*angle = acosf(w)*2.f;
	}

    Quaternion operator +(const Quaternion &right)const
    {
        return Quaternion(x+right.x, y+right.y, z+right.z, w+right.w);
    }
    Quaternion operator *(Quaternion &right)
    {
       GLfloat tmp[4];
        tmp[0]= right[3]*(*this)[0]+right[2]*(*this)[1]-right[1]*(*this)[2]+right[0]*(*this)[3];
        tmp[1]=-right[2]*(*this)[0]+right[3]*(*this)[1]+right[0]*(*this)[2]+right[1]*(*this)[3];
        tmp[2]= right[1]*(*this)[0]-right[0]*(*this)[1]+right[3]*(*this)[2]+right[2]*(*this)[3];
        tmp[3]=-right[0]*(*this)[0]-right[1]*(*this)[1]-right[2]*(*this)[2]+right[3]*(*this)[3];
        return Quaternion(tmp[0],tmp[1],tmp[2],tmp[3]);
	}

	Quaternion operator *(GLfloat scale)
	{
		return Quaternion(x*scale, y*scale, z*scale, w*scale);
	}

	Vec3 operator *(Vec3 &right)
	{
		Vec3 tmp(right);
		tmp.normalize();
		Quaternion vecQuart(tmp[0], tmp[1], tmp[2], 0.f);
		Quaternion resultQuart = vecQuart*conjugate();
		resultQuart = (*this)*resultQuart;

		return Vec3(resultQuart[0], resultQuart[1], resultQuart[2]);
	}
    GLfloat operator[](const int &index)
    {
        switch (index) {
            case 0:
                return x;
                break;
            case 1:
                return y;
                break;
            case 2:
                return z;
                break;
            case 3:
                return w;
                break;
            default:
                throw exception();
                break;
        }
    }
    void print()
    {
        cout<<"["<<x<<" "<<y<<" "<<z<<" "<<w<<"]"<<endl;
		//GLfloat *Matrix=getMatrix();
		//cout<<Matrix[0]<<" "<<Matrix[1]<<" "<<Matrix[2]<<" "<<Matrix[3]<<endl;  
		//cout<<Matrix[4]<<" "<<Matrix[5]<<" "<<Matrix[6]<<" "<<Matrix[7]<<endl;
		//cout<<Matrix[8]<<" "<<Matrix[9]<<" "<<Matrix[10]<<" "<<Matrix[11]<<endl;
		//cout<<Matrix[12]<<" "<<Matrix[13]<<" "<<Matrix[14]<<" "<<Matrix[15]<<endl;
		//delete Matrix;
    }
};

#endif /* Quaternion_hpp */
