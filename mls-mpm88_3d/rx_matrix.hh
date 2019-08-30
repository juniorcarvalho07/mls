/*! @file rx_matrix.h
	
	@brief �x�N�g���E�s�񃉃C�u���� - �s���N���X
 
	@author 
	@date  
*/

#ifndef _MATRIX_H_
#define _MATRIX_H_

//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <memory.h>
#include "rx_utility.h"

//-----------------------------------------------------------------------------
// ���`
//-----------------------------------------------------------------------------
//#define float double 

#define QUATERNION_NORMALIZATION_THRESHOLD  64

#ifndef RX_INV_PI
#define RX_INV_PI 0.318309886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  (float)(57.2957795130823208767981548141052)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (float)(0.0174532925199432957692369076848861)
#endif

#ifndef RX_EPSILON
#define RX_EPSILON     (float)(10e-6)
#endif

#ifndef RX_PI
#define RX_PI          (float)(3.1415926535897932384626433832795)    
#endif

#define equivalent(a,b)     (((a < b + RX_EPSILON) && (a > b - RX_EPSILON)) ? true : false)





//-----------------------------------------------------------------------------
// MARK:4x4�s���N���X
//-----------------------------------------------------------------------------
class rxMatrix4
{
protected:
	float m[16];

public:
	//! �R���X�g���N�^
	rxMatrix4(){ MakeIdentity(); }
	rxMatrix4(float r){ SetValue(r); }
	rxMatrix4(float *m){ SetValue(m); }
	rxMatrix4(float a00, float a01, float a02, float a03,
			float a10, float a11, float a12, float a13,
			float a20, float a21, float a22, float a23,
			float a30, float a31, float a32, float a33)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		element(0,3) = a03;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		element(1,3) = a13;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
		element(2,3) = a23;
		
		element(3,0) = a30;
		element(3,1) = a31;
		element(3,2) = a32;
		element(3,3) = a33;
	}

	//! �s����mp�ɓ�����
	void GetValue(float *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! �s����mp�ɓ�����
	template<typename T> 
	void GetValueT(T *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				mp[c++] = (T)element(i,j);
			}
		}
	}

	//! �s�����|�C���^�Ŏ擾
	const float* GetValue() const { return m; }

	//! mp�����s���ɒl������
	void SetValue(float *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mp�����s���ɒl������
	template<typename T> 
	void SetValueT(T *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				element(i,j) = (T)mp[c++];
			}
		}
	}

	//! �s���̂��ׂĂ̒l��r�ɂ���
	void SetValue(float r)
	{
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				element(i,j) = r;
			}
		}
	}

	//! �P�ʍs�񐶐�
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		element(0,3) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		element(1,3) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
		element(2,3) = 0.0;
		
		element(3,0) = 0.0; 
		element(3,1) = 0.0; 
		element(3,2) = 0.0;
		element(3,3) = 1.0;
	}

	//! �P�ʍs�����Ԃ�
	static rxMatrix4 Identity()
	{
		static rxMatrix4 mident(
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0);

		return mident;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(float s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! ���s�ړ�����(0,3)-(0,2)���ݒ�
	void SetTranslate(const Vec3 &t)
	{
		element(0,3) = t[0];
		element(1,3) = t[1];
		element(2,3) = t[2];
	}

	//! r�s�������ݒ�
	void SetRow(int r, const Vec4 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
		element(r,3) = t[3];
	}

	//! c�񐬕����ݒ�
	void SetColumn(int c, const Vec4&t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
		element(3,c) = t[3];
	}

	//! r�s�����擾
	void GetRow(int r, Vec4&t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
		t[3] = element(r,3);
	}

	//! r�s�����擾
	Vec4 GetRow(int r) const
	{
		Vec4 v;
		GetRow(r, v);
		return v;
	}

	//! c�񐬕��擾
	void GetColumn(int c, Vec4&t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
		t[3] = element(3,c);
	}

	//! c�񐬕��擾
	Vec4 GetColumn(int c) const
	{
		Vec4 v;
		GetColumn(c, v);
		return v;
	}

	//! �t�s���擾
	rxMatrix4 Inverse() const
	{
		rxMatrix4 minv;
		
		float r1[8], r2[8], r3[8], r4[8];
		float *s[4], *tmprow;
		
		s[0] = &r1[0];
		s[1] = &r2[0];
		s[2] = &r3[0];
		s[3] = &r4[0];
		
		register int i,j,p,jj;
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				s[i][j] = element(i, j);

				if(i==j){
					s[i][j+4] = 1.0;
				}
				else{
					s[i][j+4] = 0.0;
				}
			}
		}

		float scp[4];
		for(i = 0; i < 4; ++i){
			scp[i] = float(fabs(s[i][0]));
			for(j=1; j < 4; ++j)
				if(float(fabs(s[i][j])) > scp[i]) scp[i] = float(fabs(s[i][j]));
				if(scp[i] == 0.0) return minv; // singular matrix!
		}
		
		int pivot_to;
		float scp_max;
		for(i = 0; i < 4; ++i){
			// select pivot row
			pivot_to = i;
			scp_max = float(fabs(s[i][i]/scp[i]));
			// find out which row should be on top
			for(p = i+1; p < 4; ++p)
				if(float(fabs(s[p][i]/scp[p])) > scp_max){
					scp_max = float(fabs(s[p][i]/scp[p])); pivot_to = p;
				}
				// Pivot if necessary
				if(pivot_to != i)
				{
					tmprow = s[i];
					s[i] = s[pivot_to];
					s[pivot_to] = tmprow;
					float tmpscp;
					tmpscp = scp[i];
					scp[i] = scp[pivot_to];
					scp[pivot_to] = tmpscp;
				}
				
				float mji;
				// perform gaussian elimination
				for(j = i+1; j < 4; ++j)
				{
					mji = s[j][i]/s[i][i];
					s[j][i] = 0.0;
					for(jj=i+1; jj<8; jj++)
						s[j][jj] -= mji*s[i][jj];
				}
		}

		if(s[3][3] == 0.0) return minv; // singular matrix!
		
		//
		// Now we have an upper triangular matrix.
		//
		//  x x x x | y y y y
		//  0 x x x | y y y y 
		//  0 0 x x | y y y y
		//  0 0 0 x | y y y y
		//
		//  we'll back substitute to Get the inverse
		//
		//  1 0 0 0 | z z z z
		//  0 1 0 0 | z z z z
		//  0 0 1 0 | z z z z
		//  0 0 0 1 | z z z z 
		//
		
		float mij;
		for(i = 3; i > 0; --i){
			for(j = i-1; j > -1; --j){
				mij = s[j][i]/s[i][i];
				for(jj = j+1; jj < 8; ++jj){
					s[j][jj] -= mij*s[i][jj];
				}
			}
		}
		
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				minv(i,j) = s[i][j+4]/s[i][i];
			}
		}
			
		return minv;
	}

	//! �]�u�s���擾
    rxMatrix4 Transpose() const
	{
		rxMatrix4 mtrans;
		
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				mtrans(i,j) = element(j,i);
			}
		}

		return mtrans;
	}

	//! �s�����E�������|����
	rxMatrix4 &multRight(const rxMatrix4 &b)
	{
		rxMatrix4 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += mt(i,c)*b(c,j);
				}
			}
		}

		return *this;
	}    

	//! �s�������������|����
	rxMatrix4 &multLeft(const rxMatrix4 &b)
	{
		rxMatrix4 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += b(i,c)*mt(c,j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		float w = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+element(3,3));
	    
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+element(0,3))/w;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+element(1,3))/w;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+element(2,3))/w;
	}

	void multMatrixVec(Vec3 & src_and_dst) const
	{ 
		multMatrixVec(Vec3(src_and_dst), src_and_dst); 
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		float w = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+element(3,3));
	    
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+element(3,0))/w;
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+element(3,1))/w;
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+element(3,2))/w;
	}
	    

	void multVecMatrix(Vec3 & src_and_dst) const
	{ 
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! dst = M*src
	void multMatrixVec(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+src[3]*element(0,3));
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+src[3]*element(1,3));
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+src[3]*element(2,3));
		dst[3] = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+src[3]*element(3,3));
	}

	void multMatrixVec(Vec4 &src_and_dst) const
	{
		multMatrixVec(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+src[3]*element(3,0));
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+src[3]*element(3,1));
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+src[3]*element(3,2));
		dst[3] = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+src[3]*element(3,3));
	}
	    

	void multVecMatrix(Vec4 & src_and_dst) const
	{
		multVecMatrix(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = M*src
	void multMatrixDir(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)) ;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)) ;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)) ;
	}
	    

	void multMatrixDir(Vec3 & src_and_dst) const
	{
		multMatrixDir(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multDirMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+
				  src[1]*element(1,0)+
				  src[2]*element(2,0)) ;
		dst[1] = (src[0]*element(0,1)+
				  src[1]*element(1,1)+
				  src[2]*element(2,1)) ;
		dst[2] = (src[0]*element(0,2)+
				  src[1]*element(1,2)+
				  src[2]*element(2,2)) ;
	}

	void multDirMatrix(Vec3 &src_and_dst) const
	{ 
		multDirMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! �s���̗v�f���Ԃ�
	float& element(int row, int col){ return m[row | (col<<2)]; }
	const float& element(int row, int col) const { return m[row | (col<<2)]; }


	//! �I�y���[�^
	float& operator()(int row, int col){ return element(row,col); }
	const float& operator()(int row, int col) const	{ return element(row,col); }

	rxMatrix4& operator*=(const rxMatrix4 &mat)
	{
		multRight(mat);
		return *this;
	}

	rxMatrix4& operator*=(const float & r)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
			element(3,i) *= r;
		}

		return *this;
	}

	rxMatrix4& operator+=(const rxMatrix4 &mat)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
			element(3,i) += mat.element(3,i);
		}

		return *this;
	}

	friend rxMatrix4 operator*(const rxMatrix4 &m1, const rxMatrix4 &m2);
	friend Vec3 operator*(const rxMatrix4 &m, const Vec3 &v);
	friend bool operator==(const rxMatrix4 &m1, const rxMatrix4 &m2);
	friend bool operator!=(const rxMatrix4 &m1, const rxMatrix4 &m2);
    void Real(int arg1);
};

inline rxMatrix4 operator*(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	rxMatrix4 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}

inline Vec3 operator*(const rxMatrix4 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline Vec4 operator*(const rxMatrix4 &m, const Vec4 &v)
{
	Vec4 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline bool operator==(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(0,3) == m2(0,3) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(1,3) == m2(1,3) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2) &&
			m1(2,3) == m2(2,3) &&
			m1(3,0) == m2(3,0) &&
			m1(3,1) == m2(3,1) &&
			m1(3,2) == m2(3,2) &&
			m1(3,3) == m2(3,3));
}

inline bool operator!=(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	return !(m1 == m2);
}  



//-----------------------------------------------------------------------------
// MARK:3x3�s���N���X
//-----------------------------------------------------------------------------
class rxMatrix3
{
protected:
	float m[9];

public:
	//! �R���X�g���N�^
	rxMatrix3(){ makeIdentity(); }
	rxMatrix3(float r){ SetValue(r); }
	rxMatrix3(float *m){ SetValue(m); }
	rxMatrix3(float a00, float a01, float a02,
			  float a10, float a11, float a12,
			  float a20, float a21, float a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! �s����mp�ɓ�����
	void GetValue(float *mp) const
	{
		int c = 0;
		for(int j = 0; j < 3; j++){
			for(int i = 0; i < 3; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! �s����mp�ɓ�����
	void GetValue4x4(float *mp) const
	{
		mp[0]  = element(0,0);
		mp[1]  = element(1,0);
		mp[2]  = element(2,0);
		mp[3]  = 0.0;	  
						  
		mp[4]  = element(0,1);
		mp[5]  = element(1,1);
		mp[6]  = element(2,1);
		mp[7]  = 0.0;	  
						  
		mp[8]  = element(0,2);
		mp[9]  = element(1,2);
		mp[10] = element(2,2);
		mp[11] = 0.0;
	}

	//! �s�����|�C���^�Ŏ擾
	const float* GetValue() const { return m; }

	//! mp�����s���ɒl������
	void SetValue(float *mp)
	{
		int c = 0;
		for(int j = 0; j < 3; j++){
			for(int i = 0; i < 3; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mp�����s���ɒl������
	void SetValue4x4(float *mp)
	{
		element(0,0) = mp[0] ;
		element(1,0) = mp[1] ;
		element(2,0) = mp[2] ;
				 
		element(0,1)  =mp[4] ;
		element(1,1) = mp[5] ;
		element(2,1) = mp[6] ;

		element(0,2) = mp[8] ;
		element(1,2) = mp[9] ;
		element(2,2) = mp[10];
	}					  
						  
	//! �s���̂��ׂĂ̒l���ɂ���
	void SetValue(float r)  
	{					  
		for(int i = 0; i < 3; i++){
			for(int j = 0;j < 3; j++){
				element(i,j) = r;
			}			  
		}				  
	}
	void SetValue(float a00, float a01, float a02,
				  float a10, float a11, float a12,
				  float a20, float a21, float a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! �I�C���[�p�������]�s�����ݒ�
	void SetEuler(const double& yaw, const double& pitch, const double& roll) 
	{
		// yaw is CW around y-axis, pitch is CCW around x-axis, and roll is CW around z-axis
		double cy = cos(yaw); 
		double sy = sin(yaw); 
		double cp = cos(pitch); 
		double sp = sin(pitch); 
		double cr = cos(roll);
		double sr = sin(roll);

		double cc = cy*cr; 
		double cs = cy*sr; 
		double sc = sy*cr; 
		double ss = sy*sr;
		SetValue(cc+sp*ss, cs-sp*sc, -sy*cp,
				 -cp*sr,   cp*cr,    -sp,
				 sc-sp*cs, ss+sp*cc, cy*cp);
				    
	}

	//! �P�ʍs�񐶐�
	void makeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
	}

	//! �P�ʍs�����Ԃ�
	static rxMatrix3 Identity()
	{
		static rxMatrix3 mident(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0);

		return mident;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(float s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! ���s�ړ�����(0,3)-(0,2)���ݒ�
	void SetTranslate(const Vec3 &t)
	{
//		element(0,3) = t[0];
//		element(1,3) = t[1];
//		element(2,3) = t[2];
	}

	//! r�s�������ݒ�
	void SetRow(int r, const Vec3 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
	}

	//! c�񐬕����ݒ�
	void SetColumn(int c, const Vec3 &t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
	}

	//! r�s�����擾
	void GetRow(int r, Vec3 &t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
	}

	//! r�s�����擾
	Vec3 GetRow(int r) const
	{
		Vec3 v;
		GetRow(r, v);
		return v;
	}

	//! c�񐬕��擾
	void GetColumn(int c, Vec3 &t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
	}

	//! c�񐬕��擾
	Vec3 GetColumn(int c) const
	{
		Vec3 v;
		GetColumn(c, v);
		return v;
	}

	//! �t�s���擾
	rxMatrix3 Inverse() const
	{
		float d = element(0, 0)*element(1, 1)*element(2, 2)- 
				 element(0, 0)*element(2, 1)*element(1, 2)+ 
				 element(1, 0)*element(2, 1)*element(0, 2)- 
				 element(1, 0)*element(0, 1)*element(2, 2)+ 
				 element(2, 0)*element(0, 1)*element(1, 2)- 
				 element(2, 0)*element(1, 1)*element(0, 2);

		if(d == 0) d = 1;

		return	rxMatrix3( (element(1, 1)*element(2, 2)-element(1, 2)*element(2, 1))/d,
						  -(element(0, 1)*element(2, 2)-element(0, 2)*element(2, 1))/d,
						   (element(0, 1)*element(1, 2)-element(0, 2)*element(1, 1))/d,
						  -(element(1, 0)*element(2, 2)-element(1, 2)*element(2, 0))/d,
						   (element(0, 0)*element(2, 2)-element(0, 2)*element(2, 0))/d,
						  -(element(0, 0)*element(1, 2)-element(0, 2)*element(1, 0))/d,
						   (element(1, 0)*element(2, 1)-element(1, 1)*element(2, 0))/d,
						  -(element(0, 0)*element(2, 1)-element(0, 1)*element(2, 0))/d,
						   (element(0, 0)*element(1, 1)-element(0, 1)*element(1, 0))/d);	
	}

	//! �s����
	double Determinant(void) const 
	{
		return element(0, 0)*element(1, 1)*element(2, 2)- 
			   element(0, 0)*element(2, 1)*element(1, 2)+ 
			   element(1, 0)*element(2, 1)*element(0, 2)- 
			   element(1, 0)*element(0, 1)*element(2, 2)+ 
			   element(2, 0)*element(0, 1)*element(1, 2)- 
			   element(2, 0)*element(1, 1)*element(0, 2);
	}


	//! �]�u�s���擾
    rxMatrix3 Transpose() const
	{
		rxMatrix3 mtrans;
		
		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				mtrans(i, j) = element(j, i);
			}
		}

		return mtrans;
	}
		
		
	rxMatrix3 Scaled(const Vec3& s) const
	{
		return rxMatrix3(element(0, 0)*s[0], element(0, 1)*s[1], element(0, 2)*s[2],
						 element(1, 0)*s[0], element(1, 1)*s[1], element(1, 2)*s[2],
						 element(2, 0)*s[0], element(2, 1)*s[1], element(2, 2)*s[2]);
	}

	//! �s�����E�������|����
	rxMatrix3 &multRight(const rxMatrix3 &b)
	{
		rxMatrix3 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += mt(i, c)*b(c, j);
				}
			}
		}

		return *this;
	}    

	//! �s�������������|����
	rxMatrix3 &multLeft(const rxMatrix3 &b)
	{
		rxMatrix3 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += b(i, c)*mt(c, j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(0, 1)+src[2]*element(0, 2);
		dst[1] = src[0]*element(1, 0)+src[1]*element(1, 1)+src[2]*element(1, 2);
		dst[2] = src[0]*element(2, 0)+src[1]*element(2, 1)+src[2]*element(2, 2);
	}

	void multMatrixVec(Vec3 &src_and_dst) const
	{
		multMatrixVec(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(1, 0)+src[2]*element(2, 0);
		dst[1] = src[0]*element(0, 1)+src[1]*element(1, 1)+src[2]*element(2, 1);
		dst[2] = src[0]*element(0, 2)+src[1]*element(1, 2)+src[2]*element(2, 2);
	}
	    

	void multVecMatrix(Vec3 & src_and_dst) const
	{
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}


	//! �s���̗v�f���Ԃ�
	float& element(int row, int col){ return m[3*row+col]; }
	const float& element(int row, int col) const { return m[3*row+col]; }


	//! �I�y���[�^
	float& operator()(int row, int col){ return element(row, col); }
	const float& operator()(int row, int col) const	{ return element(row, col); }

	rxMatrix3& operator*=(const rxMatrix3 &mat)
	{
		multRight(mat);
		return *this;
	}

	rxMatrix3& operator*=(const float &r)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
		}

		return *this;
	}

	rxMatrix3& operator+=(const rxMatrix3 &mat)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
		}

		return *this;
	}

	friend rxMatrix3 operator*(const rxMatrix3 &m1, const rxMatrix3 &m2);
	friend Vec3 operator*(const rxMatrix3 &m, const Vec3 &v);
	friend Vec3 operator*(const Vec3 &v, const rxMatrix3 &m);
	friend bool operator==(const rxMatrix3 &m1, const rxMatrix3 &m2);
	friend bool operator!=(const rxMatrix3 &m1, const rxMatrix3 &m2);
};

inline rxMatrix3 operator*(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	rxMatrix3 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}

inline Vec3 operator*(const rxMatrix3 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline Vec3 operator*(const Vec3 &v, const rxMatrix3 &m)
{
	Vec3 product;
	
	m.multVecMatrix(v, product);
	
	return product;
}

inline bool operator==(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2));
}

inline bool operator!=(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	return !(m1 == m2);
}  


//-----------------------------------------------------------------------------
// MARK:2x2�s���N���X
//-----------------------------------------------------------------------------
class rxMatrix2
{
protected:
	float m[4];

public:
	//! �R���X�g���N�^
	rxMatrix2(){ MakeIdentity(); }
	rxMatrix2(float r){ SetValue(r); }
	rxMatrix2(float *m){ SetValue(m); }
	rxMatrix2(float a00, float a01, 
			  float a10, float a11)
	{
		element(0,0) = a00;
		element(0,1) = a01;
	
		element(1,0) = a10;
		element(1,1) = a11;
	}

	//! �s����mp�ɓ�����
	void GetValue(float *mp) const
	{
		int c = 0;
		for(int j = 0; j < 2; j++){
			for(int i = 0; i < 2; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! �s����mp�ɓ�����
	void GetValue3x3(float *mp) const
	{
		mp[0] = element(0,0);
		mp[1] = element(1,0);
		mp[2] = 0.0;
					  
		mp[3] = element(0,1);
		mp[4] = element(1,1);
		mp[5] = 0.0;	  
						  
		mp[6] = 0.0;
		mp[7] = 0.0;
		mp[8] = 0.0;
	}

	//! �s�����|�C���^�Ŏ擾
	const float* GetValue() const { return m; }

	//! mp�����s���ɒl������
	void SetValue(float *mp)
	{
		int c = 0;
		for(int j = 0; j < 2; j++){
			for(int i = 0; i < 2; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mp�����s���ɒl������
	void SetValue3x3(float *mp)
	{
		element(0,0) = mp[0];
		element(1,0) = mp[1];
				 
		element(0,1)  =mp[3];
		element(1,1) = mp[4];
	}					  
						  
	//! �s���̂��ׂĂ̒l���ɂ���
	void SetValue(float r)  
	{					  
		for(int i = 0; i < 2; i++){
			for(int j = 0;j < 2; j++){
				element(i,j) = r;
			}			  
		}				  
	}
	void SetValue(float a00, float a01, 
				  float a10, float a11)
	{
		element(0,0) = a00;
		element(0,1) = a01;
	
		element(1,0) = a10;
		element(1,1) = a11;
	}

	//! �p�������]�s�����ݒ�
	void SetEuler(const double& angle) 
	{
		// yaw is CW around y-axis, pitch is CCW around x-axis, and roll is CW around z-axis
		SetValue(cos(angle), -sin(angle), 
				 sin(angle),  cos(angle));
				    
	}

	//! �P�ʍs�񐶐�
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
	}

	//! �P�ʍs�����Ԃ�
	static rxMatrix2 Identity()
	{
		static rxMatrix2 mident(1.0, 0.0, 
								0.0, 1.0);

		return mident;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(float s)
	{
		element(0,0) = s;
		element(1,1) = s;
	}

	//! �X�P�[��(�Ίp����)���ݒ�	    
	void SetScale(const Vec2 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
	}

	//! r�s�������ݒ�
	void SetRow(int r, const Vec2 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
	}

	//! c�񐬕����ݒ�
	void SetColumn(int c, const Vec2 &t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
	}

	//! r�s�����擾
	void GetRow(int r, Vec2 &t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
	}

	//! r�s�����擾
	Vec2 GetRow(int r) const
	{
		Vec2 v;
		GetRow(r, v);
		return v;
	}

	//! c�񐬕��擾
	void GetColumn(int c, Vec2 &t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
	}

	//! c�񐬕��擾
	Vec2 GetColumn(int c) const
	{
		Vec2 v;
		GetColumn(c, v);
		return v;
	}

	float Determinant(void) const
	{
		return element(0, 0)*element(1, 1)-element(1, 0)*element(0, 1);
	}

	//! �t�s���擾
	rxMatrix2 Inverse() const
	{
		float d = element(0, 0)*element(1, 1)-element(1, 0)*element(0, 1);

		if(d == 0) d = 1;

		d = (float)1.0/d;

		return rxMatrix2 (element(1, 1)*d, -element(0, 1)*d, 
						 -element(1, 0)*d,  element(0, 0)*d);
	}

	//! �]�u�s���擾
    rxMatrix2 Transpose() const
	{
		rxMatrix2 mtrans;
		
		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				mtrans(i, j) = element(j, i);
			}
		}

		return mtrans;
	}
		
	void GramSchmidt(void)
	{
		Vec2 c0, c1;
		GetColumn(0, c0); normalize(c0);
		c1[0] = -c0[1];
		c1[1] =  c0[0];
		SetColumn(0, c0);
		SetColumn(1, c1);
	}

	void Rot(float angle)
	{
		float cosAngle = cos(angle);
		float sinAngle = sin(angle);
		element(0, 0) = element(1, 1) = cosAngle;
		element(0, 1) = -sinAngle;
		element(1, 0) =  sinAngle;
	}

	rxMatrix2 Scaled(const Vec2& s) const
	{
		return rxMatrix2(element(0, 0)*s[0], element(0, 1)*s[1], 
						 element(1, 0)*s[0], element(1, 1)*s[1]);
	}

	//! �s�����E�������|����
	rxMatrix2 &MultRight(const rxMatrix2 &b)
	{
		rxMatrix2 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				for(int c = 0; c < 2; ++c){
					element(i, j) += mt(i, c)*b(c, j);
				}
			}
		}

		return *this;
	}    

	//! �s�������������|����
	rxMatrix2 &MultLeft(const rxMatrix2 &b)
	{
		rxMatrix2 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 2; ++i){
			for(int j = 0; j < 2; ++j){
				for(int c = 0; c < 2; ++c){
					element(i, j) += b(i, c)*mt(c, j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void MultMatrixVec(const Vec2 &src, Vec2 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(0, 1);
		dst[1] = src[0]*element(1, 0)+src[1]*element(1, 1);
	}

	void MultMatrixVec(Vec2 &src_and_dst) const
	{
		MultMatrixVec(Vec2(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void MultVecMatrix(const Vec2 &src, Vec2 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(1, 0);
		dst[1] = src[0]*element(0, 1)+src[1]*element(1, 1);
	}
	    

	void MultVecMatrix(Vec2 &src_and_dst) const
	{
		MultVecMatrix(Vec2(src_and_dst), src_and_dst);
	}



	//! �I�y���[�^
	float& operator()(int row, int col){ return element(row, col); }
	const float& operator()(int row, int col) const	{ return element(row, col); }

	rxMatrix2& operator*=(const rxMatrix2 &mat)
	{
		MultRight(mat);
		return *this;
	}

	rxMatrix2& operator*=(const float &r)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) *= r;
			element(1, i) *= r;
		}

		return *this;
	}

	rxMatrix2& operator+=(const rxMatrix2 &mat)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) += mat.element(0, i);
			element(1, i) += mat.element(1, i);
		}

		return *this;
	}

	rxMatrix2& operator-=(const rxMatrix2 &mat)
	{
		for(int i = 0; i < 2; ++i){
			element(0, i) -= mat.element(0, i);
			element(1, i) -= mat.element(1, i);
		}

		return *this;
	}

protected:
	//! �s���̗v�f���Ԃ�
	float& element(int row, int col){ return m[2*row+col]; }
	const float& element(int row, int col) const { return m[2*row+col]; }

public:
	friend rxMatrix2 operator*(const rxMatrix2 &m1, const rxMatrix2 &m2);
	friend rxMatrix2 operator+(const rxMatrix2 &m1, const rxMatrix2 &m2);
	friend rxMatrix2 operator-(const rxMatrix2 &m1, const rxMatrix2 &m2);
	friend rxMatrix2 operator-(const rxMatrix2 &m);
	friend Vec2 operator*(const rxMatrix2 &m, const Vec2 &v);
	friend Vec2 operator*(const Vec2 &v, const rxMatrix2 &m);
	friend rxMatrix2 operator*(const rxMatrix2 &m, const float &s);
	friend rxMatrix2 operator*(const float &s, const rxMatrix2 &m);
	friend bool operator==(const rxMatrix2 &m1, const rxMatrix2 &m2);
	friend bool operator!=(const rxMatrix2 &m1, const rxMatrix2 &m2);
};

inline rxMatrix2 operator*(const rxMatrix2 &m1, const rxMatrix2 &m2)
{
	rxMatrix2 product;
	
	product = m1;
	product.MultRight(m2);
	
	return product;
}

inline rxMatrix2 operator+(const rxMatrix2 &m1, const rxMatrix2 &m2)
{
	rxMatrix2 res = m1;
	res += m2;
	return res;
}

inline rxMatrix2 operator-(const rxMatrix2 &m1, const rxMatrix2 &m2)
{
	rxMatrix2 res = m1;
	res -= m2;
	return res;
}

inline rxMatrix2 operator-(const rxMatrix2 &m)
{
	return rxMatrix2(-m.m[0], -m.m[1], -m.m[2], -m.m[3]);
}


inline Vec2 operator*(const rxMatrix2 &m, const Vec2 &v)
{
	Vec2 product;
	
	m.MultMatrixVec(v, product);
	
	return product;
}

inline Vec2 operator*(const Vec2 &v, const rxMatrix2 &m)
{
	Vec2 product;
	
	m.MultVecMatrix(v, product);
	
	return product;
}

inline rxMatrix2 operator*(const rxMatrix2 &m, const float &s)
{
	rxMatrix2 ms = m;
	ms *= s;
	return ms;
}

inline rxMatrix2 operator*(const float &s, const rxMatrix2 &m)
{
	rxMatrix2 ms = m;
	ms *= s;
	return ms;
}

inline bool operator==(const rxMatrix2 &m1, const rxMatrix2 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1));
}

inline bool operator!=(const rxMatrix2 &m1, const rxMatrix2 &m2)
{
	return !(m1 == m2);
}  



inline void JacobiRotate(rxMatrix2 &A, rxMatrix2 &R)
{
	// rotates A through phi in 01-plane to set A(0,1) = 0
	// rotation stored in R whose columns are eigenvectors of A
	float d = (A(0,0)-A(1,1))/(2.0f*A(0,1));
	float t = 1.0/(fabs(d)+sqrt(d*d+(float)1.0));
	if(d < 0.0f) t = -t;
	float c = 1.0f/sqrt(t*t+1);
	float s = t*c;
	A(0,0) += t*A(0,1);
	A(1,1) -= t*A(0,1);
	A(0,1) = A(1,0) = 0.0;

	// store rotation in R
	for(int k = 0; k < 2; ++k){
		float Rkp = c*R(k,0)+s*R(k,1);
		float Rkq =-s*R(k,0)+c*R(k,1);
		R(k,0) = Rkp;
		R(k,1) = Rkq;
	}
}



inline void EigenDecomposition(rxMatrix2 &A, rxMatrix2 &R)
{
	// only for symmetric matrices!
	// A = R A' R^T, where A' is diagonal and R orthonormal

	R.MakeIdentity();	// �P�ʍs��
	JacobiRotate(A, R);
}

/*!
 * A = RS, where S is symmetric and R is orthonormal -> S = (A^T A)^(1/2)
 * @param[in] A
 * @param[out] R,S
 */
inline void PolarDecomposition(const rxMatrix2 &A, rxMatrix2 &R, rxMatrix2 &S)
{
	R.MakeIdentity();

	rxMatrix2 ATA;
	ATA = A*A.Transpose();

	rxMatrix2 U;
	R.MakeIdentity();
	EigenDecomposition(ATA, U);

	float l0 = ATA(0,0); 
	if(l0 <= 0.0){
		l0 = 0.0;
	}
	else{
		l0 = 1.0/sqrt(l0);
	}

	float l1 = ATA(1,1);
	if(l1 <= 0.0){
		l1 = 0.0;
	}
	else{
		l1 = 1.0/sqrt(l1);
	}

	rxMatrix2 S1;
	S1(0,0) = l0*U(0,0)*U(0,0) + l1*U(0,1)*U(0,1);
	S1(0,1) = l0*U(0,0)*U(1,0) + l1*U(0,1)*U(1,1);
	S1(1,0) = S1(0,1);
	S1(1,1) = l0*U(1,0)*U(1,0) + l1*U(1,1)*U(1,1);
	R = A*S1;
	S = R*A.Transpose();
}





//-----------------------------------------------------------------------------
// MARK:5x5�s���N���X
//-----------------------------------------------------------------------------
class rxMatrix5
{
protected:
	float m[25];

public:
	//! �R���X�g���N�^
	rxMatrix5(){ MakeIdentity(); }
	rxMatrix5(float r){ SetValue(r); }
	rxMatrix5(float *m){ SetValue(m); }
	rxMatrix5(float a00, float a01, float a02, float a03, float a04, 
			  float a10, float a11, float a12, float a13, float a14,
			  float a20, float a21, float a22, float a23, float a24,
			  float a30, float a31, float a32, float a33, float a34, 
			  float a40, float a41, float a42, float a43, float a44)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		element(0,3) = a03;
		element(0,4) = a04;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		element(1,3) = a13;
		element(1,4) = a14;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
		element(2,3) = a23;
		element(2,4) = a24;
		
		element(3,0) = a30;
		element(3,1) = a31;
		element(3,2) = a32;
		element(3,3) = a33;
		element(3,4) = a34;
		
		element(4,0) = a40;
		element(4,1) = a41;
		element(4,2) = a42;
		element(4,3) = a43;
		element(4,4) = a44;
	}

	//! �s����mp�ɓ�����
	void GetValue(float *mp) const
	{
		int c = 0;
		for(int j = 0; j < 5; j++){
			for(int i = 0; i < 5; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! �s����mp�ɓ�����
	template<typename T> 
	void GetValueT(T *mp) const
	{
		int c = 0;
		for(int j = 0; j < 5; j++){
			for(int i = 0; i < 5; i++){
				mp[c++] = (T)element(i,j);
			}
		}
	}

	//! �s�����|�C���^�Ŏ擾
	const float* GetValue() const { return m; }

	//! mp�����s���ɒl������
	void SetValue(float *mp)
	{
		int c = 0;
		for(int j = 0; j < 5; j++){
			for(int i = 0; i < 5; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mp�����s���ɒl������
	template<typename T> 
	void SetValueT(T *mp)
	{
		int c = 0;
		for(int j = 0; j < 5; j++){
			for(int i = 0; i < 5; i++){
				element(i,j) = (T)mp[c++];
			}
		}
	}

	//! �s���̂��ׂĂ̒l��r�ɂ���
	void SetValue(float r)
	{
		for(int i = 0; i < 5; i++){
			for(int j = 0; j < 5; j++){
				element(i,j) = r;
			}
		}
	}

	//! �P�ʍs�񐶐�
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		element(0,3) = 0.0;
		element(0,4) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		element(1,3) = 0.0;
		element(1,4) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
		element(2,3) = 0.0;
		element(2,4) = 0.0;
		
		element(3,0) = 0.0; 
		element(3,1) = 0.0; 
		element(3,2) = 0.0;
		element(3,3) = 1.0;
		element(3,4) = 0.0;
		
		element(4,0) = 0.0; 
		element(4,1) = 0.0; 
		element(4,2) = 0.0;
		element(4,3) = 0.0;
		element(4,4) = 1.0;
	}

	//! �P�ʍs�����Ԃ�
	static rxMatrix5 Identity()
	{
		static rxMatrix5 mident(
			1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.0);

		return mident;
	}

	//! �]�u�s���擾
    rxMatrix5 Transpose() const
	{
		rxMatrix5 mtrans;
		
		for(int i = 0; i < 5; ++i){
			for(int j = 0; j < 5; ++j){
				mtrans(i,j) = element(j,i);
			}
		}

		return mtrans;
	}

	//! �s�����E�������|����
	rxMatrix5 &MultRight(const rxMatrix5 &b)
	{
		rxMatrix5 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 5; ++i){
			for(int j = 0; j < 5; ++j){
				for(int c = 0; c < 5; ++c){
					element(i,j) += mt(i,c)*b(c,j);
				}
			}
		}

		return *this;
	}    

	//! �s�������������|����
	rxMatrix5 &MultLeft(const rxMatrix5 &b)
	{
		rxMatrix5 mt(*this);
		SetValue(float(0));

		for(int i = 0; i < 5; ++i){
			for(int j = 0; j < 5; ++j){
				for(int c = 0; c < 5; ++c){
					element(i,j) += b(i,c)*mt(c,j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void MultMatrixVec(const float *src, float *dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+src[3]*element(0,3)+src[4]*element(0,4));
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+src[3]*element(1,3)+src[4]*element(1,4));
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+src[3]*element(2,3)+src[4]*element(2,4));
		dst[3] = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+src[3]*element(3,3)+src[4]*element(3,4));
		dst[4] = (src[0]*element(4,0)+src[1]*element(4,1)+src[2]*element(4,2)+src[3]*element(4,3)+src[4]*element(4,4));
	}

	//! dst = src*M
	void MultVecMatrix(const float *src, float *dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+src[3]*element(3,0)+src[3]*element(4,0));
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+src[3]*element(3,1)+src[3]*element(4,1));
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+src[3]*element(3,2)+src[3]*element(4,2));
		dst[3] = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+src[3]*element(3,3)+src[3]*element(4,3));
		dst[4] = (src[0]*element(0,4)+src[1]*element(1,4)+src[2]*element(2,4)+src[3]*element(3,4)+src[3]*element(4,4));
	}
	    



	//! �I�y���[�^
	float& operator()(int row, int col){ return element(row,col); }
	const float& operator()(int row, int col) const	{ return element(row,col); }

	rxMatrix5& operator*=(const rxMatrix5 &mat)
	{
		MultRight(mat);
		return *this;
	}

	rxMatrix5& operator*=(const float & r)
	{
		for(int i = 0; i < 5; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
			element(3,i) *= r;
			element(4,i) *= r;
		}
		return *this;
	}

	rxMatrix5& operator+=(const rxMatrix5 &mat)
	{
		for(int i = 0; i < 5; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
			element(3,i) += mat.element(3,i);
			element(4,i) += mat.element(4,i);
		}
		return *this;
	}

	rxMatrix5& operator-=(const rxMatrix5 &mat)
	{
		for(int i = 0; i < 5; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
			element(3,i) += mat.element(3,i);
			element(4,i) += mat.element(4,i);
		}
		return *this;
	}

	void JacobiRotate(rxMatrix5 &A, rxMatrix5 &R, int p, int q);
	void EigenDecomposition(rxMatrix5 &A, rxMatrix5 &R);
	void Invert(void);

protected:
	//! �s���̗v�f���Ԃ�
	float& element(int row, int col){ return m[5*row+col]; }
	const float& element(int row, int col) const { return m[5*row+col]; }

public:
	friend rxMatrix5 operator*(const rxMatrix5 &m1, const rxMatrix5 &m2);
	friend bool operator==(const rxMatrix5 &m1, const rxMatrix5 &m2);
	friend bool operator!=(const rxMatrix5 &m1, const rxMatrix5 &m2);
};

inline rxMatrix5 operator*(const rxMatrix5 &m1, const rxMatrix5 &m2)
{
	rxMatrix5 product;
	
	product = m1;
	product.MultRight(m2);
	
	return product;
}

inline rxMatrix5 operator+(const rxMatrix5 &m1, const rxMatrix5 &m2)
{
	rxMatrix5 res = m1;
	res += m2;
	return res;
}

inline rxMatrix5 operator-(const rxMatrix5 &m1, const rxMatrix5 &m2)
{
	rxMatrix5 res = m1;
	res -= m2;
	return res;
}


inline rxMatrix5 operator*(const rxMatrix5 &m, const float &s)
{
	rxMatrix5 ms = m;
	ms *= s;
	return ms;
}


inline bool operator==(const rxMatrix5 &m1, const rxMatrix5 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(0,3) == m2(0,3) &&
			m1(0,4) == m2(0,4) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(1,3) == m2(1,3) &&
			m1(1,4) == m2(1,4) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2) &&
			m1(2,3) == m2(2,3) &&
			m1(2,4) == m2(2,4) &&
			m1(3,0) == m2(3,0) &&
			m1(3,1) == m2(3,1) &&
			m1(3,2) == m2(3,2) &&
			m1(3,3) == m2(3,3) &&
			m1(4,4) == m2(4,4) && 
			m1(4,0) == m2(4,0) &&
			m1(4,1) == m2(4,1) &&
			m1(4,2) == m2(4,2) &&
			m1(4,3) == m2(4,3) &&
			m1(4,4) == m2(4,4));
}

inline bool operator!=(const rxMatrix5 &m1, const rxMatrix5 &m2)
{
	return !(m1 == m2);
}


#define EPSILON 1e-10
#define JACOBI_ITERATIONS 20


inline void rxMatrix5::JacobiRotate(rxMatrix5 &A, rxMatrix5 &R, int p, int q)
{
	// rotates A through phi in pq-plane to set A(p,q) = 0
	// rotation stored in R whose columns are eigenvectors of A
	float d = (A(p,p) - A(q,q))/(2.0*A(p,q));
	float t = 1.0/(fabs(d)+sqrt(d*d+1.0));
	if (d < 0.0) t = -t;
	float c = 1.0/sqrt(t*t+1);
	float s = t*c;
	A(p,p) += t*A(p,q);
	A(q,q) -= t*A(p,q);
	A(p,q) = A(q,p) = 0.0;
	// transform A
	int k;
	for (k = 0; k < 5; k++) {
		if (k != p && k != q) {
			float Akp = c*A(k,p) + s*A(k,q);
			float Akq =-s*A(k,p) + c*A(k,q);
			A(k,p) = A(p,k) = Akp;
			A(k,q) = A(q,k) = Akq;
		}
	}
	// store rotation in R
	for (k = 0; k < 5; k++) {
		float Rkp = c*R(k,p) + s*R(k,q);
		float Rkq =-s*R(k,p) + c*R(k,q);
		R(k,p) = Rkp;
		R(k,q) = Rkq;
	}
}


inline void rxMatrix5::EigenDecomposition(rxMatrix5 &A, rxMatrix5 &R)
{
	// only for symmetric matrices!
	// A = R A' R^T, where A' is diagonal and R orthonormal

	R.MakeIdentity();	// unit matrix
	int iter = 0;
	while (iter < JACOBI_ITERATIONS) {	// 10 off diagonal elements
		// find off diagonal element with maximum modulus
		int p,q;
		float a,max;
		max = -1.0f;
		for (int i = 0; i < 4; i++) {
			for (int j = i+1; j < 5; j++) {
				a = fabs(A(i,j));
				if (max < 0.0f || a > max) {
					p = i; q = j; max = a;
				}
			}
		}
		// all small enough -> done
//		if (max < EPSILON) break;  debug
		if (max <= 0.0) break;
		// rotate matrix with respect to that element
		JacobiRotate(A, R, p,q);
		iter++;
	}
}


inline void rxMatrix5::Invert()
{
	rxMatrix5 R, A = *this;
	EigenDecomposition(A, R);

	float d[5];
	int i,j,k;

	for (i = 0; i < 5; i++) {
		d[i] = A(i,i);
		if (d[i] != 0.0) d[i] = 1.0/d[i];
	}

	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			float &a = m[i*5+j];
			a = 0.0f;
			for (k = 0; k < 5; k++)
				a += d[k]*R(i,k)*R(j,k);
		}
	}
}


#endif // _MATRIX_H_
