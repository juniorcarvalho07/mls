/*!
  @file rx_utility.h

  @brief �l�X�Ȋ֐�
 
�@@author Makoto Fujisawa
�@@date  
*/

#ifndef _RX_UTILITY_
#define _RX_UTILITY_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>
//#include <cstdarg>
#include <cmath>
#include <ctime>
#include <cassert>

#include <iostream>
#include <fstream>

#include <string>
#include <vector>

#include "rx_vec.h"


//--------------------------------------------------------------------
// �萔
//--------------------------------------------------------------------
// �~����
const double RX_M_PI = 3.14159265358979323846;
const double RX_PI = 3.14159265358979323846;

//! (4/3)*�~����
const double RX_V_PI = 4.18666666666666666667;

//! ������
const double RX_ROOT2 = 1.414213562;
const double RX_ROOT3 = 1.732050808;
const double RX_ROOT5 = 2.236067977;

//! ���e�덷
const double RX_FEQ_EPS = 1.0e-10;
const double RX_EPS = 1.0e-8;

//! ��
const double RX_FEQ_INF = 1.0e10;
const double RX_FEQ_INFM = 0.999e10;

//! float�^�ő��l
//#define RX_MAX_FLOAT 3.40282347e+38F
#define RX_MAX_FLOAT 3.4e+38F

//! 1/pi
const double RX_INV_PI = 0.318309886;

//! ���[�g3
const double RX_SQRT3 = 1.7320508;

//! degree -> radian �̕ϊ��W��(pi/180.0)
const double RX_DEGREES_TO_RADIANS = 0.0174532925199432957692369076848;

//! radian -> degree �̕ϊ��W��(180.0/pi)
const double RX_RADIANS_TO_DEGREES = 57.295779513082320876798154814114;



//--------------------------------------------------------------------
// �}�N��(�e���v���[�g�֐�)
//--------------------------------------------------------------------
//! �[������
template<class T> 
inline bool RX_IS_ZERO(const T &x){ return (fabs(x) < RX_FEQ_EPS); }

//! ���e�덷���܂߂����l����
template<class T> 
inline bool RX_FEQ(const T &a, const T &b){ return (fabs(a-b) < RX_FEQ_EPS); }

//! ���e�덷���܂߂����l����(Vec3�p)
template<class T> 
inline bool RX_VEC3_FEQ(const T &a, const T &b, double eps = RX_FEQ_EPS)
{
	return ( (fabs(a[0]-b[0]) < eps) && (fabs(a[1]-b[1]) < eps) && (fabs(a[2]-b[2]) < eps) );
}

//! degree -> radian �̕ϊ�
template<class T> 
inline T RX_TO_RADIANS(const T &x){ return static_cast<T>((x)*RX_DEGREES_TO_RADIANS); }

//! radian -> degree �̕ϊ�
template<class T> 
inline T RX_TO_DEGREES(const T &x){ return static_cast<T>((x)*RX_RADIANS_TO_DEGREES); }

//! �ő��l����(2�l)
template<class T> 
inline T RX_MAX(const T &a, const T &b){ return ((a > b) ? a : b); }

//! �ő��l����(3�l)
template<class T> 
inline T RX_MAX3(const T &a, const T &b, const T &c){ return ( (a > b) ? ( (a > c) ? a : c) : ( (b > c) ? b : c)); }

//! �ŏ��l����(2�l)
template<class T> 
inline T RX_MIN(const T &a, const T &b){ return ((a < b) ? a : b); }

//! �ŏ��l����(3�l)
template<class T> 
inline T RX_MIN3(const T &a, const T &b, const T &c){ return ( (a < b) ? ( (a < c) ? a : c) : ( (b < c) ? b : c)); }

//! �l�̃N�����v(�N�����v�����l���Ԃ�)
template<class T> 
inline T RX_CLAMP(const T &x, const T &a, const T &b){ return ((x < a) ? a : (x > b) ? b : x); }

//! �t���O�̐ؑ�(t��-1���w�肵���甽�]�C0��1���w���ł���������)
template<class T>
inline void RX_TOGGLE(T &flag, int t = -1){ flag = ((t == -1) ? !flag : ((t == 0) ? 0 : 1)); }

//! 1�������^����
template<class T>
inline T RX_LERP(const T &a, const T &b, const T &t){ return a + t*(b-a); }

//! �X���b�v
template<class T>
inline void RX_SWAP(T &a, T &b){ T c; c = a; a = b; b = c; }

//! ����
template<class T>
inline T RX_SIGN(const T &x){ return (x >= 0 ? 1 : (x < 0 ? -1 : 0)); }

//! a�̕�����b�̕����ɂ��킹��
template<class T>
inline T RX_SIGN2(const T &a, const T &b){ return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a); }



//-----------------------------------------------------------------------------
// ���`
//-----------------------------------------------------------------------------
// FOR
#ifndef RXFOR3
#define RXFOR3(i0, i1, j0, j1, k0, k1) for(int i = i0; i < i1; ++i) \
											for(int j = j0; j < j1; ++j) \
												for(int k = k0; k < k1; ++k)
#endif

#ifndef RXFOR3E
#define RXFOR3E(i0, i1, j0, j1, k0, k1) for(int i = i0; i <= i1; ++i) \
											for(int j = j0; j <= j1; ++j) \
												for(int k = k0; k <= k1; ++k)
#endif

#ifndef RXFOR2
#define RXFOR2(i0, i1, j0, j1) for(int i = i0; i < i1; ++i) \
								   for(int j = j0; j < j1; ++j)
#endif

#ifndef RXFOR2E
#define RXFOR2E(i0, i1, j0, j1) for(int i = i0; i <= i1; ++i) \
								    for(int j = j0; j <= j1; ++j)
#endif




//-----------------------------------------------------------------------------
// ���낢���Ȋ֐�
//-----------------------------------------------------------------------------
namespace RXFunc 
{
	/*!
	 * �w�肵�����̔��p�X�y�[�X���Ԃ�
	 * @param[in] n 
	 * @return ���p�X�y�[�X
	 */
	inline std::string GenSpace(int n)
	{
		std::string spc;
		for(int i = 0; i < n; ++i){
			spc += " ";
		}
		return spc;
	}

	/*!
	 * Vec3�̗v�f���S�Đ�->true
	 * @param 
	 * @return 
	 */
	inline bool IsPositive(const Vec3 &x)
	{
		if(x[0] < -RX_FEQ_EPS)	return false;
		if(x[1] < -RX_FEQ_EPS)	return false;
		if(x[2] < -RX_FEQ_EPS)	return false;

		return true;
	}

	/*!
	 * 2�����ł̓_x��min_x,max_y�ň͂܂ꂽ���`���ɂ��邩�ǂ����̔���
	 * @param 
	 * @return 
	 */
	inline bool InRange(const Vec2 &x, const Vec2 &min_x, const Vec2 &max_x)
	{
		if(x[0] < min_x[0]) return false;
		if(x[1] < min_x[1]) return false;
		if(x[0] > max_x[0]) return false;
		if(x[1] > max_x[1]) return false;

		return true;
	}


	/*!
	 * std::vector�R���e�i���m�̓��όv�Z
	 */
	template<class T>
	inline T DotProduct(const std::vector<T> &a, const std::vector<T> &b)
	{
		T d = static_cast<T>(0);
		for(int i = 0; i < (int)a.size(); ++i){
			d += a[i]*b[i];
		}
		return d;
	}


	/*!
	 * 2����std::vector�R���e�i���m�̓��όv�Z
	 */
	template<class T>
	inline T DotProduct(const std::vector< std::vector<T> > &a, const std::vector< std::vector<T> > &b)
	{
		T d = static_cast<T>(0);
		int nx = (int)a.size();
		int ny = (int)a[0].size();
		for(int i = 0; i < nx; ++i){
			for(int j = 0; j < ny; ++j){
				d += a[i][j]*b[i][j];
			}
		}
		return d;
	}

	/*!
	 * 
	 * @param 
	 * @return 
	 */
	inline double Min3(const Vec3 &x)
	{
		return ( ( (x[0]<x[1]) && (x[0]<x[2]) ) ? x[0] : ( (x[1]<x[2]) ? x[1] : x[2] ) );
	}

	/*!
	 * 
	 * @param 
	 * @return 
	 */
	inline double Max3(const Vec3 &x)
	{
		return ( ( (x[0]>x[1]) && (x[0]>x[2]) ) ? x[0] : ( (x[1]>x[2]) ? x[1] : x[2] ) );
	}

	/*!
	 * 2�l�����r���đ傫�������Ԃ��֐�
	 * @param x ���r����2�l���i�[����Vec2
	 * @return �傫�����̒l
	 */
	inline double Max2(const Vec2 &x)
	{
		return ( (x[0] > x[1]) ? x[0] : x[1] );
	}

	/*!
	 * 2�l�����r���đ傫�������Ԃ��֐�
	 * @param x ���r����2�l���i�[����Vec2
	 * @return �傫�����̒l
	 */
	inline double Min2(const Vec2 &x)
	{
		return ( (x[0] < x[1]) ? x[0] : x[1] );
	}
		
	/*!
	 * [0,1]�̗����̐���
	 */
	inline double Frand(void)
	{
		return (double)(rand()/(1.0+RAND_MAX));
	}

		/*!
	 * [0,n]�̗����̐���
	 */
	inline int Nrand(int n)
	{
		return (int)(Frand()*n);
	}

	/*!
	 * [-1,1]�̗����̐���
	 */
	inline double SignedRand(void)
	{
		return 2*Frand()-1;
	}

	/*!
	 * [0,255]�̐��������̐���
	 */
	inline char ByteRand(void)
	{
		return (char)(rand() & 0xff);
	}

	/*!
	 * �w�肵���͈͂̎��������̐���
	 */
	inline double Rand(const double &_max, const double &_min)
	{
		return (_max-_min)*Frand()+_min;
	}

	/*!
	 * �w�肵���͈͂̎��������̐���(Vec2)
	 */
	inline Vec2 Rand(const Vec2 &_max, const Vec2 &_min)
	{
		return Vec2((_max[0]-_min[0])*Frand()+_min[0], (_max[1]-_min[1])*Frand()+_min[1]);
	}

	/*!
	 * �w�肵���͈͂̎��������̐���(Vec3)
	 * @param 
	 * @return 
	 */
	inline Vec3 Rand(const Vec3 &_max, const Vec3 &_min)
	{
		return Vec3((_max[0]-_min[0])*Frand()+_min[0], (_max[1]-_min[1])*Frand()+_min[1], (_max[2]-_min[2])*Frand()+_min[2]);
	}

	/*!
	 * ���K���z�̎��������̐���
	 * @param[in] m ����
	 * @param[in] s �W���΍�
	 */
	inline double NormalRand(double m, double s)
	{
		double x1, x2, w;
		do{
			x1 = 2.0*Frand()-1.0;
			x2 = 2.0*Frand()-1.0;
			w = x1*x1 + x2*x2;
		} while(w >= 1.0 || w < 1E-30);

		w = sqrt((-2.0*log(w))/w);

		x1 *= w;
		return x1*s+m;
	}
	
	/*!
	 * �]���v�Z(double�p)
	 * @param 
	 * @return 
	 */
	inline double Mod(double a, double b)
	{
		int n = (int)(a/b);
		a -= n*b;
		if(a < 0)
			a += b;
		return a;
	}


	/*!
	 * �����`�F�b�N���܂߂����s���̌v�Z
	 * @param[in] x ���������v�Z�������l
	 * @return �������̌v�Z���ʁDx�����Ȃ�0.0
	 */
	inline double Sqrt(const double &x)
	{
		return x > 0.0 ? sqrt(x) : 0.0;
	}


	/*!
	 * ���Βl(Vec3)
	 * @param a
	 * @return 
	 */
	inline Vec3 Fabs(const Vec3& a)
	{
		return Vec3(fabs(a[0]),fabs(a[1]),fabs(a[2]));
	}

	/*!
	 * ���Βl(Vec2)
	 * @param a
	 * @return 
	 */
	inline Vec2 Fabs(const Vec2& a)
	{
		return Vec2(fabs(a[0]),fabs(a[1]));
	}

	/*!
	 * Vec3�̗v�f�̒��Ő��Βl�̍ő��l���Ԃ�
	 * @param x
	 * @return 
	 */
	inline double Max3Abs(const Vec3 &x)
	{
		return Max3(Fabs(x));
	}

	/*!
	 * Vec3�z���̗v�f�̒��Ő��Βl�̍ő��l���Ԃ�
	 * @param vx
	 * @return 
	 */
	inline double Max3Abs(const std::vector<Vec3> &vx)
	{
		double t_value, max_value = 0;
		for(std::vector<Vec3>::size_type i = 0; i < vx.size(); ++i){
			if((t_value = Max3Abs(vx[i])) > max_value) max_value = t_value;
		}
		return max_value;
	}

	/*!
	 * �N�����v(�^�����ꂽ�l���w�肵���͈͊O�Ȃ��΁C���̋��E�ɐ؂��l�߂�)
	 * @param x �l
	 * @param a,b ���E�l
	 * @return �N�����v���ꂽ�l
	 */
	inline Vec3 Clamp(const Vec3& x, const Vec3& a, const Vec3& b)
	{
		return Vec3(RX_CLAMP(x[0],a[0],b[0]), RX_CLAMP(x[1],a[1],b[1]), RX_CLAMP(x[2],a[2],b[2]));
	}
	
	/*!
	 * returns true if the std::vector has a very small norm
	 * @param[in] A �x�N�g���l
	 * @return 
	 */
	inline bool IsZeroVec(const Vec3& A)
	{
		return (RX_IS_ZERO(A[0]) && RX_IS_ZERO(A[1]) && RX_IS_ZERO(A[2]));
	}


	/*!
	 * returns true if the std::vector is not very small
	 * @param[in] A �x�N�g���l
	 * @return 
	 */
	inline bool NonZeroVec(const Vec3& A)
	{
		return (!RX_IS_ZERO(A[0]) || !RX_IS_ZERO(A[1]) || !RX_IS_ZERO(A[2]));
	}


	/*!
	 * Vec3�̊e�v�f�̕��ϒl
	 * @param[in] A �x�N�g���l
	 * @return �x�N�g���l�̕���
	 */
	inline double AverageVec(Vec3 A)
	{
		return (A[0]+A[1]+A[2])/3.0;
	}

 
	/*!
	 * ��->��->��->���ƕω������T�[���O���t�p�̐F����
	 * @param[out] col �������ꂽ�F
	 * @param[in] x �l
	 * @param[in] xmin �ŏ��l
	 * @param[in] xmax �ő��l
	 */
	inline void Thermograph(double col[3], double x, const double xmin = 0.0, const double xmax = 1.0)
	{
		double l = xmax-xmin;
		if(fabs(l) < 1e-10) return;
    
		const int ncolors = 7;
		double base[ncolors][3] = { {0.0, 0.0, 0.0},
									{0.0, 0.0, 1.0},
									{0.0, 1.0, 1.0},
									{0.0, 1.0, 0.0},
									{1.0, 1.0, 0.0},
									{1.0, 0.0, 0.0},
									{1.0, 1.0, 1.0} };
		x = RX_CLAMP(((x-xmin)/l), 0.0, 1.0)*(ncolors-1);
		int i = (int)x;
		double dx = x-floor(x);
		col[0] = RX_LERP(base[i][0], base[i+1][0], dx);
		col[1] = RX_LERP(base[i][1], base[i+1][1], dx);
		col[2] = RX_LERP(base[i][2], base[i+1][2], dx);
	}
 
	/*!
	 * �O���f�[�V�����F����
	 * @param[out] col �������ꂽ�F
	 * @param[in] col1 x=xmin�̂Ƃ��̐F
	 * @param[in] col2 x=xmax�̂Ƃ��̐F
	 * @param[in] x �l
	 * @param[in] xmin �ŏ��l
	 * @param[in] xmax �ő��l
	 */
	inline void Gradation(double col[3], const double col1[3], const double col2[3], 
						  double x, const double xmin = 0.0, const double xmax = 1.0)
	{
		double l = xmax-xmin;
		if(fabs(l) < 1e-10) return;
    
		double dx = RX_CLAMP(((x-xmin)/l), 0.0, 1.0);
		col[0] = RX_LERP(col1[0], col2[0], dx);
		col[1] = RX_LERP(col1[1], col2[1], dx);
		col[2] = RX_LERP(col1[2], col2[2], dx);
	}
	
	
	
};



#endif // #ifndef _RX_UTILITY_
