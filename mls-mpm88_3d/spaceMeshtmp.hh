#ifndef SPACEMESH_H
#define SPACEMESH_H


typedef std::pair<int,int> TcP;
#include "vector.hh"
#include "kdtree.hh"
#include "sph.hh"
#include "rx_matrix.hh"
#include <math.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <algorithm>
const int BITS[4] = { 1, 2, 4, 8 };

const double OFFSET = 1.0;

const double INF = 1.0e10;




#define MX -1

const int g_MeshTable[256][19] = {

        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1101
        {2, 0, 1, 2, 0, 2, 3, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1110
        {2, 0, 1, 3, 3, 1, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0000 1111 *


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1100
        {3, 0, 4, 3, 3, 4, 2, 1, 2, 8, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1101 +
        {3, 0, 8, 3, 3, 4, 2, 1, 2, 4, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0001 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1010
        {3, 0, 1, 5, 0, 5, 3, 3, 9, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1011 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1100
        {3, 0, 1, 9, 0, 5, 3, 3, 5, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1101 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0010 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0001
        {1, 4, 1, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0010 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0011
        {4, 3, 0, 4, 3, 4, 5, 3, 5, 2, 8, 1, 9, MX, MX, MX, MX, MX, MX},	// 0011 0100 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1100
        {3, 3, 0, 4, 3, 4, 5, 3, 5, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1101 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0011 1110
        {4, 3, 0, 8, 3, 8, 9, 3, 9, 2, 4, 1, 5, MX, MX, MX, MX, MX, MX},	// 0011 1111 +


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0110
        {3, 0, 10, 3, 0, 1, 6, 1, 2, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 0111 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1010
        {3, 0, 6, 3, 0, 1, 6, 1, 2, 10, MX, MX, MX, MX, MX, MX, MX, MX, MX},// 0100 1011 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0100 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0101
        {2, 1, 2, 4, 6, 4, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0110 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 0111
        {4, 0, 4, 3, 6, 3, 4, 1, 2, 8, 10, 8, 2, MX, MX, MX, MX, MX, MX},	// 0101 1000 +
        {2, 0, 4, 3, 6, 3, 4, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1001 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1010
        {4, 0, 8, 3, 10, 3, 8, 1, 2, 4, 6, 4, 2, MX, MX, MX, MX, MX, MX},	// 0101 1011 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0101 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0011
        {1, 2, 6, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0100 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0101
        {4, 2, 10, 9, 0, 1, 5, 0, 5, 6, 0, 6, 3, MX, MX, MX, MX, MX, MX},	// 0110 0110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1010
        {3, 0, 1, 5, 0, 5, 6, 0, 6, 3, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1011 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1100
        {4, 2, 6, 5, 0, 1, 9, 0, 9, 10, 0, 10, 3, MX, MX, MX, MX, MX, MX},// 0110 1101 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 0110 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 0001
        {4, 2, 10, 9, 4, 5, 6, 0, 4, 6, 0, 6, 3, MX, MX, MX, MX, MX, MX},		// 0111 0010 *+
        {4, 2, 6, 5, 4, 9, 10, 0, 4, 10, 0, 10, 3, MX, MX, MX, MX, MX, MX},	// 0111 0011 *+
        {4, 1, 5, 4, 8, 9, 6, 0, 8, 6, 0, 6, 3, MX, MX, MX, MX, MX, MX},		// 0111 0100 *+
        {4, 1, 8, 9, 4, 5, 6, 0, 4, 6, 0, 6, 3, MX, MX, MX, MX, MX, MX},		// 0111 0101 *+
        {2, 1, 9, 4, 2, 6, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 0110 *+
        {2, 1, 5, 4, 2, 6, 9, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 0111 *+
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 1000
        {5, 4, 5, 6, 0, 4, 6, 0, 6, 3, 1, 9, 8, 2, 10, 12, MX, MX, MX},	// 0111 1001 +
        {5, 2, 6, 5, 4, 9, 10, 0, 4, 10, 0, 10, 3, 1, 12, 8, MX, MX, MX},	// 0111 1010 +
        {5, 4, 5, 6, 0, 4, 6, 0, 6, 3, 2, 10, 9, 1, 12, 8, MX, MX, MX},	// 0111 1011 +
        {5, 1, 5, 4, 2, 6, 9, 8, 12, 10, 0, 8, 10, 0, 10, 3, MX, MX, MX},	// 0111 1100 +
        {5, 1, 5, 4, 8, 9, 6, 0, 8, 6, 0, 6, 3, 2, 10, 12, MX, MX, MX},	// 0111 1101 +
        {5, 2, 6, 5, 1, 9, 4, 8, 12, 10, 0, 8, 10, 0, 10, 3, MX, MX, MX},	// 0111 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 0111 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 0110
        {3, 0, 1, 7, 1, 2, 7, 2, 3, 11, MX, MX, MX, MX, MX, MX, MX, MX, MX},// 1000 0111 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1101
        {3, 0, 1, 11, 1, 2, 7, 2, 3, 7, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1000 1111


        {4, 0, 4, 7, 2, 8, 1, 2, 11, 8, 2, 3, 11, MX, MX, MX, MX, MX, MX},// 1001 0000 +
        {1, 0, 4, 7, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0001 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0010
        {4, 0, 8, 11, 2, 4, 1, 2, 7, 4, 2, 3, 7, MX, MX, MX, MX, MX, MX},	// 1001 0011 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1101
        {3, 2, 4, 1, 2, 7, 4, 2, 3, 7, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1110 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1001 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0010
        {2, 0, 1, 5, 0, 5, 7, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0011 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0100
        {4, 0, 1, 9, 0, 9, 11, 3, 7, 5, 3, 5, 2, MX, MX, MX, MX, MX, MX},	// 1010 0101 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1011
        {2, 3, 7, 5, 3, 5, 2, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1100 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1101
        {4, 0, 1, 5, 0, 5, 7, 3, 11, 9, 3, 9, 2, MX, MX, MX, MX, MX, MX},	// 1010 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},	// 1010 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 0001
        {4, 1, 9, 8, 4, 5, 7, 3, 7, 5, 2, 3, 5, MX, MX, MX, MX, MX, MX},		// 1011 0010 *+
        {4, 1, 5, 4, 8, 9, 7, 3, 7, 9, 2, 3, 9, MX, MX, MX, MX, MX, MX},		// 1011 0011 *+
        {4, 0, 4, 7, 8, 5, 11, 3, 11, 5, 2, 3, 5, MX, MX, MX, MX, MX, MX},	// 1011 0100 *+
        {4, 0, 8, 11, 4, 5, 7, 3, 7, 5, 2, 3, 5, MX, MX, MX, MX, MX, MX},		// 1011 0101 *+
        {2, 0, 8, 7, 1, 5, 4, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 0110 *+
        {2, 0, 4, 7, 1, 5, 8, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 0111 *+
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 1000
        {5, 4, 5, 7, 3, 7, 5, 2, 3, 5, 0, 8, 11, 1, 9, 12, MX, MX, MX},	// 1011 1001 +
        {5, 1, 5, 4, 8, 9, 7, 3, 7, 9, 2, 3, 9, 0, 12, 11, MX, MX, MX},	// 1011 1010 +
        {5, 4, 5, 7, 3, 7, 5, 2, 3, 5, 1, 9, 8, 0, 12, 11, MX, MX, MX},	// 1011 1011 +
        {5, 0, 4, 7, 1, 5, 8, 12, 9, 11, 3, 11, 9, 2, 3, 9, MX, MX, MX},	// 1011 1100 +
        {5, 0, 4, 7, 8, 5, 11, 3, 11, 5, 2, 3, 5, 1, 9, 12, MX, MX, MX},	// 1011 1101 +
        {5, 1, 5, 4, 0, 8, 7, 12, 9, 11, 3, 11, 9, 2, 3, 9, MX, MX, MX},	// 1011 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1011 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0001
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0010
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0110
        {3, 1, 7, 0, 1, 6, 7, 1, 2, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 0111 *
        {1, 3, 7, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1000 *
        {4, 1, 11, 0, 1, 10, 11, 1, 2, 10, 3, 7, 6, MX, MX, MX, MX, MX, MX},	// 1100 1001 +
        {4, 1, 7, 0, 1, 6, 7, 1, 2, 6, 3, 11, 10, MX, MX, MX, MX, MX, MX},	// 1100 1010 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1011
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1100
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1101
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1110
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1100 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 0001
        {4, 0, 8, 11, 4, 6, 7, 2, 6, 4, 1, 2, 4, MX, MX, MX, MX, MX, MX},		// 1101 0010 *+
        {4, 0, 4, 7, 8, 6, 11, 2, 6, 8, 1, 2, 8, MX, MX, MX, MX, MX, MX},		// 1101 0011 *+
        {4, 3, 7, 6, 4, 10, 11, 2, 10, 4, 1, 2, 4, MX, MX, MX, MX, MX, MX},	// 1101 0100 *+
        {4, 3, 11, 10, 4, 6, 7, 2, 6, 4, 1, 2, 4, MX, MX, MX, MX, MX, MX},	// 1101 0101 *+
        {2, 0, 4, 7, 3, 11, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 0110 *+
        {2, 0, 4, 11, 3, 7, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 0111 *+
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 1000
        {5, 4, 6, 7, 2, 6, 4, 1, 2, 4, 3, 11, 10, 0, 8, 12, MX, MX, MX},	// 1101 1001 +
        {5, 0, 4, 7, 8, 6, 11, 2, 6, 8, 1, 2, 8, 3, 12, 10, MX, MX, MX},	// 1101 1010 +
        {5, 4, 6, 7, 2, 6, 4, 1, 2, 4, 0, 8, 11, 3, 12, 10, MX, MX, MX},	// 1101 1011 +
        {5, 3, 7, 6, 0, 4, 11, 8, 10, 12, 2, 10, 8, 1, 2, 8, MX, MX, MX},	// 1101 1100 +
        {5, 3, 7, 6, 4, 10, 11, 2, 10, 4, 1, 2, 4, 0, 8, 12, MX, MX, MX},	// 1101 1101 +
        {5, 0, 4, 7, 3, 11, 6, 8, 10, 12, 2, 10, 8, 1, 2, 8, MX, MX, MX},	// 1101 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1101 1111


        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 0000
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 0001
        {4, 3, 11, 10, 5, 6, 7, 1, 5, 7, 0, 1, 7, MX, MX, MX, MX, MX, MX},	// 1110 0010 *+
        {4, 3, 7, 6, 5, 10, 11, 1, 5, 11, 0, 1, 11, MX, MX, MX, MX, MX, MX},	// 1110 0011 *+
        {4, 2, 6, 5, 9, 10, 7, 1, 9, 7, 0, 1, 7, MX, MX, MX, MX, MX, MX},		// 1110 0100 *+
        {4, 2, 10, 9, 5, 6, 7, 1, 5, 7, 0, 1, 7, MX, MX, MX, MX, MX, MX},		// 1110 0101 *+
        {2, 3, 7, 6, 2, 10, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 0110 *+
        {2, 3, 7, 10, 2, 6, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 0111 *+
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 1000
        {5, 5, 6, 7, 1, 5, 7, 0, 1, 7, 2, 10, 9, 3, 11, 12, MX, MX, MX},	// 1110 1001 +
        {5, 3, 7, 6, 5, 10, 11, 1, 5, 11, 0, 1, 11, 2, 12, 9, MX, MX, MX},	// 1110 1010 +
        {5, 5, 6, 7, 1, 5, 7, 0, 1, 7, 3, 11, 10, 2, 12, 9, MX, MX, MX},	// 1110 1011 +
        {5, 2, 6, 5, 3, 7, 10, 9, 12, 11, 1, 9, 11, 0, 1, 11, MX, MX, MX},	// 1110 1100 +
        {5, 2, 6, 5, 9, 10, 7, 1, 9, 7, 0, 1, 7, 3, 11, 12, MX, MX, MX},	// 1110 1101 +
        {5, 3, 7, 6, 2, 10, 5, 9, 12, 11, 1, 9, 11, 0, 1, 11, MX, MX, MX},	// 1110 1110 +
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},		// 1110 1111


        {2, 0, 4, 7, 2, 6, 5, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0000 *
        {3, 3, 7, 6, 1, 5, 4, 2, 10, 9, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0001 *
        {3, 2, 6, 5, 3, 7, 10, 1, 9, 4, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0010 *
        {3, 3, 7, 6, 2, 10, 5, 1, 9, 4, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0011 *
        {3, 1, 5, 4, 2, 6, 9, 3, 7, 10, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0100 *
        {3, 1, 5, 4, 3, 7, 6, 2, 10, 9, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0101 *
        {3, 2, 6, 5, 1, 9, 4, 3, 7, 10, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0110 *
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 0111
        {0, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 1000
        {6, 3, 7, 6, 1, 5, 4, 2, 10, 9, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1001 +
        {6, 2, 6, 5, 3, 7, 10, 1, 9, 4, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1010 +
        {6, 3, 7, 6, 2, 10, 5, 1, 9, 4, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1011 +
        {6, 1, 5, 4, 2, 6, 9, 3, 7, 10, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1100 +
        {6, 1, 5, 4, 3, 7, 6, 2, 10, 9, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1101 +
        {6, 2, 6, 5, 1, 9, 4, 3, 7, 10, 0, 8, 11, 8, 12, 11, 12, 13, 11},	// 1111 1110 +
        {2, 1, 5, 4, 3, 7, 6, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX, MX},			// 1111 1111 *
};



const int g_EdgeTable[16][4] = {
        {MX, MX, MX, MX},	// 0
        {4, MX, MX, MX},	// 1
        {5, MX, MX, MX},	// 2
        {4, 5, MX, MX},	// 3

        {6, MX, MX, MX},	// 4
        {4, 6, MX, MX},	// 5
        {5, 6, MX, MX},	// 6
        {4, 5, 6, MX},	// 7

        {7, MX, MX, MX},	// 8
        {7, 4, MX, MX},	// 9
        {5, 7, MX, MX},	// 10
        {7, 4, 5, MX},	// 11

        {6, 7, MX, MX},	// 12
        {6, 7, 4, MX},	// 13
        {5, 6, 7, MX},	// 14
        {4, 5, 6, 7},	// 15
};



const int g_NodeTable[4][8] = {
        {3, 1, 2, 3,  4, 5, 6, 7},	// 1110
        {0, 2, 3, 0,  5, 6, 7, 4},	// 1101
        {1, 3, 0, 1,  6, 7, 4, 5},	// 1011
        {2, 0, 1, 2,  7, 4, 5, 6},	// 0111
};


const int g_VrtRotTable[4][14] = {
        {0, 1, 2, 3,  4, 5, 6, 7,  8, 9, 10, 11,  12, 13},
        {1, 2, 3, 0,  5, 6, 7, 4,  9, 10, 11, 8,  12, 13},
        {2, 3, 0, 1,  6, 7, 4, 5,  10, 11, 8, 9,  12, 13},
        {3, 0, 1, 2,  7, 4, 5, 6,  11, 8, 9, 10,  12, 13}
};

#undef MX


template<class T>
inline void RX_CLAMP(T &x, const T &a, const T &b){ x = (x < a ? a : (x > b ? b : x)); }

struct rxSSParticle
{
        Vector xp;
        Vector rp;
};


struct rxSSVertex
{
        Vector pos;
        float depth;
        int edge;
        unsigned int c;  //critical flag 0 regular: 1 max: 2 min:
        Vector avg_pos;
        float avg_num;
        bool b;//boundary flag
        std::vector< unsigned int> Edges;
        rxSSVertex(Vector p, int e = 0) : pos(Vector(p.x, p.y, p.z)), depth(p.z), edge(e),b(false), c(0) {}
};


struct rxSSEdge
{
        Vector x0, x1;
        float depth;
        bool xy;
        bool silhouette;
        unsigned int front_vertex;
        float dx;
};

struct bEdge
{
 pair <int,int>            Edge;
 float lenght;
 
 
};


struct rxSSGrid
{
        unsigned int i, j;
        unsigned int node_vrts[4];
        unsigned int num_nv;
        unsigned int edge_vrts[4];
        unsigned int num_ev;
        unsigned int back_vrts[6];
        unsigned int num_bv;

        float node_depth[4];

        unsigned int table_index0;
        unsigned int table_index1;
        //unsigned int mesh_num;
       // unsigned int mesh[6];
        unsigned int back2;
       // unsigned int v[14];
};


struct GridCell{
    std::vector<unsigned int> listSSvrtx;
    //Vec2 pmin;
    //Vec2 pmax;
    bool empty;
    bool vp;
    //Vec2 C; //centroid

    GridCell(bool e=true, bool v=false): empty(e),vp(v)  {}
};

struct GridDel{
    std::vector<GridCell> vc;
    Vec2 C; //centroid
    Vec2 Pmin;//Pmax;
    int nh,nv;
};
struct bVertex
{
  int id;
  int nedges;
  bVertex():nedges(0) {}
};

class cell
{
public:
  std::vector<int> pIds;
  std::vector<float> zValues;
  std::vector<Vector> R;
};

class BboxScreenSpace
{
public:
  float minx,miny,maxx,maxy;
  Vec2 Center;
  int minxId,minyId,maxxId,maxyId;
 
};
typedef MyKdTree SearchStructure;

class SpaceMesh{
public:

typedef void (SpaceMesh::*FuncTableIndex)(int&, int&, int[], rxSSGrid*);
   
  float screenSpace,dx,dy;
  int Nx,Ny,iNx,iNy,Li;
  std::vector<cell> cells;
  std::vector<float> vSSDMap;
  
  std::vector< Vector > vrts;
  std::vector< Vector > normals;
  vector<rxSSParticle> m_vSSPrts;
  of::ofList<int> bv;
  vector<bVertex> vBVertex;
  vector<rxSSEdge> *m_vSSEdge;
  vector<bEdge> vbEdge;
  vector<rxSSVertex> m_vSSEdgeVertex;
  vector<rxSSVertex> m_vSSVertex;
  GridDel Del;
  SearchStructure searcher;
  std::vector<std::vector<NeighborData> > neighbors;
  vector<rxSSGrid> *m_vSSMGrid;
  vector<int> m_vMeshGrid;
  std::vector<int> BV;
  std::vector <int> freebPoints;
  FuncTableIndex m_FuncTableIndex[25];
  int m_iNumNodeVrts;
  int m_iNumEdgeVrts;
  int m_iNumMesh;
  int pnum;
  float P[16];
  float MV[16];
  rxMatrix4 Pr;
  rxMatrix4 MVi;
  float m_fSSZmax;
  std::vector< std::vector<int> > polys;
  std::vector< std::vector<int> > polysDel;
  int vp[4];
  std::vector< std::vector<double> > m_vFilter;
  BboxScreenSpace Box;
  void updateTableIndexE0N4(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE1(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE2N4(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE3N23(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE3N4(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE4N2(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE4N3(int &table_index, int &vrot, int v[], rxSSGrid *g);
  void updateTableIndexE4N4(int &table_index, int &vrot, int v[], rxSSGrid *g);

  inline rxMatrix4 GetMatrixGL(float m[16])
  {
          return rxMatrix4(m[0], m[4], m[8],  m[12],
                                           m[1], m[5], m[9],  m[13],
                                           m[2], m[6], m[10], m[14],
                                           m[3], m[7], m[11], m[15]);
  }

  void initCells( int p)
  {
      int vp[4];
      glGetIntegerv(GL_VIEWPORT, vp);
    cells.clear();
    BV.clear();
    screenSpace=0.8;
    iNx=0;
    iNy=0;
    Li=0;
    pnum=p;
    m_fSSZmax=1.18;
    Nx =   vp[2]/screenSpace ;
    Ny =   vp[3]/screenSpace ;
    Box.maxx=0;Box.maxy=0;Box.minx=vp[2];Box.miny=vp[3];

        dx = (float)vp[2]/(float)(Nx);
        dy = (float)vp[3]/(float)(Ny);
        if((int)vSSDMap.size() != (Nx+1)*(Ny+1)){
                vSSDMap.clear();
                vSSDMap.resize((Nx+1)*(Ny+1));
        }

      /*  if((int)cells.size() != (Nx+1)*(Ny+1)){
                cells.clear();
                cells.resize((Nx+1)*(Ny+1));
        }*/


        for(int i = 0; i < (Nx+1)*(Ny+1); ++i){
                vSSDMap[i] = INF;
        }
      m_vFilter = CalBinomials(21);
      m_FuncTableIndex[0]  = 0;
      m_FuncTableIndex[1]  = 0;
      m_FuncTableIndex[2]  = 0;
      m_FuncTableIndex[3]  = 0;
      m_FuncTableIndex[4]  = &SpaceMesh::updateTableIndexE0N4;

      m_FuncTableIndex[5]  = &SpaceMesh::updateTableIndexE1;
      m_FuncTableIndex[6]  = &SpaceMesh::updateTableIndexE1;
      m_FuncTableIndex[7]  = &SpaceMesh::updateTableIndexE1;
      m_FuncTableIndex[8]  = &SpaceMesh::updateTableIndexE1;
      m_FuncTableIndex[9]  = &SpaceMesh::updateTableIndexE1;

      m_FuncTableIndex[10] = 0;
      m_FuncTableIndex[11] = 0;
      m_FuncTableIndex[12] = 0;
      m_FuncTableIndex[13] = 0;
      m_FuncTableIndex[14] = &SpaceMesh::updateTableIndexE2N4;

      m_FuncTableIndex[15] = 0;
      m_FuncTableIndex[16] = 0;
      m_FuncTableIndex[17] = &SpaceMesh::updateTableIndexE3N23;
      m_FuncTableIndex[18] = &SpaceMesh::updateTableIndexE3N23;
      m_FuncTableIndex[19] = &SpaceMesh::updateTableIndexE3N4;

      m_FuncTableIndex[20] = 0;
      m_FuncTableIndex[21] = 0;
      m_FuncTableIndex[22] = &SpaceMesh::updateTableIndexE4N2;
      m_FuncTableIndex[23] = &SpaceMesh::updateTableIndexE4N3;
      m_FuncTableIndex[24] = &SpaceMesh::updateTableIndexE4N4;
      m_iNumNodeVrts = 0;
      m_iNumEdgeVrts = 0;
      m_iNumMesh = 0;
      polys.clear();
      polysDel.clear();

      
      if ((int)m_vSSPrts.size() != pnum) {
      m_vSSPrts.clear ();
      m_vSSPrts.resize (pnum);
      }
    m_vSSVertex.clear();
    normals.clear();
    vrts.clear();
    m_vSSEdge= new vector<rxSSEdge>;
    m_vSSMGrid =  new vector<rxSSGrid>;
    
  }

  static std::vector< std::vector<double> > CalBinomials(int b)
  {
          std::vector< std::vector<double> > bs;
          std::vector<double> f, tmp;
          f.resize(b+1);
          tmp.resize(b+1);

          bs.resize(b+1);

          double a = 1.0;

          for(int i = 0; i < b+1; ++i){
                  f[i]   = (i == 0 ? 1 : 0);
                  tmp[i] = (i == 0 ? 1 : 0);
          }

          for(int k = 0; k < b+1; ++k){
                  for(int i = 1; i < k+1; ++i){
                          tmp[i] = f[i-1]+f[i];
                  }

                  for(int i = 1; i < k+1; ++i){
                          f[i] = tmp[i];
                  }

                  bs[k].resize(k+1);
                  for(int i = 0; i < k+1; ++i){
                          bs[k][i] = f[i]*a;
                  }

                  a *= 0.5;
          }

          return bs;
  }


  bool MygluInvertMatrix(const float m[16], float invOut[16])
  {
      double inv[16], det;
      int i;

      inv[0] = m[5]  * m[10] * m[15] -
               m[5]  * m[11] * m[14] -
               m[9]  * m[6]  * m[15] +
               m[9]  * m[7]  * m[14] +
               m[13] * m[6]  * m[11] -
               m[13] * m[7]  * m[10];

      inv[4] = -m[4]  * m[10] * m[15] +
                m[4]  * m[11] * m[14] +
                m[8]  * m[6]  * m[15] -
                m[8]  * m[7]  * m[14] -
                m[12] * m[6]  * m[11] +
                m[12] * m[7]  * m[10];

      inv[8] = m[4]  * m[9] * m[15] -
               m[4]  * m[11] * m[13] -
               m[8]  * m[5] * m[15] +
               m[8]  * m[7] * m[13] +
               m[12] * m[5] * m[11] -
               m[12] * m[7] * m[9];

      inv[12] = -m[4]  * m[9] * m[14] +
                 m[4]  * m[10] * m[13] +
                 m[8]  * m[5] * m[14] -
                 m[8]  * m[6] * m[13] -
                 m[12] * m[5] * m[10] +
                 m[12] * m[6] * m[9];

      inv[1] = -m[1]  * m[10] * m[15] +
                m[1]  * m[11] * m[14] +
                m[9]  * m[2] * m[15] -
                m[9]  * m[3] * m[14] -
                m[13] * m[2] * m[11] +
                m[13] * m[3] * m[10];

      inv[5] = m[0]  * m[10] * m[15] -
               m[0]  * m[11] * m[14] -
               m[8]  * m[2] * m[15] +
               m[8]  * m[3] * m[14] +
               m[12] * m[2] * m[11] -
               m[12] * m[3] * m[10];

      inv[9] = -m[0]  * m[9] * m[15] +
                m[0]  * m[11] * m[13] +
                m[8]  * m[1] * m[15] -
                m[8]  * m[3] * m[13] -
                m[12] * m[1] * m[11] +
                m[12] * m[3] * m[9];

      inv[13] = m[0]  * m[9] * m[14] -
                m[0]  * m[10] * m[13] -
                m[8]  * m[1] * m[14] +
                m[8]  * m[2] * m[13] +
                m[12] * m[1] * m[10] -
                m[12] * m[2] * m[9];

      inv[2] = m[1]  * m[6] * m[15] -
               m[1]  * m[7] * m[14] -
               m[5]  * m[2] * m[15] +
               m[5]  * m[3] * m[14] +
               m[13] * m[2] * m[7] -
               m[13] * m[3] * m[6];

      inv[6] = -m[0]  * m[6] * m[15] +
                m[0]  * m[7] * m[14] +
                m[4]  * m[2] * m[15] -
                m[4]  * m[3] * m[14] -
                m[12] * m[2] * m[7] +
                m[12] * m[3] * m[6];

      inv[10] = m[0]  * m[5] * m[15] -
                m[0]  * m[7] * m[13] -
                m[4]  * m[1] * m[15] +
                m[4]  * m[3] * m[13] +
                m[12] * m[1] * m[7] -
                m[12] * m[3] * m[5];

      inv[14] = -m[0]  * m[5] * m[14] +
                 m[0]  * m[6] * m[13] +
                 m[4]  * m[1] * m[14] -
                 m[4]  * m[2] * m[13] -
                 m[12] * m[1] * m[6] +
                 m[12] * m[2] * m[5];

      inv[3] = -m[1] * m[6] * m[11] +
                m[1] * m[7] * m[10] +
                m[5] * m[2] * m[11] -
                m[5] * m[3] * m[10] -
                m[9] * m[2] * m[7] +
                m[9] * m[3] * m[6];

      inv[7] = m[0] * m[6] * m[11] -
               m[0] * m[7] * m[10] -
               m[4] * m[2] * m[11] +
               m[4] * m[3] * m[10] +
               m[8] * m[2] * m[7] -
               m[8] * m[3] * m[6];

      inv[11] = -m[0] * m[5] * m[11] +
                 m[0] * m[7] * m[9] +
                 m[4] * m[1] * m[11] -
                 m[4] * m[3] * m[9] -
                 m[8] * m[1] * m[7] +
                 m[8] * m[3] * m[5];

      inv[15] = m[0] * m[5] * m[10] -
                m[0] * m[6] * m[9] -
                m[4] * m[1] * m[10] +
                m[4] * m[2] * m[9] +
                m[8] * m[1] * m[6] -
                m[8] * m[2] * m[5];

      det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

      if (det == 0)
          return false;

      det = 1.0 / det;

      for (i = 0; i < 16; i++)
          invOut[i] = inv[i] * det;

      return true;
  }

  inline int MygluUnproject(int xp,int yp,float zp,rxMatrix4 &MV, rxMatrix4 &P, float *rx,float *ry, float *rz)
  {
      int vp[4];
      Vec4 xd,x;

      glGetIntegerv(GL_VIEWPORT, vp);
      rxMatrix4 invP, invMV ,IMVQ;
      invP=P.Inverse();
      invMV=MV.Inverse();
      IMVQ = invMV*invP;


      xd[0] = -1.0+2.0*xp/vp[2];
      xd[1] = -1.0+2.0*yp/vp[3];
      xd[3] = (1.0-invP(3,2)*zp)/(invP(3,0)*xd[0]+invP(3,1)*xd[1]+invP(3,3));

      xd[0] *= xd[3];
      xd[1] *= xd[3];
      xd[2] = zp;
      x=IMVQ*xd;
      *rx=x[0];
      *ry=x[1];
      *rz=x[2];
  }

  Vector worldScreenSpace(int x, int y,float z=0) {
    int vp[4];
    float MV[16], P[16];
    static unsigned short tmp_mz;
    static GLfloat mz;
    float ox, oy, oz,
                posX,
                posY,
                posZ;

    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_MODELVIEW_MATRIX, MV);
    glGetFloatv(GL_PROJECTION_MATRIX, P);
    //int realy =vp[3]-y;
    Pr = GetMatrixGL(P);
    MVi = GetMatrixGL(MV);
    glReadBuffer(GL_FRONT);
    glReadPixels(x, y,1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT,
                            &tmp_mz);

            mz = (float) tmp_mz / 65535.0;  // maximum value for unsigned_short

            MygluUnproject(x, y, z, MVi, Pr,
                             &posX, &posY, &posZ);

    ox = posX,
    oy = posY;
    oz = posZ;
    //oz = 0.0;

    Vector o(ox, oy, oz);

    return o;
  }
  
  void  MultiM(float M1[16],float M2[16],float res[16])
  {

      int i,j,k;
      for(i=0;i<4;i++)
          for(j=0;j<4;j++)
          {
              res[i*4+j]=0.0;
              for(k=0;k<4;k++)
              {
                  res[i*4+j]+=M1[i*4+k]*M2[k*4+j];
              }
          }
  }

  inline void MultiV(float objx,float objy,float objz,float objw, float M[16],float *camx, float *camy,float *camz,float *camw)
  {
    *camx = M[0]*objx+M[1]*objy+M[2]*objz+M[3]*objw;
    *camy = M[4]*objx+M[5]*objy+M[6]*objz+M[7]*objw;
    *camz = M[8]*objx+M[9]*objy+M[10]*objz+M[11]*objw;
    *camw = M[12]*objx+M[13]*objy+M[14]*objz+M[15]*objw;
  }

  

  
  inline int MygluProjectf(float objx, float objy, float objz, float modelview[16], float projection[16], int *viewport, float *wx, float *wy, float *wz, float *ww)
    {
      float in0, in1,in2,in3;
          float out0,out1,out2,out3;

          in0=objx;
          in1=objy;
          in2=objz;
          in3=1.0;
          MultiV(in0,in1,in2,in3,modelview, &out0,&out1,&out2,&out3);
          MultiV(out0,out1,out2,out3,projection, &in0,&in1,&in2,&in3);
          if (in3 == 0.0) return 0;
          *ww=in3;
          in0 /= in3;
          in1 /= in3;
          in2 /= in3;
          /* Map x, y and z to range 0-1 */
          in0 = in0 * 0.5 + 0.5;
          in1 = in1 * 0.5 + 0.5;
          in2 = in2 * 0.5 + 0.5;

          /* Map x,y to viewport */
          in0 = in0 * viewport[2] + viewport[0];
          in1 = in1 * viewport[3] + viewport[1];

          *wx=in0;
          *wy=in1;
          *wz=in2;
            return 1;

    }


  
  void setupDephMap( SPH &sph, bool edgeok=false)
  {

    initCells(sph.particles.size());
    float P[16];
    float w,iR,rp2,f;
    float wx,wy,wz;
    float MV[16];

    int vp[4];
    int xp,yp ;

   // glutPostRedisplay();
    glMatrixMode(GL_MODELVIEW);
    //glPushMatrix();
    //glLoadIdentity();

    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_MODELVIEW_MATRIX, MV);
    glGetFloatv(GL_PROJECTION_MATRIX, P);
    Pr = GetMatrixGL(P);
    MVi = GetMatrixGL(MV);

    rxMatrix4 PMV = Pr*MVi;


    int pi;
    Vector tr,rp;
    tr.x = 0.5*sph.smoothing_radius()*vp[2]*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]);
    tr.y = 0.5*sph.smoothing_radius()*vp[3]*sqrt(P[4]*P[4]+P[5]*P[5]+P[6]*P[6]);
    tr.z = 0.5*sph.smoothing_radius()*sqrt(P[8]*P[8]+P[9]*P[9]+P[10]*P[10]);
    rp.x = tr.x/1.0/2.0;
    rp.y = tr.y/1.0/2.0;
    rp.z = tr.z;
     rp2=rp.x*rp.x;

    for(pi=0;pi< sph.particles.size();pi++)
    {
	Particle &p =sph.particles[pi];
        //p.color=red;




        Vec4 x = Vec4(p.x.x,p.x.y,p.x.z, 1.0);


        Vec4 xd = PMV*x;

        float invw = 1/xd[3];
        xp =   vp[2]*(0.5+0.5*xd[0]*invw);
        yp =   vp[3]*(0.5+0.5*xd[1]*invw);
        wz=xd[2];
        
        if (Box.maxx<=xp+rp.x)
             {
               Box.maxx =  xp+rp.x;
               //Box.maxxId = i;
             }
             if (Box.maxy<=yp+rp.x)
             {
               Box.maxy =  yp+rp.x;
               //Box.maxyId = i;
             }
             if (Box.minx>=xp-rp.x)
             {
               Box.minx =  xp-rp.x;
               //Box.minxId = i;
             }
             if (Box.miny>=yp-rp.x)
             {
               Box.miny =  yp-rp.x;
               //Box.minyId = i;
             }

         if(xp < 0 || xp >= vp[2] || yp < 0 || yp >= vp[3])
          continue;
         


          m_vSSPrts[pi].rp = rp;
          m_vSSPrts[pi].xp.x = xp;
          m_vSSPrts[pi].xp.y = yp;
          m_vSSPrts[pi].xp.z = wz;

                 int cen[2];    
                cen[0] = (xp/dx)+1;
                cen[1] = (yp/dy)+1;

                int minp[2], maxp[2];
                minp[0] = cen[0]-(rp.x/dx+1);
                minp[1] = cen[1]-(rp.y/dy+1);
                maxp[0] = cen[0]+(rp.x/dx+1);
                maxp[1] = cen[1]+(rp.y/dy+1);
                RX_CLAMP(minp[0], 0, Nx);
                RX_CLAMP(minp[1], 0, Ny);
                RX_CLAMP(maxp[0], 0, Nx);
                RX_CLAMP(maxp[1], 0, Ny);
          
                for(int j = minp[1]; j <= maxp[1]; ++j){
                        for(int i = minp[0]; i <= maxp[0]; ++i){

            iR =(float) ((i*dx)-xp)*(i*dx-xp)+(j*dy-yp)*(j*dy-yp);
            if(iR<=rp2)
	    {
              
	      f= 1.0 - iR/rp2;


              double zij = vSSDMap[i+j*(Nx+1)];
              double hij = f;

              double z = wz-rp.z*hij;
              if( z < zij){
                      vSSDMap[i+j*(Nx+1)] = z;
              }

	    }
             }
        }
    }
    
    float maxdimx,maxdimy;
        maxdimx = Box.maxx - Box.minx;
         maxdimy = (Box.maxy-Box.miny);
        Box.Center[0] = (Box.maxx + Box.minx)*0.5;
        Box.Center[1] = (Box.maxy + Box.miny)*0.5;
        
        maxdimx *= 1.5;maxdimy *= 1.4;
        Box.minx = Box.Center[0]-maxdimx*0.5; Box.maxx = Box.Center[0]+maxdimx*0.5;
        Box.miny = Box.Center[1]-maxdimy*0.5; Box.maxy = Box.Center[1]+maxdimy*0.5;

     ApplyDepthFilter(vSSDMap, Nx+1, Ny+1, 6);

     m_iNumNodeVrts = CalNodeVertex(Nx, Ny, dx, dy,m_vSSMGrid, vSSDMap);

     int edge_num = DetectSilhouetteEdgeVertex(Nx, Ny, dx, dy, vSSDMap, m_vSSPrts, vp[2], vp[3]);



     m_iNumEdgeVrts = CalEdgeVertex(Nx, Ny, dx, dy, vSSDMap, m_vSSEdge);

     m_iNumMesh = CalMesh(Nx, Ny, m_vSSMGrid, polys, 0);

     ApplySilhoutteSmoothing(m_vSSVertex, polys, 1);


     rxMatrix4 Q = Pr.Inverse();


     rxMatrix4 IMV = MVi.Inverse();

     rxMatrix4 IMVQ = IMV*Q;


     Vec4 xd;



    //int realy;
     int nv =  m_vSSVertex.size();
     vrts.resize(nv);
     for(int i = 0; i < nv; ++i){
             Vector xpf = m_vSSVertex[i].pos;
             m_vSSVertex[i].b=false;
             
             //realy = vp[3]-xpf.y;
             xd[0] = -1.0+2.0*xpf.x/vp[2];
             xd[1] = -1.0+2.0*xpf.y/vp[3];
             xd[3] = (1.0-Q(3,2)*xpf.z)/(Q(3,0)*xd[0]+Q(3,1)*xd[1]+Q(3,3));

             xd[0] *= xd[3];
             xd[1] *= xd[3];
             xd[2] = xpf.z;
             Vec4 x = IMVQ*xd;
             vrts[i] = Vector(x[0], x[1], x[2]);
     }

     
     //int cok = FindingBoundaryPoints(rp.x);
     //findingDelaunayMesh(polysDel);

     CalVertexNormals(vrts,vrts.size(),polys,polys.size(),normals);
      //m_vSSVertex.clear();
      //m_vSSVertex = std::vector<rxSSVertex>();
      m_vSSMGrid->clear();
      delete m_vSSMGrid;
      //m_vSSEdge->clear();
      //delete m_vSSEdge ;
      m_vMeshGrid.clear();
      vSSDMap.clear();
      vSSDMap = std::vector<float>();
      
      //DrawBoundingBox2D(Box);
     DrawBoundaryVertex2D(rp.x);
     
     DrawSSEdge(vp[3]);
     //DrawSSVertex(Box);
     DrawSSMesh(polys,blue,edgeok);

  }


  inline float min(float x1,float x2)
  {
      if(x1<x2)
          return x1;
      return x2;
  }

  inline float max(float x1,float x2)
  {
      if(x1>x2)
          return x1;
      return x2;
  }

void DrawSSMesh(std::vector<std::vector<int> > &polys,TColorRGB c, bool edgeok=false)
{

    int nt =polys.size();
    glColor3f(c.R, c.G, c.B);
    glBegin(GL_TRIANGLES);

    for(unsigned int i = 0; i < nt; i++){
        int id0, id1, id2;
        id0 = polys[i][0];
        id1 = polys[i][1];
        id2 = polys[i][2];

        glNormal3f(normals[id0].x, normals[id0].y, normals[id0].z);

        glVertex3f(vrts[id0].x,vrts[id0].y,vrts[id0].z);

        glNormal3f(normals[id1].x, normals[id1].y, normals[id1].z);

        glVertex3f(vrts[id1].x,vrts[id1].y,vrts[id1].z);

        glNormal3f(normals[id2].x, normals[id2].y, normals[id2].z);

        glVertex3f(vrts[id2].x,vrts[id2].y,vrts[id2].z);

    }
    glEnd();

    if(edgeok)
    {
        glColor3f(0.4, 0.4, 0.4);



        for(unsigned int i = 0; i < nt; i++){
            int id0, id1, id2;
            id0 = polys[i][0];
            id1 = polys[i][1];
            id2 = polys[i][2];
            glBegin(GL_LINE_LOOP);


            glVertex3f(vrts[id0].x,vrts[id0].y,vrts[id0].z);



            glVertex3f(vrts[id1].x,vrts[id1].y,vrts[id1].z);



            glVertex3f(vrts[id2].x,vrts[id2].y,vrts[id2].z);
            glEnd();

        }


    }

}


  void CalVertexNormals(const std::vector<Vector> &vrts, unsigned int nvrts, const std::vector< std::vector<int> > &tris, unsigned int ntris, std::vector<Vector> &nrms)
  {
          unsigned int nnrms = nvrts;
          nrms.resize(nnrms);


          for(unsigned int i = 0; i < nnrms; i++){
                  nrms[i].x = 0;
                  nrms[i].y = 0;
                  nrms[i].z = 0;
          }


          for(unsigned int i = 0; i < ntris; i++){
                  Vector vec1, vec2, normal;
                  int id0, id1, id2;
                  id0 = tris[i][0];
                  id1 = tris[i][1];
                  id2 = tris[i][2];

                  vec1 = vrts[id1]-vrts[id0];
                  vec2 = vrts[id2]-vrts[id0];
                  normal = vec1^vec2;

                  nrms[id0] += normal;
                  nrms[id1] += normal;
                  nrms[id2] += normal;
          }


          for(unsigned int i = 0; i < nnrms; i++){
                  nrms[i].normalize();
          }
  }

  void DrawSSEdge(int H)
  {
          glBegin(GL_LINES);
          for(int i = 0; i <  m_vSSEdge->size(); ++i){
                  rxSSEdge &e = m_vSSEdge->at(i);

                  if(e.silhouette){
                      //float m = min(e.d0,e.d1);
		      glLineWidth(5.0);
                      Vector x0 = worldScreenSpace( (e.x0.x), (e.x0.y),e.x0.z);
                      Vector x1 = worldScreenSpace( (e.x1.x), (e.x1.y),e.x1.z);

                          glColor3d(0.0, 0.0, 0.0);
                          glVertex3d(x0.x, x0.y, x0.z);
                          glVertex3d(x1.x, x1.y, x1.z);
                  }
                  else{
                          glColor3d(0.6, 0.6, 0.6);
			  glLineWidth(2.0);
                      Vector x0 = worldScreenSpace( (e.x0.x), (e.x0.y),e.x0.z);
                      Vector x1 = worldScreenSpace( (e.x1.x), (e.x1.y),e.x1.z);

                          glColor3d(0.0, 0.0, 0.0);
                          glVertex3d(x0.x, x0.y, x0.z);
                          glVertex3d(x1.x, x1.y, x1.z);
                  }


          }
          glEnd();
  }

  void DrawSilhouetteEdge(void)
  {
          glBegin(GL_LINES);
          for(int i = 0; i <  m_vSSEdge->size(); ++i){
                  rxSSEdge &e = m_vSSEdge->at(i);
                  if(e.silhouette){
                          Vector x0 = e.x0;
                          Vector x1 = e.x1;
                          glVertex3d(x0.x, x0.y, 0.01);
                          glVertex3d(x1.x, x1.y, 0.01);
                  }
          }
          glEnd();
  }


  void ConstructGridDelCell(BboxScreenSpace &Box,float rho)
  {
      float rho2=2.0*rho;
      float rho2inv = 1/(rho2);


      Del.nh =  (Box.maxx-Box.minx)*rho2inv;
      if(Del.nh % 2 ==0)
          Del.nh+=4;
      else
          Del.nh+=3;
      Del.nv =  (Box.maxy-Box.miny)*rho2inv;

   //std::cout << " nv = " << Del.nv << std::endl;
      if(Del.nv % 2 ==0)
          Del.nv+=4;
      else
          Del.nv+=3;
      Del.C[0] =(Box.maxx+Box.minx)*0.5;
      Del.C[1] =(Box.maxy+Box.miny)*0.5;
      if(Del.vc.size()>0)
      {
          Del.vc.clear();
      }
       Del.vc.resize(Del.nh*Del.nv);
      Del.Pmin[0]= Del.C[0]-rho*(Del.nh);
      Del.Pmin[1]= Del.C[1]-rho*(Del.nv);
      //Del.Pmax[0]= Del.C[0]+rho*(Del.nh);
      //Del.Pmax[1]= Del.C[1]+rho*(Del.nv);


      //populating cells
      int nV =m_vSSVertex.size();
       int cen[2];
     for(int i = 0; i< nV; ++i){

       Vector xpf = m_vSSVertex[i].pos;

        cen[0] =  (xpf.x-Del.Pmin[0])*rho2inv;
        cen[1] =  (xpf.y-Del.Pmin[1])*rho2inv;

        //std::cout << " nh =" << Del.nh << " cen[0] = " << cen[0] << " nv =" << Del.nv << " cen[1] = " << cen[1] << std::endl;

        //std::cout << " xpf.x =" << xpf.x << " Pmin[0] = " << Del.Pmin[0] << " xpf.y =" << xpf.y << " Pmin[1] = " << Del.Pmin[1] << std::endl;


        Del.vc[cen[0]+cen[1]*(Del.nh)].listSSvrtx.push_back(i);
        Del.vc[cen[0]+cen[1]*(Del.nh)].empty=false;


      }


  }

  bool testCavityCell(int i,int j,float rho,GridCell &cn)
  {
    float x,y,dist,rho2=2.0*rho;
    int k;
    x=Del.Pmin[0]+i*rho2+rho;y=Del.Pmin[1]+j*rho2+rho;
    for(k=0;k<cn.listSSvrtx.size();k++)
    {
      Vector xpf = m_vSSVertex[cn.listSSvrtx[k]].pos;
      dist = sqrt((x-xpf.x)*(x-xpf.x)+(y-xpf.y)*(y-xpf.y));
      if(dist<rho)
	return false;
    }
    
    return true;
  }
  
  bool isbVertexInVector(std::vector<bVertex> &vBv,bVertex v)
  {
    int i,s=vBv.size();
    for(i=0;i<s;i++)
      if(vBv[i].id==v.id)
	return true;
    return false;
  }
  
  bool isPairInVector(std::vector< bEdge> &vEdge, bEdge e)
  {
    int i,s=vEdge.size();
    bEdge et;
    for(i=0;i<s;i++)
    {
      et=vEdge[i];
      if(((e.Edge.first==et.Edge.first)&&(e.Edge.second==et.Edge.second))||((e.Edge.first==e.Edge.second)&&(e.Edge.second==et.Edge.first)))
	return true;
      
    }
    return false;
  }
  
  int FindingBoundaryPoints(float rho)
  {
      
      
      ConstructGridDelCell(Box,rho);
      float rho2=2.0*rho;
      float rho95 = rho*0.95;
      int i,j,k,n,p;
            //std::vector<int> tmpV;
      of::ofList<int> tmpV;
      std::vector<int> corrTmp;
      std::vector<int> ch; //convexHullPoints
      
      //vBVertex.clear();
      //vbEdge.clear();
      freebPoints.clear();
      for(j=0;j<Del.nv;j++)
          for(i=0;i<Del.nh;i++)
          {
              GridCell &c = Del.vc[j*Del.nh+i];
              if ((c.empty)|| (testCavityCell(i,j,rho,c)==true))
              {
                  tmpV.clear();
                  corrTmp.clear();
                  ch.clear();
                  int count=0;
                  corrTmp.push_back(count);
                  //finding centroid of the cell;
                  for (k=max(0,j-1);k<min(Del.nv,j+2);k++)
                    for (n=max(0,i-1);n<min(Del.nh,i+2);n++)

                    {
                        GridCell &cn = Del.vc[k*Del.nh+n];
                        if(!cn.empty)
                            for (int m=0;m<cn.listSSvrtx.size();m++)
                            {
                                if(tmpV.inList(cn.listSSvrtx[m])==false)
                                {
                                    tmpV.insert(cn.listSSvrtx[m]);
                                    count++; corrTmp.push_back(count);
                                }



                            }
                    }
                    
                 if(tmpV.size()>0) // this is a viewpoint cell
                 {
		     c.vp=true;
                     FindConvexHull(tmpV,ch,i,j,rho);
                     for(p=0;p<ch.size();p++)
                     {
		       bVertex v;
		       v.id=tmpV.pos(ch[p]);
		       m_vSSVertex[v.id].b=true;
		       if(isbVertexInVector(vBVertex,v)==false)
			 vBVertex.push_back(v);
                         
                     }
                     if(ch.size()>1)
		     {
		       for(p=0;p<ch.size()-1;p++)
			{
			  bEdge e;
			  e.Edge.first=tmpV.pos(ch[p]);
			  e.Edge.second = tmpV.pos(ch[p+1]);
			  e.lenght = lenghtEdge(e);
			  if ((isPairInVector(vbEdge,e)==false)&&(e.lenght<rho)&&(m_vSSVertex[e.Edge.first].Edges.size()<2)&&(m_vSSVertex[e.Edge.second].Edges.size()<2))
			  {
			    
			    
			    vbEdge.push_back(e);
			    m_vSSVertex[e.Edge.first].Edges.push_back(vbEdge.size());
			    m_vSSVertex[e.Edge.second].Edges.push_back(vbEdge.size());
			  }
			    
			}
		     }
                     tmpV.clear();
                     ch.clear();
                     //return 0;
                  }

              }

          }
          
      conectingDots(rho);
      return 0;
  }
  float lenghtEdge(bEdge e)
  {
    Vector xpf = m_vSSVertex[e.Edge.first].pos-m_vSSVertex[e.Edge.second].pos;
    xpf.z=0;
    return xpf.norm();
  }

  void FindConvexHull(of::ofList<int> &tmpV,std::vector<int> &cv,int k,int l,float rho)
  {
      Points points, result;

      float nv,invnv,x,y,xs,ys,rho2=2.0*rho;
      float rho8inv = 1/(4.0*rho2);
      x=Del.Pmin[0]+k*rho2+rho;y=Del.Pmin[1]+l*rho2+rho;
      points.push_back(Point_2(0,0));

      for(int i=0; i<tmpV.size();i++)
      {
          Vector xpf = m_vSSVertex[tmpV.pos(i)].pos;
          xs=(xpf.x-x)*rho8inv;
          ys=(xpf.y-y)*rho8inv;
          nv =sqrt((xs)*(xs)+(ys)*(ys));
          invnv=1.0/pow(nv,1.3);
          points.push_back(Point_2(xs*invnv,ys*invnv));
           //
      }
      CGAL::convex_hull_2( points.begin(), points.end(), std::back_inserter(result) );
      for(int i=1; i<points.size();i++)
      {
         for(int j=0;j<result.size();j++)
             if(points[i]==result[j])
                 cv.push_back(i-1);
      }

  }

  
  void findingDelaunayMesh(vector< vector<int> > &polys)
  {
    CDT cdt;
    Vector x;
    std::vector<PointCDT_2> points;
     std::vector<std::ptrdiff_t> indices;
     std::ptrdiff_t index=0;
std::vector<Vertex_handle> v_handle;
         
    
          for(unsigned int i = 0; i <  m_vSSVertex.size(); ++i){
		  
                   x = m_vSSVertex[i].pos;

                   points.push_back( PointCDT_2( x.x, x.y));
		  


          }
          insert_with_info(cdt, points.begin(), points.end());

          
          int s=vbEdge.size();
	 for(int i = 0; i <s ; ++i){
	   
	   //Vertex_handle va = cdt.vertex(vbEdge[i].Edge.first);
	   //Vertex_handle vb = v_handle[vbEdge[i].Edge.second];
	   //cdt.insert_constraint(va,vb);
	   
	 }
	
	 Finite_faces_iterator fc = cdt.finite_faces_begin();
  for( ; fc != cdt.finite_faces_end(); ++fc)
  {	
   Face_handle face =fc;
    std::vector<int> tri;
    tri.resize(3);
     
     tri[0]=face->vertex(0)->info();
     
     tri[1]=face->vertex(1)->info();
     
     tri[2]=face->vertex(2)->info();
     polys.push_back(tri);
  }
	 
     
  }




  void DrawSSVertex(BboxScreenSpace &Box)
  {
      Vector x;

          glBegin(GL_POINTS);

          //glColor3dv(node_color);

          /*for(int i = 0; i < m_iNumNodeVrts; ++i){
                  Vector x = m_vSSVertex[i].pos;
                  glVertex3d(x.x, x.y, 0.02);
          }*/

          //glColor3dv(edge_color);
          glColor3d(0.0, 0.0, 0.0);
          for(int i = 0; i <  m_vSSVertex.size(); ++i){
		  if(m_vSSVertex[i].b==false)
		  {
                   x = vrts[i];

                   glVertex3d(x.x, x.y, x.z);
		  }


          }
          glEnd();

    //DrawBoundingBox2D(Box);
  }
  
  void DrawDelGrid(float rho)
  {
    int i,j;
    float xp1,xp2,yp1,yp2;
    Vector x1,x2;
    float rho2=rho*2.0;
    Vector x1r,x2r;
    //char text[50];

       /*glBegin(GL_LINES);
       
       x1r=worldScreenSpace(Del.Pmin[0],Del.Pmin[1],0.0);
       x2r=worldScreenSpace(Del.Pmax[0],Del.Pmax[1],0.0);
       x1.x=x1r.x;x1.y=x1r.y; x1.z = 0.0;
       x2.x=x1r.x;x2.y=x2r.y; x2.z = 0.0;
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1.x=x1r.x;x1.y=x2r.y; x1.z = 0.0;
       x2.x=x2r.x;x2.y=x2r.y; x2.z = 0.0;
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1.x=x2r.x;x1.y=x2r.y; x1.z = 0.0;
       x2.x=x2r.x;x2.y=x1r.y; x2.z = 0.0;
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1.x=x2r.x;x1.y=x1r.y; x1.z = 0.0;
       x2.x=x1r.x;x2.y=x1r.y; x2.z = 0.0;
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       glEnd();*/

       for(j=0;j<Del.nv;j++)
       {
           for(i=0;i<Del.nh;i++)
           {
             xp1=Del.Pmin[0]+i*rho2;yp1=Del.Pmin[1]+j*rho2;
             xp2=xp1+rho2;yp2=yp1+rho2;
             GridCell &c = Del.vc[j*Del.nh+i];
             if(c.vp)
                 glColor3f(0.2,0.2,0.2);
             else
                 glColor3f(1.0,0.1,0.0);
               glBegin(GL_LINES);

               x1r=worldScreenSpace(xp1,yp1,0.0);
               x2r=worldScreenSpace(xp2,yp2,0.0);
               x1.x=x1r.x;x1.y=x1r.y; x1.z = 0.0;
               x2.x=x1r.x;x2.y=x2r.y; x2.z = 0.0;
               glVertex3d(x1.x, x1.y, x1.z);
               glVertex3d(x2.x, x2.y, x2.z);
               x1.x=x1r.x;x1.y=x2r.y; x1.z = 0.0;
               x2.x=x2r.x;x2.y=x2r.y; x2.z = 0.0;
               glVertex3d(x1.x, x1.y, x1.z);
               glVertex3d(x2.x, x2.y, x2.z);
               x1.x=x2r.x;x1.y=x2r.y; x1.z = 0.0;
               x2.x=x2r.x;x2.y=x1r.y; x2.z = 0.0;
               glVertex3d(x1.x, x1.y, x1.z);
               glVertex3d(x2.x, x2.y, x2.z);
               x1.x=x2r.x;x1.y=x1r.y; x1.z = 0.0;
               x2.x=x1r.x;x2.y=x1r.y; x2.z = 0.0;
               glVertex3d(x1.x, x1.y, x1.z);
               glVertex3d(x2.x, x2.y, x2.z);
               glEnd();
               //break;
           }
           //break;
       }

       /*for(j=0;j<Del.nv;j++)
       {
           for(i=0;i<Del.nh;i++)
           {
             xp1=Del.Pmin[0]+i*rho2+rho;yp1=Del.Pmin[1]+j*rho2+rho;
             GridCell &c = Del.vc[j*Del.nh+i];
             if(c.empty)
             {
                 glColor3f(0.2,0.2,0.2);
                 sprintf(text, "%de", j*Del.nh+i);
             }
             else
             {
                 glColor3f(1.0,0.1,0.0);
                 sprintf(text, "%df", j*Del.nh+i);
             }

             x1r=worldScreenSpace(xp1,yp1,0.0);

             glRasterPos3f(x1r.x,x1r.y,x1r.z);
             for (unsigned int K = 0; K < strlen(text); K++)
                glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[K]);


           }

       }*/


  }
  
 void classifyingCriticalPoints(float rho)
 {
   int vi,vj,nvb,nb = vBVertex.size();
float rho2=rho*2.0;
   Vector x,xn,x1,xt; //x1 tangent vecor
   bool c;
   float p,fp,fpx,px;
 for(int i = 0; i < nb; ++i){
     nvb = neighbors[i].size();
  c=true;
  vi=vBVertex[i].id;
   x= m_vSSVertex[vi].pos;
   x.z=0;
   vj=neighbors[i][1].idx;
   x1 = m_vSSVertex[vj].pos-x;
   x1.z=0;
   x1.normalize();
   m_vSSVertex[vBVertex[i].id].Edges.clear();
   freebPoints.push_back(vBVertex[i].id);
      fp=x.y-m_vSSVertex[vj].pos.y;
      if ((fp)>0)
       p=1.0;
      else
       p=-1.0;
      fpx=x.x-m_vSSVertex[vj].pos.x;
      if ((fpx)>0)
       px=1.0;
      else
       px=-1.0;
      vi=vBVertex[i].id;
   for(int j=2; j< nvb;j++)
   {
          vj = neighbors[i][j].idx;
       if(vi == vj)
           continue;
        xn = m_vSSVertex[vj].pos;
        xn.z=0;
        if((((x.y-xn.y)*p)<0.0))
        {
         c=false;
         break;
        }
        xt=(xn-x);
        xt.normalize();
        if((xt*x1)<0.15)
        {
         break;
        }

   }
   if(c)
   {
    if(p>0.0)
     m_vSSVertex[vi].c=1;
    else
     m_vSSVertex[vi].c=2;
   }
 }
  
 }
 
 
 void RemoveFronVector(std::vector< int> &myVector, int ele)
 {
   std::vector<int>::iterator position = std::find(myVector.begin(), myVector.end(), ele);
if (position != myVector.end()) // == myVector.end() means the element was not found
    myVector.erase(position);
 }
  
  int findPosition(std::vector < bVertex> &vBVertex, int id)
  {
    int i;
    for(i=0;i < vBVertex.size();i++)
      if(vBVertex[i].id==id)
	return i;
   return -1;
  }
  
 void connenctingFlux(std::vector < int> &freebPoints)
 {
 int i,j,position,refPos,pos,posn,nb = freebPoints.size();
 int comp=0;
int dir=1; // 1= up: 2=down
Vector x1,x2;
bool valid;

 do {
   
	  //choosing initial vertex
	  for( i = 0; i < nb; ++i){
	    if (m_vSSVertex[freebPoints[i]].c == 2)
	      break;
	  }
	  refPos = freebPoints[i];
	  
	  x1=m_vSSVertex[refPos].pos;
	  position = findPosition(vBVertex,refPos);
	  if(position>-1)
	  {
	    bEdge e;
	    e.Edge.first=refPos;
	    // choosing next point
	    for(j=1;j<neighbors[position].size();j++)
	    {
	      x2 = m_vSSVertex[neighbors[position][j].idx].pos;
	      if(dir==1)
	      {
		if((x2.y>=x1.y)&&(m_vSSVertex[neighbors[position][j].idx].Edges.size()<2))
		  break;
	      }
	      else
		if((x2.y<=x1.y)&&(m_vSSVertex[neighbors[position][j].idx].Edges.size()<2))
		  break;
	      
	    }
	    
	    posn=neighbors[position][j].idx;
	    e.Edge.second=posn;
	      vbEdge.push_back(e);
	      m_vSSVertex[e.Edge.first].Edges.push_back(vbEdge.size());
	      m_vSSVertex[e.Edge.second].Edges.push_back(vbEdge.size());
	      if(m_vSSVertex[posn].c==2)
		dir=1;
	      if(m_vSSVertex[posn].c==1)
		dir=2;
	      valid =true;
	    while((posn!=refPos)&&(valid==true))
	    {
		      pos=posn;
		      x1=m_vSSVertex[pos].pos;
		    position = findPosition(vBVertex,pos);
                    if(position >-1)
			{
			bEdge e;
			e.Edge.first=pos;
			// choosing next point
			for(j=1;j<neighbors[position].size();j++)
			{
			      x2 = m_vSSVertex[neighbors[position][j].idx].pos;
			      if(dir==1)
			      {
				if((x2.y>=x1.y)&&(m_vSSVertex[neighbors[position][j].idx].Edges.size()<2))
				  break;
			      }
			      else
				if((x2.y<=x1.y)&&(m_vSSVertex[neighbors[position][j].idx].Edges.size()<2))
				  break;

			}

			posn=neighbors[position][j].idx;
			if(posn<m_vSSVertex.size())
			{
			    if(m_vSSVertex[posn].c==2)
			      dir=1;
			    if(m_vSSVertex[posn].c==1)
			      dir=2;
			    e.Edge.second=posn;
			      vbEdge.push_back(e);
			      m_vSSVertex[e.Edge.first].Edges.push_back(vbEdge.size());
			      if (m_vSSVertex[e.Edge.first].Edges.size()==2)
				RemoveFronVector(freebPoints, e.Edge.first);
			      m_vSSVertex[e.Edge.second].Edges.push_back(vbEdge.size());
			      if (m_vSSVertex[e.Edge.second].Edges.size()==2)
				RemoveFronVector(freebPoints, e.Edge.second);
			}
			else
			    valid=false;
		    }
		    else
		      valid=false;

	    }
	    
	  nb = freebPoints.size();  
	  }
 	  comp++;
	  
 } while ((nb>0)&& (comp<1));
   
 }
  
  
void conectingDots(float rho)  
{
 Vector x;
 float d1,d2;
 searcher.clear();
 int nb = vBVertex.size();
 for(int i = 0; i < nb; ++i){
  
    x= m_vSSVertex[vBVertex[i].id].pos;
    x.z=0;
    searcher.insert(vBVertex[i].id,x);
  }
 searcher.init();
  searcher.queryRadius(2.0*rho);
  neighbors.resize(nb);
  for (int i = 0; i < nb; ++i) {
      neighbors[i].reserve(10);
      x= m_vSSVertex[vBVertex[i].id].pos;
      x.z=0;
      searcher.neighbors(x, neighbors[i]);
      NeighborData tmp;
      for(int j=0;j<neighbors[i].size()-1;j++)
          for(int k=j+1;k<neighbors[i].size();k++)
          {
              d1=neighbors[i][j].d;
              d2=neighbors[i][k].d;
              if(d1 > d2)
              {
                  tmp = neighbors[i][j];
                  neighbors[i][j]= neighbors[i][k];
                  neighbors[i][k]=tmp;
              }
          }

    }
    
    classifyingCriticalPoints(rho);
    //vbEdge.clear();
    //connenctingFlux(freebPoints);
     
}
  
  
  void DrawBoundaryVertex2D(float rho)
  {
      char text[50];
    
    //DrawDelGrid(rho);
      Vector x,x1,x2;

      glColor3f(0.1, 1.0,1.0);
       glBegin(GL_POINTS);
       for(int i = 0; i < vBVertex.size(); ++i){
         if(m_vSSVertex[vBVertex[i].id].c==1)
         {
          x = vrts[vBVertex[i].id];
           glVertex3d(x.x, x.y, x.z);
         }
       }
       glEnd();

     glColor3f(1.0, 1.0,0.1);
       glBegin(GL_POINTS);
       for(int i = 0; i < vBVertex.size(); ++i){
         if(m_vSSVertex[vBVertex[i].id].c==2)
         {
          x = vrts[vBVertex[i].id];
           glVertex3d(x.x, x.y, x.z);
         }
       }
       glEnd();



       glColor3f(0.1, 0.1,0.9);
        glBegin(GL_POINTS);
        for(int i = 0; i < vBVertex.size(); ++i){
          if(m_vSSVertex[vBVertex[i].id].Edges.size()==2)
	  { 
	   x = vrts[vBVertex[i].id];
            glVertex3d(x.x, x.y, x.z);
	  }
        }
        glEnd();

        glColor3f(1.0, 0.1,0.0);
        glBegin(GL_POINTS);
        for(int i = 0; i < vBVertex.size(); ++i){
          if(m_vSSVertex[vBVertex[i].id].Edges.size()<2)
	  { 
	   x = vrts[vBVertex[i].id];
            glVertex3d(x.x, x.y, x.z);
	  }
        }
        glEnd();
	

        
	
      int s=vbEdge.size();
      glColor3f(0.1, 0.1,1.0);
	 for(int i = 0; i <s ; ++i){
	   glBegin(GL_LINES);
	   x1 = vrts[vbEdge[i].Edge.first];
	   x2 = vrts[vbEdge[i].Edge.second];
	   glVertex3d(x1.x, x1.y, x1.z);
	   glVertex3d(x2.x, x2.y, x2.z);
	   glEnd();
	 }
	
      /*for(int i = 0; i < vBVertex.size(); ++i){
          sprintf(text, "%d", vBVertex[i].id);
      x = vrts[vBVertex[i].id];
      glRasterPos3f(x.x,x.y,x.z);
      for (unsigned int j = 0; j < strlen(text); j++)
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[j]);
      }*/
  }

  void DrawBoundingBox2D(BboxScreenSpace &Box)
  {
      Vector x,x1,x2;
      char text[50];
      /*glColor3f(1.0, 0.0,0.0);
      sprintf(text, "%d", Box.maxxId);
      x = vrts[Box.maxxId];
      glRasterPos3f(x.x,x.y,x.z);
      for (unsigned int j = 0; j < strlen(text); j++)
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[j]);
      glColor3f(0.0, 0.0,1.0);
      sprintf(text, "%d", Box.maxyId);
      x = vrts[Box.maxyId];
      glRasterPos3f(x.x,x.y,x.z);
      for (unsigned int j = 0; j < strlen(text); j++)
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[j]);
      glColor3f(0.0, 1.0,0.0);
      sprintf(text, "%d", Box.minxId);
      x = vrts[Box.minxId];
      glRasterPos3f(x.x,x.y,x.z);
      for (unsigned int j = 0; j < strlen(text); j++)
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[j]);
      glColor3f(1.0, 1.0,0.0);
      sprintf(text, "%d", Box.minyId);
      x = vrts[Box.minyId];
      glRasterPos3f(x.x,x.y,x.z);
      for (unsigned int j = 0; j < strlen(text); j++)
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_12, text[j]);
*/
      glColor3f(1.0,0.0,0.0);

       glBegin(GL_LINES);
       x1=worldScreenSpace(Box.minx,Box.maxy,0.0);//0.5*(vrts[Box.minxId].z+vrts[Box.maxyId].z);
       x2=worldScreenSpace(Box.maxx,Box.maxy, 0.0);//0.5*(vrts[Box.maxxId].z+vrts[Box.maxyId].z);
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1 = worldScreenSpace(Box.minx,Box.maxy,0.0);// 0.5*(vrts[Box.minxId].z+vrts[Box.maxyId].z);
       x2 = worldScreenSpace(Box.minx,Box.miny,0.0);// 0.5*(vrts[Box.minxId].z+vrts[Box.minyId].z);
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1=worldScreenSpace(Box.minx,Box.miny,0.0);// 0.5*(vrts[Box.minxId].z+vrts[Box.minyId].z);
       x2 = worldScreenSpace(Box.maxx,Box.miny,0.0);// 0.5*(vrts[Box.maxxId].z+vrts[Box.minyId].z);
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       x1 = worldScreenSpace(Box.maxx,Box.miny,0.0);// 0.5*(vrts[Box.maxxId].z+vrts[Box.minyId].z);
       x2 = worldScreenSpace(Box.maxx,Box.maxy,0.0);// = 0.5*(vrts[Box.maxxId].z+vrts[Box.maxyId].z);
       glVertex3d(x1.x, x1.y, x1.z);
       glVertex3d(x2.x, x2.y, x2.z);
       glEnd();

  }



  void ApplyDepthFilter(vector<float> &dmap, int nx, int ny, int n_filter)
  {
          int b = 2*n_filter+1;


          if( m_vFilter.size() != b){
                  m_vFilter = CalBinomials(b);
          }

          std::vector<float> tmp_map;
          tmp_map = dmap;

          std::vector<float> d;
          d.resize(b);


          for(int j = 0; j < ny; ++j){
                  for(int i = n_filter; i < nx-n_filter; ++i){
                          d[0] = tmp_map[i+j*nx];
                          if(d[0] < INF){
                                  int n = 0;
                                  for(int k = 0; k < n_filter; ++k){
                                          d[2*n+1] = tmp_map[(i-(k+1))+j*nx];
                                          d[2*n+2] = tmp_map[(i+(k+1))+j*nx];
                                          if(fabs(d[2*n+1]-d[0]) > m_fSSZmax || fabs(d[2*n+2]-d[0]) > m_fSSZmax){
                                                  break;
                                          }
                                          n++;
                                  }

                                  int bn = 2*n;

                                  float new_depth = m_vFilter[bn][n]*d[0];
                                  for(int k = 1; k <= n; ++k){
                                          new_depth += m_vFilter[bn][n-k]*d[2*k-1];
                                          new_depth += m_vFilter[bn][n+k]*d[2*k];
                                  }

                                  dmap[i+j*nx] = new_depth;
                          }
                  }
          }

          tmp_map = dmap;


          for(int i = 0; i < nx; ++i){
                  for(int j = n_filter; j < ny-n_filter; ++j){
                          d[0] = tmp_map[i+j*nx];
                          if(d[0] < INF){

                                  int n = 0;
                                  for(int k = 0; k < n_filter; ++k){
                                          d[2*n+1] = tmp_map[i+(j-(k+1))*nx];
                                          d[2*n+2] = tmp_map[i+(j+(k+1))*nx];
                                          if(fabs(d[2*n+1]-d[0]) > m_fSSZmax || fabs(d[2*n+2]-d[0]) > m_fSSZmax){
                                                  break;
                                          }
                                          n++;
                                  }

                                  int bn = 2*n;

                                  float new_depth = m_vFilter[bn][n]*d[0];
                                  for(int k = 1; k <= n; ++k){
                                          new_depth += m_vFilter[bn][n-k]*d[2*k-1];
                                          new_depth += m_vFilter[bn][n+k]*d[2*k];
                                  }

                                  dmap[i+j*nx] = new_depth;
                          }
                  }
          }

  }
  


  int CalNodeVertex(int nx, int ny, double dw, double dh, std::vector<rxSSGrid> *grid, const vector<float> &dgrid)
  {
   if(grid->size() != nx*ny){
                    grid->clear();
                    grid->resize(nx*ny);
            }

            for(int i = 0; i < nx; ++i){
                    for(int j = 0; j < ny; ++j){
                            int idx = i+j*nx;
                            for(int k = 0; k < 4; ++k){
                                    grid->at(idx).node_vrts[k] = -1;
                                    grid->at(idx).edge_vrts[k] = -1;
                                    grid->at(idx).back_vrts[k] = -1;
                            }
                            grid->at(idx).back_vrts[4] = -1;
                            grid->at(idx).back_vrts[5] = -1;
                            grid->at(idx).num_nv = 0;
                            grid->at(idx).num_ev = 0;
                            grid->at(idx).num_bv = 0;

                            grid->at(idx).i = i;
                            grid->at(idx).j = j;
                    }
            }
     

          int nv_num = 0;
          for(int j = 0; j <= ny; ++j){
                  for(int i = 0; i <= nx; ++i){
                          double d = dgrid[i+j*(nx+1)];
                          if(d < INF){
                                  Vector vrt_pos = Vector(dw*i, dh*j, d);

                                  m_vSSVertex.push_back(rxSSVertex(vrt_pos, 0));
                                  nv_num++;
                                  int vidx =  m_vSSVertex.size()-1;


                                  if(i != 0){
                                          if(j != 0){
                                                  m_vSSMGrid->at((i-1)+(j-1)*nx).node_vrts[2] = vidx;
                                                  m_vSSMGrid->at((i-1)+(j-1)*nx).num_nv++;
                                          }
                                          if(j != ny){
                                                  m_vSSMGrid->at((i-1)+(j)*nx).node_vrts[1] = vidx;
                                                  m_vSSMGrid->at((i-1)+(j)*nx).num_nv++;
                                          }
                                  }
                                  if(i != nx){
                                          if(j != 0){
                                                  m_vSSMGrid->at((i)+(j-1)*nx).node_vrts[3] = vidx;
                                                  m_vSSMGrid->at((i)+(j-1)*nx).num_nv++;
                                          }
                                          if(j != ny){
                                                  m_vSSMGrid->at((i)+(j)*nx).node_vrts[0] = vidx;
                                                  m_vSSMGrid->at((i)+(j)*nx).num_nv++;
                                          }
                                  }
                          }


                          if(i != 0){
                                  if(j != 0){
                                          m_vSSMGrid->at((i-1)+(j-1)*nx).node_depth[2] = d;
                                  }
                                  if(j != ny){
                                          m_vSSMGrid->at((i-1)+(j)*nx).node_depth[1] = d;
                                  }
                          }
                          if(i != nx){
                                  if(j != 0){
                                          m_vSSMGrid->at((i)+(j-1)*nx).node_depth[3] = d;
                                  }
                                  if(j != ny){
                                          m_vSSMGrid->at((i)+(j)*nx).node_depth[0] = d;
                                  }
                          }

                  }
          }

          return nv_num;
  }

  int DetectSilhouetteEdgeVertex(int nx, int ny, double dw, double dh, const vector<float> &dgrid,
                                                                                   const vector<rxSSParticle> &ssprts, int W, int H)
  {
          // DetectSilhouetteEdgeVertex
          int en = 0;
          int pnum =  ssprts.size();

          m_vSSEdge->resize((nx)*(ny+1)+(nx+1)*(ny));
          int yoffset = (nx)*(ny+1);


          for(int j = 0; j <= ny; ++j){
                  for(int i = 0; i < nx; ++i){
                          int i00 = i+j*(nx+1);
                          int i01 = (i+1)+j*(nx+1);

                          rxSSEdge *e = &m_vSSEdge->at(i+j*(nx));

                          e->x0 = Vector(dw*i, dh*j, dgrid[i00]);//e->d0 = dgrid[i00];
                          e->x1 = Vector(dw*(i+1), dh*j, dgrid[i01]);//e->d1 = dgrid[i01];
                          
                          e->depth = 0.5*(e->x0.z+e->x1.z);
                          
                          
                          e->xy = 0;
                          e->silhouette = 0;
                          e->front_vertex = -1;
                          e->dx = 0.0;

                          if(fabs(e->x0.z-e->x1.z) > m_fSSZmax){
                                  e->silhouette = 1;
                                  en++;
                          }
                  }
          }


          for(int j = 0; j < ny; ++j){
                  for(int i = 0; i <= nx; ++i){
                          int i00 = i+j*(nx+1);
                          int i10 = i+(j+1)*(nx+1);

                          rxSSEdge *e = &m_vSSEdge->at(yoffset+i+j*(nx+1));

                          e->x0 = Vector(dw*i, dh*j, dgrid[i00]);//e->d0 = dgrid[i00];
                          e->x1 = Vector(dw*i, dh*(j+1), dgrid[i10]);//e->d1 = dgrid[i01];
                          
                          e->depth = 0.5*(e->x0.z+e->x1.z);
                          
                          
                          
                          e->xy = 1;
                          e->silhouette = 0;
                          e->front_vertex = -1;
                          e->dx = 0.0;

                          if(fabs(e->x0.z-e->x1.z) > m_fSSZmax){
                                  e->silhouette = 1;
                                  en++;
                          }
                  }
          }

          m_vSSEdgeVertex.clear();
          for(int k = 0; k < pnum; ++k){
                  Vector xp = ssprts[k].xp;
                  Vector rp = ssprts[k].rp;

                  if(xp.x < 0 || xp.x >= W || xp.y < 0 || xp.y >= H){
                          continue;
                  }


                  int cen[2];
                  cen[0] = (xp.x/dw)+1;
                  cen[1] = (xp.y/dh)+1;

                  int minp[2], maxp[2];
                  minp[0] = cen[0]-(rp.x/dw+1);
                  minp[1] = cen[1]-(rp.y/dh+1);
                  maxp[0] = cen[0]+(rp.x/dw+1);
                  maxp[1] = cen[1]+(rp.y/dh+1);


                  RX_CLAMP(minp[0], 0, nx-1);
                  RX_CLAMP(minp[1], 0, ny-1);
                  RX_CLAMP(maxp[0], 0, nx-1);
                  RX_CLAMP(maxp[1], 0, ny-1);


                  for(int j = minp[1]; j <= maxp[1]+1; ++j){
                          for(int i = minp[0]; i <= maxp[0]; ++i){
                                  rxSSEdge &e = m_vSSEdge->at(i+j*(nx));
                                  if(!e.silhouette) continue;
                                  if(e.depth < xp.z) continue;


                                  Vector A, B, C, P[2];
                                  double r, t[2];
                                  if(e.x0.z <= e.x1.z){
                                          A = Vector(e.x0.x, e.x0.y,0.0);
                                          B = Vector(e.x1.x, e.x1.y,0.0);
                                  }
                                  else{
                                          A = Vector(e.x1.x, e.x1.y,0.0);
                                          B = Vector(e.x0.x, e.x0.y,0.0);
                                  }
                                  C = Vector(xp.x, xp.y,0.0);
                                  r = rp.x;
                                  int inter = LineCircleIntersection(A, B, C, r, P, t);
                                  if(inter == 1){
                                          if(e.front_vertex == -1 || t[0] > e.dx){
                                                  Vector vrt_pos;
                                                  vrt_pos.x = P[0].x;
                                                  vrt_pos.y = P[0].y;
                                                  vrt_pos.z = xp.z;
                                                  m_vSSEdgeVertex.push_back(rxSSVertex(vrt_pos, 1));
                                                  int eidx =  m_vSSEdgeVertex.size()-1;

                                                  e.front_vertex = eidx;
                                                  e.dx = t[0];
                                          }
                                  }

                          }
                  }

                  for(int i = minp[0]; i <= maxp[0]+1; ++i){
                          for(int j = minp[1]; j <= maxp[1]; ++j){
                                  rxSSEdge &e = m_vSSEdge->at(yoffset+i+j*(nx+1));
                                  if(!e.silhouette) continue;
                                  if(e.depth < xp.z) continue;


                                  Vector A, B, C, P[2];
                                  double r, t[2];
                                  if(e.x0.z <= e.x1.z){
                                      A = Vector(e.x0.x, e.x0.y,0.0);
                                      B = Vector(e.x1.x, e.x1.y,0.0);
                                  }
                                  else{
                                      A = Vector(e.x1.x, e.x1.y,0.0);
                                      B = Vector(e.x0.x, e.x0.y,0.0);
                                  }
                                  C = Vector(xp.x, xp.y,0.0);
                                  r = rp.x;
                                  int inter = LineCircleIntersection(A, B, C, r, P, t);
                                  if(inter == 1){
                                          if(e.front_vertex == -1 || t[0] > e.dx){
                                                  Vector vrt_pos;
                                                  vrt_pos.x = P[0].x;
                                                  vrt_pos.y = P[0].y;
                                                  vrt_pos.z = xp.z;
                                                  m_vSSEdgeVertex.push_back(rxSSVertex(vrt_pos, 1));
                                                  int eidx =  m_vSSEdgeVertex.size()-1;

                                                  e.front_vertex = eidx;
                                                  e.dx = t[0];
                                          }
                                  }

                          }
                  }
          }

          return en;
  }

  void ApplySilhoutteSmoothing(vector<rxSSVertex> &ssvrts, const vector< vector<int> > &polys, int n_iter)
  {
          // ApplySilhoutteSmoothing
          int vn =  ssvrts.size();
          int pn =  polys.size();

          for(int l = 0; l < n_iter; ++l){
                  for(int i = 0; i < vn; ++i){
                          ssvrts[i].avg_pos = Vector(0.0,0.0,0.0);
                          ssvrts[i].avg_num = 0.0;
                  }

                  int neigh[3][2] = { {1, 2}, {2, 0}, {0, 1} };

                  for(int i = 0; i < pn; ++i){
                          for(int j = 0; j < 3; ++j){
                                  int idx = polys[i][j];
                                  if(ssvrts[idx].edge){
                                          if(ssvrts[idx].avg_num == 0){
                                                  ssvrts[idx].avg_pos += ssvrts[idx].pos;
                                                  ssvrts[idx].avg_num += 1.0;
                                          }


                                          for(int k = 0; k < 2; ++k){
                                                  int nidx = polys[i][neigh[j][k]];
                                                  if(ssvrts[nidx].edge){
                                                          ssvrts[idx].avg_pos += ssvrts[nidx].pos;
                                                          ssvrts[idx].avg_num += 1.0;
                                                  }
                                                  else{
                                                          ssvrts[idx].avg_pos += 0.5*ssvrts[nidx].pos;
                                                          ssvrts[idx].avg_num += 0.5;
                                                  }
                                          }
                                  }
                          }
                  }

                  for(int i = 0; i < vn; ++i){
                          if(ssvrts[i].edge && ssvrts[i].avg_num){
                                  ssvrts[i].avg_pos /= ssvrts[i].avg_num;
                                  ssvrts[i].pos[0] = ssvrts[i].avg_pos[0];
                                  ssvrts[i].pos[1] = ssvrts[i].avg_pos[1];
                                  ssvrts[i].pos[2] = ssvrts[i].avg_pos[2];
                          }
                  }
          }
  }

  int LineCircleIntersection(const Vector &A, const Vector &B, const Vector &C, const double &r, Vector P[2], double t[2])
  {
          double rr = r*r;
          Vector AC = C-A;
          Vector BC = C-B;

          Vector v = B-A;
          double l=v.norm();
          v.normalize();
          double td = v*AC;
          Vector D = A+td*v;
          double dd = (D-C).sqrnorm();

          if(dd < rr){
                  double dt = sqrt(rr-dd);

                  double da = rr-(AC).sqrnorm();
                  double db = rr-(BC).sqrnorm();

                  int inter = 0;
                  double t1 = td-dt;
                  double t2 = td+dt;
                  if(t1 >= 0 && t1 <= l){
                          P[inter] = A+t1*v;
                          t[inter] = t1;
                          inter++;
                  }
                  if(t2 >= 0 && t2 <= l){
                          P[inter] = A+t2*v;
                          t[inter] = t2;
                          inter++;
                  }

                  return inter;
          }
          else{
                  return 0;
          }
  }


  int CalEdgeVertex(int nx, int ny, double dw, double dh, const vector<float> &dgrid, vector<rxSSEdge> *edges)
  {
          vBVertex.clear();
          vbEdge.clear();
          int ev_num = 0;
          int yoffset = (nx)*(ny+1);
          for(int i = 0; i <  edges->size(); ++i){
                  rxSSEdge e = edges->at(i);
                  if(!e.silhouette) continue;

                  int ei0, ei1, ej0, ej1;
                  if(i < yoffset){
                          ei0 = i%nx;
                          ej0 = i/nx;
                          ei1 = ei0+1;
                          ej1 = ej0;
                  }
                  else{
                          ei0 = (i-yoffset)%(nx+1);
                          ej0 = (i-yoffset)/(nx+1);
                          ei1 = ei0;
                          ej1 = ej0+1;
                  }

                  Vector vrt_pos(0.0,0.0,0.0);
                  double dx = 0.0;

                  if(e.front_vertex != -1){
                          vrt_pos = m_vSSEdgeVertex[e.front_vertex].pos;
                          dx = e.dx;
                  }
                  else{
                          if(e.x0.z <= e.x1.z){
                                  dx = binarySearchDepth(e.x0, e.x1, vrt_pos, m_fSSZmax);
                          }
                          else{
                                  dx = binarySearchDepth(e.x1, e.x0, vrt_pos, m_fSSZmax);
                          }

                          if(vrt_pos.x == 0 && vrt_pos.y == 0){
                                  cout << "edge vertex error : " << i << endl;
                          }
                  }

                  m_vSSVertex.push_back(rxSSVertex(vrt_pos, 1));
                  ev_num++;
                  int vidx =  m_vSSVertex.size()-1;
                      bVertex v;
                       v.id=vidx;
                       m_vSSVertex[v.id].b=true;
                       vBVertex.push_back(v);


                  int bidx = -1;
                  Vector back_vrt;
                  if(e.x0.z < RX_FEQ_INFM && e.x1.z < RX_FEQ_INFM){
                          float back_node_depth, nn_back_node_depth, back_depth;
                          int back_node, nn_back_node;
                          double l = 1.0;


                          if(e.x0.z > e.x1.z){
                                  back_node = ei0+ej0*(nx+1);
                                  if(i < yoffset){
                                          nn_back_node = (ei0 == 0 ? ei0 : ei0-1)+ej0*(nx+1);
                                          l = dw;
                                  }
                                  else{
                                          nn_back_node = ei0+(ej0 == 0 ? ej0 : ej0-1)*(nx+1);
                                          l = dh;
                                  }
                          }
                          else{
                                  back_node = ei1+ej1*(nx+1);
                                  if(i < yoffset){
                                          nn_back_node = (ei1 == nx ? ei1 : ei1+1)+ej1*(nx+1);
                                          l = dw;
                                  }
                                  else{
                                          nn_back_node = ei1+(ej1 == ny ? ej1 : ej1+1)*(nx+1);
                                          l = dh;
                                  }
                          }


                          back_node_depth = dgrid[back_node];
                          nn_back_node_depth = dgrid[nn_back_node];


                          back_depth = back_node_depth;
                          //back_depth = back_node_depth*((2*l-dx)/l)-nn_back_node_depth*((l-dx)/l);


                          back_vrt.x = vrt_pos.x;
                          back_vrt.y = vrt_pos.y;
                          back_vrt.z = back_depth;

                          m_vSSVertex.push_back(rxSSVertex(back_vrt, 1));
                          ev_num++;
                          bidx =  m_vSSVertex.size()-1;
                          bVertex v;
                          v.id=bidx;
                          m_vSSVertex[v.id].b=true;
                          vBVertex.push_back(v);
                  }

                  //if(bidx != -1 && back_vrt[2] < vrt_pos[2]){
                  //	int tmp = vidx;
                  //	vidx = bidx;
                  //	bidx = tmp;
                  //}

                  if(e.xy == 0){
                          if(ej0 != 0){
                                  m_vSSMGrid->at(ei0+(ej0-1)*nx).edge_vrts[2] = vidx;
                                  m_vSSMGrid->at(ei0+(ej0-1)*nx).num_ev++;
                                  if(bidx != -1){
                                          m_vSSMGrid->at(ei0+(ej0-1)*nx).back_vrts[2] = bidx;
                                          m_vSSMGrid->at(ei0+(ej0-1)*nx).num_bv++;
                                  }
                          }
                          if(ej0 != ny){
                                  m_vSSMGrid->at(ei0+ej0*nx).edge_vrts[0] = vidx;
                                  m_vSSMGrid->at(ei0+ej0*nx).num_ev++;
                                  if(bidx != -1){
                                          m_vSSMGrid->at(ei0+ej0*nx).back_vrts[0] = bidx;
                                          m_vSSMGrid->at(ei0+ej0*nx).num_bv++;
                                  }
                          }
                  }
                  else{
                          if(ei0 != 0){
                                  m_vSSMGrid->at((ei0-1)+ej0*nx).edge_vrts[1] = vidx;
                                  m_vSSMGrid->at((ei0-1)+ej0*nx).num_ev++;
                                  if(bidx != -1){
                                          m_vSSMGrid->at((ei0-1)+ej0*nx).back_vrts[1] = bidx;
                                          m_vSSMGrid->at((ei0-1)+ej0*nx).num_bv++;
                                  }
                          }
                          if(ei0 != nx){
                                  m_vSSMGrid->at(ei0+ej0*nx).edge_vrts[3] = vidx;
                                  m_vSSMGrid->at(ei0+ej0*nx).num_ev++;
                                  if(bidx != -1){
                                          m_vSSMGrid->at(ei0+ej0*nx).back_vrts[3] = bidx;
                                          m_vSSMGrid->at(ei0+ej0*nx).num_bv++;
                                  }
                          }
                  }
                  if(bidx!=-1)
                  {
                      bEdge ed;
                      ed.Edge.first=vidx;
                      ed.Edge.second=bidx;
                       vbEdge.push_back(ed);
                       m_vSSVertex[ed.Edge.first].Edges.push_back(vbEdge.size());
                       m_vSSVertex[ed.Edge.second].Edges.push_back(vbEdge.size());
                  }

          }

          return ev_num;
  }


  int CalMesh(int nx, int ny, std::vector<rxSSGrid> *grid, std::vector< std::vector<int> > &polys, int vstart)
    {
      
      
          int v[14];

          // 3 - 2
          // |   |
          // 0 - 1


          // - 6 -
          // 7   5
          // - 4 -


          //  - 10 -
          // 11     9
          //  -  8 -
          // back-2 vertex : 12,13

          int num_mesh = 0;
          for(int j = 0; j < ny; ++j){
                  for(int i = 0; i < nx; ++i){
                          rxSSGrid *g = &grid->at(i+j*nx);

                          int table_index = 0;
                          for(int k = 0; k < 4; ++k){
                                  v[k]   = g->node_vrts[k];
                                  v[k+4] = g->edge_vrts[k];
                                  v[k+8] = g->back_vrts[k];

                                  table_index |= ((v[k] != -1) ? BITS[k] : 0);		// �m�[�h���_����4�r�b�g
                                  table_index |= ((v[k+4] != -1) ? BITS[k]*16 : 0);	// �G�b�W���_����4�r�b�g
                          }
                          v[12] = -1;
                          v[13] = -1;

                          int rotation = 0;

                          g->table_index0 = table_index;
                          int update_idx = g->num_ev*5+g->num_nv;
                          if(m_FuncTableIndex[update_idx] != 0){
                                  (this->*m_FuncTableIndex[update_idx])(table_index, rotation, v, g);
                          }

                          g->table_index1 = table_index;

                          if(g_MeshTable[table_index][0] > 0){
                                  vector<int> tri;
                                  tri.resize(3);

                                  int m = g_MeshTable[table_index][0];
                                  for(int k = 0; k < m; ++k){
                                          for(int l = 0; l < 3; ++l){
                                                  int idx = g_VrtRotTable[rotation][g_MeshTable[table_index][k*3+l+1]];

                                                  if(v[idx] == -1){
                                                          v[idx] = 0;
                                                          cout << "mesh error : " << i << ", " << j << endl;
                                                  }

                                                  tri[l] = v[idx]+vstart;
                                          }
                                          polys.push_back(tri);

                                          m_vMeshGrid.push_back(i+j*nx);
                                          //g->mesh[k] = num_mesh;

                                          num_mesh++;
                                  }


                                  //g->mesh_num = m;
                          }
                  }
          }

          return num_mesh;
  }

  

  double binarySearchDepth(Vector v1, Vector v2, Vector &vr, double zmax)
  {
          //vr = 0.5*(v1+v2);

          const int JMAX = 40;
          double dx, f, fmid, xmid, rtb;
          double x1 = 0;
          double x2 = (v1-v2).norm();
          Vector dv = v2-v1;
          dv.normalize();

          f = getDepthValue(v1);
          fmid = getDepthValue(v2);
          if(fabs(f-fmid) < zmax){
                  vr = v1+2*RX_FEQ_EPS*dv;
                  vr[2] = f+2*RX_FEQ_EPS;
                  return 0.0;
          }


          rtb = f < fmid ? (dx = x2-x1, x1) : (dx = x1-x2, x2);
          vr = (f < fmid) ? v1 : v2;
          for(int j = 0; j < JMAX; ++j){
                  dx *= 0.5;
                  xmid = rtb+dx;

                  vr = v1+dv*xmid;
                  fmid = getDepthValue(vr);

                  if(fabs(f-fmid) < zmax){
                          rtb = xmid;
                  }

                  if(fabs(dx) < 0.5){
                          vr[2] = getDepthValue(vr-dx*dv);
                          return xmid;
                  }
          }

          return 0.0;
  }

  inline float GetDepthInterpT(float x, float y, int nx, int ny, float dx, float dy, const std::vector<float> &dmap)
  {
          int ix0 =  (x/dx);
          int ix1 = ix0+1;
          double ddx = x/dx-ix0;

          RX_CLAMP(ix0, 0, nx-1);
          RX_CLAMP(ix1, 0, nx-1);

          int iy0 =  (y/dx);
          int iy1 = iy0+1;
          double ddy = y/dy-iy0;

          RX_CLAMP(iy0, 0, ny-1);
          RX_CLAMP(iy1, 0, ny-1);

          float d00 = dmap[iy0*nx+ix0];
          float d01 = dmap[iy0*nx+ix1];
          float d10 = dmap[iy1*nx+ix0];
          float d11 = dmap[iy1*nx+ix1];

          return (d00*(1.0-ddx)+d01*ddx)*(1.0-ddy)+(d10*(1.0-ddx)+d11*ddx)*ddy;
  }

  float getDepthValue(Vector x)
  {
          return GetDepthInterpT(x.x, x.y, Nx+1, Ny+1, dx, dy, vSSDMap);

  }



};

void SpaceMesh::updateTableIndexE0N4(int &table_index, int &vrot, int v[], rxSSGrid *g)
{

        table_index -= ((g->i+g->j) & 0x01) ? 1 : 0;
}


void SpaceMesh::updateTableIndexE1(int &table_index, int &vrot, int v[], rxSSGrid *g)
{

        int kb = 0;
        for(int k = 0; k < 4; ++k){
                if(v[k+4] != -1){
                        int k1 = (k == 3 ? 0 : k+1);
                        kb = ((g->node_depth[k] > g->node_depth[k1]) ? BITS[k] : BITS[k1]);
                        break;
                }
        }

        kb = (~kb & 0x0F);



        table_index = (table_index & 0xF0)+kb;
}


void SpaceMesh::updateTableIndexE2N4(int &table_index, int &vrot, int v[], rxSSGrid *g)
{

        int btable = 0;
        int k0 = 0, k1;
        for(int k = 0; k <= 4; ++k){
                k1 = (k0 == 3 ? 0 : k0+1);
                if(v[k0+4] != -1){
                        btable |= (g->node_depth[k0] > g->node_depth[k1]) ? BITS[k0] : BITS[k1];
                }
                else{
                        btable |= ((btable & BITS[k0]) ? BITS[k1] : 0);
                }
                k0++;
                if(k0 == 4) k0 = 0;
        }


        btable = (btable+2 & 0x0F);

        table_index = (table_index & 0xF0)+btable;
}


void SpaceMesh::updateTableIndexE3N23(int &table_index, int &vrot, int v[], rxSSGrid *g)
{
        int btable = 0;
        int pattern = (table_index >> 4);
        int node = g_EdgeTable[pattern][0]-4;
        int R[4];
        for(int k = 0; k < 4; ++k){
                R[k] = node++;
                if(node == 4) node = 0;
        }

        int ntable = (table_index & 0x0F);


        btable |= (((ntable >> R[1]) & 1) ? 4 : 0);


        btable |= (((ntable >> R[2]) & 1) ? 2 : 0);

// DrawSSMesh
        int btable0 = 0;
        btable0 |= (((ntable >> R[0]) & 1) ? 8 : 0);
        btable0 |= (((ntable >> R[1]) & 1) ? 4 : 0);
        btable0 |= (((ntable >> R[2]) & 1) ? 2 : 0);
        btable0 |= (((ntable >> R[3]) & 1) ? 1 : 0);


        int n0 = (btable0 & 3)-1;
        int n1 = n0+1;

        if(n0 != 1){

                int add_edge = g_EdgeTable[pattern][1]-4;
                Vector add_vrt = m_vSSVertex[g->edge_vrts[add_edge]].pos;


                int ref_edge = g_EdgeTable[pattern][0]-4;
                int ref_node = (btable & 4) ? R[1] : R[2];
                if(fabs(m_vSSVertex[g->edge_vrts[ref_edge]].pos[2]-g->node_depth[ref_node]) > m_fSSZmax){

                        add_vrt.z = m_vSSVertex[g->edge_vrts[ref_edge]].pos.z;
                }
                else{

                        if(g->back_vrts[ref_edge] == -1){
                                ref_edge = g_EdgeTable[pattern][2]-4;
                        }
                        add_vrt.z = m_vSSVertex[g->back_vrts[ref_edge]].pos.z;
                }



                m_vSSVertex.push_back(rxSSVertex(add_vrt, 1));
                m_iNumEdgeVrts++;
                int vidx =  m_vSSVertex.size()-1;


                if(m_vSSVertex[vidx].pos.z < m_vSSVertex[v[add_edge+4]].pos.z){
                        v[add_edge+8] = v[add_edge+4];
                        g->back_vrts[add_edge] = v[add_edge+4];
                        g->num_bv++;

                        v[add_edge+4] = vidx;
                        g->edge_vrts[add_edge] = vidx;
                }
                else{
                        v[add_edge+8] = vidx;
                        g->back_vrts[add_edge] = vidx;
                        g->num_bv++;
                }

        }


        btable |= ((g->node_depth[R[n0]] >= g->node_depth[R[n1]]) ? 0 : 1);


        table_index &= 0xF0;
        table_index |= btable;
}



void SpaceMesh::updateTableIndexE3N4(int &table_index, int &vrot, int v[], rxSSGrid *g)
{
        int btable = 8;
        int pattern = (table_index >> 4);


        int node = g_EdgeTable[pattern][0]-4;
        int R[4];
        for(int k = 0; k < 4; ++k){
                R[k] = node++;
                if(node == 4) node = 0;
        }


        for(int k = 0; k < 3; ++k){

                if(g->node_depth[R[k]] > g->node_depth[R[k+1]]){
                        btable |= BITS[2-k];
                }
        }


        table_index &= 0xF0;
        table_index |= btable;



        int add_edge = g_EdgeTable[pattern][1]-4;
        Vector add_vrt = m_vSSVertex[g->edge_vrts[add_edge]].pos;


        float ref_depths[4];


        ref_depths[0] = g->node_depth[R[3]];
        ref_depths[1] = g->node_depth[R[0]];

        int e2 = g_EdgeTable[pattern][2]-4;
        int e3 = g_EdgeTable[pattern][0]-4;

        if(fabs(m_vSSVertex[g->edge_vrts[e2]].pos[2]-ref_depths[0]) < m_fSSZmax){
                ref_depths[2] = m_vSSVertex[g->edge_vrts[e2]].pos.z;
        }
        else{
                ref_depths[2] = m_vSSVertex[g->back_vrts[e2]].pos.z;
        }
        if(fabs(m_vSSVertex[g->edge_vrts[e3]].pos.z-ref_depths[1]) < m_fSSZmax){
                ref_depths[3] = m_vSSVertex[g->edge_vrts[e3]].pos.z;
        }
        else{
                ref_depths[3] = m_vSSVertex[g->back_vrts[e3]].pos.z;
        }


        add_vrt[2] = 0.5*(ref_depths[2]+ref_depths[3]);


        m_vSSVertex.push_back(rxSSVertex(add_vrt, 1));
        m_iNumEdgeVrts++;
        int vidx =  m_vSSVertex.size()-1;


        //v[12] = vidx;
        //g->back_vrts[4] = vidx;


        if(m_vSSVertex[vidx].pos.z < m_vSSVertex[v[add_edge+8]].pos.z){
                if(m_vSSVertex[vidx].pos.z < m_vSSVertex[v[add_edge+4]].pos.z){
                        // front vertex
                        v[12] = v[add_edge+8];
                        g->back_vrts[4] = v[add_edge+8];

                        v[add_edge+8] = v[add_edge+4];
                        g->back_vrts[add_edge] = v[add_edge+4];

                        v[add_edge+4] = vidx;
                        g->edge_vrts[add_edge] = vidx;
                }
                else{
                        // back vertex
                        v[12] = v[add_edge+8];
                        g->back_vrts[4] = v[add_edge+8];

                        v[add_edge+8] = vidx;
                        g->back_vrts[add_edge] = vidx;
                }
        }
        else{
                // back-2 vertex
                v[12] = vidx;
                g->back_vrts[4] = vidx;
        }
}



void SpaceMesh::updateTableIndexE4N2(int &table_index, int &vrot, int v[], rxSSGrid *g)
{
        int ntable = (table_index & 0x0F);
        ntable = (ntable == 5 ? 0 : 15);


        table_index &= 0xF0;
        table_index |= ntable;
}

void SpaceMesh::updateTableIndexE4N3(int &table_index, int &vrot, int v[], rxSSGrid *g)
{
        int btable = 0;
        int pattern = (table_index >> 4);
        int ntable = (table_index & 0x0F);


        int zero_node = log((double)(~ntable & 0x0F))/log(2.0);


        for(int k = 0; k < 3; ++k){
                int k0 = g_NodeTable[zero_node][k];
                int k1 = g_NodeTable[zero_node][k+1];


                if(g->node_depth[k0] > g->node_depth[k1]){
                        btable |= BITS[2-k];
                }
        }


        vrot = zero_node;


        table_index &= 0xF0;
        table_index |= btable;
}

void SpaceMesh::updateTableIndexE4N4(int &table_index, int &vrot, int v[], rxSSGrid *g)
{
        int btable = 8;
        int pattern = (table_index >> 4);
        int ntable = (table_index & 0x0F);


        int zero_node = 0;
        double max_depth = 0.0;
        for(int k = 1; k < 4; ++k){
                if(g->node_depth[k] > max_depth){
                        max_depth = g->node_depth[k];
                        zero_node = k;
                }
        }


        for(int k = 0; k < 3; ++k){
                int k0 = g_NodeTable[zero_node][k];
                int k1 = g_NodeTable[zero_node][k+1];


                if(g->node_depth[k0] > g->node_depth[k1]){
                        btable |= BITS[2-k];
                }
        }


        vrot = zero_node;


        table_index &= 0xF0;
        table_index |= btable;


        int add_edge1 = g_NodeTable[zero_node][5]-4;
        int add_edge2 = g_NodeTable[zero_node][6]-4;
        Vector add_vrt1 = m_vSSVertex[g->edge_vrts[add_edge1]].pos;
        Vector add_vrt2 = m_vSSVertex[g->edge_vrts[add_edge2]].pos;


        int ref_edge1 = g_NodeTable[zero_node][4]-4;
        int ref_edge2 = g_NodeTable[zero_node][7]-4;
        add_vrt1.z = m_vSSVertex[g->back_vrts[ref_edge1]].pos.z;
        add_vrt2.z = m_vSSVertex[g->back_vrts[ref_edge2]].pos.z;


        m_vSSVertex.push_back(rxSSVertex(add_vrt1, 1));
        m_vSSVertex.push_back(rxSSVertex(add_vrt2, 1));
        m_iNumEdgeVrts += 2;
        int vidx1 =  m_vSSVertex.size()-2;
        int vidx2 =  m_vSSVertex.size()-1;


        v[12] = vidx1;
        v[13] = vidx2;
        g->back_vrts[4] = vidx1;
        g->back_vrts[5] = vidx2;
}

#endif
