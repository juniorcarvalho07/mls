/**
 * @file    MarchingCubes.h
 * @author  Thomas Lewiner <thomas.lewiner@polytechnique.org>
 * @author  Math Dept, PUC-Rio
 * @version 0.2
 * @date    12/08/2002
 *
 * @brief   MarchingCubes Algorithm
 */
//________________________________________________


#ifndef _MARCHINGCUBES_H_
#define _MARCHINGCUBES_H_
#define ALLOC_SIZE 65536

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
//#include <time.h>
#include <memory.h>

#include <float.h>

#include "ply.h"
#include "LookUpTable.h"
#include "vector.hh"

//#if !defined(WIN32) || defined(__CYGWIN__)
//#pragma interface
//#endif // WIN32


//_____________________________________________________________________________
// types
/** unsigned char alias */
typedef unsigned char uchar ;
/** signed char alias */
typedef   signed char schar ;
/** isovalue alias */
typedef        float real  ;

//-----------------------------------------------------------------------------
// Vertex structure
/** \struct Vertex "MarchingCubes.h" MarchingCubes
 * Position and normal of a vertex
 * \brief vertex structure
 * \param x X coordinate
 * \param y Y coordinate
 * \param z Z coordinate
 * \param nx X component of the normal
 * \param ny Y component of the normal
 * \param nz Z component of the normal
 */
typedef struct
{
  real  x,  y,  z ;  /**< Vertex coordinates */
  real nx, ny, nz ;  /**< Vertex normal */
} Vertex ;

//-----------------------------------------------------------------------------
// Triangle structure
/** \struct Triangle "MarchingCubes.h" MarchingCubes
 * Indices of the oriented triange vertices
 * \brief triangle structure
 * \param v1 First vertex index
 * \param v2 Second vertex index
 * \param v3 Third vertex index
 */
typedef struct
{
  int v1,v2,v3 ;  /**< Triangle vertices */
} Triangle ;
//_____________________________________________________________________________



//_____________________________________________________________________________
/** Marching Cubes algorithm wrapper */
/** \class MarchingCubes
  * \brief Marching Cubes algorithm.
  */
class MarchingCubesL
//-----------------------------------------------------------------------------
{
// Constructors
public :
  /**
   * Main and default constructor
   * \brief constructor
   * \param size_x width  of the grid
   * \param size_y depth  of the grid
   * \param size_z height of the grid
   */
  MarchingCubesL ( const int size_x = -1, const int size_y = -1, const int size_z = -1 ):
    _originalMC(false),
  _ext_data  (false),
  _size_x    (size_x),
  _size_y    (size_y),
  _size_z    (size_z),
  _data      ((real *)NULL),
  _x_verts   (( int *)NULL),
  _y_verts   (( int *)NULL),
  _z_verts   (( int *)NULL),
  _nverts    (0),
  _ntrigs    (0),
  _Nverts    (0),
  _Ntrigs    (0),
  _vertices  (( Vertex *)NULL),
  _triangles ((Triangle*)NULL){}
  /** Destructor */
  ~MarchingCubesL() {clean_all() ;}

  
void CopyPolyhedron(std::vector<std::vector<unsigned int> > &triangles,std::vector <Vector> &MCMeshPoints,std::vector<Vector> *nrms)
{
  int i;
  unsigned int nnrms = _nverts;
          nrms->resize(nnrms);


          for(unsigned int i = 0; i < nnrms; i++){
                  nrms->at(i).x = 0;
                  nrms->at(i).y = 0;
                  nrms->at(i).z = 0;
          }
  //MCMeshPoints.erase(MCMeshPoints.begin(),MCMeshPoints.end());
  MCMeshPoints.resize(_nverts);
  
  for ( i = 0; i < _nverts; i++ )
  {
    //Vector v(_vertices[i].x,_vertices[i].y,_vertices[i].z);
    nrms->at(i).x = _vertices[i].nx;
    nrms->at(i).y = _vertices[i].ny;
    nrms->at(i).z = _vertices[i].nz;
    
    MCMeshPoints.at(i).x=_vertices[i].x;
    MCMeshPoints.at(i).y=_vertices[i].y;
    MCMeshPoints.at(i).z=_vertices[i].z;
    
  }
  
  for ( i = _ntrigs-1; i >=0 ; i-- )
  {
    std::vector<unsigned int> tri;tri.resize(3);
      tri[0] = _triangles[i].v1;
      tri[1] = _triangles[i].v2;
      tri[2] = _triangles[i].v3;
      triangles.push_back(tri);
  }
}
  

//-----------------------------------------------------------------------------
// Accessors
public :
  /** accesses the number of vertices of the generated mesh */
  inline const int nverts() const { return _nverts ; }
  /** accesses the number of triangles of the generated mesh */
  inline const int ntrigs() const { return _ntrigs ; }
  /** accesses a specific vertex of the generated mesh */
  inline Vertex   * vert( const int i ) const { if( i < 0  || i >= _nverts ) return ( Vertex *)NULL ; return _vertices  + i ; }
  /** accesses a specific triangle of the generated mesh */
  inline Triangle * trig( const int i ) const { if( i < 0  || i >= _ntrigs ) return (Triangle*)NULL ; return _triangles + i ; }

  /** accesses the vertex buffer of the generated mesh */
  inline Vertex   *vertices () { return _vertices  ; }
  /** accesses the triangle buffer of the generated mesh */
  inline Triangle *triangles() { return _triangles ; }

  /**  accesses the width  of the grid */
  inline const int size_x() const { return _size_x ; }
  /**  accesses the depth  of the grid */
  inline const int size_y() const { return _size_y ; }
  /**  accesses the height of the grid */
  inline const int size_z() const { return _size_z ; }

  /**
   * changes the size of the grid
   * \param size_x width  of the grid
   * \param size_y depth  of the grid
   * \param size_z height of the grid
   */
  inline void set_resolution( const int size_x, const int size_y, const int size_z, const real cellsizex,const real cellsizey,const real cellsizez,const real bx,const real by,const real bz ) 
  { _size_x = size_x ;  _size_y = size_y ;  _size_z = size_z ;
    _cellsizex = cellsizex ;  _cellsizey = cellsizey ;  _cellsizez = cellsizez;
     _bx = bx ;  _by = by ;  _bz = bz ;
  }
  /**
   * selects wether the algorithm will use the enhanced topologically controlled lookup table or the original MarchingCubes
   * \param originalMC true for the original Marching Cubes
   */
  inline void set_method    ( const bool originalMC = false ) { _originalMC = originalMC ; }
  /**
   * selects to use data from another class
   * \param data is the pointer to the external data, allocated as a size_x*size_y*size_z vector running in x first
   */
  inline void set_ext_data  ( real *data )
  { if( !_ext_data ) delete [] _data ;  _ext_data = data != NULL ;  if( _ext_data ) _data = data ; }
  /**
   * selects to allocate data
   */
  inline void set_int_data  () { _ext_data = false ;  _data = NULL ; }

  // Data access
  /**
   * accesses a specific cube of the grid
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline const real get_data  ( const int i, const int j, const int k ) const { return _data[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * sets a specific cube of the grid
   * \param val new value for the cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_data  ( const real val, const int i, const int j, const int k ) { _data[ i + j*_size_x + k*_size_x*_size_y] = val ; }

  // Data initialization
  /** inits temporary structures (must set sizes before call) : the grid and the vertex index per cube */
  void init_temps () 
  {
  if( !_ext_data )
    _data    = new real [_size_x * _size_y * _size_z] ;
  _x_verts = new int  [_size_x * _size_y * _size_z] ;
  _y_verts = new int  [_size_x * _size_y * _size_z] ;
  _z_verts = new int  [_size_x * _size_y * _size_z] ;

  memset( _x_verts, -1, _size_x * _size_y * _size_z * sizeof( int ) ) ;
  memset( _y_verts, -1, _size_x * _size_y * _size_z * sizeof( int ) ) ;
  memset( _z_verts, -1, _size_x * _size_y * _size_z * sizeof( int ) ) ;
}

  /** inits all structures (must set sizes before call) : the temporary structures and the mesh buffers */
  void init_all   () 
    {
  init_temps() ;

  _nverts = _ntrigs = 0 ;
  _Nverts = _Ntrigs = ALLOC_SIZE ;
  _vertices  = new Vertex  [_Nverts] ;
  _triangles = new Triangle[_Ntrigs] ;
}
  /** clears temporary structures : the grid and the main */
 inline void clean_temps() 
 {
   if( !_ext_data )
    delete [] _data;
  delete [] _x_verts;
  delete [] _y_verts;
  delete [] _z_verts;

  if( !_ext_data )
    _data     = (real*)NULL ;
  _x_verts  = (int*)NULL ;
  _y_verts  = (int*)NULL ;
  _z_verts  = (int*)NULL ;
 }
  /** clears all structures : the temporary structures and the mesh buffers */
  inline void clean_all  () 
  {
   clean_temps() ;
  delete [] _vertices  ;
  delete [] _triangles ;
  _vertices  = (Vertex   *)NULL ;
  _triangles = (Triangle *)NULL ;
  _nverts = _ntrigs = 0 ;
  _Nverts = _Ntrigs = 0 ;

  _size_x = _size_y = _size_z = -1 ;
  }


//-----------------------------------------------------------------------------
// Exportation
public :
  /**
   * PLY exportation of the generated mesh
   * \param fn  name of the PLY file to create
   * \param bin if true, the PLY will be written in binary mode
   */

//_____________________________________________________________________________


  /**
   * PLY importation of a mesh
   * \param fn  name of the PLY file to read from
   */
  void readPLY( const char *fn ) ;

  /**
   * VRML / Open Inventor exportation of the generated mesh
   * \param fn  name of the IV file to create
   */
  void writeIV ( const char *fn ) ;

  /**
   * ISO exportation of the input grid
   * \param fn  name of the ISO file to create
   */
  void writeISO( const char *fn ) ;


//-----------------------------------------------------------------------------
// Algorithm
public :
  /**
   * Main algorithm : must be called after init_all
   * \param iso isovalue
   */
  inline void run( real iso = (float)0.0 ) 
  {
 // clock_t time = clock() ;

  compute_intersection_points( iso ) ;

  for( _k = 0 ; _k < _size_z-1 ; _k++ )
  for( _j = 0 ; _j < _size_y-1 ; _j++ )
  for( _i = 0 ; _i < _size_x-1 ; _i++ )
  {
    _lut_entry = 0 ;
    for( int p = 0 ; p < 8 ; ++p )
    {
      _cube[p] = get_data( _i+((p^(p>>1))&1), _j+((p>>1)&1), _k+((p>>2)&1) ) - iso ;
      if( fabs( _cube[p] ) < FLT_EPSILON ) _cube[p] = FLT_EPSILON ;
      if( _cube[p] > 0 ) _lut_entry += 1 << p ;
    }

    process_cube( ) ;
  }

  //printf("Marching Cubes ran in %lf secs.\n", (double)(clock() - time)/CLOCKS_PER_SEC) ;
}

protected :
  /** tesselates one cube */
  inline void process_cube () 
  {
  if( _originalMC )
  {
    char nt = 0 ;
    while( casesClassic[_lut_entry][3*nt] != -1 ) nt++ ;
    add_triangle( casesClassic[_lut_entry], nt ) ;
    return ;
  }

  int   v12 = -1 ;
  _case   = cases[_lut_entry][0] ;
  _config = cases[_lut_entry][1] ;
  _subconfig = 0 ;

  switch( _case )
  {
  case  0 :
    break ;

  case  1 :
    add_triangle( tiling1[_config], 1 ) ;
    break ;

  case  2 :
    add_triangle( tiling2[_config], 2 ) ;
    break ;

  case  3 :
    if( test_face( test3[_config]) )
      add_triangle( tiling3_2[_config], 4 ) ; // 3.2
    else
      add_triangle( tiling3_1[_config], 2 ) ; // 3.1
    break ;

  case  4 :
    if( test_interior( test4[_config]) )
      add_triangle( tiling4_1[_config], 2 ) ; // 4.1.1
    else
      add_triangle( tiling4_2[_config], 6 ) ; // 4.1.2
    break ;

  case  5 :
    add_triangle( tiling5[_config], 3 ) ;
    break ;

  case  6 :
    if( test_face( test6[_config][0]) )
      add_triangle( tiling6_2[_config], 5 ) ; // 6.2
    else
    {
      if( test_interior( test6[_config][1]) )
        add_triangle( tiling6_1_1[_config], 3 ) ; // 6.1.1
      else
	  {
        v12 = add_c_vertex() ;
        add_triangle( tiling6_1_2[_config], 9 , v12) ; // 6.1.2
      }
    }
    break ;

  case  7 :
    if( test_face( test7[_config][0] ) ) _subconfig +=  1 ;
    if( test_face( test7[_config][1] ) ) _subconfig +=  2 ;
    if( test_face( test7[_config][2] ) ) _subconfig +=  4 ;
    switch( _subconfig )
      {
      case 0 :
        add_triangle( tiling7_1[_config], 3 ) ; break ;
      case 1 :
        add_triangle( tiling7_2[_config][0], 5 ) ; break ;
      case 2 :
        add_triangle( tiling7_2[_config][1], 5 ) ; break ;
      case 3 :
        v12 = add_c_vertex() ;
        add_triangle( tiling7_3[_config][0], 9, v12 ) ; break ;
      case 4 :
        add_triangle( tiling7_2[_config][2], 5 ) ; break ;
      case 5 :
        v12 = add_c_vertex() ;
        add_triangle( tiling7_3[_config][1], 9, v12 ) ; break ;
      case 6 :
        v12 = add_c_vertex() ;
        add_triangle( tiling7_3[_config][2], 9, v12 ) ; break ;
      case 7 :
        if( test_interior( test7[_config][3]) )
          add_triangle( tiling7_4_2[_config], 9 ) ;
        else
          add_triangle( tiling7_4_1[_config], 5 ) ;
        break ;
      };
    break ;

  case  8 :
    add_triangle( tiling8[_config], 2 ) ;
    break ;

  case  9 :
    add_triangle( tiling9[_config], 4 ) ;
    break ;

  case 10 :
    if( test_face( test10[_config][0]) )
    {
      if( test_face( test10[_config][1]) )
        add_triangle( tiling10_1_1_[_config], 4 ) ; // 10.1.1
      else
      {
        v12 = add_c_vertex() ;
        add_triangle( tiling10_2[_config], 8, v12 ) ; // 10.2
      }
    }
    else
    {
      if( test_face( test10[_config][1]) )
      {
        v12 = add_c_vertex() ;
        add_triangle( tiling10_2_[_config], 8, v12 ) ; // 10.2
      }
      else
      {
        if( test_interior( test10[_config][2]) )
          add_triangle( tiling10_1_1[_config], 4 ) ; // 10.1.1
        else
          add_triangle( tiling10_1_2[_config], 8 ) ; // 10.1.2
      }
    }
    break ;

  case 11 :
    add_triangle( tiling11[_config], 4 ) ;
    break ;

  case 12 :
    if( test_face( test12[_config][0]) )
    {
      if( test_face( test12[_config][1]) )
        add_triangle( tiling12_1_1_[_config], 4 ) ; // 12.1.1
      else
      {
        v12 = add_c_vertex() ;
        add_triangle( tiling12_2[_config], 8, v12 ) ; // 12.2
      }
    }
    else
    {
      if( test_face( test12[_config][1]) )
      {
        v12 = add_c_vertex() ;
        add_triangle( tiling12_2_[_config], 8, v12 ) ; // 12.2
      }
      else
      {
        if( test_interior( test12[_config][2]) )
          add_triangle( tiling12_1_1[_config], 4 ) ; // 12.1.1
        else
          add_triangle( tiling12_1_2[_config], 8 ) ; // 12.1.2
      }
    }
    break ;

  case 13 :
    if( test_face( test13[_config][0] ) ) _subconfig +=  1 ;
    if( test_face( test13[_config][1] ) ) _subconfig +=  2 ;
    if( test_face( test13[_config][2] ) ) _subconfig +=  4 ;
    if( test_face( test13[_config][3] ) ) _subconfig +=  8 ;
    if( test_face( test13[_config][4] ) ) _subconfig += 16 ;
    if( test_face( test13[_config][5] ) ) _subconfig += 32 ;
    switch( subconfig13[_subconfig] )
    {
      case 0 :/* 13.1 */
        add_triangle( tiling13_1[_config], 4 ) ; break ;

      case 1 :/* 13.2 */
        add_triangle( tiling13_2[_config][0], 6 ) ; break ;
      case 2 :/* 13.2 */
        add_triangle( tiling13_2[_config][1], 6 ) ; break ;
      case 3 :/* 13.2 */
        add_triangle( tiling13_2[_config][2], 6 ) ; break ;
      case 4 :/* 13.2 */
        add_triangle( tiling13_2[_config][3], 6 ) ; break ;
      case 5 :/* 13.2 */
        add_triangle( tiling13_2[_config][4], 6 ) ; break ;
      case 6 :/* 13.2 */
        add_triangle( tiling13_2[_config][5], 6 ) ; break ;

      case 7 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][0], 10, v12 ) ; break ;
      case 8 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][1], 10, v12 ) ; break ;
      case 9 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][2], 10, v12 ) ; break ;
      case 10 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][3], 10, v12 ) ; break ;
      case 11 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][4], 10, v12 ) ; break ;
      case 12 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][5], 10, v12 ) ; break ;
      case 13 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][6], 10, v12 ) ; break ;
      case 14 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][7], 10, v12 ) ; break ;
      case 15 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][8], 10, v12 ) ; break ;
      case 16 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][9], 10, v12 ) ; break ;
      case 17 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][10], 10, v12 ) ; break ;
      case 18 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3[_config][11], 10, v12 ) ; break ;

      case 19 :/* 13.4 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_4[_config][0], 12, v12 ) ; break ;
      case 20 :/* 13.4 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_4[_config][1], 12, v12 ) ; break ;
      case 21 :/* 13.4 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_4[_config][2], 12, v12 ) ; break ;
      case 22 :/* 13.4 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_4[_config][3], 12, v12 ) ; break ;

      case 23 :/* 13.5 */
        _subconfig = 0 ;
        if( test_interior( test13[_config][6] ) )
          add_triangle( tiling13_5_1[_config][0], 6 ) ;
        else
          add_triangle( tiling13_5_2[_config][0], 10 ) ;
        break ;
      case 24 :/* 13.5 */
        _subconfig = 1 ;
        if( test_interior( test13[_config][6] ) )
          add_triangle( tiling13_5_1[_config][1], 6 ) ;
        else
          add_triangle( tiling13_5_2[_config][1], 10 ) ;
        break ;
      case 25 :/* 13.5 */
        _subconfig = 2 ;
        if( test_interior( test13[_config][6] ) )
          add_triangle( tiling13_5_1[_config][2], 6 ) ;
        else
          add_triangle( tiling13_5_2[_config][2], 10 ) ;
        break ;
      case 26 :/* 13.5 */
        _subconfig = 3 ;
        if( test_interior( test13[_config][6] ) )
          add_triangle( tiling13_5_1[_config][3], 6 ) ;
        else
          add_triangle( tiling13_5_2[_config][3], 10 ) ;
        break ;

      case 27 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][0], 10, v12 ) ; break ;
      case 28 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][1], 10, v12 ) ; break ;
      case 29 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][2], 10, v12 ) ; break ;
      case 30 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][3], 10, v12 ) ; break ;
      case 31 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][4], 10, v12 ) ; break ;
      case 32 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][5], 10, v12 ) ; break ;
      case 33 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][6], 10, v12 ) ; break ;
      case 34 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][7], 10, v12 ) ; break ;
      case 35 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][8], 10, v12 ) ; break ;
      case 36 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][9], 10, v12 ) ; break ;
      case 37 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][10], 10, v12 ) ; break ;
      case 38 :/* 13.3 */
        v12 = add_c_vertex() ;
        add_triangle( tiling13_3_[_config][11], 10, v12 ) ; break ;

      case 39 :/* 13.2 */
        add_triangle( tiling13_2_[_config][0], 6 ) ; break ;
      case 40 :/* 13.2 */
        add_triangle( tiling13_2_[_config][1], 6 ) ; break ;
      case 41 :/* 13.2 */
        add_triangle( tiling13_2_[_config][2], 6 ) ; break ;
      case 42 :/* 13.2 */
        add_triangle( tiling13_2_[_config][3], 6 ) ; break ;
      case 43 :/* 13.2 */
        add_triangle( tiling13_2_[_config][4], 6 ) ; break ;
      case 44 :/* 13.2 */
        add_triangle( tiling13_2_[_config][5], 6 ) ; break ;

      case 45 :/* 13.1 */
        add_triangle( tiling13_1_[_config], 4 ) ; break ;

      default :
        printf("Marching Cubes: Impossible case 13?\n" ) ;  print_cube() ;
      }
      break ;

  case 14 :
    add_triangle( tiling14[_config], 4 ) ;
    break ;
  };
}
  /** tests if the components of the tesselation of the cube should be connected by the interior of an ambiguous face */
  inline bool test_face    ( schar face ) 
  {
  real A,B,C,D ;

  switch( face )
  {
  case -1 : case 1 :  A = _cube[0] ;  B = _cube[4] ;  C = _cube[5] ;  D = _cube[1] ;  break ;
  case -2 : case 2 :  A = _cube[1] ;  B = _cube[5] ;  C = _cube[6] ;  D = _cube[2] ;  break ;
  case -3 : case 3 :  A = _cube[2] ;  B = _cube[6] ;  C = _cube[7] ;  D = _cube[3] ;  break ;
  case -4 : case 4 :  A = _cube[3] ;  B = _cube[7] ;  C = _cube[4] ;  D = _cube[0] ;  break ;
  case -5 : case 5 :  A = _cube[0] ;  B = _cube[3] ;  C = _cube[2] ;  D = _cube[1] ;  break ;
  case -6 : case 6 :  A = _cube[4] ;  B = _cube[7] ;  C = _cube[6] ;  D = _cube[5] ;  break ;
  default : printf( "Invalid face code %d\n", face ) ;  print_cube() ;  A = B = C = D = 0 ;
  };

  if( fabs( A*C - B*D ) < FLT_EPSILON )
    return face >= 0 ;
  return face * A * ( A*C - B*D ) >= 0  ;  // face and A invert signs
}
  /** tests if the components of the tesselation of the cube should be connected through the interior of the cube */
  inline bool test_interior( schar s )    
  {
  real t, At=0, Bt=0, Ct=0, Dt=0, a, b ;
  char  test =  0 ;
  char  edge = -1 ; // reference edge of the triangulation

  switch( _case )
  {
  case  4 :
  case 10 :
    a = ( _cube[4] - _cube[0] ) * ( _cube[6] - _cube[2] ) - ( _cube[7] - _cube[3] ) * ( _cube[5] - _cube[1] ) ;
    b =  _cube[2] * ( _cube[4] - _cube[0] ) + _cube[0] * ( _cube[6] - _cube[2] )
             - _cube[1] * ( _cube[7] - _cube[3] ) - _cube[3] * ( _cube[5] - _cube[1] ) ;
    t = - b / (2*a) ;
    if( t<0 || t>1 ) return s>0 ;

    At = _cube[0] + ( _cube[4] - _cube[0] ) * t ;
    Bt = _cube[3] + ( _cube[7] - _cube[3] ) * t ;
    Ct = _cube[2] + ( _cube[6] - _cube[2] ) * t ;
    Dt = _cube[1] + ( _cube[5] - _cube[1] ) * t ;
    break ;

  case  6 :
  case  7 :
  case 12 :
  case 13 :
    switch( _case )
    {
    case  6 : edge = test6 [_config][2] ; break ;
    case  7 : edge = test7 [_config][4] ; break ;
    case 12 : edge = test12[_config][3] ; break ;
    case 13 : edge = tiling13_5_1[_config][_subconfig][0] ; break ;
    }
    switch( edge )
    {
    case  0 :
      t  = _cube[0] / ( _cube[0] - _cube[1] ) ;
      At = 0 ;
      Bt = _cube[3] + ( _cube[2] - _cube[3] ) * t ;
      Ct = _cube[7] + ( _cube[6] - _cube[7] ) * t ;
      Dt = _cube[4] + ( _cube[5] - _cube[4] ) * t ;
      break ;
    case  1 :
      t  = _cube[1] / ( _cube[1] - _cube[2] ) ;
      At = 0 ;
      Bt = _cube[0] + ( _cube[3] - _cube[0] ) * t ;
      Ct = _cube[4] + ( _cube[7] - _cube[4] ) * t ;
      Dt = _cube[5] + ( _cube[6] - _cube[5] ) * t ;
      break ;
    case  2 :
      t  = _cube[2] / ( _cube[2] - _cube[3] ) ;
      At = 0 ;
      Bt = _cube[1] + ( _cube[0] - _cube[1] ) * t ;
      Ct = _cube[5] + ( _cube[4] - _cube[5] ) * t ;
      Dt = _cube[6] + ( _cube[7] - _cube[6] ) * t ;
      break ;
    case  3 :
      t  = _cube[3] / ( _cube[3] - _cube[0] ) ;
      At = 0 ;
      Bt = _cube[2] + ( _cube[1] - _cube[2] ) * t ;
      Ct = _cube[6] + ( _cube[5] - _cube[6] ) * t ;
      Dt = _cube[7] + ( _cube[4] - _cube[7] ) * t ;
      break ;
    case  4 :
      t  = _cube[4] / ( _cube[4] - _cube[5] ) ;
      At = 0 ;
      Bt = _cube[7] + ( _cube[6] - _cube[7] ) * t ;
      Ct = _cube[3] + ( _cube[2] - _cube[3] ) * t ;
      Dt = _cube[0] + ( _cube[1] - _cube[0] ) * t ;
      break ;
    case  5 :
      t  = _cube[5] / ( _cube[5] - _cube[6] ) ;
      At = 0 ;
      Bt = _cube[4] + ( _cube[7] - _cube[4] ) * t ;
      Ct = _cube[0] + ( _cube[3] - _cube[0] ) * t ;
      Dt = _cube[1] + ( _cube[2] - _cube[1] ) * t ;
      break ;
    case  6 :
      t  = _cube[6] / ( _cube[6] - _cube[7] ) ;
      At = 0 ;
      Bt = _cube[5] + ( _cube[4] - _cube[5] ) * t ;
      Ct = _cube[1] + ( _cube[0] - _cube[1] ) * t ;
      Dt = _cube[2] + ( _cube[3] - _cube[2] ) * t ;
      break ;
    case  7 :
      t  = _cube[7] / ( _cube[7] - _cube[4] ) ;
      At = 0 ;
      Bt = _cube[6] + ( _cube[5] - _cube[6] ) * t ;
      Ct = _cube[2] + ( _cube[1] - _cube[2] ) * t ;
      Dt = _cube[3] + ( _cube[0] - _cube[3] ) * t ;
      break ;
    case  8 :
      t  = _cube[0] / ( _cube[0] - _cube[4] ) ;
      At = 0 ;
      Bt = _cube[3] + ( _cube[7] - _cube[3] ) * t ;
      Ct = _cube[2] + ( _cube[6] - _cube[2] ) * t ;
      Dt = _cube[1] + ( _cube[5] - _cube[1] ) * t ;
      break ;
    case  9 :
      t  = _cube[1] / ( _cube[1] - _cube[5] ) ;
      At = 0 ;
      Bt = _cube[0] + ( _cube[4] - _cube[0] ) * t ;
      Ct = _cube[3] + ( _cube[7] - _cube[3] ) * t ;
      Dt = _cube[2] + ( _cube[6] - _cube[2] ) * t ;
      break ;
    case 10 :
      t  = _cube[2] / ( _cube[2] - _cube[6] ) ;
      At = 0 ;
      Bt = _cube[1] + ( _cube[5] - _cube[1] ) * t ;
      Ct = _cube[0] + ( _cube[4] - _cube[0] ) * t ;
      Dt = _cube[3] + ( _cube[7] - _cube[3] ) * t ;
      break ;
    case 11 :
      t  = _cube[3] / ( _cube[3] - _cube[7] ) ;
      At = 0 ;
      Bt = _cube[2] + ( _cube[6] - _cube[2] ) * t ;
      Ct = _cube[1] + ( _cube[5] - _cube[1] ) * t ;
      Dt = _cube[0] + ( _cube[4] - _cube[0] ) * t ;
      break ;
    default : printf( "Invalid edge %d\n", edge ) ;  print_cube() ;  break ;
    }
    break ;

  default : printf( "Invalid ambiguous case %d\n", _case ) ;  print_cube() ;  break ;
  }

  if( At >= 0 ) test ++ ;
  if( Bt >= 0 ) test += 2 ;
  if( Ct >= 0 ) test += 4 ;
  if( Dt >= 0 ) test += 8 ;
  switch( test )
  {
  case  0 : return s>0 ;
  case  1 : return s>0 ;
  case  2 : return s>0 ;
  case  3 : return s>0 ;
  case  4 : return s>0 ;
  case  5 : if( At * Ct - Bt * Dt <  FLT_EPSILON ) return s>0 ; break ;
  case  6 : return s>0 ;
  case  7 : return s<0 ;
  case  8 : return s>0 ;
  case  9 : return s>0 ;
  case 10 : if( At * Ct - Bt * Dt >= FLT_EPSILON ) return s>0 ; break ;
  case 11 : return s<0 ;
  case 12 : return s>0 ;
  case 13 : return s<0 ;
  case 14 : return s<0 ;
  case 15 : return s<0 ;
  }

  return s<0 ;
}


//-----------------------------------------------------------------------------
// Operations
protected :
  /**
   * computes almost all the vertices of the mesh by interpolation along the cubes edges
   * \param iso isovalue
   */
  inline void compute_intersection_points( real iso ) 
  //-----------------------------------------------------------------------------
{
  for( _k = 0 ; _k < _size_z ; _k++ )
  for( _j = 0 ; _j < _size_y ; _j++ )
  for( _i = 0 ; _i < _size_x ; _i++ )
  {
    _cube[0] = get_data( _i, _j, _k ) - iso ;
    if( _i < _size_x - 1 ) _cube[1] = get_data(_i+1, _j , _k ) - iso ;
    else                   _cube[1] = _cube[0] ;

    if( _j < _size_y - 1 ) _cube[3] = get_data( _i ,_j+1, _k ) - iso ;
    else                   _cube[3] = _cube[0] ;

    if( _k < _size_z - 1 ) _cube[4] = get_data( _i , _j ,_k+1) - iso ;
    else                   _cube[4] = _cube[0] ;

    if( fabs( _cube[0] ) < FLT_EPSILON ) _cube[0] = FLT_EPSILON ;
    if( fabs( _cube[1] ) < FLT_EPSILON ) _cube[1] = FLT_EPSILON ;
    if( fabs( _cube[3] ) < FLT_EPSILON ) _cube[3] = FLT_EPSILON ;
    if( fabs( _cube[4] ) < FLT_EPSILON ) _cube[4] = FLT_EPSILON ;

    if( _cube[0] < 0 )
    {
      if( _cube[1] > 0 ) set_x_vert( add_x_vertex( ), _i,_j,_k ) ;
      if( _cube[3] > 0 ) set_y_vert( add_y_vertex( ), _i,_j,_k ) ;
      if( _cube[4] > 0 ) set_z_vert( add_z_vertex( ), _i,_j,_k ) ;
    }
    else
    {
      if( _cube[1] < 0 ) set_x_vert( add_x_vertex( ), _i,_j,_k ) ;
      if( _cube[3] < 0 ) set_y_vert( add_y_vertex( ), _i,_j,_k ) ;
      if( _cube[4] < 0 ) set_z_vert( add_z_vertex( ), _i,_j,_k ) ;
    }
  }
}


  /**
   * routine to add a triangle to the mesh
   * \param trig the code for the triangle as a sequence of edges index
   * \param n    the number of triangles to produce
   * \param v12  the index of the interior vertex to use, if necessary
   */
  inline void add_triangle ( const char* trig, char n, int v12 = -1 ) 
  {
  int    tv[3] ;

  for( int t = 0 ; t < 3*n ; t++ )
  {
    switch( trig[t] )
    {
    case  0 : tv[ t % 3 ] = get_x_vert( _i , _j , _k ) ; break ;
    case  1 : tv[ t % 3 ] = get_y_vert(_i+1, _j , _k ) ; break ;
    case  2 : tv[ t % 3 ] = get_x_vert( _i ,_j+1, _k ) ; break ;
    case  3 : tv[ t % 3 ] = get_y_vert( _i , _j , _k ) ; break ;
    case  4 : tv[ t % 3 ] = get_x_vert( _i , _j ,_k+1) ; break ;
    case  5 : tv[ t % 3 ] = get_y_vert(_i+1, _j ,_k+1) ; break ;
    case  6 : tv[ t % 3 ] = get_x_vert( _i ,_j+1,_k+1) ; break ;
    case  7 : tv[ t % 3 ] = get_y_vert( _i , _j ,_k+1) ; break ;
    case  8 : tv[ t % 3 ] = get_z_vert( _i , _j , _k ) ; break ;
    case  9 : tv[ t % 3 ] = get_z_vert(_i+1, _j , _k ) ; break ;
    case 10 : tv[ t % 3 ] = get_z_vert(_i+1,_j+1, _k ) ; break ;
    case 11 : tv[ t % 3 ] = get_z_vert( _i ,_j+1, _k ) ; break ;
    case 12 : tv[ t % 3 ] = v12 ; break ;
    default : break ;
    }

    if( tv[t%3] == -1 )
    {
      printf("Marching Cubes: invalid triangle %d\n", _ntrigs+1) ;
      print_cube() ;
    }

    if( t%3 == 2 )
    {
      if( _ntrigs >= _Ntrigs )
      {
        Triangle *temp = _triangles ;
        _triangles = new Triangle[ 2*_Ntrigs ] ;
        memcpy( _triangles, temp, _Ntrigs*sizeof(Triangle) ) ;
        delete[] temp ;
        //printf("%d allocated triangles\n", _Ntrigs) ;
        _Ntrigs *= 2 ;
      }

      Triangle *T = _triangles + _ntrigs++ ;
      T->v1    = tv[0] ;
      T->v2    = tv[1] ;
      T->v3    = tv[2] ;
    }
  }
}

  /** tests and eventually doubles the vertex buffer capacity for a new vertex insertion */
 inline void test_vertex_addition() 
 {
  if( _nverts >= _Nverts )
  {
    Vertex *temp = _vertices ;
    _vertices = new Vertex[ _Nverts*2 ] ;
    memcpy( _vertices, temp, _Nverts*sizeof(Vertex) ) ;
    delete[] temp ;
    //printf("%d allocated vertices\n", _Nverts) ;
    _Nverts *= 2 ;
  }
}
  /** adds a vertex on the current horizontal edge */
 inline int add_x_vertex() 
 {
  test_vertex_addition() ;
  Vertex *vert = _vertices + _nverts++ ;

  real u = ( _cube[0] ) / ( _cube[0] - _cube[1] ) ;
  vert->x      = (real)_bx+(_i+u)*_cellsizex;
  vert->y      = (real)_by+ _j*_cellsizey ;
  vert->z      = (real)_bz+ _k*_cellsizez ;

  vert->nx = (1-u)*get_x_grad(_i,_j,_k) + u*get_x_grad(_i+1,_j,_k) ;
  vert->ny = (1-u)*get_y_grad(_i,_j,_k) + u*get_y_grad(_i+1,_j,_k) ;
  vert->nz = (1-u)*get_z_grad(_i,_j,_k) + u*get_z_grad(_i+1,_j,_k) ;

  u = (real) sqrt( vert->nx * vert->nx + vert->ny * vert->ny +vert->nz * vert->nz ) ;
  if( u > 0 )
  {
    vert->nx /= u ;
    vert->ny /= u ;
    vert->nz /= u ;
  }


  return _nverts-1 ;
}
  /** adds a vertex on the current longitudinal edge */
 inline  int add_y_vertex() 
 {
  test_vertex_addition() ;
  Vertex *vert = _vertices + _nverts++ ;

  real u = ( _cube[0] ) / ( _cube[0] - _cube[3] ) ;

  vert->x      = (real)_bx +_i*_cellsizex;
  vert->y      = (real)_by+(_j+u)*_cellsizey;
  vert->z      = (real)_bz+ _k*_cellsizez;

  vert->nx = (1-u)*get_x_grad(_i,_j,_k) + u*get_x_grad(_i,_j+1,_k) ;
  vert->ny = (1-u)*get_y_grad(_i,_j,_k) + u*get_y_grad(_i,_j+1,_k) ;
  vert->nz = (1-u)*get_z_grad(_i,_j,_k) + u*get_z_grad(_i,_j+1,_k) ;

  u = (real) sqrt( vert->nx * vert->nx + vert->ny * vert->ny +vert->nz * vert->nz ) ;
  if( u > 0 )
  {
    vert->nx /= u ;
    vert->ny /= u ;
    vert->nz /= u ;
  }

  return _nverts-1 ;
}
  /** adds a vertex on the current vertical edge */
 inline int add_z_vertex() 
 {
  test_vertex_addition() ;
  Vertex *vert = _vertices + _nverts++ ;

  real u = ( _cube[0] ) / ( _cube[0] - _cube[4] ) ;

  vert->x      = (real) _bx +_i*_cellsizex; ;
  vert->y      = (real) _by+ _j*_cellsizey; ;
  vert->z      = (real)_bz+(_k+u)*_cellsizez;

  vert->nx = (1-u)*get_x_grad(_i,_j,_k) + u*get_x_grad(_i,_j,_k+1) ;
  vert->ny = (1-u)*get_y_grad(_i,_j,_k) + u*get_y_grad(_i,_j,_k+1) ;
  vert->nz = (1-u)*get_z_grad(_i,_j,_k) + u*get_z_grad(_i,_j,_k+1) ;

  u = (real) sqrt( vert->nx * vert->nx + vert->ny * vert->ny +vert->nz * vert->nz ) ;
  if( u > 0 )
  {
    vert->nx /= u ;
    vert->ny /= u ;
    vert->nz /= u ;
  }

  return _nverts-1 ;
}
  /** adds a vertex inside the current cube */
  inline int add_c_vertex() 
  {
  test_vertex_addition() ;
  Vertex *vert = _vertices + _nverts++ ;

  real u = 0 ;
  int   vid ;

  vert->x = vert->y = vert->z =  vert->nx = vert->ny = vert->nz = 0 ;

  // Computes the average of the intersection points of the cube
  vid = get_x_vert( _i , _j , _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_y_vert(_i+1, _j , _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_x_vert( _i ,_j+1, _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_y_vert( _i , _j , _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_x_vert( _i , _j ,_k+1) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_y_vert(_i+1, _j ,_k+1) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_x_vert( _i ,_j+1,_k+1) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_y_vert( _i , _j ,_k+1) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_z_vert( _i , _j , _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_z_vert(_i+1, _j , _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_z_vert(_i+1,_j+1, _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }
  vid = get_z_vert( _i ,_j+1, _k ) ;
  if( vid != -1 ) { ++u ; const Vertex &v = _vertices[vid] ; vert->x += v.x ;  vert->y += v.y ;  vert->z += v.z ;  vert->nx += v.nx ; vert->ny += v.ny ; vert->nz += v.nz ; }

  vert->x  /= u ;
  vert->y  /= u ;
  vert->z  /= u ;

  u = (real) sqrt( vert->nx * vert->nx + vert->ny * vert->ny +vert->nz * vert->nz ) ;
  if( u > 0 )
  {
    vert->nx /= u ;
    vert->ny /= u ;
    vert->nz /= u ;
  }

  return _nverts-1 ;
}

  /**
   * interpolates the horizontal gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
 inline real get_x_grad( const int i, const int j, const int k )
  {
  if( i > 0 )
  {
    if ( i < _size_x - 1 )
      return ( get_data( i+1, j, k ) - get_data( i-1, j, k ) ) / 2 ;
    else
      return get_data( i, j, k ) - get_data( i-1, j, k ) ;
  }
  else
    return get_data( i+1, j, k ) - get_data( i, j, k ) ;
}
  /**
   * interpolates the longitudinal gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline real get_y_grad( const int i, const int j, const int k ) 
  {
  if( j > 0 )
  {
    if ( j < _size_y - 1 )
      return ( get_data( i, j+1, k ) - get_data( i, j-1, k ) ) / 2 ;
    else
      return get_data( i, j, k ) - get_data( i, j-1, k ) ;
  }
  else
    return get_data( i, j+1, k ) - get_data( i, j, k ) ;
}
  /**
   * interpolates the vertical gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline real get_z_grad( const int i, const int j, const int k ) 
  {
  if( k > 0 )
  {
    if ( k < _size_z - 1 )
      return ( get_data( i, j, k+1 ) - get_data( i, j, k-1 ) ) / 2 ;
    else
      return get_data( i, j, k ) - get_data( i, j, k-1 ) ;
  }
  else
    return get_data( i, j, k+1 ) - get_data( i, j, k ) ;
}

  /**
   * accesses the pre-computed vertex index on the lower horizontal edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_x_vert( const int i, const int j, const int k ) const { return _x_verts[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * accesses the pre-computed vertex index on the lower longitudinal edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_y_vert( const int i, const int j, const int k ) const { return _y_verts[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * accesses the pre-computed vertex index on the lower vertical edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_z_vert( const int i, const int j, const int k ) const { return _z_verts[ i + j*_size_x + k*_size_x*_size_y] ; }

  /**
   * sets the pre-computed vertex index on the lower horizontal edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_x_vert( const int val, const int i, const int j, const int k ) { _x_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }
  /**
   * sets the pre-computed vertex index on the lower longitudinal edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_y_vert( const int val, const int i, const int j, const int k ) { _y_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }
  /**
   * sets the pre-computed vertex index on the lower vertical edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_z_vert( const int val, const int i, const int j, const int k ) { _z_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }

  /** prints cube for debug */
  inline void print_cube() { printf( "\t%f %f %f %f %f %f %f %f\n", _cube[0], _cube[1], _cube[2], _cube[3], _cube[4], _cube[5], _cube[6], _cube[7]) ; }

//-----------------------------------------------------------------------------
// Elements
protected :
  bool      _originalMC ;   /**< selects wether the algorithm will use the enhanced topologically controlled lookup table or the original MarchingCubes */
  bool      _ext_data   ;   /**< selects wether to allocate data or use data from another class */

  int       _size_x     ;  /**< width  of the grid */
  int       _size_y     ;  /**< depth  of the grid */
  int       _size_z     ;  /**< height of the grid */
  real     *_data       ;  /**< implicit function values sampled on the grid */
  
  real     _cellsizex;
  real     _cellsizey;
  real     _cellsizez;  /* size of eache cell*/

  real     _bx;
  real     _by;
  real     _bz;  /* begin of the grid*/

  real     _ex;
  real     _ey;
  real     _ez;  /* end of the grid*/

  
  int      *_x_verts    ;  /**< pre-computed vertex indices on the lower horizontal   edge of each cube */
  int      *_y_verts    ;  /**< pre-computed vertex indices on the lower longitudinal edge of each cube */
  int      *_z_verts    ;  /**< pre-computed vertex indices on the lower vertical     edge of each cube */

  int       _nverts     ;  /**< number of allocated vertices  in the vertex   buffer */
  int       _ntrigs     ;  /**< number of allocated triangles in the triangle buffer */
  int       _Nverts     ;  /**< size of the vertex   buffer */
  int       _Ntrigs     ;  /**< size of the triangle buffer */
  Vertex   *_vertices   ;  /**< vertex   buffer */
  Triangle *_triangles  ;  /**< triangle buffer */

  int       _i          ;  /**< abscisse of the active cube */
  int       _j          ;  /**< height of the active cube */
  int       _k          ;  /**< ordinate of the active cube */

  real      _cube[8]    ;  /**< values of the implicit function on the active cube */
  uchar     _lut_entry  ;  /**< cube sign representation in [0..255] */
  uchar     _case       ;  /**< case of the active cube in [0..15] */
  uchar     _config     ;  /**< configuration of the active cube */
  uchar     _subconfig  ;  /**< subconfiguration of the active cube */
};
//_____________________________________________________________________________


#endif // _MARCHINGCUBES_H_
