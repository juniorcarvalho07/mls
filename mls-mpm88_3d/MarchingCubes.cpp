/**
 * @file    MarchingCubes.cpp
 * @author  Thomas Lewiner <thomas.lewiner@polytechnique.org>
 * @author  Math Dept, PUC-Rio
 * @version 0.2
 * @date    12/08/2002
 *
 * @brief   MarchingCubes Algorithm
 */
//________________________________________________


//#if !defined(WIN32) || defined(__CYGWIN__)
//#pragma implementation
//#endif // WIN32

#include "MarchingCubes.h"

// step size of the arrays of vertices and triangles


//_____________________________________________________________________________
// print cube for debug
//void MarchingCubes::print_cube() 
//_____________________________________________________________________________



//_____________________________________________________________________________
// Constructor
/*MarchingCubes::MarchingCubes( const int size_x , const int size_y , const int size_z  ) :
//-----------------------------------------------------------------------------
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
  _triangles ((Triangle*)NULL)
{}*/
//_____________________________________________________________________________



//_____________________________________________________________________________
// Destructor
//MarchingCubes::~MarchingCubes()
//-----------------------------------------------------------------------------
//{
  //clean_all() ;
//}
//_____________________________________________________________________________



//_____________________________________________________________________________
// main algorithm
//void MarchingCubes::run( float iso )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
// init temporary structures (must set sizes before call)
//void MarchingCubes::init_temps()
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
// init all structures (must set sizes before call)
//void MarchingCubes::init_all ()
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
// clean temporary structures
//void MarchingCubes::clean_temps()
//-----------------------------------------------------------------------------
//{
  
//}
//_____________________________________________________________________________



//_____________________________________________________________________________
// clean all structures
//void MarchingCubes::clean_all()
//-----------------------------------------------------------------------------

  

//_____________________________________________________________________________



//_____________________________________________________________________________
//_____________________________________________________________________________


//_____________________________________________________________________________
// Compute the intersection points
//void MarchingCubes::compute_intersection_points( real iso )
//_____________________________________________________________________________





//_____________________________________________________________________________
// Test a face
// if face>0 return true if the face contains a part of the surface
//bool MarchingCubes::test_face( schar face )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________





//_____________________________________________________________________________
// Test the interior of a cube
// if s == 7, return true  if the interior is empty
// if s ==-7, return false if the interior is empty
//bool MarchingCubes::test_interior( schar s )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________




//_____________________________________________________________________________
// Process a unit cube
//void MarchingCubes::process_cube( )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
// Adding triangles
//void MarchingCubes::add_triangle( const char* trig, char n, int v12 )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
// Calculating gradient

//real MarchingCubes::get_x_grad( const int i, const int j, const int k ) const
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//real MarchingCubes::get_y_grad( const int i, const int j, const int k ) const
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//real MarchingCubes::get_z_grad( const int i, const int j, const int k ) const
//-----------------------------------------------------------------------------

//_____________________________________________________________________________


//_____________________________________________________________________________
// Adding vertices

//void MarchingCubes::test_vertex_addition()



//int MarchingCubes::add_x_vertex( )
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//int MarchingCubes::add_y_vertex( )
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

//int MarchingCubes::add_z_vertex( )
//-----------------------------------------------------------------------------



//int MarchingCubes::add_c_vertex( )
//-----------------------------------------------------------------------------

//_____________________________________________________________________________



//_____________________________________________________________________________
//_____________________________________________________________________________




//_____________________________________________________________________________
// Grid exportation
void MarchingCubes::writeISO(const char *fn )
//-----------------------------------------------------------------------------
{
  unsigned char buf[sizeof(float)] ;

  FILE *fp = fopen( fn, "wb" ) ;

  // header
  * (int*) buf = _size_x ;
  fwrite(buf, sizeof(float), 1, fp);
  * (int*) buf = _size_y ;
  fwrite(buf, sizeof(float), 1, fp);
  * (int*) buf = _size_z ;
  fwrite(buf, sizeof(float), 1, fp);

  * (float*) buf = -1.0f ;
  fwrite(buf, sizeof(float), 1, fp);
  * (float*) buf =  1.0f ;
  fwrite(buf, sizeof(float), 1, fp);
  * (float*) buf = -1.0f ;
  fwrite(buf, sizeof(float), 1, fp);
  * (float*) buf =  1.0f ;
  fwrite(buf, sizeof(float), 1, fp);
  * (float*) buf = -1.0f ;
  fwrite(buf, sizeof(float), 1, fp);
  * (float*) buf =  1.0f ;
  fwrite(buf, sizeof(float), 1, fp);

  for( int i = 0 ; i < _size_x ; i++ )
  {
    for( int j = 0 ; j < _size_y ; j++ )
    {
      for( int k = 0 ; k < _size_z ; k++ )
      {
        * (float*) buf = (float)get_data( i,j,k ) ;
        fwrite(buf, sizeof(float), 1, fp);
      }
    }
  }

  fclose(fp) ;
}
//_____________________________________________________________________________





//_____________________________________________________________________________
// PLY exportation
//void MarchingCubes::writePLY(const char *fn, bool bin )
//-----------------------------------------------------------------------------



//_____________________________________________________________________________
// PLY importation
void MarchingCubes::readPLY(const char *fn )
//-----------------------------------------------------------------------------
{
  typedef struct PlyFace {
    unsigned char nverts;    /* number of Vertex indices in list */
    int *verts;              /* Vertex index list */
  } PlyFace;


  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float32, Float32, offsetof( Vertex,x ), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof( Vertex,y ), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof( Vertex,z ), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof( Vertex,nx ), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof( Vertex,ny ), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };

  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof( PlyFace,verts ),
      1, Uint8, Uint8, offsetof( PlyFace,nverts )},
  };


  FILE    *fp  = fopen( fn, "r" );
  if( !fp ) return ;
  PlyFile *ply = read_ply ( fp );
  printf("Marching Cubes::readPLY(%s)...", fn ) ;

  //-----------------------------------------------------------------------------

  // gets the number of faces and vertices
  for ( int i = 0; i < ply->num_elem_types; ++i )
  {
    int elem_count ;
    char *elem_name = setup_element_read_ply ( ply, i, &elem_count );
    if ( equal_strings ( "vertex", elem_name ) )
      _Nverts = _nverts = elem_count;
    if ( equal_strings ( "face",   elem_name ) )
      _Ntrigs = _ntrigs = elem_count;
  }
  delete [] _vertices ;
  _vertices  = new Vertex  [_Nverts] ;
  delete [] _triangles ;
  _triangles = new Triangle[_Ntrigs] ;

  //-----------------------------------------------------------------------------

  /* examine each element type that is in the file (PlyVertex, PlyFace) */

  for ( int i = 0; i < ply->num_elem_types; ++i )
  {
    /* prepare to read the i'th list of elements */
    int elem_count ;
    char *elem_name = setup_element_read_ply ( ply, i, &elem_count );

    //-----------------------------------------------------------------------------
    if ( equal_strings ( "vertex", elem_name ) )
    {
      /* set up for getting PlyVertex elements */
      setup_property_ply ( ply, &vert_props[0] );
      setup_property_ply ( ply, &vert_props[1] );
      setup_property_ply ( ply, &vert_props[2] );
      setup_property_ply ( ply, &vert_props[3] );
      setup_property_ply ( ply, &vert_props[4] );
      setup_property_ply ( ply, &vert_props[5] );

      for ( int j = 0; j < _nverts; ++j )
      {
        get_element_ply ( ply, ( void * ) (_vertices + j) );
      }
      printf("   %d vertices read\n", _nverts ) ;
    }

    //-----------------------------------------------------------------------------
    else if ( equal_strings ( "face", elem_name ) )
    {
      /* set up for getting PlyFace elements */
      /* (all we need are PlyVertex indices) */

      setup_property_ply ( ply, &face_props[0] ) ;
      PlyFace     face ;
      for ( int j = 0; j < _ntrigs; ++j )
      {
        get_element_ply ( ply, ( void * ) &face );
        if( face.nverts != 3 )
        {
          printf( "not a triangulated surface: polygon %d has %d sides\n", j, face.nverts ) ;
          return ;
        }

        _triangles[j].v1 = face.verts[0] ;
        _triangles[j].v2 = face.verts[1] ;
        _triangles[j].v3 = face.verts[2] ;

        free( face.verts ) ;
      }
      printf("   %d triangles read\n", _ntrigs ) ;
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    else  /* all non-PlyVertex and non-PlyFace elements are grabbed here */
      get_other_element_ply ( ply );
    //-----------------------------------------------------------------------------
  }

  close_ply ( ply );
  free_ply  ( ply );

//  fit_to_bbox() ;
  fclose( fp ) ;
}
//_____________________________________________________________________________



//_____________________________________________________________________________
// Open Inventor / VRML 1.0 ascii exportation
void MarchingCubes::writeIV(const char *fn )
//-----------------------------------------------------------------------------
{
  FILE *fp = fopen( fn, "w" ) ;
  int   i ;

  printf("Marching Cubes::exportIV(%s)...", fn) ;

  fprintf( fp, "#Inventor V2.1 ascii \n\nSeparator { \n    ShapeHints {\n        vertexOrdering  COUNTERCLOCKWISE\n        shapeType       UNKNOWN_SHAPE_TYPE\n        creaseAngle     0.0\n    }\n Coordinate3 { \n point [  \n" ) ;
  for ( i = 0; i < _nverts; i++ )
    fprintf( fp, " %f %f %f,\n", _vertices[i].x, _vertices[i].y, _vertices[i].z ) ;
  printf("   %d vertices written\n", _nverts ) ;

  fprintf( fp, "\n ] \n} \nNormal { \nvector [ \n" ) ;
  for ( i = 0; i < _nverts; i++ )
    fprintf( fp, " %f %f %f,\n", _vertices[i].nx, _vertices[i].ny, _vertices[i].nz ) ;

  fprintf( fp, "\n ] \n} \nIndexedFaceSet { \ncoordIndex [ \n" ) ;
  for ( i = 0; i < _ntrigs; i++ )
    fprintf( fp, "%d, %d, %d, -1,\n", _triangles[i].v1, _triangles[i].v2, _triangles[i].v3 ) ;

  fprintf( fp, " ] \n } \n } \n" ) ;
  fclose( fp ) ;
  printf("   %d triangles written\n", _ntrigs ) ;
}
//_____________________________________________________________________________
