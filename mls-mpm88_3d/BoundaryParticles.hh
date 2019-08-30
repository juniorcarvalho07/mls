#ifndef BOUNDARYPARTICLES_H
#define BOUNDARYPARTICLES_H


#include <vector>
#include <fstream>



#include "vector.hh"
#include "MLS_MPM_fluid.hh"
#include "rx_matrix.hh"

struct GridC3D{
    std::vector<unsigned int> listSSvrtx;
    //Vec2 pmin;
    //Vec2 pmax;
    bool empty;
    bool vp;
    //Vec2 C; //centroid

    GridC3D(bool e=true, bool v=false): empty(e),vp(v)  {}
};

struct Grid3D{
    std::vector<GridC3D> vc;
    Vec3 C; //centroid
    Vec3 Pmin;//Pmax;
    int nh,nv,np;
};


struct Counter {
  std::size_t i, N;
  Counter(std::size_t N)
    : i(0), N(N)
  {}

  void operator()()
  {
    i++;
    if(i == N){
      std::cerr << "Counter reached " << N << std::endl;
    }
  }
  
};

struct InsertVisitor {

  Counter& c;
  InsertVisitor(Counter& c)
    : c(c)
  {}

  void before_insertion()
  {
    c();
  }

};

class BoundaryParticles{
public:
  
 Grid3D Del;
std::vector <int> freebPoints; 
//std::list<PointVectorPair> pv_list;
//RecPoint_collection pc;
//PointNormalList pn_list;
std::vector<Vector> MCMeshPoints;
std::vector<Vector> normals;
 std::vector<std::vector<unsigned int> > polys;
int nbp;
 
 /*void
computePCLMesh ( pcl::PointCloud<pcl::Normal>::Ptr &input, PolygonMesh &output,
         int hoppe_or_rbf, float iso_level, int grid_res, float extend_percentage, float off_surface_displacement)
{
  //PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  //fromPCLPointCloud2 (*input, *xyz_cloud);

 
};
 

void
saveCloud (const std::string &filename, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  saveVTKFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
};

 
 void constructPCLMesh(SPH &sph)
 {
   float default_iso_level = 0.02f;
int default_hoppe_or_rbf = 1;
float default_extend_percentage = 0.0f;
int default_grid_res = 50;
float default_off_surface_displacement = 0.01f;

 pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
 //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud_ptr (new pcl::PointCloud<pcl::PointNormal>);


   int nV =sph.particles.size();
   int k=0;
     for(int i = 0; i< nV; i++){
       if(sph.particles[i].b)
       {
	 pcl::PointXYZ basic_point;
	 basic_point.x = sph.particles[i].x.x;
	 basic_point.y = sph.particles[i].x.y;
	 basic_point.z = sph.particles[i].x.z;
	 basic_cloud_ptr->points.push_back(basic_point);
	 k++;
	 
       }
     }
     
     
   // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (basic_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (sph.smoothing_radius());
  ne.compute (*cloud_normals1);
  
  
   // Apply the marching cubes algorithm
  PolygonMesh output;
  MarchingCubes<PointNormal> *mc;
  if (default_hoppe_or_rbf == 0)
    mc = new MarchingCubesHoppe<PointNormal> ();
  else
  {
    mc = new MarchingCubesRBF<PointNormal> ();
    (reinterpret_cast<MarchingCubesRBF<PointNormal>*> (mc))->setOffSurfaceDisplacement (default_off_surface_displacement);
  }

  mc->setIsoLevel (default_iso_level);
  mc->setGridResolution (default_grid_res, default_grid_res, default_grid_res);
  mc->setPercentageExtendGrid (default_extend_percentage);
  pcl::concatenateFields(*basic_cloud_ptr,*cloud_normals1,*normal_cloud_ptr);
  mc->setInputCloud(normal_cloud_ptr);

  
  
  mc->reconstruct (output);
  delete mc;


//  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//  mls.setInputCloud (basic_cloud_ptr);
//  mls.setSearchRadius (sph.smoothing_radius());
//  mls.setPolynomialFit (true);
//  mls.setPolynomialOrder (2);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingRadius (0.005);
//  mls.setUpsamplingStepSize (0.003);
//  PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
//  mls.process (*cloud_smoothed);
//  NormalEstimationOMP<PointXYZ, Normal> ne;
//  ne.setNumberOfThreads (8);
//  ne.setInputCloud (cloud_smoothed);
//  ne.setRadiusSearch (sph.smoothing_radius());
//  Eigen::Vector4f centroid;
//  compute3DCentroid (*cloud_smoothed, centroid);
//  ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
//  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
//  ne.compute (*cloud_normals);
//  for (size_t i = 0; i < cloud_normals->size (); ++i)
//  {
//  cloud_normals->points[i].normal_x *= -1;
//  cloud_normals->points[i].normal_y *= -1;
//  cloud_normals->points[i].normal_z *= -1;
//  }
//  PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
//  pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
//  PolygonMesh output;
//  if(default_hoppe_or_rbf==2)
//  {
//       Poisson<PointNormal> poisson;
//       poisson.setDepth (9);
//       poisson.setInputCloud
//       (cloud_smoothed_normals);
//       
//       poisson.reconstruct (output);
//  }
//  else
//  {
//    MarchingCubes<PointNormal> *mc;
//     if (default_hoppe_or_rbf == 0)
//       mc = new MarchingCubesHoppe<PointNormal> ();
//     else
//     {
//       mc = new MarchingCubesRBF<PointNormal> ();
//       (reinterpret_cast<MarchingCubesRBF<PointNormal>*> (mc))->setOffSurfaceDisplacement (default_off_surface_displacement);
//     }
// 
//     
//     
//     mc->setIsoLevel (default_iso_level);
//     mc->setGridResolution (default_grid_res, default_grid_res, default_grid_res);
//     mc->setPercentageExtendGrid (default_extend_percentage);
//     //pcl::concatenateFields(*basic_cloud_ptr,*cloud_normals1,*p_n_cloud_c);
//     mc->setInputCloud(cloud_smoothed_normals);
// 
//     
// 
//     
//     mc->reconstruct (output);
//     cout << "memory ok" << std::endl;
//     delete mc;
//  }
 
  // Save into the second file
  //saveCloud ("teste.vtk", output);
DrawPCLSurafce(output,sph.particles[0].color,false);  
 };
 
 void DrawPCLSurafce(const PolygonMesh &p,TColorRGBA c,bool edgeok)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<float>Mz;
  std::vector<unsigned int>ids;
  int nv,nc,i,j,id0,id1,id2;
  nv=p.cloud.width;
  nc=p.polygons.size();
  float x,y,z,mz;
  Point_iterator pit;
  MCMeshPoints.resize(nv);
  Mz.resize(nc);
  ids.resize(nc);
  polys.resize(nc);
  
  pcl::fromPCLPointCloud2(p.cloud,*pc);
  i=0;
  //cout << "nv =" << nv <<std::endl;
  for(i=0;i<nv;i++)
  {
    x=pc->points[i].x;y=pc->points[i].y;z=pc->points[i].z;
    Vector pi(x,y,z);
    //cout<< pi;
    MCMeshPoints[i]=pi;
   
  }
  
  for(i=0;i<nc;i++){
        
	  j=0;
	 std::vector<unsigned int> t;
	t.resize(3);
        do {
	    t[j]= p.polygons[i].vertices[j];
	    
	    j++;
	    
        } while ( j<3);
	//mz/=3.0;
	//Mz[i]=mz;
	ids[i]=i;
	polys[i]=t;
  }
  	
	CalVertexNormals(&MCMeshPoints,nv,polys,nc,&normals);
	

	
// 	for(i=0;i<nc-1;i++)
// 	{
// 	  for(j=i+1;j<nc;j++)
// 	  {
// 	    if (Mz[j]<Mz[i])
// 	    {
// 	      id0=ids[i];
// 	      ids[i]=ids[j];
// 	      ids[j]=id0;
// 	    }
// 	  }
// 	}
	
	glColor4f(c.R, c.G, c.B,c.A);
    
    glBegin(GL_TRIANGLES);
	
	for( i = 0; i <nc; i++){
        
        id0 = polys[ids[i]][0];
        id1 = polys[ids[i]][1];
        id2 = polys[ids[i]][2];

        
	glNormal3f(normals[id0].x, normals[id0].y, normals[id0].z);

        glVertex3f(MCMeshPoints[id0].x,MCMeshPoints[id0].y,MCMeshPoints[id0].z);

        glNormal3f(normals[id1].x, normals[id1].y, normals[id1].z);

        glVertex3f(MCMeshPoints[id1].x,MCMeshPoints[id1].y,MCMeshPoints[id1].z);

        glNormal3f(normals[id2].x, normals[id2].y, normals[id2].z);

        glVertex3f(MCMeshPoints[id2].x,MCMeshPoints[id2].y,MCMeshPoints[id2].z);

    }
    glEnd();
    //glDisable(GL_BLEND);

    if(edgeok)
    {
        glColor3f(0.4, 0.4, 0.4);



        for( i = 0; i < nc; i++){
            
            id0 = polys[ids[i]][0];
	    id1 = polys[ids[i]][1];
	    id2 = polys[ids[i]][2];

            glBegin(GL_LINE_LOOP);


            glVertex3f(MCMeshPoints[id0].x,MCMeshPoints[id0].y,MCMeshPoints[id0].z);



            glVertex3f(MCMeshPoints[id1].x,MCMeshPoints[id1].y,MCMeshPoints[id1].z);



            glVertex3f(MCMeshPoints[id2].x,MCMeshPoints[id2].y,MCMeshPoints[id2].z);
            glEnd();

        }


    }
  
  
};*/
 
 
void CalVertexNormals(const std::vector<Vector> *vrts, unsigned int nvrts, const std::vector< std::vector<unsigned int> > &tris, unsigned int ntris, std::vector<Vector> *nrms)
  {
          unsigned int nnrms = nvrts;
          nrms->resize(nnrms);


          for(unsigned int i = 0; i < nnrms; i++){
                  nrms->at(i).x = 0;
                  nrms->at(i).y = 0;
                  nrms->at(i).z = 0;
          }


          for(unsigned int i = 0; i < ntris; i++){
                  Vector vec1, vec2, normal;
                  int id0, id1, id2;
                  id0 = tris[i][0];
                  id1 = tris[i][1];
                  id2 = tris[i][2];

                  vec1 = vrts->at(id1)-vrts->at(id0);
                  vec2 = vrts->at(id2)-vrts->at(id0);
                  normal = vec1^vec2;

                  nrms->at(id0) += normal;
                  nrms->at(id1) += normal;
                  nrms->at(id2) += normal;
          }


          for(unsigned int i = 0; i < nnrms; i++){
                  nrms->at(i).normalize();
          }
  };

  
  
  

/*void DrawPoissonSurafce(Polyhedron_3 &p,TColorRGBA c,bool edgeok)
{
  
  std::vector<float>Mz;
  std::vector<unsigned int>ids;
  int nv,nc,i,j,id0,id1,id2;
  nv=p.size_of_vertices();
  nc=p.size_of_facets();
  float x,y,z,mz;
  Point_iterator pit;
  MCMeshPoints.resize(nv);
  Mz.resize(nc);
  ids.resize(nc);
  polys.resize(nc);
  i=0;
  for(pit=p.points_begin();pit!=p.points_end();++pit)
  {
    x=pit->x();y=pit->y();z=pit->z();
    Vector pi(x,y,z);
    MCMeshPoints[i]=pi;
    i++;
  }
  i=0;
  for(  Facet_iterator iif = p.facets_begin(); iif != p.facets_end(); ++iif){
        
        
	Halfedge_facet_circulator jh = iif->facet_begin();
	j=0;
	mz=0.0;
	 std::vector<unsigned int> t;
	t.resize(3);
        do {
	    t[j]=std::distance(p.vertices_begin(), jh->vertex());
	    mz+=MCMeshPoints[t[j]].z;
	    j++;
	    
        } while ( ++jh != iif->facet_begin());
	mz/=3.0;
	Mz[i]=mz;
	ids[i]=i;
	polys[i]=t;i++;
  }
  	
	CalVertexNormals(&MCMeshPoints,nv,polys,nc,&normals);
	

	
	for(i=0;i<nc-1;i++)
	{
	  for(j=i+1;j<nc;j++)
	  {
	    if (Mz[j]<Mz[i])
	    {
	      id0=ids[i];
	      ids[i]=ids[j];
	      ids[j]=id0;
	    }
	  }
	}
	
	glColor4f(c.R, c.G, c.B,c.A);
    
    glBegin(GL_TRIANGLES);
	
	for( i = 0; i <nc; i++){
        
        id0 = polys[ids[i]][0];
        id1 = polys[ids[i]][1];
        id2 = polys[ids[i]][2];

        
	glNormal3f(normals[id0].x, normals[id0].y, normals[id0].z);

        glVertex3f(MCMeshPoints[id0].x,MCMeshPoints[id0].y,MCMeshPoints[id0].z);

        glNormal3f(normals[id1].x, normals[id1].y, normals[id1].z);

        glVertex3f(MCMeshPoints[id1].x,MCMeshPoints[id1].y,MCMeshPoints[id1].z);

        glNormal3f(normals[id2].x, normals[id2].y, normals[id2].z);

        glVertex3f(MCMeshPoints[id2].x,MCMeshPoints[id2].y,MCMeshPoints[id2].z);

    }
    glEnd();
    //glDisable(GL_BLEND);

    if(edgeok)
    {
        glColor3f(0.4, 0.4, 0.4);



        for( i = 0; i < nc; i++){
            
            id0 = polys[ids[i]][0];
	    id1 = polys[ids[i]][1];
	    id2 = polys[ids[i]][2];

            glBegin(GL_LINE_LOOP);


            glVertex3f(MCMeshPoints[id0].x,MCMeshPoints[id0].y,MCMeshPoints[id0].z);



            glVertex3f(MCMeshPoints[id1].x,MCMeshPoints[id1].y,MCMeshPoints[id1].z);



            glVertex3f(MCMeshPoints[id2].x,MCMeshPoints[id2].y,MCMeshPoints[id2].z);
            glEnd();

        }


    }
  
  
};*/


/*void DrawScaleSurafce(Reconstruction &r,TColorRGBA c,bool edgeok)
{
  Points3D Plist;
  RecPoint_iterator pit;
  for(pit=r.points_begin();pit!=r.points_end();++pit)
    Plist.push_back(*pit);
  //std::copy(p.points_begin(), p.points_end(),Plist);
  
    glColor4f(c.R, c.G, c.B,c.A);
    
    glBegin(GL_TRIANGLES);

    for( Triple_iterator it = r.surface_begin(); it != r.surface_end(); ++it )
    {
      
      glVertex3f(Plist[it->at(0)].x(),Plist[it->at(0)].y(),Plist[it->at(0)].z());
      glVertex3f(Plist[it->at(1)].x(),Plist[it->at(1)].y(),Plist[it->at(1)].z());
      glVertex3f(Plist[it->at(2)].x(),Plist[it->at(2)].y(),Plist[it->at(2)].z());
    }
      
        
        

          
	    
        
	
        

    
    glEnd();
    

    if(edgeok)
    {
        glColor3f(0.4, 0.4, 0.4);


	for( Triple_iterator it = r.surface_begin(); it != r.surface_end(); ++it )
    {   
        //glNormal3f(normals.at(id0).x, normals.at(id0).y, normals.at(id0).z);
	
	glBegin(GL_LINE_LOOP);
	
         glVertex3f(Plist.at(it->at(0)).x(),Plist.at(it->at(0)).y(),Plist.at(it->at(0)).z());
      glVertex3f(Plist.at(it->at(1)).x(),Plist.at(it->at(1)).y(),Plist.at(it->at(1)).z());
      glVertex3f(Plist.at(it->at(2)).x(),Plist.at(it->at(2)).y(),Plist.at(it->at(2)).z());
        glEnd();

    }
	
	

        

    }
  
};*/

/*int ReconstructPoissonSurface(SPH &sph, bool povok)

{
  
   CGAL::Timer task_timer;
  
   // Poisson options
    FT sm_angle = 20.0; // Min triangle angle (degrees).
    FT sm_radius = 100; // Max triangle size w.r.t. point set average spacing.
    FT sm_distance = 0.25; // Approximation error w.r.t. point set average spacing.
    std::string solver_name = "eigen"; // Sparse linear solver name.
    double approximation_ratio = 0.01;
    double average_spacing_ratio = 5;

  
  task_timer.start();
  int nV =sph.particles.size();
       
     for(int i = 0; i< nV; i++){
       if(sph.particles[i].b)
       {
	 Point_3 p(sph.particles[i].x.x,sph.particles[i].x.y,sph.particles[i].x.z);
	 PointVectorPair pv;
	 pv.first = p;
	  pv_list.push_back(pv);
       }
     }
     
         // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 30; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals<Concurrency_tag>(pv_list.begin(), pv_list.end(),
                               CGAL::First_of_pair_property_map<PointVectorPair>(),
                               CGAL::Second_of_pair_property_map<PointVectorPair>(),
                               nb_neighbors);

    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    std::list<PointVectorPair>::iterator unoriented_points_begin =
      CGAL::mst_orient_normals(pv_list.begin(), pv_list.end(),
                                 CGAL::First_of_pair_property_map<PointVectorPair>(),
                                 CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                 nb_neighbors);

    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    pv_list.erase(unoriented_points_begin, pv_list.end());
    
    std::list<PointVectorPair>::iterator lit;
    
    for(lit= pv_list.begin();lit!=pv_list.end();++lit)
    {
      Point_with_normal pn(lit->first,lit->second);
      pn_list.push_back(pn);
    }
    pv_list.clear();
      
    Counter counter(std::distance(pn_list.begin(), pn_list.end()));
    InsertVisitor visitor(counter) ;
    

    //***************************************
    // Computes implicit function
    //***************************************

    std::cerr << "Computes Poisson implicit function...\n";

    // Creates implicit function from the read points.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    Poisson_reconstruction_function function(
                              pn_list.begin(), pn_list.end(),
                              CGAL::make_identity_property_map(PointNormalList::value_type()),
                              CGAL::make_normal_of_point_with_normal_pmap(PointNormalList::value_type()),
                              visitor);

    #ifdef CGAL_EIGEN3_ENABLED
    {
      if (solver_name == "eigen")
      {
        std::cerr << "Use Eigen 3\n";
        CGAL::Eigen_solver_traits<Eigen::ConjugateGradient<CGAL::Eigen_sparse_symmetric_matrix<double>::EigenType> > solver;
        if ( ! function.compute_implicit_function(solver, visitor, 
                                                approximation_ratio,
                                                average_spacing_ratio) )
        {
          std::cerr << "Error: cannot compute implicit function" << std::endl;
          return EXIT_FAILURE;
        }
      }    
      else
      {
        std::cerr << "Error: invalid solver " << solver_name << "\n";
        return EXIT_FAILURE;
      }
    }
    #else
    {
      std::cerr << "Error: invalid solver diabled " << solver_name << "\n";
      return EXIT_FAILURE;
    }
    #endif


    
    //***************************************
    // Surface mesh generation
    //***************************************

    std::cerr << "Surface meshing...\n";

    // Computes average spacing
    FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(pn_list.begin(), pn_list.end(),
                                                       6 /* knn = 1 ring );

    // Gets one point inside the implicit surface
    Point_3 inner_point = function.get_inner_point();
    FT inner_point_value = function(inner_point);
    if(inner_point_value >= 0.0)
    {
      std::cerr << "Error: unable to seed (" << inner_point_value << " at inner_point)" << std::endl;
      return EXIT_FAILURE;
    }

    // Gets implicit function's radius
    Sphere bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());

    // Defines the implicit surface: requires defining a
  	// conservative bounding sphere centered at inner point.
    FT sm_sphere_radius = 5.0 * radius;
    FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
                      Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                      sm_dichotomy_error/sm_sphere_radius);

    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error


    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh

    

    if(tr.number_of_vertices() == 0)
      return EXIT_FAILURE;

    // Converts to polyhedron
    Polyhedron_3 output_mesh;
    CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);

    //***************************************
    // Saves reconstructed surface mesh
    //***************************************

    // Prints total reconstruction duration
    std::cerr << "Total reconstruction: " << task_timer.time() << " seconds\n";
    
    
    
    //out << output_mesh;
    
    pn_list.clear();
  DrawPoissonSurafce(output_mesh,sph.particles[0].color,false);
  if(povok)
    sph.generatePovRayFile(MCMeshPoints,normals,polys);
  MCMeshPoints.clear();
  normals.clear();
  polys.clear();
}


void Write_reconstruction(const Reconstruction& reconstruct, std::string name)
{
  std::ofstream output(name.c_str());
  output << "OFF " << reconstruct.number_of_points() << " "
         << reconstruct.number_of_triangles() << " 0\n";

  std::copy(reconstruct.points_begin(),
            reconstruct.points_end(),
            std::ostream_iterator<Point_3>(output,"\n"));
  for( Triple_iterator it = reconstruct.surface_begin(); it != reconstruct.surface_end(); ++it )
      output << "3 " << *it << std::endl;
};

int ReconstructSurface(SPH &sph)
{
  
  
    
  
   int nV =sph.particles.size();
       
     for(int i = 0; i< nV; i++){
       if(sph.particles[i].b)
       {
	 RecPoint p(sph.particles[i].x.x,sph.particles[i].x.y,sph.particles[i].x.z);
	  pc.push_back(p);
       }
     }
     
  
    // Construct the mesh in a scale space.
  Reconstruction reconstruct( 30, 200 );
  
  reconstruct.reconstruct_surface( pc.begin(), pc.end(), 5,
				   false, // Do not separate shells
				   true // Force manifold output
				   );
  //Write_reconstruction(reconstruct,"out.off");
  
  // Advancing the scale-space further and visually compare the reconstruction result
    reconstruct.increase_scale( 1.5 );

     reconstruct.reconstruct_surface();
     //Write_reconstruction(reconstruct,"out2.off");
  DrawScaleSurafce(reconstruct,sph.particles[0].color,false);
  pc.clear();
}*/

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
 void ConstructGridDelCell(SPH &sph,float rho)
  {
    nbp=0;
      float rho2=2.0*rho;
      float rho2inv = 1/(rho2);
      float mx = (sph.bbox.bmax.x + sph.bbox.bmin.x)*0.5;
      float my = (sph.bbox.bmax.y + sph.bbox.bmin.y)*0.5;
      float mz = (sph.bbox.bmax.z + sph.bbox.bmin.z)*0.5;
      float maxdimx = (sph.bbox.bmax.x - sph.bbox.bmin.x);
      float maxdimy = (sph.bbox.bmax.y - sph.bbox.bmin.y);
      float maxdimz = (sph.bbox.bmax.z - sph.bbox.bmin.z);
      maxdimx *= 1.5; maxdimy *= 1.5; maxdimz *= 1.5;
      float bx,by,bz;
      bx=(mx-maxdimx*0.5); by=(my-maxdimy*0.5);bz=(mz-maxdimz*0.5);

      Del.nh =  (maxdimx)*rho2inv;
     /* if(Del.nh % 2 ==0)
          Del.nh+=4;
      else
          Del.nh+=3;*/
      Del.nv =  (maxdimy)*rho2inv;

   //std::cout << " nv = " << Del.nv << std::endl;
      /*if(Del.nv % 2 ==0)
          Del.nv+=4;
      else
          Del.nv+=3;*/
      
      Del.np =  (maxdimz)*rho2inv;

   //std::cout << " nv = " << Del.nv << std::endl;
      /*if(Del.np % 2 ==0)
          Del.np+=4;
      else
          Del.np+=3;*/


      Del.C[0] =mx;
      Del.C[1] =my;
      Del.C[2] =mz;
      
      if(Del.vc.size()>0)
      {
          Del.vc.clear();
      }

       Del.vc.resize((Del.nh+1)*(Del.nv+1)*(Del.np)+1);
      Del.Pmin[0]= bx;
      Del.Pmin[1]= by;
      Del.Pmin[2]= bz;

      for(int i = 0; i< Del.vc.size(); i++)
          if(Del.vc[i].listSSvrtx.size()>0)
                Del.vc[i].listSSvrtx.clear();
      //Del.Pmax[0]= Del.C[0]+rho*(Del.nh);
      //Del.Pmax[1]= Del.C[1]+rho*(Del.nv);


      //populating cells
      int nV =sph.particles.size();
       int cen[3];
     for(int i = 0; i< nV; i++){
	sph.particles[i].b=false;
       Vector xpf = sph.particles[i].x;

        cen[0] =  (xpf.x-Del.Pmin[0])*rho2inv;
        cen[1] =  (xpf.y-Del.Pmin[1])*rho2inv;
	cen[2] =  (xpf.z-Del.Pmin[2])*rho2inv;

        //std::cout << " nh =" << Del.nh << " cen[0] = " << cen[0] << " nv =" << Del.nv << " cen[1] = " << cen[1] << std::endl;

        //std::cout << " xpf.x =" << xpf.x << " Pmin[0] = " << Del.Pmin[0] << " xpf.y =" << xpf.y << " Pmin[1] = " << Del.Pmin[1] << std::endl;


        Del.vc[cen[0]+cen[1]*(Del.nh)+cen[2]*(Del.nh*Del.nv)].listSSvrtx.push_back(i);
        Del.vc[cen[0]+cen[1]*(Del.nh)+cen[2]*(Del.nh*Del.nv)].empty=false;


      }


  }

  bool testCavityCell(int i,int j,int k,float rho,GridC3D &cn, SPH &sph)
  {
    float x,y,z,dist,rho2=2.0*rho;
    int l,s,in;
    x=Del.Pmin[0]+i*rho2+rho;y=Del.Pmin[1]+j*rho2+rho; z=Del.Pmin[2]+k*rho2+rho;
    s = cn.listSSvrtx.size();
    for(l=0;l<s;l++)
    {
        in = cn.listSSvrtx[l];
      Vector xpf = sph.particles[in].x;
      dist = sqrt((x-xpf.x)*(x-xpf.x)+(y-xpf.y)*(y-xpf.y)+(z-xpf.z)*(z-xpf.z));
      if(dist<rho)
	return false;
    }
    
    return true;
  }
  
  void FindConvexHull(SPH &sph,of::ofList<int> &tmpV,std::vector<int> &cv,int k,int l,int m,float rho)
  {
    Polyhedron_3 poly;
      Points3D points;
      Point_iterator pit;

      float nv,invnv,x,y,z,xs,ys,zs,rho2=2.0*rho;
      float rho8inv = 1/(4.0*rho2);
      x=Del.Pmin[0]+k*rho2+rho;y=Del.Pmin[1]+l*rho2+rho; z=Del.Pmin[2]+m*rho2+rho;
      points.push_back(Point_3(0,0,0));

      for(int i=0; i<tmpV.size();i++)
      {
          Vector xpf = sph.particles[tmpV.pos(i)].x;
          xs=(xpf.x-x)*rho8inv;
          ys=(xpf.y-y)*rho8inv;
	  zs=(xpf.z-z)*rho8inv;
          nv =sqrt((xs)*(xs)+(ys)*(ys)+(zs)*(zs));
          invnv=1.0/pow(nv,1.3);
          points.push_back(Point_3(xs*invnv,ys*invnv,zs*invnv));
           //
      }
      if(points.size()>3)
      {
          CGAL::convex_hull_3( points.begin(), points.end(), poly );

          for(int i=1; i<points.size();i++)
          {

             for(pit = poly.points_begin();pit != poly.points_end();++pit)
             {
	       
                 if(points[i]==*pit)
                     cv.push_back(i-1);

             }
          }
      }
      points.clear();

  }
  
  int FindingBoundaryPoints(SPH &sph,float rho,bool gridok, bool sok, bool povok,bool boxok=false,int smoothok=3)
  {CGAL::Timer task_timer; 
      if(sok)
      {
	task_timer.start();
      }
      ConstructGridDelCell(sph,rho);
      float rho2=2.0*rho;
      float rho95 = rho*0.95;
      int i,j,k,ii,jj,kk,p;
            //std::vector<int> tmpV;
      of::ofList<int> tmpV;
      std::vector<int> corrTmp;
      std::vector<int> ch; //convexHullPoints
      
      
      freebPoints.clear();
      for(k=0;k<Del.np;k++)
	for(j=0;j<Del.nv;j++)
          for(i=0;i<Del.nh;i++)
          {
	    
              GridC3D &c = Del.vc[(k*Del.nv*Del.nh)+j*Del.nh+i];
              if ((c.empty)|| (testCavityCell(i,j,k,rho,c,sph)==true))
              {
                  tmpV.clear();
                  corrTmp.clear();
                  ch.clear();
                  int count=0;
                  corrTmp.push_back(count);
                  //finding centroid of the cell;
                 for(kk =max(0,k-1);kk<min(Del.np,k+2);kk++)
                  for (jj=max(0,j-1);jj<min(Del.nv,j+2);jj++)
                    for (ii=max(0,i-1);ii<min(Del.nh,i+2);ii++)
		      
                    {
                        GridC3D &cn = Del.vc[kk*(Del.nv*Del.nh)+jj*Del.nh+ii];
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
                 int vid;
                    
                 if(tmpV.size()>0) // this is a viewpoint cell
                 {
		     c.vp=true;
                     FindConvexHull(sph,tmpV,ch,i,j,k,rho);
                     for(p=0;p<ch.size();p++)
                     {
		       //bVertex v;
		       //v.id=tmpV.pos(ch[p]);
                         vid = tmpV.pos(ch[p]);
                       sph.particles[vid].b=true;
		       nbp++;
		         
                     }
                    
                     tmpV.clear();
                     ch.clear();
                     //return 0;
                  }

              }

          }
      
        if(gridok)
	  DrawBPGrid(rho2);
	
	if(sok)
	{
	  Mtime.push_back(task_timer.time());
	  sph.drawSurface(sph.particle(0).color,3,5,povok,boxok);
	}
        
	
	
          return 0;
}

  inline void DrawBPGrid(float rho2)
  {
      float x,y,z;
      int ix,iy,iz;
      glLineWidth(1);
      glColor3f(1.0, 1.0, 1.0);
      glBegin(GL_LINES);
      for(iz=0;iz<=Del.np;iz++)
      {
      for ( ix = 0; ix <=Del.nh; ix++) {
          x= Del.Pmin[0]+ix*rho2;
          z= Del.Pmin[2]+iz*rho2;
        glVertex3f(x, Del.Pmin[1],z);
        glVertex3f(x, Del.Pmin[1]+Del.nv*rho2,z);
      }
      for ( iy = 0; iy <=Del.nv; iy++) {
          y= Del.Pmin[1]+iy*rho2;
          z= Del.Pmin[2]+iz*rho2;
        glVertex3f(Del.Pmin[0], y,z);
        glVertex3f(Del.Pmin[0]+Del.nh*rho2, y,z);
      }
    }
      glEnd();
  }

  
};




#endif
