#include "Depth.h"
DepthStats::DepthStats(TriMesh *mesh):_mesh(mesh){


}


void DepthStats::getPlaneFits(vector<Plane3D> &planes, vector<TriMesh::BBox> &bounds,int widthSplits,int heightSplits){
  int nv = _mesh->vertices.size();
  _mesh->need_bbox();
  vec v=_mesh->bbox.size();
  cout << v << endl;
  cout << _mesh->bbox.min << " " << _mesh->bbox.max << endl;

  double xstep = v[0] /widthSplits;
  double ystep = v[1]/ heightSplits;
  
 

  for(int wS=0; wS < widthSplits; wS++){
    for(int hS=0; hS <heightSplits; hS++){
      TriMesh::BBox stepbbox;
     Plane3D plane3D,plane3D_2;
     RansacPlane m_RansacPlane;
     std::vector<bool> inliers;
     unsigned int nInlierCount = 0;
     double dbModelScore;
     m_RansacPlane.setMaxIterationNumber(25);
     m_RansacPlane.setInlierDistanceThreshold(0.001);
     plane3D.clear();
     std::vector<Point3D> m_pPoints;
     std::vector<int> pointIndex;
     int count=0;
     stepbbox.min[0] = _mesh->bbox.min[0] + (xstep * wS);
     stepbbox.min[1]= _mesh->bbox.min[1] + (ystep * hS);
     
     stepbbox.max[0] = _mesh->bbox.min[0] + (xstep*(wS+1));
     stepbbox.max[1]= _mesh->bbox.min[1] + (ystep*(hS+1));
     //   cout << stepbbox.min << stepbbox.max<< " " <<(xstep * wS) << " " <<(ystep * hS) <<  endl;
     for (int i = 0; i < nv; i++){
       if (_mesh->vertices[i][0] >= stepbbox.min[0] &&
	   _mesh->vertices[i][0] <= stepbbox.max[0] &&
	   _mesh->vertices[i][1] >= stepbbox.min[1] &&
	   _mesh->vertices[i][1] <= stepbbox.max[1] ){
	 Point3D pt(_mesh->vertices[i][0],
		    _mesh->vertices[i][1],
		    _mesh->vertices[i][2]);
	 m_pPoints.push_back(pt);
	 pointIndex.push_back(count++);
       }
     }
     if(!pointIndex.size())
       continue;
     // --- Ransac ---------
     // --------------------
     
     nInlierCount = 0;
     inliers.clear();
     m_RansacPlane.setData(&m_pPoints, &pointIndex);
     int nReturn = m_RansacPlane.Algorithm(0.5, plane3D, inliers,
					   nInlierCount, dbModelScore);
     if (nReturn <= 0) // No model or error
       continue;
     
     
     //cout << std::endl << std::endl
     // << "New cell : return of ransac " << nReturn << std::endl;
     if (nInlierCount==0) {
       std::cout << std::endl << std::endl
		 << "no plane was found " << std::endl;
       continue;
     }
     else {
       // std::cout << "N# of inliers " << nInlierCount
       //     << "  N# of points " << pointIndex.size()
       //     << std::endl;
     }
     
     //std::cout << "Plane Score " << dbModelScore << std::endl;
     
     plane3D.info();
     // --- Min Least Square  -------
     // -----------------------------
     m_RansacPlane.bestPlaneMinLeastSquares(pointIndex, plane3D_2);
     double dbScore_2 = 0;
     int nCount = 0;
     double d;
     for (unsigned int k=0; k< nInlierCount; k++) {
       d = plane3D_2.signedDistance(m_pPoints[pointIndex[k]]);
       dbScore_2 += fabs(d);
       
       if (fabs(d) < m_RansacPlane.getInlierDistanceThreshold())
	 nCount++;
     }
     dbScore_2 /= nInlierCount;
     if (dbScore_2 < dbModelScore) {
       // MinLeastSquare Plane is better
       dbModelScore = dbScore_2;
       plane3D = plane3D_2;
       //std::cout << "--------------------------------MLQE" << std::endl;
     }

     
     planes.push_back(plane3D);
     bounds.push_back(stepbbox);
     
    }
  }
}
