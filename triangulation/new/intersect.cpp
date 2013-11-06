#include "delaunay.h"
#include <map>


int is_cell_vertex(Cell_handle c, Point &pt) {
  for(int i = 0;i < 4;i++)
    if (pt == c->vertex(i)->point()) return i;
  return -1;
}

int check_edges(Point &a,Point &b,Point &c,Point &p,Point &q, int *coplanar) throw(const char *)
{
  CGAL::Orientation orient[3];
  orient[0] = CGAL::orientation(p,q,a,b);
  orient[1] = CGAL::orientation(p,q,b,c);
  orient[2] = CGAL::orientation(p,q,c,a);
  bool res[3];
  int nb_coplanar = 0;
  for (int i = 0;i < 3 ;i++) {
    if (orient[i] == CGAL::POSITIVE)
      res[i] = false;
    else {
      res[i] = true;
      if(orient[i] == CGAL::COPLANAR)
	coplanar[nb_coplanar++] = i;
    }
  }
  if(res[0] && res[1] && res[2])
    return nb_coplanar;
  else
    return -1;
    //  bool res = pqab != CGAL::POSITIVE && pqbc != CGAL::POSITIVE && pqca != CGAL::POSITIVE;
    //  return res;
}
// check intersections of seg with edges of facet
// return nb of intersections
// intersections : index in facet_vertex_order of 1st point of edge (2nd is at index+1)
int check_intersections(Segment &seg,Facet &f,int *intersections){
  int nbc = 0;
  int j = 4 * f.second;
  Cell_handle c = f.first;
  Segment segs[3];
  for(int i = 0;i < 3;i++,j++) {
    int i1 = facet_vertex_order[j];
    int i2 = facet_vertex_order[j + 1];
    Point p1 = c->vertex(i1)->point();
    Point p2 = c->vertex(i2)->point();
    segs[i] = Segment(c->vertex(i1)->point(),c->vertex(i2)->point());
  }
  for (int i = 0;i < 3;i++) {
    if(CGAL::do_intersect(seg,segs[i])) {
      j = 4 * f.second + i;
      intersections[nbc++] = j;
      if(nbc >= 2) break;
    }
  }
  return nbc;
}

int my_intersect(Triangle &t, Segment  &s,int *coplanar)
{
  Point a = t[0];
  Point b = t[1];
  Point c = t[2];
  Point p = s.point(0);
  Point q = s.point(1);

  CGAL::Orientation abcp = CGAL::orientation(a,b,c,p);
  CGAL::Orientation abcq = CGAL::orientation(a,b,c,q);

  switch ( abcp ) {
  case CGAL::POSITIVE:
    switch ( abcq ) {
    case CGAL::POSITIVE:
      // the segment lies in the positive open halfspaces defined by the
      // triangle's supporting plane
      return -1;

    case CGAL::NEGATIVE:
      return check_edges(a,b,c,p,q,coplanar);
      /*
      return CGAL::orientation(p,q,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,c,a) != CGAL::POSITIVE;
      */
    case CGAL::COPLANAR:
      // q belongs to the triangle's supporting plane
      // p sees the triangle in counterclockwise order
      return check_edges(a,b,c,p,q,coplanar);
      /* return CGAL::orientation(p,q,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,c,a) != CGAL::POSITIVE;
      */
    default: // should not happen.
      CGAL_assertion(false);
      return -1;
    }
  case CGAL::NEGATIVE:
    switch ( abcq ) {
    case CGAL::POSITIVE: 
      // q sees the triangle in counterclockwise order
	return check_edges(a,b,c,q,p,coplanar);
      /* return CGAL::orientation(q,p,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,c,a) != CGAL::POSITIVE;
      */
	break;
    case CGAL::NEGATIVE:

      // the segment lies in the negative open halfspaces defined by the
      // triangle's supporting plane
      return -1;
      
    case CGAL::COPLANAR:
      // q belongs to the triangle's supporting plane
      // p sees the triangle in clockwise order
      return check_edges(a,b,c,q,p,coplanar);
      /* return CGAL::orientation(q,p,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,c,a) != CGAL::POSITIVE;
      */
    default: // should not happen.
      CGAL_assertion(false);
      return -1;
    }
  case CGAL::COPLANAR: // p belongs to the triangle's supporting plane
    switch ( abcq ) {
    case CGAL::POSITIVE:
      // q sees the triangle in counterclockwise order
      return check_edges(a,b,c,q,p,coplanar);
      /* return CGAL::orientation(q,p,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(q,p,c,a) != CGAL::POSITIVE;
      */
    case CGAL::NEGATIVE:
      // q sees the triangle in clockwise order
      return check_edges(a,b,c,q,p,coplanar);
      /* return CGAL::orientation(p,q,a,b) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,b,c) != CGAL::POSITIVE
	&& CGAL::orientation(p,q,c,a) != CGAL::POSITIVE;
      */
    case CGAL::COPLANAR:
      // the segment is coplanar with the triangle's supporting plane
      // as we know that it is inside the tetrahedron it intersects the face
      //      return CGAL::do_intersect_coplanar(t,s,k);
      //coplanar[0] = coplanar[1] = coplanar[2] = 3;
      return 3;

    default: // should not happen.
      CGAL_assertion(false);
      return -1;
    }
  default: // should not happen.
    CGAL_assertion(false);
    return -1;
  }
}
static char msg[512];
/* Find which facet is intersected and return next facets to check
input : seg = ray (cam->visible-point)
        in_inter : status of incident point on ray
	facets : vector of facets to check
output:  out_inter status of exit point
         marked_cells : intersected tetra is added to this vector
return : pointer to vector of facets to check at next step
 */
std::vector<Facet> *intersect(int *err,int *nb_tested,Segment &seg,Intersect &in_inter,Intersect &out_inter,std::vector<Facet> *facets,Delaunay &Tr,std::vector<Cell_handle> &marked_cells,Segment &newseg,TrStats *stats,CGAL::Geomview_stream &gv) throw(const char *){
  *err = 0;
  Point pt2 = seg.point(1);
  Point pt1 = seg.point(0);
  //  printf("INTER cam %d pt %d\n",icam,ipt);
  if(debug_stop > 1) std::cout << "INTERSECT " << facets->size() << " facets,SEG " << pt1 << ", " << pt2 << std::endl;
  bool it_segment = false;
  const Segment *int_seg;
  Facet out_facet;
  int inter_found = false;
  int nb_inter(0);
  std::vector<Facet> *new_facets = new std::vector<Facet>;
  Cell_handle cur_cell;
  CGAL::Triple<Cell_handle,Vertex_handle,Vertex_handle> out_info;
  int facet_index = 0;
  int coplanar[3];
  int nb_coplanar;
  //  std::vector<Facet> xfacets;
  for(std::vector<Facet>::iterator it0 = facets->begin();it0 != facets->end();it0++,facet_index++) {
    Cell_handle c = it0->first;
    CGAL_assertion(! Tr.is_infinite(c));
    //if(Tr.is_infinite(c)) continue;
    Triangle t = Tr.triangle(*it0);
    (*nb_tested)++;
    int nbc;
    nbc = my_intersect(t,seg,coplanar);
    if (nbc >= 0) {
      nb_coplanar = nbc;
      inter_found = true;
      cur_cell = c;
      out_facet = *it0; 
      break;
    }

  }
  if(! inter_found) {// end
    if(debug_vis > 1)
      draw_seg(gv,seg);
    if(debug_stop > 1) {
      std::cout << "BAD END " << marked_cells.size() << std::endl;
      prompt("...");
    }

    delete facets;
    *err = -1;
    return new_facets;
  }
  Edge e1, e2;
  marked_cells.push_back(cur_cell);
  stats->nb_tot_intersect++;
  bool target_reached = false;
  if(nb_coplanar == 0) {
    out_inter.int_type = INT_FACET;
  } else { // coplanar with 1 edge = intersect edge
    int j = 4 * out_facet.second;
    int i1 = j + coplanar[0];
    int ix1 = facet_vertex_order[i1];
    int ix2 = facet_vertex_order[i1+1];
    if (nb_coplanar == 1) {
      out_inter.int_type = INT_EDGE;
      out_inter.v1 = cur_cell->vertex(ix1);
      out_inter.v2 = cur_cell->vertex(ix2);
    } else { // ray coplanar avec 2 egdes = hit a vertex
      // coplanar avec 3 egdes = tangent = impossible ?
      // find vertex index
      int i2 = j + coplanar[1];
      int i = -1;
      if (facet_vertex_order[i1] == facet_vertex_order[i2] || facet_vertex_order[i1] == facet_vertex_order[i2+1])
	i = facet_vertex_order[i1];
      else if (facet_vertex_order[i1+1] == facet_vertex_order[i2] || facet_vertex_order[i1+1] == facet_vertex_order[i2+1])
	i = facet_vertex_order[i1 + 1];
      if (i < 0)
	throw("2 edges intersections without common vertex");
      out_inter.int_type = INT_VERTEX;
      out_inter.v1 = cur_cell->vertex(i);
      Point p = cur_cell->vertex(i)->point();
      if (cur_cell->vertex(i)->point() == pt2)
	target_reached = true;

    }
  }
  //if(debug_stop > 1) std::cout << "TR " << t << std::endl;
  if(debug_stop > 3) {
    std::cout << "MARKED " << marked_cells.size() << std::endl;
    prompt("...");
  }
  if(debug_vis > 3) {
    draw_all(gv,Tr);
    draw_tetra(Tr,marked_cells,gv);
  }
  if(target_reached) {
    delete facets;
    out_inter.intersected_facet = facet_index;
    return new_facets;
  }

    // Fin du marquage
    // choix des prochaines faces
  int j = out_facet.second;
  //  Cell_handle c = out_facet.first;
  out_inter.intersected_facet = facet_index;
  if (out_inter.int_type == INT_FACET) {
    Facet f = Tr.mirror_facet(out_facet);
    Cell_handle c2 = f.first;
    for(int i = 0;i < 4;i++) {
      if(i != f.second)
	new_facets->push_back(Facet(c2,i));
    }
  } else {
    std::map<Cell_handle,bool> used_cells;
    for(std::vector<Facet>::iterator it = facets->begin();it != facets->end();it++) {
      used_cells[it->first] = true;
    }
    if(out_inter.int_type == INT_EDGE) { // les faces opposees a l'arete dans les tetras autour
      stats->nb_int_edge++;
      Vertex_handle v1 = out_inter.v1;
      Vertex_handle v2 = out_inter.v2;
      Edge out_edge(cur_cell,cur_cell->index(v1),cur_cell->index(v2));
      Delaunay::Cell_circulator ifc0 = Tr.incident_cells(out_edge);
      Delaunay::Cell_circulator ifc = ifc0;
      do {
	Cell_handle c = ifc++;
	if(Tr.is_infinite(c)) continue;
	if(c == cur_cell) continue;
	if(used_cells.find(c) != used_cells.end()) continue;
	new_facets->push_back(Facet(c,c->index(v1)));
	new_facets->push_back(Facet(c,c->index(v2)));
      } while (ifc != ifc0);
    } else { // out on vertex
      stats->nb_intv++;
      Vertex_handle v = out_inter.v1;
      std::vector<Cell_handle> cells;
      Tr.finite_incident_cells(v,std::back_inserter(cells));
      for (std::vector<Cell_handle>::iterator it = cells.begin();it != cells.end();it++) {
	if(used_cells.find(*it) != used_cells.end()) continue;
	int ip = (*it)->index(v);
	new_facets->push_back(Facet(*it,ip));
      }
      
    }
  }
  delete facets;
  return new_facets;
}
