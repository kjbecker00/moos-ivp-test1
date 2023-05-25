#include "grid.hpp"

int main (int ac, char* av[]) {
    Point pin_point(0,0);
    SearchGrid grid(pin_point,100.0,50.0,0.25,0);

    for(int i = 0; i <= 100; i+=8) {
        grid.reduce_search_space(i,i/2,10,255);
    }
    
    Point p = grid.get_centroid();
    Point p1(p.m_x-5,p.m_y);
    Point p2 = grid.get_local_centroid(p1, 40);
    printf("Center of uncertainty: %s\n", p.repr().c_str());
    double percent_unknown = grid.get_remaining_uncertainty()*100;
    printf("Percent Unknown: %0.2f\n", percent_unknown);
    std::string fname = "test.pgm";
    printf("Local Center of uncertainty: %s\n", p2.repr().c_str());
    printf("Saving grid\n");
    grid.save_grid(fname);
    std::vector<Cluster> clusters = grid.get_clusters();

    printf("Clusters: %d\n", clusters.size());
    for(auto cluster: clusters) {
        printf("\tNum elements: %d\n", cluster.elements.size());
        printf("\tEdge elements: %d\n", cluster.edges_elements.size());
        printf("\tCenter (r,c): {%d,%d}\n", cluster.centroid.first, cluster.centroid.second);
    }
}   