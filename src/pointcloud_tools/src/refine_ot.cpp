#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void refine(std::string file)
{
    octomap::AbstractOcTree* tmp = octomap::AbstractOcTree::read(file);
    octomap::OcTree* ot = dynamic_cast<octomap::OcTree*>(tmp);

    std::cerr << "loaded " << file << std::endl; 
    std::cerr << ot->getClampingThresMin() << " " << ot->getClampingThresMax() << " " << ot->getOccupancyThres() << std::endl;

    double x, y, ground;
    ot->getMetricMin(x, y, ground);

    // //ot->setClampingThresMin(0.5);
    // //ot->updateInnerOccupancy();

    // double avg_occ = 0.0, min_occ = 1.0, max_occ = 0.0;
    // int count =0;
    // for(octomap::OcTree::tree_iterator it = ot->begin_tree(),
    //        end=ot->end_tree(); it!=end; ++it)
    // {
    //   //manipulate node, e.g.:
    //   std::cerr << "Node center: " << it.getCoordinate() << std::endl;
    //   //std::cerr << "Node size: " << it.getSize() << std::endl;
    //   //std::cerr << "Node value: " << it->getOccupancy()  <<  " " << ot->isNodeOccupied(*it) << std::endl;
    //     count++;
    //     double occ = it->getOccupancy();
    //     avg_occ += occ;
    //     min_occ = std::min(min_occ, occ);
    //     max_occ = std::max(max_occ, occ);

    // }
    // avg_occ /= count;
    // std::cerr << "occ " << avg_occ << " " << min_occ << " " << max_occ << std::endl;

    std::cerr << "inflate low occupancy cells..." << std::endl; 
    std::vector<octomap::OcTreeKey> unoccupied, occupied;
    for (octomap::OcTree::iterator it=ot->begin(ot->getTreeDepth());
        it != ot->end(); it++){

        double occupancy = it->getOccupancy();
        if (occupancy < 0.9 && occupancy > 0.4) {

           double x = it.getX(), y = it.getY(), z = it.getZ(); 
           octomap::point3d qry(x,y,z);

           if ( abs(qry.z() - ground) > 0.4) continue;

            double bbox[28][3] = 
        { 
            {0.1,0.1,0.1}, 
            {-0.1,-0.1,-0.1},
            {0.1, -0.1, -0.1},
            {0.1, 0.1, -0.1},
            {-0.1, -0.1, 0.1},
            {-0.1, 0.1, 0.1},
            {-0.1, 0.1, -0.1},
            {0.1, -0.1, 0.1},

            {-0.1,0.,0.}, 
            {0.,-0.1,0.},
            {0., 0., -0.1},

            {0.1,0.,0.}, 
            {0.,0.1,0.},
            {0., 0., 0.1},

            {0.1,0.1,0.}, 
            {0.,0.1,0.1},
            {0.1, 0., 0.1},

            {-0.1,-0.1,0.}, 
            {0.,-0.1,-0.1},
            {-0.1, 0., -0.1},

            {0.2,0.2,0.2}, 
            {-0.2,-0.2,-0.2},
            {0.2, -0.2, -0.2},
            {0.2, 0.2, -0.2},
            {-0.2, -0.2, 0.2},
            {-0.2, 0.2, 0.2},
            {-0.2, 0.2, -0.2},
            {0.2, -0.2, 0.2},

        };

            int count=0;
            for (int i=0; i<28; i++) {
                if (count > 20) break;

                octomap::point3d tmp(qry.x() + bbox[i][0],
                    qry.y() + bbox[i][1],
                    qry.z() + bbox[i][2]);

                octomap::OcTreeNode* node = ot->search(tmp);
                if (node != NULL) {
                    if (node->getOccupancy() < 0.4) {

                        //octomap::OcTreeKey key;
                        //if (ot->coordToKeyChecked(qry, key)) {
                            //unoccupied.push_back(key);
                            count++;
                            count++;
                            // if (occupancy < 0.1) {
                            //     count++;
                            //     ot->updateNode(key, false);
                            // }
                        //}

                    }
                }
                else //if ( abs(qry.z() - ground) < 0.2)
                {
                    //octomap::OcTreeKey key;
                    //if (ot->coordToKeyChecked(qry, key)) {
                        // ot->updateNode(key, false);
                        //unoccupied.push_back(key);
                        count++;
                    //}
                }
            }

            if (count > 8) {
                octomap::OcTreeKey key;
                if (ot->coordToKeyChecked(qry, key)){
                    unoccupied.push_back(key);
                    if (count > 10)
                        unoccupied.push_back(key);
                    if (count > 15)
                        unoccupied.push_back(key);
                }
            }
        }
    }
    for (std::vector<octomap::OcTreeKey>::iterator it=unoccupied.begin();
        it != unoccupied.end();
        it++) {
        ot->updateNode(*it, false);
    }

     std::cerr << "inflate high occupancy cells..." << std::endl; 

        for (octomap::OcTree::iterator it=ot->begin(ot->getTreeDepth());
        it != ot->end(); it++){

        double occupancy = it->getOccupancy();
        if (occupancy < 0.5) {

           double x = it.getX(), y = it.getY(), z = it.getZ(); 
           octomap::point3d qry(x,y,z);

            double bbox[28][3] = 
        { 
            {0.1,0.1,0.1}, 
            {-0.1,-0.1,-0.1},
            {0.1, -0.1, -0.1},
            {0.1, 0.1, -0.1},
            {-0.1, -0.1, 0.1},
            {-0.1, 0.1, 0.1},
            {-0.1, 0.1, -0.1},
            {0.1, -0.1, 0.1},

            {-0.1,0.,0.}, 
            {0.,-0.1,0.},
            {0., 0., -0.1},

            {0.1,0.,0.}, 
            {0.,0.1,0.},
            {0., 0., 0.1},

            {0.1,0.1,0.}, 
            {0.,0.1,0.1},
            {0.1, 0., 0.1},

            {-0.1,-0.1,0.}, 
            {0.,-0.1,-0.1},
            {-0.1, 0., -0.1},

            {0.2,0.2,0.2}, 
            {-0.2,-0.2,-0.2},
            {0.2, -0.2, -0.2},
            {0.2, 0.2, -0.2},
            {-0.2, -0.2, 0.2},
            {-0.2, 0.2, 0.2},
            {-0.2, 0.2, -0.2},
            {0.2, -0.2, 0.2},

        };

            int count=0;
            for (int i=0; i<28; i++) {
                if (count > 20) break;

                octomap::point3d tmp(qry.x() + bbox[i][0],
                    qry.y() + bbox[i][1],
                    qry.z() + bbox[i][2]);

                octomap::OcTreeNode* node = ot->search(tmp);
                if (node != NULL) {
                    if (node->getOccupancy() > 0.85) {
                        count++;
                        if (node->getOccupancy() > 0.95)
                           count++;
                   }
                }
            }

            if (count > 5) {
                octomap::OcTreeKey key;
                if (ot->coordToKeyChecked(qry, key)){
                    unoccupied.push_back(key);
                    if (count > 12)
                        occupied.push_back(key);
                    if (count > 20)
                        occupied.push_back(key);
                }
            }
        }
    }
    for (std::vector<octomap::OcTreeKey>::iterator it=occupied.begin();
        it != occupied.end();
        it++) {
        ot->updateNode(*it, true);
    }

    // for (octomap::OcTree::iterator it=ot->begin(ot->getTreeDepth());
    //     it != ot->end(); it++){

    //     double occupancy = it->getOccupancy();
    //     if (occupancy > 0.9) {

    //        double x = it.getX(), y = it.getY(), z = it.getZ(); 
    //        octomap::point3d qry(x,y,z);

    //         double bbox[20][3] = 
    //     { 
    //         {0.1,0.1,0.1}, 
    //         {-0.1,-0.1,-0.1},
    //         {0.1, -0.1, -0.1},
    //         {0.1, 0.1, -0.1},
    //         {-0.1, -0.1, 0.1},
    //         {-0.1, 0.1, 0.1},
    //         {-0.1, 0.1, -0.1},
    //         {0.1, -0.1, 0.1},

    //         {-0.1,0.,0.}, 
    //         {0.,-0.1,0.},
    //         {0., 0., -0.1},

    //         {0.1,0.,0.}, 
    //         {0.,0.1,0.},
    //         {0., 0., 0.1},

    //         {0.1,0.1,0.}, 
    //         {0.,0.1,0.1},
    //         {0.1, 0., 0.1},

    //         {-0.1,-0.1,0.}, 
    //         {0.,-0.1,-0.1},
    //         {-0.1, 0., -0.1} 
    //     };

    //         for (int i=0; i<20; i++) {
    //             octomap::point3d tmp(qry.x() + bbox[i][0],
    //                 qry.y() + bbox[i][1],
    //                 qry.z() + bbox[i][2]);

    //             octomap::OcTreeKey key;
    //             if (ot->coordToKeyChecked(tmp, key)) {
    //                 //ot->updateNode(key, true);
    //                 occupied.push_back(key);
    //                 if (occupancy > 0.95)
    //                     //ot->updateNode(key, true);
    //                     occupied.push_back(key);
    //             }
    //         }
    //     }
    // }

    // for (std::vector<octomap::OcTreeKey>::iterator it=occupied.begin();
    //     it != occupied.end();
    //     it++) {
    //     ot->updateNode(*it, true);
    // }

    // std::cerr << "inflate low occupancy cells..." << std::endl; 

    // for (octomap::OcTree::iterator it=ot->begin(ot->getTreeDepth());
    //     it != ot->end(); it++){

    //     double occupancy = it->getOccupancy();
    //     if (occupancy < 0.2) {
           
    //        double x = it.getX(), y = it.getY(), z = it.getZ(); 
    //        octomap::point3d qry(x,y,z);

    //         double bbox[24][3] = 
    //                 { {0.1,0.1,0.1}, 
    //                 {-0.1,-0.1,-0.1},
    //                 {0.1, -0.1, -0.1},
    //                 {0.1, 0.1, -0.1},
    //                 {-0.1, -0.1, 0.1},
    //                 {-0.1, 0.1, 0.1},
    //                 {-0.1, 0.1, -0.1},
    //                 {0.1, -0.1, 0.1},

    //                 {0.2,0.2,0.2}, 
    //                 {-0.2,-0.2,-0.2},
    //                 {0.2, -0.2, -0.2},
    //                 {0.2, 0.2, -0.2},
    //                 {-0.2, -0.2, 0.2},
    //                 {-0.2, 0.2, 0.2},
    //                 {-0.2, 0.2, -0.2},
    //                 {0.2, -0.2, 0.2}};

    //         for (int i=0; i<8; i++) {
    //             octomap::point3d tmp(qry.x() + bbox[i][0],
    //                 qry.y() + bbox[i][1],
    //                 qry.z() + bbox[i][2]);

    //             octomap::OcTreeKey key;
    //             if (ot->coordToKeyChecked(tmp, key)) {
    //                 ot->updateNode(key, false);
    //                 if (occupancy < 0.1)
    //                     ot->updateNode(key, false);
    //             }
    //         }

    //         if (occupancy < 0.1) {
    //             for (int i=8; i<16; i++) {
    //             octomap::point3d tmp(qry.x() + bbox[i][0],
    //                 qry.y() + bbox[i][1],
    //                 qry.z() + bbox[i][2]);

    //             octomap::OcTreeKey key;
    //             if (ot->coordToKeyChecked(tmp, key)) 
    //                 ot->updateNode(key, false);

    //                 if (occupancy < 0.6)
    //                     ot->updateNode(key, false);
    //             }
    //         }

    //         // if (occupancy < 0.1) {
    //         //     for (int i=16; i<24; i++) {
    //         //     octomap::point3d tmp(qry.x() + bbox[i][0],
    //         //         qry.y() + bbox[i][1],
    //         //         qry.z() + bbox[i][2]);

    //         //     octomap::OcTreeKey key;
    //         //     if (ot->coordToKeyChecked(tmp, key)) 
    //         //         ot->updateNode(key, false);
    //         //     }
    //         // }
    //     }
    // }

    //ot->setResolution(0.4);
    //ot->toMaxLikelihood();
    ot->prune();

    std::string out_file_name = file.substr(0, file.length() - 3) + "_refined.ot";
    std::cerr << "writing to file: " << out_file_name << std::endl;
    ot->write(out_file_name);

    delete ot;
}

int main(int argc, char** argv) 
{
    if (argc < 2){
        std::cerr << "please provide ot file" << std::endl;
        return 1;
    }

    std::string file = argv[1];

    refine(file);

    return 0;
}