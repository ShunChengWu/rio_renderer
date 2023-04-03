//
// Created by sc on 2/27/21.
//

#ifndef GRAPHSLAM_DATASET_DFGIMG_H
#define GRAPHSLAM_DATASET_DFGIMG_H

#include "dataset_base.h"

namespace PSLAM {
    class DFGIMGDataset : public DatasetDefinitionBase {
    public:
        DFGIMGDataset(INPUTE_TYPE type, const std::string &path)  {
            datasetType = type;
            folder = path;
            frame_index_counter = 1;
            number_length = 1;
            prefix_pose = "/frame-";
            prefix_depth = "/frame-";
            prefix_rgb = "/frame-";

            suffix_depth = ".depth.pgm";
            suffix_rgb = ".color.jpg";
            suffix_pose = ".pose.txt";
            if (rotate_pose_img) {
                //suffix_depth = ".rescan.rendered.depth.png";
                suffix_depth = ".rendered.depth.png";
//        suffix_rgb = ".rendered.color.jpg";
                suffix_pose = ".align.pose.txt";
            }

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 6;

            logNames = {
                    "lola_vision", "Schachbrett"
            };
        }

        std::vector<std::string> logNames;
    };
}

#endif //GRAPHSLAM_DATASET_DFGIMG_H
