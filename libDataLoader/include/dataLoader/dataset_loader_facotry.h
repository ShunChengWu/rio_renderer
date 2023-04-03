#pragma once
#include <Logging.h>
#include "dataset3RScan.h"
#include "datasetScanNet.h"
#include "dataloader_3rscan.h"
#include "dataloader_scannet.h"
#include "dataloader_DFGIMG.h"
#include "dataloader_TUMRGBD.h"
#include "dataloader_ICLNUIM.h"
#include "dataloader_img.h"
namespace PSLAM {
    struct  DataLoaderFactory {
        static DatasetLoader_base *Make(
                const std::string &pth, INPUTE_TYPE inputeType = DATASET_DETECT) {
            DatasetLoader_base *output = nullptr;
            // detect datatype by checking file name
            if(inputeType == DATASET_DETECT) {
                SCLOG(INFO) << "detect data type: ";
                if (pth.find(".sens") != std::string::npos || pth.find("scene") != std::string::npos) {
                    inputeType = DATASET_SCANNET;
                    SCLOG(INFO) << "ScanNet";
                } else if (pth.find("DFG") != std::string::npos) {
                    inputeType = DATASET_DFGIMG;
                    SCLOG(INFO) << "DFGIMG";
                } else if (pth.find("rgbd_dataset_freiburg") != std::string::npos) {
                    inputeType = DATASET_TUMRGBD;
                    SCLOG(INFO) << "TUMRGBD";
                } else if (pth.find("ICL_NUIM") != std::string::npos) {
                    inputeType = DATASET_ICLNUIM;
                    SCLOG(INFO) << "ICLNUIM";
                } else {
                    inputeType = DATASET_3RSCAN;
                    SCLOG(INFO) << "3RScan";
                }
            }

            // Create dataloader
            switch (inputeType) {
                case DATASET_3RSCAN: {
                    auto path = pth.back() == '/'? pth : pth+"/";
                    auto database = std::make_shared<Scan3RDataset>(inputeType, path);
                    output = new DatasetLoader_3RScan(database);
                    break;
                }
                case DATASET_SCANNET: {
                    auto database = std::make_shared<ScanNetDataset>(inputeType, pth);
                    output = new DatasetLoader_ScanNet(database);
                    break;
                }
                case CAM_REALSENSE: {
                    throw std::runtime_error("dataset type: CAM_REALSENSE not implemented");
//                    auto database = std::make_shared<ScanNetDataset>(datasetType, pth);
//                    output = new DatasetLoader_ScanNet(database);
                    break;
                }
                case DATASET_DFGIMG: {
                    auto database = std::make_shared<DFGIMGDataset>(inputeType, pth);
                    output = new DataLoader_DFGIMG(database);
                    break;
                }
                case DATASET_TUMRGBD: {
                    auto database = std::make_shared<DFGIMGDataset>(inputeType, pth);
                    output = new DatasetLoader_TUMRGBD(database);
                    break;
                }
                case DATASET_ICLNUIM: {
                    auto database = std::make_shared<DFGIMGDataset>(inputeType, pth);
                    output = new DatasetLoader_ICLNUIM(database);
                    break;
                };
                case DATASET_IMGSEQUENCE: {
                    auto database = std::make_shared<Scan3RDataset>(inputeType, pth);
                    output = new DatasetLoader_IMG(database);
                    break;
                }
                case DATASET_DETECT:
                    break;

            }
            return output;
        }
    };
}