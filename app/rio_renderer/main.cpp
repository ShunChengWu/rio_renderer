//
// Created by sc on 2/1/22.
//
#define STB_IMAGE_IMPLEMENTATION
#include <GUI3D/GUI3D.h>
#include "Logging.h"
#include <dataLoader/dataset_loader_facotry.h>
#include "parser.hpp"
#include "cameraParameter.h"
#include "renderer/RendererType.h"
#include "renderer/RendererFactory.h"
#include "PathTool.hpp"

struct Config {
    std::string pth_in;
    bool verbose=false;
};

void ParseCommandLine(int argc, char **argv, Config &params) {
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_in", &params.pth_in), "The input path to the sequence folder", true);
    parser.addOption(pkgcname("verbose", &params.verbose), "verbose.");
    if(parser.showMsg(params.verbose)<0) exit(-1);
}

class GUI: SC::GUI3D {
public:
    GUI(int width, int height, const std::string &path): SC::GUI3D("main",width,height,false) {
        SetRender(PSLAM::INPUTE_TYPE::DATASET_3RSCAN, width,height,path,true);
    }

    auto render(const Eigen::Matrix4f &Twc, const PSLAM::CameraParameters &camParam) {
        std::tuple<cv::Mat,cv::Mat,cv::Mat,cv::Mat> outputs;
        auto &renderedRGB  = std::get<0>(outputs);
        auto &renderedDepth = std::get<1>(outputs);
        auto &instanceImg = std::get<2>(outputs);
        auto &labelImg = std::get<3>(outputs);

        Eigen::Matrix4f t_p = Twc.transpose();//dataset_loader_->GetPose();

        auto view_pose = glUtil::GetViewMatrix(t_p);
        auto proj = glUtil::Perspective<float>(camParam.fx, camParam.fy,
                                               camParam.cx, camParam.cy,
                                               camParam.width,
                                               camParam.height,
                                               glCam->projection_control_->near, glCam->projection_control_->far);
        cv::Mat t_rgb;
        mMeshRender->Render(proj, view_pose, glCam->projection_control_->near, glCam->projection_control_->far);
        auto cast = dynamic_cast<PSLAM::MeshRenderer3RScan *>(mMeshRender.get());
        renderedDepth = cast->GetDepth();
        renderedRGB = cast->GetRGB();
        instanceImg = cast->GetInstance();
        labelImg    = cast->GetLabel();

        return outputs;
    }

private:
    std::unique_ptr<PSLAM::MeshRendererInterface> mMeshRender;

    void SetRender(PSLAM::INPUTE_TYPE inputeType, int width, int height, const std::string &path, bool align) {
#ifdef COMPILE_WITH_ASSIMP
        std::string folder, scan_id;
        PSLAM::MeshRenderType type;
        if( inputeType== PSLAM::INPUTE_TYPE::DATASET_SCANNET) {
            auto parent_folder = tools::PathTool::find_parent_folder(path, 1);
            scan_id = tools::PathTool::getFileName(parent_folder);
            folder =  tools::PathTool::find_parent_folder(parent_folder, 1);
            type = PSLAM::MeshRenderType_ScanNet;
        } else if (inputeType== PSLAM::INPUTE_TYPE::DATASET_3RSCAN) {
            auto seq_folder = tools::PathTool::find_parent_folder(path,1);
            scan_id = tools::PathTool::getFileName(seq_folder);
            folder = tools::PathTool::find_parent_folder(seq_folder,1);
            type = PSLAM::MeshRenderType_3RScan;
        } else {
            SCLOG(WARNING) << "unknown type for online rendering";
            return;
        }
        mMeshRender.reset( PSLAM::MakeMeshRenderer(width, height, folder,scan_id,type,align) );
#else
        throw std::runtime_error("did not compile with assimp");
#endif
    }
};

int main(int argc, char** argv){
    Config config;
    ParseCommandLine(argc,argv,config);

    if(config.verbose)
        SCLOG_ON(VERBOSE);

    SCLOG(INFO) << "Buliding data loader...";
    std::shared_ptr<PSLAM::DatasetLoader_base> dataset_loader_;
    auto path = config.pth_in;
    dataset_loader_.reset(PSLAM::DataLoaderFactory::Make(path, PSLAM::INPUTE_TYPE::DATASET_3RSCAN));
    dataset_loader_->Reset();
    int height = dataset_loader_->GetRGBImage().rows;
    int width  = dataset_loader_->GetRGBImage().cols;

    /*set output format string*/
    char nameRGB[] = "frame-%06d.rendered.color.jpg";
    char nameD[]   = "frame-%06d.rendered.depth.png";
    char nameI[]   = "frame-%06d.rendered.instances.png";
    char nameL[]   = "frame-%06d.rendered.labels.png";

    SCLOG(INFO) << "Run";
    GUI gui(width,height,path);
    while(true) {
        int curIdx = dataset_loader_->Retrieve();
        if(curIdx<0) {
            SCLOG(INFO) << "read the end of the dataset.";
            break;
        }
        auto pose = dataset_loader_->GetPose();
        auto outputs = gui.render(pose, dataset_loader_->GetCamParamRGB());


        auto &renderedRGB  = std::get<0>(outputs);
        auto &renderedDepth = std::get<1>(outputs);
        auto &instanceImg = std::get<2>(outputs);
        auto &labelImg = std::get<3>(outputs);

        cv::rotate(renderedRGB, renderedRGB, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(renderedDepth, renderedDepth, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(instanceImg, instanceImg, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(labelImg, labelImg, cv::ROTATE_90_CLOCKWISE);
//        cv::imshow("depth",renderedDepth);
//        cv::imshow("rgb",renderedRGB);
//        cv::imshow("instImg",instanceImg);
//        cv::imshow("labelImg",labelImg);
//        cv::waitKey();

        char buff[1024];
        sprintf(buff, nameRGB,curIdx);
        SCLOG(VERBOSE) << config.pth_in+"/"+buff;
//        break;
        cv::imwrite(config.pth_in+"/"+buff,renderedRGB);
        sprintf(buff, nameD,curIdx);
        cv::imwrite(config.pth_in+"/"+buff,renderedDepth);
        sprintf(buff, nameI,curIdx);
        cv::imwrite(config.pth_in+"/"+buff,instanceImg);
        sprintf(buff, nameL,curIdx);
        cv::imwrite(config.pth_in+"/"+buff,labelImg);

//        break;
    }
    SCLOG(INFO) << "Done";

    return 0;
}
