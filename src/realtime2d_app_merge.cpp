#define LOGURU_IMPLEMENTATION 1
#include "system/loguru.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

#include "geodesy/geodesy.hpp"

#include "Optimizer.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
using namespace ORB_SLAM2;

#include <yaml-cpp/yaml.h>
#include "realtime2d_Common.h"
#include "realtime2d_DataPackage.h"
#include "realtime2d_DOM.h"

class CameraSensorData
{
public:
    CameraSensorData() {};
    bool init_basic(Json::Value &camera_info, bool have_camera_setting, Json::Value& camera_setting);
    bool init_calib(Json::Value &camera_calib);
    bool export_yaml(std::string yaml_path);
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    double sensor_width, sensor_height, actual_focal_length;
    int image_width, image_height;
    int pitch_angle, yaw_angle;
    std::string camera_type;
    bool is_calib = false;
};

std::vector<std::string> stringSplit(const std::string &str, char delim)
{
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim))
    {
        if (!item.empty())
        {
            elems.push_back(item);
        }
    }
    return elems;
}

bool ImportLLA(std::vector<double> &lla, std::string path)
{
    if (!stlplus::file_exists(path))
    {
        return false;
    }

    std::ifstream myfile(path);
    std::string s;
    std::getline(myfile, s);

    std::vector<std::string> res = stringSplit(s, ' ');
    if (res.size() == 3)
    {
        lla = {std::atof(res[0].c_str()), std::atof(res[1].c_str()), std::atof(res[2].c_str())};
    }
    else
    {
        myfile.close();
        return false;
    }

    myfile.close();
    return true;
}

Json::Value read_json(std::string json_path)
{

    Json::Value root;
    Json::Reader json_reader;
    Json::Value value_json;
    std::ifstream json_file(json_path.c_str(), std::ios::in);
    if (!json_reader.parse(json_file, value_json))
    {
        std::cout << "Failed to parse json";
        return -1;
    }
    json_file.close();
    return value_json;
}

void write_json(std::string data_path, Json::Value &json)
{
    Json::StyledWriter _writer;
    std::ofstream _os;
    _os.open(data_path);
    _os << _writer.write(json);
    _os.close();
}

inline bool split(
    const std::string &rhs,
    const char delim,
    std::vector<std::string> &items)
{
    items.clear();
    std::stringstream ss(rhs);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        items.emplace_back(item);
    }

    // return true if the delimiter is present in the input string
    return rhs.find(delim) != std::string::npos;
}

Json::Value convert_pose_to_json(const cv::Mat &pose)
{
    Json::Value json_pose;
    json_pose.append(pose.at<float>(0, 0));
    json_pose.append(pose.at<float>(0, 1));
    json_pose.append(pose.at<float>(0, 2));
    json_pose.append(pose.at<float>(0, 3));

    json_pose.append(pose.at<float>(1, 0));
    json_pose.append(pose.at<float>(1, 1));
    json_pose.append(pose.at<float>(1, 2));
    json_pose.append(pose.at<float>(1, 3));

    json_pose.append(pose.at<float>(2, 0));
    json_pose.append(pose.at<float>(2, 1));
    json_pose.append(pose.at<float>(2, 2));
    json_pose.append(pose.at<float>(2, 3));

    json_pose.append(pose.at<float>(3, 0));
    json_pose.append(pose.at<float>(3, 1));
    json_pose.append(pose.at<float>(3, 2));
    json_pose.append(pose.at<float>(3, 3));

    return json_pose;
}
Json::Value convert_position_to_json(const cv::Mat &position)
{
    Json::Value json_pose;
    json_pose.append(position.at<float>(0, 0));
    json_pose.append(position.at<float>(1, 0));
    json_pose.append(position.at<float>(2, 0));
    return json_pose;
}

cv::Mat convert_pose(Json::Value json_pose)
{
    cv::Mat pose(4,4,CV_32FC1);
    pose.at<float>(0, 0) = json_pose[0].asFloat();
    pose.at<float>(0, 1) = json_pose[1].asFloat();
    pose.at<float>(0, 2) = json_pose[2].asFloat();
    pose.at<float>(0, 3) = json_pose[3].asFloat();

    pose.at<float>(1, 0) = json_pose[4].asFloat();
    pose.at<float>(1, 1) = json_pose[5].asFloat();
    pose.at<float>(1, 2) = json_pose[6].asFloat();
    pose.at<float>(1, 3) = json_pose[7].asFloat();

    pose.at<float>(2, 0) = json_pose[8].asFloat();
    pose.at<float>(2, 1) = json_pose[9].asFloat();
    pose.at<float>(2, 2) = json_pose[10].asFloat();
    pose.at<float>(2, 3) = json_pose[11].asFloat();

    pose.at<float>(3, 0) = json_pose[12].asFloat();
    pose.at<float>(3, 1) = json_pose[13].asFloat();
    pose.at<float>(3, 2) = json_pose[14].asFloat();
    pose.at<float>(3, 3) = json_pose[15].asFloat();

    return pose;
}

cv::Mat convert_Kmatrix(Json::Value json_pose)
{
    cv::Mat pose(3,3,CV_32FC1);
    pose.at<float>(0, 0) = json_pose[0].asFloat();
    pose.at<float>(0, 1) = json_pose[1].asFloat();
    pose.at<float>(0, 2) = json_pose[2].asFloat();

    pose.at<float>(1, 0) = json_pose[3].asFloat();
    pose.at<float>(1, 1) = json_pose[4].asFloat();
    pose.at<float>(1, 2) = json_pose[5].asFloat();

    pose.at<float>(2, 0) = json_pose[6].asFloat();
    pose.at<float>(2, 1) = json_pose[7].asFloat();
    pose.at<float>(2, 2) = json_pose[8].asFloat();

    return pose;
}

cv::Mat convert_DistCoef(Json::Value json_pose)
{
    cv::Mat pose(5,1,CV_32FC1);
    pose.at<float>(0) = json_pose[0].asFloat();
    pose.at<float>(1) = json_pose[1].asFloat();
    pose.at<float>(2) = json_pose[2].asFloat();
    pose.at<float>(3) = json_pose[3].asFloat();
    pose.at<float>(4) = json_pose[4].asFloat();
    return pose;
}

cv::Mat convert_position(Json::Value json_pose)
{
    cv::Mat pose(3,1,CV_32FC1);
    pose.at<float>(0, 0) = json_pose[0].asFloat();
    pose.at<float>(1, 0) = json_pose[1].asFloat();
    pose.at<float>(2, 0) = json_pose[2].asFloat();
    return pose;
}

void calRMSE(cv::Mat &pose, std::vector<double> &gps, std::vector<double> &RMSE, bool is_cal)
{
    double poseX = pose.at<float>(0,3);
    double poseY = pose.at<float>(1,3);
    double poseZ = pose.at<float>(2,3);

    double SquareError = (poseX - gps[0]) * (poseX - gps[0]) + (poseY - gps[1]) * (poseY - gps[1]) +(poseZ - gps[2]) * (poseZ - gps[2]); 
    RMSE.push_back(SquareError);
    if(is_cal){
        double sumRMSE = accumulate(RMSE.begin(),RMSE.end(), 0);
        double rmse = sqrt(sumRMSE/RMSE.size());
        LOG_S(INFO) <<std::setprecision(12)<<" RMSE = "<< rmse;
    }
}

cv::Vec3b getRGB(string absPath,cv::KeyPoint obs)
{
    // double fx = K.at<double>(0,0);
    // double fy = K.at<double>(1,1);
    // double cx = K.at<double>(0,2);
    // double cy = K.at<double>(1,2);

    cv::Mat src = cv::imread(absPath,-1);
    if (src.empty())
    {
        LOG_S(INFO)<< "getRGB noimg";
        return;
    }
    cv::cvtColor(src,src,CV_BGR2RGB);
    cv::Vec3b RGB = src.at<cv::Vec3b>(obs.pt);
    return RGB;
    // Eigen::Vector3d Cxyz =  quaternion*xyz + translation;
    // Eigen::Vector2d res_img;//图像坐标系
    // res_img(0) = Cxyz(0)/Cxyz(2);
    // res_img(1) = Cxyz(1)/Cxyz(2);
    // Eigen::Vector2d res_pixel;//像素坐标系
    // res_pixel[0] = res_img[0]*fx + cx;
    // res_pixel[1] = res_img[1]*fy + cy;
}

void MatPose2EigenQt(cv::Mat T, Eigen::Quaterniond &quaternion, Eigen::Vector3d &translation){
    cv::Mat R(3, 3, CV_32F);
    cv::Mat t(3, 1, CV_32F);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R.at<float>(i, j) = T.at<float>(i, j);
        }
        t.at<float>(i, 0) = T.at<float>(i, 3);
    }
    Eigen::Matrix3d eigen_R;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            eigen_R(i, j) = R.at<float>(i, j);
        }
    }
    quaternion = Eigen::Quaterniond(eigen_R);
    quaternion.normalize(); 
    translation = {t.at<float>(0, 0), t.at<float>(1, 0), t.at<float>(2, 0)};
}

cv::Mat Tcw2Tgpsc(cv::Mat Tcw, cv::Mat Tgps_from_w_new, double scale_new)
{
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc * tcw; // 世界坐标系下相机的中心位置。

    cv::Mat Twc = cv::Mat::eye(4, 4, Tcw.type());
    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(Twc.rowRange(0, 3).col(3));
    cv::Mat Rgpsc, tgpsc, Tgpsc;

    Rgpsc = Tgps_from_w_new.rowRange(0, 3).colRange(0, 3) / scale_new * Rwc;
    tgpsc = Tgps_from_w_new.rowRange(0, 3).colRange(0, 3) * Ow + Tgps_from_w_new.rowRange(0, 3).col(3);
    Tgpsc = cv::Mat::eye(4, 4, CV_32F); // Tgpsc
    Rgpsc.copyTo(Tgpsc.rowRange(0, 3).colRange(0, 3));
    tgpsc.copyTo(Tgpsc.rowRange(0, 3).col(3));
    return Tgpsc;
}

class TaskKeyframesMapoints{
public:
    TaskKeyframesMapoints(std::string strVocFile, std::string workspace_directory, std::string task_id, std::string output_directory, 
    ORBextractor* extractor=nullptr, ORBVocabulary* vocabulary=nullptr, KeyFrameDatabase* keyframedb = nullptr, bool save_colmap = false)
    :strVocFile(strVocFile),workspace_directory(workspace_directory),task_id(task_id),output_directory(output_directory),mSaveColmap(save_colmap)
    {
        std::string base_task_json_path = workspace_directory + "/" + task_id + "/realtime2d_selfcalib/keyframes_maps.json";
        Json::Value base_task_json = read_json(base_task_json_path);
        Json::Value base_images_json = base_task_json["images"];
        Json::Value base_mappoints_json = base_task_json["mappoints"];

        std::string base_task_xml_path = workspace_directory + "/" + task_id + "/realtime2d_selfcalib/descriptors.xml";
        cv::FileStorage base_task_xml_fs(base_task_xml_path, cv::FileStorage::READ);

        Tgps_from_w = convert_pose(base_task_json["Tgpsw"]);
        scale = base_task_json["scale"].asFloat();

        mK = convert_Kmatrix(base_task_json["K"]);
        mDistCoef = convert_DistCoef(base_task_json["distCoef"]);

        reference_latitude = base_task_json["reference_lla"]["lat"].asDouble();
        reference_longitude = base_task_json["reference_lla"]["lon"].asDouble();
        reference_altitude = base_task_json["reference_lla"]["alt"].asDouble();

        std::string strSettingPath =  workspace_directory + "/" + task_id + "/camera.yaml";
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mCameraType = fSettings["Camera.type"].string();
        nFeatures = fSettings["ORBextractor.nFeatures"];
        fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        nLevels = fSettings["ORBextractor.nLevels"];
        fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        fMinThFAST = fSettings["ORBextractor.minThFAST"];
        mbf = fSettings["Camera.bf"];
        mThDepth = mbf*(float)fSettings["ThDepth"]/mK.at<float>(0,0);
        LOG_S(INFO) << "ORB Extractor Parameters: " << endl;
        LOG_S(INFO) << "- mCameraType: " << mCameraType << endl;
        LOG_S(INFO) << "- Number of Features: " << nFeatures << endl;
        LOG_S(INFO) << "- Scale Levels: " << nLevels << endl;
        LOG_S(INFO) << "- Scale Factor: " << fScaleFactor << endl;
        LOG_S(INFO) << "- Initial Fast Threshold: " << fIniThFAST << endl;
        LOG_S(INFO) << "- Minimum Fast Threshold: " << fMinThFAST << endl;
        if(extractor == nullptr){
            mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
            init_extractor = true;
        }
        else{
            mpORBextractorLeft = extractor;
        }
        
        fSettings.release();

        if(vocabulary == nullptr){
            mpORBVocabulary = new ORBVocabulary();
            bool bVocLoad = mpORBVocabulary->loadFromBinaryFile(strVocFile);
            init_vocabulary = true;
        }
        else{
            mpORBVocabulary = vocabulary;
        }
        

        //Create KeyFrame Database
        if(keyframedb == nullptr){
            mpKeyFrameDatabase = new KeyFrameDatabase(*mpORBVocabulary);
            init_keyframedb = true;
        }
        else{
            mpKeyFrameDatabase = keyframedb;
        }

        //Create the Map
        mpMap = new Map(0);


        
        for (int i = 0; i < base_images_json.size(); i++)
        {
            Json::Value image_json = base_images_json[i];
            LOG_S(INFO) << "mnID: " << image_json["mnID"].asInt()<< " name: " << image_json["name"].asString()<< " absPath: " << image_json["absPath"].asString();

            int mnID = image_json["mnID"].asInt();
            std::string absPath = image_json["absPath"].asString();

            std::string base_name = stlplus::basename_part(absPath);
            std::string text_path = stlplus::folder_part(absPath) + "/" + base_name + ".txt";
            if(stlplus::file_exists(text_path)){
                std::vector<double> lla;//latitude, longitude, altitude
                ImportLLA(lla,text_path);
                bool have_gps = false;
                
                if(!lla.empty()){
                    have_gps = true;

                    double latitude, longitude, altitude;
                    latitude = lla[0];
                    longitude = lla[1];
                    altitude = lla[2];

                    xagmapper::Vec3 image_lla;
                    image_lla[0] = latitude;
                    image_lla[1] = longitude;
                    image_lla[2] = altitude;

                    Eigen::Vector3d t_lla(image_lla[1], image_lla[0], 0);
                    Eigen::Vector3d web_xyz = xagmapper::geodesy::llaToWebMercator(t_lla);

                    std::vector<double> position = {web_xyz[0], web_xyz[1], image_lla[2]};
                    mpRealPositions[mnID] = position;
                    mpLLAs[mnID] = lla;
                    // LOG_S(INFO)<< std::setprecision(12) <<"task_id: "<<task_id<<" base_name: "<<base_name<<" lla: "<<lla[0]<<" "<<lla[1]<<" "<<lla[2];
                    LOG_S(INFO)<< std::setprecision(12) <<"task_id: "<<task_id<<" base_name: "<<base_name<<" position: "<<position[0]<<" "<<position[1]<<" "<<position[2];
                }
            }
            
            cv::Mat Tcw = convert_pose(image_json["Tcw"]);

            cv::Mat gps_ = convert_position(image_json["gps"]);
            std::vector<double> gps = {gps_.at<float>(0),gps_.at<float>(1),gps_.at<float>(2)};
            // LOG_S(INFO)<< std::setprecision(12) <<"task_id: "<<task_id<<" base_name: "<<base_name<<" gps: "<<gps[0]<<" "<<gps[1]<<" "<<gps[2];

            Json::Value features_json = image_json["features"];
            std::vector<cv::KeyPoint> mvKeys;
            for(auto& feature:features_json){
                float x = feature["x"].asFloat();
                float y = feature["y"].asFloat();
                float size = feature["size"].asFloat();
                float angle = feature["angle"].asFloat();
                float response = feature["response"].asFloat();
                int octave = feature["octave"].asInt();
                cv::KeyPoint keypoint(cv::Point2f(x,y),size,angle,response,octave);
                mvKeys.push_back(keypoint);
            }
            cv::Mat mDescriptors;
            base_task_xml_fs[std::string("keyframe_"+std::to_string(mnID)).c_str()] >> mDescriptors;
            LOG_S(INFO) << "mnID: " << mnID<<" mDescriptors: "<<mDescriptors.size();

            if(mImGray.empty()){
                cv::Mat image = cv::imread(absPath);
                cv::cvtColor(image,mImGray,CV_RGB2GRAY);
            }

            Frame mCurrentFrame = Frame(mImGray, mvKeys, mDescriptors,
            mnID, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, gps, Tgps_from_w, scale, absPath, "");// scale
            mCurrentFrame.mnId = mnID;
            mCurrentFrame.mTcw = Tcw.clone();
            mCurrentFrame.ComputeBoW();

            mpFrames[mnID] = Frame(mCurrentFrame,mCurrentFrame.mAbsPath);

            KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDatabase);
            pKF->mnId = mnID;
            pKF->is_fixed = true;
            mpMap->AddKeyFrame(pKF);
            nNextId = std::max(pKF->mnId,nNextId);
            nNextId++;

        }
        base_task_xml_fs.release();
        

        vpKFs = mpMap->GetAllKeyFrames();
        for (int i = 0; i < base_mappoints_json.size(); i++)
        {
            Json::Value mappoint_json = base_mappoints_json[i];
            cv::Mat x3D = convert_position(mappoint_json["position"]);

            std::vector<std::pair<int,int>> obs;
            Json::Value obs_json = mappoint_json["observations"];
            if(obs_json.size()>0){
                for(auto& data:obs_json){
                    obs.push_back(std::make_pair(data["mnID"].asInt(),data["keypt_id"].asInt()));
                }
                int mnFirstKFid = obs[0].first;
                KeyFrame* mpCurrentKeyFrame;
                for(auto& vpKF:vpKFs){
                    if(vpKF->mnId==mnFirstKFid){
                        mpCurrentKeyFrame = vpKF;
                        break;
                    }
                }
                MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
                for(auto& data:obs){
                    KeyFrame* keyFrame;
                    for(auto& vpKF:vpKFs){
                        if(vpKF->mnId==data.first){
                            keyFrame = vpKF;
                            break;
                        }
                    }
                    pMP->AddObservation(keyFrame,data.second);      
                    keyFrame->AddMapPoint(pMP,data.second);
                }
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pMP);
            }
        }
        vpMP = mpMap->GetAllMapPoints();
        LOG_S(INFO) << "task_id: "<<task_id<<" vpKFs: "<<vpKFs.size()<< " vpMP: "<<vpMP.size();
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            pKF->UpdateConnections();
        }

        std::vector<Eigen::Vector3d> src_tw;
        std::vector<Eigen::Vector3d> tgt_tgps;
        bool init_sim3_matrix = false;
        std::vector<PlyUtils::ColorPoint_t> color_points;
        std::vector<PlyUtils::ColorPoint_t> visual_color_points;
        std::vector<double> rmse_arr;
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            int mnId = pKF->mnId;

            cv::Mat Tcw = pKF->GetPose();
            cv::Mat gps = pKF->GetGPSPosition();
            tgt_tgps.push_back(ORB_SLAM2::Converter::toVector3d(gps));

            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;// 世界坐标系下相机的中心位置。
            src_tw.push_back(ORB_SLAM2::Converter::toVector3d(Ow));

            cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
            Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
            Ow.copyTo(Twc.rowRange(0,3).col(3));

            cv::Mat Rgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) / scale * Rwc;
            cv::Mat tgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) * Ow + Tgps_from_w.rowRange(0,3).col(3);
            cv::Mat Tgpsc = cv::Mat::eye(4, 4, CV_32F); // Tgpsc
            Rgpsc.copyTo(Tgpsc.rowRange(0,3).colRange(0,3));
            tgpsc.copyTo(Tgpsc.rowRange(0,3).col(3));

            // LOG_S(INFO)<<"mnId: "<<mnId<<" Tgpsc: "<<Tgpsc<<" gps: "<<gps;
            float srcPosX = Tgpsc.at<float>(0,3);
            float srcPosY = Tgpsc.at<float>(1,3);
            float srcPosZ = Tgpsc.at<float>(2,3);

            float gpsX = gps.at<float>(0,0);
            float gpsY = gps.at<float>(1,0);
            float gpsZ = gps.at<float>(2,0);

            std::vector<double> t_gps = {gpsX, gpsY, gpsZ};
            calRMSE(Tgpsc,t_gps,rmse_arr, false);

            {
                PlyUtils::ColorPoint_t m_pose;
                m_pose.vertex[0] = srcPosX;
                m_pose.vertex[1] = srcPosY;
                m_pose.vertex[2] = srcPosZ;
                m_pose.vcolor[2] = 255;
                m_pose.vcolor[0] = m_pose.vcolor[1] = 0;
                color_points.push_back(m_pose);
            }
            {
                PlyUtils::ColorPoint_t m_pose;
                m_pose.vertex[0] = gpsX;
                m_pose.vertex[1] = gpsY;
                m_pose.vertex[2] = gpsZ;
                m_pose.vcolor[1] = 255;
                m_pose.vcolor[0] = m_pose.vcolor[2] = 0;
                color_points.push_back(m_pose);
            }
            {
                PlyUtils::ColorPoint_t m_pose;
                m_pose.vertex[0] = Twc.at<float>(0,3);
                m_pose.vertex[1] = Twc.at<float>(1,3);
                m_pose.vertex[2] = Twc.at<float>(2,3);
                m_pose.vcolor[2] = 255;
                m_pose.vcolor[0] = m_pose.vcolor[1] = 0;
                visual_color_points.push_back(m_pose);
            }
        }

        double sumRMSE = accumulate(rmse_arr.begin(),rmse_arr.end(), 0);
        double rmse = sqrt(sumRMSE/rmse_arr.size());
        LOG_S(INFO) << "tracking RMSE = "<< rmse;

        for(int i=0; i<vpMP.size(); i++)
        {
            ORB_SLAM2::MapPoint* pMP = vpMP[i];
            if(pMP->isBad())
                continue;
            cv::Mat mWorldPos = pMP->GetWorldPos();
            cv::Mat tgps = Tgps_from_w.rowRange(0,3).colRange(0,3) * mWorldPos + Tgps_from_w.rowRange(0,3).col(3);

            float gpsX = tgps.at<float>(0,0);
            float gpsY = tgps.at<float>(1,0);
            float gpsZ = tgps.at<float>(2,0);

            PlyUtils::ColorPoint_t m_pose;
            m_pose.vertex[0] = gpsX;
            m_pose.vertex[1] = gpsY;
            m_pose.vertex[2] = gpsZ;
            m_pose.vcolor[1] = 255;
            m_pose.vcolor[0] = m_pose.vcolor[2] = 255;
            color_points.push_back(m_pose);
        }

        std::string ply_path =  output_directory+ "/"+task_id + "_keyframes_landmarks.ply";  
        LOG_S(INFO) << "ply_path: "<<ply_path;
        PlyUtils::writePly(ply_path, color_points, false);


        std::string colmap_directory = output_directory+ "/"+task_id+"/";
        if (mSaveColmap&&!stlplus::folder_exists(colmap_directory))
        {
            if (!stlplus::folder_create(colmap_directory))
            {
                LOG_F(ERROR, "Cannot create  colmap_directory");
            }
        }
        if (mSaveColmap&&stlplus::folder_exists(colmap_directory))
        {
            cv::Mat srcK = cv::Mat::eye(3,3,CV_32F);
            srcK.at<float>(0,0) = mK.at<float>(0,0);
            srcK.at<float>(1,1) = mK.at<float>(1,1);
            srcK.at<float>(0,2) = mK.at<float>(0,2);
            srcK.at<float>(1,2) = mK.at<float>(1,2);
            cv::Mat Tgpsw = Tgps_from_w;

            std::string points3DPath = colmap_directory + "/points3D.txt";
            std::string imgsPath = colmap_directory + "/images.txt";
            std::string camsPath = colmap_directory + "/cameras.txt";

            std::ofstream outp3d(points3DPath.c_str());
            std::ofstream outimgs(imgsPath.c_str());
            std::ofstream outcams(camsPath.c_str());

            const std::vector<ORB_SLAM2::MapPoint*> allMapPts = mpMap->GetAllMapPoints();
            const std::vector<ORB_SLAM2::KeyFrame *> allKF= mpMap->GetAllKeyFrames();

            outp3d << "# 3D point list with one line of data per point: \n#  POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX) \n";
            outimgs << "# Image list with two lines of data per image:\n#  IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME \n#  POINTS2D[] as (X, Y, POINT3D_ID)\n";
            outcams << "# Camera list with one line of data per camera:\n# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n# Number of cameras: 1\n";
            for(int i = 0;i<allMapPts.size();i++)
            {
                MapPoint* mpt = allMapPts[i];
                if (mpt->isBad())
                    continue;
                
                const map<KeyFrame*,size_t> observations = mpt->GetObservations();
                bool isSetrgb = false;
                for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    KeyFrame *pKF = mit->first;
                    // 跳过无效关键帧
                    if (pKF->isBad() )
                        continue;

                    const cv::KeyPoint &kp = pKF->mvKeys[mit->second];// 未去畸变的点
                    if(!isSetrgb)
                    {
                        outp3d << mpt->mnId << " "; // id
                        cv::Mat worldPose = mpt->GetWorldPos();
                        cv::Mat realworldPose = Tgpsw.rowRange(0, 3).colRange(0, 3) * worldPose + Tgpsw.rowRange(0, 3).col(3);
                        outp3d << realworldPose.at<float>(0, 0) << " " << realworldPose.at<float>(1, 0) << " " << realworldPose.at<float>(2, 0) << " "; // xyz
                        
                        cv::Vec3b RGB = getRGB(pKF->absPath, kp);
                        outp3d << (int)RGB[0] << " " << (int)RGB[1] << " " << (int)RGB[2] << " "; // rgb
                        outp3d << 0 <<" "; // error
                        isSetrgb = true;
                    }
                    outp3d << pKF->mnId <<" "; //imgid
                    outp3d << mit->second <<" ";// ptid
                }
                if(!mpt->isBad())
                    outp3d << "\n";
            }
            outp3d.close();
            LOG_S(INFO)<<"save imgs.txt allKF.size() = "<<allKF.size();
            outimgs<<"# Number of images: "<<allKF.size()<< ",mean observations per image: "<<allMapPts.size()/allKF.size()<<"\n";
            for (int i = 0; i < allKF.size(); i++)
            {
                KeyFrame* pKF = allKF[i];
                // 跳过无效关键帧
                if (pKF->isBad())// || pKF->absPath.empty())
                    continue;
                outimgs << pKF->mnId << " ";//imgid
                cv::Mat Tcw = pKF->GetPose();
                cv::Mat Tgpsc = Tcw2Tgpsc(Tcw, Tgpsw, scale);

                cv::Mat Rgspcs = Tgpsc.rowRange(0, 3).colRange(0, 3);
                cv::Mat tgpsc = Tgpsc.rowRange(0, 3).col(3);
                cv::Mat Rcgps = Rgspcs.t();
                cv::Mat Ow = -Rcgps * tgpsc; 
                cv::Mat Tcgps = cv::Mat::eye(4, 4, Tgpsc.type());
                Rcgps.copyTo(Tcgps.rowRange(0, 3).colRange(0, 3));
                Ow.copyTo(Tcgps.rowRange(0, 3).col(3));

                Eigen::Quaterniond quaternion;
                Eigen::Vector3d translation;
                MatPose2EigenQt(Tcgps, quaternion, translation);
                outimgs << quaternion.w()<< " "<< quaternion.x()<< " "<< quaternion.y()<< " "<< quaternion.z()<< " ";//q
                outimgs << translation.x() << " "<< translation.y() << " "<<translation.z() << " ";//xyz
                outimgs << 1 << " ";//cam id
                string abspath = pKF->absPath;
                // vector<string> vecStrpath = splitStr(abspath,"//");
                std::string file_name = stlplus::basename_part(abspath) + "." + stlplus::extension_part(abspath);
            
                // outimgs << vecStrpath[1] << "\n"; //imgname
                outimgs << file_name << "\n"; //imgname
                const int N = pKF->N;
                std::vector<ORB_SLAM2::MapPoint *> mappt = pKF->GetMapPointMatches();//每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
                for(int j = 0;j<N;j++)
                {
                    MapPoint* pMP = mappt[j];
                    if(pMP)
                    {
                        const cv::KeyPoint &unkp = pKF->mvKeysUn[j];
                        outimgs << unkp.pt.x << " "<< unkp.pt.y << " "<< pMP->mnId<< " ";
                    }
                }
                outimgs <<"\n";
            }
            outimgs.close();
            
            outcams << "1"<< " ";//cam id
            outcams << "PINHOLE" << " ";// model
            outcams << mImGray.cols <<" " <<mImGray.rows<< " " 
                    << srcK.at<float>(0,0)<< " "<<srcK.at<float>(1,1)<< " "
                    << srcK.at<float>(0,2)<< " "<<srcK.at<float>(1,2)<< " ";
            outcams.close();
        }

    };
    ~TaskKeyframesMapoints(){
        delete mpMap;
        if(init_keyframedb)delete mpKeyFrameDatabase;
        if(init_extractor)delete mpORBextractorLeft;
        if(init_vocabulary)delete mpORBVocabulary;
    };
    vector<KeyFrame*> GetAllKeyFrames()
    {
        vpKFs = mpMap->GetAllKeyFrames();
        std::sort(vpKFs.begin(),vpKFs.end(),[](KeyFrame*a, KeyFrame*b){
            return a->mnId<b->mnId;
        });
        return vpKFs;
    }

    vector<MapPoint*> GetAllMapPoints()
    {
        vpMP = mpMap->GetAllMapPoints();
        return vpMP;
    }

    KeyFrame* AddKeyFrame(Frame& mCurrentFrame, std::vector<double>& lla){

        KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDatabase);
        pKF->mnId = nNextId;
        mpMap->AddKeyFrame(pKF);

        mpFrames[pKF->mnId] = Frame(mCurrentFrame,mCurrentFrame.mAbsPath);

        double latitude, longitude, altitude;
        latitude = lla[0];
        longitude = lla[1];
        altitude = lla[2];

        xagmapper::Vec3 image_lla;
        image_lla[0] = latitude;
        image_lla[1] = longitude;
        image_lla[2] = altitude;

        Eigen::Vector3d t_lla(image_lla[1], image_lla[0], 0);
        Eigen::Vector3d web_xyz = xagmapper::geodesy::llaToWebMercator(t_lla);

        std::vector<double> position = {web_xyz[0], web_xyz[1], image_lla[2]};
        mpRealPositions[pKF->mnId] = position;
        mpLLAs[pKF->mnId] = lla;
        
        nNextId = std::max(pKF->mnId,nNextId);
        nNextId++;

        return pKF;
    }

    MapPoint* AddMappoint(cv::Mat& x3D, KeyFrame* mpCurrentKeyFrame, map<KeyFrame*,size_t>& observations){

        MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
        for(auto& data:observations){
            pMP->AddObservation(data.first,data.second);      
            data.first->AddMapPoint(pMP,data.second);
        }

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        mpMap->AddMapPoint(pMP);

        return pMP;
    }

    std::string workspace_directory;
    std::string task_id;
    std::string output_directory;
    std::string strVocFile;
    bool mSaveColmap = false;

    ORBVocabulary* mpORBVocabulary;
    ORBextractor* mpORBextractorLeft;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Map* mpMap;
    bool init_vocabulary = false;
    bool init_extractor = false;
    bool init_keyframedb = false;

    cv::Mat mK;
    cv::Mat mDistCoef;
    cv::Mat mImGray;

    std::string mCameraType;
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    int fIniThFAST;
    int fMinThFAST;
    float mbf;
    float mThDepth;

    double reference_latitude = 0.0;
    double reference_longitude = 0.0;
    double reference_altitude = 0.0;
    cv::Mat Tgps_from_w;
    float scale;
    std::vector<KeyFrame*> vpKFs;
    std::vector<MapPoint*> vpMP;
    std::map<int, Frame> mpFrames;
    std::map<int, std::vector<double>> mpRealPositions;
    std::map<int, std::vector<double>> mpLLAs;//latitude, longitude, altitude
    long unsigned int nNextId=0;
};

void saveTaskKeyframesMapoints(TaskKeyframesMapoints& baseTask, std::string output_directory, std::string fix_str=""){

    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = baseTask.GetAllKeyFrames();

    std::vector<ORB_SLAM2::KeyFrame* > intersection_vpKFs;
    int kfs_totalnums = 0;
    int kfs_intersection_nums = 0;
    for(int i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
        kfs_totalnums++;
        std::vector<MapPoint*> TrackedMapPoints = pKF->GetMapPointMatches();
        bool is_intersection = false;
        for(auto& pMP:TrackedMapPoints){
            if(pMP!=nullptr&&!pMP->isBad()){
                std::set<std::string> folders;
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    std::string folder = stlplus::folder_part(mit->first->absPath);
                    // LOG_S(INFO)<<mit->first->mnId<<" "<<folder;
                    folders.insert(folder);
                }
                if(folders.size()>=2){
                    is_intersection = true;
                    break;
                }
            }
        }
        if(is_intersection){
            kfs_intersection_nums++;
            intersection_vpKFs.push_back(pKF);
        }
    }
    LOG_S(INFO) << "kfs_totalnums: "<< kfs_totalnums;
    LOG_S(INFO) << "kfs_intersection_nums: "<< kfs_intersection_nums;


    cv::Mat Tgps_from_w = baseTask.Tgps_from_w;
    float scale = baseTask.scale;
    std::vector<PlyUtils::ColorPoint_t> gps_points;
    std::vector<PlyUtils::ColorPoint_t> color_points;
    std::vector<double> rmse_arr;
    for(int i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
        int mnId = pKF->mnId;

        cv::Mat Tcw = pKF->GetPose();
        cv::Mat gps = pKF->GetGPSPosition();

        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat Ow = -Rwc*tcw;// 世界坐标系下相机的中心位置。

        cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));

        cv::Mat Rgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) / scale * Rwc;
        cv::Mat tgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) * Ow + Tgps_from_w.rowRange(0,3).col(3);
        cv::Mat Tgpsc = cv::Mat::eye(4, 4, CV_32F); // Tgpsc
        Rgpsc.copyTo(Tgpsc.rowRange(0,3).colRange(0,3));
        tgpsc.copyTo(Tgpsc.rowRange(0,3).col(3));

        // LOG_S(INFO)<<"mnId: "<<mnId<<" Tgpsc: "<<Tgpsc<<" gps: "<<gps;
        float srcPosX = Tgpsc.at<float>(0,3);
        float srcPosY = Tgpsc.at<float>(1,3);
        float srcPosZ = Tgpsc.at<float>(2,3);

        float gpsX = gps.at<float>(0,0);
        float gpsY = gps.at<float>(1,0);
        float gpsZ = gps.at<float>(2,0);

        std::vector<double> t_gps = {gpsX, gpsY, gpsZ};
        calRMSE(Tgpsc,t_gps,rmse_arr, false);

        bool is_intersection = false;
        for(auto&ptr:intersection_vpKFs){
            if(pKF->mnId == ptr->mnId){
                is_intersection = true;
                break;
            }
        }

        if(!is_intersection)
        {
            PlyUtils::ColorPoint_t m_pose;
            m_pose.vertex[0] = srcPosX;
            m_pose.vertex[1] = srcPosY;
            m_pose.vertex[2] = srcPosZ;
            m_pose.vcolor[2] = 255;
            m_pose.vcolor[0] = m_pose.vcolor[1] = 0;
            color_points.push_back(m_pose);
        }
        else{
            PlyUtils::ColorPoint_t m_pose;
            m_pose.vertex[0] = srcPosX;
            m_pose.vertex[1] = srcPosY;
            m_pose.vertex[2] = srcPosZ;
            m_pose.vcolor[0] = 255;
            m_pose.vcolor[2] = m_pose.vcolor[1] = 0;
            color_points.push_back(m_pose);
        }

        {
            PlyUtils::ColorPoint_t m_pose;
            m_pose.vertex[0] = gpsX;
            m_pose.vertex[1] = gpsY;
            m_pose.vertex[2] = gpsZ;
            m_pose.vcolor[1] = 255;
            m_pose.vcolor[0] = m_pose.vcolor[2] = 0;
            gps_points.push_back(m_pose);
        }
    }





    std::vector<ORB_SLAM2::MapPoint*> vpMP = baseTask.GetAllMapPoints();

    std::vector<ORB_SLAM2::MapPoint* > intersection_vpMP;
    int mappoints_totalnums = 0;
    int mappoints_intersection_nums = 0;
    for(int i=0; i<vpMP.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        mappoints_totalnums++;

        std::set<std::string> folders;
        map<KeyFrame*,size_t> observations = pMP->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            std::string folder = stlplus::folder_part(mit->first->absPath);
            // LOG_S(INFO)<<"pMP: "<<pMP->mnId<<" key: "<<mit->first->mnId<<" "<<folder;
            folders.insert(folder);
        }
        if(folders.size()>=2){
            mappoints_intersection_nums++;
            intersection_vpMP.push_back(pMP);
        }
    }
    LOG_S(INFO) << "mappoints_totalnums: "<< mappoints_totalnums;
    LOG_S(INFO) << "mappoints_intersection_nums: "<< mappoints_intersection_nums;


    for(int i=0; i<vpMP.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        cv::Mat mWorldPos = pMP->GetWorldPos();
        cv::Mat tgps = Tgps_from_w.rowRange(0,3).colRange(0,3) * mWorldPos + Tgps_from_w.rowRange(0,3).col(3);

        float gpsX = tgps.at<float>(0,0);
        float gpsY = tgps.at<float>(1,0);
        float gpsZ = tgps.at<float>(2,0);

        PlyUtils::ColorPoint_t m_pose;
        m_pose.vertex[0] = gpsX;
        m_pose.vertex[1] = gpsY;
        m_pose.vertex[2] = gpsZ;

        bool is_intersection = false;
        for(auto&ptr:intersection_vpMP){
            if(pMP->mnId == ptr->mnId){
                is_intersection = true;
                break;
            }
        }
        if(!is_intersection){
            m_pose.vcolor[1] = 255;
            m_pose.vcolor[0] = m_pose.vcolor[2] = 255;
        }
        else{
            m_pose.vcolor[1] = 0;
            m_pose.vcolor[0] = m_pose.vcolor[1] = 255;
        }
        color_points.push_back(m_pose);
    }

    double sumRMSE = accumulate(rmse_arr.begin(),rmse_arr.end(), 0);
    double rmse = sqrt(sumRMSE/rmse_arr.size());
    LOG_S(INFO) << "tracking RMSE = "<< rmse;

    std::string keyframes_landmarks_ply_path =  output_directory+ "/keyframes_landmarks.ply";  
    if(!fix_str.empty()){
        keyframes_landmarks_ply_path =  output_directory+ "/keyframes_landmarks_"+fix_str+".ply";  
    }
    LOG_S(INFO) << "keyframes_landmarks_ply_path: "<<keyframes_landmarks_ply_path;
    PlyUtils::writePly(keyframes_landmarks_ply_path, color_points, false);

    std::string gps_ply_path =  output_directory+ "/gps.ply";  
    if(!fix_str.empty()){
        gps_ply_path =  output_directory+ "/gps_"+fix_str+".ply";  
    }
    LOG_S(INFO) << "gps_ply_path: "<<gps_ply_path;
    PlyUtils::writePly(gps_ply_path, gps_points, false);
}


bool Relocalization(Frame& mCurrentFrame, vector<KeyFrame*>& vpCandidateKFs, KeyFrame** mpReferenceKF){

    mCurrentFrame.ComputeBoW();
    LOG_S(INFO) << "mCurrentFrame: "<<mCurrentFrame.mnId<<" mvKeys: "<<mCurrentFrame.mvKeys.size()<<" mDescriptors: "<<mCurrentFrame.mDescriptors.size();
    LOG_S(INFO)<<"mDescriptors: "<<mCurrentFrame.mDescriptors.row(0);


    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            LOG_S(INFO) << "[SearchByBoW] mCurrentFrame: "<<mCurrentFrame.mnId<<" pKF: "<<pKF->mnId<<" nmatches: "<<nmatches;

            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }


        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);
    int best_nGood = 100;
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
            // LOG_S(INFO) << "[pSolver] mCurrentFrame: "<<mCurrentFrame.mnId<<" Tcw: "<<Tcw;

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                // LOG_S(INFO) << "[Optimizer::PoseOptimization] mCurrentFrame: "<<mCurrentFrame.mnId<<" nGood: "<<nGood;

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<best_nGood)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=best_nGood)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>best_nGood-20 && nGood<best_nGood)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=best_nGood)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=best_nGood)
                {
                    LOG_S(INFO) << "[match] mCurrentFrame: "<<mCurrentFrame.mnId<<" match: "<<vpCandidateKFs[i]->mnId<<" nGood: "<<nGood;
                    *mpReferenceKF = vpCandidateKFs[i];
                    bMatch = true;
                    break;
                }
            }
        }
    }

    bool bOK = bMatch;
    LOG_S(INFO) << "[Relocalization] mCurrentFrame: "<<mCurrentFrame.mnId<<" bOK: "<<bOK<<" mTcw: "<<mCurrentFrame.mTcw;
    return bOK;
}

bool TrackReferenceKeyFrame(Frame& mCurrentFrame, KeyFrame* mpReferenceKF, Frame& mLastFrame){
    bool bOK = false;
    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    // LOG_S(INFO)<<"[after TrackReferenceKeyFrame SearchByBoW] checkMapPointNums: "<<checkMapPointNums()<<" nmatches: "<<nmatches;
    if(nmatches>=15){
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }
        bOK = nmatchesMap>=10;
    }
    LOG_S(INFO) << "[TrackReferenceKeyFrame] mCurrentFrame: "<<mCurrentFrame.mnId<<" bOK: "<<bOK;
    return bOK;
}


bool TrackLocalMap(Frame& mCurrentFrame, KeyFrame** mpReferenceKF){

    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        *mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = *mpReferenceKF;
    }


    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }

    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }

    Optimizer::PoseOptimization(&mCurrentFrame);

    int mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    mnMatchesInliers++;
            }
        }
    }

    // Decide if the tracking was succesful
    bool bOK = mnMatchesInliers>30;
    LOG_S(INFO) << "[TrackLocalMap] mCurrentFrame: "<<mCurrentFrame.mnId<<" bOK: "<<bOK;
    return bOK;

}

bool TrackWithMotionModel(Frame& mCurrentFrame, Frame& mLastFrame, cv::Mat& mVelocity)
{
    ORBmatcher matcher(0.9,true);

  
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,false);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,false);//2
    }
    // LOG_S(INFO)<<"[after TrackWithMotionModel SearchByProjection] checkMapPointNums: "<<checkMapPointNums();
    if(nmatches<20){
        bool bOK = false;
        LOG_S(INFO) << "[TrackWithMotionModel] mCurrentFrame: "<<mCurrentFrame.mnId<<" bOK: "<<bOK;
        return false;
    }
        

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    
    bool bOK = nmatchesMap>=20;
    LOG_S(INFO) << "[TrackWithMotionModel] mCurrentFrame: "<<mCurrentFrame.mnId<<" bOK: "<<bOK;
    return bOK;
}

bool AddMissingView(Frame& mCurrentFrame, vector<KeyFrame*>& vpCandidateKFs, bool is_tracking_mode, 
    KeyFrame** mpReferenceKF, Frame& mLastFrame, cv::Mat& mVelocity, ORB_SLAM2::Tracking::eTrackingState& mState){

    bool bOK = false;

    if(!is_tracking_mode){

        bOK = Relocalization(mCurrentFrame, vpCandidateKFs,mpReferenceKF);

        if(bOK) {
            bOK = TrackLocalMap(mCurrentFrame, mpReferenceKF);
            if(bOK){

                KeyFrame* pKF = new KeyFrame(mCurrentFrame,nullptr,nullptr);

                vector<KeyFrame*> vNeighKFs;
                int map_point_nums = 0;
                set<int> connect_keys;
                int non_fixed_nums = 0;
                int total_edges_nums = 0;
                vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
                for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
                {
                    MapPoint* pMP = *vit;
                    if(pMP){
                        if(!pMP->isBad())
                        {
                            map_point_nums++;
                            map<KeyFrame*,size_t> observations = pMP->GetObservations();
                            for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFrame* pKFi = mit->first;
                                if(pKFi->mnId == pKF->mnId){
                                    continue;
                                }
                                if(!pKFi->isBad()){
                                    if(!pKFi->is_fixed){
                                        non_fixed_nums++;
                                    }
                                    total_edges_nums++;

                                    if(connect_keys.count(pKFi->mnId)==0){
                                        connect_keys.insert(pKFi->mnId);
                                        vNeighKFs.push_back(pKFi);
                                    }
                                }
                            }
                        }
                    }
                }
                std::ostringstream oss;
                for (auto view:vNeighKFs)
                {
                    oss<<view->mnId<< " ";
                }
                LOG_S(INFO)<<"CHECK pKF: "<<pKF->mnId<<" pMP: "<<map_point_nums<<" observations: "<<oss.str()<<" size: "<<connect_keys.size();
                LOG_S(INFO)<<"CHECK pKF: "<<pKF->mnId<<" non_fixed_nums: "<<non_fixed_nums<<" total_edges_nums: "<<total_edges_nums;

                Optimizer::LocalBundleAdjustment2(pKF, vNeighKFs);

                mCurrentFrame.SetPose(pKF->GetPose());
                mCurrentFrame.mvpMapPoints = pKF->GetMapPointMatches();
                delete pKF;
                mLastFrame = mCurrentFrame;
            }
        }
        
    }
    else{

        if(mState==ORB_SLAM2::Tracking::eTrackingState::OK){
            if(mVelocity.empty()){
                bOK = TrackReferenceKeyFrame(mCurrentFrame, *mpReferenceKF, mLastFrame);
                if(!bOK){
                    bOK = Relocalization(mCurrentFrame, vpCandidateKFs,mpReferenceKF);
                }
            }
            else{
                bOK = TrackWithMotionModel(mCurrentFrame, mLastFrame, mVelocity);
                if(!bOK){
                    bOK = TrackReferenceKeyFrame(mCurrentFrame, *mpReferenceKF, mLastFrame);
                }
                if(!bOK){
                    bOK = Relocalization(mCurrentFrame, vpCandidateKFs,mpReferenceKF);
                }
            }
        }
        else{
            bOK = Relocalization(mCurrentFrame, vpCandidateKFs,mpReferenceKF);
        }

        if(bOK) {
            bOK = TrackLocalMap(mCurrentFrame, mpReferenceKF);
            if(bOK){

                KeyFrame* pKF = new KeyFrame(mCurrentFrame,nullptr,nullptr);

                vector<KeyFrame*> vNeighKFs;
                int map_point_nums = 0;
                set<int> connect_keys;
                int non_fixed_nums = 0;
                int total_edges_nums = 0;
                vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
                for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
                {
                    MapPoint* pMP = *vit;
                    if(pMP){
                        if(!pMP->isBad())
                        {
                            map_point_nums++;
                            map<KeyFrame*,size_t> observations = pMP->GetObservations();
                            for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFrame* pKFi = mit->first;
                                if(pKFi->mnId == pKF->mnId){
                                    continue;
                                }
                                if(!pKFi->isBad()){
                                    if(!pKFi->is_fixed){
                                        non_fixed_nums++;
                                    }
                                    total_edges_nums++;

                                    if(connect_keys.count(pKFi->mnId)==0){
                                        connect_keys.insert(pKFi->mnId);
                                        vNeighKFs.push_back(pKFi);
                                    }
                                }
                            }
                        }
                    }
                }
                std::ostringstream oss;
                for (auto view:vNeighKFs)
                {
                    oss<<view->mnId<< " ";
                }
                LOG_S(INFO)<<"CHECK pKF: "<<pKF->mnId<<" pMP: "<<map_point_nums<<" observations: "<<oss.str()<<" size: "<<connect_keys.size();
                LOG_S(INFO)<<"CHECK pKF: "<<pKF->mnId<<" non_fixed_nums: "<<non_fixed_nums<<" total_edges_nums: "<<total_edges_nums;

                Optimizer::LocalBundleAdjustment2(pKF, vNeighKFs);

                mCurrentFrame.SetPose(pKF->GetPose());
                mCurrentFrame.mvpMapPoints = pKF->GetMapPointMatches();
                delete pKF;

                if(!mLastFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mCurrentFrame.mTcw*LastTwc;
                }
                else
                    mVelocity = cv::Mat();

                mLastFrame = mCurrentFrame;

            }
        }


    }
    if(bOK) {
        mState = ORB_SLAM2::Tracking::eTrackingState::OK;
        LOG_S(INFO) << "[AddMissingView] Track status: OK";
    }
    else {
        mState = ORB_SLAM2::Tracking::eTrackingState::LOST;
        mCurrentFrame.mTcw = cv::Mat();
        LOG_S(INFO) << "[AddMissingView] Track status: LOST";
    }
    return bOK;
}


double computeDistance(const cv::Point3d &p1, const cv::Point3d &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

void removeOutliers(std::vector<cv::Point3d> &points, double threshold, int k)
{
    // k：每个点计算其最近的k个邻居
    std::vector<cv::Point3d> filteredPoints;
    for (size_t i = 0; i < points.size(); ++i)
    {
        std::vector<double> distances;

        // 计算每个点与其它点的距离
        for (size_t j = 0; j < points.size(); ++j)
        {
            if (i != j)
            {
                double dist = computeDistance(points[i], points[j]);
                distances.push_back(dist);
            }
        }

        // 对距离进行排序，取前k个最近的点
        std::sort(distances.begin(), distances.end());

        k = std::min((int)distances.size(), k);

        // 计算k个最近点的平均距离
        double meanDist = 0;
        for (int m = 0; m < k; ++m)
        {
            meanDist += distances[m];
        }
        meanDist /= k;

        // 判断当前点是否为离群点，若大于阈值则认为是离群点
        if (meanDist < threshold)
        {
            filteredPoints.push_back(points[i]);
        }
    }

    // 更新点云
    points = filteredPoints;
}

double computeAngleBetweenVectors(const cv::Point3d& v1, const cv::Point3d& v2) {
    // 计算点积
    double dotProduct = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

    // 计算向量的模
    double normV1 = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    double normV2 = std::sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);

    // 计算夹角的余弦值
    double cosTheta = dotProduct / (normV1 * normV2);

    // 防止浮点数精度问题导致 cosTheta 超出 [-1, 1] 范围
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

    // 计算夹角（角度）
    double angle = std::acos(cosTheta)*180.0f/CV_PI;
    
    return angle;
}

bool checkCameraSafe(int view_id, cv::Mat& Tgpsc, float angle_threshold) {
    cv::Point3d camera_position(Tgpsc.at<float>(0,3),Tgpsc.at<float>(1,3),Tgpsc.at<float>(2,3));
    cv::Point3d camera_orientation(Tgpsc.at<float>(0,2),Tgpsc.at<float>(1,2),Tgpsc.at<float>(2,2));
    cv::Point3d gravity(0,0,-1);
    double angle = computeAngleBetweenVectors(camera_orientation, gravity);
    LOG_S(INFO) << "output view_id: " << view_id << " angle: "<<angle<< " camera_orientation: "<<camera_orientation<< " camera_position: "<<camera_position;
    if(angle>angle_threshold){
        LOG_S(WARNING)<<"slam error, view_id: "<<view_id<< " angle: "<<angle;
        return false;
    }
    return true;
}

std::string saveReconstruction(std::string global_reconstruct_directory_, std::string cache_poses_directory_,
bool debug_, std::vector<PlyUtils::ColorPoint_t>& global_enu_points, 
int view_id, cv::Mat &pose, std::vector<double> &gps, std::vector<cv::Point3d> &points, std::string undist_image_path)
{
    std::string ply_path = global_reconstruct_directory_ + "/slam_" + std::to_string(view_id) + ".ply";
    std::string json_path = global_reconstruct_directory_ + "/slam_" + std::to_string(view_id) + ".json";

    std::vector<PlyUtils::ColorPoint_t> enu_points;
    for (auto &point : points)
    {
        PlyUtils::ColorPoint_t pt;
        pt.vertex[0] = point.x;
        pt.vertex[1] = point.y;
        pt.vertex[2] = point.z;
        pt.vcolor[0] = pt.vcolor[1] = pt.vcolor[2] = 255;
        enu_points.push_back(pt);
        if(debug_)global_enu_points.push_back(pt);
    }
    {
        cv::Mat Xgps = pose.rowRange(0, 3).col(3);
        PlyUtils::ColorPoint_t m_pose;
        m_pose.vertex[0] = Xgps.at<float>(0);
        m_pose.vertex[1] = Xgps.at<float>(1);
        m_pose.vertex[2] = Xgps.at<float>(2);
        m_pose.vcolor[2] = 255;
        m_pose.vcolor[0] = m_pose.vcolor[1] = 0;
        enu_points.push_back(m_pose);
        if(debug_)global_enu_points.push_back(m_pose);
    }
    {
        PlyUtils::ColorPoint_t m_pose;
        m_pose.vertex[0] = gps[0];
        m_pose.vertex[1] = gps[1];
        m_pose.vertex[2] = gps[2];
        m_pose.vcolor[1] = 255;
        m_pose.vcolor[2] = m_pose.vcolor[0] = 0;
        enu_points.push_back(m_pose);
        if(debug_)global_enu_points.push_back(m_pose);
    }
    PlyUtils::writePly(ply_path, enu_points, false);
    Json::Value json_gps;
    json_gps.append(gps[0]);
    json_gps.append(gps[1]);
    json_gps.append(gps[2]);

    Json::Value json;
    json["points_ply_path"] = ply_path;
    json["undist_image_path"] = undist_image_path;
    json["pose"] = convert_pose_to_json(pose);
    json["view_id"] = view_id;
    json["gps"] = json_gps;
    json["is_global"] = false;
    write_json(json_path, json);

    if(debug_){
        std::string _ply_path = cache_poses_directory_ + "/global_enu_points_and_pose.ply";
        LOG_S(INFO)<<"global_enu_points.size()"<<global_enu_points.size();
        PlyUtils::writePly(_ply_path, global_enu_points, false);
    }

    return json_path;
}

int main(int argc, char **argv)
{
    std::string voc_file;
    std::string workspace_directory;
    std::string output_directory;
    std::string task_filenames;
    int task_nums;
    bool use_global_optimization = true;
    bool use_pnp_densify = true;
    bool use_dom = true;
    bool save_colmap = false;
    

    CmdLine cmd;

    cmd.add(make_option(' ', workspace_directory, "workspace_directory"));
    cmd.add(make_option(' ', output_directory, "output_directory"));
    cmd.add(make_option(' ', task_filenames, "task_filenames"));
    cmd.add(make_option(' ', task_nums, "task_nums"));
    cmd.add(make_option(' ', voc_file, "voc_file"));
    cmd.add(make_option(' ', use_global_optimization, "use_global_optimization"));
    cmd.add(make_option(' ', use_pnp_densify, "use_pnp_densify"));
    cmd.add(make_option(' ', use_dom, "use_dom"));
    cmd.add(make_option(' ', save_colmap, "save_colmap"));


    try
    {
        if (argc == 1)
            throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    }
    catch (const std::string &s)
    {
        LOG_S(ERROR) << "Usage error! " << std::endl;
        return EXIT_FAILURE;
    }

    output_directory += "/";
    if (!stlplus::folder_exists(output_directory))
    {
        if (!stlplus::folder_create(output_directory))
        {
            LOG_F(ERROR, "Cannot create  output_directory");
            return false;
        }
    }

    std::string unified_coordinate_directory = output_directory +"/unified_coordinate/";
    if (!stlplus::folder_exists(unified_coordinate_directory))
    {
        if (!stlplus::folder_create(unified_coordinate_directory))
        {
            LOG_F(ERROR, "Cannot create  unified_coordinate_directory");
            return false;
        }
    }

    std::string _log_file = output_directory + "/realtime2d_merge.log";
    loguru::add_file(_log_file.c_str(), loguru::Truncate, loguru::Verbosity_MAX);

    std::vector<std::string> task_list;
    split(task_filenames,';',task_list);

    if(task_nums==0||task_list.size()==0){
        LOG_F(ERROR, "Cannot create map, task_nums == 0");
        return false;
    }

    LOG_S(INFO) << "voc_file: " << voc_file;
    LOG_S(INFO) << "workspace_directory: " << workspace_directory;
    LOG_S(INFO) << "output_directory: " << output_directory;

    LOG_S(INFO) << "task_filenames: " << task_filenames;
    LOG_S(INFO) << "task_nums: " << task_nums;
    LOG_S(INFO) << "use_global_optimization: " << use_global_optimization;
    LOG_S(INFO) << "use_pnp_densify: " << use_pnp_densify;
    LOG_S(INFO) << "use_dom: " << use_dom;
    LOG_S(INFO) << "save_colmap: " << save_colmap;

    LOG_S(INFO) << "task_list.size(): " << task_list.size();
    for(auto& task_name:task_list)
        LOG_S(INFO) << "task_name: " << task_name;
    
    std::string base_task = task_list[0];
    LOG_S(INFO) << "base_task: " << base_task;

    LOG_S(INFO) << "unified_coordinate_directory: " << unified_coordinate_directory;
    
    TaskKeyframesMapoints baseTask(voc_file,workspace_directory,base_task,unified_coordinate_directory,nullptr,nullptr,nullptr,save_colmap);

    double reference_latitude = baseTask.reference_latitude;
    double reference_longitude = baseTask.reference_longitude;
    double reference_altitude = baseTask.reference_altitude;
    ORBextractor* mpORBextractorLeft = baseTask.mpORBextractorLeft;
    ORBVocabulary* mpORBVocabulary = baseTask.mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase = baseTask.mpKeyFrameDatabase;


    #if 1

    #if 0
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = baseTask.GetAllKeyFrames();
    for (int i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* keyframe = vpKFs[i];

        cv::Mat image = cv::imread(keyframe->absPath);
        std::vector<MapPoint*> TrackedMapPoints = keyframe->GetMapPointMatches();
        std::vector<cv::KeyPoint> mvKeys;
        for(auto& pMP:TrackedMapPoints){
            if(pMP!=nullptr&&!pMP->isBad()){
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    if(mit->first->mnId==keyframe->mnId){
                        mvKeys.push_back(keyframe->mvKeys[mit->second]);
                        break;
                    }
                }
            }
        }
        cv::Mat featImage;
        cv::drawKeypoints(image, mvKeys, featImage, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("featImage",featImage);
        int key = cv::waitKey(0);
        if(key == 27){
            break;
        }
    }
    #endif

    vector<KeyFrame*> base_vpKFs = baseTask.GetAllKeyFrames();
    LOG_S(INFO) << "base_vpKFs: "<<base_vpKFs.size();

    if(task_list.size()>=2){
        for(int task_idx=1;task_idx<task_list.size();task_idx++){
            std::string task_id = task_list[task_idx];
            TaskKeyframesMapoints subTask(voc_file,workspace_directory,task_id,unified_coordinate_directory,mpORBextractorLeft,mpORBVocabulary,mpKeyFrameDatabase,save_colmap);

            std::vector<ORB_SLAM2::KeyFrame*> subTask_old_vpKFs;
            std::vector<ORB_SLAM2::KeyFrame*> subTask_new_vpKFs;

            #if 0
            std::vector<ORB_SLAM2::KeyFrame*> vpKFs = subTask.GetAllKeyFrames();
            for (int i = 0; i < vpKFs.size(); i++)
            {
                KeyFrame* keyframe = vpKFs[i];

                cv::Mat image = cv::imread(keyframe->absPath);
                std::vector<MapPoint*> TrackedMapPoints = keyframe->GetMapPointMatches();
                std::vector<cv::KeyPoint> mvKeys;
                for(auto& pMP:TrackedMapPoints){
                    if(pMP!=nullptr&&!pMP->isBad()){
                        map<KeyFrame*,size_t> observations = pMP->GetObservations();
                        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            if(mit->first->mnId==keyframe->mnId){
                                mvKeys.push_back(keyframe->mvKeys[mit->second]);
                                break;
                            }
                        }
                    }
                }
                cv::Mat featImage;
                cv::drawKeypoints(image, mvKeys, featImage, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DEFAULT);
                cv::imshow("featImage",featImage);
                int key = cv::waitKey(0);
                if(key == 27){
                    break;
                }
            }
            #endif

            std::vector<ORB_SLAM2::KeyFrame*> vpKFs = subTask.GetAllKeyFrames();
            cv::Mat mImGray;
            for (int kf_id = 0; kf_id < vpKFs.size(); kf_id++)
            {
                KeyFrame* keyframe = vpKFs[kf_id];
                Frame& mCurrentFrame = subTask.mpFrames[keyframe->mnId];

                KeyFrame* mpReferenceKF;
                Frame mLastFrame;
                cv::Mat mVelocity;
                ORB_SLAM2::Tracking::eTrackingState mState = ORB_SLAM2::Tracking::eTrackingState::LOST;
                auto& mTgpsc = subTask.mpRealPositions[keyframe->mnId];

                vector<KeyFrame*> vpCandidateKFs;
                std::vector<std::pair<KeyFrame*, double>> old_keys_arr;
                double min_distance = std::numeric_limits<double>::max();
                for(int i=0; i<base_vpKFs.size(); i++){
                    KeyFrame* pKF = base_vpKFs[i];
                    std::vector<double>& positon = baseTask.mpRealPositions[pKF->mnId];
                    double distance = std::sqrt(
                        std::pow(positon[0]-mTgpsc[0],2) +
                        std::pow(positon[1]-mTgpsc[1],2) +
                        std::pow(positon[2]-mTgpsc[2],2));
                    old_keys_arr.push_back(std::make_pair(pKF, distance));
                    min_distance = std::min(min_distance, distance);
                }
                if(min_distance>50){
                    LOG_S(INFO)<<"subTask: "<<subTask.task_id<<" keyframe: "<<keyframe->mnId<<" out of map, min_distance: "<<min_distance;
                    continue;
                }

                std::sort(old_keys_arr.begin(), old_keys_arr.end(),
                            [](const std::pair<KeyFrame*, double> &f1, const std::pair<KeyFrame*, double> &f2) {
                            return f1.second < f2.second;
                            });

                int neibor_key_nums = (old_keys_arr.size()<10)?old_keys_arr.size():10;
                for(int i=0; i<neibor_key_nums; i++){
                    KeyFrame* pKF = old_keys_arr[i].first;
                    // LOG_S(INFO) << "[old_keys_arr] pKF: "<<pKF->mnId <<" distance: "<<old_keys_arr[i].second;
                    vpCandidateKFs.push_back(pKF);
                    #if 0
                    cv::Mat left_image = cv::imread(mCurrentFrame.mAbsPath);
                    cv::Mat right_image = cv::imread(pKF->absPath);
                    cv::Mat matchedImage;
                    cv::hconcat(left_image, right_image, matchedImage);
                    cv::namedWindow("matchedImage",cv::WINDOW_NORMAL);
                    cv::imshow("matchedImage",matchedImage);
                    int key = cv::waitKey(0);
                    if(key == 27){
                        exit(-1);
                    }
                    #endif
                }

                LOG_S(INFO) <<"keyframe: "<<keyframe->mnId<< " vpCandidateKFs: "<<vpCandidateKFs.size();

                bool bOK = AddMissingView(mCurrentFrame, vpCandidateKFs, false, &mpReferenceKF,mLastFrame,mVelocity,mState);
                if(bOK){
                    subTask_old_vpKFs.push_back(keyframe);
                    KeyFrame* pKF = new KeyFrame(mCurrentFrame,nullptr,nullptr);
                    subTask_new_vpKFs.push_back(pKF);
                }

            }

            LOG_S(INFO)<<"task_id: "<<task_id<<" subTask_in_vpKFs: "<<subTask_old_vpKFs.size();
            LOG_S(INFO)<<"task_id: "<<task_id<<" subTask_new_vpKFs: "<<subTask_new_vpKFs.size();

            cv::Mat subtask_Tgps_from_w;
            float subtask_scale;
            Optimizer::AlignKeyframesUmeyama(subTask_old_vpKFs, subTask_new_vpKFs, subtask_Tgps_from_w, subtask_scale);
            LOG_S(INFO) << "subtask_Tgps_from_w: " << subtask_Tgps_from_w;
            LOG_S(INFO) << "subtask_scale: " << subtask_scale;


            // 缓存 旧的关键帧的指针 和 新的关键帧的指针 的对应关系
            std::vector<std::pair<KeyFrame*,KeyFrame*>> KeyFrameMap;
            for (int kf_id = 0; kf_id < vpKFs.size(); kf_id++)
            {
                KeyFrame* keyframe = vpKFs[kf_id];
                // LOG_S(INFO)<<"keyframe: "<<keyframe->mnId;
                bool is_new = false;
                for(auto&ptr:subTask_old_vpKFs){
                    if(ptr->mnId == keyframe->mnId){
                        is_new = true;
                        break;
                    }
                }
                // LOG_S(INFO)<<"is_new: "<<is_new;

                Frame& mCurrentFrame = subTask.mpFrames[keyframe->mnId];
                // LOG_S(INFO)<<"1 mCurrentFrame: "<<mCurrentFrame.mnId;

                cv::Mat Tcw1 = keyframe->GetPose();

                cv::Mat Rcw1 = Tcw1.rowRange(0,3).colRange(0,3);
                cv::Mat tcw1 = Tcw1.rowRange(0,3).col(3);

                cv::Mat Rw1c = Rcw1.t();
                cv::Mat Ow1 = -Rw1c*tcw1;// 世界坐标系下相机的中心位置。

                cv::Mat Rw2c = subtask_Tgps_from_w.rowRange(0,3).colRange(0,3) / subtask_scale * Rw1c;
                cv::Mat Ow2 = subtask_Tgps_from_w.rowRange(0,3).colRange(0,3) * Ow1 + subtask_Tgps_from_w.rowRange(0,3).col(3);
   
                cv::Mat Rcw2 = Rw2c.t();
                cv::Mat tcw2 = -Rcw2*Ow2;// 世界坐标系下相机的中心位置。

                cv::Mat Tcw2 = cv::Mat::eye(4, 4, CV_32F);
                Rcw2.copyTo(Tcw2.rowRange(0,3).colRange(0,3));
                tcw2.copyTo(Tcw2.rowRange(0,3).col(3));

                mCurrentFrame.mTgps_from_w = baseTask.Tgps_from_w;
                mCurrentFrame.mScale = baseTask.scale;
                mCurrentFrame.SetPose(Tcw2);

                std::vector<double>& lla = subTask.mpLLAs[keyframe->mnId];
                Eigen::Vector3d _gps_position = xagmapper::geodesy::topocentric_from_lla(lla[0], lla[1], lla[2],
                                                                                         reference_latitude, reference_longitude, reference_altitude);  

                if(mCurrentFrame.mTgpsc.empty()){
                    mCurrentFrame.mTgpsc = cv::Mat::eye(4, 4, CV_32F);
                }
                if(mCurrentFrame.gps.empty()){
                    mCurrentFrame.gps = {0,0,0};
                }
                for (int k = 0; k < 3; k++) {
                    mCurrentFrame.mTgpsc.at<float>(k, 3) = static_cast<float>(_gps_position[k]);
                    mCurrentFrame.gps[k] = _gps_position[k];
                }

                if(!is_new){
                    for(size_t idx = 0;idx<mCurrentFrame.mvpMapPoints.size();idx++){
                        mCurrentFrame.mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
                    }
                }

                KeyFrame* pKF = baseTask.AddKeyFrame(mCurrentFrame,lla);
                if(is_new){
                    std::vector<MapPoint*> TrackedMapPoints = pKF->GetMapPointMatches();
                    for(size_t idx = 0;idx<TrackedMapPoints.size();idx++){
                        ORB_SLAM2::MapPoint* pMP = TrackedMapPoints[idx];
                        if(pMP!=nullptr&&!pMP->isBad()){
                            pMP->AddObservation(pKF,idx);
                        }
                    }
                }
                KeyFrameMap.push_back(std::make_pair(keyframe,pKF));
            }


            std::vector<ORB_SLAM2::MapPoint*> vpMP = subTask.GetAllMapPoints();
            for(int i=0; i<vpMP.size(); i++)
            {
                ORB_SLAM2::MapPoint* pMP = vpMP[i];
                if(pMP->isBad())
                    continue;
                cv::Mat posw1 = pMP->GetWorldPos();
                cv::Mat posw2 = subtask_Tgps_from_w.rowRange(0,3).colRange(0,3) * posw1 + subtask_Tgps_from_w.rowRange(0,3).col(3);
                map<KeyFrame*,size_t> observations = pMP->GetObservations();

                map<KeyFrame*,size_t> new_observations;
                for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    for(auto&pair:KeyFrameMap){
                        if(pair.first->mnId == mit->first->mnId){
                            new_observations[pair.second]=mit->second;
                            break;
                        }
                    }
                }
                KeyFrame* mpRefKF;
                for(auto&pair:KeyFrameMap){
                    if(pair.first->mnId == pMP->GetReferenceKeyFrame()->mnId){
                        mpRefKF = pair.second;
                        break;
                    }
                }
                MapPoint* new_pMP = baseTask.AddMappoint(posw2,mpRefKF,new_observations);
            }

            //清除subTask_new_vpKFs中指针
            for(auto&ptr:subTask_new_vpKFs)delete ptr;

        }
    }

    LOG_S(INFO)<<"before global optimization";
    saveTaskKeyframesMapoints(baseTask, unified_coordinate_directory);
    LOG_S(INFO)<<"done saveTaskKeyframesMapoints";
    #endif



    if(use_global_optimization){
            
        std::string global_optimization_directory = output_directory + "/global_optimization/";
        if (!stlplus::folder_exists(global_optimization_directory))
        {
            if (!stlplus::folder_create(global_optimization_directory))
            {
                LOG_F(ERROR, "Cannot create  global_optimization_directory");
                return false;
            }
        }

        std::vector<ORB_SLAM2::KeyFrame*> vpKFs = baseTask.GetAllKeyFrames();
        std::vector<ORB_SLAM2::MapPoint*> vpMP = baseTask.GetAllMapPoints();

        std::string project_json_file = global_optimization_directory + "/output.json";
        Json::Value keyframes;
        // 记录添加到优化器中的顶点的最大关键帧id
        long unsigned int maxKFid = 0;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            // 跳过无效关键帧
            if(pKF->isBad())
                continue;
            Json::Value keyframe;
            keyframe["mnId"] = pKF->mnId;
            keyframe["mnFrameId"] = pKF->mnFrameId;
            keyframe["Tcw"] = convert_pose_to_json(pKF->GetPose());
            keyframe["gps"] = convert_position_to_json(pKF->GetGPSPosition());
            keyframe["Tgps_from_w"] = convert_pose_to_json(pKF->Tgps_from_w);
            keyframe["scale"] = pKF->scale;
            // keyframes[std::to_string(pKF->mnId)] = keyframe;
            keyframes.append(keyframe);
            if(pKF->mnId>maxKFid)
                maxKFid=pKF->mnId;
        }

        Json::Value landmarks;
        for(size_t i=0; i<vpMP.size(); i++)
        {
            MapPoint* pMP = vpMP[i];
            // 跳过无效地图点
            if(pMP->isBad())
                continue;
            Json::Value landmark;
            landmark["mnId"] = pMP->mnId;
            landmark["mWorldPos"] = convert_position_to_json(pMP->GetWorldPos());
            // 取出地图点和关键帧之间观测的关系
            const map<KeyFrame*,size_t> observations = pMP->GetObservations();
            Json::Value observations_json;
            for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
            {
                
                KeyFrame* pKF = mit->first;
                // 跳过不合法的关键帧
                if(pKF->isBad() || pKF->mnId>maxKFid )
                    continue;
                Json::Value observation_json;

                // 取出该地图点对应该关键帧的2D特征点
                // const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
                const cv::KeyPoint &kpUn = pKF->mvKeys[mit->second];//未去畸变的点

                Json::Value obs;
                obs.append(kpUn.pt.x);
                obs.append(kpUn.pt.y);
                observation_json["kpUn"] = obs;
                observation_json["mnId"] = pKF->mnId;
                if(pKF->mvuRight[mit->second]<0)
                {
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    observation_json["invSigma2"] = invSigma2;
                }
                observations_json.append(observation_json);
            }
            landmark["observations"] = observations_json;
            // landmarks[std::to_string(pMP->mnId)] = landmark;
            landmarks.append(landmark);
        }
        Json::Value json;
        json["keyframes"] = keyframes;
        json["landmarks"] = landmarks;
        write_json(project_json_file, json);



        std::vector<double> vec_params;
        double fx = baseTask.mK.at<float>(0, 0);
        double fy = baseTask.mK.at<float>(1, 1);
        double cx = baseTask.mK.at<float>(0, 2);
        double cy = baseTask.mK.at<float>(1, 2);
        double k1 = baseTask.mDistCoef.at<float>(0);
        double k2 = baseTask.mDistCoef.at<float>(1);
        double p1 = baseTask.mDistCoef.at<float>(2);
        double p2 = baseTask.mDistCoef.at<float>(3);
        double k3 = baseTask.mDistCoef.at<float>(4);
        vec_params.push_back(fx);
        vec_params.push_back(fy);
        vec_params.push_back(cx);
        vec_params.push_back(cy);
        vec_params.push_back(k1);
        vec_params.push_back(k2);
        vec_params.push_back(p1);
        vec_params.push_back(p2);
        vec_params.push_back(k3);

        int nIterations = 10;
        int nOptimizations = 100;
        float weight = 5;
        float weightgps = 800000;
        bool unary = false; // true为单边，false为双边
        bool realscale = false;
        bool computesim3 = true;
        double before_rmse;
        double final_rmse;
        Optimizer::GlobalBundleAdjustmentIntrinsicsDistortion(project_json_file, vec_params, global_optimization_directory,
        before_rmse, final_rmse,
        nIterations, nOptimizations, weight, weightgps, unary, realscale, computesim3, false, nullptr,baseTask.mpMap,nullptr);

        baseTask.Tgps_from_w = baseTask.mpMap->mTgps_from_w;
        baseTask.scale = baseTask.mpMap->mScale;
        baseTask.mK.at<float>(0, 0) = vec_params[0];
        baseTask.mK.at<float>(1, 1) = vec_params[1];
        baseTask.mK.at<float>(0, 2) = vec_params[2];
        baseTask.mK.at<float>(1, 2) = vec_params[3];
        baseTask.mDistCoef.at<float>(0) = vec_params[4];
        baseTask.mDistCoef.at<float>(1) = vec_params[5];
        baseTask.mDistCoef.at<float>(2) = vec_params[6];
        baseTask.mDistCoef.at<float>(3) = vec_params[7];
        baseTask.mDistCoef.at<float>(4) = vec_params[8];

        LOG_S(INFO)<<"after global optimization";
        saveTaskKeyframesMapoints(baseTask, unified_coordinate_directory,"after_opt");
        LOG_S(INFO)<<"done saveTaskKeyframesMapoints";

    }


    if(use_pnp_densify)
    {
        std::unordered_map<std::string, std::unordered_set<std::string>> folder_files_mapping;
        std::unordered_map<std::string, std::unordered_map<std::string, ORB_SLAM2::KeyFrame*>> folder_keyframes_mapping;
        std::vector<ORB_SLAM2::KeyFrame*> vpKFs = baseTask.GetAllKeyFrames();
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            pKF->is_fixed  = true;
            // 跳过无效关键帧
            if(pKF->isBad())
                continue;
            std::string folder = stlplus::folder_part(pKF->absPath);
            std::string base_name = stlplus::basename_part(pKF->absPath);
            if(folder_files_mapping.count(folder)==0){
                // folders.insert(folder);
                std::unordered_set<std::string> set;
                set.insert(stlplus::basename_part(pKF->absPath));
                folder_files_mapping[folder] = set;
            }
            else{
                std::unordered_set<std::string>& set = folder_files_mapping.at(folder);
                set.insert(stlplus::basename_part(pKF->absPath));
            }
            if(folder_keyframes_mapping.count(folder)==0){
                std::unordered_map<std::string, ORB_SLAM2::KeyFrame*> map;
                map[base_name] = pKF;
                folder_keyframes_mapping[folder] = map;
            }
            else{
                std::unordered_map<std::string, ORB_SLAM2::KeyFrame*>& map = folder_keyframes_mapping.at(folder);
                map[base_name] = pKF;
            }
        }
        for(auto&folder:folder_files_mapping){
            std::string images_directory = folder.first;
            LOG_S(INFO)<<"images_directory: "<<images_directory;
            std::vector<std::string> _files = stlplus::folder_files(images_directory);
            std::vector<std::string> files_;
            for (auto file : _files)
            {
                std::string ext = stlplus::extension_part(file);
                if (ext == "JPG" || ext == "jpg" || ext == "jpeg")
                {
                    files_.push_back(file);
                }
            }
            if (files_.size() == 0)
            {
                LOG_F(ERROR, "images not found!");
                continue;
            }
            std::map<std::string, long> timestamp_map;
            for (int i = 0; i < files_.size(); i++)
            {
                std::string file = files_[i];
                std::string basename = stlplus::basename_part(file);
                timestamp_map[file] = atof(basename.c_str());
            }
            int total_images_ = files_.size();
            std::sort(files_.begin(), files_.end(),
                    [timestamp_map](std::string &a, std::string &b)
                    {
                        return timestamp_map[a] < timestamp_map[b];
                    });

            Frame mLastFrame;
            KeyFrame* mpReferenceKF = nullptr;
            cv::Mat mVelocity;
            ORB_SLAM2::Tracking::eTrackingState mState = Tracking::eTrackingState::LOST;
            for (int ni = 0; ni < total_images_; ni++)
            {
                std::string image_path = images_directory + "/" + files_[ni];
                LOG_S(INFO) << "image_path: " << image_path << std::endl;
                std::string base_name = stlplus::basename_part(image_path);
                if(folder.second.count(base_name)==0){
                    LOG_S(INFO)<<base_name<<" is non-keyframe";
                    if(mState == Tracking::eTrackingState::LOST){
                        continue;
                    }

                    cv::Mat loadImg = cv::imread(image_path,-1);
                    if(loadImg.empty()){
                        continue;
                    }
                    
                    std::string lla_path = images_directory + "/" + stlplus::basename_part(files_[ni]) + ".txt";
                    std::vector<double> lla;
                    bool is_found = ImportLLA(lla, lla_path);
                    if(!is_found){
                        continue;
                    }

                    Eigen::Vector3d _gps_position = xagmapper::geodesy::topocentric_from_lla(lla[0], lla[1], lla[2],
                                                                                            reference_latitude, reference_longitude, reference_altitude);  
                    std::vector<double> gps = {_gps_position[0], _gps_position[1], _gps_position[2]};

                    cv::Mat mImGray;
                    cv::cvtColor(loadImg,mImGray,CV_BGR2GRAY);
                    Frame mCurrentFrame = Frame(mImGray, loadImg, timestamp_map[base_name], mpORBextractorLeft, mpORBVocabulary, baseTask.mK, baseTask.mDistCoef, baseTask.mbf, baseTask.mThDepth, gps, baseTask.Tgps_from_w, baseTask.scale, image_path, ""); // scale


                    vector<KeyFrame* > old_vpKFs;
                    for(int i=0; i<vpKFs.size(); i++)
                    {
                        KeyFrame* pKF = vpKFs[i];
                        if(!pKF->isBad()){
                            if(pKF->is_fixed){
                                old_vpKFs.push_back(pKF);
                            }
                        }
                    }
                    LOG_S(INFO) << "old_vpKFs: "<<old_vpKFs.size();



                    vector<KeyFrame*> vpCandidateKFs;
                    std::vector<std::pair<KeyFrame*, double>> old_keys_arr;
                    for(int i=0; i<old_vpKFs.size(); i++){
                        KeyFrame* pKF = old_vpKFs[i];
                        cv::Mat positon = pKF->Tgpsc.rowRange(0, 3).col(3);
                        double distance = std::sqrt(
                            std::pow(positon.at<float>(0)-mCurrentFrame.mTgpsc.at<float>(0, 3),2)+
                            std::pow(positon.at<float>(1)-mCurrentFrame.mTgpsc.at<float>(1, 3),2)+
                            std::pow(positon.at<float>(2)-mCurrentFrame.mTgpsc.at<float>(2, 3),2));
                        old_keys_arr.push_back(std::make_pair(pKF, distance));
                    }
                    std::sort(old_keys_arr.begin(), old_keys_arr.end(),
                                [](const std::pair<KeyFrame*, double> &f1, const std::pair<KeyFrame*, double> &f2) {
                                return f1.second < f2.second;
                                });
                    
                    int neibor_key_nums = (old_keys_arr.size()<10)?old_keys_arr.size():10;
                    for(int i=0; i<neibor_key_nums; i++){
                        KeyFrame* pKF = old_keys_arr[i].first;
                        // LOG_S(INFO) << "[old_keys_arr] pKF: "<<pKF->mnId <<" distance: "<<old_keys_arr[i].second;
                        vpCandidateKFs.push_back(pKF);
                    }

                    LOG_S(INFO) << "vpCandidateKFs: "<<vpCandidateKFs.size();

                    bool bOK = AddMissingView(mCurrentFrame, vpCandidateKFs, true, &mpReferenceKF,mLastFrame,mVelocity,mState);
                    if(bOK){
                        KeyFrame* pKF = baseTask.AddKeyFrame(mCurrentFrame,lla);
                        LOG_S(INFO)<<"[match] CHECK pKF: "<<pKF->mnId<< " mCurrentFrame: "<<mCurrentFrame.mnId;
                    }
                }
                else{
                    LOG_S(INFO)<<base_name<<" is keyframe";

                    KeyFrame* pKF = nullptr;
                    if(folder_keyframes_mapping[folder.first].count(base_name)!=0){
                        pKF = folder_keyframes_mapping[folder.first].at(base_name);
                    }
                    if(pKF!=nullptr){
                        LOG_S(INFO)<<"absPath: "<<pKF->absPath;

                        cv::Mat mTcw = pKF->GetPose();

                        if(mState == Tracking::eTrackingState::OK){
                            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                            mVelocity = mTcw*LastTwc;
                        }
                        mpReferenceKF = pKF;
                        mLastFrame = baseTask.mpFrames[pKF->mnId];
                        mLastFrame.SetPose(mTcw);
                        mLastFrame.mvpMapPoints = pKF->GetMapPointMatches();
                        mState = Tracking::eTrackingState::OK;
                    }
                }
            }
        }


        LOG_S(INFO)<<"after pnp";
        saveTaskKeyframesMapoints(baseTask, unified_coordinate_directory, "after_pnp");
        LOG_S(INFO)<<"done saveTaskKeyframesMapoints";
    }

    if(save_colmap){
        cv::Mat Tgpsw = baseTask.Tgps_from_w;
        float scale = baseTask.scale;
        cv::Mat srcK = baseTask.mK;
        const std::vector<ORB_SLAM2::KeyFrame*> allKF = baseTask.GetAllKeyFrames();
        const std::vector<ORB_SLAM2::MapPoint*> allMapPts = baseTask.GetAllMapPoints();

        std::string points3DPath = unified_coordinate_directory + "/points3D.txt";
        std::string imgsPath = unified_coordinate_directory + "/images.txt";
        std::string camsPath = unified_coordinate_directory + "/cameras.txt";

        std::ofstream outp3d(points3DPath.c_str());
        std::ofstream outimgs(imgsPath.c_str());
        std::ofstream outcams(camsPath.c_str());

        outp3d << "# 3D point list with one line of data per point: \n#  POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX) \n";
        outimgs << "# Image list with two lines of data per image:\n#  IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME \n#  POINTS2D[] as (X, Y, POINT3D_ID)\n";
        outcams << "# Camera list with one line of data per camera:\n# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n# Number of cameras: 1\n";
        for(int i = 0;i<allMapPts.size();i++)
        {
            MapPoint* mpt = allMapPts[i];
            if (mpt->isBad())
                continue;
            
            const map<KeyFrame*,size_t> observations = mpt->GetObservations();
            bool isSetrgb = false;
            for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame *pKF = mit->first;
                // 跳过无效关键帧
                if (pKF->isBad() )
                    continue;

                const cv::KeyPoint &kp = pKF->mvKeys[mit->second];// 未去畸变的点
                if(!isSetrgb)
                {
                    outp3d << mpt->mnId << " "; // id
                    cv::Mat worldPose = mpt->GetWorldPos();
                    cv::Mat realworldPose = Tgpsw.rowRange(0, 3).colRange(0, 3) * worldPose + Tgpsw.rowRange(0, 3).col(3);
                    outp3d << realworldPose.at<float>(0, 0) << " " << realworldPose.at<float>(1, 0) << " " << realworldPose.at<float>(2, 0) << " "; // xyz
                    
                    cv::Vec3b RGB = getRGB(pKF->absPath, kp);
                    outp3d << (int)RGB[0] << " " << (int)RGB[1] << " " << (int)RGB[2] << " "; // rgb
                    outp3d << 0 <<" "; // error
                    isSetrgb = true;
                }
                outp3d << pKF->mnId <<" "; //imgid
                outp3d << mit->second <<" ";// ptid
            }
            if(!mpt->isBad())
                outp3d << "\n";
        }
        outp3d.close();
        LOG_S(INFO)<<"save imgs.txt allKF.size() = "<<allKF.size();
        outimgs<<"# Number of images: "<<allKF.size()<< ",mean observations per image: "<<allMapPts.size()/allKF.size()<<"\n";
        for (int i = 0; i < allKF.size(); i++)
        {
            KeyFrame* pKF = allKF[i];
            // 跳过无效关键帧
            if (pKF->isBad())// || pKF->absPath.empty())
                continue;
            outimgs << pKF->mnId << " ";//imgid
            cv::Mat Tcw = pKF->GetPose();
            cv::Mat Tgpsc = Tcw2Tgpsc(Tcw, Tgpsw, scale);

            cv::Mat Rgspcs = Tgpsc.rowRange(0, 3).colRange(0, 3);
            cv::Mat tgpsc = Tgpsc.rowRange(0, 3).col(3);
            cv::Mat Rcgps = Rgspcs.t();
            cv::Mat Ow = -Rcgps * tgpsc; 
            cv::Mat Tcgps = cv::Mat::eye(4, 4, Tgpsc.type());
            Rcgps.copyTo(Tcgps.rowRange(0, 3).colRange(0, 3));
            Ow.copyTo(Tcgps.rowRange(0, 3).col(3));

            Eigen::Quaterniond quaternion;
            Eigen::Vector3d translation;
            MatPose2EigenQt(Tcgps, quaternion, translation);
            outimgs << quaternion.w()<< " "<< quaternion.x()<< " "<< quaternion.y()<< " "<< quaternion.z()<< " ";//q
            outimgs << translation.x() << " "<< translation.y() << " "<<translation.z() << " ";//xyz
            outimgs << 1 << " ";//cam id
            string abspath = pKF->absPath;
            // vector<string> vecStrpath = splitStr(abspath,"//");
            std::string file_name = stlplus::basename_part(abspath) + "." + stlplus::extension_part(abspath);
        
            // outimgs << vecStrpath[1] << "\n"; //imgname
            outimgs << file_name << "\n"; //imgname
            const int N = pKF->N;
            std::vector<ORB_SLAM2::MapPoint *> mappt = pKF->GetMapPointMatches();//每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
            for(int j = 0;j<N;j++)
            {
                MapPoint* pMP = mappt[j];
                if(pMP)
                {
                    const cv::KeyPoint &unkp = pKF->mvKeysUn[j];
                    outimgs << unkp.pt.x << " "<< unkp.pt.y << " "<< pMP->mnId<< " ";
                }
            }
            outimgs <<"\n";
        }
        outimgs.close();
        
        outcams << "1"<< " ";//cam id
        outcams << "PINHOLE" << " ";// model
        outcams << baseTask.mImGray.cols <<" " <<baseTask.mImGray.rows<< " " 
                << srcK.at<float>(0,0)<< " "<<srcK.at<float>(1,1)<< " "
                << srcK.at<float>(0,2)<< " "<<srcK.at<float>(1,2)<< " ";
        outcams.close();
    }

    if(use_dom){


        std::shared_ptr<realtime2d::openPackageData> packageddata_;
        packageddata_.reset(new realtime2d::openPackageData());
        packageddata_->setSLAMFinish(false);
        packageddata_->setDOMFinish(false);

        CameraSensorData camera_sensor_data;
        std::string yaml_path = output_directory + "/camera.yaml";
        camera_sensor_data.camera_type = baseTask.mCameraType;
        camera_sensor_data.fx = baseTask.mK.at<float>(0,0);
        camera_sensor_data.fy = baseTask.mK.at<float>(1,1);
        camera_sensor_data.cx = baseTask.mK.at<float>(0,2);
        camera_sensor_data.cy = baseTask.mK.at<float>(1,2);
        camera_sensor_data.k1 = baseTask.mDistCoef.at<float>(0);
        camera_sensor_data.k2 = baseTask.mDistCoef.at<float>(1);
        camera_sensor_data.p1 = baseTask.mDistCoef.at<float>(2);
        camera_sensor_data.p2 = baseTask.mDistCoef.at<float>(3);
        camera_sensor_data.k3 = baseTask.mDistCoef.at<float>(4);
        camera_sensor_data.export_yaml(yaml_path);
        LOG_S(INFO) << "yaml_path: " << yaml_path;

        bool debug_ = true;
        int tile_level = 20;
        double gsd = 0.15;
        int min_tile_count = 30;
        bool generate_dom_tiles = true;
        bool upload = true;
        bool remove_points_outliers_ = true;
        double points_outliers_threshold_ = 30.0;
        std::vector<std::pair<double, double>> wkt_extent;
        std::string dom_output_directory = output_directory;

        std::shared_ptr<realtime2d::openDOM> realtime2d_dom_;
        realtime2d_dom_.reset(new realtime2d::openDOM(debug_));
        realtime2d_dom_->setOutputDirectory(dom_output_directory);
        realtime2d_dom_->setPackageData(packageddata_);
        realtime2d_dom_->setYamlPath(yaml_path);
        realtime2d_dom_->setTileLevel(tile_level);
        realtime2d_dom_->setGSD(gsd);
        realtime2d_dom_->setUpdateFrequency(min_tile_count);
        realtime2d_dom_->setGenTiles(generate_dom_tiles);
        if(!wkt_extent.empty()){
            realtime2d_dom_->setWktExtent(wkt_extent);
        }
        realtime2d_dom_->initDOM();
        {
            std::string src_path = workspace_directory + "/" + baseTask.task_id + "/realtime2d_init/proj.txt";
            std::string dst_path = dom_output_directory + "/realtime2d_init/";
            std::string _tmp_output = "cp " + src_path + " "+ dst_path;
            std::system(_tmp_output.c_str());
        }
        {
            std::string src_path = workspace_directory + "/" + baseTask.task_id + "/realtime2d_init/trans_matrix.txt";
            std::string dst_path = dom_output_directory + "/realtime2d_init/";
            std::string _tmp_output = "cp " + src_path + " "+ dst_path;
            std::system(_tmp_output.c_str());
        }


        realtime2d_dom_->setUploadGlobalDOM(upload);
        realtime2d_dom_->startWork();


        std::string undistort_directory_ = output_directory + "realtime2d_undistort/";
        if (!stlplus::folder_exists(undistort_directory_))
        {
            if (!stlplus::folder_create(undistort_directory_))
            {
                LOG_F(ERROR, "Cannot create  undistort_directory_");
                return false;
            }
        }
        std::string global_reconstruct_directory_ = output_directory + "global_reconstruct/";
        if (!stlplus::folder_exists(global_reconstruct_directory_))
        {
            if (!stlplus::folder_create(global_reconstruct_directory_))
            {
                LOG_F(ERROR, "Cannot create  global_reconstruct_directory_");
                return false;
            }
        }
        std::string cache_poses_directory_ = output_directory + "realtime2d_cache_poses/";
        if (!stlplus::folder_exists(cache_poses_directory_))
        {
            if (!stlplus::folder_create(cache_poses_directory_))
            {
                LOG_F(ERROR, "Cannot create  cache_poses_directory_");
                return false;
            }
        }
        cv::Mat Tgps_from_w = baseTask.Tgps_from_w;
        float scale = baseTask.scale;
        cv::Mat K_ = baseTask.mK;
        std::vector<double> distCoeffs_ = {camera_sensor_data.k1, camera_sensor_data.k2, camera_sensor_data.p1, camera_sensor_data.p2,camera_sensor_data.k3};
        
        std::vector<std::vector<ORB_SLAM2::KeyFrame*>> folder_kfs_arr;
        {
            std::vector<ORB_SLAM2::KeyFrame*> vpKFs = baseTask.GetAllKeyFrames();

            std::map<std::string, std::vector<ORB_SLAM2::KeyFrame*>> folder_kfs_mapping;
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                // 跳过无效关键帧
                if(pKF->isBad())
                    continue;
                std::string folder = stlplus::folder_part(pKF->absPath);
                if(folder_kfs_mapping.count(folder)==0){
                    std::vector<ORB_SLAM2::KeyFrame*> set;
                    set.push_back(pKF);
                    folder_kfs_mapping[folder] = set;
                }
                else{
                    std::vector<ORB_SLAM2::KeyFrame*>& set = folder_kfs_mapping.at(folder);
                    set.push_back(pKF);
                }
            }
            
            for(auto iter = folder_kfs_mapping.begin();iter!=folder_kfs_mapping.end();iter++)
            {
                LOG_S(INFO)<<"folder_kfs_mapping, folder: "<< iter->first<<" size: "<<iter->second.size();
                folder_kfs_arr.push_back(iter->second);
            }
        }

        int num_procs = folder_kfs_arr.size();
        #pragma omp parallel for num_threads(num_procs)
        for (int y = 0; y < num_procs; y++)
        {
            std::vector<ORB_SLAM2::KeyFrame*> vpKFs = folder_kfs_arr[y];

            std::vector<double> RMSE;
            std::vector<PlyUtils::ColorPoint_t> global_enu_points;
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                // 跳过无效关键帧
                if(pKF->isBad())
                    continue;
                cv::Mat Tcw = pKF->GetPose();
                if(!Tcw.empty()){
                    
                    std::vector<MapPoint*> TrackedMapPoints = pKF->GetMapPointMatches();
                    std::vector<cv::Point3d> ptsGps;
                    for(auto& pMP:TrackedMapPoints){
                        if(pMP!=nullptr&&!pMP->isBad()){
                            cv::Mat x3D = pMP->GetWorldPos();
                            cv::Mat tgps = Tgps_from_w.rowRange(0,3).colRange(0,3) * x3D + Tgps_from_w.rowRange(0,3).col(3);

                            float x = tgps.at<float>(0,0);
                            float y = tgps.at<float>(1,0);
                            float z = tgps.at<float>(2,0);
                            ptsGps.push_back(cv::Point3d(x, y, z));
                        }
                    }
                    if(ptsGps.size()>0){
                        if (remove_points_outliers_)
                        {
                            LOG_S(INFO) << "before remove points: " << ptsGps.size();
                            removeOutliers(ptsGps, points_outliers_threshold_,20); // 调用函数剔除离群点
                            LOG_S(INFO) << "after remove points: " << ptsGps.size();
                        }
                        if(ptsGps.size()>0){

                            cv::Mat image = cv::imread(pKF->absPath);
                            std::string undist_image_path = undistort_directory_ + "/undistort_" + std::to_string(pKF->mnId) + ".png";
                            cv::Mat undistort_image;
                            cv::undistort(image, undistort_image, K_, distCoeffs_);
                            cv::imwrite(undist_image_path, undistort_image);
                            LOG_S(WARNING) << "export undistort_image, view_id: " << pKF->mnId << " " << undist_image_path;
                            
                            cv::Mat gps = pKF->GetGPSPosition();
                            vector<double> t_gps = {gps.at<float>(0), gps.at<float>(1), gps.at<float>(2)};

                            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
                            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
                            cv::Mat Rwc = Rcw.t();
                            cv::Mat Ow = -Rwc*tcw;// 世界坐标系下相机的中心位置。

                            cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
                            Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
                            Ow.copyTo(Twc.rowRange(0,3).col(3));

                            cv::Mat Rgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) / scale * Rwc;
                            cv::Mat tgpsc = Tgps_from_w.rowRange(0,3).colRange(0,3) * Ow + Tgps_from_w.rowRange(0,3).col(3);
                            cv::Mat Tgpsc = cv::Mat::eye(4, 4, CV_32F); // Tgpsc
                            Rgpsc.copyTo(Tgpsc.rowRange(0,3).colRange(0,3));
                            tgpsc.copyTo(Tgpsc.rowRange(0,3).col(3));

                            if(debug_){
                                calRMSE(Tgpsc,t_gps,RMSE,true);
                            }

                            if (checkCameraSafe(pKF->mnId, Tgpsc, 10))
                            {
                                std::string _path = saveReconstruction(
                                    global_reconstruct_directory_,cache_poses_directory_,debug_, global_enu_points,
                                    pKF->mnId, Tgpsc, t_gps, ptsGps, undist_image_path);
                                packageddata_->addFlyPose(_path);
                                usleep(1000);
                            }
                        }
                    }
                }
            }
        }
        
        packageddata_->setSLAMFinish(true);
        LOG_S(INFO) << "finish SLAM";
        while (!packageddata_->isDOMFinish())
        {
            LOG_S(INFO) << "waiting DOM to finish";
            sleep(10);
        }
        LOG_S(INFO) << "finish DOM";

    }





    
    return 0;

}

bool CameraSensorData::init_basic(Json::Value &camera_info, bool have_camera_setting, Json::Value& camera_setting)
{
    if (!camera_info.isMember("camera_type"))
    {
        LOG_F(WARNING, "task file not camera_type member");
        return EXIT_FAILURE;
    }
    else
    {
        camera_type = camera_info["camera_type"].asString();
    }
    if (!camera_info.isMember("sensor_width"))
    {
        LOG_F(WARNING, "task file not sensor_width member");
        return EXIT_FAILURE;
    }
    else
    {
        sensor_width = camera_info["sensor_width"].asDouble();
    }
    if (!camera_info.isMember("sensor_height"))
    {
        LOG_F(WARNING, "task file not sensor_height member");
        return EXIT_FAILURE;
    }
    else
    {
        sensor_height = camera_info["sensor_height"].asDouble();
    }
    if (!camera_info.isMember("actual_focal_length"))
    {
        LOG_F(WARNING, "task file not actual_focal_length member");
        return EXIT_FAILURE;
    }
    else
    {
        actual_focal_length = camera_info["actual_focal_length"].asDouble();
    }
    if (!camera_info.isMember("image_width"))
    {
        LOG_F(WARNING, "task file not image_width member");
        return EXIT_FAILURE;
    }
    else
    {
        image_width = camera_info["image_width"].asInt();
    }
    if (!camera_info.isMember("image_height"))
    {
        LOG_F(WARNING, "task file not image_height member");
        return EXIT_FAILURE;
    }
    else
    {
        image_height = camera_info["image_height"].asInt();
    }
    if (!camera_info.isMember("pitch_angle"))
    {
        LOG_F(WARNING, "task file not pitch_angle member");
        return EXIT_FAILURE;
    }
    else
    {
        pitch_angle = camera_info["pitch_angle"].asInt();
    }
    if (!camera_info.isMember("yaw_angle"))
    {
        LOG_F(WARNING, "task file not yaw_angle member");
        return EXIT_FAILURE;
    }
    else
    {
        yaw_angle = camera_info["yaw_angle"].asInt();
    }

    fx = actual_focal_length / (sensor_width / image_width);
    fy = fx;
    if(camera_type=="h20t" || camera_type=="h20" || camera_type=="m3m" || camera_type=="m3t"){
        fy = actual_focal_length / (sensor_height / image_height);
    }
    if(camera_type=="h20t"){
        camera_type = "Zenmuse H20T";
    }
    if(camera_type=="h20"){
        camera_type = "Zenmuse H20";
    }
    if(camera_type=="m3m"){
        camera_type = "Mavic 3M";
    }
    if(camera_type=="m3e"){
        camera_type = "M3E";
    }
    if(camera_type=="m3t"){
        camera_type = "M3T";
    }
    if(camera_type=="nv2"){
        camera_type = "NV2";
    }
    if(camera_type=="nv3"){
        camera_type = "NV3";
    }
    if(have_camera_setting){
        Json::Value set_camera_yfocal = camera_setting["camera_setting"]["set_camera_yfocal"];
        for(auto&json: set_camera_yfocal){
            if(json["camera_type"].asString() == camera_type){
                if(json["set_value"].asBool()){
                    fy = actual_focal_length / (sensor_height / image_height);
                }
                break;
            }
        }
    }
    cx = image_width / 2.0f;
    cy = image_height / 2.0f;
    k1 = k2 = k3 = p1 = p2 = 0;

    LOG_S(INFO) << "basic camera_type: " << camera_type;
    LOG_S(INFO) << "basic image_width: " << image_width;
    LOG_S(INFO) << "basic image_height: " << image_height;
    LOG_S(INFO) << "basic sensor_height: " << sensor_height;
    LOG_S(INFO) << "basic sensor_width: " << sensor_width;
    LOG_S(INFO) << "basic actual_focal_length: " << actual_focal_length;
    LOG_S(INFO) << "basic fx: " << fx;
    LOG_S(INFO) << "basic fy: " << fy;
    LOG_S(INFO) << "basic cx: " << cx;
    LOG_S(INFO) << "basic cy: " << cy;
    LOG_S(INFO) << "basic k1: " << k1;
    LOG_S(INFO) << "basic k2: " << k2;
    LOG_S(INFO) << "basic p1: " << p1;
    LOG_S(INFO) << "basic p2: " << p2;
    LOG_S(INFO) << "basic k3: " << k3;
    return EXIT_SUCCESS;
}

bool CameraSensorData::init_calib(Json::Value &camera_calib)
{
    is_calib = true;
    if (!camera_calib.isMember("fx"))
    {
        LOG_F(WARNING, "task file not fx member");
        return EXIT_FAILURE;
    }
    else
    {
        fx = camera_calib["fx"].asDouble();
    }

    if (!camera_calib.isMember("fy"))
    {
        LOG_F(WARNING, "task file not fy member");
        return EXIT_FAILURE;
    }
    else
    {
        fy = camera_calib["fy"].asDouble();
    }

    if (!camera_calib.isMember("cx"))
    {
        LOG_F(WARNING, "task file not cx member");
        return EXIT_FAILURE;
    }
    else
    {
        cx = camera_calib["cx"].asDouble();
    }

    if (!camera_calib.isMember("cy"))
    {
        LOG_F(WARNING, "task file not cy member");
        return EXIT_FAILURE;
    }
    else
    {
        cy = camera_calib["cy"].asDouble();
    }

    if (!camera_calib.isMember("k1"))
    {
        LOG_F(WARNING, "task file not k1 member");
        return EXIT_FAILURE;
    }
    else
    {
        k1 = camera_calib["k1"].asDouble();
    }
    if (!camera_calib.isMember("k2"))
    {
        LOG_F(WARNING, "task file not k2 member");
        return EXIT_FAILURE;
    }
    else
    {
        k2 = camera_calib["k2"].asDouble();
    }
    if (!camera_calib.isMember("k3"))
    {
        LOG_F(WARNING, "task file not k3 member");
        return EXIT_FAILURE;
    }
    else
    {
        k3 = camera_calib["k3"].asDouble();
    }

    if (!camera_calib.isMember("p1"))
    {
        LOG_F(WARNING, "task file not p1 member");
        return EXIT_FAILURE;
    }
    else
    {
        p1 = camera_calib["p1"].asDouble();
    }
    if (!camera_calib.isMember("p2"))
    {
        LOG_F(WARNING, "task file not p2 member");
        return EXIT_FAILURE;
    }
    else
    {
        p2 = camera_calib["p2"].asDouble();
    }

    LOG_S(INFO) << "calib fx: " << fx;
    LOG_S(INFO) << "calib fy: " << fy;
    LOG_S(INFO) << "calib cx: " << cx;
    LOG_S(INFO) << "calib cy: " << cy;
    LOG_S(INFO) << "calib k1: " << k1;
    LOG_S(INFO) << "calib k2: " << k2;
    LOG_S(INFO) << "calib p1: " << p1;
    LOG_S(INFO) << "calib p2: " << p2;
    LOG_S(INFO) << "calib k3: " << k3;
    return EXIT_SUCCESS;
}

bool CameraSensorData::export_yaml(std::string yaml_path)
{

    // 创建 YAML 节点
    YAML::Node config;
    config["Camera.type"] = camera_type;
    config["Camera.fx"] = fx;
    config["Camera.fy"] = fy;
    config["Camera.cx"] = cx;
    config["Camera.cy"] = cy;
    config["Camera.k1"] = k1;
    config["Camera.k2"] = k2;
    config["Camera.p1"] = p1;
    config["Camera.p2"] = p2;
    config["Camera.k3"] = k3;
    config["Camera.fps"] = 3;
    config["Camera.RGB"] = 1;

    config["ORBextractor.nFeatures"] = 4000;
    config["ORBextractor.scaleFactor"] = 1.2;
    config["ORBextractor.nLevels"] = 8;
    config["ORBextractor.iniThFAST"] = 20;
    config["ORBextractor.minThFAST"] = 7;

    config["Viewer.KeyFrameSize"] = 0.05;
    config["Viewer.KeyFrameLineWidth"] = 1;
    config["Viewer.GraphLineWidth"] = 0.9;
    config["Viewer.PointSize"] = 2;
    config["Viewer.CameraSize"] = 0.08;
    config["Viewer.CameraLineWidth"] = 3;
    config["Viewer.ViewpointX"] = 0;
    config["Viewer.ViewpointY"] = -0.7;
    config["Viewer.ViewpointZ"] = -1.8;
    config["Viewer.ViewpointF"] = 500;

    // 创建 YAML 输出流
    std::ofstream fout(yaml_path.c_str());

    // 写入 %YAML:1.0 版本标记
    fout << "%YAML:1.0\n";

    // 将 YAML 节点写入文件
    fout << config;

    fout.close();

    LOG_S(INFO) << "YAML file has been written to " << yaml_path;

    return EXIT_SUCCESS;
}
