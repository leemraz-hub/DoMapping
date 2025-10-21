#define LOGURU_IMPLEMENTATION 1
#include "system/loguru.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include <opencv2/opencv.hpp>
#include <sqlite3.h>
#include <json/json.h>
#include <yaml-cpp/yaml.h>
#include "realtime2d_Common.h"
#include "realtime2d_DataPackage.h"
#include "realtime2d_SLAM.h"
#include "realtime2d_DOM.h"
#include "realtime2d_Cloudlog.h"
#include "realtime2d_common.hpp"

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

class MappingConfigData
{
public:
    MappingConfigData() {};
    bool init(Json::Value &mapping_config);
    int input_type = 1;//ReaderType
    bool remove_points_outliers = true;
    bool use_viewer = false;
    bool generate_dom_tiles = true;
    int min_tile_count = 5;
    int tile_level = 20;
    int grab_type = 1;//GrabType
    bool debug = true;
    int video_counter = 10;
    int capture_type = 0; 
};

void write_json(std::string data_path, Json::Value &json)
{
    Json::StyledWriter _writer;
    std::ofstream _os;
    _os.open(data_path);
    _os << _writer.write(json);
    _os.close();
}

int main(int argc, char **argv)
{
    std::string realtime2d_version = std::to_string(REALTIME2D_MAJOR_VERSION) + "." + std::to_string(REALTIME2D_MINOR_VERSION) + "." + std::to_string(REALTIME2D_AREA_VERSION) + "." + std::to_string(REALTIME2D_BUILD_VERSION);

    // loguru::init(argc, argv);
    LOG_F(INFO, "Successfully initialized! realtime2d_version: %s",realtime2d_version.c_str());
    // 定义fatal处理handler
    loguru::set_fatal_handler([](const loguru::Message &message)
                              { throw std::runtime_error(std::string(message.prefix) + message.message); });


    std::string task_json_file;
    std::string input_directory;
    std::string output_directory;
    std::string config_directory;

    std::string camera_setting_json_file;
    std::string vocabulary_path;
    std::string yaml_path;
    std::string database_path;
    std::string task_id;
    double gsd = 0.150;
    bool calibrate = true;// 是否自标定标志，默认true
    bool upload = true;// 是否生成优化图tif及上传，默认true，若设置为false
    bool multi_route = true;// true为转弯的航线，false为不转弯航线


    std::string images_directory;
    std::string rtmp_url;
    std::string mqtt_topic = "none";

    CmdLine cmd;
    cmd.add(make_option(' ', task_json_file, "task_json_file"));
    cmd.add(make_option(' ', input_directory, "input_directory"));
    cmd.add(make_option(' ', output_directory, "output_directory"));
    cmd.add(make_option(' ', config_directory, "config_directory"));
    cmd.add(make_option(' ', gsd, "gsd"));

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

    if (!stlplus::file_exists(task_json_file))
    {
        printf("Cannot find task_json_file: %s\n", task_json_file.c_str());
        return EXIT_FAILURE;
    }

    output_directory += "/";
    if (!stlplus::folder_exists(output_directory))
    {
        if (!stlplus::folder_create(output_directory))
        {
            LOG_F(ERROR, "Cannot create  output_directory");
            return EXIT_FAILURE;
        }
    }

    Json::Value task_json = read_json(task_json_file);
    Json::Value model_params_json = task_json["model_params_json"];
    if (!model_params_json.isMember("task_id"))
    {
        LOG_F(WARNING, "task file not task_id member");
        return EXIT_FAILURE;
    }
    else
    {
        task_id = model_params_json["task_id"].asString();
    }

    std::string task_output_directory = output_directory + "/" + task_id + "/";
    if (!stlplus::folder_exists(task_output_directory))
    {
        if (!stlplus::folder_create(task_output_directory))
        {
            printf("Cannot create task_output_directory: %s", task_output_directory.c_str());
            return EXIT_FAILURE;
        }
    }

    std::string _log_file = task_output_directory + "/realtime2d.log";
    loguru::add_file(_log_file.c_str(), loguru::Truncate, loguru::Verbosity_MAX);

    LOG_S(INFO) << "task_json_file: " << task_json_file;
    LOG_S(INFO) << "input_directory: " << input_directory;
    LOG_S(INFO) << "output_directory: " << output_directory;
    LOG_S(INFO) << "config_directory: " << config_directory;
    LOG_S(INFO) << "task_output_directory: " << task_output_directory;
    LOG_S(INFO) << "task_id: " << task_id;
    LOG_S(INFO) << "realtime2d_version: " << realtime2d_version;


    // 创建和服务端的通讯数据库
    database_path = task_output_directory + "/realtime2d.db";
    if (stlplus::file_exists(database_path))
    {
        std::string _tmp_output = "rm -rf " + database_path;
        std::system(_tmp_output.c_str());
    }

    std::string template_database_path = config_directory + "/realtime2d.db";
    if (!stlplus::file_exists(template_database_path))
    {
        sqlite3 *db = NULL;
        char *zErrMsg = 0;
        int rc;
        // 打开指定的数据库文件,如果不存在将创建一个同名的数据库文件
        rc = sqlite3_open(database_path.c_str(), &db);
        if (rc)
        {
            LOG_F(ERROR, "Can't open database: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            return EXIT_FAILURE;
        }

        // SQL 创建表格
        {
            char sql[2048];
            memset(sql, '\0', 2048);

            sprintf(sql, "%s%s%s", "create table ", "task_connect", "(task_id TEXT PRIMARY KEY, task_status NUMBER, rtk_status NUMBER, video_status NUMBER, task_description TEXT)");
            if (sqlite3_exec(db, sql, NULL, NULL, NULL))
            {
                LOG_F(ERROR, "Failed to create table: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                return EXIT_FAILURE;
            }

            sprintf(sql, "%s%s%s", "create table ", "task_status", "(task_status NUMBER PRIMARY KEY, task_status_text TEXT)");
            if (sqlite3_exec(db, sql, NULL, NULL, NULL))
            {
                LOG_F(ERROR, "Failed to create table: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                return EXIT_FAILURE;
            }

            sprintf(sql, "%s%s%s", "create table ", "rtk_status", "(rtk_status NUMBER PRIMARY KEY, rtk_status_text TEXT)");
            if (sqlite3_exec(db, sql, NULL, NULL, NULL))
            {
                LOG_F(ERROR, "Failed to create table: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                return EXIT_FAILURE;
            }

            sprintf(sql, "%s%s%s", "create table ", "video_status", "(video_status NUMBER PRIMARY KEY, video_status_text TEXT)");
            if (sqlite3_exec(db, sql, NULL, NULL, NULL))
            {
                LOG_F(ERROR, "Failed to create table: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                return EXIT_FAILURE;
            }
        }

        // SQL 插入语句
        {
            const char *sql = "INSERT INTO task_connect (task_id, task_status, rtk_status, video_status, task_description) "
                              "VALUES (?, ?, ?, ?, ?);";

            sqlite3_stmt *stmt;

            // 准备 SQL 语句
            if (sqlite3_prepare_v2(db, sql, -1, &stmt, 0) != SQLITE_OK)
            {
                LOG_F(ERROR, "Failed to prepare statement: %s\n", sqlite3_errmsg(db));
                sqlite3_finalize(stmt);
                sqlite3_close(db);
                return EXIT_FAILURE;
            }

            // 绑定参数
            sqlite3_bind_text(stmt, 1, task_id.c_str(), -1, SQLITE_STATIC); // task_id
            sqlite3_bind_int(stmt, 2, (int)TaskStatus::UNDEFINED);          // task_status
            sqlite3_bind_int(stmt, 3, (int)RtkStatus::RTK_UNDEFINED);        // rtk_status
            sqlite3_bind_int(stmt, 4, (int)VideoStatus::VIDEO_UNDEFINED);    // video_status
            sqlite3_bind_text(stmt, 5, "UNDEFINED", -1, SQLITE_STATIC);     // task_description

            // 执行 SQL 语句
            if (sqlite3_step(stmt) != SQLITE_DONE)
            {
                LOG_F(ERROR, "Execution failed: %s\n", sqlite3_errmsg(db));
                sqlite3_finalize(stmt);
                sqlite3_close(db);
                return EXIT_FAILURE;
            }
            sqlite3_finalize(stmt);
        }

        {
            const char *sql = "INSERT INTO task_status (task_status, task_status_text) "
                              "VALUES (?, ?);";
            static const std::map<int, std::string> statusMap = createTaskStatusMap();
            for (auto &status : statusMap)
            {
                sqlite3_stmt *stmt;
                if (sqlite3_prepare_v2(db, sql, -1, &stmt, 0) != SQLITE_OK) // 准备 SQL 语句
                {
                    LOG_F(ERROR, "Failed to prepare statement: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                // 绑定参数
                sqlite3_bind_int(stmt, 1, status.first);                                 // task_id
                sqlite3_bind_text(stmt, 2, status.second.c_str(), -1, SQLITE_TRANSIENT); // task_status
                // 执行 SQL 语句
                if (sqlite3_step(stmt) != SQLITE_DONE)
                {
                    LOG_F(ERROR, "Execution failed: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                sqlite3_finalize(stmt);
            }
        }
        {
            const char *sql = "INSERT INTO rtk_status (rtk_status, rtk_status_text) "
                              "VALUES (?, ?);";
            static const std::map<int, std::string> statusMap = createRtkStatusMap();
            for (auto &status : statusMap)
            {
                sqlite3_stmt *stmt;
                if (sqlite3_prepare_v2(db, sql, -1, &stmt, 0) != SQLITE_OK) // 准备 SQL 语句
                {
                    LOG_F(ERROR, "Failed to prepare statement: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                // 绑定参数
                sqlite3_bind_int(stmt, 1, status.first);                                 // task_id
                sqlite3_bind_text(stmt, 2, status.second.c_str(), -1, SQLITE_TRANSIENT); // task_status
                // 执行 SQL 语句
                if (sqlite3_step(stmt) != SQLITE_DONE)
                {
                    LOG_F(ERROR, "Execution failed: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                sqlite3_finalize(stmt);
            }
        }
        {
            const char *sql = "INSERT INTO video_status (video_status, video_status_text) "
                              "VALUES (?, ?);";
            static const std::map<int, std::string> statusMap = createVideoStatusMap();
            for (auto &status : statusMap)
            {
                sqlite3_stmt *stmt;
                if (sqlite3_prepare_v2(db, sql, -1, &stmt, 0) != SQLITE_OK) // 准备 SQL 语句
                {
                    LOG_F(ERROR, "Failed to prepare statement: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                // 绑定参数
                sqlite3_bind_int(stmt, 1, status.first);                                 // task_id
                sqlite3_bind_text(stmt, 2, status.second.c_str(), -1, SQLITE_TRANSIENT); // task_status
                // 执行 SQL 语句
                if (sqlite3_step(stmt) != SQLITE_DONE)
                {
                    LOG_F(ERROR, "Execution failed: %s\n", sqlite3_errmsg(db));
                    sqlite3_finalize(stmt);
                    sqlite3_close(db);
                    return EXIT_FAILURE;
                }
                sqlite3_finalize(stmt);
            }
        }

        LOG_F(INFO, "You have opened a sqlite3 database named %s successfully: %s\n", database_path.c_str(), sqlite3_errmsg(db));
        sqlite3_close(db);
    }
    else{
        std::string _tmp_output = "cp " + template_database_path + " " + database_path;
        std::system(_tmp_output.c_str());

        LOG_S(INFO) << "copy database, template_database_path: " << template_database_path<< " -> "<<database_path;

        sqlite3 *db = NULL;
        char *zErrMsg = 0;
        int rc;
        // 打开指定的数据库文件,如果不存在将创建一个同名的数据库文件
        rc = sqlite3_open(database_path.c_str(), &db);
        if (rc)
        {
            LOG_F(ERROR, "Can't open database: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            return EXIT_FAILURE;
        }
        // SQL 插入语句
        {
            const char *sql = "INSERT INTO task_connect (task_id, task_status, rtk_status, video_status, task_description) "
                              "VALUES (?, ?, ?, ?, ?);";

            sqlite3_stmt *stmt;

            // 准备 SQL 语句
            if (sqlite3_prepare_v2(db, sql, -1, &stmt, 0) != SQLITE_OK)
            {
                LOG_F(ERROR, "Failed to prepare statement: %s\n", sqlite3_errmsg(db));
                sqlite3_finalize(stmt);
                sqlite3_close(db);
                return EXIT_FAILURE;
            }

            // 绑定参数
            sqlite3_bind_text(stmt, 1, task_id.c_str(), -1, SQLITE_STATIC); // task_id
            sqlite3_bind_int(stmt, 2, (int)TaskStatus::UNDEFINED);          // task_status
            sqlite3_bind_int(stmt, 3, (int)RtkStatus::RTK_UNDEFINED);        // rtk_status
            sqlite3_bind_int(stmt, 4, (int)VideoStatus::VIDEO_UNDEFINED);    // video_status
            sqlite3_bind_text(stmt, 5, "UNDEFINED", -1, SQLITE_STATIC);     // task_description

            // 执行 SQL 语句
            if (sqlite3_step(stmt) != SQLITE_DONE)
            {
                LOG_F(ERROR, "Execution failed: %s\n", sqlite3_errmsg(db));
                sqlite3_finalize(stmt);
                sqlite3_close(db);
                return EXIT_FAILURE;
            }
            sqlite3_finalize(stmt);
        }
        LOG_F(INFO, "You have opened a sqlite3 database named %s successfully: %s\n", database_path.c_str(), sqlite3_errmsg(db));
        sqlite3_close(db);
    }
    LOG_S(INFO) << "database_path: " << database_path;



    // 创建和其他命令进程的通讯数据库
    std::string grab_database_path = task_output_directory + "/grab_realtime2d.db";
    if (stlplus::file_exists(grab_database_path))
    {
        std::string _tmp_output = "rm -rf " + grab_database_path;
        std::system(_tmp_output.c_str());
    }
    {
        std::string _tmp_output = "cp " + database_path + " " + grab_database_path;
        std::system(_tmp_output.c_str());
        LOG_S(INFO) << "copy database, database_path: " << database_path<< " -> "<<grab_database_path;
    }
    LOG_S(INFO) << "grab_database_path: " << grab_database_path;
    
    std::string mqtt_config_file = config_directory + "/mqtt_config.json";
    bool use_mqtt = false;

    // vocabulary_path = config_directory + "/Vocabulary/ORBvoc.txt";
    vocabulary_path = config_directory + "/Vocabulary/ORBvoc.bin";
    LOG_S(INFO) << "vocabulary_path: " << vocabulary_path;

    images_directory = input_directory + "/" + task_id + "/";
    LOG_S(INFO) << "images_directory: " << images_directory;

    camera_setting_json_file = config_directory + "/realtime2d_camera_setting.json";
    Json::Value camera_setting;
    bool have_camera_setting = false;
    if(stlplus::file_exists(camera_setting_json_file)){
        have_camera_setting = true;
        LOG_S(INFO) << "camera_setting_json_file: " << camera_setting_json_file;
        camera_setting = read_json(camera_setting_json_file);
    }
    Json::Value camera_info;
    Json::Value mapping_config;
    Json::Value flight_info;
    std::vector<std::pair<double, double>> wkt_extent;
    MappingConfigData mapping_config_data;
    CameraSensorData camera_sensor_data;

    bool is_mqtt_dataset = false;

    try {

    
    if (!model_params_json.isMember("rtmp_url"))
    {
        LOG_F(WARNING, "task file not rtmp_url member");
        return EXIT_FAILURE;
    }
    else
    {
        rtmp_url = model_params_json["rtmp_url"].asString();
    }
    LOG_S(INFO) << "rtmp_url: " << rtmp_url;


    if (!model_params_json.isMember("mqtt_topic"))
    {
        LOG_F(WARNING, "task file not mqtt_topic member");
    }
    else
    {
        mqtt_topic = model_params_json["mqtt_topic"].asString();
    }
    LOG_S(INFO) << "mqtt_topic: " << mqtt_topic;

    if (!model_params_json.isMember("is_mqtt_dataset"))
    {
        LOG_F(WARNING, "task file not is_mqtt_dataset member");
    }
    else
    {
        is_mqtt_dataset = model_params_json["is_mqtt_dataset"].asBool();
    }
    LOG_S(INFO) << "is_mqtt_dataset: " << is_mqtt_dataset;

    if (!model_params_json.isMember("calibrate"))
    {
        LOG_F(WARNING, "task file not calibrate member");
    }
    else
    {
        calibrate = model_params_json["calibrate"].asBool();
    }
    LOG_S(INFO) << "calibrate: " << calibrate;

    if (!model_params_json.isMember("upload"))
    {
        LOG_F(WARNING, "task file not upload member");
    }
    else
    {
        upload = model_params_json["upload"].asBool();
    }
    LOG_S(INFO) << "upload: " << upload;

    if (!model_params_json.isMember("multi_route"))
    {
        LOG_F(WARNING, "task file not multi_route member");
    }
    else
    {
        multi_route = model_params_json["multi_route"].asBool();
    }
    LOG_S(INFO) << "multi_route: " << multi_route;

    if (!model_params_json.isMember("flight_info"))
    {
        LOG_F(WARNING, "task file not flight_info member");
    }
    else
    {
        int survey_resolution_mm = model_params_json["flight_info"]["survey_resolution"].asInt();
        // gsd = survey_resolution_mm / 1000.0f; // m
        LOG_S(INFO) << "survey_resolution_mm: " << survey_resolution_mm;
    }
    LOG_S(INFO) << "gsd: " << gsd;


    if (!model_params_json.isMember("camera_info"))
    {
        LOG_F(WARNING, "task file not camera_info member");
        return EXIT_FAILURE;
    }
    else
    {
        camera_info = model_params_json["camera_info"];
    }
    if (!model_params_json.isMember("mapping_config"))
    {
        LOG_F(WARNING, "task file not mapping_config member");
        return EXIT_FAILURE;
    }
    else
    {
        mapping_config = model_params_json["mapping_config"];
    }
    if (!model_params_json.isMember("flight_info"))
    {
        LOG_F(WARNING, "task file not flight_info member");
        return EXIT_FAILURE;
    }
    else
    {
        flight_info = model_params_json["flight_info"];
    }
    
    if (model_params_json.isMember("boundary"))
    {
        Json::Value boundary = model_params_json["boundary"];
        if (boundary.isMember("geometry"))
        {
            Json::Value geometry = boundary["geometry"];
            if (geometry.isMember("coordinates"))
            {
                if(geometry["coordinates"].size()>=1){
                    Json::Value coordinates = geometry["coordinates"][0];
                    LOG_S(INFO)<<"coordinates: \n"<<coordinates<<" size: "<<coordinates.size(); 
                    for(int k=0;k<coordinates.size();k++){
                        wkt_extent.push_back(std::make_pair(coordinates[k][0].asDouble(),coordinates[k][1].asDouble()));
                    }
                }
                else{
                    LOG_F(WARNING, "coordinates empty");
                }
            }
            else
            {
                LOG_F(WARNING, "task file not coordinates member");
            }
        }
        else
        {
            LOG_F(WARNING, "task file not geometry member");
        }
    }
    else
    {
        LOG_F(WARNING, "task file not boundary member");
    }

    if (mapping_config_data.init(mapping_config))
    {
        LOG_F(WARNING, "mapping_confi init fail");
        return EXIT_FAILURE;
    }

    if (camera_sensor_data.init_basic(camera_info, have_camera_setting, camera_setting))
    {
        LOG_F(WARNING, "camera_calib init_basic fail");
        return EXIT_FAILURE;
    }
    if (!model_params_json.isMember("camera_calib"))
    {
        LOG_F(WARNING, "task file not camera_calib member");
    }
    else
    {
        Json::Value camera_calib = model_params_json["camera_calib"];
        if (camera_sensor_data.init_calib(camera_calib))
        {
            LOG_F(WARNING, "camera_calib init_calib fail");
            return EXIT_FAILURE;
        }
    }

    } catch (const std::exception& e) {
        LOG_S(WARNING) << "json_error: " << e.what();

        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::FAILED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
            return EXIT_FAILURE;
        }
        if (updateTaskStatus(grab_database_path, task_id, (int)TaskStatus::FAILED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
            return EXIT_FAILURE;
        }
        return EXIT_FAILURE;
    }

    if(mqtt_topic!="none"&&mqtt_config_file!="none"&&stlplus::file_exists(mqtt_config_file)){
        use_mqtt = true;
    }

    std::string output_cloudlog_json_file = task_output_directory + "/grab_cloudlog.json";
    LOG_S(INFO) << "output_cloudlog_json_file: " << output_cloudlog_json_file;
    std::shared_ptr<realtime2d::openCloudlog> realtime2d_cloudlog_;
    realtime2d_cloudlog_.reset(new realtime2d::openCloudlog(_log_file, output_cloudlog_json_file));
    realtime2d_cloudlog_->initCloudlog();
    realtime2d_cloudlog_->setSendTime(1);//默认2秒保存依次日志
    realtime2d_cloudlog_->setMappingStatus("READY");

    std::shared_ptr<realtime2d::openPackageData> packageddata_;
    packageddata_.reset(new realtime2d::openPackageData());
    packageddata_->setSLAMFinish(false);
    packageddata_->setDOMFinish(false);

    std::shared_ptr<realtime2d::openSLAM> realtime2d_slam_;
    realtime2d_slam_.reset(new realtime2d::openSLAM(mapping_config_data.input_type, mapping_config_data.use_viewer, mapping_config_data.debug, mapping_config_data.capture_type));
    realtime2d_slam_->setOutputDirectory(task_output_directory);
    realtime2d_slam_->setGrabType(mapping_config_data.grab_type);
    realtime2d_slam_->setImagesDirectory(images_directory);
    realtime2d_slam_->setRtmpURL(rtmp_url);
    if(use_mqtt){
        realtime2d_slam_->setMQTT(mqtt_topic, mqtt_config_file);
    }
    if(is_mqtt_dataset){
        realtime2d_slam_->setUseMQTT();
    }
    realtime2d_slam_->initDataReader();

    if(!camera_sensor_data.is_calib){
        int image_width, image_height;
        bool ret = realtime2d_slam_->getImageSize(image_width, image_height);
        LOG_S(INFO) << "getImageSize, image_width: " << image_width<<" image_height: "<<image_height<<" ret: "<<ret;
        if(ret){
            camera_info["image_width"] = image_width;
            camera_info["image_height"] = image_height;
            camera_sensor_data.init_basic(camera_info, have_camera_setting, camera_setting);
        }
    }
    yaml_path = task_output_directory + "/camera.yaml";
    camera_sensor_data.export_yaml(yaml_path);
    LOG_S(INFO) << "yaml_path: " << yaml_path;

    realtime2d_slam_->setVocabularyPath(vocabulary_path);
    realtime2d_slam_->setYamlPath(yaml_path);
    realtime2d_slam_->setCalibrate(calibrate);
    realtime2d_slam_->setPackageData(packageddata_);
    realtime2d_slam_->initSLAM(); // 视频流的初始化不能保存图像到队列，其他input_type则可以
    realtime2d_slam_->setVideoCounter(mapping_config_data.video_counter);

    std::shared_ptr<realtime2d::openDOM> realtime2d_dom_;
    realtime2d_dom_.reset(new realtime2d::openDOM(mapping_config_data.debug));
    realtime2d_dom_->setOutputDirectory(task_output_directory);
    realtime2d_dom_->setPackageData(packageddata_);
    realtime2d_dom_->setYamlPath(yaml_path);
    realtime2d_dom_->setTileLevel(mapping_config_data.tile_level);
    realtime2d_dom_->setGSD(gsd);
    realtime2d_dom_->setUpdateFrequency(mapping_config_data.min_tile_count);
    realtime2d_dom_->setGenTiles(mapping_config_data.generate_dom_tiles);
    if(!wkt_extent.empty()){
        realtime2d_dom_->setWktExtent(wkt_extent);
    }
    realtime2d_dom_->initDOM();
    realtime2d_dom_->setUploadGlobalDOM(upload);

    realtime2d_slam_->setupWorkStatus(false);
    realtime2d_cloudlog_->startCloudlog();
    realtime2d_slam_->startSLAM();
    realtime2d_dom_->startWork();

    if (updateTaskStatus(database_path, task_id, (int)TaskStatus::SUCCESS))
    {
        LOG_F(ERROR, "updateTaskStatus failed");
        return EXIT_FAILURE;
    }
    if (updateTaskStatus(grab_database_path, task_id, (int)TaskStatus::SUCCESS))
    {
        LOG_F(ERROR, "updateTaskStatus failed");
        return EXIT_FAILURE;
    }
    bool is_mapping = false;
    bool is_terminated = false;
    bool is_aborted = false;
    while (1)
    {
        int task_status;
        if (getTaskStatus(grab_database_path, task_id, task_status))
        {
            LOG_F(ERROR, "getTaskStatus failed");
        }
        if (task_status == (int)TaskStatus::MAPPING)
        {
            is_mapping = true;
            break;
        }
        if (task_status == (int)TaskStatus::TERMINATED)
        {
            is_terminated = true;
            break;
        }
        LOG_S(INFO) << "wait2start";
        sleep(1);
    }

    if(is_terminated){

        realtime2d_cloudlog_->setMappingStatus("TERMINATED");
        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::TERMINATED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }

        while (!packageddata_->isSLAMFinish())
        {
            LOG_S(INFO) << "waiting SLAM to finish";
            realtime2d_slam_->stopSLAM();
            sleep(1);
        }
        LOG_S(INFO) << "finish SLAM";
        while (!packageddata_->isDOMFinish())
        {
            LOG_S(INFO) << "waiting DOM to finish";
            sleep(1);
        }
        LOG_S(INFO) << "finish DOM";
    }

    if(is_mapping){

        realtime2d_cloudlog_->setMappingStatus("RUNNING");

        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::MAPPING))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }

        realtime2d_slam_->setupWorkStatus(true);
        while (!packageddata_->isSLAMFinish())
        {
            // LOG_S(INFO) << "waiting SLAM to finish";

            int task_status;
            if (getTaskStatus(grab_database_path, task_id, task_status))
            {
                LOG_F(ERROR, "getTaskStatus failed");
            }
            if (task_status == (int)TaskStatus::TERMINATED)
            {
                if(!is_terminated){
                    is_terminated = true;
                    if (updateTaskStatus(database_path, task_id, (int)TaskStatus::TERMINATED))
                    {
                        
                        LOG_F(ERROR, "updateTaskStatus failed");
                    }
                }
                realtime2d_slam_->stopSLAM();
            }
            // LOG_S(INFO) << "wait2stop";
            sleep(1);
        }
        if(!is_terminated){
            //check slam finish reason

            realtime2d::openSLAM::FinishReason finish_reason = realtime2d_slam_->getFinishReason();

            // reason1: lla_error
            // reason2: rpy_error
            // reason3: start阶段俯仰角长时间异常
            // reason4: mapping阶段俯仰角长时间异常
            // reason5: start阶段和mapping阶段的sei延时异常
            // reason6: mapping阶段长时间lost异常
            if(finish_reason !=realtime2d::openSLAM::FinishReason::ACTIVE_STOP){
                is_aborted = true;
                realtime2d_cloudlog_->setMappingStatus("ABORTED");

                switch (finish_reason)
                {
                    case realtime2d::openSLAM::FinishReason::SEI_NO_LLA:
                        LOG_S(WARNING)<<"finish_reason: SEI_NO_LLA";
                        break;
                    case realtime2d::openSLAM::FinishReason::SEI_NO_RPY:
                        LOG_S(WARNING)<<"finish_reason: SEI_NO_RPY";
                        break;
                    case realtime2d::openSLAM::FinishReason::START_RPY_ERROR:
                        LOG_S(WARNING)<<"finish_reason: START_RPY_ERROR";
                        break;
                    case realtime2d::openSLAM::FinishReason::MAPPING_RPY_ERROR:
                        LOG_S(WARNING)<<"finish_reason: MAPPING_RPY_ERROR";
                        break;
                    case realtime2d::openSLAM::FinishReason::SEI_DELAY:
                        LOG_S(WARNING)<<"finish_reason: SEI_DELAY";
                        break;
                    case realtime2d::openSLAM::FinishReason::POSE_LOST_DELAY:
                        LOG_S(WARNING)<<"finish_reason: POSE_LOST_DELAY";
                        break;
                    default:
                        break;
                }
            }
        }
        LOG_S(INFO) << "is_aborted: "<<is_aborted;

        if(!is_aborted&&calibrate){
            std::vector<double> vec_params = realtime2d_slam_->getCalibrateParams();
            cv::Mat K = realtime2d_slam_->getCamKmatrix();
            if(vec_params.size()>0){
                double fx = K.at<double>(0, 0);
                double opt_fx = vec_params[0];
                LOG_S(INFO)<<"优化前相机焦距："<<fx<<"，"<<"优化后相机焦距："<<opt_fx;
                Json::Value camera_calib;
                camera_calib["fx"] = vec_params[0];
                camera_calib["fy"] = vec_params[1];
                camera_calib["cx"] = vec_params[2];
                camera_calib["cy"] = vec_params[3];
                camera_calib["k1"] = vec_params[4];
                camera_calib["k2"] = vec_params[5];
                camera_calib["p1"] = vec_params[6];
                camera_calib["p2"] = vec_params[7];
                camera_calib["k3"] = vec_params[8];
                std::string camera_calibration_json_path = task_output_directory + "/camera_calibration.json";
                Json::Value data;
                data["camera_calib"] = camera_calib;
                data["task_id"] = model_params_json["task_id"];
                data["camera_info"] = model_params_json["camera_info"];
                Json::Value json;
                json["camera_calibration_json"] = data;
                write_json(camera_calibration_json_path, json);
            }
        }
        LOG_S(INFO) << "finish SLAM";
        while (!packageddata_->isDOMFinish())
        {
            LOG_S(INFO) << "waiting DOM to finish";
            sleep(10);
        }
        LOG_S(INFO) << "finish DOM";

        if(mapping_config_data.input_type == realtime2d::DataReader::ReaderType::VIDEO){
            std::string cache_images_directory = task_output_directory + "realtime2d_cache_images/";
            std::string dataset_output_directory = input_directory + "/" + task_id + "/";
            if (!stlplus::folder_exists(dataset_output_directory))
            {
                if (!stlplus::folder_create(dataset_output_directory))
                {
                    printf("Cannot create dataset_output_directory: %s", dataset_output_directory.c_str());
                    return EXIT_FAILURE;
                }
            }
            if (stlplus::folder_exists(cache_images_directory))
            {
                std::string mStartWorkFilename = realtime2d_slam_->mStartWorkFilename;
                std::string mFinalWorkFilename = realtime2d_slam_->mFinalWorkFilename;
                float t1 = atof(stlplus::basename_part(mStartWorkFilename).c_str());
                float t2 = atof(stlplus::basename_part(mFinalWorkFilename).c_str());
                std::vector<std::string> filenames = stlplus::folder_files(cache_images_directory);
                std::vector<std::string> jpg_files;
                for(auto& filename:filenames){
                    float time = atof(stlplus::basename_part(filename).c_str());
                    if(time>=t1 && time<=t2){
                        std::string src_filepath = cache_images_directory + filename;
                        std::string dst_filepath = dataset_output_directory + filename;
                        std::string _tmp_output = "cp " + src_filepath + " " + dst_filepath;
                        std::system(_tmp_output.c_str());
                        // LOG_S(INFO)<<"[copy] "<<_tmp_output;
                    }
                }
            }
        }
    }

    // if(!is_aborted&&is_mapping){
    if(!is_aborted){
        realtime2d_cloudlog_->setMappingStatus("COMPLETED");
        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::COMPLETED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }
        if (updateTaskStatus(grab_database_path, task_id, (int)TaskStatus::COMPLETED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }
    }
    else{
        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::ABORT))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }
        if (updateTaskStatus(grab_database_path, task_id, (int)TaskStatus::ABORT))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
        }
    }
    int task_status;
    if (getTaskStatus(database_path, task_id, task_status))
    {
        LOG_F(ERROR, "getTaskStatus failed");
    }
    realtime2d_cloudlog_->stopCloudlog();

    return EXIT_SUCCESS;
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
    config["Camera.RGB"] = 0;

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

bool MappingConfigData::init(Json::Value &mapping_config)
{

    if (!mapping_config.isMember("input_type"))
    {
        LOG_F(WARNING, "task file not input_type member");
        return EXIT_FAILURE;
    }
    else
    {
        input_type = mapping_config["input_type"].asInt();
    }

    if (!mapping_config.isMember("remove_points_outliers"))
    {
        LOG_F(WARNING, "task file not remove_points_outliers member");
        return EXIT_FAILURE;
    }
    else
    {
        remove_points_outliers = mapping_config["remove_points_outliers"].asBool();
    }

    if (!mapping_config.isMember("use_viewer"))
    {
        LOG_F(WARNING, "task file not use_viewer member");
        return EXIT_FAILURE;
    }
    else
    {
        use_viewer = mapping_config["use_viewer"].asBool();
    }

    if (!mapping_config.isMember("grab_type"))
    {
        LOG_F(WARNING, "task file not grab_type member");
        return EXIT_FAILURE;
    }
    else
    {
        grab_type = mapping_config["grab_type"].asInt();
    }

    if (!mapping_config.isMember("tile_level"))
    {
        LOG_F(WARNING, "task file not tile_level member");
        return EXIT_FAILURE;
    }
    else
    {
        tile_level = mapping_config["tile_level"].asInt();
    }

    if (!mapping_config.isMember("min_tile_count"))
    {
        LOG_F(WARNING, "task file not min_tile_count member");
        return EXIT_FAILURE;
    }
    else
    {
        min_tile_count = mapping_config["min_tile_count"].asInt();
    }

    if (!mapping_config.isMember("generate_dom_tiles"))
    {
        LOG_F(WARNING, "task file not generate_dom_tiles member");
        return EXIT_FAILURE;
    }
    else
    {
        generate_dom_tiles = mapping_config["generate_dom_tiles"].asBool();
    }

    if (!mapping_config.isMember("debug"))
    {
        LOG_F(WARNING, "task file not debug member");
        return EXIT_FAILURE;
    }
    else
    {
        debug = mapping_config["debug"].asBool();
    }

    if (!mapping_config.isMember("video_counter"))
    {
        LOG_F(WARNING, "task file not video_counter member");
    }
    else
    {
        video_counter = mapping_config["video_counter"].asInt();
    }

    if (!mapping_config.isMember("capture_type"))
    {
        LOG_F(WARNING, "task file not capture_type member");
    }
    else
    {
        capture_type = mapping_config["capture_type"].asInt();
    }

    LOG_S(INFO) << "remove_points_outliers: " << remove_points_outliers;
    LOG_S(INFO) << "use_viewer: " << use_viewer;
    LOG_S(INFO) << "generate_dom_tiles: " << generate_dom_tiles;
    LOG_S(INFO) << "min_tile_count: " << min_tile_count;
    LOG_S(INFO) << "tile_level: " << tile_level;
    LOG_S(INFO) << "input_type: " << input_type;
    LOG_S(INFO) << "grab_type: " << grab_type;
    LOG_S(INFO) << "debug: " << debug;
    LOG_S(INFO) << "video_counter: " << video_counter;
    LOG_S(INFO) << "capture_type: " << capture_type;

    return EXIT_SUCCESS;
}

