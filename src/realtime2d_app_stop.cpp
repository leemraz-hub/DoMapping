#define LOGURU_IMPLEMENTATION 1
#include "system/loguru.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sqlite3.h>
#include "realtime2d_common.hpp"

int main(int argc, char **argv)
{

    std::string task_json_file;
    std::string output_directory;

    CmdLine cmd;
    cmd.add(make_option(' ', task_json_file, "task_json_file"));
    cmd.add(make_option(' ', output_directory, "output_directory"));

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

    if (!stlplus::folder_exists(output_directory))
    {
        printf("Cannot find output_directory: %s\n", output_directory.c_str());
        return EXIT_FAILURE;
    }

    std::string task_id;
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
    LOG_S(INFO) << "task_id: " << task_id;

    std::string task_output_directory = output_directory + "/" + task_id + "/";
    if (!stlplus::folder_exists(task_output_directory))
    {
        LOG_F(WARNING, "Cannot find task_output_directory:%s", task_output_directory.c_str());
        return EXIT_FAILURE;
    }
    std::string database_path = task_output_directory + "/grab_realtime2d.db";
    if (!stlplus::file_exists(database_path))
    {
        LOG_F(WARNING, "Cannot find database_path:%s", database_path.c_str());
        return EXIT_FAILURE;
    }

    int task_status;
    if (getTaskStatus(database_path, task_id, task_status))
    {
        LOG_F(ERROR, "getTaskStatus failed");
        return EXIT_FAILURE;
    }

    // if (task_status == (int)TaskStatus::MAPPING)
    {
        if (updateTaskStatus(database_path, task_id, (int)TaskStatus::TERMINATED))
        {
            LOG_F(ERROR, "updateTaskStatus failed");
            return EXIT_FAILURE;
        }
    }

    if (getTaskStatus(database_path, task_id, task_status))
    {
        LOG_F(ERROR, "getTaskStatus failed");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}