
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/stevencui/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../bin/";

std::shared_ptr<System> pSystem;

/**
 * @brief 从MH_05_imu0.txt中读取IMU数据(数据和时间戳) 并且存到imu_buffer里面
 */
void PubImuData()
{
    string sImu_data_file = sConfig_path + "imu/imu_pose_noise.txt";
    cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
    ifstream fsImu;
    fsImu.open(sImu_data_file.c_str());
    if (!fsImu.is_open())
    {
        cerr << "Failed to open imu file! " << sImu_data_file << endl;
        return;
    }

    std::string sImu_line;
    double dStampNSec = 0.0;
    Vector3d vAcc;
    Vector3d vGyr;
    while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
    {
        std::istringstream ssImuData(sImu_line);
        //时间戳 陀螺仪xyz 加速度计xyz
        ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        // cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
//        pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
        pSystem->PubImuData(dStampNSec, vGyr, vAcc);
        usleep(1000000*1.0/200);//200Hz
    }
    fsImu.close();
}

void PubImageData()
{   //../bin/ + "camera/"
    string sImage_file = sConfig_path + "camera/";


    // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
    // 从MH_05_cam0.txt中按行读取->sImage_line
    int file_counts = 0;
//    std::string feature_line;

    while (file_counts < 600)
    {
        //每一帧的图像数据(归一化之后的坐标)
        string Image_file_name = sImage_file + "all_points_" + std::to_string(file_counts) + ".txt";
//        std::cout<<"Image_file_name: "<<Image_file_name<<std::endl;
        ifstream fsImage;
        fsImage.open(Image_file_name.c_str());
        if (!fsImage.is_open())
        {
            cerr << "Failed to open image file! " << Image_file_name << endl;
            return;
        }

        std::string feature_line;
        double dStampNSec;
        double feature_id;
        double coor_u;
        double coor_v;

        vector<vector<double>> uv_feature_points_sets;

        while (std::getline(fsImage, feature_line) && !feature_line.empty())
        {
            vector<double> uv_feature_points;
            std::istringstream feature_data(feature_line);
            feature_data >> dStampNSec;
            feature_data >> feature_id;
            feature_data >> coor_u;
            feature_data >> coor_v;

            uv_feature_points.push_back(dStampNSec);
            uv_feature_points.push_back(feature_id);
            uv_feature_points.push_back(coor_u);
            uv_feature_points.push_back(coor_v);
            uv_feature_points_sets.push_back(uv_feature_points);
        }

//        std::cout << "======================" << std::endl;
//        for (int j = 0; j < uv_feature_points_sets.size(); ++j)
//        {
//            std::cout << uv_feature_points_sets[j][0] << " " << uv_feature_points_sets[j][1] << " " << uv_feature_points_sets[j][2] << " " << uv_feature_points_sets[j][3] << std::endl;
//        }
        //每一帧图像，保存feature_buf
        //主要修改这个函数
        pSystem->PubImageData(uv_feature_points_sets[0][0], uv_feature_points_sets);
        // cv::imshow("SOURCE IMAGE", img);
        // cv::waitKey(0);
        usleep(1000000*1.0/30);
        file_counts++;
        fsImage.close();
//        std::cout<<"======================"<<std::endl;

    }


}

int main(int argc, char **argv)
{
//    if(argc != 3)
//    {
//        // ../config/
//        cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n"
//             << "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
//        return -1;
//    }
//    sData_path = argv[1];
//    sConfig_path = argv[2];

    pSystem.reset(new System(sConfig_path));//

    std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

    // sleep(5);
    std::thread thd_PubImuData(PubImuData);

    std::thread thd_PubImageData(PubImageData);

    std::thread thd_Draw(&System::Draw, pSystem);

    thd_PubImuData.join();
    thd_PubImageData.join();

    thd_BackEnd.join();
    thd_Draw.join();

    cout << "main end... see you ..." << endl;
    return 0;
}
