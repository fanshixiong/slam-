#include "System.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace pangolin;

System::System(string sConfig_file_)
        : bStart_backend(true)
{
    string tmp_path = "/home/alex/VIO/darkbue/hw7/vins_sys_code/config/";
    string sConfig_file = tmp_path + "euroc_config.yaml";

    cout << "1 System() sConfig_file: " << sConfig_file << endl;
    readParameters(sConfig_file);

    trackerData[0].readIntrinsicParameter(sConfig_file);

    estimator.setParameter();
    ofs_pose.open("./pose_output.txt", fstream::app | fstream::out);
    if (!ofs_pose.is_open())
    {
        cerr << "ofs_pose is not open" << endl;
    }
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();
    cout << "2 System() end" << endl;
}

System::~System()
{
    bStart_backend = false;

    pangolin::QuitAll();

    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();

    ofs_pose.close();
}

/**
 * @brief 读取保存的图像信息
 * @param dStampSec 时间戳
 * @param img 图像
 */
void System::PubImageData(double dStampSec, vector<vector<double>> &img)
{

    // id, un_pts(x,y)
    map<int, cv::Point2f> cur_un_pts_map;
    pub_count++;
    //feature_points:
    //        double header;
    //        vector<Vector3d> points;
    //        vector<int> id_of_point;
    //        vector<float> u_of_point;
    //        vector<float> v_of_point;
    //        vector<float> velocity_x_of_point;
    //        vector<float> velocity_y_of_point;
    shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
    feature_points->header = dStampSec;
    cur_un_pts_map.clear();
    int fail_count = 0;
    if (feature_points->header == 0)
    {
        for (unsigned int j = 0; j < img.size(); j++)
        {
            double x = img[j][2];
            double y = img[j][3];
            double z = 1;
            double u, v;
//            u = FOCAL_LENGTH*x + COL/2.0;
//            v = FOCAL_LENGTH*y + ROW/2.0;
            u = FOCAL_LENGTH*x + 255;
            v = FOCAL_LENGTH*y + 255;
            if (u > 0 && v > 0 && u < COL && v < ROW)
            {
                feature_points->points.push_back(Vector3d(x, y, z)); //归一化坐标（从uv坐标经过相机内参得到归一化的坐标）
                feature_points->id_of_point.push_back(img[j][1]);
                feature_points->u_of_point.push_back(u);
                feature_points->v_of_point.push_back(v);
                feature_points->velocity_x_of_point.push_back(0);
                feature_points->velocity_y_of_point.push_back(0);
                cur_un_pts_map.insert(make_pair(img[j][1], cv::Point2f(u, v)));
            }
        }
    } else
    {
        //计算uv的速度
        for (unsigned int j = 0; j < img.size(); j++)
        {
            double x = img[j][2];
            double y = img[j][3];
            double z = 1;
            double u, v;
//            u = FOCAL_LENGTH*x + COL/2.0;
//            v = FOCAL_LENGTH*y + ROW/2.0;
            u = FOCAL_LENGTH*x + 255;
            v = FOCAL_LENGTH*y + 255;
//            double u_velocity, v_velocity;
            if (u > 0 && v > 0 && u < COL && v < ROW)
            {
                feature_points->points.push_back(Vector3d(x, y, z)); //归一化坐标（从uv坐标经过相机内参得到归一化的坐标）
                feature_points->id_of_point.push_back(img[j][1]);
                feature_points->u_of_point.push_back(u);
                feature_points->v_of_point.push_back(v);
                cur_un_pts_map.insert(make_pair(img[j][1], cv::Point2f(u, v)));
            } else
            {
                fail_count++;
            }
        }
    }
    std::cout << std::to_string(dStampSec) << " : failed uv points:" << fail_count << std::endl;

    pre_un_pts_map = cur_un_pts_map;
    pre_feature_set = img;
    //计算uv速度
    if (!pre_un_pts_map.empty())
    {
        double vx, vy;
        for (const auto &cur_pts:cur_un_pts_map)
        {
            auto pre_itor = pre_un_pts_map.find(cur_pts.first);
            if (pre_itor != pre_un_pts_map.end())
            {
                vx = (cur_pts.second.x - pre_itor->second.x)*30;
                vy = (cur_pts.second.y - pre_itor->second.y)*30;
            } else
            {
                vx = 0;
                vy = 0;
            }
            feature_points->velocity_x_of_point.push_back(vx);
            feature_points->velocity_y_of_point.push_back(vy);
        }
    }


    //}
    // skip the first image; since no optical speed on frist image
    if (!init_pub)
    {
        cout << "4 PubImage init_pub skip the first image!" << endl;
        init_pub = 1;
    } else
    {
        m_buf.lock();
        feature_buf.push(feature_points);
        // cout << "5 PubImage t : " << fixed << feature_points->header
        //     << " feature_buf size: " << feature_buf.size() << endl;
        m_buf.unlock();
        con.notify_one();
    }


//    cv::Mat show_img;
//	cv::cvtColor(img, show_img, CV_GRAY2RGB);
//	if (SHOW_TRACK)
//	{
//		for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
//        {
//			double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);
//			cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
//		}
//
//        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
//		cv::imshow("IMAGE", show_img);
//        cv::waitKey(1);
//	}
    // cout << "5 PubImage" << endl;

}

vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        cout<<"In measurement!"<<endl;

        if (imu_buf.empty() || feature_buf.empty())
        {
            // cerr << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td))
        {
            cerr << "wait for imu, only should happen at the beginning sum_of_wait: "
                 << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }
        //estimator.td 时间戳补偿
        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td))
        {
            cerr << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
        {
            cerr << "no imu between two image" << endl;
        }
        // cout << "1 getMeasurements img t: " << fixed << img_msg->header
        //     << " imu begin: "<< IMUs.front()->header 
        //     << " end: " << IMUs.back()->header
        //     << endl;
//        cout<<"In measurement!"<<endl;
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void System::PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                        const Eigen::Vector3d &vAcc)
{
    //包含:    Eigen::Vector3d linear_acceleration;???
    //        Eigen::Vector3d angular_velocity;
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
    imu_msg->header = dStampSec;
    imu_msg->linear_acceleration = vAcc;
    imu_msg->angular_velocity = vGyr;

//    if (dStampSec <= last_imu_t)
//    {
//        cerr << "imu message in disorder!" << endl;
//        return;
//    }
    last_imu_t = dStampSec;
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " acc: " << imu_msg->linear_acceleration.transpose()
    //     << " gyr: " << imu_msg->angular_velocity.transpose() << endl;
    m_buf.lock();
    imu_buf.push(imu_msg);
//     cout << "1 PubImuData t: " << fixed << imu_msg->header
//         << " imu_buf size:" << imu_buf.size() << endl;
    m_buf.unlock();
    con.notify_one();//随机唤醒一个等待的线程
}

// thread: visual-inertial odometry
void System::ProcessBackEnd()
{
    cout << "1 ProcessBackEnd start" << endl;
    while (bStart_backend)
    {
        cout << "1 process()!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        // 一帧图像 対应一段IMU数据
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            return (measurements = getMeasurements()).size() != 0;
        });
        if (measurements.size() > 1)
        {
            cout << "1 getMeasurements size: " << measurements.size()
                 << " imu sizes: " << measurements[0].first.size()
                 << " feature_buf size: " << feature_buf.size()
                 << " imu_buf size: " << imu_buf.size() << endl;
        }
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header;
                double img_t = imu_msg->header + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                } else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2/(dt_1 + dt_2);
                    double w2 = dt_1/(dt_1 + dt_2);
                    dx = w1*dx + w2*imu_msg->linear_acceleration.x();
                    dy = w1*dy + w2*imu_msg->linear_acceleration.y();
                    dz = w1*dz + w2*imu_msg->linear_acceleration.z();
                    rx = w1*rx + w2*imu_msg->angular_velocity.x();
                    ry = w1*ry + w2*imu_msg->angular_velocity.y();
                    rz = w1*rz + w2*imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // cout << "processing vision data with stamp:" << img_msg->header 
            //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
//                std::cout << "img_msg->points.size():" << img_msg->points.size() << std::endl;
//                std::cout << "img_msg->velocity_x_of_point:" << img_msg->velocity_x_of_point.size() << std::endl;

                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v/NUM_OF_CAM;
                int camera_id = v%NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            TicToc t_processImage;
            estimator.processImage(image, img_msg->header);

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                p_wi = estimator.Ps[WINDOW_SIZE];
                vPath_to_draw.push_back(p_wi);
                double dStamp = estimator.Headers[WINDOW_SIZE];
                cout << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " << dStamp << " p_wi: " << p_wi.transpose() << endl;
                ofs_pose << fixed << dStamp << " " << p_wi.transpose() << " " << q_wi.coeffs().transpose() << endl;
            }
        }
        m_estimator.unlock();
    }
}

void System::Draw()
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for (int i = 0; i < nPath_size - 1; ++i)
        {
//            std::cout << "In Pangolin 1:" << std::endl;
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i + 1].x(), vPath_to_draw[i + 1].y(), vPath_to_draw[i + 1].z());
        }
        glEnd();

        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            std::cout << "In Pangolin 2:" << std::endl;

            glPointSize(5);
            glBegin(GL_POINTS);
            for (int i = 0; i < WINDOW_SIZE + 1; ++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0], p_wi[1], p_wi[2]);
            }
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
