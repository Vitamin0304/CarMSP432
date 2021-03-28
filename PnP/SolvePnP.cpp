/*
 * SolvePnp.cpp
 *
 *  Created on: 2021年3月16日
 *      Author: 电脑
 */

#include <Eigen/Core>

#include "types.hpp"
#include "SRPnP.hpp"

void SolvePnP(Eigen::Matrix<double,3,3> & pixel,Eigen::Vector2d & F_Position)
{
    Eigen::Matrix3d cameraMatrix1;
    cameraMatrix1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    double fx = 661.2594371919303;
    double fy = 662.0518782496423;
    double cx = 310.9395899388371;
    double cy = 238.8397509505573;

    cameraMatrix1(0, 0) = fx;
    cameraMatrix1(1, 1) = fy;
    cameraMatrix1(0, 2) = cx;
    cameraMatrix1(1, 2) = cy;

    Eigen::Matrix<double,3,4> pixel3x4;
    pixel3x4 << 209, 370, 510, 339, 190, 373, 282, 135, 1, 1, 1, 1;
    pixel3x4 = cameraMatrix1.inverse() * pixel3x4;

    gv::Image_points imagePoints(0);
    gv::Image_point imagePoint(0, 0);
    for (int i = 0; i < 4; i++) {
        imagePoint[0] = pixel3x4(0, i);
        imagePoint[1] = pixel3x4(1, i);
        imagePoints.push_back(imagePoint);
    }

    gv::points_t  worldBoxPoints(0);
    gv::point_t tmpPoint;
    tmpPoint = Eigen::Vector3d((float)226, (float)0, (float)0);
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Eigen::Vector3d((float)226, (float)365, (float)0);
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Eigen::Vector3d((float)0, (float)365, (float)0);
    worldBoxPoints.push_back(tmpPoint);
    tmpPoint = Eigen::Vector3d((float)0, (float)0, (float)0);
    worldBoxPoints.push_back(tmpPoint);

    Eigen::Matrix3d  R;
    Eigen::Vector3d  t;

    gv::transformation_t Rt;

    Rt = gv::SRPnP::srpnp(imagePoints, worldBoxPoints);
    R = Rt.block(0, 0, 3, 3);
    t = Rt.block(0, 3, 3, 1);

    Eigen::Matrix<double, 3, 1> camera_cordinates;
    camera_cordinates = -R.inverse() * t;

    F_Position = camera_cordinates.block(0, 0, 2, 1);
}
