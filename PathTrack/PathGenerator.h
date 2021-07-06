#ifndef PATHTRACK_PATHTGENERATOR_H_
#define PATHTRACK_PATHTGENERATOR_H_

#include <cstdint>
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Apps/cJSON.h>
using namespace std;
using namespace Eigen;
namespace pathTrack {
    int sign(float x);
    float mod2pi(float x);

    class PathPart
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PathPart(Vector2f& start, Vector2f& end, bool direction);

        Vector2f start;
        Vector2f end;

        bool direction; //true表示前进，false表示后退
        //获得从起点到终点方向向前length长度位置的坐标
        virtual Vector2f GetPoint(float length) = 0;
        virtual bool IsNearTheEnd(float length, float h) = 0;

        virtual cJSON* GetJson(bool start) = 0;
    };
    class LinePath : public PathPart
    {
    public:
        LinePath(Vector2f& start, Vector2f& end, bool direction);
        Vector2f GetPoint(float length) override;
        bool IsNearTheEnd(float length, float h) override;

        float distance; //起点到终点的路程

        cJSON* GetJson(bool start) override;
    };
    class ArcPath : public PathPart
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ArcPath(Vector2f& start, Vector2f& end, bool direction, Vector2f& arcCenter);
        Vector2f GetPoint(float length) override;
        bool IsNearTheEnd(float length, float h) override;

        Vector2f arcCenter;
        float startAngle; //起始角度
        float endAngle; //扫过的角度
        float radius; //半径

        cJSON* GetJson(bool start) override;
    };

    class PathGenerator
    {
    public:
        PathGenerator(float r);
        float truningRaduis;
        float path[1024][2] = { 0 };
        uint16_t stepSum = 0;
        vector<uint16_t> unsmoothSteps;
        vector<bool> directions;
        char* Generate(Vector3f& initParam, int method, float h);
    private:
        list<PathPart*> pathParts;
//        void ComputeIntersectionPoint(float x1, Vector2f& P2, float k2, Vector2f& out);
//        void ComputeIntersectionPoint(Vector2f& P1, float k1, Vector2f& P2, float k2, Vector2f& out);
        void ComputeArc(Vector2f& A, Vector2f& C, Vector2f& E, float r, Vector2f& B_out, Vector2f& D_out, Vector2f& O_out);
        void GeneratePathPoints(float h);
        char* GetPathPartsJsonString();
    };
}


#endif
