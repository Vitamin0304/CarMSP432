#include "PathGenerator.h"

#include <cmath>
namespace pathTrack {
    PathPart::PathPart(Vector2f& start, Vector2f& end, bool direction)
        :start(start), end(end), direction(direction)
    {

    }
    LinePath::LinePath(Vector2f& start, Vector2f& end, bool direction)
        : PathPart(start, end, direction)
    {
        distance = (this->end - this->start).norm();
    }

    ArcPath::ArcPath(Vector2f& start, Vector2f& end, bool direction, Vector2f& arcCenter)
        : PathPart(start, end, direction), arcCenter(arcCenter)
    {
        Vector2f O_start = this->start - this->arcCenter;
        Vector2f O_end = this->end - this->arcCenter;
        startAngle = atan2f(O_start[1], O_start[0]);
        endAngle = atan2f(O_end[1], O_end[0]);
        radius = O_start.norm();
    }
    int sign(float x)
    {
        if (x > 0)
            return 1;
        else if (x < 0)
            return -1;
        else
            return 0;
    }
    float mod2pi(float x)
    {
        if (x > EIGEN_PI)
            x -= 2 * EIGEN_PI;
        else if (x < -EIGEN_PI)
            x += 2 * EIGEN_PI;
        return x;
    }
    Vector2f LinePath::GetPoint(float length)
    {
        return start + length / distance * (end - start);
    }
    Vector2f ArcPath::GetPoint(float length)
    {
        Vector2f n(cos(length / radius * sign(mod2pi(endAngle - startAngle)) + startAngle),
            sin(length / radius * sign(mod2pi(endAngle - startAngle)) + startAngle));
        return arcCenter + radius * n;
    }
    bool LinePath::IsNearTheEnd(float length, float h)
    {
        if (abs(distance - length) < h)
            return true;
        else
            return false;
    }
    bool ArcPath::IsNearTheEnd(float length, float h)
    {
        if (abs(radius * abs(mod2pi(endAngle - startAngle)) - length) < h)
            return true;
        else
            return false;
    }

    PathGenerator::PathGenerator(float r)
    {
        truningRaduis = r;
    }
    inline void ComputeIntersectionPoint(float x1, Vector2f& P2, float k2, Vector2f& out)
    {
        out[0] = x1;
        out[1] = P2[1] + k2 * (x1 - P2[0]);
    }
    inline void ComputeIntersectionPoint(Vector2f& P1, float k1, Vector2f& P2, float k2, Vector2f& out)
    {
        out[0] = (P2[1] - P1[1] + k1 * P1[0] - k2 * P2[0]) / (k1 - k2);
        out[1] = P1[1] + k1 * (out[0] - P1[0]);
    }
    void PathGenerator::ComputeArc(Vector2f& A, Vector2f& C, Vector2f& E, float r, Vector2f& B_out, Vector2f& D_out, Vector2f& O_out)
    {
        Vector2f CA = A - C;
        Vector2f CE = E - C;
        float cos_ACE = CA.dot(CE) / (CA.norm() * CE.norm());
        float angle_ACE = acos(cos_ACE);
        float CO_norm = r / sin(angle_ACE / 2);

        Matrix2f den;
        den.block(0, 0, 1, 2) = CA.transpose();
        den.block(1, 0, 1, 2) = CE.transpose();
        float den_det = den.determinant();

        Vector2f b(CA.norm(), CE.norm());
        b *= CO_norm * cos(angle_ACE / 2);

        Vector2f CO;
        Matrix2f num = den;
        num.block(0, 0, 2, 1) = b;
        CO[0] = num.determinant() / den_det;

        num = den;
        num.block(0, 1, 2, 1) = b;
        CO[1] = num.determinant() / den_det;

        O_out = CO + C;
        B_out = C + r / tan(angle_ACE / 2) / CA.norm() * CA;
        D_out = C + r / tan(angle_ACE / 2) / CE.norm() * CE;
    }
    void PathGenerator::GeneratePathPoints(float h)
    {
        stepSum = 0;
        list<PathPart*>::iterator pPathPart;

        for (pPathPart = pathParts.begin(); pPathPart != pathParts.end(); ++pPathPart)
        {
            PathPart* pathPart = *pPathPart;
            uint16_t step = 0;
            Vector2f P;
            while (!pathPart->IsNearTheEnd(step * h, h))
            {
                P = pathPart->GetPoint(step * h);
                path[stepSum][0] = P[0];
                path[stepSum][1] = P[1];
                step++;
                stepSum++;
            }
            P = pathPart->GetPoint(step * h);
            path[stepSum][0] = P[0];
            path[stepSum][1] = P[1];
            step++;
            stepSum++;
            unsmoothSteps.push_back(stepSum);
            directions.push_back(pathPart->direction);
        }
    }
    char* PathGenerator::Generate(Vector3f& initParam, int method, float h)
    {
        if (!pathParts.empty())
        {
            pathParts.clear();
            directions.clear();
        }

        float& r = truningRaduis; //×ªÍä°ë¾¶

        if (method == 1)
        {
            float x_D = 0.145;
            float& theta = initParam[2];
            float k = tan(theta);

            Vector2f A(initParam[0], initParam[1]);
            Vector2f E(x_D, 0.08);

            //Vector2f O(x_D + r, k * (x_D + r) - k * A[0] + A[1] - r * powf(1 + k * k, 0.5));
            //Vector2f B(O[0] + r * cos(theta + EIGEN_PI / 2), O[1] + r * sin(theta + EIGEN_PI / 2));
            //
            //if (A[0] < x_D)
            //{
            //  O[0] = x_D - r;
            //  O[1] = k* (x_D - r) - k * A[0] + A[1] - r * powf(1 + k * k, 0.5);
            //  B[0] = O[0] + r * cos(theta - EIGEN_PI / 2);
            //  B[1] = O[1] + r * sin(theta - EIGEN_PI / 2);
            //}
            //
            //Vector2f D(x_D, O[1]);

            Vector2f B, C, D, O;
            ComputeIntersectionPoint(x_D, A, k, C);
            ComputeArc(A, C, E, r, B, D, O);

            LinePath line1(A, B, false);
            ArcPath arc1(B, D, false, O);
            LinePath line2(D, E, false);

            pathParts.push_back(&line1);
            pathParts.push_back(&arc1);
            pathParts.push_back(&line2);

            GeneratePathPoints(h);
        }
        else if (method == 0)
        {
            float& theta = initParam[2];
            float k_AB = tan(theta);

            Vector2f A(initParam[0], initParam[1]);

            Vector2f H(0.0, 0.16);
            Vector2f I(0.31, 0.28);
            float k_FI = tan(50 * EIGEN_PI / 180);

            Vector2f G, F, E, D, C, B, O2, O1;
            ComputeIntersectionPoint(H, 0, I, k_FI, F);
            ComputeArc(I, F, H, r, E, G, O2);

            ComputeIntersectionPoint(A, k_AB, I, k_FI, C);
            ComputeArc(A, C, E, r, B, D, O1);

            LinePath line1(A, B, false);
            ArcPath arc1(B, D, false, O1);
            LinePath line2(D, E, false);
            ArcPath arc2(E, G, false, O2);
            LinePath line3(G, H, false);

            Vector2f H2(H[0], 0.11);
            Vector2f J(0.2, 0.135);
            Vector2f K(0.0, 0.11);
            Vector2f L(0.1, 0.11);

            LinePath line4(H, J, true);
            LinePath line5(J, K, false);
            LinePath line6(K, L, true);

            pathParts.push_back(&line1);
            pathParts.push_back(&arc1);
            pathParts.push_back(&line2);
            pathParts.push_back(&arc2);
            pathParts.push_back(&line3);
            pathParts.push_back(&line4);
            pathParts.push_back(&line5);
            pathParts.push_back(&line6);

            GeneratePathPoints(h);
        }


        return GetPathPartsJsonString();
    }

    cJSON* LinePath::GetJson(bool isStart)
    {
        cJSON* root = cJSON_CreateObject();

        cJSON_AddItemToObject(root, "type", cJSON_CreateNumber(1));
        if (isStart)
        {
            cJSON_AddItemToObject(root, "start", cJSON_CreateFloatArray(start.data(), 2));
        }
        cJSON_AddItemToObject(root, "end", cJSON_CreateFloatArray(end.data(), 2));
        cJSON_AddBoolToObject(root, "direction", direction);
        return root;
    }
    cJSON* ArcPath::GetJson(bool isStart)
    {
        cJSON* root = cJSON_CreateObject();

        cJSON_AddItemToObject(root, "type", cJSON_CreateNumber(0));
        if (isStart)
        {
            cJSON_AddItemToObject(root, "start", cJSON_CreateFloatArray(start.data(), 2));
        }
        cJSON_AddItemToObject(root, "end", cJSON_CreateFloatArray(end.data(), 2));
        cJSON_AddItemToObject(root, "center", cJSON_CreateFloatArray(arcCenter.data(), 2));
        cJSON_AddBoolToObject(root, "direction", direction);
        return root;
    }

    char* PathGenerator::GetPathPartsJsonString()
    {
        cJSON* root_json = cJSON_CreateObject();
        cJSON* path_parts_json = cJSON_CreateArray();

        list<PathPart*>::iterator pPathPart;

        for (pPathPart = pathParts.begin(); pPathPart != pathParts.end(); ++pPathPart)
        {
            PathPart* pathPart = *pPathPart;
            bool isStart = (pPathPart == pathParts.begin());

            cJSON_AddItemToArray(path_parts_json, pathPart->GetJson(isStart));
        }
        cJSON_AddItemToObject(root_json, "path_parts", path_parts_json);
        char* root = cJSON_PrintUnformatted(root_json);
        cJSON_Delete(root_json);
        return root;
    }
}
