#ifndef CONFIGURATION_UTILS_H
#define CONFIGURATION_UTILS_H
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iostream>

typedef struct
{
    float x;
    float y;
}PointXY;

static inline PointXY
    createPointXY(float x, float y){
        PointXY point;
        point.x = x;
        point.y = y;
        return point;
}
typedef struct{
    float a;//y = a*x + b
    float b;
    float theta; // 该段直线的倾角
    PointXY start_point;
    PointXY end_point;
}LinePara;

class LaserPoint
{
public:
    int index;
    float rho;
    float theta;
    float x;
    float y;

    LaserPoint(){}
    LaserPoint(int i, float r, float t) :
        index(i),
        rho(r),
        theta(t)
    {
        x = rho*cos(theta);
        y = rho*sin(theta);
    }
    ~LaserPoint(){}
    inline void setPoint(int i, float r, float t)
    {
        index = i;
        rho = r;
        theta = t;
        x = rho*cos(theta);
        y = rho*sin(theta);
    }
    inline void setPointXY(int i, float X, float Y)
    {
        index = i;
        x = X;
        y = Y;
        rho = sqrt(x*x+y*y);
        theta = atan2(y, x);
    }
    inline LaserPoint& operator=(LaserPoint p)
    {
        this->index = p.index;
        this->rho = p.rho;
        this->theta = p.theta;
        this->x = p.x;
        this->y = p.y;
        return *this;
    }
    inline float dist(LaserPoint p)
    {
        return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
    }
};

enum GoalSituation
{
    FREE_GOAL,
    DANGEROUS_GOAL
};

enum ClosestGapSituation
{
    WIDE_GAP,
    NARROW_GAP
};

enum SubgoalSituation
{
    FREE_SUBGOAL,
    DANGEROUS_SUBGOAL
};

enum SafetySituation
{
    HIGH_SAFETY,
    LOW_SAFETY
};

enum CollisionSituation
{
    NO_COLLISION,
    FORWARD_COLLISION,
    BACKWARD_COLLISION
};

static inline float getDistToPoint(PointXY p1, PointXY p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

static inline PointXY getClosestPointOnLineSegment(PointXY p, LinePara l)
{
    PointXY diff = createPointXY(l.end_point.x-l.start_point.x, l.end_point.y-l.start_point.y);
    float u = ((p.x-l.start_point.x)*diff.x+(p.y-l.start_point.y)*diff.y)/(diff.x*diff.x+diff.y*diff.y);
    if (u <= 0) return l.start_point;
    else if (u >= 1) return l.end_point;

    return createPointXY(l.start_point.x+u*diff.x, l.start_point.y+u*diff.y);

}

static inline float getDistToLine(PointXY p, LinePara l)
{
    return getDistToPoint(p, getClosestPointOnLineSegment(p, l));
}

static inline float convergeAngle(float alpha)
{
    while(alpha > M_PI)
        alpha -= 2*M_PI;
    while(alpha < -M_PI)
        alpha += 2*M_PI;
    return alpha;
}

static inline float sgn(float alpha)
{
    if(0 == alpha)
        return 0;
    else if(alpha < 0)
        return -1;
    else
        return 1;
}

static inline float sat(float alpha, float min_alpha, float max_alpha)
{
    return std::min(std::max(alpha, min_alpha), max_alpha);
}

#endif // CONFIGURATION_UTILS_H
