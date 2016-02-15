// =============================================================================
// Description: Provides a way to more easily generate and manage Bézier curves
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#ifndef BEZIER_CURVE_HPP
#define BEZIER_CURVE_HPP

#include <vector>

struct Point {
    Point(double x, double y);

    double x;
    double y;
};

class BezierCurve {
public:
    BezierCurve() = default;
    BezierCurve(const Point& pt1,
                const Point& pt2,
                const Point& pt3,
                const Point& pt4);

    void AddPoint(double x, double y);
    void Clear();

    // 'start' and 'end' represent start and end t values [0..1]
    double GetArcLength(double start, double end) const;
    double GetCurvature(double t) const;

    // Return value and first and second derivatives at parameter t
    double GetValueX(double t) const;
    double GetValueY(double t) const;
    double GetDerivativeX(double t) const;
    double GetDerivativeY(double t) const;
    double GetDerivative2X(double t) const;
    double GetDerivative2Y(double t) const;

private:
    std::vector<Point> m_pts;
};

#endif // BEZIER_CURVE_HPP