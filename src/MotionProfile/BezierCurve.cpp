// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "BezierCurve.hpp"

#include <cmath>

Point::Point(double x, double y) {
    this->x = x;
    this->y = y;
}

BezierCurve::BezierCurve(const Point& pt1, const Point& pt2, const Point& pt3,
                         const Point& pt4) {
    m_pts.push_back(pt1);
    m_pts.push_back(pt2);
    m_pts.push_back(pt3);
    m_pts.push_back(pt4);
}

void BezierCurve::AddPoint(double x, double y) { m_pts.emplace_back(x, y); }

void BezierCurve::Clear() { m_pts.clear(); }

double BezierCurve::GetArcLength(double start, double end) const {
    double length = 0.0;

    for (double t = start; t < end; t += 0.0001) {
        length += std::hypot(GetDerivativeX(t), GetDerivativeY(t)) * 0.0001;
    }

    return length;
}

double BezierCurve::GetCurvature(double t) const {
    return (GetDerivativeX(t) * GetDerivative2Y(t) -
            GetDerivativeY(t) * GetDerivative2X(t)) /
           std::pow(std::pow(GetDerivativeX(t), 2.0) +
                        std::pow(GetDerivativeY(t), 2.0),
                    1.5);
}

double BezierCurve::GetValueX(double t) const {
    return std::pow(1 - t, 3) * m_pts[0].x +
           3.0 * std::pow(1 - t, 2) * t * m_pts[1].x +
           3.0 * (1 - t) * std::pow(t, 2) * m_pts[2].x +
           std::pow(t, 3) * m_pts[3].x;
}

double BezierCurve::GetValueY(double t) const {
    return std::pow(1 - t, 3) * m_pts[0].y +
           3.0 * std::pow(1 - t, 2) * t * m_pts[1].y +
           3.0 * (1 - t) * std::pow(t, 2) * m_pts[2].y +
           std::pow(t, 3) * m_pts[3].y;
}

double BezierCurve::GetDerivativeX(double t) const {
    return 3.0 * std::pow(1 - t, 2) * (m_pts[1].x - m_pts[0].x) +
           6.0 * (1 - t) * t * (m_pts[2].x - m_pts[1].x) +
           3.0 * std::pow(t, 2) * (m_pts[3].x - m_pts[2].x);
}

double BezierCurve::GetDerivativeY(double t) const {
    return 3.0 * std::pow(1 - t, 2) * (m_pts[1].y - m_pts[0].y) +
           6.0 * (1 - t) * t * (m_pts[2].y - m_pts[1].y) +
           3.0 * std::pow(t, 2) * (m_pts[3].y - m_pts[2].y);
}

double BezierCurve::GetDerivative2X(double t) const {
    return 6.0 * (1 - t) * (m_pts[2].x - 2.0 * m_pts[1].x + m_pts[0].x) +
           6.0 * t * (m_pts[3].x - 2.0 * m_pts[2].x + m_pts[1].x);
}

double BezierCurve::GetDerivative2Y(double t) const {
    return 6.0 * (1 - t) * (m_pts[2].y - 2.0 * m_pts[1].y + m_pts[0].y) +
           6.0 * t * (m_pts[3].y - 2.0 * m_pts[2].y + m_pts[1].y);
}
