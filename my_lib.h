#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#define _USE_MATH_DEFINES

using namespace cv;
using namespace std;


enum CLPointType {LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION};
enum IntersectType {SAME, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS};

struct Edge {
    vector<Point2f> points;
    vector<int> intersectsWith;
};

struct Intersection {
    Point2f p;
    vector<int> intersectionOf;
};

CLPointType classify(Point2f p1, Point2f p2, Point2f p);
IntersectType intersect(Point2f a, Point2f b, Point2f c, Point2f d, double* t);
IntersectType cross(Point2f a, Point2f b, Point2f c, Point2f d, double* tab, double* tcd);
vector<Edge> getEdges(vector<Point2f> points);
vector<Intersection> getIntersections(vector<Point2f> points);
vector<Intersection> getAllIntersections(vector<Intersection> intersections);
void addIntersectionsToEdges(vector<Edge>& edges, vector<Intersection> intersections);
vector<Point2f> getContour(vector<Point2f> points);
void drawLine(Mat& img, Point2f p1, Point2f p2);
void drawPolygon(Mat& img, vector<Point2f>& points);

Point getCubicBezierCurvePoint(Point p0, Point p1, Point p2, Point p3, double t);
void drawCubicBezierCurve(Mat &img, Point p0, Point p1, Point p2, Point p3, int N);
Point findIntersection(Point O, double R, Point A, Point B);
void drawCirclePart(Mat& img, Point center, double radius, double alpha, double beta);

double getDist(Vec3f a, Vec3f b);
Mat kMeans(Mat& img, int k);
