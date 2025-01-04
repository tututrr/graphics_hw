#include "my_lib.h" 

using namespace std;
using namespace cv;

CLPointType classify(Point2f p1, Point2f p2, Point2f p) {
    double ax = p2.x - p1.x;
    double ay = p2.y - p1.y;
    double bx = p.x - p1.x;
    double by = p.y - p1.y;
    double s = ax * by - bx * ay;
    
    if (s > 0) return LEFT;
    if (s < 0) return RIGHT;
    if ((ax * bx < 0) || (ay * by < 0)) return BEHIND;
    if ((ax * ax + ay * ay) < (bx * bx + by * by)) return BEYOND;
    if (p1.x == p.x && p1.y == p.y) return ORIGIN;
    if (p2.x == p.x && p2.y == p.y) return DESTINATION;
    return BETWEEN;
}

IntersectType intersect(Point2f a, Point2f b, Point2f c, Point2f d, double* t) {
    double nx = d.y - c.y;
    double ny = c.x - d.x;
    CLPointType type;
    double denom = nx * (b.x - a.x) + ny * (b.y - a.y);
    if (denom == 0) {
        type = classify(c, d, a);
        if (type == LEFT || type == RIGHT) {
            return PARALLEL;
        }
        else {
            return SAME;
        }
    }
    double num = nx * (a.x - c.x) + ny * (a.y - c.y);
    *t = -num/denom;
    return SKEW;
}

IntersectType cross(Point2f a, Point2f b, Point2f c, Point2f d, double* tab, double* tcd) {
    IntersectType type = intersect(a, b, c, d, tab);
    if (type == SAME || type == PARALLEL) {
        return type;
    }
    if ((*tab < 0) || (*tab > 1)) {
        return SKEW_NO_CROSS;
    }
    intersect(c, d, a, b, tcd);
    if ((*tcd < 0) || (*tcd > 1)) {
        return SKEW_NO_CROSS;
    }
    return SKEW_CROSS;
}

vector<Edge> getEdges(vector<Point2f> points) {
    vector<Edge> edges;
    for (int i = 0; i < points.size(); ++i) {
        Edge edge;
        edge.points.push_back(points[i]);
        edge.intersectsWith.push_back(-1);
        edge.points.push_back(points[(i + 1) % points.size()]);
        edge.intersectsWith.push_back(-1);
        edges.push_back(edge);
    }
    return edges;
}

vector<Intersection> getIntersections(vector<Point2f> points) {
    int n = points.size();
    vector<Intersection> intersections;
    for (int i = 0; i < points.size(); ++i) {
        for (int j = i + 1; j < points.size(); ++j) {
            if (j == (i + 1) % n || (j + 1) % n == i) {
                continue;
            }
            double tab, tcd;
            IntersectType type = cross(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n], &tab, &tcd);
            if (type == SKEW_CROSS) {
                Intersection intersection;
                Point2f inter(points[i] + tab * (points[(i + 1) % n] - points[i]));
                intersection.p = inter;
                intersection.intersectionOf.push_back(i);
                intersection.intersectionOf.push_back(j);
                intersections.push_back(intersection);
            }
        }
    }
    return intersections;
}

vector<Intersection> getAllIntersections(vector<Intersection> intersections) {
    vector<Intersection> allIntersections(intersections.begin(), intersections.end());
    int n = intersections.size();
    for (int i = 0; i < n; ++i) {
        Intersection newIntersection;
        vector<int> intersectionOf(intersections[i].intersectionOf.rbegin(), intersections[i].intersectionOf.rend());
        Point2f inter = intersections[i].p;
        newIntersection.p = inter;
        newIntersection.intersectionOf = intersectionOf;
        allIntersections.push_back(newIntersection);
    }
    return allIntersections;
}

void addIntersectionsToEdges(vector<Edge>& edges, vector<Intersection> intersections) {
    for (int i = 0; i < edges.size(); ++i) {
        for (int k = 0; k < intersections.size(); ++k) {
            if (intersections[k].intersectionOf[0] == i) {
                for (int j = 0; j < edges[i].points.size(); ++j) {
                    if (intersections[k].p.x <= max(edges[i].points[j].x, edges[i].points[(j + 1) % edges[i].points.size()].x) &&intersections[k].p.x >= min(edges[i].points[j].x, edges[i].points[(j + 1) % edges[i].points.size()].x) 
                    && intersections[k].p.y <= max(edges[i].points[j].y, edges[i].points[(j + 1) % edges[i].points.size()].y) && intersections[k].p.y >= min(edges[i].points[j].y, edges[i].points[(j + 1) % edges[i].points.size()].y)) {
                        edges[i].points.insert(edges[i].points.begin() + j + 1, intersections[k].p);
                        edges[i].intersectsWith.insert(edges[i].intersectsWith.begin() + j + 1, intersections[k].intersectionOf[1]);
                        break;
                    }
                }
            }
        }
    }
}

vector<Point2f> getContour(vector<Point2f> points) {
    vector<Point2f> result;
    vector<Edge> edges = getEdges(points);
    vector<Intersection> intersections = getAllIntersections(getIntersections(points));
    addIntersectionsToEdges(edges, intersections);
    Point2f point = edges[0].points[0];
    Point2f intersectionPoint;
    int i = 0, j = 1;
    do {
        if (point == edges[i].points[edges[i].points.size() - 1]) {
            i++;
            j = 0;
        }
        result.push_back(point);
        point = edges[i].points[j];
        while (edges[i].intersectsWith[j] == -1) {
            j++;
            result.push_back(point);
            point = edges[i].points[j];
        }
        result.push_back(point);
        intersectionPoint = edges[i].points[j];
        int intersectingEdge = edges[i].intersectsWith[j];
        j = 0;
        while (edges[intersectingEdge].points[j] != intersectionPoint) {
            j++;
        }
        j++;
        point = edges[intersectingEdge].points[j];
        i = intersectingEdge;
    } while (point != points[0]);
    return result;
}


void drawLine(Mat& img, Point2f p1, Point2f p2) {
    if (p1 == p2) return; 

    int x2 = p2.x, y2 = p2.y;
    int x = p1.x, y = p1.y;
    int dx = (x < x2) ? x2 - x : x - x2;
    int dy = (y < y2) ? y2 - y : y - y2;
    int ix = (x < x2) ? 1 : -1;
    int iy = (y < y2) ? 1 : -1;
    int error;
    
    if (dx >= dy) {
        error = 2 * dy - dx;
        if (iy >= 0) {
            for (int i = 0; i < dx; ++i) {
                img.at<Vec3b>(img.rows - y, x) = Vec3b(255, 0, 255);
                if (error >= 0) {
                    y += iy;
                    error -= 2 * dx;
                }
                x += ix;
                error += 2 * dy;
            }
        }
        else {
            for (int i = 0; i < dx; ++i) {
                img.at<Vec3b>(img.rows - y, x) = Vec3b(255, 0, 255);
                if (error > 0) {
                    y += iy;
                    error -= 2 * dx;
                }
                x += ix;
                error += 2 * dy;
            }
        }
    }
    else {
        error = 2 * dx - dy;
        if (iy >= 0) {
            for (int i = 0; i < dy; ++i) {
                img.at<Vec3b>(img.rows - y, x) = Vec3b(255, 0, 255);
                if (error >= 0) {
                    x += ix;
                    error -= 2 * dy;
                }
                y += iy;
                error += 2 * dx;
            }
        }
        else {
            for (int i = 0; i < dy; ++i) {
                img.at<Vec3b>(img.rows - y, x) = Vec3b(255, 0, 255);
                if (error > 0) {
                    x += ix;
                    error -= 2 * dy;
                }
                y += iy;
                error += 2 * dx;
            }
        }
    }
}

void drawPolygon(Mat& img, vector<Point2f>& points) {
    for (int i = 0; i < points.size(); ++i) {
        drawLine(img, points[i], points[(i + 1) % points.size()]);
    }
}

Point getCubicBezierCurvePoint(Point p0, Point p1, Point p2, Point p3, double t) {
    double B0 = (1 - t) * (1 - t) * (1 - t);
    double B1 = 3 * t * (1 - t) * (1 - t);
    double B2 = 3 * t * t * (1 - t);
    double B3 = t * t * t;
    int x = int(round(B0 * p0.x + B1 * p1.x + B2 * p2.x + B3 * p3.x));
    int y = int(round(B0 * p0.y + B1 * p1.y + B2 * p2.y + B3 * p3.y));
    return Point(x, y);
}

void drawCubicBezierCurve(Mat &img, Point p0, Point p1, Point p2, Point p3, int N) {
    double step = 1.0 / (N - 1);
    for (int i = 0; i < N; ++i) {
        Point p = getCubicBezierCurvePoint(p0, p1, p2, p3, i * step);
        Point q = getCubicBezierCurvePoint(p0, p1, p2, p3, (i + 1) * step);
        drawLine(img, p, q);
    }
}

Point findIntersection(Point O, double R, Point A, Point B) {
    double a1 = A.x - O.x;
    double b1 = A.y - O.y;
    double c1 = R * R;

    double a2 = B.x - O.x;
    double b2 = B.y - O.y;
    double c2 = R * R;

    double d1 = c1 + O.x * a1 + O.y * b1;
    double d2 = c2 + O.x * a2 + O.y * b2;

    double det = a1 * b2 - a2 * b1;
    if (abs(det) < 1e-9) {
        cout << "Касательные параллельны или совпадают!" << endl;
        return Point(0, 0);
    }

    double x = (d1 * b2 - d2 * b1) / det;
    double y = (a1 * d2 - a2 * d1) / det;

    return Point(int(x), int(y));
}

void drawCirclePart(Mat& img, Point center, double radius, double alpha, double beta) {
    double quarterLen = M_PI * radius / 2;

    if (alpha > 180) alpha = alpha - 360;
    if (beta > 180) beta = beta - 360;
    if (alpha > beta) {
        double tmp = alpha;
        alpha = beta;
        beta = tmp;
    }
    
    double len = 2 * M_PI * radius * (beta - alpha) / 360;
    alpha = alpha * M_PI / 180;
    beta = beta * M_PI / 180;
    int numParts = int(ceil(len / quarterLen));
    double angleStep = (beta - alpha) / numParts;
    if (alpha == beta) {
        angleStep = M_PI / 2;
        numParts = 4;
    }

    for (int i = 0; i < numParts; ++i) {
        Point p0 = Point(center.x + int(round(radius * cos(alpha  + i * angleStep))), 
                         center.y + int(round(radius * sin(alpha  + i * angleStep))));
        if (i == 0 && alpha != beta) {
            drawLine(img, center, p0);
        }
        Point p3 = Point(center.x + int(round(radius * cos(alpha  + (i + 1) * angleStep))), 
                         center.y + int(round(radius * sin(alpha + (i + 1) * angleStep))));
        if (i == numParts - 1 && alpha != beta) {
            drawLine(img, center, p3);
        }
        Point Pt = findIntersection(center, radius, p0, p3);
        double d = norm(Pt - p0);
        double F = 4 / (3 + 3 * sqrt(1 + (d / radius) * (d / radius)));
        Point p1 = p0 + F * (Pt - p0);
        Point p2 = p3 + F * (Pt - p3);
        drawCubicBezierCurve(img, p0, p1, p2, p3, 1000);
    }
}

double getDist(Vec3f a, Vec3f b) {
    return sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

Mat kMeans(Mat& img, int k) {
    Mat data;
    img.convertTo(data, CV_32FC3);
    data = data.reshape(1, img.rows * img.cols);
    
    vector<Vec3f> centers(k);
    vector<int> labels(data.rows);

    srand(time(0));
    for (int i = 0; i < k; ++i) {
        centers[i] = data.at<Vec3f>(rand() % data.rows, 0);
    }

    bool hasChanged = true;
    int iterations = 0;

    while (hasChanged && iterations < 100) {
        hasChanged = false;
        for (int i = 0; i < data.rows; ++i) {
            float minDist = FLT_MAX;
            int bestCluster = 0;
            for (int j = 0; j < k; ++j) {
                float dist = getDist(data.at<Vec3f>(i, 0), centers[j]);
                if (dist < minDist) {
                    minDist = dist;
                    bestCluster = j;
                }
            }
            if (labels[i] != bestCluster) {
                labels[i] = bestCluster;
                hasChanged = true;
            }
        }

        vector<Vec3f> newCenters(k, Vec3f(0, 0, 0));
        vector<int> counts(k, 0);

        for (int i = 0; i < data.rows; ++i) {
            int cluster = labels[i];
            newCenters[cluster] += data.at<Vec3f>(i, 0);
            counts[cluster]++;
        }

        for (int j = 0; j < k; ++j) {
            if (counts[j] > 0) {
                centers[j] = newCenters[j] / counts[j];
            }
        }

        iterations++;
    }

    Mat result(img.size(), img.type());
    for (int i = 0; i < data.rows; ++i) {
        int cluster = labels[i];
        Vec3f color = centers[cluster];
        result.at<Vec3b>(i / img.cols, i % img.cols) = Vec3b(
            static_cast<uchar>(min(255.0f, max(0.0f, color[0]))),
            static_cast<uchar>(min(255.0f, max(0.0f, color[1]))),
            static_cast<uchar>(min(255.0f, max(0.0f, color[2])))
        );
    }
    return result;
}
