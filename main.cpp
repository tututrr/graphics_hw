#include "my_lib.h" 

#include <iostream>
using namespace std;

int main() {
    Mat img(1000, 1000, CV_8UC3, Scalar(0, 0, 0));

    //task 1
    vector<Point2f> points = {
        {450, 250},
        {88, 368},
        {312, 60},
        {312, 440},
        {88, 132},
    };

    vector<Intersection> intersections = getAllIntersections(getIntersections(points));
    for (int i = 0; i < intersections.size(); ++i) {
        cout << intersections[i].p << " ";
        for (int j = 0; j < intersections[i].intersectionOf.size(); ++j) {
            cout << intersections[i].intersectionOf[j] << " ";
        }
        cout << endl;
    }
    vector<Edge> edges = getEdges(points);
    addIntersectionsToEdges(edges, intersections);
    for (int i = 0; i < edges.size(); ++i) {
        cout << "Edge " << i << ": [";
        for (int j = 0; j < edges[i].points.size(); ++j) {
            cout << edges[i].points[j] << " ";
        }
        cout << "]" << endl << "Intersects with edge: [";
        for (int j = 0; j < edges[i].intersectsWith.size(); ++j) {
            cout << " " << edges[i].intersectsWith[j];
        }
        cout << "]" << endl;
    }
    vector<Point2f> contour = getContour(points);
    drawPolygon(img, points);
    imwrite("../result/task1_polygon.jpg", img);
    img.setTo(Scalar(0, 0, 0));
    drawPolygon(img, contour);
    imwrite("../result/task1_contour_polygon.jpg", img);

    //task 2
    img.setTo(Scalar(0, 0, 0));
    drawCirclePart(img, Point(600, 400), 400.0, 100.0, 150.0);
    imwrite("../result/task2_arc.png", img);

    //task3
    Mat img0 = imread("../cat.png", IMREAD_COLOR);
    if (img0.empty()) {
        cout << "Error loading image\n" << endl;
        return -1;
    }

    int k;
    cout << "Enter k: ";
    cin >> k;

    imwrite("../result/task3_cat_k_means.png", kMeans(img0, k));
    
    return 0;
}