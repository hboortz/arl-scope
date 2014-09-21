#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>


#define MIN_HESSIAN 400

using namespace cv;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage:\n\t" << argv[0] << " fiducial image\n";
        return 1;
    }

    // Load images
    Mat img_object = imread(argv[1], 0);//CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_scene = imread(argv[2], 0);//CV_LOAD_IMAGE_GRAYSCALE);

    std::vector<KeyPoint> obj_kp, scene_kp;
    Mat obj_desc, scene_desc;    

    // Detect keypoints
    SurfFeatureDetector detector(MIN_HESSIAN);
    detector.detect(img_object, obj_kp);
    detector.detect(img_scene, scene_kp);

    // Calculate feature descriptors
    SurfDescriptorExtractor extractor;
    extractor.compute(img_object, obj_kp, obj_desc);
    extractor.compute(img_scene, scene_kp, scene_desc);

    // Match descriptors
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(obj_desc, scene_desc, matches); 

    double max_dist = 0, min_dist = 100;
    for (int i = 0; i < obj_desc.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    std::vector<DMatch> good_matches;
    for (int i = 0; i < obj_desc.rows; i++)
    {
        if (matches[i].distance < 3*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_matches;
    drawMatches( img_object, obj_kp, img_scene, scene_kp,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    std::cout << good_matches.size() << " good matches\n";
    if (good_matches.size() > 0)
    {
        for (int i = 0; i < good_matches.size(); i++)
        {
            obj.push_back(obj_kp[good_matches[i].queryIdx].pt);
            scene.push_back(scene_kp[good_matches[i].trainIdx].pt);
        }

        Mat H = findHomography(obj, scene, CV_LMEDS);

        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform(obj_corners, scene_corners, H);
        
        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
      line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

    }

    imshow("Matches", img_matches);
    waitKey(0);
    return 0;
}

