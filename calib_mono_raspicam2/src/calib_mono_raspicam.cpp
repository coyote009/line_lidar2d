#include <opencv2/opencv.hpp>
//#include <raspi_cam_if.h>
#include "raspivideocap2.h"

#ifdef _WIN32

#define OCV_VER_STR CVAUX_STR(CV_VERSION_MAJOR) CVAUX_STR(CV_VERSION_MINOR) CVAUX_STR(CV_VERSION_REVISION)

#ifdef _DEBUG
#define OCV_LIB_EXT "d.lib"
#else
#define OCV_LIB_EXT ".lib"
#endif

#pragma comment( lib, "opencv_core" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgproc" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_calib3d" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_highgui" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_videoio" OCV_VER_STR OCV_LIB_EXT )

#endif

struct calib_param
{
    cv::Mat mat_cam;
    cv::Mat vec_dist;
};

static bool find_corners( const cv::Size &size_pat, const cv::Mat &img_in, std::vector<cv::Point2f> &corners )
{
    cv::Mat img_proc;
    if( img_in.channels() == 3 )
    {
        cv::cvtColor( img_in, img_proc, cv::COLOR_BGR2GRAY );
    }
    else
    {
        img_proc = img_in;
    }

    bool found = cv::findChessboardCorners( img_proc, size_pat, corners,
                                            cv::CALIB_CB_FAST_CHECK |
                                            cv::CALIB_CB_ADAPTIVE_THRESH |
                                            cv::CALIB_CB_NORMALIZE_IMAGE );

    if( found )
    {
        cv::cornerSubPix( img_proc, corners, cv::Size( 11, 11 ), cv::Size(-1,-1),
                          cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1 ) );
    }

    return found;
}

static void calibrate( const cv::Size &size_img, const cv::Size &size_pat, float pat_len, std::vector< std::vector<cv::Point2f> > &corner2d_list, calib_param &calib )
{
    std::vector< std::vector<cv::Point3f> > corner3d_list;

    for( int i = 0; i < corner2d_list.size(); i++ )
    {
        std::vector<cv::Point3f> corners;
        for( int y = 0; y < size_pat.height; y++ )
        {
            for( int x = 0; x < size_pat.width; x++ )
            {
                corners.push_back( cv::Point3f( x * pat_len, y * pat_len, 0.0 ) );
            }
        }

        corner3d_list.push_back( corners );
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    /* assume HFoV=60deg for initial guess */
    double f = size_img.width / 2.0 / tan( 30.0 / 180.0 * CV_PI );
    calib.mat_cam = ( cv::Mat_<double>( 3, 3 ) <<
        f,   0.0, size_img.width / 2.0,
        0.0,   f, size_img.height / 2.0,
        0.0, 0.0, 1.0 );

    double err;
    err = cv::calibrateCamera( corner3d_list, corner2d_list, size_img, calib.mat_cam, calib.vec_dist, rvecs, tvecs,
                               cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO );

    printf( "Reprojection error = %f\n", err );
}

static void scale_show( const std::string &win_name, const cv::Mat &img, const cv::Size &size_show )
{
    cv::Mat img_show;
    
    if( size_show != img.size() )
    {
        cv::resize( img, img_show, size_show, 0, 0, cv::INTER_LINEAR );
    }
    else
    {
        img_show = img;
    }

    cv::imshow( win_name, img_show );
}

int main()
{
    const int file_input = 0;
    std::string fname_frame_idx;
    const cv::Size size_pat( 6, 5 );
    float pat_len = 0.03;
    std::string fname_calib = "../work/calib.yml";
    //const cv::Size size_in( 1920, 1440 );
    const cv::Size size_in( 640, 480 );
    const cv::Size size_show( 640, 480 );

    //RaspiVideoCapture cap;
    RaspiVideoCapture2 cap;
    if( file_input )
    {
        fprintf( stderr, "Not implemented yet\n" );
        return 1;
    }
    else
    {
        //if( !cap.open( size_in.width, size_in.height, 30, 0, 1, 1 ) )
        if( !cap.open( size_in.width, size_in.height ) )
        {
            fprintf( stderr, "Failed to open camera\n" );
            return 1;
        }
    }

    cv::Size size_subpic = size_in / 4;
    cv::Mat img_history;
    img_history = cv::Mat::zeros( size_in, CV_8UC3 );
    scale_show( "img_history", img_history, size_show );

    calib_param calib;
    std::vector< std::vector<cv::Point2f> > corner2d_list;

    cv::FileStorage fs;
    if( !fs.open( fname_calib, cv::FileStorage::WRITE ) )
    {
        fprintf( stderr, "Failed to open calibration file\n" );
        return 1;
    }

    while( 1 )
    {
        cv::Mat img_in;
        if( !cap.read( img_in ) )
        {
            break;
        }

        scale_show( "img_in", img_in, size_show );

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
        else if( key == ' ' )
        {
            std::vector<cv::Point2f> corners;

            if( find_corners( size_pat, img_in, corners ) )
            {
                cv::Mat img_corners;

                img_in.copyTo( img_corners );
                cv::drawChessboardCorners( img_corners, size_pat, corners, true );

                cv::putText( img_in, "Use corners of this picture(y/n)?", cv::Point( 15, 15 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0), 2 );

                scale_show( "img_in", img_in, size_show );
                scale_show( "img_corners", img_corners, size_show );
                if( cv::waitKey() == 'y' )
                {
                    int corner_idx = corner2d_list.size();
                    cv::Point img_idx( corner_idx % 4, corner_idx / 4 );
                    cv::Rect roi( size_subpic.width * img_idx.x, size_subpic.height * img_idx.y,
                                  size_subpic.width, size_subpic.height );
                    cv::resize( img_corners, img_history( roi ), size_subpic, 0, 0, cv::INTER_LINEAR );
                    scale_show( "img_history", img_history, size_show );
                    cv::waitKey( 1 );

                    corner2d_list.push_back( corners );

                    if( corner2d_list.size() == 16 )
                    {
                        calibrate( size_in, size_pat, pat_len, corner2d_list, calib );

                        fs << "MAT_CAM" << calib.mat_cam;
                        fs << "VEC_DIST" << calib.vec_dist;
                        fs << "SIZE_IN" << size_in;

                        break;
                    }
                }
            }
        }
    }

    if( corner2d_list.size() < 16 )
    {
        return 1;
    }

    /* Test undistortion */
    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap( calib.mat_cam, calib.vec_dist, cv::Mat(), calib.mat_cam, size_in, CV_32FC1, mapx, mapy );

    while( 1 )
    {
        cv::Mat img_in;
        if( !cap.read( img_in ) )
        {
            break;
        }

        cv::Mat img_undist;
        cv::remap( img_in, img_undist, mapx, mapy, cv::INTER_LANCZOS4 );

        scale_show( "img_in", img_in, size_show );
        scale_show( "img_undist", img_undist, size_show );

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
    }

    cap.release();
    
    return 0;
}
