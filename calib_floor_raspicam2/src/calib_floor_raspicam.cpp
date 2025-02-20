#include <opencv2/opencv.hpp>
#include "raspivideocap2.h"

struct calib_data
{
    cv::Mat mat_cam_org;
    cv::Mat mat_cam;
    cv::Mat vec_dist;
};

static int read_calib_data( const std::string &fname_calib, const cv::Size &size_proc,
                            calib_data &calib )
{
    cv::FileStorage fs;
    if( !fs.open( fname_calib, cv::FileStorage::READ ) )
    {
        return 1;
    }

    fs["MAT_CAM"] >> calib.mat_cam_org;
    fs["VEC_DIST"] >> calib.vec_dist;

    cv::Size size_calib;
    fs["SIZE_IN"] >> size_calib;

    if( size_proc != size_calib )
    {
        double ratio = ((double) size_proc.width) / ((double) size_calib.width);

        calib.mat_cam_org.at<double>( 0, 0 ) *= ratio;
        calib.mat_cam_org.at<double>( 0, 2 ) *= ratio;
        calib.mat_cam_org.at<double>( 1, 1 ) *= ratio;
        calib.mat_cam_org.at<double>( 1, 2 ) *= ratio;
    }

    calib.mat_cam = cv::getOptimalNewCameraMatrix( calib.mat_cam_org, calib.vec_dist,
                                                   size_proc, 1.0 );

    return 0;
}

static bool find_board_coef( const cv::Size &size_pat, float pat_len,
                             const calib_data &calib, const std::vector<cv::Point2f> &corners2d,
                             cv::Mat &vec_R, cv::Mat &mat_R, cv::Mat &vec_t, cv::Mat &board_coef )
{
    std::vector<cv::Point3f> corners3d;
    for( int y = 0; y < size_pat.height; y++ )
    {
        for( int x = 0; x < size_pat.width; x++ )
        {
            corners3d.push_back( cv::Point3f( x * pat_len, y * pat_len, 0.0 ) );
        }
    }

    //std::cout << corners2d << std::endl;
    //std::cout << corners3d << std::endl;

    if( !cv::solvePnP( corners3d, corners2d, calib.mat_cam, cv::Mat(), vec_R, vec_t ) )
    {
        return false;
    }

    cv::Rodrigues( vec_R, mat_R );

    board_coef = mat_R.col( 2 );
    cv::Mat dist = vec_t.t() * board_coef;
    board_coef /= dist.at<double>( 0 );

    return true;
}

int main()
{
    const cv::Size size_proc( 640, 480 );
    const std::string fname_calib = "../../calib_mono_raspicam2/work/calib.yml";
    const std::string fname_calib_floor = "../work/calib_floor.yml";

    const cv::Size size_pat( 4, 3 );
    float pat_len = 0.03;

    calib_data calib;
    if( read_calib_data( fname_calib, size_proc, calib ) )
    {
        fprintf( stderr, "Failed to read calibration data\n" );
        return 1;
    }

    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap( calib.mat_cam_org, calib.vec_dist, cv::Mat(), calib.mat_cam,
                                 size_proc, CV_32FC1, mapx, mapy );

    RaspiVideoCapture2 cap;
    if( !cap.open( size_proc.width, size_proc.height ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    bool accm_mode = false;
    cv::Mat img_accm = cv::Mat::zeros(size_proc, CV_8UC1);
    cv::Mat img_accm_show = cv::Mat::zeros(size_proc, CV_8UC3);
    std::vector<cv::Rect> chart_rois;

    while( 1 )
    {
        cv::Mat img_in;
        cap.read( img_in );

        cv::Mat img_undist;
        cv::remap( img_in, img_undist, mapx, mapy, cv::INTER_LANCZOS4 );
        cv::imshow( "img_undist", img_undist );

        std::vector<cv::Mat> imgs_bgr;
        cv::split(img_undist, imgs_bgr);
        cv::Mat img_gray = imgs_bgr[0];

        // Accumulate image
        if(accm_mode)
        {
            cv::max(img_gray, img_accm, img_accm);
        }
        cv::imshow("img_accm", img_accm);

        for( int i = 0; i < chart_rois.size(); i++ )
        {
            cv::rectangle( img_accm_show, chart_rois[i], CV_RGB(0,255,0) );
        }
        cv::imshow("img_accm_show", img_accm_show);

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
        // Accumulate image
        else if(key == 'a')
        {
            if(!accm_mode)
            {
                img_accm = img_gray.clone();
                accm_mode = true;
            }
            else
            {
                cv::cvtColor(img_accm, img_accm_show, cv::COLOR_GRAY2BGR);
                accm_mode = false;
            }
        }
        // Select chart ROI
        else if(key == 'r')
        {
            chart_rois.clear();
            cv::selectROIs( "select ROI", img_accm, chart_rois, false );
            cv::destroyWindow( "select ROI" );
            fprintf( stdout, "%d ROIs selected\n", chart_rois.size() );
            cv::cvtColor(img_accm, img_accm_show, cv::COLOR_GRAY2BGR);
        }
        else if( key == 'f' )
        {
            if( chart_rois.size() == 0 )
            {
                fprintf( stderr, "Chart ROI not selected" );
                continue;
            }

            cv::Scalar mean = cv::mean(img_accm);
            cv::Mat masked_chart( img_accm.size(), CV_8U, mean );
            img_accm( chart_rois[0] ).copyTo( masked_chart( chart_rois[0] ) );

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners( masked_chart, size_pat, corners,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH );
            if( found )
            {
                cv::cornerSubPix( masked_chart, corners, cv::Size( 11, 11 ),
                                  cv::Size(-1,-1),
                                  cv::TermCriteria( cv::TermCriteria::EPS +
                                                    cv::TermCriteria::MAX_ITER,
                                                    30, 0.1 ) );

                cv::drawChessboardCorners(img_accm_show, size_pat, cv::Mat(corners),
                                          found);

                cv::imshow( "img_accm_show", img_accm_show );
                cv::putText( img_accm_show, "Hit space to save board coef",
                             cv::Point( 15, 15 ), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                             CV_RGB(0,255,0), 2 );
                if( cv::waitKey() == ' ' )
                {
                    cv::Mat vec_R, mat_R, vec_t, floor_coef;
                    find_board_coef( size_pat, pat_len, calib, corners,
                                     vec_R, mat_R, vec_t, floor_coef );

                    cv::FileStorage fs;
                    if( !fs.open( fname_calib_floor, cv::FileStorage::WRITE ) )
                    {
                        fprintf( stderr, "Failed to open calib file\n" );
                    }
                    else
                    {
                        fs << "VEC_R" << vec_R;
                        fs << "MAT_R" << mat_R;
                        fs << "VEC_T" << vec_t;
                        fs << "FLOOR_COEF" << floor_coef;
                    }

                    std::cout << "VEC_R" << vec_R << std::endl;
                    std::cout << "MAT_R" << mat_R << std::endl;
                    std::cout << "VEC_T" << vec_t << std::endl;
                    std::cout << "FLOOR_COEF" << floor_coef << std::endl;
                }
            }
        }
    }

    cap.release();

    return 0;
}
