#include <opencv2/opencv.hpp>
#include "raspivideocap2.h"

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

struct board_parameter
{
    std::vector<cv::Size> num_square_list;
    float square_size;
    int num_boards;

    board_parameter() :
        square_size(0.03),
        num_boards(2)
    {
        num_square_list.resize( num_boards, cv::Size( 4, 3 ) );
    }
};

struct board_corners
{
    std::vector< std::vector<cv::Point2f> > corner2d_list;
    std::vector< std::vector<cv::Point3f> > corner3d_list;

    std::vector<cv::Mat> mats_r;
    std::vector<cv::Mat> vecs_r;
    std::vector<cv::Mat> vecs_t;

    std::vector<cv::Mat> board_coef;

    board_corners(board_parameter &board_param ) :
        corner2d_list( board_param.num_boards ),
        corner3d_list( board_param.num_boards ),
        mats_r( board_param.num_boards ),
        vecs_r( board_param.num_boards ),
        vecs_t( board_param.num_boards ),
        board_coef( board_param.num_boards )
    {
        int num_boards = board_param.num_boards;
        float square_size = board_param.square_size;
        for(int k=0; k<num_boards; k++)
        {
            for(int j=0; j<board_param.num_square_list[k].height; j++)
            {
                for(int i=0; i<board_param.num_square_list[k].width; i++)
                {
                    cv::Point3f pt3d(i*square_size, j*square_size, 0.0);
                    corner3d_list[k].push_back(pt3d);
                }
            }
        }
    }
};

static void find_corners2(const board_parameter &board_param, const calib_data &calib,
                          const cv::Mat &img_accm,
                          const std::vector<cv::Rect> &chart_rois,
                          cv::Mat &img_accm_show,
                          board_corners &corner_list)
{
    for(int i=0; i<chart_rois.size(); i++)
    {
        cv::Scalar mean = cv::mean(img_accm);
        cv::Mat masked_chart( img_accm.size(), CV_8U, mean );
        img_accm( chart_rois[i] ).copyTo( masked_chart( chart_rois[i] ) );
        //char buf[16];
        //sprintf(buf, "masked chart %d", i);
        //cv::imshow(buf, masked_chart);
        //cv::waitKey();

        std::vector<cv::Point2f> points;
        bool found = cv::findChessboardCorners(
            masked_chart, board_param.num_square_list[i], points,
            cv::CALIB_CB_ADAPTIVE_THRESH);// +
        //cv::CALIB_CB_NORMALIZE_IMAGE);// + cv::CALIB_CB_FAST_CHECK);
        if(found)
        {
            cv::cornerSubPix(masked_chart, points, cv::Size(11, 11),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS +
                                              cv::TermCriteria::MAX_ITER,
                                              30, 0.1));

            cv::drawChessboardCorners(img_accm_show,
                                      board_param.num_square_list[i],
                                      cv::Mat(points), found);

            corner_list.corner2d_list[i] = points;

            std::cout << "corner2d=" << corner_list.corner2d_list[i] << std::endl;
            std::cout << "corner3d=" << corner_list.corner3d_list[i] << std::endl;

            cv::solvePnP( corner_list.corner3d_list[i], corner_list.corner2d_list[i],
                          calib.mat_cam, cv::Mat(),
                          corner_list.vecs_r[i], corner_list.vecs_t[i] );
            cv::Rodrigues( corner_list.vecs_r[i], corner_list.mats_r[i] );

            std::cout << "vec_r=" << corner_list.vecs_r[i] << std::endl;
            std::cout << "vec_t=" << corner_list.vecs_t[i] << std::endl;

            corner_list.board_coef[i] = corner_list.mats_r[i].col( 2 );
            cv::Mat dist = corner_list.vecs_t[i].t() * corner_list.board_coef[i];
            corner_list.board_coef[i] /= dist.at<double>( 0 );

            std::cout << "board_coef=" << corner_list.board_coef[i] << std::endl;
        }
    }
}

static void find_laser( const cv::Mat &mat_cam, const std::vector<cv::Rect> &laser_rois,
                        const std::vector<cv::Mat> &board_coef,
                        const cv::Mat &img_in, cv::Mat &laser_coef )
{
    cv::Mat mask_laser = ( img_in > 160 );
    cv::Mat points3d;

#define SHOW_LASER
#ifdef SHOW_LASER
    cv::Mat img_laser = cv::Mat::zeros( img_in.size(), CV_8UC1 );
#endif

    //std::cout << "mat_cam" << mat_cam << std::endl;

    for( int i = 0; i < laser_rois.size(); i++ )
    {
        cv::Mat curr_mask = cv::Mat::zeros( img_in.size(), CV_8U );
        mask_laser( laser_rois[i] ).copyTo( curr_mask( laser_rois[i] ) );

#ifdef SHOW_LASER
        mask_laser( laser_rois[i] ).copyTo( img_laser( laser_rois[i] ) );
#endif

        //cv::imshow( "mask", curr_mask );
        //cv::waitKey();

        //std::cout << "board_coef" << board_coef[i] << std::endl;

        for( int y = 0; y < curr_mask.rows; y++ )
        {
            for( int x = 0; x < curr_mask.cols; x++ )
            {
                if( !curr_mask.at<uchar>( y, x ) )
                {
                    continue;
                }

                cv::Mat vec_u = ( cv::Mat_<double>( 3, 1 ) << x, y, 1.0 );
                cv::Mat mat_cam_inv = mat_cam.inv();
                cv::Mat cam_u = mat_cam_inv * vec_u;
                cv::Mat b_cam_u = board_coef[i].t() * cam_u;
                cv::Mat pt3d = cam_u / b_cam_u.at<double>( 0 );

                points3d.push_back( pt3d.t() );

                //std::cout << "vec_u" << vec_u.t() << std::endl;
                //std::cout << "pt3d" << pt3d.t() << std::endl;
            }
        }
    }

    //std::cout << points3d << std::endl;

    cv::Mat ones = cv::Mat::ones( points3d.rows, 1, CV_64FC1 );
    cv::solve( points3d, ones, laser_coef, cv::DECOMP_SVD );

    std::cout << "laser_coef = " << laser_coef << std::endl;

#ifdef SHOW_LASER
    cv::imshow( "img_laser", img_laser );
#endif
}

static void find_laser_centers( const cv::Mat &img_in, const cv::Mat &mask_laser,
                                std::vector<cv::Point2f> &centers_2d, int min_len )
{
    centers_2d.clear();
    
    for( int y = 0; y < img_in.rows; y++ )
    {
        int counter = 0;
        float sum = 0.0;
        float weight_sum = 0.0;
        for( int x = 0; x < img_in.cols; x++ )
        {
            if( mask_laser.at<uchar>( y, x ) )
            {
                counter++;
                sum += x * img_in.at<uchar>( y, x );
                weight_sum += img_in.at<uchar>( y, x );
            }
            else
            {
                if( counter >= min_len && weight_sum != 0.0 )
                {
                    centers_2d.push_back( cv::Point2f( sum/weight_sum, y ) );
                }
                
                counter = 0;
                sum = 0.0;
                weight_sum = 0.0;
            }
        }
    }
}

static void find_laser2( const cv::Mat &mat_cam, const std::vector<cv::Rect> &laser_rois,
                         const std::vector<cv::Mat> &board_coef,
                         const cv::Mat &img_in, cv::Mat &laser_coef )
{
    cv::Mat img_proc = img_in.clone();
    cv::blur(img_proc, img_proc, cv::Size(5, 5));

    cv::Mat mask_laser;
    cv::adaptiveThreshold( img_proc, mask_laser, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                           cv::THRESH_BINARY, 15, -16 );

    cv::Mat points3d;

#define SHOW_LASER
#ifdef SHOW_LASER
    cv::Mat img_laser = cv::Mat::zeros( img_in.size(), CV_8UC1 );
#endif

    //std::cout << "mat_cam" << mat_cam << std::endl;

    for( int i = 0; i < laser_rois.size(); i++ )
    {
        cv::Mat curr_mask = cv::Mat::zeros( img_in.size(), CV_8U );
        mask_laser( laser_rois[i] ).copyTo( curr_mask( laser_rois[i] ) );

        std::vector<cv::Point2f> centers_2d;
        find_laser_centers( img_proc, curr_mask, centers_2d, 3 );

        for(int j = 0; j<centers_2d.size(); j++ )
        {
            cv::Mat vec_u = ( cv::Mat_<double>( 3, 1 ) <<
                              centers_2d[j].x, centers_2d[j].y, 1.0 );
            cv::Mat mat_cam_inv = mat_cam.inv();
            cv::Mat cam_u = mat_cam_inv * vec_u;
            cv::Mat b_cam_u = board_coef[i].t() * cam_u;
            cv::Mat pt3d = cam_u / b_cam_u.at<double>( 0 );

            points3d.push_back( pt3d.t() );

#ifdef SHOW_LASER
            img_laser.at<uchar>(centers_2d[j].y, centers_2d[j].x) = 255;
#endif
        }
    }

    //std::cout << points3d << std::endl;

    cv::Mat ones = cv::Mat::ones( points3d.rows, 1, CV_64FC1 );
    cv::solve( points3d, ones, laser_coef, cv::DECOMP_SVD );

    std::cout << "laser_coef = " << laser_coef << std::endl;

#ifdef SHOW_LASER
    cv::imshow( "img_laser", img_laser );
#endif
}

int main()
{
    const cv::Size size_proc( 640, 480 );
    //const cv::Size size_proc( 1280, 960 );
    //const cv::Size size_proc( 1920, 1440 );
    const std::string fname_calib = "../../calib_mono_raspicam2/work/calib.yml";
    const std::string fname_calib_laser = "../work/calib_laser.yml";

    calib_data calib;
    if( read_calib_data( fname_calib, size_proc, calib ) )
    {
        fprintf( stderr, "Failed to read calibration data\n" );
        return 1;
    }

    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap( calib.mat_cam_org, calib.vec_dist, cv::Mat(), calib.mat_cam,
                                 size_proc, CV_32FC1, mapx, mapy );

    //RaspiVideoCapture cap;
    RaspiVideoCapture2 cap;
    //if( !cap.open( size_proc.width, size_proc.height, 30, 0, 1, 1,
    //               RPC_PARAM_EXPOSUREMODE_OFF, 100, 10000 ) )
    if( !cap.open( size_proc.width, size_proc.height, 100000.0, 5.0 ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    board_parameter board_param;
    board_corners corners(board_param);

    std::vector<cv::Rect> laser_rois;

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

        std::vector<cv::Mat> imgs_bgr;
        cv::split(img_undist, imgs_bgr);
        cv::Mat img_gray = imgs_bgr[0];
        //cv::cvtColor( img_undist, img_gray, cv::COLOR_BGR2GRAY );

        // Show info image
        cv::Mat img_info = img_undist.clone();
        char message[] =
            "'a':toggle_accm 'r':sel_chart 'f':find_chart "
            "'s':sel_laser 'l':find_laser";
        cv::putText( img_info, message, cv::Point( 15, 15 ),
                     cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0) );
        for( int i = 0; i < laser_rois.size(); i++ )
        {
            cv::rectangle( img_info, laser_rois[i], CV_RGB(0,255,0) );
        }
        cv::imshow("img_info", img_info);

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

        int key = cv::waitKey(1);
        if(key == 27)
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
                cv::imwrite("../work/img_accm.jpg", img_accm_show);
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
        // Find chart
        else if(key == 'f')
        {
            if( chart_rois.size() != board_param.num_boards )
            {
                fprintf( stderr, "Chart ROI not selected" );
                continue;
            }

            find_corners2(board_param, calib, img_accm, chart_rois,
                          img_accm_show, corners);

            cv::imwrite("../work/img_chart.jpg", img_accm_show);
        }
        // Select laser ROI
        else if(key == 's')
        {
            laser_rois.clear();
            cv::selectROIs( "select ROI", img_gray, laser_rois, false );
            cv::destroyWindow( "select ROI" );
            fprintf( stdout, "%d ROIs selected\n", laser_rois.size() );
        }
        else if( key == 'l' )
        {
            if( laser_rois.size() != board_param.num_boards )
            {
                fprintf( stderr, "Laser ROI not selected or board corners not found" );
                continue;
            }
            
            cv::Mat laser_coef;
            find_laser2( calib.mat_cam, laser_rois, corners.board_coef, img_gray, laser_coef );
        
            cv::FileStorage fs;
            if( !fs.open( fname_calib_laser, cv::FileStorage::WRITE ) )
            {
                fprintf( stderr, "Failed to open laser calibration file.\n" );
                return 1;
            }
        
            fs << "LASER_COEF" << laser_coef;
        }
    }

    cap.release();

    return 0;
}
