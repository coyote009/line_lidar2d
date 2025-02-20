#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "raspivideocap2.h"
//#include "undistort_points.h"

struct intrinsic_param
{
    cv::Mat mat_cam;
    cv::Mat vec_dist;
    //cv::Mat xi;
    cv::Size size;
    //int frame_rate;
};

static int read_intrinsics( const std::string &fname,
                            intrinsic_param &intrinsics )
{
    cv::FileStorage fs;
    if( !fs.open( fname, cv::FileStorage::READ ) )
    {
        fprintf( stderr, "Failed to open calib file\n" );
        return 1;
    }
    
    fs["MAT_CAM"] >> intrinsics.mat_cam;
    fs["VEC_DIST"] >> intrinsics.vec_dist;
    //fs["XI"] >> intrinsics.xi;
    fs["SIZE_IN"] >> intrinsics.size;
    //fs["FRAME_RATE"] >> intrinsics.frame_rate;

    return 0;
}

static int read_laser_coef( const std::string &fname,
                            cv::Mat &laser_coef )
{
    cv::FileStorage fs;
    if( !fs.open( fname, cv::FileStorage::READ ) )
    {
        fprintf( stderr, "Failed to open laser calib file\n" );
        return 1;
    }
    
    fs["LASER_COEF"] >> laser_coef;

    return 0;
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
                //sum += x * mask_laser.at<uchar>(y, x);
                //weight_sum += mask_laser.at<uchar>(y, x);
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

static void find_laser_centers2( const cv::Mat &img_in, const cv::Mat &mask_laser,
                                 std::vector<cv::Point2f> &centers_2d, int min_len )
{
    centers_2d.clear();
    
    for( int y = 0; y < img_in.rows; y++ )
    {
        int counter = 0;
        int sum = 0;
        int weight_sum = 0;

        uchar max_lum = 0;
        float max_x;
        uchar curr_max = 0;
        for( int x = 0; x < img_in.cols; x++ )
        {
            if( mask_laser.at<uchar>( y, x ) )
            {
                uchar curr_lum = img_in.at<uchar>( y, x );

                counter++;
                sum += x * curr_lum;
                weight_sum += curr_lum;

                if( curr_lum > curr_max )
                    curr_max = curr_lum;
            }
            else
            {
                if( counter >= min_len && weight_sum != 0.0 )
                {
                    if( curr_max > max_lum )
                    {
                        max_lum = curr_max;
                        max_x = ((float)sum) / ((float)weight_sum);
                    }
                }
                
                counter = 0;
                sum = 0.0;
                weight_sum = 0.0;
                curr_max = 0;
            }
        }

        if( max_lum > 0 )
            centers_2d.push_back( cv::Point2f( max_x, y ) );
    }
}

static void compute_laser_3d( const std::vector<cv::Point2f> &pts2d,
                              std::vector<cv::Point3f> &pts3d,
                              const intrinsic_param &intrinsics,
                              const cv::Mat &laser_coef )
{
    std::vector<cv::Point2f> pts_undist;
    if( pts2d.size() )
    {
        cv::undistortPoints( pts2d, pts_undist, intrinsics.mat_cam, intrinsics.vec_dist );
    }

    //std::vector<cv::Point3f> corners_sphere;
    //undistort_points( pts2d, corners_sphere,
    //                  intrinsics.mat_cam, intrinsics.vec_dist, intrinsics.xi,
    //                  cv::Mat::eye( 3, 3, CV_64FC1 ) );

    pts3d.clear();

    for( int i = 0; i < pts_undist.size(); i++ )
    {
        cv::Mat vec_u = ( cv::Mat_<float>( 3, 1 ) << pts_undist[i].x, pts_undist[i].y, 1.0 );
        cv::Mat b_u = laser_coef.t() * vec_u;
        cv::Mat pt3d = vec_u / b_u.at<float>( 0 );

        pts3d.push_back( cv::Point3f( pt3d ) );
    }
}

struct control_param
{
    int preblur_smax;
    int preblur_slider;
    int preblur;

    int thresh_smax;
    int thresh_slider;
    int thresh;

    int erode_smax;
    int erode_slider;
    int erode;

    int dilate_smax;
    int dilate_slider;
    int dilate;

    int postblur_smax;
    int postblur_slider;
    int postblur;

    int minwd_smax;
    int minwd_slider;
    int minwd;

    int detmode_smax;
    int detmode_slider;
    int detmode;

    control_param() :
        preblur_smax( 5 ), preblur_slider( 2 ), preblur( 5 ),
        thresh_smax( 255 ), thresh_slider( 64 ), thresh( 64 ),
        erode_smax( 5 ), erode_slider( 1 ), erode( 3 ),
        dilate_smax( 5 ), dilate_slider( 2 ), dilate( 5 ),
        postblur_smax( 5 ), postblur_slider( 2 ), postblur( 5 ),
        minwd_smax( 15 ), minwd_slider( 2 ), minwd( 3 ),
        detmode_smax( 1 ), detmode_slider( 0 ), detmode( 0 )
    {}

    static void on_track_bar( int, void *ptr )
    {
        control_param *pparam = (control_param *) ptr;
        pparam->update();
    }

    void create_trackbar( const char *win_name )
    {
        cv::createTrackbar( "Pre-blur filter", win_name, &preblur_slider, preblur_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Threshold", win_name, &thresh_slider, thresh_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Erode filter", win_name, &erode_slider, erode_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Dilate filter", win_name, &dilate_slider, dilate_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Post-blur filter", win_name, &postblur_slider, postblur_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Minimum width", win_name, &minwd_slider, minwd_smax,
                            on_track_bar, (void *) this );
        cv::createTrackbar( "Detection mode", win_name, &detmode_slider, detmode_smax,
                            on_track_bar, (void *) this );
    }

    void update()
    {
        preblur = preblur_slider * 2 + 1;
        thresh = thresh_slider;
        erode = erode_slider * 2 + 1;
        dilate = dilate_slider * 2 + 1;
        postblur = postblur_slider * 2 + 1;
        minwd = minwd_slider + 1;
        detmode = detmode_slider;
    }
};

int main()
{
    const std::string fname_intrinsic( "../../calib_mono_raspicam2/work/calib.yml" );
    const std::string fname_laser_calib( "../../calib_line_chess_raspicam2/work/calib_laser.yml" );

    cv::namedWindow( "map" );

    control_param param;
    param.create_trackbar( "map" );

    intrinsic_param intrinsics;
    if( read_intrinsics( fname_intrinsic, intrinsics ) )
    {
        return 1;
    }

    cv::Mat laser_coef;
    if( read_laser_coef( fname_laser_calib, laser_coef ) )
    {
        return 1;
    }
    laser_coef.convertTo( laser_coef, CV_32F );

    RaspiVideoCapture2 cap;
    if( !cap.open( intrinsics.size.width, intrinsics.size.height, 200000.0, 3.0 ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    //float pix_per_m = 1000;
    float pix_per_m = 500;
    cv::Mat map_base = cv::Mat::zeros( 500, 500, CV_8UC1 );
    cv::line(map_base,
             cv::Point(map_base.cols/2, 0),
             cv::Point(map_base.cols/2, map_base.rows-1),
             cv::Scalar(128, 128, 128));

    float dist = 0;
    while(1)
    {
        float yval = map_base.rows - 1 - dist*pix_per_m;
        if(yval < 0)
            break;
        
        cv::line(map_base, cv::Point(0, yval), cv::Point(map_base.cols-1, yval),
                 cv::Scalar(128, 128, 128));

        char buf[32];
        sprintf( buf, "%.1fm", dist );
        cv::putText( map_base, buf, cv::Point( 0, yval ),
                     cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(255) );

        dist += 0.1;
    }

    while( 1 )
    {
        cv::Mat img_in;
        cap.read( img_in );

        std::vector<cv::Mat> imgs_bgr;
        cv::split(img_in, imgs_bgr);
        img_in = imgs_bgr[0];

        if( param.preblur > 1 )
            cv::blur( img_in, img_in, cv::Size(param.preblur, param.preblur) );

        cv::Mat mask_laser;
        cv::threshold( img_in, mask_laser, param.thresh, 255,
                       cv::THRESH_BINARY ); //cv::THRESH_TOZERO );

        if( param.erode > 1 )
        {
            cv::Mat morph_kernel1 =
                cv::getStructuringElement( cv::MORPH_RECT,
                                           cv::Size(param.erode, param.erode) );
            cv::erode( mask_laser, mask_laser, morph_kernel1 );
        }
        if( param.dilate > 1 )
        {
            cv::Mat morph_kernel2 =
                cv::getStructuringElement( cv::MORPH_RECT,
                                           cv::Size(param.dilate, param.dilate) );
            cv::dilate( mask_laser, mask_laser, morph_kernel2 );
        }

        if( param.postblur > 1 )
            cv::blur(mask_laser, mask_laser, cv::Size(param.postblur,
                                                      param.postblur));

        std::vector<cv::Point2f> centers_2d;
        if( param.detmode == 0 )
            find_laser_centers( img_in, mask_laser, centers_2d, param.minwd );
        else
            find_laser_centers2( img_in, mask_laser, centers_2d, param.minwd );
        
        // Debug
        cv::Mat img_center = cv::Mat::zeros( img_in.size(), CV_8UC1 );
        for( int i = 0; i < centers_2d.size(); i++ )
        {
            img_center.at<uchar>( centers_2d[i] ) = 255;
        }
        cv::imshow( "img_center", img_center );

        std::vector<cv::Point3f> pts3d;
        compute_laser_3d( centers_2d, pts3d, intrinsics, laser_coef );

        cv::Mat map;
        map_base.copyTo(map);
        for( int i = 0; i < pts3d.size(); i++ )
        {
            cv::Point2f pt( pts3d[i].y, pts3d[i].z );
            pt.x = map.cols/2 + pt.x * pix_per_m;
            pt.y = map.rows - pt.y * pix_per_m;

            cv::Rect region( 0, 0, map.cols, map.rows );
            if( region.contains( pt ) )
            {
                map.at<uchar>( pt ) = 255;
            }
        }

        cv::imshow("img_in", img_in);
        cv::imshow("mask_laser", mask_laser);
        cv::imshow( "map", map );

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            break;
        }
        // Debug
        else if( key == 's' )
        {
            std::cout << "centers_2d =" << centers_2d << std::endl;
            std::cout << "pts_3d =" << pts3d << std::endl;
        }
        else if( key == 'i' )
        {
            cv::imwrite("../work/img_in.jpg", img_in);
            cv::imwrite("../work/img_mask.jpg", mask_laser);
            cv::imwrite("../work/img_map.jpg", map);
            cv::imwrite("../work/img_center.jpg", img_center);

            FILE *fp = fopen("../work/data_pc.csv", "w");
            fprintf(fp, "x,y,z\n");
            for(int i=0; i<pts3d.size(); i++)
                fprintf(fp, "%f,%f,%f\n", pts3d[i].x, pts3d[i].y, pts3d[i].z);
            fclose(fp);
        }
    }

    cap.release();

    return 0;
}
