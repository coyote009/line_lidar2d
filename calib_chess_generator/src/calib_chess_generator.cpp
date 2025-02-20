#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#ifdef _WIN32

#define OCV_VER_STR CVAUX_STR( CV_VERSION_MAJOR ) CVAUX_STR( CV_VERSION_MINOR ) CVAUX_STR( CV_VERSION_REVISION )

#ifdef _DEBUG
#define OCV_LIB_EXT "d.lib"
#else
#define OCV_LIB_EXT ".lib"
#endif

#pragma comment( lib, "opencv_core" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgproc" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_aruco" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_highgui" OCV_VER_STR OCV_LIB_EXT )
#pragma comment( lib, "opencv_imgcodecs" OCV_VER_STR OCV_LIB_EXT )

#endif

struct chart_param
{
    cv::Size num_corners;
    int square_size;

    chart_param() :
        //num_corners( 6, 5 ),
        num_corners( 4, 3 ),
        square_size( 100 )
    {}
};

static int read_chart_param( const cv::String &fname_param, chart_param &param )
{
    cv::FileStorage fs;
    if( !fs.open( fname_param, cv::FileStorage::READ ) )
    {
        return 1;
    }

    fs["NUM_CORNERS"] >> param.num_corners;
    fs["SQUARE_SIZE"] >> param.square_size;

    return 0;
}

int main( int argc, char **argv )
{
    const cv::String keys =
        "{help | | Show this message}"
        "{@param_file | | Parameter file}"
        "{out | | Output file}"
        "{show | | Just show the chart (no generation)}"
        ;
    cv::CommandLineParser parser( argc, argv, keys );
    parser.about( argv[0] );

    if( parser.has( "help" ) )
    {
        parser.printMessage();
        return 1;
    }

    chart_param param;
    if( parser.has( "@param_file" ) )
    {
        cv::String fname_param = parser.get<cv::String>( "@param_file" );
        if( read_chart_param( fname_param, param ) )
        {
            fprintf( stderr, "Failed to open calibration file\n" );
            return 1;
        }
    }

    cv::String fname_out = "chart.png";
    if( parser.has( "out" ) )
    {
        fname_out = parser.get<cv::String>( "out" );
    }

    bool show_chart = false;
    if( parser.has( "show" ) )
    {
        show_chart = parser.get<bool>( "show" );
    }

    cv::Size size_img = (param.num_corners + cv::Size(1, 1))
        * param.square_size;

    cv::Mat img_board(size_img, CV_8UC3, cv::Scalar(255, 255, 255));

    for( int j=0; j < param.num_corners.height + 1; j++ )
    {
        for( int i=0; i < param.num_corners.width + 1; i++ )
        {
            if((i % 2) == (j % 2))
            {
                cv::Point pt_tl(i*param.square_size, j*param.square_size);
                cv::Point pt_br((i+1)*param.square_size-1, (j+1)*param.square_size-1); // rectangle does include br corner
                cv::rectangle(img_board, pt_tl, pt_br, cv::Scalar(0, 0, 0),
                              cv::FILLED);
            }
        }
    }

    //std::cout << cv::sum(img_board) / 255;

    if( show_chart )
    {
        std::vector<cv::Point2f> points;
        bool found = cv::findChessboardCorners(img_board, param.num_corners,
                                               points);
        cv::drawChessboardCorners(img_board, param.num_corners, cv::Mat(points),
                                  found);

        //if(found)
        //    std::cout << "Found" << std::endl;
        
        cv::imshow( "img_board", img_board );
        cv::waitKey();
    }
    else
    {
        cv::imwrite( fname_out, img_board );
    }

    return 0;
}
