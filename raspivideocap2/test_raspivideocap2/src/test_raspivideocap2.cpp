#include <iostream>
#include <opencv2/opencv.hpp>
#include "raspivideocap2.h"

int main()
{
    RaspiVideoCapture2 cap;
    if(!cap.open(640, 480))
    {
        std::cerr << "Error: Exiting ..." << std::endl;
        return 1;
    }

    cv::Mat img;
    while(1)
    {
        cap.read(img);

        cv::imshow("img", img);
        if(cv::waitKey(1) == 27)
            break;
    }

    cap.release();

    return 0;
}
