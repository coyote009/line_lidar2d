#ifndef RASPIVIDEOCAP2_H_
#define RASPIVIDEOCAP2_H_

class RaspiVideoCapture2
{
public:
    int open(unsigned int width, unsigned int height,
             float exposure_time=200000.0, float analogue_gain=10.0);
    int read(cv::Mat &img);
    void release();
};

#endif
