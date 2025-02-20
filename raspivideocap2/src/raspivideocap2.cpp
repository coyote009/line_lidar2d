#include <iostream>
#include <iomanip>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include "mapped_framebuffer.h"

#include <thread>
#include <chrono>
using namespace std::chrono_literals;

#include "raspivideocap2.h"

static std::unique_ptr<libcamera::CameraManager> cm_;
static std::shared_ptr<libcamera::Camera> camera_;
static libcamera::FrameBufferAllocator *allocator_;
static libcamera::Stream *stream_;
static std::vector<std::unique_ptr<libcamera::Request> > requests_;

static std::mutex mutex_;
static std::queue<cv::Mat> img_buffer_;

void requestComplete(libcamera::Request *request)
{
    if(request->status() == libcamera::Request::RequestCancelled)
        return;

    const std::map<const libcamera::Stream *, libcamera::FrameBuffer *> &buffers
        = request->buffers();

    for(auto bufferPair: buffers)
    {
        const libcamera::Stream *stream = bufferPair.first;
        libcamera::FrameBuffer *buffer = bufferPair.second;
        //const libcamera::FrameMetadata &metadata = buffer->metadata();

        auto cfg = stream->configuration();

        libcamera::MappedFrameBuffer mappedBuffer(
            buffer, libcamera::MappedFrameBuffer::MapFlag::Read);
        const std::vector<libcamera::Span<uint8_t> > mem = mappedBuffer.planes();
        cv::Mat image(cfg.size.height, cfg.size.width, CV_8UC3,
                      (uint8_t *)(mem[0].data()));

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(img_buffer_.size() == 4)
                img_buffer_.pop();
            img_buffer_.push(image.clone());
        }
    }
    
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}

int RaspiVideoCapture2::read(cv::Mat &img)
{
    while(1)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(img_buffer_.size() > 0)
            {
                img = img_buffer_.front();
                img_buffer_.pop();
                return 1;
            }
        }

        std::this_thread::sleep_for(1ms);
    }
}

int RaspiVideoCapture2::open(unsigned int width, unsigned int height,
                             float exposure_time, float analogue_gain)
{
    if(cm_)
    {
        std::cout << "Can't open RaspiVideoCapture2 twice..." << std::endl;
        return 0;
    }
    
    cm_ = std::make_unique<libcamera::CameraManager>();
    cm_->start();

    auto cameras = cm_->cameras();
    if(cameras.empty())
    {
        std::cout << "No cameras found" << std::endl;
        cm_->stop();
        return 0;
    }

    std::string cameraId = cameras[0]->id();
    camera_ = cm_->get(cameraId);

    camera_->acquire();

    std::unique_ptr<libcamera::CameraConfiguration> config =
        camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    
    libcamera::StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = libcamera::formats::RGB888;
    streamConfig.size = {width, height};
    std::cout << "Default viewfinder configuration is: "
              << streamConfig.toString() << std::endl;
    
    config->validate();
    std::cout << "Validated viewfinder configuration is: "
              << streamConfig.toString() << std::endl;
    
    camera_->configure(config.get());
    
    allocator_
        = new libcamera::FrameBufferAllocator(camera_);
    
    for(libcamera::StreamConfiguration &cfg: *config)
    {
        int ret = allocator_->allocate(cfg.stream());
        if(ret < 0)
        {
            std::cerr << "Can't allocate buffers" << std::endl;
            return 0;
        }
    
        size_t allocated = allocator_->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream"
                  << std::endl;
    }
    
    stream_ = streamConfig.stream();
    const std::vector<std::unique_ptr<libcamera::FrameBuffer> > &buffers
        = allocator_->buffers(stream_);
    
    for(unsigned int i = 0; i < buffers.size(); ++i)
    {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if(!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return 0;
        }
    
        const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream_, buffer.get());
        if(ret < 0)
        {
            std::cerr << "Can't set buffer for request" << std::endl;
            return 0;
        }
    
        requests_.push_back(std::move(request));
    }

    camera_->requestCompleted.connect(requestComplete);

    libcamera::ControlList controls(libcamera::controls::controls);
    controls.set(libcamera::controls::ExposureTime, exposure_time); //[usec]
    controls.set(libcamera::controls::AnalogueGain, analogue_gain);

    camera_->start(&controls);
    for(std::unique_ptr<libcamera::Request> &request: requests_)
        camera_->queueRequest(request.get());

    return 1;
}

void RaspiVideoCapture2::release()
{
    camera_->stop();
    allocator_->free(stream_);
    delete allocator_;
    camera_->release();
    camera_.reset();
    cm_->stop();
}
