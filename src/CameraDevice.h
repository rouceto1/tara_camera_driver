#include <string>
#include <opencv2/core/core.hpp>

#define FRAME_WIDTH 752
#define FRAME_HEIGHT 480

namespace tara
{

class StereoCameraDriver
{
  public:

    StereoCameraDriver(const std::string& device);

    ~StereoCameraDriver();

    void grabNextFrame(cv::Mat& img_left, cv::Mat& img_right);
    
    int getExposure();

    bool setExposure(int ExposureVal);

    bool setBrightness(int BrightnessVal);

    int set_control(uint32_t id, int val);

    void printCapabilities();

  private:

    // file descriptor for camera device
    int fd_;

    // image data buffer
    uint8_t* buffer_;

  // helper functions

    void init_mmap();
};

};
