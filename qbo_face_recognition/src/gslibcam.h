#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <gst/gst.h>

#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sstream>
#include <cv.hpp>

struct ImageData {
   cv::Mat image;
   int width, height;
   bool gstreamerPad, appPad;
};

class Camera {
public:
  std::string device_, cameraName_;
  void toMat(cv::Mat& dst, int t=100);
  Camera(std::string device, int w, int h, std::string cameraName);
  ~Camera();
private:
  //globals
  GstElement *pipeline_;
  struct ImageData imageData_;

  //forward declarations
  //gboolean processData(GstPad *pad, GstBuffer *buffer, gpointer u_data);
};


