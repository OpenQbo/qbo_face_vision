#include "gslibcam.h"
#include <highgui.h>

gboolean static processData(GstPad *pad, GstBuffer *gBuffer, gpointer u_data);

Camera::Camera(std::string device, int w, int h, std::string cameraName) : device_(device), cameraName_(cameraName)
{
        imageData_.width=w;
        imageData_.height=h;
        gst_init(0,0);
        std::cout << "Gstreamer Version: " << gst_version_string() << std::endl;

        GError *error = 0; //assignment to zero is a gst requirement
        std::stringstream config;
        config << "v4l2src device=" << device_ << " ! video/x-raw-rgb,width=" << imageData_.width << ",height=" << imageData_.height << " ! queue max-size-buffers=1 max-size-time=0 ! ffmpegcolorspace ! identity name=" << cameraName_ << " ! fakesink";
        std::cout << config.str().c_str() << std::endl;
        pipeline_ = gst_parse_launch(config.str().c_str(),&error);
        if (pipeline_ == NULL) {
                std::cout << error->message << std::endl;
                exit(-1);
        }


        GstElement *probe = gst_bin_get_by_name(GST_BIN(pipeline_), cameraName_.c_str());
        if (probe == NULL) {
                std::cout << "Your Gstreamer pipeline needs an identity element." << std::endl;
                exit(-1);
        }

        imageData_.appPad=false;
        imageData_.gstreamerPad = true;
        GstPad *pad = gst_element_get_pad(probe, "src"); 
        gst_pad_add_buffer_probe(pad, G_CALLBACK(processData), &this->imageData_);

        gst_element_set_state(pipeline_, GST_STATE_PAUSED);

        gst_object_unref(pad);

        if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
                std::cout << "Failed to PAUSE." << std::endl;
                exit(-1);
        } else {
                std::cout << "stream is PAUSED." << std::endl;
        }


        /*if (preroll)*/ {
                //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
                //I am told this is needed and am erring on the side of caution.
                gst_element_set_state(pipeline_, GST_STATE_PLAYING);

                if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
                        std::cout << "Failed to PLAY." << std::endl;
                        exit(-1);
                } else {
                        std::cout << "stream is PLAYING." << std::endl;
                }

                gst_element_set_state(pipeline_, GST_STATE_PAUSED);

                if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
                        std::cout << "Failed to PAUSE." << std::endl;
                        exit(-1);
                } else {
                        std::cout << "stream is PAUSED." << std::endl;
                }
        }


        //processVideo
        imageData_.gstreamerPad = true;
        imageData_.appPad=false;
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        std::cout << "stream is PLAYING." << std::endl;
}

Camera::~Camera()
{
  std::cout << "\nquitting..." << std::endl;
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);
}

void Camera::toMat(cv::Mat& dst, int t)
{
  while(!imageData_.appPad)
    usleep(t);

  imageData_.appPad = false;
  //copiamos la imagen que tenemos a la que nos piden
  imageData_.image.copyTo(dst);
  imageData_.gstreamerPad = true;
}

gboolean static processData(GstPad *pad, GstBuffer *gBuffer, gpointer u_data)
{
  if (!((struct ImageData*)u_data)->gstreamerPad) return TRUE;
  ((struct ImageData*)u_data)->gstreamerPad = false;

  if(gBuffer!=NULL&&pad!=NULL)
  {
    if (((struct ImageData*)u_data)->image.empty())
    {
      const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
      GstStructure *structure = gst_caps_get_structure(caps,0);
      gst_structure_get_int(structure,"width",&((struct ImageData*)u_data)->width);
      gst_structure_get_int(structure,"height",&((struct ImageData*)u_data)->height);
      ((struct ImageData*)u_data)->image.create(cv::Size2i(((struct ImageData*)u_data)->width,((struct ImageData*)u_data)->height),CV_8UC3);
    }

    memcpy(((struct ImageData*)u_data)->image.data, gBuffer->data, sizeof(unsigned char)*((struct ImageData*)u_data)->width*((struct ImageData*)u_data)->height*3);

    ((struct ImageData*)u_data)->appPad = true;
  }
  return TRUE;
}
