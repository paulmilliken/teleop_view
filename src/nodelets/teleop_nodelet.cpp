/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <std_msgs/Bool.h>
#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: teleop_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that teleop_view exits.
static void destroyNode(GtkWidget *widget, gpointer data)
{
  /// @todo On ros::shutdown(), the node hangs. Why?
  //ros::shutdown();
  exit(0); // brute force solution
}

static void destroyNodelet(GtkWidget *widget, gpointer data)
{
  // We can't actually unload the nodelet from here, but we can at least
  // unsubscribe from the image topic.
  reinterpret_cast<image_transport::Subscriber*>(data)->shutdown();
}
#endif

namespace teleop_view {

class TeleopNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_image;
  ros::Subscriber sub_mirror;
  ros::Publisher pub;

  boost::mutex image_mutex_;
  sensor_msgs::ImageConstPtr last_msg_;
  cv::Mat last_image_;
  
  std::string window_name_;
  boost::format filename_format_;
  int count_;

  virtual void onInit();
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void displayStaleImageText(void);
  bool stale_flag;
  int staleness_counter;
  void mirrorCb(const std_msgs::Bool msg);
  bool mirror_flag;
  bool odd_frame;
/*  static void mouseCb(int event, int x, int y, int flags, void* param); */

public:
  TeleopNodelet();

  ~TeleopNodelet();
};

TeleopNodelet::TeleopNodelet()
  : filename_format_(""), count_(0)
{
}

TeleopNodelet::~TeleopNodelet()
{
  cv::destroyWindow(window_name_);
}

void TeleopNodelet::onInit()
{
  stale_flag = false;
  staleness_counter = 0;
  mirror_flag = false;
  odd_frame=false;
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();

  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport = "raw";
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
  // Internal option, should be used only by the teleop_view node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  bool autosize;
  local_nh.param("autosize", autosize, false);
  
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  if (autosize) {
    cv::namedWindow(window_name_, CV_WINDOW_AUTOSIZE);
  } else {
    cv::namedWindow(window_name_, CV_WINDOW_NORMAL);
  }
  cvSetWindowProperty(window_name_.c_str(), CV_WND_PROP_FULLSCREEN, 
							CV_WINDOW_FULLSCREEN);
#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
  else
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_image);
#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), 
                                                        getPrivateNodeHandle());
  sub_image = it.subscribe(topic, 1, &TeleopNodelet::imageCb, this, hints);
  sub_mirror = nh.subscribe("/axis/mirror", 1, &TeleopNodelet::mirrorCb, this);
  pub = nh.advertise<std_msgs::Bool>("is_stale", 1);
  ros::Rate r(2);
  std_msgs::Bool msg;
  while(ros::ok()) {
    staleness_counter++;
    if (staleness_counter>2) {
      stale_flag = true; // stale_flag defaults to true until reset by imageCb()
      TeleopNodelet::displayStaleImageText();
    } else {
      stale_flag = false;
    }
    msg.data = stale_flag;
    pub.publish(msg);
    r.sleep();
  }
}

void TeleopNodelet::mirrorCb(const std_msgs::Bool msg)
{
  mirror_flag = msg.data;
}
    
void TeleopNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  image_mutex_.lock();
  odd_frame = (!odd_frame); // toggle odd_frame between true and false

  // May want to view raw bayer data, which CvBridge doesn't know about
  if (msg->encoding.find("bayer") != std::string::npos)
  {
    last_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t*>(&msg->data[0]), msg->step);
  }
  // We want to scale floating point images so that they display nicely
  else if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image_bridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    float max_val = 0;
    for(int i = 0; i < float_image.rows; ++i)
    {
      for(int j = 0; j < float_image.cols; ++j)
      {
        max_val = std::max(max_val, float_image(i, j));
      }
    }

    if(max_val > 0)
    {
      float_image /= max_val;
    }
    last_image_ = float_image;
  } 
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  } 

  // last_image_ may point to data owned by last_msg_, so we hang onto it for
  // the sake of other callback functions.
  last_msg_ = msg;

  if (!last_image_.empty()) {
    staleness_counter = 0;
    if (mirror_flag) {
      cv::flip(last_image_, last_image_, 1);
      cv::putText(last_image_, "Mirrored", cv::Point(20, 30),
			CV_FONT_HERSHEY_PLAIN, 1.5, CV_RGB(250,0,0), 2);
    }
  }
  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();
  if (!last_image_.empty()) {
    if (odd_frame) {
      cv::imshow(window_name_, last_image_);
    }
  }
}

void TeleopNodelet::displayStaleImageText(void)
{
  if (! last_image_.empty()) {
    cv::putText(last_image_, "Stream is not live", cv::Point(50, 100), 
			CV_FONT_HERSHEY_PLAIN, 4, CV_RGB(250,0,0), 5);
    cv::imshow(window_name_, last_image_);
  }
}

} // namespace teleop_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( teleop_view::TeleopNodelet, nodelet::Nodelet)
