#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <boost/format.hpp>
#include <fstream>

#include "Classifier2.h"
#include "GraphUtils.h"
#define joint_num 6
using namespace std;



class tf_image_extractor
{
private:
  tf::TransformListener listener;
  tf::MessageFilter<sensor_msgs::Image> *tf_filter;
  tf::StampedTransform transform;
  vector<tf::StampedTransform> transform_v;

  vector<cv::Mat> imgs_vec_input_;
  vector<float> joint_vec_input_;
  string model_file;
  string trained_file;
  Classifier* c;

  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  ros::NodeHandle nh_;
  
  string frame_name[joint_num];
  int clip_length;

  vector<vector<float> > visual_result;
  int nlabel;
  vector<string> label_name;

public:

  tf_image_extractor():
    listener(), 
    nlabel(5), //going to write a function to load the number of label
    clip_length(10),
    frame_name{"/l_forearm_link","/l_gripper_palm_link","/l_gripper_tool_frame","/r_forearm_link","/r_gripper_palm_link","/r_gripper_tool_frame"}
  {
    ros::NodeHandle local_nh("~");
    image_sub.subscribe(nh_, "/kinect_head_c2/rgb/image_rect_color" , 3);
    tf_filter = new tf::MessageFilter<sensor_msgs::Image>(image_sub, listener, "base_link", 3);
    tf_filter->registerCallback (boost::bind(&tf_image_extractor::imageCb, this, _1) );
    transform_v.resize(joint_num);
    

    model_file  = "data/deploy_lstmbk.prototxt";
    trained_file = "data/snapshots_lstm_RGB_iter_10000bk.caffemodel";
    c = new Classifier(model_file, trained_file);

    visual_result.resize(nlabel);
    string label[5]={"0","1","2","3","4"};
    label_name.assign(label, label+5);

  }

  ~tf_image_extractor()
  {
    delete c;
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
      {
	for(int i=0 ; i<joint_num; i++){
	  listener.lookupTransform("/base_link", frame_name[i] ,ros::Time(0) ,transform);
	  transform_v[i]=transform;
	}
	try
	  {

	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  } catch(cv_bridge::Exception)
	  {
	    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
	  }
      }
    catch (tf::TransformException ex)
      {
	ros::Duration(0.1).sleep();
      }

    cv::Mat received_img = cv_ptr->image;


    if (!received_img.empty()) 
      {
	
	if(imgs_vec_input_.size()>=clip_length) //slid window
	  {
	    imgs_vec_input_.erase(imgs_vec_input_.begin(), imgs_vec_input_.begin()+3);
	    joint_vec_input_.erase(joint_vec_input_.begin(), joint_vec_input_.begin()+18*3); //1 image correspond to 18 joint positions
	  }
	imgs_vec_input_.push_back(received_img);

	for (int i=0; i<transform_v.size(); i++)
	  {
	    joint_vec_input_.push_back(transform_v[i].getOrigin().x());
	    joint_vec_input_.push_back(transform_v[i].getOrigin().y());
	    joint_vec_input_.push_back(transform_v[i].getOrigin().z());
	  }
	
	if(imgs_vec_input_.size()==clip_length)
	  {

	     vector<float> output = c->Predict(imgs_vec_input_,joint_vec_input_, clip_length);
	     cout<<output.size()<<endl;
	     for(int i=0; i<output.size(); i++)
	       {
		 cout<<output[i]<<endl;
	
		 plotHistogram(received_img,output,label_name);
	       }
	  }
	
      }
  }

  void plotHistogram(cv::Mat src, const vector<float> value, const vector<string> label_name)
  {
    //    if(value.size() == label.size())
    // {
    int bins = value.size();
    int h = cvCeil(src.cols/(bins+1));
    int offset = src.cols;
    int his_offset = src.cols*1.25;

    Mat display(src.rows, src.cols*2, src.type(), cv::Scalar(255,255,255));
    for(int i=0; i<bins; i++)
      {
	int length = cvCeil(value[i]*src.cols*0.5);
	rectangle(display, cv::Point(his_offset,i*h), cv::Point(his_offset+length, (i+1)*h), cv::Scalar(0,255,255), -1);
	putText(display,"push button", cv::Point(offset,(i+1)*h),FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,0,0));
      }
    src.copyTo(display(cv::Rect(0,0,src.cols,src.rows)));
    cv::imshow("result", display);
    cv::waitKey(100);
    //	} 
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_image_extractor");
  tf_image_extractor ti_ex;
  ros::spin();  
  
  }

