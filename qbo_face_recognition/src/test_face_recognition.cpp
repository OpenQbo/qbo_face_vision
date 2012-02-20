/*
 * test_face_recognition.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: Arturo Bajuelos Castillo
 */

#include "ros/ros.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <string>


#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>


#include "gslibcam.h"

#include <cv_bridge/cv_bridge.h>
#include <new_face_recognition/RecognizeFace.h>
#include <new_face_recognition/Teach.h>
cv::Mat img, img_BGR;
cv::Rect selection;
cv::Point origin;
int select_object;
int track_object;

using namespace std;

cv::CascadeClassifier face_classifier_;


/*To test FaceRecognizer**/
ros::ServiceClient client_svm;
ros::ServiceClient client_teach;
ros::ServiceClient client_nb;


new_face_recognition::RecognizeFace srv;
new_face_recognition::Teach srv_teach;

cv_bridge::CvImage cv_image;


unsigned int detectFaces(cv::Mat image, std::vector<cv::Rect> &faces)
{
	int flag= CV_HAAR_FIND_BIGGEST_OBJECT;
	cv::Size size=cv::Size();

	face_classifier_.detectMultiScale(image, faces, 1.1, 2,  flag, size);

	return faces.size();
}

string sendToRecognizerSVM(cv::Mat face)
{
	cv_image.image = face;

	cv_image.header.stamp=ros::Time::now();
	cv_image.header.frame_id="face";
	cv_image.encoding = "mono8";

	cv_image.toImageMsg(srv.request.face);

	string name_detected = "";
	//Use the service
	if (client_svm.call(srv))
	{
		//ROS_INFO("Sum: %ld and %s", (long int)srv.response.sum, ((std::string)srv.response.name).c_str());

		name_detected = (std::string)(srv.response.name);
	}
	else
	{
		ROS_ERROR("Failed to call service recognize face");
		return "";
	}

	return name_detected;

}

void teachRecognizer()
{

	srv_teach.request.update_path = "/home/arturobc/ros_workspace/Object_images/New_Objects";

	if (client_teach.call(srv_teach))
	{
		//ROS_INFO("Sum: %ld and %s", (long int)srv.response.sum, ((std::string)srv.response.name).c_str());
		ROS_INFO("DONE!");
	}
	else
	{
		ROS_ERROR("Failed to call service recognize face");
		return;
	}
}

//string sendToRecognizerNaiveBayes(cv::Mat face)
//{
//	cv_image.image = face;
//
//	cv_image.header.stamp=ros::Time::now();
//	cv_image.header.frame_id="face";
//	cv_image.encoding = "mono8";
//
//	cv_image.toImageMsg(srv.request.face);
//
//	string name_detected = "";
//
//
//	//Use the service
//	if (client_nb.call(srv))
//	{
//		//ROS_INFO("Sum: %ld and %s", (long int)srv.response.sum, ((std::string)srv.response.name).c_str());
//
//		name_detected = (std::string)(srv.response.name);
//	}
//	else
//	{
//		ROS_ERROR("Failed to call service recognize face NAIVE BAYES");
//		return "";
//	}
//
//	return name_detected;
//
//}


int main(int argc, char**argv)
{


    cv::namedWindow("Camera Image Window",1);
    cvMoveWindow("Camera Image Window", 0,0);

	//Integer for time mark used to calculate the frame ratio
	timeval startTime, endTime;
	double diff;

	//Preparing face classifier
	face_classifier_.load("/opt/ros/diamondback/stacks/vision_opencv/opencv2/opencv/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml");

	char opt;

	Camera camera("/dev/video0", 320 ,240, "camera0");

	cv::Mat img, img_BGR, grayscale;

	cv::Rect object_roi;

	vector<cv::Rect> faces;

	int frame_id = 0;

	ros::init(argc, argv, "face_recognition_tester");


	ros::NodeHandle nh;

	/*For face recognizer*/
	client_svm = nh.serviceClient<new_face_recognition::RecognizeFace>("/face_recognizer/recognize_face_with_stabilizer");
	client_teach = nh.serviceClient<new_face_recognition::Teach>("/face_recognizer/teach");
//	client_nb = nh.serviceClient<new_face_recognition::RecognizeFace>("/face_recognizer/recognize_face");



	string name_detected_svm = "";
//	string name_detected_nb = "", name_detected_BOW = "";
	//Normal cycle
	while(1)
	{
		//Start timer
		gettimeofday(&startTime, NULL);

		camera.toMat(img);
		cv::cvtColor(img, img_BGR, CV_RGB2BGR);
		cv::cvtColor(img, grayscale, CV_BGR2GRAY);

		/*************Cycle Beginning***************/


		detectFaces(img, faces);

		if(faces.size()>0)
		{
			object_roi = faces[0];

			name_detected_svm = sendToRecognizerSVM(grayscale(object_roi));
//			name_detected_nb = sendToRecognizerNaiveBayes(grayscale(object_roi));
//			name_detected_BOW = sendToRecognizerBagsOfWords(img(object_roi));
		}



		/*************Cycle end***************/


		//Draw name of person in image
		cv::putText(img_BGR, "BOW+SVM: "+name_detected_svm, cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
//		cv::putText(img_BGR, "PCA+Naive Bayes: "+name_detected_nb, cv::Point(20,40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 2);
//		cv::putText(img_BGR, "BOW+SVM: "+name_detected_BOW, cv::Point(20,60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);

		cv::rectangle(img_BGR, object_roi, cv::Scalar(0,0,255), 2);



		cv::imshow("Camera Image Window", img_BGR);

//		if(faces.size()>0)
//		{
//
//			cv::Mat face_img = grayscale(object_roi);
//			cv::Mat mask = cv::Mat::zeros(face_img.size(), face_img.type());
//			cv::Mat mask2 = cv::Mat::ones(face_img.size(), face_img.type())*255/2.;
//
//			cv::RotatedRect rot_rect;
//			rot_rect.angle = 0;
//			rot_rect.size = cv::Size(face_img.cols*0.85,face_img.rows*0.95);
//			rot_rect.center = cv::Point(object_roi.width/2, object_roi.height/2 );
//			cv::ellipse(mask, rot_rect, cv::Scalar(255,255,255), -1);
//			cv::ellipse(mask2, rot_rect, cv::Scalar(0,0,0), -1);
//
//
//			cv::equalizeHist(face_img, face_img);
//
//			cv::bitwise_and(face_img, mask, face_img);
//			cv::bitwise_or(face_img, mask2, face_img);
//
//			printf("Face dimentions: (%d, %d)\n", face_img.rows, face_img.cols);
//
//			cv::imshow("Face", face_img);
//		}

		//WaitKey to show image
		opt = cv::waitKey(50);

		if(opt == 't' || opt == 'T')
		{
			teachRecognizer();
		}
		else if (opt>0)
		{
			printf("Program ended!\n");
			break;
		}
		//Get stop time to calc the time difference
		gettimeofday(&endTime, NULL);

		diff = double((endTime.tv_sec-startTime.tv_sec)*1000);
		diff+= double((endTime.tv_usec-startTime.tv_usec)/1000);

		//Print time period
		printf("Frame %d rate: %6.3lg ms\n", frame_id, diff);


		frame_id++;
	}


// the camera will be deinitialized automatically in VideoCapture destructor
return 0;

}

