/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 OpenQbo, Inc.
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <qbo_talk/Text2Speach.h>

#include <qbo_listen/Listened.h>

#include <qbo_face_recognition/GetName.h>
#include <qbo_face_recognition/Teach.h>
#include <qbo_face_tracking/FacePosAndSize.h>


#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations


using std::string;
using std::vector;

ros::NodeHandle * private_nh_;
ros::Subscriber listener_sub;
ros::Subscriber listener_sub_2;
ros::Subscriber image_sub_;
ros::Subscriber face_pos_sub_ ;
ros::ServiceClient client_talker;
qbo_talk::Text2Speach srv_talker;


ros::ServiceClient client_get_name_;
ros::ServiceClient client_teach;

qbo_face_recognition::GetName srv_get_name_;
qbo_face_recognition::Teach srv_teach;

vector<cv::Mat> received_faces_;

int num_images_to_hold_ = 20;
double wait_for_name_tolerance_ = 4.0;


string new_persons_path_ = "/opt/qbo/ros_stacks/qbo_apps/qbo_face_recognition/faces/new_faces";

bool learn_request = false;
string name_to_learn_ = "";

bool face_detected_ = false; /*Bool and indicates if a face hasn't been detected so as to ingnore listening */


/*
 * Method that, by receiving a string, makes the robot talk that string
 */
void speak_this(string to_speak)
{
	srv_talker.request.command = to_speak;

	if (client_talker.call(srv_talker))
		ROS_INFO("Talked: %s", to_speak.c_str());
	else
		ROS_ERROR("Failed to call the service of qbo_talk");
}

/*
 * Image callback of qbo_face_tracking images to store the images when learning is needed
 */
void faceImageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("Cv_bridge exception by receiving image of qbo_face_tracking: %s", e.what());
		return;
    }

    cv::Mat image_received;

    //Erase vector if it is full
	while((int)received_faces_.size()>=num_images_to_hold_)
		received_faces_.erase(received_faces_.begin());

	//Append it to vector of received messages
	received_faces_.push_back(cv_ptr->image);

	ROS_INFO("Face image received");
}

/*
*/
void facePosCallback(const qbo_face_tracking::FacePosAndSize::ConstPtr& face_pos)
{
	if(face_pos->face_detected)
	{
		face_detected_ = true;
	}
	else
	{
		face_detected_ = false;
	}
}


/*
 * Get the name recognizer by Qbo's face recognizer
 * If string is "", then the face has not been recognized
 */
string getNameFromFaceRecognizer()
{
	string name_recognized = "", str_detected;
	bool bool_detected = false;

	ros::Time time_saved = ros::Time::now();
	ros::Duration time_diff;

	while(1)
	{
		//Use the service
		if (client_get_name_.call(srv_get_name_))
		{
			str_detected = (std::string)(srv_get_name_.response.name);
			bool_detected = srv_get_name_.response.recognized;

			if(bool_detected)
			{
				name_recognized = str_detected;
				break;
			}
		}
		else
		{
			ROS_ERROR("Failed to call service get name from qbo_face_recognition");
			return "";
		}

		time_diff = ros::Time::now() - time_saved;

		if(time_diff.toSec()>=wait_for_name_tolerance_)
			break;
	}

	return name_recognized;
}


/*
 * Given the person's name, it stored the images received in the proper
 * folder and uses the qbo_face_recognition service to teach to the recognizer the new person.
 */
bool learnPerson(string person_name)
{


	/*
	 * Clear vector of received faces
	 */
	received_faces_.clear();

	ROS_INFO("Collecting images for %s", person_name.c_str());

	//Subscribe to face image topic
	image_sub_=private_nh_->subscribe<sensor_msgs::Image>("/qbo_face_tracking/face_image",1,&faceImageCallback);

	//Waiting for a minimum number of face images to arrive
	while((int)received_faces_.size()<num_images_to_hold_)
		ros::spinOnce();

	//Unsubscribe to topic of faces
	image_sub_.shutdown();

	if(!boost::filesystem::is_directory(new_persons_path_))
	{
		//Create the main folder in the save path
		boost::filesystem::create_directory(new_persons_path_);
	}

	if(!boost::filesystem::is_directory(new_persons_path_+"/"+person_name))
		//Create the person's folder
		boost::filesystem::create_directory(new_persons_path_+"/"+person_name);

	string time_now;
	stringstream out;
	out << time(NULL);
	time_now = out.str();


	ROS_INFO("Saving face images in a image files...");
	//Save images
	int image_index = 0;
	for(unsigned int i = 0; i<received_faces_.size();i++)
	{
		string filename;
		stringstream out_2;

		out_2 <<new_persons_path_+"/"+person_name<<"/"<<time_now<<"_"<<image_index<<".jpg";
		filename = out_2.str();

		vector<int> params;

		params.push_back(CV_IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		cv::imwrite(filename.c_str(), received_faces_[i], params);
		image_index++;
	}

	ROS_INFO("Faces images for %s saved. Calling the teach service of Qbo face recognition node.", person_name.c_str());
	srv_teach.request.update_path = new_persons_path_;

	speak_this("I am training myself");
	if (client_teach.call(srv_teach))
	{
		ROS_INFO("Learning DONE!");
	}
	else
	{
		ROS_ERROR("Failed to call service teach new face");
		return false;
	}





	//Delete all folders in new persons path, so as to not repeat these images in the next learning phase
	for (boost::filesystem::directory_iterator itr(new_persons_path_); itr!=boost::filesystem::directory_iterator(); ++itr)
	{
		if (boost::filesystem::is_directory(itr->status()))
		{
			std::string person_name=itr->path().filename();
			boost::filesystem::remove_all(new_persons_path_+"/"+person_name);
		}
	}
	return true;
}


void listenerCallback(const qbo_listen::ListenedConstPtr& msg)
{

	std::string listened = msg->msg;

	ROS_INFO("Listened: %s", listened.c_str());

	
	if(! face_detected_)
	{
		ROS_INFO("Have have not been detected. Ignoring all commands");
		return;
	}
		

	if(learn_request) //A name has been asked to be learned
	{
		if(string(listened) == "YES I DID") //Confirm the name
		{

			//Unsubscribe to topic of listen
			listener_sub.shutdown();
			listener_sub_2.shutdown();

			speak_this("OK "+name_to_learn_+ ". Let me see how you look");

			if(learnPerson(name_to_learn_))
				speak_this("OK. I am Ready to recognize you");
			else
				speak_this("Oh oh. Problem. I could not train myself");

			learn_request = false;

			/*
			 * Re-subscribe to topic of listening
			 */
		//	listener_sub = private_nh_->subscribe<qbo_listen::Listened>("/listen/en_face_recog",20,&listenerCallback);
			listener_sub_2 = private_nh_->subscribe<qbo_listen::Listened>("/listen/en_default",20,&listenerCallback);

		}
		else if(string(listened) == "NO" || string(listened) == "NO I DID NOT")
		{
			learn_request = false;
			speak_this("If your name is not "+name_to_learn_+". What is your name?");
			//cv::waitKey(5000);
		}
		else
			return;
	}

	/*
	 * Query sentences
	 */
	if(string(listened) == "WHAT IS MY NAME" || string(listened) == "WHAT'S MY NAME")
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			//speak_this("This is a "+recognized_person);
			speak_this("Your name is "+rec_name);
		else
			speak_this("Sorry but I don't know");
	}
	else if(string(listened) == "HI CUBE O HOW ARE YOU")
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			//speak_this("This is a "+recognized_person);
			speak_this("Hello "+rec_name+". I am fine thank you");
		else
			speak_this("Hi. I am fine thank you");
	}

	else if(string(listened) == "HELLO CUBE E O" || string(listened) == "HELLO CUBE")
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			speak_this("Hello "+rec_name+". How are you?");
		else
			speak_this("Hello. How are you?");
	}

	else if(string(listened) == "ARE YOU SURE")
	{
		string rec_name;
		rec_name = getNameFromFaceRecognizer();

		if(rec_name != "")
			speak_this("Yes. I am sure. Your name is "+rec_name);
		else
			speak_this("Sorry but I am not sure.");
	}

	/*
	 * Learning sentences
	 */
	else
	{
		vector<string> words;
		boost::split(words, listened, boost::is_any_of(" "));
		if(words.size()> 3 && words[0]=="MY" && words[1] == "NAME") //My name is #####
		{	//Erase "MY NAME IS"
			for(unsigned int i = 0; i<3;i++)
			{
				words.erase(words.begin());
			}

			string person_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				person_name+=words[i];

				if(i!=words.size()-1)
					person_name+=" ";
			}

			if(person_name == "JUAN WHO")
				person_name = "WHOANDHO";


			speak_this("Did you say "+person_name+"?");
			learn_request = true;
			name_to_learn_ = person_name;
		}
		else if(words.size()> 2 && words[0]=="I" && words[1] == "AM") //I am #####
		{
			//Erase I AM
			for(unsigned int i = 0; i<2;i++)
			{
				words.erase(words.begin());
			}


			string person_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				person_name+=words[i];

				if(i!=words.size()-1)
					person_name+=" ";
			}

			if(person_name == "JUAN WHO")
				person_name = "WHOANDHO";

			speak_this("Did you say "+person_name+"?");
			learn_request = true;
			name_to_learn_ = person_name;
		}

		//Hello Cube O My name is
		else if(words.size()> 6 && words[0]=="HELLO" && words[1] == "CUBE" && words[3] == "MY")
		{
			//Erase HELLO CUBE O MY NAME IS
			for(unsigned int i = 0; i<6;i++)
			{
				words.erase(words.begin());
			}


			string person_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				person_name+=words[i];

				if(i!=words.size()-1)
					person_name+=" ";
			}

			if(person_name == "JUAN WHO")
				person_name = "WHOANDHO";

			speak_this("Did you say "+person_name+"?");
			learn_request = true;
			name_to_learn_ = person_name;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qbo_face_recognition_demo");

	private_nh_ = new ros::NodeHandle;

	/*
	 * Set service client for qbo talker
	 */
	client_talker = private_nh_->serviceClient<qbo_talk::Text2Speach>("/Qbo/festivalSay");

	/*
	 * Set service clients for face recognition
	 */
	client_get_name_ = private_nh_->serviceClient<qbo_face_recognition::GetName>("/qbo_face_recognition/get_name");
	client_teach = private_nh_->serviceClient<qbo_face_recognition::Teach>("/qbo_face_recognition/teach");

	/*
	 * Set listener subscriber to listen to the respective topics
	 */
//	listener_sub = private_nh_->subscribe<qbo_listen::Listened>("/listen/en_face_recog",20,&listenerCallback);
	listener_sub_2 = private_nh_->subscribe<qbo_listen::Listened>("/listen/en_default",20,&listenerCallback);


	/*
	* Callback for face tracking to check for faces
	*/

    face_pos_sub_ = private_nh_->subscribe<qbo_face_tracking::FacePosAndSize>("/qbo_face_tracking/face_pos_and_size", 10, &facePosCallback);


	/*
	 * Set ROS parameters
	 */
	private_nh_->param<int>("/qbo_face_recognition_demo/num_images_to_hold", num_images_to_hold_, 20);
	private_nh_->param<double>("/qbo_face_recognition_demo/wait_for_name_tolerance", wait_for_name_tolerance_, 4.0);

	/*
	 * Get update path parameter of the node
	 */
	private_nh_->getParam("/qbo_face_recognition/update_path", new_persons_path_);

	if(new_persons_path_ == "")
		new_persons_path_ = "/opt/qbo/ros_stacks/qbo_apps/qbo_face_recognition/faces/new_faces";

	ROS_INFO("Faced Recognition Demo Launched. Ready for incoming orders");
//	speak_this("I am ready to recognize faces");
	ros::spin();

	private_nh_->deleteParam("/qbo_face_recognition_demo/num_images_to_hold");
	private_nh_->deleteParam("/qbo_face_recognition_demo/wait_for_name_tolerance");

	return 0;
}

