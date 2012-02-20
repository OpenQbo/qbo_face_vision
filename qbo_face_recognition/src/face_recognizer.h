/*
 * CFaceRecognizer.h
 *
 *  Created on: Mar 23, 2011
 *      Author: arturobc
 */

#ifndef FACERECOGNIZER_H_
#define FACERECOGNIZER_H_

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
#include <ctype.h>
#include <ml.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations

#include "qbo_face_recognition/RecognizeFace.h"
#include "qbo_face_recognition/GetName.h"
#include "qbo_face_recognition/Teach.h"

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "person.h"

using std::string;
using std::vector;

class FaceRecognizer {
private:

	void onInit();

	enum RECOGNITION_TYPE {
		PCA_SVM,
		BAG_OF_WORDS_SVM
	};


	/*
	 * ROS parameters variables
	*/
	//Stabilizer
	int stabilizer_threshold_;
	int stabilizer_max_;

	//Faces images paths
	string faces_db_path_;
	string update_path_;

	//Recognition type
	int recog_type_;

	//BoW
	double bow_certainty_threshold_;
	double descriptors_match_threshold_;
	int num_of_desc_per_face_; //Number of descriptors to be extracted per face
	bool linear_kernel_;

	//PCA
	int pca_dimension_;
	cv::Size image_size_; //Image size of the faces in the PCA, in pixels


	/*
	 * Vector of persons' images
	 */
	vector<Person> persons_;


	/*
	 * Variable for PCA
	 */
	cv::PCA pca_;

	/*Variables for BOW*/
	cv::Ptr<cv::FeatureDetector> descriptor_detector_;
	cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;
	cv::BruteForceMatcher<cv::L2<float> > desc_matcher_;

	//Variables for Bag of Words approach
	cv::Ptr<cv::BOWKMeansTrainer> bowTrainer_;
	cv::Ptr<cv::BOWImgDescriptorExtractor> bowExtractor_;
	cv::Mat persons_vocabulary_;

	/* Variables for the stabilizer**/
	std::vector<int> stabilizer_states_;

	/*
	 * ROS Variables
	 */
	ros::NodeHandle private_nh_;
	ros::ServiceServer service_;
	ros::ServiceServer service2_;
	ros::ServiceServer service3_;
	ros::ServiceServer service4_;



	/*Methods*/
	/*
	 * Set ROS parameters
	 */
	void setROSParams();

	/*
	 * Delete the ROS params so as to be newly loaded new time
	 */
	void deleteROSParams();


	/*
	 * Given the persons, load their images
	 */
	int loadAllPersonsImages();

	/*
	 * Used by loadAllPersonsImages method to load the images of a person individually
	 */
	int loadOnePersonImages(Person &person);

	/*
	 * Load the persons, given the faces db path. Fills the vector of Persons
	 *
	 * If succeed, return 0. If < 0, then an error ocurred
	 */
	int loadPersonsNames();


	//For the PCA approach

	/*
	 * Stores Face Recognizer files necessary to recognize faces. It stores
	 * the SVM classifiers for each face, and the PCA necessary to generate the coeffs
	 * Returns true if succeeded and false if not.
	 */
	bool storePCA();


	/*
	 * Loads the pca.xml.gz from faces directory and returns true. If file is not found, then
	 * it will return false;
	 */
	bool loadPCA(string path ="");


	/*
	 * Load svm.xml.gz classifier files. Returns true if succesfully loaded all svm files.
	 * Return false if for at least one object, an svm file has not been found
	 */
	bool loadPCA_SVMClassifiers();


	/*
	 * Trains the SVM Classifiers for each Person, using the PCA
	 */
	void trainPCA_SVMClassifiers();



	void buildPCA();
	string recognizePCA_SVM(cv::Mat img);
	int teachPCA_Recognizer(string update_path = "");


	//For the Bags of Words Approach
	/*
	 * Given the objects, load their descriptors file from the images folder
	 * If a descriptors file isn't found for an object, then it extracts descriptors from
	 * all images of that object
	 */
	int loadAllPersonsDescriptors();


	/*
	 * Given an object, load its descriptors from the images folder.
	 * Return true if succeeded
	 */
	int loadOnePersonDescriptors(Person &person);

	int extractOnePersonDescriptors(Person &person);


	/*
	 * Load svm.xml classifier files. Returns true if succesfully loaded all svm files.
	 * Return false if for at least one object, an svm file has not been found
	 */
	bool loadBOW_SVMClassifiers();

	/*
	 * Trains the SVM Classifiers used for the Bags of Words Approach
	 */
	int trainBOW_SVMClassifiers();

	/*
	 *	Given a path, it extracts the images of each's object's folder,  copy the files to the respective folders in
	 *	objects's main path, add's it to the model, generate the vocabulary, retrain the classifiers.
	 *	Return 0 if succeeded, and != 0 if not succeeded.
	 */
	int teachBOW_Recognizer(string update_path = "");

	/*
	 * Loads the vocabulary.xml from objects_path and returns true. If file is not found, then
	 * it will return false;
	 */
	bool loadVocabulary(string input_path = "");

	/*
	 * Used to train the vocabulary for BOW. Returns the number of descriptors used to train the vocabulary
	 */
	int trainVocabulary();

	/*
	 * Recognize using the BOW SVM classifiers
	 */
	string recognizeBOW_SVM(cv::Mat img);


	//Service for face recognition

	bool recognizeService(qbo_face_recognition::RecognizeFace::Request  &req, qbo_face_recognition::RecognizeFace::Response &res);

	bool recognizeStabilizerService(qbo_face_recognition::RecognizeFace::Request  &req, qbo_face_recognition::RecognizeFace::Response &res);

	bool getNameService(qbo_face_recognition::GetName::Request  &req, qbo_face_recognition::GetName::Response &res);

	bool teachService(qbo_face_recognition::Teach::Request  &req, qbo_face_recognition::Teach::Response &res );


	//Auxiliary functions to get SVM params
	void setSVMParams( CvSVMParams& svmParams, cv::Mat responses);

	void setSVMTrainAutoParams( CvParamGrid& c_grid, CvParamGrid& gamma_grid,
	                            CvParamGrid& p_grid, CvParamGrid& nu_grid,
	                            CvParamGrid& coef_grid, CvParamGrid& degree_grid );

	/*Equalizes image of the face in size (provided by width and height) and in brightness
	 * Returns the equalized image
	 */
	cv::Mat equalizeFace(cv::Mat face);


	/*
	 * Given an image, it extracts the good SURF descriptors from it and add it to the object's model
	 * Returns the number of descriptors added to the model. If <0, then an error has occurred
	 */
	int addDescriptorsToPerson(cv::Mat image, Person &person);

public:

	FaceRecognizer();
	virtual ~FaceRecognizer();


};

#endif /* FACERECOGNIZER_H_ */
