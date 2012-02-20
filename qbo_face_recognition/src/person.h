/*
 * CFaceObject.h
 *
 *  Created on: Jun 9, 2011
 *      Author: Arturo Bajuelos Castillo
 */

#ifndef PERSON_H_
#define PERSON_H_

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>
#include <ml.h>

using std::string;
using std::vector;

class Person {
public:
	Person(string name="");

	vector<cv::Mat> images_;

	string name_;

	string images_dir_path_;

	cv::Ptr<CvSVM> pca_svm_;

	cv::Mat pca_proj_;

	cv::Mat descriptors_;

	cv::Ptr<CvSVM> bow_svm_; //Used for Bag of Words Approach


	virtual ~Person();
};

#endif /* PERSON_H_ */
