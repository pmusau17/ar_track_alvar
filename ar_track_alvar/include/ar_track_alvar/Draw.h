/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#ifndef DRAW_H
#define DRAW_H

/**
 * \file Draw.h
 *
 * \brief This file implements a collection of functions that are used to
 * visualize lines, contours and corners for debugging purposes.
 */

#include "Alvar.h"
#include "Util.h"
#include "Camera.h"
#include "Line.h"
#include <sstream>

namespace alvar {

/** \brief Draws the bounding box of a connected component (Blob).
 *  \param image Pointer to the destination image.
 *  \param points Vector of points that determine the bounding box.
 *  \param color Use CV_RGB(red,green,blue) to determine the color of the bounding box.
 *  \param label A label to show in the center of the bounding box.
 */
template<class PointType>
void inline DrawBB(cv::Mat *image, const std::vector<PointType>& points, CvScalar color, std::string label="")
{
    if (points.size() < 2) {
        return;
    }
    PointType minimum = PointType(image->rows, image->cols);
    PointType maximum = PointType(0, 0);
    for (int i = 0; i < (int)points.size(); ++i) {
        PointType current = points.at(i);
        if (current.x < minimum.x) minimum.x = current.x;
        if (current.x > maximum.x) maximum.x = current.x;
        if (current.y < minimum.y) minimum.y = current.y;
        if (current.y > maximum.y) maximum.y = current.y;
    }
    cv::line(*image, cv::Point((int)minimum.x,(int)minimum.y), cv::Point((int)maximum.x,(int)minimum.y), color);
	  cv::line(*image, cv::Point((int)maximum.x,(int)minimum.y), cv::Point((int)maximum.x,(int)maximum.y), color);
	  cv::line(*image, cv::Point((int)maximum.x,(int)maximum.y), cv::Point((int)minimum.x,(int)maximum.y), color);
	  cv::line(*image, cv::Point((int)minimum.x,(int)maximum.y), cv::Point((int)minimum.x,(int)minimum.y), color);
    // if (!label.empty()) {
    //     CvFont font;
		//cvInitFont(&font, 0, 0.5, 0.5, 0);
		cv::putText(*image, label.c_str(), cv::Point((int)minimum.x+1,(int)minimum.y+2), cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    }
}

/** \brief Draws a set of points.
 *  \param image Pointer to the destination image.
 *  \param points Array of CvPoints to be visualzed.
 *  \param color Use CV_RGB(red,green,blue) to determine the color.
*/
void ALVAR_EXPORT DrawPoints(cv::Mat *image, const std::vector<CvPoint>& points, CvScalar color);

/** \brief Draws lines between consecutive points stored in vector (polyline).
  * \param image	Pointer to the destination image.
  * \param points	Vector of points that determine the polyline.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
  * \param loop		If true, the polyline is closed.
*/
template<class PointType>
void inline DrawLines(cv::Mat *image, const std::vector<PointType>& points, CvScalar color, bool loop=true)
{
	for(unsigned i = 1; i < points.size(); ++i)
		cv::line(*image, cv::Point((int)points[i-1].x,(int)points[i-1].y), cv::Point((int)points[i].x,(int)points[i].y), color);
	if (loop) {
		cv::line(*image, cv::Point((int)points[points.size()-1].x,(int)points[points.size()-1].y), cv::Point((int)points[0].x,(int)points[0].y), color);
	}
}

/** \brief			Draws a line.
  * \param image	Pointer to the destination image.
  * \param line		Line struct to be drawn.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
*/
void ALVAR_EXPORT DrawLine(cv::Mat* image, const alvar::Line	line, cv::Scalar color = cv::Scalar(0,255,0));

/** \brief Draws points of the contour that is obtained by \e Labeling class.
  * \param image	Pointer to the destination image.
  * \param contour	Controur sequence.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
*/
void ALVAR_EXPORT DrawPoints(cv::Mat* image, const CvSeq* contour, cv::Scalar color = cv::Scalar(255,0,0));


/** \brief Draws circles to the contour points that are obtained by \e Labeling class.
  * \param image	Pointer to the destination image.
  * \param contour	Controur sequence.
  * \param radius	Circle radius in pixels.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
*/
void ALVAR_EXPORT DrawCircles(cv::Mat* image, const CvSeq* contour, int radius, cv::Scalar color = cv::Scalar(255,0,0));

/** \brief Draws lines between consecutive contour points.
  * \param image	Pointer to the destination image.
  * \param contour	Controur sequence.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
*/
void ALVAR_EXPORT DrawLines(cv::Mat* image, const CvSeq* contour, cv::Scalar color = cv::Scalar(255,0,0));

/** \brief Draws circles to the array of points.
  * \param image	Pointer to the destination image.
  * \param points	Vector of points to be visualized.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
  * \param radius	Circle radius in pixels.
*/
template<class PointType>
void inline DrawPoints(cv::Mat *image, const std::vector<PointType>& points, cv::Scalar color, int radius=1)
{
	for(unsigned i = 0; i < points.size(); ++i)
		cv::circle(*image, cv::Point((int)points[i].x,(int)points[i].y), radius, color);
}

/** \brief Draws OpenCV ellipse.
  * \param image	Pointer to the destination image.
  * \param ellipse	Ellipse struct in OpenCV format.
  * \param color	Use CV_RGB(red,green,blue) to determine the color.
  * \param fill		If false, only the outline is drawn.
  * \param par		The ellipse width and height are grown by \e par pixels.
*/
void ALVAR_EXPORT DrawCVEllipse(cv::Mat* image, CvBox2D& ellipse, cv::Scalar color, bool fill=false, double par=0);

/** \brief This function is used to construct a texture image which is needed to hide a marker from the original video frame. See \e SampleMarkerHide.cpp for example implementation. 
  * \param image		Pointer to the original video frame from where the hiding texture is calculated.
  * \param hide_texture	Pointer to the destination image where the resulting texture is stored.
  * \param cam			Camera object that is used for marker tracking.
  * \param gl_modelview	Current model view matrix.
  * \param topleft		Top left limit of the texture area in marker coordinate frame.
  * \param botright		Bottom right limit of the texture area in marker coordinate frame.
 */
void ALVAR_EXPORT BuildHideTexture(cv::Mat *image, cv::Mat *hide_texture, 
	alvar::Camera *cam, double gl_modelview[16], 
	alvar::PointDouble topleft, 	alvar::PointDouble botright);

/** \brief Draws the texture generated by \e BuildHideTexture to given video frame. For better performance, use OpenGL instead. See \e SampleMarkerHide.cpp for example implementation. 
 *  \param image		Pointer to the destination image where the texture is drawn.
 *  \param texure		Pointer to the texture image genereated by \e BuildHideTexture.
 *  \param cam			Camera object that is used for marker tracking.
 *  \param gl_modelview	Current model view matrix.
 *  \param topleft		Top left limit of the texture area in marker coordinate frame.
 *  \param botright		Bottom right limit of the texture area in marker coordinate frame. 
 */
void ALVAR_EXPORT DrawTexture(cv::Mat *image, cv::Mat *texture, 
	alvar::Camera *cam, double gl_modelview[16], 
	alvar::PointDouble topleft, alvar::PointDouble botright);
  
//}// namespace alvar

#endif
