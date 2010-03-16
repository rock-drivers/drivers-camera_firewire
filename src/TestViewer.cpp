/*
 * Copyright (c) 2009, Sun Microsystems, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in 
 *    the documentation and/or other materials provided with the distribution.
 *  * Neither the name of Sun Microsystems, Inc. nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

#include <iostream>
#include <highgui.h>
#include <cv.h>
#include "CamFireWire.h"
//#include "../../camera_avt_guppy/src/AVT_Guppy.h"
#include <libraw1394/raw1394.h>

using namespace camera;

void plotCameras ( std::vector<CamInfo> &cam_infos)
{
    int inum = cam_infos.size();
    std::cout << "Found " << inum<< " cameras:\n";

    for(int i=0;i<inum;i++)
    {
        std::cout << "Name: "<< cam_infos[i].display_name << std::endl;
        std::cout << "ID: "<< cam_infos[i].interface_id << std::endl;
        std::cout << "unique ID: "<< cam_infos[i].unique_id << std::endl;
        std::cout << "IP: "<< cam_infos[i].ip_settings.current_ip_address;
        std::cout << std::endl << std::endl ;
    }
    std::cout<<std::endl;
}

int main(int argc, char**argv)
{
    bool stereo = false;

    Frame left_frame;
    Frame right_frame;

    //frame_size_t size(752,480);
    frame_size_t size(640,480);

    
    left_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);
    if(stereo) right_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);

    CamFireWire left_camera;
    CamFireWire right_camera;
    CamInterface &left_cam = left_camera;
    CamInterface &right_cam = right_camera;

    //find and display all cameras
    std::vector<CamInfo> cam_infos;
    left_cam.listCameras(cam_infos);
    std::cerr << "cam.isOpen = " << left_cam.isOpen() << std::endl;
    cvWaitKey(100);
    left_cam.open(cam_infos[0], Master);
    left_cam.setAttrib(camera::int_attrib::IsoSpeed, 400);
  
    if(stereo) right_cam.open(cam_infos[1], Monitor);
    if(stereo) left_cam.setAttrib(camera::int_attrib::IsoSpeed, 200);
    if(stereo) right_cam.setAttrib(camera::int_attrib::IsoSpeed, 200);

    cvWaitKey(100);
    std::cerr << "cam.isOpen = " << left_cam.isOpen() << std::endl;
    frame_size_t fs;
  
    fs.height = 480;
    fs.width = 640; //fs.width = 752;
    cv::namedWindow("left",1);
    if(stereo) cv::namedWindow("right",1);
    cvMoveWindow("left",10,10);
    if(stereo) cvMoveWindow("right",800,10);

    cvWaitKey(100);

    left_cam.setFrameSettings(fs, MODE_BAYER_RGGB, 8, false);
    left_cam.setAttrib(int_attrib::GainValue, 16);
    left_cam.setAttrib(enum_attrib::GammaToOn);
    left_cam.setAttrib(int_attrib::ExposureValue, 300);
    left_cam.setAttrib(int_attrib::WhitebalValueBlue, 580);
    left_cam.setAttrib(int_attrib::WhitebalValueRed, 650);
    left_cam.setAttrib(int_attrib::AcquisitionFrameCount, 200);

    if(stereo)
    {
        right_cam.setFrameSettings(fs, MODE_BAYER_RGGB, 8, false);
        right_cam.setAttrib(int_attrib::GainValue, 16);
        right_cam.setAttrib(enum_attrib::GammaToOn);
        right_cam.setAttrib(int_attrib::ExposureValue,300);
        right_cam.setAttrib(int_attrib::WhitebalValueBlue, 580);
        right_cam.setAttrib(int_attrib::WhitebalValueRed, 650);
	right_cam.setAttrib(int_attrib::AcquisitionFrameCount, 200);
    }

    cvWaitKey(50);
    timeval ts, te;
    
    gettimeofday(&ts,NULL);

    left_cam.setAttrib(camera::double_attrib::FrameRate,30);
    if(stereo) right_cam.setAttrib(camera::double_attrib::FrameRate,30);
    
    left_camera.clearBuffer();
    if(stereo) right_camera.clearBuffer();
    
    for(int i = 0 ; i< 200 ; i++)
    {
	left_cam.grab(SingleFrame, 10);
        std::cerr << "retrieving..." << std::endl;
	left_cam.retrieveFrame(left_frame,0);
	if(stereo) right_cam.retrieveFrame(right_frame,0);;
	imshow("left",left_frame.convertToCvMat());
	if(stereo) imshow("right", right_frame.convertToCvMat());
	cvWaitKey(2);
    }
    
    gettimeofday(&te,NULL);
    std::cerr << (1000000*te.tv_sec+te.tv_usec - 1000000*ts.tv_sec-ts.tv_usec)/1000000.0 << " seconds" << std::endl;
    cvWaitKey();

    //close camera
    left_cam.close();
    if(stereo) right_cam.close();
   
    return 0;
}


