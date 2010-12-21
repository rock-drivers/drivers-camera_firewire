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
#include <libraw1394/raw1394.h>
#include <dc1394/dc1394.h>
#include <sys/stat.h>

using namespace camera;
using namespace base::samples::frame;

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
    std::cerr << "argv[1] = " << argv[1] << std::endl;

    bool stereo = true;

    Frame left_frame;
    Frame right_frame;

    //frame_size_t size(752,480);
    frame_size_t size(640,480);

    left_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);
    if(stereo) right_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);
        
    // create a new firewire bus device
    dc1394_t *dc_device = dc1394_new();

    CamFireWire left_camera;
    left_camera.setDevice(dc_device);
    left_camera.cleanup();
    
    CamFireWire right_camera;
    right_camera.setDevice(dc_device);

    CamInterface &left_cam = left_camera;
    CamInterface &right_cam = right_camera;
    
    //find and display all cameras
    std::vector<CamInfo> cam_infos ;
    left_cam.listCameras(cam_infos);

    left_cam.open(cam_infos[1], Master);
    left_cam.setAttrib(camera::int_attrib::IsoSpeed, 400);
    left_cam.setAttrib(camera::double_attrib::FrameRate, 15);        

    if(stereo) 
    {
        if(!right_cam.open(cam_infos[0], Monitor))
        {
	    left_cam.setAttrib(camera::double_attrib::FrameRate, 15);
	    right_cam.setAttrib(camera::double_attrib::FrameRate, 15);
	    left_cam.close();
	    right_cam.close();
	    left_cam.open(cam_infos[0], Master);
	    right_cam.open(cam_infos[1], Monitor);
	    right_cam.setAttrib(camera::int_attrib::IsoSpeed, 400);
        }
    }

    frame_size_t fs;
  
    fs.height = 480;
    fs.width = 640; //fs.width = 752;
    cv::namedWindow("left",1);
    if(stereo) cv::namedWindow("right",1);
    cvMoveWindow("left",10,10);
    if(stereo) cvMoveWindow("right",800,10);

    left_cam.setFrameSettings(fs, MODE_BAYER_RGGB, 8, false);
    left_cam.setAttrib(int_attrib::GainValue, 16);
    left_cam.setAttrib(enum_attrib::GammaToOn);
    left_cam.setAttrib(int_attrib::ExposureValue, 300);
    left_cam.setAttrib(int_attrib::WhitebalValueBlue, 580);
    left_cam.setAttrib(int_attrib::WhitebalValueRed, 650);
    left_cam.setAttrib(int_attrib::AcquisitionFrameCount, 200);
    left_cam.setAttrib(enum_attrib::ExposureModeToManual);
    //left_cam.setAttrib(enum_attrib::ExposureModeToAuto);

    if(stereo)
    {
        right_cam.setFrameSettings(fs, MODE_BAYER_RGGB, 8, false);
        right_cam.setAttrib(int_attrib::GainValue, 16);
        right_cam.setAttrib(enum_attrib::GammaToOn);
        right_cam.setAttrib(int_attrib::ExposureValue,300);
        right_cam.setAttrib(int_attrib::WhitebalValueBlue, 580);
        right_cam.setAttrib(int_attrib::WhitebalValueRed, 650);
	right_cam.setAttrib(int_attrib::AcquisitionFrameCount, 200);
        right_cam.setAttrib(enum_attrib::ExposureModeToManual);
	//right_cam.setAttrib(enum_attrib::ExposureModeToAuto);

    }

    timeval ts, te, tcurr, tprev;
    
    gettimeofday(&ts,NULL);
    gettimeofday(&tcurr,NULL);

    left_cam.setAttrib(camera::double_attrib::FrameRate, 4);
    if(stereo) right_cam.setAttrib(camera::double_attrib::FrameRate,4);

    left_camera.clearBuffer();
    if(stereo) right_camera.clearBuffer();
    
    int total_frames = 0;
	
    time_t rawtime;
    tm * ptm;

    time ( &rawtime );

    ptm = gmtime ( &rawtime );

    char path[100];
    sprintf(path, "./started_at_%d_%02d_%02d_%02dh%02dm%02ds", 1900+ptm->tm_year, 1+ptm->tm_mon, ptm->tm_mday, 2+ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

    mkdir(path, 0777);    
    
    left_cam.grab(Continuously, 20);
    if(stereo)   right_cam.grab(Continuously, 20);
    
    for(int i = 0 ; i< 10000 ; i++)
    {
	uint32_t val;

        cvWaitKey(10);
	left_camera.retrieveFrame(left_frame,0);
	if(stereo) right_camera.retrieveFrame(right_frame,0);;

	imshow("left",left_frame.convertToCvMat());
	if(stereo) imshow("right", right_frame.convertToCvMat());
	
	char *filename = new char[100];
	sprintf(filename, "%s/left%08d_ts%13ld.pgm",path, i, left_frame.time);
	//cv::imwrite(filename,left_frame.convertToCvMat());
	
	if(stereo)
        {
	    sprintf(filename, "%s/right%08d.pgm",path,i);
	    //cv::imwrite(filename,right_frame.convertToCvMat());
        }	
	
	if(cvWaitKey(2) != -1) {total_frames = i; break;}
	
	tprev=tcurr;
	gettimeofday(&tcurr,NULL);
        double delta_t = (1000000*tcurr.tv_sec+tcurr.tv_usec - 1000000*tprev.tv_sec-tprev.tv_usec)/1000000.0;
	std::cerr << "\n" << delta_t << " seconds = " << 1.0/delta_t << " fps" << std::endl;
	total_frames = i;
    }
   
    gettimeofday(&te,NULL);
    double total_time = (1000000*te.tv_sec+te.tv_usec - 1000000*ts.tv_sec-ts.tv_usec)/1000000.0;
    std::cerr << total_time << " seconds" << std::endl;
    std::cerr << total_frames / total_time << " fps avg" << std::endl;
    cvWaitKey();

    left_cam.setAttrib(camera::double_attrib::FrameRate,15);
    if(stereo) right_cam.setAttrib(camera::double_attrib::FrameRate,15);
    
    //close camera
    left_cam.close();
    if(stereo) right_cam.close();
   
    return 0;
}


