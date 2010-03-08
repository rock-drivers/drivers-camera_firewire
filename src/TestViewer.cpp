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
//#include "../../prosilica/GigE/CamGigEProsilica.h"
#include "CamFireWire.h"
#include "../../camera_avt_guppy/AVT_Guppy.h"
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

    frame_size_t size(752,480);

    left_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);
    if(stereo) right_frame.init(size.width,size.height,3,MODE_BAYER_RGGB,false);


    AVT_Guppy left_camera;
    AVT_Guppy right_camera;
    CamInterface &left_cam = left_camera;
    CamInterface &right_cam = right_camera;

    //find and display all cameras
    std::vector<CamInfo> cam_infos;
    left_cam.listCameras(cam_infos);
    std::cerr << "cam.isOpen = " << left_cam.isOpen() << std::endl;
    cvWaitKey(100);
    left_cam.open(cam_infos[0], Master);
    cvWaitKey(100);
    if(stereo) right_cam.open(cam_infos[1], Monitor);
    cvWaitKey(100);
    //showCamInfos(cam_infos);
    std::cerr << "cam.isOpen = " << left_cam.isOpen() << std::endl;
    frame_size_t fs;
  
    fs.height = 480;
    fs.width = 752;
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

    if(stereo)
    {
        right_cam.setFrameSettings(fs, MODE_BAYER_RGGB, 8, false);
        right_cam.setAttrib(int_attrib::GainValue, 16);
        right_cam.setAttrib(enum_attrib::GammaToOn);
        right_cam.setAttrib(int_attrib::ExposureValue,300);
        right_cam.setAttrib(int_attrib::WhitebalValueBlue, 580);
        right_cam.setAttrib(int_attrib::WhitebalValueRed, 650);
    }

    dc1394_video_set_framerate(left_camera.dc_camera, DC1394_FRAMERATE_60);

    cvWaitKey(50);
    timeval ts, te;
    
    gettimeofday(&ts,NULL);
    std::cerr << "grabbing...";
                      left_cam.grab(SingleFrame,1);
                      //right_cam.grab(SingleFrame,1);
        std::cerr << "broadcasting...";

        left_camera.broadcastGrabCommand();                  //  left_cam.grab(SingleFrame,1);
        cvWaitKey(50);
        left_camera.broadcastGrabCommand();                  //  left_cam.grab(SingleFrame,1);
        cvWaitKey(50);


        for(int i = 0 ; i < 50 ; i++)
        {
            left_camera.broadcastGrabCommand();
            cvWaitKey(40);
        }


    for(int i = 0 ; i < 50 ; i++)
    {
        //timeval t1; gettimeofday(&t1,NULL); std::cerr << "before grab " << t1.tv_sec%1000 + t1.tv_usec/1000000.0 << std::endl;

        //timeval t2; gettimeofday(&t2,NULL); std::cerr << "before grab " << t2.tv_sec%1000 + t2.tv_usec/1000000.0 << std::endl;

        //timeval t3; gettimeofday(&t3,NULL); std::cerr << "before grab " << t3.tv_sec%1000 + t3.tv_usec/1000000.0 << std::endl;
        //std::cerr << "retrieving frame..." << std::endl;

        std::cerr << "broadcast...";

                          //  left_cam.grab(SingleFrame,1);

        //timeval t4; gettimeofday(&t4,NULL); std::cerr << "before grab " << t4.tv_sec%1000 + t4.tv_usec/1000000.0 << std::endl;
                std::cerr << "retr right...";
        if(stereo) right_cam.retrieveFrame(right_frame, 1000);
                  //      right_cam.grab(SingleFrame,1);
                std::cerr << "retr left...";

                left_cam.retrieveFrame(left_frame, 1000);



//cvWaitKey(230);

        //timeval t5; gettimeofday(&t5,NULL); std::cerr << "before grab " << t5.tv_sec%1000 + t5.tv_usec/1000000.0 << std::endl;

        cv::imshow("left",left_frame.convertToCvMat());
        if(stereo) cv::imshow("right",right_frame.convertToCvMat());

        //std::cerr << "image showed" << std::endl;
        cvWaitKey(50);

    }
    
    gettimeofday(&te,NULL);

    std::cerr << (1000000*te.tv_sec+te.tv_usec - 1000000*ts.tv_sec-ts.tv_usec)/1000000.0 << " seconds" << std::endl;

    cvWaitKey();

    //plotCameras(cam_infos);
/*
    try
    {
        //opens a specific camera
        tCamInfo info;
        info.display_name = "GE1900C";
        std::cout << "open camera " << info.display_name << "\n";
        camera.open2(info);
        
        //configure the camera to match the frame;
        camera.setFrameSettings(frame);

        //start capturing
        camera.grab(eContinuously,10);

        //display 100 frames
        int i=0;
        while(i<100)
        {
            if (camera.isFrameAvailable())
            {
                camera >> frame;
                //convert frame to openCV
                cv::Mat img(size.height,size.width, CV_8UC3, frame.getImagePtr());
                cv::imshow("image",img);
                i++;
            }
            cv::waitKey(2);
        }
    }
    catch(std::runtime_error e)
    { std::cout <<  e.what() << std::endl << std::endl;}
*/
    //close camera
    left_cam.close();
    if(stereo) right_cam.close();
   
    return 0;
}


