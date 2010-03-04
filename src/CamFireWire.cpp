/* 
 * File:   CamFireWire.cpp
 * Author: Christopher Gaudig, DFKI Bremen
 * 
 * Created on February 23, 2010, 4:57 PM
 */

#include <iostream>
#include <opencv/highgui.h>
#include "CamFireWire.h"

namespace camera
{

    CamFireWire::CamFireWire()
    {
        dc_camera = NULL;
        hdr_enabled = false;
        data_depth = 8;
    }

    CamFireWire::~CamFireWire()
    {
    }

    int CamFireWire::listCameras(std::vector<CamInfo>&cam_infos)const
    {
        dc1394_t *dc_device = dc1394_new();
        dc1394camera_list_t *list;
        dc1394error_t err;
    
        // get list of available cameras
        err = dc1394_camera_enumerate (dc_device, &list);

        std::cout << "enumeration: " << dc1394_error_get_string(err) << std::endl;

        //std::cout << list->ids->unit << err << std::endl;

        dc1394camera_t *dc_camera;

        if(list->num == 0)
            std::cout << "no cam found!" << std::endl;

        for(int i = 0 ; i < list->num ; i++)
        {
            dc_camera = dc1394_camera_new(dc_device, list->ids[i].guid);
            CamInfo cam_info;

            cam_info.unique_id = dc_camera->guid;
            cam_info.display_name = dc_camera->model;
            cam_info.interface_type = InterfaceFirewire;
            
            showCamInfo(cam_info);

            cam_infos.push_back(cam_info);
            dc1394_camera_free(dc_camera);
        }
        return list->num;
    }

    bool CamFireWire::open(const CamInfo &cam,const AccessMode mode)
    {
        dc1394_t *dc_device = dc1394_new();
        dc1394camera_list_t *list;
        dc1394error_t err;

        // get list of available cameras
        err = dc1394_camera_enumerate (dc_device, &list);
        dc_camera = dc1394_camera_new(dc_device, list->ids[0].guid);
        dc1394_camera_free_list(list);
        dc1394_reset_bus(dc_camera);
        std::cerr << "bus reset" << std::endl;
        dc1394_capture_setup(dc_camera,1,DC1394_CAPTURE_FLAGS_DEFAULT);

        act_grab_mode_= Stop;
    }
    
    bool CamFireWire::isOpen()const
    {
        if(dc_camera == NULL)
            return false;
        else
            return true;
    }

    bool CamFireWire::close()
    {
        if(dc_camera != NULL)
        {
            dc1394_capture_stop(dc_camera);
            dc1394_camera_free(dc_camera);
            dc_camera = NULL;
        }
    }


    bool CamFireWire::grab(const GrabMode mode, const int buffer_len)
    {
        //check if someone tries to change the grab mode
        //during grabbing
        if(act_grab_mode_ != Stop && mode != Stop)
        {
            if(act_grab_mode_ != mode)
                 throw std::runtime_error("Stop grabbing before switching the"
                                          " grab mode!");
            else
                return true;
        }

        switch(mode)
        {
            case Stop:
                dc1394_video_set_transmission(dc_camera, DC1394_OFF);
                dc1394_capture_stop(dc_camera);
                break;
            case SingleFrame:
                if(!dc_camera->one_shot_capable)
                    throw std::runtime_error("Camera is not one-shot capable!");
                
                dc1394_video_set_transmission(dc_camera, DC1394_OFF);
                dc1394_video_set_one_shot(dc_camera, DC1394_ON);
                break;
            case MultiFrame:
                dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
                break;
            case Continuously:
                dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
                break;
            default:
                throw std::runtime_error("Unknown grab mode!");
        }
        act_grab_mode_ = mode;
    }

    bool CamFireWire::retrieveFrame(Frame &frame,const int timeout)
    {

        
        std::cerr << "dequeuing..." << std::endl;
        //dc1394video_frame_t *tmp = frames.front();
        //frames.pop_front();

        std::cerr << "setting one shot ..." << std::endl;
        dc1394_video_set_one_shot(dc_camera, DC1394_ON);
        std::cerr << "one shot set" << std::endl;
        std::cerr << "dequeue = " << dc1394_capture_dequeue(dc_camera, DC1394_CAPTURE_POLICY_WAIT, &camFrame) << std::endl;
        std::cerr << "init frame..." << std::endl;
        Frame tmp;
        tmp.init(image_size_.width, image_size_.height, data_depth,
                camera::MODE_BAYER_RGGB, hdr_enabled);

        std::cerr << "setting frame's img..." << std::endl;
        tmp.setImage((const char *)camFrame->image, camFrame->size[0] * camFrame->size[1]);
        filter::Frame2RGGB::process(tmp,frame);

        std::cerr << "enqueuing..." << std::endl;
        //cvWaitKey(1);

        std::cerr << dc1394_capture_enqueue(dc_camera, camFrame);

        //frames.push_back(tmp);
    }

    bool CamFireWire::setFrameSettings(const frame_size_t size,
                                  const frame_mode_t mode,
                                  const  uint8_t color_depth,
                                  const bool resize_frames)
    {
        dc1394video_modes_t vmst;
        dc1394_video_get_supported_modes(dc_camera,&vmst);
        std::cerr << "supp vid modes " << vmst.modes[0] << std::endl;
        std::cerr << "supp vid modes " << vmst.modes[1] << std::endl;
        switch(mode)
        {
            case MODE_GRAYSCALE:
                std::cerr << "set mode " << dc1394_video_set_mode(dc_camera, vmst.modes[1])<< std::endl;
               
                break;
                 case MODE_RGB:
                std::cerr << "set mode " << dc1394_video_set_mode(dc_camera, vmst.modes[1])<< std::endl;

                break;
                 case MODE_BAYER_RGGB:
                std::cerr << "set mode " << dc1394_video_set_mode(dc_camera, vmst.modes[1])<< std::endl;

                break;
            default:
                throw std::runtime_error("Unknown frame mode!");
        }
        image_size_ = size;
        image_mode_ = mode;
        image_color_depth_ = color_depth;
    }

    bool CamFireWire::isFrameAvailable()
    {
        
    }


    bool CamFireWire::setAttrib(const int_attrib::CamAttrib attrib,const int value)
    {
        dc1394feature_t feature;
        switch(attrib)
        {
            case int_attrib::ExposureValue:
                feature = DC1394_FEATURE_SHUTTER;
                break;
            case int_attrib::GainValue:
                feature = DC1394_FEATURE_GAIN;
                break;
            case int_attrib::WhitebalValueRed:
                break;
            default:
                throw std::runtime_error("Unknown attribute!");
        };



        uint32_t t;
        dc1394_feature_get_value(dc_camera, feature , &t);
        std::cerr << "feature was " << t;
        dc1394_feature_set_value(dc_camera, feature , value);
        dc1394_feature_get_value(dc_camera, feature , &t);
        std::cerr << " and is now " << t << std::endl;

        return false;
    };


    bool CamFireWire::setAttrib(const enum_attrib::CamAttrib attrib)
    {
        dc1394feature_t feature;
        dc1394switch_t value;
        switch(attrib)
        {
            case enum_attrib::GammaToOn:
                feature = DC1394_FEATURE_GAMMA;
                value = DC1394_ON;
                break;
            case enum_attrib::GammaToOff:
                feature = DC1394_FEATURE_GAMMA;
                value = DC1394_OFF;
                break;
            default:
                throw std::runtime_error("Unknown attribute!");
        };



        
        dc1394_feature_set_power(dc_camera, feature , value);
     

        return false;
    };

}