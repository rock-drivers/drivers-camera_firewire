/*
 * File:   CamFireWire.cpp
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#include <iostream>
#include "camera_interface/CamInfoUtils.h"
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "CamFireWire.h"
#include <dc1394/dc1394.h>


using namespace base::samples::frame;

namespace cv
{

	static cv::Mat rotateImage(const Mat& source, double angle)
	{
    		Point2f src_center(source.cols/2.0, source.rows/2.0);
    		Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    		Mat dst;
    		warpAffine(source, dst, rot_mat, source.size());
    		return dst;
	}


}

namespace camera
{

CamFireWire::CamFireWire()
{
    // init parameters
    dc_camera = NULL;
    hdr_enabled = false;
    multi_shot_count = 0;
}

bool CamFireWire::cleanup()
{
  // find the cameras on the bus
    dc1394camera_list_t *list;
    dc1394_camera_enumerate (dc_device, &list);
    
    // use the first camera on the bus to issue a bus reset
    dc1394camera_t *tmp_camera = dc1394_camera_new(dc_device, list->ids[0].guid);
    uint32_t val;
    if ( dc1394_video_get_bandwidth_usage(tmp_camera, &val) == DC1394_SUCCESS &&
    dc1394_iso_release_bandwidth(tmp_camera, val) == DC1394_SUCCESS )
    std::cout << "Succesfully released " << val << " bytes of Bandwidth." << std::endl;
    /*if ( dc1394_video_get_iso_channel(tmp_camera, &val) == DC1394_SUCCESS &&
    dc1394_iso_release_channel(tmp_camera, val) == DC1394_SUCCESS )
    std::cout << "Succesfully released ISO channel #" << val << "." << std::endl;
*/
    //dc1394_camera_free(tmp_camera);
    std::cerr << "list->num = " << list->num << std::endl;
    if(list->num > 1)
    {
      tmp_camera = dc1394_camera_new(dc_device, list->ids[1].guid);
      dc1394_reset_bus(tmp_camera);
      if ( dc1394_video_get_bandwidth_usage(tmp_camera, &val) == DC1394_SUCCESS &&
      dc1394_iso_release_bandwidth(tmp_camera, val) == DC1394_SUCCESS )
      std::cout << "Successfully released " << val << " bytes of Bandwidth." << std::endl;
      if ( dc1394_video_get_iso_channel(tmp_camera, &val) == DC1394_SUCCESS &&
      dc1394_iso_release_channel(tmp_camera, val) == DC1394_SUCCESS )
      std::cout << "Successfully released ISO channel #" << val << "." << std::endl;
    }
  //  dc1394_camera_free(tmp_camera);
}

CamFireWire::~CamFireWire()
{
  dc1394_iso_release_all(dc_camera);
  dc1394_camera_free(dc_camera);
  dc1394_free(dc_device);
}

bool CamFireWire::setDevice(dc1394_t *dev)
{
    dc_device = dev;
}

// list all cameras on the firewire bus
int CamFireWire::listCameras(std::vector<CamInfo>&cam_infos)const
{

    dc1394camera_list_t *list;
    dc1394error_t err;

    // get list of available cameras
    err = dc1394_camera_enumerate (dc_device, &list);

    // temporary camera pointer (for getting the cam_infos)
    dc1394camera_t *tmp_camera;

    // if no camera is found on the bus
    if (list->num == 0)
        std::cout << "no cam found!" << std::endl;

    // get the cam_info for each camera on the bus
    for (int i = 0 ; i < list->num ; i++)
    {
	// get the i-th camera
        tmp_camera = dc1394_camera_new(dc_device, list->ids[i].guid);
        
	// get, set and display the corresponding cam_info
	CamInfo cam_info;
        cam_info.unique_id = tmp_camera->guid;
        cam_info.display_name = tmp_camera->model;
        cam_info.interface_type = InterfaceFirewire;
        showCamInfo(cam_info);

	// add the current cam_info to the vector cam_infos and release the temporary camera
        cam_infos.push_back(cam_info);
        dc1394_camera_free(tmp_camera);
    }

    // return the number of cameras found
    return list->num;
}

// open the camera specified by the CamInfo cam
bool CamFireWire::open(const CamInfo &cam,const AccessMode mode)
{
    dc1394camera_list_t *list;
    dc1394error_t err;

    // get list of available cameras
    err = dc1394_camera_enumerate (dc_device, &list);
    
    // get camera with the given uid and release the list
    dc_camera = dc1394_camera_new(dc_device, cam.unique_id);
    dc1394_camera_free_list(list);
    
    // set the current grab mode to "Stop"
    act_grab_mode_= Stop;
    
    if(0 != dc1394_capture_setup(dc_camera,8, DC1394_CAPTURE_FLAGS_DEFAULT)) return false;
    dc1394_video_set_transmission(dc_camera, DC1394_ON);
    dc1394_camera_set_broadcast(dc_camera, DC1394_FALSE);
    return true;
}

// returns true if the camera is open
bool CamFireWire::isOpen()const
{
    if (dc_camera == NULL)
        return false;
    else
        return true;
}

// stop capturing and get rid of the camera
bool CamFireWire::close()
{
    if (dc_camera != NULL)
    {
        dc1394_capture_stop(dc_camera);
        dc1394_camera_free(dc_camera);
	//dc1394_free(dc_device);
        dc_camera = NULL;
    }
}

// start grabbing using the given GrabMode mode and write frame into a buffer of lenght buffer_len
bool CamFireWire::grab(const GrabMode mode, const int buffer_len)
{
    //check if someone tries to change the grab mode
    //during grabbing
    if (act_grab_mode_ != Stop && mode != Stop)
    {
        if (act_grab_mode_ != mode)
            throw std::runtime_error("Stop grabbing before switching the"
                                     " grab mode!");
        else
	{
	    std::cerr << "returning true...";
            return true;
	}
    }




    // start grabbing using the given GrabMode mode
    switch (mode)
    {
    // stop transmitting and capturing frames
    case Stop:
        dc1394_video_set_transmission(dc_camera, DC1394_OFF);
        dc1394_capture_stop(dc_camera);
        break;
    
    // grab one frame only (one-shot mode)
    case SingleFrame:
        if (!dc_camera->one_shot_capable)
            throw std::runtime_error("Camera is not one-shot capable!");
      
	std::cerr << "setting one shot register...";
//	uint32_t t1;
//	std::cerr << dc1394_get_control_register(dc_camera,0x834, &t1) << " ";
//	std::cerr << "intena delay = " << t1 << std::endl;
	dc1394_camera_set_broadcast(dc_camera, DC1394_TRUE);
		//dc1394_set_control_register(dc_camera,0x614, 0x0);
	
	//dc1394_set_adv_control_register(dc_camera, 0x340, 0);
		//dc1394_video_set_one_shot(dc_camera, DC1394_ON); //for exposure test
		//dc1394_set_control_register(dc_camera, 0x614, 0x00000000); //stop free-run
	
		//dc1394_set_control_register(dc_camera, 0x830, 0x82000000); //set to trigger_mode_0
//dc1394_set_control_register(dc_camera,0x61c, 0x80000000); //this triggers the one-shot
		std::cerr << "now setting external trigger power to on" << std::endl;
		//dc1394_external_trigger_set_power(dc_camera, DC1394_ON);
		dc1394_feature_set_power(dc_camera, DC1394_FEATURE_TRIGGER, DC1394_ON); //for exposure test
		std::cerr << "done" << std::endl;
		dc1394_camera_set_broadcast(dc_camera, DC1394_FALSE);

	std::cerr << "done";
        break;
	
    // grab N frames (N previously defined by setting AcquisitionFrameCount
    case MultiFrame:
        dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
	if(multi_shot_count == 0)
	  throw std::runtime_error("Set AcquisitionFrameCount (multi-shot) to a positive number before calling grab()!");
	//dc1394_video_set_multi_shot(dc_camera, multi_shot_count, DC1394_ON);
	//start transmitting frames
	//dc1394_video_set_transmission(dc_camera,DC1394_ON);
	dc1394_set_control_register(dc_camera,0x614, 0);
	std::cerr << "multi-shot count = " << multi_shot_count << std::endl;
	dc1394_set_control_register(dc_camera,0x61c, 0x40000000 + multi_shot_count);

        break;
	
    // start grabbing frames continuously (using the framerate set beforehand)
    case Continuously:
        dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
        dc1394_feature_set_power(dc_camera, DC1394_FEATURE_TRIGGER, DC1394_OFF);
        dc1394_video_set_transmission(dc_camera,DC1394_ON);
        break;
    default:
        throw std::runtime_error("Unknown grab mode!");
    }
    act_grab_mode_ = mode;
    if (act_grab_mode_ = SingleFrame)
      act_grab_mode_ = Stop;
}

// retrieve a frame from the camera
bool CamFireWire::retrieveFrame(Frame &frame,const int timeout)
{
timeval tim;
             gettimeofday(&tim, NULL);
             double t1=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);



    bool color = true;
  
    // dequeue a frame using the dc1394-frame tmp_frame
    dc1394video_frame_t *tmp_frame=NULL;

std::cerr << "now trying to dequeue\n";
gettimeofday(&tim, NULL);
             double tbd=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);

while(DC1394_SUCCESS!=dc1394_capture_dequeue(dc_camera, DC1394_CAPTURE_POLICY_WAIT, &tmp_frame ))
{ std::cerr << "failed to dequeue\n";
	usleep(1000);}
	

//dc1394feature_t feature = DC1394_FEATURE_SHUTTER;
//uint value;
//dc1394_feature_get_value(dc_camera, feature , &value);

//std::cerr << "exposure is at " << value << "\n";

gettimeofday(&tim, NULL);
             double tad=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);
std::cerr << "the DEqueuing took " << tad-tbd << " ms\n";


std::cerr << "dequeuing succeeded!\n";

    std::cerr << "timestamp: " << tmp_frame->timestamp / 1000<< "\n";
    
    Frame upsidedown_frame;

    if(color)
    {
      // create a new DFKI frame and copy the data from tmp_frame
      Frame tmp;
      tmp.init(image_size_.width, image_size_.height, data_depth, MODE_BAYER_RGGB, hdr_enabled);
      tmp.setImage((const char *)tmp_frame->image, tmp_frame->size[0] * tmp_frame->size[1]);
      
      // convert the bayer pattern image to RGB
	  //camera::Helper::convertBayerToRGB24(tmp_frame->image, test.getImagePtr(), 640, 480, MODE_BAYER_RGGB);
      filter::Frame2RGGB::process(tmp,upsidedown_frame);
    }
    else
    {
      upsidedown_frame.init(image_size_.width, image_size_.height, data_depth, MODE_BAYER_RGGB, hdr_enabled);
      upsidedown_frame.setImage((const char *)tmp_frame->image, tmp_frame->size[0] * tmp_frame->size[1]);
    }
    
    frame.init(image_size_.width, image_size_.height, data_depth, MODE_RGB, hdr_enabled);
    
    cv::Mat correctImageMat = rotateImage(upsidedown_frame.convertToCvMat(),180);

    cvtColor(correctImageMat, correctImageMat, CV_BGR2RGB);

    frame.setImage((const char *)correctImageMat.data, tmp_frame->size[0] * tmp_frame->size[1]*3);

gettimeofday(&tim, NULL);
             double tbe=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);

    // re-queue the frame previously used for dequeueing
    dc1394_capture_enqueue(dc_camera,tmp_frame);
gettimeofday(&tim, NULL);
             double tae=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);
std::cerr << "the ENqueuing took " << tae-tbe << " ms\n";



gettimeofday(&tim, NULL);
             double t2=tim.tv_sec*1000.0+(tim.tv_usec/1000.0);

std::cerr << "the whole retrieval took " << t2-t1 << " ms\n";

}

// sets the frame size, mode, color depth and whether frames should be resized
bool CamFireWire::setFrameSettings(const frame_size_t size,
                                   const frame_mode_t mode,
                                   const  uint8_t color_depth,
                                   const bool resize_frames)
{
    dc1394video_modes_t vmst;
    dc1394_video_get_supported_modes(dc_camera,&vmst);

    switch (mode)
    {
    case MODE_BAYER_RGGB:
        std::cerr << "set mode " << dc1394_video_set_mode(dc_camera, DC1394_VIDEO_MODE_640x480_MONO8)<< std::endl;
	data_depth = 8;
        break;
    default:
        throw std::runtime_error("Unknown frame mode!");
    }
    image_size_ = size;
    image_mode_ = mode;
    image_color_depth_ = color_depth;
}

// (should) return true if the camera is ready for the next one-shot capture
bool CamFireWire::isReadyForOneShot()
{
    uint32_t one_shot;
    
    // get the camera's one-shot register
    dc1394_get_control_register(dc_camera,0x0061C,&one_shot);
    printf("one shot is %x ",one_shot);
    fflush(stdout);
    
    // the first bit is 1 when the cam is not ready and 0 when ready for a one-shot
    return (one_shot & 0x80000000UL) ? false : true;
}

// set integer-valued attributes
bool CamFireWire::setAttrib(const int_attrib::CamAttrib attrib,const int value)
{
    std::cerr << "setting int attrib...";
    // the feature (attribute) we want to set
    dc1394feature_t feature;
    
    switch (attrib)
    {
    // set the shutter time
    case int_attrib::ExposureValue:
        feature = DC1394_FEATURE_SHUTTER;
        dc1394_feature_set_value(dc_camera, feature , value);
        break;
	
    // set the gain
    case int_attrib::GainValue:
        feature = DC1394_FEATURE_GAIN;
        dc1394_feature_set_value(dc_camera, feature , value);
        break;
	
    // set the red white-balance value
    case int_attrib::WhitebalValueRed:
        uint32_t ub;
        uint32_t vr;
        dc1394_feature_whitebalance_get_value(dc_camera, &ub, &vr);
        dc1394_feature_whitebalance_set_value(dc_camera,ub,value);
        break;
	
    // set the blue white-balance value
    case int_attrib::WhitebalValueBlue:
        dc1394_feature_whitebalance_get_value(dc_camera, &ub, &vr);
        dc1394_feature_whitebalance_set_value(dc_camera,value,vr);
        break;
	
    // set the camera's isochronous transfer speed on the bus in Mbps
    case int_attrib::IsoSpeed:
        dc1394speed_t speed;
        switch (value)
        {
        case 400:
            speed = DC1394_ISO_SPEED_400;
            break;
        case 200:
            speed = DC1394_ISO_SPEED_200;
            break;
        case 100:
            speed = DC1394_ISO_SPEED_100;
            break;
        default:
            throw std::runtime_error("Unsupported Iso Speed!");
        };
	std::cerr << "now setting iso speed...";
        dc1394_video_set_iso_speed(dc_camera, speed);
	std::cerr << "done" << std::endl;
        break;
	
    // set the number of frames to capture in multi-shot mode
    case int_attrib::AcquisitionFrameCount:
        multi_shot_count = value;
        break;
	
    // the attribute given is not supported (yet)
    default:
        std::cerr << attrib;
        throw std::runtime_error("Unknown attribute!");
    };

    return false;
};

// set enum attributes
bool CamFireWire::setAttrib(const enum_attrib::CamAttrib attrib)
{
    // the feature (attribute) we want to set
    dc1394feature_t feature;
    
    // the (enum) value we want to set
    dc1394switch_t value;
    
    // the feature mode (auto/manual) we want to set
    dc1394feature_mode_t mode;
    
    switch (attrib)
    {
    // turn gamma on
    case enum_attrib::GammaToOn:
        feature = DC1394_FEATURE_GAMMA;
        value = DC1394_ON;
	// set the desired attribute/feature value
        dc1394_feature_set_power(dc_camera, feature , value);
        break;
	
    // turn gamma off
    case enum_attrib::GammaToOff:
        feature = DC1394_FEATURE_GAMMA;
        value = DC1394_OFF;
	// set the desired attribute/feature value
        dc1394_feature_set_power(dc_camera, feature , value);
        break;
	
    // turn auto exposure on
    case enum_attrib::ExposureModeToAuto:
        feature = DC1394_FEATURE_SHUTTER;
        mode = DC1394_FEATURE_MODE_AUTO;
	dc1394_feature_set_mode(dc_camera, feature, mode);
        break;

    // turn auto exposure off
    case enum_attrib::ExposureModeToManual:
        feature = DC1394_FEATURE_SHUTTER;
        mode = DC1394_FEATURE_MODE_MANUAL;
	dc1394_feature_set_mode(dc_camera, feature, mode);
        break;

    // turn auto gain on
    case enum_attrib::GainModeToAuto:
        feature = DC1394_FEATURE_GAIN;
        mode = DC1394_FEATURE_MODE_AUTO;
	dc1394_feature_set_mode(dc_camera, feature, mode);
        break;

    // turn auto gain off
    case enum_attrib::GainModeToManual:
        feature = DC1394_FEATURE_GAIN;
        mode = DC1394_FEATURE_MODE_MANUAL;
	dc1394_feature_set_mode(dc_camera, feature, mode);
        break;

    // attribute unknown or not supported (yet)
    default:
        throw std::runtime_error("Unknown attribute!");
    };

    

    return false;
};

// set double-valued attributes
bool CamFireWire::setAttrib(const double_attrib::CamAttrib attrib, const double value)
{
    // the feature/attribute we want to set
    dc1394feature_t feature;
    
    // the desired video framerate
    dc1394framerate_t framerate;
    
    switch (attrib)
    {
    // set the framerate
    case double_attrib::FrameRate:
        if (value==30)
            framerate = DC1394_FRAMERATE_30;
        else if (value==60)
            framerate = DC1394_FRAMERATE_60;
        else if (value==15)
            framerate = DC1394_FRAMERATE_15;
	else if (value==8)
	    framerate = DC1394_FRAMERATE_7_5;
        else if (value==4)
	    framerate = DC1394_FRAMERATE_3_75;
        else
            throw std::runtime_error("Framerate not supported! Use 15, 30 or 60 fps.");

	// the actual framerate-setting
        dc1394_video_set_framerate(dc_camera, framerate);
        break;
    
    // attribute unknown or not supported (yet)
    default:
        throw std::runtime_error("Unknown attribute!");
    };
    
    return false;
};

// should return true whenever a frame is available
bool CamFireWire::isFrameAvailable()
{
  dc1394_capture_get_fileno(dc_camera);
  
}

bool CamFireWire::clearBuffer()
{
    dc1394video_frame_t *tmp;
    //while(dc1394_capture_dequeue(dc_camera, DC1394_CAPTURE_POLICY_POLL, &tmp));
    
    bool endFound = false;
    dc1394error_t err;
    
    while (!endFound) {
       err = dc1394_capture_dequeue(dc_camera,DC1394_CAPTURE_POLICY_POLL, &tmp);
       if (tmp && err==DC1394_SUCCESS)
       {
          dc1394_capture_enqueue(dc_camera, tmp);
       } else
       {
          endFound=true;
       }
}
}

}


