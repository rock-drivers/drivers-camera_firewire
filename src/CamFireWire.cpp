/*
 * File:   CamFireWire.cpp
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#include <iostream>
#include "camera_interface/CamInfoUtils.h"
#include "CamFireWire.h"
#include <dc1394/dc1394.h>
#include <dc1394/vendor/avt.h>
#include <base-logging/Logging.hpp>
#include <dc1394/control.h>
#include <unistd.h>


using namespace base::samples::frame;

namespace camera
{

CamFireWire::CamFireWire()
{
    // init parameters
    dc_camera = NULL;
    hdr_enabled = false;
    multi_shot_count = 0;
    data_depth = 0;
    frame_mode = MODE_UNDEFINED;
}

bool CamFireWire::cleanup()
{
    // find the cameras on the bus
    dc1394camera_list_t *list;
    dc1394_camera_enumerate (dc_device, &list);
    
    // use the first camera on the bus to issue a bus reset
    dc1394camera_t *tmp_camera = dc1394_camera_new(dc_device, list->ids[0].guid);
    uint32_t val;
    dc1394_video_get_bandwidth_usage(tmp_camera, &val);
    dc1394_iso_release_bandwidth(tmp_camera, val);

    if(list->num > 1)
    {
        tmp_camera = dc1394_camera_new(dc_device, list->ids[1].guid);
        dc1394_reset_bus(tmp_camera);
        dc1394_video_get_iso_channel(tmp_camera, &val);
        dc1394_iso_release_channel(tmp_camera, val);
    }
    return true;
}

CamFireWire::~CamFireWire()
{
    if (dc_camera)
    {
	dc1394_iso_release_all(dc_camera);
	dc1394_camera_free(dc_camera);
    }
    if (dc_device)
	dc1394_free(dc_device);
}

bool CamFireWire::setDevice(dc1394_t *dev)
{
    if(!dev)
	return false;
    
    dc_device = dev;
    return true;
}

// list all cameras on the firewire bus
int CamFireWire::listCameras(std::vector<CamInfo>&cam_infos)const
{
    dc1394camera_list_t *list;
    dc1394error_t err;

    if (!dc_device)
	return -1;

    // get list of available cameras
    if(checkHandleError(dc1394_camera_enumerate (dc_device, &list)))
	return -1;

    // temporary camera pointer (for getting the cam_infos)
    dc1394camera_t *tmp_camera;

    // if no camera is found on the bus
    if (list->num == 0)
    {
        std::cout << "no cam found!" << std::endl;
	return -1;
    }

    // get the cam_info for each camera on the bus
    for (int i = 0 ; i < list->num ; i++)
    {
	// get the i-th camera
        tmp_camera = dc1394_camera_new(dc_device, list->ids[i].guid);
	if(!tmp_camera)
	{
	    std::cout << "failed to open camera for listing!" << std::endl;
	    dc1394_camera_free_list(list);
	    return -1;
	}
        
	// get and set the corresponding cam_info
	CamInfo cam_info;
        cam_info.unique_id = tmp_camera->guid;
        cam_info.display_name = tmp_camera->model;
        cam_info.interface_type = InterfaceFirewire;

	// add the current cam_info to the vector cam_infos and release the temporary camera
        cam_infos.push_back(cam_info);
        dc1394_camera_free(tmp_camera);
    }
    
    int numCams = list->num;
    
    dc1394_camera_free_list(list);

    // return the number of cameras found
    return numCams;
}

// open the camera specified by the CamInfo cam
bool CamFireWire::open(const CamInfo &cam,const AccessMode mode)
{
    if (!dc_device)
	return false;

    // get camera with the given uid and release the list
    dc_camera = dc1394_camera_new(dc_device, cam.unique_id);
    if(!dc_camera)
	return false;
    
    // set the current grab mode to "Stop"
    act_grab_mode_= Stop;

    if(checkHandleError(dc1394_camera_set_broadcast(dc_camera, DC1394_FALSE)))
    {
	if(dc_camera)
	{
	    dc1394_camera_free(dc_camera);
	    dc_camera = 0;
	    return false;
	}
    }

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
        dc_camera = NULL;
    }
    return true;
}

// start grabbing using the given GrabMode mode and write frame into a buffer of lenght buffer_len
bool CamFireWire::grab(const GrabMode mode, const int buffer_len)
{
    if (!dc_camera)
	return false;

    //check if someone tries to change the grab mode during grabbing
    if (act_grab_mode_ != Stop && mode != Stop)
    {
        if (act_grab_mode_ != mode)
            throw std::runtime_error("Stop grabbing before switching the grab mode!");
        else
            return true;
    }

    dc1394error_t err = DC1394_SUCCESS;
    // start grabbing using the given GrabMode mode
    switch (mode)
    {
    // stop transmitting and capturing frames
    case Stop:
        err = dc1394_video_set_transmission(dc_camera, DC1394_OFF);
        if(checkHandleError(err))
            return false;
        
        err = dc1394_capture_stop(dc_camera);
        if(checkHandleError(err))
            return false;
        break;

    // grab one frame only (one-shot mode)
    case SingleFrame:
        if (!dc_camera->one_shot_capable)
            throw std::runtime_error("Camera is not one-shot capable!");

        err = dc1394_capture_setup(dc_camera, buffer_len, DC1394_CAPTURE_FLAGS_DEFAULT);
        if(checkHandleError(err))
            return false;
        
        err = dc1394_video_set_transmission(dc_camera,DC1394_ON);
        break;

    // grab N frames (N previously defined by setting AcquisitionFrameCount
    case MultiFrame:
        if (!dc_camera->multi_shot_capable)
            throw std::runtime_error("Camera is not multi-shot capable!");
        
        err = dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
        if(checkHandleError(err))
            return false;
        
        if(multi_shot_count == 0)
          throw std::runtime_error("Set AcquisitionFrameCount (multi-shot) to a positive number before calling grab()!");
        dc1394_set_control_register(dc_camera,0x614, 0);
        dc1394_set_control_register(dc_camera,0x61c, 0x40000000 + multi_shot_count);
        break;
	
    // start grabbing frames continuously (using the framerate set beforehand)
    case Continuously:
        err = dc1394_capture_setup(dc_camera,buffer_len,DC1394_CAPTURE_FLAGS_DEFAULT);
        if(checkHandleError(err))
            return false;
	
        err = dc1394_video_set_transmission(dc_camera,DC1394_ON);
        if(checkHandleError(err))
            return false;
        break;
    default:
        throw std::runtime_error("Unknown grab mode!");
    }
    
    // check if whitebalance is in one push auto mode
    dc1394feature_mode_t feature_mode;
    dc1394_feature_get_mode(dc_camera, DC1394_FEATURE_WHITE_BALANCE, &feature_mode);
    if((mode == SingleFrame || mode == Continuously) && feature_mode == DC1394_FEATURE_MODE_ONE_PUSH_AUTO)
    {
        // wait for two seconds and turn transmission on again
        sleep(2);
        err = dc1394_video_set_transmission(dc_camera,DC1394_ON);
    }
    
    if(0 != err)
    {
        std::cerr << "prepare for grab failed, libdc1394 error: " <<
            dc1394_error_get_string(err) << std::endl;
        return false;
    }
    act_grab_mode_ = mode;
    if (act_grab_mode_ == SingleFrame)
      act_grab_mode_ = Stop;
    
    return true;
}

// retrieve a frame from the camera
bool CamFireWire::retrieveFrame(Frame &frame,const int timeout)
{
    if (!dc_camera)
	return false;
  
    // dequeue a frame using the dc1394-frame tmp_frame
    dc1394video_frame_t *tmp_frame=NULL;

    int ret = dc1394_capture_dequeue(dc_camera, DC1394_CAPTURE_POLICY_POLL, &tmp_frame);
    
    frame.init(image_size_.width, image_size_.height, data_depth, frame_mode);
    frame.setHDR(hdr_enabled);

    if(ret == DC1394_SUCCESS)
    {
        if(tmp_frame == NULL)
        {
            throw std::runtime_error("Recieved frame is empty.");
        }
        else
        {
            frame.setImage((const char *)tmp_frame->image, tmp_frame->image_bytes);
            // set the frame's timestamps (secs and usecs)
            frame.time = base::Time::fromMicroseconds(tmp_frame->timestamp);
            frame.setStatus(STATUS_VALID);
        }
    }
    else
    {
        frame.setStatus(STATUS_INVALID);
        
        // re-queue the frame previously used for dequeueing
        dc1394_capture_enqueue(dc_camera, tmp_frame);

        return false;
    }

    // re-queue the frame previously used for dequeueing
    dc1394_capture_enqueue(dc_camera, tmp_frame);

    return true;
}

// sets the frame size, mode, color depth and whether frames should be resized
bool CamFireWire::setFrameSettings(const frame_size_t size,
                                   const frame_mode_t mode,
                                   const  uint8_t color_depth,
                                   const bool resize_frames)
{
    if (!dc_camera)
	return false;
    
    if (mode == MODE_BAYER)
        frame_mode = MODE_BAYER_BGGR;
    else
        frame_mode = mode;

    int channel_count = Frame::getChannelCount(mode);
    if (channel_count <= 0)
        throw std::runtime_error("Unknown frame mode!");
    data_depth = (color_depth * 8) / channel_count;
    
    dc1394video_mode_t selected_mode = DC1394_VIDEO_MODE_640x480_MONO8;
    
    switch (mode)
    {
    case MODE_BAYER:
    case MODE_BAYER_BGGR:
    case MODE_BAYER_RGGB:
    case MODE_BAYER_GRBG:
    case MODE_BAYER_GBRG:
        if (isVideoModeSupported(DC1394_VIDEO_MODE_FORMAT7_3) && isVideo7RAWModeSupported(data_depth))
        {
            selected_mode = DC1394_VIDEO_MODE_FORMAT7_3;
            uint32_t max_height = 0;
            uint32_t max_width = 0;
            dc1394_format7_get_max_image_size(dc_camera, selected_mode, &max_width, &max_height);
            if (size.height <= max_height && size.width <= max_width)
            {
                dc1394_format7_set_image_size(dc_camera, selected_mode, size.width, size.height);
                dc1394_format7_set_image_position(dc_camera, selected_mode, 
                                            (max_width - (uint32_t)size.width) * 0.5, 
                                            (max_height - (uint32_t)size.height) * 0.5);
                    
                dc1394color_coding_t depth;
                switch(data_depth)
                {
                    case 8:
                        depth = DC1394_COLOR_CODING_RAW8;
                        break;
                    case 16:
                        depth = DC1394_COLOR_CODING_RAW16;
                        break;
                    case 24:
                        depth = DC1394_COLOR_CODING_RGB8;
                        break;
                    default:
                        throw std::runtime_error("Data depth is not supported!");
                }
                dc1394_format7_set_color_coding(dc_camera, selected_mode, depth);
            }
            else
            {
                throw std::runtime_error("Resolution is not supported!");
            }
            break;
        }
        else if (isVideoModeSupported(DC1394_VIDEO_MODE_FORMAT7_0) && isVideo7RAWModeSupported(data_depth))
        {
            selected_mode = DC1394_VIDEO_MODE_FORMAT7_0;
            uint32_t max_height = 0;
            uint32_t max_width = 0;
            dc1394_format7_get_max_image_size(dc_camera, selected_mode, &max_width, &max_height);
            if (size.height <= max_height && size.width <= max_width)
            {
                dc1394_format7_set_image_size(dc_camera, selected_mode, size.width, size.height);
                dc1394_format7_set_image_position(dc_camera, selected_mode, 
                                            (max_width - (uint32_t)size.width) * 0.5, 
                                            (max_height - (uint32_t)size.height) * 0.5);
                dc1394_format7_set_color_coding(dc_camera, selected_mode, 
                                             data_depth == 16 ? DC1394_COLOR_CODING_RAW16 : DC1394_COLOR_CODING_RAW8);
            }
            else
            {
                throw std::runtime_error("Resolution is not supported!");
            }
            break;
        }
        //if format 7 is not supported use MONO as RAW mode instead
    case MODE_GRAYSCALE:
        if(size.height <= 480) 
        {
            if (data_depth == 8)
                selected_mode = DC1394_VIDEO_MODE_640x480_MONO8;
            else if (data_depth == 16)
                selected_mode = DC1394_VIDEO_MODE_640x480_MONO16;
        }
        else if(size.height <= 600)
        {
            if (data_depth == 8)
                selected_mode = DC1394_VIDEO_MODE_800x600_MONO8;
            else if (data_depth == 16)
                selected_mode = DC1394_VIDEO_MODE_800x600_MONO16;
        }
        else if(size.height <= 768)
        {
            if (data_depth == 8)
                selected_mode = DC1394_VIDEO_MODE_1024x768_MONO8;
            else if (data_depth == 16)
                selected_mode = DC1394_VIDEO_MODE_1024x768_MONO16;
        }
         else if(size.height <= 960)
        {
            if (data_depth == 8)
                selected_mode = DC1394_VIDEO_MODE_1280x960_MONO8;
            else if (data_depth == 16)
                selected_mode = DC1394_VIDEO_MODE_1280x960_MONO16;
        }
        else
        {
            if (data_depth == 8)
                selected_mode = DC1394_VIDEO_MODE_1600x1200_MONO8;
            else if (data_depth == 16)
                selected_mode = DC1394_VIDEO_MODE_1600x1200_MONO16;
        }
        break;
    case MODE_RGB:
        if(data_depth == 8)
        {
            if(size.height <= 480) 
            {
                selected_mode = DC1394_VIDEO_MODE_640x480_RGB8;
            }
            else if(size.height <= 600)
            {
                selected_mode = DC1394_VIDEO_MODE_800x600_RGB8;
            }
            else if(size.height <= 768)
            {
                selected_mode = DC1394_VIDEO_MODE_1024x768_RGB8;
            }
            else if(size.height <= 960)
            {
                selected_mode = DC1394_VIDEO_MODE_1280x960_RGB8;
            }
            else
            {
                selected_mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
            }
        }
        else
        {
            throw std::runtime_error("Only 8 bit color depth is supported for mod RGB!");
        }
        break;
    case MODE_UYVY:
        if(size.height <= 120) 
        {
            selected_mode = DC1394_VIDEO_MODE_160x120_YUV444;
        }
        else if(size.height <= 240) 
        {
            selected_mode = DC1394_VIDEO_MODE_320x240_YUV422;
        }
        else if(size.height <= 480) 
        {
            selected_mode = DC1394_VIDEO_MODE_640x480_YUV422;
        }
        else if(size.height <= 600) 
        {
            selected_mode = DC1394_VIDEO_MODE_800x600_YUV422;
        }
        else if(size.height <= 768) 
        {
            selected_mode = DC1394_VIDEO_MODE_1024x768_YUV422;
        }
        else if(size.height <= 960) 
        {
            selected_mode = DC1394_VIDEO_MODE_1280x960_YUV422;
        }
        else
        {
            selected_mode = DC1394_VIDEO_MODE_1600x1200_YUV422;
        }
        break;
    default:
        throw std::runtime_error("Unknown frame mode!");
    }
    
    
    // check if video mode is supported
    if(isVideoModeSupported(selected_mode))
        dc1394_video_set_mode(dc_camera, selected_mode);
    else
        throw std::runtime_error("Video mode is not supported!");
    
    image_size_ = size;
    image_mode_ = mode;
    image_color_depth_ = color_depth;
    return true;
}

// (should) return true if the camera is ready for the next one-shot capture
bool CamFireWire::isReadyForOneShot()
{
    if (!dc_camera)
	return false;

    uint32_t one_shot;
    
    // get the camera's one-shot register
    dc1394_get_control_register(dc_camera,0x0061C,&one_shot);
    printf("one shot is %x ",one_shot);
    fflush(stdout);
    
    // the first bit is 1 when the cam is not ready and 0 when ready for a one-shot
    return (one_shot & 0x80000000UL) ? false : true;
}

// check if video mode is supported
bool CamFireWire::isVideoModeSupported(const dc1394video_mode_t mode)
{
    if (!dc_camera)
    return false;
    
    dc1394video_modes_t vmst;
    if(checkHandleError(dc1394_video_get_supported_modes(dc_camera,&vmst)))
	return false;
    
       
    bool mode_supported = false;
    for(int i = 0; i < vmst.num; i++)
    {
        if (vmst.modes[i] == mode)
        {
            mode_supported = true;
            break;
        }
    }
    
    return mode_supported;
}

bool CamFireWire::isVideo7RAWModeSupported(int depth)
{
    dc1394color_codings_t codings;
    if(checkHandleError(dc1394_format7_get_color_codings(dc_camera, DC1394_VIDEO_MODE_FORMAT7_0, &codings)))
	return false;
    
    dc1394color_coding_t coding = DC1394_COLOR_CODING_RAW8;
    if (depth == 16)
        coding = DC1394_COLOR_CODING_RAW16;
    
    bool mode_supported = false;
    for(int i = 0; i < codings.num; i++)
    {
        if(codings.codings[i] == coding)
        {
            mode_supported = true;
            break;
        }
    }
    
    return mode_supported;
}

// check if integer-valued attributes are available
bool CamFireWire::isAttribAvail(const int_attrib::CamAttrib attrib)
{
    if (!dc_camera)
	return false;
    
    dc1394bool_t isPresent = DC1394_FALSE;
    dc1394error_t ret = DC1394_SUCCESS;
    
    switch (attrib)
    {
	case int_attrib::ExposureValue:
	    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_EXPOSURE, &isPresent);
	    break;
	case int_attrib::GainValue:
	    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_GAIN, &isPresent);
	    break;
	case int_attrib::ShutterValue:
	    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_SHUTTER, &isPresent);
	    break;
	case int_attrib::WhitebalValueRed:
	    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_WHITE_BALANCE, &isPresent);
	    break;
	case int_attrib::WhitebalValueBlue:
	    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_WHITE_BALANCE, &isPresent);
	    break;
	case int_attrib::IsoSpeed:
	    return true;
	case int_attrib::AcquisitionFrameCount:
	    return true;
	case int_attrib::HDRValue:
	    dc1394_avt_adv_feature_info_t avt_info;
	    ret = dc1394_avt_get_advanced_feature_inquiry(dc_camera, &avt_info);
	    isPresent = avt_info.HDR_Mode;
	    break;
	default:
	    return false;
    };
    if(checkHandleError(ret))
	return false;

    return (bool)isPresent;
}

// check if double-valued attributes are available
bool CamFireWire::isAttribAvail(const double_attrib::CamAttrib attrib)
{
    if (!dc_camera)
	return false;

    switch (attrib)
    {
    case double_attrib::FrameRate:
        return true;
    default:
        return false;
    };
    return false;
}

// check if string attributes are available
bool CamFireWire::isAttribAvail(const str_attrib::CamAttrib attrib)
{
    return false;
}

bool CamFireWire::checkForTriggerSource(const dc1394trigger_source_t source)
{
    dc1394error_t ret = DC1394_SUCCESS;
    dc1394trigger_sources_t sources;
    
    //get list of supported sources from camera
    ret = dc1394_external_trigger_get_supported_sources(dc_camera, &sources);
    if(checkHandleError(ret))
	return false;
    
    //check if our source is in the list
    for(int i = 0; i < sources.num; i++)
    {
	if(sources.sources[i] == source)
	    return true;
    }
    
    /* fix an error: check if camera has the external trigger feature
     * in this case the camera does provide the trigger source 0 and a trigger source software,
     * even if it tells it doesn't.
     */
    dc1394bool_t present;
    ret = dc1394_feature_is_present(dc_camera, DC1394_FEATURE_TRIGGER, &present);
    if(checkHandleError(ret))
        return false;
    if((source == DC1394_TRIGGER_SOURCE_0 || source == DC1394_TRIGGER_SOURCE_SOFTWARE) && present == DC1394_TRUE)
        return true;
    
    //we don't support 'source'
    return false;
}

bool CamFireWire::checkHandleError(dc1394error_t error) const
{
    if(error != DC1394_SUCCESS)
    {
	const char *errorString = dc1394_error_get_string(error);
	LOG_ERROR_S << errorString << std::endl;
	return true;
    }
    return false;
}

// check if enum attributes are available
bool CamFireWire::isAttribAvail(const enum_attrib::CamAttrib attrib)
{
    if (!dc_camera)
	return false;
    
    dc1394feature_t feature;

    
    dc1394error_t ret = DC1394_SUCCESS;
    
    switch (attrib)
    {
	case enum_attrib::FrameStartTriggerModeToSyncIn1:
	    return checkForTriggerSource(DC1394_TRIGGER_SOURCE_0);
	break;
	case enum_attrib::FrameStartTriggerModeToSyncIn2:
	    return checkForTriggerSource(DC1394_TRIGGER_SOURCE_1);
	break;
	case enum_attrib::FrameStartTriggerModeToSyncIn3:
	    return checkForTriggerSource(DC1394_TRIGGER_SOURCE_2);
	break;
	case enum_attrib::FrameStartTriggerModeToSyncIn4:
	    return checkForTriggerSource(DC1394_TRIGGER_SOURCE_3);
	break;
    case enum_attrib::FrameStartTriggerModeToFreerun:
    case enum_attrib::FrameStartTriggerModeToFixedRate:
	case enum_attrib::FrameStartTriggerModeToSoftware:
	    return checkForTriggerSource(DC1394_TRIGGER_SOURCE_SOFTWARE);
	break;
	case enum_attrib::FrameStartTriggerEventToEdgeRising:
        return true;
        break;
    case enum_attrib::FrameStartTriggerEventToEdgeFalling:
        return true;
        break;
    case enum_attrib::GammaToOn:
        feature = DC1394_FEATURE_GAMMA;
        break;
    case enum_attrib::GammaToOff:
        feature = DC1394_FEATURE_GAMMA;
        break;
    case enum_attrib::ExposureModeToAuto:
        feature = DC1394_FEATURE_EXPOSURE;
        break;
    case enum_attrib::ExposureModeToManual:
        feature = DC1394_FEATURE_EXPOSURE;
        break;
    case enum_attrib::ExposureModeToAutoOnce:
        feature = DC1394_FEATURE_EXPOSURE;
        break;
    case enum_attrib::GainModeToAuto:
        feature = DC1394_FEATURE_GAIN;
        break;
    case enum_attrib::GainModeToManual:
        feature = DC1394_FEATURE_GAIN;
        break;
	case enum_attrib::ShutterModeToAuto:
        feature = DC1394_FEATURE_SHUTTER;
        break;
    case enum_attrib::ShutterModeToManual:
        feature = DC1394_FEATURE_SHUTTER;
        break;
    case enum_attrib::WhitebalModeToAuto:
        feature = DC1394_FEATURE_WHITE_BALANCE;
        break;
    case enum_attrib::WhitebalModeToAutoOnce:
        feature = DC1394_FEATURE_WHITE_BALANCE;
        break;
    case enum_attrib::WhitebalModeToManual:
        feature = DC1394_FEATURE_WHITE_BALANCE;
        break;
    default:
        return false;
    };
    
    dc1394bool_t isPresent = DC1394_FALSE;
    ret = dc1394_feature_is_present(dc_camera, feature, &isPresent);
    if(checkHandleError(ret))
	return false;
    
    if (isPresent == DC1394_TRUE)
        return true;
    else
        return false;
}

// set integer-valued attributes
bool CamFireWire::setAttrib(const int_attrib::CamAttrib attrib,const int value)
{
    if (!dc_camera)
	return false;

    // the feature (attribute) we want to set
    dc1394feature_t feature;
   
    dc1394error_t ret = DC1394_SUCCESS;
	
	uint32_t current_value;

    switch (attrib)
    {
    // set the shutter time
    case int_attrib::ExposureValue:
	feature = DC1394_FEATURE_EXPOSURE;
	// For unknown reasons, when setting a value, get_value must be 
	// called first otherwise set_value has no effect
	dc1394_feature_get_value(dc_camera, feature , &current_value);
	ret = dc1394_feature_set_value(dc_camera, feature , value);
	break;
	
    // set the gain
    case int_attrib::GainValue:
	feature = DC1394_FEATURE_GAIN;
	// For unknown reasons, when setting a value, get_value must be 
	// called first otherwise set_value has no effect
	dc1394_feature_get_value(dc_camera, feature , &current_value);
	ret = dc1394_feature_set_value(dc_camera, feature , value);
	break;
	
	// set the shutter
    case int_attrib::ShutterValue:
	feature = DC1394_FEATURE_SHUTTER;
	// For unknown reasons, when setting a value, get_value must be 
	// called first otherwise set_value has no effect
	dc1394_feature_get_value(dc_camera, feature , &current_value);
	ret = dc1394_feature_set_value(dc_camera, feature , value);
	break;
        
    // set the red white-balance value
    case int_attrib::WhitebalValueRed:
        uint32_t ub;
        uint32_t vr;
        ret = dc1394_feature_whitebalance_get_value(dc_camera, &ub, &vr);
	if(checkHandleError(ret))
	    return false;
        ret = dc1394_feature_whitebalance_set_value(dc_camera,ub,value);
        break;
	
    // set the blue white-balance value
    case int_attrib::WhitebalValueBlue:
        ret = dc1394_feature_whitebalance_get_value(dc_camera, &ub, &vr);
        if(checkHandleError(ret))
	    return false;
        ret = dc1394_feature_whitebalance_set_value(dc_camera,value,vr);
        break;
	
    // set the camera's isochronous transfer speed on the bus in Mbps
    case int_attrib::IsoSpeed:
        dc1394speed_t speed;
        switch (value)
        {
        case 800:
            speed = DC1394_ISO_SPEED_800;
            break;
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
        ret = dc1394_video_set_iso_speed(dc_camera, speed);
        break;
	case int_attrib::OperationMode:
	    dc1394operation_mode_t mode;
	    switch(value)
	    {
	    case 'B':
		mode = DC1394_OPERATION_MODE_1394B;
		break;
        default:
            mode = DC1394_OPERATION_MODE_LEGACY;
	    }
	    ret = dc1394_video_set_operation_mode(dc_camera, mode);
	    break;
    // set the number of frames to capture in multi-shot mode
    case int_attrib::AcquisitionFrameCount:
        multi_shot_count = value;
        break;
        
    case int_attrib::HDRValue:
    {
        uint32_t hdr_value = (uint32_t)value;
        
        // unpack voltage values
        uint32_t kneepoint1_voltage1 = hdr_value & 0xFFUL;
        uint32_t kneepoint1_voltage2 = (hdr_value >> 8) & 0xFFUL;
        uint32_t kneepoint2_voltage1 = (hdr_value >> 16) & 0xFFUL;
        uint32_t kneepoint2_voltage2 = (hdr_value >> 24) & 0xFFUL;
        
        // get actual settings
        uint32_t points_nb, kneepoint1, kneepoint2, kneepoint3;
        dc1394bool_t hdr;
        ret = dc1394_avt_get_multiple_slope(dc_camera, &hdr, &points_nb, &kneepoint1, &kneepoint2, &kneepoint3);
        if(checkHandleError(ret))
	    return false;
        
        if(kneepoint1_voltage1 > 0 || kneepoint1_voltage2 > 0)
        {
            // activate hdr
            uint32_t kneepoint1_time = 1;
            kneepoint1 = (kneepoint1 & 0x00FFFFFFUL) | ((kneepoint1_voltage1 & 0xFFUL) << 24);
            kneepoint1 = (kneepoint1 & 0xFF00FFFFUL) | ((kneepoint1_voltage2 & 0xFFUL) << 16);
            kneepoint1 = (kneepoint1 & 0xFFFF0000UL) | (kneepoint1_time & 0xFFFFUL);
            
            if(kneepoint2_voltage1 > 0 || kneepoint2_voltage2 > 0)
            {
                // use two kneepoints
                points_nb = 2;
                uint32_t kneepoint2_time = 1;
                kneepoint2 = (kneepoint2 & 0x00FFFFFFUL) | ((kneepoint2_voltage1 & 0xFFUL) << 24);
                kneepoint2 = (kneepoint2 & 0xFF00FFFFUL) | ((kneepoint2_voltage2 & 0xFFUL) << 16);
                kneepoint2 = (kneepoint2 & 0xFFFF0000UL) | (kneepoint2_time & 0xFFFFUL);
            }
            else 
            {
                // use one kneepoint
                points_nb = 1;
                kneepoint2 = 0;
            }
            kneepoint3 = 0;
            ret = dc1394_avt_set_multiple_slope(dc_camera, DC1394_TRUE, points_nb, kneepoint1, kneepoint2, kneepoint3);
            if(checkHandleError(ret))
		return false;
	    hdr_enabled = true;
        }
        else
        {
            // deactivate hdr
            ret = dc1394_avt_set_multiple_slope(dc_camera, DC1394_FALSE, points_nb, kneepoint1, kneepoint2, kneepoint3); 
            if(checkHandleError(ret))
		return false;
	    hdr_enabled = false;
        }
        break;
    }
    // the attribute given is not supported (yet)
    default:
        throw std::runtime_error("Unknown attribute!");
    };
    
    if(checkHandleError(ret))
	return false;
            
    return true;
}

int CamFireWire::getAttrib(const int_attrib::CamAttrib attrib)
{
    if (!dc_camera)
	return false;

    // the feature (attribute) we want to get
    dc1394feature_t feature;

    // the value we want to get from the cam
    uint32_t value;

    switch(attrib)
    {
        // get the current exposure value from the cam
        case int_attrib::ExposureValue:
            feature = DC1394_FEATURE_EXPOSURE;
            dc1394_feature_get_value(dc_camera, feature , &value);
            return (int)value;
            break;

        // attribute unknown or not supported (yet)
        default:
            throw std::runtime_error("Unknown attribute!");
    }
}

// get double attributes
double CamFireWire::getAttrib(const double_attrib::CamAttrib attrib)
{
    if (!dc_camera)
    return false;
    
    dc1394framerate_t dc_framerate;
    double framerate = 0;
    
    switch(attrib)
    {
        // get current frame rate
        case double_attrib::FrameRate:
            dc1394_video_get_framerate(dc_camera, &dc_framerate);
            switch(dc_framerate)
            {
                case DC1394_FRAMERATE_1_875:
                    framerate = 1.875;
                    break;
                case DC1394_FRAMERATE_3_75:
                    framerate = 3.75;
                    break;
                case DC1394_FRAMERATE_7_5:
                    framerate = 7.5;
                    break;
                case DC1394_FRAMERATE_15:
                    framerate = 15;
                    break;
                case DC1394_FRAMERATE_30:
                    framerate = 30;
                    break;
                case DC1394_FRAMERATE_60:
                    framerate = 60;
                    break;
                case DC1394_FRAMERATE_120:
                    framerate = 120;
                    break;
                case DC1394_FRAMERATE_240:
                    framerate = 240;
                    break;
            }
            return framerate;
            break;
            
        // attribute unknown or not supported (yet)
        default:
            throw std::runtime_error("Unknown attribute!");
    }
}

dc1394error_t CamFireWire::setTriggerSource(const dc1394trigger_source_t trigger_source)
{
	dc1394error_t result = dc1394_external_trigger_set_source(dc_camera, trigger_source);
	if(result != DC1394_SUCCESS)
	    return result;
	
	if(trigger_source != DC1394_TRIGGER_SOURCE_SOFTWARE)
	{
        result = dc1394_feature_set_power(dc_camera, DC1394_FEATURE_TRIGGER, DC1394_ON);
        if(result != DC1394_SUCCESS)
        return result;
        
	    result = dc1394_software_trigger_set_power(dc_camera, DC1394_OFF);
	    if(result != DC1394_SUCCESS)
		return result;

	    result = dc1394_external_trigger_set_power(dc_camera, DC1394_ON);
	}
	else
	{
        result = dc1394_feature_set_power(dc_camera, DC1394_FEATURE_TRIGGER, DC1394_OFF);
        if(result != DC1394_SUCCESS)
        return result;
        
	    result = dc1394_external_trigger_set_power(dc_camera, DC1394_OFF);
	    if(result != DC1394_SUCCESS)
		return result;

	    result = dc1394_software_trigger_set_power(dc_camera, DC1394_ON);
	}
	
	return result;
}

// set enum attributes
bool CamFireWire::setAttrib(const enum_attrib::CamAttrib attrib)
{
    if (!dc_camera)
	return false;

    // the feature (attribute) we want to set
    dc1394feature_t feature;
    
    // the (enum) value we want to set
    dc1394switch_t value;
    
    // the feature mode (auto/manual) we want to set
    dc1394feature_mode_t mode;
    
    //result of the set operation
    dc1394error_t result = DC1394_SUCCESS;

    switch (attrib)
    {
	case enum_attrib::FrameStartTriggerModeToSyncIn1:
	    result = setTriggerSource(DC1394_TRIGGER_SOURCE_0);
	    break;
	
	case enum_attrib::FrameStartTriggerModeToSyncIn2:
	    result = setTriggerSource(DC1394_TRIGGER_SOURCE_1);
	    break;
	
	case enum_attrib::FrameStartTriggerModeToSyncIn3:
	    result = setTriggerSource(DC1394_TRIGGER_SOURCE_2);
	    break;

	case enum_attrib::FrameStartTriggerModeToSyncIn4:
	    result = setTriggerSource(DC1394_TRIGGER_SOURCE_3);
	    break;
        
    case enum_attrib::FrameStartTriggerModeToFreerun:
    case enum_attrib::FrameStartTriggerModeToFixedRate:
    case enum_attrib::FrameStartTriggerModeToSoftware:
        result = setTriggerSource(DC1394_TRIGGER_SOURCE_SOFTWARE);
        break;

	    
	case enum_attrib::FrameStartTriggerEventToEdgeRising:
	    //note this is a hack, as the camera interface does not support
	    //trigger mode 1. We hardcode to Mode 0
	    
	    //Mode zero triggers on falling edgde by default, therefore 
	    //we invert the polarity, that should in theory give us Rising edge
	    result = dc1394_external_trigger_set_polarity(dc_camera, DC1394_TRIGGER_ACTIVE_HIGH);
	    if(checkHandleError(result))
		return false;
	    
	    result = dc1394_external_trigger_set_mode(dc_camera, DC1394_TRIGGER_MODE_0);
	    if(checkHandleError(result))
		return false;
	    
	    break;
	    
	case enum_attrib::FrameStartTriggerEventToEdgeFalling:
	    result = dc1394_external_trigger_set_polarity(dc_camera, DC1394_TRIGGER_ACTIVE_LOW);
	    if(checkHandleError(result))
		return false;
	    
	    result = dc1394_external_trigger_set_mode(dc_camera, DC1394_TRIGGER_MODE_0);
	    if(checkHandleError(result))
		return false;
	    break;
	    
	// turn gamma on
	case enum_attrib::GammaToOn:
	    feature = DC1394_FEATURE_GAMMA;
	    value = DC1394_ON;
	    // set the desired attribute/feature value
	    result = dc1394_feature_set_power(dc_camera, feature , value);
	    break;
	    
	// turn gamma off
	case enum_attrib::GammaToOff:
	    feature = DC1394_FEATURE_GAMMA;
	    value = DC1394_OFF;
	    // set the desired attribute/feature value
	    result = dc1394_feature_set_power(dc_camera, feature , value);
	    break;
	    
	// turn auto exposure on
	case enum_attrib::ExposureModeToAuto:
	    feature = DC1394_FEATURE_EXPOSURE;
	    mode = DC1394_FEATURE_MODE_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// turn auto exposure off
	case enum_attrib::ExposureModeToManual:
	    feature = DC1394_FEATURE_EXPOSURE;
	    mode = DC1394_FEATURE_MODE_MANUAL;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// tell camera to do a single auto-exposure and then keep the setting fixed
	case enum_attrib::ExposureModeToAutoOnce:
	    feature = DC1394_FEATURE_EXPOSURE;
	    mode = DC1394_FEATURE_MODE_ONE_PUSH_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// turn auto gain on
	case enum_attrib::GainModeToAuto:
	    feature = DC1394_FEATURE_GAIN;
	    mode = DC1394_FEATURE_MODE_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// turn auto gain off
	case enum_attrib::GainModeToManual:
	    feature = DC1394_FEATURE_GAIN;
	    mode = DC1394_FEATURE_MODE_MANUAL;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;
	    
	// turn auto shutter time on
	case enum_attrib::ShutterModeToAuto:
	    feature = DC1394_FEATURE_SHUTTER;
	    mode = DC1394_FEATURE_MODE_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// turn auto shutter time off
	case enum_attrib::ShutterModeToManual:
	    feature = DC1394_FEATURE_SHUTTER;
	    mode = DC1394_FEATURE_MODE_MANUAL;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;

	// turn auto white balance on
	case enum_attrib::WhitebalModeToAuto:
	    feature = DC1394_FEATURE_WHITE_BALANCE;
	    mode = DC1394_FEATURE_MODE_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    
	    //dc1394bool_t result;
	    //uint32_t r;
	    //dc1394_get_register(dc_camera, 0x404, &r);
	    //std::cout << "reg 404 contains " << r << std::endl;
	    //dc1394feature_mode_t m;
	    //dc1394_feature_get_mode(dc_camera, feature, &m);
	    //dc1394_feature_is_present(dc_camera, feature, &result);
	    //dc1394_feature_has_auto_mode(dc_camera, feature, &result);
	    //std::cout << "awb is there? = " << result << " , mode = "<< m << std::endl; 
	    break;

	// turn one push auto white balance on
	case enum_attrib::WhitebalModeToAutoOnce:
	    feature = DC1394_FEATURE_WHITE_BALANCE;
	    mode = DC1394_FEATURE_MODE_ONE_PUSH_AUTO;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;
	    
	// turn manual white balance on
	case enum_attrib::WhitebalModeToManual:
	    feature = DC1394_FEATURE_WHITE_BALANCE;
	    mode = DC1394_FEATURE_MODE_MANUAL;
	    result = dc1394_feature_set_mode(dc_camera, feature, mode);
	    break;
	// attribute unknown or not supported (yet)
	default:
	    throw std::runtime_error("Unknown attribute!");
    };

    if(checkHandleError(result))
	return false;
    
    return true;
};

// set double-valued attributes
bool CamFireWire::setAttrib(const double_attrib::CamAttrib attrib, const double value)
{
    if (!dc_camera)
	return false;
    
    //result of the set operation
    dc1394error_t result = DC1394_SUCCESS;
    
    switch (attrib)
    {
    // set the framerate
    case double_attrib::FrameRate:
        dc1394video_mode_t video_mode;
        dc1394_video_get_mode(dc_camera, &video_mode);
        if(DC1394_VIDEO_MODE_FORMAT7_MIN <= video_mode && video_mode <= DC1394_VIDEO_MODE_FORMAT7_MAX)
        {
            unsigned int unit_bytes, max_bytes;
            if(checkHandleError(dc1394_format7_get_packet_parameters(dc_camera, video_mode, &unit_bytes, &max_bytes)))
                return false;

            //frame_size [bytes] * frame_rate [Hz] / 8000 Hz (see FAQ v2 for libdc1394)
            unsigned int packet_size = image_size_.height*image_size_.width * value/8000;

            if (unit_bytes == 0)
                unit_bytes = max_bytes;

            //round up to multiple of unit_bytes. This is needed for correct setting of packet_size.
            packet_size = (packet_size + unit_bytes - 1) / unit_bytes * unit_bytes;

            if (packet_size > max_bytes)
                throw std::runtime_error("Framerate too high for this mode 7");

            result = dc1394_format7_set_packet_size(dc_camera, video_mode, packet_size);
        }
        else
        {
            // the desired video framerate
            dc1394framerate_t framerate;

            if (value==30)
                framerate = DC1394_FRAMERATE_30;
            else if (value==60)
                framerate = DC1394_FRAMERATE_60;
            else if (value==120)
                framerate = DC1394_FRAMERATE_120;
            else if (value==240)
                framerate = DC1394_FRAMERATE_240;
            else if (value==15)
                framerate = DC1394_FRAMERATE_15;
            else if (value==7.5)
                framerate = DC1394_FRAMERATE_7_5;
            else if (value==3.75)
                framerate = DC1394_FRAMERATE_3_75;
            else if (value == 1.875)
                framerate = DC1394_FRAMERATE_1_875;
            else
                throw std::runtime_error("Framerate not supported by the dc1394 protocol!");
        
            if(!isFramerateSupported(framerate))
                throw std::runtime_error("Framerate is not supported by the actual video mode!");
            // the actual framerate-setting
            result = dc1394_video_set_framerate(dc_camera, framerate);
        }
        break;
    
    // attribute unknown or not supported (yet)
    default:
        throw std::runtime_error("Unknown attribute!");
    };
    
    if(checkHandleError(result))
        return false;
    
    return true;
};

bool CamFireWire::isFramerateSupported(const dc1394framerate_t framerate)
{
    dc1394video_mode_t video_mode;
    dc1394framerates_t supported_framerates;
    if(checkHandleError(dc1394_video_get_mode(dc_camera, &video_mode)))
	return false;
    
    if(video_mode >= DC1394_VIDEO_MODE_MAX - 7)
        return true;
    
    if(checkHandleError(dc1394_video_get_supported_framerates(dc_camera, video_mode, &supported_framerates)))
	return false;
    
    bool supported = false; 
    for(int i = 0; i < supported_framerates.num; i++)
    {
        if(supported_framerates.framerates[i] == framerate)
        {
            supported = true;
            break;
        }
    }
    return supported;
}

// returns true whenever a frame is available
bool CamFireWire::isFrameAvailable()
{
    if (!dc_camera)
	return false;
    
    fd_set set;
    FD_ZERO (&set);
    FD_SET (dc1394_capture_get_fileno(dc_camera), &set);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    int i = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
    
    if (i > 0) return true;
    return false;
}

bool CamFireWire::clearBuffer()
{
    if (!dc_camera)
	return false;

    dc1394video_frame_t *tmp = 0;
    
    bool endFound = false;
    dc1394error_t err;
    
    while (!endFound) {
        err = dc1394_capture_dequeue(dc_camera,DC1394_CAPTURE_POLICY_POLL, &tmp);
        if (tmp && err==DC1394_SUCCESS)
        {
            if(checkHandleError(dc1394_capture_enqueue(dc_camera, tmp)))
	    {
		std::cout << "clearBuffer(): Failed to enqueue frame" << std::endl;
		return false;
	    };
        } 
        else
        {
           endFound=true;
        }
    }
    return true;
} // end clearBuffer

int CamFireWire::getFileDescriptor() const
{
    if (!dc_camera)
	return -1;
    return dc1394_capture_get_fileno(dc_camera);
}

} // end namespace camera
