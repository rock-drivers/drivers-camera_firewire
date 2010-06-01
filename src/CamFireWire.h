/*
 * File:   CamFireWire.h
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#ifndef _CAMFIREWIRE_H
#define	_CAMFIREWIRE_H

#include "camera_interface/CamInterface.h"
#include <dc1394/dc1394.h>
#include "./filter/frame2rggb.h"
#include <cv.h>

namespace camera
{
class CamFireWire : public CamInterface
{
public:
    CamFireWire();
    virtual ~CamFireWire();

    int listCameras(std::vector<CamInfo>&cam_infos)const;
    bool open(const CamInfo &cam,const AccessMode mode);
    bool isOpen()const;
    bool close();
    bool prepareQueueForGrabbing(const int queue_len);
    bool grab(const GrabMode mode, const int buffer_len);
    bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout);
    bool setFrameSettings(const base::samples::frame::frame_size_t size,
                          const base::samples::frame::frame_mode_t mode,
                          const  uint8_t color_depth,
                          const bool resize_frames);
    bool isFrameAvailable();
    bool setAttrib(const int_attrib::CamAttrib attrib,const int value);
    bool setAttrib(const enum_attrib::CamAttrib attrib);
    bool setAttrib(const double_attrib::CamAttrib attrib, const double value);
    bool isReadyForOneShot();
    bool clearBuffer();
    bool cleanup();
    bool setDevice(dc1394_t *dev);

public:
    dc1394camera_t *dc_camera;

private:
    dc1394_t *dc_device;
    bool hdr_enabled;
    int data_depth;
    int frame_size_in_byte_;
    int multi_shot_count;


};
}

#endif	/* _CAMFIREWIRE_H */

