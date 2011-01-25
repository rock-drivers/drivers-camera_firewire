/*
 * File:   CamFireWire.h
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#ifndef _CAMFIREWIRE_H
#define	_CAMFIREWIRE_H

#include "camera_interface/CamInterface.h"
#include "./filter/frame2rggb.h"
#include "./cam_fw_types.h"

struct __dc1394_camera;
typedef __dc1394_camera dc1394camera_t;
struct __dc1394_t;
typedef __dc1394_t dc1394_t;

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
    bool undistortFrame(base::samples::frame::Frame &in, base::samples::frame::Frame &out, CalibrationData calib);
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

    /** Returns the file descriptor that can be used to wait for frames using
     * select()
     *
     * It is valid only after grab() has been called, and only until the
     * grabbing did not stop
     */
    int getFileDescriptor() const;
    
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

