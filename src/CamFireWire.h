/*
 * File:   CamFireWire.h
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#ifndef _CAMFIREWIRE_H
#define	_CAMFIREWIRE_H

#include "camera_interface/CamInterface.h"
#include "base/samples/frame.h"
#include "./filter/frame2rggb.h"
#include "./cam_fw_types.h"
#include <dc1394/types.h>
#include <dc1394/log.h>

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
    //grab() may change the filedescriptor, and mode==Stop closes it
    bool grab(const GrabMode mode, const int buffer_len);
    bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout);
    bool setFrameSettings(const base::samples::frame::frame_size_t size,
                          const base::samples::frame::frame_mode_t mode,
                          const  uint8_t color_depth,
                          const bool resize_frames);
    bool isFrameAvailable();
    bool isAttribAvail(const int_attrib::CamAttrib attrib);
    bool isAttribAvail(const double_attrib::CamAttrib attrib);
    bool isAttribAvail(const str_attrib::CamAttrib attrib);
    bool isAttribAvail(const enum_attrib::CamAttrib attrib);
    int getAttrib(const int_attrib::CamAttrib attrib);
    double getAttrib(const double_attrib::CamAttrib attrib);
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
     * It is valid only after grab() has been called(with mode != Stop),
     * and only until grab() is called again(for whatever mode)
     */
    int getFileDescriptor() const;
    
public:
    dc1394camera_t *dc_camera;

private:
    bool isVideoModeSupported(const dc1394video_mode_t mode);
    bool isVideo7RAWModeSupported(int depth);
    /**
     * Checks if an error was reported and logs it.
     * @return true if an error was reported
     * @return false if NO error was reported
     * */
    bool checkHandleError(dc1394error_t error) const;
    
    dc1394_t *dc_device;
    base::samples::frame::Frame unconverted_frame;
    base::samples::frame::frame_mode_t frame_mode;
    int data_depth;
    bool hdr_enabled;
    int frame_size_in_byte_;
    int multi_shot_count;


};
}

#endif	/* _CAMFIREWIRE_H */

