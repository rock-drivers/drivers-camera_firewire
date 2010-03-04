/* 
 * File:   CamFireWire.h
 * Author: Christopher Gaudig, DFKI Bremen
 *
 * Created on February 23, 2010, 4:57 PM
 */

#ifndef _CAMFIREWIRE_H
#define	_CAMFIREWIRE_H

#include "/home/cgaudig/CSurvey/drivers/camera_interface/src/CamInterface.h"
#include "../arch.h"
#include <dc1394/dc1394.h>
#include <list>
#include "../CamInfoUtils.h"
#include "../extern/filter/frame2rggb.h"


namespace camera
{
    class CamFireWire : public CamInterface
    {
    public:
        CamFireWire();
        virtual ~CamFireWire();

        //! Retrieves bla.
        /*! This function is bla.
            \param bla bla
            \return returns bla
            \attention bla
         */
        int listCameras(std::vector<CamInfo>&cam_infos)const;
        bool open(const CamInfo &cam,const AccessMode mode);
        bool isOpen()const;
        bool close();
        bool prepareQueueForGrabbing(const int queue_len);
        bool grab(const GrabMode mode, const int buffer_len);
        bool retrieveFrame(Frame &frame,const int timeout);
        bool setFrameSettings(const frame_size_t size,
                                      const frame_mode_t mode,
                                      const  uint8_t color_depth,
                                      const bool resize_frames);
        bool isFrameAvailable();
        bool setAttrib(const int_attrib::CamAttrib attrib,const int value);
        bool setAttrib(const enum_attrib::CamAttrib attrib);


    private:
        dc1394camera_t *dc_camera;
        dc1394video_frame_t *camFrame;
        dc1394video_frame_t *camFrame2;
        std::list<dc1394video_frame_t*> frames;
        bool hdr_enabled;
        int data_depth;
        int frame_size_in_byte_;

    };
}

#endif	/* _CAMFIREWIRE_H */

