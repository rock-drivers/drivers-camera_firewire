#include "frame2rggb.h"

#include <dc1394/conversions.h>
#include <iostream>

using namespace base::samples::frame;
namespace filter
{
    bool Frame2RGGB::process(const Frame &in, Frame &out)
    {
        if (in.getFrameMode() != MODE_BAYER_RGGB)
        {
            std::cerr << "Frame2RGGB: "
                << __FUNCTION__ << " (" << __FILE__ << ", line "
                << __LINE__ << "): " << "debayering possible only with raw images"
                << std::endl;
            return false;
        }

        // Create output image
        out.init(in.getWidth(), in.getHeight(), in.getDataDepth(), MODE_RGB);

        if (in.getDataDepth() <= 8)
        {
            const uint8_t *inptr  = static_cast<const uint8_t *>(in.getImageConstPtr());
            uint8_t       *outptr = static_cast<uint8_t *>(out.getImagePtr());

            dc1394_bayer_decoding_8bit(inptr, outptr, in.getWidth(), in.getHeight(),
                                       DC1394_COLOR_FILTER_RGGB, DC1394_BAYER_METHOD_SIMPLE);
        }
        else if (in.getDataDepth() <= 16)
        {
            const uint16_t *inptr  = reinterpret_cast<const uint16_t *>(in.getImageConstPtr());
            uint16_t       *outptr = reinterpret_cast<uint16_t *>(out.getImagePtr());

            dc1394_bayer_decoding_16bit(inptr, outptr, in.getWidth(), in.getHeight(),
                                        DC1394_COLOR_FILTER_RGGB, DC1394_BAYER_METHOD_SIMPLE, 16);
        }
        else
        {
            std::cerr << "Frame2RGGB: "
                << __FUNCTION__ << " (" << __FILE__ << ", line "
                << __LINE__ << "): " << "colour depth > 16 bit not supported"
                << std::endl;
            return false;
        }

        out.time = in.time;

        return true;
    }

}


