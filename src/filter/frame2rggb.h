#ifndef FILTER_FRAME2RGGB
#define FILTER_FRAME2RGGB 1

#include "../../../camera_interface/src/frame.h"

namespace filter
{

    class Frame2RGGB
    {
        public:
            static bool process(const camera::Frame &in, camera::Frame &out);
    };

}

#endif /* FILTER_FRAME2RGB */

