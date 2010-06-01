#ifndef FILTER_FRAME2RGGB
#define FILTER_FRAME2RGGB 1

#include "base/samples/frame.h"

namespace filter
{

    class Frame2RGGB
    {
        public:
            static bool process(const base::samples::frame::Frame &in, base::samples::frame::Frame &out);
    };

}

#endif /* FILTER_FRAME2RGB */

