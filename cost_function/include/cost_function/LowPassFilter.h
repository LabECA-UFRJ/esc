#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include "filters/transfer_function.h"

class LowPassFilter : public filters::SingleChannelTransferFunctionFilter<double>
{
public:
    LowPassFilter(double cutoffOmega, double samplingPeriod)
    {
        auto alpha = (samplingPeriod * cutoffOmega) / (1.0 + samplingPeriod * cutoffOmega);

        a_.clear();
        a_.push_back(1);
        a_.push_back(alpha - 1);

        b_.clear();
        b_.push_back(alpha);

        // Create the input and output buffers of the correct size.
        input_buffer_.reset(new filters::RealtimeCircularBuffer<double>(b_.size() - 1, temp_));
        output_buffer_.reset(new filters::RealtimeCircularBuffer<double>(a_.size() - 1, temp_));

        configured_ = true;
    }
};

#endif
