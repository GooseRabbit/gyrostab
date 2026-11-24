#pragma once

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

class Servo
{
public:
    Servo()
        : attached_(false), attachSucceeds_(true), channel_(0),
          periodCalls_(0), attachCalls_(0), writeCalls_(0),
          lastPeriodHz_(0), lastPin_(-1), lastMinimumPulseUs_(0),
          lastMaximumPulseUs_(0)
    {
    }

    bool attached() const { return attached_; }

    void setPeriodHertz(int hertz)
    {
        ++periodCalls_;
        lastPeriodHz_ = hertz;
    }

    int attach(int pin, int minimumPulseUs, int maximumPulseUs)
    {
        ++attachCalls_;
        lastPin_ = pin;
        lastMinimumPulseUs_ = minimumPulseUs;
        lastMaximumPulseUs_ = maximumPulseUs;
        attached_ = attachSucceeds_;
        return channel_;
    }

    void writeMicroseconds(int)
    {
        ++writeCalls_;
    }

    void setAttachSucceeds(bool value) { attachSucceeds_ = value; }
    void setAttached(bool value) { attached_ = value; }
    void setChannel(int value) { channel_ = value; }

    int periodCalls() const { return periodCalls_; }
    int attachCalls() const { return attachCalls_; }
    int writeCalls() const { return writeCalls_; }
    int lastPeriodHz() const { return lastPeriodHz_; }
    int lastPin() const { return lastPin_; }
    int lastMinimumPulseUs() const { return lastMinimumPulseUs_; }
    int lastMaximumPulseUs() const { return lastMaximumPulseUs_; }

private:
    bool attached_;
    bool attachSucceeds_;
    int channel_;
    int periodCalls_;
    int attachCalls_;
    int writeCalls_;
    int lastPeriodHz_;
    int lastPin_;
    int lastMinimumPulseUs_;
    int lastMaximumPulseUs_;
};
