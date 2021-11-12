#pragma once

#include "DMAController.hpp"
#include "DeviceTraits.hpp"

#include <vector>

namespace Device
{

class DeviceStream
{
    using BytesCollectionType = std::vector<uint8_t>;
    using ActionHandler = std::function<void(BytesCollectionType& data)>;

protected:
    BytesCollectionType buffer_;

public:
    ActionHandler onTransferErrorHappened;

public:
    DeviceStream();
};

class InputDeviceStream : public DeviceStream
{
public:
    ActionHandler onDataSentHandler;
    ActionHandler onHalfDataSentHandler;
};

class OutputDeviceStream : public DeviceStream
{
public:
    ActionHandler onDataArrived;
    ActionHandler onHalfDataArrived;
};

} // namespace Device
