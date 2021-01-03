#ifndef I_BUS_CHANNEL
#define I_BUS_CHANNEL

#include "IBus.h"
#include "IReceiverChannel.h"

class IBusChannel : public IReceiverChannel {
private:
  IBus* m_iBus;
  uint8_t m_channelId;

public:
  IBusChannel();
  IBusChannel(IBus* ibus, uint8_t channelId);

  virtual uint16_t getData() override;
  virtual uint8_t getId() override;
};

#endif
