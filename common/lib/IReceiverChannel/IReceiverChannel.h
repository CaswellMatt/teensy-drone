#ifndef I_RECEIVER_CHANNEL
#define I_RECEIVER_CHANNEL

class IReceiverChannel {
public:
  virtual uint16_t getData() = 0;
  virtual uint8_t getId() = 0;
};

#endif
