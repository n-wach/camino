#ifndef Camino_options_h
#define Camino_options_h

// This file defines a number of configuration options for Camino. If you
// #define any of these options before including Camino.h, the default value
// will be overridden.

// The UART port used by Camino. See arch.h for the supported values for each
// architecture.
#if !defined(PORT)
  #define PORT 0
#endif

// The maximum length of the data section for packets (to and from the Arduino).
// Value must be at least 16 and at most 250. Making it smaller reduces the
// memory footprint of Camino.
# if !defined(MAX_DATA_LENGTH)
  #define MAX_DATA_LENGTH 250
#endif

// Command timeout in milliseconds. When a new byte in a packet is received, we
// check if this period has elapsed since the start of the packet. If it has,
// the earlier bytes are discarded and Camino assumes that a new packet is
// beginning.
#if !defined(COMMAND_TIMEOUT_MS)
  #define COMMAND_TIMEOUT_MS 100
#endif

// Macros for transmission hooks. Sometimes you may need to do some preparation
// before a packet can be sent. For instance, you may have a pin that enables a
// pull down resistor to remove noise from your transmission line when not
// sending anything. Or when debugging, it may be useful to shine an LED when
// packets are being sent.
# if !defined(initTransmissions)
  // called in begin()
  #define initTransmissions() {}
#endif
# if !defined(beginTransmission)
  // called before the first byte is sent in a response packet.
  #define beginTransmission() {}
#endif
# if !defined(endTransmission)
  // called after the last byte is sent in a response packet.
  #define endTransmission() {}
#endif

#endif
