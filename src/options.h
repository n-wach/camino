#if !defined(Camino_options_h) && !defined(IN_CAMINO_LIBRARY) 
#define Camino_options_h

#include "Arduino.h"


// This file defines a number of configuration options for Camino. If you
// #define any of these options before including Camino.h, the default value
// will be overridden.

// The UART port used by Camino. See arch.h for the supported values for each
// architecture.
#if !defined(CAMINO_SERIAL_PORT)
  #define CAMINO_SERIAL_PORT 0
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


byte packetDataArray[MAX_DATA_LENGTH];
byte packetArray[2 + 1 + MAX_DATA_LENGTH + 1];
byte Camino_COMMAND_TIMEOUT_MS  __attribute__((used, retain)) = COMMAND_TIMEOUT_MS;
byte Camino_MAX_DATA_LENGTH __attribute__((used, retain)) = MAX_DATA_LENGTH;



// Architecture flags listed in packages/arduino/hardware/avr/1.8.6/boards.txt
// See also: https://docs.arduino.cc/software/ide-v1/tutorials/arduino-ide-v1-basics/#boards
// Supported by Camino:
//  ATmega1280/2560:
//    #define ARDUINO_AVR_MEGA
//          | ARDUINO_AVR_MEGA2560
//          | ARDUINO_AVR_ADK
//    #define CAMINO_SERIAL_PORT 0, 1, 2, 3
//
//  ATmega132/168/328P:
//    #define ARDUINO_AVR_UNO
//          | ARDUINO_AVR_UNO_WIFI_DEV_ED
//          | ARDUINO_AVR_NANO
//          | ARDUINO_AVR_MINI
//          | ARDUINO_AVR_ETHERNET
//          | ARDUINO_AVR_FIO
//          | ARDUINO_AVR_BT
//          | ARDUINO_AVR_PRO
//          | ARDUINO_AVR_LILYPAD
//          | ARDUINO_AVR_DUEMILANOVE
//    #define CAMINO_SERIAL_PORT 0
//
// Not officially supported (file a PR!):
//  ATmega32u4:
//    #define ARDUINO_AVR_LEONARDO
//          | ARDUINO_AVR_LEONARDO_ETH
//          | ARDUINO_AVR_MICRO
//          | ARDUINO_AVR_ESPLORA
//          | ARDUINO_AVR_YUN
//          | ARDUINO_AVR_YUNMINI
//          | ARDUINO_AVR_LILYPAD_USB
//    #define CAMINO_SERIAL_PORT 1
//
//  ATtiny85:
//    #define ARDUINO_AVR_GEMMA
//
//  Others:
//    #define ARDUINO_AVR_CIRCUITPLAY
//    #define ARDUINO_AVR_INDUSTRIAL101
//    #define ARDUINO_AVR_LININO_ONE
//    #define ARDUINO_AVR_ROBOT_CONTROL
//    #define ARDUINO_AVR_ROBOT_MOTOR
//    #define ARDUINO_AVR_NG





#if defined(ARDUINO_AVR_UNO) \
  || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) \
  || defined(ARDUINO_AVR_NANO) \
  || defined(ARDUINO_AVR_MINI) \
  || defined(ARDUINO_AVR_ETHERNET) \
  || defined(ARDUINO_AVR_FIO) \
  || defined(ARDUINO_AVR_BT) \
  || defined(ARDUINO_AVR_PRO) \
  || defined(ARDUINO_AVR_LILYPAD) \
  || defined(ARDUINO_AVR_DUEMILANOVE) \
  || defined(ARDUINO_AVR_MEGA) \
  || defined(ARDUINO_AVR_MEGA2560) \
  || defined(ARDUINO_AVR_ADK)

    #if CAMINO_SERIAL_PORT != 0 \
      && !(defined(ARDUINO_AVR_MEGA) \
      || defined(ARDUINO_AVR_MEGA2560) \
      || defined(ARDUINO_AVR_ADK))
        #error "ATmega132/168/328P: PORT must be 0"
    #endif

  #if CAMINO_SERIAL_PORT == 0

    #ifndef USART0_UDRE_vect
      #define USART0_UDRE_vect USART_UDRE_vect
      #define USART0_RX_vect USART_RX_vect
    #endif

    volatile byte& Camino_UDR = UDR0;
    volatile byte& Camino_UCSRA = UCSR0A;
    volatile byte& Camino_UCSRB = UCSR0B;
    volatile byte& Camino_UCSRC = UCSR0C;
    volatile uint16_t& Camino_UBRR = UBRR0;
    ISR_ALIAS(USART0_UDRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART0_RX_vect, Camino_ByteReadable_vect);

  #elif CAMINO_SERIAL_PORT == 1

    volatile byte& Camino_UDR = UDR1;
    volatile byte& Camino_UCSRA = UCSR1A;
    volatile byte& Camino_UCSRB = UCSR1B;
    volatile byte& Camino_UCSRC = UCSR1C;
    volatile uint16_t& Camino_UBRR = UBRR1;
    ISR_ALIAS(USART1_UDRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART1_RX_vect, Camino_ByteReadable_vect); 

  #elif CAMINO_SERIAL_PORT == 2

    volatile byte& Camino_UDR = UDR2;
    volatile byte& Camino_UCSRA = UCSR2A;
    volatile byte& Camino_UCSRB = UCSR2B;
    volatile byte& Camino_UCSRC = UCSR2C;
    volatile uint16_t& Camino_UBRR = UBRR2;
    ISR_ALIAS(USART2_UDRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART2_RX_vect, Camino_ByteReadable_vect);

  #elif CAMINO_SERIAL_PORT == 3

    volatile byte& Camino_UDR = UDR3;
    volatile byte& Camino_UCSRA = UCSR3A;
    volatile byte& Camino_UCSRB = UCSR3B;
    volatile byte& Camino_UCSRC = UCSR3C;
    volatile uint16_t& Camino_UBRR = UBRR3;
    ISR_ALIAS(USART3_UDRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART3_RX_vect, Camino_ByteReadable_vect);

  #else
    #error "ATmega1280/2560: PORT must be 0, 1, 2, or 3"
  #endif
#elif defined(ARDUINO_AVR_LEONARDO) \
  || defined(ARDUINO_AVR_LEONARDO_ETH) \
  || defined(ARDUINO_AVR_MICRO) \
  || defined(ARDUINO_AVR_ESPLORA) \
  || defined(ARDUINO_AVR_YUN) \
  || defined(ARDUINO_AVR_YUNMINI) \
  || defined(ARDUINO_AVR_LILYPAD_USB)
  #error "ATmega32u4: Not supported. See arch.h to add support and file a PR!"
#elif defined(ARDUINO_AVR_NANO_EVERY)

  #define USB 3
  #if CAMINO_SERIAL_PORT == 1
    ISR_ALIAS(USART1_DRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART1_RXC_vect, Camino_ByteReadable_vect);
    USART_t& Camino_USART = USART1;
    const register8_t Camino_USART_Mux = PORTMUX_USART1_enum::PORTMUX_USART1_ALT1_gc;
    auto Camino_SetPinMode = static_cast<void(*)()>([]()-> void {PORTC.DIRSET = _BV(4);});
  #elif CAMINO_SERIAL_PORT == 3
    ISR_ALIAS(USART3_DRE_vect, Camino_ByteSent_vect);
    ISR_ALIAS(USART3_RXC_vect, Camino_ByteReadable_vect);
    USART_t& Camino_USART = USART3;
    const register8_t Camino_USART_Mux = PORTMUX_USART3_enum::PORTMUX_USART3_ALT1_gc;
    auto Camino_SetPinMode = static_cast<void(*)()>([]()-> void {PORTB.DIRSET = _BV(4);});
  #else
    #error "Nano Every: For USB serial PORT must be USB (3), for Pins PORT must be 1."
  #endif

#elif defined(ARDUINO_AVR_GEMMA)
  #error "ATtiny85: Not supported. See arch.h to add support and file a PR!"
#elif defined(ARDUINO_SAM_DUE)

  extern void serialEvent();
  #define USB_P 0
  #define USB_N 4
  #if CAMINO_SERIAL_PORT == 0
    Stream& SerialStream = Serial;
    auto serialBegin = static_cast<void(*)(unsigned long)>([](unsigned long baud)-> void {Serial.begin(baud);});
    auto serialAvailableForWrite = static_cast<int(*)()>([]()->int {return Serial.availableForWrite();});
  #elif CAMINO_SERIAL_PORT == 1
    Stream& SerialStream = Serial1;
    auto serialBegin = static_cast<void(*)(unsigned long)>([](unsigned long baud)-> void {Serial1.begin(baud);});
    auto serialAvailableForWrite = static_cast<int(*)()>([]()->int {return Serial1.availableForWrite();;});
  #elif CAMINO_SERIAL_PORT == 2
    Stream& SerialStream = Serial2;
    auto serialBegin = static_cast<void(*)(unsigned long)>([](unsigned long baud)-> void {Serial2.begin(baud);});
    auto serialAvailableForWrite = static_cast<int(*)()>([]()->int {return Serial2.availableForWrite();});
  #elif CAMINO_SERIAL_PORT == 3
    Stream& SerialStream = Serial3;
    auto serialBegin = static_cast<void(*)(unsigned long)>([](unsigned long baud)-> void {Serial3.begin(baud);});
    auto serialAvailableForWrite = static_cast<int(*)()>([]()->int {return Serial3.availableForWrite();});
  #elif CAMINO_SERIAL_PORT == 4
    Stream& SerialStream = SerialUSB;
    auto serialBegin = static_cast<void(*)(unsigned long)>([](unsigned long baud)-> void {SerialUSB.begin(baud);});
    auto serialAvailableForWrite = static_cast<int(*)()>([]()->int {return SerialUSB.availableForWrite();});
  #else 
    #error "Due: Native USB Serial PORT: USB_N (4), Programmer Serial PORT: USB_P (0)"
  #endif
  
#else
  # error "Unsupported board. See arch.h to add support and file a PR!"
#endif

#endif