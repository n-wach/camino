#ifndef Camino_arch_h
#define Camino_arch_h

#include "Arduino.h"
#include "options.h"



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

  extern byte& Camino_UDR;
  extern byte& Camino_UCSRA;
  extern byte& Camino_UCSRB;
  extern byte& Camino_UCSRC;
  extern uint16_t& Camino_UBRR;

#elif defined(ARDUINO_AVR_NANO_EVERY)

  extern USART_t& Camino_USART;
  extern const register8_t Camino_USART_Mux;
  extern void (*Camino_SetPinMode)();

#elif defined(ARDUINO_SAM_DUE)

  extern Stream& SerialStream;
  extern void (*serialBegin)(unsigned long baud);
  extern int (*serialAvailableForWrite)();
  
#elif defined(ARDUINO_AVR_LEONARDO) \
  || defined(ARDUINO_AVR_LEONARDO_ETH) \
  || defined(ARDUINO_AVR_MICRO) \
  || defined(ARDUINO_AVR_ESPLORA) \
  || defined(ARDUINO_AVR_YUN) \
  || defined(ARDUINO_AVR_YUNMINI) \
  || defined(ARDUINO_AVR_LILYPAD_USB)
  #error "ATmega32u4: Not supported. See arch.h to add support and file a PR!"
#elif defined(ARDUINO_AVR_GEMMA)
  #error "ATtiny85: Not supported. See arch.h to add support and file a PR!"
#else
  # error "Unsupported board. See arch.h to add support and file a PR!"
#endif

#endif
