#ifndef Camino_arch_h
#define Camino_arch_h

#include "wiring_private.h"

// Architecture flags listed in packages/arduino/hardware/avr/1.8.6/boards.txt
// Supported by Camino:
//  #define ARDUINO_AVR_MEGA | ARDUINO_AVR_MEGA2560 | ARDUINO_AVR_ADK
//    PORT = 0, 1, 2
//  #define ARDUINO_AVR_UNO
//    PORT = 0
// Not officially supported (file a PR!):
//  #define ARDUINO_AVR_ADK
//  #define ARDUINO_AVR_BT
//  #define ARDUINO_AVR_CIRCUITPLAY
//  #define ARDUINO_AVR_DUEMILANOVE
//  #define ARDUINO_AVR_ESPLORA
//  #define ARDUINO_AVR_ETHERNET
//  #define ARDUINO_AVR_FIO
//  #define ARDUINO_AVR_GEMMA
//  #define ARDUINO_AVR_INDUSTRIAL101
//  #define ARDUINO_AVR_LEONARDO
//  #define ARDUINO_AVR_LEONARDO_ETH
//  #define ARDUINO_AVR_LILYPAD
//  #define ARDUINO_AVR_LILYPAD_USB
//  #define ARDUINO_AVR_LININO_ONE
//  #define ARDUINO_AVR_MICRO
//  #define ARDUINO_AVR_MINI
//  #define ARDUINO_AVR_NANO
//  #define ARDUINO_AVR_NG
//  #define ARDUINO_AVR_PRO
//  #define ARDUINO_AVR_ROBOT_CONTROL
//  #define ARDUINO_AVR_ROBOT_MOTOR
//  #define ARDUINO_AVR_UNO_WIFI_DEV_ED
//  #define ARDUINO_AVR_YUN
//  #define ARDUINO_AVR_YUNMINI

#if defined(ARDUINO_AVR_UNO)
  #define Camino_InitPort(baudRate) do {\
    /* configure baudrate registers of UART */ \
    uint16_t clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L); \
    UBRR0H = clockRate >> 8; \
    UBRR0L = clockRate & 0xff; \
    sbi(UCSR0A, U2X0);   /* enable double rate (2X flag) */ \
    sbi(UCSR0B, RXEN0);  /* enable RX interrupt */ \
    sbi(UCSR0B, TXEN0);  /* enable TX interrupt */ \
    sbi(UCSR0B, RXCIE0); /* enable serial receive complete interrupt */ \
    UCSR0C = 0x06;       /* set 8 bit, no parity, 1 stop */ \
  } while(0)
  #define Camino_SendByte(v) do { \
    UDR0 = v; \
  } while(0)
  #define Camino_EnableByteSentISR() sbi(UCSR0B, UDRIE0)
  #define Camino_DisableByteSentISR() cbi(UCSR0B, UDRIE0)
  #define Camino_ReadByte() (UDR0)
  #define Camino_ByteReadable_vect USART_RX_vect
  #define Camino_ByteSent_vect USART_UDRE_vect
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_ADK)
  #if PORT == 0
    #define Camino_InitPort(baudRate) do {\
      /* configure baudrate registers of UART */ \
      uint16_t clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L); \
      UBRR0H = clockRate >> 8; \
      UBRR0L = clockRate & 0xff; \
      sbi(UCSR0A, U2X0);   /* enable double rate (2X flag) */ \
      sbi(UCSR0B, RXEN0);  /* enable RX interrupt */ \
      sbi(UCSR0B, TXEN0);  /* enable TX interrupt */ \
      sbi(UCSR0B, RXCIE0); /* enable serial receive complete interrupt */ \
      UCSR0C = 0x06;       /* set 8 bit, no parity, 1 stop */ \
    } while(0)
    #define Camino_SendByte(v) do { \
      UDR0 = v; \
    } while(0)
    #define Camino_EnableByteSentISR() sbi(UCSR0B, UDRIE0)
    #define Camino_DisableByteSentISR() cbi(UCSR0B, UDRIE0)
    #define Camino_ReadByte() (UDR0)
    #define Camino_ByteReadable_vect USART0_RX_vect
    #define Camino_ByteSent_vect USART0_UDRE_vect
  #elif PORT == 1
    #define Camino_InitPort(baudRate) do {\
      /* configure baudrate registers of UART */ \
      uint16_t clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L); \
      UBRR1H = clockRate >> 8; \
      UBRR1L = clockRate & 0xff; \
      sbi(UCSR1A, U2X1);   /* enable double rate (2X flag) */ \
      sbi(UCSR1B, RXEN1);  /* enable RX interrupt */ \
      sbi(UCSR1B, TXEN1);  /* enable TX interrupt */ \
      sbi(UCSR1B, RXCIE1); /* enable serial receive complete interrupt */ \
      UCSR1C = 0x06;       /* set 8 bit, no parity, 1 stop */ \
    } while(0)
    #define Camino_SendByte(v) do { \
      UDR0 = v; \
    } while(0)
    #define Camino_EnableByteSentISR() sbi(UCSR1B, UDRIE1)
    #define Camino_DisableByteSentISR() cbi(UCSR1B, UDRIE1)
    #define Camino_ReadByte() (UDR1)
    #define Camino_ByteReadable_vect USART1_RX_vect
    #define Camino_ByteSent_vect USART1_UDRE_vect
  #elif PORT == 2
    #define Camino_InitPort(baudRate) do {\
      /* configure baudrate registers of UART */ \
      uint16_t clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L); \
      UBRR2H = clockRate >> 8; \
      UBRR2L = clockRate & 0xff; \
      sbi(UCSR2A, U2X2);   /* enable double rate (2X flag) */ \
      sbi(UCSR2B, RXEN2);  /* enable RX interrupt */ \
      sbi(UCSR2B, TXEN2);  /* enable TX interrupt */ \
      sbi(UCSR2B, RXCIE2); /* enable serial receive complete interrupt */ \
      UCSR2C = 0x06;       /* set 8 bit, no parity, 1 stop */ \
    } while(0)
    #define Camino_SendByte(v) do { \
      UDR0 = v; \
    } while(0)
    #define Camino_EnableByteSentISR() sbi(UCSR2B, UDRIE2)
    #define Camino_DisableByteSentISR() cbi(UCSR2B, UDRIE2)
    #define Camino_ReadByte() (UDR2)
    #define Camino_ByteReadable_vect USART2_RX_vect
    #define Camino_ByteSent_vect USART2_UDRE_vect
  #else
    #error "Unsupported PORT for ARDUINO_AVR_MEGA"
  #endif
#else
  # error "Unsupported board. See arch.h to add support"
#endif

#endif
