#ifndef Camino_arch_h
#define Camino_arch_h

// Configure register and interrupt names based on architecture and PORT.
#if PORT == -1
  #define UCSRNA            UCSRA
  #define UCSRNB            UCSRB
  #define UCSRNC            UCSRC
  #define U2XN              U2X
  #define UBRRNH            UBRRH
  #define UBRRNL            UBRRL
  #define RXENN             RXEN
  #define TXENN             TXEN
  #define RXCIEN            RXCIE
  #define USARTN_RX_vect    USART_RX_vect
  #define USARTN_UDRE_vect  USART_UDRE_vect
  #define UDRN              UDR
  #define UDRIEN            UDRIE
#elif PORT == 0
  #define UCSRNA            UCSR0A
  #define UCSRNB            UCSR0B
  #define UCSRNC            UCSR0C
  #define U2XN              U2X0
  #define UBRRNH            UBRR0H
  #define UBRRNL            UBRR0L
  #define RXENN             RXEN0
  #define TXENN             TXEN0
  #define RXCIEN            RXCIE0
  #ifdef ARDUINO_AVR_UNO
    #define USARTN_RX_vect    USART_RX_vect
    #define USARTN_UDRE_vect  USART_UDRE_vect
  #else
    #define USARTN_RX_vect    USART0_RX_vect
    #define USARTN_UDRE_vect  USART0_UDRE_vect
  #endif
  #define UDRN              UDR0
  #define UDRIEN            UDRIE0
#elif PORT == 1
  #define UCSRNA            UCSR1A
  #define UCSRNB            UCSR1B
  #define UCSRNC            UCSR1C
  #define U2XN              U2X1
  #define UBRRNH            UBRR1H
  #define UBRRNL            UBRR1L
  #define RXENN             RXEN1
  #define TXENN             TXEN1
  #define RXCIEN            RXCIE1
  #define USARTN_RX_vect    USART1_RX_vect
  #define USARTN_UDRE_vect  USART1_UDRE_vect
  #define UDRN              UDR1
  #define UDRIEN            UDRIE1
#elif PORT == 2
  #define UCSRNA            UCSR2A
  #define UCSRNB            UCSR2B
  #define UCSRNC            UCSR2C
  #define U2XN              U2X2
  #define UBRRNH            UBRR2H
  #define UBRRNL            UBRR2L
  #define RXENN             RXEN2
  #define TXENN             TXEN2
  #define RXCIEN            RXCIE2
  #define USARTN_RX_vect    USART2_RX_vect
  #define USARTN_UDRE_vect  USART2_UDRE_vect
  #define UDRN              UDR2
  #define UDRIEN            UDRIE2
#endif

#endif
