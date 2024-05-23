#include "arch.h"



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

    void Camino_InitPort(unsigned long baudRate)  {
        
        //Make sure global interrupts are disabled during initialization
        uint8_t oldSREG = SREG;
        cli();

        //UART0 bit macros are used for all UART ports as they are identical

        /* configure baudrate registers of UART */ 
        uint16_t clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L); 
        Camino_UBRR = clockRate;
 
        Camino_UCSRA |= _BV(U2X0); //enable double rate (2X flag)
        Camino_UCSRB |= _BV(RXEN0); //enable RX interrupt
        Camino_UCSRB |= _BV(TXEN0); //enable TX interrupt
        Camino_UCSRB |= _BV(RXCIE0); //enable serial receive complete interrupt
        Camino_UCSRC = 0x06;       // set 8 bit, no parity, 1 stop

        SREG = oldSREG;
    } 

    //UART0 bit macros are used for all UART ports as they are identical
    void Camino_SendByte(byte b){ Camino_UDR = b; } 
    void Camino_EnableByteSentISR(){ Camino_UCSRB |= _BV(UDRIE0); }
    void Camino_DisableByteSentISR(){ Camino_UCSRB &= ~(_BV(UDRIE0)); }
    byte Camino_ReadByte() { return Camino_UDR; }

#elif defined(ARDUINO_AVR_NANO_EVERY)

    void Camino_InitPort(unsigned long baudRate) 
    {
        //Make sure global interrupts are disabled during initialization
        uint8_t oldSREG = SREG;
        cli();

        // Setup port mux
        PORTMUX.USARTROUTEA |= Camino_USART_Mux;

        // disable double rate (2X flag)
        Camino_USART.CTRLB &= ~(USART_RXMODE_CLK2X_gc);

        // configure baudrate registers of UART
        int32_t baud_setting = (((8 * F_CPU) / baudRate) + 1) / 2;
        int8_t sigrow_val = SIGROW.OSC16ERR5V;
        baud_setting += (baud_setting * sigrow_val) / 1024;
        Camino_USART.BAUD = (int16_t)baud_setting;

        // set 8 bit, no parity, 1 stop
        Camino_USART.CTRLC = USART_CHSIZE_8BIT_gc | USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;

        // enable RX and TX interrupts
        Camino_USART.CTRLB |= USART_RXEN_bm;
        Camino_USART.CTRLB |= USART_TXEN_bm;

        // enable serial receive complete interrupt
        Camino_USART.CTRLA |= USART_RXCIE_bm;

        //Set TX pin output
        Camino_SetPinMode();

        // Restore SREG content
        SREG = oldSREG;
    } 

    void Camino_SendByte(byte b){ Camino_USART.TXDATAL = b;  } 
    void Camino_EnableByteSentISR(){ Camino_USART.CTRLA |= USART_DREIE_bm; }
    void Camino_DisableByteSentISR(){ Camino_USART.CTRLA &= ~(USART_DREIE_bm); }
    byte Camino_ReadByte() { return Camino_USART.RXDATAL; }

#elif defined(ARDUINO_SAM_DUE)

    extern void Camino_ByteSent_vect();
    extern void Camino_ByteReadable_vect();
    bool camino_tx_done;

    void Camino_InitPort(unsigned long baudRate){ serialBegin(baudRate);}
    void Camino_SendByte(byte b){ SerialStream.write(b);} 
    void Camino_EnableByteSentISR(){ camino_tx_done = false; while (!camino_tx_done && serialAvailableForWrite()) Camino_ByteSent_vect();}
    void Camino_DisableByteSentISR(){ camino_tx_done = true;}
    byte Camino_ReadByte(){ return SerialStream.read();}
    void serialEvent(){ while (SerialStream.available()) Camino_ByteReadable_vect();}
    void serialEventRun(void){ Camino_EnableByteSentISR(); if (SerialStream.available()) serialEvent();}


#elif defined(ARDUINO_AVR_LEONARDO) \
  || defined(ARDUINO_AVR_LEONARDO_ETH) \
  || defined(ARDUINO_AVR_MICRO) \
  || defined(ARDUINO_AVR_ESPLORA) \
  || defined(ARDUINO_AVR_YUN) \
  || defined(ARDUINO_AVR_YUNMINI) \
  || defined(ARDUINO_AVR_LILYPAD_USB)
  #error "ATmega32u4: Not supported. See arch.h to add support and file a PR!"

#endif