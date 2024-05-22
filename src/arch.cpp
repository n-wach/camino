#define IN_CAMINO_LIBRARY
#include "arch.h"
#undef IN_CAMINO_LIBRARY


#ifdef ARDUINO_AVR_NANO_EVERY

    void Camino_InitPort(long baudRate) 
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

        //Set up the rx pin
        pinMode(24, INPUT_PULLUP);

        //Set up the tx pin
        digitalWrite(25, HIGH);
        pinMode(25, OUTPUT);

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

    void Camino_InitPort(long baudRate){ serialBegin(baudRate);}
    void Camino_SendByte(byte b){ SerialStream.write(b);} 
    void Camino_EnableByteSentISR(){ camino_tx_done = false; while (!camino_tx_done && serialAvailableForWrite()) Camino_ByteSent_vect();}
    void Camino_DisableByteSentISR(){ camino_tx_done = true;}
    byte Camino_ReadByte(){ return SerialStream.read();}
    void serialEvent(){ while (SerialStream.available()) Camino_ByteReadable_vect();}
    void serialEventRun(void){ Camino_EnableByteSentISR(); if (SerialStream.available()) serialEvent();}

#endif