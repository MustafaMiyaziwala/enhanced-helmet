# Enhanced Construction Helmet

[Google Drive](https://drive.google.com/drive/folders/10Tn_0vgJD5vS_DCNFxtLP7ykrcBd-zxB?usp=share_link)

## Contents

- [Wireless Communication](#wireless-communication)
- [Capacitive Touch Input](#capacitive-touch-input)
- [Headlamp](#headlamp)

## Wireless Communication

Uses UART (`USART1`) over DMA to communicate with other devices using Xbee modules. Messages can target specific devices or all devices and contain a message type enum and an arbitrary data payload. 

## XBee Connections

- +3.3V and ground to the Xbee's `VCC` and `GND`
- `XBEE_USART_RX` to the XBee's `DOUT`
- `XBEE_USART_TX` to the XBee's `DIN`
- `XBEE_USART_RTS` to the XBee's `RTS`
- `XBEE_USART_CTS` to the XBee's `CTS`

### XBee Configuration

- Channel: 10
- Baud Rate: 115200
- DIO7 Configuration: CTS
- DIO6 Configuration: RTS
- Transparent Mode (Default Mode, don't have to change anything here just make sure API mode isn't on)

### Usage

Modify `xbee.h` an `xbee.c`. Add message types to `XBee_Command` and message processing logic to `XBee_Resolve()`. Transmit messages using `XBee_Transmit(data)` with a pointer to an `XBee_Data` struct containing your message.

## Capacitive Touch Input

TODO

## Headlamp

Use the `toggle_headlamp()` function defined in `headlamp.h` to toggle the headlamp (uses `TIM10` for timing).