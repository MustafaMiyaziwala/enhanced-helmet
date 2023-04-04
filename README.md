# Enhanced Construction Helmet

[Google Drive](https://drive.google.com/drive/folders/10Tn_0vgJD5vS_DCNFxtLP7ykrcBd-zxB?usp=share_link)

## Contents

- [Wireless Communication](#wireless-communication)
- [Capacitive Touch Input](#capacitive-touch-input)
- [Headlamp](#headlamp)

## Wireless Communication

Uses UART (`USART1`) over DMA to communicate with other devices using Xbee modules. Messages can target specific devices or all devices and contain a message type enum and an arbitrary data payload. Connect +3.3V and ground to the Xbee's `VCC` and `GND`, connect `XBEE_USART_RX` to the XBee's `DOUT`, and connect `XBEE_USART_TX` to the XBee's `DIN`.

### XBee Configuration

- Channel: 10
- Baud Rate: 115200
- Transparent Mode (Default Mode)

### Usage

Modify `xbee.h` an `xbee.c`. Add message types to `XBee_Command` and message processing logic to `XBee_Resolve()`. Transmit messages using `XBee_Transmit(data)` with a pointer to an `XBee_Data` struct containing your message.

## Capacitive Touch Input

TODO

## Headlamp

Use the `toggle_headlamp()` function defined in `headlamp.h` to toggle the headlamp (uses `TIM10` for timing).