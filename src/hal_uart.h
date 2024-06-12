
#ifndef INCLUDE_HAL_UART_H_
#define INCLUDE_HAL_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART0_PORT = 0,
    UART1_PORT,
    UART2_PORT,
    UART3_PORT,
} uart_port_t;
 
typedef void (*uart_recv_callback_t)(uart_port_t uart_port, uint8_t *data, uint8_t len);

int hal_uart_init(uart_recv_callback_t cb);

void uart1_write_bytes(const uint8_t *src, size_t size);

 
#ifdef __cplusplus
}
#endif

#endif

