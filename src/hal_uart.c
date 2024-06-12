/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
  
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "hal_uart.h"

LOG_MODULE_REGISTER(hal_uart, LOG_LEVEL_DBG);

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
// static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

#define UART_BUF_SIZE               64
#define UART_WAIT_FOR_RX            5000   // US
 
static uint8_t __aligned(4) uart_msgq_buff[UART_BUF_SIZE * 2];
static struct k_msgq uart_msgq;
 
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
 
	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		break;

    case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		break;

	case UART_RX_RDY: 
		LOG_DBG("UART_RX_RDY");
		printk("evt->data.rx.buf = %s | %d/%d\r\n", evt->data.rx.buf, evt->data.rx.len, evt->data.rx.offset);
		k_msgq_put(&uart_msgq, &evt->data.rx, K_NO_WAIT);
		break;
	
	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
        break;

	case UART_RX_BUF_REQUEST: {
		LOG_DBG("UART_RX_BUF_REQUEST..");
		uart_rx_buf_rsp(uart_dev, uart_msgq_buff, UART_BUF_SIZE);
		break;
	}
	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		//printk("evt->data.rx_buf = %s\r\n", evt->data.rx_buf.buf);
		break;

	default:
		break;
	}
} 

#define UART_STACK_SIZE     512
K_THREAD_STACK_DEFINE(uart_stack, UART_STACK_SIZE);
struct k_thread uart_thread;

static uart_recv_callback_t uart_recv_callback = NULL; 
 
static void uart_read_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("<%s>, uart1...", __func__);
	struct uart_event_rx rx;
	rx.buf = k_malloc(UART_BUF_SIZE);
	k_msgq_init(&uart_msgq, uart_msgq_buff, UART_BUF_SIZE, 2);
	while (true) {
		uint16_t len = 0;
		uint8_t  buff[UART_BUF_SIZE];
		k_msgq_get(&uart_msgq, &rx, K_FOREVER);  // 第一包数据
		len = rx.len;
		if (k_msgq_get(&uart_msgq, &rx, K_MSEC(10)) == 0) {  // 等待第二包数据
			//LOG_INF("rx = %s | %d", rx.buf, rx.len);
			memcpy(buff, rx.buf + (UART_BUF_SIZE - len), len);  // tail.
			memcpy(buff + len, rx.buf, rx.len);   // head.
			len += rx.len;
		} else {  // 单包
			memcpy(buff, rx.buf + rx.offset, rx.len);
		}
		buff[len] = '\0';
		// LOG_INF("rx_buff2 = %s | %d", buff, len);
		// LOG_HEXDUMP_INF(buff, len, "hex");

		uart_recv_callback(UART0_PORT, buff, len);
	}
}
 
int hal_uart_init(uart_recv_callback_t cb)
{
    int err;
 
	if (!device_is_ready(uart_dev)) {
		LOG_WRN("UART device not found!");
		return -ENODEV;
	}
 
	err = uart_callback_set(uart_dev, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}
 
	err = uart_rx_enable(uart_dev, uart_msgq_buff, UART_BUF_SIZE, UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		return err;
	}
 
    uart_recv_callback = cb;
 
    k_thread_create(&uart_thread, uart_stack, UART_STACK_SIZE, uart_read_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
 
    LOG_INF("%s ok...", __func__);
	return 0;
}

 