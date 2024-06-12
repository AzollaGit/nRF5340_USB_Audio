/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_usb.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>

#include "macros_common.h"
#include "data_fifo.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, LOG_LEVEL_DBG);

#define USB_FRAME_SIZE_STEREO    (((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) * 2)

static struct data_fifo *fifo_tx;
 
#include "hal_i2s.h"
#include "audio_i2s.h"
#define FIFO_RX_BLOCK_COUNT (CONFIG_FIFO_FRAME_SPLIT_NUM * CONFIG_FIFO_RX_FRAME_COUNT)
DATA_FIFO_DEFINE(fifo_rx, FIFO_RX_BLOCK_COUNT, WB_UP(BLOCK_SIZE_BYTES));
 
static uint32_t rx_num_overruns;
static bool rx_first_data;
static bool tx_first_data;

K_SEM_DEFINE(sem, 0, 1);
 

#if (CONFIG_STREAM_BIDIRECTIONAL)
static uint32_t tx_num_underruns;

NET_BUF_POOL_FIXED_DEFINE(pool_out, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_FRAME_SIZE_STEREO, 8, net_buf_destroy);

static void data_write(const struct device *dev)
{
	int ret;

	if (fifo_tx == NULL) {
		return;
	}

	void *data_out;
	size_t data_out_size;
	struct net_buf *buf_out;

	buf_out = net_buf_alloc(&pool_out, K_NO_WAIT);

	ret = data_fifo_pointer_last_filled_get(fifo_tx, &data_out, &data_out_size, K_NO_WAIT);
	if (ret) {
		tx_num_underruns++;
		if ((tx_num_underruns % 100) == 1) {
			LOG_WRN("USB TX underrun. Num: %d", tx_num_underruns);
		}
		net_buf_unref(buf_out);

		return;
	}

	memcpy(buf_out->data, data_out, data_out_size);
	data_fifo_block_free(fifo_tx, data_out);

	if (data_out_size == usb_audio_get_in_frame_size(dev)) {
		ret = usb_audio_send(dev, buf_out, data_out_size);
		if (ret) {
			LOG_WRN("USB TX failed, ret: %d", ret);
			net_buf_unref(buf_out);
		}

	} else {
		LOG_WRN("Wrong size write: %d", data_out_size);
	}

	if (!tx_first_data) {
		LOG_INF("USB TX first data sent.");
		tx_first_data = true;
	}
}
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */

uint8_t usb_rx_buff[1920];
size_t  usb_rx_len = 0;
static void usb_data_received(const struct device *dev, struct net_buf *buffer, size_t size)
{
#if 1
	memcpy(usb_rx_buff + usb_rx_len, buffer->data, size);
	net_buf_unref(buffer);
	usb_rx_len += size;
	if (usb_rx_len >= 1920 - 192) {
		usb_rx_len = 0;
		k_sem_give(&sem);
	}
	return;
#endif

	int ret;
	void *data_in;
 
	if (buffer == NULL || size == 0 || buffer->data == NULL) {
		/* This should never happen */
		ERR_CHK(-EINVAL);
	}
 
	/* Receive data from USB */
	if (size != USB_FRAME_SIZE_STEREO) {
		LOG_WRN("Wrong length: %d", size);
		net_buf_unref(buffer);
		return;
	}

	//LOG_INF("size = %d", size);

	ret = data_fifo_pointer_first_vacant_get(&fifo_rx, &data_in, K_NO_WAIT);

	/* RX FIFO can fill up due to retransmissions or disconnect */
	if (ret == -ENOMEM) {
		void *temp;
		size_t temp_size;

		rx_num_overruns++;
		if ((rx_num_overruns % 100) == 1) {
			LOG_WRN("USB RX overrun. Num: %d", rx_num_overruns);
		}

		ret = data_fifo_pointer_last_filled_get(&fifo_rx, &temp, &temp_size, K_NO_WAIT);
		ERR_CHK(ret);

		data_fifo_block_free(&fifo_rx, temp);

		ret = data_fifo_pointer_first_vacant_get(&fifo_rx, &data_in, K_NO_WAIT);
	}

	ERR_CHK_MSG(ret, "RX failed to get block");

	memcpy(data_in, buffer->data, size);

	// LOG_HEXDUMP_INF(buffer->data, size, "fifo_rx");

	ret = data_fifo_block_lock(&fifo_rx, &data_in, size);
	ERR_CHK_MSG(ret, "Failed to lock block");
 
	net_buf_unref(buffer);
 
	// return;
	static uint8_t rx_cnt = 0;
	if (++rx_cnt >= 10) {
		rx_cnt = 0;
		k_sem_give(&sem);
	}
 	
}

static void usb_feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt)
{
	// LOG_DBG("Control selector %d for channel %d updated", evt->cs, evt->channel);
	switch (evt->cs) {
	case USB_AUDIO_FU_MUTE_CONTROL:
		/* Fall through */
		LOG_HEXDUMP_INF(evt->val, evt->val_len, "USB_AUDIO_FU_MUTE_CONTROL");
		break;
	case USB_AUDIO_FU_VOLUME_CONTROL:
		LOG_HEXDUMP_INF(evt->val, evt->val_len, "USB_AUDIO_FU_VOLUME_CONTROL");
		break;
	default:
		LOG_HEXDUMP_INF(evt->val, evt->val_len, "default");
		break;
	}
}

static const struct usb_audio_ops ops = {
	.data_received_cb = usb_data_received,
	.feature_update_cb = usb_feature_update,
#if (CONFIG_STREAM_BIDIRECTIONAL)
	.data_request_cb = data_write,
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */
};
 

void audio_usb_stop(void)
{
	rx_first_data = false;
	tx_first_data = false;
	fifo_tx = NULL;
}

int audio_usb_disable(void)
{
	int ret;

	audio_usb_stop();

	ret = usb_disable();
	if (ret) {
		LOG_ERR("Failed to disable USB");
		return ret;
	}

	return 0;
}

void usb_pipeline_rx_fifo(char *pcm_raw_data)
{
	int ret;
	uint32_t blocks_alloced_num;
	uint32_t blocks_locked_num;
 
	void *tmp_pcm_raw_data[CONFIG_FIFO_FRAME_SPLIT_NUM];
 
	size_t pcm_raw_len;
	size_t pcm_block_size;
	/* Get PCM data from I2S */
	/* Since one audio frame is divided into a number of
		* blocks, we need to fetch the pointers to all of these
		* blocks before copying it to a continuous area of memory
		* before sending it to the encoder
		*/
	ret = data_fifo_num_used_get(&fifo_rx, &blocks_alloced_num, &blocks_locked_num);
	if (ret || blocks_alloced_num == 0 || blocks_locked_num == 0) {
		LOG_WRN("[%d]RX alloced: %d, locked: %d", ret, blocks_alloced_num, blocks_locked_num);
		return;
	}
	 
	pcm_raw_len = 0;
	//LOG_DBG(COLOR_CYAN "RX alloced: %d, locked: %d" COLOR_RESET, blocks_alloced_num, blocks_locked_num);
	for (uint8_t i = 0; i < CONFIG_FIFO_FRAME_SPLIT_NUM; i++) {
		ret = data_fifo_pointer_last_filled_get(&fifo_rx, &tmp_pcm_raw_data[i], &pcm_block_size, K_FOREVER);
		ERR_CHK(ret);
		memcpy(pcm_raw_data + pcm_raw_len, tmp_pcm_raw_data[i], pcm_block_size);
		pcm_raw_len += pcm_block_size;
		data_fifo_block_free(&fifo_rx, tmp_pcm_raw_data[i]);
	}
	//LOG_INF("pcm_raw_len = %d", pcm_raw_len);
}
 
/*
 * This handler function is called every time I2S needs new buffers for
 * TX and RX data.
 *
 * The new TX data buffer is the next consumer block in out.fifo.
 *
 * The new RX data buffer is the first empty slot of in.fifo.
 * New I2S RX data is located in rx_buf_released, and is locked into
 * the in.fifo message queue.
 */
static void audio_datapath_i2s_blk_complete(uint32_t frame_start_ts_us, uint32_t *rx_buf_released, uint32_t const *tx_buf_released)
{
	if (k_sem_take(&sem, K_NO_WAIT) == 0) {
		/* Test tone takes over audio stream */
		audio_i2s_set_next_buf((const uint8_t *)usb_rx_buff, NULL);
		//printk("i2s_wri.\n");
	}
}
 

// 注意：USB_Audio 只支持48K, 2通道立体声！
int audio_usb_init(void)
{
	int ret;
	const struct device *hs_dev = DEVICE_DT_GET(DT_NODELABEL(hs_0));

	if (!device_is_ready(hs_dev)) {
		LOG_ERR("USB Headset Device not ready");
		return -EIO;
	}

	usb_audio_register(hs_dev, &ops);

	ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("Failed to enable USB");
		return ret;
	}
 
	ret = data_fifo_init(&fifo_rx);
	ERR_CHK_MSG(ret, "Failed to set up rx FIFO");
 
	audio_i2s_blk_comp_cb_register(audio_datapath_i2s_blk_complete);
	audio_i2s_init();
	
	k_sleep(K_MSEC(1000));
	audio_i2s_start((const uint8_t *)usb_rx_buff, NULL);
 
	LOG_INF("Ready for USB host to send/receive.");
 
	return 0;
}


static struct k_poll_signal encoder_sig;
 
void audio_system_encoder_start(void)
{
	LOG_DBG("Encoder started");
	// k_poll_signal_raise(&encoder_sig, 0);
	k_sem_give(&sem);
}

void audio_system_encoder_stop(void)
{
	k_poll_signal_reset(&encoder_sig);
}


 
static void encoder_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	uint32_t blocks_alloced_num;
	uint32_t blocks_locked_num;
 
	void *tmp_pcm_raw_data[CONFIG_FIFO_FRAME_SPLIT_NUM];
	uint8_t pcm_raw_data[FRAME_SIZE_BYTES];

	size_t pcm_raw_len = 0;
 
	size_t pcm_block_size;

	k_sleep(K_MSEC(1000));

	LOG_INF("encoder_thread...");
 
	while (1) {
 
		/* Don't start encoding until the stream needing it has started */
		k_sem_take(&sem, K_FOREVER);
 
		/* Get PCM data from I2S */
		/* Since one audio frame is divided into a number of
		 * blocks, we need to fetch the pointers to all of these
		 * blocks before copying it to a continuous area of memory
		 * before sending it to the encoder
		 */
		ret = data_fifo_num_used_get(&fifo_rx, &blocks_alloced_num, &blocks_locked_num);
		if (ret || blocks_alloced_num == 0 || blocks_locked_num == 0) {
			LOG_WRN("[%d]RX alloced: %d, locked: %d", ret, blocks_alloced_num, blocks_locked_num);
			continue;
		}
		
		pcm_raw_len = 0;
		//LOG_DBG(COLOR_CYAN "RX alloced: %d, locked: %d" COLOR_RESET, blocks_alloced_num, blocks_locked_num);
		for (uint8_t i = 0; i < CONFIG_FIFO_FRAME_SPLIT_NUM; i++) {
			ret = data_fifo_pointer_last_filled_get(&fifo_rx, &tmp_pcm_raw_data[i], &pcm_block_size, K_FOREVER);
			//ERR_CHK(ret);
			if (ret) LOG_WRN("RET = %d", ret);
			memcpy(pcm_raw_data + pcm_raw_len, tmp_pcm_raw_data[i], pcm_block_size);
			pcm_raw_len += pcm_block_size;
			data_fifo_block_free(&fifo_rx, tmp_pcm_raw_data);
		}

		//LOG_INF("pcm_raw_len = %d", pcm_raw_len);

		if (blocks_locked_num > 0) {
			/*** Data exchange ***/
			audio_i2s_set_next_buf((const uint8_t *)pcm_raw_data, NULL);
		}
	}
}
 
// K_THREAD_DEFINE(encoder_thread_tid, 4 * 1024, encoder_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);
