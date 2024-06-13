/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>

#include "macros_common.h"
#include "data_fifo.h"

#include "tone.h"
#include "contin_array.h"
#include "pcm_stream_channel_modifier.h"

#include "audio_usb.h"
#include "hal_i2s.h"
#include "audio_i2s.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, LOG_LEVEL_DBG);

#define USB_FRAME_SIZE_STEREO    (((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) * 2)

static struct data_fifo *fifo_tx;
DATA_FIFO_DEFINE(fifo_rx, (CONFIG_FIFO_FRAME_SPLIT_NUM * CONFIG_FIFO_RX_FRAME_COUNT), WB_UP(BLOCK_SIZE_BYTES));

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

}
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */

static uint8_t pcm_raw_data[USB_FRAME_SIZE_STEREO * 2];
static size_t  pcm_raw_len = 0;

static uint8_t pcm_left_data[USB_FRAME_SIZE_STEREO];
static uint8_t pcm_right_data[USB_FRAME_SIZE_STEREO];

uint16_t usb_enomem_cnt = 0;

/*
 * This handler function is called every time I2S needs new buffers for
 * TX and RX data.
 *
 * The new TX data buffer is the next consumer block in out.fifo.
 */
static void audio_datapath_i2s_blk_complete(uint32_t frame_start_ts_us, uint32_t *rx_buf_released, uint32_t const *tx_buf_released)
{
	int ret;
	void *tmp_pcm_raw_data[2];
	size_t pcm_block_size;
 
	for (uint8_t i = 0; i < 2; i++) {
		ret = data_fifo_pointer_last_filled_get(&fifo_rx, &tmp_pcm_raw_data[i], &pcm_block_size, K_NO_WAIT);
		if (ret) return;
		memcpy(pcm_raw_data + pcm_raw_len, tmp_pcm_raw_data[i], pcm_block_size);
		pcm_raw_len += pcm_block_size;
		data_fifo_block_free(&fifo_rx, tmp_pcm_raw_data[i]);
		if (pcm_raw_len == USB_FRAME_SIZE_STEREO * 2) {
			size_t output_size;
			pscm_two_channel_split(pcm_raw_data, pcm_raw_len, 16, pcm_left_data, pcm_right_data, &output_size);
			audio_i2s_set_next_buf((const uint8_t *)pcm_right_data, NULL);
			pcm_raw_len = 0;
			return;
		}
	}
}

static void usb_data_received(const struct device *dev, struct net_buf *buffer, size_t size)
{
	int ret;
	void *data_in;

	ret = data_fifo_pointer_first_vacant_get(&fifo_rx, &data_in, K_NO_WAIT);
	/* RX FIFO can fill up due to retransmissions or disconnect */
	if (ret == -ENOMEM) {
		//LOG_WRN("ENOMEM");
		usb_enomem_cnt++;
		data_fifo_empty(&fifo_rx);
	} else {
		memcpy(data_in, buffer->data, size);
		ret = data_fifo_block_lock(&fifo_rx, &data_in, size);
		ERR_CHK_MSG(ret, "Failed to lock block");
	}

	net_buf_unref(buffer);
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
	
	k_sleep(K_MSEC(100));
	
	audio_i2s_start((const uint8_t *)pcm_right_data, NULL);
 
	LOG_INF("Ready for USB host to send/receive.");

	while (1) {
		k_sleep(K_MSEC(1000));
		printk("usb_enomem_cnt = %d\n", usb_enomem_cnt);
	}
 
	return 0;
}
