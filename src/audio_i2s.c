/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <nrfx_i2s.h>
#include <nrfx_clock.h>

// #include "audio_sync_timer.h"

#ifdef CONFIG_NRF5340_AUDIO
 
#define I2S_NL 	DT_NODELABEL(i2s0)
PINCTRL_DT_DEFINE(I2S_NL);

/* Audio clock - nRF5340 Analog Phase-Locked Loop (APLL) */
#define APLL_FREQ_CENTER 	39854
#define APLL_FREQ_MIN	 	36834
#define APLL_FREQ_MAX	 	42874

#define HFCLKAUDIO_12_288_MHZ 	APLL_FREQ_CENTER   // 39854

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;
 
#if CONFIG_AUDIO_SAMPLE_RATE_16000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_384X
#elif CONFIG_AUDIO_SAMPLE_RATE_24000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_256X
#elif CONFIG_AUDIO_SAMPLE_RATE_48000_HZ
#define CONFIG_AUDIO_RATIO NRF_I2S_RATIO_128X
#else
#error "Current AUDIO_SAMPLE_RATE_HZ setting not supported"
#endif

#ifndef NRFX_I2S0_INST_IDX		
#define NRFX_I2S0_INST_IDX	0
#endif
 
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(0);


static const nrfx_i2s_config_t i2s_cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.sdin_pin = NRF_I2S_PIN_NOT_CONNECTED,
	.mck_pin = NRF_I2S_PIN_NOT_CONNECTED,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = CONFIG_AUDIO_RATIO,
	.mck_setup = NRF_I2S_MCK_32MDIV2,
#if (CONFIG_AUDIO_BIT_DEPTH_16)
	.sample_width = NRF_I2S_SWIDTH_16BIT,
#elif (CONFIG_AUDIO_BIT_DEPTH_24)
	.sample_width = NRF_I2S_SWIDTH_24BIT,
#elif (CONFIG_AUDIO_BIT_DEPTH_32)
	.sample_width = NRF_I2S_SWIDTH_32BIT,
#else
#error Invalid bit depth selected
#endif /* (CONFIG_AUDIO_BIT_DEPTH_16) */
	.channels = NRF_I2S_CHANNELS_RIGHT,
	.clksrc = NRF_I2S_CLKSRC_ACLK,
	.enable_bypass = false,
};

static i2s_blk_comp_callback_t i2s_blk_comp_callback;
 
// 调用 audio_i2s_start(i2s_tx_buf, NULL)， 就会一直刷新这个回调后
static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	uint32_t frame_start_ts = 0x01;//audio_sync_timer_capture_get();

	if ((status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) && i2s_blk_comp_callback) {
		i2s_blk_comp_callback(frame_start_ts, released_bufs->p_rx_buffer, released_bufs->p_tx_buffer);
	} else {
		printk("<i2s_comp_handler>, status = %d\n", status);
	}
}

nrfx_err_t audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);
#if 0
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}
#endif

	const nrfx_i2s_buffers_t i2s_buf = { 
		.p_rx_buffer = rx_buf,
		.p_tx_buffer = (uint32_t *)tx_buf,
	};

	nrfx_err_t ret;
	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);
	return ret;
}

nrfx_err_t audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_IDLE);
#if 0
	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == GATEWAY)) {
		__ASSERT_NO_MSG(rx_buf != NULL);
	}

	if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
		__ASSERT_NO_MSG(tx_buf != NULL);
	}
#endif
	const nrfx_i2s_buffers_t i2s_buf = { 
		.p_rx_buffer = rx_buf,
		.p_tx_buffer = (uint32_t *)tx_buf, 
	};

	nrfx_err_t ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, I2S_SAMPLES_NUM, 0);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	state = AUDIO_I2S_STATE_STARTED;

	printk("audio_i2s_start... %d\n", I2S_SAMPLES_NUM);

	return ret;
}

void audio_i2s_stop(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);

	nrfx_i2s_stop(&i2s_inst);

	state = AUDIO_I2S_STATE_IDLE;
}

void audio_i2s_blk_comp_cb_register(i2s_blk_comp_callback_t blk_comp_callback)
{
	i2s_blk_comp_callback = blk_comp_callback;
}


void hfclkaudio_set(uint16_t freq_value)
{
	uint16_t freq_val = freq_value;

	freq_val = MIN(freq_val, APLL_FREQ_MAX);
	freq_val = MAX(freq_val, APLL_FREQ_MIN);

	nrfx_clock_hfclkaudio_config_set(freq_val);
}

ISR_DIRECT_DECLARE(i2s_isr_handler) 
{
	nrfx_i2s_0_irq_handler();
	ISR_DIRECT_PM();
		
	return 1; 
}

void audio_i2s_init(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_UNINIT);

	nrfx_err_t ret;

	hfclkaudio_set(HFCLKAUDIO_12_288_MHZ);

	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;

	/* Wait for ACLK to start */
	while (!NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED) {
		k_sleep(K_MSEC(1));
	}

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	__ASSERT_NO_MSG(ret == 0);

	// 会报错：Has IRQ_CONNECT or IRQ_DIRECT_CONNECT accidentally been invoked on the same irq multiple times?
	// IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_0_irq_handler, 0);
	IRQ_DIRECT_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), i2s_isr_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));
 
	ret = nrfx_i2s_init(&i2s_inst, &i2s_cfg, i2s_comp_handler);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);
 
	state = AUDIO_I2S_STATE_IDLE;

	printk("nrfx_i2s_init NRFX_SUCCESS...\r\n");
}
#endif
