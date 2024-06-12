/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <string.h>
 
 
/** I2S 总线接口！
 * SCLK -> BCLK   (Serial Clock)：串行时钟线，也称位时钟(BCLK)，数字音频的每一位数据都对应有一个CK脉冲，它的频率为：2*采样频率*量化位数，2代表左右两个通道数据。
 * WS   -> LRCK   (Word Select)：字段选择线，也称帧时钟(LRC)线，表明当前传输数据的声道，不同标准有不同的定义。WS线的频率等于采样频率(FS)。
 * CLKIN-> MCLK   (Main Clock): 主时钟(MCK)
 * SDO  -> SDOUT  (Serial Data)：串行数据线，用于发送或接收两个时分复用的数据通道上的数据(仅半双工模式)，如果是全双工模式，该信号仅用于发送数据。
 * SDI  -> SDIN     
 */

#define I2S_TX_NODE  DT_NODELABEL(i2s0)
#define I2S_RX_NODE  DT_NODELABEL(i2s0)
static const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);
static const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
 
#define SAMPLE_FREQUENCY    48000   //44100
#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   ((SAMPLE_FREQUENCY / 100) * NUMBER_OF_CHANNELS)
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000
  

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);
 

bool i2s_prepare_transfer(const struct device *i2s_dev_tx)
{
	int ret;
 
	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, BLOCK_SIZE);

		ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	return true;
}


void i2s_process_block_data(int16_t *echo_block, void *mem_block, uint32_t number_of_samples)
{
    for (int i = 0; i < number_of_samples; ++i) {
        int16_t *sample = &((int16_t *)mem_block)[i];
        *sample += echo_block[i];
        echo_block[i] = (*sample) / 2;
    }
}
 

int hal_i2s_write(void *mem_block, size_t block_size)
{
    int ret;
 
    // ret = i2s_buf_write(i2s_dev_tx, mem_block, block_size);
    ret = i2s_write(i2s_dev_tx, mem_block, block_size);
    if (ret < 0) {
        printk("Failed to write data: %d\n", ret);
    }

    // static bool is_start = true;
    // if (is_start) {
    //    is_start = false;
    //     ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    //     if (ret < 0) {
    //         printk("Failed to trigger command %d on TX: %d\n", I2S_TRIGGER_START, ret);
    //         return ret;
    //     }
    // }
 
    return ret;
}

int hal_i2s_init(void)
{
    int ret;
	struct i2s_config config;

	printk("I2S echo sample\n");

	if (!device_is_ready(i2s_dev_tx)) {
		printk("%s is not ready\n", i2s_dev_tx->name);
		return -EPERM;
	}
 
	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &mem_slab;
	config.block_size = BLOCK_SIZE;
	config.timeout = TIMEOUT;
    ret = i2s_configure(i2s_dev_tx, I2S_DIR_BOTH, &config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return ret;
	}
  
    i2s_prepare_transfer(i2s_dev_tx);
 
    return ret;
}
