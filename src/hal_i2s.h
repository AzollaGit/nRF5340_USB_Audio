#ifndef INCLUDE_HAL_I2S_H_
#define INCLUDE_HAL_I2S_H_

#ifdef __cplusplus
extern "C" {
#endif
 
int hal_i2s_init(void);

int hal_i2s_write(void *mem_block, size_t block_size);

void i2s_process_block_data(int16_t *echo_block, void *mem_block, uint32_t number_of_samples);
 
#ifdef __cplusplus
}
#endif

#endif
