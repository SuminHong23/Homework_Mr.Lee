/*----------------------------------------------------------------------------/
  Lovyan GFX - Graphics library for embedded devices.

Original Source:
 https://github.com/lovyan03/LovyanGFX/

Licence:
 [FreeBSD](https://github.com/lovyan03/LovyanGFX/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)

Contributors:
 [ciniml](https://github.com/ciniml)
 [mongonta0716](https://github.com/mongonta0716)
 [tobozo](https://github.com/tobozo)

Modified heavily for the ESP32 HUB75 DMA library by:
 [mrfaptastic](https://github.com/mrfaptastic)
  
/----------------------------------------------------------------------------*/


#include "esp32_i2s_parallel_dma.h"
#include "esp_log.h"
#include "driver/i2s_common.h"
#include "driver/i2s.h"

#include <sdkconfig.h>

#include "driver/i2s_std.h"
#include "driver/i2s.h"

#define I2S_PORT I2S_NUM_0
#define I2S_CLK_GPIO  18  // WR 핀에 해당
#define I2S_DATA_GPIO 17  // 데이터 핀 중 하나 (예: D0)
#define DMA_BUFFER_SIZE 512

#if defined (CONFIG_IDF_TARGET_ESP32S2)
  #pragma message "Compiling for ESP32-S2"
#else
  #pragma message "Compiling for original ESP32 (released 2016)"  
#endif

#include <driver/gpio.h>
#if (ESP_IDF_VERSION_MAJOR == 5)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <soc/gpio_sig_map.h>
#include <soc/i2s_periph.h> //includes struct and reg


#if defined (ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#include <esp_err.h>
#include <esp_log.h>

// Get CPU freq function.
#include <soc/rtc.h>

uint8_t _dma_buffer_a[DMA_BUFFER_SIZE];  // I2S에 쓸 데이터 버퍼
size_t _dma_buffer_size = DMA_BUFFER_SIZE;

/*

	volatile bool previousBufferFree = true;

	static void IRAM_ATTR i2s_isr(void* arg) {

		// From original Sprite_TM Code
		//REG_WRITE(I2S_INT_CLR_REG(1), (REG_READ(I2S_INT_RAW_REG(1)) & 0xffffffc0) | 0x3f);
		
		// Clear flag so we can get retriggered
		SET_PERI_REG_BITS(I2S_INT_CLR_REG(ESP32_I2S_DEVICE), I2S_OUT_EOF_INT_CLR_V, 1, I2S_OUT_EOF_INT_CLR_S);                      
		
		// at this point, the previously active buffer is free, go ahead and write to it
		previousBufferFree = true;
	}

	bool DRAM_ATTR i2s_parallel_is_previous_buffer_free() {
		return previousBufferFree;
	}
*/

	// Static
	i2s_dev_t* getDev()
	{
	  #if defined (CONFIG_IDF_TARGET_ESP32S2)
		  return &I2S0;
	  #else
		  return (ESP32_I2S_DEVICE == 0) ? &I2S0 : &I2S1;
	  #endif

	}

	// Static
	void _gpio_pin_init(int pin)
	{
		if (pin >= 0)
		{
		  gpio_pad_select_gpio(pin);
		  //gpio_hi(pin);
		  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
		  gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);      // esp32s3 as well?
		}
	}
  
	inline int i2s_parallel_get_memory_width(int port, int width) {
	  switch(width) {
		case 8:
		
		#if !defined (CONFIG_IDF_TARGET_ESP32S2)   
		
			  // Only I2S1 on the legacy ESP32 WROOM MCU supports space saving single byte 8 bit parallel access
			  if(port == 1)
			  {
				return 1;
			  } else {
				return 2;
			  }
		#else 
				return 1;
		#endif

		case 16:
		  return 2;
		case 24:
		  return 4;
		default:
		  return -ESP_ERR_INVALID_ARG;
	  }
	}
  

  void Bus_Parallel16::config(const config_t& cfg)
  {
      _cfg = cfg;
      _dev = getDev();
  }
 
 bool Bus_Parallel16::init(void)
{
    if (_cfg.parallel_width != 16) {
        ESP_LOGE("Bus_Parallel16", "Only 16-bit parallel bus is supported.");
        return false;
    }

    // I2S 설정
    i2s_config_t i2s_config = {
        .sample_rate = _cfg.bus_freq,  // 예: 10 MHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // 하나의 채널만 사용
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
    };

    // I2S 핀 설정 (병렬 버스는 커스텀 라우팅 필요)
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_CLK_GPIO,        // WR 신호로 사용
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = I2S_DATA_GPIO,     // 병렬 데이터 시작 핀 (D0)
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t err;
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE("Bus_Parallel16", "Failed to install I2S driver: %d", err);
        return false;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        ESP_LOGE("Bus_Parallel16", "Failed to set I2S pins: %d", err);
        return false;
    }

    // TODO: 병렬 16비트 버스를 위한 GPIO 매트릭스 설정 필요
    // ESP32-S3의 GPIO Matrix를 사용하여 D0~D15를 I2S 데이터 라인에 매핑하는 작업은 수동으로 추가해야 함

    return true;
}

  void Bus_Parallel16::dma_transfer_start()
{
    size_t bytes_written;
    // 예시: _dma_buffer_a에 데이터가 있다고 가정하고 전송
    i2s_write(I2S_PORT, _dma_buffer_a, _dma_buffer_size, &bytes_written, portMAX_DELAY);

    // 전송 시작 (보통 install 시 자동 시작되므로 생략 가능)
    i2s_start(I2S_PORT);
}

  void Bus_Parallel16::dma_transfer_stop()
{
    i2s_stop(I2S_PORT);
}




  void Bus_Parallel16::release(void)
  {
    if (_dmadesc_a)
    {
      heap_caps_free(_dmadesc_a);
      _dmadesc_a = nullptr;
      _dmadesc_count = 0;
    }

    if (_dmadesc_b)
    {
      heap_caps_free(_dmadesc_b);
      _dmadesc_b = nullptr;
      _dmadesc_count = 0;      
    }
  }

  void Bus_Parallel16::enable_double_dma_desc(void)
  {
    _double_dma_buffer = true;
  }

  // Need this to work for double buffers etc.
  bool Bus_Parallel16::allocate_dma_desc_memory(size_t len)
  {
    if (_dmadesc_a) heap_caps_free(_dmadesc_a); // free all dma descrptios previously
    
    _dmadesc_count = len; 
    _dmadesc_last  = len-1; 

    ESP_LOGI("ESP32/S2", "Allocating memory for %d DMA descriptors.", (int)len);    

    _dmadesc_a= (HUB75_DMA_DESCRIPTOR_T*)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (_dmadesc_a == nullptr)
    {
      ESP_LOGE("ESP32/S2", "ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
      return false;
    }


    if (_double_dma_buffer)
    {
      if (_dmadesc_b) heap_caps_free(_dmadesc_b); // free all dma descrptios previously

      ESP_LOGD("ESP32/S2", "Allocating the second buffer (double buffer enabled).");              

      _dmadesc_b = (HUB75_DMA_DESCRIPTOR_T*)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

      if (_dmadesc_b == nullptr)
      {
        ESP_LOGE("ESP32/S2", "ERROR: Couldn't malloc _dmadesc_b. Not enough memory.");
        _double_dma_buffer = false;
        return false;
      }
    }
    
    _dmadesc_a_idx  = 0;
    _dmadesc_b_idx  = 0;

    ESP_LOGD("ESP32/S2", "Allocating %d bytes of memory for DMA descriptors.", (int)sizeof(HUB75_DMA_DESCRIPTOR_T) * len);       

    return true;

  }

  void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b)
  {
    static constexpr size_t MAX_DMA_LEN = (4096-4);

    if (size > MAX_DMA_LEN)
    {
      size = MAX_DMA_LEN;
      ESP_LOGW("ESP32/S2", "Creating DMA descriptor which links to payload with size greater than MAX_DMA_LEN!");            
    }

    if ( !dmadesc_b )
    {
      if ( (_dmadesc_a_idx+1) > _dmadesc_count) {
        ESP_LOGE("ESP32/S2", "Attempted to create more DMA descriptors than allocated memory for. Expecting a maximum of %u DMA descriptors", (unsigned int)_dmadesc_count);          
        return;
      }
    }

    volatile lldesc_t *dmadesc;
    volatile lldesc_t *next;
    bool eof = false;
    
    if ( (dmadesc_b == true) ) // for primary buffer
    {
        dmadesc      = &_dmadesc_b[_dmadesc_b_idx];

        next = (_dmadesc_b_idx < (_dmadesc_last) ) ? &_dmadesc_b[_dmadesc_b_idx+1]:_dmadesc_b;       
        eof  = (_dmadesc_b_idx == (_dmadesc_last));
    }
    else
    {
        dmadesc      = &_dmadesc_a[_dmadesc_a_idx];

        // https://stackoverflow.com/questions/47170740/c-negative-array-index
        next = (_dmadesc_a_idx < (_dmadesc_last) ) ? _dmadesc_a + _dmadesc_a_idx+1:_dmadesc_a;       
        eof  = (_dmadesc_a_idx == (_dmadesc_last));
    }

    if ( _dmadesc_a_idx == (_dmadesc_last) ) {
      ESP_LOGW("ESP32/S2", "Creating final DMA descriptor and linking back to 0.");             
    } 

    dmadesc->size     = size;
    dmadesc->length   = size;
    dmadesc->buf      = (uint8_t*) data; 
    dmadesc->eof      = eof;         
    dmadesc->sosf     = 0;         
    dmadesc->owner    = 1;         
    dmadesc->qe.stqe_next = (lldesc_t*) next;         
    dmadesc->offset   = 0;       

    if ( (dmadesc_b == true) ) { // for primary buffer
      _dmadesc_b_idx++; 
    } else {
      _dmadesc_a_idx++; 
    }
  
  } // end create_dma_desc_link

  void Bus_Parallel16::flip_dma_output_buffer(int buffer_id) // pass by reference so we can change in main matrixpanel class
  {
	  
      // Setup interrupt handler which is focussed only on the (page 322 of Tech. Ref. Manual)
      // "I2S_OUT_EOF_INT: Triggered when rxlink has finished sending a packet" (when dma linked list with eof = 1 is hit)
  	  
      if ( buffer_id == 1) { 

		//fix _dmadesc_ loop issue #407
		//need to connect the up comming _dmadesc_ not the old one
		_dmadesc_b[_dmadesc_last].qe.stqe_next = &_dmadesc_b[0];	  

		_dmadesc_a[_dmadesc_last].qe.stqe_next = &_dmadesc_b[0]; // Start sending out _dmadesc_b (or buffer 1)
		      
      } else { 

        _dmadesc_a[_dmadesc_last].qe.stqe_next = &_dmadesc_a[0];
	      
        _dmadesc_b[_dmadesc_last].qe.stqe_next = &_dmadesc_a[0]; // Start sending out _dmadesc_a (or buffer 0)
		
      }

/*
      previousBufferFree = false;  
      while (!previousBufferFree);
*/
           


  } // end flip

