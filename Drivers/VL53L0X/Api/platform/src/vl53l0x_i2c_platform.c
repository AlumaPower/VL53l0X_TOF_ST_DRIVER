/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform.c
 * \brief  Code function defintions for Ewok Platform Layer
 *
 */


#include <stdio.h>    // sprintf(), vsnprintf(), printf()
#ifdef _MSC_VER
#define snprintf _snprintf
#endif
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif


UART_HandleTypeDef hlpuart1;
char  debug_string[VL53L0X_MAX_STRING_LENGTH_PLT];

#define I2C_READ 1
#define I2C_WRITE 0
#define MIN_COMMS_VERSION_MAJOR     1
#define MIN_COMMS_VERSION_MINOR     8
#define MIN_COMMS_VERSION_BUILD     1
#define MIN_COMMS_VERSION_REVISION  0

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

I2C_HandleTypeDef hi2c1;
// bool_t _check_min_version(void)
// {
//     bool_t min_version_comms_dll = FALSE;
//     int32_t status   = STATUS_OK;
//     COMMS_VERSION_INFO comms_version_info;


//     status = RANGING_SENSOR_COMMS_Get_Version( &comms_version_info );
//     if(status != STATUS_OK)
//     {
//         RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
//     }
//     else
//     {
//         // combined check of major, minor and build
//         if (
//             (comms_version_info.major > MIN_COMMS_VERSION_MAJOR) ||
//             ((comms_version_info.major == MIN_COMMS_VERSION_MAJOR) && (comms_version_info.minor > MIN_COMMS_VERSION_MINOR)) ||
//             ((comms_version_info.major == MIN_COMMS_VERSION_MAJOR) && (comms_version_info.minor == MIN_COMMS_VERSION_MINOR) && (comms_version_info.build >= MIN_COMMS_VERSION_BUILD))
//             )
//         {
//             min_version_comms_dll = TRUE;
//         }

//         // subsequent check of svn revision
//         if ( comms_version_info.revision < MIN_COMMS_VERSION_REVISION )
//         {
//             min_version_comms_dll = FALSE;
//         }
//     }

//     return min_version_comms_dll;
// }

// int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
// {
//     static const int32_t cmax_devices = 4;

//     int32_t status   = STATUS_OK;
//     int32_t argc     = 1;
//     DWORD devCount = 0;

//     char *pargv    = (char *)malloc(argc);
//     uint8_t *pdev_ids = (uint8_t *)malloc(cmax_devices);

//     // patch
//     char options_speed_buffer[256];
//     char* options_speed = &options_speed_buffer[0];
//     char* options[] = {"I2C_SPEED_KHZ=400"};
//     sprintf(options_speed,"I2C_SPEED_KHZ=%d",comms_speed_khz);

//     if (!_check_min_version())
//     {
//         status = STATUS_DLL_INIT_FAILED;
//     }
//     if(status == STATUS_OK)
//     {
//         status = RANGING_SENSOR_COMMS_Enum_Devices(cmax_devices, pdev_ids, &devCount);
//     }

//     if(status != STATUS_OK)
//     {
//         RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
//     }

//     if(status == STATUS_OK)
//     {
// //        status = RANGING_SENSOR_COMMS_Init_V2W8((uint32_t)pdev_ids[0], 0, &pargv);
//         status = RANGING_SENSOR_COMMS_Init_V2W8(0, 1, options);

//         if(status != STATUS_OK)
//         {
//             RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
//         }

// 		if(status == STATUS_OK)
// 		{
// 			status = RANGING_SENSOR_COMMS_Set_I2C_Bus_Speed(comms_speed_khz);

// 			if(status != STATUS_OK)
// 			{
// 				RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
// 			}
// 		}
//     }

//     free(pdev_ids);
//     free(pargv);

//     return status;
// }

// int32_t VL53L0X_comms_close(void)
// {
//     int32_t status = STATUS_OK;

//     RANGING_SENSOR_COMMS_Fini_V2W8();

//     if(status != STATUS_OK)
//     {
//         RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
//     }

//     return status;
// }

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    uint8_t msg[count + 1];
// #ifdef VL53L0X_LOG_ENABLE
    int32_t i = 0;
    char value_as_str[VL53L0X_MAX_STRING_LENGTH_PLT];
    char *pvalue_as_str;
    uint8_t MSG[100] = {'\0'};
    pvalue_as_str =  value_as_str;
    msg[0] = index;
    for(i = 0 ; i < count ; i++)
    {
        msg[i+1] = pdata[i];
        sprintf(pvalue_as_str,"%02X", *(pdata+i));

        pvalue_as_str += 2;
    }
    sprintf(MSG, "Write reg : 0x%04X, Val : 0x%s\n", msg[0], value_as_str);
//	 HAL_UART_Transmit(&hlpuart1, MSG, sizeof(MSG) , 100);

// #endif

    // i2c_write_blocking(i2c_default, address, &index, 1, true);
	 HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)((address << 1) | I2C_WRITE), msg, count+1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
//    HAL_I2C_Mem_Write(&hi2c1, (uint16_t)address, index,
//                                        sizeof(index), pdata, count, 100);
//    i2c_write_blocking(i2c_default, address, msg, count+1, false);

    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;

// #ifdef VL53L0X_LOG_ENABLE
    int32_t      i = 0;
    char   value_as_str[VL53L0X_MAX_STRING_LENGTH_PLT];
    char *pvalue_as_str;
// #endif
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)((address << 1) | I2C_WRITE), &index, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((address << 1) | I2C_READ), pdata, count, 100);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    }
//   i2c_write_blocking(i2c_default, address, &index, 1, true);
//    i2c_read_blocking(i2c_default, address, pdata, count, false);

// #ifdef VL53L0X_LOG_ENABLE
    // Build  value as string;
    pvalue_as_str =  value_as_str;
    uint8_t msg[100] = {'\0'};
    for(i = 0 ; i < count ; i++)
    {
        sprintf(pvalue_as_str, "%02X", *(pdata+i));
        pvalue_as_str += 2;
    }
    sprintf(msg, "Write reg : 0x%04X, Val : 0x%s\n", index, value_as_str);
//	 HAL_UART_Transmit(&hlpuart1, msg, sizeof(msg) , 100);
// #endif
    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t  buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}

// int32_t VL53L0X_platform_wait_us(int32_t wait_us)
// {
//     int32_t status = STATUS_OK;
//     float wait_ms = (float)wait_us/1000.0f;

//     /*
//      * Use windows event handling to perform non-blocking wait.
//      */
//     HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
//     WaitForSingleObject(hEvent, (int)(wait_ms + 0.5f));

// #ifdef VL53L0X_LOG_ENABLE
//     trace_i2c("Wait us : %6d\n", wait_us);
// #endif

//     return status;

// }


// int32_t VL53L0X_wait_ms(int32_t wait_ms)
// {
//     int32_t status = STATUS_OK;

//     /*
//      * Use windows event handling to perform non-blocking wait.
//      */
//     HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
//     WaitForSingleObject(hEvent, wait_ms);

// #ifdef VL53L0X_LOG_ENABLE
//     trace_i2c("Wait ms : %6d\n", wait_ms);
// #endif

//     return status;

// }


int32_t VL53L0X_set_gpio(uint8_t level)
{
    int32_t status = STATUS_OK;
    //status = VL53L0X_set_gpio_sv(level);
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Set GPIO = %d;\n", level);
#endif

    return status;

}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    int32_t status = STATUS_OK;
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Get GPIO = %d;\n", *plevel);
#endif
    return status;
}


int32_t VL53L0X_release_gpio(void)
{
    int32_t status = STATUS_OK;
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Releasing force on GPIO\n");
#endif
    return status;

}

// int32_t VL53L0X_cycle_power(void)
// {
//     int32_t status = STATUS_OK;
// #ifdef VL53L0X_LOG_ENABLE
//     trace_i2c("// cycle sensor power\n");
// #endif
//     status = RANGING_SENSOR_COMMS_Cycle_Sensor_Power();

// 	if(status != STATUS_OK)
//     {
//         RANGING_SENSOR_COMMS_Get_Error_Text(debug_string);
//     }

// 	return status;
// }


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
       *ptimer_freq_hz = 0;
       return STATUS_FAIL;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
       *ptimer_count = 0;
       return STATUS_FAIL;
}
