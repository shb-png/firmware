
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/uds/uds.h"

#include "main.h"
#include "daq_hub.h"
#include "daq_sd.h"

#define DAQ_BL_CMD_NTP_DATE   0x30
#define DAQ_BL_CMD_NTP_TIME   0x31
#define DAQ_BL_CMD_NTP_GET    0x32

#define DAQ_BL_CMD_HANDSHAKE  0x40
#define DAQ_BL_CMD_LOG_ENABLE 0x41
#define DAQ_BL_CMD_LOG_STATUS 0x42
#define DAQ_BL_CMD_LED_DISCO  0x43

#define DAQ_BL_CMD_LIST_DIR   0x50

#define DAQ_UDS_CMD_RTC_SET_DATE_TIME     0x60
#define DAQ_UDS_CMD_RTC_GET_DATE_TIME     0x61

#define DAQ_UDS_DATE_YEAR_MASK      (0xFF << 24)
#define DAQ_UDS_DATE_MONTH_MASK     (0xFF << 16)
#define DAQ_UDS_DATE_WEEKDAY_MASK   (0xFF << 8)
#define DAQ_UDS_DATE_DAY_MASK       (0xFF)

#define DAQ_UDS_DATE_TIME_DECODE_MSK  (0xFF)

#define DAQ_UDS_DATE_YEAR_OFFSET      (24)
#define DAQ_UDS_DATE_MONTH_OFFSET     (16)
#define DAQ_UDS_DATE_WEEKDAY_OFFSET   (8)
#define DAQ_UDS_DATE_DAY_OFFSET       (0)

#define DAQ_UDS_TIME_OFFSET             (32)
#define DAQ_UDS_TIME_HOURS_OFFSET       (DAQ_UDS_TIME_OFFSET)
#define DAQ_UDS_TIME_MINUTES_OFFSET     (DAQ_UDS_TIME_OFFSET + 8)
#define DAQ_UDS_TIME_SECONDS_OFFSET     (DAQ_UDS_TIME_OFFSET + 16)

#define DAQ_UDS_DATE_TIME_RET_OK_OLD_TIME         (0)
#define DAQ_UDS_DATE_TIME_RET_OK_NEW_TIME         (1)
#define DAQ_UDS_DATE_TIME_RET_TIMESTAMP_GET_FAIL  (2)



static RTC_timestamp_t start_time =
{
    .date = {.month_bcd=RTC_MONTH_FEBRUARY,
             .weekday=RTC_WEEKDAY_TUESDAY,
             .day_bcd=0x27,
             .year_bcd=0x24},
    .time = {.hours_bcd=0x18,
             .minutes_bcd=0x27,
             .seconds_bcd=0x00,
             .time_format=RTC_FORMAT_24_HOUR},
};

typedef struct __attribute__((packed))
{
  uint8_t day;
  uint8_t weekday;
  uint8_t month;
  uint8_t year;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t ret_code;
} uds_resp_time_sync_t;

static void uds_process_cmd_ntp_get(void)
{
    RTC_timestamp_t time;
    if (PHAL_getTimeRTC(&time))
        debug_printf("DAQ time: 20%02d-%02d-%02d %02d:%02d:%02d\n",
            time.date.year_bcd, time.date.month_bcd,
            time.date.day_bcd,  time.time.hours_bcd,
            time.time.minutes_bcd, time.time.seconds_bcd);
}

/**
 * Process DAQ-specific UDS commands
 */
void uds_handle_sub_command_callback(uint8_t cmd, uint64_t data)
{
    switch (cmd)
    {
        case UDS_CMD_SYS_RST:
            daq_shutdown_hook(); // NVIC reset / bootloader if loaded (override sys)
            NVIC_SystemReset(); // Don't want to wait for power off if entered for bootloader mode
            break;

        case DAQ_BL_CMD_HANDSHAKE:
            PHAL_toggleGPIO(ERROR_LED_PORT, ERROR_LED_PIN);
            break;

        case DAQ_BL_CMD_NTP_DATE: // send date first
            start_time.date.day_bcd = data & 0xff;
            start_time.date.weekday = (data >> 8) & 0xf;
            start_time.date.month_bcd = (data >> 12) & 0xff;
            start_time.date.year_bcd = (data >> 20) & 0xff;
            break;
        case DAQ_BL_CMD_NTP_TIME: // then time
            start_time.time.seconds_bcd = data & 0xff;
            start_time.time.minutes_bcd = (data >> 8) & 0xff;
            start_time.time.hours_bcd = (data >> 16) & 0xff;
            PHAL_configureRTC(&start_time, true); // now sync
            break;
        case DAQ_BL_CMD_NTP_GET:
            uds_process_cmd_ntp_get();
            break;
        case DAQ_UDS_CMD_RTC_SET_DATE_TIME:
        {
          RTC_timestamp_t time;
          uds_resp_time_sync_t response;
          if (PHAL_getTimeRTC(&time))
          {
            response.year = time.date.year_bcd;
            response.month = time.date.month_bcd;
            response.weekday = start_time.date.weekday;
            response.day = time.date.day_bcd;
            response.hours = time.time.hours_bcd;
            response.minutes = time.time.minutes_bcd;
            response.seconds = time.time.seconds_bcd;
            response.ret_code = DAQ_UDS_DATE_TIME_RET_OK_OLD_TIME;
          }
          else
          {
            response.year = 0;
            response.month = 0;
            response.weekday = 0;
            response.day = 0;
            response.hours = 0;
            response.minutes = 0;
            response.seconds = 0;
            response.ret_code = DAQ_UDS_DATE_TIME_RET_OK_OLD_TIME;
          }
          udsFrameSend(*(uint64_t *)&response);
          start_time.date.year_bcd = (data >> DAQ_UDS_DATE_YEAR_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.date.month_bcd = (data >> DAQ_UDS_DATE_MONTH_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.date.weekday = (data >> DAQ_UDS_DATE_WEEKDAY_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.date.day_bcd = (data >> DAQ_UDS_DATE_DAY_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.time.hours_bcd = (data >> DAQ_UDS_TIME_HOURS_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.time.minutes_bcd = (data >> DAQ_UDS_TIME_MINUTES_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.time.seconds_bcd = (data >> DAQ_UDS_TIME_SECONDS_OFFSET) & DAQ_UDS_DATE_TIME_DECODE_MSK;
          start_time.time.time_format = RTC_FORMAT_24_HOUR;

          PHAL_configureRTC(&start_time, true);
          break;
        }
        case DAQ_UDS_CMD_RTC_GET_DATE_TIME:
        {
          uds_resp_time_sync_t updatedResponse;
          RTC_timestamp_t time;
          if (PHAL_getTimeRTC(&time))
          {
            updatedResponse.year = time.date.year_bcd;
            updatedResponse.month = time.date.month_bcd;
            updatedResponse.weekday = start_time.date.weekday;
            updatedResponse.day = time.date.day_bcd;
            updatedResponse.hours = time.time.hours_bcd;
            updatedResponse.minutes = time.time.minutes_bcd;
            updatedResponse.seconds = time.time.seconds_bcd;
            updatedResponse.ret_code = DAQ_UDS_DATE_TIME_RET_OK_NEW_TIME;
          }
          else
          {
            updatedResponse.year = time.date.year_bcd;
            updatedResponse.month = time.date.month_bcd;
            updatedResponse.weekday = start_time.date.weekday;
            updatedResponse.day = time.date.day_bcd;
            updatedResponse.hours = time.time.hours_bcd;
            updatedResponse.minutes = time.time.minutes_bcd;
            updatedResponse.seconds = time.time.seconds_bcd;
            updatedResponse.ret_code = DAQ_UDS_DATE_TIME_RET_TIMESTAMP_GET_FAIL;
          }
          udsFrameSend(*(uint64_t *)&updatedResponse);
          break;
        }
    }
}
