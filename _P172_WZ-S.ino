//#ifdef USES_P172
/*

  This plug in is written by Dmitry (rel22 ___ inbox.ru)
  Plugin is based upon SenseAir plugin by Daniel Tedenljung info__AT__tedenljungconsulting.com
  Additional features based on https://geektimes.ru/post/285572/ by Gerben (infernix__AT__gmail.com)

  This plugin reads the CO2 value from MH-Z19 NDIR Sensor

  Pin-out:
  Hd o
  SR o   o PWM
  Tx o   o AOT
  Rx o   o GND
  Vo o   o Vin
  (bottom view)
  Skipping pin numbers due to inconsistancies in individual data sheet revisions.
  MHZ19:  Connection:
  VCC     5 V
  GND     GND
  Tx      ESP8266 1st GPIO specified in Device-settings
  Rx      ESP8266 2nd GPIO specified in Device-settings
*/

// Uncomment the following define to enable the detection range commands:
//#define ENABLE_DETECTION_RANGE_COMMANDS

#define PLUGIN_172
#define PLUGIN_ID_172         172
#define PLUGIN_NAME_172       "Gases - HCHO WZ-S"
#define PLUGIN_VALUENAME1_172 "PPB"
#define PLUGIN_VALUENAME2_172 "ug/m3"
#define PLUGIN_READ_TIMEOUT   300
#define WZ_S_SIG 0XFF
#define WZ_S_SIZE 9

#include <ESPeasySerial.h>

/* �����ϴ�ģʽ
 * /---------+---------+---------+---------+---------+---------+---------+---------+---------\
 * | Byte 0  | Byte 1  | Byte 2  | Byte 3  | Byte 4  | Byte 5  | Byte 6  | Byte 7  | Byte 8  |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | Start   |  ����   |  ��λ   | С����  |����Ũ�� |����Ũ�� | ������  | ������  | Checksum|
 * | Byte    |  ����   |         |  λ��   |  ��λ   |  ��λ   |  ��λ   |  ��λ   |         |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | 0xFF    | 0x017   | ppb=0x04| 0x00    |  0x00   | 0x25    | 0x07    | 0xD0    | 0x25    |
 * \---------+---------+---------+---------+---------+---------+---------+---------+---------/
 * �ʴ�ģʽ
 * /---------+---------+---------+---------+---------+---------+---------+---------+---------\
 * | Byte 0  | Byte 1  | Byte 2  | Byte 3  | Byte 4  | Byte 5  | Byte 6  | Byte 7  | Byte 8  |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | Start   |  ����   |����Ũ�� |����Ũ�� |  ����   |  ����   |����Ũ�� |����Ũ�� | Checksum|
 * | Byte    |         |��λug/m3|��λug/m3|         |         | ��λppb | ��λppb |         |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | 0xFF    | 0x86    | 0x00    | 0x2A    | 0x00    | 0x00    | 0x00    |  0x20   |  0x30   |
 * \---------+---------+---------+---------+---------+---------+---------+---------+---------/
 * ����Ũ��ֵ=����Ũ�ȸ�λ*256+����Ũ�ȵ�λ
 * Ũ�ȸ�λ��Ũ�ȵ�λ����16���ƻ���Ϊ10���ƺ��ڴ��뱾��ʽ����
 */

struct P172_data_struct : public PluginTaskData_base {
  P172_data_struct() {
    reset();
    sensorResets = 0;
  }

  ~P172_data_struct() { reset(); }

  void reset() {
    if (easySerial != nullptr) {
      delete easySerial;
      easySerial = nullptr;
    }
    linesHandled = 0;
    checksumFailed = 0;
    nrUnknownResponses = 0;
    ++sensorResets;
  }

  bool init(const int16_t serial_rx, const int16_t serial_tx) {
    if (serial_rx < 0 || serial_tx < 0)
      return false;
    reset();
    easySerial = new ESPeasySerial(serial_rx, serial_tx);
    easySerial->begin(9600);
    lastInitTimestamp = millis();
    initTimePassed = false;
    byte nbBytesSent = send_wzsCmd(0x02);
    if (nbBytesSent != 9) {
      return false;
    }
    return isInitialized();
  }

  bool isInitialized() const {
    return easySerial != nullptr;
  }

  byte calculateChecksum() const {
    byte checksum = 0;
    for (byte i = 1; i < (WZ_S_SIZE - 1); i++)
      checksum += wzsResp[i];
    return ((~checksum)+1);
  }

  size_t send_wzsCmd(byte CommandId)
  {
    if (!isInitialized()) return 0;
    switch(CommandId) {
    	case 0x01:
    		memcpy_P(&wzsResp[0], wzsCmdtoReportMode, sizeof(wzsCmdtoReportMode));
    		break;
    	case 0x02:
    		memcpy_P(&wzsResp[0], wzsCmdtoQueryMode, sizeof(wzsCmdtoQueryMode));
    		break;
    	case 0x00:
    		memcpy_P(&wzsResp[0], wzsCmdRead, sizeof(wzsCmdRead));
    		break;
    }
    if (!initTimePassed) {
      // Allow for 3 minutes of init time.
      initTimePassed = timePassedSince(lastInitTimestamp) > 180000;
    }

    return easySerial->write(wzsResp, sizeof(wzsResp));
  }

  bool read_ppb(unsigned int &ppb, unsigned int &ugm3) {
    if (!isInitialized()) return false;
    //send read PPM command
    byte nbBytesSent = send_wzsCmd(0x00);
    if (nbBytesSent != 9) {
      return false;
    }
    // get response
    memset(wzsResp, 0, sizeof(wzsResp));

    long timer = millis() + PLUGIN_READ_TIMEOUT;
    int counter = 0;
    while (!timeOutReached(timer) && (counter < 9)) {
      if (easySerial->available() > 0) {
        byte value = easySerial->read();
        if ((counter == 0 && value == 0xFF) || counter > 0) {
          wzsResp[counter++] = value;
        }
      } else {
        delay(10);
      }
    }
    if (counter < 9) {
      // Timeout
      return false;
    }
    ++linesHandled;
    if ( !(wzsResp[8] == calculateChecksum()) ) {
      ++checksumFailed;
      return false;
    }
    if (wzsResp[0] == 0xFF && wzsResp[1] == 0x86) {
      //calculate CO2 PPM
      ugm3 = (static_cast<unsigned int>(wzsResp[2]) << 8) + wzsResp[3];
			ppb  = (static_cast<unsigned int>(wzsResp[6]) << 8) + wzsResp[7];
      return true;
    }
    return false;
  }

  bool receivedCommandAcknowledgement(bool& expectReset) {
    expectReset = false;
    if (wzsResp[0] == 0xFF)  {
      switch (wzsResp[1]) {
        case 0x86: // query mode
        	break;
        case 0x17: // report mode
          expectReset = true;
          break;
        default:
          ++nrUnknownResponses;
          return false;
      }
      byte checksum = calculateChecksum();
      return wzsResp[8] == checksum;
    }
    ++nrUnknownResponses;
    return false;
  }

  String getBufferHexDump() {
    String result;
    result.reserve(27);
    for (int i = 0; i < 9; ++i) {
      result += ' ';
      result += String(wzsResp[i], HEX);
    }
    return result;
  }

  byte wzsCmdtoQueryMode[9] = { 0xFF,0x01,0x78,0x41,0x00,0x00,0x00,0x00,0x46 };
  byte wzsCmdtoReportMode[9] ={ 0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47 };
  byte wzsCmdRead[9]        = { 0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79 };

  uint32_t linesHandled = 0;
  uint32_t checksumFailed = 0;
  uint32_t sensorResets = 0;
  uint32_t nrUnknownResponses = 0;
  unsigned long lastInitTimestamp = 0;

  ESPeasySerial *easySerial = nullptr;
  byte wzsResp[9];    // 9 byte response buffer
  bool initTimePassed = false;
};


boolean Plugin_172(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {

    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_172;
        Device[deviceCount].Type = DEVICE_TYPE_DUAL;
        Device[deviceCount].VType = SENSOR_TYPE_TRIPLE;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = 2;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_172);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_172));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_172));
        break;
      }

    case PLUGIN_GET_DEVICEGPIONAMES:
      {
        serialHelper_getGpioNames(event);
        break;
      }

    case PLUGIN_WEBFORM_SHOW_CONFIG:
      {
        string += serialHelper_getSerialTypeLabel(event);
        success = true;
        break;
      }


    case PLUGIN_WEBFORM_LOAD:
      {
        serialHelper_webformLoad(event);
        P172_html_show_stats(event);
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        P172_data_struct *P172_data =
            static_cast<P172_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P172_data) {
          return success;
        }
        serialHelper_webformSave(event);
        success = true;
        break;
      }

    case PLUGIN_INIT:
      {
        initPluginTaskData(event->TaskIndex, new P172_data_struct());
        success = P172_performInit(event);
        break;
      }

    case PLUGIN_EXIT: {
      clearPluginTaskData(event->TaskIndex);
      success = true;
      break;
    }

    case PLUGIN_WRITE:
      {
        P172_data_struct *P172_data =
            static_cast<P172_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P172_data) {
          return success;
        }
        success = true;
        break;
      }

    case PLUGIN_READ:
      {
        P172_data_struct *P172_data =
            static_cast<P172_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P172_data) {
          return success;
        }
        bool expectReset = false;
        unsigned int ppb = 0;
        unsigned int ugm3 = 0;
        if (P172_data->read_ppb(ppb, ugm3)) {
            String log = F("WZ-S: ");
            UserVar[event->BaseVarIndex] = (float)ppb;
            UserVar[event->BaseVarIndex + 1] = (float)ugm3;

            // Log values in all cases
            log += F("PPB value: ");
            log += ppb;
            log += F(" ug/M3 values: ");
            log += ugm3;
            addLog(LOG_LEVEL_INFO, log);
            success = true;
            break;
        }else if (P172_data->receivedCommandAcknowledgement(expectReset))  {
          addLog(LOG_LEVEL_INFO, F("WZ-s: Received command acknowledgment! "));
          if (expectReset) {
            addLog(LOG_LEVEL_INFO, F("Expecting sensor reset..."));
          }
          success = false;
          break;
        // log verbosely anything else that the sensor reports
        } else {
          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("WZ-s: Unknown response:");
            log += P172_data->getBufferHexDump();
            addLog(LOG_LEVEL_INFO, log);
          }
          // Check for stable reads and allow unstable reads the first 3 minutes after reset.
          if (P172_data->nrUnknownResponses > 10 && P172_data->initTimePassed) {
            P049_performInit(event);
          }
          success = false;
          break;
        }
        break;
      }
  }
  return success;
}

bool P172_performInit(struct EventStruct *event) {
  bool success = false;
  const int16_t serial_rx = CONFIG_PIN1;
  const int16_t serial_tx = CONFIG_PIN2;
  P172_data_struct *P172_data =
      static_cast<P172_data_struct *>(getPluginTaskData(event->TaskIndex));
  if (nullptr == P172_data) {
    return success;
  }
  if (P172_data->init(serial_rx, serial_tx)) {
    success = true;
    addLog(LOG_LEVEL_INFO, F("WZ-S: Init OK "));

    //delay first read, because hardware needs to initialize on cold boot
    //otherwise we get a weird value or read error
    schedule_task_device_timer(event->TaskIndex, millis() + 15000);
  }
  return success;
}

void P172_html_show_stats(struct EventStruct *event) {
  P172_data_struct *P172_data =
      static_cast<P172_data_struct *>(getPluginTaskData(event->TaskIndex));
  if (nullptr == P172_data) {
    return;
  }

  addRowLabel(F("Checksum (pass/fail/reset)"));
  String chksumStats;
  chksumStats = P172_data->linesHandled;
  chksumStats += '/';
  chksumStats += P172_data->checksumFailed;
  chksumStats += '/';
  chksumStats += P172_data->sensorResets;
  addHtml(chksumStats);
}

//#endif // USES_P172
