#include "CYdLidar.h"
#include "common.h"
#include <map>



using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() : lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_Intensities       = false;
  m_FixedResolution   = false;
  m_Exposure          = false;
  m_Reversion         = false;
  m_AutoReconnect     = true;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  m_SampleRate        = 9;
  m_ScanFrequency     = 7;
  isScanning          = false;
  node_counts         = 720;
  each_angle          = 0.5;
  m_FrequencyOffset   = 0.4;
  m_isMultipleRate    = false;
  m_IgnoreArray.clear();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }
}



/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  bool ret = false;

  if (isScanning) {
    lidarPtr->startMotor();
    ret = true;
  }

  return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
    lidarPtr->stopMotor();
    isScanning = false;
  }

  return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
  if (!lidarPtr) {
    return false;
  }

  result_t op_result;
  device_health healthinfo;

  op_result = lidarPtr->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    if (healthinfo.status == 0) {
      ydlidar::console.message("YDLidar running correctly ! The health status is good");
    } else {
      ydlidar::console.error("YDLidar running correctly ! The health status is bad");
    }

    if (healthinfo.status == 2) {
      ydlidar::console.error("Error, Yd Lidar internal error detected. Please reboot the device to retry.");
      return false;
    } else {
      return true;
    }

  } else {
    ydlidar::console.error("Error, cannot retrieve Yd Lidar health code: %x", op_result);
    return false;
  }

}

bool CYdLidar::getDeviceInfo(int &type) {

  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t ans = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(ans)) {
    ydlidar::console.error("get DeviceInfo Error");
    return false;
  }

  std::string model;
  sampling_rate _rate;
  int _samp_rate = 4;
  int bad = 0;

  m_isMultipleRate = false;
  type = devinfo.model;

  switch (devinfo.model) {
  case YDlidarDriver::YDLIDAR_F4:
    model = "F4";
    break;

  case YDlidarDriver::YDLIDAR_T1:
    model = "T1";
    break;

  case YDlidarDriver::YDLIDAR_F2:
    model = "F2";
    break;

  case YDlidarDriver::YDLIDAR_S4:
    model = "S4";
    break;

  case YDlidarDriver::YDLIDAR_G4: {
    model = "G4";
    ans = lidarPtr->getSamplingRate(_rate);

    if (IS_OK(ans)) {
      switch (m_SampleRate) {
      case 4:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
        break;

      case 8:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
        break;

      case 9:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
      }

      while (_samp_rate != _rate.rate) {
        ans = lidarPtr->setSamplingRate(_rate);

        if (!IS_OK(ans)) {
          bad++;

          if (bad > 5) {
            break;
          }
        }
      }

      switch (_rate.rate) {
      case YDlidarDriver::YDLIDAR_RATE_4K:
        _samp_rate = 4;
        break;

      case YDlidarDriver::YDLIDAR_RATE_8K:
        node_counts = 1440;
        each_angle = 0.25;
        _samp_rate = 8;
        break;

      case YDlidarDriver::YDLIDAR_RATE_9K:
        node_counts = 1440;
        each_angle = 0.25;
        _samp_rate = 9;
        break;

      default:
        break;
      }


    }

  }
  break;

  case YDlidarDriver::YDLIDAR_X4:
    model = "X4";
    break;

  case YDlidarDriver::YDLIDAR_G4PRO:
    model = "G4Pro";
    break;

  case YDlidarDriver::YDLIDAR_F4PRO: {
    model = "F4Pro";
    ans = lidarPtr->getSamplingRate(_rate);

    if (IS_OK(ans)) {
      switch (m_SampleRate) {
      case 4:
        _samp_rate = 0;
        break;

      case 6:
        _samp_rate = 1;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
      }

      while (_samp_rate != _rate.rate) {
        ans = lidarPtr->setSamplingRate(_rate);

        if (!IS_OK(ans)) {
          bad++;

          if (bad > 5) {
            break;
          }
        }
      }

      switch (_rate.rate) {
      case 0:
        _samp_rate = 4;
        break;

      case 1:
        node_counts = 1440;
        each_angle = 0.25;
        _samp_rate = 6;
        break;
      }

    }

  }
  break;

  case  YDlidarDriver::YDLIDAR_G4C:
    model = "G4C";
    break;

  case  YDlidarDriver::YDLIDAR_G10:
    model = "G10";
    _samp_rate = 10;
    break;

  case  YDlidarDriver::YDLIDAR_S4B:
    model = "S4B";
    break;

  case  YDlidarDriver::YDLIDAR_S2:
    model = "S2";
    break;

  case  YDlidarDriver::YDLIDAR_G25:
    model = "G25";
    ans = lidarPtr->getSamplingRate(_rate);

    if (IS_OK(ans)) {
      switch (m_SampleRate) {
      case 8:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
        break;

      case 16:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
        break;

      case 18:
        _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
      }

      while (_samp_rate != _rate.rate) {
        ans = lidarPtr->setSamplingRate(_rate);

        if (!IS_OK(ans)) {
          bad++;

          if (bad > 5) {
            break;
          }
        }
      }

      switch (_rate.rate) {
      case YDlidarDriver::YDLIDAR_RATE_4K:
        _samp_rate = 8;
        node_counts = 1440;
        each_angle = 0.25;
        break;

      case YDlidarDriver::YDLIDAR_RATE_8K:
        node_counts = 2400;
        each_angle = 0.15;
        _samp_rate = 16;
        break;

      case YDlidarDriver::YDLIDAR_RATE_9K:
        node_counts = 2600;
        each_angle = 0.1;
        _samp_rate = 18;
        break;

      default:
        break;
      }


    }

    m_isMultipleRate = true;

    break;

  default:
    model = "Unknown";
    break;
  }

  m_SampleRate = _samp_rate;
  lidarPtr->setMultipleRate(m_isMultipleRate);



  unsigned int maxv = (unsigned int)(devinfo.firmware_version >> 8);
  unsigned int midv = (unsigned int)(devinfo.firmware_version & 0xff) / 10;
  unsigned int minv = (unsigned int)(devinfo.firmware_version & 0xff) % 10;
  ydlidar::console.show("[YDLIDAR] Connection established in [%s]:\n"
                        "Firmware version: %u.%u.%u\n"
                        "Hardware version: %u\n"
                        "Model: %s\n"
                        "Serial: ",
                        m_SerialPort.c_str(),
                        maxv,
                        midv,
                        minv,
                        (unsigned int)devinfo.hardware_version,
                        model.c_str());

  for (int i = 0; i < 16; i++) {
    ydlidar::console.show("%01X", devinfo.serialnum[i] & 0xff);
  }

  ydlidar::console.show("\n");

  ydlidar::console.message("[YDLIDAR INFO] Current Sampling Rate : %dK", _samp_rate);


  if (devinfo.model == YDlidarDriver::YDLIDAR_G4 ||
      devinfo.model == YDlidarDriver::YDLIDAR_F4PRO ||
      devinfo.model == YDlidarDriver::YDLIDAR_G4C ||
      devinfo.model == YDlidarDriver::YDLIDAR_G10 ||
      devinfo.model == YDlidarDriver::YDLIDAR_G25) {
    checkScanFrequency();
  } else {
  }

  return true;


}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  float freq = 7.0f;
  scan_frequency _scan_frequency;
  int hz = 0;

  if (5 <= m_ScanFrequency && m_ScanFrequency <= 12) {
    result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;

    if (IS_OK(ans)) {
      freq = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - freq;

      if (hz > 0) {
        while (hz != 0) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz--;
        }

        freq = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz != 0) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz++;
        }

        freq = _scan_frequency.frequency / 100.0f;
      }
    }

    node_counts = m_SampleRate * 1000 / (m_ScanFrequency - m_FrequencyOffset);
    each_angle = 360.0 / node_counts;
  }

  ydlidar::console.message("[YDLIDAR INFO] Current Scan Frequency : %fHz", freq - m_FrequencyOffset);

  return true;

}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      ydlidar::console.error("Create Driver fail");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    ydlidar::console.error("[CYdLidar] Error, cannot bind to the specified serial port %s",
                           m_SerialPort.c_str());
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!lidarPtr) {
    return false;
  }

  if (lidarPtr->isscanning()) {
    return true;
  }

  std::map<int, bool> checkmodel;
  checkmodel.insert(std::map<int, bool>::value_type(115200, false));
  checkmodel.insert(std::map<int, bool>::value_type(128000, false));
  checkmodel.insert(std::map<int, bool>::value_type(153600, false));
  checkmodel.insert(std::map<int, bool>::value_type(230400, false));
  checkmodel.insert(std::map<int, bool>::value_type(512000, false));

  bool ret = false;
  int m_type = -1;

  while (!ret) {
    ret = getDeviceHealth();

    if (!ret) {
      delay(1000);
    }

    if (!getDeviceInfo(m_type) && !ret) {
      checkmodel[m_SerialBaudrate] = true;
      map<int, bool>::iterator it;

      for (it = checkmodel.begin(); it != checkmodel.end(); ++it) {
        if (it->second) {
          continue;
        }

        lidarPtr->disconnect();
        delete lidarPtr;
        lidarPtr = nullptr;
        m_SerialBaudrate = it->first;

        if (!checkCOMMs()) {
          return false;
        } else {
          break;
        }
      }

      if (it == checkmodel.end()) {
        return false;
      }
    } else {
      ret = true;
      break;
    }
  }

  m_Intensities = false;

  if (m_type == YDlidarDriver::YDLIDAR_S4 || m_type == YDlidarDriver::YDLIDAR_S4B) {
    if (m_SerialBaudrate == 153600 || m_type == YDlidarDriver::YDLIDAR_S4B) {
      m_Intensities = true;
    }

    if (m_Intensities) {
      scan_exposure exposure;
      int cnt = 0;

      while ((lidarPtr->setLowExposure(exposure) == RESULT_OK) && (cnt < 3)) {
        if (exposure.exposure != m_Exposure) {
          ydlidar::console.message("set EXPOSURE MODEL SUCCESS!!!");
          break;
        }

        cnt++;
      }

      if (cnt >= 4) {
        ydlidar::console.warning("set LOW EXPOSURE MODEL FALIED!!!");
      }
    }
  }

  lidarPtr->setIntensities(m_Intensities);

  // start scan...
  result_t s_result = lidarPtr->startScan();

  if (!IS_OK(s_result)) {
    s_result = lidarPtr->startScan();

    if (!IS_OK(s_result)) {
      ydlidar::console.error("[CYdLidar] Error starting scanning mode: %x", s_result);
      isScanning = false;
      return false;
    }
  }

  lidarPtr->setAutoReconnect(m_AutoReconnect);
  ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  isScanning = true;
  return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  bool ret = true;

  if (!isScanning) {
    ret = checkCOMMs();

    if (ret && (ret = checkStatus())) {
      if (ret) {
        ret = turnOn();
      }
    }
  }

  return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  bool ret = true;

  if (!checkCOMMs()) {
    ydlidar::console.error("[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
    return false;
  }

  if (!checkStatus()) {
    ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.");
  }

  if (!turnOn()) {
    ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.");

  }

  return ret;

}
