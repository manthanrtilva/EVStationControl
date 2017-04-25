
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cerrno>
#include <cstddef>
#include <thread>

#include <modbus/modbus.h>

typedef enum REG_OUTLET_STATE : uint8_t {
  IDLE_NOT_CONNECTED = 0x00,
  NOT_CHARGHING_CONNECTED = 0x01,
  CHARGIN_CONNECTED = 0x02,
  FUALTED = 99,
  UKNOWN
} REG_OUTLET_STATE;
typedef enum REG_OUTLET_FAULT_SEVERITY : uint16_t {

} REG_OUTLET_FAULT_SEVERITY;
typedef struct SchniederStationData {
  SchniederStationData() : m_oOutletState(REG_OUTLET_STATE::UKNOWN) {}
  REG_OUTLET_STATE m_oOutletState;
} SchniederStationData;

class SchniederL2Station {
public:
  SchniederL2Station(ros::Publisher &pub) : oRosPub{pub} {
    // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  }
  ~SchniederL2Station() {
    if (mPtrModbusCtx) {
      modbus_close(mPtrModbusCtx);
      modbus_free(mPtrModbusCtx);
    }
  }
  void onCommand(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "enableCharge") {
    	writeModbusRegister(1100,1);
    } else if (msg->data == "disableCharge") {
    	writeModbusRegister(1100,0);
    } else {
      ROS_ERROR("Unknown command:%s", msg->data.c_str());
    }
  }
  void setMobuseRetryCount(uint8_t value) { mUInt8MobuseRetryCount = value; }
  uint8_t getMobuseRetryCount() { return mUInt8MobuseRetryCount; }
  void setMobuseRetryDelay(uint32_t value) { mUInt32MobuseRetryDelay = value; }
  uint32_t getMobuseRetryDelay() { return mUInt32MobuseRetryDelay; }
  void setMobuseResponseTimeout(uint32_t value) { mUint32ModbusResponseTimeout = value; }
  uint32_t getMobuseResponseTimeout() { return mUint32ModbusResponseTimeout; }
  void task() {
    uint16_t regs[2];
    if (readModbusRegisters(1200, 1, regs) > 0) {
      if (regs[0] != m_oOutletState) {
        std_msgs::String msg;
        switch (regs[0]) {
        case REG_OUTLET_STATE::IDLE_NOT_CONNECTED:
          m_oOutletState = REG_OUTLET_STATE::IDLE_NOT_CONNECTED;
          msg.data = "IDLE_NOT_CONNECTED";
          break;
        case REG_OUTLET_STATE::NOT_CHARGHING_CONNECTED:
          m_oOutletState = REG_OUTLET_STATE::NOT_CHARGHING_CONNECTED;
          msg.data = "NOT_CHARGHING_CONNECTED";
          break;
        case REG_OUTLET_STATE::CHARGIN_CONNECTED:
          m_oOutletState = REG_OUTLET_STATE::CHARGIN_CONNECTED;
          msg.data = "CHARGIN_CONNECTED";
          break;
        case REG_OUTLET_STATE::FUALTED:
          m_oOutletState = REG_OUTLET_STATE::FUALTED;
          msg.data = "FUALTED";
          break;
        case REG_OUTLET_STATE::UKNOWN:
          m_oOutletState = REG_OUTLET_STATE::UKNOWN;
          msg.data = "UKNOWN";
          break;
        }
        m_oOutletState = static_cast<REG_OUTLET_STATE>(regs[0]);
        oRosPub.publish(msg);
      }
    }
  }

protected:
private:
  int readModbusRegisters(int add, int nb, uint16_t *dest) {
    int ret = 0;
    if (!mPtrModbusCtx) {
      ROS_INFO("modbus is not connected");
      if (connectModbus() != 0) {
        ROS_ERROR("cann't connect modbus");
      }
    }
    if (mPtrModbusCtx) {
      ret = modbus_read_registers(mPtrModbusCtx, add, nb, dest);
      ROS_DEBUG("first try:%d", ret);
      if (ret != -1) {
        for (uint8_t i = 1; i < mUInt8MobuseRetryCount; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(mUInt32MobuseRetryDelay));
          ret = modbus_read_registers(mPtrModbusCtx, add, nb, dest);
          if (ret != -1) {
            ROS_DEBUG("modbus read success");
            break;
          }
          ret = errno;
        }
      } else {
        ROS_DEBUG("modbus read success");
      }
    }
    return ret;
  }

  int writeModbusRegister(int add, int value) {
    int ret = 0;
    if (!mPtrModbusCtx) {
      ROS_INFO("modbus is not connected");
      if (connectModbus() != 0) {
        ROS_ERROR("cann't connect modbus");
      }
    }
    if (mPtrModbusCtx) {
      ret = modbus_write_register(mPtrModbusCtx, add, value);
      ROS_INFO("first try:%d", ret);
      if (ret != -1) {
        for (uint8_t i = 1; i < mUInt8MobuseRetryCount; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(mUInt32MobuseRetryDelay));
          ret = modbus_write_register(mPtrModbusCtx, add, value);
          if (ret != -1) {
            ROS_INFO("modbus write success");
            break;
          }
          ret = errno;
        }
      } else {
        ROS_INFO("modbus write success");
      }
    }
    return ret;
  }
  int connectModbus() {
    int ret = 0;
    mPtrModbusCtx = modbus_new_rtu(mStrDevice.c_str(), mInt32ModbusBaudrate, mCharModbusParity, mInt32ModbusDatabits,
                                   mInt32ModbusStopbits);
    if (mPtrModbusCtx) {
      if (modbus_set_slave(mPtrModbusCtx, mInt32ModbusSlaveid) == 0) {
        if (modbus_connect(mPtrModbusCtx) == 0) {
          struct timeval response_timeout;
          response_timeout.tv_sec = mUint32ModbusResponseTimeout / 1000;
          response_timeout.tv_usec = 1000 * (mUint32ModbusResponseTimeout % 1000);
          modbus_set_response_timeout(mPtrModbusCtx, &response_timeout);
          ROS_INFO("modbus connected");
        } else {
          ret = errno;
          modbus_close(mPtrModbusCtx);
          modbus_free(mPtrModbusCtx);
          mPtrModbusCtx = nullptr;
        }
      } else {
        ret = errno;
        modbus_free(mPtrModbusCtx);
        mPtrModbusCtx = nullptr;
      }
    } else {
      ret = errno;
      modbus_free(mPtrModbusCtx);
      mPtrModbusCtx = nullptr;
    }
    return ret;
  }

private:
  std::string mStrDevice{"/dev/ttyUSB0"};
  ros::Publisher &oRosPub;
  modbus_t *mPtrModbusCtx{nullptr};
  REG_OUTLET_STATE m_oOutletState{REG_OUTLET_STATE::UKNOWN};
  uint8_t mUInt8MobuseRetryCount{3};
  uint32_t mUInt32MobuseRetryDelay{500};      // in milliseconds
  uint32_t mUint32ModbusResponseTimeout{500}; // in  milliseconds
  int32_t mInt32ModbusBaudrate{19200};
  char mCharModbusParity{'N'};
  int32_t mInt32ModbusDatabits{8};
  int32_t mInt32ModbusStopbits{1};
  int32_t mInt32ModbusSlaveid{1};
};

int main(int argc, char **argv) {
  std_msgs::String msg;
  ros::init(argc, argv, "schnieder_l2_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("schnieder_l2", 1000);
  msg.data = "boot";
  pub.publish(msg);
  ros::Rate loop_rate(1);
  int count = 0;
  SchniederL2Station oStation(pub);
  ros::Subscriber sub = n.subscribe("schnieder_l2_control", 1000, &SchniederL2Station::onCommand,&oStation);
  while (ros::ok()) {
    oStation.task();
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
