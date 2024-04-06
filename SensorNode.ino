/*

*/
#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "Universal SensorNode";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBt;
boolean confirmRequestPending = true;

void BTConfirmRequestCallback(uint32_t numVal)
{
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success)
{
  confirmRequestPending = false;
  if (success)
  {
    Serial.println("Pairing success!!");
  }
  else
  {
    Serial.println("Pairing failed, rejected by user!!");
  }
}

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial2.begin(9600);
  SerialBt.begin(device_name);

  SerialBt.onConfirmRequest(BTConfirmRequestCallback);
  SerialBt.onAuthComplete(BTAuthCompleteCallback);
  SerialBt.begin("Sensor Node"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // read from port 0, send to port 2:
  if (Serial.available() > 0) {
    int data0 = Serial.read();
    Serial2.write(data0);
  }

  // read from port 2, send to port 0:
  if (Serial2.available() > 0) {
    String data2 = Serial2.readString();
    Serial.println(data2);
  }

  //if (confirmRequestPending)
  //{
  //  if (Serial.available())
  //  {
  //    int dat = Serial.read();
  //    if (dat == 'Y' || dat == 'y')
  //    {
  //      SerialBt.confirmReply(true);
  //    }
  //    else
  //    {
  //      SerialBt.confirmReply(false);
  //    }
  //  }
  //}
  //else
  //{
  //  if (Serial.available())
  //  {
  //    SerialBt.write(Serial.read());
  //  }
  //  // read from Bluetooth, send to port 0:
  //  if(SerialBt.available() > 0){
  //    int dataBt = SerialBt.read();
  //    Serial.write(dataBt);
  //  }
  //  delay(20);
  //}
}
