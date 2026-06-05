#include <Arduino.h>

#define RXD2 16
#define TXD2 17

HardwareSerial STM32Serial(2);

uint8_t rxBuf[125];

float modbusPayloadToFloat(uint8_t *buf)
{
    union
    {
        float f;
        uint8_t b[4];
    } u;

    u.b[3] = buf[0];
    u.b[2] = buf[1];
    u.b[1] = buf[2];
    u.b[0] = buf[3];

    return u.f;
}

void printMeterData()
{
    Serial.println();
    Serial.println("========== MFM383 ==========");
    Serial.printf("SlaveID       = %d\n", rxBuf[0]);

    Serial.printf("V1N        = %.2f\n", modbusPayloadToFloat(&rxBuf[3]));
    Serial.printf("V2N        = %.2f\n", modbusPayloadToFloat(&rxBuf[7]));
    Serial.printf("V3N        = %.2f\n", modbusPayloadToFloat(&rxBuf[11]));
    Serial.printf("AvgVLN     = %.2f\n", modbusPayloadToFloat(&rxBuf[15]));

    Serial.printf("V12        = %.2f\n", modbusPayloadToFloat(&rxBuf[19]));
    Serial.printf("V23        = %.2f\n", modbusPayloadToFloat(&rxBuf[23]));
    Serial.printf("V31        = %.2f\n", modbusPayloadToFloat(&rxBuf[27]));
    Serial.printf("AvgVLL     = %.2f\n", modbusPayloadToFloat(&rxBuf[31]));

    Serial.printf("I1         = %.2f\n", modbusPayloadToFloat(&rxBuf[35]));
    Serial.printf("I2         = %.2f\n", modbusPayloadToFloat(&rxBuf[39]));
    Serial.printf("I3         = %.2f\n", modbusPayloadToFloat(&rxBuf[43]));
    Serial.printf("AvgCurrent = %.2f\n", modbusPayloadToFloat(&rxBuf[47]));

    Serial.printf("kW1        = %.2f\n", modbusPayloadToFloat(&rxBuf[51]));
    Serial.printf("kW2        = %.2f\n", modbusPayloadToFloat(&rxBuf[55]));
    Serial.printf("kW3        = %.2f\n", modbusPayloadToFloat(&rxBuf[59]));

    Serial.printf("kVA1       = %.2f\n", modbusPayloadToFloat(&rxBuf[63]));
    Serial.printf("kVA2       = %.2f\n", modbusPayloadToFloat(&rxBuf[67]));
    Serial.printf("kVA3       = %.2f\n", modbusPayloadToFloat(&rxBuf[71]));

    Serial.printf("kVAR1      = %.2f\n", modbusPayloadToFloat(&rxBuf[75]));
    Serial.printf("kVAR2      = %.2f\n", modbusPayloadToFloat(&rxBuf[79]));
    Serial.printf("kVAR3      = %.2f\n", modbusPayloadToFloat(&rxBuf[83]));

    Serial.printf("TotalkW    = %.2f\n", modbusPayloadToFloat(&rxBuf[87]));
    Serial.printf("TotalkVA   = %.2f\n", modbusPayloadToFloat(&rxBuf[91]));
    Serial.printf("TotalkVAR  = %.2f\n", modbusPayloadToFloat(&rxBuf[95]));

    Serial.printf("PF1        = %.2f\n", modbusPayloadToFloat(&rxBuf[99]));
    Serial.printf("PF2        = %.2f\n", modbusPayloadToFloat(&rxBuf[103]));
    Serial.printf("PF3        = %.2f\n", modbusPayloadToFloat(&rxBuf[107]));
    Serial.printf("AvgPF      = %.2f\n", modbusPayloadToFloat(&rxBuf[111]));

    Serial.printf("Frequency  = %.2f\n", modbusPayloadToFloat(&rxBuf[115]));
    Serial.printf("kWh        = %.2f\n", modbusPayloadToFloat(&rxBuf[119]));

    Serial.println("============================");
    Serial.println();
}

void sendRequest(uint8_t meterID)
{
    uint8_t cmd[2];

    cmd[0] = meterID;
    cmd[1] = 0x04;

    while(STM32Serial.available())
    {
        STM32Serial.read();
    }

    STM32Serial.write(cmd, 2);

    Serial.printf("TX -> %02X %02X\n",
                  cmd[0],
                  cmd[1]);

    uint32_t startTime = millis();

    while(STM32Serial.available() < 125)
    {
        if(millis() - startTime > 1000)
        {
            Serial.println("Timeout waiting STM32");
            return;
        }
    }

    STM32Serial.readBytes(rxBuf, 125);

    Serial.printf("RX %d bytes\n", 125);

    printMeterData();
}

void setup()
{
    Serial.begin(115200);

    STM32Serial.begin(
        9600,
        SERIAL_8N1,
        RXD2,
        TXD2
    );

    Serial.println();
    Serial.println("Nhap:");
    Serial.println("0x01");
    Serial.println("0x02");
    Serial.println();
}

void loop()
{
    if(Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');

        cmd.trim();

        if(cmd == "0x01")
        {
            sendRequest(0x01);
        }
        else if(cmd == "0x02")
        {
            sendRequest(0x02);
        }
        else
        {
            Serial.println("Chi nhap 0x01 hoac 0x02");
        }
    }
}