// Test motor wheel mode
#define VERSION 2

#if (VERSION == 1)
    #include "DxlMaster.h"
#else
    #include "DxlMaster2.h"
#endif

#define ADR_LED  65
#define ADR_TORQUE  64

#define ID_1  0x01
DynamixelDevice device(ID_1);

uint8_t led_state = true;

void setup() {
    DxlMaster.begin(57600);
    Serial.begin(57600);
    Serial.println("Start Simple test scketch");

    device.protocolVersion(2);

    Serial.print("Version: ");
    Serial.println(device.protocolVersion());

    Serial.print("Ping: ");
    Serial.println(device.ping(), HEX);

    delay(500);
}

void loop() 
{
    DynamixelStatus status = 0xFF; 

    Serial.println("\n ***Loop start***");

    // 0. Ping ----------------------------------------------/
    Serial.print("0. Ping: ");
    status = device.ping();
    if (status == DYN_STATUS_OK)
        Serial.print("\tOk!");
    else
        Serial.print("\tErr!");
    Serial.println(" Status = " + String(status));
    
    // 1. Info ----------------------------------------------/
    Serial.print("1. Info: ");
    status = 0xFF;
    uint8_t buf[3];
    status = device.ping(buf);
    uint16_t num = buf[0] | (buf[1] << 8); 
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
        Serial.print(" Number: ");
        Serial.print(num, HEX);
        Serial.print(" Version: ");
        Serial.print(buf[2], HEX);
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.println(" Status = " + String(status));

    // 2. Read ----------------------------------------------/
    Serial.print("2. Read: ");
    status = 0xFF;
    uint8_t id = 0;
    status = device.read(7, id);
    if (id == ID_1) {
        Serial.print("\tOk! ID = ");
        Serial.print(id);
    }
    else {
        Serial.print("\tErr! ID = ");
        Serial.print(id);
    }
    Serial.print(" Status = ");
    Serial.println(status);
    
    // 3. Write ----------------------------------------------/
    Serial.print("3. Write: ");
    status = 0xFF;
    status = device.write(ADR_LED, led_state);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk! LED = ");
        Serial.print(led_state);
    }
    else {
        Serial.print("\tErr!");
    }
    led_state = !led_state;
    Serial.print(" Status = ");
    Serial.println(status);

    // 4. Reg Write ----------------------------------------------/
    Serial.print("4. Reg Write ");
    status = 0xFF;
    status = device.regWrite(ADR_LED, led_state);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    led_state = !led_state;
    Serial.print(" Status = ");
    Serial.println(status);

    // 5. Action ----------------------------------------------/
    Serial.print("5. Action ");
    status = 0xFF;
    status = device.action();
    Serial.println("\tAlways Ok!");

    // 6. Reset ----------------------------------------------/
    Serial.print("6. Reset: ");
    status = 0xFF;
    status = device.reset(0x02);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);
    delay(400);
    // 7. Reboot ----------------------------------------------/
    Serial.print("7. Reboot: ");
    status = 0xFF;
    status = device.reboot();
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);

    delay(700);
    // 8. Clear ----------------------------------------------/
    Serial.print("8. Clear: ");
    status = 0xFF;
    uint8_t clr[5] = {0x01, 0x44, 0x58, 0x4C, 0x22};
    status = device.clear(5, clr);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);

    delay(700);
    // 9. Backup ----------------------------------------------/
    Serial.print("9. Backup: ");
    status = 0xFF;

    Serial.println();

    uint8_t restore[5] = {0x01, 0x43, 0x54, 0x52, 0x4c};
    status = device.backup(5, restore);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }

    Serial.print(" Status = ");
    Serial.println(status);

    delay(700);
    // 10. Sync Read ----------------------------------------------/
    Serial.print("10. Sync Read: ");
    status = 0xFF;
    uint8_t buf10[4];
    uint8_t add10[1] = {ID_1};
    status = device.syncRead(1, add10, 0, 4, buf10);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);

    delay(700);
    // 11. Sync Write ----------------------------------------------/
    Serial.print("11. Sync Write: ");
    status = 0xFF;
    uint8_t arr_id[1] = {1};
   status = device.syncWrite(1, arr_id, ADR_LED, 1, &led_state);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk! LED = ");
        Serial.print(led_state);
    }
    else {
        Serial.print("\tErr!");
    }
    led_state = !led_state;
    Serial.print(" Status = ");
    Serial.println(status);

    delay(700);
    // 13. Bulk Read ----------------------------------------------/
    Serial.print("13. Bulk Read: ");
    status = 0xFF;
    const uint8_t tx_size = 5; // 5 * number of ids
    const uint8_t id_1 = 1;
    const uint8_t size_1 = 10;
    const uint8_t addr_1 = 31;
    uint8_t tx13[tx_size] = {id_1, addr_1, 0, size_1, 0}; 
    uint8_t rx13[size_1];
    status = device.bulkRead(1, tx13, tx_size, rx13);
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);
    delay(700);

    // 14. Bulk Write ----------------------------------------------/
    Serial.print("14. Bulk Write: ");
    status = 0xFF;
    uint8_t tx14[] = {1, 65, 0, 1, 0, led_state}; //id + addr + size + params
    status = device.bulkWrite(1, tx14, sizeof(tx14));
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk! LED = ");
        Serial.print(led_state);
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.print(" Status = ");
    Serial.println(status);
    while(1);
    
    delay(5000);
}

