#include <esp_now.h>
#include <WiFi.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "esp_log.h" // Add this at the top
 
// Packet Constants
#define SYNC_BYTE_1 0xC7
#define SYNC_BYTE_2 0x7C
#define END_BYTE    0x01
#define HEADER_LEN  3   // Sync1 + Sync2 + Counter
#define NUM_DATA    60  // 20 Nodes * 3 Bytes
#define PACKET_LEN  (NUM_DATA + HEADER_LEN + 1) // 64 Bytes Total
 
// Connection Constants
#define NUM_NODES 20
#define NODE_TIMEOUT 40      
#define OFFLINE_THRESHOLD 3000
 
// MAC Addresses
uint8_t nodeMACs[NUM_NODES][6] = {
  {0xE4, 0xB3, 0x23, 0xB0, 0xE4, 0xF0}, {0xDC, 0x1E, 0xD5, 0x60, 0xD9, 0x98},
  {0xE4, 0xB3, 0x23, 0xAF, 0x87, 0xC0}, {0xDC, 0x1E, 0xD5, 0x60, 0xD9, 0x94},
  {0xE4, 0xB3, 0x23, 0xB0, 0xB4, 0x04}, {0xE4, 0xB3, 0x23, 0xAF, 0x51, 0xF8},
  {0xE4, 0xB3, 0x23, 0xAF, 0xC2, 0x24}, {0xE4, 0xB3, 0x23, 0xB0, 0x5C, 0x18},
  {0xE4, 0xB3, 0x23, 0xAF, 0xD6, 0x84}, {0xE4, 0xB3, 0x23, 0xB1, 0x12, 0x6C},
  {0xE4, 0xB3, 0x23, 0xAF, 0xCE, 0xD4}, {0xE4, 0xB3, 0x23, 0xB0, 0xE0, 0x4C},
  {0xDC, 0x1E, 0xD5, 0x60, 0xD9, 0x58}, {0xE4, 0xB3, 0x23, 0xAF, 0x65, 0x48},
  {0xE4, 0xB3, 0x23, 0xAE, 0xFA, 0xA0}, {0xE4, 0xB3, 0x23, 0xB0, 0xB6, 0xBC},
  {0xE4, 0xB3, 0x23, 0xB0, 0x59, 0x24}, {0xDC, 0x1E, 0xD5, 0x60, 0xD9, 0xB4},
  {0xE4, 0xB3, 0x23, 0xAF, 0x87, 0x24}, {0xE4, 0xB3, 0x23, 0xAF, 0x4A, 0x00}
};
 
// Buffers and State
uint8_t packetBuffer[PACKET_LEN];
uint8_t packetCounter = 0;
unsigned long lastSeen[NUM_NODES];
bool lastDrawnStatus[NUM_NODES];
volatile int currentPollingID = -1;
volatile bool dataReceived = false;
 
void drawBlock(int id, bool online) {
    int col = id % 5; int row = id / 5;
    int x1 = (col * 48) + 2; int y1 = (row * 33) + 2;
    int x2 = x1 + 44; int y2 = y1 + 29;
    UWORD color = online ? GREEN : RED;
    Paint_DrawRectangle(x1, y1, x2, y2, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    char label[3]; sprintf(label, "%02d", id + 1);
    Paint_DrawString_EN(x1 + 8, y1 + 4, label, &Font20, color, BLACK);
}
 
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    int polled = currentPollingID;
    if (polled >= 0 && polled < NUM_NODES && len == 3) {
        if (memcmp(info->src_addr, nodeMACs[polled], 6) == 0) {
            // Map 3 bytes from node directly into the packet buffer data section
            memcpy(&packetBuffer[HEADER_LEN + (polled * 3)], data, 3);
            lastSeen[polled] = millis();
            dataReceived = true;
        }
    }
}
            dataReceived = true;
        }
    }
}
 
void setup() {
    Serial.begin(115200);
    Config_Init(); LCD_Init(); LCD_SetBacklight(1000);
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 90, BLACK);
    Paint_Clear(BLACK);
 
    // Initialize Static Packet Header/Footer
    packetBuffer[0] = SYNC_BYTE_1;
    packetBuffer[1] = SYNC_BYTE_2;
    packetBuffer[PACKET_LEN - 1] = END_BYTE;
 
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE("POLL", "esp_now_init failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
 
    for (int i = 0; i < NUM_NODES; i++) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, nodeMACs[i], 6);
        if (esp_now_add_peer(&peer) != ESP_OK) {
            ESP_LOGW("POLL", "Failed to add peer %d", i + 1);
        }
        drawBlock(i, false); 
        lastDrawnStatus[i] = false;
    }
}
 
void loop() {
    unsigned long cycleStart = millis(); // Start time

    // 1. Initialize Data Section to -1 (0xFF) before each poll
    memset(&packetBuffer[HEADER_LEN], 0xFF, NUM_DATA);
 
    // 2. Deterministic Polling Sequence
    for (int i = 0; i < NUM_NODES; i++) {
        dataReceived = false;
        currentPollingID = i;
        
        uint8_t pollSignal = 0xA5;
        esp_now_send(nodeMACs[i], &pollSignal, 1);
 
        unsigned long startWait = millis();
        while (!dataReceived && (millis() - startWait < NODE_TIMEOUT)) {
            delayMicroseconds(100); 
        }
    }
    currentPollingID = -1;

    // 3. Finalize Packet and Send
    packetBuffer[2] = packetCounter;
    Serial.write(packetBuffer, PACKET_LEN); // Send 64 bytes binary
    packetCounter++;
 
    // 4. Update Screen Status
    for (int i = 0; i < NUM_NODES; i++) {
        bool isOnline = (lastSeen[i] != 0) && (millis() - lastSeen[i] < OFFLINE_THRESHOLD);
        if (isOnline != lastDrawnStatus[i]) {
            drawBlock(i, isOnline);
            lastDrawnStatus[i] = isOnline;
        }
    }

    // Small delay to stabilize the 1Hz-ish cycle
    delay(250); 
    
    unsigned long cycleEnd = millis(); // End time
    ESP_LOGI("POLL", "Polling cycle duration: %lu ms", cycleEnd - cycleStart);
}