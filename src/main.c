#include "definitions.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// Wi-Fi Credentials (change these to match your network)
// =============================================================
#define WIFI_SSID "sujan"   // Your Wi-Fi network name
#define WIFI_PWD  "sujandurai"   // Your Wi-Fi password

// =============================================================
// Wi-Fi / ESP-01S Driver (inlined - no external file needed)
// =============================================================
#define ESP_BUFFER_SIZE 256

static char espBuffer[ESP_BUFFER_SIZE];
static uint16_t bufferIndex = 0;

// Software delay (~1ms per call at 48MHz, adjust count if needed)
void delay_ms(uint32_t ms) {
    uint32_t count = (48000UL / 4UL) * ms;
    while (count--) {
        __asm__ volatile("nop");
    }
}

// Send a string to the ESP-01S via SERCOM2 using direct register access
static void ESP_Write(const char* str) {
    while (*str) {
        while (!(SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        SERCOM2_REGS->USART_INT.SERCOM_DATA = (uint16_t)*str++;
    }
}

// Clear SERCOM2 receive errors (BUFOVF from ESP boot messages overflowing the 1-byte FIFO)
static void SERCOM2_ClearErrors(void) {
    SERCOM2_REGS->USART_INT.SERCOM_STATUS = (uint16_t)(
        SERCOM_USART_INT_STATUS_BUFOVF_Msk |
        SERCOM_USART_INT_STATUS_FERR_Msk   |
        SERCOM_USART_INT_STATUS_PERR_Msk);
    SERCOM2_REGS->USART_INT.SERCOM_INTFLAG = (uint8_t)SERCOM_USART_INT_INTFLAG_ERROR_Msk;
}

// Flush any pending bytes from ESP (e.g. boot messages) before sending a command
static void ESP_FlushRx(uint32_t wait_ms) {
    uint32_t idle = 0;
    SERCOM2_ClearErrors(); // clear overflow errors first
    while (idle < wait_ms) {
        if (SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) {
            (void)SERCOM2_REGS->USART_INT.SERCOM_DATA; // read and discard
            idle = 0; // reset idle count when data comes in
        } else {
            delay_ms(1);
            idle++;
        }
    }
    SERCOM2_ClearErrors(); // clear any errors that arose during flush
}

// Send an AT command and wait for the expected response substring.
// Uses a tight spin loop (NO sleep) to avoid BUFOVF:
// at 115200 baud bytes arrive every 86µs — sleeping even 1ms between polls
// causes the 1-byte RX FIFO to overflow and lose bytes.
// Timeout is approximate: ~8000 spin iterations ≈ 1ms at 48MHz with -O1.
static bool ESP_SendCmd(const char* cmd, const char* expected, uint32_t timeout_ms) {
    // Flush stale RX bytes + clear errors before sending
    ESP_FlushRx(100);

    memset(espBuffer, 0, ESP_BUFFER_SIZE);
    bufferIndex = 0;

    ESP_Write(cmd);

    // Wait for last byte to fully shift out (TXC = shift register empty)
    while (!(SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk));

    // Tight polling — noDataCount ticks only when NO byte is available.
    // When a byte arrives, noDataCount resets so we don't time out mid-response.
    // ~8000 iterations with no data ≈ 1ms (calibrated for 48MHz -O1)
    const uint32_t TICKS_PER_MS = 8000UL;
    uint32_t deadline = timeout_ms * TICKS_PER_MS;
    uint32_t noDataCount = 0;

    while (noDataCount < deadline) {
        if (SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) {
            uint8_t data = (uint8_t)SERCOM2_REGS->USART_INT.SERCOM_DATA;
            if (bufferIndex < ESP_BUFFER_SIZE - 1) {
                espBuffer[bufferIndex++] = (char)data;
                espBuffer[bufferIndex]   = '\0';
            }
            if (strstr(espBuffer, expected) != NULL) {
                return true;
            }
            noDataCount = 0; // reset timeout while data is flowing
        } else {
            noDataCount++;
        }
    }
    return false;
}

static void WIFI_Init(void) {
    bufferIndex = 0;
    memset(espBuffer, 0, ESP_BUFFER_SIZE);
}

static bool WIFI_IsAlive(void) {
    return ESP_SendCmd("AT\r\n", "OK", 2000);
}

static bool WIFI_SetMode(uint8_t mode) {
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%u\r\n", mode);
    return ESP_SendCmd(cmd, "OK", 3000);
}

// Connect to Wi-Fi. On success prints: WIFI CONNECT "SSID"
static bool WIFI_Connect(const char* ssid, const char* pwd) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    return ESP_SendCmd(cmd, "WIFI GOT IP", 20000);
}


// Baud: 115200 @ 48MHz
#define UART_BAUD_115200 63019

/* --- DUAL UART INITIALIZATION (SERCOM2 on PA08/PA09, SERCOM5 on PB02/PB03) --- */
void Standalone_Init(void) {
    // Enable MCLK for SERCOM2, SERCOM3, and SERCOM5
    MCLK_REGS->MCLK_APBCMASK |= MCLK_APBCMASK_SERCOM2_Msk |
                                 MCLK_APBCMASK_SERCOM3_Msk |
                                 MCLK_APBCMASK_SERCOM5_Msk;

    // Enable GCLK for SERCOM2 (ch19), SERCOM3 (ch20), SERCOM5 (ch22)
    GCLK_REGS->GCLK_PCHCTRL[19] = GCLK_PCHCTRL_GEN(0x0) | GCLK_PCHCTRL_CHEN_Msk;
    GCLK_REGS->GCLK_PCHCTRL[20] = GCLK_PCHCTRL_GEN(0x0) | GCLK_PCHCTRL_CHEN_Msk;
    GCLK_REGS->GCLK_PCHCTRL[22] = GCLK_PCHCTRL_GEN(0x0) | GCLK_PCHCTRL_CHEN_Msk;
    while (!(GCLK_REGS->GCLK_PCHCTRL[19] & GCLK_PCHCTRL_CHEN_Msk));
    while (!(GCLK_REGS->GCLK_PCHCTRL[20] & GCLK_PCHCTRL_CHEN_Msk));
    while (!(GCLK_REGS->GCLK_PCHCTRL[22] & GCLK_PCHCTRL_CHEN_Msk));

    // SERCOM2 (PA08/PA09) and SERCOM3 (PB08/PB09) are already physically configured 
    // in PORT_Initialize() using `PORT_SEC_REGS`. 
    // For SERCOM5 (PB02/PB03), we must also configure pins using secure registers:
    
    // SERCOM5: PB02=PAD0(TX), PB03=PAD1(RX) -- MUX D (0x3)
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] = 0x33U; // MUX D for PB02 and PB03
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[2] |= PORT_PINCFG_PMUXEN_Msk; // Enable Peripheral Muxing
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[3] |= PORT_PINCFG_PMUXEN_Msk;

    // Init SERCOM2 (ESP-01S: TX=PA08/PAD0, RX=PA09/PAD1, RXPO=1)
    // RXPO=1 is correct: ESP TX is on PA09. Bridge mode confirmed this works.
    SERCOM2_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
    while (SERCOM2_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);
    SERCOM2_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE(0x1) | SERCOM_USART_INT_CTRLA_TXPO(0x0) | SERCOM_USART_INT_CTRLA_RXPO(0x1) | SERCOM_USART_INT_CTRLA_DORD_Msk;
    SERCOM2_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE(0x0) | SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk;
    while (SERCOM2_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    SERCOM2_REGS->USART_INT.SERCOM_BAUD = (uint16_t)UART_BAUD_115200;
    SERCOM2_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
    while (SERCOM2_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);
    // Clear any overflow/framing errors left from ESP boot messages
    SERCOM2_REGS->USART_INT.SERCOM_STATUS = (uint16_t)(SERCOM_USART_INT_STATUS_BUFOVF_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_PERR_Msk);
    SERCOM2_REGS->USART_INT.SERCOM_INTFLAG = (uint8_t)SERCOM_USART_INT_INTFLAG_ERROR_Msk;

    // Init SERCOM3 (PC/PuTTY on PA22/PA23)
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE(0x1) | SERCOM_USART_INT_CTRLA_TXPO(0x0) | SERCOM_USART_INT_CTRLA_RXPO(0x1) | SERCOM_USART_INT_CTRLA_DORD_Msk;
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE(0x0) | SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk;
    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    SERCOM3_REGS->USART_INT.SERCOM_BAUD = (uint16_t)UART_BAUD_115200;
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);

    // Init SERCOM5 (second device on PB02/PB03)
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE(0x1) | SERCOM_USART_INT_CTRLA_TXPO(0x0) | SERCOM_USART_INT_CTRLA_RXPO(0x1) | SERCOM_USART_INT_CTRLA_DORD_Msk;
    SERCOM5_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE(0x0) | SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk;
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    SERCOM5_REGS->USART_INT.SERCOM_BAUD = (uint16_t)UART_BAUD_115200;
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);
}

void PC_Print(const char *str) {
    while (*str) {
        while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)*str++;
    }
}

int main(void) {
    SYS_Initialize(NULL);
    Standalone_Init();

    PC_Print("\r\n--- UART TRANSPARENT BRIDGE ---\r\n");
    PC_Print("SERCOM2 (PA08/PA09) <-> PuTTY (SERCOM3 PA22/PA23)\r\n");
    PC_Print("SERCOM5 (PB02/PB03) <-> PuTTY (SERCOM3 PA22/PA23)\r\n");

    // -------------------------------------------------------
    // Wait for ESP-01S to finish booting (needs ~2-3 seconds)
    // Use 5000ms here to account for compiler optimization
    // making delay_ms run faster than expected at -O1
    // -------------------------------------------------------
    PC_Print("Waiting for ESP-01S to boot...\r\n");
    delay_ms(5000);

    // Flush any ESP boot messages before we start sending commands
    ESP_FlushRx(500);


    // -------------------------------------------------------
    // Auto-connect using WIFI_IsAlive/WIFI_Connect
    // These use the tight-polling ESP_SendCmd (no delay_ms)
    // so bytes are never lost to BUFOVF.
    // -------------------------------------------------------
    WIFI_Init();
    bool esp_alive = false;
    for (uint8_t retry = 0; retry < 5; retry++) {
        PC_Print("Checking ESP-01S...\r\n");
        if (WIFI_IsAlive()) {
            esp_alive = true;
            break;
        }
        delay_ms(1000);
    }

    if (esp_alive) {
        PC_Print("ESP-01S is alive!\r\n");

        PC_Print("Setting Station Mode...\r\n");
        WIFI_SetMode(1);
        delay_ms(500);

        PC_Print("Connecting to Wi-Fi: " WIFI_SSID "\r\n");
        if (WIFI_Connect(WIFI_SSID, WIFI_PWD)) {
            char msg[64];
            snprintf(msg, sizeof(msg), "WIFI CONNECT \"%s\"\r\n", WIFI_SSID);
            PC_Print(msg);
        } else {
            PC_Print("Wi-Fi connection FAILED. Check SSID/password.\r\n");
        }
    } else {
        PC_Print("ESP-01S NOT responding after 5 retries.\r\n");
    }


    PC_Print("\r\nEntering bridge mode. Type your commands now...\r\n\r\n");

    while (1) {
        /* --- Bridge 1: PuTTY (SERCOM3) <-> ESP (SERCOM2 on PA08/PA09) --- */

        // 1a. Read from PuTTY, Send to SERCOM2 (ESP)
        if (SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) {
            uint8_t data = (uint8_t)SERCOM3_REGS->USART_INT.SERCOM_DATA;

            // Send to SERCOM2 (ESP)
            while (!(SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
            SERCOM2_REGS->USART_INT.SERCOM_DATA = (uint16_t)data;

            // Also send to SERCOM5 (second device on PB02/PB03)
            while (!(SERCOM5_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
            SERCOM5_REGS->USART_INT.SERCOM_DATA = (uint16_t)data;

            // If Enter (\r), send \n to both devices and echo to PuTTY
            if (data == '\r') {
                while (!(SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
                SERCOM2_REGS->USART_INT.SERCOM_DATA = (uint16_t)'\n';

                while (!(SERCOM5_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
                SERCOM5_REGS->USART_INT.SERCOM_DATA = (uint16_t)'\n';

                while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
                SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)'\n';
            }
        }

        // 1b. Read from SERCOM2 (ESP), Send to PuTTY
        if (SERCOM2_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) {
            uint8_t data = (uint8_t)SERCOM2_REGS->USART_INT.SERCOM_DATA;
            while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
            SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)data;
        }

        /* --- Bridge 2: SERCOM5 (PB02/PB03) -> PuTTY (SERCOM3) --- */

        // 2. Read from SERCOM5 (PB02/PB03), Send to PuTTY
        if (SERCOM5_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) {
            uint8_t data = (uint8_t)SERCOM5_REGS->USART_INT.SERCOM_DATA;
            while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
            SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)data;
        }
    }
    return 0;
}