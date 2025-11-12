#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <RotaryEncoder.h>

#define ADC 26      // ADC0. - battery (not used for pico)

#define TFT_CS 17
#define TFT_CLK 18
#define TFT_MOSI 19
#define TFT_DC 20
#define TFT_RST 21

// bool setRX(NOT_A_PIN); // or setMISO()
// bool setCS(TFT_CS);
// bool setSCK(18);
// bool setTX(19); // or setMOSI()


#define CLICKS_PER_STEP 4
#define PSTEP 32

SPISettings spisettings(1000000, MSBFIRST, SPI_MODE0);
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_DC, TFT_CS);
// int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST);

RotaryEncoder *encoder[4] = { nullptr, nullptr, nullptr, nullptr };

void handleLoop(){
    for( uint8_t e = 0; e < 4; e++ ){
        encoder[e]->tick(); // just call tick() to check the state.
    }
}

// hw_timer_t *timer = NULL;

int col_pins[6] = {10, 8, 6, 4, 2, 1};
int row_pins[4] = {7, 9, 5, 11};
int encoder_pins[4][2] = {{10, 11}, {12, 13}, {14, 15}, {16, 17}};

uint8_t sw_stat[3][6] = {
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
};

int32_t enc_diff[4] = {0, 0, 0, 0};

uint8_t overwrite_feed = 0;
char tmp_str[100];

struct rx_data_t {
    float pos[6];
    int16_t ow_feed;
    int16_t ow_rapid;
    int16_t ow_spindle;
    uint16_t leds;
    float jog_scale;
    float spindle_speed;
    int16_t stats;
    int8_t tool;
    int8_t aux;
};

const int rx_data_t_size = sizeof(rx_data_t);

uint8_t last_mode = 255;
rx_data_t last_values;
float last_batt_voltage = -1.0;

union rx_Data_t{
    rx_data_t values;
    byte data[rx_data_t_size];
};
rx_Data_t rx_data;  

struct tx_data_t {
    int16_t jog;
    int16_t ow_feed;
    int16_t ow_rapid;
    int16_t ow_spindle;
    uint32_t buttons;
};
const int tx_data_t_size = sizeof(tx_data_t);

union tx_Data_t{
    tx_data_t values;
    byte data[tx_data_t_size];
};
tx_Data_t tx_data;  

uint8_t update = 0;

void display_clear() {
    tft.fillScreen(ST7735_BLACK);
    tft.setRotation(3);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

    tft.drawRect(1, 1, 202, 6 * PSTEP + 5 + 2, ST7735_WHITE);
    tft.drawRect(2, 2, 200, 6 * PSTEP + 5, ST7735_BLUE);

    tft.setTextSize(3);
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.setCursor(10, 10 + 0 * PSTEP);
    tft.print("X: ");
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.setCursor(10, 10 + 1 * PSTEP);
    tft.print("Y: ");
    tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
    tft.setCursor(10, 10 + 2 * PSTEP);
    tft.print("Z: ");
    tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    if ((rx_data.values.stats & (1<<0)) == 0) {
        tft.setCursor(10, 10 + 3 * PSTEP);
        tft.print("A: ");
        tft.setCursor(10, 10 + 4 * PSTEP);
        tft.print("B: ");
        tft.setCursor(10, 10 + 5 * PSTEP);
        tft.print("C: ");
    } else {
        tft.setTextSize(2);
        tft.setCursor(10, 10 + 4 * PSTEP + 4);
        tft.print("Speed:  ");
        tft.setCursor(10, 10 + 5 * PSTEP + 4);
        tft.print("Tool#:  ");
    }
    tft.setTextSize(3);
    tft.setCursor(10, 10 + 6 * PSTEP);

    uint8_t n = 0;
    for (n = 0; n < 6; n++) {
        last_values.pos[n] = rx_data.values.pos[n] + 1;
    }
    last_values.stats = rx_data.values.stats + 1;
    last_values.aux = rx_data.values.aux + 1;
    last_values.leds = rx_data.values.leds + 1;
    last_values.ow_feed = rx_data.values.ow_feed + 1;
    last_values.ow_rapid = rx_data.values.ow_rapid + 1;
    last_values.ow_spindle = rx_data.values.ow_spindle + 1;
    last_values.jog_scale = rx_data.values.jog_scale + 1;
    last_values.spindle_speed = rx_data.values.spindle_speed + 1;
    last_values.tool = rx_data.values.tool + 1;

}

void setup() {
    uint8_t n = 0;
    Serial.begin(115200);
    while (!Serial);
    Serial.println("PicoMPG OK");
    for (n = 0; n < 6; n++) {
        pinMode(col_pins[n], OUTPUT);
        digitalWrite(col_pins[n], 1);
    }
    pinMode(row_pins[0], INPUT_PULLUP);
    pinMode(row_pins[1], INPUT_PULLUP);
    pinMode(row_pins[2], INPUT_PULLUP);
    pinMode(row_pins[3], INPUT);

    // Latch Modes
    // • FOUR0: encoder pins are always LOW in latch position.
    // • FOUR3: encoder pins are always HIGH in latch position.
    // • TWO03: encoder pins are both LOW or HIGH in latch position.
    for (uint8_t e = 0; e < 4; e++){
        encoder[e] = new RotaryEncoder(
            encoder_pins[e][0],
            encoder_pins[e][1],
            RotaryEncoder::LatchMode::TWO03
        );
        // register interrupt routine
        attachInterrupt(digitalPinToInterrupt(encoder_pins[e][0]), handleLoop, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoder_pins[e][1]), handleLoop, CHANGE);
    }

    SPI.begin();
    // tft.begin(78000000);
    display_clear();
    
}


void matrix_read() {
    uint8_t sw = 0;
    uint8_t n = 0;
    digitalWrite(row_pins[3], 0);
    pinMode(row_pins[3], INPUT);
    for (sw = 0; sw < 6; sw++) {
        for (n = 0; n < 6; n++) {
            pinMode(col_pins[n], INPUT);
        }
        pinMode(col_pins[sw], OUTPUT);
        digitalWrite(col_pins[sw], 0);
        sw_stat[0][sw] = digitalRead(row_pins[0]);
        sw_stat[1][sw] = digitalRead(row_pins[1]);
        sw_stat[2][sw] = digitalRead(row_pins[2]);
    }


    for (n = 0; n < 6; n++) {
        pinMode(col_pins[n], INPUT);
    }
    pinMode(row_pins[3], OUTPUT);
    digitalWrite(row_pins[3], 1);

    for (n = 0; n < 6; n++) {
        if ((rx_data.values.leds & (1<<n)) != 0) {
            pinMode(col_pins[n], OUTPUT);
            digitalWrite(col_pins[n], 0);
        } else {
            pinMode(col_pins[n], INPUT);
        }
    }

}


void update_stats() {
    uint8_t n = 0;
    for (n = 0; n < 6; n++) {
        if (rx_data.values.pos[n] > 99999.0) {
            rx_data.values.pos[n] = 99999.0;
        }
        if (rx_data.values.pos[n] > 99999.0) {
            rx_data.values.pos[n] = 99999.0;
        }
    }

    tx_data.values.jog = encoder[0] -> getPosition();
    encoder[0] -> setPosition(0);

    tx_data.values.buttons = 0;
    for (n = 0; n < 6; n++) {
        if (sw_stat[0][n] == 0) {
            tx_data.values.buttons |= (1<<n);
        }
    }
    for (n = 0; n < 6; n++) {
        if (sw_stat[1][n] == 0) {
            tx_data.values.buttons |= (1<<(n+6));
        }
    }
    for (n = 0; n < 6; n++) {
        if (sw_stat[2][n] == 0) {
            tx_data.values.buttons |= (1<<(n+12));
        }
    }
}



void loop() {
    uint8_t sw = 0;
    uint8_t n = 0;
    uint8_t nr = 0;

    matrix_read();
    tx_data.values.ow_feed = encoder[1] -> getPosition();
    tx_data.values.ow_rapid = encoder[2] -> getPosition();
    tx_data.values.ow_spindle = encoder[3] -> getPosition();

    uint8_t rx_buffer[rx_data_t_size];
    uint8_t tx_buffer[tx_data_t_size];
    uint8_t rlen = Serial.readBytes(rx_buffer, rx_data_t_size);

    if (rlen == rx_data_t_size) {
        for (n = 0; n < rx_data_t_size; n++) {
            rx_data.data[n] = rx_buffer[n];
        }

        update_stats();

        for (n = 0; n < tx_data_t_size; n++) {
            tx_buffer[n] = tx_data.data[n];
        }
        Serial.write(tx_buffer, tx_data_t_size);

    }

    tft.setTextSize(2);
    tft.setTextColor(ST7735_BLUE, ST7735_BLACK);

    tft.setCursor(2, 5 + 4 * 45 + 25);
    if ((rx_data.values.stats & (1<<1)) != 0) {
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.print("On  ");
    } else {
        tft.setTextColor(ST7735_RED, ST7735_BLACK);
        tft.print("Off ");
    }
    tft.setCursor(65, 5 + 4 * 45 + 25);
    if ((rx_data.values.stats & (1<<2)) != 0) {
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.print("RUN  ");
    } else {
        tft.setTextColor(ST7735_RED, ST7735_BLACK);
        tft.print("STOP ");
    }

    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.setCursor(142, 5 + 4 * 45 + 25);
    if ((rx_data.values.stats & (1<<3)) != 0) {
        tft.print("AUTO ");
    } else if ((rx_data.values.stats & (1<<4)) != 0) {
        tft.print("MAN  ");
    } else if ((rx_data.values.stats & (1<<5)) != 0) {
        tft.print("MDI  ");
    } else {
        tft.setTextColor(ST7735_RED, ST7735_BLACK);
        tft.print(" --  ");
    }

    uint8_t num_axis = 6;
    // display-mode
    if ((rx_data.values.stats & (1<<0)) == 0) {
        if (last_mode != 0) {
            last_mode = 0;
            display_clear();
        }
        // axis positions and homed
        tft.setTextSize(3);
        for (n = 0; n < 6; n++) {
            if (last_values.leds != rx_data.values.leds || last_values.pos[n] != rx_data.values.pos[n]) {
                last_values.pos[n] = rx_data.values.pos[n];
                if ((rx_data.values.leds & (1<<(8+n))) != 0) {
                    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
                } else {
                    tft.setTextColor(ST7735_RED, ST7735_BLACK);
                }
                tft.setCursor(50, 10 + n * PSTEP);
                sprintf(tmp_str, "%08.2f", rx_data.values.pos[n]);
                tft.print(tmp_str);
            }
            last_values.leds = rx_data.values.leds;
        }
    } else {
        if (last_mode != 1) {
            last_mode = 1;
            display_clear();
        }
        // axis positions and homed
        tft.setTextSize(3);
        for (n = 0; n < 3; n++) {
            if (last_values.leds != rx_data.values.leds || last_values.pos[n] != rx_data.values.pos[n]) {
                last_values.pos[n] = rx_data.values.pos[n];
                if ((rx_data.values.leds & (1<<(8+n))) != 0) {
                    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
                } else {
                    tft.setTextColor(ST7735_RED, ST7735_BLACK);
                }
                tft.setCursor(50, 10 + n * PSTEP);
                sprintf(tmp_str, "%08.2f", rx_data.values.pos[n]);
                tft.print(tmp_str);
            }
            last_values.leds = rx_data.values.leds;
        }

        // coolant mist/flood
        tft.setTextSize(1);
        tft.setCursor(90, 10 + 5 * PSTEP - 1);
        if ((rx_data.values.stats & (1<<6)) != 0) {
            tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        } else {
            tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        }
        tft.print("mist");

        tft.setCursor(90, 10 + 5 * PSTEP + 10);
        if ((rx_data.values.stats & (1<<7)) != 0) {
            tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        } else {
            tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        }
        tft.print("flood");

        // speed
        tft.setTextSize(3);
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        if (last_values.spindle_speed != rx_data.values.spindle_speed) {
            tft.setCursor(105, 10 + 4 * PSTEP);
            sprintf(tmp_str, "%05.0f", rx_data.values.spindle_speed * 60.0);
            tft.print(tmp_str);
        }
        last_values.spindle_speed = rx_data.values.spindle_speed;
        // tool
        if (last_values.tool != rx_data.values.tool) {
            tft.setCursor(105, 10 + 5 * PSTEP);
            sprintf(tmp_str, "  %03.0f", (float)(rx_data.values.tool));
            tft.print(tmp_str);
        }
        last_values.tool = rx_data.values.tool;
    }

    if (last_values.ow_feed != rx_data.values.ow_feed) {
        last_values.ow_feed = rx_data.values.ow_feed;
        nr = 0;
        tft.setTextSize(1);
        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.setCursor(215, 5 + nr * 47 + 2);
        tft.print("Feed:");
        tft.setTextSize(3);
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(230+16, 5 + nr * 45 + 25);
        sprintf(tmp_str, "%3i%%", rx_data.values.ow_feed);
        tft.print(tmp_str);
    }

    if (last_values.ow_rapid != rx_data.values.ow_rapid) {
        last_values.ow_rapid = rx_data.values.ow_rapid;
        nr = 1;
        tft.setTextSize(1);
        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.setCursor(215, 5 + nr * 47 + 2);
        tft.print("Rapid:");
        tft.setTextSize(3);
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(230+16, 5 + nr * 45 + 25);
        sprintf(tmp_str, "%3i%%", rx_data.values.ow_rapid);
        tft.print(tmp_str);
    }

    if (last_values.ow_spindle != rx_data.values.ow_spindle) {
        last_values.ow_spindle = rx_data.values.ow_spindle;
        nr = 2;
        tft.setTextSize(1);
        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.setCursor(215, 5 + nr * 47 + 2);
        tft.print("Spindle:");
        tft.setTextSize(3);
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(230+16, 5 + nr * 45 + 25);
        sprintf(tmp_str, "%3i%%", rx_data.values.ow_spindle);
        tft.print(tmp_str);
    }

    if (last_values.jog_scale != rx_data.values.jog_scale) {
        last_values.jog_scale = rx_data.values.jog_scale;
        nr = 4;
        tft.setTextSize(1);
        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.setCursor(215, 5 + nr * 47 + 2);
        tft.print("Jog-Scale:");
        tft.setTextSize(3);
        tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
        tft.setCursor(230, 5 + nr * 45 + 25);
        sprintf(tmp_str, "%04.3f", rx_data.values.jog_scale);
        tft.print(tmp_str);
    }



    delay(5);

}
