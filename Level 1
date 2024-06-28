/*
//Session 1 ino version


#define led 14  // LED is connected to GPIO 14
void setup(){
pinMode(led, OUTPUT); // Configure LED pin as output
}
void loop(){
digitalWrite(led, HIGH); // Turn LED on (HIGH)
delay(2000); // Delay for 2 seconds
digitalWrite(led, LOW); // Turn LED off (LOW)
delay(1000); // Delay for 1 second
}


//Session 1 embedded c version


#include <esp_idf_lib.h>
#include <driver/gpio.h>

#define LED_GPIO GPIO_NUM_14  // LED is connected to GPIO 14

void app_main(void)
{
    // Configure LED pin as output
    gpio_config_t io_conf = {};
    io_conf.pin_bit = (1ULL << LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);


    while (true) {
        gpio_set_level(LED_GPIO, 1); // Turn LED on (HIGH)
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds

        gpio_set_level(LED_GPIO, 0); // Turn LED off (LOW)
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
return 0 ;
}


//Session 2 ino version


#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}


//Session 2 embedded c version


#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_util.h>
#include <esp_spp_api.h>

#define SERVICE_UUID { /* SPP UUID (e.g., 0x000F, 0x0000) */ }  // Replace with actual SPP UUID
#define DEVICE_NAME "ESP32test"  // Bluetooth device name

// Connection status flag
static bool spp_connected = false;

// Data received callback function
static void spp_data_recv_cb(uint8_t *data, uint32_t len) {
  // Process received data (e.g., send to serial port)
  Serial.write(data, len);
}

// Connection established callback function
static void spp_conn_estab_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    spp_connected = true;
    Serial.println("Connected to Bluetooth device!");
  } else if (event == ESP_SPP_SRV_DISCONNECT_EVT) {
    spp_connected = false;
    Serial.println("Disconnected from Bluetooth device!");
  }
}

void setup() {
  Serial.begin(115200);

  esp_bt_controller_config_t cfg = BT_CONTROLLER_CONFIG_INIT();
  esp_bt_controller_init(&cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

  esp_bt_uuid_t spp_uuid = SERVICE_UUID;
  esp_spp_dev_t spp_dev = {
      .spp_service_uuid = spp_uuid,
      .spp_reg_param = {
          .spp_appearance = 0,
      },
  };

  esp_spp_profile_cb_t spp_profile_cb = {
      .on_connect = spp_conn_estab_cb,
      .on_disconnect = spp_conn_estab_cb,
      .on_data_received = spp_data_recv_cb,
  };

  // Register SPP service
  esp_bt_gap_create_conn_params_t conn_params = ESP_BT_GAP_CONNECT_PARAM_INIT();
  esp_bt_gap_register_service(esp_spp_init, &spp_profile_cb, &spp_uuid, &spp_dev, &conn_params);

  Serial.printf("ESP32 device name: %s\n", DEVICE_NAME);
}

void loop() {
  if (spp_connected) {
    // Check for data to send to Bluetooth device (e.g., read from serial port)
    if (Serial.available()) {
      uint8_t data = Serial.read();
      esp_spp_write(0, &data, 1);  // Send data to connected device (SPP channel 0)
    }
  }
  delay(100);
}


//Session 3 ino version


// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}


//Session 3 embedded c version


#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_webserver.h>
#include <driver/gpio.h>

// Define Wi-Fi network credentials (replace with yours)
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Web server port
#define WEB_SERVER_PORT 80
static esp_webserver_t *server;

// Output pin definitions and initial states
const int output26 = 26;
const int output27 = 27;
String output26State = "off";
String output27State = "off";

// Timeout value in milliseconds
const long timeoutTime = 2000;
unsigned long currentTime = 0;
unsigned long previousTime = 0;

static const char* TAG = "web_server";

// Function to connect to Wi-Fi
static void connect_to_wifi() {
  esp_wifi_init(&wifi_init_config);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(wifi_config, ssid, password);
  esp_wifi_start();
  ESP_LOGI(TAG, "Connecting to %s...", ssid);

  while (esp_wifi_connect() != WIFI_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, ".");
  }

  ESP_LOGI(TAG, "Connected with IP Address:");
  ESP_LOGI(TAG, "%s", esp_interface_get_mac_addr_string(ESP_IF_WIFI_STA));
}

// Function to handle root path requests (/)
static void handle_root(httpd_req_t *req) {
  char temp[100];
  int content_length = snprintf(temp, sizeof(temp), 
      "<!DOCTYPE html><html><head><title>ESP32 Web Server</title>\n"
      "... (remaining HTML content with button styling) ..." // Replace with your desired HTML content
      "<p>GPIO 26 - State %s</p>\n"
      "... (logic to display current state and buttons for GPIO 26/27) ..."
      "</body></html>", output26State.c_str(), output27State.c_str());
  httpd_resp_send_chunk(req, temp, content_length, true);
  
  // ... (logic to handle GPIO control based on request path) ...
  
  httpd_resp_send_chunk(req, NULL, 0, true);  // Send end-of-response
}

// Function to handle not found requests
static void handle_not_found(httpd_req_t *req) {
  httpd_resp_send_404(req);
}

// Web server event handler
static esp_err_t event_handler(httpd_event_t event, void* ctx, httpd_req_t* req) {
  if (event == HTTPD_EVENT_REQUEST) {
    httpd_uri_t uri = *httpd_req_get_uri(req);
    if (httpd_uri_strcmp(uri, "/") == 0) {
      handle_root(req);
    } else {
      handle_not_found(req);
    }
  }
  return ESP_OK;
}

// Setup function
static void setup() {
  Serial.begin(115200);

  // Initialize GPIO pins as outputs
  gpio_set_direction(output26, GPIO_MODE_OUTPUT);
  gpio_set_direction(output27, GPIO_MODE_OUTPUT);

  // Set initial output states
  gpio_set_level(output26, LOW);
  gpio_set_level(output27, LOW);

  // Connect to Wi-Fi
  connect_to_wifi();

  // Initialize the web server
  server = httpd_start(WEB_SERVER_PORT, event_handler);
  if (server != NULL) {
    ESP_LOGI(TAG, "Web Server Started!");
  } else {
    ESP_LOGE(TAG, "Failed to Start Web Server!");
  }
}


//Session 4 ino version


//rgb led


#define PIN_RED    23 // GPIO23
#define PIN_GREEN  22 // GPIO22
#define PIN_BLUE   21 // GPIO21

void setup() {
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);
}

void loop() {
  // color code #00C9CC (R = 0,   G = 201, B = 204)
  setColor(0, 201, 204);

  delay(1000); // keep the color 1 second

  // color code #F7788A (R = 247, G = 120, B = 138)
  setColor(247, 120, 138);

  delay(1000); // keep the color 1 second

  // color code #34A853 (R = 52,  G = 168, B = 83)
  setColor(52, 168, 83);

  delay(1000); // keep the color 1 second
}

void setColor(int R, int G, int B) {
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}


//key pad


#include <Keypad.h>

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // four columns

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_rows[ROW_NUM]      = {19, 18, 5, 17}; // GPIO19, GPIO18, GPIO5, GPIO17 connect to the row pins
byte pin_column[COLUMN_NUM] = {16, 4, 0, 2};   // GPIO16, GPIO4, GPIO0, GPIO2 connect to the column pins

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

void setup() {
  Serial.begin(9600);
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    Serial.println(key);
  }
}


//Session 4 embedded c version


//rgb led


#include <esp32/rom/ets.h> // For `ets_intr_lock` and `ets_intr_unlock`
#include <driver/gpio.h>   // For `gpio_set_direction` (optional, if using GPIO configuration)
#define PIN_RED   23
#define PIN_GREEN 22
#define PIN_BLUE  21
#define R_REG(reg)   *((volatile uint32_t*)(reg))
#define W_REG(reg, val)  R_REG(reg) = (val)
void configure_pins_as_outputs() {
    ets_intr_lock(); // Disable interrupts for register access

    // Set pin function registers (example for GPIO23-21)
    R_REG(GPIO_FUNC0_OUT_REG) &= ~((1 << PIN_RED) | (1 << PIN_GREEN) | (1 << PIN_BLUE));
    R_REG(GPIO_FUNC0_OUT_CONF_REG) |= ((1 << PIN_RED) | (1 << PIN_GREEN) | (1 << PIN_BLUE));

    ets_intr_unlock(); // Re-enable interrupts
}
void set_pwm_duty_cycle(int pin, int duty_cycle) {
    // Adjust these values based on your timer configuration and desired PWM frequency
    const int timer_num = 0;
    const int timer_channel = (pin == PIN_RED) ? 0 : ((pin == PIN_GREEN) ? 1 : 2); // Map pin to timer channel
    const int timer_period = 1000; // Adjust for desired PWM frequency (e.g., 1 kHz for 1000 Hz)
    const int duty_cycle_max = timer_period - 1; // Maximum duty cycle value

    duty_cycle = duty_cycle > duty_cycle_max ? duty_cycle_max : duty_cycle; // Clamp duty


//key pad


#include <esp_event.h>
#include <esp_log.h>
#include <driver/gpio.h>

#define ROW_NUM 4  // Number of rows
#define COL_NUM 4  // Number of columns

char keymap[ROW_NUM][COL_NUM] = {  // Key mapping
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

const int row_pins[ROW_NUM] = {19, 18, 5, 17};  // Row pin connections (adjust as needed)
const int col_pins[COL_NUM] = {16, 4, 0, 2};   // Column pin connections (adjust as needed)

char scan_keypad() {
  char key = 0;

  // Set all row pins to output (high)
  for (int i = 0; i < ROW_NUM; i++) {
    gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
    gpio_set_level(row_pins[i], 1);
  }

  // Scan each column for pressed key
  for (int j = 0; j < COL_NUM; j++) {
    gpio_set_direction(col_pins[j], GPIO_MODE_INPUT);  // Set column pin as input
    gpio_set_pull_type(col_pins[j], GPIO_PULLUP);         // Enable pull-up resistor

    // Check if any row pin is low (indicating a pressed key)
    for (int i = 0; i < ROW_NUM; i++) {
      if (gpio_get_level(row_pins[i]) == 0) {
        key = keymap[i][j];  // Get the corresponding key from the mapping
        break;
      }
    }

    gpio_set_direction(col_pins[j], GPIO_MODE_OUTPUT);  // Set column pin back to output (high)
  }

  // Set all row pins back to input (high impedance)
  for (int i = 0; i < ROW_NUM; i++) {
    gpio_set_direction(row_pins[i], GPIO_MODE_INPUT);
  }

  return key;
}

static void setup() {
  Serial.begin(9600);

  // Set all row and column pins to output (initial state)
  for (int i = 0; i < ROW_NUM; i++) {
    gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
  }
  for (int i = 0; i < COL_NUM; i++) {
    gpio_set_direction(col_pins[i], GPIO_MODE_OUTPUT);
  }
}

static void loop() {
  char key = scan_keypad();
  if (key) {
    Serial.println(key);
  }
  delay(100);  // Debounce delay (adjust as needed)
}

*\

