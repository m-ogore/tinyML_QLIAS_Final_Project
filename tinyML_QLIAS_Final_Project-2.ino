#include <DHT.h>  // Including library for dht

//include edge impulse library here
#include <Quality_Leaf_IoT_Assessment_System_inferencing.h>

#define THRESHOLD 0.7

/////////////////////////////////////////////////////////////////
#define FREQUENCY_HZ        166
#define INTERVAL_MS         (100000 / (FREQUENCY_HZ + 1))
/////////////////////////////////////////////////////////////////


static unsigned long last_interval_ms = 0;
// to classify 1 frame of data you need EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE values
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// keep track of where we are in the feature array
size_t feature_ix = 0;


#define button1 D5
bool button_State;

// Assign pin to DHT11 Sensor pin
#define DHTPIN D7          //pin where the dht11 is connected
DHT dht(DHTPIN, DHT11);

// RGB pin diclaration
#define redPin D2// pin D0 = GPIO 16;
#define greenPin D3// pin SD2 = GPIO 9;
#define bluePin D4 // pin SD3 = GPIO 10;

// TCS3200 RGB color sensor pin diclaration
#define s0 D9  
#define s1 D8 
#define s2 D11 
#define s3 D10 
#define out D12

// Diclare PW Max and Min
int redMin = 43;
int redMax = 172;
int greenMin = 58;
int greenMax = 246;
int blueMin = 48;
int blueMax = 186;

// Declare color Variables
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

// RGB LED variable assignation
int red;
int green;
int blue;
float hum;
float temp;


int* redValue;
int* greenValue;
int* blueValue;
float* h; 
float* t; 



// Display scroll text on the LCD
char * messagePadded = "                    QLIAS System.                ";
// String variable for sending SMS
String Data_SMS;

void setup() {
  
  Serial1.begin(9600); // Establish GSM SIM800L Connection
  Serial.begin(9600);

  pinMode(button1, INPUT_PULLUP);
  
  dht.begin(); // Establish dht11 starting up

  // Assign RGB LED pin Output    
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
        
  // Assign TCS3200 color sensor pin Input and pin Output 
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  
  pinMode(out, INPUT);  
  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);
   
}

void loop() {

    static unsigned long last_interval_ms = 0;
    
  tinyML();

    
  }

void tinyML(){
    String leaf_health;
      if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();
        
        readSensor();

  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE); 
  Serial.println("TEMPERATURE IS");
  Serial.println(*t);
 


        // keep filling the features array until it's full
        features[feature_ix++] = *t;
        features[feature_ix++] = *h;
        features[feature_ix++] = *redValue;
        features[feature_ix++] = *greenValue;
        features[feature_ix++] = *blueValue;
   // ei_printf("Edge Impulse standalone inferencing (Arduino)\n");


        // features buffer full? then classify!
        if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_impulse_result_t result;

            // create signal from features frame
            signal_t signal;
            numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

            // run classifier
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            ei_printf("run_classifier returned: %d\n", res);
            if (res != 0) return;

            // print predictions
            ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

            // print the predictions
            
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);

                                                  

                 
               if (result.classification[ix].value > THRESHOLD)
                  { 
                    leaf_health = result.classification[ix].label;
            
                  }

            }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("anomaly:\t%.3f\n", result.anomaly);
        //    lcd.print(result.anomaly);
        //oled.clear();
  //oled.set1X();
  //oled.print(result.anomaly);

        #endif

            // reset features frame
            feature_ix = 0;
        }
        
            SendMessage(leaf_health, t, h, redValue, greenValue, blueValue);
    }

  }
void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}




void readSensor(){  
  redPW = getRedPW();
  if(red > green && green > blue){
    red = 255;
    green = green/2;
    blue = 0;
  }
  if(red > blue && blue > green){
    red = 255;
    blue = blue/2;
    green = 0;
  }
  redValue = &red;
  greenValue = &green;
  blueValue = &blue;
  delay(200);
  
  greenPW = getGreenPW();
  if(green > red && red > blue){
    green = 255;
    red = red/2;
    blue = 0;
  }
  if(green > blue && blue > red){
    green = 255;
    blue = blue/2;
    red = 0;
  }

  redValue = &red;
  greenValue = &green;
  blueValue = &blue;
  delay(200);

  bluePW = getBluePW();
  if(blue > red && red > green){
    blue = 255;
    red = red/2;
    green = 0;
  }
      
  if(blue > green && green > red){
    blue = 255;
    green = green/2;
    red = 0;
  }
  redValue = &red;
  greenValue = &green;
  blueValue = &blue;

  delay(200);

  hum = dht.readHumidity();
  temp = dht.readTemperature();

  t = &temp;
  h = &hum;

  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(*t);
  Serial.print(" degrees Celcius, Humidity: ");
  Serial.println(*h);
  Serial.print("RedColor = ");
  Serial.print(*redValue);
  Serial.print(" - GreenColor = ");
  Serial.print(*greenValue);
  Serial.print(" - BlueColor = ");
  Serial.println(*blueValue);
  
  analogWrite(redPin, *redValue);
  analogWrite(bluePin, *blueValue);
  analogWrite(greenPin, *greenValue);
  

}

//void SendMessage(String result){ // Accept the output string of the tinyML inference
void SendMessage(String result, float* tem, float* hum, int* redV,int* greenV, int* blueV) {
  Serial.println("Sending Text...");
  Serial1.print("AT+CMGF=1\r"); // Set the shield to SMS mode
  delay(100);
  Serial1.print("AT+CMGS=\"+250739377380\"\r");  
  delay(500); 
  Data_SMS = "Temp = " +String(*t,1)+" C" + "\nHumudity = "+String(*h,1)+ "%" + "\nResult = "+result+"";
  Serial1.println(Data_SMS); //the content of the message
  //Serial1.print("\r"); 
  delay(500);
  Serial1.print((char)26);//the ASCII code of the ctrl+z is 26 (required according to the datasheet)
  delay(100);
  Serial1.println();
  Serial.println("Text Sent.");
  delay(500);
  Serial1.println("AT+CIPSPRT=0");
  delay(100);
  ShowSerialData();
  Serial1.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//start up the connection
  delay(500);
  ShowSerialData();
  Serial1.println("AT+CIPSEND");//begin send data to remote server
  delay(4000);
  ShowSerialData();
  String str="GET https://api.thingspeak.com/update?api_key=KMK97V6ZYF2ERUEB&field1=" + String(*t) +"&field2="+String(*h) +"&field3="+String(*redValue) +"&field4="+String(*greenValue) +"&field5="+String(*blueValue);
  //Serial.println(str);
  Serial1.println(str);//begin send data to remote server
  delay(100);
  ShowSerialData();
  Serial1.println((char)26);//sending
  delay(100); //waitting for reply, important the time is base on the condition of internet 
  Serial1.println();
  ShowSerialData();
  Serial1.println("AT+CIPSHUT"); //close the connection
  delay(100);
  ShowSerialData();
  delay(500);
}

int getRedPW(){
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);
  int PW; 
  PW = pulseIn(out, LOW);
  red = 255 - PW;
}
  
int getGreenPW(){
  digitalWrite(s2, HIGH);  
  digitalWrite(s3, HIGH);
  int PW; 
  PW = pulseIn(out, LOW);
  green = 255 - PW;
}

int getBluePW(){
  digitalWrite(s2, LOW);  
  digitalWrite(s3, HIGH);
  int PW; 
  PW = pulseIn(out, LOW);
  blue = 255 - PW;
}

void ShowSerialData() {
  while(Serial1.available()!=0)
  Serial.write(Serial1.read());
  delay(5000); 
  
}
