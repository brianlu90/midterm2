#include "mbed.h"
#include <cmath>
#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"
#include "uLCD_4DGL.h"
#include "mbed_events.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
using namespace std::chrono;

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

InterruptIn button(USER_BUTTON);
uLCD_4DGL uLCD(D1, D0, D2);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
BufferedSerial pc(USBTX, USBRX);

WiFiInterface *wifi;
int PredictGesture(float* output);
int GestureOutput(void);
//void ulcd_draw(int angle, int conf);
//void Gesture_detect(void);
void Gesture_main(void);
//void confirm(void);
//void AccInit(int16_t *accInit);
//void Angle_main(void);
//void AccGet(void);
void AngleMsr(Arguments *in, Reply *out);
void GestureUI(Arguments *in, Reply *out);
void GestureUI_End(Arguments *in, Reply *out);
RPCFunction rpcGesture(&GestureUI, "GestureUI");
RPCFunction rpcGesture_End(&GestureUI_End, "GestureUI_End");
RPCFunction rpcfeature(&AngleMsr, "Feature");
//RPCFunction rpcfeature_End(&AngleMsr_End, "Feature_End");
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client, int result, int select);
void messageArrived(MQTT::MessageData& md);
void close_mqtt(void);

Thread Gesture;
Thread Angle;
Thread GestureDct;
EventQueue gestureQueue;
EventQueue gestureDctQueue;
EventQueue angleQueue;
Thread message;
EventQueue mqtt_queue;

int ges = 0;
int sel = 0;
int angle_over  = 0;
int state = 0;
int mode1 = 0;
int mode2 = 0;
int data_length = 0;
float AccData[700];
int feature[30];
const char* topic1 = "Cls Gesture";
const char* topic2 = "Ext Feature";
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
MQTT::Client<MQTTNetwork, Countdown> *client_global;

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

int main()
{
  wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }

    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.162";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic1, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    if (client.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    message.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    client_global = &client;
 /*****************************************************************/
 /*****************************************************************/
    BSP_ACCELERO_Init();
    /*uLCD.reset();
    button.rise(&confirm);*/
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    led1 = 0;
    led2 = 1;
    led3 = 0;
    while (true) {
      if (closed) break;
      memset(buf, 0, 256);      // clear buffer
      for(int i=0; i<255; i++) {
          char recv = fgetc(devin);
          if (recv == '\r' || recv == '\n') {
            printf("\r\n");
            break;
          }
          buf[i] = fputc(recv, devout);
      }
      RPC::call(buf, outbuf);
    }
/**************************************************************/
/**************************************************************/
    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic1)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.unsubscribe(topic2)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
}

int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

int GestureOutput(void) {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  //error_reporter->Report("Set up successful...\n");

  while (true) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);
///////////////////////////////////////////
  memset(AccData, 0, 700);
  for (int i = 0; i < input_length; i++) {
    AccData[i] = model_input->data.f[i];
  }
  data_length = input_length;
//////////////////////////////////////////////

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
      return gesture_index;
      //error_reporter->Report(config.output_message[gesture_index]);
    }
  }
}

/*void ulcd_draw(int angle, int conf) {
  if (conf == 0) {
    if (angle == 0) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(0xFFFFFF);
      uLCD.printf("30\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("45\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("60\n");
    } else if (angle == 1) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(0x000000);
      uLCD.printf("30\n");
      uLCD.textbackground_color(0xFFFFFF);
      uLCD.printf("45\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("60\n");
    } else if (angle == 2) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(0x000000);
      uLCD.printf("30\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("45\n");
      uLCD.textbackground_color(0xFFFFFF);
      uLCD.printf("60\n");
    } else {
      uLCD.locate(0,0);
      uLCD.printf("ERROR 1\n");
    }
  } else if (conf == 1) {
    if (angle == 0) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(BLUE);
      uLCD.printf("30\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("45\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("60\n");
    } else if (angle == 1) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(0x000000);
      uLCD.printf("30\n");
      uLCD.textbackground_color(BLUE);
      uLCD.printf("45\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("60\n");
    } else if (angle == 2) {
      uLCD.locate(0,0);
      uLCD.textbackground_color(0x000000);
      uLCD.printf("30\n");
      uLCD.textbackground_color(0x000000);
      uLCD.printf("45\n");
      uLCD.textbackground_color(BLUE);
      uLCD.printf("60\n");
    } else {
      uLCD.locate(0,0);
      uLCD.printf("ERROR 2\n");
    }
  } else {
    uLCD.locate(0,0);
    uLCD.printf("ERROR 3\n");
  }
}*/

/*void Gesture_detect(void)
{
  if (mode1 == 0) {
    state = 1;
    ges = GestureOutput();  // get gesture result
    state = 0;
    /*if ((ges == 0) && (state == 0) && (mode1 == 0)) {
      ulcd_draw(0, 0);
    } else if ((ges == 1) && (state == 0) && (mode1 == 0)) {
      ulcd_draw(1, 0);
    } else if ((ges ==2) && (state == 0) && (mode1 == 0)){
      ulcd_draw(2, 0);
    }
  }
}*/

void GestureUI(Arguments *in, Reply *out)
{
  mode1 = 0;
  Gesture.start(callback(&gestureQueue, &EventQueue::dispatch_forever));
  gestureQueue.call(&Gesture_main);
}

void Gesture_main(void)
{
  char buffer[200];
  int i, j;
  int run_times;
  float V_init, V_data, inr_pdct;
  int deg_x, deg_y, deg_z, predict = 0;

  memset(feature, 0, 30);
  while (mode1 == 0) {
    ges = 0;
    Angle.start(callback(&angleQueue, &EventQueue::dispatch_forever));
    GestureDct.start(callback(&gestureDctQueue, &EventQueue::dispatch_forever));
    run_times = 0;
    while ((run_times < 20) && (mode1 == 0)) {
      ges = GestureOutput();
      led1 = 0;
      led2 = 0;
      led3 = 0;
      if (ges == 0) {
        led1 = 1;
      } else if (ges == 1) {
          led2 = 1;
      } else {
          led3 = 1;
      }
      mqtt_queue.call(&publish_message, client_global, ges, 0);
      predict = 0;
      for (i = 0, j = 1; i < data_length - 6; i += 3, j++) {
        V_init = sqrt(AccData[i] * AccData[i]
                    + AccData[i + 1] * AccData[i + 1]
                    + AccData[i + 2] * AccData[i + 2]);
        V_data = sqrt(AccData[i + 3] * AccData[i + 3]
                    + AccData[i + 4] * AccData[i + 4]
                    + AccData[i + 5] * AccData[i + 5]);
        inr_pdct = (AccData[i] * AccData[i + 3]
                  + AccData[i + 1] * AccData[i + 4]
                  + AccData[i + 2] * AccData[i + 5]);
        deg_z = acos(inr_pdct / (V_init * V_data)) * 180.0 / 3.141;
        //feature[j] = (deg > 30);
        for (int j = 1; j < 15; j++) {
          predict += (deg_z > (j * 10));
        }
      }
      feature[run_times] = predict;
      //printf("%d %d\n", ges, predict);
      run_times++;
      ThisThread::sleep_for(500ms);
    }
    ThisThread::sleep_for(5s);
  }
}

void GestureUI_End(Arguments *in, Reply *out)
{
  mode1 = 1;
  /*for (int i = 0; i < 30; i++) {
    printf("%d ", feature[i]);
  }
  printf("\n");*/
  led1 = 0;
}


void messageArrived(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client, int result, int select) {
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "%d", result);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc;
    if (select == 0) {
        rc = client->publish(topic1, message);
    } else {
        rc = client->publish(topic2, message);
    }
    
    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %d %d\r\n", select, result);
}

void close_mqtt() {
    closed = true;
}

/*void AccGet(Arguments *in, Reply *out)
{
  mode2 = 0;
  Angle.start(callback(&angleQueue, &EventQueue::dispatch_forever));
  angleQueue.call(&Angle_main);
}*/

/*void Angle_main(void)
{
  int16_t accData[3] = {0};
  float V_init, V_data, inr_pdct;
  int deg;
  char tilt_angle[200];
  int16_t AccData[3] = {0};

    //AccData(AccData);
    uLCD.cls();
    uLCD.textbackground_color(0x000000);
    angle_over = 0;
    led3 = 1;
    while ((angle_over == 0) && (mode2 == 0)) {
      BSP_ACCELERO_AccGetXYZ(accData);
      V_init = sqrt(AccData[0] * AccData[0]
                    + AccData[1] * AccData[1]
                    + AccData[2] * AccData[2]);
      V_data = sqrt(accData[0] * accData[0]
                    + accData[1] * accData[1]
                    + accData[2] * accData[2]);
      inr_pdct = (AccData[0] * accData[0]
                  + AccData[1] * accData[1]
                  + AccData[2] * accData[2]);
      deg = acos(inr_pdct / (V_init * V_data)) * 180.0 / 3.141;
      sprintf(tilt_angle, "angle = %d", deg);
      uLCD.locate(0,0);
      uLCD.printf("%3d degree\n", deg);
      if (sel == 0) {
        if (deg > 30) {
          angle_over = 1;
        } 
      } else if (sel == 1) {
        if (deg > 45) {
          angle_over = 1;
        }
      } else if (sel == 2) {
        if (deg > 60) {
          angle_over = 1;
        }
      }
    }
    if (mode2 == 0) {
      mqtt_queue.call(&publish_message, client_global, tilt_angle, 1);
    }
  }
    while (state == 0);
    while (state == 1) {
      BSP_ACCELERO_AccGetXYZ(accData);
      ThisThread::sleep_for(50ms);
    }
}*/

/*void AccInit(int16_t *accInit)
{
  int i = 0;
  
  while (i < 10) {
    led3 = 1;
    ThisThread::sleep_for(200ms);
    led3 = 0;
    ThisThread::sleep_for(200ms);
    i++;
  }
  BSP_ACCELERO_AccGetXYZ(accInit);
}*/

void AngleMsr(Arguments *in, Reply *out)
{
  for (int i = 0; i < 7; i++) {
    mqtt_queue.call(&publish_message, client_global, feature[i], 1);
  }
}


