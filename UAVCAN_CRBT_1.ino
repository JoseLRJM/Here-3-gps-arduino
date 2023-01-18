#include <CAN.h>

#define id_estado_nodo            0x1001550A
#define time_to_pub_estado_nodo   1000 

///// variables para publicar estado del nodo ESP32
uint32_t time_estado_nodo = 0;
byte health_byte;
unsigned long last_time_to_pub_estado_nodo =0;

uint32_t CAN_id_frame;
int CAN_tamano_frame;
byte CAN_frame[8];
int fase_DNA = 0;
byte DNA_UNI_ID[19];

bool CAN_nuevo_frame_dan = false;
bool CAN_nuevo_frame_gps = false;
bool CAN_nuevo_frame_generico = false;
byte i = 0;

uint16_t crcAddByte(uint16_t crc_val, uint8_t byte){
    crc_val ^= (uint16_t) ((uint16_t) (byte) << 8U);
    for (uint8_t j = 0; j < 8; j++)
    {
        if (crc_val & 0x8000U)
        {
            crc_val = (uint16_t) ((uint16_t) (crc_val << 1U) ^ 0x1021U);
        }
        else
        {
            crc_val = (uint16_t) (crc_val << 1U);
        }
    }
    return crc_val;
}

uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature){
    for (uint16_t shift_val = 0; shift_val < 64; shift_val = (uint16_t)(shift_val + 8U))
    {
        crc_val = crcAddByte(crc_val, (uint8_t) (data_type_signature >> shift_val));
    }
    return crc_val;
}

uint16_t crcAdd(uint16_t crc_val, const uint8_t* bytes, size_t len){
    while (len--)
    {
        crc_val = crcAddByte(crc_val, *bytes++);
    }
    return crc_val;
}

void get_can(int packetSize){
    CAN_id_frame = CAN.packetId();
    CAN_tamano_frame = packetSize;
    for(int i=0; i<packetSize ; i++){CAN_frame[i] = CAN.read();}
    
    if((CAN_id_frame & 0xFF0000FF) == 0x18000000){
      CAN_nuevo_frame_dan = true;
    }else if(CAN_id_frame == 0X804277D){
      CAN_nuevo_frame_gps = true;
    }else if(CAN_id_frame == 0X803E97D){
      
    }else{
      CAN_nuevo_frame_generico = true;
    }
}

void DNA_CAN(){
  if(fase_DNA == 0){   
      byte DNA_frame_0[8] = {0x00, CAN_frame[1], CAN_frame[2], CAN_frame[3], CAN_frame[4], CAN_frame[5], CAN_frame[6], 0xC0};
      sen_can_frame(0x1000010A, DNA_frame_0 , 8);
      DNA_UNI_ID [0]= 0xA3;
      DNA_UNI_ID [1]= 0x39;
      DNA_UNI_ID [2]= 0x00;
      for(int i=1; i < 7; i++) {DNA_UNI_ID [2 + i] = CAN_frame[i];}
      fase_DNA = 1;
  }
  else if(fase_DNA == 1){ 
      for(int i=1; i < 7; i++) {DNA_UNI_ID [8 + i] = CAN_frame[i];}
      uint16_t crc = 0xFFFFU;
      crc = crcAdd(crc, DNA_UNI_ID, 15);
      byte DNA_frame_1A[8] = {(uint8_t) crc, (uint8_t) (crc >> 8U), 0x00, DNA_UNI_ID [3], DNA_UNI_ID [4],DNA_UNI_ID [5], DNA_UNI_ID [6], 0x81};
      byte DNA_frame_1B[8] = {DNA_UNI_ID [7], DNA_UNI_ID [8], DNA_UNI_ID [9], DNA_UNI_ID [10], DNA_UNI_ID [11], DNA_UNI_ID [12], DNA_UNI_ID [13], 0x21};   
      byte DNA_frame_1C[2] = {DNA_UNI_ID [14], 0x41};
      sen_can_frame(0x1000010A, DNA_frame_1A, 8);
      sen_can_frame(0x1000010A, DNA_frame_1B, 8);
      sen_can_frame(0x1000010A, DNA_frame_1C, 2);
      fase_DNA = 2;
  }
  else if(fase_DNA == 2){
      
      for(int i=1; i < 6; i++) {DNA_UNI_ID [14 + i] = CAN_frame[i];}
      uint16_t crc = 0xFFFFU;
      DNA_UNI_ID [2] = 0xFA;
      crc = crcAdd(crc, DNA_UNI_ID, 19);
      byte DNA_frame_2A[8] = {(uint8_t) crc, (uint8_t) (crc >> 8U), 0xFA, DNA_UNI_ID [3], DNA_UNI_ID [4],DNA_UNI_ID [5], DNA_UNI_ID [6], 0x82};
      byte DNA_frame_2B[8] = {DNA_UNI_ID [7], DNA_UNI_ID [8], DNA_UNI_ID [9], DNA_UNI_ID [10], DNA_UNI_ID [11], DNA_UNI_ID [12], DNA_UNI_ID [13], 0x22};
      byte DNA_frame_2C[6] = {DNA_UNI_ID [14], DNA_UNI_ID [15], DNA_UNI_ID [16], DNA_UNI_ID [17], DNA_UNI_ID [18], 0x42};
      sen_can_frame(0x1000010A, DNA_frame_2A, 8);
      sen_can_frame(0x1000010A, DNA_frame_2B, 8);
      sen_can_frame(0x1000010A, DNA_frame_2C, 6);
      fase_DNA = 0;
  }
}

void set_led_can (byte red, byte green, byte blue){
  uint16_t Rgb565 = (((red & 0xf8)<<8) + ((green & 0xfc)<<3)+(blue>>3));
  byte CAN_led_frame[4];
  uint32_t id_led = 0x1204390A;
  CAN_led_frame[0] = 0x00;
  CAN_led_frame[2] = byte(Rgb565);
  CAN_led_frame[1] = byte(Rgb565 >> 8);
  CAN_led_frame[3] = 0xC0;
  sen_can_frame(id_led,CAN_led_frame,4);
}

void setup() {
  Serial.begin(500000);
  while (!Serial);

  Serial.println("CAN Receiver");

  CAN.setPins(4,5);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  delay(3000);
  CAN.onReceive(get_can);  
  
}

void loop() {
  if (CAN_nuevo_frame_generico){
    CAN_nuevo_frame_generico = false;
    Serial.print("RECIBIDO: ");
    Serial.print(CAN_id_frame,HEX);
    Serial.print(" - ");
    for(int i=0; i<CAN_tamano_frame; i++){
      Serial.print(CAN_frame[i],HEX);
      Serial.print(" ");
    }
    Serial.println(); 
  }
  
  if(CAN_nuevo_frame_dan){
    DNA_CAN();
    CAN_nuevo_frame_dan = false;
  }
  set_led_can(255,255,255);
  delay(500);
  send_estado_nodo_HD();
}

static void send_estado_nodo_HD(){
  if ((millis()-last_time_to_pub_estado_nodo) > time_to_pub_estado_nodo){
    if(time_estado_nodo == 0){health_byte=B00001000;}
    else{health_byte=B00000000;}
    
    byte buff_estado_nodo[8];
    byte * time_estado_nodo_arr_byte = (byte *) &time_estado_nodo;
    
    buff_estado_nodo[0]= time_estado_nodo_arr_byte[0];
    buff_estado_nodo[1]= time_estado_nodo_arr_byte[1];
    buff_estado_nodo[2]= time_estado_nodo_arr_byte[2];
    buff_estado_nodo[3]= time_estado_nodo_arr_byte[3];
    buff_estado_nodo[4]= health_byte;
    buff_estado_nodo[5]= 0x00; 
    buff_estado_nodo[6]= 0x00;
    buff_estado_nodo[7]= B11000000 | (time_estado_nodo_arr_byte[0] & B00011111);
    
    sen_can_frame(id_estado_nodo,buff_estado_nodo,8);
    //print_can_frame(1,id_estado_nodo,buff_estado_nodo,8);
    time_estado_nodo ++;
    last_time_to_pub_estado_nodo = millis();
  }
}

static void sen_can_frame(uint32_t id_nodo, byte buff[], int zize){
  CAN.beginExtendedPacket(id_nodo);
  Serial.print("ENVIADO: ");
  Serial.print(id_nodo,HEX);
  Serial.print(" ");
    for(int i=0; i<zize; i++){
      CAN.write(buff[i]);
      Serial.print(buff[i],HEX);
      Serial.print(" ");
    }
    CAN.endPacket();
    Serial.println();
}
