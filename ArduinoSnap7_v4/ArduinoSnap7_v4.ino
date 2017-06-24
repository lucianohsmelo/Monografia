/*---------------------------------------------------------
 ArduinoSnap7

 Created Fev 2016
 by Luciano Melo 
---------------------------------------------------------*/

// Biblioteca usada na comunicação Ethernet
#include <SPI.h> 

// Biblioteca usada na comunicação Ethernet


// Biblioteca usada para resetar a placa pelo software
#include <avr/wdt.h>

// Biblioteca usada na comunicação Snap7
#include "Settimino.h"

EthernetClient client;

#define DO_IT_SMALL
#define outSignal 7
#define ledModo1 8
#define ledModo2 9
#define ledSinalaliza 5

// MAC Adress Arduino
byte mac[] = { 
  0x90, 0xA2, 0xDA, 0x0F, 0x08, 0xE11 };

// Endereço de IP do CLP  
IPAddress PLC(192,168,0,100);   // PLC Address

//Declaração de variáveis
byte Buffer[12];
int Result, atvS7Client, DB, i, IP[4];
int contDeslM, contLigaM;
uint16_t sizeBuffer, Rack, Slot;
int modoM, statusSig, StatusPLC;
float tempM1, tempM2, auxFloat; 
word tempDesl, auxWord;
byte myByte, auxByte;
S7Client Client;
unsigned long Elapsed; // To calc the execution time
bool noEthernet;
int LastStatus = -1;
//---------------------------------------------------------
// função para obter parâmentros de comunicação
 void GetParams(){
  i = 0;
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    i = i + 1;
    IP[i] = (Ethernet.localIP()[thisByte]);
  }
  Serial.print("My IP address: "); 
  for (i = 1; i <= 4; i++){
    Serial.print(IP[i]);
    if (i < 4){
    Serial.print(".");
      }
    }
   Serial.println();
 }
//---------------------------------------------------------
// função sinalizar atividade do Arduino
int checkS7Client(int value){ 
  if (value == 0){
    value = 1;
  }
  else {
    value = 0;
  }
  return value;
}
//---------------------------------------------------------
// função para verificar temperatura
float checkTemp(int analog){ 
  float temp = (float(analog)*5/(1023))/0.01;
  return temp;
}
//---------------------------------------------------------
// função para piscar LED de sinalização
 void blinkPin(){
  digitalWrite(ledSinalaliza,HIGH);
  delay(100);
  digitalWrite(ledSinalaliza,LOW);
 }
//---------------------------------------------------------
// Reverse float/dword/dint
void LittleToBigEndian4(void *ptr){
  byte *pb;
  byte tmp;
  pb=(byte*)(ptr);
  // Swap byte 4 with byte 1
  tmp=*(pb+3);
  *(pb+3)=*pb;
  *pb=tmp;
  // Swap byte 3 with byte 2
  tmp=*(pb+2);
  *(pb+2)=*(pb+1);
  *(pb+1)=tmp;
 }
//---------------------------------------------------------
// Reverse word/int
void LittleToBigEndian2(void *ptr){
  byte *pb;
  byte tmp;
  pb=(byte*)(ptr);
  // Swap byte 2 with byte 1
  tmp=*(pb+1);
  *(pb+1)=*pb;
  *pb=tmp;
 }
//---------------------------------------------------------
// função para escrever valores no Terminal Serial
void writeTerminalSerial(){
  Serial.print("Temperatura Motor 1: ");
  Serial.println(tempM1);
  Serial.print("Temperatura Motor 2: ");
  Serial.println(tempM2);
  Serial.print("Temperatura de desligamento: ");
  Serial.println(tempDesl);
}
//---------------------------------------------------------
// Sinal elétrico Modo 1
void outSignalModo1(){
  digitalWrite(ledModo1,HIGH);
  digitalWrite(ledModo2,LOW);
  Serial.println("Modo 1 selecionado.");
  modoM = 0;
  bitWrite(myByte,0,modoM);
  if (tempM1 > tempDesl || tempM2 > tempDesl){
    contLigaM = 0;
    contDeslM = contDeslM + 1;
    if (contDeslM >= 5){
      contDeslM = 0;
      digitalWrite(outSignal, LOW);
      statusSig = 0;
      bitWrite(myByte,1,statusSig);
    }
  }else{
    contDeslM = 0;
    contLigaM = contLigaM + 1;
    if (contLigaM >= 5){
        contLigaM = 0;
        digitalWrite(outSignal, HIGH);
        statusSig = 1;
        bitWrite(myByte,1,statusSig);
      }
    }
    
    if (statusSig == 1){
        Serial.println("Sinal = 5V");
    }else if (statusSig == 0){
        Serial.println("Sinal = 0V");
    }
}
//---------------------------------------------------------
// Sinal elétrico Modo 2
void outSignalModo2(){
  digitalWrite(ledModo1,LOW);
  digitalWrite(ledModo2,HIGH);
  Serial.println("Modo 2 selecionado");
  modoM = 1;
  bitWrite(myByte,0,modoM);
  if (statusSig == 1){ //Modo manual selecionado
      digitalWrite(outSignal, 1);
      Serial.println("Sinal = 5V");
      statusSig = 1;
      bitWrite(myByte,1,statusSig);
  }else if (statusSig == 0){
      digitalWrite(outSignal, 0);
      statusSig = 0;
      bitWrite(myByte,1,statusSig);
      Serial.println("Sinal = 0V");
  }
}
//---------------------------------------------------------
// Inicia contagem de tempo
void MarkTime(){
  Elapsed=millis();
}
//---------------------------------------------------------
// Calcula tempo de ciclo do algoritmo
void ShowTime(){
  // Calcs the time
  Elapsed=millis()-Elapsed;
  Serial.print("Job time (ms) : ");
  Serial.println(Elapsed);   
  Serial.println();
}
//---------------------------------------------------------
// Tenta fazer conexão com o CLP
bool Connect(){
    int Result=Client.ConnectTo(PLC, 
                                Rack,  
                                Slot); 
    Serial.print("Connecting to ");
    Serial.println(PLC);  
    if (Result==0) 
    {
      Serial.println("Connected !");
    }
    else
      Serial.println("Connection error");
    return Result==0;
}
//---------------------------------------------------------
// Escreve o código do erro
void CheckError(int ErrNo){
  Serial.print("Error No. 0x");
  Serial.println(ErrNo, HEX);
  
  // Checks if it's a Severe Error => we need to disconnect
  if (ErrNo & 0x00FF)
  {
    Serial.println("SEVERE ERROR, disconnecting.");
    Client.Disconnect(); 
  }
}
//---------------------------------------------------------
// Setup: Inicia comunicação Serial e Ethernet
void setup() {
  MarkTime();
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.println("Inicando Snap7 Client. Aguarde...");
  // Declara pinos como Inputs/Outputs
  pinMode(2, INPUT);  // define pino 2 como entrada
  pinMode(3, INPUT);  // define pino 3 como entrada
  pinMode(4, INPUT);  // define pino 4 como entrada
  pinMode(5, OUTPUT); // define pino 5 como saída
  pinMode(6, OUTPUT); // define pino 6 como saída
  pinMode(7, OUTPUT); // define pino 7 como saída
  pinMode(8, OUTPUT); // define pino 8 como saída
  pinMode(9, OUTPUT); // define pino 9 como saída
  //LED de sinalazição acesso
  digitalWrite(ledSinalaliza,HIGH);
  
  if (Ethernet.begin(mac) == 0) {
    // Se a conexão Ethernet não for iniciada
    noEthernet = true;
    Serial.println("Falha em configurar conexao Ethernet");
  }else{
    // Se a conexão Ethernet for iniciada
    // Obtém endereço de IP do Arduino
    GetParams();
    // Endereço de IP do Arduino
    IPAddress Local(IP[1],IP[2],IP[3],IP[4]);
    // Inicia conexão Ethernet
    Ethernet.begin(mac, Local);
    // Data Block 1 (DB1)
    DB = 4;
    // Rack/Slot - CPU S7 300
    Rack = 0;
    Slot = 2;
    noEthernet == false;
    Connect();
  }
  ShowTime();
  delay(2000); 
}
//---------------------------------------------------------
// Main Loop
void loop() {
    // Se não for possível estabelecer uma conexão Ethernet 
    // com a rede
   if (noEthernet == true){
    while(1){
      Serial.println("Snap7 Client");
    Serial.println("Falha em configurar conexao Ethernet");
      tempM1 = checkTemp(analogRead(0));
      tempM2 = checkTemp(analogRead(1));
      blinkPin();
      outSignalModo1();
      writeTerminalSerial();
      Serial.print("Reiniciando Arduino em: ");
      Serial.print((300000 - millis())/1000);
      Serial.println(" segundos");
      Serial.println();
      delay(1000);
      if (millis() >= 299999){
        wdt_enable(WDTO_2S);
        Serial.println("Arduino reiniciado");
        while(1); // Espera até reiniciar o Arduino
      }
    }
  }
  // Se foi estabelecida uma conexão Ethernet com a rede
    Serial.println("Snap7 Client");
    atvS7Client = checkS7Client(atvS7Client);
    tempM1 = checkTemp(analogRead(0));
    tempM2 = checkTemp(analogRead(0)+1);
    
    // Verifica estado do CLP
    Result = Client.GetPlcStatus(&StatusPLC);
    if (Result==0)
      {
        if (StatusPLC!=LastStatus)
        {
          Serial.print("Estado do CLP: ");
          switch (StatusPLC)
          {
            case S7CpuStatusUnknown:
                Serial.println("'UNKNOWN'");
                break;
            case S7CpuStatusRun:
              Serial.println("'RUNNING'");
              break;
           case S7CpuStatusStop:
              Serial.println("'STOPPED'");
              break;
          }
        }
    }     
    //Se o Arduino não estabelecer uma conexão com o PLC
    if (!Client.Connected){
      Serial.println("Falha de conexao com CLP");
      MarkTime();
      outSignalModo1();
      blinkPin();
      Connect();
      writeTerminalSerial();
      ShowTime();
      delay(900);
    }
    // Se o Arduino estabelecer conexão com o PLC 
    if (Client.Connected){
      Serial.println("CLP conectado");
      MarkTime();
      digitalWrite(ledSinalaliza,HIGH);
      Result=Client.GetDBSize(DB,       // DB Number = 1
                            &sizeBuffer);  // In input contains our buffer size                
      if (Result != 0){
        CheckError(Result);
      }
      // Leitura da área de memória DB1 do CLP
      Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
                           DB,        // DB = 1
                           0,        // Start from byte N.0
                          sizeBuffer,     // We need "Size" bytes
                       &Buffer);  // Put them into our Buffer                
      // Se não for possível ler a memória do CLP
      if (Result != 0){
        CheckError(Result);
      }

      tempDesl = S7.WordAt(&Buffer, 8);
      modoM = S7.BitAt(&Buffer, 10, 0);
      statusSig = S7.BitAt(&Buffer, 10, 1);
  
      if (modoM == 0){
        outSignalModo1();
      }else if (modoM == 1){
        outSignalModo2();
      }

      
      bitWrite(myByte,2,atvS7Client);
      
      writeTerminalSerial();
  
      LittleToBigEndian4(&tempM1); 
      Result=Client.WriteArea(S7AreaDB, 
                              DB, 
                              0, 
                              sizeof(float), 
                              &tempM1);
      if (Result != 0){
        CheckError(Result);
      }
  
      LittleToBigEndian4(&tempM2); 
      Result=Client.WriteArea(S7AreaDB, 
                              DB, 
                              4, 
                              sizeof(float), 
                              &tempM2);
      if (Result != 0){
        CheckError(Result);
      }
  
      LittleToBigEndian2(&tempDesl);
      Result=Client.WriteArea(S7AreaDB,  
                              DB, 
                              8, 
                              sizeof(word), 
                              &tempDesl);
      if (Result != 0){
        CheckError(Result);
      }
    
      Result=Client.WriteArea(S7AreaDB, 
                              DB, 
                              10, 
                              sizeof(byte), 
                              &myByte);
      if (Result != 0){
        CheckError(Result);
      }
      ShowTime();
      delay(1000);
      }
}



