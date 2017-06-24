#include <SPI.h> //Inclui a Biblioteca SPI
#include <Ethernet.h> //Inclui a Biblioteca Ethernet
#include "Mudbus.h" //Inclui a Biblioteca Mudbus
// Biblioteca usada para resetar a placa pelo software
#include <avr/wdt.h>
// declaração de variáveis

//pino digital 7 como saída do sinal elétrico
#define outSignal 7
#define ledModo1 8 //LED de inciação de funcionamento em Modo 1
#define ledModo2 9
#define ledSinalaliza 5
int AtvMbClient, AtvMbServer, aux;
int statusSig, modoM; 
float tempM1, tempM2;
int tempM1Int, tempM1Dec, tempM2Int, tempM2Dec, tempDesl;
int MbDisc, contDeslM, contLigaM;
uint8_t IP[4], gw[4], sm[4];
bool noEthernet;

Mudbus Mb;
//---------------------------------------------------------
// função para a escrever em um registrador
void WriteHoldingRegister(int reg, int value){ 
  Mb.R[reg] = value; 
}
//---------------------------------------------------------
// função para ler um registrador
int ReadHoldginRegister(int reg){ 
  int value = Mb.R[reg];
  return value;
}
//---------------------------------------------------------
// função para sinalizar atividade do Arduino
int checkMbServer(int value){ 
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
// função para obter parâmentros de comunicação
void GetParams(){
  int i = 0;
  byte thisByte;
  for (thisByte = 0; thisByte < 4; thisByte++) {
    i = i + 1;
    IP[i] = (Ethernet.localIP()[thisByte]);
  }
  i = 0;
  for (thisByte = 0; thisByte < 4; thisByte++) {
    i = i + 1;
    gw[i] = (Ethernet.gatewayIP()[thisByte]);
  }
  i = 0;
  for (thisByte = 0; thisByte < 4; thisByte++) {
    i = i + 1;
    sm[i] = (Ethernet.subnetMask()[thisByte]);
  } 
    Serial.print("IP address: "); 
  for (int i = 1; i <= 4; i++){
    Serial.print(IP[i]);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Getway: "); 
  for (int i = 1; i <= 4; i++){
    Serial.print(gw[i]);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Submask: ");
    for (int i = 1; i <= 4; i++){
    Serial.print(sm[i]);
    Serial.print(".");
  }
  Serial.println();
 }
//---------------------------------------------------------
// função para piscar LED de sinalização
 void blinkPin(){
  digitalWrite(ledSinalaliza,HIGH);
  delay(300);
  digitalWrite(ledSinalaliza,LOW);
 }
//---------------------------------------------------------
// função para escrever os valores no Terminal Serial 
void writeTerminalSerial(){
  Serial.print("Temp. Motor 1: ");
  Serial.println(tempM1);
  Serial.print("Temp. Motor 2: ");
  Serial.println(tempM2);
  Serial.print("Temp. de desligamento: ");
  Serial.println(tempDesl);

}
//---------------------------------------------------------
// Sinal elétrico Modo 1
void outSignalModo1(){
  digitalWrite(ledModo1,HIGH);
  digitalWrite(ledModo2,LOW);
  Serial.println("Modo 1 selecionado.");
  modoM = 0;
  WriteHoldingRegister(6,modoM);
  if (tempM1 > tempDesl || tempM2 > tempDesl){
    contLigaM = 0;
    contDeslM = contDeslM + 1;
    if (contDeslM >= 5){
      contDeslM = 0;
      digitalWrite(outSignal, LOW);
      statusSig = 0;
    }
    }else{
    contDeslM = 0;
    contLigaM = contLigaM + 1;
    if (contLigaM >= 5){
      contLigaM = 0;
      digitalWrite(outSignal, HIGH);
      statusSig = 1;
    }
    }
    
    if (statusSig == 1){
        Serial.println("Sinal = 5V");
    }else{
        Serial.println("Sinal = 0V");
    }
}
//---------------------------------------------------------
// Sinal elétrico Modo 2
void outSignalModo2(){
  digitalWrite(ledModo1,LOW);
  digitalWrite(ledModo2,HIGH);
  Serial.println("Modo 2 selecionado");
  if (statusSig == 1){ //Modo manual selecionado
      digitalWrite(outSignal, 1);
      Serial.println("Sinal = 5V");
  }else if (statusSig == 0){
      digitalWrite(outSignal, 0);
      Serial.println("Sinal = 0V");
  }
}
//---------------------------------------------------------
// Setup: Inicia comunicação Serial e Ethernet
void setup(){
   //Inicial Serial (caso seja necessário)
  Serial.begin(9600);
  
  // Declara pinos como Inputs/Outputs
  pinMode(2, INPUT);  // define pino 2 como entrada
  pinMode(3, INPUT);  // define pino 3 como entrada
  pinMode(4, INPUT);  // define pino 4 como entrada
  pinMode(5, OUTPUT); // define pino 5 como saída
  pinMode(6, OUTPUT); // define pino 6 como saída
  pinMode(7, OUTPUT); // define pino 7 como saída
  pinMode(8, OUTPUT); // define pino 8 como saída
  pinMode(9, OUTPUT); // define pino 9 como saída

  Serial.println("Iniciando Modbus TCP Server. Aguarde...");
  digitalWrite(ledSinalaliza,HIGH);
  uint8_t mac[]     = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };
  if (Ethernet.begin(mac) == 0) {
    // no point in carrying on, so do nothing forevermore:
    noEthernet = true;
  } else{

   // Endereço de IP do Server (Arduino)
  uint8_t ip[]      = { 192, 168, 1, 105 };
  uint8_t gateway[] = { 192, 168, 1, 1 };
  uint8_t subnet[]  = { 255, 255, 255, 0 };

//    GetParams();
//    // Endereço de IP do Server (Arduino)
//    uint8_t ip[]      = { IP[1], IP[2], IP[3], 101 };
//    // Gateway do Server (Arduino) 
//    uint8_t gateway[] = {gw[1], gw[2], gw[3], gw[4] }; 
//     // Máscara do Server (Arduino)  
//    uint8_t subnet[]  = { sm[1], sm[2], sm[3], sm[4] };
//    // Inicia a conexão Ethernet
//    Ethernet.begin(mac, ip, gateway, subnet);
//    Serial.println("Conexao Ethernet configurada");
//    noEthernet = false;
  }
  MbDisc = 11;
  Serial.println();
  //Atraso de 2 segundos
  delay(2000);
} // Fim void setup()
//---------------------------------------------------------
//executado a cada ciclo de clock
void loop() { 
  // Se não foi possível configurar a conexão Ethernet
  if (noEthernet == true){
    tempDesl = -50; 
    while(1){
      Serial.println("Modbus TCP Server");
      Serial.println("Falha em configurar conexao Ethernet");
      tempM1 = checkTemp(analogRead(0));
      tempM2 = checkTemp(analogRead(0));
      blinkPin();
      outSignalModo1();
      writeTerminalSerial();
      Serial.print("Reiniciando Arduino em: ");
      Serial.print((300000 - millis())/1000);
      Serial.println(" segundos");
      Serial.println();
      if (millis() >= 299999){
        wdt_enable(WDTO_2S);
        Serial.println("Arduino reiniciado");
        while(1); // Espera até reiniciar o Arduino
      }
      delay(1000);
    }
  }
  Serial.println("Modbus TCP Server");
  // converter o valor analógico em temperatura
  tempM1 = checkTemp(analogRead(0));
  tempM2 = checkTemp(analogRead(0)+1);

  //Inicializa a comunicação ModbusTCP
  Mb.Run();
  //Sinalizador de atividade do Arduino
  AtvMbServer = checkMbServer(AtvMbServer);
  WriteHoldingRegister(7,AtvMbServer);
  
  Serial.print("IP LOCAL: ");
  Serial.println(Ethernet.localIP());
  // Verifica a conexão do Cliente Modbus
  AtvMbClient = Mb.Reads;
  if (AtvMbClient > aux){
    MbDisc = 0;
  }else{
    MbDisc = MbDisc + 1;
  }
  aux = AtvMbClient;
  if (MbDisc >= 100){
    MbDisc = 25;
  }
  
  //Se o Cliente estiver conectado
  if (MbDisc <= 5){ 
    digitalWrite(ledSinalaliza,LOW);
    Serial.println("CLP conectado");
    // Leitura dos registradores
    tempDesl = ReadHoldginRegister(4);
    statusSig =  ReadHoldginRegister(5); 
    modoM = ReadHoldginRegister(6);

    // dividir parte inteira da decimal
    tempM1Int = tempM1;
    tempM1Dec = 100*(tempM1 - tempM1Int);
    tempM2Int = tempM2;
    tempM2Dec = 100*(tempM2 - tempM2Int);
    // Verifica modo de operação selecionado
    if (modoM == 0){ // Modo Automático selecionado
      outSignalModo1();
    }
    if (modoM == 1){
      outSignalModo2(); // Modo Manual selecionado
    }
    //Escrita no Client(PCL)
    //WriteHoldingRegister(int Reg,int value); 
    WriteHoldingRegister(0,tempM1Int); 
    WriteHoldingRegister(1,tempM1Dec); 
    WriteHoldingRegister(2,tempM2Int); 
    WriteHoldingRegister(3,tempM2Dec); 
    WriteHoldingRegister(5,statusSig);
  }
  // Se o Cliente Modbus estiver desconectado
  else{
    Serial.println("Falha de conexao com CLP");
    WriteHoldingRegister(6, tempDesl);
    blinkPin();
    outSignalModo1();
  }
  writeTerminalSerial();
  Serial.println();
  delay(500);
}



