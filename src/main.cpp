// --------------------------------------------------------------------------------------------- //
//
// UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE
// CAR-KARA EMBEDDED SYSTEMS & DATA AQUISITION
//
// LOAD CELL MAIN
//
// -> CKA4932
//
// AUTHOR: MAURO CHIOZZI
//
// --------------------------------------------------------------------------------------------- //

#include <Arduino.h>
#include <Wire.h>

#include <HX711.h>

// --------------------------------------------------------------------------------------------- //
// Códigos de requisição
#define DISPOSITIVO_INICIALIZANDO 0xFD
#define DISPOSITIVO_OCUPADO 0xFE
#define REQUISICAO_NAO_ENCONTRADA 0xFF

// Valor da requisição. Usado para o master pedir informações específicas via I2C
uint8_t requisicao;

// --------------------------------------------------------------------------------------------- //
// Se usar em modo debug, setar como true
#define DEBUG true

unsigned long ultima_leitura_serial;

#if DEBUG
#define myDebug Serial
#endif

// --------------------------------------------------------------------------------------------- //
// Flags para status do dispositivo

// Se ainda está inicializando o dispositivo, seta como true para não consumir a requisição e
// informar para o master que está sendo inicializado
bool is_slave_inicializando;
// Trava as requisições I2C em partes cruciais do código, para não misturar as informações antigas
// com as novas no momento da conversão GPS -> POLLH e ECEF
bool is_slave_ocupado;

// --------------------------------------------------------------------------------------------- //
// Variáveis globais para as pontes
//
//                          Ponte A (1 & 2)                 ^ y
//                        \                         |
//                         \                      z o --> x
//                          \         Ponte B (1 & 2)
//                           o -------
//                          /
//                         /
//                        /
//                          Ponte C (1 & 2)
//
// Declaração de cada ponte em cada elemento elástico
Bridge ponte_a1, ponte_a2;
Bridge ponte_b1, ponte_b2;
Bridge ponte_c1, ponte_c2;

// Offsets de cada ponte para tarar o resultado
int offset_a1, offset_a2;
int offset_b1, offset_b2;
int offset_c1, offset_c2;

// Constantes de proporcionalidade. Devem ser calibrados periodicamente e armazenados em uma
// memória não volátil #TODO: ver a lib EEPROM do ATmega328P
int k_a1, k_a2;
int k_b1, k_b2;
int k_c1, k_c2;

// Valores das resultantes encontradas a cada interação
int forca_x, forca_y, forca_z;
int momento_roll, momento_pitch, momento_yax;

// Devem ser calculadas em 20 Hz
unsigned long ultimo_calculo_resultantes;

// --------------------------------------------------------------------------------------------- //
//
// Definição das funções. São implementadas no fim do arquivo.
//
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
// Inicializacao do dispositivo
void inicializacao();
// Inicializa Pontes
void inicializaPontes();
// Inicializacao da comunicação i2c
void inicializaI2C();
// Inicializacao do debug
void inicializaDebug();

// --------------------------------------------------------------------------------------------- //
// Rotina que ficará sendo executada no código
void rotina();

// --------------------------------------------------------------------------------------------- //
// Pontes de Wheatstone
// Calcula os coefientes de proporcionalidade de cada ponte
void calibraCoeficientesProporcionalidade();
// Calcula o offset para ser compensando quando não houver carga na ponte
void calculaOffSetsPontes();
// Recupera o valor lido pela ponte
void getValorBrutoPonte(Bridge &ponte);
// Calcula as forças resultantes de cada componente
void calculaResultantes();

// --------------------------------------------------------------------------------------------- //
// I2C
// Função que será chamada quando o dispositivo for requisitado
void quandoRequisitado();
// Função que será chamada quando o dispositivo receber alguma informação
void quandoReceber(int quantitadeBytes);
// Consome a requisição (seta como 0 o valor da requisição) se ela for realizada com sucesso
void consumirRequisicao();
// Se houver algum erro no processo da requisição, não consome a requisição para poder ser
// executada corretamente na próxima requisição da master
bool possuiRequisicaoPendente();
// Wire.write só envia um byte por vez. Por tanto, para tipos com mais de um byte
// (int, float, double, long, etc), é necessário enviar um byte de cada vez.
// long possue 4 bytes no ATmega328
void escreverQuatroBytesWire(long longParaEnviar);
// int possue 2 bytes
void escreverDoisBytesWire(int intParaEnviar);

// --------------------------------------------------------------------------------------------- //
//
// Código Principal
//
// --------------------------------------------------------------------------------------------- //

int main()
{
  inicializacao();

  while (true)
  {
    rotina();
  }
}

// --------------------------------------------------------------------------------------------- //
//
// Implementação das funções
//
// Implemeta as duas principais primeiro, inicialização e rotina, para ficar mais acessível.
// Essas duas são as mais acessadas revisadas no código. As demais funções são implementadas
// depois.
//
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
void inicializacao()
{
  // Função init do Arduino
  init();
  // inicializa as pontes
  inicializaPontes();
  // inicializa a comunicação I2c
  inicializaI2C();
  // Inicializa o debug, se for setado true
  inicializaDebug();

  ultima_leitura_serial = millis();

  // Pronto, tudo inicializado
  is_slave_inicializando = false;
}

// --------------------------------------------------------------------------------------------- //
void rotina()
{
  if ((millis() - ultima_leitura_serial > 75) && !possuiRequisicaoPendente())
  {

    ultima_leitura_serial = millis();

    // Trava as requisições aqui, para não ser enviado informações que ainda estão
    // sendo convertidas
    is_slave_ocupado = true;

    // Libera as requisições
    is_slave_ocupado = false;
  }
}

// --------------------------------------------------------------------------------------------- //
// Demais funções

void inicializaPontes()
{
  calibraCoeficientesProporcionalidade();
  calculaOffSetsPontes();
}

void inicializaI2C()
{
  // Endereço slave do dispositivo
  const uint8_t meu_endereco_I2C = 0x17;

  // Inicializações do I2C
  Wire.begin(meu_endereco_I2C);
  Wire.onReceive(quandoReceber);
  Wire.onRequest(quandoRequisitado);
}

void inicializaDebug()
{
// Se setado como debug, inicializa a serial
#if DEBUG
  myDebug.begin(115200);
  myDebug.println("Debugando...");
#endif
}

void calibraCoeficientesProporcionalidade()
{
}

void calculaOffSetsPontes()
{
}

void getValorBrutoPonte(Bridge &ponte)
{
}

void calculaResultantes()
{
}

void quandoRequisitado()
{
  // Quando as requisições forem executadas com sucesso, ela é consumida depois.
  if (is_slave_ocupado)
  { // Não consome a requisição
    Wire.write(DISPOSITIVO_OCUPADO);
  }
  else if (is_slave_inicializando)
  { // Não consome a requisição
    Wire.write(DISPOSITIVO_INICIALIZANDO);
  }
  else if (requisicao == 0x05)
  {                            // Requisicao das forças: 12 Bytes
    escreverDoisBytesWire(12); // Fx
    escreverDoisBytesWire(13); // Fy
    escreverDoisBytesWire(14); // Fz

    consumirRequisicao();
  }
  else
  {

#if DEBUG
    myDebug.print(F("Requisicao nao encontrada: "));
    myDebug.println(requisicao);
    delay(500);
#endif
    // Requisicao solicitada não foi encontrada
    Wire.write(REQUISICAO_NAO_ENCONTRADA);

    consumirRequisicao(); // Nesse caso, consome para evitar loop infinito
  }
}

// --------------------------------------------------------------------------------------------- //

void quandoReceber(int quantitadeBytes)
{
  if (Wire.available())
  {
    requisicao = Wire.read();

#if DEBUG
    myDebug.print("Requisicao recebida: ");
    myDebug.println(requisicao);
#endif
  }
}

// --------------------------------------------------------------------------------------------- //

void consumirRequisicao()
{
  requisicao = 0x00;
}

// --------------------------------------------------------------------------------------------- //

bool possuiRequisicaoPendente()
{
  return (requisicao != 0x00);
}

// --------------------------------------------------------------------------------------------- //

void escreverQuatroBytesWire(long longParaEnviar)
{
  Wire.write(longParaEnviar >> 24);          // BBBB BBBB XXXX XXXX XXXX XXXX XXXX XXXX
                                             // ---- ---- ---- ---- ---- ---- BBBB BBBB
  Wire.write((longParaEnviar >> 16) & 0xFF); // XXXX XXXX BBBB BBBB XXXX XXXX XXXX XXXX
                                             // ---- ---- ---- ---- XXXX XXXX BBBB BBBB
                                             // ---- ---- ---- ---- ---- ---- 1111 1111
                                             // ---- ---- ---- ---- ---- ---- BBBB BBBB
  Wire.write((longParaEnviar >> 8) & 0xFF);  // XXXX XXXX XXXX XXXX BBBB BBBB XXXX XXXX
                                             // ---- ---- XXXX XXXX XXXX XXXX BBBB BBBB
                                             // ---- ---- ---- ---- ---- ---- 1111 1111
                                             // ---- ---- ---- ---- ---- ---- BBBB BBBB
  Wire.write((longParaEnviar)&0xFF);         // XXXX XXXX XXXX XXXX XXXX XXXX BBBB BBBB
                                             // ---- ---- ---- ---- ---- ---- 1111 1111
                                             // ---- ---- ---- ---- ---- ---- BBBB BBBB
}

void escreverDoisBytesWire(int intParaEnviar)
{
  Wire.write((intParaEnviar >> 8) & 0xFF);
  Wire.write((intParaEnviar)&0xFF);
}

// --------------------------------------------------------------------------------------------- //
// FIM
// --------------------------------------------------------------------------------------------- //
