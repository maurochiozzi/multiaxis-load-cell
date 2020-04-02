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
//                          Ponte A (1 & 2)                     ^ y
//                        \                                     |
//                         \                                  z o --> x
//                          \         Ponte B (3 & 4)
//                           o -------
//                          /
//                         /
//                        /
//                          Ponte C (5 & 6)
//
// Obs.: as pontes impares estarão nas faces laterais, enquanto as pontes pares estarão nas faces
// superiores e inferiores do elemento elástico
//
// Todos os HX711 possuem o mesmo SCK, que é o pino digital 9
#define BRIDGE_SCK 9

// Declaração de cada ponte em cada elemento elástico
Bridge pontes[6] = {
    Bridge(8, BRIDGE_SCK), Bridge(7, BRIDGE_SCK),  // Pontes 1 & 2
    Bridge(6, BRIDGE_SCK), Bridge(5, BRIDGE_SCK),  // Pontes 3 & 4
    Bridge(2, BRIDGE_SCK), Bridge(3, BRIDGE_SCK)}; // Pontes 5 & 6

const int DISTANCIA_SG = 6; // 6 mm do ponto O até o centro do strain gauge

// Temporario. Essas escalas devem ser calibradas periodicamente, uma vez que o conjunto esteja
// finalizado
float scales[6] = {10000.f, 10000.f,
                   10000.f, 10000.f,
                   10000.f, 10000.f};

// Forcas aferidas por cada ponte
int forcas_pontes[6] = {0};

// Separação dos objetos, para ficar mais intuivo no calculo das resultantes
int *forcas_pontes_a = (&forcas_pontes[0]);
int *forcas_pontes_b = (&forcas_pontes[2]);
int *forcas_pontes_c = (&forcas_pontes[4]);

// Valores das resultantes encontradas a cada interação
int forca_x, forca_y, forca_z;
int momento_roll, momento_pitch, momento_yaw;

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
void setCoeficientesProporcionalidade();
// Calcula o offset para ser compensando quando não houver carga na ponte
void calculaOffSetsPontes();
// Recupera todas as forças aferidas pelas pontes
void getForcasPontes();
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
  getForcasPontes();

  if ((millis() - ultimo_calculo_resultantes > 50) && !possuiRequisicaoPendente())
  {

    ultimo_calculo_resultantes = millis();

    // Trava as requisições aqui, para não ser enviado informações que ainda estão
    // sendo convertidas
    is_slave_ocupado = true;

    calculaResultantes();

    // Libera as requisições
    is_slave_ocupado = false;
  }
}

// --------------------------------------------------------------------------------------------- //
// Demais funções

void inicializaPontes()
{
  calculaOffSetsPontes();
  setCoeficientesProporcionalidade();
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
  for (int i = 0; i < 6; i++)
  {
    scales[i] = 10000.f;
  }
}

void setCoeficientesProporcionalidade()
{
  for (int i = 6; i < 6; i++)
  {
    pontes[i].set_scale(scales[i]);
  }
}

void calculaOffSetsPontes()
{
  for (int i = 6; i < 6; i++)
  {
    pontes[i].tare();
  }
}

void getForcasPontes()
{
  for (int i = 0; i < 6; i++)
  {
    if (pontes[i].is_ready())
    {
      forcas_pontes[i] = pontes[i].get_units();
    }
  }
}

void calculaResultantes()
{
  forca_x++;
  forca_y++;
  forca_z++;

  momento_pitch++;
  momento_roll++;
  momento_yaw++;
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
  {                                 // Requisicao das forças: 12 Bytes
    escreverDoisBytesWire(forca_x); // Fx
    escreverDoisBytesWire(forca_y); // Fy
    escreverDoisBytesWire(forca_z); // Fz

    consumirRequisicao();
  }
  else if (requisicao == 0x06)
  {                                       // Requisicao dos momentos: 12 Bytes
    escreverDoisBytesWire(momento_pitch); // Fx
    escreverDoisBytesWire(momento_roll);  // Fy
    escreverDoisBytesWire(momento_yaw);   // Fz

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
