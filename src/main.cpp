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
//  Informações I2C

// Endereço I2C
#define SLAVE_ADDRESS 0x17

// Códigos de requisição
#define DISPOSITIVO_INICIALIZANDO 0xFD
#define DISPOSITIVO_OCUPADO 0xFE
#define REQUISICAO_NAO_ENCONTRADA 0xFF

// Valor da requisição. Usado para o master pedir informações específicas via I2C
uint8_t requisicao;

// --------------------------------------------------------------------------------------------- //
// Se usar em modo debug, setar como true
#define DEBUG true
#define BUZZER true

#if DEBUG
#define myDebug Serial
#define BAUDRATE 115200
unsigned long ultima_leitura_serial;
#endif

#if BUZZER
#define BUZZER_PIN 13
#endif

// -----------#---------------------------------------------------------------------------------- //
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

#define DISTANCIA_SG 6 // 6 mm do ponto O até o centro do strain gauge

// Temporario. Essas escalas devem ser calibradas periodicamente com um peso de referência
float coef_proporcao[6] = {
    208219.81, 226134.46,
    212822.10, 222634.70,
    211122.60, 218470.76};

#define GRAVIDADE 9.81                            // metros / s^2
const float PESO_REFERENCIA = 0.1851 * GRAVIDADE; //quilogramas

// Forcas aferidas por cada ponte
float forcas_pontes[6] = {0};

// Separação dos objetos, para ficar mais intuivo no calculo das resultantes
float *forcas_pontes_a = (&forcas_pontes[0]);
float *forcas_pontes_b = (&forcas_pontes[2]);
float *forcas_pontes_c = (&forcas_pontes[4]);

// Valores das resultantes encontradas a cada interação
float forca_x, forca_y, forca_z;
float momento_roll, momento_pitch, momento_yaw;

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
// Inicializa o Buzzer para notificações sonoras
void inicializaBuzzer();

// --------------------------------------------------------------------------------------------- //
// Rotina que ficará sendo executada no código
void rotina();

// --------------------------------------------------------------------------------------------- //
// Pontes de Wheatstone
// Calcula os coefientes de proporcionalidade de cada ponte
void calibraCoeficientesProporcionalidade();
void setCoeficientesProporcionalidade();
// Calcula o offset para ser compensando quando não houver carga na ponte
void setOffSetsPontes();
// Filtra os ruidos de grande intensidade das pontes, uma por vez
float filtraValorPonte(float valor_anterior, float valor_atual);
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

// --------------------------------------------------------------------------------------------- //
void alertaSonoro(int qnt_alertas);

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

  // Inicializa o debug, se for setado true
#if DEBUG
  inicializaDebug();
#endif

  // Inicializa o buzzer, se for setado true
#if BUZZER
  inicializaBuzzer();
#endif

  // inicializa as pontes
  inicializaPontes();

  // inicializa a comunicação I2c
  inicializaI2C();

  // Depois que todas as funções forem inicializadas, inicia o timer para envio dos debugs
#if DEBUG
  ultima_leitura_serial = millis();
#endif

  ultimo_calculo_resultantes = millis();
  // Pronto, tudo inicializado
  is_slave_inicializando = false;
}

// --------------------------------------------------------------------------------------------- //
void rotina()
{
  // A cada interação verifica se os HX711 estão com os valores prontos, e realiza a leitura das
  // forças atuando em cada ponte
  getForcasPontes();

  // Com as forças lidas, calcula as resultantes
  calculaResultantes();

#if DEBUG
  // Debug qualquer informação aqui
  if ((millis() - ultima_leitura_serial > 50) && !possuiRequisicaoPendente())
  {
    ultima_leitura_serial = millis();

    for (int i = 0; i < 6; i++)
    {
      Serial.print(forcas_pontes[i]);
      Serial.print(",");
    }

    Serial.println("");
  }
#endif
}

// --------------------------------------------------------------------------------------------- //
// Demais funções

void inicializaPontes()
{
  // Em cada inicialização, o sistema deve calcular o offset de cada ponte
  setOffSetsPontes();
  // TODO: temporario. As pontes devem ser calibradas periodicamente
  // calibraCoeficientesProporcionalidade();
  // Apos calibração, seta os coeficientes de cada ponte
  setCoeficientesProporcionalidade();
}

void inicializaI2C()
{
  // Inicializações do I2C
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(quandoReceber);
  Wire.onRequest(quandoRequisitado);
}

void inicializaDebug()
{
  myDebug.begin(BAUDRATE);
}

void inicializaBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
}

void calibraCoeficientesProporcionalidade()
{

  // Recomenda-se a utilização do buzzer para facilitar o acompanhamento do processo de calibração
  // do algoritmo. A versão final desse método será uma sistema de equações.
#if BUZZER
  alertaSonoro(2);
#endif

  for (int i = 0; i < 6; i++)
  {
#if BUZZER
    alertaSonoro(1);
#endif

    // Delay para que haja tempo de mover o peso de referência de uma ponte para outra
    delay(2000);

    // O calculo do coefienciente, nesse caso, é bastante simples
    coef_proporcao[i] = pontes[i].get_value(10) / PESO_REFERENCIA;

#if DEBUG
    Serial.println(coef_proporcao[i]);
#endif
  }

#if BUZZER
  alertaSonoro(3);
#endif
}

void setCoeficientesProporcionalidade()
{
  for (int i = 0; i < 6; i++)
  {
    pontes[i].set_scale(coef_proporcao[i]);
  }
}

void setOffSetsPontes()
{
  for (int i = 0; i < 6; i++)
  {
    pontes[i].tare();
  }
}

float filtraValorPonte(float valor_anterior, float valor_atual)
{
  if (abs(valor_atual - valor_anterior) / 100 <= 0.02)
  {
    return valor_atual;
  }

  return valor_anterior;
}

void getForcasPontes()
{
  for (int i = 0; i < 6; i++)
  {
    if (pontes[i].is_ready())
    {
      forcas_pontes[i] = filtraValorPonte(forcas_pontes[i], pontes[i].get_units());
    }
  }
}

void calculaResultantes()
{
  if ((millis() - ultimo_calculo_resultantes > 50) && !possuiRequisicaoPendente())
  {
    ultimo_calculo_resultantes = millis();
    // Trava as requisições aqui, para não ser enviado informações que ainda estão
    // sendo convertidas
    is_slave_ocupado = true;

    // TODO: Por enquanto, não faz nada

    // Libera as requisições
    is_slave_ocupado = false;
  }
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
  {
    // Requisicao das forças: 12 Bytes
    escreverQuatroBytesWire((long)(forca_x * 1000)); // Fx
    escreverQuatroBytesWire((long)(forca_y * 1000)); // Fy
    escreverQuatroBytesWire((long)(forca_z * 1000)); // Fz

    consumirRequisicao();
  }
  else if (requisicao == 0x06)
  {
    // Requisicao dos momentos: 12 Bytes
    escreverQuatroBytesWire((long)(momento_pitch * 1000));
    escreverQuatroBytesWire((long)(momento_roll * 1000));
    escreverQuatroBytesWire((long)(momento_yaw * 1000));

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

void alertaSonoro(int qnt_alertas)
{
  for (int i = 0; i < qnt_alertas; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

// --------------------------------------------------------------------------------------------- //
// FIM
// --------------------------------------------------------------------------------------------- //
