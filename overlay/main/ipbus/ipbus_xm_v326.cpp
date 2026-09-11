
/*
 * BDK XM V1.0.7.1 - Pioneer DEH-P650 XM IP-BUS engine
 *
 * Native ESP-IDF port of the hardware-validated XM V3.26 Arduino state
 * machine. Protocol bytes/fields are preserved; only the HAL integration
 * is adapted to the existing BDK compatibility layer.
 *
 * Confirmed on the DEH-P650 project before this port:
 *   LINK 0x11E / APP 0x1E
 *   XM1/XM2/XM3 in STATUS61[7] high nibble
 *   CH001..255, BCD channel fields
 *   COM11 fast key response
 *   STATUS61[17..24] visible text window
 *   DISP metadata page cycle and 200 ms scrolling
 *   7 s session-silence recovery
 */
#include "bdk_arduino_compat.h"
#include "ipbus_xm_v326.h"
#include "driver/gptimer.h"
#include <string.h>

// PIONEER DEH-P650 - BDK XM V1.0.7.1 / XM V3.26 PROTOCOL CORE / LINK 0x11E APP 0x1E
//   manter a fonte XM fixa usando a identidade confirmada
//   com c/d, usando a codificacao BCD confirmada no DEH-P650.
//   Arduino-ESP32 core 3.3.11 / TX RMT Core 3 + RX por GPIO ISR + GPTimer.
//   esta V3 mantem a identidade XM confirmada e implementa
//   formato de canal confirmado no DEH-P650: [8]=centena e [9]=BCD.

// PINOS

#define PIN_R       16
#define PIN_S1      17
#define PIN_TX      18
#define PIN_OE      19
#define PIN_STB     21

// ENDERECOS

#define RADIO_ADDR     0x100
#define RADIO_ID8      0x00
#define DEVICE_ID8     0x1E
#define DEVICE_ADDR    0x11E

// IEBUS

#define CONTROL_WRITE  0x0F

#define START_H        171
#define START_L         20

#define BIT0_H          32
#define BIT0_L           6

#define BIT1_H          22
#define BIT1_L          16

#define ACK_H           22
#define ACK_L           16

// ACK

#define ACK_ASSERT_US   12
#define ACK_RELEASE_US  24

// PACOTES

#define SEND_OK_LEN       2
#define INIT50_LEN        8
#define COM50_LEN         8
#define COM_STATUS_LEN    3
#define STATUS61_LEN     26

#define V4_COM1023_LEN    12
#define V4_INIT1037_LEN   18
#define V4_COM1034_LEN     9
#define V4_COM1020_LEN     9
#define V4_COM1038_LEN     9

// RMT

#define RMT_TICK_NS       1000.0f

#define TX_ITEMS           384

#define MAX_BITS           400
#define MAX_DATA_BYTES      32

#define START_MIN_US       130
#define START_MAX_US       220

#define HIGH_MIN_US         12
#define HIGH_MAX_US        180

#define BIT_THRESHOLD_US    27
#define FRAME_GAP_US       100

#define RESPONSE_TIMEOUT     80UL
#define A1_SEARCH_MS        300UL

#define RESPONSE_GAP_MS       8

// O Timer1 do firmware original gera EV_STATUS aproximadamente
// a cada 2 segundos e envia STATUS = <01|00> 10.
#define V4_STATUS_PERIOD_MS 2000UL

// pelo menos 2 STATUS periódicos já tiverem A1 e tiverem passado 8 s
#define V4_STATUS_FIRST_MS  1000UL

// FILA

#define QUEUE_SIZE 20

struct AppFrame
{
  uint32_t timeUs;

  uint8_t control;
  uint8_t length;

  uint8_t data[MAX_DATA_BYTES];

  bool controlParityOK;
  bool lengthParityOK;
  bool dataParityOK;
};

DRAM_ATTR volatile AppFrame appQueue[QUEUE_SIZE];

volatile uint8_t queueWrite = 0;
volatile uint8_t queueRead = 0;

volatile bool queueOverflow = false;

// ISR

void IRAM_ATTR busISR();
void IRAM_ATTR bootGPIOISR();
bool IRAM_ATTR ackTimerISR(
  gptimer_handle_t timer,
  const gptimer_alarm_event_data_t *edata,
  void *user_ctx
);

rmt_obj_t *rmtTX = nullptr;

gptimer_handle_t ackTimer = nullptr;

float txTickNs = 0;

rmt_data_t txBuf[TX_ITEMS];

size_t txCount = 0;

bool isrAttached = false;
static volatile bool g_bdkXmEarlyBootArmed = false;

// Handle declarado antes das ISRs para permitir wake-up imediato
// ao concluir um frame do Pioneer.
static TaskHandle_t g_bdkXmTaskHandle = nullptr;

// IDENTIDADE XM FIXA

// FRAME DE BOOT
struct XmBootFrame {
  bool found = false;
  uint16_t bits = 0;
  uint16_t startH = 0;
  uint16_t startL = 0;
  uint8_t bit[MAX_BITS];
};

// ============================================================
// BDK XM V1.0.7.1 - FORWARD DECLARATIONS PARA ESP-IDF/C++
//
// A versão Arduino original recebia protótipos automáticos do builder.
// No ESP-IDF, funções chamadas antes de sua definição precisam ser
// declaradas explicitamente.
// ============================================================
bool getQueuedFrame(AppFrame &f);
bool respondXmCommand30Historical(uint8_t keyCode);
void processXmCommand30(const AppFrame &f);


// PACOTES

uint8_t pktSendOK[SEND_OK_LEN];

uint8_t pktInit50[INIT50_LEN];

uint8_t pktV4Init50Exact[INIT50_LEN];

uint8_t pktCom50[COM50_LEN];

uint8_t pktComStatusInit[COM_STATUS_LEN];

uint8_t pktComStatusReady[COM_STATUS_LEN];

uint8_t pktStatus61[STATUS61_LEN];

// COMMAND30 -> SEND_OK -> gap 8 ms -> COM_11.
// A estrutura COM_11 e historica; DEVICE_ID e keyCode sao os do XM observado.
#define XM_COM11_LEN 9
uint8_t pktXmCom11[XM_COM11_LEN];

uint8_t pktV4Com1023[V4_COM1023_LEN];
uint8_t pktV4Init1037[V4_INIT1037_LEN];
uint8_t pktV4Com1034[V4_COM1034_LEN];
uint8_t pktV4Com1020[V4_COM1020_LEN];
uint8_t pktV4Com1038[V4_COM1038_LEN];

bool runInProgress = false;

uint32_t lastWaitMessage = 0;

uint32_t validFrames = 0;
uint32_t invalidFrames = 0;

uint32_t command00 = 0;
uint32_t command01 = 0;
uint32_t command10 = 0;
uint32_t command30 = 0;
uint32_t command40 = 0;
uint32_t command50 = 0;
uint32_t command70 = 0;
uint32_t commandA1 = 0;
uint32_t commandOther = 0;

uint32_t response01 = 0;
uint32_t response40 = 0;

// XM - CANAL BCD + COMMAND30/COM_11 + BANDA CONFIRMADA
//   STATUS61[7] = nibble alto banda XM1/2/3 + nibble baixo preset P:3
//   STATUS61[8] = centena do canal (0x00, 0x01, 0x02)
//   STATUS61[9] = dois ultimos digitos em BCD empacotado
//   STATUS61[10..13] = preservados conforme base validada
//   BAND 0x20 -> alterna XM1/XM2/XM3 e solicita o perfil de audio correspondente

uint16_t xmChannel = 1;
uint8_t xmBand = 1;
uint8_t xmLastKey = 0x00;
uint32_t xmLastKeyMs = 0;

// V3.2: cada COMMAND30 valido do HU conta como evento real.
// Fluxo historico de tecla: COMMAND30 -> SEND_OK -> 8 ms -> COM_11 -> A1.
#define XM_COM11_MODE                  0x04
#define XM_COM11_A1_TIMEOUT_MS         140UL
#define XM_CHANNEL_A1_TIMEOUT_MS       140UL
#define XM_STATUS_GUARD_AFTER_KEY_MS   180UL

bool xmChannelDirty = false;
uint32_t xmChannelNotBeforeMs = 0;
uint32_t xmChannelTxCount = 0;
uint32_t xmChannelTxFail = 0;

bool xmWaitingCom11A1 = false;
bool xmWaitingChannelA1 = false;
uint32_t xmCom11SentAtMs = 0;
uint32_t xmChannelSentAtMs = 0;
uint32_t xmCom11TxCount = 0;
uint32_t xmCom11AckCount = 0;
uint32_t xmCom11TimeoutCount = 0;
uint32_t xmChannelAckCount = 0;
uint32_t xmChannelAckTimeoutCount = 0;

bool xmInitialPublishPending = true;
bool xmInitialPublished = false;

uint8_t xmLastProcessedKey = 0x00;
bool xmLastCommandChangedChannel = false;

// V3.10 - BANDA XM CONFIRMADA EMPIRICAMENTE NO DEH-P650
//
// STATUS61[7]:
//   0x13 -> XM1 + P:3
//   0x23 -> XM2 + P:3
//   0x33 -> XM3 + P:3
//
// Portanto:
//   nibble alto = banda XM (1, 2, 3)
//   nibble baixo = preset (3, no teste atual)
//
// 0x03 removeu o indicador XM, por isso nao e usado no runtime normal.
uint8_t xmBandNumber = 1;       // 1..3
uint8_t xmPresetNumber = 3;     // preserva P:3 observado
uint8_t xmStatus61Byte7 = 0x13; // XM1 + P:3
uint32_t xmBandChangeCount = 0;

// V1.0.5 - DISPLAY / CANAIS
//
// Canais usados no runtime: CH001..CH255.
// CH000 e a pagina diagnostica antiga foram removidos.
// STATUS61[17..24] continua como janela de 8 caracteres.
// Pagina principal: ARTISTA - FAIXA, passo de 650 ms.
//
#define XM_DISP_KEY 0xDD
#define XM_METADATA_PAGE_COUNT 4
#define XM_SCROLL_PERIOD_MS 650UL
#define XM_SCROLL_START_PAUSE_MS 650UL

// ============================================================
// BDK XM V1.0.7.1 - PRIORIDADE DE COMUNICACAO IP-BUS
// ============================================================
#define BDK_XM_TASK_PRIORITY          8
#define BDK_XM_RUNTIME_VERBOSE        0

// V1.0.7 - mesma estrategia de late-boot usada na base Multi-CD
// Operational V2.13 validada quando radio e ESP energizam juntos.
#define XM_FIRST_FORCED_BOOT_MS       150UL
#define XM_FORCED_BOOT_RETRY_MS       600UL
#define XM_MAX_FORCED_BOOT_ATTEMPTS     8

portMUX_TYPE xmBtMetaMux = portMUX_INITIALIZER_UNLOCKED;

char xmBtArtist[128] = "NO ARTIST";
char xmBtTitle[128]  = "NO TITLE";
char xmBtAlbum[128]  = "NO ALBUM";

volatile bool xmBtConnected = false;
volatile uint32_t xmBtMetadataGeneration = 0;
uint32_t xmBtHandledGeneration = 0;

// Estado da página/rolagem deve ser declarado ANTES dos helpers que o utilizam.
uint8_t xmMetadataPage = 0;
uint32_t xmDispMetadataChangeCount = 0;

uint8_t xmScrollOffset = 0;
uint32_t xmNextScrollMs = 0;
uint32_t xmScrollUpdateCount = 0;

// ============================================================
// V3.26 - SUPERVISOR DE RECUPERACAO APOS PARTIDA / RESET DO HU
//
// O runtime XM normal recebe frames válidos do HU continuamente
// (A1, COMMAND00, COMMAND70, teclas etc.).
//
// Se o HU reiniciar durante a partida do carro, o ESP pode continuar
// preso na sessão antiga. Após um período longo sem NENHUM frame válido,
// abandonamos a sessão atual e voltamos para a captura de boot.
//
// Não inventa novo handshake.
// Não transmite pacote de "reconnect".
// Apenas rearma exatamente a sequência de boot já validada.
//
// Limites conservadores:
// - só arma após 5 s de sessão;
// - 7 s sem frame válido = sessão considerada perdida.
#define XM_RECOVERY_ARM_DELAY_MS 5000UL
#define XM_RECOVERY_SILENCE_MS   7000UL

uint32_t xmLastValidFrameMs = 0;
uint32_t xmRecoveryArmAtMs = 0;
uint32_t xmRecoveryCount = 0;
bool xmRecoveryRequested = false;



static const char *xmProfileDisplayName()
{
  switch (xmBandNumber)
  {
    case 2: return "DSP2";
    case 3: return "HIFI";
    case 1:
    default:
      return "DSP-1";
  }
}

static void copyXmMetadataSource(char *dst, size_t dstSize)
{
  if (!dst || dstSize == 0) return;

  // Pagina 1 = identificacao simples do perfil DSP.
  if (xmMetadataPage == 1)
  {
    strncpy(dst, xmProfileDisplayName(), dstSize - 1);
    dst[dstSize - 1] = 0;
    return;
  }

  char artist[128];
  char title[128];
  bool connected;

  portENTER_CRITICAL(&xmBtMetaMux);

  connected = xmBtConnected;

  strncpy(artist, xmBtArtist, sizeof(artist) - 1);
  artist[sizeof(artist) - 1] = 0;

  strncpy(title, xmBtTitle, sizeof(title) - 1);
  title[sizeof(title) - 1] = 0;

  portEXIT_CRITICAL(&xmBtMetaMux);

  if (!connected)
  {
    strncpy(dst, "BT WAIT", dstSize - 1);
    dst[dstSize - 1] = 0;
    return;
  }

  if (xmMetadataPage == 2)
  {
    strncpy(dst, artist, dstSize - 1);
    dst[dstSize - 1] = 0;
    return;
  }

  if (xmMetadataPage == 3)
  {
    strncpy(dst, title, dstSize - 1);
    dst[dstSize - 1] = 0;
    return;
  }

  // Pagina 0 (padrao):
  // ARTISTA - FAIXA em uma unica sequencia continua.
  // A janela de 8 caracteres avanca no serviceXmMetadataScroll()
  // a cada XM_SCROLL_PERIOD_MS = 650 ms.
  if (artist[0] && title[0])
  {
    snprintf(dst, dstSize, "%s - %s", artist, title);
  }
  else if (artist[0])
  {
    strncpy(dst, artist, dstSize - 1);
    dst[dstSize - 1] = 0;
  }
  else if (title[0])
  {
    strncpy(dst, title, dstSize - 1);
    dst[dstSize - 1] = 0;
  }
  else
  {
    strncpy(dst, "BT AUDIO", dstSize - 1);
    dst[dstSize - 1] = 0;
  }
}




bool first40Seen = false;
bool repeated40Seen = false;

uint32_t first40Ms = 0;
uint32_t firstRepeated40Ms = 0;

bool waitingA1After40 = false;

uint32_t a1After40Count = 0;

bool waitingA1AfterCommand01Init50 = false;
bool waitingA1AfterReady = false;

bool readySentAfterCommand01 = false;
bool readyAckAfterCommand01 = false;

uint32_t readySentAtMs = 0;
uint32_t readyAckAtMs = 0;

uint8_t v4Mode = 0x00;
uint8_t v4OldMode = 0x00;
uint8_t v4Status = 0x01;  // 01=connecting/init, 00=ready

bool v4InitRequested = false;
bool v4Ready = false;

bool v4WaitingCom1023A1 = false;
bool v4WaitingInit1037A1 = false;
bool v4WaitingStatusA1 = false;

uint32_t v4Com1023Sent = 0;
uint32_t v4Com1023Ack = 0;
uint32_t v4Init1037Sent = 0;
uint32_t v4Init1037Ack = 0;

uint32_t v4StatusSent = 0;
uint32_t v4StatusAck = 0;
uint32_t v4StatusTxFail = 0;
uint32_t v4NextStatusMs = 0;

uint32_t v4BranchF6 = 0;
uint32_t v4Branch04 = 0;
uint32_t v4Branch0506 = 0;
uint32_t v4BranchStart02 = 0;
uint32_t v4BranchStop01 = 0;
uint32_t v4Branch03 = 0;

// ISR FRAME STATE

// FIX6: o boot nao depende mais do RMT RX.
// usada pelo decodificador IP-BUS provado no projeto.
volatile bool bootRawActive = false;
volatile bool bootRawReady = false;
volatile uint16_t bootRawBits = 0;
DRAM_ATTR volatile uint8_t bootRawBit[MAX_BITS];
volatile uint16_t bootRawStartH = 0;
volatile uint32_t bootRawLastEdgeUs = 0;
volatile uint8_t bootRawLastLevel = 0;
volatile uint32_t bootRawEdgeCount = 0;
volatile uint32_t bootRawStartCount = 0;
volatile uint32_t bootRaw124Count = 0;

volatile bool engineActive = false;

volatile bool frameActive = false;
volatile bool targetFrame = false;
volatile bool storedThisFrame = false;

volatile uint16_t liveBits = 0;

volatile uint8_t liveIndividual = 0;

volatile uint16_t liveMaster = 0;
volatile uint16_t liveSlave = 0;

volatile uint8_t livePMaster = 0;
volatile uint8_t livePSlave = 0;

volatile uint8_t liveControl = 0;
volatile uint8_t controlXor = 0;

volatile bool liveControlParityOK = false;

volatile uint8_t liveLength = 0;
volatile uint8_t lengthXor = 0;

volatile bool liveLengthParityOK = false;

DRAM_ATTR volatile uint8_t currentData[MAX_DATA_BYTES];

volatile uint8_t dataValue = 0;
volatile uint8_t dataXor = 0;

volatile bool lastDataParityOK = false;
volatile bool allDataParityOK = true;

volatile uint16_t lastDataIndex = 0;

volatile uint16_t expectedBits = 0;

// TIMER

volatile uint32_t lastEdgeTick = 0;
volatile uint8_t lastLevel = 0;

volatile uint64_t sessionStart64 = 0;

volatile uint8_t ackStage = 0;
volatile uint64_t ackBase64 = 0;

// PARIDADE

uint8_t IRAM_ATTR parityBits(
  uint32_t value,
  uint8_t count
)
{
  uint8_t p = 0;

  for (
    uint8_t i = 0;
    i < count;
    i++
  )
  {
    p ^=
      (
        value >>
        i
      ) &
      1U;
  }

  return p;
}

// CHECKSUM

uint8_t swapNib(
  uint8_t v
)
{
  return
    (uint8_t)(
      (v << 4) |
      (v >> 4)
    );
}

uint8_t pioneerChecksum(
  bool individual,
  uint8_t sender,
  uint8_t receiver,
  uint8_t control,
  uint8_t length,
  const uint8_t *data,
  uint8_t count
)
{
  uint16_t sum =
    swapNib(
      (uint8_t)(
        sender +
        receiver
      )
    );

  sum +=
    individual ?
    0xA0 :
    0x20;

  sum += control;
  sum += length;

  for (
    uint8_t i = 0;
    i < count;
    i++
  )
  {
    sum += data[i];
  }

  return
    (uint8_t)sum;
}

void hex2(
  uint8_t v
)
{
  if (
    v < 0x10
  )
  {
    Serial.print('0');
  }

  Serial.print(
    v,
    HEX
  );
}

// RMT HELPERS

// DECODE RMT

uint32_t getBits(
  const XmBootFrame &f,
  uint16_t pos,
  uint8_t count
)
{
  uint32_t value = 0;

  for (
    uint8_t i = 0;
    i < count;
    i++
  )
  {
    value =
      (
        value << 1
      ) |
      f.bit[
        pos + i
      ];
  }

  return value;
}

// BOOT

bool bootBroadcast(
  const XmBootFrame &f
)
{
  if (
    f.bits < 123 ||
    f.bit[0] != 0
  )
  {
    return false;
  }

  if (
    getBits(
      f,
      1,
      12
    ) !=
    RADIO_ADDR
  )
  {
    return false;
  }

  if (
    getBits(
      f,
      14,
      12
    ) !=
    0x1FF
  )
  {
    return false;
  }

  if (
    getBits(
      f,
      28,
      4
    ) !=
    0x0F
  )
  {
    return false;
  }

  if (
    getBits(
      f,
      34,
      8
    ) !=
    0x08
  )
  {
    return false;
  }

  const uint8_t expected[8] =
  {
    0x50,
    0x50,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
    0xD5
  };

  for (
    uint8_t i = 0;
    i < 8;
    i++
  )
  {
    if (
      (uint8_t)
      getBits(
        f,
        44 +
        i * 10,
        8
      ) !=
      expected[i]
    )
    {
      return false;
    }
  }

  return true;
}

// TX BUILDER

bool txSymbol(
  uint16_t h,
  uint16_t l
)
{
  if (
    txCount >=
    TX_ITEMS
  )
  {
    return false;
  }

  txBuf[txCount].level0 =
    1;

  txBuf[txCount].duration0 =
    h;

  txBuf[txCount].level1 =
    0;

  txBuf[txCount].duration1 =
    l;

  txCount++;

  return true;
}

bool txBit(
  uint8_t bit
)
{
  return
    bit ?
    txSymbol(
      BIT1_H,
      BIT1_L
    ) :
    txSymbol(
      BIT0_H,
      BIT0_L
    );
}

bool txBits(
  uint32_t value,
  uint8_t count
)
{
  for (
    int8_t i =
      count - 1;
    i >= 0;
    i--
  )
  {
    if (
      !txBit(
        (
          value >>
          i
        ) &
        1U
      )
    )
    {
      return false;
    }
  }

  return true;
}

bool buildWrite(
  const uint8_t *data,
  uint8_t length
)
{
  memset(
    txBuf,
    0,
    sizeof(txBuf)
  );

  txCount =
    0;

  if (
    !txSymbol(
      START_H,
      START_L
    ) ||
    !txBit(1)
  )
  {
    return false;
  }

  if (
    !txBits(
      DEVICE_ADDR,
      12
    ) ||
    !txBit(
      parityBits(
        DEVICE_ADDR,
        12
      )
    )
  )
  {
    return false;
  }

  if (
    !txBits(
      RADIO_ADDR,
      12
    ) ||
    !txBit(
      parityBits(
        RADIO_ADDR,
        12
      )
    )
  )
  {
    return false;
  }

  if (
    !txSymbol(
      ACK_H,
      ACK_L
    )
  )
  {
    return false;
  }

  if (
    !txBits(
      CONTROL_WRITE,
      4
    ) ||
    !txBit(
      parityBits(
        CONTROL_WRITE,
        4
      )
    )
  )
  {
    return false;
  }

  if (
    !txSymbol(
      ACK_H,
      ACK_L
    )
  )
  {
    return false;
  }

  if (
    !txBits(
      length,
      8
    ) ||
    !txBit(
      parityBits(
        length,
        8
      )
    )
  )
  {
    return false;
  }

  if (
    !txSymbol(
      ACK_H,
      ACK_L
    )
  )
  {
    return false;
  }

  for (
    uint8_t i = 0;
    i < length;
    i++
  )
  {
    if (
      !txBits(
        data[i],
        8
      ) ||
      !txBit(
        parityBits(
          data[i],
          8
        )
      ) ||
      !txSymbol(
        ACK_H,
        ACK_L
      )
    )
    {
      return false;
    }
  }

  return true;
}

// RMT - BDK native compatibility layer
//
// The TX channel remains configured in ESP-IDF. RX/runtime paths detach
// GPIO18 from RMT and return it to ordinary GPIO. setTXMode() re-attaches
// the matrix before every transmission, preserving the proven V3.3.2 fix.

bool initTX()
{
  rmtTX =
    rmtInit(
      PIN_TX,
      RMT_TX_MODE,
      RMT_MEM_256
    );

  if (!rmtTX) return false;

  txTickNs =
    rmtSetTick(
      rmtTX,
      RMT_TICK_NS
    );

  return txTickNs > 0.0f;
}

// TIMER ACK

bool initAckTimer()
{
  gptimer_config_t config = {};
  config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  config.direction = GPTIMER_COUNT_UP;
  config.resolution_hz = 1000000UL; // 1 tick = 1 us

  if (gptimer_new_timer(&config, &ackTimer) != ESP_OK)
  {
    ackTimer = nullptr;
    return false;
  }

  gptimer_event_callbacks_t callbacks = {};
  callbacks.on_alarm = ackTimerISR;

  if (gptimer_register_event_callbacks(ackTimer, &callbacks, nullptr) != ESP_OK)
  {
    return false;
  }

  if (gptimer_enable(ackTimer) != ESP_OK) return false;
  if (gptimer_start(ackTimer) != ESP_OK) return false;

  return true;
}

static inline uint64_t IRAM_ATTR ackTimerReadUs()
{
  uint64_t value = 0;
  if (ackTimer)
  {
    gptimer_get_raw_count(ackTimer, &value);
  }
  return value;
}

bool IRAM_ATTR scheduleAck()
{
  if (ackStage != 0 || !ackTimer) return false;

  if ((((GPIO.in >> PIN_STB) & 1U) == 0)) return false;

  uint64_t ackNow = ackTimerReadUs();

  ackBase64 = ackNow;
  ackStage = 1;

  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = ackNow + ACK_ASSERT_US;
  alarm.flags.auto_reload_on_alarm = false;

  if (gptimer_set_alarm_action(ackTimer, &alarm) != ESP_OK)
  {
    ackStage = 0;
    return false;
  }

  return true;
}

bool IRAM_ATTR ackTimerISR(
  gptimer_handle_t timer,
  const gptimer_alarm_event_data_t *edata,
  void *user_ctx
)
{
  (void)edata;
  (void)user_ctx;

  if (ackStage == 1)
  {
    if ((((GPIO.in >> PIN_STB) & 1U) == 0))
    {
      GPIO.out_w1tc = 1UL << PIN_TX;
      ackStage = 0;
      gptimer_set_alarm_action(timer, nullptr);
      return false;
    }

    GPIO.out_w1ts = 1UL << PIN_TX;
    ackStage = 2;

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = ackBase64 + ACK_RELEASE_US;
    alarm.flags.auto_reload_on_alarm = false;
    gptimer_set_alarm_action(timer, &alarm);
    return false;
  }

  if (ackStage == 2)
  {
    GPIO.out_w1tc = 1UL << PIN_TX;
    ackStage = 0;
    gptimer_set_alarm_action(timer, nullptr);
  }

  return false;
}

static inline void IRAM_ATTR wakeIpbusTaskFromISR()
{
  TaskHandle_t task = g_bdkXmTaskHandle;
  if (task == nullptr) return;

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);

  if (higherPriorityTaskWoken == pdTRUE)
  {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR queueCurrentFrame(
  uint64_t now64
)
{
  if (
    storedThisFrame
  )
  {
    return;
  }

  uint8_t next =
    queueWrite + 1;

  if (
    next >=
    QUEUE_SIZE
  )
  {
    next =
      0;
  }

  if (
    next ==
    queueRead
  )
  {
    queueOverflow =
      true;

    return;
  }

  uint8_t index =
    queueWrite;

  appQueue[index].timeUs =
    (uint32_t)(
      now64 -
      sessionStart64
    );

  appQueue[index].control =
    liveControl;

  appQueue[index].length =
    liveLength;

  appQueue[index].controlParityOK =
    liveControlParityOK;

  appQueue[index].lengthParityOK =
    liveLengthParityOK;

  appQueue[index].dataParityOK =
    allDataParityOK;

  uint8_t len =
    liveLength;

  if (
    len >
    MAX_DATA_BYTES
  )
  {
    len =
      MAX_DATA_BYTES;
  }

  for (
    uint8_t i = 0;
    i < len;
    i++
  )
  {
    appQueue[index].data[i] =
      currentData[i];
  }

  queueWrite =
    next;

  storedThisFrame =
    true;

  // A task IP-BUS pode estar bloqueada aguardando exatamente este frame.
  wakeIpbusTaskFromISR();
}

// RESET FRAME ISR

void IRAM_ATTR resetLiveFrame()
{
  GPIO.out_w1tc =
    1UL <<
    PIN_TX;

  ackStage =
    0;

  frameActive =
    true;

  targetFrame =
    false;

  storedThisFrame =
    false;

  liveBits =
    0;

  liveIndividual =
    0;

  liveMaster =
    0;

  liveSlave =
    0;

  livePMaster =
    0;

  livePSlave =
    0;

  liveControl =
    0;

  controlXor =
    0;

  liveControlParityOK =
    false;

  liveLength =
    0;

  lengthXor =
    0;

  liveLengthParityOK =
    false;

  for (
    uint8_t i = 0;
    i < MAX_DATA_BYTES;
    i++
  )
  {
    currentData[i] =
      0;
  }

  dataValue =
    0;

  dataXor =
    0;

  lastDataParityOK =
    false;

  allDataParityOK =
    true;

  lastDataIndex =
    0;

  expectedBits =
    0;
}

// BOOT RX ISR - GPIO16 DIRETO
// O RMT RX do Arduino-ESP32 3.x nao reproduziu o comportamento
// do RMT legado usado pela V63.1. Para o boot, registramos apenas

void IRAM_ATTR bootGPIOISR()
{
  uint32_t now =
    (uint32_t)esp_timer_get_time();

  uint8_t current =
    (
      GPIO.in >>
      PIN_R
    ) &
    1U;

  uint8_t previous =
    bootRawLastLevel;

  uint32_t duration =
    now -
    bootRawLastEdgeUs;

  bootRawLastEdgeUs =
    now;

  bootRawLastLevel =
    current;

  bootRawEdgeCount++;

  if (
    bootRawReady
  )
  {
    return;
  }

  if (
    current != 0 ||
    previous != 1
  )
  {
    return;
  }

  uint32_t highUs =
    duration;

  if (
    highUs >=
      START_MIN_US &&
    highUs <=
      START_MAX_US
  )
  {
    bootRawActive =
      true;

    bootRawReady =
      false;

    bootRawBits =
      0;

    bootRawStartH =
      (uint16_t)highUs;

    bootRawStartCount++;

    return;
  }

  if (
    !bootRawActive
  )
  {
    return;
  }

  if (
    highUs <
      HIGH_MIN_US ||
    highUs >
      HIGH_MAX_US
  )
  {
    bootRawActive =
      false;

    bootRawBits =
      0;

    return;
  }

  uint16_t index =
    bootRawBits;

  if (
    index >=
      MAX_BITS
  )
  {
    bootRawActive =
      false;

    bootRawBits =
      0;

    return;
  }

  bootRawBit[index] =
    highUs >=
      BIT_THRESHOLD_US ?
    0 :
    1;

  index++;

  bootRawBits =
    index;

  if (
    index >=
      124
  )
  {
    bootRawReady =
      true;

    bootRawActive =
      false;

    bootRaw124Count++;

    // Evita depender do polling de 1 ms/RTOS para reconhecer o boot.
    wakeIpbusTaskFromISR();
  }
}

// BUS ISR

void IRAM_ATTR busISR()
{
  if (
    !engineActive
  )
  {
    return;
  }

  // esp_timer_get_time() e seguro em ISR e fornece tempo em us.
  uint64_t now64 =
    (uint64_t)esp_timer_get_time();

  uint32_t now =
    (uint32_t)now64;

  uint8_t current =
    (
      GPIO.in >>
      PIN_R
    ) &
    1U;

  uint8_t previous =
    lastLevel;

  uint32_t duration =
    now -
    lastEdgeTick;

  lastEdgeTick =
    now;

  lastLevel =
    current;

  if (
    current == 1
  )
  {
    if (
      !frameActive
    )
    {
      return;
    }

    uint16_t slot =
      liveBits;

    // ADDRESS ACK

    if (
      slot == 27
    )
    {
      bool headerOK =
        liveIndividual == 1 &&
        liveMaster ==
          RADIO_ADDR &&
        liveSlave ==
          DEVICE_ADDR &&
        livePMaster ==
          parityBits(
            RADIO_ADDR,
            12
          ) &&
        livePSlave ==
          parityBits(
            DEVICE_ADDR,
            12
          );

      if (
        headerOK
      )
      {
        targetFrame =
          true;

        scheduleAck();
      }

      return;
    }

    if (
      !targetFrame
    )
    {
      return;
    }

    // CONTROL ACK

    if (
      slot == 33
    )
    {
      if (
        liveControlParityOK &&
        liveControl ==
          CONTROL_WRITE
      )
      {
        scheduleAck();
      }

      return;
    }

    // LENGTH ACK

    if (
      slot == 43
    )
    {
      if (
        liveLengthParityOK &&
        liveLength > 0 &&
        liveLength <=
          MAX_DATA_BYTES
      )
      {
        scheduleAck();
      }

      return;
    }

    // DATA ACK

    if (
      slot >= 53 &&
      (
        (
          slot -
          53
        ) %
        10
      ) == 0
    )
    {
      uint16_t byteIndex =
        (
          slot -
          53
        ) /
        10;

      if (
        byteIndex <
          liveLength &&
        byteIndex <
          MAX_DATA_BYTES &&
        lastDataParityOK &&
        lastDataIndex ==
          byteIndex
      )
      {
        scheduleAck();
      }

      return;
    }

    return;
  }

  if (
    previous != 1
  )
  {
    return;
  }

  uint32_t highUs =
    duration;

  if (
    highUs >=
      START_MIN_US &&
    highUs <=
      START_MAX_US
  )
  {
    resetLiveFrame();

    return;
  }

  if (
    !frameActive ||
    highUs <
      HIGH_MIN_US ||
    highUs >
      HIGH_MAX_US
  )
  {
    return;
  }

  uint16_t index =
    liveBits;

  if (
    index >=
    MAX_BITS
  )
  {
    return;
  }

  uint8_t bit =
    highUs >=
      BIT_THRESHOLD_US ?
    0 :
    1;

  if (
    index == 0
  )
  {
    liveIndividual =
      bit;
  }

  else if (
    index >= 1 &&
    index <= 12
  )
  {
    liveMaster =
      (
        liveMaster << 1
      ) |
      bit;
  }

  else if (
    index == 13
  )
  {
    livePMaster =
      bit;
  }

  else if (
    index >= 14 &&
    index <= 25
  )
  {
    liveSlave =
      (
        liveSlave << 1
      ) |
      bit;
  }

  else if (
    index == 26
  )
  {
    livePSlave =
      bit;
  }

  else if (
    index >= 28 &&
    index <= 31
  )
  {
    liveControl =
      (
        liveControl << 1
      ) |
      bit;

    controlXor ^=
      bit;
  }

  else if (
    index == 32
  )
  {
    liveControlParityOK =
      bit ==
      controlXor;
  }

  else if (
    index >= 34 &&
    index <= 41
  )
  {
    liveLength =
      (
        liveLength << 1
      ) |
      bit;

    lengthXor ^=
      bit;
  }

  else if (
    index == 42
  )
  {
    liveLengthParityOK =
      bit ==
      lengthXor;

    if (
      liveControl ==
        CONTROL_WRITE &&
      liveLength > 0 &&
      liveLength <=
        MAX_DATA_BYTES
    )
    {
      expectedBits =
        44 +
        liveLength *
        10;
    }
  }

  else if (
    index >= 44
  )
  {
    uint16_t rel =
      index -
      44;

    uint16_t byteIndex =
      rel /
      10;

    uint8_t pos =
      rel %
      10;

    if (
      byteIndex <
      MAX_DATA_BYTES
    )
    {
      if (
        pos <= 7
      )
      {
        if (
          pos == 0
        )
        {
          dataValue =
            0;

          dataXor =
            0;
        }

        dataValue =
          (
            dataValue << 1
          ) |
          bit;

        dataXor ^=
          bit;
      }

      else if (
        pos == 8
      )
      {
        currentData[
          byteIndex
        ] =
          dataValue;

        lastDataIndex =
          byteIndex;

        lastDataParityOK =
          bit ==
          dataXor;

        if (
          !lastDataParityOK
        )
        {
          allDataParityOK =
            false;
        }
      }
    }
  }

  liveBits++;

  if (
    targetFrame &&
    liveControl ==
      CONTROL_WRITE &&
    expectedBits > 0 &&
    liveBits >=
      expectedBits &&
    !storedThisFrame
  )
  {
    queueCurrentFrame(
      now64
    );
  }
}

void stopAppListener()
{
  engineActive =
    false;

  GPIO.out_w1tc =
    1UL <<
    PIN_TX;

  GPIO.out_w1ts =
    1UL <<
    PIN_OE;

  if (ackTimer)
  {
    gptimer_set_alarm_action(ackTimer, nullptr);
  }

  if (
    isrAttached
  )
  {
    detachInterrupt(
      digitalPinToInterrupt(
        PIN_R
      )
    );

    isrAttached =
      false;
  }
}

bool waitSTBHigh(
  uint32_t timeoutMs
)
{
  uint32_t start =
    millis();

  while (
    millis() -
      start <
      timeoutMs
  )
  {
    if (
      digitalRead(
        PIN_STB
      ) ==
      HIGH
    )
    {
      return true;
    }

    delay(1);
  }

  return false;
}

bool startAppListener()
{
  stopAppListener();

  pinMatrixOutDetach(
    PIN_TX,
    false,
    false
  );

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
  if (!waitSTBHigh(500)) return false;
  frameActive = false;
  targetFrame = false;
  liveBits = 0;
  ackStage = 0;
  lastEdgeTick = (uint32_t)esp_timer_get_time();
  lastLevel = digitalRead(PIN_R);
  engineActive = true;
  attachInterrupt(digitalPinToInterrupt(PIN_R), busISR, CHANGE);
  isrAttached = true;
  digitalWrite(PIN_OE, LOW);
  return true;
}

// TX MODE

void setTXMode()
{
  stopAppListener();

  // HCT244 em Hi-Z enquanto o RMT e preparado.
  GPIO.out_w1ts = 1UL << PIN_OE;

  // Nivel de repouso do sinal de entrada do HCT244.
  GPIO.out_w1tc = 1UL << PIN_TX;

  // startAppListener()/prepareBootCapture() devolvem GPIO18 ao GPIO.
  // Reanexa explicitamente o RMT TX antes de CADA transmissao.
  pinMatrixOutAttach(
    PIN_TX,
    RMT_SIG_OUT0_IDX,
    false,
    false
  );
}

bool sendPacket(
  const uint8_t *data,
  uint8_t length
)
{
  if (
    !buildWrite(
      data,
      length
    )
  )
  {
    return false;
  }

  setTXMode();

  uint32_t start =
    millis();

  while (
    millis() -
      start <
      500
  )
  {
    if (
      digitalRead(
        PIN_STB
      ) ==
        HIGH &&
      digitalRead(
        PIN_R
      ) ==
        LOW
    )
    {
      break;
    }

    delay(1);
  }

  if (
    digitalRead(
      PIN_STB
    ) ==
      LOW ||
    digitalRead(
      PIN_R
    ) !=
      LOW
  )
  {
    return false;
  }

  GPIO.out_w1tc =
    1UL <<
    PIN_OE;

  delayMicroseconds(
    5
  );

  bool ok =
    rmtWriteBlocking(rmtTX, txBuf, txCount);

  GPIO.out_w1ts =
    1UL <<
    PIN_OE;

  return ok;
}

// mas o RX nao usa mais RMT. O GPIO16 e decodificado por busISR().

bool startRMTListener()
{
  return startAppListener();
}

bool sendExpectA1(
  const char *name,
  const uint8_t *data,
  uint8_t length
)
{
  Serial.print(
    name
  );

  Serial.print(
    " -> "
  );

  if (
    !sendPacket(
      data,
      length
    )
  )
  {
    Serial.println(
      "TX FALHOU"
    );

    return false;
  }

  noInterrupts();

  queueRead =
    0;

  queueWrite =
    0;

  queueOverflow =
    false;

  interrupts();

  if (
    !startRMTListener()
  )
  {
    Serial.println(
      "RX GPIO FALHOU"
    );

    return false;
  }

  uint32_t start =
    millis();

  uint8_t framesSeen =
    0;

  while (
    millis() -
      start <
      A1_SEARCH_MS
  )
  {
    AppFrame af;

    if (
      !getQueuedFrame(
        af
      )
    )
    {
      delayMicroseconds(
        100
      );

      continue;
    }

    framesSeen++;

    bool valid =
      af.control ==
        CONTROL_WRITE &&
      af.length ==
        2 &&
      af.data[0] ==
        0xA1 &&
      af.controlParityOK &&
      af.lengthParityOK &&
      af.dataParityOK;

    if (
      valid
    )
    {
      uint8_t calc =
        pioneerChecksum(
          true,
          RADIO_ID8,
          DEVICE_ID8,
          af.control,
          af.length,
          af.data,
          1
        );

      valid =
        calc ==
        af.data[1];
    }

    if (
      valid
    )
    {
      uint8_t responseCs =
        af.data[1];

      stopAppListener();

      Serial.print(
        "A1 "
      );

      hex2(
        responseCs
      );

      Serial.print(
        " OK"
      );

      if (
        framesSeen >
          1
      )
      {
        Serial.print(
          " (apos "
        );

        Serial.print(
          framesSeen
        );

        Serial.print(
          " frames)"
        );
      }

      Serial.println();

      return true;
    }
  }

  stopAppListener();

  Serial.println(
    "A1 NAO ENCONTRADO"
  );

  return false;
}

// PREPARE PACKETS

void preparePackets()
{

  pktSendOK[0] =
    0xA1;

  pktSendOK[1] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      SEND_OK_LEN,
      pktSendOK,
      1
    );

  const uint8_t initBase[7] =
  {
    0x50,
    0x10,
    DEVICE_ID8,
    0x00,
    0x20,
    0x00,
    0x04
  };

  memcpy(
    pktInit50,
    initBase,
    7
  );

  pktInit50[7] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      INIT50_LEN,
      pktInit50,
      7
    );

  // XM DISCOVERY - INIT_50 baseado no ipbus_v4.0 / command.h

  const uint8_t v4Init50ExactBase[7] =
  {
    0x50,
    0x10,
    DEVICE_ID8,
    0x00,
    0x00,
    0x01,
    0x04
  };

  memcpy(
    pktV4Init50Exact,
    v4Init50ExactBase,
    7
  );

  pktV4Init50Exact[7] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      INIT50_LEN,
      pktV4Init50Exact,
      7
    );

  const uint8_t com50Base[7] =
  {
    0x50,
    0x10,
    DEVICE_ID8,
    0x00,
    0x20,
    0x01,
    0x04
  };

  memcpy(
    pktCom50,
    com50Base,
    7
  );

  pktCom50[7] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      COM50_LEN,
      pktCom50,
      7
    );

  // STATUS INIT

  pktComStatusInit[0] =
    0x01;

  pktComStatusInit[1] =
    0x10;

  pktComStatusInit[2] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      COM_STATUS_LEN,
      pktComStatusInit,
      2
    );

  // STATUS READY

  pktComStatusReady[0] =
    0x00;

  pktComStatusReady[1] =
    0x10;

  pktComStatusReady[2] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      COM_STATUS_LEN,
      pktComStatusReady,
      2
    );
}

// STATUS 61

void updateStatus61()
{
  memset(
    pktStatus61,
    0,
    sizeof(pktStatus61)
  );

  pktStatus61[0] =
    0x61;

  pktStatus61[1] =
    0x10;

  pktStatus61[2] =
    DEVICE_ID8;

  pktStatus61[3] =
    0x00;

  pktStatus61[4] =
    0x20;

  pktStatus61[5] =
    0x04;

  pktStatus61[6] =
    0x00;

  pktStatus61[7] =
    0x06;

  pktStatus61[8] =
    0x01;

  pktStatus61[9] =
    0x00;

  pktStatus61[10] =
    0x00;

  pktStatus61[11] =
    0x00;

  pktStatus61[12] =
    0x00;

  pktStatus61[13] =
    0x01;

  pktStatus61[25] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      STATUS61_LEN,
      pktStatus61,
      25
    );
}

// XM DISCOVERY - PACOTES BASEADOS NO ipbus_v4.0 ORIGINAL

void buildV4Com1023(
  uint8_t mode
)
{
  const uint8_t base[11] =
  {
    0x10, 0x10, DEVICE_ID8, 0x00,
    0x23, 0x01, 0x00, 0x83,
    0x41, 0x00, mode
  };

  memcpy(
    pktV4Com1023,
    base,
    11
  );

  pktV4Com1023[11] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      V4_COM1023_LEN,
      pktV4Com1023,
      11
    );
}

void buildV4Init1037(
  uint8_t mode
)
{
  const uint8_t base[17] =
  {
    0x10, 0x10, DEVICE_ID8, 0x00,
    0x37, 0x01, 0x00, 0x83,
    0x41, 0x00, mode, 0x40,
    0xF6, 0x4A, 0x03, 0x03,
    0x01
  };

  memcpy(
    pktV4Init1037,
    base,
    17
  );

  pktV4Init1037[17] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      V4_INIT1037_LEN,
      pktV4Init1037,
      17
    );
}

void buildV4Com1034(
  uint8_t mode,
  uint8_t eventCode
)
{
  const uint8_t base[8] =
  {
    0x10, 0x10, DEVICE_ID8, 0x00,
    0x34, mode, 0x40, eventCode
  };

  memcpy(
    pktV4Com1034,
    base,
    8
  );

  pktV4Com1034[8] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      V4_COM1034_LEN,
      pktV4Com1034,
      8
    );
}

void buildV4Com1020(
  uint8_t mode,
  uint8_t eventCode
)
{
  const uint8_t base[8] =
  {
    0x10, 0x10, DEVICE_ID8, 0x00,
    0x20, mode, 0x40, eventCode
  };

  memcpy(
    pktV4Com1020,
    base,
    8
  );

  pktV4Com1020[8] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      V4_COM1020_LEN,
      pktV4Com1020,
      8
    );
}

void buildV4Com1038(
  uint8_t mode,
  uint8_t eventCode
)
{
  const uint8_t base[8] =
  {
    0x10, 0x10, DEVICE_ID8, 0x00,
    0x38, mode, 0x40, eventCode
  };

  memcpy(
    pktV4Com1038,
    base,
    8
  );

  pktV4Com1038[8] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      V4_COM1038_LEN,
      pktV4Com1038,
      8
    );
}

void prepareV4Packets()
{
  buildV4Com1023(
    0x00
  );

  buildV4Init1037(
    0x00
  );

  buildV4Com1034(
    0x00,
    0x04
  );

  buildV4Com1020(
    0x00,
    0x06
  );

  buildV4Com1038(
    0x00,
    0x03
  );
}

// FIX6 - RECONSTROI TODOS OS PACOTES PARA O ID ATIVO

void refreshIdentityPackets()
{
  preparePackets();
  updateStatus61();
  prepareV4Packets();
}

bool selfTest()
{
  return
    pktSendOK[SEND_OK_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, SEND_OK_LEN, pktSendOK, SEND_OK_LEN - 1) &&
    pktInit50[INIT50_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, INIT50_LEN, pktInit50, INIT50_LEN - 1) &&
    pktV4Init50Exact[INIT50_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, INIT50_LEN, pktV4Init50Exact, INIT50_LEN - 1) &&
    pktCom50[COM50_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, COM50_LEN, pktCom50, COM50_LEN - 1) &&
    pktComStatusInit[COM_STATUS_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, COM_STATUS_LEN, pktComStatusInit, COM_STATUS_LEN - 1) &&
    pktComStatusReady[COM_STATUS_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, COM_STATUS_LEN, pktComStatusReady, COM_STATUS_LEN - 1) &&
    pktStatus61[STATUS61_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, STATUS61_LEN, pktStatus61, STATUS61_LEN - 1) &&
    pktV4Com1023[V4_COM1023_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, V4_COM1023_LEN, pktV4Com1023, V4_COM1023_LEN - 1) &&
    pktV4Init1037[V4_INIT1037_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, V4_INIT1037_LEN, pktV4Init1037, V4_INIT1037_LEN - 1) &&
    pktV4Com1034[V4_COM1034_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, V4_COM1034_LEN, pktV4Com1034, V4_COM1034_LEN - 1) &&
    pktV4Com1020[V4_COM1020_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, V4_COM1020_LEN, pktV4Com1020, V4_COM1020_LEN - 1) &&
    pktV4Com1038[V4_COM1038_LEN - 1] == pioneerChecksum(true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE, V4_COM1038_LEN, pktV4Com1038, V4_COM1038_LEN - 1);
}

bool runInitialSequence()
{
  Serial.println();

  Serial.println(
    "=============================================="
  );

  Serial.println(
    " SEQUENCIA BOOT XM PROVADA / LINK 0x11E APP 0x1E"
  );

  Serial.println(
    "=============================================="
  );

  Serial.print(
    "SEND_OK -> "
  );

  if (
    !sendPacket(
      pktSendOK,
      SEND_OK_LEN
    )
  )
  {
    Serial.println(
      "TX FALHOU"
    );

    return false;
  }

  Serial.println(
    "TX OK"
  );

  delay(20);

  if (
    !sendExpectA1(
      "INIT_50",
      pktInit50,
      INIT50_LEN
    )
  )
  {
    return false;
  }

  delay(20);

  bool ok =
    sendExpectA1(
      "COM_50",
      pktCom50,
      COM50_LEN
    );

  if (
    !ok
  )
  {
    Serial.println(
      "COM_50 sem A1; continuando."
    );
  }

  delay(20);

  updateStatus61();

  Serial.println(
    "STATUS61              : XM1 / P:3 / canal BCD / texto XM"
  );

  ok =
    sendExpectA1(
      "COM_61 XM STATUS",
      pktStatus61,
      STATUS61_LEN
    );

  if (
    !ok
  )
  {
    Serial.println(
      "COM_61 sem A1; continuando."
    );
  }

  delay(20);

  ok =
    sendExpectA1(
      "COM_STATUS INIT",
      pktComStatusInit,
      COM_STATUS_LEN
    );

  if (
    !ok
  )
  {
    Serial.println(
      "COM_STATUS INIT sem A1."
    );
  }

  delay(20);

  Serial.println(
    "COM_STATUS READY       : NAO ENVIADO AQUI; original libera apos INIT_10_37"
  );

  Serial.println();

  Serial.println(
    "*** BOOT XM EMULATOR V1 FINALIZADO EM STATUS INIT; AGUARDANDO COMMAND01/F6 ***"
  );

  return true;
}

bool getQueuedFrame(
  AppFrame &f
)
{
  noInterrupts();

  if (
    queueRead ==
    queueWrite
  )
  {
    interrupts();

    return false;
  }

  uint8_t index =
    queueRead;

  memcpy(
    &f,
    (const void *)
    &appQueue[index],
    sizeof(f)
  );

  queueRead++;

  if (
    queueRead >=
    QUEUE_SIZE
  )
  {
    queueRead =
      0;
  }

  interrupts();

  return true;
}

// PRINT VALID FRAME

bool respondSendOKOnly()
{
  stopAppListener();

  bool ok =
    sendPacket(
      pktSendOK,
      SEND_OK_LEN
    );

  bool armed =
    startAppListener();

  return
    ok &&
    armed;
}

bool respond01()
{

  v4Mode =
    0x00;

  v4OldMode =
    0x00;

  stopAppListener();

  bool ok1 =
    sendPacket(
      pktSendOK,
      SEND_OK_LEN
    );

  delay(
    RESPONSE_GAP_MS
  );

  bool ok2 =
    sendPacket(
      pktV4Init50Exact,
      INIT50_LEN
    );

  bool armed =
    startAppListener();

  if (
    ok1 &&
    ok2 &&
    armed
  )
  {
    waitingA1AfterCommand01Init50 =
      true;
  }

  Serial.print(
    "RESPOSTA 01 V63      : SEND_OK="
  );

  Serial.print(
    ok1 ?
      "OK" :
      "FAIL"
  );

  Serial.print(
    " INIT_50_EXATO="
  );

  Serial.print(
    ok2 ?
      "OK" :
      "FAIL"
  );

  Serial.print(
    " PACKET="
  );

  for (
    uint8_t i = 0;
    i < INIT50_LEN;
    i++
  )
  {
    hex2(
      pktV4Init50Exact[i]
    );

    Serial.print(' ');
  }

  Serial.print(
    " AGUARDA_A1="
  );

  Serial.print(
    waitingA1AfterCommand01Init50 ?
      "SIM" :
      "NAO"
  );

  Serial.print(
    " LISTENER="
  );

  Serial.println(
    armed ?
      "OK" :
      "FAIL"
  );

  return
    ok1 &&
    ok2 &&
    armed;
}

// Documented mapping: HU 0x10 -> SEND_OK + COM_STATUS.

bool respond10()
{
  return respondSendOKOnly();
}

bool respond40(const AppFrame &request)
{
  const uint8_t message7 = request.length > 6 ? request.data[6] : 0xFF;
  const uint8_t message9 = request.length > 8 ? request.data[8] : 0xFF;

  stopAppListener();
  const bool okSendOK = sendPacket(pktSendOK, SEND_OK_LEN);
  bool responseOK = true;

  if (message9 == 0xF6) {
    v4BranchF6++;
    v4OldMode = 0xFF;
    v4InitRequested = true;
    waitingA1AfterCommand01Init50 = false;
    buildV4Com1023(v4Mode);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1023, V4_COM1023_LEN);
    v4Com1023Sent++;
    v4WaitingCom1023A1 = responseOK;
    v4OldMode = v4Mode;
  } else if (message9 == 0x04) {
    v4Branch04++;
    buildV4Com1034(v4Mode, message9);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1034, V4_COM1034_LEN);
    waitingA1After40 = responseOK;
  } else if (message9 == 0x06 || message9 == 0x05) {
    v4Branch0506++;
    buildV4Com1020(v4Mode, message9);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1020, V4_COM1020_LEN);
    waitingA1After40 = responseOK;
  } else if (message7 == 0x02) {
    v4BranchStart02++;
    v4Mode = 0x02;
    buildV4Com1034(v4Mode, message7);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1034, V4_COM1034_LEN);
    waitingA1After40 = responseOK;
  } else if (message7 == 0x01) {
    v4BranchStop01++;
    v4Mode = 0x00;
    buildV4Com1038(v4Mode, message7);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1038, V4_COM1038_LEN);
    waitingA1After40 = responseOK;
  } else if (message9 == 0x03) {
    v4Branch03++;
    buildV4Com1038(v4Mode, message9);
    delay(RESPONSE_GAP_MS);
    responseOK = sendPacket(pktV4Com1038, V4_COM1038_LEN);
    waitingA1After40 = responseOK;
  }

  const bool armed = startAppListener();
  return okSendOK && responseOK && armed;
}

bool sendV4Init1037Now()
{
  stopAppListener();
  buildV4Init1037(v4Mode);
  const bool ok = sendPacket(pktV4Init1037, V4_INIT1037_LEN);
  const bool armed = startAppListener();
  if (ok && armed) {
    v4Init1037Sent++;
    v4WaitingInit1037A1 = true;
    v4InitRequested = false;
    v4Status = 0x00;
    v4Ready = true;
    if (v4NextStatusMs == 0) v4NextStatusMs = millis() + V4_STATUS_PERIOD_MS;
  }
  return ok && armed;
}

// V63 - STATUS PERIODICO ORIGINAL (01 10 / 00 10)

void serviceV4Status()
{
  if (!v4Ready || v4NextStatusMs == 0) return;
  const uint32_t now = millis();
  if ((int32_t)(now - v4NextStatusMs) < 0) return;

  if (xmWaitingCom11A1 || xmWaitingChannelA1 || xmChannelDirty ||
      (xmLastKeyMs != 0 && (uint32_t)(now - xmLastKeyMs) < XM_STATUS_GUARD_AFTER_KEY_MS)) {
    v4NextStatusMs = now + 80UL;
    return;
  }

  if (v4WaitingCom1023A1 || v4WaitingInit1037A1 || waitingA1After40 ||
      waitingA1AfterCommand01Init50) {
    v4NextStatusMs = now + 50UL;
    return;
  }

  const bool ok = sendPacket(pktComStatusReady, COM_STATUS_LEN);
  const bool armed = startAppListener();
  if (ok && armed) {
    v4StatusSent++;
    v4WaitingStatusA1 = true;
  } else {
    v4StatusTxFail++;
  }
  v4NextStatusMs = millis() + V4_STATUS_PERIOD_MS;
}

// PROCESS FRAME

void processAppFrame(const AppFrame &f)
{
  if (f.length == 0 || f.length > MAX_DATA_BYTES) {
    invalidFrames++;
    return;
  }

  const bool parityOK = f.controlParityOK && f.lengthParityOK && f.dataParityOK;
  const uint8_t calc = pioneerChecksum(true, RADIO_ID8, DEVICE_ID8,
                                        f.control, f.length, f.data, f.length - 1);
  if (!parityOK || calc != f.data[f.length - 1]) {
    invalidFrames++;
    return;
  }

  validFrames++;

  // V3.26: qualquer frame válido vindo do HU prova que a sessão IP-BUS
  // ainda está viva.
  xmLastValidFrameMs = millis();

  const uint8_t command = f.data[0];

  if (command == 0xA1) {
    commandA1++;
    if (xmWaitingCom11A1) {
      xmWaitingCom11A1 = false;
      xmCom11AckCount++;
      if (xmChannelDirty) xmChannelNotBeforeMs = millis();
      return;
    }
    if (xmWaitingChannelA1) {
      xmWaitingChannelA1 = false;
      xmChannelAckCount++;
      return;
    }
    if (v4WaitingCom1023A1) {
      v4WaitingCom1023A1 = false;
      v4Com1023Ack++;
      sendV4Init1037Now();
      return;
    }
    if (v4WaitingInit1037A1) {
      v4WaitingInit1037A1 = false;
      v4Init1037Ack++;
      readyAckAfterCommand01 = true;
      readyAckAtMs = millis();
      return;
    }
    if (v4WaitingStatusA1) {
      v4WaitingStatusA1 = false;
      v4StatusAck++;
      return;
    }
    if (waitingA1AfterCommand01Init50) {
      waitingA1AfterCommand01Init50 = false;
      return;
    }
    if (waitingA1After40) {
      waitingA1After40 = false;
      a1After40Count++;
    }
    return;
  }

  if (command == 0x00) {
    command00++;
    respondSendOKOnly();
    return;
  }

  if (command == 0x01) {
    command01++;
    response01++;
    respond01();
    return;
  }

  if (command == 0x10) {
    command10++;
    respond10();
    return;
  }

  if (command == 0x30) {
    command30++;
    const uint8_t keyCode = f.length > 6 ? f.data[6] : 0xFF;
    if (v4WaitingStatusA1) v4WaitingStatusA1 = false;
    if (xmWaitingChannelA1) xmWaitingChannelA1 = false;
    if (xmWaitingCom11A1) xmWaitingCom11A1 = false;
    processXmCommand30(f);
    const bool txOK = respondXmCommand30Historical(keyCode);
    if (!txOK) Serial.println("XM KEY TX FAIL");
    return;
  }

  if (command == 0x40) {
    command40++;
    response40++;
    respond40(f);
    return;
  }

  if (command == 0x50) {
    command50++;
    respondSendOKOnly();
    return;
  }

  if (command == 0x70) {
    command70++;
    respondSendOKOnly();
    return;
  }

  commandOther++;
  respondSendOKOnly();
}

// XM V3.2 - COM_11 HISTORICO PARA COMMAND30

void buildXmCom11(uint8_t keyCode, bool keyRelease)
{
  pktXmCom11[0] = 0x11;
  pktXmCom11[1] = 0x10;
  pktXmCom11[2] = DEVICE_ID8;
  pktXmCom11[3] = 0x00;
  pktXmCom11[4] = keyRelease ? 0x20 : 0x24;
  pktXmCom11[5] = XM_COM11_MODE;
  pktXmCom11[6] = 0x30;
  pktXmCom11[7] = keyCode;
  pktXmCom11[8] = pioneerChecksum(
    true, DEVICE_ID8, RADIO_ID8, CONTROL_WRITE,
    XM_COM11_LEN, pktXmCom11, XM_COM11_LEN - 1
  );
}

bool respondXmCommand30Historical(uint8_t keyCode)
{
  const bool keyRelease = (keyCode == 0x00);

  stopAppListener();

  bool okSend = sendPacket(pktSendOK, SEND_OK_LEN);

  // O fast-path validado permanece em 8 ms. Aqui usamos atraso em
  // microssegundos para nao depender da resolucao do tick do FreeRTOS.
  if (okSend) delayMicroseconds((uint32_t)RESPONSE_GAP_MS * 1000U);

  buildXmCom11(keyCode, keyRelease);

  bool okCom11 = false;
  if (okSend)
  {
    okCom11 = sendPacket(pktXmCom11, XM_COM11_LEN);
  }

  bool armed = startAppListener();

  if (okSend && okCom11 && armed)
  {
    xmWaitingCom11A1 = true;
    xmCom11SentAtMs = millis();
    xmCom11TxCount++;
    return true;
  }

  xmWaitingCom11A1 = false;
  return false;
}

// XM V3 - HELPERS DE CANAL REAL

static uint8_t xmChannelHundreds(uint16_t ch)
{
  return (uint8_t)(ch / 100U);
}

static uint8_t xmChannelLowBcd(uint16_t ch)
{
  uint8_t low = (uint8_t)(ch % 100U);
  uint8_t tens = (uint8_t)(low / 10U);
  uint8_t ones = (uint8_t)(low % 10U);
  return (uint8_t)((tens << 4) | ones);
}




void buildXmMetadataWindow(char out8[9])
{
  char src[128];
  copyXmMetadataSource(src, sizeof(src));
  const size_t len = strlen(src);

  // Espaço visual entre o final e o reinicio da frase.
  const uint8_t gap = 3;
  const size_t cycleLen = len + gap;

  for (uint8_t i = 0; i < 8; i++)
  {
    size_t pos = (xmScrollOffset + i) % cycleLen;

    if (pos < len)
    {
      out8[i] = src[pos];
    }
    else
    {
      out8[i] = ' ';
    }
  }

  out8[8] = '\0';
}



void updateXmChannelStatus61()
{
  updateStatus61();

  pktStatus61[7] = xmStatus61Byte7;

  for (uint8_t i = 8; i <= 13; i++)
  {
    pktStatus61[i] = 0x00;
  }

  pktStatus61[8] = xmChannelHundreds(xmChannel);
  pktStatus61[9] = xmChannelLowBcd(xmChannel);

  char window8[9];
  buildXmMetadataWindow(window8);

  for (uint8_t i = 0; i < 8; i++)
  {
    pktStatus61[17 + i] =
      (uint8_t)window8[i];
  }

  pktStatus61[STATUS61_LEN - 1] =
    pioneerChecksum(
      true,
      DEVICE_ID8,
      RADIO_ID8,
      CONTROL_WRITE,
      STATUS61_LEN,
      pktStatus61,
      STATUS61_LEN - 1
    );
}

void serviceXmChannelUpdate()
{
  const uint32_t now = millis();

  if (xmWaitingCom11A1)
  {
    if ((uint32_t)(now - xmCom11SentAtMs) < XM_COM11_A1_TIMEOUT_MS) return;
    xmWaitingCom11A1 = false;
    xmCom11TimeoutCount++;
  }

  if (xmWaitingChannelA1)
  {
    if ((uint32_t)(now - xmChannelSentAtMs) < XM_CHANNEL_A1_TIMEOUT_MS) return;
    xmWaitingChannelA1 = false;
    xmChannelAckTimeoutCount++;
  }

  // V3 enviava um STATUS61 de 26 bytes cedo demais, antes do COMMAND01/F6.
  if (xmInitialPublishPending && !xmInitialPublished && v4BranchStart02 > 0)
  {
    xmInitialPublishPending = false;
    xmInitialPublished = true;
    xmChannelDirty = true;
    xmChannelNotBeforeMs = now + 60UL;
  }

  if (!xmChannelDirty) return;
  if ((int32_t)(now - xmChannelNotBeforeMs) < 0) return;

  if (
    v4WaitingCom1023A1 || v4WaitingInit1037A1 || v4WaitingStatusA1 ||
    waitingA1After40 || waitingA1AfterCommand01Init50
  )
  {
    xmChannelNotBeforeMs = now + 10UL;
    return;
  }

  if (digitalRead(PIN_STB) != HIGH || digitalRead(PIN_R) != LOW)
  {
    xmChannelNotBeforeMs = now + 5UL;
    return;
  }

  updateXmChannelStatus61();
  bool ok = sendPacket(pktStatus61, STATUS61_LEN);
  bool armed = startAppListener();
  updateStatus61();

  if (ok && armed)
  {
    xmChannelTxCount++;
    xmChannelDirty = false;
    xmWaitingChannelA1 = true;
    xmChannelSentAtMs = millis();

#if BDK_XM_RUNTIME_VERBOSE
    Serial.print("XM CH TX #");
    Serial.print(xmChannelTxCount);
    Serial.print(" -> CH");
    if (xmChannel < 100) Serial.print('0');
    if (xmChannel < 10) Serial.print('0');
    Serial.print(xmChannel);
    Serial.print(" [7]=");
    hex2(xmStatus61Byte7);
    Serial.print(" [8]=");
    hex2(xmChannelHundreds(xmChannel));
    Serial.print(" [9]=");
    hex2(xmChannelLowBcd(xmChannel));
    Serial.print(" | XM");
    Serial.print(xmBandNumber);
    Serial.print(" P:");
    Serial.print(xmPresetNumber);
    Serial.print(" | META=");
    Serial.print(xmMetadataPage);
    Serial.print(" OFF=");
    Serial.print(xmScrollOffset);

    Serial.print(" \"");

    char window8[9];
    buildXmMetadataWindow(window8);
    Serial.print(window8);
    Serial.println("\"");
#endif
  }
  else
  {
    xmChannelTxFail++;
    xmChannelNotBeforeMs = millis() + 30UL;
  }
}




void processXmCommand30(const AppFrame &f)
{
  xmLastCommandChangedChannel = false;
  xmLastProcessedKey = 0xFF;

  if (f.length < 7) return;
  if (
    f.data[0] != 0x30 ||
    f.data[1] != 0x52 ||
    f.data[3] != DEVICE_ID8
  )
  {
    return;
  }

  const uint8_t key = f.data[6];

  xmLastProcessedKey = key;
  xmLastKey = key;
  xmLastKeyMs = millis();

  if (key == 0x00)
  {
    return;
  }

  if (key == 0x21)
  {
    if (xmChannel >= 255)
    {
      xmChannel = 1;
    }
    else
    {
      xmChannel++;
    }

    // CH000 nao e mais usado. Reinicia a visualizacao da faixa.
    xmMetadataPage = 0;
    xmScrollOffset = 0;
    xmNextScrollMs = millis() + XM_SCROLL_START_PAUSE_MS;

    xmLastCommandChangedChannel = true;
    xmChannelDirty = true;
    xmChannelNotBeforeMs = millis();

    if (bdkAvrcConnected())
    {
      bdkAvrcNext();
    }

    return;
  }

  if (key == 0x22)
  {
    if (xmChannel <= 1)
    {
      xmChannel = 255;
    }
    else
    {
      xmChannel--;
    }

    xmMetadataPage = 0;
    xmScrollOffset = 0;
    xmNextScrollMs = millis() + XM_SCROLL_START_PAUSE_MS;

    xmLastCommandChangedChannel = true;
    xmChannelDirty = true;
    xmChannelNotBeforeMs = millis();

    if (bdkAvrcConnected())
    {
      bdkAvrcPrevious();
    }

    return;
  }

  if (key == 0x20)
  {
    xmBandNumber++;
    if (xmBandNumber > 3)
    {
      xmBandNumber = 1;
    }

    xmStatus61Byte7 =
      (uint8_t)((xmBandNumber << 4) | (xmPresetNumber & 0x0F));

    xmBandChangeCount++;

    // Audio profile follows the actual XM band.
    bdkXmBandChanged(xmBandNumber);

    xmChannelDirty = true;
    xmChannelNotBeforeMs = millis();
    return;
  }

  if (key == XM_DISP_KEY)
  {
    xmMetadataPage++;
    if (xmMetadataPage >= XM_METADATA_PAGE_COUNT)
    {
      xmMetadataPage = 0;
    }

    xmDispMetadataChangeCount++;

    // Começa sempre pelo inicio da nova informação.
    xmScrollOffset = 0;
    xmNextScrollMs = millis() + XM_SCROLL_START_PAUSE_MS;

    xmChannelDirty = true;
    xmChannelNotBeforeMs = millis();
    return;
  }
}

void resetMetrics()
{
  validFrames = invalidFrames = 0;
  command00 = command01 = command10 = command30 = 0;
  command40 = command50 = command70 = commandA1 = commandOther = 0;
  response01 = response40 = 0;

  xmChannel = 1;
  xmBand = 1;
  xmLastKey = 0x00;
  xmLastKeyMs = 0;
  xmChannelDirty = false;
  xmChannelNotBeforeMs = 0;
  xmChannelTxCount = 0;
  xmChannelTxFail = 0;
  xmWaitingCom11A1 = false;
  xmWaitingChannelA1 = false;
  xmCom11SentAtMs = 0;
  xmChannelSentAtMs = 0;
  xmCom11TxCount = xmCom11AckCount = xmCom11TimeoutCount = 0;
  xmChannelAckCount = xmChannelAckTimeoutCount = 0;
  xmInitialPublishPending = true;
  xmInitialPublished = false;
  xmLastProcessedKey = 0x00;
  xmLastCommandChangedChannel = false;

  xmBandNumber = 1;
  xmPresetNumber = 3;
  xmStatus61Byte7 = 0x13;
  xmBandChangeCount = 0;

  bdkXmBandChanged(1);

  xmMetadataPage = 0;
  xmDispMetadataChangeCount = 0;

  xmScrollOffset = 0;
  xmNextScrollMs = 0;
  xmScrollUpdateCount = 0;

  xmLastValidFrameMs = 0;
  xmRecoveryArmAtMs = 0;
  xmRecoveryRequested = false;

  waitingA1After40 = false;
  a1After40Count = 0;
  waitingA1AfterCommand01Init50 = false;
  waitingA1AfterReady = false;
  readySentAfterCommand01 = false;
  readyAckAfterCommand01 = false;
  readySentAtMs = readyAckAtMs = 0;

  v4Mode = 0x00;
  v4OldMode = 0x00;
  v4Status = 0x01;
  v4InitRequested = false;
  v4Ready = false;
  v4WaitingCom1023A1 = false;
  v4WaitingInit1037A1 = false;
  v4WaitingStatusA1 = false;
  v4Com1023Sent = v4Com1023Ack = 0;
  v4Init1037Sent = v4Init1037Ack = 0;
  v4StatusSent = v4StatusAck = v4StatusTxFail = 0;
  v4NextStatusMs = 0;
  v4BranchF6 = v4Branch04 = v4Branch0506 = 0;
  v4BranchStart02 = v4BranchStop01 = v4Branch03 = 0;

  noInterrupts();
  queueWrite = queueRead = 0;
  queueOverflow = false;
  interrupts();
}


static void serviceXmExternalMetadata()
{
  uint32_t generation;

  portENTER_CRITICAL(&xmBtMetaMux);
  generation = xmBtMetadataGeneration;
  portEXIT_CRITICAL(&xmBtMetaMux);

  if (generation == xmBtHandledGeneration)
  {
    return;
  }

  xmBtHandledGeneration = generation;

  // V1.0.5:
  // AVRCP atualiza somente ARTISTA/TITULO/ALBUM.
  // O numero do canal XM fica exclusivamente sob controle do IP-BUS.
  // Assim, um TRACK_NUM=1/1 do telefone nao pode mais fazer
  // CH002 voltar indevidamente para CH001.
  xmMetadataPage = 0;
  xmScrollOffset = 0;
  xmNextScrollMs = millis() + XM_SCROLL_START_PAUSE_MS;

  xmChannelDirty = true;
  xmChannelNotBeforeMs = millis();
}



void serviceXmMetadataScroll()
{
  if (!v4Ready) return;

  const uint32_t now = millis();

  if (xmNextScrollMs == 0)
  {
    // Mantém os primeiros 8 caracteres parados por um instante,
    // depois começa a rolagem rápida.
    xmNextScrollMs = now + XM_SCROLL_START_PAUSE_MS;
    return;
  }

  if ((int32_t)(now - xmNextScrollMs) < 0)
  {
    return;
  }

  // Não disputa o barramento com transações que já aguardam A1.
  if (
    xmWaitingCom11A1 ||
    xmWaitingChannelA1 ||
    v4WaitingCom1023A1 ||
    v4WaitingInit1037A1 ||
    v4WaitingStatusA1 ||
    waitingA1After40 ||
    waitingA1AfterCommand01Init50 ||
    waitingA1AfterReady
  )
  {
    xmNextScrollMs = now + 50;
    return;
  }

  char metadataSource[128];
  copyXmMetadataSource(metadataSource, sizeof(metadataSource));

  const size_t len =
    strlen(metadataSource);

  const uint8_t gap = 3;
  const size_t cycleLen = len + gap;

  xmScrollOffset++;

  bool wrapped = false;

  if (xmScrollOffset >= cycleLen)
  {
    xmScrollOffset = 0;
    wrapped = true;
  }

  xmScrollUpdateCount++;

  // Reutiliza o mesmo caminho já validado de STATUS61 de canal/texto.
  xmChannelDirty = true;
  xmChannelNotBeforeMs = now;

  xmNextScrollMs =
    now + (wrapped ? XM_SCROLL_START_PAUSE_MS : XM_SCROLL_PERIOD_MS);
}



bool serviceXmRecoverySupervisor()
{
  if (xmRecoveryRequested)
  {
    return true;
  }

  const uint32_t now = millis();

  // Não julga a sessão nos primeiros segundos após entrar no runtime.
  if ((int32_t)(now - xmRecoveryArmAtMs) < 0)
  {
    return false;
  }

  if ((uint32_t)(now - xmLastValidFrameMs) < XM_RECOVERY_SILENCE_MS)
  {
    return false;
  }

  xmRecoveryRequested = true;
  xmRecoveryCount++;

  Serial.println();
  Serial.println("==================================================");
  Serial.println("*** XM V3.26 RECOVERY: SESSAO IP-BUS PERDIDA ***");
  Serial.print("Sem frame valido do HU por ");
  Serial.print((uint32_t)(now - xmLastValidFrameMs));
  Serial.println(" ms.");
  Serial.println("Encerrando runtime e rearmando captura de boot...");
  Serial.println("Nenhum handshake alternativo sera inventado.");
  Serial.println("==================================================");
  Serial.println();

  return true;
}


void runApplicationSession()
{
  resetMetrics();
  sessionStart64 = (uint64_t)esp_timer_get_time();

  xmLastValidFrameMs = millis();
  xmRecoveryArmAtMs = millis() + XM_RECOVERY_ARM_DELAY_MS;
  xmRecoveryRequested = false;

  Serial.println("BDK XM V1.0.7.1 | XM V3.26 + LDAC METADATA + BAND DSP");
  Serial.println("XM1=DSP-1 | XM2=DSP2 | XM3=HIFI | ARTIST - TITLE / 650 ms.");
  Serial.println("IP-BUS PRIORITY: task=8 / ISR wake / runtime log quiet.");
  Serial.println("CHANNEL: IP-BUS only / CH001..CH255 / CH000 disabled.");
  Serial.println("BOOT: RX early + late-boot Multi-CD strategy.");
  Serial.println("Recovery: 7 s sem frame valido -> rearma boot automaticamente.");
  if (!startAppListener()) {
    Serial.println("*** FALHA LISTENER XM ***");
    return;
  }
  while (true) {
    AppFrame f;
    while (getQueuedFrame(f)) processAppFrame(f);

    serviceXmExternalMetadata();
    serviceXmChannelUpdate();
    serviceV4Status();
    serviceXmMetadataScroll();

    // Se a comunicação desaparecer por tempo suficiente, sai da sessão.
    // executeXm() chamará prepareBootCapture() e voltará a esperar o
    // broadcast normal do rádio.
    if (serviceXmRecoverySupervisor())
    {
      break;
    }

    // Task de alta prioridade, mas bloqueada quando o barramento esta
    // ocioso. Qualquer frame completo acorda imediatamente pela ISR.
    // O timeout de 1 tick mantem STATUS/recovery/metadata funcionando.
    ulTaskNotifyTake(pdTRUE, 1);
  }
}

// PREPARE BOOT - RX GPIO DIRETO (FIX5)

bool prepareBootCapture()
{
  stopAppListener();
  g_bdkXmEarlyBootArmed = false;

  pinMatrixOutDetach(
    PIN_TX,
    false,
    false
  );

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
  digitalWrite(PIN_OE, HIGH);
  pinMode(PIN_R, INPUT);
  delay(10);
  noInterrupts();
  bootRawActive = false;
  bootRawReady = false;
  bootRawBits = 0;
  bootRawStartH = 0;
  bootRawEdgeCount = 0;
  bootRawStartCount = 0;
  bootRaw124Count = 0;
  bootRawLastEdgeUs = (uint32_t)esp_timer_get_time();
  bootRawLastLevel = digitalRead(PIN_R);
  interrupts();
  attachInterrupt(digitalPinToInterrupt(PIN_R), bootGPIOISR, CHANGE);
  isrAttached = true;
  lastWaitMessage = millis();
  return true;
}

// ============================================================
// BDK XM V1.0.7.1 - C++ DECLARATION ORDER FIX
//
// ESP-IDF compiles this file as normal C++.
// These globals must be declared before executeXm(), which uses them.
// No protocol/timing/boot behavior changed in this fix.
// ============================================================
static volatile bool g_bdkXmReady = false;
static volatile bool g_bdkXmLinked = false;
static uint32_t g_xmNextForcedBootMs = 0;
static uint8_t g_xmForcedBootAttempts = 0;

void executeXm()
{
  runInProgress = true;
  refreshIdentityPackets();

  if (!runInitialSequence())
  {
    Serial.println("*** BOOT IP-BUS FALHOU; REARMANDO ***");
    g_bdkXmLinked = false;
    prepareBootCapture();
    runInProgress = false;
    return;
  }

  // O DEH-P650 confirmou a sequencia inicial XM.
  g_bdkXmLinked = true;

  runApplicationSession();

  g_bdkXmLinked = false;

  if (xmRecoveryRequested)
  {
    Serial.println("*** RECOVERY: CAPTURA DE BOOT REARMADA ***");
  }

  prepareBootCapture();

  // Tambem rearma late-boot apos perda/reentrada.
  g_xmForcedBootAttempts = 0;
  g_xmNextForcedBootMs = millis() + XM_FIRST_FORCED_BOOT_MS;

  runInProgress = false;
}

// ============================================================
// NATIVE ESP-IDF TASK / PUBLIC BRIDGE
// ============================================================


static void asciiMetadata(
  const uint8_t *src,
  size_t srcLen,
  char *dst,
  size_t dstSize)
{
  if (!dst || dstSize == 0) return;

  size_t out = 0;

  if (src)
  {
    for (size_t i = 0; i < srcLen && out + 1 < dstSize; ++i)
    {
      uint8_t c = src[i];

      if (c >= 0x20 && c <= 0x7E)
      {
        dst[out++] = (char)c;
        continue;
      }

      // Common Portuguese/Latin UTF-8 accents -> ASCII for the Pioneer.
      if (c == 0xC3 && i + 1 < srcLen)
      {
        uint8_t n = src[++i];
        char mapped = 0;

        if ((n >= 0x80 && n <= 0x85) || (n >= 0xA0 && n <= 0xA5))
          mapped = (n < 0xA0) ? 'A' : 'a';
        else if (n == 0x87 || n == 0xA7)
          mapped = (n == 0x87) ? 'C' : 'c';
        else if ((n >= 0x88 && n <= 0x8B) || (n >= 0xA8 && n <= 0xAB))
          mapped = (n < 0xA0) ? 'E' : 'e';
        else if ((n >= 0x8C && n <= 0x8F) || (n >= 0xAC && n <= 0xAF))
          mapped = (n < 0xA0) ? 'I' : 'i';
        else if (n == 0x91 || n == 0xB1)
          mapped = (n == 0x91) ? 'N' : 'n';
        else if ((n >= 0x92 && n <= 0x96) || (n >= 0xB2 && n <= 0xB6))
          mapped = (n < 0xA0) ? 'O' : 'o';
        else if ((n >= 0x99 && n <= 0x9C) || (n >= 0xB9 && n <= 0xBC))
          mapped = (n < 0xA0) ? 'U' : 'u';

        if (mapped) dst[out++] = mapped;
      }
    }
  }

  while (out > 0 && dst[out - 1] == ' ') --out;
  dst[out] = 0;
}

static uint32_t parseTrackNumber(
  const uint8_t *data,
  size_t len)
{
  uint32_t value = 0;
  bool any = false;

  for (size_t i = 0; i < len; ++i)
  {
    uint8_t c = data[i];
    if (c < '0' || c > '9') break;
    any = true;
    value = value * 10U + (uint32_t)(c - '0');
    if (value > 9999U) break;
  }

  return any ? value : 0;
}

static void ipbusXmSetupInternal()
{
  Serial.begin(115200);

  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
  pinMode(PIN_R, INPUT);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_STB, INPUT);

  refreshIdentityPackets();

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" BDK XM V1.0.7.1 - XM V3.26 NATIVE ESP-IDF");
  Serial.println(" LINK 0x11E / APP 0x1E");
  Serial.println(" XM1 DSP-1 | XM2 DSP2 | XM3 HIFI");
  Serial.println("==================================================");

  if (!selfTest())
  {
    Serial.println("*** SELF TEST CHECKSUM FALHOU ***");
    return;
  }

  if (!initAckTimer())
  {
    Serial.println("*** ERRO GPTIMER ACK ***");
    return;
  }

  if (!initTX())
  {
    Serial.println("*** ERRO RMT TX ***");
    return;
  }

  if (!g_bdkXmEarlyBootArmed)
  {
    if (!prepareBootCapture())
    {
      Serial.println("*** ERRO CAPTURA BOOT ***");
      return;
    }
  }
  else
  {
    Serial.println("RX IP-BUS ja armado no inicio do app_main; captura preservada.");
  }

  bdkXmBandChanged(1);

  g_bdkXmLinked = false;
  g_xmForcedBootAttempts = 0;
  g_xmNextForcedBootMs = millis() + XM_FIRST_FORCED_BOOT_MS;

  g_bdkXmReady = true;

  Serial.println("Aguardando broadcast IP-BUS...");
  Serial.println("Late-boot XM: 150 ms / 600 ms / 8 tentativas.");
}

static void ipbusXmLoopInternal()
{
  if (runInProgress)
  {
    delay(1);
    return;
  }

  if (bootRawReady)
  {
    XmBootFrame boot;

    noInterrupts();

    uint16_t rawBits = bootRawBits;
    if (rawBits > MAX_BITS) rawBits = MAX_BITS;

    boot.found = true;
    boot.bits = rawBits;
    boot.startH = bootRawStartH;
    boot.startL = 0;

    for (uint16_t i = 0; i < rawBits; ++i)
    {
      boot.bit[i] = bootRawBit[i];
    }

    bootRawReady = false;

    interrupts();

    if (bootBroadcast(boot))
    {
      stopAppListener();
      g_xmForcedBootAttempts = 0;
      Serial.println("BOOT IP-BUS XM OK");
      executeXm();
      return;
    }
  }

  // CAMINHO 2: late boot, como no Multi-CD Operational V2.13.
  // Se o broadcast inicial foi perdido, tenta a sequencia XM ja validada
  // sem inventar bytes. sendPacket() ainda exige STB/R seguros.
  if (
    !g_bdkXmLinked &&
    g_xmForcedBootAttempts < XM_MAX_FORCED_BOOT_ATTEMPTS &&
    (int32_t)(millis() - g_xmNextForcedBootMs) >= 0
  )
  {
    g_xmForcedBootAttempts++;

    Serial.print("*** XM LATE-BOOT TENTATIVA ");
    Serial.print(g_xmForcedBootAttempts);
    Serial.print("/");
    Serial.print(XM_MAX_FORCED_BOOT_ATTEMPTS);
    Serial.println(" -> sequencia XM validada ***");

    executeXm();

    if (!g_bdkXmLinked)
    {
      g_xmNextForcedBootMs = millis() + XM_FORCED_BOOT_RETRY_MS;

      if (g_xmForcedBootAttempts >= XM_MAX_FORCED_BOOT_ATTEMPTS)
      {
        Serial.println("*** XM LATE-BOOT ESGOTADO; aguardando broadcast periodico ***");
      }
    }

    return;
  }

  if (millis() - lastWaitMessage >= 5000UL)
  {
    lastWaitMessage = millis();
    Serial.println("Aguardando broadcast IP-BUS XM...");
  }

  // Dorme ate o broadcast completar ou por no maximo 1 tick.
  // bootGPIOISR() acorda esta task assim que os 124 bits terminam.
  ulTaskNotifyTake(pdTRUE, 1);
}


extern "C" bool ipbusXmEarlyArm(void)
{
  // Caminho minimo: somente GPIO/ISR de RX.
  // /OE fica HIGH para manter a saida HCT244 em Hi-Z.
  if (isrAttached)
  {
    g_bdkXmEarlyBootArmed = true;
    return true;
  }

  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);

  pinMode(PIN_R, INPUT);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_STB, INPUT);

  noInterrupts();
  bootRawActive = false;
  bootRawReady = false;
  bootRawBits = 0;
  bootRawStartH = 0;
  bootRawEdgeCount = 0;
  bootRawStartCount = 0;
  bootRaw124Count = 0;
  bootRawLastEdgeUs = (uint32_t)esp_timer_get_time();
  bootRawLastLevel = digitalRead(PIN_R);
  interrupts();

  attachInterrupt(digitalPinToInterrupt(PIN_R), bootGPIOISR, CHANGE);
  isrAttached = true;
  g_bdkXmEarlyBootArmed = true;

  return true;
}

extern "C" void ipbusXmEarlyDisarm(void)
{
  if (isrAttached)
  {
    detachInterrupt(digitalPinToInterrupt(PIN_R));
    isrAttached = false;
  }

  g_bdkXmEarlyBootArmed = false;

  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
}

static void bdkXmTask(void *arg)
{
  (void)arg;

  ipbusXmSetupInternal();

  if (!g_bdkXmReady)
  {
    Serial.println("*** BDK XM TASK NAO FICOU PRONTA ***");
    g_bdkXmTaskHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  for (;;)
  {
    ipbusXmLoopInternal();
  }
}

extern "C" bool ipbusXmStart(void)
{
  if (g_bdkXmTaskHandle != nullptr)
  {
    return true;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
    bdkXmTask,
    "ipbus_xm_v326",
    12288,
    nullptr,
    BDK_XM_TASK_PRIORITY,
    &g_bdkXmTaskHandle,
    0
  );

  if (ok != pdPASS)
  {
    g_bdkXmTaskHandle = nullptr;
    return false;
  }

  uint32_t started = millis();

  while (!g_bdkXmReady && (millis() - started) < 1500UL)
  {
    delay(5);
  }

  return g_bdkXmReady;
}

extern "C" bool ipbusXmIsLinked(void)
{
  return g_bdkXmLinked;
}

extern "C" bool ipbusXmWaitForInitialLink(uint32_t timeoutMs)
{
  const uint32_t start = millis();

  while (!g_bdkXmLinked)
  {
    if ((uint32_t)(millis() - start) >= timeoutMs)
    {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return true;
}

extern "C" bool ipbusXmReady(void)
{
  return g_bdkXmReady;
}

extern "C" uint8_t ipbusXmCurrentBand(void)
{
  return xmBandNumber;
}

extern "C" void ipbusXmAvrcConnection(bool connected)
{
  portENTER_CRITICAL(&xmBtMetaMux);
  xmBtConnected = connected;
  xmBtMetadataGeneration++;
  portEXIT_CRITICAL(&xmBtMetaMux);
}

extern "C" void ipbusXmAvrcMetadata(
  uint8_t attr,
  const uint8_t *data,
  size_t len)
{
  if (!data) return;

  char temp[128];
  asciiMetadata(data, len, temp, sizeof(temp));

  bool metadataChanged = false;
  portENTER_CRITICAL(&xmBtMetaMux);

  if (attr == ESP_AVRC_MD_ATTR_ARTIST)
  {
    strncpy(xmBtArtist, temp[0] ? temp : "NO ARTIST", sizeof(xmBtArtist) - 1);
    xmBtArtist[sizeof(xmBtArtist) - 1] = 0;
    metadataChanged = true;
  }
  else if (attr == ESP_AVRC_MD_ATTR_TITLE)
  {
    strncpy(xmBtTitle, temp[0] ? temp : "NO TITLE", sizeof(xmBtTitle) - 1);
    xmBtTitle[sizeof(xmBtTitle) - 1] = 0;
    metadataChanged = true;
  }
  else if (attr == ESP_AVRC_MD_ATTR_ALBUM)
  {
    strncpy(xmBtAlbum, temp[0] ? temp : "NO ALBUM", sizeof(xmBtAlbum) - 1);
    xmBtAlbum[sizeof(xmBtAlbum) - 1] = 0;
    metadataChanged = true;
  }

  if (metadataChanged)
  {
    xmBtMetadataGeneration++;
  }

  portEXIT_CRITICAL(&xmBtMetaMux);

  // V1.0.5:
  // ESP_AVRC_MD_ATTR_TRACK_NUM e deliberadamente ignorado para o canal XM.
  // Alguns celulares/players reportam "1/1" a cada nova faixa.
  // Esse valor nao representa o canal XM do Pioneer.
  if (attr == ESP_AVRC_MD_ATTR_TRACK_NUM)
  {
    return;
  }
}
