#include <Arduino.h>
#include <Wire.h>

/*
  立创·天空星 STM32F407VGT6 + ST7796(8080-16bit) + FT6336U
  ------------------------------------------------------------
  按“FSMC 标准接线”实现：

  LCD 数据线
    D0->PD14 D1->PD15 D2->PD0  D3->PD1
    D4->PE7  D5->PE8  D6->PE9  D7->PE10
    D8->PE11 D9->PE12 D10->PE13 D11->PE14
    D12->PE15 D13->PD8 D14->PD9 D15->PD10

  LCD 控制线
    RD->PD4(NOe) WR->PD5(NWE) CS->PD7(NE1) RS/DC->PD11(A16)
    RST->PA5, BL->PB13(默认高电平点亮)

  触摸 FT6336U
    SDA->PB11, SCL->PB10, I2C地址 0x38

  说明：
  - 不使用 HAL SRAM 头文件，不使用 HAL_SRAM_Init。
  - 直接用寄存器初始化 FSMC，兼容 Arduino STM32 环境。
*/

// ======================== 屏幕与触摸参数 ========================
static const uint16_t LCD_W = 320;
static const uint16_t LCD_H = 480;

static const uint8_t PIN_LCD_RST = PA5;
static const uint8_t PIN_LCD_BL = PB13;

static const uint8_t PIN_TP_SDA = PB11;
static const uint8_t PIN_TP_SCL = PB10;
static const uint8_t FT6336_ADDR = 0x38;

#ifndef TOUCH_MAP_MODE
#define TOUCH_MAP_MODE 0 // 0:直通 1:CW90 2:180 3:CCW90
#endif

// NE1 + A16 地址映射
#define LCD_REG16 (*((volatile uint16_t *)0x60000000U))
#define LCD_RAM16 (*((volatile uint16_t *)0x60020000U))

// ======================== 游戏参数（铺满全屏） ========================
static const uint8_t COLS = 8;
static const uint8_t ROWS = 11;
static const uint8_t TYPES = 6;
static const uint16_t CELL = 40;
static const uint16_t BOARD_Y = CELL; // 顶部留一排格子高度给积分

static uint8_t board[ROWS][COLS];
static bool selected = false;
static int8_t selR = -1;
static int8_t selC = -1;
static uint32_t score = 0;

// ======================== FSMC 初始化 ========================
static inline void gpioSetAF12(GPIO_TypeDef *port, uint8_t pin)
{
  port->MODER = (port->MODER & ~(0x3UL << (pin * 2U))) | (0x2UL << (pin * 2U));
  port->OTYPER &= ~(1UL << pin);
  port->OSPEEDR = (port->OSPEEDR & ~(0x3UL << (pin * 2U))) | (0x3UL << (pin * 2U));
  port->PUPDR &= ~(0x3UL << (pin * 2U));

  volatile uint32_t *afr = (pin < 8U) ? &port->AFR[0] : &port->AFR[1];
  uint32_t shift = (pin & 0x7U) * 4U;
  *afr = (*afr & ~(0xFUL << shift)) | (0xCUL << shift); // AF12
}

static void fsmcInit()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
  RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
  __DSB();

  // GPIOD: D0,D1,D2,D3,NOE,NWE,NE1,D13,D14,D15,A16
  gpioSetAF12(GPIOD, 14);
  gpioSetAF12(GPIOD, 15);
  gpioSetAF12(GPIOD, 0);
  gpioSetAF12(GPIOD, 1);
  gpioSetAF12(GPIOD, 4);
  gpioSetAF12(GPIOD, 5);
  gpioSetAF12(GPIOD, 7);
  gpioSetAF12(GPIOD, 8);
  gpioSetAF12(GPIOD, 9);
  gpioSetAF12(GPIOD, 10);
  gpioSetAF12(GPIOD, 11);

  // GPIOE: D4..D12
  gpioSetAF12(GPIOE, 7);
  gpioSetAF12(GPIOE, 8);
  gpioSetAF12(GPIOE, 9);
  gpioSetAF12(GPIOE, 10);
  gpioSetAF12(GPIOE, 11);
  gpioSetAF12(GPIOE, 12);
  gpioSetAF12(GPIOE, 13);
  gpioSetAF12(GPIOE, 14);
  gpioSetAF12(GPIOE, 15);

  // BCR1: MBKEN=1, MUXEN=0, MTYP=SRAM, MWID=16bit, WREN=1
  FSMC_Bank1->BTCR[0] =
      FSMC_BCR1_MBKEN |
      FSMC_BCR1_MWID_0 |
      FSMC_BCR1_WREN;

  // BTR1 读时序：ADDSET=3 DATAST=8 BUSTURN=1
  FSMC_Bank1->BTCR[1] =
      (3U << FSMC_BTR1_ADDSET_Pos) |
      (1U << FSMC_BTR1_BUSTURN_Pos) |
      (8U << FSMC_BTR1_DATAST_Pos);

  // BWTR1 写时序：ADDSET=1 DATAST=5
  FSMC_Bank1E->BWTR[0] =
      (1U << FSMC_BWTR1_ADDSET_Pos) |
      (5U << FSMC_BWTR1_DATAST_Pos);

  // 使能扩展写时序 EXTMOD
  FSMC_Bank1->BTCR[0] |= FSMC_BCR1_EXTMOD;
}

// ======================== LCD 基础函数 ========================
static inline void lcdWriteCmd(uint16_t cmd) { LCD_REG16 = cmd; }
static inline void lcdWriteData(uint16_t data) { LCD_RAM16 = data; }

static void lcdWriteCmdData(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  lcdWriteCmd(cmd);
  while (len--)
  {
    lcdWriteData(*data++);
  }
}

static void lcdSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  lcdWriteCmd(0x2A);
  lcdWriteData(x0 >> 8);
  lcdWriteData(x0 & 0xFF);
  lcdWriteData(x1 >> 8);
  lcdWriteData(x1 & 0xFF);

  lcdWriteCmd(0x2B);
  lcdWriteData(y0 >> 8);
  lcdWriteData(y0 & 0xFF);
  lcdWriteData(y1 >> 8);
  lcdWriteData(y1 & 0xFF);

  lcdWriteCmd(0x2C);
}

static void lcdPushColor(uint16_t color, uint32_t count)
{
  while (count--)
  {
    lcdWriteData(color);
  }
}

static void lcdFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if (x >= LCD_W || y >= LCD_H || w == 0 || h == 0)
    return;

  if (x + w > LCD_W)
    w = LCD_W - x;
  if (y + h > LCD_H)
    h = LCD_H - y;

  lcdSetAddrWindow(x, y, x + w - 1, y + h - 1);
  lcdPushColor(color, (uint32_t)w * h);
}

static void lcdDrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if (w < 2 || h < 2)
    return;
  lcdFillRect(x, y, w, 1, color);
  lcdFillRect(x, y + h - 1, w, 1, color);
  lcdFillRect(x, y, 1, h, color);
  lcdFillRect(x + w - 1, y, 1, h, color);
}

static void lcdReset()
{
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(5);
  digitalWrite(PIN_LCD_RST, LOW);
  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(120);
}

static void lcdInitST7796()
{
  lcdReset();

  lcdWriteCmd(0x01); // SWRESET
  delay(120);

  lcdWriteCmd(0x11); // SLPOUT
  delay(120);

  uint8_t pixfmt[] = {0x55}; // 16bit
  lcdWriteCmdData(0x3A, pixfmt, 1);

  uint8_t madctl[] = {0x48}; // 320x480 竖屏
  lcdWriteCmdData(0x36, madctl, 1);

  lcdWriteCmd(0x21); // INVON
  delay(10);
  lcdWriteCmd(0x29); // DISPON
  delay(20);
}

static void lcdDrawStartupBars()
{
  const uint16_t bars[8] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0x07FF, 0xF81F, 0xFFFF, 0x0000};
  const uint16_t h = LCD_H / 8;
  for (uint8_t i = 0; i < 8; ++i)
  {
    lcdFillRect(0, i * h, LCD_W, h, bars[i]);
  }
}

// ======================== 触摸 ========================
static bool ft6336ReadRaw(uint16_t &x, uint16_t &y)
{
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0)
    return false;

  Wire.requestFrom((int)FT6336_ADDR, 5);
  if (Wire.available() < 5)
    return false;

  uint8_t td = Wire.read() & 0x0F;
  uint8_t xh = Wire.read();
  uint8_t xl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t yl = Wire.read();

  if (td == 0)
    return false;

  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
  return true;
}

static void touchToScreen(uint16_t rawX, uint16_t rawY, uint16_t &sx, uint16_t &sy)
{
  switch (TOUCH_MAP_MODE)
  {
  case 1:
    sx = LCD_W - 1 - rawY;
    sy = rawX;
    break;
  case 2:
    sx = LCD_W - 1 - rawX;
    sy = LCD_H - 1 - rawY;
    break;
  case 3:
    sx = rawY;
    sy = LCD_H - 1 - rawX;
    break;
  default:
    sx = rawX;
    sy = rawY;
    break;
  }

  if (sx >= LCD_W)
    sx = LCD_W - 1;
  if (sy >= LCD_H)
    sy = LCD_H - 1;
}

// ======================== 游戏逻辑 ========================
static uint16_t colorOf(uint8_t t)
{
  switch (t)
  {
  case 0:
    return 0xF800;
  case 1:
    return 0x07E0;
  case 2:
    return 0x001F;
  case 3:
    return 0xFFE0;
  case 4:
    return 0x07FF;
  default:
    return 0xF81F;
  }
}

static void drawDigit3x5(uint16_t x, uint16_t y, uint8_t d, uint16_t fg, uint16_t bg, uint8_t scale)
{
  static const uint8_t font[10][5] = {
      {0b111, 0b101, 0b101, 0b101, 0b111},
      {0b010, 0b110, 0b010, 0b010, 0b111},
      {0b111, 0b001, 0b111, 0b100, 0b111},
      {0b111, 0b001, 0b111, 0b001, 0b111},
      {0b101, 0b101, 0b111, 0b001, 0b001},
      {0b111, 0b100, 0b111, 0b001, 0b111},
      {0b111, 0b100, 0b111, 0b101, 0b111},
      {0b111, 0b001, 0b001, 0b001, 0b001},
      {0b111, 0b101, 0b111, 0b101, 0b111},
      {0b111, 0b101, 0b111, 0b001, 0b111},
  };

  if (d > 9)
    d = 0;

  for (uint8_t row = 0; row < 5; ++row)
  {
    for (uint8_t col = 0; col < 3; ++col)
    {
      bool on = (font[d][row] >> (2 - col)) & 0x01;
      lcdFillRect(x + col * scale, y + row * scale, scale, scale, on ? fg : bg);
    }
  }
}

static void drawScoreHud()
{
  // 占用一整排格子高度（40像素）
  lcdFillRect(0, 0, LCD_W, BOARD_Y, 0x0000);
  lcdDrawRect(0, 0, LCD_W, BOARD_Y, 0xFFFF);
  lcdFillRect(0, BOARD_Y - 2, LCD_W, 2, 0x7BEF);

  // 左侧色块做标题标记
  lcdFillRect(8, 8, 24, 24, 0xFFE0);
  lcdDrawRect(8, 8, 24, 24, 0xFFFF);

  uint32_t v = score;
  uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
  for (int8_t i = 5; i >= 0; --i)
  {
    digits[i] = v % 10;
    v /= 10;
  }

  const uint8_t scale = 5;
  uint16_t x = 48;
  for (uint8_t i = 0; i < 6; ++i)
  {
    drawDigit3x5(x, 7, digits[i], 0xFFFF, 0x0000, scale);
    x += (3 * scale) + 6;
  }
}

static bool causesMatchAt(uint8_t r, uint8_t c)
{
  uint8_t t = board[r][c];

  uint8_t l = 0;
  for (int8_t cc = c - 1; cc >= 0 && board[r][cc] == t; --cc)
    ++l;
  uint8_t rr = 0;
  for (uint8_t cc = c + 1; cc < COLS && board[r][cc] == t; ++cc)
    ++rr;
  if (l + rr + 1 >= 3)
    return true;

  uint8_t u = 0;
  for (int8_t rr2 = r - 1; rr2 >= 0 && board[rr2][c] == t; --rr2)
    ++u;
  uint8_t d = 0;
  for (uint8_t rr2 = r + 1; rr2 < ROWS && board[rr2][c] == t; ++rr2)
    ++d;
  return u + d + 1 >= 3;
}

static void initBoard()
{
  for (uint8_t r = 0; r < ROWS; ++r)
    for (uint8_t c = 0; c < COLS; ++c)
      do
      {
        board[r][c] = random(TYPES);
      } while (causesMatchAt(r, c));
}

static void drawCell(uint8_t r, uint8_t c, bool hl)
{
  uint16_t x = c * CELL;
  uint16_t y = BOARD_Y + r * CELL;
  lcdFillRect(x + 2, y + 2, CELL - 4, CELL - 4, colorOf(board[r][c]));
  lcdDrawRect(x + 1, y + 1, CELL - 2, CELL - 2, 0x0000);

  if (hl)
  {
    lcdDrawRect(x, y, CELL, CELL, 0xFFFF);
  }
}

static void drawBoard()
{
  for (uint8_t r = 0; r < ROWS; ++r)
    for (uint8_t c = 0; c < COLS; ++c)
      drawCell(r, c, selected && r == (uint8_t)selR && c == (uint8_t)selC);

  drawScoreHud();
}

static bool findMatches(bool mark[ROWS][COLS])
{
  bool found = false;

  for (uint8_t r = 0; r < ROWS; ++r)
    for (uint8_t c = 0; c < COLS; ++c)
      mark[r][c] = false;

  for (uint8_t r = 0; r < ROWS; ++r)
  {
    uint8_t s = 0;
    while (s < COLS)
    {
      uint8_t e = s + 1;
      while (e < COLS && board[r][e] == board[r][s])
        ++e;
      if (e - s >= 3)
      {
        found = true;
        for (uint8_t c = s; c < e; ++c)
          mark[r][c] = true;
      }
      s = e;
    }
  }

  for (uint8_t c = 0; c < COLS; ++c)
  {
    uint8_t s = 0;
    while (s < ROWS)
    {
      uint8_t e = s + 1;
      while (e < ROWS && board[e][c] == board[s][c])
        ++e;
      if (e - s >= 3)
      {
        found = true;
        for (uint8_t r = s; r < e; ++r)
          mark[r][c] = true;
      }
      s = e;
    }
  }

  return found;
}

static uint16_t clearDrop(bool mark[ROWS][COLS])
{
  uint16_t n = 0;
  for (uint8_t c = 0; c < COLS; ++c)
  {
    int8_t wr = ROWS - 1;
    for (int8_t r = ROWS - 1; r >= 0; --r)
    {
      if (!mark[r][c])
      {
        board[wr][c] = board[r][c];
        --wr;
      }
      else
      {
        ++n;
      }
    }

    while (wr >= 0)
    {
      board[wr][c] = random(TYPES);
      --wr;
    }
  }
  return n;
}

static void resolveMatches()
{
  bool mark[ROWS][COLS];
  while (findMatches(mark))
  {
    uint16_t n = clearDrop(mark);
    score += (uint32_t)n * 10;
    drawBoard();
    delay(40);
  }
}

static bool canSwapMakeMatch(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2)
{
  uint8_t t = board[r1][c1];
  board[r1][c1] = board[r2][c2];
  board[r2][c2] = t;
  bool ok = causesMatchAt(r1, c1) || causesMatchAt(r2, c2);
  t = board[r1][c1];
  board[r1][c1] = board[r2][c2];
  board[r2][c2] = t;
  return ok;
}

static bool hasPossibleMoves()
{
  for (uint8_t r = 0; r < ROWS; ++r)
    for (uint8_t c = 0; c < COLS; ++c)
    {
      if (c + 1 < COLS && canSwapMakeMatch(r, c, r, c + 1))
        return true;
      if (r + 1 < ROWS && canSwapMakeMatch(r, c, r + 1, c))
        return true;
    }
  return false;
}

static void shuffleBoard()
{
  for (uint8_t i = 0; i < 120; ++i)
  {
    uint8_t r1 = random(ROWS), c1 = random(COLS);
    uint8_t r2 = random(ROWS), c2 = random(COLS);
    uint8_t t = board[r1][c1];
    board[r1][c1] = board[r2][c2];
    board[r2][c2] = t;
  }
  resolveMatches();
}

static bool hitCell(uint16_t x, uint16_t y, uint8_t &r, uint8_t &c)
{
  if (x >= LCD_W || y < BOARD_Y || y >= LCD_H)
    return false;

  c = x / CELL;
  r = (y - BOARD_Y) / CELL;
  return (r < ROWS && c < COLS);
}

static void processTap(uint16_t sx, uint16_t sy)
{
  uint8_t r, c;
  if (!hitCell(sx, sy, r, c))
  {
    selected = false;
    drawBoard();
    return;
  }

  if (!selected)
  {
    selected = true;
    selR = r;
    selC = c;
    drawBoard();
    return;
  }

  int8_t dr = abs((int)r - selR);
  int8_t dc = abs((int)c - selC);
  if (dr + dc != 1)
  {
    selR = r;
    selC = c;
    drawBoard();
    return;
  }

  uint8_t t = board[selR][selC];
  board[selR][selC] = board[r][c];
  board[r][c] = t;

  if (causesMatchAt(selR, selC) || causesMatchAt(r, c))
  {
    selected = false;
    drawBoard();
    resolveMatches();

    if (!hasPossibleMoves())
    {
      shuffleBoard();
      drawBoard();
    }
  }
  else
  {
    t = board[selR][selC];
    board[selR][selC] = board[r][c];
    board[r][c] = t;
    selected = false;
    drawBoard();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("ST7796 FSMC Match-3 start");

  pinMode(PIN_LCD_RST, OUTPUT);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  Wire.setSDA(PIN_TP_SDA);
  Wire.setSCL(PIN_TP_SCL);
  Wire.begin();
  Wire.setClock(400000);

  randomSeed(analogRead(PA8) + micros());
  score = 0;

  fsmcInit();
  lcdInitST7796();
  lcdDrawStartupBars();
  delay(250);

  initBoard();
  resolveMatches();
  drawBoard();

  if (!hasPossibleMoves())
  {
    shuffleBoard();
    drawBoard();
  }

  Serial.println("Ready");
}

void loop()
{
  static bool down = false;

  uint16_t rx, ry;
  bool touched = ft6336ReadRaw(rx, ry);

  if (touched && !down)
  {
    uint16_t sx, sy;
    touchToScreen(rx, ry, sx, sy);
    processTap(sx, sy);
  }

  down = touched;
  delay(8);
}
