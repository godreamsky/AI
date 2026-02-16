#include <Arduino.h>
#include <Wire.h>

/*
  立创·天空星 STM32F407VGT6 + 4" IPS (ST7796, i8080 16-bit) + FT6336U
  --------------------------------------------------------------------
  基于“已验证触控测试程序”同款底层驱动实现的消消乐小游戏。

  说明：
  1) 请按你的实际连线修改引脚定义。
  2) 本代码保持 bit-bang 8080 16-bit 写总线方式，便于和测试程序一致。
  3) 若触控坐标方向不一致，调整 touchToScreen() 中映射。
*/

// ---------------------- LCD i8080 16-bit 引脚（按需修改） ----------------------
static const uint8_t LCD_D[16] = {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,       // D0..D7
    PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15  // D8..D15
};

static const uint8_t PIN_LCD_CS = PC0;
static const uint8_t PIN_LCD_RS = PC1; // RS/DC: 0=command, 1=data
static const uint8_t PIN_LCD_WR = PC2;
static const uint8_t PIN_LCD_RD = PC3;
static const uint8_t PIN_LCD_RST = PC4;
static const uint8_t PIN_LCD_BL = PC5; // 背光控制（高电平点亮）

// --------------------------- FT6336U I2C 引脚（按需修改） -----------------------
static const uint8_t PIN_TP_SDA = PB7;
static const uint8_t PIN_TP_SCL = PB6;
static const uint8_t FT6336_ADDR = 0x38;

// 屏幕分辨率（竖屏 320x480）
static const uint16_t LCD_W = 320;
static const uint16_t LCD_H = 480;

// ------------------------------- 游戏参数 -------------------------------------
static const uint8_t ROWS = 8;
static const uint8_t COLS = 8;
static const uint8_t TYPES = 6;

static const uint16_t CELL = 36;
static const uint16_t BOARD_X = 16;
static const uint16_t BOARD_Y = 72;

// HUD 区域
static const uint16_t HUD_H = 56;

static uint8_t board[ROWS][COLS];
static bool selected = false;
static int8_t selR = -1;
static int8_t selC = -1;
static uint32_t score = 0;

static inline void wrStrobe()
{
  digitalWrite(PIN_LCD_WR, LOW);
  digitalWrite(PIN_LCD_WR, HIGH);
}

static inline void busWrite16(uint16_t value)
{
  for (uint8_t i = 0; i < 16; ++i)
  {
    digitalWrite(LCD_D[i], (value >> i) & 0x01);
  }
  wrStrobe();
}

static inline void lcdWriteCmd(uint8_t cmd)
{
  digitalWrite(PIN_LCD_CS, LOW);
  digitalWrite(PIN_LCD_RS, LOW);
  busWrite16(cmd);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static inline void lcdWriteData8(uint8_t data)
{
  digitalWrite(PIN_LCD_CS, LOW);
  digitalWrite(PIN_LCD_RS, HIGH);
  busWrite16(data);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static inline void lcdWriteData16(uint16_t data)
{
  digitalWrite(PIN_LCD_CS, LOW);
  digitalWrite(PIN_LCD_RS, HIGH);
  busWrite16(data);
  digitalWrite(PIN_LCD_CS, HIGH);
}

void lcdWriteCmdData(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  lcdWriteCmd(cmd);
  for (uint8_t i = 0; i < len; ++i)
  {
    lcdWriteData8(data[i]);
  }
}

void lcdSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t ca[4] = {
      (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
      (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
  uint8_t ra[4] = {
      (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
      (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};

  lcdWriteCmdData(0x2A, ca, 4);
  lcdWriteCmdData(0x2B, ra, 4);
  lcdWriteCmd(0x2C);
}

void lcdPushColor(uint16_t color, uint32_t count)
{
  digitalWrite(PIN_LCD_CS, LOW);
  digitalWrite(PIN_LCD_RS, HIGH);
  while (count--)
  {
    busWrite16(color);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
}

void lcdFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
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

void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if (x >= LCD_W || y >= LCD_H)
    return;
  lcdSetAddrWindow(x, y, x, y);
  lcdWriteData16(color);
}

void lcdDrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if (w < 2 || h < 2)
    return;
  lcdFillRect(x, y, w, 1, color);
  lcdFillRect(x, y + h - 1, w, 1, color);
  lcdFillRect(x, y, 1, h, color);
  lcdFillRect(x + w - 1, y, 1, h, color);
}

void lcdFillScreen(uint16_t color)
{
  lcdFillRect(0, 0, LCD_W, LCD_H, color);
}

void lcdReset()
{
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(10);
  digitalWrite(PIN_LCD_RST, LOW);
  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(120);
}

void lcdInitST7796()
{
  lcdReset();

  lcdWriteCmd(0x01); // SWRESET
  delay(120);

  lcdWriteCmd(0x11); // SLPOUT
  delay(120);

  uint8_t madctl[] = {0x48}; // 竖屏方向
  lcdWriteCmdData(0x36, madctl, 1);

  uint8_t pixfmt[] = {0x55};
  lcdWriteCmdData(0x3A, pixfmt, 1);

  lcdWriteCmd(0x21); // INVON
  delay(10);

  lcdWriteCmd(0x29); // DISPON
  delay(20);
}

bool ft6336ReadRaw(uint16_t &x, uint16_t &y)
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

void touchToScreen(uint16_t rawX, uint16_t rawY, uint16_t &sx, uint16_t &sy)
{
  // 默认与你的测试程序一致（竖屏直通）
  sx = rawX;
  sy = rawY;

  if (sx >= LCD_W)
    sx = LCD_W - 1;
  if (sy >= LCD_H)
    sy = LCD_H - 1;
}

uint16_t colorOf(uint8_t t)
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

bool causesMatchAt(uint8_t r, uint8_t c)
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

void initBoard()
{
  for (uint8_t r = 0; r < ROWS; ++r)
  {
    for (uint8_t c = 0; c < COLS; ++c)
    {
      do
      {
        board[r][c] = random(TYPES);
      } while (causesMatchAt(r, c));
    }
  }
}

void drawCell(uint8_t r, uint8_t c, bool hl)
{
  uint16_t x = BOARD_X + c * CELL;
  uint16_t y = BOARD_Y + r * CELL;
  uint16_t fill = colorOf(board[r][c]);

  lcdFillRect(x + 2, y + 2, CELL - 4, CELL - 4, fill);
  lcdDrawRect(x + 1, y + 1, CELL - 2, CELL - 2, 0x0000);

  if (hl)
  {
    lcdDrawRect(x, y, CELL, CELL, 0xFFFF);
  }
}

// 极简 7 段数字绘制（只做分数）
void segH(uint16_t x, uint16_t y, uint16_t w, uint16_t t, uint16_t color) { lcdFillRect(x, y, w, t, color); }
void segV(uint16_t x, uint16_t y, uint16_t t, uint16_t h, uint16_t color) { lcdFillRect(x, y, t, h, color); }

void drawDigit(uint16_t x, uint16_t y, uint8_t d, uint16_t color, uint16_t bg)
{
  const uint8_t map[10] = {
      0b1111110, // 0
      0b0110000, // 1
      0b1101101, // 2
      0b1111001, // 3
      0b0110011, // 4
      0b1011011, // 5
      0b1011111, // 6
      0b1110000, // 7
      0b1111111, // 8
      0b1111011  // 9
  };

  uint8_t m = map[d % 10];
  uint16_t t = 3;
  uint16_t w = 14;
  uint16_t h = 22;

  lcdFillRect(x, y, w + 2 * t, h + 2 * t, bg);

  // a b c d e f g
  if (m & 0b1000000) segH(x + t, y, w, t, color);
  if (m & 0b0100000) segV(x + t + w, y + t, t, h / 2 - t / 2, color);
  if (m & 0b0010000) segV(x + t + w, y + h / 2 + t / 2, t, h / 2 - t / 2, color);
  if (m & 0b0001000) segH(x + t, y + h, w, t, color);
  if (m & 0b0000100) segV(x, y + h / 2 + t / 2, t, h / 2 - t / 2, color);
  if (m & 0b0000010) segV(x, y + t, t, h / 2 - t / 2, color);
  if (m & 0b0000001) segH(x + t, y + h / 2, w, t, color);
}

void drawScore()
{
  uint16_t x0 = 170;
  uint16_t y0 = 18;
  lcdFillRect(120, 10, 190, 40, 0x0000);

  // SCORE 标题条
  lcdFillRect(16, 12, 90, 36, 0x39E7);
  lcdDrawRect(16, 12, 90, 36, 0xFFFF);
  lcdFillRect(22, 18, 12, 24, 0xF800);
  lcdFillRect(38, 18, 12, 24, 0x07E0);
  lcdFillRect(54, 18, 12, 24, 0x001F);
  lcdFillRect(70, 18, 12, 24, 0xFFE0);

  uint32_t v = score;
  uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
  for (int8_t i = 5; i >= 0; --i)
  {
    digits[i] = v % 10;
    v /= 10;
  }

  for (uint8_t i = 0; i < 6; ++i)
  {
    drawDigit(x0 + i * 24, y0, digits[i], 0xFFFF, 0x0000);
  }
}

void drawBoardFrame()
{
  lcdFillRect(0, 0, LCD_W, LCD_H, 0x0000);
  lcdFillRect(0, 0, LCD_W, HUD_H, 0x2104);
  drawScore();

  lcdDrawRect(BOARD_X - 3, BOARD_Y - 3, COLS * CELL + 6, ROWS * CELL + 6, 0xFFFF);
}

void drawBoard()
{
  for (uint8_t r = 0; r < ROWS; ++r)
  {
    for (uint8_t c = 0; c < COLS; ++c)
    {
      bool hl = selected && r == (uint8_t)selR && c == (uint8_t)selC;
      drawCell(r, c, hl);
    }
  }
}

bool findMatches(bool mark[ROWS][COLS])
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

uint16_t clearDrop(bool mark[ROWS][COLS])
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

void flashTop(uint16_t color)
{
  lcdFillRect(0, 0, LCD_W, 6, color);
  delay(40);
  lcdFillRect(0, 0, LCD_W, 6, 0x2104);
}

void resolveMatches()
{
  bool mark[ROWS][COLS];
  while (findMatches(mark))
  {
    uint16_t n = clearDrop(mark);
    score += (uint32_t)n * 10;
    flashTop(0xFFE0);
    drawScore();
    drawBoard();
    delay(60);
  }
}

bool canSwapMakeMatch(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2)
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

bool hasPossibleMoves()
{
  for (uint8_t r = 0; r < ROWS; ++r)
  {
    for (uint8_t c = 0; c < COLS; ++c)
    {
      if (c + 1 < COLS && canSwapMakeMatch(r, c, r, c + 1))
        return true;
      if (r + 1 < ROWS && canSwapMakeMatch(r, c, r + 1, c))
        return true;
    }
  }
  return false;
}

void shuffleBoard()
{
  for (uint8_t i = 0; i < 80; ++i)
  {
    uint8_t r1 = random(ROWS), c1 = random(COLS);
    uint8_t r2 = random(ROWS), c2 = random(COLS);
    uint8_t t = board[r1][c1];
    board[r1][c1] = board[r2][c2];
    board[r2][c2] = t;
  }
  resolveMatches();
  drawBoard();
}

bool hitCell(uint16_t x, uint16_t y, uint8_t &r, uint8_t &c)
{
  if (x < BOARD_X || y < BOARD_Y)
    return false;
  uint16_t lx = x - BOARD_X;
  uint16_t ly = y - BOARD_Y;
  if (lx >= COLS * CELL || ly >= ROWS * CELL)
    return false;
  c = lx / CELL;
  r = ly / CELL;
  return true;
}

void processTap(uint16_t sx, uint16_t sy)
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
      flashTop(0xF800);
      shuffleBoard();
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

void gpioInit()
{
  for (uint8_t i = 0; i < 16; ++i)
  {
    pinMode(LCD_D[i], OUTPUT);
    digitalWrite(LCD_D[i], LOW);
  }

  pinMode(PIN_LCD_CS, OUTPUT);
  pinMode(PIN_LCD_RS, OUTPUT);
  pinMode(PIN_LCD_WR, OUTPUT);
  pinMode(PIN_LCD_RD, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT);
  pinMode(PIN_LCD_BL, OUTPUT);

  digitalWrite(PIN_LCD_CS, HIGH);
  digitalWrite(PIN_LCD_RS, HIGH);
  digitalWrite(PIN_LCD_WR, HIGH);
  digitalWrite(PIN_LCD_RD, HIGH);
  digitalWrite(PIN_LCD_RST, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("Match-3 start");

  randomSeed(analogRead(PA8) + micros());

  gpioInit();

  Wire.setSDA(PIN_TP_SDA);
  Wire.setSCL(PIN_TP_SCL);
  Wire.begin();
  Wire.setClock(400000);

  lcdInitST7796();
  drawBoardFrame();

  initBoard();
  resolveMatches();
  drawBoard();

  if (!hasPossibleMoves())
    shuffleBoard();

  Serial.println("Ready.");
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

    Serial.print("tap x=");
    Serial.print(sx);
    Serial.print(" y=");
    Serial.println(sy);
  }

  down = touched;
  delay(8);
}
