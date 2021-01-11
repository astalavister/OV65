
#include <Arduino.h>

byte bukva_B[8] = {
    B11110,
    B10000,
    B10000,
    B11110,
    B10001,
    B10001,
    B11110,
    B00000,
}; // Буква "Б"
byte bukva_G[8] = {
    B11111,
    B10001,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B00000,
}; // Буква "Г"
byte bukva_D[8] = {
    B01111,
    B00101,
    B00101,
    B01001,
    B10001,
    B11111,
    B10001,
    B00000,
}; // Буква "Д"
byte bukva_ZH[8] = {
    B10101,
    B10101,
    B10101,
    B11111,
    B10101,
    B10101,
    B10101,
    B00000,
}; // Буква "Ж"
byte bukva_Z[8] = {
    B01110,
    B10001,
    B00001,
    B00010,
    B00001,
    B10001,
    B01110,
    B00000,
}; // Буква "З"
byte bukva_I[8] = {
    B10001,
    B10011,
    B10011,
    B10101,
    B11001,
    B11001,
    B10001,
    B00000,
}; // Буква "И"
byte bukva_IY[8] = {
    B01110,
    B00000,
    B10001,
    B10011,
    B10101,
    B11001,
    B10001,
    B00000,
}; // Буква "Й"
byte bukva_L[8] = {
    B00011,
    B00111,
    B00101,
    B00101,
    B01101,
    B01001,
    B11001,
    B00000,
}; // Буква "Л"
byte bukva_P[8] = {
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B00000,
}; // Буква "П"
byte bukva_Y[8] = {
    B10001,
    B10001,
    B10001,
    B01010,
    B00100,
    B01000,
    B10000,
    B00000,
}; // Буква "У"
byte bukva_F[8] = {
    B00100,
    B11111,
    B10101,
    B10101,
    B11111,
    B00100,
    B00100,
    B00000,
}; // Буква "Ф"
byte bukva_TS[8] = {
    B10010,
    B10010,
    B10010,
    B10010,
    B10010,
    B10010,
    B11111,
    B00001,
}; // Буква "Ц"
byte bukva_CH[8] = {
    B10001,
    B10001,
    B10001,
    B01111,
    B00001,
    B00001,
    B00001,
    B00000,
}; // Буква "Ч"
byte bukva_Sh[8] = {
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B11111,
    B00000,
}; // Буква "Ш"
byte bukva_Shch[8] = {
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B10101,
    B11111,
    B00001,
}; // Буква "Щ"
byte bukva_Mz[8] = {
    B10000,
    B10000,
    B10000,
    B11110,
    B10001,
    B10001,
    B11110,
    B00000,
}; // Буква "Ь"
byte bukva_IYI[8] = {
    B10001,
    B10001,
    B10001,
    B11001,
    B10101,
    B10101,
    B11001,
    B00000,
}; // Буква "Ы"
byte bukva_Yu[8] = {
    B10010,
    B10101,
    B10101,
    B11101,
    B10101,
    B10101,
    B10010,
    B00000,
}; // Буква "Ю"
byte bukva_Ya[8] = {
    B01111,
    B10001,
    B10001,
    B01111,
    B00101,
    B01001,
    B10001,
    B00000,
}; // Буква "Я"
// иконка термометра:
byte thermometerIcon[8] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B11111,
  B01110
};
// иконка капли:
byte kaplyaIcon[8] = {
  B00000,
  B01100,
  B01100,
  B11110,
  B11110,
  B11111,
  B11111,
  B01110
};
/*
extern byte bukva_G[8] ; // Буква "Г"
extern byte bukva_D[8] ; // Буква "Д"
extern byte bukva_ZH[8] ; // Буква "Ж"
extern byte bukva_Z[8] ; // Буква "З"
extern byte bukva_I[8] ; // Буква "И"
extern byte bukva_IY[8] ; // Буква "Й"
extern byte bukva_L[8] ; // Буква "Л"
extern byte bukva_P[8] ; // Буква "П"
extern byte bukva_Y[8] ; // Буква "У"
extern byte bukva_F[8] ; // Буква "Ф"
extern byte bukva_TS[8] ; // Буква "Ц"
extern byte bukva_CH[8] ; // Буква "Ч"
extern byte bukva_Sh[8] ; // Буква "Ш"
extern byte bukva_Shch[8] ; // Буква "Щ"
extern byte bukva_Mz[8] ; // Буква "Ь"
extern byte bukva_IYI[8]; // Буква "Ы"
extern byte bukva_Yu[8] ; // Буква "Ю"
extern byte bukva_Ya[8] ; // Буква "Я"
// иконка термометра:
extern byte thermometerIcon[8];
// иконка капли:
extern byte kaplyaIcon[8] ;*/
