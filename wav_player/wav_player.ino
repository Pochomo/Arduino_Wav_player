// Databus portF로
#define LCD_DATABUS PORTF
// E, Rs 용 PortB로
#define LCD_CONTROL PORTB

// 현재 LCD랑 SD모두 DDRB를 사용중인데 둘의 충돌을 막기 위해 정의
// E
#define LCD_E PB5
// RS
#define LCD_RS PB6

// SD 카드 사용을 위한 SPI 통신
#include <SPI.h>
//8.3 이상 문자열 사용하기 위해 sdFat 사용
#include <SdFat.h>
const uint8_t SD_CS_PIN = 53;


// 노래 길이 판단 LCD에 몇 퍼센트 진행됐는지 출력 위해서 (WAV file 활용)
volatile uint32_t played = 0;
uint32_t totalSample = 0;
// 진행도 출력
volatile uint16_t progress = 0;
// 00.0 % 로 출력
volatile uint16_t progress10 = 0;

bool playing = false;
// rotary encoder 회전 값
volatile int16_t rotary_cnt = 0;
// rotary encoder 버튼 클릭 여부 클릭되면 다음 스탭으로 가거나 노래 실행해야되니깐(playing stopped)
volatile bool rotary_pressed = false;
// 현재 LCD 상태에 따라 숫자로 분류 
//0 -> 노래 장르 선택
//1 -> 노래 선택
volatile int8_t ui = 0;
// 장르 리스트 인덱스
volatile int8_t genre_idx = 0;
// 노래 리스트 인덱스 
volatile int8_t sing_idx = 0;
// 리스트 크기 sdfat 스캔
uint8_t genre_cnt = 0;
// 노래 리스트 크기 마찬가지로 sdfat으로 스캔
uint8_t sing_cnt = 0;
// 회전하기 전에 이전 값 저장을 위한 변수
volatile int16_t before_cnt = 0;
//====================== wav ============

const uint16_t BUF_SIZE = 3000;
unsigned char buf[2][BUF_SIZE];
volatile uint8_t curBuf = 0;
volatile uint16_t bufPos[2] = {0, 0};
// 버퍼에 실제로 채워진 데이터 길이
volatile uint16_t bufLen[2] = {0, 0};
// 버퍼가 비어있는지 여부
volatile uint16_t bufEmpty[2] = {true, true};

// 1, 2, 4 에 따라 mono인지 stereo인지 16bit인지 짝이 지어진다.
uint8_t bytesPerSample = 0;
//44khz 16bit 구현을 구한 전역변수 22k로 낮춰서 구현한다.
volatile bool rate44k = false;
// struct 활용해서 wav 정보 저장소를 만들었다
struct WavInfo {
  uint32_t sampleRate;
  uint16_t numChannels;
  uint16_t bitsPerSample;
  uint32_t dataSize;
} wavInfo;

// 처음에는 배열의 크기를 정하고 프로젝트를 진행하고자 했으나
// 더 많은 파일을 읽을 수 있게 하기 위해 
// SDcard를 읽고 갯수를 파악하도록 했다.
//const uint8_t max_genre = 4;
//const uint8_t max_song = 5;
//const uint8_t name_len = 28;
// 영어 스페이스 언더바
//const uint8_t genre_name_len = 28;
// 장르 디렉토리 이름
//char genreName[max_genre][genre_name_len];
// 노래 이름 장르 노래 노래제목
//char songName[max_song][name_len];
// 장르별 곡 개수
//uint8_t songCount[max_genre];
// ================================================

// 현재 출력중인 장르
volatile int8_t cur_genre = -1;
// 현재 출력중인 노래
volatile int8_t cur_song = -1;

//================================================
  
//=============== LED 관련 변수 ===================
// LED 갯수에 따라서 스킵 초 결정
volatile uint8_t skipS = 1;
// 예를 들어 0x03은 LED 2개 켜진것이다.
const uint8_t ledP[7] = {
  0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F
};
// 뒤로 보내기
volatile bool sw1 = false;
// 앞으로 보내기
volatile bool sw2 = false;
// 시작 위치 기억용으로 초기화
uint32_t startPos = 0;

SdFat sd;
SdFile wavFile;

// SDcard 클래스에 Sd 카드 관련 함수 및 wavfile 실행 함수를 추가했다.
// 추가적으로 wavHeader, fillbuffer 전역함수를 사용한다.
// wav 파일 헤더를 파싱해 sample rate, 채널 수, bit 수, 데이터 크기를 아래 정의한 
// wavInfo struct에 저장한 후 선택된 곡을 실제 재생함수 startplay 및 stopplay를 통해 실행한다.
class Sdcard {
  public:

    // root 디렉토리, 현재 탐색 중인 디렉토리 및 파일을 조작한다.
    SdFile root;
    SdFile dir;
    SdFile file;

    // Sdcard class
    bool check_wav(const char *name);
    // 루트에서 장르 갯수 카운팅
    uint8_t genreCount();

    // idx 번째 장르 디렉토리 이름 ouput에 넣는다.
    bool genreName(uint8_t idx, char *output, uint8_t maxLen);

    // gidx, sidx에 해당하는 곡 파일 이름을 output에 채워 넣는다. 
    bool songfromIdx(uint8_t gidx, uint8_t sidx, char *output, uint8_t maxLen);

    // gidx 번째 장르 디렉토리 안 웨이브 파일 갯수 리턴
    uint8_t songcount(uint8_t gidx);
    
    // SD 스캔해서 genre_cnt, sing_cnt
    // genre_idx, sing_idx 초기화
    void scan();

    // gidx가 바꼈을때 곡 개수를 다시 초기화
    void loadSong(uint8_t gidx);
    // gidx, sidx에 해당하는 곡을 열고 재생한다.
    void startplay(int8_t gidx, int8_t sidx);
    // 현재 재생 중인 wav 파일 닫고 stop
    void stopplay();


};

// LED class LED 켜진 갯수만큼 스킵을 하는 멤버함수와
// LED를 사용하기 위한 start()를 통해 포트를 출력용으로 초기화
// skip()에서 skipS 값을 갱신한다
class LED {
  public:
    bool start();

    // analogread()를 통해 1~7초 스킵 범위 결정
    // 해당 초에 맞는 LED 패턴을 PORTC에 출력한다.
    void skip();
};

// LCD 클래스 16x2 문자 LCD에 장르/곡 제목, PLAY/STOP 상태, 재생 진행도를 출력
// start()에서 LCD 초기화
// LCD_command, LCD_data, LCD_initialize, LCD_string 을 통해 LCD_device_driver 구현
class LCD {
  public:
    // LCD 관련 멤버함수들
    // LCD 포트 초기화 및 LCD 초기화
    bool start();
    //LCD_device_driver구현을 위한 함수들
    void LCD_command(unsigned char command);
    void LCD_data(unsigned char data);
    void LCD_initialize(void);
    void LCD_string(unsigned char command, const char *string);
    void Set_font();
    // [<-] 출력
    void Pointer();
    // genre_idx에 해당하는 디렉토리 이름 []에 감싸서 출력
    void printdic();
    // sing_idx에 해당하는 곡 제목 출력
    void printTitle();
    // playing or stopped 출력
    void printPlayORStop();
    // 2행 끝에 곡 진행도 출력
    void printpercentage();
    // LCD 지우고 ui == 0 이면 장르화면, ui == 1이면 곡화면으로 판단
    void clearLCD();
};

// 로터리 클래스
// ExtInterruptInit()에서 PD1(버튼), PD2/PD3(A/B 채널)을 입력 + 내부 풀업으로 설정하고
// INT1, INT3 인터럽트를 falling edge로 구성한다.
// ISR(INT1_vect) ISR(INT3_vect)는 전역 변수 
// rotary_pressed와 rotary_cnt를 갱신
// updateRotary()와 handleRotary()에서 로터리 회전수를 업데이트한다
class Rotary {
  public:
    // 다른 클래스와 구성을 맞추기 위해 사용
    // 이번 프로젝트에서는 사용하지않았다.
    bool start();
    // Rotary 사용위해 External Interrupt 정의
    void ExtInterruptInit();

};

// Sdcard class 멤버 함수 및 변수 사용 위해서
Sdcard sdcard;

// ==================== LED 파트 =====================
bool LED::start() {
  // PORTC 사용하며 led1~7 출력 모드로 설정
  DDRC |= 0x7F;
  // 시작 전에는 모두 끝 상태로
  PORTC &= ~0x7F;

  return true;
}
void LED::skip() {
  static uint32_t last = 0;
  // 현재 시간 갱신
  uint32_t now = millis();  

  // analogRead를 통해 핀에서 값 읽고
  uint16_t v = analogRead(A9);

  // 0~1023을 7 구간으로 나눈다.
  // 1023/7이 146이라 v/146 이 0~7 근처 값이 나오고 + 1 해서 1~8이 되게 한 후
  // 7을 넘는 값들은 7 로 잘라낸다.
  uint8_t s = v / 146 + 1;
  // 최소 1초
  if (s<1) {
    s = 1;
  }
  if (s>7) {
    s = 7;
  }
  
  // 스킵 시간 초기화
  skipS = s;

  PORTC = (PORTC & ~0x7F);
  // 스킵 초 만큼 led 표시
  PORTC = ledP[skipS - 1];

}

// ==================== LCD 파트 =====================
bool LCD::start(){
  DDRF |= 0xFF;
  // PB5, PB6만 출력으로
  DDRB |= _BV(LCD_E);
  DDRB |= _BV(LCD_RS);
  // 약간의 delay 후 초기화
  delay(50);
  LCD_initialize();
  Set_font();
  return true;
} 
void LCD::LCD_command(unsigned char command){
  
  // // E = 0, Rs = 0 으로
  // LCD_CONTROL = 0x00;
  // // output command databus에 출력한다.
  // LCD_DATABUS = command;
  // // E = 1, 즉 데이터 읽기를 시작
  // LCD_CONTROL = 0x20;
  // 위의 코드에서 덮어쓰는 문제 발생
  
  // E=0, RS=0으로 하는데 다른 비트는 유지한다.
  PORTB &= ~_BV(LCD_E);
  PORTB &= ~_BV(LCD_RS);

  LCD_DATABUS = command;

  // E 1 데이터 읽기 시작
  PORTB |= _BV(LCD_E);
  
  // 250ns 만큼 delay
  asm volatile(" PUSH R0 ");
  asm volatile(" POP R0 ");
  // 다시 E = 0
  PORTB &= ~_BV(LCD_E);
  
  // 처리 대기
  delayMicroseconds(50);
}

void LCD::LCD_data(unsigned char data){
  // // E = 0, Rs = 1로 문자 데이터라고 LCD에게 알려줄 준비하고
  // LCD_CONTROL = 0x40;
  // // output data를 databus에 초기화
  // LCD_DATABUS = data;
  // // E = 1, Rs = 1 write 트리거
  // LCD_CONTROL = 0x60;
  
  // E=0, RS=1 데이터 모드
  PORTB &= ~_BV(LCD_E);
  PORTB |= _BV(LCD_RS);

  LCD_DATABUS = data;

  PORTB |= _BV(LCD_E);

  // 250ns 만큼 delay
  asm volatile(" PUSH R0 ");
  asm volatile(" POP R0 ");

  // E = 0, Rs = 1 데이터 처리
  PORTB &= ~_BV(LCD_E);

  // 처리 대기
  delayMicroseconds(50);
}

void LCD::LCD_initialize(void){
  // 8bit 2line, 5x7 dot를 사용한다.
  // 0011 1000
  LCD_command(0x38);
  // display control 디스플레이가 켜지고 커서가 꺼진다.
  LCD_command(0x0C);
  // entry mode set
  LCD_command(0x06);
  // display clear
  LCD_command(0x01);
  delay(2);
}

// LCDdp string 보여준다.
void LCD::LCD_string(unsigned char command, const char *string){
  LCD_command(command);
  while(*string != '\0'){
    LCD_data(*string);
    string++;
  }
}

void LCD::Set_font(){
  // 사용하면 쓸 예정
  unsigned char point[1] = {0x0E};

}

// 디렉토리 끝에 도달하면 출력
void LCD::Pointer(){
  //[<-] 0x5B, 0x3C, 0x2D, 0x5D 
  // 항상 맨 처음에 있는 화살표
  LCD_command(0x80);
  LCD_data(0x5B);
  LCD_data(0x3C);
  LCD_data(0x2D);
  LCD_data(0x5D);
}

// 디렉토리 이름 출력할 함수 [] 감싸서 
void LCD::printdic() {
  LCD_command(0x80);

  // 장르 제목 여유있게 40으로
  char gname[40];
  const char *genre = "";

  // 장르 갯수가 0보다 크고 인덱스가 0보다 크거나 같고
  // 장르 인덱스가 갯수보다 작을(인덱스는 항상 갯수보다 작아야하기에) 경우
  // 장르 이름을 가져올 수 있음
  if (genre_cnt > 0 && genre_idx >= 0 && (uint8_t)genre_idx < genre_cnt
    && sdcard.genreName((uint8_t)genre_idx, gname, sizeof(gname))) {
    genre = gname;
  }

  // 글자 수 넘어가면 ] 출력 하지 않는다.
  uint8_t len = strlen(genre);
  if (len <= 14) {
    // [
    LCD_data(0x5B);
    
    uint8_t cnt = 0;
    // []포함하니깐 14글자 까지만 
    while (*genre && cnt < 14) {
      LCD_data(*genre++);
      cnt++;
    }
    //[
    LCD_data(0x5D);
  } else {
    // 14글자 넘어가면 ] 버린다
    LCD_data(0x5B);

    uint8_t cnt = 0;
    // 위보다 한글자 더 출력
    while (*genre && cnt < 15) {
      LCD_data(*genre++);
      cnt++;
    }
  }
}

// 노래 제목 출력할 함수
void LCD::printTitle() {
  // 노래 제목 SDfat 사용
  LCD_command(0x80);
  const char *title = "";
  char sname[40];

  // 위함수와 비슷한데 sing관련을 추가해 곡 제목을 출력한다.
  if (genre_cnt > 0 && sing_cnt > 0 &&
  genre_idx >= 0 && (uint8_t)genre_idx < genre_cnt
  && (uint8_t)sing_idx < sing_cnt &&
  sdcard.songfromIdx((uint8_t)genre_idx, (uint8_t)sing_idx, sname, sizeof(sname))) {
    title = sname;
  }
  const char *p = title;
  uint8_t cnt = 0;
  while(*p && cnt < 16) {
    LCD_data(*p++);
    cnt++;
  }

}

void LCD::printPlayORStop() {
  if (playing == true) {
    LCD_string(0xC0, "PLAYING");
  }
  else { LCD_string(0xC0, "STOPPED");}
}

// 음악 재생 % 출력할 함수
void LCD::printpercentage() {
  // 2번째 라인 오른쪽 끝에
  // CA 위치에서 100% 자리 만든다.
  // 99.9 -> 100% 가 되게하면됨
  LCD_command(0xCA);
  
  // 노래 진행도
  uint16_t p = progress10;

  // 1000으로 표현 한 이유는 0 -> 0.0 1 -> 0.1 이렇게 스케일링 편하게 하기 위해
  if (p > 1000) {p = 1000;}

  if (p == 1000) {
    // 100 % 출력
    LCD_data(' ');
    LCD_data(' ');
    LCD_data('1');
    LCD_data('0');
    LCD_data('0');
    LCD_data('%');
  } else {
    // 0부터 99
    uint8_t i = p / 10;
    // 0 부터 9
    uint8_t j = p % 10;

    // 10, 1 자리
    uint8_t k = i / 10;
    uint8_t z = i % 10;
    
    LCD_data(' ');
    // 10자리
    LCD_data('0' + k);
    // 1자리
    LCD_data('0' + z);
    LCD_data('.');
    // 소수 첫번째 자리 
    LCD_data('0' + j);
    LCD_data('%');
  }
}

void LCD::clearLCD() {
  // 화면 지우고
  LCD_command(0x01);
  delay(2);

  if (ui == 0) {
    //장르 선택
    printdic();

  } else {
    // 맨 끝이면 [<-]
    if (sing_idx == sing_cnt) {
      Pointer();
    } else {
      printTitle();
    }
  }
  // 진행 상태 출력
  printPlayORStop();
} 

// ==================== ========== =====================
LCD lcd;
LED led;
// ==================== SW =====================

// 스위치에 따라 앞/뒤로 노래를 점프 시키는 함수
void jump(int8_t direction){
  // direction > 0 이면 앞으로 < 0 이면 뒤로 보낸다
  if (!wavFile.isOpen()) {return;}
  if (wavInfo.sampleRate == 0) {return;}

  // 한 샘플당 바이트수 (bit/8) * 채널 수
  uint8_t bytePerSample = (wavInfo.bitsPerSample / 8) * wavInfo.numChannels;
  if (bytePerSample == 0) {return;}

  // skip초 만큼 건너뛸 바이트 수 초기화
  uint32_t skipBytes = (uint32_t)skipS * wavInfo.sampleRate * bytePerSample;

  // 현재 위치
  uint32_t curPos = wavFile.curPosition();
  if (curPos < startPos) curPos = startPos;

  // data 청크 내에서 현재 위치가 data 시작점으로 부터 얼마나 떨어져있는지
  uint32_t curdiff = curPos - startPos;
  uint32_t tmpdiff;

  if (direction > 0) {
    // 앞으로 스킵
    if (curdiff + skipBytes >= wavInfo.dataSize) {
      // 넘어갈 위치가 끝이면 마지막 맨끝 바로 앞으로 이동
      tmpdiff = wavInfo.dataSize - bytePerSample;
    }
    else {
      // 아직 data 범위 안이면 skipBytes 만큼 점프
      tmpdiff = curdiff + skipBytes;
    }
  } else {
    // 뒤로
    // 현재 위치에서 skipBytes 만큼 뒤로 갔을 때 0 보다 크면 그만큼 뒤로
    if (curdiff > skipBytes) {
      tmpdiff = curdiff - skipBytes;
    }
    else {
      // 너무 많이 가면 음수가 되니깐 0으로
      tmpdiff = 0;
    }
  }

  // 채널 비트 수에 맞게 초기화
  tmpdiff -= tmpdiff % bytePerSample;

  // diff를 sample 인덱스로 변환한다
  if (bytePerSample != 0) {
    played = tmpdiff / bytePerSample;
  } else {
    played = 0;
  }

  uint32_t newPos = startPos + tmpdiff;

  // 스킵 할때는 잠시 중단 하고 넘긴다
  playing = false;

  // 파일 위치 이동 시키고
  wavFile.seekSet(newPos);

  // 버퍼 초기화하고 다시 채운다
  curBuf = 0;
  bufPos[0] = bufPos[1] = 0;
  bufLen[0] = bufLen[1] = 0;
  bufEmpty[0] = bufEmpty[1] = true;

  fillBuffer(0);
  fillBuffer(1);

  playing = true;
}

void skipF(){
  // 앞으로 스킵
  jump(1);
}

void skipB(){
  // 뒤로 스킵
  jump(-1);
}

// sw용 초기화
void skipSwInit() {
  // sw1 
  DDRD &= ~_BV(DDD0);
  PORTD |= _BV(PORTD0);
  // sw2
  DDRE &= ~_BV(DDE5);
  PORTE |= _BV(PORTE5);
 
  // INT0 : falling sw1
  EICRA |= _BV(ISC01);
  EICRA &= ~_BV(ISC00);

  // INT5 : falling sw2
  EICRB |= _BV(ISC51);
  EICRB &= ~_BV(ISC50);

  // interrupt flag clear
  EIFR |= _BV(INTF0);
  EIFR |= _BV(INTF5);
  
  // interrupt enable
  EIMSK |= _BV(INT0);
  EIMSK |= _BV(INT5);
   
}

// sw ISR 뒤로 INT0
ISR(INT0_vect) {
  sw1 = true;
}

// sw ISR 앞으로 INT5
ISR(INT5_vect) {
  sw2 = true;
}
// ==================== ========== =====================

// ==================== Timer 파트 =====================
// Tiemr1을 Wav timer로 사용 22.05khz, 44.1khz
// ISR(TIMER1_COMPA_vect)에서 샘플 1프레임씩 처리한다
void TimerInit() {
  // Timer1 카운터 값 초기화
  TCNT1 = 0;
  // Interrupt Enable, 및 Flag clear
  TIMSK1 |= _BV(OCIE1A);
  TIFR1 |= _BV(OCF1A);
}

void timer_setting_44_10KHZ() {
  //Timer/Counter 1을 사용해 44.1 KHzz로 PWM 갱신
  //CTC 모드 사용 즉 WGM13, WGM12, WGM11, WGM10 = 0100 
  
  TCCR1A &= ~_BV(WGM10);
  TCCR1A &= ~_BV(WGM11);
  TCCR1B |= _BV(WGM12);
  TCCR1B &= ~_BV(WGM13);

  // N = 1 cs10 = 1, cs 11=0, cs12 =0
  TCCR1B |= _BV(CS10);
  TCCR1B &= ~_BV(CS11);
  TCCR1B &= ~_BV(CS12);

  // N = 1 f= 16000000 / (N*(1+OCR1A)) -> OCR1A = 361.811이기 때문에 362로 설정
  OCR1A = 362;

}

void timer_setting_22_05KHZ() {
  //Timer/Counter 1을 사용해 22.05KHz로 PWM 갱신
  //CTC 모드 사용 즉 WGM13, WGM12, WGM11, WGM10 = 0100 

  TCCR1A &= ~_BV(WGM10);
  TCCR1A &= ~_BV(WGM11);
  TCCR1B |= _BV(WGM12);
  TCCR1B &= ~_BV(WGM13);

  // N = 1 cs10 = 1, cs 11=0, cs12 =0
  TCCR1B |= _BV(CS10);
  TCCR1B &= ~_BV(CS11);
  TCCR1B &= ~_BV(CS12);


  // N = 1 f= 16000000 / (N*(1+OCR1A)) -> OCR1A = 724.623이기 때문에 725로 설정
  OCR1A = 725;

}

// 8-bit pwm 생성하기 위한 timer2/timer4
void timer2_setting() {
  //Timer/Counter 2, Timer/Counter 4를 사용해 8bit PWM 생성
  //두개 모두 Fast PWM 8 bit 사용하니깐 WGM23, WGM22, WGM21, WGM20 = 0101
  // 출력 핀 노드
  // PWM_L
  DDRH |= _BV(PH6);
  // PWM_H
  DDRB |= _BV(PB4);

  // 비반전 출력
  TCCR2A |= _BV(COM2A1);
  TCCR2A &= ~_BV(COM2A0);
  TCCR2A |= _BV(COM2B1);
  TCCR2A &= ~_BV(COM2B0);

  // Fast Pwm 8 bit (0b011) WGM23이 없다
  TCCR2A |= _BV(WGM20);
  TCCR2A |= _BV(WGM21);
  TCCR2B &= ~_BV(WGM22);

  // cs 0b001
  TCCR2B &= ~_BV(CS22);
  TCCR2B &= ~_BV(CS21);
  TCCR2B |= _BV(CS20);

  // 소리 128씩 대칭 되도록
  OCR2A = 128;
  OCR2B = 128;
}

void timer4_setting() {
  
  // 출력 핀 노드 7번 PH4 사용 -> DDRH, PH6를 출력용으로 사용
  // PWM_L
  DDRH |= _BV(PH4);
  // PWM_H
  DDRH |= _BV(PH3);
  // 비반전 출력 0b0101
  TCCR4A |= _BV(COM4A1);
  TCCR4A &= ~_BV(COM4A0);
  TCCR4A |= _BV(COM4B1);
  TCCR4A &= ~_BV(COM4B0);

  //Fast PWM 8 bit 사용하니깐 WGM23, WGM22, WGM21, WGM20 = 0101
  TCCR4A |= _BV(WGM40);
  TCCR4A &= ~_BV(WGM41);
  TCCR4B |= _BV(WGM42);
  TCCR4B &= ~_BV(WGM43);
  
  // cs 0b001
  TCCR4B &= ~_BV(CS42);
  TCCR4B &= ~_BV(CS41);
  TCCR4B |= _BV(CS40);

  // 소리 128 H, L가 대칭 되도록
  OCR4A = 128;
  OCR4B = 128;
}

// Timer interrupt
ISR(TIMER1_COMPA_vect) {
  // 정지하면서 무음 (128)
  if (!playing) {
    // 남은 데이터 없으면 정지
    OCR2A = OCR2B = 128;
    OCR4A = OCR4B = 128;
    playing = false;
    return;
  }

  uint8_t id = curBuf;
  
  // 현재 배열 버퍼가 다 떨어졌으면 다른 버퍼로 변경
  if (bufPos[id] + bytesPerSample > bufLen[id]) {
    bufEmpty[id] = true;
    bufPos[id] = 0;
    // ^= 사용 0 -> 1로 변경
    curBuf ^= 1;
    id = curBuf;

    // 나머지 한개 버퍼도 비어있으면 끝낸다
    if (bufEmpty[id] || bufPos[id] + bytesPerSample > bufLen[id]) {
      OCR2A = OCR2B = 128;
      OCR4A = OCR4B = 128;
      return;
    }
  }

  int16_t sampleL16 = 0;
  int16_t sampleR16 = 0;

  // 8 bit 16 bit mono stereo 모두 처리한다
  if (wavInfo.bitsPerSample == 8) {
    // 8bit는 0~255
    uint8_t rawLeft = buf[id][bufPos[id]++];
    // 16bit로 확장
    int16_t tmpLeft = (((int)rawLeft - 128) * 256);
    
    if (wavInfo.numChannels == 1) {
      // mono = 양쪽 같은 소리
      sampleL16 = tmpLeft;
      sampleR16 = tmpLeft;
    } else {
      // stereo = l, r따로
      uint8_t rawRight = buf[id][bufPos[id]++];
      int16_t tmpRight = (((int)rawRight - 128) * 256);

      sampleL16 = tmpLeft;
      sampleR16 = tmpRight;
    }
  } else if (wavInfo.bitsPerSample == 16) {
    // 리틀 앤디안 사용해서 가장 낮은 비트 먼저 구현
    uint8_t b0 = buf[id][bufPos[id]++];
    uint8_t b1 = buf[id][bufPos[id]++];
    // 리틀 앤디안을 사용해서 b0랑 b1을 하나의 16비트로 구성한다.
    // 예를 들어 b0 = 0x34 이면 b1 = 0x12 최종 값은 
    // b1(0x0034) b0(0x1200) 0x1234로 구성된다.
    int16_t tmpLeft = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));

    if (wavInfo.numChannels == 1) {
      // mono는 양쪽 같은소리
      sampleL16 = tmpLeft;
      sampleR16 = tmpLeft;
    } else {
      // stereo L(2바이트), R(2바이트)로 나눠서
      uint8_t b2 = buf[id][bufPos[id]++];
      uint8_t b3 = buf[id][bufPos[id]++];
      int16_t tmpRight = (int16_t)((uint16_t)b2 | ((uint16_t)b3 << 8));

      sampleL16 = tmpLeft;
      sampleR16 = tmpRight;
    }
  } else {
    // 그 외의 경우 무음
    OCR2A = OCR2B = 128;
    OCR4A = OCR4B = 128;
    playing = false;
    return;
  }

  // 강의 노트대로 16bit signed를 unsigned로 변경 0~65535
  uint16_t uL = (uint16_t)(sampleL16 + 32768);
  uint16_t uR = (uint16_t)(sampleR16 + 32768);

  // 상위 8비트, 하위 8비트로 H/L 나누기
  uint8_t HighLeft = (uint8_t)(uL >> 8);
  uint8_t LowLeft = (uint8_t)(uL & 0xFF);

  uint8_t HighRight = (uint8_t)(uR >> 8);
  uint8_t LowRight = (uint8_t)(uR & 0xFF);

  // PWM 출력
  // LEFT: Timer2
  OCR2A = HighLeft;
  OCR2B = LowLeft;

  // RIGHT: Timer4
  OCR4A = HighRight;
  OCR4B = LowRight;

  // 샘플 카운트
  uint8_t frame = 1;

  // 44.1kHz 16bit면 샘플 하나 더 건너뛴다
  if (rate44k) {
    // 버퍼 범위 안에서만 추가로 스킵
    if (bufPos[id] + bytesPerSample <= bufLen[id]) {
      // 샘플 1프레임 더 건너뜀(L R 한쌍으로)
      bufPos[id] += bytesPerSample;
      // 총 2프레임 소비했다고 카운트
      frame = 2;
    }
  }

  played += frame;
  if (totalSample != 0 && played > totalSample) {
    played = totalSample;
  }

  // 이 버퍼도 다 썼으면 empty 표시 및 다음 버퍼로 넘어갈 준비
  if (bufPos[id] + bytesPerSample > bufLen[id]) {
    bufEmpty[id] = true;
    bufPos[id] = 0;
    curBuf ^= 1;
  }
}


// ==============================================================================

// ======================== External Interrupt part =============================
bool Rotary::start(){
  return true;
}
// 스위치 눌림
// 메인 loop에서 handleRotary()가 처리할 수 있도록 flag만
ISR(INT1_vect) {
  // 메인 loop에서 처리 하도록 flag만
  rotary_pressed = true;
}
ISR(INT3_vect) {
  // 로터리 회전 처리 
  // PD2의 현재 사애는 PIND 레지스터 2번 핀에 들어잇음
  if (PIND & _BV(PD2)){
    rotary_cnt--; // 시계 방향
  } else {
    rotary_cnt++; // 반시계 방향
  }
}

// INT3를 이용
void Rotary::ExtInterruptInit(){
  // PD3, PD2, PD1을 입력으로 설정
  DDRD &= ~_BV(DDD3);
  DDRD &= ~_BV(DDD2);
  DDRD &= ~_BV(DDD1);

  // 풀업, 입력핀 풀업으로, rotary encoder가 ground로 갈때 low로 떨어지도록
  // 그래서 PORTD를 건드린다.
  // DDR은 출력이지만 PORT가 1이면 high라 보내고 0이면 low면 끊는다
  PORTD |= _BV(PORTD3);
  PORTD |= _BV(PORTD2);
  PORTD |= _BV(PORTD1);

  // 1 0 으로 falling edge 사용
  // High -> low로 떨어질 때 INT3 발생
  // a가 떨어질 때, 그 순간의 b가 1 이면 cw
  // a가 edge, 그 순간 b가 0이면 ccw
  EICRA &= ~_BV(ISC30);
  EICRA |= _BV(ISC31);

  // INT1 (PD1)
  EICRA &= ~_BV(ISC10);
  EICRA |= _BV(ISC11);

  // interrupt flag 제거
  EIFR |= _BV(INTF3);
  EIFR |= _BV(INTF1);
  
  // INT3, INT1 interrrupt 활성화
  EIMSK |= _BV(INT3);
  EIMSK |= _BV(INT1);
  
}
// 로터리 회전 업데이트 함수
void updateRotary() {
  // 회전 변화량이 없다.
  if (rotary_cnt == before_cnt) return;
  int16_t diff = rotary_cnt - before_cnt;
  before_cnt = rotary_cnt;
  
  // sd 카드 읽어서 값이 변할때만 화면 변하게 하기위해
  bool changed = false;

  // 장르 선택 화면
  if (ui == 0)  {
    // 방향에 따라 하나씩만 이동
    if (diff < 0 && genre_idx < genre_cnt - 1) {
      genre_idx++;
      changed = true;
    } else if (diff > 0 && genre_idx > 0) {
      genre_idx--;
      changed = true;
    }
  }
  if (ui == 1)  {
    // 재생 중일 때는 회전 무시해야하니깐
    if (playing) return ;

    // [<-] 위해서 배열 크기 조정
    if (diff < 0 && sing_idx < sing_cnt) {
      sing_idx++;
      changed = true;
    } else if (diff > 0 && sing_idx > 0) {
      sing_idx--;
      changed = true;
    }
  }

  // 인덱스 바뀐 경우만 화면 변경
  if (changed) {
    lcd.clearLCD();
  }
}

// 로터리 눌리면 ui 변경하고 곡 선택 처리
void handleRotary() {
  // 아직 INT1 interrupt에서 버튼 눌림이 들어오지 않으면 반환
  if (!rotary_pressed) return;

  // 한번 처리했으니 버튼 눌림 플래그 클리어
  rotary_pressed = false;
  
  // ui == 0이니깐 장르 선택 화면
  if (ui == 0) {
    // 현재 선택된 genre_idx에 대해 곡 목록 로드
    sdcard.loadSong(genre_idx);
    // 곡 인덱스 0 번부터
    sing_idx = 0;
    // 곡 선택 화면으로 변환
    ui = 1;
  }
  else if (ui == 1) {
    // sing_idx == sing_cnt 면 곡 리스트의 마지막 뒤에 있는 [<-] 항목 선택한 상태
    if (sing_idx == sing_cnt) {
      // 재생 중이면 곡 정지 후 초기화
      if (playing) {
        sdcard.stopplay();
      }
      // UI를 다시 장르 선택 화면으로
      ui = 0;
      // 곡 인덱스 0으로
      sing_idx = 0;
    } else {
      if (!playing) {
        // 이미 같은 곡 열어놨고 현재 선택된 장르 곡과 재생 중인 곡이 같으며
        // 중간에 틀고 다른 곡을 틀지 않고 끝까지 안들은 상태면 다시 이어서 재생하는 로직
        if (wavFile.isOpen() && cur_genre == genre_idx && cur_song == sing_idx
          && played > 0 && played < totalSample) {
            // 이어서 재생
            playing = true;
          }  else {
          // 다른 곡 선택했거나 처음 재생하면 처음부터
          sdcard.startplay(genre_idx, sing_idx);
        }
      } else {
        // 재생 중에 버튼 눌릴경우
        playing = false; // 정지
        // 뮤트
        OCR2A = OCR2B = 128;
        OCR4A = OCR4B = 128;
      }
    }
  }

  // ui 나 다른 변수들이 변경됐으니 그것에 맞춰 LCD clear
  lcd.clearLCD();
 
}

//=======================Sdcard N wav player===================

bool Sdcard::check_wav(const char *name) {
  int len = strlen(name);
  
  // .wav만 체크하고 아니면 false 반환
  if (len > 4 &&  name[len-4] == '.' &&  name[len-3] == 'w' &&  name[len-2] == 'a'
      &&  name[len-1]== 'v') {
        return true;
  }
  else {
    return false;
  }
}

uint8_t Sdcard::genreCount(){
  // root는 public으로 선언, SD 카드의 루트 디렉토리 연다
  if (!root.open("/")) {
    return 0;
  }

  uint8_t cnt = 0;
  // dir 도 public으로 선언
  while (dir.openNext(&root, O_READ)) {
    // 디렉토리가 아니면 닫고 continue로 while문 반복
    if (!dir.isDir()) {
      dir.close();
      continue;
    }
    // 디렉토리면 wav 있는지 확인
    bool check = false;
    while (file.openNext(&dir, O_READ)) {
      if (file.isFile()) {
        // 여유있게 크기 40으로 배열
        char name[40];
        file.getName(name, sizeof(name));
        // .wav 파일 하나 있으면 장르로
        if (check_wav(name)) {
          check = true;
        }
      }
      file.close();
      if (check) {break;}
    }

    dir.close();
    // 유효한 장르만 카운트 증가
    if (check) {
      cnt ++;
    }
  }

  root.close();
  return cnt;
}

// idx 번째 장르 이름 가져오기
bool Sdcard::genreName(uint8_t idx, char *output, uint8_t maxLen){
  output[0] = '\0';

  if (!root.open("/")) {
    return false; // bool타입이니깐
  }

  // 유효한 장르 세기 위한 tmp
  uint8_t tindex = 0;
  // 원하는 인덱스면 true로 
  bool correct = false;
  while (dir.openNext(&root, O_READ)) {
    // 디렉토리가 아니면 닫고 continue로 while문 반복
    if (!dir.isDir()) {
      dir.close();
      continue;
    }

    // wav파일 있는지
    bool check = false;
    while (file.openNext(&dir, O_READ)) {
      if (file.isFile()) {
        // 여유있게 크기 40으로 배열
        char name[40];
        file.getName(name, sizeof(name));
        if (check_wav(name)) {
          check = true;
        }
      }
      file.close();
      if (check) {break;}
    }
    
    // wav 파일 없으면 close()
    if (!check) {
      dir.close();
      continue;
    }

    // 장르 하나 발견 된 상황 idx가 찾고자 하는 idx값
    if (tindex == idx){
      char gname[40];
      dir.getName(gname, sizeof(gname));
      // 원본 gname을 output에 maxLen-1 길이 만큼 복사
      strncpy(output, gname, maxLen - 1);
      // 끝임을 인식하기 위해 \0 추가
      output[maxLen - 1] = '\0';
      correct = true;
      dir.close();
      break;
    }

    // 다음 인덱스 탐색
    tindex++;
    dir.close();
  }

  root.close();
  // correct 반환한다.
  return correct;
}

// 장르 곡 개수 체크
uint8_t Sdcard::songcount(uint8_t gidx) {
  // 장르 이름
  char gname[40];
  // 장르 이름 가져오는 함수 실행해서 false 면 uint8_t니깐 0 반환
  if (!genreName(gidx, gname, sizeof(gname))) {
    return 0;
  } 

  // 장르 오픈 실패시 0
  if (!dir.open(gname, O_READ)) {
    return 0;
  }

  uint8_t cnt = 0;
  // dir 안 순차적으로 탐색하면서 .wav 갯수 카운팅
  while (file.openNext(&dir, O_READ)) {
    if (!file.isFile()) {
      file.close();
      continue;
    }

    // 파일명
    char fname[40];
    file.getName(fname, sizeof(fname));
    if (check_wav(fname)) {
      cnt++;
    }
    file.close();
  }

  dir.close();
  return cnt;
}

bool Sdcard::songfromIdx(uint8_t gidx, uint8_t sidx, char *output, uint8_t maxLen){
  // 기본값으로 빈 문자열 초기화
  output[0] = '\0';

  char gname[40];
  // gidx에 해당하는 장르 이름 못 얻으면 false반환
  if(!genreName(gidx, gname, sizeof(gname))) {
    return false;
  }

  // 디렉토리 오픈 실패시 false
  if(!dir.open(gname, O_READ)) {
    Serial.println("dir open fail");
    return false;
  }

  // wav파일 중 몇번 째인지
  uint8_t idx = 0;

  // 장르 디렉토리 안에서 wav 파일만 세면서 sidx 번쨰 찾기
  bool check = false;
  while (file.openNext(&dir, O_READ)) {
    if (!file.isFile()) {
      file.close();
      continue;
    }

    char name[40];
    file.getName(name, sizeof(name));
    // wav가 아니면 무시한다
    if (!check_wav(name)) {
      file.close();
      continue;
    }

    // 현재 idx가 찾는 sidx면 이름 output에 복사
    if (idx == sidx) {
      strncpy(output, name, maxLen - 1);
      // 끝임을 인식하기 위해 \0 추가
      output[maxLen - 1] = '\0';
      check = true;
      file.close();
      break;
    }

    idx++;
    file.close();
  }
  dir.close();
  return check;
}

// SD 카드 전체를 스캔해 장르 개수 곡 개수 초기화
void Sdcard::scan() {
  // 루트 디렉토리에서 장르 개수 세서 genre_cnt에 저장
  genre_cnt = genreCount();

  // 장르 인덱스
  genre_idx = 0;
  // 노래 인덱스
  sing_idx = 0;

  // 장르가 없으면 0으로 초기화
  if(genre_cnt == 0) {
    sing_cnt = 0;
    return;
  }

  // 첫 장르의 곡 개수 확인 genre_idx를 전역에서 int8_t로 선언했으니
  sing_cnt = songcount((uint8_t) genre_idx);
}

// 장르 바뀔때 호출되는 loadSong
void Sdcard::loadSong(uint8_t gidx) {
  // 장르 없으면 0 으로 다 초기화 후 리턴
  if (genre_cnt == 0) {
    sing_cnt = 0;
    sing_idx = 0;
    return;
  }

  // gidx가 범위를 벗어나면 인덱스 보정
  if (gidx >= genre_cnt) {
    gidx = genre_cnt - 1;
  }

  // 장르곡개수 계산
  sing_cnt = songcount(gidx);

  // 곡 없으면 sing_cnt =0 유지
  if (sing_cnt == 0) {
    sing_cnt = 0;
  } // 이전에 선택됐던 곡이 인덱스 범위 벗어나면 0 리셋
  else if (sing_idx >= sing_cnt) {
    sing_idx = 0;
  }
}

void Sdcard::startplay(int8_t gidx, int8_t sidx) {
  // 재생 중이면 노래 못 바꾸니깐 예외 처리
  if (playing) {
    return;
  }

  // 장르 없으면 리턴
  if (genre_cnt == 0) {return;}

  // 장르 인덱스 범위 보정
  if (gidx < 0) gidx = 0;
  if ((uint8_t)gidx >= genre_cnt) gidx = genre_cnt - 1;
  
  // 곡 인덱스 범위 보정
  if (sidx < 0) sidx = 0;
  if ((uint8_t)sidx >= sing_cnt) sidx = sing_cnt - 1;
  
  char gname[40];
  char sname[40];

  // 장르 이름 초기화
  if (!genreName((uint8_t)gidx, gname, sizeof(gname))) {
    return;
  }

  // 곡 이름 초기화
  if (!songfromIdx((uint8_t)gidx, (uint8_t)sidx, sname, sizeof(sname))) {
    return;
  }


  // 이미 열려있느면 닫고 다시
  if (wavFile.isOpen()) {
    wavFile.close();
  }

  // 장르 디렉토리 먼저 연다
  if (!dir.open(gname, O_READ)) {
    playing = false;
    return;
  }

  // 그 디렉토리 안에서 곡 파일을 연다
  if (!wavFile.open(&dir, sname, O_READ)) {
    dir.close();
    playing = false;
    return;
  }
  dir.close();

  // WAV 헤더 파싱 (RIFF, fmt, data etc)
  // 44100인지 22050
  if (!wavHeader(wavFile)) {
    Serial.println("2 fail");
    wavFile.close();
    playing = false;
    return;
  }
  
  // SampleRate확인해서 Timer1 설정
  audioTimer();

  // 진행도 관련변수 다 초기화
  played = 0;
  progress10 = 0;
  progress = 0;


  // 버퍼 초기화
  curBuf = 0;
  bufPos[0] = bufPos[1]= 0;
  bufLen[0] = bufLen[1] = 0;
  bufEmpty[0] = bufEmpty[1] = true;

  // 0번 1번 미리 채우기
  fillBuffer(0);
  fillBuffer(1);

  // pwm 중간값으로 설정 무음
  OCR2A = OCR2B = 128;
  OCR4A = OCR4B = 128;

  delay(250);

  TimerInit();

  playing = true;

  // 현재 재생중인 노래 업데이트
  cur_genre = gidx;
  cur_song = sidx;
  genre_idx = gidx;
  sing_idx = sidx;

  lcd.printTitle();
  lcd.printPlayORStop();

}

void Sdcard::stopplay() {
  if (wavFile.isOpen()) {
    wavFile.close();
  }
  
  playing = false;

  // 재생 중인 곡 정보 초기화
  cur_genre = -1;
  cur_song = -1;
  // 중앙값 맞춰서 mute
  OCR2A = 128;
  OCR2B = 128;
  OCR4A = 128;
  OCR4B = 128;

  lcd.printPlayORStop();
}

bool wavHeader(SdFile &file) {
  //웨이브파일 헤더 파싱
  char id[4];
  uint32_t size32;
  uint16_t size16;
  
  // 1. RIFF
  if (file.read(id, 4) != 4) return false;
  //id에 읽어온 4바이트가 RIFF랑 같은지 확인 strncmp를 사용해 비교했다.
  if (strncmp(id, "RIFF", 4) != 0) return false;

  // chunkSize는 건너뛰고
  file.read(&size32, 4);

  // 2. WAV
  if (file.read(id, 4) != 4) return false;
  //id에 읽어온 4바이트가 RIFF랑 같은지 확인 strncmp를 사용해 비교했다.
  if (strncmp(id, "WAVE", 4) != 0) return false;

  // 3. fmt 서브청크
  // "fmt " 로 공백 포함 4
  if (file.read(id, 4) != 4) return false; 
  //id에 읽어온 4바이트가 RIFF랑 같은지 확인 strncmp를 사용해 비교했다.
  if (strncmp(id, "fmt ", 4) != 0) return false;

  // Subchunk1Size
  file.read(&size32, 4);

  // AudioFormat
  file.read(&size16, 2);
  if (size16 != 1) return false;

  // Numchannels (2)
  file.read(&wavInfo.numChannels, 2);
  
  // SampleRate
  file.read(&wavInfo.sampleRate, 4);

  // ByteRate
  uint32_t byteRate;
  file.read(&byteRate, 4);

  // BlockAlign
  file.read(&size16, 2);

  // bitsPerSample
  file.read(&wavInfo.bitsPerSample, 2);

  // fmt 서브청크가 16보다 크면 나머지는 스킵한다.
  if (size32 > 16) {
    uint32_t skip = size32 - 16;
    while (skip--) file.read(&id[0], 1);
  }

  // data 청크 찾는다
  while (true) {
    // Subchunk ID
    if (file.read(id, 4) != 4) return false;
    // Subchunk Size
    file.read(&size32, 4);

    if (strncmp(id, "data", 4) == 0) {
      // data 찾음
      wavInfo.dataSize = size32;
      // 지금 파일 위치가 바로 sample data의 시작 위치이다.
      startPos = file.curPosition();
      break;
    } else {
      // 다른 청크면 건너뛰기 seekCur을 사용
      file.seekCur(size32);
    }
  }
  
  // 전체 샘플 개수 datasize / bytesPerSample(tmp)
  uint8_t tmp = (wavInfo.bitsPerSample / 8) * wavInfo.numChannels;
  bytesPerSample = tmp;

  if (tmp == 0) {
    totalSample = 0;
  }
  else {
    totalSample = wavInfo.dataSize / tmp;
  }

  return true;
}

// id는 어느 버퍼를 채울지 정한다. raw 바이트를 읽고 해석은 Interrupt에서
bool fillBuffer(uint8_t id) {

  // 이미 데이터가 남아있으면 더 읽지 않는다.
  if (!bufEmpty[id] && bufLen[id] > 0) return true;

  // 새로 채울 길이 초기화
  bufLen[id] = 0;
  bufPos[id] = 0;
  
  // raw 데이터 읽기 SD카드에서 BUF_SIZE 만큼 raw 데이터 읽어서 buf[id]에 저장
  int n = wavFile.read(buf[id], BUF_SIZE);

  // 더 이상 읽을 데이터 없으면 버퍼는 비어있다고 표시하고 false
  if (n <= 0) {
    bufLen[id] = 0;
    bufEmpty[id] = true;
    return false;
  }

  // 처음 재생 했을때 잡음이 들리는 경우가 있는데, 그 경우를 대비해
  // 로직 추가, 온전한 샘플만 담는다.
  if (bytesPerSample != 0) {
    int rem = n % bytesPerSample;
    if (rem != 0) {
      n -= rem;
    }
  }

  // 최종적으로 버퍼에 유효한 데이터 길이 저장
  bufLen[id] = (uint16_t)n;
  // 이제 데이터가 있으니 empty = false
  bufEmpty[id] = false;

  return true;
}

// sampleRate를 파악하고 그 sample
void audioTimer() {
  // 매번 초기화 하고
  rate44k = false;

  if (wavInfo.sampleRate >= 40000) {
      // 44.1 khz면 half곱해서 절반으로 처리
      // 즉 22.05 타이머 처럼 사용하고 샘플을 반만사용한다.
      // 이 것을 사용한 이유는 44.1khz timer를 구현했을때 부하가 발생해
      // 잡음이 심하고 delay되는 문제가 발생했다.
      timer_setting_22_05KHZ();
      rate44k = true;
  } else if (wavInfo.sampleRate >= 20000) {
    timer_setting_22_05KHZ();
  } else {
    playing = false;
  }
}

Rotary rotary;

void setup() {
  Serial.begin(115200);

  //SS(53) 출력
  DDRB |= _BV(DDB0);
  // 기본 HIGH로
  PORTB |= _BV(PORTB0);

  // SD 카드 초기화 초기화 실패하면 메시지 출력 후 무한루프 정지
  if (!sd.begin(SD_CS_PIN)) {
    Serial.println("failed");
    Serial.println(SD_CS_PIN);

    while(1);
  }

  // sdcard 스캔해서 장르 개수 장르 곡 인덱스 변수 초기화
  sdcard.scan();

  // 처음엔 0번 장르
  genre_idx = 0;
  // 곡 목록 채우기
  sdcard.loadSong(genre_idx);
  // 곡 인덱스 0번부터
  sing_idx = 0;
  
  // lcd 및 led 초기화
  lcd.start();
  led.start();

  // L R 채널 PWM 출력용 Timer2 4 초기화
  timer2_setting();
  timer4_setting();

  // 로터리 엔코더 external interrupt 설정
  rotary.ExtInterruptInit();
  
  // 앞 뒤로 스킵 스위치 초기화
  skipSwInit();

  // interrupt 가능하게.
  sei();

  // 현재 ui 상태 장르 or 곡 화면에 따른 LCD 화면 초기화
  lcd.clearLCD();
}

// 곡 진행률 % 계산해 LCD에 표시
void updateprogress(){
  static uint32_t last = 0;
  // 재생 중이 아니면 진행률 갱신필요없음
  if (!playing) return;

  uint32_t now = millis();
  // 200ms 마다 한번만갱신 LCD가 많이 깜빡이는 문제가 생겨 이를 방지하기 위해
  // 추가한 로직
  if (now - last < 200) {return;}
  last = now;


  // 샘플 개수 모르면 계산 불가
  if (totalSample == 0) {return;}

  uint32_t s = played;
  if (s > totalSample) {s = totalSample;}

  //0.0~100%까지, 64비트로 곱한 이유는 44khz 처리할때 40%까지 밖에 처리를 못하기때문
  uint16_t p = (uint16_t)((1000 * (uint64_t)s) / totalSample);
  if (p > 1000) p = 1000;

  // 100% 되면 stopped으로 변경
  if (p == 1000 && playing){
      if (sing_idx + 1 >= sing_cnt){
        sdcard.stopplay();
    } 
  }

  // 소수 한자리 까지 표현 (0~1000)
  progress10 = p;
  // 정수 %
  progress = p / 10;

  // LCD 2행 끝에 출력
  lcd.printpercentage();
}

// 100% 까지 다 재생하면 자동으로 다음곡 재생
void playNext() {
  // 우선 멈추고
  sdcard.stopplay();

  // 그 다음 곡이 있으면 넘어가고 없으면 그냥 playing false 상태 유지
  if (sing_idx + 1 < sing_cnt) {
    sing_idx++;
    // 곡 제목 ui 갱신
    lcd.clearLCD();
    // 같은 장르 다음곡을 재생한다 다음곡이 있을 경우
    sdcard.startplay(genre_idx, sing_idx);
  }
}

void loop() {

  // skip 시간 및 led 업데이트 
  led.skip();

  // 로터리 처리
  updateRotary();
  handleRotary();

  // 진행도 업데이트
  updateprogress();

  // sw1 눌리면 뒤로, sw2는 앞으로 둘다 재생일 때만 작동 
  if (sw1){
    sw1 = false;
    if (playing) {
      skipB();
    }
  }
  if (sw2){
    sw2 = false;
    if (playing) {
      skipF();
    }
  }
  // 버퍼 채우기
  if (playing) {
    // 1이면 0으로
    uint8_t filled = curBuf ^ 1;

    if (bufEmpty[filled]) {
      // 파일 끝까지 읽엇으면
      if (!fillBuffer(filled)) {
        playNext();
      }
    }
  }
}