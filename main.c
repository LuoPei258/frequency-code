#include <reg51.h>
#include <absacc.h>
#include <ctype.h>

sbit LED=P0^0;
sbit RANGE_LED = P0^7;

unsigned char DispBuf[8];
bit SWTR;
bit SWTF;
unsigned int SWTV;

// === T0 溢出计数器 ===
volatile unsigned int T0_Overflow = 0;

void T0INTSVC() interrupt 1
{
    T0_Overflow++;
}

void T1INTSVC() interrupt 3
{
    code unsigned char com[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
    static unsigned char n = 0;
    
    XBYTE[0x7800] = 0xFF;
    XBYTE[0x7801] = ~DispBuf[n];
    XBYTE[0x7800] = ~com[n];
    n = (n + 1) & 0x07;
    
    if ( SWTR )
    {
        if ( --SWTV == 0 ) {SWTF = 1; TR0 = 0;}
    }
}

void DispClear()
{
    unsigned char i;
    for ( i=0; i<8; i++ ) DispBuf[i] = 0x00;
}

void DispChar(unsigned char x, unsigned char c, bit dp)
{
    code unsigned char Tab[] =
    {
        0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
        0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71
    };
    unsigned char t;
    x &= 0x07;
    x = 7 - x;
    
    if ( c == '-' ) t = 0x40;
    else {
        t = toint(c);
        if ( t < 16 ) t = Tab[t];
        else t = 0x00;
    }
    if ( dp ) t |= 0x80;
    else t &= 0x7F;
    
    DispBuf[x] = t;
}

void DispStr(unsigned char x, unsigned char *s)
{
    unsigned char c;
    unsigned char dp = 0;
    for (;;)
    {
        c = *s++;
        dp = 0;
        if ( c == '\0' ) break;
        if ( *s == '.' )
        {
            dp = 1;
            s++;
        }
        DispChar(x++,c,dp);
    }
}

void Delay(unsigned int t)
{
    SWTV = t;
    SWTR = 1;
    while ( !SWTF );
    SWTR = 0;
    SWTF = 0;
}

void IntToStr(unsigned char idata *s, unsigned int c)
{
    code unsigned int Tab[] = {100000,10000, 1000, 100, 10};
    unsigned char i;
    unsigned int t;
    
    for ( i=0; i<5; i++ )
    {
        t = c / Tab[i];
        *s++ = '0' + t;
        c -= t * Tab[i];
    }
    *s++ = '0' + c;
    *s = '\0';
}

// 高频转字符串，只在 is_kHz = 1 时调用
void FreqToStr(unsigned char idata *s, unsigned long f)
{
    unsigned int ip, fp;
    ip = f / 1000;
    fp = f % 1000;
    *s++ = '0' + (ip / 100) % 10;
    *s++ = '0' + (ip / 10) % 10;
    *s++ = '0' + ip % 10;
    *s++ = '.';
    *s++ = '0' + fp / 100;
    *s++ = '0' + (fp / 10) % 10;
    *s++ = '0' + fp % 10;
    *s = '\0';
}

/* ===================== 绝对安全的读 T0 函数 ===================== */
unsigned long Read_T0_Safe()
{
    unsigned char th1, tl, th2;
    unsigned int ovf;
    
    ET0 = 0; 
    ovf = T0_Overflow;
    
    do {
        th1 = TH0; 
        tl = TL0; 
        th2 = TH0;
    } while (th1 != th2);
    
    if (TF0) {
        ovf++;
        TF0 = 0;
    }
    ET0 = 1; 
    
    return ((unsigned long)ovf * 65536) + ((unsigned int)th1 << 8) + tl;
}

/* ===================== 纯粹测频，不判定量程 ===================== */
unsigned long MeasureFreq(void)
{
    unsigned long total_pulses;
    unsigned int val;

    // 1. 先 100ms 粗测
    T0_Overflow = 0;
    TH0 = TL0 = 0; TF0 = 0;
    TR0 = 1;
    Delay(1000); 
    
    val = (TH0 << 8) | TL0;

    // 2. 只要粗测低于 2kHz（val < 200），统统进入 3 秒长闸门，保精度
    if (val < 200) 
    {
        T0_Overflow = 0;
        TH0 = TL0 = 0; TF0 = 0;
        TR0 = 1;
        Delay(30000); // 3 秒闸门
        
        total_pulses = Read_T0_Safe();
        return (total_pulses + 1) / 3; // 精确到 1Hz
    }
    else 
    {
        // 3. >= 2kHz，进入 1 秒闸门
        T0_Overflow = 0;
        TH0 = TL0 = 0; TF0 = 0;
        TR0 = 1;
        Delay(10000); // 1 秒闸门
        
        total_pulses = Read_T0_Safe();
        
        return total_pulses; 
    }
}

void DispInit()
{
    DispClear();
}

void SysInit()
{
    SWTR = 0; SWTF = 0; SWTV = 0;
    T0_Overflow = 0;

    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1 = TL1 = 0x6A;
    ET1 = 1; TR1 = 1; EA = 1;  

    TMOD &= 0xF0;
    TMOD |= 0x05; // T0 计数
    ET0 = 1; 
    
    DispInit();
}

void main()
{
    unsigned long freq;
    unsigned long last_freq = 0;
    unsigned long temp_freq = 0;
    unsigned char is_kHz;
    unsigned char s[8]; 
    
    SysInit();
    RANGE_LED = 1; 
    
    for (;;)
    {
        LED = 0; 
        freq = MeasureFreq(); // 只获取绝对频率
        
        // === 连续确认滤波算法（针对低频防跳动） ===
        // 我们依然对小于 1kHz 的数据进行滤波
        if (freq < 1000) 
        {
            if (freq == last_freq) {
                temp_freq = freq;
            } else if (freq == temp_freq) {
                last_freq = freq;
            } else {
                temp_freq = freq;
            }
        }
        else 
        {
            last_freq = freq;
            temp_freq = freq;
        }
        
        // === 核心判定：在拿到精准值后，再决定量程 ===
        if (last_freq >= 1000) 
        {
            is_kHz = 1;
            RANGE_LED = 0; // 亮灯
            FreqToStr(s, last_freq); // 显示带小数点的格式
        } 
        else 
        {
            is_kHz = 0;
            RANGE_LED = 1; // 灭灯
            IntToStr(s, (unsigned int)last_freq); // 显示纯整数格式
        }
        
        LED = 1; 
        Delay(1000); 
        DispStr(1, s); 
    }
}