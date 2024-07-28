#ifndef _MS_COMM_H_
#define _MS_COMM_H_

void jumpToApp(int addr);
void lega_memset(char *buf,char value, int size);
void lega_memcpy(char *dst, const char *src, int size);
int lega_memcmp(const char *str1, const char *str2, int size);
int lega_strcmp(const char *str1, const char *str2);
void delay(unsigned int cycles);
void udelay(unsigned int us);
int convert_str_to_int(char *str);
void convert_int_to_str(unsigned int val, unsigned int type, char *ch);
void roll_index(volatile unsigned char *idx, int size);
unsigned long AlignWord(unsigned char * pucData);
/// Macro to read a register
#define REG_RD(addr)              (*(volatile unsigned int *)(addr))
/// Macro to write a register
#define REG_WR(addr, value)       (*(volatile unsigned int *)(addr)) = (value)

void lega_write32_bit(unsigned int reg, unsigned char start_bit, unsigned char len, unsigned int src_val);
unsigned int lega_read32_bit(unsigned int reg, unsigned char start_bit, unsigned char len);
#endif //_MS_COMM_H_