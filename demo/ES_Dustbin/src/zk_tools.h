/*
 * 查找字符串中是否有指定的字符串
 * 入参：	*str_source 源字符串
 * 		len_s 源字符串长度
 * 		*str_starget 待查找的目标字符串
 * 		len_t 目标字符串长度
 * 返回：	0 ：没找到
 * 		其它：待查找的目标字符串在源字符串中的偏移量+1
 */
extern unsigned int zk_str_findstr(unsigned char *str_source,
		unsigned int len_s, unsigned char *str_target, unsigned int len_t);
extern void Buff_ASC2HEX(unsigned char *s, unsigned char l, unsigned char *p);
extern void Buff_ASC2DEC(unsigned char *s, unsigned char l, unsigned char *p);
extern void Buff_HEX2ASC(unsigned char *s, unsigned char l, unsigned char *p);

extern void zk_CntChkSUM(unsigned char *p, unsigned char l,unsigned char *r);

