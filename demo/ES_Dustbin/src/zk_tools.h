/*
 * �����ַ������Ƿ���ָ�����ַ���
 * ��Σ�	*str_source Դ�ַ���
 * 		len_s Դ�ַ�������
 * 		*str_starget �����ҵ�Ŀ���ַ���
 * 		len_t Ŀ���ַ�������
 * ���أ�	0 ��û�ҵ�
 * 		�����������ҵ�Ŀ���ַ�����Դ�ַ����е�ƫ����+1
 */
extern unsigned int zk_str_findstr(unsigned char *str_source,
		unsigned int len_s, unsigned char *str_target, unsigned int len_t);
extern void Buff_ASC2HEX(unsigned char *s, unsigned char l, unsigned char *p);
extern void Buff_ASC2DEC(unsigned char *s, unsigned char l, unsigned char *p);
extern void Buff_HEX2ASC(unsigned char *s, unsigned char l, unsigned char *p);

extern void zk_CntChkSUM(unsigned char *p, unsigned char l,unsigned char *r);

