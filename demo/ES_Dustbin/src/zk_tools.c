/*
 * �����ַ������Ƿ���ָ�����ַ���
 * ��Σ�	*str_source Դ�ַ���
 * 		len_s Դ�ַ�������
 * 		*str_starget �����ҵ�Ŀ���ַ���
 * 		len_t Ŀ���ַ�������
 * ���أ�	0 ��û�ҵ�
 * 		�����������ҵ�Ŀ���ַ�����Դ�ַ����е�ƫ����+1
 */
unsigned int zk_str_findstr(unsigned char *str_source, unsigned int len_s,
		unsigned char *str_target, unsigned int len_t) {
	unsigned int i, j;
	unsigned char flag;
	if (len_s <= len_t)
		return 0;
	for (i = 0; i < len_s - len_t; i++) {
		flag = 0;
		for (j = 0; j < len_t; j++)
			if (*(str_source + i + j) != *(str_target + j)) {
				flag = 1;
				j = len_t;
			}
		if (flag == 0)
			return i + 1;
	}
	return 0;
}

unsigned char ASC_HEX(unsigned char IN_ASC) {
	switch (IN_ASC) {
	case '0':
		return 0;
	case '1':
		return 1;
	case '2':
		return 2;
	case '3':
		return 3;
	case '4':
		return 4;
	case '5':
		return 5;
	case '6':
		return 6;
	case '7':
		return 7;
	case '8':
		return 8;
	case '9':
		return 9;
	case 'A':
		return 10;
	case 'B':
		return 11;
	case 'C':
		return 12;
	case 'D':
		return 13;
	case 'E':
		return 14;
	case 'F':
		return 15;
	case 'a':
		return 10;
	case 'b':
		return 11;
	case 'c':
		return 12;
	case 'd':
		return 13;
	case 'e':
		return 14;
	case 'f':
		return 15;
	default:
		return 0;
	}

}

unsigned char HEX_ASC(unsigned char IN_HEX) {
	switch (IN_HEX) {
	case 0x00:
		return '0';
	case 0x01:
		return '1';
	case 0x02:
		return '2';
	case 0x03:
		return '3';
	case 0x04:
		return '4';
	case 0x05:
		return '5';
	case 0x06:
		return '6';
	case 0x07:
		return '7';
	case 0x08:
		return '8';
	case 0x09:
		return '9';
	case 0x0A:
		return 'A';
	case 0x0B:
		return 'B';
	case 0x0C:
		return 'C';
	case 0x0D:
		return 'D';
	case 0x0E:
		return 'E';
	case 0x0F:
		return 'F';
	default:
		return '0';
	}
}

//��ASC��ʽ���ַ���ѹ����HEX
//����
//���Ϊ�ַ�����3A6B2F��
//ѹ����Ϊ0x3A6B2F
//��Σ�*s ��ѹ�����ַ�������������Ϊż���������Լ��Ĺ����㣩		l ��ѹ�����ַ������ȣ�������Ϊż����
//���Σ�*p ѹ��������ݣ������Զ�Ϊl��һ�룩
//���أ���
void Buff_ASC2HEX(unsigned char *s, unsigned char l, unsigned char *p) {
	unsigned char i;
	for (i = 0; i < (l / 2); i++) {
		*(p + i) = ASC_HEX(*(s + i * 2));
		*(p + i) <<= 4;
		*(p + i) += ASC_HEX(*(s + i * 2 + 1));
	}
}

void Buff_ASC2DEC(unsigned char *s, unsigned char l, unsigned char *p) {
	unsigned char i;
	unsigned char result = 0;
	for (i = 0; i < l; i++) {
		result *= 10;
		result += ASC_HEX(*(s + i));
	}
	*p = result;
}

//��HEX�ַ���ת��ASC
void Buff_HEX2ASC(unsigned char *s, unsigned char l, unsigned char *p) {
	unsigned char i;
	for (i = 0; i < l; i++) {
		*(p + i * 2) = HEX_ASC((*(s + i) >> 4));
		*(p + i * 2 + 1) = HEX_ASC((*(s + i) & 0x0F));
	}
}

void zk_CntChkSUM(unsigned char *p, unsigned char l,unsigned char *r) {
	unsigned char i, result = 0;
	unsigned char CheckData[l/2];
	Buff_ASC2HEX(p, l, CheckData);
	for (i = 0; i < l/2; i++)
		result += CheckData[i];
	result ^= 0xFF;
	Buff_HEX2ASC(&result, 1, r);
}

