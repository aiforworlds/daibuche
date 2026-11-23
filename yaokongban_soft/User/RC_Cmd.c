/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "stm32f10x.h"                  // Device header
#include "RC_Cmd.h"
#include "Delay.h"
#include "APPTime.h"

uint8_t Cmd_Arr[18];
char Cmd_String[40];

void RC_Pin_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1; // 设置为010，禁用JTAG，保留SWD（PB4引脚默认为NJTRST引脚，不是GPIO）
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;		//PD & SET
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;				//AUX
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB,&GPIO_Initstructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_4);										//PD低电平工作
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);										//SET低电平设置模式，高电平通信模式
}

//将字符串转换成byte数组：
//参数：字符串，输出数组地址，数组的长度
//用法：hexStringToByteArray("AA5A22331122000000040064000000120006",Cmd_Arr,18);
void hexStringToByteArray(char *hexString, uint8_t *byteArray, size_t arraySize)
{
	size_t hexStringLength = strlen(hexString);
	if (hexStringLength / 2 > arraySize) {
		Serial_Printf("Error: Hex string is too long for the provided array.\n");
		return;
	}

	for (size_t i = 0; i < hexStringLength; i += 2) {
		sscanf(hexString + i, "%2hhX", &byteArray[i / 2]);
	}
}
//将byte数组转换成字符串：
//参数：数组，数组的长度，输出字符串数组
//用法：byteArrayToHexString(Cmd_Arr,18,Cmd_String);
void byteArrayToHexString(const uint8_t *byteArray, size_t arraySize, char *hexString)
{
	if (hexString == NULL) {
		Serial_Printf("Error: Output string is NULL.\n");
	return;
	}
	// 每个字节将被转换为两个十六进制字符，加上字符串结束符 '\0'
	size_t hexStringLength = arraySize * 2 + 1;
	for (size_t i = 0; i < arraySize; i++) {
		// 使用 sprintf 来格式化单个字节为两个十六进制字符
		sprintf(hexString + i * 2, "%02X", byteArray[i]);
	}
	// 确保字符串以 null 结尾
	hexString[hexStringLength - 1] = '\0';
}

//遥控模块命令：
void RC_cmd(uint8_t choice)
{
	for (uint8_t i; i < 18; i++)        //清零数组
	{
		Cmd_Arr[i] = 0;
	}
	for (uint8_t i; i < 18; i++)        //清零数组
	{
		Cmd_String[i] = '\0';
	}

	switch (choice)
	{
		case 1:     //发送初始化命令
		{
			/*
					《命令构成》
			 AA5A   ｜ 2233  ｜ 1122  ｜  00   ｜  00    ｜  00  ｜   04   ｜  00   ｜ 64  ｜  00   ｜   00    ｜   00  ｜    12   ｜   00  ｜  06
			固定开头 ｜ 模块ID ｜ 组网ID ｜ 分隔位 ｜ RF功率 ｜ 分隔位 ｜ 波特率 ｜ 分隔位 ｜ 信道 ｜ 保留位 ｜ 空中速率  ｜ 保留位 ｜ 命令长度 ｜ 保留位 ｜ 校验

			RF功率：
				0-14 越小功率越大
			波特率：
				0	600bps
				1	1200bps
				2	2400bps
				3	4800bps
				4	9600bps
				5	19200bps
				6	38400bps
			信道：
				0-127
			空中速率：
				0-2 越大速率越高
			校验：
				以上所有字节相加保留低八位

			*/

			hexStringToByteArray("AA5A22331122000000040064000000120006",Cmd_Arr,18);
			Serial_3_SendArray(Cmd_Arr,18);
			break;
		}
		case 2:		//发送配置命令
		{
			Serial_3_SetBaudRate(9600);
			Delay_ms(20);
			hexStringToByteArray("AA5A060814570000000500630002001200F9",Cmd_Arr,18);
			Serial_3_SendArray(Cmd_Arr,18);
			Delay_ms(20);

			Serial_3_SetBaudRate(19200);
			break;
		}
		default:
		{

		}

	}

}

void RC_setSysInit(void)
{
	RC_Pin_Init();
	// Delay_ms(20);
	// RC_cmd(2);
	// Delay_ms(20);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);	//将模块设置为工作模式
}

/**
  * @brief  计算8位CRC校验值
  * @param  data: 数据指针
  * @param  length: 数据长度
  * @retval 计算得到的CRC值
  */
 uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length)
 {
	 uint8_t crc = 0x00; // 初始值
	 uint8_t poly = 0x31; // CRC-8/MAXIM多项式

	 while (length--)
	 {
		 crc ^= *data++; // 与数据异或

		 for (uint8_t i = 0; i < 8; i++)
		 {
			 if (crc & 0x80) {
				 crc = (crc << 1) ^ poly;
			 }
			 else {
				 crc <<= 1;
			 }
		 }
	 }

	 return crc;
 }

void RC_send(const uint8_t *data, uint8_t cmd_id, size_t data_size)
{
    // 计算总包长度 = 包头(1) + timestamp(4) + cmd_id(1) + 数据(data_size) + 包尾(1) + CRC(1)
    const size_t header_size = sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint8_t);
    const size_t total_size = header_size + data_size + 2; // +2表示包尾和CRC

    // 创建发送缓冲区（使用栈内存，建议最大包长不超过256字节）
    uint8_t send_buf[SERIAL_3_TX_BUFFER_SIZE];
    if(total_size > sizeof(send_buf)) {
        // 处理错误：数据过长
        return;
    }

    // 填充包头部分
    RC_pack_t *pack = (RC_pack_t *)send_buf;
    pack->start_flag = 0x5B;
	pack->timestamp = NTP_time;
    pack->cmd_id = cmd_id;

    // 填充数据部分
    memcpy(send_buf + header_size, data, data_size);

    // 添加包尾
    size_t end_pos = header_size + data_size;
    send_buf[end_pos] = 0x5D;

    // 计算CRC（包头+数据+包尾）
    uint8_t crc = Calculate_CRC8(send_buf, end_pos + 1);
    send_buf[end_pos + 1] = crc;

    // 发送完整数据包
    Serial_3_SendArray(send_buf, total_size);
}

// ******************* log报错码 *******************

static log_t Log[MAX_LOG_COUNT] = {0};
static uint16_t log_index = 0;      // 当前日志索引
static uint16_t log_count = 0;      // 当前日志数量

void Log_add(uint8_t error_number, const char *message)
{
    // 确保message不为NULL
    if (message == NULL) {
        message = "";
    }

    // 添加日志
    Log[log_index].app_time = APP_time;
    Log[log_index].num = error_number;

    // 安全拷贝字符串，防止溢出
    strncpy(Log[log_index].message, message, MAX_LOG_MESSAGE - 1);
    Log[log_index].message[MAX_LOG_MESSAGE - 1] = '\0'; // 确保字符串终止

    // 更新索引和计数
    log_index = (log_index + 1) % MAX_LOG_COUNT;
    if (log_count < MAX_LOG_COUNT) {
        log_count++;
    }
}

void Log_print(void)
{
    uint16_t start_index;
    uint16_t i;

    if (log_count == 0) {
        Serial_Printf("No logs available.\r\n");
        return;
    }

    // 计算起始索引（如果是循环缓冲区）
    if (log_count < MAX_LOG_COUNT) {
        start_index = 0;
    } else {
        start_index = log_index;
    }

    // 打印所有日志
    for (i = 0; i < log_count; i++) {
        uint16_t idx = (start_index + i) % MAX_LOG_COUNT;
        Serial_Printf("[%u] Time: %u, Error: 0x%02X, Message: %s\r\n",
                     i+1, Log[idx].app_time, Log[idx].num, Log[idx].message);
		Delay_ms(10);
    }
}

void Log_reset(void)
{
    Serial_Printf("Logs reset. Current saved logs:\r\n");
	Log_print();
	memset(Log, 0, sizeof(Log));
    log_index = 0;
    log_count = 0;
}
