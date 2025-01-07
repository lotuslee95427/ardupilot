/*
  DSM decoder, based on src/modules/px4iofirmware/dsm.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
  基于PX4Firmware中的DSM解码器,由Andrew Tridgell修改用于AP_HAL_*
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>

#include "dsm.h"

#define DSM_FRAME_SIZE		16		/**<DSM帧大小(字节)*/
#define DSM_FRAME_CHANNELS	7		/**<最大支持的DSM通道数*/

static uint64_t dsm_last_frame_time;		/**< 上一帧DSM开始的时间戳 */
static unsigned dsm_channel_shift;			/**< 通道分辨率, 0=未知, 10=10位, 11=11位 */

//#define DEBUG

#ifdef DEBUG
# define debug(fmt, args...)	printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/**
 * 尝试解码单个通道的原始数据
 *
 * DSM*协议没有提供任何显式的帧边界,
 * 所以我们通过帧间延迟来检测DSM帧边界。
 *
 * 最小DSM帧间隔是11ms;以115200bps传输16字节
 * DSM帧传输时间约为1.4ms。
 *
 * 我们期望只在字节到达需要处理时被调用,
 * 如果两次调用间隔超过5ms,
 * 我们读取的第一个字节将是DSM帧的第一个字节。
 *
 * 在DSM帧丢失字节的情况下,这也提供了一定程度的保护。
 * 当然,如果我们不丢失字节会更好...
 *
 * 收到完整的DSM帧后我们尝试解码它
 *
 * @param[in] raw DSM帧中16位原始通道值
 * @param[in] shift 原始数据中通道号的位置
 * @param[out] channel 返回的通道号指针
 * @param[out] value 返回的通道值指针
 * @return true=原始值成功解码
 */
static bool
dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff)
		return false;

	*channel = (raw >> shift) & 0xf;

	uint16_t data_mask = (1 << shift) - 1;
	*value = raw & data_mask;

	//debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

	return true;
}

/**
 * 尝试判断接收的是10位还是11位通道值
 *
 * @param[in] reset true=重置10/11位状态为未知
 */
static void
dsm_guess_format(bool reset, const uint8_t dsm_frame[16])
{
	static uint32_t	cs10;
	static uint32_t	cs11;
	static unsigned samples;

	/* 重置10/11位嗅探通道掩码 */
	if (reset) {
		cs10 = 0;
		cs11 = 0;
		samples = 0;
		dsm_channel_shift = 0;
		return;
	}

	/* 以10位和11位模式扫描当前DSM帧中的通道 */
	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		const uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		/* 如果通道解码成功,记住分配的编号 */
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31))
			cs10 |= (1 << channel);

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31))
			cs11 |= (1 << channel);

		/* XXX 如果我们关心,可以在这里查看相位位来决定1帧还是2帧格式 */
	}

	/* 等待直到我们看到足够多的帧 - 通常5帧就足够了 */
	if (samples++ < 5)
		return;

	/*
	 * 遍历合理的嗅探通道集合,看看10位或11位模式的解码
	 * 是否产生了我们认识的东西。
	 *
	 * XXX 注意由于DSM2高分辨率流中似乎存在bug,
	 * 在某些情况下当我们认为我们在与DSM2接收机以高分辨率模式通信时,
	 * 我们可能需要嗅探更长时间(以便我们可以理想地拒绝它)。
	 * 参见例如 http://git.openpilot.org/cru/OPReview-116 中关于此问题的讨论
	 */
	static uint32_t masks[] = {
		0x3f,	/* 6个通道 (DX6) */
		0x7f,	/* 7个通道 (DX7) */
		0xff,	/* 8个通道 (DX8) */
		0x1ff,	/* 9个通道 (DX9等) */
		0x3ff,	/* 10个通道 (DX10) */
		0x1fff,	/* 13个通道 (DX10t) */
		0x3fff	/* 18个通道 (DX10) */
	};
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for (unsigned i = 0; i < sizeof(masks)/sizeof(masks[0]); i++) {

		if (cs10 == masks[i])
			votes10++;

		if (cs11 == masks[i])
			votes11++;
	}

	if ((votes11 == 1) && (votes10 == 0)) {
		dsm_channel_shift = 11;
		debug("DSM: 11-bit format");
		return;
	}

	if ((votes10 == 1) && (votes11 == 0)) {
		dsm_channel_shift = 10;
		debug("DSM: 10-bit format");
		return;
	}

	/* 调用自身重置状态...我们必须重试 */
	debug("DSM: format detect fail, 10: 0x%08x %u 11: 0x%08x %u", cs10, votes10, cs11, votes11);
	dsm_guess_format(true, dsm_frame);
}

/**
 * 解码整个DSM帧(所有包含的通道)
 *
 */
bool
dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16], uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	/*
	debug("DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
		dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
		dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
	*/
	/*
	 * 如果我们至少一秒钟没有信号,重置格式猜测启发式
	 */
	if (((frame_time - dsm_last_frame_time) > 1000000) && (dsm_channel_shift != 0))
            dsm_guess_format(true, dsm_frame);

	/* 我们收到了一些我们认为是DSM帧的东西 */
	dsm_last_frame_time = frame_time;

	/* 如果我们不知道帧格式,更新猜测状态机 */
	if (dsm_channel_shift == 0) {
            dsm_guess_format(false, dsm_frame);
            return false;
	}

	/*
	 * 前两个字节的编码不确定,所以我们暂时忽略它们。
	 *
	 * 每个通道是一个16位无符号值,包含10位或11位通道值
	 * 和4位通道号,左移10位或11位。MSB也可能被设置
	 * 以指示协议变体中的第二帧,其中传输超过7个通道。
	 */

	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		const uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value))
			continue;

		/* 忽略超出范围的通道 */
		if (channel >= max_values)
			continue;

		/* 更新解码的通道计数 */
		if (channel >= *num_values)
			*num_values = channel + 1;

		/* 将0-1024 / 0-2048值转换为1000-2000 ppm编码。 */
		if (dsm_channel_shift == 10)
			value *= 2;

		/*
		 * Spektrum缩放是特殊的。有这些基本考虑:
		 *
		 *   * 中点是1520 us
		 *   * 100%行程通道是±400 us
		 *
		 * 我们遵循原始的Spektrum缩放(因此默认设置将从1100-1900 us缩放),
		 * 但我们不遵循奇怪的1520 us中心点,而是(正确地)将中心点设在1500 us周围。
		 * 这是为了在不需要用户在数字链路上校准的情况下获得有用的东西。
		 */

		/* 缩放整数以获得良好的精度同时保持效率 */
		value = ((((int)value - 1024) * 1000) / 1700) + 1500;

		/*
		 * 将解码的通道存储到R/C输入缓冲区中,考虑到我们对通道分配的不同想法。
		 *
		 * 具体来说,rc_channel_data中的前四个通道是横滚、俯仰、油门、偏航,
		 * 但来自DSM接收机的前四个通道是油门、横滚、俯仰、偏航。
		 */
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;
            break;

		default:
			break;
		}

		values[channel] = value;
	}

	/*
	 * Spektrum喜欢在更高的通道号中发送垃圾数据来填充它们的数据包。
	 * 我们不知道他们的发射机系列中有13通道的型号,
	 * 所以如果我们得到13个通道的计数,我们将返回12
	 * (最后一个稳定的数据索引)。
	 */
	if (*num_values == 13)
		*num_values = 12;

#if 0
	if (dsm_channel_shift == 11) {
		/* 设置11位数据指示器 */
		*num_values |= 0x8000;
	}
#endif

	/*
	 * XXX 注意我们可能在这里处于故障保护状态;我们需要弄清楚如何检测到这一点。
	 */
	return true;
}

#if defined(TEST_MAIN_PROGRAM) || defined(TEST_HEX_STRING)
static uint8_t dsm_partial_frame_count;
static uint8_t dsm_frame[DSM_FRAME_SIZE];
static enum DSM_DECODE_STATE {
	DSM_DECODE_STATE_DESYNC = 0,
	DSM_DECODE_STATE_SYNC
} dsm_decode_state = DSM_DECODE_STATE_DESYNC;
static uint64_t dsm_last_rx_time;            /**< 上次接收数据的时间戳 */
static uint16_t dsm_chan_count;
static uint16_t dsm_frame_drops;

static bool
dsm_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
	  uint16_t *num_values, bool *dsm_11_bit, unsigned *frame_drops, uint16_t max_channels)
{

	/* 这由解码状态机设置,一旦所有可解码的内容都被解码后将默认为false */
	bool decode_ret = false;

	/* 继续解码直到我们消耗完缓冲区 */
	for (unsigned d = 0; d < len; d++) {

		/* 溢出检查 */
		if (dsm_partial_frame_count == sizeof(dsm_frame) / sizeof(dsm_frame[0])) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (BUF LIM)\n");
#endif
		}

		if (dsm_partial_frame_count == DSM_FRAME_SIZE) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (PACKET LIM)\n");
#endif
		}

#ifdef DSM_DEBUG
#if 1
		printf("dsm state: %s%s, count: %d, val: %02x\n",
		       (dsm_decode_state == DSM_DECODE_STATE_DESYNC) ? "DSM_DECODE_STATE_DESYNC" : "",
		       (dsm_decode_state == DSM_DECODE_STATE_SYNC) ? "DSM_DECODE_STATE_SYNC" : "",
		       dsm_partial_frame_count,
		       (unsigned)frame[d]);
#endif
#endif

		switch (dsm_decode_state) {
		case DSM_DECODE_STATE_DESYNC:

			/* 我们处于去同步状态,只对帧标记感兴趣 */
			if ((now - dsm_last_rx_time) > 5000) {
				printf("resync %u\n", dsm_partial_frame_count);
				dsm_decode_state = DSM_DECODE_STATE_SYNC;
				dsm_partial_frame_count = 0;
				dsm_chan_count = 0;
				dsm_frame[dsm_partial_frame_count++] = frame[d];
			}

			break;

		case DSM_DECODE_STATE_SYNC: {
				dsm_frame[dsm_partial_frame_count++] = frame[d];

				/* 解码我们得到和期望的任何内容 */
				if (dsm_partial_frame_count < DSM_FRAME_SIZE) {
					break;
				}

				/*
				 * 很好,看起来我们可能有一帧。继续解码它。
				 */
				decode_ret = dsm_decode(now, dsm_frame, values, &dsm_chan_count, max_channels);

#if 1
                                printf("%u %u: ", ((unsigned)(now/1000)) % 1000000, len);
                                for (uint8_t i=0; i<DSM_FRAME_SIZE; i++) {
                                    printf("%02x ", (unsigned)dsm_frame[i]);
                                }
                                printf("\n");
#endif
	
                                
				/* 我们消耗了部分帧,重置 */
				dsm_partial_frame_count = 0;

				/* 如果解码失败,将协议设置为去同步 */
				if (decode_ret == false) {
					dsm_decode_state = DSM_DECODE_STATE_DESYNC;
					dsm_frame_drops++;
					printf("drop ");
					for (uint8_t i=0; i<DSM_FRAME_SIZE; i++) {
						printf("%02x ", (unsigned)dsm_frame[i]);
					}
					printf("\n");
				}
			}
			break;

		default:
#ifdef DSM_DEBUG
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}

	if (frame_drops) {
		*frame_drops = dsm_frame_drops;
	}

	if (decode_ret) {
		*num_values = dsm_chan_count;
	}

	dsm_last_rx_time = now;

	/* 默认返回false */
	return decode_ret;
}
#endif


#ifdef TEST_MAIN_PROGRAM
/*
  测试工具,用于在Linux下使用USB串口适配器
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>

static uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)));
}

int main(int argc, const char *argv[])
{
    int fd = open(argv[1], O_RDONLY|O_CLOEXEC);
    if (fd == -1) {
        perror(argv[1]);
        exit(1);
    }

    struct termios options;

    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr");
        exit(1);
    }
    tcflush(fd, TCIOFLUSH);

    uint16_t values[18];
    memset(values, 0, sizeof(values));

    while (true) {
        uint8_t b[16];
        uint16_t num_values = 0;
        fd_set fds;
        struct timeval tv;
    
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // 检查是否有可用字节
        if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
            break;
        }

        ssize_t nread;
        if ((nread = read(fd, b, sizeof(b))) < 1) {
            break;
        }

        bool dsm_11_bit;
        unsigned frame_drops;
        
        if (dsm_parse(micros64(), b, nread, values, &num_values, &dsm_11_bit, &frame_drops, 18)) {
#if 1
            printf("%u: ", num_values);
            for (uint8_t i=0; i<num_values; i++) {
                printf("%u:%4u ", i+1, values[i]);
            }
            printf("\n");
#endif
        }
    }
}
#elif defined(TEST_HEX_STRING)
/*
  测试工具,提供十六进制字符串进行解码
 */
#include <string.h>

int main(int argc, const char *argv[])
{
    uint8_t b[16];
    uint64_t t = 0;

    for (uint8_t i=1; i<argc; i++) {
        unsigned v;
        if (sscanf(argv[i], "%02x", &v) != 1 || v > 255) {
            printf("Bad hex value at %u : %s\n", (unsigned)i, argv[i]);
            return 1;
        }
        b[i-1] = v;
    }
    uint16_t values[18];
    memset(values, 0, sizeof(values));
    
    while (true) {
        uint16_t num_values = 0;
        bool dsm_11_bit;
        unsigned frame_drops;

        t += 11000;
        
        if (dsm_parse(t, b, sizeof(b), values, &num_values, &dsm_11_bit, &frame_drops, 18)) {
#if 1
            printf("%u: ", num_values);
            for (uint8_t i=0; i<num_values; i++) {
                printf("%u:%4u ", i+1, values[i]);
            }
            printf("\n");
#endif
        }
    }
}
#endif
