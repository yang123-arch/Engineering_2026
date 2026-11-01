#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "main.h"

/*
 * 环形缓冲区（Linux kfifo 风格）——单生产者/单消费者无锁安全
 *
 * 约束: maxElems 必须是 2 的幂，用位与代替取模
 */

typedef struct {
  void *pBuffer; // 缓冲区基址 (void* 通用, 内部按 uint8_t* 操作)
  volatile uint32_t writeIdx; // 写索引 —— 只被生产者修改
  volatile uint32_t readIdx;  // 读索引 —— 只被消费者修改
  uint32_t maxElems; // 最大元素数 (2 的幂, 如 64, 256)
  uint32_t elemSize; // 单个元素字节数
  uint32_t mask;     // maxElems - 1 (用于位与替代取模)
} RingBuffer_t;

/* 返回码 */
enum {
  RB_OK = 0,         // 成功
  RB_ERR_PARAM = 1,  // 参数错误
  RB_ERR_NOSPC = 2,  // 空闲空间不足 (写)
  RB_ERR_NODATA = 3, // 可读数据不足 (读)
};

/* ── 类型安全的静态声明宏 ── */
#define RING_BUFFER_DEFINE(name, type, size)                                   \
  type name##_buf[(size)];                                                     \
  RingBuffer_t name = {0};                                                     \
  _Static_assert((size) && !((size) & ((size)-1)), "size must be power of 2")

/* =============== API =============== */

/* 初始化: 传入已分配好的缓冲区 */
void RingBuffer_Init(RingBuffer_t *rb, void *buf, uint32_t maxElems,
                     uint32_t elemSize);

/* 写入: src → 环形缓冲, count 个元素。返回 RB_OK / RB_ERR_* */
uint8_t RingBuffer_Write(RingBuffer_t *rb, const void *src, uint32_t count);

/* ISR-safe SPSC write. It never waits and never calls an RTOS API. */
uint8_t RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *src,
                                uint32_t count);

/* 读取: 环形缓冲 → dst, count 个元素。返回 RB_OK / RB_ERR_* */
uint8_t RingBuffer_Read(RingBuffer_t *rb, void *dst, uint32_t count);

/* 预读: 同 Read 但不移动读指针 —— 校验失败可回退 (Linux kfifo_out_peek) */
uint8_t RingBuffer_Peek(RingBuffer_t *rb, void *dst, uint32_t count);

/* 读出全部未读元素, 返回实际读出的元素个数 */
uint32_t RingBuffer_ReadAll(RingBuffer_t *rb, void *dst);

/* Number of readable elements and remaining writable elements. */
uint32_t RingBuffer_Count(const RingBuffer_t *rb);
uint32_t RingBuffer_Free(const RingBuffer_t *rb);

/* 跳过 count 个已读元素 (等价于读后丢弃) */
void RingBuffer_Skip(RingBuffer_t *rb, uint32_t count);

#endif
