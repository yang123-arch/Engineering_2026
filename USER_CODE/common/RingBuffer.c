#include "RingBuffer.h"
#include <string.h>

/* ── 内部辅助: 最大元素数必须是 2 的幂 ── */
static inline int _is_power_of_two(uint32_t n) { return n && !(n & (n - 1)); }

/* 查询 */
/* 单调递增索引: 差值无需取模, uint32_t 自然溢出保证正确性 (Linux kfifo 同) */
uint32_t RingBuffer_Count(const RingBuffer_t *rb) {
  if (!rb)
    return 0;
  return rb->writeIdx - rb->readIdx;
}
uint32_t RingBuffer_Free(const RingBuffer_t *rb) {
  if (!rb)
    return 0;
  return rb->maxElems - RingBuffer_Count(rb);
}
static inline uint32_t RingBuffer_IsEmpty(const RingBuffer_t *rb) {
  return rb->writeIdx == rb->readIdx;
}
static inline uint32_t RingBuffer_IsFull(const RingBuffer_t *rb) {
  return RingBuffer_Count(rb) == rb->maxElems;
}
/* 重置为空 */
static inline void RingBuffer_Reset(RingBuffer_t *rb) {
  rb->writeIdx = rb->readIdx = 0;
}
/* ================================================================
 * RingBuffer_Init
 * ================================================================ */
void RingBuffer_Init(RingBuffer_t *rb, void *buf, uint32_t maxElems,
                     uint32_t elemSize) {
  if (!rb || !buf || !maxElems || !elemSize || !_is_power_of_two(maxElems))
    return; /* 参数错误, 调用方应先检查 */

  rb->pBuffer = buf;
  rb->maxElems = maxElems;
  rb->elemSize = elemSize;
  rb->mask = maxElems - 1;
  rb->writeIdx = 0;
  rb->readIdx = 0;
}

/* ================================================================
 * RingBuffer_Write
 *
 * 参照 Linux kfifo_copy_in: 分两段 memcpy 处理回绕
 * ================================================================ */
uint8_t RingBuffer_Write(RingBuffer_t *rb, const void *src, uint32_t count) {
  if (!rb || !src || !count || !rb->pBuffer)
    return RB_ERR_PARAM;
  if (RingBuffer_Free(rb) < count)
    return RB_ERR_NOSPC;

  uint32_t pos = rb->writeIdx & rb->mask; /* 写指针模运算 */
  uint32_t tail = rb->maxElems - pos;     /* 到缓冲区末尾的剩余 */
  uint32_t len = count * rb->elemSize;
  uint8_t *buf = (uint8_t *)rb->pBuffer;

  if (len <= tail * rb->elemSize) {
    memcpy(buf + pos * rb->elemSize, src, len);
  } else {
    /* 回绕: 先写尾部, 再写头部 */
    uint32_t first = tail * rb->elemSize;
    memcpy(buf + pos * rb->elemSize, src, first);
    memcpy(buf, (const uint8_t *)src + first, len - first);
  }

  /*
   * 单生产者场景: 先写数据再更新索引, 消费者看到新索引时数据已就绪
   * 等效于 Linux kfifo 的 smp_wmb() + 索引更新
   */
  __DMB();
  rb->writeIdx += count;
  return RB_OK;
}

uint8_t RingBuffer_WriteFromISR(RingBuffer_t *rb, const void *src,
                                uint32_t count) {
  if (!rb || !src || !count || !rb->pBuffer)
    return RB_ERR_PARAM;
  if (RingBuffer_Free(rb) < count)
    return RB_ERR_NOSPC;

  uint32_t pos = rb->writeIdx & rb->mask;
  uint32_t tail = rb->maxElems - pos;
  uint32_t len = count * rb->elemSize;
  uint8_t *buf = (uint8_t *)rb->pBuffer;

  if (len <= tail * rb->elemSize) {
    memcpy(buf + pos * rb->elemSize, src, len);
  } else {
    uint32_t first = tail * rb->elemSize;
    memcpy(buf + pos * rb->elemSize, src, first);
    memcpy(buf, (const uint8_t *)src + first, len - first);
  }

  __DMB();
  rb->writeIdx += count;
  return RB_OK;
}

/* ================================================================
 * RingBuffer_Read
 * ================================================================ */
uint8_t RingBuffer_Read(RingBuffer_t *rb, void *dst, uint32_t count) {
  if (!rb || !dst || !count || !rb->pBuffer)
    return RB_ERR_PARAM;
  if (RingBuffer_Count(rb) < count)
    return RB_ERR_NODATA;

  uint32_t pos = rb->readIdx & rb->mask;
  uint32_t tail = rb->maxElems - pos;
  uint32_t len = count * rb->elemSize;
  uint8_t *buf = (uint8_t *)rb->pBuffer;

  if (len <= tail * rb->elemSize) {
    memcpy(dst, buf + pos * rb->elemSize, len);
  } else {
    uint32_t first = tail * rb->elemSize;
    memcpy(dst, buf + pos * rb->elemSize, first);
    memcpy((uint8_t *)dst + first, buf, len - first);
  }

  /*
   * 单消费者场景: 先读走数据再更新索引, 生产者看到新索引时空间已释放
   */
  __DMB();
  rb->readIdx += count;
  return RB_OK;
}

/* ================================================================
 * RingBuffer_ReadAll
 * ================================================================ */
uint32_t RingBuffer_ReadAll(RingBuffer_t *rb, void *dst) {
  uint32_t cnt = RingBuffer_Count(rb);
  if (cnt == 0 || !dst)
    return 0;
  RingBuffer_Read(rb, dst, cnt);
  return cnt;
}

/* ================================================================
 * RingBuffer_Peek (Linux kfifo_out_peek 等价)
 *
 * 与 Read 相同但不消费数据 —— 帧校验失败时无需回退
 * ================================================================ */
uint8_t RingBuffer_Peek(RingBuffer_t *rb, void *dst, uint32_t count) {
  if (!rb || !dst || !count || !rb->pBuffer)
    return RB_ERR_PARAM;
  if (RingBuffer_Count(rb) < count)
    return RB_ERR_NODATA;

  uint32_t pos = rb->readIdx & rb->mask;
  uint32_t tail = rb->maxElems - pos;
  uint32_t len = count * rb->elemSize;
  uint8_t *buf = (uint8_t *)rb->pBuffer;

  if (len <= tail * rb->elemSize) {
    memcpy(dst, buf + pos * rb->elemSize, len);
  } else {
    uint32_t first = tail * rb->elemSize;
    memcpy(dst, buf + pos * rb->elemSize, first);
    memcpy((uint8_t *)dst + first, buf, len - first);
  }
  /* 读指针不变 —— 这是 Peek 与 Read 的唯一区别 */
  return RB_OK;
}

/* ================================================================
 * RingBuffer_Skip
 * ================================================================ */
void RingBuffer_Skip(RingBuffer_t *rb, uint32_t count) {
  if (!rb || !count || RingBuffer_Count(rb) < count)
    return;
  __DMB();
  rb->readIdx += count;
}
