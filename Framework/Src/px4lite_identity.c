/**
 * @file px4lite_identity.c
 * @brief Unified identity handling for school labels, MAVLink routing and RemoteID.
 */

#include "px4lite_identity.h"

#include "px4lite_config.h"
#include "px4lite_platform.h"

#include <string.h>

#define IDENTITY_SHA256_BLOCK_LEN 64U
#define IDENTITY_SHA256_DIGEST_LEN 32U

typedef struct {
  uint32_t state[8];
  uint64_t bit_len;
  uint8_t data[IDENTITY_SHA256_BLOCK_LEN];
  uint8_t data_len;
} Identity_Sha256Context_t;

static const uint32_t s_sha256_k[64] = {
  0x428A2F98UL, 0x71374491UL, 0xB5C0FBCFUL, 0xE9B5DBA5UL,
  0x3956C25BUL, 0x59F111F1UL, 0x923F82A4UL, 0xAB1C5ED5UL,
  0xD807AA98UL, 0x12835B01UL, 0x243185BEUL, 0x550C7DC3UL,
  0x72BE5D74UL, 0x80DEB1FEUL, 0x9BDC06A7UL, 0xC19BF174UL,
  0xE49B69C1UL, 0xEFBE4786UL, 0x0FC19DC6UL, 0x240CA1CCUL,
  0x2DE92C6FUL, 0x4A7484AAUL, 0x5CB0A9DCUL, 0x76F988DAUL,
  0x983E5152UL, 0xA831C66DUL, 0xB00327C8UL, 0xBF597FC7UL,
  0xC6E00BF3UL, 0xD5A79147UL, 0x06CA6351UL, 0x14292967UL,
  0x27B70A85UL, 0x2E1B2138UL, 0x4D2C6DFCUL, 0x53380D13UL,
  0x650A7354UL, 0x766A0ABBUL, 0x81C2C92EUL, 0x92722C85UL,
  0xA2BFE8A1UL, 0xA81A664BUL, 0xC24B8B70UL, 0xC76C51A3UL,
  0xD192E819UL, 0xD6990624UL, 0xF40E3585UL, 0x106AA070UL,
  0x19A4C116UL, 0x1E376C08UL, 0x2748774CUL, 0x34B0BCB5UL,
  0x391C0CB3UL, 0x4ED8AA4AUL, 0x5B9CCA4FUL, 0x682E6FF3UL,
  0x748F82EEUL, 0x78A5636FUL, 0x84C87814UL, 0x8CC70208UL,
  0x90BEFFFAUL, 0xA4506CEBUL, 0xBEF9A3F7UL, 0xC67178F2UL
};

static uint32_t Identity_Rotr32(uint32_t value, uint8_t bits)
{
  return (value >> bits) | (value << (32U - bits));
}

static uint32_t Identity_Fnv1aUpdate(uint32_t hash, uint8_t value)
{
  hash ^= value;
  return hash * 16777619UL;
}

static uint32_t Identity_BuildNodeHash(void)
{
  uint32_t uid_words[3];
  uint32_t hash = 2166136261UL;
  uint8_t i;
  uint8_t shift;

  if (Px4Lite_PlatformGetHardwareUid(uid_words, 3U) != PX4LITE_OK) {
    return 0U;
  }

  for (i = 0U; i < 3U; i++) {
    for (shift = 0U; shift < 32U; shift = (uint8_t)(shift + 8U)) {
      hash = Identity_Fnv1aUpdate(hash, (uint8_t)((uid_words[i] >> shift) & 0xFFU));
    }
  }
  return hash;
}

static void Identity_ReadUidBytes(uint8_t uid_bytes[12])
{
  uint32_t uid_words[3];
  uint8_t i;
  uint8_t offset;

  memset(uid_bytes, 0, 12U);
  if (Px4Lite_PlatformGetHardwareUid(uid_words, 3U) != PX4LITE_OK) { return; }

  for (i = 0U; i < 3U; i++) {
    offset = (uint8_t)(i * 4U);
    uid_bytes[offset + 0U] = (uint8_t)((uid_words[i] >> 24) & 0xFFU);
    uid_bytes[offset + 1U] = (uint8_t)((uid_words[i] >> 16) & 0xFFU);
    uid_bytes[offset + 2U] = (uint8_t)((uid_words[i] >> 8) & 0xFFU);
    uid_bytes[offset + 3U] = (uint8_t)(uid_words[i] & 0xFFU);
  }
}

static void Identity_Sha256Init(Identity_Sha256Context_t *ctx)
{
  ctx->data_len = 0U;
  ctx->bit_len = 0ULL;
  ctx->state[0] = 0x6A09E667UL;
  ctx->state[1] = 0xBB67AE85UL;
  ctx->state[2] = 0x3C6EF372UL;
  ctx->state[3] = 0xA54FF53AUL;
  ctx->state[4] = 0x510E527FUL;
  ctx->state[5] = 0x9B05688CUL;
  ctx->state[6] = 0x1F83D9ABUL;
  ctx->state[7] = 0x5BE0CD19UL;
}

static void Identity_Sha256Transform(Identity_Sha256Context_t *ctx, const uint8_t data[IDENTITY_SHA256_BLOCK_LEN])
{
  uint32_t m[64];
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t f;
  uint32_t g;
  uint32_t h;
  uint32_t t1;
  uint32_t t2;
  uint8_t i;

  for (i = 0U; i < 16U; i++) {
    uint8_t j = (uint8_t)(i * 4U);
    m[i] = ((uint32_t)data[j] << 24U) | ((uint32_t)data[j + 1U] << 16U) | ((uint32_t)data[j + 2U] << 8U) | (uint32_t)data[j + 3U];
  }
  for (i = 16U; i < 64U; i++) {
    uint32_t s0 = Identity_Rotr32(m[i - 15U], 7U) ^ Identity_Rotr32(m[i - 15U], 18U) ^ (m[i - 15U] >> 3U);
    uint32_t s1 = Identity_Rotr32(m[i - 2U], 17U) ^ Identity_Rotr32(m[i - 2U], 19U) ^ (m[i - 2U] >> 10U);
    m[i] = m[i - 16U] + s0 + m[i - 7U] + s1;
  }

  a = ctx->state[0];
  b = ctx->state[1];
  c = ctx->state[2];
  d = ctx->state[3];
  e = ctx->state[4];
  f = ctx->state[5];
  g = ctx->state[6];
  h = ctx->state[7];

  for (i = 0U; i < 64U; i++) {
    uint32_t s1 = Identity_Rotr32(e, 6U) ^ Identity_Rotr32(e, 11U) ^ Identity_Rotr32(e, 25U);
    uint32_t ch = (e & f) ^ ((~e) & g);
    uint32_t s0 = Identity_Rotr32(a, 2U) ^ Identity_Rotr32(a, 13U) ^ Identity_Rotr32(a, 22U);
    uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    t1 = h + s1 + ch + s_sha256_k[i] + m[i];
    t2 = s0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

static void Identity_Sha256Update(Identity_Sha256Context_t *ctx, const uint8_t *data, uint8_t len)
{
  uint8_t i;

  for (i = 0U; i < len; i++) {
    ctx->data[ctx->data_len++] = data[i];
    if (ctx->data_len == IDENTITY_SHA256_BLOCK_LEN) {
      Identity_Sha256Transform(ctx, ctx->data);
      ctx->bit_len += 512ULL;
      ctx->data_len = 0U;
    }
  }
}

static void Identity_Sha256Final(Identity_Sha256Context_t *ctx, uint8_t hash[IDENTITY_SHA256_DIGEST_LEN])
{
  uint8_t i;
  uint64_t bit_len;

  i = ctx->data_len;
  ctx->data[i++] = 0x80U;
  if (i > 56U) {
    while (i < 64U) { ctx->data[i++] = 0U; }
    Identity_Sha256Transform(ctx, ctx->data);
    i = 0U;
  }
  while (i < 56U) { ctx->data[i++] = 0U; }

  bit_len = ctx->bit_len + ((uint64_t)ctx->data_len * 8ULL);
  ctx->data[56] = (uint8_t)(bit_len >> 56U);
  ctx->data[57] = (uint8_t)(bit_len >> 48U);
  ctx->data[58] = (uint8_t)(bit_len >> 40U);
  ctx->data[59] = (uint8_t)(bit_len >> 32U);
  ctx->data[60] = (uint8_t)(bit_len >> 24U);
  ctx->data[61] = (uint8_t)(bit_len >> 16U);
  ctx->data[62] = (uint8_t)(bit_len >> 8U);
  ctx->data[63] = (uint8_t)bit_len;
  Identity_Sha256Transform(ctx, ctx->data);

  for (i = 0U; i < 8U; i++) {
    hash[(uint8_t)(i * 4U) + 0U] = (uint8_t)(ctx->state[i] >> 24U);
    hash[(uint8_t)(i * 4U) + 1U] = (uint8_t)(ctx->state[i] >> 16U);
    hash[(uint8_t)(i * 4U) + 2U] = (uint8_t)(ctx->state[i] >> 8U);
    hash[(uint8_t)(i * 4U) + 3U] = (uint8_t)ctx->state[i];
  }
}

static void Identity_BuildSerialNumber(char out[PX4LITE_IDENTITY_SN_MAX_LEN])
{
  static const char alphabet[] = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
  Identity_Sha256Context_t ctx;
  uint8_t uid_bytes[12];
  uint8_t digest[IDENTITY_SHA256_DIGEST_LEN];
  uint64_t value = 0ULL;
  int8_t pos;
  uint8_t i;

  Identity_ReadUidBytes(uid_bytes);
  Identity_Sha256Init(&ctx);
  Identity_Sha256Update(&ctx, uid_bytes, (uint8_t)sizeof(uid_bytes));
  Identity_Sha256Final(&ctx, digest);

  for (i = 0U; i < 8U; i++) {
    value = (value << 8U) | (uint64_t)digest[i];
  }

  for (pos = (int8_t)(PX4LITE_IDENTITY_SN_LEN - 1U); pos >= 0; --pos) {
    out[(uint8_t)pos] = alphabet[(uint8_t)(value % 34ULL)];
    value /= 34ULL;
  }
  out[PX4LITE_IDENTITY_SN_LEN] = '\0';
}

const char *Px4Lite_IdentityGetDeviceName(void)
{
  return PX4LITE_DEVICE_NAME;
}

uint32_t Px4Lite_IdentityGetUidHash(void)
{
  static uint8_t s_hash_valid = 0U;
  static uint32_t s_cached_hash = 0U;

  if (s_hash_valid == 0U) {
    s_cached_hash = Identity_BuildNodeHash();
    s_hash_valid = 1U;
  }
  return s_cached_hash;
}

uint8_t Px4Lite_IdentityGetNumericId(void)
{
  static uint8_t s_cached_id = 0U;
  uint32_t hash;

  if (s_cached_id != 0U) { return s_cached_id; }

  hash = Px4Lite_IdentityGetUidHash();
  if (hash == 0U) {
    s_cached_id = 1U;
  } else {
    s_cached_id = (uint8_t)(1U + (hash % 250U));
  }
  return s_cached_id;
}

uint8_t Px4Lite_IdentityGetNodeId(void)
{
  return Px4Lite_IdentityGetNumericId();
}

uint8_t Px4Lite_IdentityGetMavlinkSystemId(void)
{
  return Px4Lite_IdentityGetNumericId();
}

void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity)
{
  uint8_t pos;
  uint8_t id;
  const char *prefix = PX4LITE_DEVICE_NAME;

  if ((out == 0) || (capacity == 0U)) { return; }

  for (pos = 0U; pos < capacity; pos++) { out[pos] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_FULL_ID_MAX_LEN) { return; }

  pos = 0U;
  while ((prefix != 0) && (*prefix != '\0') && ((uint8_t)(pos + 5U) < capacity)) {
    out[pos++] = *prefix++;
  }
  id = Px4Lite_IdentityGetNumericId();
  out[pos++] = '-';
  out[pos++] = (char)('0' + (id / 100U));
  out[pos++] = (char)('0' + ((id / 10U) % 10U));
  out[pos++] = (char)('0' + (id % 10U));
  out[pos] = '\0';
}

void Px4Lite_IdentityFormatSerialNumber(char *out, uint8_t capacity)
{
  static uint8_t s_sn_valid = 0U;
  static char s_cached_sn[PX4LITE_IDENTITY_SN_MAX_LEN];
  uint8_t i;

  if ((out == 0) || (capacity == 0U)) { return; }
  for (i = 0U; i < capacity; i++) { out[i] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_SN_MAX_LEN) { return; }

  if (s_sn_valid == 0U) {
    Identity_BuildSerialNumber(s_cached_sn);
    s_sn_valid = 1U;
  }
  for (i = 0U; i < PX4LITE_IDENTITY_SN_MAX_LEN; i++) {
    out[i] = s_cached_sn[i];
  }
}

void Px4Lite_IdentityFormatVendorProductId(char *out, uint8_t capacity)
{
  char sn[PX4LITE_IDENTITY_SN_MAX_LEN];
  const char *prefix = PX4LITE_VENDOR_ID_PREFIX;
  uint8_t pos = 0U;
  uint8_t i;

  if ((out == 0) || (capacity == 0U)) { return; }
  for (i = 0U; i < capacity; i++) { out[i] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_VENDOR_ID_MAX_LEN) { return; }

  while ((prefix != 0) && (*prefix != '\0') && (pos < (uint8_t)(capacity - 1U))) {
    out[pos++] = *prefix++;
  }

  Px4Lite_IdentityFormatSerialNumber(sn, (uint8_t)sizeof(sn));
  for (i = 0U; (i < PX4LITE_IDENTITY_SN_LEN) && (pos < (uint8_t)(capacity - 1U)); i++) {
    out[pos++] = sn[i];
  }
  out[pos] = '\0';
}
