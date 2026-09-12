/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright 2026 Dj */
/*
 * Apple SEP driver — private declarations for the C shim.
 *
 * Glue between the Rust driver and kernel interfaces whose ABI is awkward or
 * unsafe to restate in Rust: `struct hwrng`, whose layout moves with
 * CONFIG_LOCKDEP; the file interface, whose O_* flags are macro-derived and so
 * absent from the Rust bindings; and the crypto API, whose shash descriptor is
 * sized by a macro at its call site.
 *
 * No protocol logic lives in the shim. Included only by the shim translation
 * units.
 */

#ifndef SEP_SHIM_H
#define SEP_SHIM_H

#include <linux/types.h>

/* -- hwrng_shim.c ------------------------------------------------------- */

void *sep_hwrng_alloc(void);
void sep_hwrng_free(void *mem);
int sep_hwrng_register(void *mem, const char *name,
			      unsigned short quality, void *ctx,
			      int (*read)(void *ctx, void *data, size_t max,
					  bool wait));
void sep_hwrng_unregister(void *mem);

/* -- store_shim.c ------------------------------------------------------- */

void *sep_store_open(const char *path);
void *sep_store_open_trunc(const char *path);
void *sep_store_open_ro(const char *path);
void *sep_store_open_block(const char *path, int writable);
void sep_store_close(void *handle);
long long sep_store_size(void *handle);
long sep_store_read(void *handle, long long off, void *buf, size_t len);
long sep_store_write(void *handle, long long off, const void *buf,
			    size_t len);
int sep_store_sync(void *handle);

/* -- sha_shim.c --------------------------------------------------------- */

int sep_sha256(const void *a, size_t alen, const void *b, size_t blen,
		      unsigned char *out);

/* -- crypto_shim.c ------------------------------------------------------ */

/* HMAC-SHA256 over one message, 32 bytes out. */
int sep_hmac_sha256(const void *key, size_t keylen,
			   const void *data, size_t datalen, u8 *out);

/*
 * AES-256-GCM in place over `buf` = `aadlen` bytes of AAD followed by the
 * payload. Returns -EBADMSG on auth failure, which the caller must treat as
 * fatal.
 */
int sep_gcm(int encrypt, const void *key, size_t keylen,
		   const void *iv, size_t ivlen, size_t aadlen,
		   void *buf, size_t buflen, size_t datalen);

/*
 * Fill `buf` from the kernel CSPRNG, seeded, or fail. For key-bag secrets only;
 * anything measuring the enclave's own entropy (0x59 device-key probe,
 * /dev/hwrng) stays on SEP.
 */
int sep_random_bytes(void *buf, size_t len);

/* -- p256_shim.c ------------------------------------------------------- */

/*
 * ECIES sender key agreement for the ref-key (op 0x22) SE-seal, with a fresh
 * ephemeral P-256 keypair. `peer_pub_be` is the recipient's 64-byte public
 * point (0x04 prefix stripped), `eph_pub_be_out` receives the 64-byte ephemeral
 * public point, `shared_be_out` the 32-byte shared secret. All big-endian.
 */
int sep_p256_sender(const void *peer_pub_be, void *eph_pub_be_out,
			   void *shared_be_out);

/* -- sensor_shim.c ------------------------------------------------------ */

int sep_sensor_register(void);
void sep_sensor_unregister(void);
int sep_sensor_bound(void);
int sep_sensor_power_line(void);
int sep_sensor_cs_timing_mode(void);
int sep_sensor_power_cycle(void);
int sep_sensor_power_source(void);
int sep_sensor_power(int on);
int sep_sensor_xfer(const void *tx, void *rx, size_t len);
int sep_sensor_xfer_tx(const void *tx, size_t len);
int sep_sensor_xfer2(const void *tx, size_t tx_len, void *rx, size_t rx_len);

/* -- bio_shim.c --------------------------------------------------------- */

void *sep_bio_register(const char *name, unsigned short mode, void *ctx,
			  int (*f_open)(void *),
			  void (*f_release)(void *),
			  long (*f_ioctl)(void *, unsigned int, unsigned long),
			  int (*f_ready)(void *));
void sep_bio_unregister(void *dev);
void sep_bio_wake(void *dev);
int sep_bio_capable_admin(void);
__u64 sep_bio_monotonic_ns(void);
__u64 sep_bio_boottime_ns(void);

/* -- trusted_shim.c ---------------------------------------------------- */

/*
 * The kernel trusted-key source (keyctl). The shim owns the framework structs
 * and registration; the sealing policy is in Rust (trusted.rs), reached through
 * the callbacks passed to sep_tk_register(). The payload struct is only
 * forward-declared here since these prototypes take it by pointer; trusted_shim.c
 * pulls in <keys/trusted-type.h> for the definition.
 */
struct trusted_key_payload;

typedef int (*sep_tk_init_fn)(void);
typedef int (*sep_tk_seal_fn)(struct trusted_key_payload *p, char *datablob);
typedef int (*sep_tk_unseal_fn)(struct trusted_key_payload *p, char *datablob);
typedef int (*sep_tk_random_fn)(unsigned char *key, size_t key_len);
typedef void (*sep_tk_exit_fn)(void);

int sep_tk_register(sep_tk_init_fn init, sep_tk_seal_fn seal,
		       sep_tk_unseal_fn unseal, sep_tk_random_fn random,
		       sep_tk_exit_fn exit);
void sep_tk_unregister(void);

int sep_tk_register_key_type(void);
void sep_tk_unregister_key_type(void);

size_t sep_tk_max_key_size(void);
size_t sep_tk_max_blob_size(void);

unsigned char *sep_tk_key_ptr(struct trusted_key_payload *p);
unsigned int sep_tk_key_len(const struct trusted_key_payload *p);
void sep_tk_set_key_len(struct trusted_key_payload *p, unsigned int n);

unsigned char *sep_tk_blob_ptr(struct trusted_key_payload *p);
unsigned int sep_tk_blob_len(const struct trusted_key_payload *p);
void sep_tk_set_blob_len(struct trusted_key_payload *p, unsigned int n);

#endif /* SEP_SHIM_H */
