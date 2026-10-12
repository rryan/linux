/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple SoC system sleep helpers
 *
 * Each power-down call (..._off(), ..._gate()) records what it changed so
 * that the matching ..._restore() puts back exactly those register fields;
 * a restore without a preceding power-down call is a no-op. If a power-down
 * call fails part way, the matching restore still undoes the part that was
 * applied.
 *
 * apple_sleep_pmgr_wake() sets bit 31 of PMGR_SOC + 0xc000, the first PMGR
 * write needed after the SoC returns from suspend-to-RAM.
 *
 * apple_sleep_ap_wake_arm() masks every AP wake source except those listed
 * in apple,ap-wake-sources, for suspend-to-RAM; it returns -ENOENT if the
 * node lists none.
 *
 * All calls are atomic (spinlocked regmaps, busy-waits of at most 8 ms per
 * register) and may be made with interrupts disabled. The power-down calls
 * return -ENODEV if the PMGR sleep node is absent; the others then do
 * nothing.
 */

#ifndef _LINUX_SOC_APPLE_SLEEP_H_
#define _LINUX_SOC_APPLE_SLEEP_H_

#include <linux/errno.h>

#if IS_ENABLED(CONFIG_APPLE_SLEEP)

int apple_sleep_sio_off(void);
void apple_sleep_sio_restore(void);
int apple_sleep_afr_off(void);
void apple_sleep_afr_restore(void);
int apple_sleep_soc_clusters_off(void);
void apple_sleep_soc_clusters_restore(void);
int apple_sleep_cpm_gate(void);
void apple_sleep_cpm_restore(void);
void apple_sleep_pmgr_wake(void);
int apple_sleep_ap_wake_arm(void);
void apple_sleep_ap_wake_restore(void);

#else

static inline int apple_sleep_sio_off(void) { return -ENODEV; }
static inline void apple_sleep_sio_restore(void) { }
static inline int apple_sleep_afr_off(void) { return -ENODEV; }
static inline void apple_sleep_afr_restore(void) { }
static inline int apple_sleep_soc_clusters_off(void) { return -ENODEV; }
static inline void apple_sleep_soc_clusters_restore(void) { }
static inline int apple_sleep_cpm_gate(void) { return -ENODEV; }
static inline void apple_sleep_cpm_restore(void) { }
static inline void apple_sleep_pmgr_wake(void) { }
static inline int apple_sleep_ap_wake_arm(void) { return -ENODEV; }
static inline void apple_sleep_ap_wake_restore(void) { }

#endif

#endif /* _LINUX_SOC_APPLE_SLEEP_H_ */
