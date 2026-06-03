/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_TOPOLOGY_H
#define _ASM_RISCV_TOPOLOGY_H

#include <linux/arch_topology.h>

#ifdef CONFIG_RISCV_ISA_SSCPUUTIL

#include <asm/csr.h>

#ifdef CONFIG_64BIT
static inline u64 get_corecyc(void)
{
	return csr_read(CSR_CORECYC);
}

static inline u64 get_acttime(void)
{
	return csr_read(CSR_ACTTIME);
}
#else
static inline u64 get_corecyc(void)
{
	u32 hi, lo;

	do {
		hi = csr_read(CSR_CORECYCH);
		lo = csr_read(CSR_CORECYC);
	} while (hi != csr_read(CSR_CORECYCH));

	return ((u64)hi << 32) | lo;
}

static inline u64 get_acttime(void)
{
	u32 hi, lo;

	do {
		hi = csr_read(CSR_ACTTIMEH);
		lo = csr_read(CSR_ACTTIME);
	} while (hi != csr_read(CSR_ACTTIMEH));

	return ((u64)hi << 32) | lo;
}
#endif /* CONFIG_64BIT */

void update_freq_counters_refs(void);
#endif /* CONFIG_RISCV_ISA_SSCPUUTIL */

/* Replace task scheduler's default frequency-invariant accounting */
#define arch_scale_freq_tick		topology_scale_freq_tick
#define arch_set_freq_scale		topology_set_freq_scale
#define arch_scale_freq_capacity	topology_get_freq_scale
#define arch_scale_freq_invariant	topology_scale_freq_invariant

/* Replace task scheduler's default cpu-invariant accounting */
#define arch_scale_cpu_capacity	topology_get_cpu_scale

/* Enable topology flag updates */
#define arch_update_cpu_topology	topology_update_cpu_topology

#include <asm-generic/topology.h>

#endif /* _ASM_RISCV_TOPOLOGY_H */
