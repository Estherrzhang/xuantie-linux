// SPDX-License-Identifier: GPL-2.0-only
/*
 * arch/riscv/kernel/topology.c
 *
 * Frequency Invariance support for RISC-V using the Sscucnt extension
 * (CPU Utilization Counter Extension).
 *
 * The Sscucnt extension provides a pair of CSR counters:
 *   CSR_SSCUCNT_CORECYC  - core-cycles counter, increments at the actual CPU
 *                          frequency
 *   CSR_SSCUCNT_ACTTIME  - actual-time counter, increments at a fixed
 *                          frequency equal to the nominal CPU frequency
 *
 * The ratio delta_CORECYC / delta_ACTTIME approximates freq_curr / freq_nominal,
 * which is exactly the frequency-invariant utilisation scale factor required
 * by the task scheduler.
 *
 */

#include <linux/arch_topology.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/pm_opp.h>
#include <linux/sched.h>

#ifdef CONFIG_ACPI_CPPC_LIB
#include <acpi/cppc_acpi.h>
#endif

#include <asm/csr.h>
#include <asm/cpufeature.h>
#include <asm/delay.h>
#include <asm/hwcap.h>
#include <asm/topology.h>

#ifdef CONFIG_RISCV_ISA_SSCUCNT
#define read_corecyc()	get_corecyc()
#define read_acttime()	get_acttime()
#else
#define read_corecyc()	(0UL)
#define read_acttime()	(0UL)
#endif

static DEFINE_PER_CPU(u64, arch_active_time_prev);
static DEFINE_PER_CPU(u64, arch_core_cycles_prev);

#undef pr_fmt
#define pr_fmt(fmt) "Sscucnt: " fmt

static DEFINE_PER_CPU_READ_MOSTLY(unsigned long, arch_max_freq_scale);

/* Bitmask of CPUs for which Sscucnt-based FIE has been successfully set up */
static cpumask_var_t sscucnt_fie_cpus;

static inline bool freq_counters_valid(int cpu)
{
	if ((cpu >= nr_cpu_ids) || !cpumask_test_cpu(cpu, cpu_present_mask))
		return false;

	if (!riscv_cpu_has_extension_likely(cpu, RISCV_ISA_EXT_SSCUCNT)) {
		pr_debug("CPU%d: Sscucnt counters are not supported.\n", cpu);
		return false;
	}

	return true;
}


/*
 * Probe strategies for obtaining the maximum CPU frequency.
 * Tried in order; the first one that returns non-zero wins.
 *
 *   1. ACPI CPPC          - server platforms with ACPI
 *   2. DT clock-frequency - embedded / HAPS with DTS
 *   3. OPP table          - platforms with operating-points-v2
 */
static u64 get_max_freq_acpi_cppc(int cpu)
{
#ifdef CONFIG_ACPI_CPPC_LIB
	struct cppc_perf_caps caps;

	if (!acpi_disabled && !cppc_get_perf_caps(cpu, &caps)) {
		u64 rate;

		/*
		 * cppc nominal_freq is in MHz when nonzero,
		 * otherwise fall back to highest_perf (abstract units,
		 * not directly usable as Hz -> skip).
		 */
		if (caps.nominal_freq) {
			rate = (u64)caps.nominal_freq * 1000000ULL;
			pr_info("CPU%d: max_rate from ACPI CPPC: %llu Hz\n",
					cpu, rate);
			return rate;
		}
	}
#endif
	return 0;
}

static u64 get_max_freq_dt(int cpu)
{
	struct device_node *cn;
	u32 freq;
	u64 rate = 0;

	cn = of_cpu_device_node_get(cpu);
	if (!cn)
		return 0;

	/*
	 * "clock-frequency" is the standard ePAPR/DT property for
	 * the CPU's nominal operating frequency in Hz.
	 */
	if (!of_property_read_u32(cn, "clock-frequency", &freq) && freq) {
		rate = (u64)freq;
		pr_info("CPU%d: max_rate from DT clock-frequency: %llu Hz\n",
				cpu, rate);
		goto out;
	}

	/*
	 * Some vendor DTs use "cpu-freq" instead.
	 */
	if (!of_property_read_u32(cn, "cpu-freq", &freq) && freq) {
		rate = (u64)freq;
		pr_info("CPU%d: max_rate from DT cpu-freq: %llu Hz\n",
				cpu, rate);
		goto out;
	}

out:
	of_node_put(cn);
	return rate;
}

static u64 get_max_freq_opp(int cpu)
{
	struct device *dev;
	unsigned long freq;
	int ret;

	dev = get_cpu_device(cpu);
	if (!dev)
		return 0;

	/*
	 * Try to add OPP table from DT if not already done.
	 * Returns -EEXIST if already registered (harmless).
	 */
	ret = dev_pm_opp_of_add_table(dev);
	if (ret && ret != -EEXIST) {
		pr_debug("CPU%d: no OPP table in DT (ret=%d)\n", cpu, ret);
		return 0;
	}

	{
		struct dev_pm_opp *opp;

		freq = ULONG_MAX;
		opp = dev_pm_opp_find_freq_floor(dev, &freq);
		if (IS_ERR(opp)) {
			/* clean up if we just added it and it's empty */
			if (ret != -EEXIST)
				dev_pm_opp_of_remove_table(dev);
			return 0;
		}
		dev_pm_opp_put(opp);
	}

	if (!freq) {
		if (ret != -EEXIST)
			dev_pm_opp_of_remove_table(dev);
		return 0;
	}

	pr_info("CPU%d: max_rate from OPP table: %lu Hz\n", cpu, freq);
	return (u64)freq;
}

static u64 sscucnt_get_max_rate(int cpu)
{
	u64 rate = 0;

	/* 1. ACPI CPPC */
	rate = get_max_freq_acpi_cppc(cpu);
	if (rate)
		return rate;

	/* 2. DT clock-frequency */
	rate = get_max_freq_dt(cpu);
	if (rate)
		return rate;

	/* 3. OPP table */
	rate = get_max_freq_opp(cpu);
	if (rate)
		return rate;

	/* no source found */
	return 0;
}

/**
 * sscucnt_set_max_ratio - pre-compute the ref/max frequency scale factor
 * @cpu:      logical CPU number
 * @max_rate: maximum CPU frequency in Hz
 * @ref_rate: fixed frequency of the Sscucnt reference counter in Hz
 *
 * Stores the ratio (ref_rate / max_rate) × SCHED_CAPACITY_SCALE in
 * arch_max_freq_scale[cpu].
 *
 * Returns 0 on success, -EINVAL if either rate is zero or the ratio
 * underflows to zero.
 */
static int sscucnt_set_max_ratio(int cpu, u64 max_rate, u64 ref_rate)
{
	u64 ratio;

	if (unlikely(!max_rate || !ref_rate)) {
		pr_debug("CPU%d: invalid maximum or reference frequency.\n", cpu);
		return -EINVAL;
	}

	/*
	 * Pre-compute the fixed ratio between the reference counter frequency
	 * and the maximum CPU frequency:
	 *
	 *                          ref_rate
	 *   arch_max_freq_scale = ────────── × SCHED_CAPACITY_SCALE²
	 *                          max_rate
	 */
	ratio = ref_rate << (2 * SCHED_CAPACITY_SHIFT);
	ratio = div64_u64(ratio, max_rate);
	if (!ratio) {
		WARN_ONCE(1, "Sscucnt reference frequency too low.\n");
		return -EINVAL;
	}

	per_cpu(arch_max_freq_scale, cpu) = (unsigned long)ratio;

	return 0;
}


void update_freq_counters_refs(void)
{
	this_cpu_write(arch_core_cycles_prev, read_corecyc());
	this_cpu_write(arch_active_time_prev, read_acttime());
}

/**
 * sscucnt_scale_freq_tick - update arch_freq_scale on every scheduler tick
 *
 * Registered as the set_freq_scale callback of sscucnt_sfd and invoked by
 * topology_scale_freq_tick() → arch_scale_freq_tick() on every tick.
 *
 * Computes:
 *
 *          Δcorecyc    arch_max_freq_scale
 *   scale = ──────── × ────────────────────
 *          Δacttime    SCHED_CAPACITY_SCALE
 *
 * and writes it to arch_freq_scale for the current CPU.  The result is
 * clamped to [0, SCHED_CAPACITY_SCALE].
 */
static void sscucnt_scale_freq_tick(void)
{
	u64 corecyc_t0, acttime_t0;
	u64 corecyc_t1, acttime_t1;
	u64 delta_corecyc, delta_acttime;
	u64 scale;

	corecyc_t0 = this_cpu_read(arch_core_cycles_prev);
	acttime_t0 = this_cpu_read(arch_active_time_prev);

	update_freq_counters_refs();

	corecyc_t1 = this_cpu_read(arch_core_cycles_prev);
	acttime_t1 = this_cpu_read(arch_active_time_prev);

	if (unlikely(corecyc_t1 <= corecyc_t0 ||
		     acttime_t1 <= acttime_t0))
		return;

	delta_acttime = acttime_t1 - acttime_t0;
	delta_corecyc = corecyc_t1 - corecyc_t0;

	/*
	 *          Δcorecyc    arch_max_freq_scale
	 *   scale = ──────── × ────────────────────
	 *          Δacttime    SCHED_CAPACITY_SCALE
	 */
	scale = delta_corecyc * this_cpu_read(arch_max_freq_scale);
	scale  = div64_u64(scale >> SCHED_CAPACITY_SHIFT,
					delta_acttime);

	scale = min_t(unsigned long, scale, SCHED_CAPACITY_SCALE);
	this_cpu_write(arch_freq_scale, (unsigned long)scale);

	pr_debug("CPU%d: freq_scale=%lu (%lu%%)\n",
		 smp_processor_id(), (unsigned long)scale,
		 (unsigned long)scale * 100 / SCHED_CAPACITY_SCALE);
}

static struct scale_freq_data sscucnt_sfd = {
	.source        = SCALE_FREQ_SOURCE_ARCH,
	.set_freq_scale = sscucnt_scale_freq_tick,
};

/**
 * sscucnt_fie_setup - enable Sscucnt-based FIE for a set of CPUs
 * @cpus: the set of CPUs to enable FIE for
 *
 * For each CPU in @cpus, validates counter availability and computes the
 * max-frequency ratio.  If all CPUs pass, registers sscucnt_sfd as the
 * SCALE_FREQ_SOURCE_ARCH source for those CPUs.
 *
 * The function is idempotent: CPUs already in sscucnt_fie_cpus are skipped.
 */
static void sscucnt_fie_setup(const struct cpumask *cpus)
{
	int cpu;

	/* Nothing to do if all CPUs in this policy are already set up */
	if (unlikely(cpumask_subset(cpus, sscucnt_fie_cpus)))
		return;

	for_each_cpu(cpu, cpus) {
		if (!freq_counters_valid(cpu)) {
			pr_info("CPU%d: freq_counters_valid() failed, skip FIE setup\n", cpu);
			return;
		}

		u64 max_rate = sscucnt_get_max_rate(cpu);

		if (!max_rate) {
			pr_info("CPU%d: failed to determine max frequency, skip FIE setup\n", cpu);
			return;
		}

		pr_info("CPU%d: max_rate=%llu Hz, riscv_timebase=%lu Hz\n",
			 cpu, max_rate, riscv_timebase);

		if (sscucnt_set_max_ratio(cpu, max_rate, riscv_timebase)) {
			pr_info("CPU%d: sscucnt_set_max_ratio() failed, skip FIE setup\n", cpu);
			return;
		}
	}

	cpumask_or(sscucnt_fie_cpus, sscucnt_fie_cpus, cpus);

	topology_set_scale_freq_source(&sscucnt_sfd, sscucnt_fie_cpus);

	pr_info("CPUs[%*pbl]: Sscucnt FIE enabled\n",
		cpumask_pr_args(cpus));
}

/**
 * init_sscucnt_fie - initialise Sscucnt-based FIE for all present CPUs
 *
 * Runs as a core_initcall.  Directly sets up frequency invariance for
 * all present CPUs without depending on cpufreq policy notifications.
 */
static int __init init_sscucnt_fie(void)
{
	if (!zalloc_cpumask_var(&sscucnt_fie_cpus, GFP_KERNEL))
		return -ENOMEM;

	sscucnt_fie_setup(cpu_present_mask);

	return 0;
}
core_initcall(init_sscucnt_fie);
