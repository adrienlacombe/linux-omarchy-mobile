/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MALIBU_CPM_H
#define _LINUX_MALIBU_CPM_H

struct device;

int malibu_cpm_ufs_decouple_enable(struct device *supplier);
void malibu_cpm_ufs_log_analog_rails(struct device *supplier, const char *phase);

#endif /* _LINUX_MALIBU_CPM_H */
