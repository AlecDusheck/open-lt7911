// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Choice between the USB-C (DP Alt Mode) input and the native DP input.
 *
 * input_select runs once at boot: it sets the defaults (USB-C input) and
 * lets the input-config getter override them; the getter reports no
 * config, so the defaults stay. input_typec then gates the whole PD stack.
 */
#include <string.h>

#include "fw.h"

/* Apply the input config when the getter supplies one. */
void input_config_load(void)
{
    __xdata struct input_config cfg;

    memset(&cfg, 0, sizeof cfg);
    if (input_config_get(&cfg)) {
        input_typec = cfg.typec;
        typec_receptacle = cfg.receptacle;
        typec_dp_lane_count = cfg.dp_lane_count;
        dp_pin_assignment = cfg.pin_assignment;
    }
}

/* Select the input: USB-C by default, then the config. */
void input_select(void)
{
    input_typec = 1;
    typec_receptacle = 1;
    typec_dp_lane_count = 4;
    dp_pin_assignment = 4;
    input_config_load();
}

/* Fill an input config (DP input, 1, 4, 4); returns 0 (no config),
 * so callers keep their own values. */
uint8_t input_config_get(__xdata struct input_config *cfg)
{
    if (cfg) {
        cfg->typec = 0;
        cfg->receptacle = 1;
        cfg->dp_lane_count = 4;
        cfg->pin_assignment = 4;
    }
    return 0;
}
