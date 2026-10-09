#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Poll and print events from fc_evt (rate limited inside)
void fc_log_poll(void);

#ifdef __cplusplus
}
#endif
