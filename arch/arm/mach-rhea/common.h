#ifndef __MACh_RHEA_COMMON_H
#define __MACh_RHEA_COMMON_H

extern struct platform_device board_serial_device;

void __init board_add_common_devices(void);
void __init board_add_sdio_devices(void);
void __init board_common_reserve(void);

#endif /* __MACh_RHEA_COMMON_H */
