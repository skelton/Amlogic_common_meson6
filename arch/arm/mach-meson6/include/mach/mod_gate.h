#ifndef __MOD_GATE_H
#define __MOD_GATE_H

typedef enum {
    MOD_VDEC = 0,
    MOD_AUDIO,
    MOD_HDMI,
    MOD_VENC,
    MOD_TCON,
    MOD_LVDS,
    MOD_MIPI,
    MOD_BT656,
    MOD_SPI,
    MOD_UART0,
    MOD_UART1,
    MOD_UART2,
    MOD_UART3,
    MOD_ROM,
    MOD_EFUSE,
    MOD_RANDOM_NUM_GEN,
    MOD_ETHERNET,
    MOD_MEDIA_CPU,
    MOD_GE2D,
    MOD_VIDEO_IN,
    MOD_VIU2,
    MOD_AUD_IN,
    MOD_AUD_OUT,
    MOD_AHB,
    MOD_DEMUX,
    MOD_SMART_CARD,
    MOD_SDHC,
    MOD_STREAM,
    MOD_BLK_MOV,
    MOD_MISC_DVIN,
    MOD_MISC_RDMA,
    MOD_USB0,
    MOD_USB1,
    MOD_SDIO,
    MOD_VI_CORE,
    MOD_LED_PWM,
    MOD_MAX_NUM,
}mod_type_t;

extern void switch_mod_gate_by_type(mod_type_t type, int flag);
extern void switch_mod_gate_by_name(const char* mod_name, int flag);
extern void power_gate_init(void);

#endif /* __MOD_GATE_H */
