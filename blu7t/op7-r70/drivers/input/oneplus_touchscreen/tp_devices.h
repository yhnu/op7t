 /***********************************************************
 * Description : OnePlus touchpanel driver
 * 
 * File        : tp_devices.h      
 *
 * Function    : touchpanel public interface  
 * 
 * Version     : V1.0 
 *
 ***********************************************************/
#ifndef OP_TP_DEVICES_H
#define OP_TP_DEVICES_H
//device list define
typedef enum tp_dev{
    TP_OFILM,
    TP_BIEL,
    TP_TRULY,
    TP_BOE,
    TP_G2Y,
    TP_TPK,
    TP_JDI,
    TP_SAMSUNG,
    TP_DSJM,
    TP_BOE_B8,
    TP_INNOLUX,
    TP_HIMAX_DPT,
    TP_AUO,
    TP_DEPUTE,
    TP_UNKNOWN,
}tp_dev;

struct tp_dev_name {
    tp_dev type;
    char name[32];
};

#endif

