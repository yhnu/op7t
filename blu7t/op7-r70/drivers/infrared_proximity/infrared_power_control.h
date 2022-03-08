#include <linux/alarmtimer.h>

#define INFRARED_TAG                  "[oneplus_infrared] "
#define INFRARED_ERR(fmt, args...)    printk(KERN_ERR INFRARED_TAG" %s : "fmt,__FUNCTION__,##args)
#define INFRARED_LOG(fmt, args...)    printk(KERN_INFO INFRARED_TAG" %s : "fmt,__FUNCTION__,##args)


typedef struct oneplus_infrared_state {
    int                     infrared_power_enable;
    int                     infrared_shutdown_state;
    int                     infrared_shutdown_state2;
    int		                infrared_irq;
    int                     irq_times;
    unsigned int	        infrared_gpio;
    struct device           *dev;
    struct regulator        *vdd;
    struct pinctrl          *pctrl;
    struct pinctrl_state    *shutdown_state;
    struct delayed_work	    infrared_irq_check_work;
} oneplus_infrared_state;