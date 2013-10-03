



#ifndef FAST_GPIO_UAPI_HH
#define FAST_GPIO_UAPI_HH

#include <linux/types.h>

#define FAST_GPIO_IOC_MAGIC 0xB7


#define FAST_GPIO_IOC_RD_VALUE  _IOR(FAST_GPIO_IOC_MAGIC, 1, __u8)
#define FAST_GPIO_IOC_WR_VALUE  _IOW(FAST_GPIO_IOC_MAGIC, 2, __u8)


#endif
