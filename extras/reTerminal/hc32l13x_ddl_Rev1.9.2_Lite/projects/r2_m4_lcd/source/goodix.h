#ifndef __GOODIX_H__
#define __GOODIX_H__


#define GOODIX_CONTACT_SIZE		    8
#define GOODIX_MAX_CONTACT_SIZE		9
#define GOODIX_MAX_CONTACTS		    10


/* Register defines */
#define GOODIX_REG_COMMAND		        0x8040
#define GOODIX_CMD_SCREEN_OFF		    0x05

#define GOODIX_READ_COOR_ADDR		    0x814E
#define GOODIX_GT1X_REG_CONFIG_DATA	    0x8050
#define GOODIX_GT9X_REG_CONFIG_DATA	    0x8047
#define GOODIX_REG_ID			        0x8140

#define GOODIX_BUFFER_STATUS_READY	    (((uint32_t)0x01)<<7)//BIT(7)
#define GOODIX_HAVE_KEY			        (((uint32_t)0x01)<<4)//BIT(4)
#define GOODIX_BUFFER_STATUS_TIMEOUT	20


#endif // __GOODIX_H__
