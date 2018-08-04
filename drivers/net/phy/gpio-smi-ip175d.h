#ifndef __GPIO_SMI_IP175D__
#define  __GPIO_SMI_IP175D__

#define GPIO_SMI_MARK 'L'

typedef unsigned short		u16;

typedef struct smi_ioctl_data {
	u16       phy_id; 
	u16       reg_num;
	u16       val_in;
	u16       val_out;
}smi_data_t;

#define SMI_IOCTL_READ _IOR(GPIO_SMI_MARK, 0,smi_data_t)
#define SMI_IOCTL_WRITE _IOW(GPIO_SMI_MARK, 1,smi_data_t)

#endif	
