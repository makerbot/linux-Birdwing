//	Header file for buzzer driver
//	MSS 8 Nov 2013

//Includes


//Structs
struct buzzer_dev_t{
	const char	*label;
	dev_t		devt;
	struct buzzer_platform_data	*pdev;
	struct mutex	buzz_lock;
	u8		*buffer;
};

struct buzzer_ops{
	int	(*open)(struct buzzer_dev_t *dev);
	int	(*read)(struct buzzer_dev_t *dev);
	void	(*write)(struct buzzer_dev_t *dev);
	void	(*release)(struct buzzer_dev_t *dev);
	struct module *owner;
}

//Functions
//static int buzzer_open(struct);
//static int buzzer_read(struct);
//static void buzzer_write(struct);
//static void buzzer_close(struct);

//static void synth(u16, u16, u16, u32, u16);
