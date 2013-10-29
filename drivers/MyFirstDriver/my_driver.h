//	Header file
//	My First Driver
//	MSS 14 Oct 2013


struct mf_device{
	const char 	*label;
	unsigned int 	foo;
	unsigned int	bar;
};

struct first_ops{
	int 	(*open)(struct mf_device *dev);
	int	(*read)(struct mf_device *dev);
	void 	(*write)(struct mf_device *dev);
	void	(*release)(struct mf_device *dev);
	struct module	*owner;
};
