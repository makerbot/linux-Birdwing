

#ifndef FAST_GPIO_HH
#define FAST_GPIO_HH

struct fast_gpio_pin {

    int             gpio;
    bool            direction;
};

struct fast_gpio_platform_data {

    struct gpio_pin *pins;
    int             npins;
    

};


#endif
