#include <zmk_led_widget/widget.h>

#if IS_ENABLED(CONFIG_LED_WIDGET)
    if (cfg->indicate_battery) {
        indicate_battery();
    }
#endif