void rgb_light(int color){
    int color_dict[8][3] = {
    {0,0,0}, // OFF
    {0,0,1}, // BLUE
    {0,1,0}, // GREEN
    {0,1,1}, // CYAN
    {1,0,0}, // RED
    {1,0,1}, // PURPLE
    {1,1,0}, // YELLOW
    {1,1,1}}; // WHITE
    gpio_set_level(RGB_RED, !color_dict[color][0]);
    gpio_set_level(RGB_GREEN, !color_dict[color][1]);
    gpio_set_level(RGB_BLUE, !color_dict[color][2]);
}


float mapping(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}