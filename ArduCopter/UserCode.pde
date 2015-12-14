/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// CHANGE Roberto
static AP_HAL::AnalogSource *DistanceSensorReader;
float DistanceSensor;
int flag_o = 0;
static void read_MLX90614();
void init_MLX90614();
static float MLX_90614_tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)

static float MLX_90614_tempData = 0x0000; // zero out the data

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    // CHANGE Roberto
    DistanceSensorReader = hal.analogin->channel(1);  // analog input A1
    init_MLX90614();
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

// CHANGE Roberto
void Obstacle_Update() {
  DistanceSensor = DistanceSensorReader->voltage_average()*1023/5;
  if(DistanceSensor < 100)
    flag_o = 1;
  else
    flag_o = 0;
}
/* Initialize IR sensor. */
void init_MLX90614(){
        i2c_init(); //Initialise the i2c bus
        PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
}

static void read_MLX90614() {

        static int32_t dev = 0x5A<<1;
        static int32_t data_low = 0;
        static int32_t data_high = 0;
        static int32_t pec = 0;

        i2c_start_wait(dev+I2C_WRITE);
        i2c_write(0x07);

        // read

        i2c_rep_start(dev+I2C_READ);
        data_low = i2c_readAck(); //Read 1 byte and then send ack
        data_high = i2c_readAck(); //Read 1 byte and then send ack
        pec = i2c_readNak();
        i2c_stop();

        MLX_90614_tempData = (double)(((data_high & 0x007F) << 8) + data_low);
        MLX_90614_tempData = (MLX_90614_tempData * MLX_90614_tempFactor)-0.01;

        hal.console->printf_P(PSTR("%d Kelvins \n"), (int16_t)MLX_90614_tempData); // no need to print but for val checking

}
