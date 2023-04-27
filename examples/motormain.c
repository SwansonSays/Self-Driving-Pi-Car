

#include "main.h"

bool terminate = false;


void  Handler(int signal);

int main(void)
{	
    //1.System Initialization
    if(DEV_ModuleInit())
        exit(0);

    //2.Motor Initialization
	//Button_Press(15);
        
   	if (gpioInitialise() < 0)
   	{
      		fprintf(stderr, "pigpio initialisation failed\n");
      		return 1;
   	}
	//initialise the counter for motors
	int ret = initLS7336RChip(SPI0_CE0);
	int retTwo = initLS7336RChip(SPI0_CE1);
	if(ret!=0){
	    printf("Error initializing the LS7336R chip. Error code: %d\n",ret);
	}
	if(retTwo!=0){
		printf("Error initializing the LS7336R chip. Error code: %d\n",retTwo);
	}
	int speedA = 100;
        int speedB = 100;
	//gpioSetMode(15,PI_INPUT);
	signal(SIGINT, Handler);	
	Motor_Init();
	//Button_Press(15);
        printf("Motor_Run\r\n");
	Motor_Run(MOTORA, FORWARD, speedB);
        Motor_Run(MOTORB, BACKWARD, speedA);
	
	uint8_t counter_pins[] = {

		(uint8_t)SPI0_CE0,
		(uint8_t)SPI0_CE1	

	};
	
	double hall_sensor_vals[2] = { 0 };	
	CounterArgs* hall_sensor_args[2];
	pthread_t threads[2];
	
	for (size_t i = 0; i < 2; i++){
		CounterArgs* args = malloc(sizeof(CounterArgs));
		args->speed_val = &hall_sensor_vals[i];
		args->chip_enable = counter_pins[i];
		args->p_terminate = &terminate;
		
		hall_sensor_args[i] = args;
		pthread_create(&threads[i], NULL, read_counter, (void*)args);
	}

	while(!terminate){

		printf("%lf, %lf\n\n", hall_sensor_vals[0],hall_sensor_vals[1]);
		sleep(1);
	}
	for (size_t i = 0; i < 2; ++i)
        {
                pthread_join(&threads[i], NULL);
                free(hall_sensor_args[i]);
                hall_sensor_args[i] = NULL;
        } 
	DEV_ModuleExit();
	return 0;
}

void  Handler(int signal)
{
    //System Exit
    printf("\r\nHandler:Motor Stop\r\n");
    terminate = true;               
        


    //3.System Exit
        Motor_Stop(MOTORA);
        Motor_Stop(MOTORB);
        gpioTerminate();
        pthread_exit(NULL);
    
    exit(0);
}
