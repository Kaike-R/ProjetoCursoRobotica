/*
 * File:          Controlador1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 128
#define QtddSensorDist 8

static WbDeviceTag motorE, motorD;
static WbDeviceTag sensorDist[QtddSensorDist];
static double sensorDist_Valor[QtddSensorDist];

static WbDeviceTag GroundE, GroundD;
static double GroundE_Valor, GroundD_Valor;


#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28
static double weights[QtddSensorDist][2] = {{-1.3, -1.0},{-1.3, -1.0},{-0.5, 0.5},{0.0, 0.0},{0.0, 0.0},{0.05, -0.5},{-0.75, 0},{-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};





/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
 static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}
 
 
 
 
 
 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  int i,j;
  double Velocidades[2];
  int variavel = 0;
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  //MOTORES
  motorE = wb_robot_get_device("left wheel motor");
  motorD = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(motorE, INFINITY);
  wb_motor_set_position(motorD, INFINITY);
  
  wb_motor_set_velocity(motorE, 0);
  wb_motor_set_velocity(motorD, 0);
  
  //SENSORES
  sensorDist[0] = wb_robot_get_device("ps0");
  sensorDist[1] = wb_robot_get_device("ps1");
  sensorDist[2] = wb_robot_get_device("ps2");
  sensorDist[3] = wb_robot_get_device("ps3");
  sensorDist[4] = wb_robot_get_device("ps4");
  sensorDist[5] = wb_robot_get_device("ps5");
  sensorDist[6] = wb_robot_get_device("ps6");
  sensorDist[7] = wb_robot_get_device("ps7");
  
  for(i=0;i<QtddSensorDist;i++)
    wb_distance_sensor_enable(sensorDist[i], get_time_step());
  
  //POSIÇÃO ROBO
  //ATENÇÃO -> NÃO ESQUEÇA DE HABILITAR O SUPERVISOR *E* NOMEAR (DEF) O ROBO!
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("e-Puck");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF ePuck node found in the current world file\n");
    exit(1);
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *posicao_robo;
  
  
  //sensor de solo
  GroundE = wb_robot_get_device("IR1");
  wb_distance_sensor_enable(GroundE, get_time_step());
  
  GroundD = wb_robot_get_device("IR0");
  wb_distance_sensor_enable(GroundD, get_time_step());

  
  
  
  
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     
    for (i=0;i<QtddSensorDist;i++)
      sensorDist_Valor[i] = wb_distance_sensor_get_value(sensorDist[i])/4096;
       //printf("estou aqui");
    for (i = 0; i < 2; i++) {
    Velocidades[i] = 0.0;
    for (j = 0; j < QtddSensorDist; j++)
      Velocidades[i] += sensorDist_Valor[j] * weights[j][i];

    Velocidades[i] = offsets[i] + (Velocidades[i] * MAX_SPEED) +  ((double)rand()/RAND_MAX-.5);
    if (Velocidades[i] > MAX_SPEED)
      Velocidades[i] = MAX_SPEED;
    else if (Velocidades[i] < -MAX_SPEED)
      Velocidades[i] = -MAX_SPEED;
  }
   
    
    
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     posicao_robo = wb_supervisor_field_get_sf_vec3f(trans_field);
    
  GroundE_Valor = wb_distance_sensor_get_value(GroundE);
     GroundD_Valor = wb_distance_sensor_get_value(GroundD);

    //if (GroundD_Valor<5 || GroundE_Valor<5){
      //if (GroundD_Valor<5) Velocidades[LEFT] = Velocidades[LEFT]*0.5;
      //else if (GroundE_Valor<5) Velocidades[RIGHT] = Velocidades[RIGHT]*0.5;
    //}
    
    
    
    if (variavel == 0){
    
      if(GroundD_Valor>5 && GroundE_Valor>5){
        if(GroundD_Valor==1000 || GroundE_Valor==1000){
          printf("Não valido");
        }
        if(GroundD_Valor==11.41 || GroundE_Valor==11.51){
          printf("Não valido");
        }
        if(GroundD_Valor>11 || GroundE_Valor>11){
          printf("Não valido");
        }
        else{
          variavel ++;
         }
      }
      else{
        Velocidades[0] = 5;
        Velocidades[1] = 5;
      }
    }
    if (variavel == 1){
      //Velocidades[0] = 0;
      //Velocidades[1] = 3;
      if(GroundD_Valor<5 || GroundE_Valor<5){
        if (GroundD_Valor<5){
          Velocidades[LEFT] = 0;
          Velocidades[RIGHT] = 3.5;
        }
        else if (GroundE_Valor<5){
         Velocidades[LEFT] = 3.5;
          Velocidades[RIGHT] = 0;
        }
      }
      else{
        Velocidades[LEFT] = 5;
        Velocidades[RIGHT] = 5;
      }
      
    }
    
    
    
    
    wb_motor_set_velocity(motorE, Velocidades[LEFT]);
    wb_motor_set_velocity(motorD, Velocidades[RIGHT]); 
    //posicao_Caixa1 = wb_supervisor_field_get_sf_vec3f(Caixa1_field);
    printf("Velocidades: %6.2f  %6.2f  |  Posição Robo: %6.2f  %6.2f |  Sensores de Pista: Esq:%6.2f  Dir:%6.2f\n",Velocidades[LEFT],Velocidades[RIGHT],posicao_robo[0], posicao_robo[2],GroundE_Valor, GroundD_Valor);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
