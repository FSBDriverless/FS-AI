#include <stdio.h>
#include <math.h>

// Constantes del controlador
//Especificación del vehiculo y ajustar el lookahead segun la planificación
#define LOOKAHEAD_DISTANCE 30.0 // Distancia de enfoque en metros
#define MAX_STEERING_ANGLE 0.6 // Ángulo máximo de giro del volante en radianes

// Datos del vehículo
//Datos que los calcula el propio SLAM
double vehicle_x; // Posición x del vehículo en metros
double vehicle_y; // Posición y del vehículo en metros
double vehicle_yaw; // Orientación del vehículo en radianes
//Dato que se puede obtener de los inversores
double vehicle_speed; // Velocidad del vehículo en metros por segundo

// Datos de la trayectoria
//Datos que se obtendran de la planificación de trayectoria
double path_x[100]; // Coordenadas x de la trayectoria en metros
double path_y[100]; // Coordenadas y de la trayectoria en metros
int path_length; // Longitud de la trayectoria




// Función para calcular el punto de enfoque del lookahead
//Pasa como variables la posición del coche en x e y
int get_nearest_index(double x, double y) {
  // Inicializar la distancia mínima (ajustar según las necesidades) y el índice
  double min_distance = 100000.0;
  int min_index = 0;

  // Recorrer la trayectoria y encontrar el punto más cercano
  for (int i = 0; i < path_length; i++) {

      //Calcula la diferencia entre las coordenadas del vehiculo y las coordenadas de la trayectoria
    double dx = x - path_x[i];
    double dy = y - path_y[i];
    double distance = sqrt(dx * dx + dy * dy);

    //
    if (distance < min_distance) {

    //Si no se quiere que sea la mínima mínima se le puede añadir un pequeño desfase para que el punto del lookahead esté ligeramente adelantado con respecto al coche
      min_distance = distance;
      min_index = i;
    }
  }

  // Devolver el índice del punto más cercano
  return min_index;
}

// Función principal
int main() {
  // Leer los datos del vehículo
  //Estos sería los datos obtenidos del SLAM e inversores en tiempo real
  printf("Ingrese la posición x del vehículo: ");
  scanf("%lf", &vehicle_x);
  printf("Ingrese la posición y del vehículo: ");
  scanf("%lf", &vehicle_y);
  printf("Ingrese la orientación del vehículo: ");
  scanf("%lf", &vehicle_yaw);
  printf("Ingrese la velocidad del vehículo: ");
  scanf("%lf", &vehicle_speed);

  // Leer los datos de la trayectoria
  printf("Ingrese la longitud de la trayectoria: ");
  scanf("%d", &path_length);

    // Leer las coordenadas x e y de la trayectoria
    //Estos sería los datos obtenidos de la planificación de trayectoria
  for (int i = 0; i < path_length; i++) {
    printf("Ingrese la coordenada x del punto %d: ", i + 1);
    scanf("%lf", &path_x[i]);
    printf("Ingrese la coordenada y del punto %d: ", i + 1);
    scanf("%lf", &path_y[i]);
  }



  // Calcular el punto de enfoque del lookahead
  int nearest_index = get_nearest_index(vehicle_x, vehicle_y);
  //asignar ese punto de la trayectoria obtenido a dos variables focus_x y focus_y
  double focus_x = path_x[nearest_index];
  double focus_y = path_y[nearest_index];

  // Verificar si el punto de enfoque está a la distancia de enfoque
  double dx = focus_x - vehicle_x;
  double dy = focus_y - vehicle_y;
  double distance = sqrt(dx * dx + dy * dy);
  if (distance > LOOKAHEAD_DISTANCE) {
    // Calcular el índice del punto de enfoque a la distancia de enfoque
    //(double) para convertir nearest_index en doble precisión y poder sumarlo a la división sin error
    //(int) para convertir todo el resultado a entero y asignarlo a index de tipo int
    int index = (int)((double)nearest_index + vehicle_speed / 0.1);
    if (index >= path_length) index = path_length - 1;
    focus_x = path_x[index];
    focus_y = path_y[index];
  }

  // Calcular el ángulo de giro del volante
  //Nota M_PI es la constante pi definida en la biblioteca math.h
  double alpha = atan2(focus_y - vehicle_y, focus_x - vehicle_x) - vehicle_yaw;
  alpha = fmod(alpha + M_PI, 2.0 * M_PI) - M_PI; // Ajustar el ángulo a [-180, 180] grados

  // Limitar el ángulo de giro del volante al giro max o min
  double steering_angle = alpha;
  if (steering_angle > MAX_STEERING_ANGLE) steering_angle = MAX_STEERING_ANGLE;
  if (steering_angle < -MAX_STEERING_ANGLE) steering_angle = -MAX_STEERING_ANGLE;


 //En este momento pasariamos el angulo de giro al coche


  // Imprimir el ángulo de giro del volante; este comando para el AI es inutil pero se utiliza para saber que efectivamente el programa calcula el angulo de giro sin error
  printf("Ángulo de giro del volante: %lf\n", steering_angle);




 //Control de velocidad (en desarrollo - Alberto)
 //Añadir estas lineas al final del control lateral para controlar la velocidad segun el angulo de giro del volante
 //mayor angulo de giro menor velocidad y viceversa

  // Calcular la velocidad deseada
  double desired_speed = 30.0; // Velocidad máxima en metros por segundo
  double kp = 0.1; // Constante de proporcionalidad
  double kd = 0.1; // Constante de derivación
  desired_speed -= kp * fabs(alpha) + kd * distance;
  if (desired_speed < 0.0) desired_speed = 0.0;

  // Ajustar la velocidad del vehículo
  // (Aquí se debe utilizar un actuador para controlar la velocidad del vehículo)
  printf("Velocidad deseada: %lf\n", desired_speed);

    return 0;
}