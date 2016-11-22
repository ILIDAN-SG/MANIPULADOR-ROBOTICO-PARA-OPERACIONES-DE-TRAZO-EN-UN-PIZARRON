/*
INSTITUTO POLITECNICO NACIONAL
UNIDAD PROFESIONAL INTERDICIPLINARIA DE INGENIERIA Y 
TECNOLOGIAS AVANZADAS

MANIPULADOR ROBOTICO PARALELO DE DOS GRADOS DE LIBERTAD PARA
OPERACIONES DE TRAZO EN UN PIZARRON

ESCRITO POR:
VARGAS RAMIREZ ANSELMO

2016
*/


//Libreria linea de comandos
#include <Shell.h>
//Libreria funciones matematicas
#include <math.h>
//Libreria para gestion de servo
#include <Servo.h>

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: PINES
//Interrupcion 0 - Reinicio motores apagados
#define RESET_SYSTEM  2
//Interrupcion 1 - Pausa por hardware
#define PAUSE       3
//Leer estado de los controladores de Motores a Pasos
#define STATE       4

//Controlador motor 1 (izquierda)
#define DIR_M1      5
#define STEP_M1     6

//Controlador motor 2 (derecha)
#define DIR_M2      7
#define STEP_M2     8

//Servomotor
#define SERVO       11

//Se crea un objeto de la clase Servo
Servo apoyo;

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: VARIABLES GLOBALES
//Tiempo entre pulsos de motores (useg)
//(Max=700)
#define TP 3000
//Unidad base de movimiento: 0.4mm por paso (full step)
float UB=0.04;
//Longitud del pizarron
float L_MAX=120;
//Limites de dibujo (cm)
float X_LIM_IZQ=10;
float X_LIM_DER=110;
float Y_LIM_SUP=20;
float Y_LIM_INF=55;

//Coordenadas del HOME del sistema
float HOME_X=60;
float HOME_Y=55;

//Variables para las ecuaciones de la cinematica
float l1_home;
float l2_home;
float x;
float y;

float l1_sig;
float l2_sig;

float l1_actual;
float l2_actual;

//numero de decimales mostrados en puerto serie
int pres = 7;

//Contador de pasos
long count_m1;
long count_m2;

//Variable para el tiempo en interrupcion 0
long T0=0;

//Variable de tiempo en interrupcion 1
long T1=0;

//Posicion del servomotor
int pos;

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: FUNCIONES
//------------------------------------------------------------------Movimiento de los motores
//Un paso en una dirección
void m1_step(int dir_m1){
  if (dir_m1==0){
    digitalWrite(DIR_M1,HIGH);
    count_m1-=1;
  }
  else{
    digitalWrite(DIR_M1,LOW);
    count_m1+=1;
    }
  digitalWrite(STEP_M1,HIGH);
  delayMicroseconds(TP);
  digitalWrite(STEP_M1,LOW);
  delayMicroseconds(TP);
 }  

void m2_step(int dir_m2){
  //digitalWrite(ENABLE_M2,LOW);
  if (dir_m2==0){
    digitalWrite(DIR_M2,HIGH);
    count_m2+=1;
  }
  else{
    digitalWrite(DIR_M2,LOW);
    count_m2-=1;
    }
  digitalWrite(STEP_M2,HIGH);
  delayMicroseconds(TP);
  digitalWrite(STEP_M2,LOW);
  delayMicroseconds(TP);
 }

//Frenos
void m1_brake(void){
  digitalWrite(STEP_M1,LOW);
}

void m2_brake(void){
  digitalWrite(STEP_M2,LOW);
}

//------------------------------------------------------------------------Cinematica directa
void CD(float l1,float l2,float &x,float &y){
  x=((L_MAX*L_MAX)+(l1*l1)-(l2*l2))/(2*L_MAX);
  y=sqrt((l1*l1)-(((L_MAX*L_MAX)+(l1*l1)-(l2*l2))/(2*L_MAX))*(((L_MAX*L_MAX)+(l1*l1)-(l2*l2))/(2*L_MAX)));
}

//------------------------------------------------------------------------Cinematica inversa
void CI( float x, float y,float &l1_sig,float &l2_sig){
  l1_sig=sqrt(x*x+y*y);
  l2_sig=sqrt((L_MAX-x)*(L_MAX-x)+y*y);
}

//------------------------------------ Imprimir cifras de punto flotante por el puerto serial
//Tomado de: http://forum.arduino.cc/index.php?topic=44216.0
void printDouble( double val, byte precision){
 // prints val with number of decimal places determine by precision
 // precision is a number from 0 to 6 indicating the desired decimial places
 // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

 Serial.print (int(val));  //prints the int part
 if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
      mult *=10;
      
   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
     padding--;
   while(  padding--)
     Serial.print("0");
   Serial.print(frac,DEC) ;
 }
}

//----------------------------------------------------------------- Funcion mover a nuevo punto
//Nota: El argumento de las variables debe pasarse por 
//referencia para poder así modificar la variable global
//de lo contrario solo se crea una copia del valor
void mov(float &l1_actual,float &l2_actual,float x,float y, int draw){
  int flag;
  //Se verifica que el siguiente movimiento se encuentre dentro del espacio de trabajo 
  flag=check_mov(x,y);
  //1 - permitido 0 - no permitido
  if (flag==1){
      //Despliega o no el apoyo depeniendo la operacion
      if(draw==1){
        retract();
        //Se comunica por puerto serie que se esta realizando
        shell_println("D"); 
      }
      else{
        deploy();
        shell_println("m");    
      }
      //delay(500);
      //Dirección de los motores
      int dir_m1,dir_m2;
      //Diferencia de longtudes de las correas
      float dl1,dl2;
      //Variables del algoritmo de Bresenham: error y contador 
      float error,i;
        
      //Se obtiene las longitudes de las correas para el punto destino
      //mediante las ecuaciones de la cinematica inversa
      CI(x,y,l1_sig,l2_sig);
    
      //Algoritmo de Bresenham
      //Se obtiene la diferencia de las correas
      dl1=abs(l1_actual-l1_sig);
      dl2=abs(l2_actual-l2_sig);
      //Se define el sentido del movimiento de los motores  
      if (l1_actual<l1_sig)
        dir_m1=1;
      else
        dir_m1=0;
      if (l2_actual<l2_sig)
        dir_m2=0;
      else
        dir_m2=1;
      //Pendiente menor a 45°  
      if(dl1>dl2){
        error=2*dl2-dl1;
        for(i=0;i<dl1;i+=UB){     
          if(error>=0){
            error-=2*dl1;
            m2_step(dir_m2);        
          }
          error+=2*dl2;
          m1_step(dir_m1);
        }
      }
      //Pendiente mayor a 45° 
      else {
        error = 2*dl1-dl2;
        for(i=0;i<dl2;i+=UB) {      
          if(error>=0) {
            error-=2*dl2;
            m1_step(dir_m1);
          }      
          error+=2*dl1;
          m2_step(dir_m2);
        }
      }  
      //Se actualiza la posicion de la h. de dibujo        
      l1_actual=l1_sig;
      l2_actual=l2_sig;
      
      //Se obtienen las coordenadas x,y actuales
      CD(l1_actual,l2_actual,x,y);
      //Buffer para funcion dtostrf
      char temp1[10];
      char temp2[10];  
      //Se envia por puerto serie las coordenadas cartesianas
      shell_print("X");
      shell_println(dtostrf(x,5,2,temp1));
      //Doble -> ACII: dtostrf(variable,tamaño_de_salida,precision,variable_temporal)
      shell_print("Y");
      shell_println(dtostrf(y,5,2,temp2));
      //Se reporta por puerto serie que la operacion termino
      shell_println("ok");
  }  
}

//------------------------------------------------------------------ Funcion poligono n lados
void polygon(int n, int r,float &l1_actual, float &l2_actual,float &x,float &y){
    int flag,draw;
    float fig_org_x,fig_org_y,rot,part;
    //Se obtienen las coordenadas x,y actuales
    CD(l1_actual,l2_actual,x,y);
    //Verifica si la figura estara dentro del area de trabajo
    flag=check(x,y,r);
    //Si estadentro procede, de lo contrario retorna mensaje de error
    if (flag==1){
      //Manda mensaje de que el sistema se encuentra dibujando
      shell_println("D");
      //Rota las figuras 
      if(n==4)
        //Si es un cuadrado los lados de la figura seran paralelos al pizarron
        rot=M_PI_4;
      else
        //De otro modo mantiene siempre un vertice verticalmente
        rot=M_PI_2-((2*M_PI)/n)+M_PI;
      part=(2*M_PI)/n;
      //Se guardan las coordenadas del origen de las figuras
      fig_org_x=x;
      fig_org_y=y;
      //Mueve la H. de dibujo al primer vertice sin dibujar
      x=fig_org_x+(r*cos(rot));
      y=fig_org_y+(r*sin(rot));
      draw=0;
      mov(l1_actual,l2_actual,x,y,draw);
      //Dibuja todos los lados de la figura
      draw=1;
      for(float i=0;i<=(2*M_PI);i+=part){     
        x=fig_org_x+(r*cos(i+rot));
        y=fig_org_y+(r*sin(i+rot));

        mov(l1_actual,l2_actual,x,y,draw);
      }
      //Regresa la h. de dibujo al origen de la pieza sin dibujar
      draw=0;
      x=fig_org_x;
      y=fig_org_y;
      mov(l1_actual,l2_actual,x,y,draw);
      shell_println("d");
    }
}

//-------------------------------------------------------------- Realizar un cuadrado de L cm
void cuad (float &l1_actual, float &l2_actual,float &x,float &y){
    float L=10;
    int flag;
    //Se obtienen las coordenadas x,y actuales
    CD(l1_actual,l2_actual,x,y);
    //Verifica si la figura estara dentro del area de trabajo
    flag=check(x,y,L);      
    if (flag==1){
      //mueve el plumon dibujando
      int draw=1;
      
      x=x+L;
      y=y;
  
      delay(1500);
      
      mov(l1_actual,l2_actual,x,y,draw);
      delay(300);
  
      x=x;
      y=y+L;
      mov(l1_actual,l2_actual,x,y,draw);
      delay(300);
  
      x=x-L;
      y=y;
      mov(l1_actual,l2_actual,x,y,draw);
      delay(300);
  
      x=x;
      y=y-L;
      mov(l1_actual,l2_actual,x,y,draw);
      delay(300);
    }
}

//------------------------------------------------------------------------------ Funcion home
void go_home(){
  int draw=0;
  CD(l1_home,l2_home,x,y);
  
  mov(l1_actual,l2_actual,x,y,draw);
  //Correguir error
  correct_e(count_m1,count_m2);
}
//------------------------------------ Correguir pasos perdidos por las operaciones decimales
void correct_e(long &count_m1, long &count_m2){
  for(;;){
    if(count_m1==0&&count_m2==0){
      break;
    }    
    if(count_m1==0){      
    }
    else{ 
      if(count_m1>0){
        //m1_step(0);
        digitalWrite(DIR_M1,HIGH);
        digitalWrite(STEP_M1,HIGH);
        delayMicroseconds(TP);
        digitalWrite(STEP_M1,LOW);
        delayMicroseconds(TP);
        count_m1--;
      }
      else{
        //m1_step(1);
        digitalWrite(DIR_M1,LOW);
        digitalWrite(STEP_M1,HIGH);
        delayMicroseconds(TP);
        digitalWrite(STEP_M1,LOW);
        delayMicroseconds(TP);
        count_m1++;
      }       
    }
    
    if(count_m2==0){      
    }
    else{ 
      if(count_m2>0){
        //m2_step(1);
        digitalWrite(STEP_M2,LOW);
        delayMicroseconds(TP);
        digitalWrite(STEP_M2,LOW);
        delayMicroseconds(TP);
        count_m2--;
      }
      else{
        //m2_step(0);
        digitalWrite(STEP_M2,HIGH);
        delayMicroseconds(TP);
        digitalWrite(STEP_M2,LOW);
        delayMicroseconds(TP);
        count_m2++;
      }       
    }        
  }
}

//-------------------------------------------------------------- Reiniciar contadores de paso
void restart_counts(long &count_m1,long count_m2){
  count_m1=0;
  count_m2=0;
}
//-------------------------------------------------------------------- Funcion deplegar apoyo
void deploy(void){
//  apoyo.write(5);      
  if (pos!=5){
    for (pos = 90; pos > 5; pos -= 1) { 
      apoyo.write(pos);             
      delay(15);                       
    }
  } 
}
//--------------------------------------------------------------------- Funcion retraer apoyo
void retract(void){
//  apoyo.write(90);
  if (pos!=90){
    for (pos = 5; pos < 90; pos += 1) { 
      apoyo.write(pos);             
      delay(15);                       
    }
  }
}
//---------------------------------------------------------------- Funcion reset por software
//Apuntador al comenzo del programa
void (*resetFunc)(void)=0;

//------------------- Funcion para verificar que la figura se encuentra dentro de los limites
int check(float &x,float &y,float R){
  if (((x-R)>=X_LIM_IZQ)&&((x+R)<=X_LIM_DER)&&((y-R)>=Y_LIM_SUP)&&((y+R)<=Y_LIM_INF)){    
    return 1;
  }
  else{
    //Comando figura fuera del espacio de trabajo
    shell_println("F");
    return 0;
  }
}

//--------------- Funcion para verificar que el movimiento se encuentra dentro de los limites
int check_mov(float &x,float &y){
  if (((x-0)>=X_LIM_IZQ)&&((x+0)<=X_LIM_DER)&&((y-0)>=Y_LIM_SUP)&&((y+0)<=Y_LIM_INF)){    
    return 1;
  }
  else{
    //Comando movimiento fuera del espacio de trabajo
    shell_println("f");
    return 0;
  }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Shell
//--------------------------------------------------------------------------- Esperar comando
int shell_reader(char * data)
{
  // Wrapper for Serial.read() method
  if (Serial.available()) {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

//------------------------------------------------------------- Escribir por el puerto serial
void shell_writer(char data)
{
  // Wrapper for Serial.write() method
  Serial.write(data);
}

//----------------------------------------------------------------- Comando mover nuevo punto
int command_mov(int argc, char** argv)
{ 
  int draw;
  //Obtiene valores del comando recibido por puerto serial
  //Coordenadas a las que ir
  x = strtod(argv[1],NULL);
  y = strtod(argv[2],NULL);  
  draw = strtod(argv[3],NULL);
  mov(l1_actual,l2_actual,x,y,draw);
  
  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//------------------------------------------------------------------------------ Comando home
int command_home(int argc, char** argv)
{     
  go_home();

//  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//-------------------------------------------------------------------------- Comando cuadrado
int command_cuad(int argc, char** argv)
{ 
    
  cuad(l1_actual,l2_actual,x,y);

//  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//------------------------------------------------------------------------- Comando triangulo
int command_tria(int argc, char** argv)
{ 
  delay(500);
}
//-------------------------------------------------------------- Comando secuencia automatica
//Secuancia que dibuja cinco cuadrados
int command_repetir(int argc, char** argv)
{ 
  if (digitalRead(STATE)==LOW){
    shell_println("M");    
  }
  else{
    delay(5000);
    //Desplegar apoyo en los movimientos
    int draw=0;
    
    cuad(l1_actual,l2_actual,x,y);
    delay(300);
    
    mov(l1_actual,l2_actual,10,17,draw);
    delay(300);
    cuad(l1_actual,l2_actual,x,y);
  
    mov(l1_actual,l2_actual,10,40,draw);
    delay(300);
    cuad(l1_actual,l2_actual,x,y);
  
    mov(l1_actual,l2_actual,60,30,draw);
    delay(300);
    
    mov(l1_actual,l2_actual,100,17,draw);
    cuad(l1_actual,l2_actual,x,y);
    delay(300);
    mov(l1_actual,l2_actual,100,40,draw);
    cuad(l1_actual,l2_actual,x,y);
    delay(300);
    go_home();
  }  
}

//------------------------------------------------------------------- Comando desplegar apoyo
int command_desplegar(int argc, char** argv)
{     
  deploy();

//  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//--------------------------------------------------------------------- Comando retraer apoyo
int command_retraer(int argc, char** argv)
{     
  retract();

//  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//------------------------------------------------------------------ Comando poligono n lados
int command_poligono(int argc, char** argv)
{  
  int n,r;
   //Obtiene valores del comando recibido por puerto serial
  n = strtod(argv[1],NULL);
  r = strtod(argv[2],NULL);    
  polygon(n,r,l1_actual,l2_actual,x,y);

//  shell_println(">\n");
  // Return success code
  return SHELL_RET_SUCCESS;
}
//::::::::::::::::::::::::::::::::::::::::::::::::::: Funciones de servicio de interrupciones
//---------------------------------------------------------------------------- Interrupcion 1
//Pausa
void pause(void){

    while (digitalRead(PAUSE)==LOW){
      m1_brake();
      m2_brake();
    
      T1=millis();    
  }
}


//---------------------------------------------------------------------------- Interrupcion 0
//Calibracion manual. Reinicia todo el programa
void reset(void){
  //Evitar efecto rebote
  if (millis()>(T0+1500)){
    int s;
    //Espera donde este
    delay(1500);
    //Leer el estado de los controladores de los MaP
    //1=Apagados 0=Encendidos
    s=digitalRead(STATE);
    //Actualiza el tiempo para la siguente interrupcion
    //delay(500);
    T0=millis();
    //Reinicio por software
    //Si los controladores estan apagados reinicia variables
    //y ejecuta el programa desde el principio
    if (s==1){
        //REALIZAR TODO LO QUE SE HACE AL PRINCIPIO DEL PROGRAMA
        //Coordenadas iniciales
        l1_actual=l1_home;
        l2_actual=l2_home;
        //Reinicia contadores
        restart_counts(count_m1,count_m2);
        //Habilitar frenos
        m1_brake();
        m2_brake();
        //Desplegar apoyo
        pos=5;
        apoyo.write(pos);       
       
        delay(1000);
        //Reinicio por software
        resetFunc();      
    }
  }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Configuracion
void setup()
{
  // Preparacion puerto serie
  Serial.begin(9600);
  delay(1000);

  //Controladores
  pinMode(STEP_M1,OUTPUT);
  pinMode(DIR_M1,OUTPUT);
  pinMode(STEP_M2,OUTPUT);
  pinMode(DIR_M2,OUTPUT);
  //Leer estado de los controladores
  pinMode(STATE,INPUT);

  //Servomotor
  apoyo.attach(SERVO);

  //Iniciando libreria shell
  shell_init(shell_reader, shell_writer, 0);
  //Registrando comandos
      //Mover a un punto
      shell_register(command_mov, "m");
      //Mover a home
      shell_register(command_home, "h");
      //Hacer un cuadrado
      shell_register(command_cuad, "c");
      //Hacer un triangulo
      shell_register(command_tria, "t");
      //Repetir una ruta
      shell_register(command_repetir, "s");
      //Desplegar apoyo por comando
      shell_register(command_desplegar, "d");
      //Retraer apoyo por comando
      shell_register(command_retraer, "r");
      //Dibujar poligono
      shell_register(command_poligono, "p");
      
  //Interrupciones
  //Interrupcion,funcion_servicio,modo
      attachInterrupt(0,reset,RISING);
//      attachInterrupt(1,pause,FALLING);

  //--------------------------------------- Acciones que se realizan solo una vez
      //Coordenadas iniciales (x,y)->(l1,l2)
      CI(HOME_X,HOME_Y,l1_home,l2_home);
      l1_actual=l1_home;
      l2_actual=l2_home;
      
      //Reiniciar contadores de paso
      count_m1=0;
      count_m2=0;
      //Habilitar frenos
      m1_brake();
      m2_brake();
      //Desplegar apoyo
      deploy();

      //Enviar por puerto serie las coordenadas iniciales
      char temp1[10];
      char temp2[10];
      shell_print("X");
      shell_println(dtostrf(HOME_X,5,2,temp1));
      shell_print("Y");
      shell_println(dtostrf(HOME_Y,5,2,temp2));
}

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Ciclo infinito
void loop()
{
  //Esperar comando
  shell_task();  
}
